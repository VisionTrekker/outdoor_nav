// Copyright 2026, outdoor_nav maintainers.
#include "nav2_input_bridge/align_state_machine.hpp"

#include <cmath>

namespace nav2_input_bridge {

// 默认构造：state_ 走 AlignState::INIT 初始值，下一次 update() 自动转 WAITING_DATA。
//   配置走 Config{} 默认值（max_attempts=3, threshold=0.2 rad, interval=0s）。
AlignStateMachine::AlignStateMachine() = default;

// 配置构造：调用方注入 Config；其余成员走默认初始化。
AlignStateMachine::AlignStateMachine(const Config &cfg) : config_(cfg) {}

// 推进内部时钟。仅累加 elapsed_s_，不触发任何状态迁移。
//   间隔门 (relatch_interval_s) 在 update() 内根据 elapsed_s_ 差值判定。
void AlignStateMachine::tick(double dt) { elapsed_s_ += dt; }

// 计算 T_ENU_odom 偏移量。
//   推导：已知锁存瞬间的 T_ENU_base（base 在 ENU 下）和 T_odom_base（base 在 odom 下），
//   要把后续所有 odom 位姿映射到 ENU：
//       T_ENU_base = T_ENU_odom · T_odom_base
//   ⇒   T_ENU_odom = T_ENU_base · T_odom_base⁻¹
Eigen::Isometry3d AlignStateMachine::computeOffset(const Eigen::Isometry3d &T_ENU_base,
                                                   const Eigen::Isometry3d &T_odom_base) {
  return T_ENU_base * T_odom_base.inverse();
}

// 把锁存偏移量施加到任意 T_odom_base 上，得到 T_ENU_base。
//   若尚未锁存（latched_=false），返回单位阵作为"哨兵"，调用方应据此跳过发布。
Eigen::Isometry3d AlignStateMachine::applyOffset(const Eigen::Isometry3d &T_odom_base) const {
  if (!latched_) {
    return Eigen::Isometry3d::Identity();
  }
  return T_ENU_odom_ * T_odom_base;
}

// 8 项条件全部就绪判定。
//   - 7 个 bool 全 true；
//   - 时间一致性差 <= 阈值（默认 0.5 s）
bool AlignStateMachine::allInputsReady(const AlignInputs &i) const {
  return i.aft_mapped_seen && i.mavros_global_valid && i.mavros_heading_valid &&
         i.gp_origin_verified && i.gps_quality_ok && i.ekf2_cov_low && i.ekf2_state_ok &&
         (i.time_consistency_diff_s <= i.time_consistency_max_s);
}

// 用 AlignInputs 中缓存的 first_aft_mapped + T_ENU_base_latch 计算 T_ENU_odom_，
// 同时提取两个标量指标供监控：
//   last_yaw_offset_ : T_ENU_odom_ 绕 Z 轴的旋转角（航向偏差）
//   last_off_yaw_    : ‖R_z - e_z‖，仅对 pitch/roll 敏感，纯 yaw 为 0
//                      取值范围 [0, sqrt(2)]，阈值 0.2 区分"水平面附近"与"已倾斜"
//
// off_yaw 几何意义：旋转矩阵第 3 列（Z 轴在 ENU 中的方向）应等于 (0,0,1)。
// 偏离单位 Z 的欧氏距离正好捕捉 X/Y 旋转（pitch/roll），不响应纯 Z 旋转（yaw）。
//   例子：
//     单位阵                 → 0                              < 0.2 → LATCHED
//     R(0.01, 0, 0) X 轴 0.01 → ≈ 0.01                        < 0.2 → LATCHED
//     R(π/4, 0, 0) X 轴 45°  → norm((0, -0.707, -0.293))≈0.77 > 0.2 → RELATCHING
void AlignStateMachine::latchFromInputs(const AlignInputs &i) {
  T_ENU_odom_ = computeOffset(i.T_ENU_base_latch, i.first_aft_mapped);

  // 监控用：航向偏差 = atan2(R(1,0), R(0,0))，即绕 Z 轴的旋转角
  last_yaw_offset_ = atan2(T_ENU_odom_.linear()(1, 0), T_ENU_odom_.linear()(0, 0));

  // 监控用：off_yaw = ‖R_z 列 - e_z‖ = sqrt(R(0,2)² + R(1,2)² + (R(2,2)-1)²)
  //         用于检测锁存结果中是否存在非水平的 pitch/roll 偏差
  last_off_yaw_ =
    std::sqrt(T_ENU_odom_.linear()(0, 2) * T_ENU_odom_.linear()(0, 2) +
              T_ENU_odom_.linear()(1, 2) * T_ENU_odom_.linear()(1, 2) +
              (T_ENU_odom_.linear()(2, 2) - 1.0) * (T_ENU_odom_.linear()(2, 2) - 1.0));

  latched_ = true;
}

// 状态机主推进（每 100ms 由 InputBridgeNode::evaluateAlign 调用一次）。
//
// 状态迁移图：
//   INIT ──► WAITING_DATA ──（allReady）──► READY_TO_LATCH
//                                              │
//                                              ├─ off_yaw ≤ threshold ─► LATCHED  (稳态)
//                                              │
//                                              └─ off_yaw  > threshold ─► RELATCHING
//                                                                       │
//                                                  attempts++ > max ───┴─► FATAL
//
// RELATCHING 状态额外受 interval_s 门控：自进入 RELATCHING 以来
//   tick 累计时间 < relatch_interval_s 时保持原态，避免 100ms 定时器内
//   立即重试造成反复锁存失败。interval_s=0 时 (默认) 立即放行。
//
// 计数器语义：
//   relatch_attempts_ 在 READY_TO_LATCH 和 RELATCHING 两处均 += 1，
//   也就是说一次完整的"重锁存循环"会让计数 +2。
//   实测 max_attempts=3 时，约 2 次外部触发即到 FATAL 上限，
//   这是经过单测验证的故意行为，不要在 RELATCHING 中重置计数。
bool AlignStateMachine::update(const AlignInputs &i) {
  const AlignState prev = state_;
  switch (state_) {
  case AlignState::INIT:
    // 首次进入：纯过场，用于日志/调试区分"刚构造"与"已在等待"
    state_ = AlignState::WAITING_DATA;
    break;

  case AlignState::WAITING_DATA:
    // 等待 8 项条件；只有都满足才推进到 READY_TO_LATCH
    if (allInputsReady(i)) {
      state_ = AlignState::READY_TO_LATCH;
    }
    // 注意：若条件不满足，保持在 WAITING_DATA（不会推进，也不会有副作用）
    break;

  case AlignState::READY_TO_LATCH:
    // 计算并锁存 T_ENU_odom_，清零 fatal 计时
    latchFromInputs(i);
    resetFatalElapsedS();
    if (last_off_yaw_ > config_.relatch_off_yaw_threshold_rad) {
      // 锁存结果带明显 pitch/roll 偏差 → 视为"坏锁存"，进入重锁存流程
      state_ = AlignState::RELATCHING;
      relatch_attempts_ += 1; // 累加，不重置
      relatch_enter_elapsed_s_ = elapsed_s_; // 记录进入时刻，用于 interval gate
    } else {
      // 锁存结果在水平面附近 → 进入稳态 LATCHED
      state_ = AlignState::LATCHED;
    }
    break;

  case AlignState::LATCHED:
    // 稳态：除非外部服务 requestRelatch()，否则什么都不做
    break;

  case AlignState::RELATCHING:
    // 间隔门：自进入 RELATCHING 以来累计 ticks < relatch_interval_s 时保持原态，
    //   避免 100ms 定时器内立刻重试造成"雪崩式"反复锁存失败。
    //   interval_s=0 时 (默认) 立即放行，保持向后兼容。
    if ((elapsed_s_ - relatch_enter_elapsed_s_) < config_.relatch_interval_s) {
      break;
    }
    relatch_attempts_++;
    if (relatch_attempts_ > config_.relatch_max_attempts) {
      // 超过重锁存上限 → 进入致命态
      state_ = AlignState::FATAL;
    } else {
      // 回到等待态，重新收集 8 项输入
      state_ = AlignState::WAITING_DATA;
    }
    break;

  case AlignState::FATAL:
    // 致命态稳态：只能由 ~/input_bridge/relatch 服务跳出
    break;
  }
  return state_ != prev;
}

// 外部服务 ~/input_bridge/relatch 调用入口。
//   只在 LATCHED 或 FATAL 时生效，跳到 RELATCHING 并清零 attempts。
//   在其他状态调用本函数是 no-op。
void AlignStateMachine::requestRelatch() {
  if (state_ == AlignState::LATCHED || state_ == AlignState::FATAL) {
    state_ = AlignState::RELATCHING;
    relatch_attempts_ = 0;
    resetFatalElapsedS();
  }
}

// 强制进入 FATAL。供 evaluateAlign() 在 fatal_timeout_s_ 超时时调用：
//   不再反复 update()（若此时 state 是 WAITING_DATA 且 8 项输入不满足，
//   update 不会让状态前进，while 循环会卡死），直接置位 FATAL 退出。
void AlignStateMachine::forceFatal() { state_ = AlignState::FATAL; }

} // namespace nav2_input_bridge
