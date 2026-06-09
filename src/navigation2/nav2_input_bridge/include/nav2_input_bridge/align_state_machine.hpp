// Copyright 2026, outdoor_nav maintainers.
// Pure C++ state machine for FAST-LIVO2 ↔ mavros ENU frame alignment.
// No rclcpp dependency — fully unit-testable.
//
// FAST-LIVO2 ↔ mavros ENU 坐标系对齐所用的纯 C++ 状态机。
//
// 作用：把 SLAM (`/aft_mapped_to_init`, odom 坐标系) 给出的位姿，重新投影
// 到 PX4 EKF2 的 ENU (map) 坐标系下，发布到 `/mavros/vision_pose/pose`
// 作为 EKF2 的外部位置观测 (EKF2_EV_CTRL=3)。
#ifndef NAV2_INPUT_BRIDGE__ALIGN_STATE_MACHINE_HPP_
#define NAV2_INPUT_BRIDGE__ALIGN_STATE_MACHINE_HPP_

#include <Eigen/Geometry>
#include <array>
#include <cstdint>

namespace nav2_input_bridge {

// 状态机枚举：FAST-LIVO2 ↔ mavros 对齐过程的状态
//   INIT           — 构造后初始态，仅停留一拍即进入 WAITING_DATA（用于日志/调试）
//   WAITING_DATA   — 等待 8 项输入条件全部就绪（spec §2.4）
//   READY_TO_LATCH — 输入就绪，调用 latchFromInputs() 计算 T_ENU_odom
//   LATCHED        — 已锁存，T_ENU_odom 偏移量稳定（稳态；只能由 relatch 服务退出）
//   RELATCHING     — 锁存后 off_yaw 超阈值（>0.2 rad），触发重锁存
//   FATAL          — 60s 仍未就绪 或 重锁存超 3 次，进入致命态（只能由 ~/input_bridge/relatch
//   服务退出）
enum class AlignState : std::uint8_t {
  INIT = 0,
  WAITING_DATA = 1,
  READY_TO_LATCH = 2,
  LATCHED = 3,
  RELATCHING = 4,
  FATAL = 5
};

// 输入快照：进入 READY_TO_LATCH 所需的所有 8 项条件
//   全部为 true 且 time_consistency_diff_s <= time_consistency_max_s 时，才视为就绪。
struct AlignInputs {
  bool aft_mapped_seen = false;         // 是否至少收到一帧 /aft_mapped_to_init（FAST-LIVO2 SLAM 输出）
  bool mavros_global_valid = false;     // /mavros/global_position/global 是否给出可用 fix（非 NO_FIX，坐标有限）
  bool mavros_heading_valid = false;    // /mavros/global_position/compass_hdg 是否给出有限且在 [-720,720] 范围内的航向
  bool gp_origin_verified = false;      // /mavros/global_position/gp_origin 是否收到并经范围校验（lat ∈ [-90,90] 等）
  bool gps_quality_ok = false;          // GPS 质量门：fix_type>=6 (RTK_FIXED) && sat>=12 && h_acc<=500mm（参数可配）
  bool ekf2_cov_low = false;            // /mavros/local_position/odom 协方差：sqrt(cov[0]+cov[7]+cov[14]) <= max_position_rmse（默认 0.5 m）, cov[35] <= max_yaw_variance（默认 0.05 rad²）
  bool ekf2_state_ok = false;           // /mavros/local_position/odom 是否正在被发布（任何一帧即认为 EKF2 在线）
  double time_consistency_diff_s = 0.0; // 时间一致性 |stamp(slam) - stamp(local_odom)|，需 <= time_consistency_max_s
  double time_consistency_max_s = 0.5;  // 默认阈值，可在外部配置

  // 首次收到的 SLAM 位姿 T_odom_base（用作相对锚点）
  Eigen::Isometry3d first_aft_mapped{Eigen::Isometry3d::Identity()};
  // 由 mavros global+compass_hdg 算出的 T_ENU_base 锚点
  Eigen::Isometry3d T_ENU_base_latch{Eigen::Isometry3d::Identity()};

  // 构造 T_ENU_base_latch 所需的最新原始值（evaluateAlign() 读取后调用 llaToEnu）
  double last_lat = 0.0; // GPS 纬度（度）
  double last_lon = 0.0; // GPS 经度（度）
  double last_alt = 0.0; // GPS 高度（m）
  // 注意：mavros 的 compass_hdg 单位是「度 [0, 360)」，构造旋转矩阵前必须 *M_PI/180
  double last_compass_hdg_deg = 0.0;
};

class AlignStateMachine {
public:
  // 构造函数：默认状态为 INIT，偏移量为单位阵
  AlignStateMachine();

  // 静态数学工具：给定锁存瞬间的 T_ENU_base 与 T_odom_base，计算 T_ENU_odom = T_ENU_base · T_odom_base⁻¹
  // 即把 SLAM 局部 odom 坐标系对齐到 ENU (map) 坐标系下。
  static Eigen::Isometry3d computeOffset(const Eigen::Isometry3d &T_ENU_base,
                                         const Eigen::Isometry3d &T_odom_base);

  // 把锁存后的偏移量施加到任意 T_odom_base 上：T_ENU_base = T_ENU_odom · T_odom_base
  // 若尚未锁存，返回单位阵（调用方应据此判定是否发布）。
  [[nodiscard]] Eigen::Isometry3d applyOffset(const Eigen::Isometry3d &T_odom_base) const;

  // 状态机主入口：注入一帧 AlignInputs 快照。
  //   - 在 WAITING_DATA 时检查 allInputsReady()，满足则跳到 READY_TO_LATCH；
  //   - 在 READY_TO_LATCH 时调 latchFromInputs() 锁存，按 off_yaw 阈值决定走向 LATCHED 还是 RELATCHING；
  //   - 在 RELATCHING 时累计 attempts>3 后跳 FATAL，否则回到 WAITING_DATA；
  //   - LATCHED / FATAL 为稳态，需要 requestRelatch() 才能跳出。
  // 返回：本次调用是否发生了状态迁移（便于上层发布 ~/state topic）。
  [[nodiscard]] bool update(const AlignInputs &inputs);

  // 强制重锁存：服务 ~/input_bridge/relatch 调用。
  //   仅当 LATCHED 或 FATAL 时才生效，跳到 RELATCHING 并清零 attempts。
  void requestRelatch();

  // 强制进入 FATAL：evaluateAlign() 在 fatal_timeout_s_ 超时时调用，
  //   绕过 update() 路径，避免 WAITING_DATA + 输入未就绪时 while 死锁。
  void forceFatal();

  // 状态查询/访问器
  AlignState state() const { return state_; }
  const Eigen::Isometry3d &latchedOffset() const { return T_ENU_odom_; } // 已锁存的 T_ENU_odom
  int relatchAttempts() const { return relatch_attempts_; }              // 重锁存累计次数
  double fatalElapsedS() const { return fatal_elapsed_s_; } // 在 WAITING/RELATCHING 累计秒数
  void addFatalElapsedS(double dt) { fatal_elapsed_s_ += dt; } // 外部定时器每拍 0.1s 累加
  void resetFatalElapsedS() { fatal_elapsed_s_ = 0.0; }        // 进入 READY_TO_LATCH 时清零

  // 监控/测试用：最近一次锁存的标量指标
  double lastOffYaw() const { return last_off_yaw_; }       // ‖R_z - e_z‖，检测 pitch/roll
  double lastYawOffset() const { return last_yaw_offset_; } // atan2(R(1,0), R(0,0))，Y 轴旋转角

  // 仅供测试：清空锁存状态
  void resetLatched() {
    latched_ = false;
    T_ENU_odom_ = Eigen::Isometry3d::Identity();
  }

private:
  // 8 项条件全部为 true 才返回 true
  bool allInputsReady(const AlignInputs &i) const;
  // 用 first_aft_mapped + T_ENU_base_latch 计算并锁存 T_ENU_odom_
  void latchFromInputs(const AlignInputs &i);

  AlignState state_ = AlignState::INIT;
  Eigen::Isometry3d T_ENU_odom_ = Eigen::Isometry3d::Identity(); // 锁存后的偏移量
  bool latched_ = false;        // 是否处于 LATCHED
  int relatch_attempts_ = 0;    // 重锁存累计次数
  double fatal_elapsed_s_ = 0.0; // 等待/重锁存阶段累计时间
  double last_off_yaw_ = 0.0;    // 最近一次 off_yaw（监控用）
  double last_yaw_offset_ = 0.0; // 最近一次 yaw_offset（监控用）
};

} // namespace nav2_input_bridge

#endif // NAV2_INPUT_BRIDGE__ALIGN_STATE_MACHINE_HPP_
