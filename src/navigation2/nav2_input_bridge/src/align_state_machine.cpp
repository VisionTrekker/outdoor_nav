// Copyright 2026, outdoor_nav maintainers.
#include "nav2_input_bridge/align_state_machine.hpp"

namespace nav2_input_bridge
{

AlignStateMachine::AlignStateMachine() = default;

Eigen::Isometry3d AlignStateMachine::computeOffset(
  const Eigen::Isometry3d & T_ENU_base,
  const Eigen::Isometry3d & T_odom_base)
{
  return T_ENU_base * T_odom_base.inverse();
}

Eigen::Isometry3d AlignStateMachine::applyOffset(const Eigen::Isometry3d & T_odom_base) const
{
  if (!latched_) {
    return Eigen::Isometry3d::Identity();
  }
  return T_ENU_odom_ * T_odom_base;
}

bool AlignStateMachine::allInputsReady(const AlignInputs & i) const
{
  return i.aft_mapped_seen && i.mavros_global_valid && i.mavros_heading_valid &&
         i.gp_origin_verified && i.gps_quality_ok && i.ekf2_cov_low &&
         i.ekf2_state_ok && (i.time_consistency_diff_s <= i.time_consistency_max_s);
}

void AlignStateMachine::latchFromInputs(const AlignInputs & i)
{
  T_ENU_odom_ = computeOffset(i.T_ENU_base_latch, i.first_aft_mapped);
  // Extract scalars for monitoring.
  last_yaw_offset_ = atan2(T_ENU_odom_.linear()(1, 0), T_ENU_odom_.linear()(0, 0));
  last_off_yaw_ = std::fabs(T_ENU_odom_.linear()(0, 1) + T_ENU_odom_.linear()(1, 0)) / 2.0;
  latched_ = true;
}

bool AlignStateMachine::update(const AlignInputs & i)
{
  const AlignState prev = state_;
  switch (state_) {
    case AlignState::INIT:
      state_ = AlignState::WAITING_DATA;
      break;
    case AlignState::WAITING_DATA:
      if (allInputsReady(i)) {
        state_ = AlignState::READY_TO_LATCH;
      }
      break;
    case AlignState::READY_TO_LATCH:
      latchFromInputs(i);
      relatch_attempts_ = 0;
      resetFatalElapsedS();
      if (last_off_yaw_ > 0.2) {
        state_ = AlignState::RELATCHING;
        relatch_attempts_ = 1;
      } else {
        state_ = AlignState::LATCHED;
      }
      break;
    case AlignState::LATCHED:
      // LATCHED is steady-state. Only relatch via service.
      break;
    case AlignState::RELATCHING:
      // Caller (InputBridgeNode) sleeps 5s then re-enters WAITING_DATA via update().
      relatch_attempts_++;
      if (relatch_attempts_ > 3) {
        state_ = AlignState::FATAL;
      } else {
        state_ = AlignState::WAITING_DATA;
      }
      break;
    case AlignState::FATAL:
      // Steady-state; only ~/input_bridge/relatch can exit.
      break;
  }
  return state_ != prev;
}

void AlignStateMachine::requestRelatch()
{
  if (state_ == AlignState::LATCHED || state_ == AlignState::FATAL) {
    state_ = AlignState::RELATCHING;
    relatch_attempts_ = 0;
    resetFatalElapsedS();
  }
}

}  // namespace nav2_input_bridge
