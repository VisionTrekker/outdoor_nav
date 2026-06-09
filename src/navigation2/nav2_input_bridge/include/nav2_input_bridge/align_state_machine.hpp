// Copyright 2026, outdoor_nav maintainers.
// Pure C++ state machine for FAST-LIVO2 ↔ mavros ENU frame alignment.
// No rclcpp dependency — fully unit-testable.
#ifndef NAV2_INPUT_BRIDGE__ALIGN_STATE_MACHINE_HPP_
#define NAV2_INPUT_BRIDGE__ALIGN_STATE_MACHINE_HPP_

#include <array>
#include <cstdint>
#include <Eigen/Geometry>

namespace nav2_input_bridge
{

enum class AlignState : std::uint8_t
{
  INIT = 0,
  WAITING_DATA = 1,
  READY_TO_LATCH = 2,
  LATCHED = 3,
  RELATCHING = 4,
  FATAL = 5
};

// Snapshot of all 8 conditions required to enter READY_TO_LATCH (spec §2.4).
struct AlignInputs
{
  bool aft_mapped_seen = false;        // at least one /aft_mapped_to_init message received
  bool mavros_global_valid = false;    // /mavros/global_position/global with usable fix
  bool mavros_heading_valid = false;   // /mavros/global_position/compass_hdg with valid heading
  bool gp_origin_verified = false;     // /mavros/global_position/gp_origin received and validated
  bool gps_quality_ok = false;         // mavros GPSRAW: fix_type>=6, sat>=12, h_acc<=500 mm
  bool ekf2_cov_low = false;           // /mavros/local_position/odom covariance within bounds
  bool ekf2_state_ok = false;          // /mavros/local_position/odom is being published
  double time_consistency_diff_s = 0.0;  // |stamp(slam) - stamp(local_odom)| in seconds
  double time_consistency_max_s = 0.5;   // configured threshold (timestamps within 0.5 s)

  // Latch material:
  Eigen::Isometry3d first_aft_mapped{Eigen::Isometry3d::Identity()};  // T_odom_base at first msg
  Eigen::Isometry3d T_ENU_base_latch{Eigen::Isometry3d::Identity()};  // constructed from mavros global+heading

  // Latest raw values for constructing T_ENU_base_latch on demand (Task 6):
  double last_lat = 0.0;
  double last_lon = 0.0;
  double last_alt = 0.0;
  double last_compass_hdg_deg = 0.0;
};

class AlignStateMachine
{
public:
  AlignStateMachine();

  // Static math helper: T_ENU_odom = T_ENU_base · T_odom_base⁻¹
  static Eigen::Isometry3d computeOffset(
    const Eigen::Isometry3d & T_ENU_base,
    const Eigen::Isometry3d & T_odom_base);

  // Apply latched offset to a per-msg odom pose. Returns identity if not latched.
  [[nodiscard]] Eigen::Isometry3d applyOffset(const Eigen::Isometry3d & T_odom_base) const;

  // Feed current snapshot of inputs. Updates state_ and (on transition to LATCHED)
  // latches T_ENU_odom_ from the stored first_aft_mapped_ and T_ENU_base_latch_.
  // Returns true if state changed.
  [[nodiscard]] bool update(const AlignInputs & inputs);

  // Force re-latch (e.g. service call). Goes to RELATCHING.
  void requestRelatch();

  AlignState state() const { return state_; }
  const Eigen::Isometry3d & latchedOffset() const { return T_ENU_odom_; }
  int relatchAttempts() const { return relatch_attempts_; }
  double fatalElapsedS() const { return fatal_elapsed_s_; }
  void addFatalElapsedS(double dt) { fatal_elapsed_s_ += dt; }
  void resetFatalElapsedS() { fatal_elapsed_s_ = 0.0; }

  // For tests / monitoring.
  double lastOffYaw() const { return last_off_yaw_; }
  double lastYawOffset() const { return last_yaw_offset_; }

  // For tests to clear latch.
  void resetLatched() { latched_ = false; T_ENU_odom_ = Eigen::Isometry3d::Identity(); }

private:
  bool allInputsReady(const AlignInputs & i) const;
  void latchFromInputs(const AlignInputs & i);

  AlignState state_ = AlignState::INIT;
  Eigen::Isometry3d T_ENU_odom_ = Eigen::Isometry3d::Identity();
  bool latched_ = false;
  int relatch_attempts_ = 0;
  double fatal_elapsed_s_ = 0.0;
  double last_off_yaw_ = 0.0;
  double last_yaw_offset_ = 0.0;
};

}  // namespace nav2_input_bridge

#endif  // NAV2_INPUT_BRIDGE__ALIGN_STATE_MACHINE_HPP_
