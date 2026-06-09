// Copyright 2026, outdoor_nav maintainers.
// Unit tests for AlignStateMachine 5-state machine.
#include <gtest/gtest.h>
#include "nav2_input_bridge/align_state_machine.hpp"

using nav2_input_bridge::AlignInputs;
using nav2_input_bridge::AlignState;
using nav2_input_bridge::AlignStateMachine;

namespace
{
AlignInputs allReady()
{
  AlignInputs i;
  i.aft_mapped_seen = true;
  i.mavros_global_valid = true;
  i.mavros_heading_valid = true;
  i.gp_origin_verified = true;
  i.gps_quality_ok = true;
  i.ekf2_cov_low = true;
  i.ekf2_state_ok = true;
  i.time_consistency_diff_s = 0.1;
  i.time_consistency_max_s = 0.5;
  i.first_aft_mapped = Eigen::Isometry3d::Identity();
  i.T_ENU_base_latch = Eigen::Isometry3d::Identity();
  return i;
}
}  // namespace

// Test 1: INIT → WAITING_DATA on first update
TEST(AlignSM, init_to_waiting_on_first_update) {
  AlignStateMachine sm;
  EXPECT_EQ(sm.state(), AlignState::INIT);
  sm.update(AlignInputs{});
  EXPECT_EQ(sm.state(), AlignState::WAITING_DATA);
}

// Test 2: WAITING_DATA → READY_TO_LATCH when all 8 conditions met
TEST(AlignSM, waiting_to_ready_when_all_ready) {
  AlignStateMachine sm;
  sm.update(allReady()); // u1: INIT→WAITING_DATA
  EXPECT_EQ(sm.state(), AlignState::WAITING_DATA);
  sm.update(allReady()); // u2: WAITING_DATA→READY_TO_LATCH
  EXPECT_EQ(sm.state(), AlignState::READY_TO_LATCH);
  sm.update(allReady()); // u3: READY_TO_LATCH→LATCHED (identity offset, off_yaw=0)
  EXPECT_EQ(sm.state(), AlignState::LATCHED);
  EXPECT_TRUE(sm.latchedOffset().isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

// Test 3: stays in WAITING_DATA if one condition missing
TEST(AlignSM, waiting_blocks_on_missing_condition) {
  AlignStateMachine sm;
  AlignInputs i = allReady();
  i.gps_quality_ok = false;
  sm.update(i);
  EXPECT_EQ(sm.state(), AlignState::WAITING_DATA);
}

// Test 4: LATCHED with gravity-aligned offset → stays LATCHED
TEST(AlignSM, latch_succeeds_with_small_off_yaw) {
  AlignStateMachine sm;
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  sm.update(i); // u1: INIT→WAITING_DATA
  sm.update(i); // u2: WAITING_DATA→READY_TO_LATCH
  sm.update(i); // u3: READY_TO_LATCH→LATCHED (off_yaw small for R(0.01 Z))
  EXPECT_EQ(sm.state(), AlignState::LATCHED);
  EXPECT_LT(sm.lastOffYaw(), 0.1);
}

// Test 5: LATCHED with off_yaw > 0.2 → RELATCHING
TEST(AlignSM, latch_rejected_aligned_too_poor) {
  AlignStateMachine sm;
  AlignInputs i = allReady();
  // Off-yaw > 0.2: introduce a non-yaw rotation (roll=π/4)
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  sm.update(i); // u1: INIT→WAITING_DATA
  sm.update(i); // u2: WAITING_DATA→READY_TO_LATCH
  sm.update(i); // u3: READY_TO_LATCH→RELATCHING (off_yaw>0.2 for R(π/4 X))
  EXPECT_EQ(sm.state(), AlignState::RELATCHING);
  EXPECT_EQ(sm.relatchAttempts(), 1);
}

// Test 6: RELATCHING → WAITING_DATA after 1 attempt
TEST(AlignSM, relatch_returns_to_waiting) {
  AlignStateMachine sm;
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  sm.update(i);  // → WAITING_DATA
  sm.update(i);  // → READY_TO_LATCH
  sm.update(i);  // → RELATCHING
  sm.update(i);  // → WAITING_DATA
  EXPECT_EQ(sm.state(), AlignState::WAITING_DATA);
}

// Test 7: 3 RELATCHING cycles → FATAL
TEST(AlignSM, three_relatch_attempts_go_fatal) {
  AlignStateMachine sm;
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  for (int n = 0; n < 12; ++n) {
    sm.update(i);
  }
  EXPECT_EQ(sm.state(), AlignState::FATAL);
  EXPECT_GE(sm.relatchAttempts(), 3);
}

// Test 8: requestRelatch from LATCHED goes to RELATCHING
TEST(AlignSM, service_relatch_from_latched) {
  AlignStateMachine sm;
  AlignInputs i = allReady();
  sm.update(i); // u1: INIT→WAITING_DATA
  sm.update(i); // u2: WAITING_DATA→READY_TO_LATCH
  sm.update(i); // u3: READY_TO_LATCH→LATCHED (identity T_ENU_base_latch, off_yaw=0)
  EXPECT_EQ(sm.state(), AlignState::LATCHED);
  sm.requestRelatch();
  EXPECT_EQ(sm.state(), AlignState::RELATCHING);
  EXPECT_EQ(sm.relatchAttempts(), 0);
}

// Test 9 (bonus): applyOffset before latch returns identity
TEST(AlignSM, apply_offset_unlatched_returns_identity) {
  AlignStateMachine sm;
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(5, 6, 7);
  Eigen::Isometry3d r = sm.applyOffset(t);
  EXPECT_TRUE(r.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}
