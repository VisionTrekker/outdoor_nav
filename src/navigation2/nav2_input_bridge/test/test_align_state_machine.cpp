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

// Test 10: configurable off_yaw threshold accepts a rotation that the
// default 0.2 rad threshold would reject.
//   R(0.4 X) has off_yaw ≈ 0.397 — rejected by default (0.2) but accepted
//   when threshold is raised to 0.5.
TEST(AlignSM, latch_with_custom_off_yaw_threshold_allows_small_latch) {
  nav2_input_bridge::AlignStateMachine::Config cfg;
  cfg.relatch_off_yaw_threshold_rad = 0.5;
  AlignStateMachine sm(cfg);
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  sm.update(i); // → WAITING_DATA
  sm.update(i); // → READY_TO_LATCH
  sm.update(i); // → LATCHED (off_yaw≈0.397 < cfg threshold 0.5)
  EXPECT_EQ(sm.state(), AlignState::LATCHED);
  EXPECT_GT(sm.lastOffYaw(), 0.2); // 实际值仍 > 默认阈值 (0.2)
  EXPECT_LT(sm.lastOffYaw(), 0.5); // 但 < 自定义阈值 (0.5)
}

// Test 11: configurable off_yaw threshold rejects when over the limit.
//   R(0.6 X) has off_yaw ≈ 0.591 — also rejected by the new threshold 0.5.
TEST(AlignSM, latch_with_custom_off_yaw_threshold_rejects_large_latch) {
  nav2_input_bridge::AlignStateMachine::Config cfg;
  cfg.relatch_off_yaw_threshold_rad = 0.5;
  AlignStateMachine sm(cfg);
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX()).toRotationMatrix();
  sm.update(i); // → WAITING_DATA
  sm.update(i); // → READY_TO_LATCH
  sm.update(i); // → RELATCHING (off_yaw≈0.591 > cfg threshold 0.5)
  EXPECT_EQ(sm.state(), AlignState::RELATCHING);
  EXPECT_EQ(sm.relatchAttempts(), 1);
}

// Test 12: configurable max_attempts reaches FATAL sooner.
//   max_attempts=2 means after 2 full relatch cycles (counter reaches 4 in the
//   READY-then-RELATCHING-then-update transition pattern), state goes FATAL.
TEST(AlignSM, relatch_max_attempts_two_goes_fatal) {
  nav2_input_bridge::AlignStateMachine::Config cfg;
  cfg.relatch_max_attempts = 2;
  AlignStateMachine sm(cfg);
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  // 7 步 trace:
  //   u0: INIT→WAIT
  //   u1: WAIT→READY
  //   u2: READY→RELATCHING (attempts=1, enter=0)
  //   u3: RELATCHING interval gate (0<0 false) → 放行; attempts=2, 2>2 false → WAIT
  //   u4: WAIT→READY
  //   u5: READY→RELATCHING (attempts=3, enter=0)
  //   u6: RELATCHING: attempts=4, 4>2 true → FATAL
  for (int n = 0; n < 7; ++n) {
    sm.update(i);
  }
  EXPECT_EQ(sm.state(), AlignState::FATAL);
  EXPECT_GE(sm.relatchAttempts(), 3);
}

// Test 13: relatch_interval_s blocks update() during the wait window.
//   interval_s=5: after entering RELATCHING, update() with <5s of ticked
//   time must NOT advance; only after 5s ticked can it proceed.
TEST(AlignSM, relatch_interval_blocks_update_during_wait) {
  nav2_input_bridge::AlignStateMachine::Config cfg;
  cfg.relatch_interval_s = 5.0;
  AlignStateMachine sm(cfg);
  AlignInputs i = allReady();
  i.T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  sm.update(i); // → WAITING_DATA
  sm.update(i); // → READY_TO_LATCH
  sm.update(i); // → RELATCHING (entry timestamp recorded)
  EXPECT_EQ(sm.state(), AlignState::RELATCHING);
  // 在 5s 等待窗内连续 tick + update: 必须保持 RELATCHING
  for (int n = 0; n < 4; ++n) {
    sm.tick(1.0);  // 共推进 4s
    sm.update(i);
    EXPECT_EQ(sm.state(), AlignState::RELATCHING)
      << "tick #" << (n + 1) << " (累计 " << (n + 1) << "s): 不应前进";
  }
  // 跨过 5s 阈值后再 tick + update: 应前进到 WAITING_DATA (默认 max_attempts=3)
  sm.tick(2.0);  // 累计 6s
  sm.update(i);
  EXPECT_EQ(sm.state(), AlignState::WAITING_DATA);
}
