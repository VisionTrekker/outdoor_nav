// Copyright 2026, outdoor_nav maintainers.
// Unit tests for AlignStateMachine math helpers.
#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Geometry>
#include "nav2_input_bridge/align_state_machine.hpp"

namespace nav2_input_bridge {

// Test 1: identity offset — SLAM odom == mavros ENU, T_ENU_odom = I
TEST(AlignMath, identity_offset) {
  Eigen::Isometry3d T_odom_base_latch = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_ENU_base_latch = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_ENU_odom = AlignStateMachine::computeOffset(T_ENU_base_latch, T_odom_base_latch);
  EXPECT_TRUE(T_ENU_odom.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

// Test 2: yaw offset 90° — SLAM odom X = ENU North, mavros yaw = East (π/2)
TEST(AlignMath, yaw_offset_90deg) {
  Eigen::Isometry3d T_odom_base_latch = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_ENU_base_latch = Eigen::Isometry3d::Identity();
  T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T_ENU_odom = AlignStateMachine::computeOffset(T_ENU_base_latch, T_odom_base_latch);
  double yaw_offset = atan2(T_ENU_odom.linear()(1, 0), T_ENU_odom.linear()(0, 0));
  EXPECT_NEAR(yaw_offset, M_PI / 2, 1e-6);
}

// Test 3: combined yaw + translation — yaw=π/4, trans=(3,4,0)
TEST(AlignMath, combined_yaw_translation) {
  Eigen::Isometry3d T_odom_base_latch = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_ENU_base_latch = Eigen::Isometry3d::Identity();
  T_ENU_base_latch.linear() =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_ENU_base_latch.translation() = Eigen::Vector3d(3.0, 4.0, 0.0);
  Eigen::Isometry3d T_ENU_odom = AlignStateMachine::computeOffset(T_ENU_base_latch, T_odom_base_latch);
  EXPECT_NEAR(T_ENU_odom.translation().x(), 3.0, 1e-9);
  EXPECT_NEAR(T_ENU_odom.translation().y(), 4.0, 1e-9);
  double yaw = atan2(T_ENU_odom.linear()(1, 0), T_ENU_odom.linear()(0, 0));
  EXPECT_NEAR(yaw, M_PI / 4, 1e-6);
}

}  // namespace nav2_input_bridge
