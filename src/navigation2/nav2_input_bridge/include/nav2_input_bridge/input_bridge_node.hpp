// Copyright 2026
#ifndef NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
#define NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace nav2_input_bridge
{

class InputBridgeNode : public rclcpp::Node
{
public:
  explicit InputBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg, const std::string & source);

  void onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg);
  void publishStopGoal();

  void llaToEnu(
    double lat_deg, double lon_deg, double alt_m,
    double lat0_deg, double lon0_deg, double alt0_m,
    double * east_m, double * north_m, double * up_m) const;

  std::string local_odom_topic_;
  std::string goal_input_topic_;
  std::string uav_goal_input_topic_;
  std::string target_detected_topic_;
  std::string goal_output_topic_;
  std::string goal_output_frame_;
  bool require_goal_fix_;
  bool goal_yaw_from_bearing_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_detected_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex origin_mutex_;
  bool origin_valid_{false};
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};

  std::mutex state_mutex_;
  bool target_detected_{false};
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  static constexpr double WGS84_R_{6378137.0};
};

}  // namespace nav2_input_bridge

#endif  // NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
