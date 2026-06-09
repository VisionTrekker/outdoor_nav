// Copyright 2026
#ifndef NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
#define NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/gpsraw.hpp"
#include "nav2_input_bridge/align_state_machine.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Geometry>

namespace nav2_input_bridge {

class InputBridgeNode : public rclcpp::Node {
public:
  explicit InputBridgeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using FollowPath = nav2_msgs::action::FollowPath;

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg, const std::string &source);

  void onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg);
  void publishStopGoal();

  // ===== SLAM align path callbacks =====
  void onSlamOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg);
  void onLocalOdomAlign(const nav_msgs::msg::Odometry::SharedPtr msg);
  void evaluateAlign();
  void publishAlignedPose(const nav_msgs::msg::Odometry::SharedPtr &odom_msg);
  void onRelatchService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onCompassHdg(const std_msgs::msg::Float32::SharedPtr msg);
  void onGpOrigin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg);

  void llaToEnu(double lat_deg, double lon_deg, double alt_m, double lat0_deg, double lon0_deg,
                double alt0_m, double *east_m, double *north_m, double *up_m) const;

  // ===== SLAM align path =====
  std::unique_ptr<nav2_input_bridge::AlignStateMachine> align_sm_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_raw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_; // for ekf2 covariance
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relatch_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr align_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr compass_hdg_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_origin_sub_;

  // Cached latest inputs (filled by callbacks, consumed by align evaluation).
  AlignInputs cached_inputs_;
  std::mutex align_inputs_mutex_;
  rclcpp::TimerBase::SharedPtr align_eval_timer_;

  // ROS 2 parameters
  bool enable_slam_align_ = false;
  std::string slam_odom_topic_;
  std::string slam_vision_pose_topic_;
  std::string gps_raw_topic_;
  int gps_fix_type_min_ = 6;
  int gps_sat_min_ = 12;
  int gps_h_acc_max_mm_ = 500;
  double ekf2_max_position_rmse_ = 0.5;
  double ekf2_max_yaw_variance_ = 0.05;
  int relatch_max_attempts_ = 3;
  double relatch_interval_s_ = 5.0;
  double relatch_off_yaw_threshold_rad_ = 0.2;
  double fatal_timeout_s_ = 60.0;
  bool keep_publishing_on_mavros_death_ = true;
  bool auto_relatch_on_slam_die_ = false;
  double slam_die_timeout_s_ = 30.0;
  rclcpp::Time slam_last_msg_time_;
  rclcpp::Time state_start_time_;

  std::string local_odom_topic_;
  std::string goal_input_topic_;
  std::string uav_goal_input_topic_;
  std::string target_detected_topic_;
  std::string follow_path_action_topic_;
  std::string goal_output_topic_;
  std::string goal_output_frame_;
  bool require_goal_fix_;
  bool goal_yaw_from_bearing_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_detected_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_planner_pub_;

  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_action_client_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex origin_mutex_;
  bool origin_valid_{false};
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};

  static constexpr double WGS84_R_{6378137.0};
};

} // namespace nav2_input_bridge

#endif // NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
