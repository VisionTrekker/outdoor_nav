// Copyright 2026

#include "nav2_input_bridge/input_bridge_node.hpp"

#include <cmath>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_input_bridge
{

InputBridgeNode::InputBridgeNode(const rclcpp::NodeOptions & options)
: Node("nav2_input_bridge", options)
{
  local_odom_topic_ = declare_parameter("local_odom_topic", std::string("/mavros/local_position/odom"));
  goal_input_topic_ = declare_parameter("goal_input_topic", std::string("/gp_goal"));
  uav_goal_input_topic_ = declare_parameter("uav_goal_input_topic", std::string("/uav/target_gps"));
  target_detected_topic_ = declare_parameter("target_detected_topic", std::string("/target_detected"));
  goal_output_topic_ = declare_parameter("goal_output_topic", std::string("/goal_pose"));
  goal_output_frame_ = declare_parameter("goal_output_frame", std::string("map"));
  require_goal_fix_ = declare_parameter("require_nav_sat_fix", true);
  goal_yaw_from_bearing_ = declare_parameter("goal_yaw_from_bearing", false);
  follow_path_action_topic_ = declare_parameter("follow_path_action_topic", std::string("/follow_path"));

  const double ref_lat = declare_parameter("reference_latitude", 0.0);
  const double ref_lon = declare_parameter("reference_longitude", 0.0);
  const double ref_alt = declare_parameter("reference_altitude", 0.0);
  {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    if (std::abs(ref_lat) > 90.0 || std::abs(ref_lon) > 180.0) {
      origin_valid_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "Invalid ENU reference: latitude in [-90,90], longitude in [-180,180]");
    } else {
      origin_lat_ = ref_lat;
      origin_lon_ = ref_lon;
      origin_alt_ = ref_alt;
      origin_valid_ = true;
      RCLCPP_INFO(
        get_logger(),
        "ENU origin from parameters: lat=%.8f deg, lon=%.8f deg, alt=%.3f m",
        ref_lat, ref_lon, ref_alt);
    }
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rclcpp::QoS odom_qos(10);
  odom_qos.best_effort();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    local_odom_topic_, odom_qos,
    std::bind(&InputBridgeNode::onOdom, this, std::placeholders::_1));

  rclcpp::QoS goal_qos(10);
  goal_qos.best_effort();
  goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    goal_input_topic_, goal_qos,
    std::bind(&InputBridgeNode::onGoalFix, this, std::placeholders::_1));

  uav_goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    uav_goal_input_topic_, goal_qos,
    std::bind(&InputBridgeNode::onUavGoalFix, this, std::placeholders::_1));

  target_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
    target_detected_topic_, 10,
    std::bind(&InputBridgeNode::onTargetDetected, this, std::placeholders::_1));

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_output_topic_, 10);

  stop_planner_pub_ = create_publisher<std_msgs::msg::Bool>("stop_planner", 10);

  follow_path_action_client_ = rclcpp_action::create_client<FollowPath>(
    this,
    follow_path_action_topic_);
}

void InputBridgeNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg->header.frame_id.empty() || msg->child_frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Odometry: empty frame_id or child_frame_id");
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = msg->header.frame_id;
  t.child_frame_id = msg->child_frame_id;
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  // t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.translation.z = 0.0;
  t.transform.rotation = msg->pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

void InputBridgeNode::llaToEnu(
  double lat_deg, double lon_deg, double alt_m,
  double lat0_deg, double lon0_deg, double alt0_m,
  double * east_m, double * north_m, double * up_m) const
{
  // Spherical tangent-plane projection (same algorithm used in PX4 MapProjection)
  const double R = WGS84_R_;
  const double lat_r = lat_deg * M_PI / 180.0;
  const double lon_r = lon_deg * M_PI / 180.0;
  const double lat0_r = lat0_deg * M_PI / 180.0;
  const double lon0_r = lon0_deg * M_PI / 180.0;
  const double sin_lat = sin(lat_r);
  const double cos_lat = cos(lat_r);
  const double sin_lat0 = sin(lat0_r);
  const double cos_lat0 = cos(lat0_r);
  const double dlon = lon_r - lon0_r;
  const double cos_dlon = cos(dlon);

  double arg = sin_lat0 * sin_lat + cos_lat0 * cos_lat * cos_dlon;
  arg = std::min(1.0, std::max(-1.0, arg));
  const double c = acos(arg);
  double k = 1.0;
  if (fabs(c) > 1e-6) {
    k = c / sin(c);
  }
  const double north = k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * cos_dlon) * R;
  const double east  = k * cos_lat * sin(dlon) * R;

  *east_m  = east;
  *north_m = north;
  *up_m    = alt_m - alt0_m;
}

// 发布目标位置（GPS 坐标投影转换到 局部ENU坐标）
void InputBridgeNode::processGoalFix(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg,
  const std::string & source)
{
  if (require_goal_fix_ &&
    msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s goal has no GPS fix", source.c_str());
    return;
  }

  bool have_origin = false;
  double lat0 = 0.0, lon0 = 0.0, alt0 = 0.0;
  {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    have_origin = origin_valid_;
    lat0 = origin_lat_;
    lon0 = origin_lon_;
    alt0 = origin_alt_;
  }

  if (!have_origin) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "ENU reference invalid: fix reference_latitude, reference_longitude, reference_altitude");
    return;
  }

  double east = 0.0, north = 0.0, up = 0.0;
  llaToEnu(
    msg->latitude, msg->longitude, msg->altitude,
    lat0, lon0, alt0,
    &east, &north, &up);

  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = goal_output_frame_;
  out.pose.position.x = east;
  out.pose.position.y = north;
  out.pose.position.z = up;

  if (goal_yaw_from_bearing_) {
    const double yaw = std::atan2(east, north);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    out.pose.orientation = tf2::toMsg(q);
  } else {
    out.pose.orientation.w = 1.0;
  }

  goal_pub_->publish(out);

  RCLCPP_INFO(
    get_logger(),
    "[%s] Goal published: E=%.2f m, N=%.2f m, U=%.2f m",
    source.c_str(), east, north, up);
}

void InputBridgeNode::onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  processGoalFix(msg, "manual");
}

void InputBridgeNode::onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  processGoalFix(msg, "uav");
}

// 发送取消导航任务指令
void InputBridgeNode::publishStopGoal()
{
  RCLCPP_WARN(get_logger(), "[yolo] Target detected! Cancelling current FollowPath tasks...");

  // 先通知 straight_planner 停止发送新目标（不依赖 action server）
  if (stop_planner_pub_) {
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = true;
    stop_planner_pub_->publish(stop_msg);
    RCLCPP_INFO(get_logger(), "[yolo] (1) Sent stop signal to straight_planner.");
  }

  // 再尝试取消 FollowPath action
  if (!follow_path_action_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "FollowPath action server not ready yet, skipping cancel.");
    return;
  }
  follow_path_action_client_->async_cancel_all_goals();

  RCLCPP_INFO(get_logger(), "[yolo] (2) Sent cancel request to FollowPath action server.");
}

// 处理目标检测信号
void InputBridgeNode::onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }

  publishStopGoal();
}

}  // namespace nav2_input_bridge
