// Copyright 2026

#include "nav2_input_bridge/input_bridge_node.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_input_bridge {

InputBridgeNode::InputBridgeNode(const rclcpp::NodeOptions &options)
  : Node("nav2_input_bridge", options) {
  local_odom_topic_ =
    declare_parameter("local_odom_topic", std::string("/mavros/local_position/odom"));
  goal_input_topic_ = declare_parameter("goal_input_topic", std::string("/gp_goal"));
  uav_goal_input_topic_ = declare_parameter("uav_goal_input_topic", std::string("/uav/target_gps"));
  target_detected_topic_ =
    declare_parameter("target_detected_topic", std::string("/target_detected"));
  goal_output_topic_ = declare_parameter("goal_output_topic", std::string("/goal_pose"));
  goal_output_frame_ = declare_parameter("goal_output_frame", std::string("map"));
  require_goal_fix_ = declare_parameter("require_nav_sat_fix", true);
  goal_yaw_from_bearing_ = declare_parameter("goal_yaw_from_bearing", false);
  follow_path_action_topic_ =
    declare_parameter("follow_path_action_topic", std::string("/follow_path"));

  // Align path parameters
  this->declare_parameter("enable_slam_align", false);
  this->declare_parameter("slam_odom_topic", "/aft_mapped_to_init");
  this->declare_parameter("slam_vision_pose_topic", "/mavros/vision_pose/pose");
  this->declare_parameter("gps_raw_topic", "/mavros/gpsstatus/gps1/raw");
  this->declare_parameter("gps_quality.fix_type_min", 6);
  this->declare_parameter("gps_quality.sat_min", 12);
  this->declare_parameter("gps_quality.h_acc_max_mm", 500);
  this->declare_parameter("ekf2.max_position_rmse", 0.5);
  this->declare_parameter("ekf2.max_yaw_variance", 0.05);
  this->declare_parameter("relatch.max_attempts", 3);
  this->declare_parameter("relatch.interval_s", 5.0);
  this->declare_parameter("relatch.off_yaw_threshold_rad", 0.2);
  this->declare_parameter("fatal_timeout_s", 60.0);
  this->declare_parameter("keep_publishing_on_mavros_death", true);
  this->declare_parameter("auto_relatch_on_slam_die", false);
  this->declare_parameter("slam_die_timeout_s", 30.0);

  const double ref_lat = declare_parameter("reference_latitude", 0.0);
  const double ref_lon = declare_parameter("reference_longitude", 0.0);
  const double ref_alt = declare_parameter("reference_altitude", 0.0);
  {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    if (std::abs(ref_lat) > 90.0 || std::abs(ref_lon) > 180.0) {
      origin_valid_ = false;
      RCLCPP_ERROR(get_logger(),
                   "Invalid ENU reference: latitude in [-90,90], longitude in [-180,180]");
    } else {
      origin_lat_ = ref_lat;
      origin_lon_ = ref_lon;
      origin_alt_ = ref_alt;
      origin_valid_ = true;
      RCLCPP_INFO(get_logger(),
                  "ENU origin from parameters: lat=%.8f deg, lon=%.8f deg, alt=%.3f m", ref_lat,
                  ref_lon, ref_alt);
    }
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rclcpp::QoS odom_qos(10);
  odom_qos.best_effort();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    local_odom_topic_, odom_qos, std::bind(&InputBridgeNode::onOdom, this, std::placeholders::_1));

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

  follow_path_action_client_ =
    rclcpp_action::create_client<FollowPath>(this, follow_path_action_topic_);

  this->get_parameter("enable_slam_align", enable_slam_align_);
  if (enable_slam_align_) {
    this->get_parameter("slam_odom_topic", slam_odom_topic_);
    this->get_parameter("slam_vision_pose_topic", slam_vision_pose_topic_);
    this->get_parameter("gps_raw_topic", gps_raw_topic_);
    this->get_parameter("gps_quality.fix_type_min", gps_fix_type_min_);
    this->get_parameter("gps_quality.sat_min", gps_sat_min_);
    this->get_parameter("gps_quality.h_acc_max_mm", gps_h_acc_max_mm_);
    this->get_parameter("ekf2.max_position_rmse", ekf2_max_position_rmse_);
    this->get_parameter("ekf2.max_yaw_variance", ekf2_max_yaw_variance_);
    this->get_parameter("relatch.max_attempts", relatch_max_attempts_);
    this->get_parameter("relatch.interval_s", relatch_interval_s_);
    this->get_parameter("relatch.off_yaw_threshold_rad", relatch_off_yaw_threshold_rad_);
    this->get_parameter("fatal_timeout_s", fatal_timeout_s_);
    this->get_parameter("keep_publishing_on_mavros_death", keep_publishing_on_mavros_death_);
    this->get_parameter("auto_relatch_on_slam_die", auto_relatch_on_slam_die_);
    this->get_parameter("slam_die_timeout_s", slam_die_timeout_s_);

    align_sm_ = std::make_unique<nav2_input_bridge::AlignStateMachine>();
    state_start_time_ = this->get_clock()->now();

    slam_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      slam_odom_topic_, 10, std::bind(&InputBridgeNode::onSlamOdom, this, std::placeholders::_1));
    gps_raw_sub_ = this->create_subscription<mavros_msgs::msg::GPSRAW>(
      gps_raw_topic_, 10, std::bind(&InputBridgeNode::onGpsRaw, this, std::placeholders::_1));
    local_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", 10,
      std::bind(&InputBridgeNode::onLocalOdomAlign, this, std::placeholders::_1));
    vision_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(slam_vision_pose_topic_, 10);
    relatch_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/input_bridge/relatch", std::bind(&InputBridgeNode::onRelatchService, this,
                                          std::placeholders::_1, std::placeholders::_2));
    align_state_pub_ = this->create_publisher<std_msgs::msg::String>("~/state", 10);

    compass_hdg_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/mavros/global_position/compass_hdg", 10,
      std::bind(&InputBridgeNode::onCompassHdg, this, std::placeholders::_1));
    gp_origin_sub_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
      "/mavros/global_position/gp_origin", 10,
      std::bind(&InputBridgeNode::onGpOrigin, this, std::placeholders::_1));

    align_eval_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&InputBridgeNode::evaluateAlign, this));
  }
}

void InputBridgeNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->header.frame_id.empty() || msg->child_frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
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

void InputBridgeNode::llaToEnu(double lat_deg, double lon_deg, double alt_m, double lat0_deg,
                               double lon0_deg, double alt0_m, double *east_m, double *north_m,
                               double *up_m) const {
  // Spherical tangent-plane projection (same algorithm used in PX4 MapProjection)
  // For 5km+ scenarios, switch to the ellipsoid algorithm (see commented block below)
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
  const double east = k * cos_lat * sin(dlon) * R;

  *east_m = east;
  *north_m = north;
  *up_m = alt_m - alt0_m;

#if 0
  // =============================================================================
  // Ellipsoid projection (WGS84) — for scenarios beyond 5km range
  //
  // Replaces the spherical approximation above. The ellipsoid model eliminates
  // the ~0.1-0.3% spherical error (e.g., ~15m at 5km).
  //
  // WGS84 ellipsoid parameters
  // const double a = 6378137.0;                // Semi-major axis (m)
  // const double f = 1.0 / 298.257223563;    // Flattening
  // const double e2 = f * (2.0 - f);         // First eccentricity squared
  //
  // Convert geodetic (lat, lon, alt) to ECEF (X, Y, Z)
  // auto lla_to_ecef = [&](double lat, double lon, double alt,
  //                        double & X, double & Y, double & Z) {
  //   const double phi = lat * M_PI / 180.0;
  //   const double lam = lon * M_PI / 180.0;
  //   const double sin_phi = sin(phi);
  //   const double cos_phi = cos(phi);
  //   const double sin_lam = sin(lam);
  //   const double cos_lam = cos(lam);
  //   const double N = a / sqrt(1.0 - e2 * sin_phi * sin_phi);
  //   X = (N + alt) * cos_phi * cos_lam;
  //   Y = (N + alt) * cos_phi * sin_lam;
  //   Z = (N * (1.0 - e2) + alt) * sin_phi;
  // };
  //
  // double X, Y, Z;
  // double X0, Y0, Z0;
  // lla_to_ecef(lat_deg, lon_deg, alt_m, X, Y, Z);
  // lla_to_ecef(lat0_deg, lon0_deg, alt0_m, X0, Y0, Z0);
  //
  // ECEF delta
  // const double dx = X - X0;
  // const double dy = Y - Y0;
  // const double dz = Z - Z0;
  //
  // Rotate ECEF delta to ENU using origin's geodetic position
  // const double phi0 = lat0_deg * M_PI / 180.0;
  // const double lam0 = lon0_deg * M_PI / 180.0;
  // const double sin_phi0 = sin(phi0);
  // const double cos_phi0 = cos(phi0);
  // const double sin_lam0 = sin(lam0);
  // const double cos_lam0 = cos(lam0);
  //
  // ENU rotation matrix (applied to ECEF delta vector)
  // *east_m  = -sin_lam0 * dx           + cos_lam0 * dy;
  // *north_m = -sin_phi0 * cos_lam0 * dx - sin_phi0 * sin_lam0 * dy + cos_phi0 * dz;
  // *up_m    =  cos_phi0 * cos_lam0 * dx + cos_phi0 * sin_lam0 * dy + sin_phi0 * dz;
  // =============================================================================
#endif
}

// 发布目标位置（GPS 坐标投影转换到 局部ENU坐标）
void InputBridgeNode::processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                                     const std::string &source) {
  if (require_goal_fix_ && msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s goal has no GPS fix",
                         source.c_str());
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
  llaToEnu(msg->latitude, msg->longitude, msg->altitude, lat0, lon0, alt0, &east, &north, &up);

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

  RCLCPP_INFO(get_logger(), "[%s] Goal published: E=%.2f m, N=%.2f m, U=%.2f m", source.c_str(),
              east, north, up);
}

void InputBridgeNode::onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  processGoalFix(msg, "manual");
  if (!enable_slam_align_)
    return;
  // Only cache align input if the fix is usable (not NO_FIX) and values are finite.
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      !std::isfinite(msg->altitude)) {
    return;
  }
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  cached_inputs_.last_lat = msg->latitude;
  cached_inputs_.last_lon = msg->longitude;
  cached_inputs_.last_alt = msg->altitude;
  cached_inputs_.mavros_global_valid = true;
}

void InputBridgeNode::onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  processGoalFix(msg, "uav");
  if (!enable_slam_align_)
    return;
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      !std::isfinite(msg->altitude)) {
    return;
  }
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  cached_inputs_.last_lat = msg->latitude;
  cached_inputs_.last_lon = msg->longitude;
  cached_inputs_.last_alt = msg->altitude;
  cached_inputs_.mavros_global_valid = true;
}

// 发送取消导航任务指令
void InputBridgeNode::publishStopGoal() {
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
void InputBridgeNode::onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data) {
    return;
  }

  publishStopGoal();
}

// ===== SLAM align path =====

void InputBridgeNode::onSlamOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  // CRITICAL safety: reject NaN/Inf position components before they reach the
  // latched transform that PX4 EKF2 will fuse for vehicle control.
  if (!std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y) ||
      !std::isfinite(msg->pose.pose.position.z)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onSlamOdom: rejected non-finite position");
    return;
  }
  const double qw = msg->pose.pose.orientation.w;
  const double qx = msg->pose.pose.orientation.x;
  const double qy = msg->pose.pose.orientation.y;
  const double qz = msg->pose.pose.orientation.z;
  if (!std::isfinite(qw) || !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onSlamOdom: rejected non-finite quaternion");
    return;
  }
  const double qnorm_sq = qw * qw + qx * qx + qy * qy + qz * qz;
  if (qnorm_sq < 1e-6 || std::fabs(qnorm_sq - 1.0) > 0.05) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onSlamOdom: rejected non-unit quaternion (|q|^2=%.4f)", qnorm_sq);
    return;
  }

  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  if (!cached_inputs_.aft_mapped_seen) {
    cached_inputs_.first_aft_mapped = Eigen::Isometry3d::Identity();
    cached_inputs_.first_aft_mapped.translation() = Eigen::Vector3d(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    cached_inputs_.first_aft_mapped.linear() = q.toRotationMatrix();
  }
  cached_inputs_.aft_mapped_seen = true;
  slam_last_msg_time_ = this->get_clock()->now();

  if (align_sm_->state() == nav2_input_bridge::AlignState::LATCHED) {
    publishAlignedPose(msg);
  }
}

void InputBridgeNode::onGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  // Use satellites_visible (per docs/PX4_mavros_topic_info.md) — not status.satellites_used
  cached_inputs_.gps_quality_ok = (msg->fix_type >= static_cast<uint8_t>(gps_fix_type_min_)) &&
                                  (msg->satellites_visible >= static_cast<uint8_t>(gps_sat_min_)) &&
                                  (msg->h_acc <= static_cast<uint32_t>(gps_h_acc_max_mm_));
}

void InputBridgeNode::onLocalOdomAlign(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  const auto &c = msg->pose.covariance; // 36-element row-major 6x6
  // Validate the four covariance terms we use are finite and non-negative.
  if (!std::isfinite(c[0]) || !std::isfinite(c[7]) || !std::isfinite(c[14]) ||
      !std::isfinite(c[35]) || c[0] < 0.0 || c[7] < 0.0 || c[14] < 0.0 || c[35] < 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onLocalOdomAlign: rejected non-finite or negative covariance");
    return;
  }
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  double pos_trace = c[0] + c[7] + c[14];
  double yaw_var = c[35];
  cached_inputs_.ekf2_cov_low =
    (std::sqrt(pos_trace) <= ekf2_max_position_rmse_) && (yaw_var <= ekf2_max_yaw_variance_);
  cached_inputs_.ekf2_state_ok = true; // local_odom arriving means EKF2 is publishing
}

void InputBridgeNode::evaluateAlign() {
  if (!enable_slam_align_ || !align_sm_)
    return;

  AlignInputs snap;

  {
    std::lock_guard<std::mutex> lock(align_inputs_mutex_);
    snap = cached_inputs_;

    // Construct T_ENU_base_latch from latest mavros global + heading
    if (snap.mavros_global_valid && snap.mavros_heading_valid) {
      double x_e = 0, y_n = 0, z_u = 0;
      std::lock_guard<std::mutex> olock(origin_mutex_);
      if (origin_valid_) {
        this->llaToEnu(snap.last_lat, snap.last_lon, snap.last_alt, origin_lat_, origin_lon_,
                       origin_alt_, &x_e, &y_n, &z_u);
        Eigen::Isometry3d T_ENU_base_snap = Eigen::Isometry3d::Identity();
        T_ENU_base_snap.translation() = Eigen::Vector3d(x_e, y_n, z_u);
        // compass_hdg is DEGREES [0, 360) — must deg2rad
        T_ENU_base_snap.linear() =
          Eigen::AngleAxisd(snap.last_compass_hdg_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
        snap.T_ENU_base_latch = T_ENU_base_snap;
      }
    }
  }

  // FATAL timeout: count time in WAITING_DATA/RELATCHING, transition to FATAL after 60s
  if (align_sm_->state() == nav2_input_bridge::AlignState::WAITING_DATA ||
      align_sm_->state() == nav2_input_bridge::AlignState::RELATCHING) {
    align_sm_->addFatalElapsedS(0.1);
    if (align_sm_->fatalElapsedS() >= fatal_timeout_s_) {
      // Force FATAL by repeated relatch until attempts>3
      while (align_sm_->state() != nav2_input_bridge::AlignState::FATAL) {
        (void)align_sm_->update(snap);
      }
    }
  }

  // Drive state machine
  const AlignState prev_state = align_sm_->state();
  (void)align_sm_->update(snap);

  // Publish state for monitoring
  if (align_sm_->state() != prev_state && align_state_pub_) {
    std_msgs::msg::String s;
    switch (align_sm_->state()) {
    case nav2_input_bridge::AlignState::INIT:
      s.data = "INIT";
      break;
    case nav2_input_bridge::AlignState::WAITING_DATA:
      s.data = "WAITING_DATA";
      break;
    case nav2_input_bridge::AlignState::READY_TO_LATCH:
      s.data = "READY_TO_LATCH";
      break;
    case nav2_input_bridge::AlignState::LATCHED:
      s.data = "LATCHED";
      break;
    case nav2_input_bridge::AlignState::RELATCHING:
      s.data = "RELATCHING";
      break;
    case nav2_input_bridge::AlignState::FATAL:
      s.data = "FATAL";
      break;
    }
    align_state_pub_->publish(s);
  }

  if (align_sm_->state() == nav2_input_bridge::AlignState::LATCHED) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                         "LATCH COMPLETE: yaw_offset=%.4f rad (%.2f deg), off_yaw=%.4f rad, "
                         "T_ENU_odom.t=[%.3f, %.3f, %.3f]",
                         align_sm_->lastYawOffset(), align_sm_->lastYawOffset() * 180.0 / M_PI,
                         align_sm_->lastOffYaw(), align_sm_->latchedOffset().translation().x(),
                         align_sm_->latchedOffset().translation().y(),
                         align_sm_->latchedOffset().translation().z());
  }
}

void InputBridgeNode::publishAlignedPose(const nav_msgs::msg::Odometry::SharedPtr &odom_msg) {
  if (!align_sm_ || !vision_pose_pub_)
    return;
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  T_odom_base.translation() = Eigen::Vector3d(
    odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
  T_odom_base.linear() =
    Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                       odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
      .toRotationMatrix();

  Eigen::Isometry3d T_ENU_base = align_sm_->applyOffset(T_odom_base);
  if (T_ENU_base.isApprox(Eigen::Isometry3d::Identity(), 1e-9))
    return;

  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = odom_msg->header.stamp;
  out.header.frame_id = "map";
  out.pose.position.x = T_ENU_base.translation().x();
  out.pose.position.y = T_ENU_base.translation().y();
  out.pose.position.z = T_ENU_base.translation().z();
  Eigen::Quaterniond q(T_ENU_base.linear());
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
  vision_pose_pub_->publish(out);
}

void InputBridgeNode::onRelatchService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!align_sm_) {
    response->success = false;
    response->message = "align not enabled";
    return;
  }
  align_sm_->requestRelatch();
  response->success = true;
  response->message = "relatch requested";
}

void InputBridgeNode::onCompassHdg(const std_msgs::msg::Float32::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  if (!std::isfinite(msg->data) || msg->data < -720.0f || msg->data > 720.0f) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onCompassHdg: rejected non-finite or out-of-range value %.2f", msg->data);
    return;
  }
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  cached_inputs_.last_compass_hdg_deg = msg->data;
  cached_inputs_.mavros_heading_valid = true;
}

void InputBridgeNode::onGpOrigin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  const double lat = msg->position.latitude;
  const double lon = msg->position.longitude;
  const double alt = msg->position.altitude;
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || !std::isfinite(alt)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "onGpOrigin: rejected out-of-range or non-finite (lat=%.4f lon=%.4f alt=%.2f)", lat, lon,
      alt);
    return;
  }
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  cached_inputs_.gp_origin_verified = true;
}

} // namespace nav2_input_bridge
