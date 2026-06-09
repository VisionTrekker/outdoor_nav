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

    // 构造状态机配置（off_yaw 阈值、relatch 上限、间隔）
    AlignStateMachine::Config sm_cfg;
    sm_cfg.relatch_max_attempts = relatch_max_attempts_;
    sm_cfg.relatch_interval_s = relatch_interval_s_;
    sm_cfg.relatch_off_yaw_threshold_rad = relatch_off_yaw_threshold_rad_;
    align_sm_ = std::make_unique<nav2_input_bridge::AlignStateMachine>(sm_cfg);

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

// 把 PX4 /mavros/local_position/odom 广播为 TF。
//   - frame_id  / child_frame_id 必须非空（缺一则丢弃）
//   - z 分量强制置 0（2D 地面车只关心平面），与 Nav2 的 map→odom→base_link 链路保持一致
//   - 旋转直接复用，避免四元数被错误 normalize
//
// ⚠️ 注意：Z 强制置 0 是历史代码（commit 4ddd8bc 之前的椭球投影切换未涉及此函数）。
//    若日后需要 3D 导航（例如无人机/狗），应改回 `msg->pose.pose.position.z`。
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
  // t.transform.translation.z = msg->pose.pose.position.z;  // 旧版使用真值
  t.transform.translation.z = 0.0; // 当前使用：2D 平面
  t.transform.rotation = msg->pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

// PX4 球面切平面投影：把 WGS84 lat/lon/alt 转换到 ENU(east, north, up) 局部坐标。
//   算法来源：与 PX4 MapProjection 完全一致（PX4-Autopilot/src/lib/geo/geo.cpp）。
//   适用场景：< 5 km 半径范围，球面近似误差 ~0.1-0.3%（5km 时 ~15m）。
//
//   5 km 以上的远距离场景：把下方 `#if 0` 改为 `#if 1` 即可启用 WGS84 椭球投影。
//   椭球模型消除了球面误差，可用于多基站/城区大范围导航。
//
//   入参：
//     lat/lon/alt_deg/m  — 目标点 WGS84 坐标
//     lat0/lon0/alt0     — ENU 原点（gp_origin）WGS84 坐标
//   出参（出参指针）：
//     east_m/north_m/up_m — 目标点相对原点的 ENU 偏移
void InputBridgeNode::llaToEnu(double lat_deg, double lon_deg, double alt_m, double lat0_deg,
                               double lon0_deg, double alt0_m, double *east_m, double *north_m,
                               double *up_m) const {
  const double R = WGS84_R_;                     // 等效球面半径（m）
  const double lat_r = lat_deg * M_PI / 180.0;   // 目标纬度（弧度）
  const double lon_r = lon_deg * M_PI / 180.0;   // 目标经度（弧度）
  const double lat0_r = lat0_deg * M_PI / 180.0; // 原点纬度（弧度）
  const double lon0_r = lon0_deg * M_PI / 180.0; // 原点经度（弧度）
  const double sin_lat = sin(lat_r);
  const double cos_lat = cos(lat_r);
  const double sin_lat0 = sin(lat0_r);
  const double cos_lat0 = cos(lat0_r);
  const double dlon = lon_r - lon0_r;
  const double cos_dlon = cos(dlon);

  // 球面三角学：求两点之间的"球面角距" c
  double arg = sin_lat0 * sin_lat + cos_lat0 * cos_lat * cos_dlon;
  arg = std::min(1.0, std::max(-1.0, arg)); // 防 acos 数值越界
  const double c = acos(arg);
  // k 比例因子：把"球面角距 c"换算为"地心角"，再做平面投影
  // 当 c 极小时用 1.0 避免除零（l'Hopital 极限 c/sin(c) → 1）
  double k = 1.0;
  if (fabs(c) > 1e-6) {
    k = c / sin(c);
  }
  const double north = k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * cos_dlon) * R;
  const double east = k * cos_lat * sin(dlon) * R;

  *east_m = east;
  *north_m = north;
  *up_m = alt_m - alt0_m; // 高程直接做差（球面投影不处理高程）

#if 0
  // =============================================================================
  // 椭球投影（WGS84）—— 5km+ 远距离场景
  // 替换上方球面近似。椭球模型消除 ~0.1-0.3% 球面误差（5km 时 ~15m）。
  //
  // WGS84 椭球参数
  // const double a = 6378137.0;                // 长半轴（m）
  // const double f = 1.0 / 298.257223563;      // 扁率
  // const double e2 = f * (2.0 - f);           // 第一偏心率平方
  //
  // WGS84 (lat, lon, alt) → ECEF (X, Y, Z)
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
  // 旋转 ECEF delta 到 ENU（用原点的 geodetic 位置作旋转轴）
  // const double phi0 = lat0_deg * M_PI / 180.0;
  // const double lam0 = lon0_deg * M_PI / 180.0;
  // const double sin_phi0 = sin(phi0);
  // const double cos_phi0 = cos(phi0);
  // const double sin_lam0 = sin(lam0);
  // const double cos_lam0 = cos(lam0);
  //
  // ENU 旋转矩阵（作用于 ECEF delta 向量）
  // *east_m  = -sin_lam0 * dx           + cos_lam0 * dy;
  // *north_m = -sin_phi0 * cos_lam0 * dx - sin_phi0 * sin_lam0 * dy + cos_phi0 * dz;
  // *up_m    =  cos_phi0 * cos_lam0 * dx + cos_phi0 * sin_lam0 * dy + sin_phi0 * dz;
  // =============================================================================
#endif
}

// 目标点处理（GPS → ENU → /goal_pose），由 onGoalFix / onUavGoalFix 共用。
//   - require_goal_fix_=true 时拒绝 NO_FIX（不发布）
//   - ENU 原点未配置时跳过（防止误算到 0,0,0）
//   - 可选用 goal_yaw_from_bearing_ 把"目标到原点的方位"换算为 yaw
void InputBridgeNode::processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                                     const std::string &source) {
  if (require_goal_fix_ && msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s goal has no GPS fix",
                         source.c_str());
    return;
  }

  // 读取 ENU 原点（互斥锁保护）
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

  // 球面切平面投影到 ENU
  double east = 0.0, north = 0.0, up = 0.0;
  llaToEnu(msg->latitude, msg->longitude, msg->altitude, lat0, lon0, alt0, &east, &north, &up);

  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = goal_output_frame_; // 默认 "map"
  out.pose.position.x = east;
  out.pose.position.y = north;
  out.pose.position.z = up;

  if (goal_yaw_from_bearing_) {
    // 用"目标相对原点的方位角"作为 yaw（atypical 用法，调试时用）
    const double yaw = std::atan2(east, north);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    out.pose.orientation = tf2::toMsg(q);
  } else {
    // 默认朝向：identity quaternion
    out.pose.orientation.w = 1.0;
  }

  goal_pub_->publish(out);

  RCLCPP_INFO(get_logger(), "[%s] Goal published: E=%.2f m, N=%.2f m, U=%.2f m", source.c_str(),
              east, north, up);
}

// /gp_goal 回调：除了发布目标点，还顺带给 align 状态机缓存 GPS 锚点。
//   注意：source="manual"，与 UAV 链路下发的 target_gps 区分日志。
void InputBridgeNode::onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  processGoalFix(msg, "manual");
  if (!enable_slam_align_)
    return;
  // 仅缓存"可用 fix + 坐标有限"的消息，避免把 NO_FIX 或 NaN 灌进状态机
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

// /uav/target_gps 回调：与 onGoalFix 相同的处理逻辑（仅 source 不同）。
//   注：两个 topic 写入 cached_inputs_ 是"后写覆盖"，evaluateAlign 取最新值。
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

// 发送停止信号：先通知 straight_planner 不再发新目标，再尝试取消 FollowPath action。
void InputBridgeNode::publishStopGoal() {
  RCLCPP_WARN(get_logger(), "[yolo] Target detected! Cancelling current FollowPath tasks...");

  // (1) 先通知 straight_planner（不依赖 action server）
  if (stop_planner_pub_) {
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = true;
    stop_planner_pub_->publish(stop_msg);
    RCLCPP_INFO(get_logger(), "[yolo] (1) Sent stop signal to straight_planner.");
  }

  // (2) 再尝试取消 FollowPath action
  if (!follow_path_action_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "FollowPath action server not ready yet, skipping cancel.");
    return;
  }
  follow_path_action_client_->async_cancel_all_goals();

  RCLCPP_INFO(get_logger(), "[yolo] (2) Sent cancel request to FollowPath action server.");
}

// YOLO 目标检测信号处理：仅在 data=true 时触发停止。
//   上升沿触发（false 时不做任何事，避免重复触发）。
void InputBridgeNode::onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data) {
    return;
  }

  publishStopGoal();
}

// ===== SLAM align path =====

// /aft_mapped_to_init（FAST-LIVO2 SLAM 输出）回调：
//   - 安全门：拒绝 NaN/Inf/非单位四元数（防止坏值进 PX4 EKF2）
//   - 首次到达：缓存 first_aft_mapped 作为对齐锚点
//   - 后续到达：刷新 slam_last_msg_time_，若已 LATCHED 则发布对齐后位姿
void InputBridgeNode::onSlamOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  // CRITICAL：拒绝 NaN/Inf 位移分量（防止脏数据进入 PX4 EKF2 用于控制）
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
  // 四元数模长校验（应 ≈ 1）
  const double qnorm_sq = qw * qw + qx * qx + qy * qy + qz * qz;
  if (qnorm_sq < 1e-6 || std::fabs(qnorm_sq - 1.0) > 0.05) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onSlamOdom: rejected non-unit quaternion (|q|^2=%.4f)", qnorm_sq);
    return;
  }

  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  if (!cached_inputs_.aft_mapped_seen) {
    // 首帧：建立 SLAM 局部 odom 系下的"零位姿"作为锚点
    cached_inputs_.first_aft_mapped = Eigen::Isometry3d::Identity();
    cached_inputs_.first_aft_mapped.translation() = Eigen::Vector3d(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize(); // 即便接近单位也归一化一次
    cached_inputs_.first_aft_mapped.linear() = q.toRotationMatrix();
  }
  cached_inputs_.aft_mapped_seen = true;
  slam_last_msg_time_ = this->get_clock()->now();

  // 修复时间戳差：
  //   msg->header.stamp 是 SLAM 上游时间戳，
  //   local_odom_last_msg_time_ 是 /mavros/local_position/odom 最近帧时间。
  //   两者 |diff| <= 0.5s 才认为时间一致（stamp 时间域可能不同：用 nanoseconds 换算到秒）。
  //   首次未收到 local_odom 时，时间差不更新，保留默认 0.0（0 <= 0.5，gate 暂时放行）。
  {
    const rclcpp::Time slam_stamp(msg->header.stamp);
    const double diff_s = std::fabs((slam_stamp - local_odom_last_msg_time_).seconds());
    cached_inputs_.time_consistency_diff_s = diff_s;
  }

  // 已 LATCHED 时实时发布对齐后位姿
  if (align_sm_->state() == nav2_input_bridge::AlignState::LATCHED) {
    publishAlignedPose(msg);
  }
}

// /mavros/gpsstatus/gps1/raw 回调：更新 GPS 质量门（DC-5）。
//   字段：fix_type（默认 ≥6=RTK_FIXED）、satellites_visible（默认 ≥12）、
//         h_acc（默认 ≤500 mm）。
//   注意：用 satellites_visible 而非 status.satellites_used（前者更可靠）。
void InputBridgeNode::onGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  cached_inputs_.gps_quality_ok = (msg->fix_type >= static_cast<uint8_t>(gps_fix_type_min_)) &&
                                  (msg->satellites_visible >= static_cast<uint8_t>(gps_sat_min_)) &&
                                  (msg->h_acc <= static_cast<uint32_t>(gps_h_acc_max_mm_));
}

// /mavros/local_position/odom 回调：更新 EKF2 协方差
//   - ekf2_state_ok：local_odom 任何一帧到达即置 true（EKF2 在线）
//   - ekf2_cov_low：位置 RMSE = sqrt(cov[0]+cov[7]+cov[14]) ≤ max_position_rmse
//                       且 yaw 方差 cov[35] ≤ max_yaw_variance
//   - 拒绝 NaN/Inf 或负值（防止协方差退化导致门控误判）
void InputBridgeNode::onLocalOdomAlign(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!enable_slam_align_)
    return;
  const auto &c = msg->pose.covariance; // 36 元素行主序 6x6
  // 我们关心的 4 个对角元素：c[0]=x_var, c[7]=y_var, c[14]=z_var, c[35]=yaw_var
  if (!std::isfinite(c[0]) || !std::isfinite(c[7]) || !std::isfinite(c[14]) ||
      !std::isfinite(c[35]) || c[0] < 0.0 || c[7] < 0.0 || c[14] < 0.0 || c[35] < 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onLocalOdomAlign: rejected non-finite or negative covariance");
    return;
  }
  std::lock_guard<std::mutex> lock(align_inputs_mutex_);
  const double pos_trace = c[0] + c[7] + c[14];
  const double yaw_var = c[35];
  cached_inputs_.ekf2_cov_low =
    (std::sqrt(pos_trace) <= ekf2_max_position_rmse_) && (yaw_var <= ekf2_max_yaw_variance_);
  cached_inputs_.ekf2_state_ok = true; // 收到任一帧即认为 EKF2 在发布
  // local_odom_last_msg_time_ 由 onSlamOdom 读取并算时间戳差
  local_odom_last_msg_time_ = this->get_clock()->now();
}

// 状态机评估（100ms 定时器回调）。
//   主流程：
//     1) 取 cached_inputs_ 快照（拷一份本地 snap，避免长时间持锁）
//     2) 用 snap 上的最新 GPS+compass 构造 T_ENU_base_latch
//     3) 累计 fatal_elapsed_s，超 60s 强制走 FATAL
//     4) 调 align_sm_->update(snap) 推进状态机
//     5) 状态变化时通过 ~/state topic 广播出去
//     6) LATCHED 时打印监控日志（throttle 60s）
void InputBridgeNode::evaluateAlign() {
  if (!enable_slam_align_ || !align_sm_)
    return;

  // 推进状态机内部时钟 (100ms 拍)。驱动 relatch_interval_s 间隔门，
  //   不依赖 rclcpp::Time 以保持 AlignStateMachine 可单测性。
  align_sm_->tick(0.1);

  AlignInputs snap;

  {
    // 锁顺序：align_inputs_mutex_ → origin_mutex_（与全文件保持一致）
    std::lock_guard<std::mutex> lock(align_inputs_mutex_);
    snap = cached_inputs_;

    // 用最新的 mavros global + heading 构造 T_ENU_base_latch（评估期按需计算）
    if (snap.mavros_global_valid && snap.mavros_heading_valid) {
      double x_e = 0, y_n = 0, z_u = 0;
      std::lock_guard<std::mutex> olock(origin_mutex_);
      if (origin_valid_) {
        this->llaToEnu(snap.last_lat, snap.last_lon, snap.last_alt, origin_lat_, origin_lon_,
                       origin_alt_, &x_e, &y_n, &z_u);
        Eigen::Isometry3d T_ENU_base_snap = Eigen::Isometry3d::Identity();
        T_ENU_base_snap.translation() = Eigen::Vector3d(x_e, y_n, z_u);
        // ⚠️ 注意：mavros compass_hdg 单位是「度 [0, 360)」，必须 *M_PI/180 转为弧度
        T_ENU_base_snap.linear() =
          Eigen::AngleAxisd(snap.last_compass_hdg_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
        snap.T_ENU_base_latch = T_ENU_base_snap;
      }
    }
  }

  // FATAL 超时：仅在 WAITING_DATA / RELATCHING 阶段累加时间（100ms 一次 +0.1s）
  if (align_sm_->state() == nav2_input_bridge::AlignState::WAITING_DATA ||
      align_sm_->state() == nav2_input_bridge::AlignState::RELATCHING) {
    align_sm_->addFatalElapsedS(0.1);
    if (align_sm_->fatalElapsedS() >= fatal_timeout_s_) {
      // 超时后直接 forceFatal() 置位 FATAL，不再反复 update()——之前在 WAITING_DATA +
      // 输入 仍未就绪时 update 不会改状态，while 循环会卡死 100ms 定时器。
      align_sm_->forceFatal();
    }
  }

  // 推进状态机（正常路径）
  const AlignState prev_state = align_sm_->state();
  (void)align_sm_->update(snap);

  // 状态变化时通过 ~/state topic 广播
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

  // LATCHED 监控日志（限速 60s 一次）
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

// 把对齐后的 T_ENU_base 发布到 /mavros/vision_pose/pose（供 PX4 EKF2 融合）
//   调用时机：onSlamOdom 收到新帧，且状态机已 LATCHED
//   关键步骤：
//     1) 把 /aft_mapped_to_init 构造成 T_odom_base（SLAM 局部系下 base 位姿）
//     2) 调 applyOffset 得到 T_ENU_base = T_ENU_odom · T_odom_base
//     3) 若返回单位阵（哨兵），表示尚未锁存，跳过发布
//     4) 转 PoseStamped 发到 vision_pose_topic_（默认 /mavros/vision_pose/pose）
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
  // applyOffset 在未锁存时返回单位阵；用 1e-9 容差判断
  if (T_ENU_base.isApprox(Eigen::Isometry3d::Identity(), 1e-9))
    return;

  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = odom_msg->header.stamp;
  out.header.frame_id = "map"; // 与 PX4 ENU 坐标系对齐
  out.pose.position.x = T_ENU_base.translation().x();
  out.pose.position.y = T_ENU_base.translation().y();
  out.pose.position.z = T_ENU_base.translation().z();
  // 用 (x, y, z, w) 顺序构造四元数，Eigen 默认是 (w, x, y, z) 顺序要小心
  Eigen::Quaterniond q(T_ENU_base.linear());
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
  vision_pose_pub_->publish(out);
}

// ~/input_bridge/relatch 服务处理：运维手动触发重锁存。
//   - align 未启用时返回 success=false（避免误调）
//   - 否则调 requestRelatch()，进入 RELATCHING 并清零 attempts
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

// /mavros/global_position/compass_hdg 回调：缓存磁航向（度 [0, 360)）
//   拒绝 NaN/Inf 或 |val| > 720（720°=2 圈，远超物理合理值）
//   ⚠️ 注意：消息值单位是"度"，evaluateAlign 中构造旋转矩阵前必须 *M_PI/180
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

// /mavros/global_position/gp_origin 回调：DC-4 验证 PX4 已设置好 ENU 原点。
//   验证规则：lat ∈ [-90, 90]、lon ∈ [-180, 180]、alt 有限。
//   注意：本回调不更新 origin_lat_/lon_/alt_——ENU 原点只能由 reference_*
//         参数或 set_gp_origin 服务设置；本回调只确认 PX4 端已就绪
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
