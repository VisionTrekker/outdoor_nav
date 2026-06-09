// Copyright 2026
//
// 节点说明：
//   本文件声明 nav2_input_bridge ROS 2 节点。本节点承担两个相互独立的职责：
//
//   1) 目标点路径（goal path）—— 把 GPS 目标点（/gp_goal、/uav/target_gps）从
//      WGS84 lat/lon 通过 PX4 球面切平面投影到 ENU，发布 /goal_pose 给
//      nav2_straight_planner；并在 YOLO 触发 /target_detected 时通知
//      straight_planner 停止 + 取消 FollowPath action。
//
//   2) SLAM 对齐路径（slam align path，可选，参数 enable_slam_align 开启）——
//      在 FAST-LIVO2 与 PX4 EKF2 之间建立坐标偏移：把 /aft_mapped_to_init
//      （odom 坐标系下的 SLAM 输出）映射到 ENU (map) 下，发布到
//      /mavros/vision_pose/pose，作为 EKF2 的 EV 外部位置观测
//      （EKF2_EV_CTRL=3）。
//
// 锁顺序约定（避免死锁）：align_inputs_mutex_ → origin_mutex_。
// 任意函数获取多把锁时必须按此顺序。
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

  // ===== 通用 odom 与目标点路径回调 =====
  // onOdom：把 PX4 /mavros/local_position/odom 通过 TF 广播为 odom→base_link
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  // processGoalFix：共享的 GPS→ENU→/goal_pose 处理函数（两个 topic 共用）
  void processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg, const std::string &source);

  void onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg);
  void publishStopGoal();

  // ===== SLAM 对齐路径回调（仅在 enable_slam_align=true 时挂载） =====
  // onSlamOdom：/aft_mapped_to_init FAST-LIVO2 SLAM 输出，作为对齐锚点 + 持续 source
  void onSlamOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  // onGpsRaw：/mavros/gpsstatus/gps1/raw 用于 GPS 质量门（fix_type/sat/h_acc）
  void onGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg);
  // onLocalOdomAlign：/mavros/local_position/odom 用于 EKF2 协方差
  void onLocalOdomAlign(const nav_msgs::msg::Odometry::SharedPtr msg);
  // evaluateAlign：100ms 定时器，注入缓存到 align_sm_，驱动状态机
  void evaluateAlign();
  // publishAlignedPose：LATCHED 后把对齐后的 T_ENU_base 发到 vision_pose_pub_
  void publishAlignedPose(const nav_msgs::msg::Odometry::SharedPtr &odom_msg);
  // onRelatchService：~/input_bridge/relatch 服务处理函数
  void onRelatchService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // onCompassHdg：/mavros/global_position/compass_hdg 单位为度 [0, 360)
  void onCompassHdg(const std_msgs::msg::Float32::SharedPtr msg);
  // onGpOrigin：/mavros/global_position/gp_origin ENU 原点
  void onGpOrigin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg);

  // PX4 球面切平面投影 lat/lon/alt → ENU（与 PX4 MapProjection 一致）
  //   5km+ 场景下可改用 #if 1 启用文件内已注释好的椭球算法
  void llaToEnu(double lat_deg, double lon_deg, double alt_m, double lat0_deg, double lon0_deg,
                double alt0_m, double *east_m, double *north_m, double *up_m) const;

  // ===== SLAM 对齐路径成员 =====
  std::unique_ptr<nav2_input_bridge::AlignStateMachine> align_sm_; // 状态机实例
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_raw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_; // 用于 EKF2 协方差
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_; // → EKF2
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relatch_srv_;      // ~/input_bridge/relatch
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr align_state_pub_; // ~/state 状态广播
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr compass_hdg_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_origin_sub_;

  // 缓存：各回调写入，evaluateAlign 读取并拷贝快照
  AlignInputs cached_inputs_;
  std::mutex align_inputs_mutex_;                 // 保护 cached_inputs_
  rclcpp::TimerBase::SharedPtr align_eval_timer_; // 100ms 评估定时器

  // ROS 2 参数（声明默认值；具体值由 launch 文件 yaml 覆盖）
  bool enable_slam_align_ = false;              // 启用 SLAM 对齐路径
  std::string slam_odom_topic_;                 // 默认 /aft_mapped_to_init
  std::string slam_vision_pose_topic_;          // 默认 /mavros/vision_pose/pose
  std::string gps_raw_topic_;                   // 默认 /mavros/gpsstatus/gps1/raw
  int gps_fix_type_min_ = 6;                    // 最低 fix_type（RTK_FIXED）
  int gps_sat_min_ = 12;                        // 最少可见卫星数
  int gps_h_acc_max_mm_ = 500;                  // 最大水平精度（mm）
  double ekf2_max_position_rmse_ = 0.5;         // EKF2 位置 RMSE 阈值（m）
  double ekf2_max_yaw_variance_ = 0.05;         // EKF2 yaw 方差上限（rad²）
  int relatch_max_attempts_ = 3;                // 重锁存最大次数（注入 align_sm_ Config）
  double relatch_interval_s_ = 5.0;             // 重锁存最小间隔（s, 注入 Config）
  double relatch_off_yaw_threshold_rad_ = 0.2;  // off_yaw 阈值（rad, 注入 Config）
  double fatal_timeout_s_ = 60.0;               // FATAL 超时（s, 父节点用)
  rclcpp::Time slam_last_msg_time_;             // SLAM 最近一帧时间戳
  rclcpp::Time local_odom_last_msg_time_;       // local_odom 最近一帧时间戳（用于 DC-8）

  // 目标点路径相关参数
  std::string local_odom_topic_;         // 默认 /mavros/local_position/odom
  std::string goal_input_topic_;         // 默认 /gp_goal
  std::string uav_goal_input_topic_;     // 默认 /uav/target_gps
  std::string target_detected_topic_;    // 默认 /target_detected
  std::string follow_path_action_topic_; // 默认 /follow_path
  std::string goal_output_topic_;        // 默认 /goal_pose
  std::string goal_output_frame_;        // 默认 map
  bool require_goal_fix_;                // 是否要求 GPS 有效 fix
  bool goal_yaw_from_bearing_;           // 是否用 bearing 计算 yaw

  // 目标点路径相关订阅/发布
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_detected_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_planner_pub_; // 通知 straight_planner 停止

  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_action_client_; // 用于 cancel goals

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ENU 原点状态（来自 reference_latitude/longitude/altitude 参数）
  std::mutex origin_mutex_; // 保护 ENU 原点
  bool origin_valid_{false};
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};

  // WGS84 球面投影的等效半径（米），与 PX4 MapProjection 一致
  static constexpr double WGS84_R_{6378137.0};
};

} // namespace nav2_input_bridge

#endif // NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
