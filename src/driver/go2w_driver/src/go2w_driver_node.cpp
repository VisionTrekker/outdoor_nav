#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include "sport_client.hpp"

using namespace std::chrono_literals;

class Go2WDriverNode : public rclcpp::Node {
public:
  Go2WDriverNode()
      : Node("go2w_driver"), sport_client_(this), last_cmd_time_(now()),
        last_vx_(0.0f), last_vy_(0.0f), last_vyaw_(0.0f),
        cmd_received_(false), robot_ready_(false) {

    this->declare_parameter("vx_max", 1.5);
    this->declare_parameter("vy_max", 0.6);
    this->declare_parameter("vyaw_max", 1.0);
    this->declare_parameter("cmd_timeout_sec", 0.5);
    this->declare_parameter("startup_delay_sec", 2.0);
    this->declare_parameter("require_ready", true);

    float vx_max = static_cast<float>(this->get_parameter("vx_max").as_double());
    float vy_max = static_cast<float>(this->get_parameter("vy_max").as_double());
    float vyaw_max = static_cast<float>(this->get_parameter("vyaw_max").as_double());
    cmd_timeout_ = this->get_parameter("cmd_timeout_sec").as_double();
    double startup_delay = this->get_parameter("startup_delay_sec").as_double();
    bool require_ready = this->get_parameter("require_ready").as_bool();

    // 为了在开发机上测试，这里先手动改为了 robot_ready_
    if (!require_ready) {
      robot_ready_ = true;
      RCLCPP_WARN(get_logger(), "require_ready=false, skipping robot state check");
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this, vx_max, vy_max, vyaw_max](
            const geometry_msgs::msg::Twist::SharedPtr cmd) {
          if (!robot_ready_) {
            RCLCPP_WARN(get_logger(), "Robot not ready, ignoring cmd_vel");
            return;
          }
          last_cmd_time_ = now();
          cmd_received_ = true;

          last_vx_ = std::clamp(static_cast<float>(cmd->linear.x), -vx_max, vx_max);
          last_vy_ = std::clamp(static_cast<float>(cmd->linear.y), -vy_max, vy_max);
          last_vyaw_ = std::clamp(static_cast<float>(cmd->angular.z), -vyaw_max, vyaw_max);

          sport_client_.Move(req_, last_vx_, last_vy_, last_vyaw_);
          RCLCPP_DEBUG(get_logger(), "Move: vx=%.2f vy=%.2f vyaw=%.2f", last_vx_, last_vy_, last_vyaw_);
        });

    state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        "/lf/sportmodestate", 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
          state_ = *msg;
          state_received_ = true;

          if (msg->mode == 1 && !robot_ready_) {
            robot_ready_ = true;
            RCLCPP_INFO(get_logger(), "Robot ready (mode=1, balance stand)");
          }
        });

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/go2w/imu", 10);
    battery_pub_ =
        create_publisher<std_msgs::msg::Float32>("/go2w/battery", 10);

    move_timer_ = create_wall_timer(
        100ms, [this]() {
          if (!robot_ready_) {
            return;
          }

          double elapsed = (now() - last_cmd_time_).seconds();
          if (cmd_received_ && elapsed > cmd_timeout_) {
            sport_client_.StopMove(req_);
            cmd_received_ = false;
            last_vx_ = last_vy_ = last_vyaw_ = 0.0f;
            RCLCPP_DEBUG(get_logger(), "cmd_vel timeout, StopMove sent");
          } else if (cmd_received_) {
            sport_client_.Move(req_, last_vx_, last_vy_, last_vyaw_);
          }
        });

    state_timer_ = create_wall_timer(
        50ms, [this]() {
          if (!state_received_) {
            return;
          }

          sensor_msgs::msg::Imu imu_msg;
          imu_msg.header.stamp = now();
          imu_msg.header.frame_id = "go2w_imu_link";

          const auto &q = state_.imu_state.quaternion;
          imu_msg.orientation.x = q[0];
          imu_msg.orientation.y = q[1];
          imu_msg.orientation.z = q[2];
          imu_msg.orientation.w = q[3];

          const auto &gyro = state_.imu_state.gyroscope;
          imu_msg.angular_velocity.x = gyro[0];
          imu_msg.angular_velocity.y = gyro[1];
          imu_msg.angular_velocity.z = gyro[2];

          const auto &accel = state_.imu_state.accelerometer;
          imu_msg.linear_acceleration.x = accel[0];
          imu_msg.linear_acceleration.y = accel[1];
          imu_msg.linear_acceleration.z = accel[2];

          for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = 0;
            imu_msg.angular_velocity_covariance[i] = 0;
            imu_msg.linear_acceleration_covariance[i] = 0;
          }
          imu_msg.orientation_covariance[0] = -1;

          imu_pub_->publish(imu_msg);
        });

    startup_timer_ = create_wall_timer(
        std::chrono::duration<double>(startup_delay),
        [this]() {
          startup_timer_->cancel();
          RCLCPP_INFO(get_logger(), "Sending BalanceStand command...");
          sport_client_.BalanceStand(req_);
        });

    RCLCPP_INFO(get_logger(), "Go2W driver node initialized");
    RCLCPP_INFO(get_logger(), "  vx_max=%.2f, vy_max=%.2f, vyaw_max=%.2f",
                vx_max, vy_max, vyaw_max);
    RCLCPP_INFO(get_logger(), "  cmd_timeout=%.2fs", cmd_timeout_);
  }

private:
  SportClient sport_client_;
  unitree_api::msg::Request req_;
  unitree_go::msg::SportModeState state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;

  rclcpp::TimerBase::SharedPtr move_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  rclcpp::Time last_cmd_time_;
  float last_vx_;
  float last_vy_;
  float last_vyaw_;
  bool cmd_received_;
  bool robot_ready_;
  bool state_received_{false};
  double cmd_timeout_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2WDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
