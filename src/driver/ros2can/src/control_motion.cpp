#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct MotionStep {
  double linear_x;
  double linear_y;
  double angular_z;
  double duration;
};

class MotionController {
public:
  explicit MotionController(const rclcpp::Node::SharedPtr &nh) : nh_(nh) {
    cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    registerMotionModes();
  }

  void executeSelectedMode() {
    std::string motion_mode = nh_->declare_parameter("motion_mode", std::string("default"));
    RCLCPP_INFO(nh_->get_logger(), "Selected motion mode: %s", motion_mode.c_str());

    auto it = motion_modes_.find(motion_mode);
    if (it != motion_modes_.end()) {
      executeMotionSequence(it->second);
    } else {
      RCLCPP_ERROR(nh_->get_logger(), "Invalid motion mode: %s. Available modes:",
                   motion_mode.c_str());
      for (const auto &pair : motion_modes_) {
        RCLCPP_ERROR(nh_->get_logger(), "  - %s", pair.first.c_str());
      }
    }
  }

private:
  void registerMotionModes() {
    motion_modes_["mode1"] = {
        {0.5, 0.0, 0.0, 3.0},
        {0.0, 0.0, 0.0, 0.5},
        {0.5, 0.0, 0.5, 3.0},
        {0.0, 0.0, 0.0, 1.0},
    };

    motion_modes_["mode2"] = {
        {0.5, 0.0, 0.0, 2.0},   {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, M_PI / 2, 2.0},
        {0.0, 0.0, 0.0, 0.5}, {0.5, 0.0, 0.0, 2.0},   {0.0, 0.0, 0.0, 0.5},
        {0.0, 0.0, M_PI / 2, 2.0}, {0.0, 0.0, 0.0, 0.5}, {0.5, 0.0, 0.0, 2.0},
        {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, M_PI / 2, 2.0}, {0.0, 0.0, 0.0, 0.5},
        {0.5, 0.0, 0.0, 2.0},
    };

    motion_modes_["mode3"] = {
        {0.0, 0.0, 1.0, 2.0},   {0.0, 0.0, 0.0, 0.5}, {0.5, 0.0, 0.0, 2.0},
        {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, -1.0, 2.0}, {0.0, 0.0, 0.0, 0.5},
        {0.5, 0.0, 0.0, 2.0},
    };

    motion_modes_["mode4"] = {
        {0.5, 0.0, 0.0, 3.0},   {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, M_PI / 4, 4.0},
        {0.0, 0.0, 0.0, 0.5}, {0.5, 0.0, 0.0, 3.0},   {0.0, 0.0, 0.0, 0.5},
        {0.0, 0.5, 0.0, 2.0},   {0.0, 0.0, 0.0, 0.5}, {0.0, -0.5, 0.0, 2.0},
    };

    motion_modes_["default"] = {{0.0, 0.0, 0.0, 0.1}};
  }

  void executeMotionSequence(const std::vector<MotionStep> &sequence) {
    rclcpp::WallRate rate(10);

    for (const auto &step : sequence) {
      int cycles = static_cast<int>(step.duration * 10);

      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = step.linear_x * 1000;
      cmd_vel.linear.y = step.linear_y * 1000;
      cmd_vel.angular.z = step.angular_z * 1000;

      RCLCPP_INFO(nh_->get_logger(),
                  "Executing step: x=%.1f m/s, y=%.1f m/s, z=%.1f rad/s for %.1f seconds",
                  step.linear_x, step.linear_y, step.angular_z, step.duration);

      for (int i = 0; i < cycles && rclcpp::ok(); ++i) {
        cmd_vel_pub_->publish(cmd_vel);
        rate.sleep();
      }
    }

    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(nh_->get_logger(), "Motion sequence completed");
  }

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::map<std::string, std::vector<MotionStep>> motion_modes_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("control_motion", rclcpp::NodeOptions().use_global_arguments(true));
  MotionController controller(nh);
  controller.executeSelectedMode();
  rclcpp::shutdown();
  return 0;
}
