#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control_publisher");

  node->declare_parameter("linear_x", 500.0);
  node->declare_parameter("linear_y", 0.0);
  node->declare_parameter("angular_z", 0.0);

  double linear_x = node->get_parameter("linear_x").as_double();
  double linear_y = node->get_parameter("linear_y").as_double();
  double angular_z = node->get_parameter("angular_z").as_double();

  auto control_pub =
      node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  rclcpp::WallRate loop_rate(10);

  int count = 0;
  while (rclcpp::ok() && count <= 100) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = angular_z;

    control_pub->publish(cmd_vel);
    RCLCPP_INFO(node->get_logger(), "Publish Control command: x:%f  y:%f  z:%f",
                cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    count++;
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
