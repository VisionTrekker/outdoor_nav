// Copyright 2026, outdoor_nav maintainers.
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "nav2_input_bridge/livox_bridge_converter.hpp"

namespace nav2_input_bridge {

class LivoxPointcloud2Bridge : public rclcpp::Node {
public:
  LivoxPointcloud2Bridge()
  : rclcpp::Node("livox_pointcloud2_bridge") {
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", rclcpp::QoS(10).best_effort(),
      std::bind(&LivoxPointcloud2Bridge::onLidar, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_pc2", 10);
  }

private:
  void onLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    auto pc2 = convertCustomMsgToPointCloud2(*msg);
    if (pc2) pub_->publish(*pc2);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace nav2_input_bridge

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_input_bridge::LivoxPointcloud2Bridge>());
  rclcpp::shutdown();
  return 0;
}
