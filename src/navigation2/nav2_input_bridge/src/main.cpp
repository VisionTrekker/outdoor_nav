// Copyright 2026

#include <memory>

#include "nav2_input_bridge/input_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_input_bridge::InputBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
