// Copyright 2026

// 节点入口（标准 ROS 2 模式）：
//   1) rclcpp::init        — 解析 ROS 2 参数、初始化中间件
//   2) make_shared<Node>   — 构造 InputBridgeNode，触发构造函数挂载订阅/发布/服务
//   3) spin                — 阻塞处理回调循环（直到收到 SIGINT）
//   4) shutdown            — 清理资源
//   5) return 0            — 正常退出
//
// 备注：rclcpp::shutdown() 收到 Ctrl+C 或外部 kill 时会令 spin() 返回。
//       节点的析构由 shared_ptr 引用计数自动触发。

#include <memory>

#include "nav2_input_bridge/input_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_input_bridge::InputBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
