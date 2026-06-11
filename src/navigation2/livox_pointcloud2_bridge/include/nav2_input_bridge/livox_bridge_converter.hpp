// Copyright 2026, outdoor_nav maintainers.
#ifndef NAV2_INPUT_BRIDGE__LIVOX_BRIDGE_CONVERTER_HPP_
#define NAV2_INPUT_BRIDGE__LIVOX_BRIDGE_CONVERTER_HPP_

#include <memory>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace nav2_input_bridge {

// 把 Livox CustomMsg 转为 sensor_msgs/PointCloud2 (6 fields, point_step=22)。
//
// 字段映射 (对应 spec §4.2):
//   x, y, z:    float32, 直传
//   intensity:  float32 = line + reflectivity / 10000.0
//                          (line 整数部分 + reflectivity 小数部分)
//   ring:       uint16  = line 直传
//   time:       float32 = offset_time * 1e-9 (单位 s, 范围 0..0.1)
//
// 不映射: point.tag (PointCloud2 标准无此字段),
//         msg.timebase (顶层, 用 ROS header.stamp 代替)。
//
// 返回: PointCloud2 智能指针；输入 points 为空时返回 nullptr（不发布）。
std::shared_ptr<sensor_msgs::msg::PointCloud2>
convertCustomMsgToPointCloud2(const livox_ros_driver2::msg::CustomMsg& msg);

}  // namespace nav2_input_bridge

#endif  // NAV2_INPUT_BRIDGE__LIVOX_BRIDGE_CONVERTER_HPP_
