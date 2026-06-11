// Copyright 2026, outdoor_nav maintainers.
#include "nav2_input_bridge/livox_bridge_converter.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace nav2_input_bridge {

// PCL point type with XYZ + intensity + ring + time (22 bytes/point).
// 放在 nav2_input_bridge 命名空间下，因为 pcl::PointCloud<T> 要求 T 是
// 可在编译期识别的具体类型；PCL 宏的 register 操作会在全局注册。
struct PointXYZIRT {
  PCL_ADD_POINT4D;     // x, y, z + 1 padding byte = 16 bytes
  float intensity;     // 4 bytes (offset 16)
  uint16_t ring;       // 2 bytes (offset 20)
  float time;          // 4 bytes (offset 22, total)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace nav2_input_bridge

// PCL point-cloud macro 必须在全局命名空间调用。
POINT_CLOUD_REGISTER_POINT_STRUCT(
  nav2_input_bridge::PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (float, time, time))

namespace nav2_input_bridge {

std::shared_ptr<sensor_msgs::msg::PointCloud2>
convertCustomMsgToPointCloud2(const livox_ros_driver2::msg::CustomMsg& msg) {
  if (msg.points.empty()) return nullptr;

  pcl::PointCloud<PointXYZIRT> cloud;
  cloud.header.frame_id = msg.header.frame_id;
  cloud.width = static_cast<uint32_t>(msg.point_num);
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.reserve(msg.point_num);
  for (const auto& p : msg.points) {
    PointXYZIRT pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    pt.intensity = static_cast<float>(p.line) +
                   static_cast<float>(p.reflectivity) / 100.0f;
    pt.ring = p.line;
    pt.time = static_cast<float>(p.offset_time) * 1e-9f;
    cloud.points.push_back(pt);
  }

  auto pc2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
  ::pcl::toROSMsg(cloud, *pc2);
  pc2->header.stamp = msg.header.stamp;
  pc2->header.frame_id = msg.header.frame_id;
  return pc2;
}

}  // namespace nav2_input_bridge
