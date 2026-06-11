cmake_minimum_required(VERSION 3.16)
project(livox_pointcloud2_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# --- library: 纯转换函数（便于 gtest 直接调用）---
add_library(livox_bridge_converter
  src/livox_bridge_converter.cpp
)
target_include_directories(livox_bridge_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(livox_bridge_converter
  PUBLIC livox_ros_driver2 pcl_conversions sensor_msgs
)
target_link_libraries(livox_bridge_converter PUBLIC ${PCL_LIBRARIES})

# --- executable: ROS 2 节点---
add_executable(livox_pointcloud2_bridge_node
  src/livox_pointcloud2_bridge_node.cpp
)
target_link_libraries(livox_pointcloud2_bridge_node
  PRIVATE livox_bridge_converter)
ament_target_dependencies(livox_pointcloud2_bridge_node
  PUBLIC
  rclcpp
  livox_ros_driver2
  sensor_msgs)

install(TARGETS livox_pointcloud2_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS livox_bridge_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# --- tests---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_msg_conversion
    test/test_msg_conversion.cpp
    src/livox_bridge_converter.cpp
  )
  target_include_directories(test_msg_conversion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_msg_conversion
    livox_ros_driver2
    pcl_conversions
    sensor_msgs)
  target_link_libraries(test_msg_conversion ${PCL_LIBRARIES})
endif()

ament_package()
