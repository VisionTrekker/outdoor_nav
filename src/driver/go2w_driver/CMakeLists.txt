cmake_minimum_required(VERSION 3.8)
project(go2w_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(go2w_driver_node src/go2w_driver_node.cpp)
ament_target_dependencies(go2w_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  unitree_go
  unitree_api
)
target_link_libraries(go2w_driver_node nlohmann_json::nlohmann_json)
target_include_directories(go2w_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS go2w_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
