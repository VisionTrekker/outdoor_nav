cmake_minimum_required(VERSION 3.8)
project(nav2_input_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(input_bridge_node
  src/main.cpp
  src/input_bridge_node.cpp
  src/align_state_machine.cpp
)

ament_target_dependencies(input_bridge_node
  PUBLIC
  geometry_msgs
  nav_msgs
  nav2_msgs
  mavros_msgs
  geographic_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(input_bridge_node PUBLIC Eigen3::Eigen)

target_include_directories(input_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS input_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_align_math
    test/test_align_math.cpp
    src/align_state_machine.cpp
  )
  target_include_directories(test_align_math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_align_math Eigen3::Eigen)
  ament_target_dependencies(test_align_math rclcpp)

  ament_add_gtest(test_align_state_machine
    test/test_align_state_machine.cpp
    src/align_state_machine.cpp
  )
  target_include_directories(test_align_state_machine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_align_state_machine Eigen3::Eigen)
  ament_target_dependencies(test_align_state_machine rclcpp)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  nav2_msgs
  mavros_msgs
  geographic_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
