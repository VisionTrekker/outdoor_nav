cmake_minimum_required(VERSION 3.8)
project(ros2can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig QUIET)
set(ROS2CAN_CAN_AVAILABLE FALSE)
if(PkgConfig_FOUND)
  pkg_check_modules(UDEV QUIET libudev)
endif()
find_library(CANBUS_LIB canbus HINTS /usr/local/lib /usr/lib ${CMAKE_INSTALL_PREFIX}/lib)
find_library(USB_LIB usb-1.0 HINTS /usr/local/lib /usr/lib)

if(UDEV_FOUND AND CANBUS_LIB AND USB_LIB)
  set(ROS2CAN_CAN_AVAILABLE TRUE)
  message(STATUS "ros2can: CAN targets enabled (libcanbus + usb + udev)")
  set(CAN_LINK_LIBS ${UDEV_LIBRARIES} ${CANBUS_LIB} ${USB_LIB})
else()
  message(
    WARNING
      "ros2can: libcanbus and/or libusb / libudev not found — skipping car_driver, "
      "control_subscriber, odm_battery_publisher. Install vendor CAN libs to /usr/local/lib.")
endif()

add_executable(control_publisher src/control_publisher.cpp)
ament_target_dependencies(control_publisher rclcpp geometry_msgs)
target_include_directories(control_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(control_motion src/control_motion.cpp)
ament_target_dependencies(control_motion rclcpp geometry_msgs)
target_include_directories(control_motion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(INSTALL_TARGETS control_publisher control_motion)

if(ROS2CAN_CAN_AVAILABLE)
  add_executable(control_subscriber src/control_subscriber.cpp)
  ament_target_dependencies(control_subscriber rclcpp geometry_msgs)
  target_include_directories(control_subscriber PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(control_subscriber ${CAN_LINK_LIBS})

  add_executable(car_driver src/run.cpp)
  ament_target_dependencies(car_driver rclcpp geometry_msgs std_msgs)
  target_include_directories(car_driver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(car_driver ${CAN_LINK_LIBS})

  add_executable(odm_battery_publisher src/odm_battery_publisher.cpp)
  ament_target_dependencies(odm_battery_publisher rclcpp std_msgs)
  target_include_directories(odm_battery_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(odm_battery_publisher ${CAN_LINK_LIBS})

  list(
    APPEND
    INSTALL_TARGETS
    control_subscriber
    car_driver
    odm_battery_publisher
  )
endif()

install(TARGETS ${INSTALL_TARGETS} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)

ament_package()
