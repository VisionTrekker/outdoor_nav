#include "canbus.h"
#include <cstring>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

int dev = 0;
int can_port0 = 0;
int can_port1 = 1;
Can_Msg rxmsg[100];

bool init_can(rclcpp::Logger logger) {
  int ret;
  Can_Config cancfg;
  int devs = CAN_ScanDevice();
  if (devs <= 0) {
    RCLCPP_ERROR(logger, "No CAN device found");
    return false;
  }

  ret = CAN_OpenDevice(dev, can_port0);
  if (ret != 0) {
    RCLCPP_ERROR(logger, "Failed to open CAN 1.");
    return false;
  }
  RCLCPP_INFO(logger, "CAN 1 open succeed!!");
  ret = CAN_OpenDevice(dev, can_port1);
  if (ret != 0) {
    RCLCPP_ERROR(logger, "Failed to open CAN 2.");
    return false;
  }
  RCLCPP_INFO(logger, "CAN 2 open succeed!!");
  cancfg.model = 0;
  cancfg.configs = 0;
  cancfg.baudrate = 500000;
  cancfg.configs |= 0x0001;
  cancfg.configs |= 0x0002;
  cancfg.configs |= 0x0004;

  ret = CAN_Init(dev, can_port0, &cancfg);
  if (ret != 0) {
    RCLCPP_ERROR(logger, "Failed to init CAN 1.");
    return false;
  }
  RCLCPP_INFO(logger, "CAN 1 init succeed!");
  CAN_SetFilter(dev, can_port0, 0, 0, 0, 0, 1);
  ret = CAN_Init(dev, can_port1, &cancfg);
  if (ret != 0) {
    RCLCPP_ERROR(logger, "Failed to init CAN 2.");
    return false;
  }
  RCLCPP_INFO(logger, "CAN 2 init succeed!");
  CAN_SetFilter(dev, can_port1, 0, 0, 0, 0, 1);
  RCLCPP_INFO(logger, "CAN initialized successfully.");
  return true;
}

int32_t bytesToInt32(const char *data) {
  return (static_cast<int32_t>(static_cast<uint8_t>(data[0])) << 24) |
         (static_cast<int32_t>(static_cast<uint8_t>(data[1])) << 16) |
         (static_cast<int32_t>(static_cast<uint8_t>(data[2])) << 8) |
         static_cast<int32_t>(static_cast<uint8_t>(data[3]));
}

int receive_can_msgs(rclcpp::Logger logger,
                     const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &voltage_pub,
                     const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr &left_odom_pub,
                     const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr &right_odom_pub) {
  if (!init_can(logger)) {
    return -1;
  }
  int ret;
  memset(&rxmsg[0], 0, sizeof(rxmsg[0]));
  ret = CAN_Receive(dev, can_port1, rxmsg, 100, 100);
  if (ret <= 0) {
    CAN_CloseDevice(dev, can_port0);
    CAN_CloseDevice(dev, can_port1);
    return ret;
  }

  for (int i = 0; i < ret; i++) {
    Can_Msg *msg = &rxmsg[i];

    if (msg->ID == 0x211) {
      if (msg->DataLen < 4) {
        RCLCPP_WARN(logger, "Invalid data length for ID 0x211: expected >=4, got %d", msg->DataLen);
        continue;
      }

      uint8_t high_byte = static_cast<uint8_t>(msg->Data[2]);
      uint8_t low_byte = static_cast<uint8_t>(msg->Data[3]);
      uint16_t raw_voltage = (high_byte << 8) | low_byte;
      float actual_voltage = raw_voltage / 10.0f;

      std_msgs::msg::Float32 voltage_msg;
      voltage_msg.data = actual_voltage;
      voltage_pub->publish(voltage_msg);

      RCLCPP_INFO(logger, "[Battery Voltage] ID:0x211 Raw:0x%04X (%u) --> %.1f V", raw_voltage,
                  raw_voltage, actual_voltage);
    } else if (msg->ID == 0x311) {
      if (msg->DataLen < 8) {
        RCLCPP_WARN(logger, "Invalid data length for ID 0x311: expected 8, got %d", msg->DataLen);
        continue;
      }

      int32_t left_odom = bytesToInt32(&msg->Data[0]);
      int32_t right_odom = bytesToInt32(&msg->Data[4]);

      std_msgs::msg::Int32 left_msg;
      std_msgs::msg::Int32 right_msg;
      left_msg.data = left_odom;
      right_msg.data = right_odom;
      left_odom_pub->publish(left_msg);
      right_odom_pub->publish(right_msg);

      RCLCPP_INFO(logger, "[Odom] ID:0x311 Left:%d mm Right:%d mm", left_odom, right_odom);
    }
  }

  CAN_CloseDevice(dev, can_port0);
  CAN_CloseDevice(dev, can_port1);
  return ret;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("odm_battery_publisher");

  if (!init_can(nh->get_logger())) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to init system.");
    rclcpp::shutdown();
    return -1;
  }
  CAN_CloseDevice(dev, can_port0);
  CAN_CloseDevice(dev, can_port1);

  auto voltage_pub = nh->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
  auto left_odom_pub = nh->create_publisher<std_msgs::msg::Int32>("left_wheel_odom", 10);
  auto right_odom_pub = nh->create_publisher<std_msgs::msg::Int32>("right_wheel_odom", 10);

  rclcpp::WallRate loop_rate(100);

  while (rclcpp::ok()) {
    receive_can_msgs(nh->get_logger(), voltage_pub, left_odom_pub, right_odom_pub);
    loop_rate.sleep();
    rclcpp::spin_some(nh);
  }

  rclcpp::shutdown();
  return 0;
}
