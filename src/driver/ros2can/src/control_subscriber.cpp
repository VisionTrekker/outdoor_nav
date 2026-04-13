#include "canbus.h"
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

int dev = 0;
int can_port0 = 0;
int can_port1 = 1;
Can_Msg txmsg;

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

void floatToBytes(float value, char *data, int index) {
  int16_t int_val = static_cast<int16_t>(value);
  data[index] = (int_val >> 8) & 0xFF;
  data[index + 1] = int_val & 0xFF;
}

class ControlSubscriberNode : public rclcpp::Node {
public:
  ControlSubscriberNode() : Node("control_subscriber") {
    if (!init_can(this->get_logger())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init system.");
      throw std::runtime_error("CAN init failed");
    }
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        std::bind(&ControlSubscriberNode::controlCmdCallback, this,
                  std::placeholders::_1));
  }

  ~ControlSubscriberNode() override {
    CAN_CloseDevice(dev, can_port0);
    CAN_CloseDevice(dev, can_port1);
  }

private:
  void controlCmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd) {
    RCLCPP_INFO(this->get_logger(), "Control command: linear x:%f, linear y:%f, linear z:%f",
                cmd->linear.x, cmd->linear.y, cmd->linear.z);
    RCLCPP_INFO(this->get_logger(), "Control command: angular x:%f, angular y:%f, angular z:%f",
                cmd->angular.x, cmd->angular.y, cmd->angular.z);
    memset(&txmsg, 0, sizeof(txmsg));
    txmsg.ID = 0x111;
    txmsg.DataLen = 8;

    floatToBytes(cmd->linear.x, txmsg.Data, 0);
    floatToBytes(cmd->angular.z, txmsg.Data, 2);
    floatToBytes(cmd->linear.y, txmsg.Data, 4);
    txmsg.Data[6] = 0x00;
    txmsg.Data[7] = 0x00;

    int ret = CAN_Transmit(dev, can_port1, &txmsg, 1, 100);
    if (ret <= 0) {
      RCLCPP_WARN(this->get_logger(), "CAN_Transmit failed");
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Sent CAN frame: [%02x %02x %02x %02x %02x %02x %02x %02x]",
          static_cast<unsigned char>(txmsg.Data[0]),
          static_cast<unsigned char>(txmsg.Data[1]),
          static_cast<unsigned char>(txmsg.Data[2]),
          static_cast<unsigned char>(txmsg.Data[3]),
          static_cast<unsigned char>(txmsg.Data[4]),
          static_cast<unsigned char>(txmsg.Data[5]),
          static_cast<unsigned char>(txmsg.Data[6]),
          static_cast<unsigned char>(txmsg.Data[7]));
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ControlSubscriberNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("control_subscriber"), "%s", e.what());
    rclcpp::shutdown();
    return -1;
  }
  rclcpp::shutdown();
  return 0;
}
