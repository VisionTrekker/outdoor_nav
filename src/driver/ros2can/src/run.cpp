// Subscribes to cmd_vel, publishes battery voltage; CAN I/O.
#include "canbus.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

int dev = 0;
int can_port0 = 0;
int can_port1 = 1;
Can_Msg txmsg;
Can_Msg rxmsg[100];

static std::atomic<bool> g_can_closed{false};

/** Reset + close both channels; small delay lets USB/libusb release the device (helps 2nd launch). */
static void hard_release_can() {
  (void)CAN_Reset(dev, can_port1);
  (void)CAN_Reset(dev, can_port0);
  (void)CAN_CloseDevice(dev, can_port1);
  (void)CAN_CloseDevice(dev, can_port0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

static void close_can_devices() {
  if (g_can_closed.exchange(true)) {
    return;
  }
  hard_release_can();
}

int ControlByCan(void) {
  int ret;
  memset(&txmsg, 0, sizeof(txmsg));
  txmsg.ID = 0x421;
  txmsg.Data[0] = 0x01;
  txmsg.Data[1] = 0x00;
  txmsg.Data[2] = 0x00;
  txmsg.Data[3] = 0x00;
  txmsg.Data[4] = 0x00;
  txmsg.Data[5] = 0x00;
  txmsg.Data[6] = 0x00;
  txmsg.Data[7] = 0x00;
  txmsg.DataLen = 8;
  ret = CAN_Transmit(dev, can_port1, &txmsg, 1, 100);
  printf("CAN_Transmit items: %d\r\n", ret);
  return ret;
}

bool init_can(rclcpp::Logger logger) {
  constexpr int kMaxAttempts = 10;
  Can_Config cancfg;
  cancfg.model = 0;
  cancfg.configs = 0;
  cancfg.baudrate = 500000;
  cancfg.configs |= 0x0001;
  cancfg.configs |= 0x0002;
  cancfg.configs |= 0x0004;

  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    if (attempt > 0) {
      RCLCPP_WARN(logger, "CAN init retry %d/%d (releasing stale handles)...", attempt + 1,
                  kMaxAttempts);
      hard_release_can();
    }

    CAN_CloseDevice(dev, can_port0);
    CAN_CloseDevice(dev, can_port1);

    int devs = CAN_ScanDevice();
    if (devs <= 0) {
      RCLCPP_ERROR(logger, "No CAN device found");
      continue;
    }

    int ret = CAN_OpenDevice(dev, can_port0);
    if (ret != 0) {
      RCLCPP_ERROR(logger, "Failed to open CAN 1 (channel 0).");
      continue;
    }
    RCLCPP_INFO(logger, "CAN 1 open succeed!!");

    ret = CAN_OpenDevice(dev, can_port1);
    if (ret != 0) {
      RCLCPP_ERROR(logger, "Failed to open CAN 2 (channel 1).");
      (void)CAN_CloseDevice(dev, can_port0);
      hard_release_can();
      continue;
    }
    RCLCPP_INFO(logger, "CAN 2 open succeed!!");

    ret = CAN_Init(dev, can_port0, &cancfg);
    if (ret != 0) {
      RCLCPP_ERROR(logger, "Failed to init CAN 1.");
      hard_release_can();
      continue;
    }
    RCLCPP_INFO(logger, "CAN 1 init succeed!");
    CAN_SetFilter(dev, can_port0, 0, 0, 0, 0, 1);

    ret = CAN_Init(dev, can_port1, &cancfg);
    if (ret != 0) {
      RCLCPP_ERROR(logger, "Failed to init CAN 2.");
      hard_release_can();
      continue;
    }
    RCLCPP_INFO(logger, "CAN 2 init succeed!");
    CAN_SetFilter(dev, can_port1, 0, 0, 0, 0, 1);
    RCLCPP_INFO(logger, "CAN initialized successfully.");

    ControlByCan();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(logger, "Control by CAN.");
    return true;
  }

  RCLCPP_ERROR(logger, "CAN init failed after %d attempts.", kMaxAttempts);
  return false;
}

void floatToBytes(float value, char *data, int index) {
  int16_t int_val = static_cast<int16_t>(value);
  data[index] = (int_val >> 8) & 0xFF;
  data[index + 1] = int_val & 0xFF;
}

int16_t bytesToInt16(const char *data) {
  return (static_cast<int16_t>(static_cast<uint8_t>(data[0]) << 8)) |
         static_cast<int16_t>(static_cast<uint8_t>(data[1]));
}

class CarDriverNode : public rclcpp::Node {
public:
  CarDriverNode() : Node("car_driver") {
    if (!init_can(get_logger())) {
      throw std::runtime_error("Failed to init CAN");
    }

    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("/battery_voltage", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        std::bind(&CarDriverNode::controlCmdCallback, this, std::placeholders::_1));
  }

  ~CarDriverNode() override { close_can_devices(); }

  void poll_can() { receive_can_msgs(); }

private:
  void controlCmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd) {
    RCLCPP_INFO(get_logger(), "Control command: linear x:%f, linear y:%f, linear z:%f",
                cmd->linear.x, cmd->linear.y, cmd->linear.z);
    RCLCPP_INFO(get_logger(), "Control command: angular x:%f, angular y:%f, angular z:%f",
                cmd->angular.x, cmd->angular.y, cmd->angular.z);

    memset(&txmsg, 0, sizeof(txmsg));
    txmsg.ID = 0x111;
    txmsg.DataLen = 8;

    floatToBytes((cmd->linear.x * 1000), txmsg.Data, 0);
    floatToBytes((cmd->angular.z * 1000), txmsg.Data, 2);
    floatToBytes((cmd->linear.y * 1000), txmsg.Data, 4);
    txmsg.Data[6] = 0x00;
    txmsg.Data[7] = 0x00;

    int ret = CAN_Transmit(dev, can_port1, &txmsg, 1, 100);
    if (ret <= 0) {
      RCLCPP_WARN(get_logger(), "CAN_Transmit failed");
      close_can_devices();
    }
  }

  int receive_can_msgs() {
    int ret;
    memset(&rxmsg[0], 0, sizeof(rxmsg[0]));
    ret = CAN_Receive(dev, can_port1, rxmsg, 100, 100);
    if (ret <= 0) {
      return ret;
    }

    for (int i = 0; i < ret; i++) {
      Can_Msg *msg = &rxmsg[i];

      if (msg->ID == 0x211) {
        if (msg->DataLen < 4) {
          RCLCPP_WARN(get_logger(), "Invalid data length for ID 0x211: expected >=4, got %d",
                      msg->DataLen);
          continue;
        }

        uint8_t high_byte = static_cast<uint8_t>(msg->Data[2]);
        uint8_t low_byte = static_cast<uint8_t>(msg->Data[3]);
        uint16_t raw_voltage = (high_byte << 8) | low_byte;
        float actual_voltage = raw_voltage / 10.0f;

        std_msgs::msg::Float32 voltage_msg;
        voltage_msg.data = actual_voltage;
        voltage_pub_->publish(voltage_msg);

        RCLCPP_DEBUG(get_logger(), "[Battery Voltage] ID:0x211 Raw:0x%04X (%u) --> %.1f V",
                     raw_voltage, raw_voltage, actual_voltage);
      } else if (msg->ID == 0x221) {
        if (msg->DataLen < 6) {
          RCLCPP_WARN(get_logger(), "Invalid data length for ID 0x221: expected >=6, got %d",
                      msg->DataLen);
          continue;
        }

        int16_t vx_raw = bytesToInt16(&msg->Data[0]);
        int16_t vth_raw = bytesToInt16(&msg->Data[2]);
        int16_t vy_raw = bytesToInt16(&msg->Data[4]);

        RCLCPP_DEBUG(get_logger(),
                     "[Speed Feedback] ID:0x221 vx:%d mm/s (%.3f m/s), vy:%d mm/s (%.3f m/s), "
                     "vth:%d (0.001 rad/s) (%.3f rad/s)",
                     vx_raw, (vx_raw / 1000.0), vy_raw, (vy_raw / 1000.0), vth_raw,
                     (vth_raw * 0.001));
      }
    }
    return ret;
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);


  try {
    auto node = std::make_shared<CarDriverNode>();
    rclcpp::WallRate loop_rate(50);
    while (rclcpp::ok()) {
      node->poll_can();
      loop_rate.sleep();
      rclcpp::spin_some(node);
    }
    node.reset();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("car_driver"), "%s", e.what());
    close_can_devices();
    rclcpp::shutdown();
    return -1;
  }
  rclcpp::shutdown();
  return 0;
}
