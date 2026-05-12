#ifndef GO2W_DRIVER__SPORT_CLIENT_HPP_
#define GO2W_DRIVER__SPORT_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

#include <nlohmann/json.hpp>

constexpr int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
constexpr int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
constexpr int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
constexpr int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
constexpr int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
constexpr int32_t ROBOT_SPORT_API_ID_MOVE = 1008;

class SportClient {
public:
  explicit SportClient(rclcpp::Node *node) : node_(node) {
    req_puber_ = node_->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);
  }

  void Damp(unitree_api::msg::Request &req) {
    ResetRequest(req, ROBOT_SPORT_API_ID_DAMP);
    req_puber_->publish(req);
  }

  void BalanceStand(unitree_api::msg::Request &req) {
    ResetRequest(req, ROBOT_SPORT_API_ID_BALANCESTAND);
    req_puber_->publish(req);
  }

  void StopMove(unitree_api::msg::Request &req) {
    ResetRequest(req, ROBOT_SPORT_API_ID_STOPMOVE);
    req_puber_->publish(req);
  }

  void StandUp(unitree_api::msg::Request &req) {
    ResetRequest(req, ROBOT_SPORT_API_ID_STANDUP);
    req_puber_->publish(req);
  }

  void StandDown(unitree_api::msg::Request &req) {
    ResetRequest(req, ROBOT_SPORT_API_ID_STANDDOWN);
    req_puber_->publish(req);
  }

  void RecoveryStand(unitree_api::msg::Request &req) {
    ResetRequest(req, ROBOT_SPORT_API_ID_RECOVERYSTAND);
    req_puber_->publish(req);
  }

  void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw) {
    ResetRequest(req, ROBOT_SPORT_API_ID_MOVE);
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;
    req.parameter = js.dump();
    req_puber_->publish(req);
  }

private:
  void ResetRequest(unitree_api::msg::Request &req, int32_t api_id) {
    req.header.identity.id = 0;
    req.header.identity.api_id = api_id;
    req.header.lease.id = 0;
    req.header.policy.priority = 0;
    req.header.policy.noreply = false;
    req.parameter.clear();
    req.binary.clear();
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
  rclcpp::Node *node_;
};

#endif // GO2W_DRIVER__SPORT_CLIENT_HPP_
