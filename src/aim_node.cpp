#include "rclcpp/rclcpp.hpp"
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/gimbal_state.hpp"
#include "rm_aim_sim_bridge/msg/telemetry.hpp"
#include "rm_aim_sim_bridge/msg/aim_command.hpp"

class AimNode : public rclcpp::Node {
 public:
  AimNode() : Node("aim_node") {
    rclcpp::QoS qos{1};
    qos.reliable().durability_volatile();

    armor_sub_ = this->create_subscription<rm_aim_sim_bridge::msg::ArmorCornersArray>(
        "/aim_sim/armors", qos,
        std::bind(&AimNode::OnArmors, this, std::placeholders::_1));

    gimbal_sub_ = this->create_subscription<rm_aim_sim_bridge::msg::GimbalState>(
        "/aim_sim/gimbal", qos,
        std::bind(&AimNode::OnGimbal, this, std::placeholders::_1));

    telemetry_sub_ = this->create_subscription<rm_aim_sim_bridge::msg::Telemetry>(
        "/aim_sim/telemetry", qos,
        std::bind(&AimNode::OnTelemetry, this, std::placeholders::_1));

    command_pub_ = this->create_publisher<rm_aim_sim_bridge::msg::AimCommand>(
        "/aim_sim/command", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&AimNode::OnTimer, this));
  }

 private:
  void OnArmors(const rm_aim_sim_bridge::msg::ArmorCornersArray::SharedPtr msg) {
    latest_armors_ = msg;
  }

  void OnGimbal(const rm_aim_sim_bridge::msg::GimbalState::SharedPtr msg) {
    latest_gimbal_ = msg;
  }

  void OnTelemetry(const rm_aim_sim_bridge::msg::Telemetry::SharedPtr msg) {
    latest_telemetry_ = msg;
  }

  void OnTimer() {
    if (!latest_armors_) {
      return;
    }

    rm_aim_sim_bridge::msg::AimCommand cmd;
    cmd.header = latest_armors_->header;
    cmd.session_id = latest_armors_->session_id;

    if (latest_armors_->armors.empty()) {
      cmd.yaw = 0.0;
      cmd.pitch = 0.0;
      cmd.enable = false;
      cmd.fire = false;
      command_pub_->publish(cmd);
      return;
    }

    const auto &corners = latest_armors_->armors[0].corners;

    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;

    for (const auto &p : corners) {
      cx += p.x;
      cy += p.y;
      cz += p.z;
    }

    cx /= 4.0;
    cy /= 4.0;
    cz /= 4.0;

    double dx = cx;
    double dy = cy;
    double dz = cz;

    double horizontal = std::sqrt(dx * dx + dy * dy);

    double target_yaw = std::atan2(dy, dx);

   

    double v = latest_telemetry_->speed;
    double g = latest_telemetry_->gravity;

    double v4 = v * v * v * v;
    double disc = v4 - g * (g * horizontal * horizontal + 2.0 * dz * v * v);

    if (disc < 0.0) {
    // 打不到，先不开火
    cmd.enable = false;
    cmd.fire = false;
    command_pub_->publish(cmd);
    return;
    }

    double sqrt_disc = std::sqrt(disc);
    double tan_theta = (v * v - sqrt_disc) / (g * horizontal);
    double target_pitch = std::atan(tan_theta);
    
    if (!std::isfinite(target_yaw) || !std::isfinite(target_pitch)) {
      cmd.yaw = 0.0;
      cmd.pitch = 0.0;
      cmd.enable = false;
      cmd.fire = false;
      command_pub_->publish(cmd);
      return;
    }

    
    cmd.yaw = target_yaw;
      cmd.pitch = target_pitch;
      cmd.enable = true;
      cmd.fire = true;
    command_pub_->publish(cmd);
  }

  rclcpp::Subscription<rm_aim_sim_bridge::msg::ArmorCornersArray>::SharedPtr armor_sub_;
  rclcpp::Subscription<rm_aim_sim_bridge::msg::GimbalState>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<rm_aim_sim_bridge::msg::Telemetry>::SharedPtr telemetry_sub_;
  rclcpp::Publisher<rm_aim_sim_bridge::msg::AimCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rm_aim_sim_bridge::msg::ArmorCornersArray::ConstSharedPtr latest_armors_;
  rm_aim_sim_bridge::msg::GimbalState::ConstSharedPtr latest_gimbal_;
  rm_aim_sim_bridge::msg::Telemetry::ConstSharedPtr latest_telemetry_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimNode>());
  rclcpp::shutdown();
  return 0;
}