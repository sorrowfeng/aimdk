/**
 * @file hand_teleop_node.cpp
 * @brief VR 手柄 -> 雷赛灵巧手遥操节点
 *
 * 雷赛灵巧手定义 (HandType=3):
 *   0: 拇指旋转/侧摆
 *   1: 拇指弯曲
 *   2: 食指弯曲
 *   3: 中指弯曲
 *   4: 无名指弯曲
 *   5: 小指弯曲
 *
 * 映射方案:
 *   joint0 (拇指旋转) : axis_x   -> [-1.75, 1.75]
 *   joint1 (拇指弯曲) : axis_y   -> [0, 1.40]
 *   joint2 (食指弯曲) : index_trig -> [0, 1.40]
 *   joint3 (中指弯曲) : hand_trig  -> [0, 1.40]
 *   joint4 (无名指)   : hand_trig  -> [0, 1.40]
 *   joint5 (小指)     : hand_trig  -> [0, 1.40]
 */

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "aimdk_msgs/msg/hand_command.hpp"
#include "aimdk_msgs/msg/hand_command_array.hpp"
#include "aimdk_msgs/msg/hand_type.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "aimdk_msgs/msg/vr_controller_state.hpp"
#include "aimdk_msgs/msg/vr_data.hpp"

namespace hand_teleop {

inline double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

inline float ClampF(float v, float lo, float hi) {
  return std::max(lo, std::min(v, hi));
}

// 雷赛手 6 关节最大值（参考 rc_hand_sync.py）
constexpr float kMaxThumbRot = 1.75f;   // joint0
constexpr float kMaxThumbBend = 1.40f;  // joint1
constexpr float kMaxFingerBend = 1.40f; // joint2~5

struct HandMapping {
  std::vector<float> MapLeft(const aimdk_msgs::msg::VRControllerState& ctrl) {
    std::vector<float> pos(6, 0.0f);
    // 拇指旋转: axis_x [-1,1] -> [-1.75, 1.75]
    pos[0] = ClampF(ctrl.axis_x, -1.0f, 1.0f) * kMaxThumbRot;
    // 拇指弯曲: axis_y [-1,1] -> [0, 1.40] (向上推=弯曲)
    pos[1] = (ClampF(ctrl.axis_y, -1.0f, 1.0f) + 1.0f) * 0.5f * kMaxThumbBend;
    // 食指: index_trig
    pos[2] = ClampF(ctrl.index_trig, 0.0f, 1.0f) * kMaxFingerBend;
    // 中/无/小: hand_trig
    float hand = ClampF(ctrl.hand_trig, 0.0f, 1.0f) * kMaxFingerBend;
    pos[3] = hand;
    pos[4] = hand;
    pos[5] = hand;
    return pos;
  }

  std::vector<float> MapRight(const aimdk_msgs::msg::VRControllerState& ctrl) {
    // 左右手映射对称
    return MapLeft(ctrl);
  }

  // Demo 模式: 正弦波测试
  std::vector<float> MapDemo(double t, bool is_left) {
    std::vector<float> pos(6, 0.0f);
    float phase = is_left ? 0.0f : static_cast<float>(M_PI);
    pos[0] = std::sin(t + phase) * 0.5f * kMaxThumbRot;
    pos[1] = (std::sin(t * 0.7f + phase) + 1.0f) * 0.5f * kMaxThumbBend;
    pos[2] = (std::sin(t * 1.2f + phase) + 1.0f) * 0.5f * kMaxFingerBend;
    pos[3] = pos[2];
    pos[4] = pos[2];
    pos[5] = pos[2];
    return pos;
  }
};

class HandTeleopNode : public rclcpp::Node {
 public:
  HandTeleopNode() : Node("hand_teleop_node") {
    DeclareParameters();
    InitCommunication();
  }

 private:
  void DeclareParameters() {
    declare_parameter<std::string>("hand_command_topic",
                                   "/aima/hal/joint/hand/command");
    declare_parameter<std::string>("vr_data_topic", "/tmp/vr_data");
    declare_parameter<bool>("demo_mode", false);
    declare_parameter<double>("vr_timeout_sec", 0.5);
  }

  void InitCommunication() {
    std::string hand_topic = get_parameter("hand_command_topic").as_string();
    std::string vr_topic = get_parameter("vr_data_topic").as_string();
    demo_mode_ = get_parameter("demo_mode").as_bool();
    vr_timeout_sec_ = get_parameter("vr_timeout_sec").as_double();

    hand_pub_ = create_publisher<aimdk_msgs::msg::HandCommandArray>(
        hand_topic, rclcpp::SensorDataQoS());

    if (!demo_mode_) {
      vr_sub_ = create_subscription<aimdk_msgs::msg::VRData>(
          vr_topic, 10,
          std::bind(&HandTeleopNode::VRDataCallback, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "等待 VR 数据: %s", vr_topic.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Demo 模式: 自动正弦波测试灵巧手");
    }

    // 50Hz 控制定时器
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&HandTeleopNode::ControlLoop, this));
  }

  void VRDataCallback(const aimdk_msgs::msg::VRData::SharedPtr msg) {
    latest_vr_data_ = msg;
    last_vr_time_ = now();
  }

  void ControlLoop() {
    std::vector<float> left_pos(6, 0.0f);
    std::vector<float> right_pos(6, 0.0f);

    if (demo_mode_) {
      double t = now().seconds();
      left_pos = mapper_.MapDemo(t, true);
      right_pos = mapper_.MapDemo(t, false);
    } else if (latest_vr_data_) {
      double dt = (now() - last_vr_time_).seconds();
      if (dt > vr_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "VR 数据超时 %.2fs，手部保持自然张开", dt);
        // 保持全 0 (自然张开)
      } else {
        const auto& states = latest_vr_data_->vr_controller_states;
        if (states.size() > 0) {
          left_pos = mapper_.MapLeft(states[0]);
        }
        if (states.size() > 1) {
          right_pos = mapper_.MapRight(states[1]);
        }
      }
    } else {
      // 无数据时保持自然张开
    }

    PublishHandCommand(left_pos, right_pos);
  }

  void PublishHandCommand(const std::vector<float>& left_pos,
                          const std::vector<float>& right_pos) {
    auto msg = std::make_unique<aimdk_msgs::msg::HandCommandArray>();
    msg->header = aimdk_msgs::msg::MessageHeader();
    msg->header.stamp = now();

    // 雷赛手 HandType = 3
    msg->left_hand_type.value = 3;
    msg->right_hand_type.value = 3;

    msg->left_hands = BuildHandCommands(left_pos);
    msg->right_hands = BuildHandCommands(right_pos);

    hand_pub_->publish(std::move(msg));
  }

  std::vector<aimdk_msgs::msg::HandCommand> BuildHandCommands(
      const std::vector<float>& positions) {
    std::vector<aimdk_msgs::msg::HandCommand> cmds;
    cmds.reserve(positions.size());
    for (float pos : positions) {
      aimdk_msgs::msg::HandCommand cmd;
      cmd.name = "";  // rc_hand_sync.py 里 name 为空
      cmd.position = pos;
      cmd.velocity = 0.3f;
      cmd.acceleration = 0.0f;
      cmd.deceleration = 0.0f;
      cmd.effort = 0.0f;
      cmds.push_back(cmd);
    }
    return cmds;
  }

  rclcpp::Publisher<aimdk_msgs::msg::HandCommandArray>::SharedPtr hand_pub_;
  rclcpp::Subscription<aimdk_msgs::msg::VRData>::SharedPtr vr_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  aimdk_msgs::msg::VRData::SharedPtr latest_vr_data_;
  rclcpp::Time last_vr_time_{0, 0, RCL_ROS_TIME};

  HandMapping mapper_;
  bool demo_mode_{false};
  double vr_timeout_sec_{0.5};
};

}  // namespace hand_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hand_teleop::HandTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
