/**
 * @file vr_teleop_node.cpp
 * @brief X2 VR 遥操骨架节点 — 基于 x2_ultra.urdf 的手臂 IK
 *
 * 启动流程：
 *   1. 收到第一次手臂状态后，自动平滑运动到 Ready Pose（双手抬起、肘关节平直）
 *   2. 保持在 Ready Pose 等待 VR 数据
 *   3. VR 数据为**相对空间位置**（偏移量），直接叠加到 Ready Pose 的腕部位置上进行 IK
 *
 * 重要发现：X2 Ultra 的 URDF 显示手臂为 7-DOF：
 *   shoulder_pitch / shoulder_roll / shoulder_yaw / elbow /
 *   wrist_yaw / wrist_pitch / wrist_roll
 *
 * 必须与 hand_teleop_node 并行运行，且机器人须处于 Develop_MC 模式。
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <mutex>

#include "aimdk_msgs/msg/joint_command_array.hpp"
#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "aimdk_msgs/msg/vr_controller_state.hpp"
#include "aimdk_msgs/msg/vr_data.hpp"

namespace vr_teleop {

inline double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

// ─────────────────────────────────────────────────────────
// 7-DOF 手臂关节定义（按 x2_ultra.urdf 更新）
// ─────────────────────────────────────────────────────────
constexpr int kArmDofs = 14;
struct JointInfo {
  std::string name;
  double lower_limit;
  double upper_limit;
  double kp;
  double kd;
};

const std::vector<JointInfo> kArmJoints = {
    // 左臂
    {"left_shoulder_pitch_joint", -3.08, 2.04, 20.0, 2.0},
    {"left_shoulder_roll_joint", -0.061, 2.993, 20.0, 2.0},
    {"left_shoulder_yaw_joint", -2.556, 2.556, 20.0, 2.0},
    {"left_elbow_joint", -2.3556, 0.0, 20.0, 2.0},
    {"left_wrist_yaw_joint", -2.556, 2.556, 20.0, 2.0},
    {"left_wrist_pitch_joint", -0.558, 0.558, 20.0, 2.0},
    {"left_wrist_roll_joint", -1.571, 0.724, 20.0, 2.0},
    // 右臂
    {"right_shoulder_pitch_joint", -3.08, 2.04, 20.0, 2.0},
    {"right_shoulder_roll_joint", -2.993, 0.061, 20.0, 2.0},
    {"right_shoulder_yaw_joint", -2.556, 2.556, 20.0, 2.0},
    {"right_elbow_joint", -2.3556, 0.0, 20.0, 2.0},
    {"right_wrist_yaw_joint", -2.556, 2.556, 20.0, 2.0},
    {"right_wrist_pitch_joint", -0.558, 0.558, 20.0, 2.0},
    {"right_wrist_roll_joint", -0.724, 1.571, 20.0, 2.0},
};

// ─────────────────────────────────────────────────────────
// 轻量级 7-DOF 数值 IK（基于 x2_ultra.urdf 几何）
// ─────────────────────────────────────────────────────────
class SimpleArmIK {
 public:
  double shoulder_roll_offset_y = 0.0495;
  double shoulder_yaw_offset_x = 0.001224;
  double shoulder_yaw_offset_z = -0.121195;
  double elbow_offset_x = 0.0140;
  double elbow_offset_y = 0.0002;
  double elbow_offset_z = -0.08295;
  double wrist_yaw_offset_x = -0.01328;
  double wrist_yaw_offset_y = -0.000098;
  double wrist_yaw_offset_z = -0.12058;
  double wrist_pitch_offset_x = 0.00049;
  double wrist_pitch_offset_z = -0.0820;

  double damping = 0.05;
  int max_iterations = 30;
  double convergence_tol = 1e-4;

  Eigen::Vector3d ForwardPosition(const Eigen::VectorXd& q) const {
    double sp = q(0), sr = q(1), sy = q(2), ep = q(3);

    Eigen::Matrix3d R_sp =
        Eigen::AngleAxisd(sp, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d R_sr =
        Eigen::AngleAxisd(sr, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d R_sy =
        Eigen::AngleAxisd(sy, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d shoulder_yaw_pos =
        R_sp * (Eigen::Vector3d(0.0, shoulder_roll_offset_y, 0.0) +
                R_sr * Eigen::Vector3d(shoulder_yaw_offset_x, 0.0,
                                       shoulder_yaw_offset_z));

    Eigen::Matrix3d R_shoulder = R_sp * R_sr * R_sy;
    Eigen::Vector3d elbow_pos =
        shoulder_yaw_pos +
        R_shoulder *
            Eigen::Vector3d(elbow_offset_x, elbow_offset_y, elbow_offset_z);

    Eigen::Matrix3d R_elbow =
        R_shoulder *
        Eigen::AngleAxisd(ep, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Vector3d wrist_yaw_pos =
        elbow_pos +
        R_elbow *
            Eigen::Vector3d(wrist_yaw_offset_x, wrist_yaw_offset_y,
                           wrist_yaw_offset_z);
    Eigen::Vector3d wrist_pos =
        wrist_yaw_pos +
        R_elbow *
            Eigen::Vector3d(wrist_pitch_offset_x, 0.0, wrist_pitch_offset_z);

    return wrist_pos;
  }

  Eigen::MatrixXd ComputePositionJacobian(const Eigen::VectorXd& q) const {
    constexpr double dq = 1e-6;
    Eigen::MatrixXd J(3, 4);
    Eigen::Vector3d p0 = ForwardPosition(q);
    for (int i = 0; i < 4; ++i) {
      Eigen::VectorXd q_pert = q;
      q_pert(i) += dq;
      J.col(i) = (ForwardPosition(q_pert) - p0) / dq;
    }
    return J;
  }

  Eigen::VectorXd SolvePosition(const Eigen::Vector3d& target_pos,
                                const Eigen::VectorXd& q_init) const {
    Eigen::VectorXd q = q_init;
    for (int iter = 0; iter < max_iterations; ++iter) {
      Eigen::Vector3d err = target_pos - ForwardPosition(q);
      if (err.norm() < convergence_tol) break;

      Eigen::MatrixXd J = ComputePositionJacobian(q);
      Eigen::Matrix3d JJt = J * J.transpose();
      JJt.diagonal().array() += damping * damping;
      Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);

      for (int i = 0; i < 4; ++i) {
        q(i) += Clamp(dq(i), -0.05, 0.05);
        q(i) = Clamp(q(i), kArmJoints[i].lower_limit,
                     kArmJoints[i].upper_limit);
      }
    }
    return q;
  }
};

inline void QuaternionToZYX(const geometry_msgs::msg::Quaternion& q,
                            double& roll, double& pitch, double& yaw) {
  Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
  Eigen::Vector3d rpy = eq.toRotationMatrix().eulerAngles(2, 1, 0);
  yaw = rpy(0);
  pitch = rpy(1);
  roll = rpy(2);
}

// ─────────────────────────────────────────────────────────
// 主节点
// ─────────────────────────────────────────────────────────
class VRTeleopNode : public rclcpp::Node {
 public:
  VRTeleopNode() : Node("vr_teleop_node"), ruckig_(0.01) {
    DeclareParameters();
    InitReadyPose();
    InitCommunication();
    InitRuckig();
    if (demo_mode_) {
      InitKeyboard();
      RCLCPP_INFO(get_logger(),
                  "键盘 Demo 控制: W/S=前后(X)  A/D=左右(Y)  R/F=上下(Z)  Space=重置  T=切换左右手  H=回到初始位置 | 步进 5mm");
    }
  }

  ~VRTeleopNode() {
    if (demo_mode_) {
      stop_keyboard_ = true;
      if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
      }
      RestoreKeyboard();
    }
  }

  void ReturnToInitialPose() {
    if (!has_initial_q_ || !arm_cmd_pub_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "节点退出，开始平滑回到初始位置...");

    // 以当前 ruckig 状态为起点
    for (int i = 0; i < kArmDofs; ++i) {
      ruckig_input_.target_position[i] = initial_q_(i);
      ruckig_input_.target_velocity[i] = 0.0;
      ruckig_input_.target_acceleration[i] = 0.0;
    }

    int max_steps = 500;  // 5s timeout
    int steps = 0;
    while (steps++ < max_steps) {
      auto result = ruckig_.update(ruckig_input_, ruckig_output_);
      if (result == ruckig::Result::Error) {
        RCLCPP_ERROR(get_logger(), "回原点 Ruckig 错误");
        break;
      }

      aimdk_msgs::msg::JointCommandArray arm_cmd;
      arm_cmd.header.stamp = now();
      for (int i = 0; i < kArmDofs; ++i) {
        aimdk_msgs::msg::JointCommand joint;
        joint.name = kArmJoints[i].name;
        joint.position = ruckig_output_.new_position[i];
        joint.velocity = ruckig_output_.new_velocity[i];
        joint.stiffness = kArmJoints[i].kp;
        joint.damping = kArmJoints[i].kd;
        arm_cmd.joints.push_back(joint);
      }
      arm_cmd_pub_->publish(arm_cmd);

      for (int i = 0; i < kArmDofs; ++i) {
        ruckig_input_.current_position[i] = ruckig_output_.new_position[i];
        ruckig_input_.current_velocity[i] = ruckig_output_.new_velocity[i];
        ruckig_input_.current_acceleration[i] =
            ruckig_output_.new_acceleration[i];
      }

      if (result == ruckig::Result::Finished) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(get_logger(), "已回到初始位置。");
  }

 private:
  void DeclareParameters() {
    declare_parameter<std::string>("arm_state_topic",
                                   "/aima/hal/joint/arm/state");
    declare_parameter<std::string>("arm_command_topic",
                                   "/aima/hal/joint/arm/command");
    declare_parameter<std::string>("vr_data_topic", "/udp_vr_bridge/arm_target");
    declare_parameter<double>("control_frequency", 100.0);
    declare_parameter<bool>("demo_mode", false);
    declare_parameter<double>("vr_timeout_sec", 0.5);
    declare_parameter<double>("position_scale", 1.0);  // VR数据已是相对量，scale默认1.0

    // Ready Pose 参数：上臂自然下垂，肘关节弯曲约 90°，使前臂保持水平平直
    // [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll]
    declare_parameter<std::vector<double>>(
        "ready_pose_left",
        std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
        "ready_pose_right",
        std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0});
  }

  void InitReadyPose() {
    auto left_vec = get_parameter("ready_pose_left").as_double_array();
    auto right_vec = get_parameter("ready_pose_right").as_double_array();

    if (left_vec.size() != 7 || right_vec.size() != 7) {
      RCLCPP_FATAL(get_logger(),
                   "ready_pose_left/right 必须是长度为 7 的数组！");
      rclcpp::shutdown();
      return;
    }

    ready_pose_left_ = Eigen::Map<Eigen::VectorXd>(left_vec.data(), 7);
    ready_pose_right_ = Eigen::Map<Eigen::VectorXd>(right_vec.data(), 7);

    // 预计算 ready pose 下的腕部位置（肩膀局部坐标）
    left_wrist_ready_pos_ = left_ik_.ForwardPosition(ready_pose_left_);
    right_wrist_ready_pos_ = right_ik_.ForwardPosition(ready_pose_right_);

    RCLCPP_INFO(get_logger(),
                "Ready Pose 已设置 | 左腕位置: (%.3f, %.3f, %.3f) | 右腕位置: (%.3f, %.3f, %.3f)",
                left_wrist_ready_pos_.x(), left_wrist_ready_pos_.y(),
                left_wrist_ready_pos_.z(), right_wrist_ready_pos_.x(),
                right_wrist_ready_pos_.y(), right_wrist_ready_pos_.z());
  }

  void InitCommunication() {
    std::string arm_state_topic = get_parameter("arm_state_topic").as_string();
    std::string arm_cmd_topic = get_parameter("arm_command_topic").as_string();
    std::string vr_topic = get_parameter("vr_data_topic").as_string();
    double control_freq = get_parameter("control_frequency").as_double();
    demo_mode_ = get_parameter("demo_mode").as_bool();
    vr_timeout_sec_ = get_parameter("vr_timeout_sec").as_double();
    position_scale_ = get_parameter("position_scale").as_double();

    arm_state_sub_ = create_subscription<aimdk_msgs::msg::JointStateArray>(
        arm_state_topic, rclcpp::SensorDataQoS(),
        std::bind(&VRTeleopNode::ArmStateCallback, this,
                  std::placeholders::_1));

    arm_cmd_pub_ = create_publisher<aimdk_msgs::msg::JointCommandArray>(
        arm_cmd_topic, rclcpp::SensorDataQoS());

    if (!demo_mode_) {
      vr_sub_ = create_subscription<aimdk_msgs::msg::VRData>(
          vr_topic, 10,
          std::bind(&VRTeleopNode::VRDataCallback, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "等待 VR 数据 topic: %s", vr_topic.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  "Demo 模式已启用，将使用键盘 WASD 模拟相对位置遥操");
    }

    auto period_ms =
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq));
    control_timer_ = create_wall_timer(
        period_ms, std::bind(&VRTeleopNode::ControlLoop, this));

    current_q_.setZero(kArmDofs);
  }

  void InitRuckig() {
    for (int i = 0; i < kArmDofs; ++i) {
      ruckig_input_.max_velocity[i] = 1.0;
      ruckig_input_.max_acceleration[i] = 2.0;
      ruckig_input_.max_jerk[i] = 10.0;
    }
  }

  void ArmStateCallback(const aimdk_msgs::msg::JointStateArray::SharedPtr msg) {
    for (int i = 0; i < kArmDofs; ++i) {
      for (const auto& joint : msg->joints) {
        if (joint.name == kArmJoints[i].name) {
          current_q_(i) = joint.position;
          break;
        }
      }
    }

    if (!ruckig_initialized_) {
      for (int i = 0; i < kArmDofs; ++i) {
        ruckig_input_.current_position[i] = current_q_(i);
        ruckig_input_.current_velocity[i] = 0.0;
        ruckig_input_.current_acceleration[i] = 0.0;
      }
      initial_q_ = current_q_;
      has_initial_q_ = true;
      ruckig_initialized_ = true;
      RCLCPP_INFO(get_logger(), "手臂状态已接收，Ruckig 初始化完成。准备运动到 Ready Pose...");
    }
  }

  void VRDataCallback(const aimdk_msgs::msg::VRData::SharedPtr msg) {
    latest_vr_data_ = msg;
    last_vr_time_ = now();
    if (!has_vr_data_) {
      has_vr_data_ = true;
      RCLCPP_INFO(get_logger(), "收到首次 VR 数据，开始遥操跟随。");
    }
  }

  void ControlLoop() {
    if (emergency_stop_ || !ruckig_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "等待手臂关节状态...");
      return;
    }

    // 急停：双侧 key_one + key_two
    if (latest_vr_data_ && latest_vr_data_->vr_controller_states.size() >= 2) {
      const auto& left = latest_vr_data_->vr_controller_states[0];
      const auto& right = latest_vr_data_->vr_controller_states[1];
      if (left.key_one && left.key_two && right.key_one && right.key_two) {
        TriggerEmergencyStop();
        return;
      }
    }

    if (demo_mode_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Demo 模式运行中 | T=切换左右手 H=回初始位置");
    }

    Eigen::VectorXd ik_targets = MapVRToArmTargets();
    for (int i = 0; i < kArmDofs; ++i) {
      ik_targets(i) = Clamp(ik_targets(i), kArmJoints[i].lower_limit,
                            kArmJoints[i].upper_limit);
    }

    for (int i = 0; i < kArmDofs; ++i) {
      ruckig_input_.target_position[i] = ik_targets(i);
      ruckig_input_.target_velocity[i] = 0.0;
      ruckig_input_.target_acceleration[i] = 0.0;
    }

    auto result = ruckig_.update(ruckig_input_, ruckig_output_);
    if (result == ruckig::Result::Error) {
      RCLCPP_ERROR(get_logger(), "Ruckig 轨迹规划错误");
      return;
    }

    aimdk_msgs::msg::JointCommandArray arm_cmd;
    arm_cmd.header.stamp = now();
    for (int i = 0; i < kArmDofs; ++i) {
      aimdk_msgs::msg::JointCommand joint;
      joint.name = kArmJoints[i].name;
      joint.position = ruckig_output_.new_position[i];
      joint.velocity = ruckig_output_.new_velocity[i];
      joint.stiffness = kArmJoints[i].kp;
      joint.damping = kArmJoints[i].kd;
      arm_cmd.joints.push_back(joint);
    }
    arm_cmd_pub_->publish(arm_cmd);

    for (int i = 0; i < kArmDofs; ++i) {
      ruckig_input_.current_position[i] = ruckig_output_.new_position[i];
      ruckig_input_.current_velocity[i] = ruckig_output_.new_velocity[i];
      ruckig_input_.current_acceleration[i] =
          ruckig_output_.new_acceleration[i];
    }

    // 回到初始位置完成后，清除标志
    if (returning_home_ && result == ruckig::Result::Finished) {
      returning_home_ = false;
      RCLCPP_INFO(get_logger(), "已回到初始位置。");
    }
  }

  // ─────────────────────────────────────────────────────────
  // VR -> 手臂关节目标
  // ─────────────────────────────────────────────────────────
  Eigen::VectorXd MapVRToArmTargets() {
    Eigen::VectorXd targets(kArmDofs);
    targets.setZero();

    if (returning_home_) {
      targets = initial_q_;
      return targets;
    }

    if (demo_mode_) {
      Eigen::Vector3d left_delta, right_delta;
      {
        std::lock_guard<std::mutex> lock(demo_delta_mutex_);
        left_delta = demo_left_delta_;
        right_delta = demo_right_delta_;
      }
      Eigen::VectorXd q_left =
          left_ik_.SolvePosition(left_wrist_ready_pos_ + left_delta,
                                 ready_pose_left_);
      Eigen::VectorXd q_right =
          right_ik_.SolvePosition(right_wrist_ready_pos_ + right_delta,
                                  ready_pose_right_);
      targets.segment(0, 7) = q_left;
      targets.segment(7, 7) = q_right;
      return targets;
    }

    // 尚未收到 VR 数据：保持在 Ready Pose
    if (!has_vr_data_) {
      targets.segment(0, 7) = ready_pose_left_;
      targets.segment(7, 7) = ready_pose_right_;
      return targets;
    }

    // VR 数据超时保护
    double dt = (now() - last_vr_time_).seconds();
    if (dt > vr_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "VR 数据超时 %.2fs，手臂回到 Ready Pose", dt);
      targets.segment(0, 7) = ready_pose_left_;
      targets.segment(7, 7) = ready_pose_right_;
      return targets;
    }

    const auto& states = latest_vr_data_->vr_controller_states;

    // ───── 左手柄 -> 左臂 ─────
    if (states.size() > 0) {
      const auto& ctrl = states[0];
      // VR 数据为相对位置：直接叠加到 Ready Pose 腕部位置
      Eigen::Vector3d vr_delta(ctrl.position.x * position_scale_,
                               ctrl.position.y * position_scale_,
                               ctrl.position.z * position_scale_);
      Eigen::Vector3d left_target = left_wrist_ready_pos_ + vr_delta;

      Eigen::VectorXd q_left =
          left_ik_.SolvePosition(left_target, ready_pose_left_);

      // 姿态映射
      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      QuaternionToZYX(ctrl.orientation, roll, pitch, yaw);
      q_left(4) = yaw;
      q_left(5) = pitch;
      q_left(6) = roll;

      targets.segment(0, 7) = q_left;
    } else {
      targets.segment(0, 7) = ready_pose_left_;
    }

    // ───── 右手柄 -> 右臂 ─────
    if (states.size() > 1) {
      const auto& ctrl = states[1];
      Eigen::Vector3d vr_delta(ctrl.position.x * position_scale_,
                               ctrl.position.y * position_scale_,
                               ctrl.position.z * position_scale_);
      Eigen::Vector3d right_target = right_wrist_ready_pos_ + vr_delta;

      Eigen::VectorXd q_right =
          right_ik_.SolvePosition(right_target, ready_pose_right_);

      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      QuaternionToZYX(ctrl.orientation, roll, pitch, yaw);
      q_right(4) = yaw;
      q_right(5) = pitch;
      q_right(6) = roll;

      targets.segment(7, 7) = q_right;
    } else {
      targets.segment(7, 7) = ready_pose_right_;
    }

    return targets;
  }

  // ─────────────────────────────────────────────────────────
  // 键盘 Demo 控制（仅 Linux/POSIX）
  // ─────────────────────────────────────────────────────────
  void InitKeyboard() {
    tcgetattr(STDIN_FILENO, &old_tio_);
    struct termios new_tio = old_tio_;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    keyboard_thread_ = std::thread(&VRTeleopNode::KeyboardLoop, this);
  }

  void RestoreKeyboard() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
  }

  void KeyboardLoop() {
    char c;
    constexpr double kStep = 0.005;
    while (!stop_keyboard_ && rclcpp::ok()) {
      ssize_t n = read(STDIN_FILENO, &c, 1);
      if (n == 1) {
        std::lock_guard<std::mutex> lock(demo_delta_mutex_);
        switch (c) {
          case 'w': case 'W':
            if (demo_control_left_) demo_left_delta_.x() += kStep;
            else demo_right_delta_.x() += kStep;
            break;
          case 's': case 'S':
            if (demo_control_left_) demo_left_delta_.x() -= kStep;
            else demo_right_delta_.x() -= kStep;
            break;
          case 'a': case 'A':
            if (demo_control_left_) demo_left_delta_.y() += kStep;
            else demo_right_delta_.y() += kStep;
            break;
          case 'd': case 'D':
            if (demo_control_left_) demo_left_delta_.y() -= kStep;
            else demo_right_delta_.y() -= kStep;
            break;
          case 'r': case 'R':
            if (demo_control_left_) demo_left_delta_.z() += kStep;
            else demo_right_delta_.z() += kStep;
            break;
          case 'f': case 'F':
            if (demo_control_left_) demo_left_delta_.z() -= kStep;
            else demo_right_delta_.z() -= kStep;
            break;
          case ' ':
            demo_left_delta_.setZero();
            demo_right_delta_.setZero();
            break;
          case 't': case 'T':
            demo_control_left_ = !demo_control_left_;
            std::cout << "[Demo] 切换控制手: "
                      << (demo_control_left_ ? "左手" : "右手") << std::endl;
            break;
          case 'h': case 'H':
            returning_home_ = true;
            demo_left_delta_.setZero();
            demo_right_delta_.setZero();
            std::cout << "[Demo] 触发回到初始位置" << std::endl;
            break;
        }
        std::cout << "[Demo] 当前控制: " << (demo_control_left_ ? "左" : "右")
                  << " 左偏移(" << demo_left_delta_.x() << ", "
                  << demo_left_delta_.y() << ", " << demo_left_delta_.z()
                  << ") 右偏移(" << demo_right_delta_.x() << ", "
                  << demo_right_delta_.y() << ", " << demo_right_delta_.z()
                  << ")" << std::endl;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }

  void TriggerEmergencyStop() {
    RCLCPP_ERROR(get_logger(), "!!! 急停触发，停止控制循环并发送保持命令 !!!");
    emergency_stop_ = true;
    control_timer_->cancel();

    aimdk_msgs::msg::JointCommandArray arm_cmd;
    arm_cmd.header.stamp = now();
    for (int i = 0; i < kArmDofs; ++i) {
      aimdk_msgs::msg::JointCommand joint;
      joint.name = kArmJoints[i].name;
      joint.position = ruckig_input_.current_position[i];
      joint.velocity = 0.0;
      joint.stiffness = kArmJoints[i].kp;
      joint.damping = kArmJoints[i].kd;
      arm_cmd.joints.push_back(joint);
    }
    arm_cmd_pub_->publish(arm_cmd);
  }

  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr
      arm_state_sub_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Subscription<aimdk_msgs::msg::VRData>::SharedPtr vr_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  ruckig::Ruckig<kArmDofs> ruckig_;
  ruckig::InputParameter<kArmDofs> ruckig_input_;
  ruckig::OutputParameter<kArmDofs> ruckig_output_;
  bool ruckig_initialized_{false};

  Eigen::VectorXd current_q_{kArmDofs};
  Eigen::VectorXd initial_q_{kArmDofs};
  bool has_initial_q_{false};
  aimdk_msgs::msg::VRData::SharedPtr latest_vr_data_;
  rclcpp::Time last_vr_time_{0, 0, RCL_ROS_TIME};

  SimpleArmIK left_ik_;
  SimpleArmIK right_ik_;

  // Ready Pose
  Eigen::VectorXd ready_pose_left_{7};
  Eigen::VectorXd ready_pose_right_{7};
  Eigen::Vector3d left_wrist_ready_pos_;
  Eigen::Vector3d right_wrist_ready_pos_;
  bool has_vr_data_{false};
  bool emergency_stop_{false};

  bool demo_mode_{false};
  double vr_timeout_sec_{0.5};
  double position_scale_{1.0};

  // Demo 键盘控制
  struct termios old_tio_;
  std::thread keyboard_thread_;
  bool stop_keyboard_{false};
  std::mutex demo_delta_mutex_;
  Eigen::Vector3d demo_left_delta_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d demo_right_delta_{Eigen::Vector3d::Zero()};
  bool demo_control_left_{true};

  // 手动回初始位置状态
  bool returning_home_{false};
};

}  // namespace vr_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vr_teleop::VRTeleopNode>();
  RCLCPP_INFO(rclcpp::get_logger("vr_teleop"),
              "VR Teleop (C++ IK, URDF-based, Ready-Pose) 节点启动，开始 spin...");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("vr_teleop"), "节点关闭。");
  rclcpp::shutdown();
  return 0;
}
