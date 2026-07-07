/**
 * @file vr_teleop_node.cpp
 * @brief X2 VR 遥操骨架节点 — 基于 x2_ultra.urdf 的手臂 IK
 *
 * 启动流程：
 *   1. 收到第一次手臂状态后，自动平滑运动到 Ready Pose（双手抬起、肘关节平直）
 *   2. 直接监听 UDP JSON v3，读取 teleop.left/right.pose
 *   3. teleop position 是校准后的相对端点，直接叠加到 Ready Pose 的腕部控制点上进行 IK
 *
 * 重要发现：X2 Ultra 的 URDF 显示手臂为 7-DOF：
 *   shoulder_pitch / shoulder_roll / shoulder_yaw / elbow /
 *   wrist_yaw / wrist_pitch / wrist_roll
 *
 * 机器人须处于 Develop_MC 模式。灵巧手命令由本节点解析 robot_control.hands 后发布。
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <mutex>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>

#include "aimdk_msgs/msg/joint_command_array.hpp"
#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "aimdk_msgs/msg/hand_command.hpp"
#include "aimdk_msgs/msg/hand_command_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace vr_teleop {

inline double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

constexpr double kPi = 3.14159265358979323846;
constexpr int kLeisaiHandDofs = 6;
constexpr std::array<double, kLeisaiHandDofs> kLeisaiHandMaxPosition = {
    1.75, 1.40, 1.40, 1.40, 1.40, 1.40};

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

inline Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  return (rz * ry * rx).toRotationMatrix();
}

struct ArmGeometry {
  Eigen::Vector3d shoulder_roll_origin{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d shoulder_roll_fixed_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d shoulder_yaw_origin{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d shoulder_yaw_fixed_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d elbow_origin{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d elbow_fixed_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d wrist_yaw_origin{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d wrist_yaw_fixed_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d wrist_pitch_origin{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d wrist_pitch_fixed_rotation{Eigen::Matrix3d::Identity()};
};

inline ArmGeometry MakeLeftArmGeometry() {
  ArmGeometry g;
  g.shoulder_roll_origin =
      Eigen::Vector3d(-0.000499991072907324, 0.0495, 0.0);
  g.shoulder_yaw_origin =
      Eigen::Vector3d(0.00122423638490546, 0.0, -0.121194848262102);
  g.shoulder_yaw_fixed_rotation =
      RotationFromRpy(0.0, -0.00597565694965793, 0.0);
  g.elbow_origin =
      Eigen::Vector3d(0.0140000000488655, 0.000199986464082369,
                      -0.0829500000002385);
  g.elbow_fixed_rotation =
      RotationFromRpy(0.0, 0.00597565694965793, 0.0);
  g.wrist_yaw_origin =
      Eigen::Vector3d(-0.0132797235062217, -0.0000975487744990788,
                      -0.120575783591803);
  g.wrist_pitch_origin =
      Eigen::Vector3d(0.000490001227040848, 0.0, -0.0819985359552045);
  g.wrist_pitch_fixed_rotation =
      RotationFromRpy(0.0, -0.00597566028430406, 0.0);
  return g;
}

inline ArmGeometry MakeRightArmGeometry() {
  ArmGeometry g;
  g.shoulder_roll_origin =
      Eigen::Vector3d(-0.000499999994070091, -0.0495000000000002, 0.0);
  g.shoulder_roll_fixed_rotation =
      RotationFromRpy(0.0, 0.000154003092926045, 0.0);
  g.shoulder_yaw_origin =
      Eigen::Vector3d(0.000499999999999765, 0.0, -0.121199999999934);
  g.elbow_origin =
      Eigen::Vector3d(0.0139999999999662, -0.0002000000000143,
                      -0.0829500000000745);
  g.wrist_yaw_origin =
      Eigen::Vector3d(-0.0140000017961392, 0.000097526096753231,
                      -0.120694276209686);
  g.wrist_yaw_fixed_rotation =
      RotationFromRpy(0.0, 0.0, 0.0000327341906322599);
  g.wrist_pitch_origin = Eigen::Vector3d(0.0, 0.0, -0.081799999999999);
  g.wrist_pitch_fixed_rotation =
      RotationFromRpy(0.0, 0.0, -0.0000327341932546355);
  return g;
}

// ─────────────────────────────────────────────────────────
// URDF 4-DOF 位置 IK
//
// J1..J4: shoulder_pitch / shoulder_roll / shoulder_yaw / elbow
// FK 控制点: wrist_pitch_link 原点；J5..J7 不参与位置 IK。
// 坐标系: 各臂 shoulder_pitch_link 零位局部坐标。
// ─────────────────────────────────────────────────────────
class SimpleArmIK {
 public:
  explicit SimpleArmIK(const ArmGeometry& geometry) : geometry_(geometry) {}
  SimpleArmIK() : SimpleArmIK(MakeLeftArmGeometry()) {}

  double damping = 0.05;
  int max_iterations = 30;
  double convergence_tol = 1e-4;

  Eigen::Vector3d ForwardPosition(const Eigen::VectorXd& q) const {
    double sp = q(0), sr = q(1), sy = q(2), ep = q(3);

    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(sp, Eigen::Vector3d::UnitY()).toRotationMatrix();

    auto apply_joint = [&](const Eigen::Vector3d& origin,
                           const Eigen::Matrix3d& fixed_rotation,
                           const Eigen::Matrix3d& joint_rotation) {
      p += R * origin;
      R = R * fixed_rotation * joint_rotation;
    };
    auto apply_fixed = [&](const Eigen::Vector3d& origin,
                           const Eigen::Matrix3d& fixed_rotation) {
      p += R * origin;
      R = R * fixed_rotation;
    };

    apply_joint(geometry_.shoulder_roll_origin,
                geometry_.shoulder_roll_fixed_rotation,
                Eigen::AngleAxisd(sr, Eigen::Vector3d::UnitX())
                    .toRotationMatrix());
    apply_joint(geometry_.shoulder_yaw_origin,
                geometry_.shoulder_yaw_fixed_rotation,
                Eigen::AngleAxisd(sy, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix());
    apply_joint(geometry_.elbow_origin, geometry_.elbow_fixed_rotation,
                Eigen::AngleAxisd(ep, Eigen::Vector3d::UnitY())
                    .toRotationMatrix());
    apply_fixed(geometry_.wrist_yaw_origin,
                geometry_.wrist_yaw_fixed_rotation);
    apply_fixed(geometry_.wrist_pitch_origin,
                geometry_.wrist_pitch_fixed_rotation);

    return p;
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
                                const Eigen::VectorXd& q_init,
                                const std::vector<JointInfo>& joint_limits,
                                int joint_offset,
                                const Eigen::VectorXd* q_preferred = nullptr,
                                double posture_gain = 0.0,
                                double iteration_step_limit = 0.05,
                                double max_solution_delta = 0.0) const {
    Eigen::VectorXd q = q_init;
    for (int iter = 0; iter < max_iterations; ++iter) {
      Eigen::Vector3d err = target_pos - ForwardPosition(q);
      if (err.norm() < convergence_tol) break;

      Eigen::MatrixXd J = ComputePositionJacobian(q);
      Eigen::Matrix3d JJt = J * J.transpose();
      JJt.diagonal().array() += damping * damping;
      Eigen::MatrixXd j_pinv =
          J.transpose() * JJt.ldlt().solve(Eigen::Matrix3d::Identity());
      Eigen::VectorXd dq = j_pinv * err;

      if (q_preferred != nullptr && posture_gain > 0.0) {
        Eigen::VectorXd posture_err(4);
        for (int i = 0; i < 4; ++i) {
          posture_err(i) = (*q_preferred)(i) - q(i);
        }
        Eigen::MatrixXd nullspace =
            Eigen::MatrixXd::Identity(4, 4) - j_pinv * J;
        dq += nullspace * (posture_gain * posture_err);
      }

      for (int i = 0; i < 4; ++i) {
        q(i) += Clamp(dq(i), -iteration_step_limit, iteration_step_limit);
        q(i) = Clamp(q(i), joint_limits[joint_offset + i].lower_limit,
                     joint_limits[joint_offset + i].upper_limit);
      }
    }
    if (max_solution_delta > 0.0) {
      for (int i = 0; i < 4; ++i) {
        q(i) = Clamp(q(i), q_init(i) - max_solution_delta,
                     q_init(i) + max_solution_delta);
        q(i) = Clamp(q(i), joint_limits[joint_offset + i].lower_limit,
                     joint_limits[joint_offset + i].upper_limit);
      }
    }
    return q;
  }

 private:
  ArmGeometry geometry_;
};

inline double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

struct ControllerState {
  std::string name;
  geometry_msgs::msg::Vector3 raw_position;
  geometry_msgs::msg::Vector3 position;
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
  double wrist_roll{0.0};
  double wrist_pitch{0.0};
  double wrist_yaw{0.0};
  double axis_x{0.0};
  double axis_y{0.0};
  double index_trig{0.0};
  double hand_trig{0.0};
  bool key_one{false};
  bool key_two{false};
};

struct ControllerSnapshot {
  bool has_data{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::vector<ControllerState> states;
};

struct JsonObjectRange {
  size_t begin{0};
  size_t end{0};  // one past matching '}'
};

struct JsonArrayRange {
  size_t begin{0};
  size_t end{0};  // one past matching ']'
};

struct HandRawCommand {
  bool has_left{false};
  bool has_right{false};
  std::array<double, kLeisaiHandDofs> left{};
  std::array<double, kLeisaiHandDofs> right{};
};

struct UdpControlCommand {
  bool zero_all{false};
};

class JsonFieldReader {
 public:
  explicit JsonFieldReader(const std::string& json) : json_(json) {}

  bool RootObject(JsonObjectRange* out) const {
    size_t pos = SkipWs(0);
    if (pos >= json_.size() || json_[pos] != '{') return false;
    size_t end = 0;
    if (!FindMatching(pos, '{', '}', &end)) return false;
    *out = JsonObjectRange{pos, end + 1};
    return true;
  }

  bool GetObject(const JsonObjectRange& object, const std::string& key,
                 JsonObjectRange* out) const {
    size_t value = 0;
    if (!FindTopLevelKey(object, key, &value)) return false;
    value = SkipWs(value);
    if (value >= json_.size() || json_[value] != '{') return false;
    size_t end = 0;
    if (!FindMatching(value, '{', '}', &end)) return false;
    *out = JsonObjectRange{value, end + 1};
    return true;
  }

  bool GetArray(const JsonObjectRange& object, const std::string& key,
                JsonArrayRange* out) const {
    size_t value = 0;
    if (!FindTopLevelKey(object, key, &value)) return false;
    value = SkipWs(value);
    if (value >= json_.size() || json_[value] != '[') return false;
    size_t end = 0;
    if (!FindMatching(value, '[', ']', &end)) return false;
    *out = JsonArrayRange{value, end + 1};
    return true;
  }

  bool GetNumberArray(const JsonArrayRange& array,
                      std::vector<double>* numbers) const {
    numbers->clear();
    size_t pos = SkipWs(array.begin + 1);
    const size_t array_close = array.end - 1;
    if (pos == array_close) return true;

    while (pos < array_close) {
      char* end_ptr = nullptr;
      const char* start = json_.c_str() + pos;
      errno = 0;
      const double number = std::strtod(start, &end_ptr);
      if (start == end_ptr || errno == ERANGE) return false;
      numbers->push_back(number);

      pos = SkipWs(static_cast<size_t>(end_ptr - json_.c_str()));
      if (pos == array_close) return true;
      if (pos >= array_close || json_[pos] != ',') return false;
      pos = SkipWs(pos + 1);
    }
    return pos == array_close;
  }

  std::optional<double> GetNumber(const JsonObjectRange& object,
                                  const std::string& key) const {
    size_t value = 0;
    if (!FindTopLevelKey(object, key, &value)) return std::nullopt;
    value = SkipWs(value);
    if (value >= json_.size()) return std::nullopt;

    char* end_ptr = nullptr;
    const char* start = json_.c_str() + value;
    errno = 0;
    const double number = std::strtod(start, &end_ptr);
    if (start == end_ptr || errno == ERANGE) return std::nullopt;
    return number;
  }

  std::optional<bool> GetBool(const JsonObjectRange& object,
                              const std::string& key) const {
    size_t value = 0;
    if (!FindTopLevelKey(object, key, &value)) return std::nullopt;
    value = SkipWs(value);
    if (json_.compare(value, 4, "true") == 0) return true;
    if (json_.compare(value, 5, "false") == 0) return false;
    return std::nullopt;
  }

  std::optional<std::string> GetString(const JsonObjectRange& object,
                                       const std::string& key) const {
    size_t value = 0;
    if (!FindTopLevelKey(object, key, &value)) return std::nullopt;
    value = SkipWs(value);
    std::string parsed;
    size_t next = 0;
    if (!ReadString(value, &parsed, &next)) return std::nullopt;
    return parsed;
  }

 private:
  size_t SkipWs(size_t pos) const {
    while (pos < json_.size() &&
           (json_[pos] == ' ' || json_[pos] == '\n' ||
            json_[pos] == '\r' || json_[pos] == '\t')) {
      ++pos;
    }
    return pos;
  }

  bool ReadString(size_t quote_pos, std::string* out, size_t* next) const {
    if (quote_pos >= json_.size() || json_[quote_pos] != '"') return false;
    std::string value;
    bool escape = false;
    for (size_t i = quote_pos + 1; i < json_.size(); ++i) {
      const char c = json_[i];
      if (escape) {
        switch (c) {
          case '"': value.push_back('"'); break;
          case '\\': value.push_back('\\'); break;
          case '/': value.push_back('/'); break;
          case 'b': value.push_back('\b'); break;
          case 'f': value.push_back('\f'); break;
          case 'n': value.push_back('\n'); break;
          case 'r': value.push_back('\r'); break;
          case 't': value.push_back('\t'); break;
          default: value.push_back(c); break;
        }
        escape = false;
        continue;
      }
      if (c == '\\') {
        escape = true;
        continue;
      }
      if (c == '"') {
        *out = value;
        *next = i + 1;
        return true;
      }
      value.push_back(c);
    }
    return false;
  }

  bool FindMatching(size_t open_pos, char open, char close,
                    size_t* close_pos) const {
    bool in_string = false;
    bool escape = false;
    int depth = 0;
    for (size_t i = open_pos; i < json_.size(); ++i) {
      const char c = json_[i];
      if (in_string) {
        if (escape) {
          escape = false;
        } else if (c == '\\') {
          escape = true;
        } else if (c == '"') {
          in_string = false;
        }
        continue;
      }
      if (c == '"') {
        in_string = true;
      } else if (c == open) {
        ++depth;
      } else if (c == close) {
        --depth;
        if (depth == 0) {
          *close_pos = i;
          return true;
        }
      }
    }
    return false;
  }

  bool FindTopLevelKey(const JsonObjectRange& object, const std::string& key,
                       size_t* value_pos) const {
    int depth = 0;
    for (size_t i = object.begin; i < object.end; ++i) {
      const char c = json_[i];
      if (c == '"') {
        std::string candidate;
        size_t next = 0;
        if (!ReadString(i, &candidate, &next)) return false;
        if (depth == 1) {
          size_t colon = SkipWs(next);
          if (colon < object.end && json_[colon] == ':' &&
              candidate == key) {
            *value_pos = colon + 1;
            return true;
          }
        }
        i = next > 0 ? next - 1 : i;
        continue;
      }

      if (c == '{' || c == '[') {
        ++depth;
        continue;
      }
      if (c == '}' || c == ']') {
        --depth;
        continue;
      }
    }
    return false;
  }

  const std::string& json_;
};

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
                  "键盘 Demo 控制: W/S=前后(X)  A/D=左右(Y)  R/F=上下(Z)  Space=重置  T=切换左右手  H=回到初始位置  Z=全部回零 | 步进 5mm");
    }
  }

  ~VRTeleopNode() {
    StopUdpJson();
    if (demo_mode_) {
      stop_keyboard_ = true;
      if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
      }
      RestoreKeyboard();
    }
  }

  void ReturnToInitialPose() {
    std::lock_guard<std::mutex> lock(control_mutex_);
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
    declare_parameter<std::string>("hand_command_topic",
                                   "/aima/hal/joint/hand/command");
    declare_parameter<int>("udp_port", 9999);
    declare_parameter<int>("udp_receive_buffer", 65535);
    declare_parameter<bool>("require_safe_to_execute", true);
    declare_parameter<double>("control_frequency", 100.0);
    declare_parameter<bool>("demo_mode", false);
    declare_parameter<double>("vr_timeout_sec", 0.5);
    declare_parameter<double>("position_scale", 1.0);  // 当前按接收位置已是相对量处理
    // 坐标重映射：沿用上一版 udp_vr_bridge 默认映射。
    declare_parameter<std::string>("coord_map_x", "-z");
    declare_parameter<std::string>("coord_map_y", "-x");
    declare_parameter<std::string>("coord_map_z", "y");
    declare_parameter<double>("teleop_limit_x", 0.30);
    declare_parameter<double>("teleop_limit_y", 0.30);
    declare_parameter<double>("teleop_limit_z", 0.30);
    declare_parameter<double>("ik_posture_gain", 0.03);
    declare_parameter<double>("ik_iteration_step_limit", 0.05);
    declare_parameter<double>("ik_max_solution_delta", 0.0);
    declare_parameter<double>("hand_command_velocity", 0.3);
    declare_parameter<double>("ruckig_max_velocity", 2.0);
    declare_parameter<double>("ruckig_max_acceleration", 4.0);
    declare_parameter<double>("ruckig_max_jerk", 20.0);
    declare_parameter<double>("elbow_straight_target", -0.2);
    declare_parameter<double>("elbow_full_extension_delta", 0.30);
    declare_parameter<double>("elbow_posture_gain", 0.08);
    declare_parameter<bool>("debug_teleop_log", true);
    declare_parameter<int>("debug_teleop_log_period_ms", 200);

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

    // 预计算 ready pose 下的腕部控制点位置（肩膀局部坐标）
    left_wrist_ready_pos_ = left_ik_.ForwardPosition(ready_pose_left_);
    right_wrist_ready_pos_ = right_ik_.ForwardPosition(ready_pose_right_);

    RCLCPP_INFO(get_logger(),
                "Ready Pose 已设置 | 左腕控制点: (%.3f, %.3f, %.3f) | 右腕控制点: (%.3f, %.3f, %.3f)",
                left_wrist_ready_pos_.x(), left_wrist_ready_pos_.y(),
                left_wrist_ready_pos_.z(), right_wrist_ready_pos_.x(),
                right_wrist_ready_pos_.y(), right_wrist_ready_pos_.z());
  }

  void InitCommunication() {
    std::string arm_state_topic = get_parameter("arm_state_topic").as_string();
    std::string arm_cmd_topic = get_parameter("arm_command_topic").as_string();
    std::string hand_cmd_topic =
        get_parameter("hand_command_topic").as_string();
    demo_mode_ = get_parameter("demo_mode").as_bool();
    udp_port_ = get_parameter("udp_port").as_int();
    udp_receive_buffer_ = get_parameter("udp_receive_buffer").as_int();
    require_safe_to_execute_ =
        get_parameter("require_safe_to_execute").as_bool();
    vr_timeout_sec_ = get_parameter("vr_timeout_sec").as_double();
    position_scale_ = get_parameter("position_scale").as_double();
    coord_map_ = {get_parameter("coord_map_x").as_string(),
                  get_parameter("coord_map_y").as_string(),
                  get_parameter("coord_map_z").as_string()};
    teleop_limit_x_ = get_parameter("teleop_limit_x").as_double();
    teleop_limit_y_ = get_parameter("teleop_limit_y").as_double();
    teleop_limit_z_ = get_parameter("teleop_limit_z").as_double();
    ik_posture_gain_ = get_parameter("ik_posture_gain").as_double();
    ik_iteration_step_limit_ =
        get_parameter("ik_iteration_step_limit").as_double();
    ik_max_solution_delta_ = get_parameter("ik_max_solution_delta").as_double();
    hand_command_velocity_ =
        get_parameter("hand_command_velocity").as_double();
    elbow_straight_target_ = get_parameter("elbow_straight_target").as_double();
    elbow_full_extension_delta_ =
        get_parameter("elbow_full_extension_delta").as_double();
    elbow_posture_gain_ = get_parameter("elbow_posture_gain").as_double();
    debug_teleop_log_ = get_parameter("debug_teleop_log").as_bool();
    debug_teleop_log_period_ms_ =
        std::max<int64_t>(1, get_parameter("debug_teleop_log_period_ms").as_int());

    arm_state_sub_ = create_subscription<aimdk_msgs::msg::JointStateArray>(
        arm_state_topic, rclcpp::SensorDataQoS(),
        std::bind(&VRTeleopNode::ArmStateCallback, this,
                  std::placeholders::_1));

    arm_cmd_pub_ = create_publisher<aimdk_msgs::msg::JointCommandArray>(
        arm_cmd_topic, rclcpp::SensorDataQoS());
    hand_cmd_pub_ = create_publisher<aimdk_msgs::msg::HandCommandArray>(
        hand_cmd_topic, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        get_logger(),
        "Teleop 映射: x=%s y=%s z=%s | limit=(%.2f %.2f %.2f)m | IK posture=%.3f step=%.3f max_delta=%.3f",
        coord_map_[0].c_str(), coord_map_[1].c_str(), coord_map_[2].c_str(),
        teleop_limit_x_, teleop_limit_y_, teleop_limit_z_, ik_posture_gain_,
        ik_iteration_step_limit_, ik_max_solution_delta_);

    if (!demo_mode_) {
      StartUdpJson();
      RCLCPP_INFO(get_logger(),
                  "直接监听 UDP JSON v3: 0.0.0.0:%d | 控制循环按 UDP 接收帧触发 | 手部命令: %s",
                  udp_port_, hand_cmd_topic.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  "Demo 模式已启用，将使用键盘 WASD 模拟相对位置遥操");
      const double control_freq = get_parameter("control_frequency").as_double();
      ruckig_.delta_time = 1.0 / std::max(1e-6, control_freq);
      auto period_ms =
          std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq));
      control_timer_ = create_wall_timer(
          period_ms, std::bind(&VRTeleopNode::ControlLoop, this));
    }

    current_q_.setZero(kArmDofs);
  }

  void InitRuckig() {
    const double max_velocity = get_parameter("ruckig_max_velocity").as_double();
    const double max_acceleration =
        get_parameter("ruckig_max_acceleration").as_double();
    const double max_jerk = get_parameter("ruckig_max_jerk").as_double();

    for (int i = 0; i < kArmDofs; ++i) {
      ruckig_input_.max_velocity[i] = max_velocity;
      ruckig_input_.max_acceleration[i] = max_acceleration;
      ruckig_input_.max_jerk[i] = max_jerk;
    }
  }

  void ArmStateCallback(const aimdk_msgs::msg::JointStateArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(control_mutex_);
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
      moving_to_ready_ = true;
      if (!demo_mode_) {
        StartReadyPoseTimer();
      }
      RCLCPP_INFO(get_logger(), "手臂状态已接收，Ruckig 初始化完成。准备运动到 Ready Pose...");
    }
  }

  void StartReadyPoseTimer() {
    if (ready_pose_timer_) {
      ready_pose_timer_->reset();
      return;
    }

    const double control_freq = get_parameter("control_frequency").as_double();
    ruckig_.delta_time = 1.0 / std::max(1e-6, control_freq);
    const auto period_ms =
        std::chrono::milliseconds(
            std::max(1, static_cast<int>(1000.0 / control_freq)));
    ready_pose_timer_ = create_wall_timer(
        period_ms, std::bind(&VRTeleopNode::ControlLoop, this));
  }

  bool UpdateControllerInput(const std::vector<ControllerState>& states) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const bool first_input = !has_input_data_;
    latest_controller_states_ = states;
    last_input_time_ = now();
    has_input_data_ = true;
    return first_input;
  }

  ControllerSnapshot GetControllerSnapshot() {
    std::lock_guard<std::mutex> lock(input_mutex_);
    ControllerSnapshot snapshot;
    snapshot.has_data = has_input_data_;
    snapshot.stamp = last_input_time_;
    snapshot.states = latest_controller_states_;
    return snapshot;
  }

  void StartUdpTimeoutTimer() {
    if (demo_mode_) return;

    const double control_freq = get_parameter("control_frequency").as_double();
    const auto period_ms =
        std::chrono::milliseconds(
            std::max(1, static_cast<int>(1000.0 / control_freq)));
    if (!udp_timeout_timer_) {
      udp_timeout_timer_ = create_wall_timer(
          period_ms, std::bind(&VRTeleopNode::CheckUdpTimeout, this));
      return;
    }
    udp_timeout_timer_->reset();
  }

  void StopUdpTimeoutTimer() {
    if (udp_timeout_timer_) {
      udp_timeout_timer_->cancel();
    }
  }

  void CheckUdpTimeout() {
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (demo_mode_ || emergency_stop_ || !ruckig_initialized_ ||
          moving_to_ready_ || returning_home_ || returning_zero_) {
        return;
      }
    }

    const ControllerSnapshot input = GetControllerSnapshot();
    if (!input.has_data) {
      return;
    }

    const double dt = (now() - input.stamp).seconds();
    if (dt <= vr_timeout_sec_) {
      return;
    }

    if (!returning_timeout_ready_.exchange(true)) {
      RCLCPP_WARN(get_logger(),
                  "UDP 遥操 %.2fs 未更新，双臂回到 Ready Pose。", dt);
    }
    ControlLoop();
  }

  void StartUdpJson() {
    udp_sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock_fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "创建 UDP socket 失败: %s", std::strerror(errno));
      rclcpp::shutdown();
      return;
    }

    int reuse = 1;
    setsockopt(udp_sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (udp_receive_buffer_ > 0) {
      int rcvbuf = udp_receive_buffer_;
      setsockopt(udp_sock_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    }

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;
    setsockopt(udp_sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout,
               sizeof(timeout));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(udp_port_));
    if (bind(udp_sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
        0) {
      RCLCPP_FATAL(get_logger(), "绑定 UDP 端口 %d 失败: %s", udp_port_,
                   std::strerror(errno));
      close(udp_sock_fd_);
      udp_sock_fd_ = -1;
      rclcpp::shutdown();
      return;
    }

    stop_udp_ = false;
    udp_thread_ = std::thread(&VRTeleopNode::UdpJsonLoop, this);
  }

  void StopUdpJson() {
    stop_udp_ = true;
    if (udp_sock_fd_ >= 0) {
      close(udp_sock_fd_);
    }
    if (udp_thread_.joinable()) {
      udp_thread_.join();
    }
    udp_sock_fd_ = -1;
  }

  void UdpJsonLoop() {
    const size_t buffer_size =
        static_cast<size_t>(std::max(1024, udp_receive_buffer_));
    std::vector<char> buffer(buffer_size);

    while (!stop_udp_ && rclcpp::ok()) {
      sockaddr_in src_addr{};
      socklen_t src_len = sizeof(src_addr);
      const ssize_t received =
          recvfrom(udp_sock_fd_, buffer.data(), buffer.size(), 0,
                   reinterpret_cast<sockaddr*>(&src_addr), &src_len);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ||
            stop_udp_) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "UDP 接收异常: %s", std::strerror(errno));
        continue;
      }
      if (received == 0) continue;

      std::string payload(buffer.data(), static_cast<size_t>(received));
      std::vector<ControllerState> states;
      HandRawCommand hand_raw;
      UdpControlCommand command;
      std::string reason;
      if (!ParseUdpJsonV3(payload, &states, &hand_raw, &command, &reason)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "UDP JSON v3 解析失败: %s", reason.c_str());
        continue;
      }

      if (UpdateControllerInput(states)) {
        RCLCPP_INFO(get_logger(), "收到首次 UDP JSON v3，开始遥操跟随。");
      }
      if (command.zero_all) {
        returning_timeout_ready_ = false;
        StopUdpTimeoutTimer();
        RequestZeroAll();
      } else if (!states.empty()) {
        returning_timeout_ready_ = false;
        StartUdpTimeoutTimer();
      }
      if (hand_raw.has_left || hand_raw.has_right) {
        PublishHandCommand(hand_raw);
      }
      ControlLoop();
    }
  }

  bool ParseUdpJsonV3(const std::string& payload,
                      std::vector<ControllerState>* states,
                      HandRawCommand* hand_raw,
                      UdpControlCommand* command,
                      std::string* reason) const {
    JsonFieldReader reader(payload);
    JsonObjectRange root;
    if (!reader.RootObject(&root)) {
      *reason = "top level is not an object";
      return false;
    }

    const auto packet_version = reader.GetNumber(root, "packet_version");
    if (!packet_version || static_cast<int>(*packet_version) != 3) {
      *reason = "packet_version is not 3";
      return false;
    }

    const std::string operator_mode =
        reader.GetString(root, "operator_mode").value_or("");

    bool motion_allowed = true;
    if (require_safe_to_execute_) {
      JsonObjectRange safety;
      const bool safe_to_execute =
          reader.GetObject(root, "safety", &safety)
              ? reader.GetBool(safety, "safe_to_execute").value_or(false)
              : false;
      motion_allowed = safe_to_execute && operator_mode != "stop_signal";
    }

    states->clear();
    states->reserve(2);
    command->zero_all = operator_mode == "zero_all";
    ParseRobotControl(reader, root, motion_allowed, hand_raw, command);

    JsonObjectRange teleop;
    if (!reader.GetObject(root, "teleop", &teleop)) {
      if (command->zero_all) return true;
      *reason = "missing teleop";
      return false;
    }

    JsonObjectRange left;
    JsonObjectRange right;
    const bool has_left = reader.GetObject(teleop, "left", &left);
    const bool has_right = reader.GetObject(teleop, "right", &right);
    if (!has_left && !has_right) {
      if (command->zero_all) return true;
      *reason = "missing teleop.left/right";
      return false;
    }

    states->push_back(BuildControllerStateFromJson(reader, left, has_left, "left",
                                                   motion_allowed));
    states->push_back(BuildControllerStateFromJson(reader, right, has_right,
                                                   "right", motion_allowed));
    return true;
  }

  ControllerState BuildControllerStateFromJson(
      const JsonFieldReader& reader, const JsonObjectRange& controller,
      bool has_controller, const std::string& side,
      bool motion_allowed) const {
    ControllerState ctrl;
    ctrl.name = side;

    JsonObjectRange input;
    if (has_controller && reader.GetObject(controller, "input", &input)) {
      JsonObjectRange thumbstick;
      if (reader.GetObject(input, "thumbstick", &thumbstick)) {
        ctrl.axis_x = reader.GetNumber(thumbstick, "x").value_or(0.0);
        ctrl.axis_y = reader.GetNumber(thumbstick, "y").value_or(0.0);
      }
      ctrl.index_trig = reader.GetNumber(input, "trigger").value_or(0.0);
      ctrl.hand_trig = reader.GetNumber(input, "grip").value_or(0.0);
      ctrl.key_one = reader.GetBool(input, "primary_pressed").value_or(false);
      ctrl.key_two = reader.GetBool(input, "secondary_pressed").value_or(false);
    }

    if (!has_controller || !motion_allowed ||
        !ControllerPoseLive(reader, controller)) {
      ctrl.raw_position = geometry_msgs::msg::Vector3();
      ctrl.position = geometry_msgs::msg::Vector3();
      return ctrl;
    }

    JsonObjectRange pose;
    JsonObjectRange position;
    if (!reader.GetObject(controller, "pose", &pose)) {
      return ctrl;
    }
    if (reader.GetObject(pose, "position", &position)) {
      ctrl.raw_position.x = reader.GetNumber(position, "x").value_or(0.0);
      ctrl.raw_position.y = reader.GetNumber(position, "y").value_or(0.0);
      ctrl.raw_position.z = reader.GetNumber(position, "z").value_or(0.0);
      ctrl.position = ApplyCoordinateMap(ctrl.raw_position);
    }
    // 按 UDP JSON v3 协议读取 teleop.*.pose.orientation，单位 deg。
    ReadControllerWristAngles(reader, pose, &ctrl);
    return ctrl;
  }

  bool ReadControllerWristAngles(const JsonFieldReader& reader,
                                 const JsonObjectRange& pose,
                                 ControllerState* ctrl) const {
    JsonObjectRange orientation;
    if (!reader.GetObject(pose, "orientation", &orientation)) {
      return false;
    }
    const auto roll_deg = reader.GetNumber(orientation, "roll");
    const auto pitch_deg = reader.GetNumber(orientation, "pitch");
    const auto yaw_deg = reader.GetNumber(orientation, "yaw");
    if (!roll_deg || !pitch_deg || !yaw_deg) {
      return false;
    }

    ctrl->roll_deg = *roll_deg;
    ctrl->pitch_deg = *pitch_deg;
    ctrl->yaw_deg = *yaw_deg;
    ctrl->wrist_roll = DegreesToRadians(ctrl->roll_deg);
    ctrl->wrist_pitch = DegreesToRadians(ctrl->pitch_deg);
    ctrl->wrist_yaw = DegreesToRadians(ctrl->yaw_deg);
    return true;
  }

  double MapCoordinateAxis(const geometry_msgs::msg::Vector3& raw,
                           const std::string& spec) const {
    if (spec.empty()) {
      return 0.0;
    }

    const bool negative = spec.front() == '-';
    const std::string axis = negative ? spec.substr(1) : spec;
    double value = 0.0;
    if (axis == "x") {
      value = raw.x;
    } else if (axis == "y") {
      value = raw.y;
    } else if (axis == "z") {
      value = raw.z;
    }
    return negative ? -value : value;
  }

  geometry_msgs::msg::Vector3 ApplyCoordinateMap(
      const geometry_msgs::msg::Vector3& raw) const {
    geometry_msgs::msg::Vector3 mapped;
    mapped.x = MapCoordinateAxis(raw, coord_map_[0]);
    mapped.y = MapCoordinateAxis(raw, coord_map_[1]);
    mapped.z = MapCoordinateAxis(raw, coord_map_[2]);
    return mapped;
  }

  Eigen::Vector3d BuildTeleopDelta(const ControllerState& ctrl) const {
    return Eigen::Vector3d(
        Clamp(ctrl.position.x * position_scale_, -teleop_limit_x_,
              teleop_limit_x_),
        Clamp(ctrl.position.y * position_scale_, -teleop_limit_y_,
              teleop_limit_y_),
        Clamp(ctrl.position.z * position_scale_, -teleop_limit_z_,
              teleop_limit_z_));
  }

  void ParseRobotControl(const JsonFieldReader& reader,
                         const JsonObjectRange& root,
                         bool motion_allowed,
                         HandRawCommand* hand_raw,
                         UdpControlCommand* command) const {
    *hand_raw = HandRawCommand();

    JsonObjectRange robot_control;
    if (!reader.GetObject(root, "robot_control", &robot_control)) {
      if (command->zero_all) {
        hand_raw->has_left = true;
        hand_raw->has_right = true;
        hand_raw->left.fill(0.0);
        hand_raw->right.fill(0.0);
      }
      return;
    }

    command->zero_all =
        command->zero_all ||
        reader.GetBool(robot_control, "zero_all").value_or(false);

    JsonObjectRange arms;
    if (reader.GetObject(robot_control, "arms", &arms)) {
      command->zero_all =
          command->zero_all ||
          reader.GetBool(arms, "zero_all").value_or(false);
    }

    JsonObjectRange hands;
    if (reader.GetObject(robot_control, "hands", &hands)) {
      ReadHandRawSide(reader, hands, "left", motion_allowed,
                      &hand_raw->has_left, &hand_raw->left);
      ReadHandRawSide(reader, hands, "right", motion_allowed,
                      &hand_raw->has_right, &hand_raw->right);
    }

    if (command->zero_all) {
      hand_raw->has_left = true;
      hand_raw->has_right = true;
      hand_raw->left.fill(0.0);
      hand_raw->right.fill(0.0);
    }
  }

  void ReadHandRawSide(const JsonFieldReader& reader,
                       const JsonObjectRange& hands,
                       const std::string& side,
                       bool motion_allowed,
                       bool* has_side,
                       std::array<double, kLeisaiHandDofs>* raw) const {
    JsonArrayRange array;
    std::vector<double> numbers;
    if (!reader.GetArray(hands, side, &array) ||
        !reader.GetNumberArray(array, &numbers) ||
        numbers.size() != raw->size()) {
      return;
    }

    *has_side = true;
    if (!motion_allowed) {
      raw->fill(0.0);
      return;
    }

    for (size_t i = 0; i < raw->size(); ++i) {
      (*raw)[i] = Clamp(numbers[i], 0.0, 10000.0);
    }
  }

  bool ControllerPoseLive(const JsonFieldReader& reader,
                          const JsonObjectRange& controller) const {
    const auto valid = reader.GetBool(controller, "valid");
    if (valid && !*valid) {
      return false;
    }
    const auto calibration_applied =
        reader.GetBool(controller, "calibration_applied");
    if (calibration_applied && !*calibration_applied) {
      return false;
    }
    const std::string quality =
        reader.GetString(controller, "quality").value_or("live");
    return quality.empty() || quality == "live";
  }

  std::vector<aimdk_msgs::msg::HandCommand> BuildHandCommands(
      const std::array<double, kLeisaiHandDofs>& raw) const {
    std::vector<aimdk_msgs::msg::HandCommand> cmds;
    cmds.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
      aimdk_msgs::msg::HandCommand cmd;
      cmd.name = "";
      cmd.position =
          Clamp(raw[i], 0.0, 10000.0) / 10000.0 * kLeisaiHandMaxPosition[i];
      cmd.velocity = hand_command_velocity_;
      cmd.acceleration = 0.0;
      cmd.deceleration = 0.0;
      cmd.effort = 0.0;
      cmds.push_back(cmd);
    }
    return cmds;
  }

  void PublishHandCommand(const HandRawCommand& hand_raw) {
    if (!hand_cmd_pub_) return;

    std::array<double, kLeisaiHandDofs> zero{};
    const auto& left = hand_raw.has_left ? hand_raw.left : zero;
    const auto& right = hand_raw.has_right ? hand_raw.right : zero;

    auto msg = std::make_unique<aimdk_msgs::msg::HandCommandArray>();
    msg->header.stamp = now();
    msg->left_hand_type.value = 3;
    msg->right_hand_type.value = 3;
    msg->left_hands = BuildHandCommands(left);
    msg->right_hands = BuildHandCommands(right);
    hand_cmd_pub_->publish(std::move(msg));
  }

  void ControlLoop() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (emergency_stop_ || !ruckig_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "等待手臂关节状态...");
      return;
    }

    const rclcpp::Time control_now = now();
    if (!demo_mode_) {
      if (last_control_time_.nanoseconds() > 0) {
        const double dt = (control_now - last_control_time_).seconds();
        ruckig_.delta_time = Clamp(dt, 0.001, 0.10);
      }
      last_control_time_ = control_now;
    }

    if (zero_all_requested_.exchange(false)) {
      returning_zero_ = true;
      returning_home_ = false;
      RCLCPP_WARN(get_logger(), "收到全部回零指令，双臂 14 轴平滑回到 0。");
    }

    ControllerSnapshot input = GetControllerSnapshot();

    // 急停：双侧 key_one + key_two
    if (input.states.size() >= 2) {
      const auto& left = input.states[0];
      const auto& right = input.states[1];
      if (left.key_one && left.key_two && right.key_one && right.key_two) {
        TriggerEmergencyStop();
        return;
      }
    }

    if (demo_mode_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Demo 模式运行中 | T=切换左右手 H=回初始位置");
    }

    Eigen::VectorXd ik_targets = MapVRToArmTargets(input);
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

    if (result == ruckig::Result::Finished) {
      if (moving_to_ready_) {
        moving_to_ready_ = false;
        if (ready_pose_timer_) {
          ready_pose_timer_->cancel();
        }
        RCLCPP_INFO(get_logger(), "已到达 Ready Pose。");
      }
      if (returning_timeout_ready_) {
        returning_timeout_ready_ = false;
        StopUdpTimeoutTimer();
        RCLCPP_INFO(get_logger(), "UDP 遥操停止，双臂已回到 Ready Pose。");
      }
      if (returning_zero_) {
        returning_zero_ = false;
        RCLCPP_INFO(get_logger(), "双臂已全部回零。");
      }
      if (returning_home_) {
        returning_home_ = false;
        RCLCPP_INFO(get_logger(), "已回到初始位置。");
      }
    }
  }

  // ─────────────────────────────────────────────────────────
  // VR -> 手臂关节目标
  // ─────────────────────────────────────────────────────────
  double ElbowRestTarget(const Eigen::VectorXd& ready_pose,
                         const Eigen::Vector3d& vr_delta) const {
    const double full_extension_delta =
        std::max(1e-6, elbow_full_extension_delta_);
    const double t = Clamp(vr_delta.norm() / full_extension_delta, 0.0, 1.0);
    return ready_pose(3) + (elbow_straight_target_ - ready_pose(3)) * t;
  }

  void ApplyControllerWrist(const ControllerState& ctrl, Eigen::VectorXd& q,
                            int joint_offset) const {
    // orientation.pitch/yaw/roll 在解析时已由 deg 转 rad。
    // 机器人侧按实机方向交换 roll/pitch，并反向 roll:
    // yaw -> wrist_yaw, -roll -> wrist_pitch, pitch -> wrist_roll.
    q(4) = Clamp(ctrl.wrist_yaw, kArmJoints[joint_offset + 4].lower_limit,
                 kArmJoints[joint_offset + 4].upper_limit);
    q(5) = Clamp(-ctrl.wrist_roll, kArmJoints[joint_offset + 5].lower_limit,
                 kArmJoints[joint_offset + 5].upper_limit);
    q(6) = Clamp(ctrl.wrist_pitch, kArmJoints[joint_offset + 6].lower_limit,
                 kArmJoints[joint_offset + 6].upper_limit);
  }

  void LogTeleopDebug(const char* side,
                      const ControllerState& ctrl,
                      const Eigen::Vector3d& vr_delta,
                      const Eigen::Vector3d& target_pos,
                      const Eigen::Vector3d& fk_pos,
                      const Eigen::VectorXd& q) {
    if (!debug_teleop_log_) return;

    const rclcpp::Time log_now = now();
    rclcpp::Time& last_log_time =
        std::strcmp(side, "LEFT") == 0 ? last_left_debug_log_time_
                                       : last_right_debug_log_time_;
    const int64_t period_ns = debug_teleop_log_period_ms_ * 1000000LL;
    if (last_log_time.nanoseconds() > 0 &&
        (log_now - last_log_time).nanoseconds() < period_ns) {
      return;
    }
    last_log_time = log_now;

    const Eigen::Vector3d ik_error = target_pos - fk_pos;
    RCLCPP_INFO(
        get_logger(),
        "[%s] teleop_raw=(%.4f %.4f %.4f) teleop_mapped=(%.4f %.4f %.4f) "
        "delta=(%.4f %.4f %.4f) "
        "target=(%.4f %.4f %.4f) fk=(%.4f %.4f %.4f) "
        "err=(%.4f %.4f %.4f) err_norm=%.5f "
        "IK4=(%.4f %.4f %.4f %.4f) rpy_deg=(r=%.2f p=%.2f y=%.2f) "
        "rpy_rad=(r=%.4f p=%.4f y=%.4f) wrist_out=(yaw<-y %.4f, pitch<--r %.4f, roll<-p %.4f)",
        side,
        ctrl.raw_position.x, ctrl.raw_position.y, ctrl.raw_position.z,
        ctrl.position.x, ctrl.position.y, ctrl.position.z,
        vr_delta.x(), vr_delta.y(), vr_delta.z(),
        target_pos.x(), target_pos.y(), target_pos.z(),
        fk_pos.x(), fk_pos.y(), fk_pos.z(),
        ik_error.x(), ik_error.y(), ik_error.z(), ik_error.norm(),
        q(0), q(1), q(2), q(3),
        ctrl.roll_deg, ctrl.pitch_deg, ctrl.yaw_deg,
        ctrl.wrist_roll, ctrl.wrist_pitch, ctrl.wrist_yaw,
        q(4), q(5), q(6));
  }

  Eigen::VectorXd CommandSeed(int joint_offset) const {
    Eigen::VectorXd seed(7);
    for (int i = 0; i < 7; ++i) {
      seed(i) = ruckig_input_.current_position[joint_offset + i];
    }
    return seed;
  }

  Eigen::VectorXd MapVRToArmTargets(const ControllerSnapshot& input) {
    Eigen::VectorXd targets(kArmDofs);
    targets.setZero();
    constexpr double kReadyPoseZeroThreshold = 0.005;

    if (returning_home_) {
      targets = initial_q_;
      return targets;
    }

    if (returning_zero_) {
      targets.setZero();
      return targets;
    }

    if (moving_to_ready_) {
      targets.segment(0, 7) = ready_pose_left_;
      targets.segment(7, 7) = ready_pose_right_;
      return targets;
    }

    if (returning_timeout_ready_) {
      targets.segment(0, 7) = ready_pose_left_;
      targets.segment(7, 7) = ready_pose_right_;
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
                                 CommandSeed(0), kArmJoints, 0,
                                 &ready_pose_left_, ik_posture_gain_,
                                 ik_iteration_step_limit_,
                                 ik_max_solution_delta_);
      Eigen::VectorXd q_right =
          right_ik_.SolvePosition(right_wrist_ready_pos_ + right_delta,
                                  CommandSeed(7), kArmJoints, 7,
                                  &ready_pose_right_, ik_posture_gain_,
                                  ik_iteration_step_limit_,
                                  ik_max_solution_delta_);
      targets.segment(0, 7) = q_left;
      targets.segment(7, 7) = q_right;
      return targets;
    }

    // 尚未收到 UDP JSON v3 数据：保持在 Ready Pose
    if (!input.has_data) {
      targets.segment(0, 7) = ready_pose_left_;
      targets.segment(7, 7) = ready_pose_right_;
      return targets;
    }

    // UDP JSON v3 数据超时保护
    double dt = (now() - input.stamp).seconds();
    if (dt > vr_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "UDP JSON v3 数据超时 %.2fs，手臂回到 Ready Pose", dt);
      targets.segment(0, 7) = ready_pose_left_;
      targets.segment(7, 7) = ready_pose_right_;
      return targets;
    }

    const auto& states = input.states;

    // ───── 左手柄 -> 左臂 ─────
    if (states.size() > 0) {
      const auto& ctrl = states[0];
      // teleop position 是校准后的相对位置：直接叠加到 Ready Pose 腕部控制点。
      Eigen::Vector3d vr_delta = BuildTeleopDelta(ctrl);
      const bool left_near_zero = vr_delta.norm() < kReadyPoseZeroThreshold;
      if (left_near_zero) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "LEFT teleop position near zero, returning to Ready Pose");
        vr_delta.setZero();
      }

      Eigen::Vector3d left_target = left_wrist_ready_pos_ + vr_delta;

      Eigen::VectorXd q_left =
          left_ik_.SolvePosition(left_target, CommandSeed(0), kArmJoints, 0,
                                 &ready_pose_left_,
                                 ik_posture_gain_, ik_iteration_step_limit_,
                                 ik_max_solution_delta_);
      if (left_near_zero) {
        q_left(4) = ready_pose_left_(4);
        q_left(5) = ready_pose_left_(5);
        q_left(6) = ready_pose_left_(6);
      } else {
        ApplyControllerWrist(ctrl, q_left, 0);
      }
      LogTeleopDebug("LEFT", ctrl, vr_delta, left_target,
                     left_ik_.ForwardPosition(q_left), q_left);
      targets.segment(0, 7) = q_left;
    } else {
      targets.segment(0, 7) = ready_pose_left_;
    }

    // ───── 右手柄 -> 右臂 ─────
    if (states.size() > 1) {
      const auto& ctrl = states[1];
      Eigen::Vector3d vr_delta = BuildTeleopDelta(ctrl);
      const bool right_near_zero = vr_delta.norm() < kReadyPoseZeroThreshold;
      if (right_near_zero) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "RIGHT teleop position near zero, returning to Ready Pose");
        vr_delta.setZero();
      }

      Eigen::Vector3d right_target = right_wrist_ready_pos_ + vr_delta;

      Eigen::VectorXd q_right =
          right_ik_.SolvePosition(right_target, CommandSeed(7), kArmJoints, 7,
                                  &ready_pose_right_,
                                  ik_posture_gain_, ik_iteration_step_limit_,
                                  ik_max_solution_delta_);
      if (right_near_zero) {
        q_right(4) = ready_pose_right_(4);
        q_right(5) = ready_pose_right_(5);
        q_right(6) = ready_pose_right_(6);
      } else {
        ApplyControllerWrist(ctrl, q_right, 7);
      }
      LogTeleopDebug("RIGHT", ctrl, vr_delta, right_target,
                     right_ik_.ForwardPosition(q_right), q_right);
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
            returning_zero_ = false;
            demo_left_delta_.setZero();
            demo_right_delta_.setZero();
            std::cout << "[Demo] 触发回到初始位置" << std::endl;
            break;
          case 'z': case 'Z':
            RequestZeroAll();
            demo_left_delta_.setZero();
            demo_right_delta_.setZero();
            std::cout << "[Demo] 触发双臂全部回零" << std::endl;
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
    if (control_timer_) {
      control_timer_->cancel();
    }

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

  void RequestZeroAll() {
    zero_all_requested_ = true;
  }

  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr
      arm_state_sub_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<aimdk_msgs::msg::HandCommandArray>::SharedPtr hand_cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr ready_pose_timer_;
  rclcpp::TimerBase::SharedPtr udp_timeout_timer_;

  ruckig::Ruckig<kArmDofs> ruckig_;
  ruckig::InputParameter<kArmDofs> ruckig_input_;
  ruckig::OutputParameter<kArmDofs> ruckig_output_;
  bool ruckig_initialized_{false};

  Eigen::VectorXd current_q_{kArmDofs};
  Eigen::VectorXd initial_q_{kArmDofs};
  bool has_initial_q_{false};
  bool moving_to_ready_{false};
  std::mutex control_mutex_;
  std::mutex input_mutex_;
  std::vector<ControllerState> latest_controller_states_;
  rclcpp::Time last_input_time_{0, 0, RCL_ROS_TIME};
  bool has_input_data_{false};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  std::atomic_bool zero_all_requested_{false};
  std::atomic_bool returning_timeout_ready_{false};

  SimpleArmIK left_ik_{MakeLeftArmGeometry()};
  SimpleArmIK right_ik_{MakeRightArmGeometry()};

  // Ready Pose
  Eigen::VectorXd ready_pose_left_{7};
  Eigen::VectorXd ready_pose_right_{7};
  Eigen::Vector3d left_wrist_ready_pos_;
  Eigen::Vector3d right_wrist_ready_pos_;
  bool emergency_stop_{false};

  bool demo_mode_{false};
  bool require_safe_to_execute_{true};
  int udp_port_{9999};
  int udp_receive_buffer_{65535};
  int udp_sock_fd_{-1};
  std::atomic_bool stop_udp_{false};
  std::thread udp_thread_;
  double vr_timeout_sec_{0.5};
  double position_scale_{1.0};
  std::array<std::string, 3> coord_map_{"-z", "-x", "y"};
  double teleop_limit_x_{0.30};
  double teleop_limit_y_{0.30};
  double teleop_limit_z_{0.30};
  double ik_posture_gain_{0.03};
  double ik_iteration_step_limit_{0.05};
  double ik_max_solution_delta_{0.0};
  double hand_command_velocity_{0.3};
  double elbow_straight_target_{-0.2};
  double elbow_full_extension_delta_{0.30};
  double elbow_posture_gain_{0.08};
  bool debug_teleop_log_{true};
  int64_t debug_teleop_log_period_ms_{200};
  rclcpp::Time last_left_debug_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_debug_log_time_{0, 0, RCL_ROS_TIME};

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
  bool returning_zero_{false};
};

}  // namespace vr_teleop

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vr_teleop::VRTeleopNode>();
  RCLCPP_INFO(rclcpp::get_logger("vr_teleop"),
              "VR Teleop (UDP JSON v3, C++ IK, Ready-Pose) 节点启动，开始 spin...");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("vr_teleop"), "节点关闭。");
  rclcpp::shutdown();
  return 0;
}
