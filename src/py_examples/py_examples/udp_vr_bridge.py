#!/usr/bin/env python3
"""
UDP VR 遥操桥接节点

接收 VR 设备通过 UDP 发送的 JSON 数据，解析后发布为 ROS 2 消息：
  - /udp_vr_bridge/arm_target       -> aimdk_msgs.msg.VRData (供 vr_teleop_node 使用)
  - /aima/hal/joint/hand/command    -> aimdk_msgs.msg.HandCommandArray

需要配合 vr_teleop_node 一起运行：
  1. ros2 run examples vr_teleop_node
  2. ros2 run py_examples udp_vr_bridge

JSON 格式示例:
{
  "hands": [
    {
      "hand": "left",
      "relative_position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
      "finger_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    {
      "hand": "right",
      ...
    }
  ]
}

说明:
  - relative_position: 末端相对空间位置偏移（直接发给手臂 IK）
  - orientation: 手腕欧拉角（pitch/yaw/roll，度，ZYX顺序），会转换为四元数
  - finger_joints: 雷赛灵巧手 6 关节目标角度，直接下发
"""

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header

from aimdk_msgs.msg import (
    HandCommand,
    HandCommandArray,
    HandType,
    MessageHeader,
    VRControllerState,
    VRData,
)


class UDPVRBridgeNode(Node):
    def __init__(self):
        super().__init__("udp_vr_bridge")

        self.declare_parameter("udp_port", 9999)
        self.declare_parameter("vr_data_topic", "/udp_vr_bridge/arm_target")
        self.declare_parameter("hand_command_topic", "/aima/hal/joint/hand/command")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("hand_command_velocity", 1.0)
        # 坐标重映射：解决 VR 手柄坐标系与机器人手臂坐标系不一致的问题
        # 默认值基于当前设备观察：手柄左右(x)->手臂前后，手柄上下(y)->手臂左右
        self.declare_parameter("coord_map_x", "-z")
        self.declare_parameter("coord_map_y", "-x")
        self.declare_parameter("coord_map_z", "y")

        udp_port = self.get_parameter("udp_port").value
        vr_topic = self.get_parameter("vr_data_topic").value
        hand_topic = self.get_parameter("hand_command_topic").value
        publish_rate = self.get_parameter("publish_rate").value
        self.hand_command_velocity = self.get_parameter("hand_command_velocity").value
        self.coord_map = {
            "x": self.get_parameter("coord_map_x").value,
            "y": self.get_parameter("coord_map_y").value,
            "z": self.get_parameter("coord_map_z").value,
        }

        self.vr_pub = self.create_publisher(VRData, vr_topic, 10)
        self.hand_pub = self.create_publisher(HandCommandArray, hand_topic, 10)

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("0.0.0.0", udp_port))
        self.udp_sock.settimeout(0.05)

        self.latest_data = None
        self.latest_data_seq = 0
        self.last_logged_seq = -1
        self.data_lock = threading.Lock()

        self.stop_udp = False
        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self._publish_loop)

        self.get_logger().info(
            f"UDP VR Bridge 已启动，监听端口 {udp_port} | "
            f"VRData->{vr_topic} | HandCommandArray->{hand_topic}"
        )

    def _udp_loop(self):
        while not self.stop_udp and rclpy.ok():
            try:
                data, addr = self.udp_sock.recvfrom(4096)
                json_data = json.loads(data.decode("utf-8"))
                with self.data_lock:
                    self.latest_data = json_data
                    self.latest_data_seq += 1
            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"UDP JSON 解析失败: {e}")
            except Exception as e:
                self.get_logger().error(f"UDP 接收异常: {e}")

    def _publish_loop(self):
        with self.data_lock:
            data = self.latest_data
            data_seq = self.latest_data_seq

        if data is None:
            return

        now = self.get_clock().now().to_msg()

        # --- VRData ---
        vr_msg = VRData()
        vr_msg.header = Header()
        vr_msg.header.stamp = now

        hands = data.get("hands", [])
        if not isinstance(hands, list) or len(hands) < 2:
            self.get_logger().warn(
                f"UDP JSON hands format invalid, expected 2 items: {data}"
            )
            return

        left_data = hands[0] if isinstance(hands[0], dict) else {}
        right_data = hands[1] if isinstance(hands[1], dict) else {}

        if left_data.get("hand", "").lower() not in ("", "left"):
            self.get_logger().warn(
                f"hands[0].hand should be left, got: {left_data.get('hand')}"
            )
        if right_data.get("hand", "").lower() not in ("", "right"):
            self.get_logger().warn(
                f"hands[1].hand should be right, got: {right_data.get('hand')}"
            )

        vr_msg.vr_controller_states.append(
            self._build_controller_state(left_data, "left")
        )
        vr_msg.vr_controller_states.append(
            self._build_controller_state(right_data, "right")
        )

        self.vr_pub.publish(vr_msg)

        # 调试日志：每秒打印一次原始坐标 vs 映射后坐标
        if data_seq != self.last_logged_seq and (left_data or right_data):
            self._log_coord_debug(left_data, right_data)
            self.last_logged_seq = data_seq

        # --- HandCommandArray ---
        hand_msg = HandCommandArray()
        hand_msg.header = MessageHeader()
        hand_msg.header.stamp = now
        hand_msg.left_hand_type = HandType()
        hand_msg.left_hand_type.value = 3
        hand_msg.right_hand_type = HandType()
        hand_msg.right_hand_type.value = 3

        if left_data:
            hand_msg.left_hands = self._build_hand_commands(left_data)
        if right_data:
            hand_msg.right_hands = self._build_hand_commands(right_data)

        self.hand_pub.publish(hand_msg)

    def _log_coord_debug(self, left_data: dict, right_data: dict):
        def fmt_raw(d: dict) -> str:
            p = d.get("relative_position", {})
            return f"({p.get('x',0):.3f},{p.get('y',0):.3f},{p.get('z',0):.3f})"

        def fmt_mapped(v: Vector3) -> str:
            return f"({v.x:.3f},{v.y:.3f},{v.z:.3f})"

        def fmt_orientation(d: dict) -> str:
            o = d.get("orientation", {})
            return (
                f"(pitch={float(o.get('pitch', 0.0)):.3f}, "
                f"yaw={float(o.get('yaw', 0.0)):.3f}, "
                f"roll={float(o.get('roll', 0.0)):.3f})"
            )

        def fmt_finger_joints(d: dict) -> str:
            finger_joints = d.get("finger_joints", [])
            if not isinstance(finger_joints, list):
                return "INVALID"
            return "[" + ", ".join(f"{float(v):.3f}" for v in finger_joints) + "]"

        def fmt_hand(side: str, d: dict) -> str:
            if not d:
                return f"{side}: missing"

            raw = fmt_raw(d)
            mapped = fmt_mapped(self._apply_coord_map(d.get("relative_position", {})))
            orientation = fmt_orientation(d)
            finger_joints = fmt_finger_joints(d)
            hand_name = d.get("hand", "")
            return (
                f"{side}: hand={hand_name or 'N/A'}"
                f" raw_pos={raw}"
                f" mapped_pos={mapped}"
                f" orientation={orientation}"
                f" finger_joints={finger_joints}"
            )

        log_parts = [
            fmt_hand("LEFT", left_data),
            fmt_hand("RIGHT", right_data),
        ]

        if log_parts:
            self.get_logger().info("UDP RX | " + " || ".join(log_parts))

    def _apply_coord_map(self, pos: dict) -> Vector3:
        """根据参数 coord_map_x/y/z 重映射 VR 坐标到机器人坐标"""
        raw = {
            "x": float(pos.get("x", 0.0)),
            "y": float(pos.get("y", 0.0)),
            "z": float(pos.get("z", 0.0)),
        }

        mapped = {}
        for axis in ("x", "y", "z"):
            spec = self.coord_map[axis]
            if spec.startswith("-"):
                src = spec[1:]
                sign = -1.0
            else:
                src = spec
                sign = 1.0
            mapped[axis] = sign * raw.get(src, 0.0)

        return Vector3(x=mapped["x"], y=mapped["y"], z=mapped["z"])

    def _build_controller_state(self, d: dict, side: str) -> VRControllerState:
        ctrl = VRControllerState()
        ctrl.name = side
        ctrl.position = self._apply_coord_map(d.get("relative_position", {}))
        # 暂不处理姿态，固定为单位四元数
        ctrl.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # 保持兼容性：axis/trig 默认 0
        ctrl.axis_x = 0.0
        ctrl.axis_y = 0.0
        ctrl.index_trig = 0.0
        ctrl.hand_trig = 0.0
        ctrl.key_one = False
        ctrl.key_two = False
        return ctrl

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        # ZYX 顺序：Rz(yaw) * Ry(pitch) * Rx(roll)，输入为度
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        return Quaternion(x=x, y=y, z=z, w=w)

    def _build_hand_commands(self, d: dict) -> list:
        hand_angles = d.get("finger_joints", [])
        if not isinstance(hand_angles, list) or len(hand_angles) != 6:
            self.get_logger().warn(f"finger_joints 数据格式异常: {hand_angles}")
            return []

        cmds = []
        for pos in hand_angles:
            cmd = HandCommand()
            cmd.name = ""
            cmd.position = float(pos)
            cmd.velocity = float(self.hand_command_velocity)
            cmd.acceleration = 0.0
            cmd.deceleration = 0.0
            cmd.effort = 0.0
            cmds.append(cmd)
        return cmds

    def destroy_node(self):
        self.stop_udp = True
        if self.udp_thread.is_alive():
            self.udp_thread.join(timeout=1.0)
        self.udp_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPVRBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
