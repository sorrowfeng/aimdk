#!/usr/bin/env python3
"""
debug_mc_state.py
调试工具：订阅 /aima/mc/common/state，实时打印运动控制状态信息。

用法:
  ros2 run py_examples debug_mc_state              # 默认模式，状态变化时打印
  ros2 run py_examples debug_mc_state --all        # 打印所有消息
  ros2 run py_examples debug_mc_state --motion-only # 只打印有动作时的消息
"""

import argparse
import json
import sys
from datetime import datetime

import rclpy
from aimdk_msgs.msg import McCommonState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

def _msg_to_dict(msg, depth=0):
    """将 ROS 消息转换为可读的字典格式"""
    if depth > 5:
        return str(msg)

    result = {}
    try:
        # 获取消息的所有字段
        slots = msg.get_fields_and_field_types()
        for field_name in slots:
            try:
                value = getattr(msg, field_name)
                # 如果是嵌套消息，递归处理
                if hasattr(value, 'get_fields_and_field_types'):
                    result[field_name] = _msg_to_dict(value, depth + 1)
                else:
                    result[field_name] = value
            except Exception:
                result[field_name] = str(getattr(msg, field_name, ''))
    except Exception:
        return str(msg)
    return result


def _format_dict(d, indent=2):
    """格式化字典为易读的字符串"""
    lines = []
    for key, value in d.items():
        if isinstance(value, dict):
            lines.append(f"{' ' * indent}{key}:")
            lines.append(_format_dict(value, indent + 2))
        else:
            lines.append(f"{' ' * indent}{key}: {value}")
    return '\n'.join(lines)


# 动作 ID 到名称的映射（便于阅读）
MOTION_NAMES = {
    "1001": "举手",
    "1002": "挥手",
    "1003": "握手",
    "1004": "飞吻",
    "1005": "点赞",
    "1006": "比耶",
    "1007": "比心",
    "1008": "击掌",
    "1009": "平举",
    "1010": "平举",
    "1012": "转身",
    "1011": "胸前挥手",
    "1013": "敬礼",
    "3001": "鞠躬",
    "3007": "动感光波",
    "3008": "拥抱",
    "3009": "双手打叉",
    "3011": "加油",
    "3017": "鼓掌",
    "3024": "挠头",
    "3025": "抓屁股",
    "3031": "拜拜",
}

# 播放状态映射
PLAYER_STATES = {
    0: "IDLE",
    1: "LOADING",
    2: "PLAYING",
    3: "PAUSED",
    4: "STOPPING",
}

# 控制区域映射
CONTROL_AREAS = {
    1: "左手",
    2: "右手",
    3: "双手",
    11: "双手",
}


class DebugMcState(Node):
    def __init__(self, print_all=False, motion_only=False):
        super().__init__("debug_mc_state")

        self.print_all = print_all
        self.motion_only = motion_only
        self._last_state = None
        self._msg_count = 0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            McCommonState, "/aima/mc/common/state", self._on_state, qos
        )

        self.get_logger().info("debug_mc_state 已启动，监听 /aima/mc/common/state")
        if print_all:
            self.get_logger().info("模式: 打印所有消息")
        elif motion_only:
            self.get_logger().info("模式: 只打印有动作时的消息")
        else:
            self.get_logger().info("模式: 状态变化时打印")

    def _on_state(self, msg):
        self._msg_count += 1
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # 提取关键字段
        motion_id = msg.motion_status.motion
        player_state = msg.motion_status.player_state.value
        area = msg.motion_status.control_area.value

        # 获取可读名称
        motion_name = MOTION_NAMES.get(motion_id, "未知")
        state_name = PLAYER_STATES.get(player_state, f"未知({player_state})")
        area_name = CONTROL_AREAS.get(area, f"未知({area})")

        # 构建当前状态
        cur_state = (motion_id, player_state, area)

        # 根据模式决定是否打印
        should_print = False
        if self.print_all:
            should_print = True
        elif self.motion_only:
            should_print = player_state == 2 and motion_id != ""
        else:
            should_print = cur_state != self._last_state

        if should_print:
            self._last_state = cur_state

            # 格式化输出
            print(f"\n{'='*60}")
            print(f"[{now}] 消息 #{self._msg_count}")
            print(f"{'='*60}")
            print(f"动作 ID:     {motion_id:<10} ({motion_name})")
            print(f"播放状态:    {player_state:<10} ({state_name})")
            print(f"控制区域:    {area:<10} ({area_name})")
            print(f"{'─'*60}")
            print(f"原始消息:")
            msg_dict = _msg_to_dict(msg)
            print(_format_dict(msg_dict))
            print(f"{'='*60}")

            # 如果是新动作开始，特别提示
            if player_state == 2 and motion_id != "":
                print(f">>> 动作开始: {motion_name} (ID: {motion_id})")
                if area in {3, 11}:
                    print(f"    执行区域: 双手")
                elif area == 1:
                    print(f"    执行区域: 左手")
                elif area == 2:
                    print(f"    执行区域: 右手")

            # 如果动作结束，提示恢复
            elif player_state == 0 and self._last_state and self._last_state[1] == 2:
                print(f">>> 动作结束，恢复待机状态")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="调试工具：实时打印运动控制状态",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  ros2 run py_examples debug_mc_state              # 状态变化时打印
  ros2 run py_examples debug_mc_state --all        # 打印所有消息
  ros2 run py_examples debug_mc_state --motion-only # 只打印有动作时的消息

状态说明:
  播放状态 (player_state):
    0 = IDLE (空闲)
    1 = LOADING (加载中)
    2 = PLAYING (播放中)
    3 = PAUSED (暂停)
    4 = STOPPING (停止中)

  控制区域 (control_area):
    1 = 左手
    2 = 右手
    3 = 双手
    11 = 双手

  动作 ID (motion):
    1001-1013 = 单人动作
    3001-3031 = 双人/全身动作
        """,
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="打印所有接收到的消息（包括重复状态）",
    )
    parser.add_argument(
        "--motion-only",
        action="store_true",
        help="只打印有动作时的消息（player_state=2）",
    )

    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)
    node = DebugMcState(
        print_all=parsed_args.all,
        motion_only=parsed_args.motion_only,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n共接收 {node._msg_count} 条消息")
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
