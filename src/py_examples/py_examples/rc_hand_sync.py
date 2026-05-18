#!/usr/bin/env python3
"""
rc_hand_sync.py
订阅 /aima/mc/common/state，根据预设动作 ID + 控制区域联动雷赛灵巧手。

雷赛灵巧手关节索引（6个）:
  0: 拇指旋转/侧摆
  1: 拇指弯曲
  2: 食指弯曲
  3: 中指弯曲
  4: 无名指弯曲
  5: 小指弯曲
position: 0.0=张开, 1.0=握紧

控制区域 area:
  1  = 左手
  2  = 右手
  3  = 双手
  11 = 双手
"""

import argparse
import sys

import rclpy
from aimdk_msgs.msg import (
    HandCommand,
    HandCommandArray,
    HandType,
    McCommonState,
    MessageHeader,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ─────────────────────────────────────────────────────────
# 手型预设  [拇指旋, 拇指弯, 食指, 中指, 无名, 小指]
# 0.0=张开/伸直  最大值=[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
# ─────────────────────────────────────────────────────────
PRESETS = {
    # 基础手型
    # 自然张开，待机状态
    "neutral": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    # 举手 / 挥手 / 拜拜 / 平举：掌心朝外，五指完全伸直
    "open_wave": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    # 握手：四指微握做握手准备，拇指侧展
    "handshake": [1.05, 0.49, 0.154, 0.196, 0.126, 0.14],
    # 飞吻：拇指与食指指尖相触
    "flying_kiss": [0.35, 1.12, 0.0, 0.0, 1.40, 1.40],
    # 比心（韩式）：拇指与食指形成心形
    "heart": [0.875, 0.70, 0.56, 0.56, 0.56, 0.56],
    # 敬礼：四指并拢伸直，拇指稍收
    "salute": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    # 鼓掌 / 击掌：掌心张开迎接击打
    # 2000/10000*1.40=0.28
    "clap": [0.0, 0.28, 0.0, 0.0, 0.0, 0.0],
    # 加油 / 打叉：握紧拳头
    "fist": [1.40, 0.91, 1.40, 1.40, 1.40, 1.40],
    # 拥抱：五指微张，手掌呈捧物状
    "hug": [0.70, 0.35, 0.28, 0.28, 0.28, 0.28],
    # 动感光波 / 胸前挥手：五指舒展，轻松姿态
    "light_wave": [1.05, 0.70, 0.0, 0.0, 0.0, 0.0],
    # 挠头 / 抓屁股：五指弯曲呈抓握状
    "scratch": [0.70, 0.35, 0.28, 0.28, 0.28, 0.28],  # 与 hug 相同
    # 鞠躬：手臂贴体，手掌自然放松微握
    "bow": [0.35, 1.12, 0.0, 0.0, 0.0, 0.0],
    # 点赞：拇指竖起，四指握拳
    "thumbs_up": [0.0, 0.0, 1.40, 1.40, 1.40, 1.40],
    # 比耶：食指中指伸直呈 V 字，其他握拳
    "peace_sign": [1.40, 1.00, 0.0, 0.0, 1.40, 1.40],
}

# ─────────────────────────────────────────────────────────
# 预设动作 ID → 手型
# motion_status.motion 字段为字符串，如 "1002"
# ─────────────────────────────────────────────────────────
MOTION_GESTURE = {
    "1001": "open_wave",  # 举手     — 五指伸直张开
    "1002": "open_wave",  # 挥手     — 五指伸直张开
    "1003": "handshake",  # 握手     — 四指微握准备握手
    "1004": "flying_kiss",  # 飞吻     — 拇指食指相触
    "1005": "thumbs_up",  # 点赞     — 拇指竖起四指握拳
    "1006": "peace_sign",  # 比耶     — 食指中指伸直呈 V
    "1007": "heart",  # 比心     — 韩式比心
    "1008": "clap",  # 击掌     — 掌心张开
    "1009": "open_wave",  # 平举     — 五指伸直
    "1010": "open_wave",  # 平举     — 五指伸直
    "1012": "light_wave",  # 转身     — 手指舒展
    "1011": "light_wave",  # 胸前挥手 — 手指舒展
    "1013": "salute",  # 敬礼     — 四指并拢
    "3001": "bow",  # 鞠躬     — 手掌放松贴体
    "3007": "light_wave",  # 动感光波 — 手指舒展律动
    "3008": "hug",  # 拥抱     — 双手微张捧物状
    "3009": "fist",  # 双手打叉 — 握拳交叉
    "3011": "fist",  # 加油     — 握拳上举
    "3017": "clap",  # 鼓掌     — 掌心张开鼓掌
    "3024": "scratch",  # 挠头     — 五指弯曲
    "3025": "scratch",  # 抓屁股   — 五指弯曲
    "3031": "open_wave",  # 拜拜     — 五指伸直挥动
}

# area: 1=左手, 2=右手, 3/11=双手
BOTH_AREAS = {3, 11}


def _build_hand_static(positions: list) -> list:
    """静态辅助函数：构建手部指令列表"""
    cmds = []
    for pos in positions:
        cmd = HandCommand()
        cmd.name = ""
        cmd.position = float(pos)
        cmd.velocity = 0.3
        cmd.acceleration = 0.0
        cmd.deceleration = 0.0
        cmd.effort = 0.0
        cmds.append(cmd)
    return cmds


class RcHandSync(Node):
    def __init__(self):
        super().__init__("rc_hand_sync")

        self.hand_pub = self.create_publisher(
            HandCommandArray, "/aima/hal/joint/hand/command", 10
        )

        self.target_left = PRESETS["neutral"]
        self.target_right = PRESETS["neutral"]
        self._last_motion = ""
        self._last_log_state = None  # 上次打印的 (motion, player_state, area)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            McCommonState, "/aima/mc/common/state", self._on_state, qos
        )

        # 50 Hz 持续发布
        self.create_timer(0.02, self._publish_hand)

        self.get_logger().info("rc_hand_sync 已启动")

    def _on_state(self, msg):
        motion_id = msg.motion_status.motion  # 字符串，如 "1002"
        player_state = msg.motion_status.player_state.value  # 0=IDLE 2=PLAYING
        area = msg.motion_status.control_area.value  # 1/2/3/11

        # 状态变化才打印，避免刷屏
        cur_state = (motion_id, player_state, area)
        if cur_state != self._last_log_state:
            self._last_log_state = cur_state
            self.get_logger().info(
                f"[motion_status] motion={repr(motion_id)} player_state={player_state} area={area}"
            )

        if player_state == 2 and motion_id != self._last_motion:
            # 新动作开始
            self._last_motion = motion_id
            gesture = MOTION_GESTURE.get(motion_id, "neutral")
            positions = PRESETS[gesture]

            if area in BOTH_AREAS:
                self.target_left = positions
                self.target_right = positions
            elif area == 1:
                self.target_left = positions
                self.target_right = PRESETS["neutral"]
            elif area == 2:
                self.target_left = PRESETS["neutral"]
                self.target_right = positions

            self.get_logger().info(f"动作 {motion_id} area={area} → 手型: {gesture}")

        elif player_state == 0 and self._last_motion != "":
            # 动作结束，恢复自然
            self._last_motion = ""
            self.target_left = PRESETS["neutral"]
            self.target_right = PRESETS["neutral"]
            self.get_logger().info("动作结束，恢复自然手型")

    def _publish_hand(self):
        msg = HandCommandArray()
        msg.header = MessageHeader()
        msg.left_hand_type = HandType(value=3)
        msg.right_hand_type = HandType(value=3)
        msg.left_hands = self._build_hand(self.target_left)
        msg.right_hands = self._build_hand(self.target_right)
        self.hand_pub.publish(msg)

    def _build_hand(self, positions: list) -> list:
        cmds = []
        for pos in positions:
            cmd = HandCommand()
            cmd.name = ""
            cmd.position = float(pos)
            cmd.velocity = 0.3
            cmd.acceleration = 0.0
            cmd.deceleration = 0.0
            cmd.effort = 0.0
            cmds.append(cmd)
        return cmds


def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description="雷赛灵巧手控制节点",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  ros2 run py_examples rc_hand_sync              # 监听模式：订阅话题自动切换
  ros2 run py_examples rc_hand_sync --gesture fist       # 单次模式：双手握拳
  ros2 run py_examples rc_hand_sync -g open_wave -a left # 单次模式：左手张开

可用预设手势:
  neutral     - 自然张开（待机状态）
  open_wave   - 五指伸直张开（挥手/举手）
  handshake   - 握手准备姿势
  flying_kiss - 飞吻（拇指食指相触）
  heart       - 韩式比心
  salute      - 敬礼
  clap        - 鼓掌/击掌（掌心张开）
  fist        - 握拳
  hug         - 拥抱姿势
  light_wave  - 动感光波（手指舒展）
  scratch     - 挠头/抓握
  bow         - 鞠躬姿势
        """,
    )
    parser.add_argument(
        "-g",
        "--gesture",
        type=str,
        default=None,
        help="预设手势名称（如果不指定则进入监听模式）",
    )
    parser.add_argument(
        "-a",
        "--area",
        type=str,
        choices=["left", "right", "both"],
        default="both",
        help="控制区域：left=左手, right=右手, both=双手（默认：both）",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=2.0,
        help="单次模式持续时间，单位秒（默认：2.0）",
    )

    # 解析参数（过滤掉 ROS2 特定参数）
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    # 单次模式：直接发送手势指令
    if parsed_args.gesture:
        gesture_name = parsed_args.gesture
        if gesture_name not in PRESETS:
            print(f'错误: 未知的手势 "{gesture_name}"')
            print(f"可用预设: {list(PRESETS.keys())}")
            rclpy.shutdown()
            sys.exit(1)

        # 将字符串转换为 area 代码
        area_map = {"left": 1, "right": 2, "both": 3}
        area_code = area_map[parsed_args.area]

        positions = PRESETS[gesture_name]
        area_str = {"left": "左手", "right": "右手", "both": "双手"}[parsed_args.area]

        print(
            f"单次模式: 手势={gesture_name}, 区域={area_str}, 持续时间={parsed_args.duration}s"
        )
        print(f"目标位置: {positions}")

        # 创建临时节点发布指令
        node = Node("rc_hand_sync_once")
        hand_pub = node.create_publisher(
            HandCommandArray, "/aima/hal/joint/hand/command", 10
        )

        # 等待发布器就绪
        import time

        time.sleep(0.5)

        # 构建并发布消息
        msg = HandCommandArray()
        msg.header = MessageHeader()
        msg.left_hand_type = HandType(value=3)
        msg.right_hand_type = HandType(value=3)

        # 根据 area 设置目标位置
        if area_code in BOTH_AREAS:
            msg.left_hands = _build_hand_static(positions)
            msg.right_hands = _build_hand_static(positions)
        elif area_code == 1:
            msg.left_hands = _build_hand_static(positions)
            msg.right_hands = _build_hand_static(PRESETS["neutral"])
        else:  # area_code == 2
            msg.left_hands = _build_hand_static(PRESETS["neutral"])
            msg.right_hands = _build_hand_static(positions)

        hand_pub.publish(msg)
        print(f"已发送指令，保持 {parsed_args.duration} 秒...")
        time.sleep(parsed_args.duration)

        # 恢复自然状态
        msg.left_hands = _build_hand_static(PRESETS["neutral"])
        msg.right_hands = _build_hand_static(PRESETS["neutral"])
        hand_pub.publish(msg)
        print("已恢复自然手型")

        node.destroy_node()
        rclpy.shutdown()
        return

    # 监听模式：原有行为
    node = RcHandSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
