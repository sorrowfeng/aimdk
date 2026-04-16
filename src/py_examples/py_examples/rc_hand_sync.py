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
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from aimdk_msgs.msg import McCommonState
from aimdk_msgs.msg import HandCommandArray, HandCommand, HandType, MessageHeader

# ─────────────────────────────────────────────────────────
# 手型预设  [拇指旋, 拇指弯, 食指, 中指, 无名, 小指]
# 0.0=张开/伸直  最大值=[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
# ─────────────────────────────────────────────────────────
PRESETS = {
    # 基础手型
    # 自然张开，待机状态           max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'neutral':      [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],  

    # 举手 / 挥手 / 拜拜 / 平举：掌心朝外，五指完全伸直
    # 拇指旋转微展，其余全开       max:[1.75, ----  ----  ----  ----  ----]
    'open_wave':    [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],  

    # 握手：四指微握做握手准备，拇指侧展
    # 各关节约 21% 最大值          max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'handshake':    [0.5, 0.3, 0.3, 0.3, 0.3, 0.3],  

    # 飞吻：拇指与食指指尖相触，其余三指自然展开
    # 拇指旋 46%，弯 36%，食指 36% max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'flying_kiss':  [0.8, 0.5, 0.5, 0.1, 0.1, 0.1],  

    # 比心（韩式）：拇指与食指形成心形，其余三指内扣
    # 拇指旋 11%，中无小 57%       max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'heart':        [0.2, 0.4, 0.4, 0.8, 0.8, 0.8],  

    # 敬礼：四指并拢伸直，拇指稍收
    # 拇指旋转 34%，其余全开       max:[1.75, ----  ----  ----  ----  ----]
    'salute':       [0.6, 0.0, 0.0, 0.0, 0.0, 0.0],  

    # 鼓掌 / 击掌：掌心张开迎接击打
    # 全部伸直张开                 max:[----  ----  ----  ----  ----  ----]
    'clap':         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  

    # 加油 / 打叉：握紧拳头
    # 各弯曲关节约 57% 最大值      max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'fist':         [0.5, 0.8, 0.8, 0.8, 0.8, 0.8],  

    # 拥抱：五指微张，手掌呈捧物状
    # 各弯曲关节约 14% 最大值      max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'hug':          [0.3, 0.2, 0.2, 0.2, 0.2, 0.2],  

    # 动感光波 / 胸前挥手：五指舒展，轻松姿态
    # 各弯曲关节约 7% 最大值       max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'light_wave':   [0.3, 0.1, 0.1, 0.1, 0.1, 0.1],  

    # 挠头 / 抓屁股：五指弯曲呈抓握状
    # 各弯曲关节约 36% 最大值      max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'scratch':      [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  

    # 鞠躬：手臂贴体，手掌自然放松微握
    # 各弯曲关节约 21% 最大值      max:[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]
    'bow':          [0.4, 0.3, 0.3, 0.3, 0.3, 0.3],  
}

# ─────────────────────────────────────────────────────────
# 预设动作 ID → 手型
# motion_status.motion 字段为字符串，如 "1002"
# ─────────────────────────────────────────────────────────
MOTION_GESTURE = {
    '1001': 'open_wave',    # 举手     — 五指伸直张开
    '1002': 'open_wave',    # 挥手     — 五指伸直张开
    '1003': 'handshake',    # 握手     — 四指微握准备握手
    '1004': 'flying_kiss',  # 飞吻     — 拇指食指相触
    '1007': 'heart',        # 比心     — 韩式比心
    '1008': 'clap',         # 击掌     — 掌心张开
    '1010': 'open_wave',    # 平举     — 五指伸直
    '1011': 'light_wave',   # 胸前挥手 — 手指舒展
    '1013': 'salute',       # 敬礼     — 四指并拢
    '3001': 'bow',          # 鞠躬     — 手掌放松贴体
    '3007': 'light_wave',   # 动感光波 — 手指舒展律动
    '3008': 'hug',          # 拥抱     — 双手微张捧物状
    '3009': 'fist',         # 双手打叉 — 握拳交叉
    '3011': 'fist',         # 加油     — 握拳上举
    '3017': 'clap',         # 鼓掌     — 掌心张开鼓掌
    '3024': 'scratch',      # 挠头     — 五指弯曲
    '3025': 'scratch',      # 抓屁股   — 五指弯曲
    '3031': 'open_wave',    # 拜拜     — 五指伸直挥动
}

# area: 1=左手, 2=右手, 3/11=双手
BOTH_AREAS = {3, 11}


def _build_hand_static(positions: list) -> list:
    """静态辅助函数：构建手部指令列表"""
    cmds = []
    for pos in positions:
        cmd = HandCommand()
        cmd.name = ''
        cmd.position = float(pos)
        cmd.velocity = 0.3
        cmd.acceleration = 0.0
        cmd.deceleration = 0.0
        cmd.effort = 0.0
        cmds.append(cmd)
    return cmds


class RcHandSync(Node):
    def __init__(self):
        super().__init__('rc_hand_sync')

        self.hand_pub = self.create_publisher(
            HandCommandArray, '/aima/hal/joint/hand/command', 10)

        self.target_left  = PRESETS['neutral']
        self.target_right = PRESETS['neutral']
        self._last_motion = ''
        self._last_log_state = None  # 上次打印的 (motion, player_state, area)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            McCommonState,
            '/aima/mc/common/state',
            self._on_state,
            qos
        )

        # 50 Hz 持续发布
        self.create_timer(0.02, self._publish_hand)

        self.get_logger().info('rc_hand_sync 已启动')

    def _on_state(self, msg):
        print('callback fired')  # 最原始验证，确认回调是否触发
        motion_id    = msg.motion_status.motion          # 字符串，如 "1002"
        player_state = msg.motion_status.player_state.value  # 0=IDLE 2=PLAYING
        area         = msg.motion_status.control_area.value  # 1/2/3/11

        # 状态变化才打印，避免刷屏
        cur_state = (motion_id, player_state, area)
        if cur_state != self._last_log_state:
            self._last_log_state = cur_state
            self.get_logger().info(
                f'[motion_status] motion={repr(motion_id)} player_state={player_state} area={area}'
            )

        if player_state == 2 and motion_id != self._last_motion:
            # 新动作开始
            self._last_motion = motion_id
            gesture = MOTION_GESTURE.get(motion_id, 'neutral')
            positions = PRESETS[gesture]

            if area in BOTH_AREAS:
                self.target_left  = positions
                self.target_right = positions
            elif area == 1:
                self.target_left  = positions
                self.target_right = PRESETS['neutral']
            elif area == 2:
                self.target_left  = PRESETS['neutral']
                self.target_right = positions

            self.get_logger().info(
                f'动作 {motion_id} area={area} → 手型: {gesture}')

        elif player_state == 0 and self._last_motion != '':
            # 动作结束，恢复自然
            self._last_motion = ''
            self.target_left  = PRESETS['neutral']
            self.target_right = PRESETS['neutral']
            self.get_logger().info('动作结束，恢复自然手型')

    def _publish_hand(self):
        msg = HandCommandArray()
        msg.header = MessageHeader()
        msg.left_hand_type  = HandType(value=3)
        msg.right_hand_type = HandType(value=3)
        msg.left_hands  = self._build_hand(self.target_left)
        msg.right_hands = self._build_hand(self.target_right)
        self.hand_pub.publish(msg)

    def _build_hand(self, positions: list) -> list:
        cmds = []
        for pos in positions:
            cmd = HandCommand()
            cmd.name         = ''
            cmd.position     = float(pos)
            cmd.velocity     = 0.3
            cmd.acceleration = 0.0
            cmd.deceleration = 0.0
            cmd.effort       = 0.0
            cmds.append(cmd)
        return cmds


def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='雷赛灵巧手控制节点',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
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
        '''
    )
    parser.add_argument(
        '-g', '--gesture',
        type=str,
        default=None,
        help='预设手势名称（如果不指定则进入监听模式）'
    )
    parser.add_argument(
        '-a', '--area',
        type=str,
        choices=['left', 'right', 'both'],
        default='both',
        help='控制区域：left=左手, right=右手, both=双手（默认：both）'
    )
    parser.add_argument(
        '-d', '--duration',
        type=float,
        default=2.0,
        help='单次模式持续时间，单位秒（默认：2.0）'
    )

    # 解析参数（过滤掉 ROS2 特定参数）
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    # 单次模式：直接发送手势指令
    if parsed_args.gesture:
        gesture_name = parsed_args.gesture
        if gesture_name not in PRESETS:
            print(f'错误: 未知的手势 "{gesture_name}"')
            print(f'可用预设: {list(PRESETS.keys())}')
            rclpy.shutdown()
            sys.exit(1)

        # 将字符串转换为 area 代码
        area_map = {'left': 1, 'right': 2, 'both': 3}
        area_code = area_map[parsed_args.area]

        positions = PRESETS[gesture_name]
        area_str = {'left': '左手', 'right': '右手', 'both': '双手'}[parsed_args.area]

        print(f'单次模式: 手势={gesture_name}, 区域={area_str}, 持续时间={parsed_args.duration}s')
        print(f'目标位置: {positions}')

        # 创建临时节点发布指令
        node = Node('rc_hand_sync_once')
        hand_pub = node.create_publisher(
            HandCommandArray, '/aima/hal/joint/hand/command', 10)

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
            msg.right_hands = _build_hand_static(PRESETS['neutral'])
        else:  # area_code == 2
            msg.left_hands = _build_hand_static(PRESETS['neutral'])
            msg.right_hands = _build_hand_static(positions)

        hand_pub.publish(msg)
        print(f'已发送指令，保持 {parsed_args.duration} 秒...')
        time.sleep(parsed_args.duration)

        # 恢复自然状态
        msg.left_hands = _build_hand_static(PRESETS['neutral'])
        msg.right_hands = _build_hand_static(PRESETS['neutral'])
        hand_pub.publish(msg)
        print('已恢复自然手型')

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


if __name__ == '__main__':
    main()
