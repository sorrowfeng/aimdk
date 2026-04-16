import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import HandCommandArray, HandCommand, HandType, MessageHeader

# 雷赛灵巧手每只手 6 个关节，驱动层按数组索引匹配
# 索引对应关系（根据实际硬件确认）:
#   0: 拇指旋转/侧摆
#   1: 拇指弯曲
#   2: 食指弯曲
#   3: 中指弯曲
#   4: 无名指弯曲
#   5: 小指弯曲
LEISAI_JOINT_COUNT = 6  # 每只手关节数
SWITCH_INTERVAL = 50    # 每隔多少次发布切换到下一关节（50Hz × 50 = 1秒）

# 每个关节的最大位置
# 索引: 0(拇指旋转), 1(拇指弯曲), 2(食指), 3(中指), 4(无名指), 5(小指)
JOINT_MAX = [1.75, 1.40, 1.40, 1.40, 1.40, 1.40]


class LeisaiHandControl(Node):
    def __init__(self):
        super().__init__('leisai_hand_control')

        # 逐根手指序列：0~5，左右手同步
        self.target_finger = 0
        self._seq_tick = 0

        self.publisher_ = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )
        self.timer_ = self.create_timer(0.02, self._publish)  # 50 Hz

        self.get_logger().info('Leisai hand control started (FINGER_SEQ mode)')

    def _build_cmd(self, position: float) -> HandCommand:
        cmd = HandCommand()
        cmd.name = ''
        cmd.position = float(position)
        cmd.velocity = 0.3
        cmd.acceleration = 0.0
        cmd.deceleration = 0.0
        cmd.effort = 0.0
        return cmd

    def _publish(self):
        # 所有关节默认 0.0，目标关节左右手同步弯曲到 1.0
        left_positions  = [0.0] * LEISAI_JOINT_COUNT
        right_positions = [0.0] * LEISAI_JOINT_COUNT
        left_positions[self.target_finger]  = JOINT_MAX[self.target_finger]
        right_positions[self.target_finger] = JOINT_MAX[self.target_finger]

        msg = HandCommandArray()
        msg.header = MessageHeader()
        msg.left_hand_type  = HandType(value=3)
        msg.right_hand_type = HandType(value=3)
        msg.left_hands  = [self._build_cmd(p) for p in left_positions]
        msg.right_hands = [self._build_cmd(p) for p in right_positions]
        self.publisher_.publish(msg)

        # 计时切换到下一关节
        self._seq_tick += 1
        if self._seq_tick >= SWITCH_INTERVAL:
            self._seq_tick = 0
            self.get_logger().info(f'[FINGER_SEQ] joint[{self.target_finger}]')
            self.target_finger = (self.target_finger + 1) % LEISAI_JOINT_COUNT


def main(args=None):
    rclpy.init(args=args)
    node = LeisaiHandControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
