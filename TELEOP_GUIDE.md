# AgiBot X2 遥操与交互系统使用指南

本文档汇总当前 AgiBot X2 机器人上已经验证可用的手臂遥操、灵巧手控制，以及 AI 语音交互功能。

---

## 一、前置条件

### 1.1 必须进入 `Develop_MC` 模式

在进行任何手臂或灵巧手遥操前，机器人必须处于 `Develop_MC` 模式，否则 MC/HAL 层会冲突，导致控制命令被覆盖。

```bash
# 进入开发模式
ros2 run py_examples migrate_system_state Develop_MC

# 测试完成后切回正常状态
ros2 run py_examples migrate_system_state Ready
```

成功进入后应看到类似输出：

```text
State="Develop_MC"
Migration to Develop_MC completed successfully!
```

### 1.2 编译

```bash
cd ~/aimdk
source /opt/ros/humble/setup.bash

# C++ examples（vr_teleop_node、hand_teleop_node）
colcon build --packages-select examples --cmake-args -DCMAKE_BUILD_TYPE=Release

# Python py_examples（udp_vr_bridge、ai_voice_chat）
colcon build --packages-select py_examples

source install/setup.bash
```

---

## 二、节点总览

| 节点 | 包 | 语言 | 功能 |
|------|-----|------|------|
| `vr_teleop_node` | `examples` | C++ | 手臂 IK + Ruckig 平滑，发布 `JointCommandArray` |
| `udp_vr_bridge` | `py_examples` | Python | UDP 接收 JSON，发布 `VRData` + `HandCommandArray` |
| `hand_teleop_node` | `examples` | C++ | 独立的 VR 手柄到灵巧手映射节点 |
| `ai_voice_chat` | `py_examples` | Python | 百度 ASR + AI 问答 + TTS 播报 |

---

## 三、VR 遥操（推荐 UDP 方案）

### 3.1 启动方式

```bash
# 终端 1：手臂 IK + Ruckig 平滑
ros2 run examples vr_teleop_node

# 终端 2：UDP JSON 桥接
ros2 run py_examples udp_vr_bridge

# 可自定义端口
ros2 run py_examples udp_vr_bridge --ros-args -p udp_port:=12345
```

> 使用 `udp_vr_bridge` 时，不需要再运行 `hand_teleop_node`，因为 UDP 桥接节点已经直接发布手部命令。

### 3.2 UDP JSON 数据格式

VR 设备向机器人本机 `9999` 端口（默认）发送 UDP JSON。当前正式协议使用 `hands[]` 数组，而不是旧版顶层 `left/right` 结构。

```json
{
  "hands": [
    {
      "hand": "left",
      "relative_position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
      "finger_joints": [1.4, 0.7, 0.7, 0.7, 0.7, 0.7]
    },
    {
      "hand": "right",
      "relative_position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
      "finger_joints": [1.4, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
  ]
}
```

#### 字段说明

| 字段 | 类型 | 含义 |
|------|------|------|
| `hands[].hand` | string | 手标识，取值为 `left` 或 `right` |
| `hands[].relative_position.x/y/z` | float | 相对 **Ready Pose** 的末端空间偏移，用于手臂 IK |
| `hands[].orientation.roll/pitch/yaw` | float | 保留的姿态字段 |
| `hands[].finger_joints[0~5]` | float[6] | 灵巧手 6 关节目标角度，直接下发 |

#### 当前关于姿态字段的说明

- `orientation` 目前是保留字段，协议层仍然接收。
- 当前版本的 `udp_vr_bridge` 暂时将姿态固定为单位四元数发布。
- 这意味着手腕 `wrist_yaw / wrist_pitch / wrist_roll` 目前不会跟随 UDP 的 `orientation` 输入。

#### 左右手索引约定

`VRData.msg` 约定：

- `vr_controller_states[0]` 固定表示左手
- `vr_controller_states[1]` 固定表示右手

当前 `udp_vr_bridge` 已按这个顺序稳定发布。即使某一只手当前没有有效数据，也会保留其索引位置，避免左右手错位。

#### 灵巧手关节顺序

- `finger_joints[0]`：拇指旋转 / 侧摆
- `finger_joints[1]`：拇指弯曲
- `finger_joints[2]`：食指弯曲
- `finger_joints[3]`：中指弯曲
- `finger_joints[4]`：无名指弯曲
- `finger_joints[5]`：小指弯曲

### 3.3 VR 坐标系映射到机器人手臂坐标系

`udp_vr_bridge` 默认通过 `coord_map_x/y/z` 参数，把 VR 手柄坐标映射到机器人坐标。当前 AgiBot X2 实测默认值如下：

```bash
ros2 run py_examples udp_vr_bridge \
  --ros-args \
  -p coord_map_x:="-z" \
  -p coord_map_y:="-x" \
  -p coord_map_z:="y"
```

#### 映射结果

| 手柄动作 | VR 原始轴 | 机器人手臂方向 | 映射参数 |
|----------|-----------|----------------|----------|
| 左右移动 | `x` | 机器人 `y`（左右） | `coord_map_y = "-x"` |
| 上下移动 | `y` | 机器人 `z`（上下） | `coord_map_z = "y"` |
| 前后移动 | `z` | 机器人 `x`（前后） | `coord_map_x = "-z"` |

> 映射参数可以加负号取反，例如 `"-x"`。如果 VR 设备或左右手坐标定义不同，可动态调整这三个参数。

### 3.4 手臂 IK 与平滑逻辑

`vr_teleop_node` 的内部流程如下：

1. 等待第一帧 `JointStateArray`，记录当前状态到 `initial_q_`
2. 自动进入 `Ready Pose`
   - 默认 `ready_pose_left/right = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]`
   - 表示肘关节弯曲约 90 度
3. 将 UDP 的 `relative_position` 偏移叠加到 `Ready Pose` 末端位置
4. 对前 4 个关节做位置 IK
   - `shoulder_pitch`
   - `shoulder_roll`
   - `shoulder_yaw`
   - `elbow`
5. 用 `ruckig::Ruckig<14>` 在 100Hz 下做轨迹平滑
6. 发布到 `/aima/hal/joint/arm/command`

说明：

- 左右臂 IK 现在分别使用各自的关节限位。
- 手腕 3 个关节的接口仍保留，但当前 UDP bridge 暂未启用姿态跟随。

### 3.5 Demo 键盘模式（无 VR 设备时调试）

```bash
ros2 run examples vr_teleop_node --ros-args -p demo_mode:=true
```

| 按键 | 功能 |
|------|------|
| `W/S` | 当前控制手前后移动（X 轴），步进 5mm |
| `A/D` | 当前控制手左右移动（Y 轴），步进 5mm |
| `R/F` | 当前控制手上下移动（Z 轴），步进 5mm |
| `Space` | 清零双手偏移，回到 Ready Pose |
| `T` | 切换左手 / 右手控制 |
| `H` | 双臂平滑回到 `initial_q_` |

---

## 四、灵巧手独立控制

### 4.1 通过 `hand_teleop_node` 遥操

如果有真实 VR 手柄数据：

```bash
ros2 run examples hand_teleop_node --ros-args -p demo_mode:=true
```

- `axis_x` -> 拇指旋转
- `axis_y` -> 拇指弯曲
- `index_trig` -> 食指
- `hand_trig` -> 中、无名、小指

### 4.2 通过命令行直接发布

```bash
ros2 topic pub /aima/hal/joint/hand/command aimdk_msgs/msg/HandCommandArray "{
  left_hand_type: {value: 3},
  right_hand_type: {value: 3},
  left_hands: [
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0}
  ],
  right_hands: [
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0},
    {name: '', position: 0.0, velocity: 0.1, acceleration: 0.0, deceleration: 0.0, effort: 0.0}
  ]
}"
```

> `HandType = 3` 表示雷赛灵巧手。

---

## 五、AI 语音问答

### 5.1 运行

```bash
ros2 run py_examples ai_voice_chat
```

### 5.2 功能说明

- 音频输入：`/aima/hal/audio/capture`（通道 3，16kHz S16LE）
- ASR：百度在线语音识别 REST API
- 唤醒词：包含“小智”“机器人”“小白”等关键词时才触发
- AI 问答：通过 Kimi API 代理调用模型
- TTS 播报：调用 `PlayTts` 服务
- 本地意图：例如“IP 是多少”可直接本地查询，不走 AI

### 5.3 依赖

```bash
pip install requests anthropic
```

---

## 六、Topic 一览

| Topic | 类型 | 发布者 | 说明 |
|-------|------|--------|------|
| `/udp_vr_bridge/arm_target` | `VRData` | `udp_vr_bridge` | 手臂目标位姿 |
| `/aima/hal/joint/arm/command` | `JointCommandArray` | `vr_teleop_node` | 14-DOF 手臂平滑后关节命令 |
| `/aima/hal/joint/hand/command` | `HandCommandArray` | `udp_vr_bridge` / `hand_teleop_node` | 灵巧手 6 关节命令 |
| `/aima/hal/audio/capture` | `AudioCapture` | HAL | 麦克风原始音频 |

---

## 七、安全与注意事项

1. 必须处于 `Develop_MC` 模式，否则手臂命令会被 MC 层覆盖。
2. `vr_teleop_node` 退出时不会自动回到初始位置；在 Demo 模式下可用 `H` 手动回位。
3. UDP JSON 中 `finger_joints` 数组长度必须为 `6`，格式错误时该只手不会发布灵巧手命令。
4. 发送端协议当前固定为 `hands[0]=left`、`hands[1]=right`，接收端也按这个顺序解析。
5. 当前 UDP 协议不包含急停按键字段，如需急停需要走其他控制链路。
6. `orientation` 当前属于保留字段，协议可以继续发送，但当前版本不会实际驱动手腕姿态。
