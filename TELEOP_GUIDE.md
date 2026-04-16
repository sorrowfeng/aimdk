# AgiBot X2 遥操与交互系统使用指南

本文档汇总了当前 AgiBot X2 机器人上已验证的手臂遥操、灵巧手控制、以及 AI 语音交互功能的使用方法。

---

## 一、前置条件

### 1.1 必须进入 Develop_MC 模式

在进行任何手臂/灵巧手遥操前，机器人必须处于 `Develop_MC` 开发者模式，否则 MC/HAL 层会冲突，导致控制命令被覆盖。

```bash
# 进入开发者模式
ros2 run py_examples migrate_system_state Develop_MC

# 测试完成后切回正常状态
ros2 run py_examples migrate_system_state Ready
```

成功进入后应看到输出：

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
| `hand_teleop_node` | `examples` | C++ | 独立的 VR 手柄→灵巧手映射节点 |
| `ai_voice_chat` | `py_examples` | Python | 百度在线 ASR + Claude AI 问答 + TTS 播报 |

---

## 三、VR 遥操（推荐 UDP 方案）

### 3.1 启动方式

```bash
# 终端 1：手臂 IK + Ruckig 平滑
ros2 run examples vr_teleop_node

# 终端 2：UDP JSON 桥接
ros2 run py_examples udp_vr_bridge
# 可自定义端口：--ros-args -p udp_port:=12345
```

> 使用 `udp_vr_bridge` 时**不需要**再运行 `hand_teleop_node`，因为 UDP 桥接节点已直接发布了手部命令。

### 3.2 UDP JSON 数据格式

VR 设备向机器人本机 `9999` 端口（默认）发送 UDP JSON：

```json
{
  "left": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "hand": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "key_one": false,
    "key_two": false
  },
  "right": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "hand": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "key_one": false,
    "key_two": false
  }
}
```

#### 字段说明

| 字段 | 类型 | 含义 |
|------|------|------|
| `position.x/y/z` | float | 手腕相对 **Ready Pose** 的空间偏移（米），用于手臂 IK |
| `rotation.roll` | float | 手腕 roll（弧度）→ `wrist_roll_joint` |
| `rotation.pitch` | float | 手腕 pitch（弧度）→ `wrist_pitch_joint` |
| `rotation.yaw` | float | 手腕 yaw（弧度）→ `wrist_yaw_joint` |
| `hand[0~5]` | float[6] | 雷赛灵巧手 6 关节目标角度，**直接下发** |
| `key_one/key_two` | bool | 双手同时为 true 触发 **急停** |

#### 灵巧手关节顺序

- `hand[0]`：拇指旋转 / 侧摆
- `hand[1]`：拇指弯曲
- `hand[2]`：食指弯曲
- `hand[3]`：中指弯曲
- `hand[4]`：无名指弯曲
- `hand[5]`：小指弯曲

### 3.3 VR 坐标系映射到机器人手臂坐标系

`udp_vr_bridge` 默认通过 `coord_map_x/y/z` 参数将 VR 手柄坐标映射到机器人坐标。当前 AgiBot X2 实测正确的默认映射如下：

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

> **说明**：映射参数值可加负号取反（如 `"-x"`）。如果不同 VR 设备或左右手坐标定义不同，可动态调整这三个参数。

### 3.4 手臂 IK 与平滑逻辑

`vr_teleop_node` 的内部流程：

1. **等待关节状态**：收到第一次 `JointStateArray` 后，记录当前位置为 `initial_q_`
2. **自动运动到 Ready Pose**：
   - 默认 `ready_pose_left/right = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]`
   - 表示肘关节弯曲约 90°，前臂保持水平平直
3. **相对位置遥操**：所有 `position` 偏移都叠加在 Ready Pose 的腕部位置上
4. **IK 解算**：
   - 前 4 个关节（shoulder_pitch/roll/yaw + elbow）用 **DLS 数值 IK** 求解腕部位置
   - 后 3 个关节（wrist_yaw/pitch/roll）直接从传入的欧拉角映射
5. **平滑输出**：`ruckig::Ruckig<14>` 在 100Hz 下做轨迹平滑，然后发布到 `/aima/hal/joint/arm/command`

### 3.5 Demo 键盘模式（无 VR 设备时测试）

```bash
ros2 run examples vr_teleop_node --ros-args -p demo_mode:=true
```

| 按键 | 功能 |
|------|------|
| `W/S` | 当前控制手 前后移动（X 轴），步进 5mm |
| `A/D` | 当前控制手 左右移动（Y 轴），步进 5mm |
| `R/F` | 当前控制手 上下移动（Z 轴），步进 5mm |
| `Space` | 重置双手偏移为 0（回到 Ready Pose） |
| `T` | 切换左手 / 右手控制 |
| `H` | 双臂平滑回到 **initial_q_**（启动前位置） |

---

## 四、灵巧手独立控制

### 4.1 通过 hand_teleop_node 遥操

如果有真实的 VR 手柄数据（`/tmp/vr_data`）：

```bash
ros2 run examples hand_teleop_node --ros-args -p demo_mode:=true
```

- `axis_x` → 拇指旋转
- `axis_y` → 拇指弯曲
- `index_trig` → 食指
- `hand_trig` → 中/无/小指

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

- **音频输入**：`/aima/hal/audio/capture`（通道 3，16kHz S16LE）
- **ASR**：百度短语音识别标准版 REST API（在线）
- **唤醒词**：包含 "小智" / "机器人" / "小白" 等才触发 AI
- **AI 问答**：Claude `claude-sonnet-4-6`（通过 Kimi Coding API 代理）
- **TTS 播报**：`PlayTts` 服务，播报期间自动屏蔽麦克风
- **本地意图**：识别到 "IP 是多少" 等关键词时，直接本地获取 IP 播报，**不走 AI**

### 5.3 依赖

```bash
pip install requests anthropic
```

---

## 六、Topic 一览

| Topic | 类型 | 发布者 | 说明 |
|-------|------|--------|------|
| `/udp_vr_bridge/arm_target` | `VRData` | `udp_vr_bridge` | 手臂目标位姿 + 急停按键 |
| `/aima/hal/joint/arm/command` | `JointCommandArray` | `vr_teleop_node` | 14-DOF 手臂平滑后关节命令 |
| `/aima/hal/joint/hand/command` | `HandCommandArray` | `udp_vr_bridge` / `hand_teleop_node` | 灵巧手 6 关节命令 |
| `/aima/hal/audio/capture` | `AudioCapture` | HAL | 麦克风原始音频 |

---

## 七、安全与注意事项

1. **必须处于 `Develop_MC` 模式**，否则手臂命令会被 MC 层覆盖
2. **急停逻辑**：双手同时 `key_one && key_two` 为 true 时，`vr_teleop_node` 触发急停并停止控制循环
3. `vr_teleop_node` 退出时**不会**自动回到初始位置，需要手动按 `H` 键（仅限 Demo 模式）
4. UDP JSON 中的 `hand` 数组长度须为 **6**，格式错误会丢弃不发布
