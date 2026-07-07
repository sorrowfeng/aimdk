# AgiBot X2 遥操与交互系统使用指南

本文档汇总当前 AgiBot X2 机器人上已经验证可用的手臂遥操、灵巧手控制，以及 AI 语音交互功能。

---

## 一、前置条件

### 1.1 遥操启动流程

进入开发者模式后启动 `vr_teleop_node`。该节点会直接监听 UDP JSON v3，同时发布手臂和灵巧手命令。

```bash
# 进入工作目录
cd aimdk

# 应用环境变量
source install/setup.bash

# 进入开发模式
ros2 run py_examples migrate_system_state Develop_MC

# 启动手臂控制程序
ros2 run examples vr_teleop_node
```

### 1.2 必须进入 `Develop_MC` 模式

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

### 1.3 编译

```bash
cd ~/aimdk
source /opt/ros/humble/setup.bash

# C++ examples（vr_teleop_node）
colcon build --packages-select examples --cmake-args -DCMAKE_BUILD_TYPE=Release

# Python py_examples（语音、工具节点）
colcon build --packages-select py_examples

source install/setup.bash
```

---

## 二、节点总览

| 节点 | 包 | 语言 | 功能 |
|------|-----|------|------|
| `vr_teleop_node` | `examples` | C++ | UDP JSON v3 接收 + 手臂 IK + 手腕姿态映射 + 灵巧手命令 |
| `ai_voice_chat` | `py_examples` | Python | 百度 ASR + AI 问答 + TTS 播报 |

---

## 三、VR 遥操（推荐 UDP 方案）

### 3.1 启动方式

```bash
# 默认监听 UDP 9999
ros2 run examples vr_teleop_node

# 可自定义端口
ros2 run examples vr_teleop_node --ros-args -p udp_port:=12345
```

不再需要运行 Python UDP bridge，也不再通过 `VRData` 中间 topic 转发。

### 3.2 UDP JSON 数据格式

VR 设备向机器人本机 `9999` 端口（默认）发送 PICO UDP JSON v3。机器人端只解析 `packet_version=3` 的包，并以同一包内的 `safety.safe_to_execute == true` 作为运动许可。

```json
{
  "packet_version": 3,
  "operator_mode": "active_stream",
  "safety": {
    "safe_to_execute": true
  },
  "controllers": {
    "left": {
      "quality": "live",
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
      },
      "input": {
        "thumbstick": {"x": 0.0, "y": 0.0},
        "trigger": 0.0,
        "grip": 0.0,
        "primary_pressed": false,
        "secondary_pressed": false
      }
    },
    "right": {
      "quality": "live",
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
      },
      "input": {
        "thumbstick": {"x": 0.0, "y": 0.0},
        "trigger": 0.0,
        "grip": 0.0,
        "primary_pressed": false,
        "secondary_pressed": false
      }
    }
  },
  "robot_control": {
    "hands": {
      "unit": "raw",
      "left": [8000, 0, 0, 0, 0, 0],
      "right": [8000, 0, 0, 0, 0, 0]
    }
  }
}
```

#### 字段说明

| 字段 | 类型 | 含义 |
|------|------|------|
| `packet_version` | number | 必须为 `3` |
| `operator_mode` | string | `stop_signal` 时停止消费位姿；其它状态仍以 `safe_to_execute` 为准 |
| `safety.safe_to_execute` | bool | 唯一运动执行许可 |
| `controllers.left/right.quality` | string | 只有 `live` 或空值时消费该侧 pose |
| `controllers.left/right.pose.position.x/y/z` | number | 控制器相对位移，叠加到 Ready Pose 腕部控制点 |
| `controllers.left/right.pose.orientation.pitch/yaw/roll` | number | 控制器欧拉角，单位 deg；机器人侧 `roll -> wrist_yaw`、`pitch -> wrist_pitch`、`yaw -> wrist_roll` |
| `controllers.left/right.input.primary_pressed/secondary_pressed` | bool | 双手两个按键同时按下触发急停 |
| `robot_control.hands.left/right` | number[6] | 灵巧手 raw 命令，范围 `0..10000` |

#### 姿态与安全

- `pose.position` 已按发送端校准输出，接收端不再做首帧归零。
- `pose.orientation` 按 UDP JSON v3 协议解析；接收端只做 `deg -> rad`，再交换 `yaw/roll` 输出到 `wrist_yaw / wrist_pitch / wrist_roll`。
- `safety.safe_to_execute=false`、`operator_mode="stop_signal"`、JSON 解析失败或 UDP 超时都会让手臂回到 Ready Pose 或保持安全姿态。

#### 左右手索引约定

接收端固定从 `controllers.left` 映射左臂，从 `controllers.right` 映射右臂；不再依赖数组索引或 `VRData.msg`。

#### 灵巧手关节顺序

`robot_control.hands.left/right` 是 `0..10000` raw 值，接收端会按雷赛手最大关节量换算为 HAL 位置命令：

- `hands[0]`：拇指旋转 / 侧摆，最大 `1.75`
- `hands[1]`：拇指弯曲，最大 `1.40`
- `hands[2]`：食指弯曲，最大 `1.40`
- `hands[3]`：中指弯曲，最大 `1.40`
- `hands[4]`：无名指弯曲，最大 `1.40`
- `hands[5]`：小指弯曲，最大 `1.40`

### 3.3 VR 坐标系映射到机器人手臂坐标系

`vr_teleop_node` 默认通过 `coord_map_x/y/z` 参数，把控制器坐标映射到机器人手臂坐标。当前 AgiBot X2 实测默认值如下：

```bash
ros2 run examples vr_teleop_node \
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
3. 将 `controllers.left/right.pose.position` 偏移叠加到 `Ready Pose` 腕部控制点
4. 对前 4 个关节做阻尼最小二乘位置 IK
   - `shoulder_pitch`
   - `shoulder_roll`
   - `shoulder_yaw`
   - `elbow`
5. 手腕 3 个关节直接消费控制器欧拉角，并按关节限位裁剪
6. 用 `ruckig::Ruckig<14>` 在 100Hz 下做轨迹平滑
7. 发布到 `/aima/hal/joint/arm/command`

说明：

- 左右臂 IK 现在分别使用各自的关节限位。
- 肘关节带有姿态偏置项，手伸远时逐步趋向更伸直的肘部目标，减少肘部折叠。
- 手腕姿态不参与肩肘位置 IK，避免大角度手腕旋转反向污染手臂位置解。

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

## 四、灵巧手控制

### 4.1 通过 UDP JSON v3 遥操

`vr_teleop_node` 会直接解析 `robot_control.hands.left/right`，换算为雷赛手 6 关节位置后发布到 `/aima/hal/joint/hand/command`。

原始 raw 值范围为 `0..10000`。接收端按 `[1.75, 1.40, 1.40, 1.40, 1.40, 1.40]` 的最大关节量转换。

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
| `/aima/hal/joint/arm/command` | `JointCommandArray` | `vr_teleop_node` | 14-DOF 手臂平滑后关节命令 |
| `/aima/hal/joint/hand/command` | `HandCommandArray` | `vr_teleop_node` | 灵巧手 6 关节命令 |
| `/aima/hal/audio/capture` | `AudioCapture` | HAL | 麦克风原始音频 |

---

## 七、安全与注意事项

1. 必须处于 `Develop_MC` 模式，否则手臂命令会被 MC 层覆盖。
2. `vr_teleop_node` 退出时不会自动回到初始位置；在 Demo 模式下可用 `H` 手动回位。
3. UDP JSON 必须为 `packet_version=3`，旧版 `hands[]` 协议不再支持。
4. 机器人端只以 `safety.safe_to_execute` 作为执行许可；安全字段为 false 时不消费控制器位姿。
5. `robot_control.hands.left/right` 数组长度必须为 `6`，格式错误时该包不会发布对应手部命令。
6. 双手 `primary_pressed && secondary_pressed` 同时为 true 时触发急停。
