# AgiBot AIMDK - CLAUDE.md

## 项目概述

`aimdk` 是 AgiBot X2 系列机器人的 ROS 2 软件开发包，包含消息定义、示例节点和遥操控制程序。

**支持的机器人**：AgiBot X2（pm01 / se01 / sa01 等）

## 目录结构

```
aimdk/
├── src/
│   ├── aimdk_msgs/          # ROS 2 消息/服务定义（156+ 接口文件）
│   ├── examples/             # C++ 示例和遥操节点
│   │   ├── src/vr_teleop/
│   │   │   ├── vr_teleop_node.cpp    # 主手臂 IK + Ruckig 平滑节点
│   │   │   ├── hand_teleop_node.cpp  # 灵巧手遥操节点
│   │   │   └── parse_x2_urdf.py      # URDF 运动学参数提取脚本
│   │   ├── src/hal/                  # HAL 示例（摄像头、传感器等）
│   │   ├── src/mc/                   # 运动控制示例
│   │   ├── src/interaction/          # 交互示例（TTS、表情）
│   │   └── CMakeLists.txt
│   └── py_examples/          # Python 示例和工具节点
│       ├── py_examples/
│       │   ├── udp_vr_bridge.py      # UDP JSON -> VRData + HandCommandArray
│       │   ├── ai_voice_chat.py      # 百度 ASR + Claude AI 问答
│       │   ├── voice_command_ip.py   # 语音指令查询 IP
│       │   ├── migrate_system_state.py
│       │   └── ...
│       ├── setup.py
│       └── package.xml
├── TELEOP_GUIDE.md           # 遥操系统完整使用说明
└── ...
```

## 技术栈

- **ROS 2 Humble**：`rclcpp` / `rclpy`，`ament_cmake` / `ament_python`
- **C++17**：`examples` 包，主要遥操逻辑
- **Python 3.10+**：`py_examples` 包，桥接和交互逻辑
- **第三方库**：
  - `Eigen3`：手臂 IK 矩阵运算
  - `ruckig`：14-DOF 关节轨迹平滑（100Hz）
  - `OpenCV` / `cv_bridge`：摄像头示例
  - `requests`：`ai_voice_chat` 百度 ASR 调用
  - `anthropic`：`ai_voice_chat` AI 问答调用

## 构建命令

```bash
cd /home/agi/aimdk
source /opt/ros/humble/setup.bash

# 构建 C++ examples（vr_teleop_node、hand_teleop_node）
colcon build --packages-select examples --cmake-args -DCMAKE_BUILD_TYPE=Release

# 构建 Python py_examples（udp_vr_bridge、ai_voice_chat）
colcon build --packages-select py_examples

# 同时构建两个包
colcon build --packages-select examples py_examples

source install/setup.bash
```

## 远程部署

### 机器人 SSH 信息

- **IP**: `172.16.20.110`
- **用户名**: `agi`
- **密码**: `1`
- **代码路径**: `/home/agi/aimdk`

### 快速部署脚本（Windows/Git Bash）

在本地修改 `src/py_examples/py_examples/udp_vr_bridge.py` 后，可自动复制到机器人并编译：

```bash
# 创建 askpass 脚本（只需执行一次）
printf '#!/bin/bash\necho 1\n' > /tmp/askpass.sh
chmod +x /tmp/askpass.sh

# 复制文件
DISPLAY=dummy:0 SSH_ASKPASS=/tmp/askpass.sh scp \
  -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
  src/py_examples/py_examples/udp_vr_bridge.py \
  agi@172.16.20.110:/home/agi/aimdk/src/py_examples/py_examples/udp_vr_bridge.py \
  </dev/null

# 远程编译
DISPLAY=dummy:0 SSH_ASKPASS=/tmp/askpass.sh ssh \
  -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
  agi@172.16.20.110 \
  "cd /home/agi/aimdk && source /opt/ros/humble/setup.bash && colcon build --packages-select py_examples" \
  </dev/null
```

## 核心运行模式

### 手臂/灵巧手遥操前必须做的事

```bash
ros2 run py_examples migrate_system_state Develop_MC
```

> **绝对必要**：机器人必须处于 `Develop_MC` 模式，否则 MC/HAL 层会冲突，关节命令会被覆盖。测试完成后切回 `Ready`：

```bash
ros2 run py_examples migrate_system_state Ready
```

## 当前活跃节点

### 1. UDP VR 遥操链路（推荐）

```bash
# 终端 1：手臂 IK + Ruckig 平滑
ros2 run examples vr_teleop_node

# 终端 2：UDP JSON 桥接（默认端口 9999）
ros2 run py_examples udp_vr_bridge
```

- `udp_vr_bridge` 接收 JSON，发布到 `/udp_vr_bridge/arm_target`（`VRData`）和 `/aima/hal/joint/hand/command`（`HandCommandArray`）
- `vr_teleop_node` 订阅 `VRData`，解算 IK 后发布平滑的 14-DOF 手臂命令到 `/aima/hal/joint/arm/command`
- **不需要**运行 `hand_teleop_node`

JSON 格式详见 `TELEOP_GUIDE.md`。

### 2. AI 语音问答

```bash
ros2 run py_examples ai_voice_chat
```

- 音频源：`/aima/hal/audio/capture`（通道 3，16kHz S16LE）
- 百度在线 ASR → 唤醒词过滤 → Claude AI（`claude-sonnet-4-6` via Kimi API）→ TTS 播报
- 额外依赖：`pip install requests anthropic`

### 3. Demo 键盘模式（无 VR 设备调试）

```bash
ros2 run examples vr_teleop_node --ros-args -p demo_mode:=true
```

- `W/S/A/D/R/F`：控制当前手（左/右）的空间偏移，步进 5mm
- `T`：切换左右手
- `Space`：偏移清零（回到 Ready Pose）
- `H`：双臂平滑回到启动前的 `initial_q_`

## 关键 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/udp_vr_bridge/arm_target` | `aimdk_msgs/VRData` | 手臂目标位姿（由 udp_vr_bridge 发布） |
| `/aima/hal/joint/arm/command` | `aimdk_msgs/JointCommandArray` | 14-DOF 手臂命令（vr_teleop_node） |
| `/aima/hal/joint/hand/command` | `aimdk_msgs/HandCommandArray` | 灵巧手 6 关节命令 |
| `/aima/hal/audio/capture` | `aimdk_msgs/AudioCapture` | 麦克风原始音频 |

## 架构模式

- **手臂控制**：DLS 数值 IK（位置）+ 欧拉角直接映射（姿态）+ `ruckig` 平滑
  - `vr_teleop_node.cpp:SimpleArmIK` 只解算前 4 个 DOF（shoulder_pitch/roll/yaw + elbow）
  - wrist_yaw/pitch/roll 直接从 `orientation` 四元数提取
- **Ready Pose 策略**：启动后自动从当前状态平滑抬升到 elbow=-1.57 rad，后续所有 UDP `position` 都是相对此 Ready Pose 的偏移
- **急停**：双手 `key_one && key_two` 同时为 true 时，`vr_teleop_node` 触发急停并停止控制循环
- **灵巧手**：`HandType=3` 表示雷赛手，6 关节顺序固定（拇指 rot→bend，食指→中指→无名指→小指）

## 重要文件路径

- 手臂 IK 节点：`src/examples/src/vr_teleop/vr_teleop_node.cpp`
- UDP 桥接节点：`src/py_examples/py_examples/udp_vr_bridge.py`
- AI 语音节点：`src/py_examples/py_examples/ai_voice_chat.py`
- 使用说明：`TELEOP_GUIDE.md`

## 测试与调试

### UDP 链路测试脚本

项目提供了独立的 UDP 测试工具，可在任意能连通机器人的电脑上运行：

```bash
# 1. 发送固定 JSON 数据（50Hz，持续 5 秒）
python3 scripts/test_udp_vr.py -f scripts/test_vr_data.json --rate 50 --duration 5

# 2. 生成动态波浪数据（手指正弦开合，便于观察平滑效果）
python3 scripts/test_udp_vr.py --wave --rate 50 --duration 10

# 3. 单次发送
python3 scripts/test_udp_vr.py -f scripts/test_vr_data.json --once
```

脚本文件：
- `scripts/test_udp_vr.py` — 测试发送端
- `scripts/test_vr_data.json` — 示例 JSON 数据

### 验证 Topic

```bash
# 查看手臂命令频率
ros2 topic hz /aima/hal/joint/arm/command

# 查看手部命令内容
ros2 topic echo /aima/hal/joint/hand/command

# 查看 VR 数据接收频率
ros2 topic hz /udp_vr_bridge/arm_target
```

## 给 Agent 的提示

- 修改 C++ 节点后必须重新 `colcon build --packages-select examples`
- 修改 Python 节点后必须重新 `colcon build --packages-select py_examples`（setup.py 变更才需要，`*.py` 源码修改通常无需重编译，但保险起见同步后都 build 一次）
- 任何手臂/手部的控制代码，都要检查是否涉及 `Develop_MC` 模式的前提
- `aimdk_msgs` 的接口文件不要轻易修改，会影响整个工作空间
