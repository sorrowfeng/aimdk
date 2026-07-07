#!/usr/bin/env python3
"""
X2 MuJoCo simulation driven by PICO UDP JSON v3 packets.

Default model:
  docs/X2_URDF-v1.3.0/scene.xml

Examples:
  python scripts/x2_mujoco_teleop_sim.py
  python scripts/x2_mujoco_teleop_sim.py --no-scene
  python scripts/x2_mujoco_teleop_sim.py --no-udp-control
  python scripts/x2_mujoco_teleop_sim.py --no-scene --no-udp-control

Test sender:
  python scripts/test_udp_vr.py --ip 127.0.0.1 --wave --rate 50 --duration 10
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
X2_URDF_DIR = REPO_ROOT / "docs" / "X2_URDF-v1.3.0"
DEFAULT_MODEL_PATH = X2_URDF_DIR / "scene.xml"
NO_SCENE_MODEL_PATH = X2_URDF_DIR / "x2_ultra.xml"
UDP_PORT = 9999

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]

LEG_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]

HEAD_JOINTS = [
    "head_yaw_joint",
    "head_pitch_joint",
]

CONTROLLED_JOINTS = LEG_JOINTS + WAIST_JOINTS + HEAD_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

READY_POSE_LEFT = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0], dtype=float)
READY_POSE_RIGHT = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0], dtype=float)


def clamp(value: float, low: float, high: float) -> float:
    return min(max(float(value), low), high)


def as_float(value: Any, default: float = 0.0) -> float:
    if isinstance(value, bool):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def yaw_to_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


@dataclass
class ControllerState:
    side: str
    position: np.ndarray
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    key_one: bool = False
    key_two: bool = False
    live: bool = True


class SimpleArmIK:
    """Port of the C++ SimpleArmIK used by vr_teleop_node."""

    def __init__(self) -> None:
        self.shoulder_roll_offset_y = 0.0495
        self.shoulder_yaw_offset_x = 0.001224
        self.shoulder_yaw_offset_z = -0.121195
        self.elbow_offset_x = 0.0140
        self.elbow_offset_y = 0.0002
        self.elbow_offset_z = -0.08295
        self.wrist_yaw_offset_x = -0.01328
        self.wrist_yaw_offset_y = -0.000098
        self.wrist_yaw_offset_z = -0.12058
        self.wrist_pitch_offset_x = 0.00049
        self.wrist_pitch_offset_z = -0.0820

        self.damping = 0.05
        self.max_iterations = 30
        self.convergence_tol = 1e-4

    def wrist_center_position(self, q7: np.ndarray) -> np.ndarray:
        sp, sr, sy, ep = [float(v) for v in q7[:4]]

        r_sp = rot_y(sp)
        r_sr = rot_x(sr)
        r_sy = rot_z(sy)

        shoulder_yaw_pos = r_sp @ (
            np.array([0.0, self.shoulder_roll_offset_y, 0.0], dtype=float)
            + r_sr @ np.array([self.shoulder_yaw_offset_x, 0.0, self.shoulder_yaw_offset_z], dtype=float)
        )

        r_shoulder = r_sp @ r_sr @ r_sy
        elbow_pos = shoulder_yaw_pos + r_shoulder @ np.array(
            [self.elbow_offset_x, self.elbow_offset_y, self.elbow_offset_z],
            dtype=float,
        )

        r_elbow = r_shoulder @ rot_y(ep)
        wrist_yaw_pos = elbow_pos + r_elbow @ np.array(
            [self.wrist_yaw_offset_x, self.wrist_yaw_offset_y, self.wrist_yaw_offset_z],
            dtype=float,
        )
        return wrist_yaw_pos + r_elbow @ np.array(
            [self.wrist_pitch_offset_x, 0.0, self.wrist_pitch_offset_z],
            dtype=float,
        )

    def position_jacobian(self, q7: np.ndarray) -> np.ndarray:
        dq = 1e-6
        p0 = self.wrist_center_position(q7)
        jac = np.zeros((3, 4), dtype=float)
        for idx in range(4):
            q_pert = q7.copy()
            q_pert[idx] += dq
            jac[:, idx] = (self.wrist_center_position(q_pert) - p0) / dq
        return jac

    def solve_arm_position(
        self,
        target_pos: np.ndarray,
        q_init: np.ndarray,
        joint_limits: list[tuple[float, float]],
        elbow_rest_target: float,
        posture_gain: float,
    ) -> np.ndarray:
        q = q_init.astype(float).copy()
        for _ in range(self.max_iterations):
            err = target_pos - self.wrist_center_position(q)
            if float(np.linalg.norm(err)) < self.convergence_tol:
                break

            jac = self.position_jacobian(q)
            jjt = jac @ jac.T
            jjt[np.diag_indices(3)] += self.damping * self.damping
            j_pinv = jac.T @ np.linalg.solve(jjt, np.eye(3))

            posture_step = np.zeros(4, dtype=float)
            posture_step[3] = posture_gain * (elbow_rest_target - q[3])
            delta = j_pinv @ err + (np.eye(4) - j_pinv @ jac) @ posture_step

            for idx in range(4):
                low, high = joint_limits[idx]
                q[idx] = clamp(q[idx] + clamp(delta[idx], -0.05, 0.05), low, high)
        return q


class X2MuJoCoSim:
    def __init__(self, model_path: Path) -> None:
        self.model_path = model_path.resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")

        print(f"[load] MuJoCo model: {self.model_path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)

        self.joint_name2id: dict[str, int] = {}
        self.joint_qpos_ids: dict[str, int] = {}
        self.joint_dof_ids: dict[str, int] = {}
        self.joint_limits: dict[str, tuple[float, float]] = {}
        self.body_name2id: dict[str, int] = {}
        self.actuator_joint_names: list[str | None] = []

        self._build_name_maps()
        self.free_qpos_addr = self.joint_qpos_ids.get("floating_base_joint")
        self.base_initial_pos = np.zeros(3, dtype=float)
        if self.free_qpos_addr is not None:
            self.base_initial_pos = self.data.qpos[self.free_qpos_addr : self.free_qpos_addr + 3].copy()

        mujoco.mj_forward(self.model, self.data)
        print(
            f"[load] ready: nq={self.model.nq}, nv={self.model.nv}, "
            f"nu={self.model.nu}, bodies={self.model.nbody}, joints={len(self.joint_name2id)}"
        )

    def _build_name_maps(self) -> None:
        for jid in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if not name:
                continue
            self.joint_name2id[name] = jid
            self.joint_qpos_ids[name] = int(self.model.jnt_qposadr[jid])
            self.joint_dof_ids[name] = int(self.model.jnt_dofadr[jid])
            if int(self.model.jnt_limited[jid]):
                low, high = [float(v) for v in self.model.jnt_range[jid]]
            else:
                low, high = -math.inf, math.inf
            self.joint_limits[name] = (low, high)

        for bid in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if name:
                self.body_name2id[name] = bid

        for aid in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[aid, 0])
            if joint_id >= 0:
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                self.actuator_joint_names.append(name)
            else:
                self.actuator_joint_names.append(None)

    def has_joint(self, name: str) -> bool:
        return name in self.joint_qpos_ids

    def limit_for_joint(self, name: str) -> tuple[float, float]:
        return self.joint_limits.get(name, (-math.inf, math.inf))

    def clip_joint(self, name: str, value: float) -> float:
        low, high = self.limit_for_joint(name)
        if math.isfinite(low) and math.isfinite(high):
            return clamp(value, low, high)
        return float(value)

    def get_joint_angle(self, name: str) -> float:
        return float(self.data.qpos[self.joint_qpos_ids[name]])

    def set_joint_angle(self, name: str, value: float) -> None:
        if name not in self.joint_qpos_ids:
            return
        self.data.qpos[self.joint_qpos_ids[name]] = self.clip_joint(name, value)

    def set_joint_angles(self, q_dict: dict[str, float]) -> None:
        for name, value in q_dict.items():
            self.set_joint_angle(name, value)

    def sync_ctrl_from_qpos(self) -> None:
        for aid, joint_name in enumerate(self.actuator_joint_names):
            if joint_name in self.joint_qpos_ids:
                self.data.ctrl[aid] = self.get_joint_angle(joint_name)

    def configure_viewer_joint_sliders(self) -> None:
        for aid, joint_name in enumerate(self.actuator_joint_names):
            if joint_name not in self.joint_qpos_ids:
                continue
            low, high = self.limit_for_joint(joint_name)
            if not (math.isfinite(low) and math.isfinite(high)):
                continue
            self.model.actuator_ctrlrange[aid, 0] = low
            self.model.actuator_ctrlrange[aid, 1] = high
            self.model.actuator_ctrllimited[aid] = 1
        self.sync_ctrl_from_qpos()

    def apply_viewer_joint_sliders(self) -> None:
        for aid, joint_name in enumerate(self.actuator_joint_names):
            if joint_name in self.joint_qpos_ids:
                self.set_joint_angle(joint_name, float(self.data.ctrl[aid]))

    def get_arm_joints(self, side: str) -> np.ndarray:
        names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        return np.array([self.get_joint_angle(name) for name in names if self.has_joint(name)], dtype=float)

    def set_base_pose(self, base_pose: np.ndarray) -> None:
        if self.free_qpos_addr is None:
            return
        addr = self.free_qpos_addr
        self.data.qpos[addr : addr + 3] = self.base_initial_pos + np.array(
            [base_pose[0], base_pose[1], 0.0],
            dtype=float,
        )
        self.data.qpos[addr + 3 : addr + 7] = yaw_to_quat_wxyz(float(base_pose[2]))
        mujoco.mj_normalizeQuat(self.model, self.data.qpos)

    def forward(self, preserve_ctrl: bool = False) -> None:
        self.data.qvel[:] = 0.0
        if not preserve_ctrl:
            self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def print_joint_summary(self) -> None:
        print("[joints] controlled names present:")
        for name in CONTROLLED_JOINTS:
            if self.has_joint(name):
                low, high = self.limit_for_joint(name)
                print(f"  {name:<32} [{low:+.4f}, {high:+.4f}]")
            else:
                print(f"  {name:<32} missing")


class X2PicoTeleopSim:
    def __init__(
        self,
        sim: X2MuJoCoSim,
        udp_control_enabled: bool = True,
        coord_map: tuple[str, str, str] = ("-z", "-x", "y"),
        position_scale: float = 1.0,
        vr_timeout_sec: float = 0.5,
        max_joint_speed: float = 2.0,
        elbow_straight_target: float = -0.2,
        elbow_full_extension_delta: float = 0.30,
        elbow_posture_gain: float = 0.08,
    ) -> None:
        self.sim = sim
        self.udp_port = UDP_PORT
        self.udp_control_enabled = udp_control_enabled
        self.coord_map = coord_map
        self.position_scale = position_scale
        self.vr_timeout_sec = vr_timeout_sec
        self.max_joint_speed = max_joint_speed
        self.viewer_joint_sliders = not udp_control_enabled
        self.elbow_straight_target = elbow_straight_target
        self.elbow_full_extension_delta = elbow_full_extension_delta
        self.elbow_posture_gain = elbow_posture_gain

        self.left_ik = SimpleArmIK()
        self.right_ik = SimpleArmIK()
        self.ready_pose_left = READY_POSE_LEFT.copy()
        self.ready_pose_right = READY_POSE_RIGHT.copy()
        self.left_wrist_ready_pos = self.left_ik.wrist_center_position(self.ready_pose_left)
        self.right_wrist_ready_pos = self.right_ik.wrist_center_position(self.ready_pose_right)

        self.lock = threading.Lock()
        self.running = True
        self.emergency_stop = False
        self.last_sequence: int | None = None
        self.last_receive_time: float | None = None
        self._timeout_reported = False
        self._hz_count = 0
        self._hz_last_time = time.time()
        self._control_hz = 0.0
        self.base_pose = np.zeros(3, dtype=float)
        self.base_velocity = np.zeros(2, dtype=float)  # linear_x, angular_z

        self.target_q: dict[str, float] = {}
        self._install_initial_targets()
        self.sim.set_joint_angles(self.target_q)
        self.sim.forward()
        if self.viewer_joint_sliders:
            self.sim.configure_viewer_joint_sliders()

        print(f"[init] UDP port: {self.udp_port}")
        print(f"[init] UDP control: {'enabled' if self.udp_control_enabled else 'disabled'}")
        print(
            "[init] ready wrist local: "
            f"L={self.left_wrist_ready_pos.round(4).tolist()} "
            f"R={self.right_wrist_ready_pos.round(4).tolist()}"
        )

    def _install_initial_targets(self) -> None:
        for name in LEG_JOINTS + WAIST_JOINTS + HEAD_JOINTS:
            if self.sim.has_joint(name):
                self.target_q[name] = self.sim.clip_joint(name, 0.0)
        for name, value in zip(LEFT_ARM_JOINTS, self.ready_pose_left):
            if self.sim.has_joint(name):
                self.target_q[name] = self.sim.clip_joint(name, float(value))
        for name, value in zip(RIGHT_ARM_JOINTS, self.ready_pose_right):
            if self.sim.has_joint(name):
                self.target_q[name] = self.sim.clip_joint(name, float(value))

    def udp_receiver_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.udp_port))
        sock.settimeout(1.0)
        print(f"[udp] listening on 0.0.0.0:{self.udp_port}")

        while self.running:
            try:
                payload, addr = sock.recvfrom(65535)
                self.process_udp_payload(payload, addr)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                print(f"[udp] error: {exc}")
        sock.close()

    def process_udp_payload(self, payload: bytes, addr: tuple[str, int] | None = None) -> bool:
        try:
            packet = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return False

        if packet.get("packet_version") != 3:
            return False

        sequence = packet.get("sequence")
        if isinstance(sequence, (int, float)) and not isinstance(sequence, bool):
            seq = int(sequence)
            if self.last_sequence is not None and seq <= self.last_sequence:
                return False
            self.last_sequence = seq

        motion_allowed = self._packet_is_safe(packet)
        controllers = self._parse_controllers(packet, motion_allowed)

        if self.udp_control_enabled and self._both_estop_buttons_pressed(controllers):
            self.emergency_stop = True
            motion_allowed = False
            print("[estop] both controller buttons are pressed; simulation control frozen")

        robot_control = packet.get("robot_control", {})
        if not isinstance(robot_control, dict):
            robot_control = {}

        with self.lock:
            self.last_receive_time = time.time()
            self._timeout_reported = False
            if self.udp_control_enabled and not self.emergency_stop:
                self._apply_robot_control(robot_control, controllers, motion_allowed)

        self._hz_count += 1
        now = time.time()
        elapsed = now - self._hz_last_time
        if elapsed >= 1.0:
            self._control_hz = self._hz_count / elapsed
            self._hz_count = 0
            self._hz_last_time = now

        return True

    def _packet_is_safe(self, packet: dict[str, Any]) -> bool:
        safety = packet.get("safety", {})
        if not isinstance(safety, dict):
            return False
        if packet.get("operator_mode") == "stop_signal":
            return False
        return bool(safety.get("safe_to_execute", False))

    def _parse_controllers(self, packet: dict[str, Any], motion_allowed: bool) -> dict[str, ControllerState]:
        out: dict[str, ControllerState] = {}
        controllers = packet.get("controllers", {})
        if not isinstance(controllers, dict):
            controllers = {}

        for side in ("left", "right"):
            node = controllers.get(side, {})
            if not isinstance(node, dict):
                node = {}
            quality = str(node.get("quality", "live") or "live")
            live = quality == "live" or quality == ""

            position = np.zeros(3, dtype=float)
            roll = pitch = yaw = 0.0
            if motion_allowed and live:
                pose = node.get("pose", {})
                if isinstance(pose, dict):
                    raw_position = pose.get("position", {})
                    if isinstance(raw_position, dict):
                        raw = np.array(
                            [
                                as_float(raw_position.get("x")),
                                as_float(raw_position.get("y")),
                                as_float(raw_position.get("z")),
                            ],
                            dtype=float,
                        )
                        position = self._apply_coord_map(raw)
                    raw_orientation = pose.get("orientation", {})
                    if isinstance(raw_orientation, dict):
                        roll, pitch, yaw = self._parse_orientation(raw_orientation)

            input_node = node.get("input", {})
            if not isinstance(input_node, dict):
                input_node = {}
            out[side] = ControllerState(
                side=side,
                position=position,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                key_one=bool(input_node.get("primary_pressed", False)),
                key_two=bool(input_node.get("secondary_pressed", False)),
                live=live,
            )
        return out

    def _parse_orientation(self, raw: dict[str, Any]) -> tuple[float, float, float]:
        if all(key in raw for key in ("roll", "pitch", "yaw")):
            return (
                math.radians(as_float(raw.get("roll"))),
                math.radians(as_float(raw.get("pitch"))),
                math.radians(as_float(raw.get("yaw"))),
            )
        return 0.0, 0.0, 0.0

    def _apply_coord_map(self, raw: np.ndarray) -> np.ndarray:
        axis_index = {"x": 0, "y": 1, "z": 2}

        def map_axis(spec: str) -> float:
            sign = -1.0 if spec.startswith("-") else 1.0
            axis = spec[1:] if spec.startswith("-") else spec
            idx = axis_index.get(axis)
            if idx is None:
                return 0.0
            return sign * float(raw[idx])

        return np.array([map_axis(spec) for spec in self.coord_map], dtype=float)

    def _both_estop_buttons_pressed(self, controllers: dict[str, ControllerState]) -> bool:
        left = controllers.get("left")
        right = controllers.get("right")
        return bool(left and right and left.key_one and left.key_two and right.key_one and right.key_two)

    def _apply_robot_control(
        self,
        robot_control: dict[str, Any],
        controllers: dict[str, ControllerState],
        motion_allowed: bool,
    ) -> None:
        if not motion_allowed:
            self._set_ready_arm_targets()
            self.base_velocity[:] = 0.0
            return

        explicit_arm_targets = self._apply_direct_arm_arrays(robot_control)
        if not explicit_arm_targets:
            self._apply_controller_ik(controllers)

        self._apply_head(robot_control.get("head", {}))
        body = robot_control.get("body", {})
        if isinstance(body, dict):
            self._apply_waist(body.get("waist", {}))
        self._apply_base(robot_control.get("base", {}))
        self._apply_named_joints(robot_control.get("joints"))

    def _set_ready_arm_targets(self) -> None:
        for name, value in zip(LEFT_ARM_JOINTS, self.ready_pose_left):
            if self.sim.has_joint(name):
                self.target_q[name] = self.sim.clip_joint(name, float(value))
        for name, value in zip(RIGHT_ARM_JOINTS, self.ready_pose_right):
            if self.sim.has_joint(name):
                self.target_q[name] = self.sim.clip_joint(name, float(value))

    def _apply_direct_arm_arrays(self, robot_control: dict[str, Any]) -> bool:
        arms = robot_control.get("arms", {})
        if not isinstance(arms, dict):
            return False

        applied = False
        for side, names in (("left", LEFT_ARM_JOINTS), ("right", RIGHT_ARM_JOINTS)):
            values = arms.get(side)
            if isinstance(values, list) and len(values) == 7:
                for name, value in zip(names, values):
                    if self.sim.has_joint(name):
                        self.target_q[name] = self.sim.clip_joint(name, as_float(value))
                applied = True
        return applied

    def _apply_controller_ik(self, controllers: dict[str, ControllerState]) -> None:
        left = controllers.get("left")
        right = controllers.get("right")
        if left is not None:
            q_left = self._solve_controller_side(
                side="left",
                ctrl=left,
                ik=self.left_ik,
                ready_pose=self.ready_pose_left,
                ready_wrist_pos=self.left_wrist_ready_pos,
                joint_names=LEFT_ARM_JOINTS,
            )
            for name, value in zip(LEFT_ARM_JOINTS, q_left):
                if self.sim.has_joint(name):
                    self.target_q[name] = self.sim.clip_joint(name, float(value))

        if right is not None:
            q_right = self._solve_controller_side(
                side="right",
                ctrl=right,
                ik=self.right_ik,
                ready_pose=self.ready_pose_right,
                ready_wrist_pos=self.right_wrist_ready_pos,
                joint_names=RIGHT_ARM_JOINTS,
            )
            for name, value in zip(RIGHT_ARM_JOINTS, q_right):
                if self.sim.has_joint(name):
                    self.target_q[name] = self.sim.clip_joint(name, float(value))

    def _solve_controller_side(
        self,
        side: str,
        ctrl: ControllerState,
        ik: SimpleArmIK,
        ready_pose: np.ndarray,
        ready_wrist_pos: np.ndarray,
        joint_names: list[str],
    ) -> np.ndarray:
        q_init = np.array([self.target_q.get(name, ready_pose[idx]) for idx, name in enumerate(joint_names)], dtype=float)
        joint_limits = [self.sim.limit_for_joint(name) for name in joint_names]
        vr_delta = ctrl.position * self.position_scale
        target_pos = ready_wrist_pos + vr_delta
        elbow_rest_target = self._elbow_rest_target(ready_pose, vr_delta)
        q = ik.solve_arm_position(
            target_pos=target_pos,
            q_init=q_init,
            joint_limits=joint_limits,
            elbow_rest_target=elbow_rest_target,
            posture_gain=self.elbow_posture_gain,
        )

        # Match vr_teleop_node: controller roll -> wrist_yaw,
        # controller pitch -> wrist_pitch, controller yaw -> wrist_roll.
        wrist_values = [ctrl.roll, ctrl.pitch, ctrl.yaw]
        for idx, value in zip((4, 5, 6), wrist_values):
            low, high = joint_limits[idx]
            q[idx] = clamp(value, low, high)

        return q

    def _elbow_rest_target(self, ready_pose: np.ndarray, vr_delta: np.ndarray) -> float:
        full_extension_delta = max(1e-6, self.elbow_full_extension_delta)
        ratio = clamp(float(np.linalg.norm(vr_delta)) / full_extension_delta, 0.0, 1.0)
        return float(ready_pose[3] + (self.elbow_straight_target - ready_pose[3]) * ratio)

    def _apply_head(self, head: Any) -> None:
        if not isinstance(head, dict):
            return
        mapping = {
            "head_yaw_joint": "yaw",
            "head_pitch_joint": "pitch",
        }
        for joint, field in mapping.items():
            if self.sim.has_joint(joint) and field in head:
                self.target_q[joint] = self.sim.clip_joint(joint, math.radians(as_float(head.get(field))))

    def _apply_waist(self, waist: Any) -> None:
        if not isinstance(waist, dict):
            return
        mapping = {
            "waist_yaw_joint": "yaw",
            "waist_pitch_joint": "pitch",
            "waist_roll_joint": "roll",
        }
        for joint, field in mapping.items():
            if self.sim.has_joint(joint) and field in waist:
                self.target_q[joint] = self.sim.clip_joint(joint, math.radians(as_float(waist.get(field))))

    def _apply_base(self, base: Any) -> None:
        if not isinstance(base, dict):
            self.base_velocity[:] = 0.0
            return
        self.base_velocity[0] = as_float(base.get("linear_x"))
        self.base_velocity[1] = as_float(base.get("angular_z"))

    def _apply_named_joints(self, joints: Any) -> None:
        if isinstance(joints, dict):
            for name, value in joints.items():
                if isinstance(name, str) and self.sim.has_joint(name):
                    self.target_q[name] = self.sim.clip_joint(name, as_float(value))
            return
        if isinstance(joints, list):
            for item in joints:
                if not isinstance(item, dict):
                    continue
                name = item.get("name")
                if isinstance(name, str) and self.sim.has_joint(name):
                    self.target_q[name] = self.sim.clip_joint(name, as_float(item.get("position")))

    def step(self, dt: float) -> None:
        if self.viewer_joint_sliders:
            self.sim.apply_viewer_joint_sliders()
            self.sim.set_base_pose(self.base_pose)
            self.sim.forward(preserve_ctrl=True)
            return

        with self.lock:
            target_q = dict(self.target_q)
            base_velocity = self.base_velocity.copy()
            timed_out = (
                self.udp_control_enabled
                and self.last_receive_time is not None
                and self.vr_timeout_sec > 0.0
                and (time.time() - self.last_receive_time) > self.vr_timeout_sec
            )
            if timed_out:
                for name, value in zip(LEFT_ARM_JOINTS, self.ready_pose_left):
                    if self.sim.has_joint(name):
                        target_q[name] = self.sim.clip_joint(name, float(value))
                for name, value in zip(RIGHT_ARM_JOINTS, self.ready_pose_right):
                    if self.sim.has_joint(name):
                        target_q[name] = self.sim.clip_joint(name, float(value))
                base_velocity[:] = 0.0
                if not self._timeout_reported:
                    print(f"[timeout] no UDP data for {self.vr_timeout_sec:.2f}s; arms return to ready pose")
                    self._timeout_reported = True

            yaw = float(self.base_pose[2])
            linear_x, angular_z = [float(v) for v in base_velocity]
            self.base_pose[0] += linear_x * math.cos(yaw) * dt
            self.base_pose[1] += linear_x * math.sin(yaw) * dt
            self.base_pose[2] += angular_z * dt

        if self.max_joint_speed <= 0.0:
            self.sim.set_joint_angles(target_q)
        else:
            max_step = self.max_joint_speed * dt
            for name, target in target_q.items():
                if not self.sim.has_joint(name):
                    continue
                current = self.sim.get_joint_angle(name)
                self.sim.set_joint_angle(name, current + clamp(target - current, -max_step, max_step))

        self.sim.set_base_pose(self.base_pose)
        self.sim.forward()

    def run(self, use_viewer: bool = True, duration: float = 0.0) -> None:
        udp_thread = None
        if self.udp_control_enabled:
            udp_thread = threading.Thread(target=self.udp_receiver_loop, daemon=True)
            udp_thread.start()

        start_time = time.time()
        last_time = start_time
        print("=" * 72)
        print("X2 MuJoCo PICO Teleop Simulation")
        print(f"  model: {self.sim.model_path}")
        print(f"  UDP control: {'0.0.0.0:' + str(self.udp_port) if self.udp_control_enabled else 'disabled'}")
        print("=" * 72)

        try:
            if use_viewer:
                with mujoco.viewer.launch_passive(self.sim.model, self.sim.data) as viewer:
                    while viewer.is_running() and self.running:
                        now = time.time()
                        dt = max(1e-4, min(now - last_time, 0.05))
                        last_time = now
                        self.step(dt)
                        viewer.sync()
                        if duration > 0.0 and now - start_time >= duration:
                            break
                        time.sleep(0.001)
            else:
                while self.running:
                    now = time.time()
                    dt = max(1e-4, min(now - last_time, 0.05))
                    last_time = now
                    self.step(dt)
                    if duration > 0.0 and now - start_time >= duration:
                        break
                    time.sleep(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if udp_thread:
                udp_thread.join(timeout=0.2)
            print("[done] simulation exited")


def main() -> int:
    parser = argparse.ArgumentParser(description="X2 MuJoCo PICO UDP JSON v3 teleop simulation")
    parser.add_argument(
        "--udp-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable UDP JSON v3 control on port 9999",
    )
    parser.add_argument(
        "--scene",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Load scene.xml with floor/skybox; --no-scene loads the STL robot model only",
    )
    args = parser.parse_args()

    model_path = DEFAULT_MODEL_PATH if args.scene else NO_SCENE_MODEL_PATH
    sim = X2MuJoCoSim(model_path)

    controller = X2PicoTeleopSim(
        sim=sim,
        udp_control_enabled=args.udp_control,
    )

    controller.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
