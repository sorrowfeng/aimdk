#!/usr/bin/env python3
"""
parse_x2_urdf.py
从 X2 URDF 中提取手臂运动学参数，供 C++ IK 代码参考。

用法:
  python3 parse_x2_urdf.py /path/to/x2_ultra.urdf
"""

import sys
import xml.etree.ElementTree as ET
import math


def parse_vector(s):
    return [float(x) for x in s.split()]


def magnitude(v):
    return math.sqrt(sum(x * x for x in v))


def extract_arm_params(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        limit = joint.find("limit")
        axis = joint.find("axis")
        jtype = joint.get("type")

        joints[name] = {
            "type": jtype,
            "origin_xyz": parse_vector(origin.get("xyz")) if origin is not None else [0, 0, 0],
            "origin_rpy": parse_vector(origin.get("rpy")) if origin is not None else [0, 0, 0],
            "axis": parse_vector(axis.get("xyz")) if axis is not None else [0, 0, 1],
            "lower": float(limit.get("lower")) if limit is not None else -3.14,
            "upper": float(limit.get("upper")) if limit is not None else 3.14,
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else "",
            "child": joint.find("child").get("link") if joint.find("child") is not None else "",
        }

    # 左臂链
    left_chain = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
    ]

    # 右臂链
    right_chain = [
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
    ]

    def print_chain(chain_name, chain):
        print(f"\n{'='*60}")
        print(f"{chain_name}")
        print(f"{'='*60}")

        for name in chain:
            j = joints[name]
            print(f"{name:30s} axis=({j['axis'][0]:.0f} {j['axis'][1]:.0f} {j['axis'][2]:.0f})  "
                  f"limit=[{j['lower']:7.4f}, {j['upper']:7.4f}]  "
                  f"origin=({j['origin_xyz'][0]:.5f}, {j['origin_xyz'][1]:.5f}, {j['origin_xyz'][2]:.5f})")

        # 计算关键连杆长度
        sp = joints[chain[0]]
        sr = joints[chain[1]]
        sy = joints[chain[2]]
        elbow = joints[chain[3]]
        wrist_yaw = joints[chain[4]]
        wrist_pitch = joints[chain[5]]

        # 肩宽和肩高（从 torso 到 shoulder pitch 的偏移）
        print(f"\n--- 从 torso 到 shoulder_pitch 的偏移 ---")
        print(f"  xyz=({sp['origin_xyz'][0]:.5f}, {sp['origin_xyz'][1]:.5f}, {sp['origin_xyz'][2]:.5f})")

        #  shoulder_pitch -> shoulder_yaw 长度
        shoulder_to_yaw = magnitude(sr["origin_xyz"]) + magnitude(sy["origin_xyz"])
        print(f"\n--- shoulder_pitch -> shoulder_yaw 组合长度 ---")
        print(f"  shoulder_roll offset  = {magnitude(sr['origin_xyz']):.5f} m")
        print(f"  shoulder_yaw offset   = {magnitude(sy['origin_xyz']):.5f} m")
        print(f"  组合近似长度          = {shoulder_to_yaw:.5f} m")

        # shoulder_yaw -> elbow
        upper_arm = magnitude(elbow["origin_xyz"])
        print(f"\n--- shoulder_yaw -> elbow (上臂) ---")
        print(f"  长度 = {upper_arm:.5f} m")

        # elbow -> wrist_yaw
        forearm_1 = magnitude(wrist_yaw["origin_xyz"])
        print(f"\n--- elbow -> wrist_yaw (前臂段1) ---")
        print(f"  长度 = {forearm_1:.5f} m")

        # wrist_yaw -> wrist_pitch
        forearm_2 = magnitude(wrist_pitch["origin_xyz"])
        print(f"\n--- wrist_yaw -> wrist_pitch (前臂段2) ---")
        print(f"  长度 = {forearm_2:.5f} m")

        # 前臂总长
        print(f"\n--- 前臂总长度 (elbow -> wrist_pitch) ---")
        print(f"  总长 = {forearm_1 + forearm_2:.5f} m")

    print_chain("左臂", left_chain)
    print_chain("右臂", right_chain)

    # 肩宽
    left_sp = joints["left_shoulder_pitch_joint"]["origin_xyz"]
    right_sp = joints["right_shoulder_pitch_joint"]["origin_xyz"]
    shoulder_width = abs(left_sp[1] - right_sp[1])
    shoulder_height = left_sp[2]

    print(f"\n{'='*60}")
    print("全局尺寸")
    print(f"{'='*60}")
    print(f"肩宽 (y 方向间距) = {shoulder_width:.5f} m")
    print(f"肩高 (z 方向相对 torso) = {shoulder_height:.5f} m")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 parse_x2_urdf.py <x2_ultra.urdf>")
        sys.exit(1)
    extract_arm_params(sys.argv[1])
