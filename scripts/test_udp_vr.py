#!/usr/bin/env python3
"""
UDP VR 遥操链路测试脚本

用法示例：
  # 1. 从 JSON 文件发送（固定数据）
  python3 scripts/test_udp_vr.py -f scripts/test_vr_data.json --rate 50

  # 2. 单轴独立测试（推荐）
  python3 scripts/test_udp_vr.py --wave --axis x --rate 50 --duration 10  # 只左右摆动
  python3 scripts/test_udp_vr.py --wave --axis y --rate 50 --duration 10  # 只上下摆动
  python3 scripts/test_udp_vr.py --wave --axis z --rate 50 --duration 10  # 只前后摆动

  # 3. 全轴波浪模式
  python3 scripts/test_udp_vr.py --wave --rate 50 --duration 10

  # 4. 单次发送
  python3 scripts/test_udp_vr.py -f scripts/test_vr_data.json --once
"""

import argparse
import json
import math
import socket
import time


def build_wave_data(t: float, axis: str = None) -> dict:
    """生成动态测试数据：可单轴独立测试，也可全轴一起摆动"""
    trigger = 0.5 + 0.5 * math.sin(t * 2.0)  # 0.0 ~ 1.0
    finger_val = round(1.4 * trigger, 3)

    # 基础摆动幅度
    offset = 0.03 * math.sin(t * 0.4 * math.pi)
    z_offset = 0.05 * math.sin(t * 0.3 * math.pi)

    # 默认全 0
    x = y = z = 0.0

    if axis == "x":
        x = offset
    elif axis == "y":
        y = offset
    elif axis == "z":
        z = z_offset
    else:
        # 全轴模式（旧版混合运动，用于整体观察）
        x = offset
        y = offset * 0.5
        z = z_offset

    return {
        "hands": [
            {
                "hand": "left",
                "relative_position": {
                    "x": round(x, 4),
                    "y": round(y, 4),
                    "z": round(z, 4),
                },
                "orientation": {
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "roll": 0.0,
                },
                "finger_joints": [1.400, finger_val, finger_val, finger_val, finger_val, finger_val],
            },
            {
                "hand": "right",
                "relative_position": {
                    "x": round(-x, 4) if axis != "z" else round(x, 4),
                    "y": round(-y, 4) if axis != "z" else round(y, 4),
                    "z": round(z, 4),
                },
                "orientation": {
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "roll": 0.0,
                },
                "finger_joints": [1.400, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        ]
    }


def main():
    parser = argparse.ArgumentParser(description="UDP VR Bridge 链路测试工具")
    parser.add_argument("--ip", default="172.16.20.110", help="目标 IP (默认: 172.16.20.110)")
    parser.add_argument("--port", "-p", type=int, default=9999, help="目标端口 (默认: 9999)")
    parser.add_argument("--file", "-f", help="JSON 数据文件路径")
    parser.add_argument("--wave", action="store_true", help="生成波浪测试数据（无需 --file）")
    parser.add_argument("--axis", choices=["x", "y", "z"], help="单轴测试：只摆动指定轴（配合 --wave）")
    parser.add_argument("--rate", "-r", type=float, default=50.0, help="发送频率 Hz (默认: 50)")
    parser.add_argument("--duration", "-d", type=float, default=0.0, help="发送时长(秒)，0 表示无限循环")
    parser.add_argument("--once", action="store_true", help="只发送一次")
    args = parser.parse_args()

    if not args.file and not args.wave:
        parser.error("必须指定 --file 或 --wave 之一")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate
    start_time = time.time()
    count = 0
    static_payload = None

    if args.file:
        with open(args.file, "r", encoding="utf-8") as fp:
            static_payload = json.dumps(json.load(fp)).encode("utf-8")

    axis_info = f" 单轴:{args.axis}" if args.axis else " 全轴"
    print(f"[TEST] 目标: {args.ip}:{args.port} | 频率: {args.rate}Hz{axis_info}")
    if args.once:
        print("[TEST] 模式: 单次发送")
    elif args.duration > 0:
        print(f"[TEST] 模式: 持续发送 {args.duration}s")
    else:
        print("[TEST] 模式: 无限循环 (Ctrl+C 停止)")

    try:
        while True:
            if args.wave:
                payload = json.dumps(build_wave_data(time.time() - start_time, args.axis)).encode("utf-8")
            else:
                payload = static_payload

            sock.sendto(payload, (args.ip, args.port))
            count += 1
            elapsed = time.time() - start_time

            if args.once:
                print(f"[TEST] 已发送 {len(payload)} 字节")
                break

            if args.duration > 0 and elapsed >= args.duration:
                print(f"[TEST] 完成。共发送 {count} 帧，耗时 {elapsed:.2f}s")
                break

            if count % int(args.rate) == 0:
                print(f"[TEST] 运行中... {count} 帧 | {elapsed:.1f}s")

            next_time = start_time + count * interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n[TEST] 用户中断。共发送 {count} 帧，耗时 {elapsed:.2f}s")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
