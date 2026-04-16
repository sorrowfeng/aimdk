#!/usr/bin/env python3
"""
语音指令节点 - IP 查询

监听语音指令，当用户说出包含"IP是多少/IP地址/我的IP"等词时，
自动获取本机 IP 并通过 TTS 播报。

音频来源: /aima/hal/audio/capture（原始多通道 PCM，16kHz S16LE，通道3为有效麦克风）
识别方式: faster-whisper（支持中英文混读）+ 能量 VAD

依赖安装:
    pip install faster-whisper numpy

模型会在首次运行时自动下载（~150MB），也可提前下载放到 ~/.cache/huggingface/
"""

import os
import socket
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from aimdk_msgs.msg import AudioCapture
from aimdk_msgs.srv import PlayTts

# ── 配置 ──────────────────────────────────────────────────────────────────────

TTS_SERVICE = "/aimdk_5Fmsgs/srv/PlayTts"
SAMPLE_RATE = 16000
MIC_CHANNEL = 3          # 经实测通道3有效

# 能量 VAD 参数
ENERGY_THRESHOLD = 80    # RMS 超过此值视为有声音（可根据环境调整）
SPEECH_FRAMES_MIN = 5    # 至少连续N帧有声才视为语音开始
SILENCE_FRAMES_END = 20  # 连续N帧静音后结束本次录音（约 0.6s）
MAX_SPEECH_FRAMES = 200  # 最长录音帧数，防止无限积累（约 6s）

# 触发 IP 查询的关键词（识别文本包含任一即触发，不区分大小写）
IP_QUERY_KEYWORDS = ["ip是多少", "ip地址", "我的ip", "ip号", "ip多少",
                     "IP是多少", "IP地址", "我的IP"]

# ─────────────────────────────────────────────────────────────────────────────


class VoiceCommandIpNode(Node):
    def __init__(self):
        super().__init__("voice_command_ip")

        self._init_asr()

        # VAD 状态
        self._speech_buf: list[np.ndarray] = []
        self._silence_count = 0
        self._speech_count = 0
        self._recording = False

        # ASR 在独立线程执行，避免阻塞 ROS 回调
        self._asr_lock = threading.Lock()
        self._asr_busy = False

        # TTS 播报中时屏蔽麦克风输入，防止录到自己说的话
        self._tts_mute_until = 0.0

        self.inited = False
        self.mic_channels = 0
        self.ref_channels = 0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=500,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.sub = self.create_subscription(
            AudioCapture,
            "/aima/hal/audio/capture",
            self._audio_callback,
            qos,
        )

        self.tts_client = self.create_client(PlayTts, TTS_SERVICE)
        self.get_logger().info(f"等待 TTS 服务: {TTS_SERVICE} ...")
        if self.tts_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("TTS 服务已就绪")
        else:
            self.get_logger().warn("TTS 服务未就绪")

        self.get_logger().info(
            f"语音指令节点已启动（能量阈值={ENERGY_THRESHOLD}），等待语音输入..."
        )

    # ── ASR 初始化 ──────────────────────────────────────────────────────────

    def _init_asr(self):
        try:
            from faster_whisper import WhisperModel  # type: ignore
        except ImportError:
            self.get_logger().error("未安装 faster-whisper，请运行: pip install faster-whisper")
            raise

        self.get_logger().info("加载 Whisper 模型（首次运行会自动下载）...")
        # 使用本地模型路径，避免从网络下载
        model_path = os.path.expanduser("~/faster-whisper-small")
        self.whisper = WhisperModel(model_path, device="cpu", compute_type="int8")
        self.get_logger().info("Whisper 模型加载完成")

    # ── 音频接收与能量 VAD ──────────────────────────────────────────────────

    def _audio_callback(self, msg: AudioCapture):
        import time
        if time.monotonic() < self._tts_mute_until:
            return  # TTS 播报中，屏蔽输入

        if not self.inited:
            self.mic_channels = msg.mic_channels
            self.ref_channels = msg.ref_channels
            total = self.mic_channels + self.ref_channels
            self.get_logger().info(
                f"音频格式: mic={self.mic_channels} ref={self.ref_channels} "
                f"total={total} rate={SAMPLE_RATE} channel_used={MIC_CHANNEL}"
            )
            self.inited = True

        raw = np.frombuffer(bytes(msg.data.data), dtype=np.int16)
        total_ch = self.mic_channels + self.ref_channels
        if total_ch == 0 or len(raw) % total_ch != 0:
            return

        samples = raw.reshape(-1, total_ch)
        frame = samples[:, MIC_CHANNEL].copy()
        rms = int(np.sqrt(np.mean(frame.astype(np.float32) ** 2)))

        is_speech = rms > ENERGY_THRESHOLD

        if not self._recording:
            if is_speech:
                self._speech_count += 1
                self._speech_buf.append(frame)
                if self._speech_count >= SPEECH_FRAMES_MIN:
                    self._recording = True
                    self._silence_count = 0
            else:
                self._speech_count = 0
                self._speech_buf.clear()
        else:
            self._speech_buf.append(frame)
            if is_speech:
                self._silence_count = 0
            else:
                self._silence_count += 1

            if (self._silence_count >= SILENCE_FRAMES_END
                    or len(self._speech_buf) >= MAX_SPEECH_FRAMES):
                pcm = np.concatenate(self._speech_buf).astype(np.float32) / 32768.0
                self._speech_buf = []
                self._recording = False
                self._silence_count = 0
                self._speech_count = 0
                self._run_asr_async(pcm)

    # ── ASR（在独立线程执行）───────────────────────────────────────────────

    def _run_asr_async(self, pcm: np.ndarray):
        with self._asr_lock:
            if self._asr_busy:
                self.get_logger().warn("ASR 正忙，跳过本次识别")
                return
            self._asr_busy = True

        t = threading.Thread(target=self._run_asr, args=(pcm,), daemon=True)
        t.start()

    def _run_asr(self, pcm: np.ndarray):
        try:
            segments, _ = self.whisper.transcribe(
                pcm,
                language="zh",
                beam_size=3,
                vad_filter=True,
            )
            text = "".join(s.text for s in segments).strip()
            if text:
                self.get_logger().info(f"识别结果: 「{text}」")
                self._match_and_respond(text)
            else:
                self.get_logger().debug("未识别到有效文字")
        except Exception as e:
            self.get_logger().error(f"ASR 异常: {e}")
        finally:
            with self._asr_lock:
                self._asr_busy = False

    # ── 意图匹配 ────────────────────────────────────────────────────────────

    def _match_and_respond(self, text: str):
        lower = text.lower().replace(" ", "")
        if any(kw.lower() in lower for kw in IP_QUERY_KEYWORDS):
            ip = self._get_ip()
            ip_spoken = ip.replace(".", "点")
            reply = f"您好，本机IP地址是 {ip_spoken}"
            self.get_logger().info(f"[IP查询] {reply}")
            self._play_tts(reply)

    # ── 获取 IP ─────────────────────────────────────────────────────────────

    def _get_ip(self) -> str:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "无法获取"

    # ── TTS 播报 ─────────────────────────────────────────────────────────────

    def _play_tts(self, text: str):
        if not self.tts_client.service_is_ready():
            self.get_logger().warn("TTS 服务不可用，跳过播报")
            return

        import time
        # 按字数估算播报时长（约 4 字/秒），多留 1.5s 余量
        estimated_sec = len(text) / 4.0 + 1.5
        self._tts_mute_until = time.monotonic() + estimated_sec
        self.get_logger().info(f"屏蔽麦克风 {estimated_sec:.1f}s")

        req = PlayTts.Request()
        req.tts_req.text = text
        req.tts_req.domain = "voice_command_ip"
        req.tts_req.trace_id = "ip_query"
        req.tts_req.is_interrupted = True
        req.tts_req.priority_weight = 0
        req.tts_req.priority_level.value = 6

        req.header.header.stamp = self.get_clock().now().to_msg()
        future = self.tts_client.call_async(req)
        future.add_done_callback(self._on_tts_done)

    def _on_tts_done(self, future):
        try:
            resp = future.result()
            if resp and resp.tts_resp.is_success:
                self.get_logger().info("TTS 播报成功")
            else:
                self.get_logger().warn("TTS 播报失败")
        except Exception as e:
            self.get_logger().error(f"TTS 回调异常: {e}")


# ─────────────────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceCommandIpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
