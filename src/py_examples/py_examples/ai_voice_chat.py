#!/usr/bin/env python3
"""
AI 语音问答节点

监听语音指令，通过百度在线 ASR 识别用户问题后，调用 Claude (Anthropic API) 进行回答，
并通过 TTS 播报 AI 的回复。

音频来源: /aima/hal/audio/capture（原始多通道 PCM，16kHz S16LE，通道3为有效麦克风）
识别方式: 百度短语音识别标准版 REST API
AI 模型: claude-sonnet-4-6 (通过 Kimi Coding API 代理)

依赖安装:
    pip install numpy anthropic requests
"""

import base64
import json
import os
import socket
import threading
import time

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

# 能量 VAD 参数（与 voice_command_ip.py 保持一致）
ENERGY_THRESHOLD = 80    # RMS 超过此值视为有声音
SPEECH_FRAMES_MIN = 5    # 至少连续N帧有声才视为语音开始
SILENCE_FRAMES_END = 20  # 连续N帧静音后结束本次录音（约 0.6s）
MAX_SPEECH_FRAMES = 200  # 最长录音帧数，防止无限积累（约 6s）

# 百度语音识别配置（请替换为您在百度智能云申请的凭证）
BAIDU_API_KEY = "IAQce9LH1OICHVe4az9WATVt"
BAIDU_SECRET_KEY = "tMnRfj2cA91isK7Wo3JNL3gxZ881lXMq"
BAIDU_CUID = "robot_ai_voice_chat_001"
BAIDU_DEV_PID = 1537     # 普通话输入法模型，适合短句识别

# AI 配置（硬编码）
ANTHROPIC_API_KEY = "sk-kimi-RqmirzzWlKE9sGiXhQleXAtqA2PZDqJHLEwmtBc6TY2dRGilAnr0Pj7hIaNyW9iI"
ANTHROPIC_BASE_URL = "https://api.kimi.com/coding/"
AI_MODEL = "claude-sonnet-4-6"
AI_SYSTEM_PROMPT = (
    "你是一个搭载在人形机器人上的 AI 助手。请用简洁、自然、口语化的中文回答，"
    "控制在 100 字以内，方便语音播报。回答要有礼貌，适合公众场合互动。"
)

# 唤醒/触发关键词（识别文本包含任一即触发，不区分大小写）
WAKE_KEYWORDS = [
    "小智", "小志", "小知", "小致",
    "机器人", "小白",
]

# ─────────────────────────────────────────────────────────────────────────────


try:
    import requests  # type: ignore
except ImportError:
    requests = None  # type: ignore


try:
    import anthropic  # type: ignore
except ImportError:
    anthropic = None  # type: ignore


class AIVoiceChatNode(Node):
    def __init__(self):
        super().__init__("ai_voice_chat")

        self._init_asr()
        self._init_ai()

        # VAD 状态
        self._speech_buf: list[np.ndarray] = []
        self._silence_count = 0
        self._speech_count = 0
        self._recording = False

        # ASR / AI 在独立线程执行，避免阻塞 ROS 回调
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
            f"AI 语音问答节点已启动（能量阈值={ENERGY_THRESHOLD}），"
            f"唤醒词: {WAKE_KEYWORDS}，等待语音输入..."
        )

    # ── ASR 初始化 ──────────────────────────────────────────────────────────

    def _init_asr(self):
        if requests is None:
            self.get_logger().error("未安装 requests，请运行: pip install requests")
            raise RuntimeError("缺少 requests 依赖")

        if BAIDU_API_KEY == "YOUR_BAIDU_API_KEY" or BAIDU_SECRET_KEY == "YOUR_BAIDU_SECRET_KEY":
            self.get_logger().error("请先配置 BAIDU_API_KEY 和 BAIDU_SECRET_KEY")
            raise RuntimeError("百度 API 凭证未配置")

        self.get_logger().info("正在获取百度语音识别 access_token...")
        self.baidu_token = self._fetch_baidu_token()
        self.get_logger().info("百度 access_token 获取成功")

    def _fetch_baidu_token(self) -> str:
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {
            "grant_type": "client_credentials",
            "client_id": BAIDU_API_KEY,
            "client_secret": BAIDU_SECRET_KEY,
        }
        resp = requests.post(url, params=params, timeout=10)
        data = resp.json()
        if "access_token" not in data:
            raise RuntimeError(f"获取百度 token 失败: {data}")
        return data["access_token"]

    # ── AI 客户端初始化 ─────────────────────────────────────────────────────

    def _init_ai(self):
        if anthropic is None:
            self.get_logger().error("未安装 anthropic，请运行: pip install anthropic")
            raise RuntimeError("缺少 anthropic 依赖")

        self.ai_client = anthropic.Anthropic(
            api_key=ANTHROPIC_API_KEY,
            base_url=ANTHROPIC_BASE_URL,
        )
        self.get_logger().info("AI 客户端初始化完成")

    # ── 音频接收与能量 VAD ──────────────────────────────────────────────────

    def _audio_callback(self, msg: AudioCapture):
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
                # 直接拼接原始 int16 帧，后面再处理为 pcm 数据
                pcm_int16 = np.concatenate(self._speech_buf).astype(np.int16)
                self._speech_buf = []
                self._recording = False
                self._silence_count = 0
                self._speech_count = 0
                self._run_asr_async(pcm_int16)

    # ── ASR（在独立线程执行）───────────────────────────────────────────────

    def _run_asr_async(self, pcm_int16: np.ndarray):
        with self._asr_lock:
            if self._asr_busy:
                self.get_logger().warn("ASR 正忙，跳过本次识别")
                return
            self._asr_busy = True

        t = threading.Thread(target=self._run_asr, args=(pcm_int16,), daemon=True)
        t.start()

    def _run_asr(self, pcm_int16: np.ndarray):
        try:
            text = self._baidu_asr(pcm_int16)
            if not text or len(text) < 2:
                self.get_logger().debug("识别结果过短，忽略")
                return
            self.get_logger().info(f"识别结果: 「{text}」")
            self._match_and_respond(text)
        except Exception as e:
            self.get_logger().error(f"ASR 异常: {e}")
        finally:
            with self._asr_lock:
                self._asr_busy = False

    def _baidu_asr(self, pcm_int16: np.ndarray) -> str:
        speech_bytes = pcm_int16.tobytes()
        # 过滤过短音频 (< 0.5s，16000 采样点 * 2 字节)
        if len(speech_bytes) < 16000:
            return ""

        speech_b64 = base64.b64encode(speech_bytes).decode("utf-8")

        payload = {
            "format": "pcm",
            "rate": 16000,
            "channel": 1,
            "cuid": BAIDU_CUID,
            "token": self.baidu_token,
            "dev_pid": BAIDU_DEV_PID,
            "speech": speech_b64,
            "len": len(speech_bytes),
        }

        url = "https://vop.baidu.com/server_api"
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        data_str = json.dumps(payload, ensure_ascii=False)
        resp = requests.post(url, headers=headers, data=data_str.encode("utf-8"), timeout=10)
        data = resp.json()

        err_no = data.get("err_no")
        if err_no == 0:
            result = data.get("result", [])
            return "".join(result).strip() if result else ""
        elif err_no == 3301:
            self.get_logger().debug("百度 ASR: 音频质量不佳(3301)，忽略")
            return ""
        elif err_no == 3302:  # token 过期
            self.get_logger().warn("百度 token 过期，尝试刷新...")
            self.baidu_token = self._fetch_baidu_token()
            payload["token"] = self.baidu_token
            data_str = json.dumps(payload, ensure_ascii=False)
            resp = requests.post(url, headers=headers, data=data_str.encode("utf-8"), timeout=10)
            data = resp.json()
            if data.get("err_no") == 0:
                result = data.get("result", [])
                return "".join(result).strip() if result else ""
        raise RuntimeError(f"百度 ASR 错误: err_no={err_no}, err_msg={data.get('err_msg')}")

    # ── 意图匹配 ────────────────────────────────────────────────────────────

    def _match_and_respond(self, text: str):
        lower = text.lower().replace(" ", "")

        # 1) 优先检查 IP 查询意图
        ip_keywords = ["ip是多少", "ip地址", "我的ip", "ip号", "ip多少"]
        if any(kw.lower() in lower for kw in ip_keywords):
            ip = self._get_ip()
            ip_spoken = ip.replace(".", "点")
            self._play_tts(f"您好，本机IP地址是 {ip_spoken}")
            return

        # 2) 唤醒词过滤：包含唤醒词才进入 AI 问答
        if not any(kw.lower() in lower for kw in WAKE_KEYWORDS):
            self.get_logger().debug(f"未命中唤醒词，忽略: {text}")
            return

        # 去掉唤醒词本身，只保留问题内容
        question = text
        for kw in WAKE_KEYWORDS:
            question = question.replace(kw, "").replace(kw.lower(), "")
        question = question.strip("，,。. \t")
        if not question:
            self._play_tts("我在，请问有什么可以帮您的？")
            return

        self.get_logger().info(f"[AI 提问] {question}")
        self._run_ai_async(question)

    # ── AI 问答（在独立线程执行）─────────────────────────────────────────────

    def _get_ip(self) -> str:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "无法获取"

    def _run_ai_async(self, question: str):
        t = threading.Thread(target=self._run_ai, args=(question,), daemon=True)
        t.start()

    def _run_ai(self, question: str):
        try:
            response = self.ai_client.messages.create(
                model=AI_MODEL,
                max_tokens=512,
                system=AI_SYSTEM_PROMPT,
                messages=[
                    {"role": "user", "content": question},
                ],
                timeout=30.0,
            )
            answer = ""
            for block in response.content:
                if block.type == "text":
                    answer += block.text
            answer = answer.strip()
            if answer:
                self.get_logger().info(f"[AI 回答] {answer}")
                self._play_tts(answer)
            else:
                self.get_logger().warn("AI 返回空内容")
                self._play_tts("抱歉，我没有听清楚，能请您再说一遍吗？")
        except Exception as e:
            self.get_logger().error(f"AI 调用异常: {e}")
            self._play_tts("抱歉，我的网络出了点问题，请稍后再试。")

    # ── TTS 播报 ─────────────────────────────────────────────────────────────

    def _play_tts(self, text: str):
        if not self.tts_client.service_is_ready():
            self.get_logger().warn("TTS 服务不可用，跳过播报")
            return

        # 按字数估算播报时长（约 4 字/秒），多留 1.5s 余量
        estimated_sec = len(text) / 4.0 + 1.5
        self._tts_mute_until = time.monotonic() + estimated_sec
        self.get_logger().info(f"屏蔽麦克风 {estimated_sec:.1f}s")

        req = PlayTts.Request()
        req.tts_req.text = text
        req.tts_req.domain = "ai_voice_chat"
        req.tts_req.trace_id = "ai_chat"
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
        node = AIVoiceChatNode()
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
