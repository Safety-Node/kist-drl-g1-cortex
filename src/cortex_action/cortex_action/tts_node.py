"""tts_node — Text-to-Speech [REQ-29]. Port of the workstation TTSProvider.

    SpeakCommand (/cortex/tts/say)  -> Clova REST -> resample 16k -> AudioPCM
                                                                     (/bridge/cmd/audio_out)

Synthesis runs on a worker thread (one asyncio.run per call) so the executor never
blocks on the cloud round-trip. Echoes command_id in a CommandStatus stream
(/cortex/tts/status) — ACCEPTED → EXECUTING → SUCCEEDED | CANCELED | ABORTED — so
the orchestrator registry tracks speech like any other action.

PC resamples Clova 24 kHz → 16 kHz (REQ-29). Echo-cancel is passive: NX
speaker_node raises SpeakerState.playing, stt_node mutes on it. Credentials
NCP_CLOVA_CLIENT_ID / _SECRET (env); missing → warn + drop, not crash.
"""

import asyncio
import io
import logging
import os
import wave
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from cortex_msgs.msg import CommandStatus, SpeakCommand
from g1_onboard_msgs.msg import AudioPCM, EstopFlag

# CommandStatus states this node actually reaches. CANCELING is skipped —
# asyncio cancel is effectively instant, there is no wind-down window.
_TERMINAL = (CommandStatus.IDLE, CommandStatus.CANCELED,
             CommandStatus.SUCCEEDED, CommandStatus.ABORTED)


# ===========================================================================
# Config
# ===========================================================================
class TTSBackend(str, Enum):
    NAVER_CLOVA = 'naver_clova'


@dataclass
class TTSConfig:
    """Runtime configuration (populated from ROS parameters)."""

    backend: TTSBackend = TTSBackend.NAVER_CLOVA
    language_code: str = 'ko-KR'          # KIST demo speaks Korean
    sample_rate_hz: int = 16000           # wire format: AudioPCM locked rate
    # Clova WAV request rate. Premium WAV accepts [8000, 16000, 24000, 48000];
    # request a high-quality native rate and resample on the PC (REQ-29).
    clova_sample_rate_hz: int = 24000
    voice: str = 'nara'                   # see api.ncloud-docs.com (vendor list is long)
    speed: int = 0                        # Clova [-5, +5] speed offset
    naver_api_url: str = 'https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts'
    client_id_env: str = 'NCP_CLOVA_CLIENT_ID'
    client_secret_env: str = 'NCP_CLOVA_CLIENT_SECRET'
    # On timeout: log + drop, no retry — a late audio cue would desync the flow,
    # and TTS failure does not fail a sub-task.
    request_timeout_s: float = 5.0


# ===========================================================================
# Node
# ===========================================================================
class TtsNode(Node):
    # AudioPCM per-message payload limit is 65500 B; 32000 B ~= 1.0s @ 16kHz mono
    # int16 leaves headroom. NX speaker_node consumes a chunk queue and reports
    # progress via SpeakerState.current_chunk_id / queue_depth — one utterance
    # per message is explicitly NOT the design.
    _PUBLISH_CHUNK_BYTES = 32000

    def __init__(self) -> None:
        super().__init__('tts_node')

        # --- parameters -> TTSConfig --------------------------------------
        self.declare_parameter('say_topic', '/cortex/tts/say')
        self.declare_parameter('barge_in_topic', '/cortex/tts/stop')
        self.declare_parameter('status_topic', '/cortex/tts/status')
        self.declare_parameter('status_rate_hz', 20.0)   # CommandStatus heartbeat
        self.declare_parameter('audio_out_topic', '/bridge/cmd/audio_out')
        self.declare_parameter('estop_topic', '/bridge/safety/estop')
        self.declare_parameter('backend', TTSBackend.NAVER_CLOVA.value)
        self.declare_parameter('language_code', 'ko-KR')
        self.declare_parameter('sample_rate_hz', 16000)
        self.declare_parameter('clova_sample_rate_hz', 24000)
        self.declare_parameter('voice', 'nara')
        self.declare_parameter('speed', 0)
        self.declare_parameter('request_timeout_s', 5.0)

        g = self.get_parameter
        self._config = TTSConfig(
            backend=TTSBackend(g('backend').value),
            language_code=g('language_code').value,
            sample_rate_hz=int(g('sample_rate_hz').value),
            clova_sample_rate_hz=int(g('clova_sample_rate_hz').value),
            voice=g('voice').value,
            speed=int(g('speed').value),
            request_timeout_s=float(g('request_timeout_s').value),
        )

        # --- state ---------------------------------------------------------
        self._estop_active = False
        self._inflight_task: Optional[asyncio.Task] = None
        self._inflight_loop: Optional[asyncio.AbstractEventLoop] = None
        # CommandStatus the orchestrator registry reads. A terminal state persists
        # (level-triggered) until the next SpeakCommand replaces it, so a dropped
        # transition is self-healing. Guarded because it is set from the worker
        # thread and read/published from the executor.
        self._status_lock = threading.Lock()
        self._cmd_id = 0
        self._state = CommandStatus.IDLE

        self._client_id = os.environ.get(self._config.client_id_env)
        self._client_secret = os.environ.get(self._config.client_secret_env)
        if not self._client_id or not self._client_secret:
            # Graceful degrade rather than fail-fast, so the node still runs in a
            # scaffold/demo-dry setup. synthesize() logs + drops.
            self.get_logger().warning(
                f'{self._config.client_id_env} / {self._config.client_secret_env} not set '
                '— synthesize() will log + drop until credentials are present')

        # --- io -------------------------------------------------------------
        grp = ReentrantCallbackGroup()
        self.audio_pub = self.create_publisher(AudioPCM, g('audio_out_topic').value, 10)
        # Status is a latched, reliable stream — same contract the orchestrator
        # asks of the Gearsonic Handler, so the registry treats speech uniformly.
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            CommandStatus, g('status_topic').value, latched)

        self.create_subscription(
            SpeakCommand, g('say_topic').value, self._on_say, 10, callback_group=grp)
        self.create_subscription(
            Bool, g('barge_in_topic').value, self._on_barge_in, 10, callback_group=grp)
        self.create_subscription(
            EstopFlag, g('estop_topic').value, self._on_estop_msg, 10, callback_group=grp)

        # Heartbeat: republish current status so `EXECUTING` during a multi-second
        # Clova round-trip keeps the registry's staleness clock fresh.
        self.create_timer(1.0 / float(g('status_rate_hz').value),
                          self._publish_status, callback_group=grp)

        # Synthesis runs here, off the executor thread.
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='tts')

        self.get_logger().info(
            f"tts_node up (backend={self._config.backend.value}, voice={self._config.voice}, "
            f"say={g('say_topic').value} -> {g('audio_out_topic').value}, "
            f"status={g('status_topic').value})")

    def destroy_node(self) -> bool:
        self.cancel()
        self._pool.shutdown(wait=False)
        return super().destroy_node()

    # --- status (CommandStatus stream for the orchestrator registry) ------
    def _set_status(self, cmd_id: int, state: int) -> None:
        with self._status_lock:
            self._cmd_id, self._state = cmd_id, state
        self._publish_status()

    def _publish_status(self) -> None:
        # Published on transition AND on the heartbeat timer: a non-terminal state
        # keeps the registry's staleness clock fresh, a terminal state persists
        # (level-triggered) until the next command.
        with self._status_lock:
            cmd_id, state = self._cmd_id, self._state
        msg = CommandStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_id = cmd_id
        msg.state = state
        self.status_pub.publish(msg)

    # --- subscriptions ----------------------------------------------------
    def _on_say(self, msg: SpeakCommand) -> None:
        """Hand the text to the worker; never block the executor."""
        self._set_status(msg.command_id, CommandStatus.ACCEPTED)
        self._pool.submit(self._synth_worker, msg.command_id, msg.text)

    def _on_barge_in(self, msg: Bool) -> None:
        """Barge-in: user interrupted, cut playback synthesis immediately."""
        if msg.data:
            self.get_logger().info('barge-in: cancelling in-flight synthesis')
            self.cancel()

    def _on_estop_msg(self, msg: EstopFlag) -> None:
        """E-STOP: cancel current synth; gate future say requests.

        Clearing E-STOP does NOT auto-resume the killed utterance (a G1 staying
        audible during E-STOP would confuse the operator) — it just re-allows
        future synthesize() calls.
        """
        self._estop_active = bool(msg.active)
        self.get_logger().info(
            f"E-STOP {'ACTIVE' if self._estop_active else 'CLEARED'} (reason={msg.reason})")
        if self._estop_active:
            self.cancel()

    # --- synthesis --------------------------------------------------------
    def _synth_worker(self, command_id: int, text: str) -> None:
        """One asyncio.run per call — matches the workstation's per-call loop."""
        self._set_status(command_id, CommandStatus.EXECUTING)
        try:
            asyncio.run(self.synthesize(command_id, text))
        except Exception:  # noqa: BLE001 - a failed announcement must not kill the worker
            self.get_logger().exception(f'synthesis worker failed for {text!r}')
            self._set_status(command_id, CommandStatus.ABORTED)

    async def synthesize(self, command_id: int, text: str) -> None:
        """Synthesize ``text`` and publish the PCM stream (fire-and-forget).

        Gate on E-STOP, POST to Clova, decode WAV, resample to the wire rate,
        publish. Sets the terminal CommandStatus (SUCCEEDED / CANCELED / ABORTED)
        so the orchestrator registry knows the command stopped.
        """
        if self._estop_active:
            self.get_logger().info(f'synthesize: E-STOP active — aborting (text={text!r})')
            self._set_status(command_id, CommandStatus.CANCELED)
            return
        text = (text or '').strip()
        if not text or not self._client_id or not self._client_secret:
            self.get_logger().warning(f'synthesize: nothing to do — dropping (text={text!r})')
            self._set_status(command_id, CommandStatus.ABORTED)
            return

        self._inflight_task = asyncio.current_task()
        self._inflight_loop = asyncio.get_running_loop()
        try:
            wav_bytes = await self._http_post_clova(text)
            if wav_bytes is None:
                self._set_status(command_id, CommandStatus.ABORTED)
                return
            # E-STOP / barge-in may have fired during the network round-trip.
            if self._estop_active:
                self.get_logger().info('synthesize: E-STOP fired mid-request — dropping audio')
                self._set_status(command_id, CommandStatus.CANCELED)
                return
            pcm, src_rate, channels = self._decode_wav(wav_bytes)
            if pcm is None:
                self._set_status(command_id, CommandStatus.ABORTED)
                return
            self._publish(self._resample_to_wire(pcm, src_rate, channels))
            self._set_status(command_id, CommandStatus.SUCCEEDED)
        except asyncio.CancelledError:
            self.get_logger().info('synthesize: cancelled (barge-in / E-STOP / stop)')
            self._set_status(command_id, CommandStatus.CANCELED)
            raise
        except Exception:  # noqa: BLE001 - log + drop, no retry (TTSConfig policy)
            self.get_logger().exception(f'synthesize: failed; dropping (text={text!r})')
            self._set_status(command_id, CommandStatus.ABORTED)
        finally:
            self._inflight_task = None
            self._inflight_loop = None

    def cancel(self) -> None:
        """Cancel any in-flight synthesis (barge-in / E-STOP / shutdown)."""
        task, loop = self._inflight_task, self._inflight_loop
        if task is None or loop is None:
            return
        # cancel() runs on an executor/DDS thread, not the synthesis loop thread.
        try:
            loop.call_soon_threadsafe(task.cancel)
        except RuntimeError:
            pass        # loop already closed — nothing to do

    @property
    def is_synthesizing(self) -> bool:
        """True while a request is in flight to the cloud TTS.

        For "is the NX speaker actually emitting sound right now?", read
        SpeakerState.playing instead — that flag is raised by NX speaker_node
        based on actual playback, not PC-side synthesis status.
        """
        return self._inflight_task is not None

    # --- backend: Naver Clova Voice Premium REST --------------------------
    async def _http_post_clova(self, text: str) -> Optional[bytes]:
        """POST ``text`` to Clova /tts; return raw WAV bytes (or None on failure)."""
        import aiohttp

        headers = {
            'X-NCP-APIGW-API-KEY-ID': self._client_id,
            'X-NCP-APIGW-API-KEY': self._client_secret,
            'Content-Type': 'application/x-www-form-urlencoded',
        }
        body = {
            'speaker': self._config.voice,
            'text': text,
            'format': 'wav',
            'speed': self._config.speed,
            'sampling-rate': self._config.clova_sample_rate_hz,
        }
        timeout = aiohttp.ClientTimeout(total=self._config.request_timeout_s)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(self._config.naver_api_url, headers=headers,
                                    data=body) as resp:
                if resp.status != 200:
                    detail = await resp.text()
                    self.get_logger().error(f'Clova HTTP {resp.status} — {detail[:200]}')
                    return None
                return await resp.read()

    # --- WAV decode + resample (PC resample responsibility, REQ-29) -------
    @staticmethod
    def _decode_wav(wav_bytes: bytes):
        """Return (pcm_int16_bytes, framerate, channels) from a WAV payload."""
        try:
            with io.BytesIO(wav_bytes) as buf, wave.open(buf, 'rb') as wf:
                channels = wf.getnchannels()
                framerate = wf.getframerate()
                sampwidth = wf.getsampwidth()
                frames = wf.readframes(wf.getnframes())
            if sampwidth != 2:
                logging.error('unexpected WAV sample width %d (need 16-bit)', sampwidth)
                return None, 0, 0
            return frames, framerate, channels
        except Exception:  # noqa: BLE001
            logging.exception('WAV decode failed')
            return None, 0, 0

    def _resample_to_wire(self, pcm: bytes, src_rate: int, channels: int) -> bytes:
        """Downmix to mono and resample ``pcm`` (int16) to ``sample_rate_hz``."""
        samples = np.frombuffer(pcm, dtype=np.int16)
        if channels == 2:
            samples = samples.reshape(-1, 2).mean(axis=1)   # interleaved L/R -> mono
        samples = samples.astype(np.float32)

        dst_rate = self._config.sample_rate_hz
        if src_rate == dst_rate or samples.size == 0:
            return samples.astype(np.int16).tobytes()

        n_dst = int(round(samples.size * dst_rate / src_rate))
        if n_dst <= 0:
            return b''
        # Linear interpolation resample (adequate for speech TTS playback).
        x_old = np.linspace(0.0, 1.0, samples.size, endpoint=False)
        x_new = np.linspace(0.0, 1.0, n_dst, endpoint=False)
        return np.interp(x_new, x_old, samples).astype(np.int16).tobytes()

    # --- publish ----------------------------------------------------------
    def _publish(self, pcm: bytes) -> None:
        """Publish 16 kHz mono int16 PCM; chunk to fit the AudioPCM payload limit."""
        if not pcm:
            return
        n = 0
        for i in range(0, len(pcm), self._PUBLISH_CHUNK_BYTES):
            msg = AudioPCM()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.sample_rate = AudioPCM.SAMPLE_RATE
            msg.channels = AudioPCM.CHANNELS
            msg.bit_depth = AudioPCM.BIT_DEPTH
            msg.data = list(pcm[i:i + self._PUBLISH_CHUNK_BYTES])
            self.audio_pub.publish(msg)
            n += 1
        self.get_logger().info(f'published {len(pcm)} bytes in {n} chunk(s)')


def main(args=None) -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    node = TtsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
