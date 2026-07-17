"""stt_node — streaming Speech-to-Text [REQ-27].

Port of the workstation STTProvider (TASK-42) to rclpy. The DSP, echo-cancel and
Google streaming logic are carried over unchanged; only the transport swapped:
UnitreeG1Provider push callbacks -> ROS subscriptions.

    AudioPCM  (/bridge/sensors/audio_pcm)  -> filter -> queue -> backend worker
                                                                      |
    std_msgs/String (/cortex/stt/transcript) <---------- TranscriptEvent

Echo cancellation: mic input is dropped while SpeakerState.playing is true, with
an ``echo_cancel_tail_ms`` tail-off after it clears. Silent PCM of equal length is
injected instead so Google's idle timeout does not kill the stream. No active
coordination with tts_node is needed — NX speaker_node owns the playing flag.
``echo_cancel_lead_ms`` (default 0) can pre-mute before the DDS hop lands; it is
the only path that would need a tts_node -> stt_node signal, and it is off.

One module by convention (see task_srv_provider.py: no pytest infra, so the
modularity payoff is nil). Sections: filter · types/config · node.
"""

import base64
import json
import logging
import os
import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.signal import butter, sosfilt
from std_msgs.msg import String

from g1_onboard_msgs.msg import AudioPCM, EstopFlag, SpeakerState

_MAX_RECONNECT = 10


def _load_google_credentials() -> Optional[Any]:
    """Return google.oauth2 Credentials from GOOGLE_APPLICATION_CREDENTIALS_B64,
    or None to let the SDK auto-discover via GOOGLE_APPLICATION_CREDENTIALS path.
    """
    b64 = os.environ.get('GOOGLE_APPLICATION_CREDENTIALS_B64')
    if not b64:
        return None
    from google.oauth2 import service_account
    info = json.loads(base64.b64decode(b64))
    return service_account.Credentials.from_service_account_info(info)


# ===========================================================================
# Speech-band filter
# ===========================================================================
class StreamingSpeechFilter:
    """청크 경계를 넘어 zi 상태를 유지하는 실시간 IIR 대역 필터.

    HPF(Butterworth 4차) + LPF(Butterworth 6차) 직렬 구성. 저주파 진동·DC 및
    광대역 노이즈를 제거하고 음성 대역을 보존한다.
    """

    def __init__(self, sample_rate: int = 16000, highpass_hz: float = 120.0,
                 lowpass_hz: float = 3800.0) -> None:
        hp_sos = butter(4, highpass_hz, btype='highpass', fs=sample_rate, output='sos')
        lp_sos = butter(6, lowpass_hz, btype='lowpass', fs=sample_rate, output='sos')
        self._sos = np.vstack([hp_sos, lp_sos])
        self._zi = np.zeros((self._sos.shape[0], 2), dtype=np.float64)

    def process(self, samples: np.ndarray, gain_linear: float = 1.0) -> np.ndarray:
        y, self._zi = sosfilt(self._sos, samples.astype(np.float64), zi=self._zi)
        return np.clip(np.rint(y * gain_linear), -32768, 32767).astype(np.int16)


# ===========================================================================
# Types + config
# ===========================================================================
class STTBackend(str, Enum):
    GOOGLE_CLOUD = 'google_cloud'
    DUMMY = 'dummy'       # local verification; no credentials needed


class STTState(str, Enum):
    """Connection state, published for GUI display.

    start()           : IDLE -> CONNECTING -> STREAMING
    gRPC stream error : STREAMING -> RECONNECTING -> STREAMING (or FAILED)
    stop()            : any -> IDLE
    """

    IDLE = 'idle'
    CONNECTING = 'connecting'
    STREAMING = 'streaming'
    RECONNECTING = 'reconnecting'
    FAILED = 'failed'


@dataclass
class TranscriptEvent:
    text: str
    ts: float                            # monotonic seconds
    is_final: bool = True
    confidence: Optional[float] = None   # backend-specific; None if not reported


@dataclass
class STTConfig:
    """Runtime configuration (populated from ROS parameters)."""

    backend: STTBackend = STTBackend.GOOGLE_CLOUD
    language_code: str = 'ko-KR'
    sample_rate_hz: int = 16000          # AudioPCM.SAMPLE_RATE (locked format)
    # interim_results filtering is this node's sole responsibility: when False,
    # only is_final=True events are published. Downstream does not re-filter.
    interim_results: bool = False
    # Speech band IIR filter (HPF + LPF).
    highpass_hz: float = 120.0   # Hz — removes low-freq vibration / DC
    lowpass_hz: float = 5500.0   # Hz — preserves Korean fricatives (ㅅ/ㅎ/ㅊ up to ~6kHz)
    # Input gain applied after filtering. +6 dB ~= x2 amplitude. Clips to int16.
    input_gain_db: float = 0.0   # gain 없음 — 노이즈 환경에서 증폭은 역효과
    # Drop mic input N ms AFTER SpeakerState.playing clears (trailing echo).
    echo_cancel_tail_ms: int = 200
    # Drop mic input N ms BEFORE playing rises: the DDS speaker_state hop trails
    # actual audio by ~50-100 ms. Default 0 — the right value depends on the
    # observed LAN latency, and it needs a tts->stt onset signal to be useful.
    echo_cancel_lead_ms: int = 0


# ===========================================================================
# Node
# ===========================================================================
class SttNode(Node):
    def __init__(self) -> None:
        super().__init__('stt_node')

        # --- parameters -> STTConfig --------------------------------------
        self.declare_parameter('audio_topic', '/bridge/sensors/audio_pcm')
        self.declare_parameter('transcript_topic', '/cortex/stt/transcript')
        self.declare_parameter('speaker_state_topic', '/bridge/sensors/speaker_state')
        self.declare_parameter('estop_topic', '/bridge/safety/estop')
        self.declare_parameter('backend', STTBackend.GOOGLE_CLOUD.value)
        self.declare_parameter('language_code', 'ko-KR')
        self.declare_parameter('sample_rate_hz', 16000)
        self.declare_parameter('interim_results', False)
        self.declare_parameter('highpass_hz', 120.0)
        self.declare_parameter('lowpass_hz', 5500.0)
        self.declare_parameter('input_gain_db', 0.0)
        self.declare_parameter('echo_cancel_tail_ms', 200)
        self.declare_parameter('echo_cancel_lead_ms', 0)

        g = self.get_parameter
        self._config = STTConfig(
            backend=STTBackend(g('backend').value),
            language_code=g('language_code').value,
            sample_rate_hz=int(g('sample_rate_hz').value),
            interim_results=bool(g('interim_results').value),
            highpass_hz=float(g('highpass_hz').value),
            lowpass_hz=float(g('lowpass_hz').value),
            input_gain_db=float(g('input_gain_db').value),
            echo_cancel_tail_ms=int(g('echo_cancel_tail_ms').value),
            echo_cancel_lead_ms=int(g('echo_cancel_lead_ms').value),
        )

        # --- state ---------------------------------------------------------
        self._state = STTState.IDLE      # running == (state != IDLE)
        self._estop_active: bool = False
        self._speaker_playing: bool = False   # cached from SpeakerState
        self._echo_tail_end: Optional[float] = None   # monotonic tail deadline
        self._lead_mute_end: Optional[float] = None   # monotonic lead deadline

        self._speech_filter = StreamingSpeechFilter(
            sample_rate=self._config.sample_rate_hz,
            highpass_hz=self._config.highpass_hz,
            lowpass_hz=self._config.lowpass_hz,
        )
        self._gain_linear = 10.0 ** (self._config.input_gain_db / 20.0)

        self._audio_queue: Optional[queue.Queue] = None
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # --- io -------------------------------------------------------------
        grp = ReentrantCallbackGroup()
        self.pub = self.create_publisher(String, g('transcript_topic').value, 10)
        self.create_subscription(
            AudioPCM, g('audio_topic').value, self._on_audio_msg, 10, callback_group=grp)
        self.create_subscription(
            SpeakerState, g('speaker_state_topic').value, self._on_speaker_state, 10,
            callback_group=grp)
        self.create_subscription(
            EstopFlag, g('estop_topic').value, self._on_estop_msg, 10, callback_group=grp)

        self._start_backend()
        self.get_logger().info(
            f"stt_node up (backend={self._config.backend.value}, "
            f"lang={self._config.language_code}, rate={self._config.sample_rate_hz}, "
            f"audio={g('audio_topic').value} -> {g('transcript_topic').value})")

    # --- lifecycle --------------------------------------------------------
    def _start_backend(self) -> None:
        self._state = STTState.CONNECTING
        self._stop_event.clear()
        self._audio_queue = queue.Queue(maxsize=200)

        if self._config.backend == STTBackend.GOOGLE_CLOUD:
            target, name = self._google_worker, 'stt_google_worker'
        elif self._config.backend == STTBackend.DUMMY:
            target, name = self._dummy_worker, 'stt_dummy_worker'
        else:
            self._state = STTState.IDLE
            raise ValueError(f'stt_node: unknown backend {self._config.backend!r}')

        self._worker_thread = threading.Thread(target=target, name=name, daemon=True)
        self._worker_thread.start()
        self._state = STTState.STREAMING

    def destroy_node(self) -> bool:
        """Stop the backend worker before tearing the node down."""
        if self._state != STTState.IDLE:
            self._stop_event.set()
            if self._audio_queue is not None:
                try:
                    self._audio_queue.put_nowait(None)      # poison pill
                except queue.Full:
                    pass
            if self._worker_thread is not None:
                self._worker_thread.join(timeout=5.0)
                if self._worker_thread.is_alive():
                    self.get_logger().warning('worker thread did not stop within 5s')
                self._worker_thread = None
            self._audio_queue = None
            self._state = STTState.IDLE
        return super().destroy_node()

    @property
    def state(self) -> STTState:
        return self._state

    # --- subscriptions ----------------------------------------------------
    def _on_speaker_state(self, msg: SpeakerState) -> None:
        self._speaker_playing = bool(msg.playing)

    def _on_estop_msg(self, msg: EstopFlag) -> None:
        self._estop_active = bool(msg.active)
        self.get_logger().info(
            f"E-STOP {'ACTIVE' if self._estop_active else 'CLEARED'} (reason={msg.reason})")

    def _on_audio_msg(self, msg: AudioPCM) -> None:
        """AudioPCM -> the provider's chunk path. ts is monotonic-at-receipt so it
        is comparable with the echo-mute deadlines."""
        self._on_audio_chunk(bytes(msg.data), time.monotonic())

    # --- audio path (ported unchanged) ------------------------------------
    def _on_audio_chunk(self, pcm: bytes, ts: float) -> None:
        """Drop while E-STOP or echo-muted; forward to backend queue otherwise."""
        # 1) E-STOP gate — hard block; no silence injection needed
        if self._estop_active:
            return

        # 2) Echo-cancel gate (speaker playing + tail-off + leading edge)
        if self._check_echo_mute(ts):
            # Inject silence of identical length so Google's idle timeout does
            # not terminate the stream during long TTS playback.
            if self._audio_queue is not None:
                try:
                    self._audio_queue.put_nowait(bytes(len(pcm)))
                except queue.Full:
                    pass
            return

        # 3) Apply speech filter then feed to backend queue
        if self._audio_queue is not None:
            try:
                samples = np.frombuffer(pcm, dtype=np.int16)
                filtered = self._speech_filter.process(samples, self._gain_linear)
                self._audio_queue.put_nowait(filtered.tobytes())
            except queue.Full:
                self.get_logger().debug('audio queue full, dropping chunk')

    def _check_echo_mute(self, ts: float) -> bool:
        """True if the mic should be muted at audio timestamp ``ts``."""
        # Leading-edge mute (notify_tts_onset equivalent; disabled at lead_ms=0)
        if self._lead_mute_end is not None and ts < self._lead_mute_end:
            return True

        if self._speaker_playing:
            # Extend tail deadline while speaker is active
            self._echo_tail_end = ts + self._config.echo_cancel_tail_ms / 1000.0
            return True

        # Tail-off window
        if self._echo_tail_end is not None and ts < self._echo_tail_end:
            return True

        return False

    def notify_tts_onset(self) -> None:
        """Pre-mute the mic for ``echo_cancel_lead_ms`` starting now.

        Kept from the provider port. Unused while echo_cancel_lead_ms=0; wiring it
        cross-node would need a tts_node -> stt_node onset topic.
        """
        if self._config.echo_cancel_lead_ms > 0:
            self._lead_mute_end = time.monotonic() + self._config.echo_cancel_lead_ms / 1000.0

    # --- emit -------------------------------------------------------------
    def _emit_transcript(self, event: TranscriptEvent) -> None:
        """Publish the transcript. Called from the worker thread."""
        self.pub.publish(String(data=event.text))
        self.get_logger().info(
            f'transcript: {event.text!r} (final={event.is_final}, conf={event.confidence})')

    def _drain_audio_queue(self) -> int:
        """재연결 전 쌓인 stale 오디오를 모두 버린다. 버린 청크 수를 반환."""
        dropped = 0
        if self._audio_queue is None:
            return dropped
        while True:
            try:
                self._audio_queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        return dropped

    # --- Google Cloud STT backend -----------------------------------------
    def _google_request_gen(self):
        """Yield StreamingRecognizeRequest objects consumed from the audio queue."""
        try:
            from google.cloud import speech
        except ImportError:
            self.get_logger().error(
                'google-cloud-speech not installed — pip install -r requirements.txt')
            return

        while not self._stop_event.is_set():
            try:
                chunk = self._audio_queue.get(timeout=0.02)
            except queue.Empty:
                continue
            if chunk is None:  # poison pill
                break
            yield speech.StreamingRecognizeRequest(audio_content=chunk)

    def _google_worker(self) -> None:
        """Auto-reconnect loop for Google Cloud bidi gRPC stream (~5 min limit)."""
        try:
            from google.cloud import speech
        except ImportError:
            self.get_logger().error(
                'google-cloud-speech not installed — pip install -r requirements.txt')
            self._state = STTState.FAILED
            return

        # SpeechClient 한 번만 생성 — gRPC 채널 재사용. 에러로 client 자체가
        # 망가진 경우에만 재생성.
        credentials = _load_google_credentials()

        def _new_client():
            return speech.SpeechClient(credentials=credentials) if credentials \
                else speech.SpeechClient()

        client = _new_client()
        recognition_config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self._config.sample_rate_hz,
            language_code=self._config.language_code,
            enable_automatic_punctuation=True,
        )
        streaming_config = speech.StreamingRecognitionConfig(
            config=recognition_config,
            interim_results=self._config.interim_results,
        )

        reconnect_count = 0
        backoff = 1.0

        while not self._stop_event.is_set():
            if reconnect_count > _MAX_RECONNECT:
                self._state = STTState.FAILED
                self.get_logger().error(
                    f'max reconnect attempts ({_MAX_RECONNECT}) exhausted -> FAILED')
                break

            try:
                if reconnect_count == 0:
                    self.get_logger().info('Google streaming session started')
                else:
                    self._state = STTState.RECONNECTING
                    self.get_logger().info(
                        f'Google reconnecting (attempt {reconnect_count}/{_MAX_RECONNECT})')

                responses = client.streaming_recognize(
                    config=streaming_config,
                    requests=self._google_request_gen(),
                )
                self._state = STTState.STREAMING

                for response in responses:
                    if self._stop_event.is_set():
                        return
                    for result in response.results:
                        if not result.alternatives:
                            continue
                        alt = result.alternatives[0]
                        if result.is_final or self._config.interim_results:
                            confidence = alt.confidence if alt.confidence > 0 else None
                            self._emit_transcript(TranscriptEvent(
                                text=alt.transcript,
                                ts=time.monotonic(),
                                is_final=result.is_final,
                                confidence=confidence,
                            ))

                # Stream ended normally (single_utterance or ~5 min rotation)
                if not self._stop_event.is_set():
                    dropped = self._drain_audio_queue()
                    self.get_logger().info(
                        f'Google stream ended, restarting (flushed {dropped} stale chunks)')
                    reconnect_count = 0
                    backoff = 1.0

            except Exception as exc:  # noqa: BLE001 - worker must never die
                if self._stop_event.is_set():
                    break
                self._drain_audio_queue()
                reconnect_count += 1
                self.get_logger().warning(
                    f'Google stream error ({exc}) — retry {reconnect_count}/'
                    f'{_MAX_RECONNECT} in {backoff:.1f}s')
                self._stop_event.wait(timeout=backoff)
                backoff = min(backoff * 2, 30.0)
                # gRPC 채널 수준 에러면 client 재생성
                try:
                    client = _new_client()
                except Exception:  # noqa: BLE001
                    pass

        if self._state != STTState.FAILED:
            self._state = STTState.IDLE
        self.get_logger().info('Google worker stopped')

    # --- DUMMY backend (local verification — no credentials) --------------
    def _dummy_worker(self) -> None:
        """Decode non-silent PCM chunks as UTF-8 and emit TranscriptEvents.

        Exercise scripts feed text bytes instead of real PCM so the filter chain
        runs without a live mic or cloud credentials. All-zero chunks (silence
        injected by echo-cancel) are dropped.
        """
        self.get_logger().info('DUMMY worker started')
        while not self._stop_event.is_set():
            try:
                chunk = self._audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if chunk is None:  # poison pill
                break
            if not any(chunk):          # echo-cancel silence injection
                continue
            try:
                text = chunk.decode('utf-8').strip()
            except UnicodeDecodeError:
                continue
            if not text:
                continue
            self._emit_transcript(TranscriptEvent(
                text=text, ts=time.monotonic(), is_final=True, confidence=None))
        self.get_logger().info('DUMMY worker stopped')


def main(args=None) -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    node = SttNode()
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
