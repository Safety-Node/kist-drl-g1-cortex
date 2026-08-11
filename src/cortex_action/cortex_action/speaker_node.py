"""
speaker_node — TTS playback on the robot speaker (cortex/PC side).

Subscribes  /bridge/cmd/audio_out      (g1_onboard_msgs/AudioPCM)  <- tts_node
Publishes   /bridge/audio/speaker_state (g1_onboard_msgs/SpeakerState) -> stt_node
            (echo-cancel hint: stt mutes its mic while playing == true)

Playback is via the Unitree SDK AudioClient.PlayStream (a DDS RPC to the robot's
audio service — NOT raw ALSA), so this must run on a host that shares the robot's
DDS domain. Ported from the (archived) onboard speaker_node; only the topics and
package changed — the robot speaker is Unitree hardware, so the playback code is
the same regardless of which host drives it.

Pipeline LOCKED: 16 kHz / 16-bit / mono. Non-conforming AudioPCM is dropped.

Requires the `unitree_sdk2py` Python package (not on PyPI — install from the
Unitree SDK). Domain/interface come from parameters; the ChannelFactory patch
joins rclpy's already-active cyclonedds domain instead of creating a new one.
"""
import threading
import time
from collections import deque
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from g1_onboard_msgs.msg import AudioPCM, SpeakerState

import unitree_sdk2py.core.channel as _sdk_ch
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def _patch_channel_factory() -> None:
    """Join the already-active cyclonedds domain instead of creating one.

    rmw_cyclonedds_cpp (rclpy) and unitree_sdk2py share libddsc.so. Calling
    Domain(0, config) after rclpy.init() raises DDSException because domain 0
    already exists, so we skip straight to DomainParticipant(), which joins the
    active domain. Mirrors the onboard imu_node/speaker_node.
    """
    from cyclonedds.domain import DomainParticipant as _DDP

    def _init(self, id: int, networkInterface=None, qos=None) -> bool:
        if self.__class__._ChannelFactory__initialized:
            return True
        with self.__class__._ChannelFactory__init_lock:
            if self.__class__._ChannelFactory__initialized:
                return True
            try:
                self.__class__._ChannelFactory__participant = _DDP(id)
            except Exception:
                return False
            self.__class__._ChannelFactory__qos = qos
            self.__class__._ChannelFactory__initialized = True
            return True

    _sdk_ch.ChannelFactory.Init = _init


_patch_channel_factory()


# Pipeline LOCK (AudioPCM.msg)
SAMPLE_RATE = 16000
CHANNELS = 1
BIT_DEPTH = 16

# SpeakerState.current_chunk_id is uint32; 0 is reserved for idle.
IDLE_CHUNK_ID = 0
MAX_CHUNK_ID = 0xFFFFFFFF


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__('speaker_node')

        # --- Parameters --------------------------------------------------
        self.declare_parameter('audio_out_topic', '/bridge/cmd/audio_out')
        self.declare_parameter('speaker_state_topic', '/bridge/audio/speaker_state')
        self.declare_parameter('network_interface', '')  # patched Init ignores it
        self.declare_parameter('domain_id', 0)
        self.declare_parameter('max_queue_depth', 50)
        self.declare_parameter('app_name', 'cortex_tts')

        audio_out_topic: str = self.get_parameter('audio_out_topic').value
        speaker_state_topic: str = self.get_parameter('speaker_state_topic').value
        iface: str = self.get_parameter('network_interface').value
        domain_id: int = self.get_parameter('domain_id').value
        self._max_q: int = int(self.get_parameter('max_queue_depth').value)
        self._app_name: str = self.get_parameter('app_name').value

        if not (1 <= self._max_q < 256):
            # SpeakerState.queue_depth is uint8 -> cap < 256.
            raise ValueError(
                f"max_queue_depth must be in [1, 255]; got {self._max_q}")

        # --- State -------------------------------------------------------
        self._queue: "deque[Tuple[int, bytes]]" = deque(maxlen=self._max_q)
        self._lock = threading.Lock()
        self._next_chunk_id: int = 1  # 0 reserved for idle
        self._current_chunk_id: int = IDLE_CHUNK_ID
        self._stream_id: Optional[str] = None  # new id per playback session

        # --- AudioClient -------------------------------------------------
        ChannelFactory().Init(domain_id, iface)
        self._client = AudioClient()
        self._client.SetTimeout(10.0)
        self._client.Init()
        self.get_logger().info(
            f"AudioClient ready (iface={iface!r}, domain={domain_id}, "
            f"app_name={self._app_name}, max_queue_depth={self._max_q})")

        # --- ROS I/O -----------------------------------------------------
        self._sub = self.create_subscription(
            AudioPCM, audio_out_topic, self._on_pcm, 10)
        self._state_pub = self.create_publisher(
            SpeakerState, speaker_state_topic, 10)
        self.get_logger().info(
            f"subscribe {audio_out_topic} -> AudioClient.PlayStream, "
            f"publish {speaker_state_topic}")

        # --- Writer thread ----------------------------------------------
        self._running = True
        self._wake = threading.Event()
        self._thread = threading.Thread(
            target=self._writer_loop, name='speaker_writer', daemon=True)
        self._thread.start()

        self._publish_state()  # initial idle state

    # -------------------------------------------------------------------
    # Subscription path (push)
    # -------------------------------------------------------------------
    def _validate(self, msg: AudioPCM) -> bool:
        if (msg.sample_rate != SAMPLE_RATE
                or msg.channels != CHANNELS
                or msg.bit_depth != BIT_DEPTH):
            self.get_logger().warn(
                "Drop AudioPCM: format mismatch "
                f"(got {msg.sample_rate}Hz/{msg.channels}ch/{msg.bit_depth}bit, "
                f"locked={SAMPLE_RATE}/{CHANNELS}/{BIT_DEPTH})")
            return False
        if len(msg.data) == 0:
            return False
        if len(msg.data) % 2 != 0:
            self.get_logger().warn("Drop AudioPCM: odd byte length (not int16-aligned)")
            return False
        return True

    def _alloc_chunk_id(self) -> int:
        cid = self._next_chunk_id
        nxt = cid + 1
        if nxt > MAX_CHUNK_ID:
            nxt = 1  # skip 0 (idle)
        self._next_chunk_id = nxt
        return cid

    def _on_pcm(self, msg: AudioPCM) -> None:
        if not self._validate(msg):
            return

        chunk_id = self._alloc_chunk_id()
        pcm = bytes(msg.data)  # uint8[] -> bytes

        with self._lock:
            dropped = len(self._queue) == self._max_q  # deque(maxlen) auto-drops oldest
            self._queue.append((chunk_id, pcm))

        if dropped:
            self.get_logger().warn(
                f"Queue full ({self._max_q}); dropped OLDEST chunk")

        self._wake.set()
        self._publish_state()  # push transition

    # -------------------------------------------------------------------
    # Writer thread (pop + PlayStream)
    # -------------------------------------------------------------------
    def _writer_loop(self) -> None:
        while self._running:
            chunk: Optional[Tuple[int, bytes]] = None
            with self._lock:
                if self._queue:
                    chunk = self._queue.popleft()

            if chunk is None:
                # idle: clear session if needed, then sleep
                if self._current_chunk_id != IDLE_CHUNK_ID:
                    self._current_chunk_id = IDLE_CHUNK_ID
                    self._stream_id = None
                    self._publish_state()  # idle transition
                self._wake.wait(timeout=0.1)
                self._wake.clear()
                continue

            chunk_id, pcm_bytes = chunk

            # Start a new playback session when coming out of idle
            if self._stream_id is None:
                self._stream_id = str(int(time.time() * 1000))

            self._current_chunk_id = chunk_id
            self._publish_state()  # pop transition (now playing this chunk)

            try:
                code, _ = self._client.PlayStream(
                    self._app_name, self._stream_id, pcm_bytes)
                if code != 0:
                    self.get_logger().error(
                        f"PlayStream failed: code={code} chunk_id={chunk_id}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"PlayStream raised: {e!r}")

    # -------------------------------------------------------------------
    # State publish
    # -------------------------------------------------------------------
    def _publish_state(self) -> None:
        with self._lock:
            depth = len(self._queue)
        msg = SpeakerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'speaker'
        msg.playing = (self._current_chunk_id != IDLE_CHUNK_ID)
        msg.current_chunk_id = self._current_chunk_id
        msg.queue_depth = depth
        self._state_pub.publish(msg)

    # -------------------------------------------------------------------
    # Shutdown: drain queue -> PlayStop -> idle state
    # -------------------------------------------------------------------
    def destroy_node(self) -> None:
        self.get_logger().info("speaker_node shutting down; draining queue...")
        self._running = False
        self._wake.set()
        self._thread.join(timeout=5.0)

        try:
            while True:
                with self._lock:
                    if not self._queue:
                        break
                    chunk_id, pcm_bytes = self._queue.popleft()
                if self._stream_id is None:
                    self._stream_id = str(int(time.time() * 1000))
                self._current_chunk_id = chunk_id
                try:
                    self._client.PlayStream(
                        self._app_name, self._stream_id, pcm_bytes)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(f"drain PlayStream raised: {e!r}")
        finally:
            try:
                self._client.PlayStop(self._app_name)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"PlayStop raised: {e!r}")
            self._current_chunk_id = IDLE_CHUNK_ID
            self._stream_id = None
            try:
                self._publish_state()  # final idle state
            except Exception:  # noqa: BLE001
                pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
