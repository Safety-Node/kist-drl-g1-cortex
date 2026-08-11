"""mic_bridge_node — ext-sensor AudioChunk -> AudioPCM adapter [REQ-27].

kist-ext-sensor-io is a GENERIC external-sensor I/O repo: it captures the mic
with ALSA and publishes its own self-describing DDS type (kist_msgs::AudioChunk)
on ``rt/kist/mic/<name>/audio`` — raw interleaved PCM at the device's native
rate/channels, with NO knowledge of cortex's message schema. It deliberately
does NOT publish g1_onboard_msgs/AudioPCM (that would tie the sensor repo to one
consumer's contract). So the conversion is OUR job, on the cortex side.

This node bridges the two worlds:

    kist_msgs::AudioChunk        (raw CycloneDDS type, ext-sensor)
      rt/kist/mic/<name>/audio
              |
              v   subscribe via cyclonedds-python (same DDS domain, no rmw)
       downmix -> mono, resample -> 16 kHz, keep S16_LE
              |
              v   publish via rclpy
    g1_onboard_msgs/AudioPCM     (/bridge/sensors/audio_pcm)  -> stt_node

Why two DDS stacks in one process: AudioChunk is NOT a ROS message, so rclpy
can't subscribe to it. We read it with the raw cyclonedds Python binding (the
same libddsc rmw_cyclonedds uses) and re-emit a proper ROS message with rclpy.
Both join the SAME DDS domain, so this must run on a host that shares the
robot's domain (same as speaker_node).

Pipeline OUT is LOCKED to AudioPCM's contract: 16 kHz / mono / 16-bit LE.

Deps beyond the ROS ones: ``cyclonedds`` (the Python binding — a separate pip
package from the C library that ships with rmw_cyclonedds), plus numpy/scipy
(already used by stt_node) for downmix/resample.

NOTE (on-robot verify pending): DDS type matching is by structural type name +
members. The dataclass below MUST stay byte-for-byte compatible with
ext-sensor's ``idl/kist_audio_frames.idl`` (module kist_msgs, struct AudioChunk).
If ext-sensor changes that IDL, update this class or the reader silently sees no
data.
"""
import threading
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.signal import resample_poly

from g1_onboard_msgs.msg import AudioPCM

# --- Raw ext-sensor DDS type (mirror of kist_audio_frames.idl) -------------
# cyclonedds-python builds the DDS type descriptor from this dataclass; the
# typename and field order/types must match idlc's output exactly.
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence, uint8, uint32, uint64, int64
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader


@dataclass
class AudioChunk(IdlStruct, typename="kist_msgs::AudioChunk"):
    seq: uint64
    stamp_ns: int64
    sample_rate: uint32
    channels: uint32
    format: str
    frame_id: str
    data: sequence[uint8]  # octet[] == uint8[]; interleaved PCM


# AudioPCM contract (locked)
OUT_RATE = 16000
OUT_CHANNELS = 1
OUT_BIT_DEPTH = 16


class MicBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('mic_bridge_node')

        # --- Parameters --------------------------------------------------
        # audio_chunk_topic: raw DDS topic ext-sensor publishes on. The mic
        # `name` in ext-sensor's config.yaml namespaces it; "array" is the
        # reSpeaker (16 kHz, no resample needed). NOT ROS-mangled — this is the
        # literal DDS topic string.
        self.declare_parameter('audio_chunk_topic', 'rt/kist/mic/array/audio')
        self.declare_parameter('audio_pcm_topic', '/bridge/sensors/audio_pcm')
        self.declare_parameter('domain_id', 0)
        # channel: which interleaved channel to keep. >=0 selects that channel
        # (0 = reSpeaker's processed/beamformed output); -1 averages all.
        self.declare_parameter('channel', 0)
        self.declare_parameter('frame_id', 'mic')

        self._chunk_topic: str = self.get_parameter('audio_chunk_topic').value
        pcm_topic: str = self.get_parameter('audio_pcm_topic').value
        self._domain_id: int = int(self.get_parameter('domain_id').value)
        self._channel: int = int(self.get_parameter('channel').value)
        self._frame_id: str = self.get_parameter('frame_id').value

        # --- ROS out -----------------------------------------------------
        self._pub = self.create_publisher(AudioPCM, pcm_topic, 10)

        # --- Raw DDS in --------------------------------------------------
        self._dp = DomainParticipant(self._domain_id)
        self._topic = Topic(self._dp, self._chunk_topic, AudioChunk)
        self._reader = DataReader(self._dp, self._topic)
        self.get_logger().info(
            f"subscribe (raw DDS) {self._chunk_topic} -> convert -> publish "
            f"{pcm_topic} (16k/mono/16bit), domain={self._domain_id}, "
            f"channel={self._channel}")

        # warn once per distinct malformed input, not every 100 ms
        self._warned_format = ''
        self._warned_channels = 0

        # --- Reader thread ----------------------------------------------
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, name='mic_bridge_reader', daemon=True)
        self._thread.start()

    # -------------------------------------------------------------------
    def _read_loop(self) -> None:
        # take_iter blocks up to `timeout`, then yields nothing — the timeout
        # lets the loop re-check self._running for clean shutdown.
        from cyclonedds.util import duration
        while self._running:
            try:
                for sample in self._reader.take_iter(timeout=duration(milliseconds=200)):
                    if not self._running:
                        break
                    self._on_chunk(sample)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"DDS read loop error: {e!r}")

    def _on_chunk(self, chunk: AudioChunk) -> None:
        if chunk.format != 'S16_LE':
            if self._warned_format != chunk.format:
                self._warned_format = chunk.format
                self.get_logger().warn(
                    f"Drop AudioChunk: format {chunk.format!r} != 'S16_LE' "
                    "(only signed 16-bit LE is supported)")
            return

        ch = int(chunk.channels)
        if ch < 1:
            return

        raw = bytes(chunk.data)
        if len(raw) < 2 * ch:
            return
        # trim to whole frames, then (frames, channels)
        usable = (len(raw) // (2 * ch)) * (2 * ch)
        samples = np.frombuffer(raw[:usable], dtype='<i2').reshape(-1, ch)

        # --- downmix -> mono --------------------------------------------
        if ch == 1:
            mono = samples[:, 0]
        elif self._channel < 0:
            # average in int32 to avoid int16 overflow, then clip back
            mono = samples.astype(np.int32).mean(axis=1)
            mono = np.clip(mono, -32768, 32767).astype(np.int16)
        else:
            sel = self._channel if self._channel < ch else 0
            if self._channel >= ch and self._warned_channels != ch:
                self._warned_channels = ch
                self.get_logger().warn(
                    f"channel={self._channel} >= channels={ch}; using channel 0")
            mono = samples[:, sel]

        # --- resample -> 16 kHz -----------------------------------------
        in_rate = int(chunk.sample_rate)
        if in_rate != OUT_RATE:
            # resample_poly wants float; keep in int32 range then cast
            g = np.gcd(in_rate, OUT_RATE)
            up, down = OUT_RATE // g, in_rate // g
            res = resample_poly(mono.astype(np.float32), up, down)
            mono = np.clip(np.rint(res), -32768, 32767).astype(np.int16)
        else:
            mono = np.ascontiguousarray(mono, dtype=np.int16)

        if mono.size == 0:
            return

        # --- publish AudioPCM -------------------------------------------
        msg = AudioPCM()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.sample_rate = OUT_RATE
        msg.channels = OUT_CHANNELS
        msg.bit_depth = OUT_BIT_DEPTH
        msg.data = mono.tobytes()  # uint8[] payload, int16 LE
        self._pub.publish(msg)

    # -------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
