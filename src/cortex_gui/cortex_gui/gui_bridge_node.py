"""gui_bridge_node — ROS -> WebSocket egress adapter for kist-drl-g1-gui.

The display renderer (kist-drl-g1-gui, REQ-41) is ROS-free: it connects over a
WebSocket and, per frame, expects two messages —

    text (JSON) : {"scenario": str, "subtask": {"name", "i", "n"} | null,
                   "state": "idle" | "active" | "success" | "failed"}
    binary      : the latest camera JPEG bytes

This node is a THIN adapter: it subscribes to the orchestrator's TaskStatus (and
optionally a camera topic), translates to that contract, and serves it. No
decisions live here — `state` semantics come from orchestrator_node; the JSON
shape is owned by the GUI repo (do not invent fields). Replacing the workstation
GUIBackground publisher with this keeps the renderer unchanged: just point `?ws=`
at this host.

Two concurrency domains: rclpy spins on the main thread; a websockets server runs
on a daemon thread with its own asyncio loop. They meet at a latest-value cell
(newest status / newest frame overwrite the old — a mailbox, not a queue), so a
slow client can never back-pressure the ROS side.
"""

import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cortex_msgs.msg import TaskStatus


# TaskStatus.state -> the GUI's four display states. PREEMPTED maps to idle: a
# preempt is immediately followed by the new scenario's RUNNING, so it is a
# transient the renderer should not flash red for.
_STATE_NAME = {
    TaskStatus.STATE_IDLE: 'idle',
    TaskStatus.STATE_RUNNING: 'active',
    TaskStatus.STATE_SUCCEEDED: 'success',
    TaskStatus.STATE_FAILED: 'failed',
    TaskStatus.STATE_PREEMPTED: 'idle',
}


def _status_to_json(msg: TaskStatus) -> str:
    state = _STATE_NAME.get(msg.state, 'idle')
    # subtask is null when idle / no sub-task in flight (renderer contract).
    if state == 'idle' or not msg.current_subtask:
        subtask = None
    else:
        subtask = {
            'name': msg.current_subtask,
            'i': int(msg.subtask_index),     # 0-based; the renderer shows i+1/n
            'n': int(msg.subtask_count),
        }
    return json.dumps(
        {'scenario': msg.task_name, 'subtask': subtask, 'state': state},
        ensure_ascii=False,                  # scenario / sub-task names are Korean
    )


class GuiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('gui_bridge_node')

        self.declare_parameter('status_topic', '/cortex/task_status')
        self.declare_parameter('camera_topic', '/bridge/sensors/camera/color/compressed')
        # compressed: forward JPEG bytes as-is (cheap). raw: encode Image -> JPEG
        # (needs pillow+numpy). none: status-only (renderer shows NO SIGNAL).
        self.declare_parameter('camera_transport', 'compressed')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8081)
        self.declare_parameter('stream_rate_hz', 15.0)

        g = self.get_parameter
        self._transport = str(g('camera_transport').value).lower()
        self._host = str(g('ws_host').value)
        self._port = int(g('ws_port').value)
        self._rate = float(g('stream_rate_hz').value)

        # --- latest-value cell (written by ROS cbs, read by the WS pump) -----
        self._status_json: str = _status_to_json(TaskStatus())   # seed = idle
        self._frame: bytes = b''
        self._frame_seq: int = 0                 # bumped per new frame; pump dedupes

        # --- io --------------------------------------------------------------
        self.create_subscription(
            TaskStatus, g('status_topic').value, self._on_status, 10)
        self._subscribe_camera(g('camera_topic').value)

        # --- websocket server on its own asyncio loop / daemon thread --------
        self._clients: set = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5.0)
        self.get_logger().info(
            f'gui_bridge up: ws://{self._host}:{self._port} '
            f'(camera transport={self._transport}, {self._rate:g} Hz)')

    # --- ROS callbacks (main thread) -------------------------------------
    def _on_status(self, msg: TaskStatus) -> None:
        self._status_json = _status_to_json(msg)

    def _subscribe_camera(self, topic: str) -> None:
        if self._transport == 'none':
            return
        if self._transport == 'compressed':
            self.create_subscription(CompressedImage, topic, self._on_compressed, 10)
        elif self._transport == 'raw':
            self.create_subscription(Image, topic, self._on_raw, 10)
        else:
            self.get_logger().warning(
                f'unknown camera_transport {self._transport!r}; camera disabled')

    def _on_compressed(self, msg: CompressedImage) -> None:
        # Already JPEG/PNG on the wire — forward the bytes untouched.
        self._frame = bytes(msg.data)
        self._frame_seq += 1

    def _on_raw(self, msg: Image) -> None:
        jpeg = _encode_jpeg(msg)
        if jpeg is not None:
            self._frame = jpeg
            self._frame_seq += 1

    # --- websocket side (daemon thread) ----------------------------------
    def _serve(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self._run_server())
        except Exception as exc:                 # noqa: BLE001 - log & let thread end
            self.get_logger().error(f'ws server stopped: {exc}')

    async def _run_server(self) -> None:
        import websockets

        async with websockets.serve(self._client_handler, self._host, self._port):
            self._ready.set()
            await self._pump()

    async def _client_handler(self, ws) -> None:
        self._clients.add(ws)
        self.get_logger().info(f'gui client connected ({len(self._clients)})')
        try:
            async for _ in ws:                   # renderer never sends; just await close
                pass
        finally:
            self._clients.discard(ws)
            self.get_logger().info(f'gui client left ({len(self._clients)})')

    async def _pump(self) -> None:
        """Broadcast latest status (on change) + latest frame (on change) to all
        clients at stream_rate. Latest-value: a slow client drops frames, never
        stalls ROS."""
        period = 1.0 / self._rate if self._rate > 0 else 1.0 / 15.0
        last_status = None
        last_seq = -1
        while True:
            status = self._status_json
            seq = self._frame_seq
            send_status = status != last_status
            send_frame = seq != last_seq and self._frame
            if self._clients and (send_status or send_frame):
                frame = self._frame
                dead = []
                for ws in list(self._clients):
                    try:
                        if send_status:
                            await ws.send(status)
                        if send_frame:
                            await ws.send(frame)
                    except Exception:            # noqa: BLE001 - drop broken client
                        dead.append(ws)
                for ws in dead:
                    self._clients.discard(ws)
            last_status, last_seq = status, seq
            await asyncio.sleep(period)

    def request_shutdown(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)


def _encode_jpeg(msg: Image):
    """sensor_msgs/Image -> JPEG bytes. Lazy deps: only the raw transport needs
    them. Returns None (keeps previous frame) on an unsupported encoding."""
    try:
        import numpy as np
        from PIL import Image as PilImage
    except ImportError:
        return None
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8'):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.width, 3)
        if enc == 'bgr8':
            arr = arr[:, :, ::-1]
        pil = PilImage.fromarray(arr, 'RGB')
    elif enc in ('mono8', '8uc1'):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)
        pil = PilImage.fromarray(arr, 'L')
    else:
        return None
    import io
    buf = io.BytesIO()
    pil.save(buf, format='JPEG', quality=70)
    return buf.getvalue()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.request_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
