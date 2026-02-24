#============================= camera/rostopic_latest ===========================
"""
ROS2 (rclpy) topic camera adapters with latest-only ("drop-latest") buffering.

Design goals:
- Minimal behavioral change vs camera.rostopic usage scripts:
    - start(), stop(), get_frames() remain
    - get_frames() returns only the numpy image (no header tuple)
- Adds metadata accessors and drop/overwrite stats without breaking old code.
- Uses a single-slot mailbox: newest frame overwrites the previous one.
- ROS2 uses QoS depth=1 (KEEP_LAST) to avoid backlog at the middleware layer.

Notes:
- stop() calls rclpy.shutdown() (per current project scope). If you later run
  multi-node programs in the same process, we should revisit ownership.
"""

from __future__ import annotations

import threading
import numpy as np

# Optional ROS2 imports so the camera repo can still be installed on machines
# without ROS2 (e.g., macOS). The ROS2 classes will raise a helpful error if used.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    _HAS_ROS2 = True
except Exception:  # pragma: no cover
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    HistoryPolicy = None
    Image = None
    _HAS_ROS2 = False

import camera.base as Camera


def _require_ros2():
    if not _HAS_ROS2:
        raise ImportError(
            "ROS2 python packages not found (rclpy / sensor_msgs). "
            "Source ROS2 (e.g., 'source /opt/ros/humble/setup.bash') and ensure rclpy is available."
        )


def _maybe_init_rclpy():
    _require_ros2()
    # rclpy.init() is safe to call once; if already initialized, it can raise.
    # We'll just ignore errors to preserve minimal setup friction.
    try:
        if not rclpy.ok():
            rclpy.init()
    except Exception:
        # Already initialized or managed elsewhere
        pass


def _default_qos(depth: int = 1) -> QoSProfile:
    # Match typical camera streaming semantics: keep last, depth=1.
    # Start with RELIABLE to match your publisher info. If you see drops, keep RELIABLE.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


#-------------------------------------------------------------------------
#==================== ROSTopic / Camera Configuration ====================
#-------------------------------------------------------------------------

class CfgROSCam(Camera.CfgCamera):
    """Configuration specifier for ROS-connected camera (ROS2)."""

    def __init__(self, init_dict=None, key_list=None, new_allowed=True):
        if init_dict is None:
            init_dict = CfgROSCam.get_default_settings()
        super().__init__(init_dict, key_list, new_allowed)

    @staticmethod
    def get_default_settings():
        return dict(topicPath="", topicName="")


#-------------------------------------------------------------------------
#======================== ROSTopic / Color Camera ========================
#-------------------------------------------------------------------------

class Color(Camera.Color):
    """Latest-only ROS2 color image subscriber (drop-latest mailbox)."""

    def __init__(self, configs=None, K=None) -> None:
        super().__init__(configs, K)

        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._sub = None

        self._lock = threading.Lock()

        self._latest_img = None
        self._latest_header = None
        self._encoding = None

        self._initialized = False
        self._received = 0
        self._version = 0

        # For meaningful overwrite counting:
        # overwritten increments when a new frame arrives before the previous one is "consumed"
        self._delivered_version = 0
        self._overwritten = 0

    def start(self):
        _maybe_init_rclpy()

        # Create node + subscription + background spinner
        self._node = Node("camera_color_latest")
        topic = self.configs.topicPath + "/" + self.configs.topicName

        self._sub = self._node.create_subscription(
            Image,
            topic,
            self.streamCB,
            _default_qos(depth=1),
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def stop(self):
        # 1) Stop executor first so callbacks/spin loop stops touching the node
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass

        # 2) Join spin thread so it fully exits
        try:
            if self._spin_thread is not None and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass

        # 3) Destroy subscriptions
        try:
            if self._node is not None and self._sub is not None:
                self._node.destroy_subscription(self._sub)
                self._sub = None
        except Exception:
            pass

        # (RGBD: destroy _color_sub and _depth_sub similarly)

        # 4) Destroy node
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass

        self._executor = None
        self._node = None
        self._spin_thread = None

        # 5) Shutdown ROS2 context last
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def streamCB(self, msg: Image):
        # Convert message bytes to numpy (store as-is; color assumed uint8)
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, -1)
        except ValueError:
            img = arr.reshape(msg.height, msg.width)

        with self._lock:
            # Overwrite detection: if the last produced version hasn't been delivered yet
            if self._initialized and (self._version != self._delivered_version):
                self._overwritten += 1
            else:
                self._initialized = True

            self._latest_img = img
            self._latest_header = msg.header
            self._encoding = getattr(msg, "encoding", None)

            self._received += 1
            self._version += 1

    def get_frames(self):
        with self._lock:
            # Mark current version as "delivered"
            self._delivered_version = self._version
            return self._latest_img

    # ---- Optional metadata/stats helpers ----
    def has_frames(self) -> bool:
        with self._lock:
            return self._initialized

    def get_header(self):
        with self._lock:
            return self._latest_header

    def get_stamp(self):
        with self._lock:
            return None if self._latest_header is None else self._latest_header.stamp

    def get_frame_id(self):
        with self._lock:
            return None if self._latest_header is None else self._latest_header.frame_id

    def get_seq(self):
        # ROS2 Header has no seq field
        return None

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "initialized": self._initialized,
                "received": self._received,
                "overwritten": self._overwritten,
                "version": self._version,
                "delivered_version": self._delivered_version,
                "encoding": self._encoding,
            }


#-------------------------------------------------------------------------
#======================== ROSTopic / Depth Camera ========================
#-------------------------------------------------------------------------

class Depth(Camera.Grayscale):
    """Latest-only ROS2 depth image subscriber (drop-latest mailbox). Stores raw as received."""

    def __init__(self, configs=None, K=None) -> None:
        super().__init__(configs, K)

        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._sub = None

        self._lock = threading.Lock()

        self._latest_img = None
        self._latest_header = None
        self._encoding = None

        self._initialized = False
        self._received = 0
        self._version = 0

        self._delivered_version = 0
        self._overwritten = 0

    def start(self):
        _maybe_init_rclpy()

        self._node = Node("camera_depth_latest")
        topic = self.configs.topicPath + "/" + self.configs.topicName

        self._sub = self._node.create_subscription(
            Image,
            topic,
            self.streamCB,
            _default_qos(depth=1),
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def stop(self):
        # 1) Stop executor first so callbacks/spin loop stops touching the node
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass

        # 2) Join spin thread so it fully exits
        try:
            if self._spin_thread is not None and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass

        # 3) Destroy subscriptions
        try:
            if self._node is not None and self._sub is not None:
                self._node.destroy_subscription(self._sub)
                self._sub = None
        except Exception:
            pass

        # (RGBD: destroy _color_sub and _depth_sub similarly)

        # 4) Destroy node
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass

        self._executor = None
        self._node = None
        self._spin_thread = None

        # 5) Shutdown ROS2 context last
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def streamCB(self, msg: Image):
        enc = getattr(msg, "encoding", "") or ""
        # Store exactly as received; choose dtype from encoding
        if enc in ("16UC1", "mono16", ""):
            dtype = np.uint16
        elif enc == "32FC1":
            dtype = np.float32
        else:
            dtype = np.uint16

        arr = np.frombuffer(msg.data, dtype=dtype)
        try:
            img = arr.reshape(msg.height, msg.width, -1)
        except ValueError:
            img = arr.reshape(msg.height, msg.width)

        with self._lock:
            if self._initialized and (self._version != self._delivered_version):
                self._overwritten += 1
            else:
                self._initialized = True

            self._latest_img = img
            self._latest_header = msg.header
            self._encoding = getattr(msg, "encoding", None)

            self._received += 1
            self._version += 1

    def get_frames(self):
        with self._lock:
            self._delivered_version = self._version
            return self._latest_img

    # ---- Optional metadata/stats helpers ----
    def has_frames(self) -> bool:
        with self._lock:
            return self._initialized

    def get_header(self):
        with self._lock:
            return self._latest_header

    def get_stamp(self):
        with self._lock:
            return None if self._latest_header is None else self._latest_header.stamp

    def get_frame_id(self):
        with self._lock:
            return None if self._latest_header is None else self._latest_header.frame_id

    def get_seq(self):
        return None

    def get_encoding(self):
        with self._lock:
            return self._encoding

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "initialized": self._initialized,
                "received": self._received,
                "overwritten": self._overwritten,
                "version": self._version,
                "delivered_version": self._delivered_version,
                "encoding": self._encoding,
            }


#-------------------------------------------------------------------------
#========================= ROSTopic / RGBD Camera ========================
#-------------------------------------------------------------------------

class RGBD(Camera.Base):
    """Latest-only ROS2 RGB + Depth subscriber (best-effort pairing)."""

    def __init__(self, color_cfg=None, depth_cfg=None, K=None) -> None:
        super().__init__(configs=None, K=K)

        self.color_cfg = color_cfg
        self.depth_cfg = depth_cfg

        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None

        self._color_sub = None
        self._depth_sub = None

        self._lock = threading.Lock()

        # RGB mailbox
        self._rgb = None
        self._rgb_header = None
        self._rgb_enc = None
        self._rgb_initialized = False
        self._rgb_received = 0
        self._rgb_version = 0
        self._rgb_delivered_version = 0
        self._rgb_overwritten = 0

        # Depth mailbox
        self._depth = None
        self._depth_header = None
        self._depth_enc = None
        self._depth_initialized = False
        self._depth_received = 0
        self._depth_version = 0
        self._depth_delivered_version = 0
        self._depth_overwritten = 0

    def start(self):
        _maybe_init_rclpy()

        self._node = Node("camera_rgbd_latest")

        ctopic = self.color_cfg.topicPath + "/" + self.color_cfg.topicName
        dtopic = self.depth_cfg.topicPath + "/" + self.depth_cfg.topicName

        self._color_sub = self._node.create_subscription(
            Image, ctopic, self.colorCB, _default_qos(depth=1)
        )
        self._depth_sub = self._node.create_subscription(
            Image, dtopic, self.depthCB, _default_qos(depth=1)
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def stop(self):
        # 1) Stop executor first so callbacks/spin loop stops touching the node
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass

        # 2) Join spin thread so it fully exits
        try:
            if self._spin_thread is not None and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass

        # 3) Destroy subscriptions (BOTH)
        try:
            if self._node is not None and self._color_sub is not None:
                self._node.destroy_subscription(self._color_sub)
                self._color_sub = None
        except Exception:
            pass

        try:
            if self._node is not None and self._depth_sub is not None:
                self._node.destroy_subscription(self._depth_sub)
                self._depth_sub = None
        except Exception:
            pass

        # 4) Destroy node
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass

        self._executor = None
        self._node = None
        self._spin_thread = None

        # 5) Shutdown ROS2 context last
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def colorCB(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, -1)
        except ValueError:
            img = arr.reshape(msg.height, msg.width)

        with self._lock:
            if self._rgb_initialized and (self._rgb_version != self._rgb_delivered_version):
                self._rgb_overwritten += 1
            else:
                self._rgb_initialized = True

            self._rgb = img
            self._rgb_header = msg.header
            self._rgb_enc = getattr(msg, "encoding", None)

            self._rgb_received += 1
            self._rgb_version += 1

    def depthCB(self, msg: Image):
        enc = getattr(msg, "encoding", "") or ""
        if enc in ("16UC1", "mono16", ""):
            dtype = np.uint16
        elif enc == "32FC1":
            dtype = np.float32
        else:
            dtype = np.uint16

        arr = np.frombuffer(msg.data, dtype=dtype)
        try:
            img = arr.reshape(msg.height, msg.width, -1)
        except ValueError:
            img = arr.reshape(msg.height, msg.width)

        with self._lock:
            if self._depth_initialized and (self._depth_version != self._depth_delivered_version):
                self._depth_overwritten += 1
            else:
                self._depth_initialized = True

            self._depth = img
            self._depth_header = msg.header
            self._depth_enc = getattr(msg, "encoding", None)

            self._depth_received += 1
            self._depth_version += 1

    def get_frames(self):
        with self._lock:
            self._rgb_delivered_version = self._rgb_version
            self._depth_delivered_version = self._depth_version
            return self._rgb, self._depth

    def get_headers(self):
        with self._lock:
            return self._rgb_header, self._depth_header

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "rgb": {
                    "initialized": self._rgb_initialized,
                    "received": self._rgb_received,
                    "overwritten": self._rgb_overwritten,
                    "version": self._rgb_version,
                    "delivered_version": self._rgb_delivered_version,
                    "encoding": self._rgb_enc,
                },
                "depth": {
                    "initialized": self._depth_initialized,
                    "received": self._depth_received,
                    "overwritten": self._depth_overwritten,
                    "version": self._depth_version,
                    "delivered_version": self._depth_delivered_version,
                    "encoding": self._depth_enc,
                },
            }

#============================= camera/rostopic_latest ===========================