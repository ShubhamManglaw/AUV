"""ROS 2 detector node rate-capped to 5 Hz with H3 banner (§10.5)."""

import os
import threading
from pathlib import Path

import rclpy
from ps11_common.classes import ClassDatabase
from ps11_common.image_utils import image_to_numpy, numpy_to_image
from ps11_common.params import get_config_path, load_yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from ps11_perception.detector import Detection, YOLODetector


def _find_model_file(model_path_str: str) -> Path:
    """Resolve model weights file from env or relative paths."""
    env_path = os.environ.get("PS11_MODEL")
    if env_path and Path(env_path).is_file():
        return Path(env_path)

    p = Path(model_path_str)
    if p.is_file():
        return p.resolve()

    # Search from PS11_ROOT
    ps11_root = os.environ.get("PS11_ROOT")
    if ps11_root:
        candidate = Path(ps11_root) / model_path_str
        if candidate.is_file():
            return candidate.resolve()

    # Search upwards from current file
    cur = Path(__file__).resolve()
    for parent in cur.parents:
        cand = parent / model_path_str
        if cand.is_file():
            return cand.resolve()

    raise FileNotFoundError(f"Model file not found: {model_path_str}")


class DetectorNode(Node):
    """Subscribes to camera image, caps processing rate, publishes detections and annotated images."""

    def __init__(self) -> None:
        super().__init__("detector")

        # Load parameters from perception.yaml
        config_path = get_config_path("perception.yaml")
        params = load_yaml(config_path).get("detector", {})

        model_path_str = self.declare_parameter(
            "model_path", params.get("model_path", "ml/weights/best.pt")
        ).value
        model_file = _find_model_file(str(model_path_str))

        device = str(
            self.declare_parameter("device", params.get("device", "cuda:0")).value
        )
        conf_threshold = float(
            self.declare_parameter(
                "conf_threshold", float(params.get("conf_threshold", 0.35))
            ).value
        )
        iou_threshold = float(
            self.declare_parameter(
                "iou_threshold", float(params.get("iou_threshold", 0.5))
            ).value
        )
        imgsz = int(
            self.declare_parameter("imgsz", int(params.get("imgsz", 640))).value
        )
        self.max_rate_hz = float(
            self.declare_parameter(
                "max_rate_hz", float(params.get("max_rate_hz", 5.0))
            ).value
        )
        input_topic = str(
            self.declare_parameter(
                "input_image_topic", "/vehicle/camera/image_raw"
            ).value
        )

        self.get_logger().info(
            f"Initializing YOLODetector from {model_file} on device {device} (rate cap: {self.max_rate_hz} Hz)"
        )

        # Initialize detector logic
        class_db = ClassDatabase()
        self.detector = YOLODetector(
            model_path=model_file,
            device=device,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            imgsz=imgsz,
            class_db=class_db,
        )

        self.banner_text = "SIMULATION | NAV: kinematic (M1) | detector rate capped"

        # State and locks for rate capping (always take newest frame)
        self._lock = threading.Lock()
        self._latest_msg: Image | None = None
        self._new_frame_available: bool = False

        # Publishers
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.det_pub = self.create_publisher(
            Detection2DArray, "/vehicle/perception/detections", qos_rel
        )
        self.annotated_pub = self.create_publisher(
            Image, "/vehicle/perception/image_annotated", qos_rel
        )

        # Subscriptions: support both SensorDataQoS (BEST_EFFORT) and RELIABLE
        self.create_subscription(
            Image, input_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(Image, input_topic, self._on_image, qos_rel)

        # Rate-capping timer: ticks at max_rate_hz (default 5 Hz)
        timer_period = 1.0 / self.max_rate_hz
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f"DetectorNode ready. Subscribing to {input_topic}, publishing /vehicle/perception/detections"
        )

    def _on_image(self, msg: Image) -> None:
        """Cache incoming image; always retain the newest frame."""
        with self._lock:
            self._latest_msg = msg
            self._new_frame_available = True

    def _on_timer(self) -> None:
        """Periodic inference worker enforcing the rate cap."""
        msg: Image | None = None
        with self._lock:
            if not self._new_frame_available or self._latest_msg is None:
                return
            msg = self._latest_msg
            self._new_frame_available = False

        if msg is None:
            return

        try:
            img_rgb = image_to_numpy(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert Image message to numpy: {e}")
            return

        # Run inference
        detections: list[Detection] = self.detector.detect(img_rgb)

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id or "camera_optical_frame"

        for det in detections:
            d2d = Detection2D()
            d2d.header = det_array.header

            # Bounding box
            bbox = BoundingBox2D()
            if hasattr(bbox.center, "position"):
                bbox.center.position.x = float(det.cx)
                bbox.center.position.y = float(det.cy)
            else:
                bbox.center.x = float(det.cx)
                bbox.center.y = float(det.cy)
            bbox.size_x = float(det.w)
            bbox.size_y = float(det.h)
            d2d.bbox = bbox

            # Class hypothesis
            hyp = ObjectHypothesis()
            if hasattr(hyp, "class_id"):
                hyp.class_id = str(det.class_id)
            else:
                hyp.id = str(det.class_id)
            hyp.score = float(det.confidence)

            hyp_pose = ObjectHypothesisWithPose()
            if hasattr(hyp_pose, "hypothesis"):
                hyp_pose.hypothesis = hyp
            else:
                if hasattr(hyp_pose, "id"):
                    hyp_pose.id = str(det.class_id)
                hyp_pose.score = float(det.confidence)

            d2d.results.append(hyp_pose)
            det_array.detections.append(d2d)

        # Publish detections
        self.det_pub.publish(det_array)

        # Annotate and publish image
        annotated_rgb = self.detector.annotate(
            img_rgb, detections, banner_text=self.banner_text
        )
        annotated_msg = numpy_to_image(
            annotated_rgb,
            encoding="rgb8",
            header=det_array.header,
        )
        self.annotated_pub.publish(annotated_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
