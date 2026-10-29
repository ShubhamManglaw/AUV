"""ROS 2 node wrapping ByteTrack multi-object tracking (§10.6).

Subscribes:
  /vehicle/perception/detections (vision_msgs/Detection2DArray)

Publishes:
  /vehicle/perception/tracks (ps11_interfaces/TrackArray)
"""

from __future__ import annotations

import numpy as np
import rclpy
from ps11_common.params import load_yaml
from ps11_interfaces.msg import Track, TrackArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray

from ps11_perception.tracker import ObjectTracker


class TrackerNode(Node):
    """Subscribes to 2D detections, runs ByteTrack, and publishes confirmed tracks."""

    def __init__(self) -> None:
        super().__init__("tracker")

        # Load parameters from perception.yaml (§1.4, no hard-coded constants)
        cfg = load_yaml("perception.yaml").get("tracker", {})
        self.declare_parameter("min_hits", int(cfg.get("min_hits", 1)))
        self.declare_parameter("min_mean_conf", float(cfg.get("min_mean_conf", 0.12)))
        self.declare_parameter(
            "track_activation_threshold",
            float(cfg.get("track_activation_threshold", 0.10)),
        )
        self.declare_parameter(
            "lost_track_buffer", int(cfg.get("lost_track_buffer", 45))
        )
        self.declare_parameter(
            "minimum_matching_threshold",
            float(cfg.get("minimum_matching_threshold", 0.25)),
        )
        self.declare_parameter(
            "recency_weight_power",
            float(cfg.get("recency_weight_power", 2.0)),
        )
        self.declare_parameter(
            "debris_priority_boost",
            float(cfg.get("debris_priority_boost", 1.5)),
        )

        min_hits = int(self.get_parameter("min_hits").value)
        min_mean_conf = float(self.get_parameter("min_mean_conf").value)
        track_act = float(self.get_parameter("track_activation_threshold").value)
        lost_buf = int(self.get_parameter("lost_track_buffer").value)
        match_thresh = float(self.get_parameter("minimum_matching_threshold").value)
        recency_pow = float(self.get_parameter("recency_weight_power").value)
        debris_boost = float(self.get_parameter("debris_priority_boost").value)

        self.tracker = ObjectTracker(
            min_hits=min_hits,
            min_mean_conf=min_mean_conf,
            track_activation_threshold=track_act,
            lost_track_buffer=lost_buf,
            minimum_matching_threshold=match_thresh,
            recency_weight_power=recency_pow,
            debris_priority_boost=debris_boost,
        )

        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.track_pub = self.create_publisher(
            TrackArray, "/vehicle/perception/tracks", qos_rel
        )

        self.create_subscription(
            Detection2DArray,
            "/vehicle/perception/detections",
            self._on_detections,
            qos_rel,
        )

        self.get_logger().info(
            f"TrackerNode initialized (min_hits={min_hits}, min_mean_conf={min_mean_conf})"
        )

    def _on_detections(self, msg: Detection2DArray) -> None:
        boxes_xyxy: list[list[float]] = []
        confs: list[float] = []
        classes: list[int] = []

        for d in msg.detections:
            # Centre position and dimensions
            if hasattr(d.bbox.center, "position"):
                cx = float(d.bbox.center.position.x)
                cy = float(d.bbox.center.position.y)
            else:
                cx = float(d.bbox.center.x)
                cy = float(d.bbox.center.y)

            w = float(d.bbox.size_x)
            h = float(d.bbox.size_y)

            x1 = cx - w / 2.0
            y1 = cy - h / 2.0
            x2 = cx + w / 2.0
            y2 = cy + h / 2.0
            boxes_xyxy.append([x1, y1, x2, y2])

            # Class and score
            if len(d.results) > 0:
                hyp = d.results[0]
                if hasattr(hyp, "hypothesis"):
                    cls_str = str(hyp.hypothesis.class_id)
                    score = float(hyp.hypothesis.score)
                else:
                    cls_str = str(hyp.id)
                    score = float(hyp.score)
                try:
                    c_id = int(cls_str)
                except ValueError:
                    c_id = 0
            else:
                c_id = 0
                score = 0.0

            classes.append(c_id)
            confs.append(score)

        if len(boxes_xyxy) > 0:
            xyxy_arr = np.array(boxes_xyxy, dtype=float)
            conf_arr = np.array(confs, dtype=float)
            cls_arr = np.array(classes, dtype=int)
        else:
            xyxy_arr = np.empty((0, 4), dtype=float)
            conf_arr = np.empty((0,), dtype=float)
            cls_arr = np.empty((0,), dtype=int)

        confirmed = self.tracker.update(xyxy_arr, conf_arr, cls_arr)

        track_array = TrackArray()
        track_array.header = msg.header

        for t in confirmed:
            trk = Track()
            trk.header = msg.header
            trk.track_id = int(t.track_id)
            trk.class_id = int(t.class_id)
            trk.confidence = float(t.confidence)
            trk.u = float(t.u)
            trk.v = float(t.v)
            trk.width = float(t.width)
            trk.height = float(t.height)
            trk.hits = int(t.hits)
            track_array.tracks.append(trk)

        self.track_pub.publish(track_array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
