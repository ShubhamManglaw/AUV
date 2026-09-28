"""ROS 2 Geolocator Node (§10.7).

Transforms 2D tracks into 3D geolocated observations on the seabed:
Inputs:
  /vehicle/perception/tracks (TrackArray)
  /vehicle/camera/camera_info (CameraInfo)
  /vehicle/altitude (Range)
  /vehicle/nav/odom (Odometry) [optional, for sigma_nav]
  TF tree (target_frame -> camera_optical_frame at image timestamp)

Outputs:
  /vehicle/perception/observations (ObservationArray)
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ps11_interfaces.msg import Observation, ObservationArray, TrackArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Range
from tf2_ros import Buffer, TransformException, TransformListener

from ps11_perception.geo import project_pixel_to_seabed, quat_to_rot_matrix


class GeolocatorNode(Node):
    """Projects confirmed 2D bounding boxes to 3D world positions on the seabed."""

    def __init__(self) -> None:
        super().__init__("geolocator")

        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("sigma_px", 8.0)
        self.declare_parameter("sigma_alt_m", 0.1)
        self.declare_parameter("sigma_nav_m", 0.2)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("camera_optical_frame", "camera_optical_frame")

        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.sigma_px = float(self.get_parameter("sigma_px").value)
        self.sigma_alt_m = float(self.get_parameter("sigma_alt_m").value)
        self.default_sigma_nav_m = float(self.get_parameter("sigma_nav_m").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.camera_optical_frame = str(
            self.get_parameter("camera_optical_frame").value
        )

        # Default camera matrix (640x480, 90 deg HFOV -> fx = 320.0)
        self.k_matrix = np.array(
            [[320.0, 0.0, 320.0], [0.0, 320.0, 240.0], [0.0, 0.0, 1.0]], dtype=float
        )
        self._camera_info_received = False

        self._latest_altitude: float | None = None
        self._current_sigma_nav_m = self.default_sigma_nav_m

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.obs_pub = self.create_publisher(
            ObservationArray, "/vehicle/perception/observations", qos_rel
        )

        # Subscriptions
        self.create_subscription(
            CameraInfo,
            "/vehicle/camera/camera_info",
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            "/vehicle/altitude",
            self._on_altitude,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/vehicle/nav/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackArray,
            "/vehicle/perception/tracks",
            self._on_tracks,
            qos_rel,
        )

        self.get_logger().info(
            f"GeolocatorNode initialized (max_range={self.max_range_m} m, target_frame={self.target_frame})"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) == 9:
            self.k_matrix = np.array(msg.k, dtype=float).reshape((3, 3))
            self._camera_info_received = True

    def _on_altitude(self, msg: Range) -> None:
        if not math.isnan(msg.range) and msg.range > 0.0:
            self._latest_altitude = float(msg.range)

    def _on_odom(self, msg: Odometry) -> None:
        cov = msg.pose.covariance
        # Var(x) = cov[0], Var(y) = cov[7]
        var_xy = (cov[0] + cov[7]) / 2.0
        if var_xy > 0.0:
            self._current_sigma_nav_m = math.sqrt(var_xy)
        else:
            self._current_sigma_nav_m = self.default_sigma_nav_m

    def _on_tracks(self, msg: TrackArray) -> None:
        if len(msg.tracks) == 0:
            obs_array = ObservationArray()
            obs_array.header = msg.header
            obs_array.header.frame_id = self.target_frame
            self.obs_pub.publish(obs_array)
            return

        source_frame = (
            msg.header.frame_id if msg.header.frame_id else self.camera_optical_frame
        )

        # Look up transform from target_frame (map) to source_frame (camera_optical_frame)
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            # Fall back to latest available transform if exact stamp is not yet in buffer
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as e:
                self.get_logger().debug(
                    f"TF lookup failed from {self.target_frame} to {source_frame}: {e}"
                )
                return

        tx = tf_stamped.transform.translation.x
        ty = tf_stamped.transform.translation.y
        tz = tf_stamped.transform.translation.z
        camera_pos = np.array([tx, ty, tz], dtype=float)

        rot = tf_stamped.transform.rotation
        camera_rot = quat_to_rot_matrix(rot.x, rot.y, rot.z, rot.w)

        # Fallback altitude if altimeter has not arrived yet: distance to default seabed z=-15.0 or 2.5m
        if self._latest_altitude is not None:
            altitude = self._latest_altitude
        elif tz < 0.0:
            altitude = max(0.5, tz - (-15.0))
        else:
            altitude = 2.5

        obs_array = ObservationArray()
        obs_array.header = msg.header
        obs_array.header.frame_id = self.target_frame

        for trk in msg.tracks:
            res = project_pixel_to_seabed(
                u=trk.u,
                v=trk.v,
                k_matrix=self.k_matrix,
                camera_pos_map=camera_pos,
                camera_rot_map=camera_rot,
                altitude_m=altitude,
                max_range_m=self.max_range_m,
                sigma_nav_m=self._current_sigma_nav_m,
                sigma_px=self.sigma_px,
                sigma_alt_m=self.sigma_alt_m,
            )
            if res is not None:
                obs = Observation()
                obs.header = msg.header
                obs.header.frame_id = self.target_frame
                obs.track_id = int(trk.track_id)
                obs.class_id = int(trk.class_id)
                obs.confidence = float(trk.confidence)
                obs.position = Point(x=float(res.x), y=float(res.y), z=float(res.z))
                obs.sigma_xy_m = float(res.sigma_xy_m)
                obs.range_m = float(res.range_m)
                obs_array.observations.append(obs)

        self.obs_pub.publish(obs_array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GeolocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
