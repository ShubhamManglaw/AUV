"""odom_noise ROS 2 node (§9.2, T1.3).

Sensor-model node: turns privileged simulator ground truth into a realistic
noisy navigation estimate. Consumes /sim/gt/odom (allowed under H2) and
publishes:
- /vehicle/nav/odom (nav_msgs/Odometry) at publish_rate_hz
- TF map -> base_link from the same noisy pose

The horizontal position error is the ROS-free OdomErrorModel random walk;
covariance is filled with the current modelled variance. Twist passes through
unchanged (the plan defines no twist noise). Orientation is rebuilt from the
noisy yaw, so roll/pitch in the ground truth is dropped (planar kinematic
world).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from ps11_nav.error_model import (
    OdomErrorModel,
    quaternion_about_yaw,
    yaw_from_quaternion,
)


class OdomNoiseNode(Node):
    """Publish the noisy navigation estimate and the map -> base_link TF."""

    def __init__(self) -> None:
        super().__init__("odom_noise")

        # Parameters (defaults mirror nav.yaml; the params file is authoritative)
        self.declare_parameter("position_drift_rate", 0.005)
        self.declare_parameter("heading_bias_rad", 0.00872665)
        self.declare_parameter("initial_covariance_pos", 0.01)
        self.declare_parameter("initial_covariance_heading", 0.001)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("seed", 42)

        drift_rate = (
            self.get_parameter("position_drift_rate").get_parameter_value().double_value
        )
        heading_bias = (
            self.get_parameter("heading_bias_rad").get_parameter_value().double_value
        )
        initial_pos_var = (
            self.get_parameter("initial_covariance_pos")
            .get_parameter_value()
            .double_value
        )
        initial_heading_var = (
            self.get_parameter("initial_covariance_heading")
            .get_parameter_value()
            .double_value
        )
        publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self._model = OdomErrorModel(
            position_drift_rate=drift_rate,
            heading_bias_rad=heading_bias,
            initial_pos_var=initial_pos_var,
            initial_heading_var=initial_heading_var,
            seed=seed,
        )

        self._latest_gt: Odometry | None = None
        self._last_gt_xy: tuple[float, float] | None = None

        # Honesty rule H2: odom_noise is a sensor-model node and may read /sim/*.
        self.create_subscription(Odometry, "/sim/gt/odom", self._on_gt_odom, 10)
        self._odom_pub = self.create_publisher(Odometry, "/vehicle/nav/odom", 10)
        self._tf_pub = TransformBroadcaster(self)

        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f"Initialized odom_noise: drift_rate={drift_rate}, "
            f"heading_bias_rad={heading_bias}, seed={seed}, "
            f"publish_rate_hz={publish_rate_hz}"
        )

    def _on_gt_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_gt_xy is not None:
            self._model.step(x - self._last_gt_xy[0], y - self._last_gt_xy[1])
        self._last_gt_xy = (x, y)
        self._latest_gt = msg

    def _on_timer(self) -> None:
        gt = self._latest_gt
        if gt is None:
            self.get_logger().warning(
                "No /sim/gt/odom received yet; nothing to publish.",
                throttle_duration_sec=5.0,
            )
            return

        stamp = self.get_clock().now().to_msg()
        pose = gt.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        noisy_x, noisy_y, noisy_yaw = self._model.noisy_pose(
            pose.position.x, pose.position.y, yaw
        )
        quat_x, quat_y, quat_z, quat_w = quaternion_about_yaw(noisy_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = noisy_x
        odom.pose.pose.position.y = noisy_y
        odom.pose.pose.position.z = pose.position.z
        odom.pose.pose.orientation.x = quat_x
        odom.pose.pose.orientation.y = quat_y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w

        pos_var = self._model.position_variance()
        covariance = [0.0] * 36
        covariance[0] = pos_var  # x
        covariance[7] = pos_var  # y
        covariance[35] = self._model.heading_variance()  # yaw
        odom.pose.covariance = covariance
        odom.twist = gt.twist

        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = noisy_x
        transform.transform.translation.y = noisy_y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = quat_x
        transform.transform.rotation.y = quat_y
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w
        self._tf_pub.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
