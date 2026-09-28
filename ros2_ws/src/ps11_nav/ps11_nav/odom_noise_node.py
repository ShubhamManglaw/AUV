"""ROS 2 node for odom_noise (§9.2, T1.3)."""

from __future__ import annotations

import math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros

from ps11_common.params import load_yaml
from ps11_nav.odom_noise import OdomNoiseModel


def euler_from_quaternion(
    x: float, y: float, z: float, w: float
) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class OdomNoiseNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_noise")

        nav_cfg = load_yaml("nav.yaml").get("odom_noise", {})
        drift_rate = float(nav_cfg.get("position_drift_rate", 0.005))
        heading_bias = float(nav_cfg.get("heading_bias_rad", 0.00872665))
        init_cov_pos = float(nav_cfg.get("initial_covariance_pos", 0.01))
        init_cov_heading = float(nav_cfg.get("initial_covariance_heading", 0.001))
        seed_val = nav_cfg.get("seed", 0)
        seed = int(seed_val) if seed_val is not None else 0

        self.model = OdomNoiseModel(
            position_drift_rate=drift_rate,
            heading_bias_rad=heading_bias,
            initial_covariance_pos=init_cov_pos,
            initial_covariance_heading=init_cov_heading,
            seed=seed,
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_gt = self.create_subscription(
            Odometry,
            "/sim/gt/odom",
            self._on_gt_odom,
            qos,
        )
        self.pub_nav = self.create_publisher(
            Odometry,
            "/vehicle/nav/odom",
            qos,
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            "odom_noise node initialized (TF map -> base_link, /vehicle/nav/odom)"
        )

    def _on_gt_odom(self, msg: Odometry) -> None:
        gt_pos = msg.pose.pose.position
        gt_ori = msg.pose.pose.orientation
        gt_roll, gt_pitch, gt_yaw = euler_from_quaternion(
            gt_ori.x, gt_ori.y, gt_ori.z, gt_ori.w
        )

        noisy_x, noisy_y, noisy_z, noisy_roll, noisy_pitch, noisy_yaw, cov = (
            self.model.update(
                gt_x=gt_pos.x,
                gt_y=gt_pos.y,
                gt_z=gt_pos.z,
                gt_roll=gt_roll,
                gt_pitch=gt_pitch,
                gt_yaw=gt_yaw,
            )
        )

        qx, qy, qz, qw = quaternion_from_euler(noisy_roll, noisy_pitch, noisy_yaw)

        # Publish noisy Odometry
        noisy_odom = Odometry()
        noisy_odom.header.stamp = msg.header.stamp
        noisy_odom.header.frame_id = "map"
        noisy_odom.child_frame_id = "base_link"

        noisy_odom.pose.pose.position.x = noisy_x
        noisy_odom.pose.pose.position.y = noisy_y
        noisy_odom.pose.pose.position.z = noisy_z

        noisy_odom.pose.pose.orientation.x = qx
        noisy_odom.pose.pose.orientation.y = qy
        noisy_odom.pose.pose.orientation.z = qz
        noisy_odom.pose.covariance = cov

        # Copy velocity in child frame
        noisy_odom.twist = msg.twist

        self.pub_nav.publish(noisy_odom)

        # Broadcast TF map -> base_link (§9.4)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = noisy_x
        t.transform.translation.y = noisy_y
        t.transform.translation.z = noisy_z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomNoiseNode()
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
