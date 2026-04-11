#!/usr/bin/env python3
"""
Ground truth localization relay for AWSIM.
Converts AWSIM's ground truth Odometry to PoseWithCovarianceStamped,
TwistWithCovarianceStamped, and TF broadcast for Autoware's localization pipeline.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, TransformStamped, PoseStamped, AccelWithCovarianceStamped
from autoware_vehicle_msgs.msg import VelocityReport
from tf2_ros import TransformBroadcaster


class GroundTruthRelay(Node):
    def __init__(self):
        super().__init__('ground_truth_relay')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/pose_with_covariance',
            10
        )

        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/localization/twist_estimator/twist_with_covariance',
            10
        )

        # Also publish directly to kinematic_state for modules that read it
        self.odom_pub = self.create_publisher(
            Odometry,
            '/localization/kinematic_state',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish to pose_twist_fusion_filter output to satisfy diagnostics
        self.fusion_pose_pub = self.create_publisher(
            PoseStamped,
            '/localization/pose_twist_fusion_filter/pose',
            10
        )

        self.sub = self.create_subscription(
            Odometry,
            '/awsim/ground_truth/localization/kinematic_state',
            self.odom_callback,
            sensor_qos
        )

        # Publish acceleration (derived from velocity changes)
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped,
            '/localization/acceleration',
            10
        )

        # Subscribe to AWSIM's smoothed velocity report (more stable than raw odometry twist)
        self.vel_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.vel_callback,
            10
        )
        self.smoothed_vel_x = 0.0
        self.smoothed_vel_y = 0.0
        self.smoothed_heading_rate = 0.0

        self.get_logger().info('Ground truth relay started')
        self.received = False
        self.last_twist = None
        self.last_stamp = None

    def vel_callback(self, msg: VelocityReport):
        self.smoothed_vel_x = msg.longitudinal_velocity
        self.smoothed_vel_y = msg.lateral_velocity
        self.smoothed_heading_rate = msg.heading_rate

    def odom_callback(self, msg: Odometry):
        if not self.received:
            self.get_logger().info(
                f'First ground truth received: x={msg.pose.pose.position.x:.1f}, '
                f'y={msg.pose.pose.position.y:.1f}, z={msg.pose.pose.position.z:.1f}'
            )
            self.received = True

        # Publish PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose

        # Override twist with AWSIM's smoothed velocity report (filters physics oscillation)
        msg.twist.twist.linear.x = self.smoothed_vel_x
        msg.twist.twist.linear.y = self.smoothed_vel_y
        msg.twist.twist.angular.z = self.smoothed_heading_rate

        # Publish TwistWithCovarianceStamped
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = msg.header
        twist_msg.header.frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'
        twist_msg.twist = msg.twist

        # Publish Odometry directly to kinematic_state
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist

        self.pose_pub.publish(pose_msg)
        self.twist_pub.publish(twist_msg)
        self.odom_pub.publish(odom_msg)

        # Publish PoseStamped for pose_twist_fusion diagnostics
        fusion_msg = PoseStamped()
        fusion_msg.header = msg.header
        fusion_msg.header.frame_id = 'map'
        fusion_msg.pose = msg.pose.pose
        self.fusion_pose_pub.publish(fusion_msg)

        # Publish acceleration (finite difference of twist)
        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header = msg.header
        accel_msg.header.frame_id = 'base_link'
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_twist is not None and self.last_stamp is not None:
            dt = stamp_sec - self.last_stamp
            if dt > 1e-6:
                accel_msg.accel.accel.linear.x = (msg.twist.twist.linear.x - self.last_twist.linear.x) / dt
                accel_msg.accel.accel.linear.y = (msg.twist.twist.linear.y - self.last_twist.linear.y) / dt
                accel_msg.accel.accel.linear.z = (msg.twist.twist.linear.z - self.last_twist.linear.z) / dt
        self.last_twist = msg.twist.twist
        self.last_stamp = stamp_sec
        self.accel_pub.publish(accel_msg)

        # Broadcast map -> base_link TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GroundTruthRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
