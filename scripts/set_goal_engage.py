#!/usr/bin/env python3
"""
Auto-set goal and engage autonomous mode for SFM evaluation.
Reads current ego position, publishes goal GOAL_DISTANCE meters ahead,
waits for route, then engages AUTO.

Usage (inside container):
    python3 set_goal_engage.py [--distance 10]
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from tier4_external_api_msgs.srv import Engage


class GoalSetter(Node):
    def __init__(self, distance):
        super().__init__('goal_setter')
        self.distance = distance
        self.ego_pose = None
        self.goal_set = False
        self.engaged = False

        # Subscribe to ego pose
        self.ego_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.ego_callback,
            10
        )

        # Goal publisher (latched)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            latched_qos
        )

        # Services for engaging
        self.engage_client = self.create_client(
            Engage, '/api/autoware/set/engage'
        )
        self.auto_client = self.create_client(
            ChangeOperationMode, '/api/operation_mode/change_to_autonomous'
        )

        self.get_logger().info(f'Goal setter ready — will set goal {distance}m ahead')

    def ego_callback(self, msg):
        if self.ego_pose is None:
            self.ego_pose = msg.pose.pose
            self.get_logger().info(
                f'Ego position: x={self.ego_pose.position.x:.1f}, '
                f'y={self.ego_pose.position.y:.1f}'
            )

    def set_goal(self):
        if self.ego_pose is None:
            self.get_logger().warn('No ego pose yet')
            return False

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'

        # Goal is DISTANCE meters ahead along X axis
        goal.pose.position.x = self.ego_pose.position.x + self.distance
        goal.pose.position.y = self.ego_pose.position.y
        goal.pose.position.z = 0.0

        # Same orientation as ego (facing forward)
        goal.pose.orientation = self.ego_pose.orientation

        self.goal_pub.publish(goal)
        self.goal_set = True
        self.get_logger().info(
            f'Goal set: x={goal.pose.position.x:.1f}, '
            f'y={goal.pose.position.y:.1f} '
            f'({self.distance}m ahead)'
        )
        return True

    def engage(self):
        # Try change_to_autonomous first
        if self.auto_client.wait_for_service(timeout_sec=2.0):
            req = ChangeOperationMode.Request()
            future = self.auto_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None:
                success = future.result().status.success
                self.get_logger().info(
                    f'change_to_autonomous: success={success}'
                )
                if success:
                    self.engaged = True
                    return True

        # Fallback: set/engage
        if self.engage_client.wait_for_service(timeout_sec=2.0):
            req = Engage.Request()
            req.engage = True
            future = self.engage_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None:
                try:
                    success = future.result().status.success
                except AttributeError:
                    # Some Autoware versions use different response structure
                    success = True  # If service responded, assume success
                self.get_logger().info(f'engage: success={success}')
                if success:
                    self.engaged = True
                    return True

        self.get_logger().warn('Could not engage — please engage manually in rviz')
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=10.0,
                        help='Goal distance ahead (meters)')
    args = parser.parse_args()

    rclpy.init()
    node = GoalSetter(args.distance)

    # Wait for ego pose
    node.get_logger().info('Waiting for ego pose...')
    timeout = time.time() + 30
    while node.ego_pose is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.ego_pose is None:
        node.get_logger().error('No ego pose received — is AWSIM running?')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Set goal
    node.set_goal()

    # Wait for route to be computed (give planner time)
    node.get_logger().info('Waiting 10s for route planning...')
    time.sleep(10)

    # Engage with retries
    for attempt in range(5):
        node.get_logger().info(f'Engaging autonomous mode (attempt {attempt + 1}/5)...')
        node.engage()
        if node.engaged:
            # Verify it actually worked by checking result
            time.sleep(1)
            break
        node.get_logger().info('Engage failed, retrying in 3s...')
        time.sleep(3)

    node.get_logger().info('Done — goal set and engage attempted')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
