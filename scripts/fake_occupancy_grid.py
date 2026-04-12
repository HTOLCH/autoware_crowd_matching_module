#!/usr/bin/env python3
"""
Publishes an empty occupancy grid map at low rate so behavior_path_planner
can proceed without a full perception/occupancy_grid pipeline running.

This is a sim-only workaround. On the bus, the real perception stack
publishes /perception/occupancy_grid_map/map. In sim with PERCEPTION_MODE
set to minimal or none, nothing produces it.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class FakeOccupancyGrid(Node):
    def __init__(self):
        super().__init__('fake_occupancy_grid')
        self.pub = self.create_publisher(
            OccupancyGrid, '/perception/occupancy_grid_map/map', 10)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz
        self.get_logger().info('Fake occupancy grid publisher started')

    def publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.5       # 0.5 m per cell
        msg.info.width = 200             # 100 m wide
        msg.info.height = 200            # 100 m deep
        # Centered on map origin (-50m, -50m)
        msg.info.origin.position.x = -50.0
        msg.info.origin.position.y = -50.0
        msg.info.origin.orientation.w = 1.0
        # All cells free (value 0)
        msg.data = [0] * (msg.info.width * msg.info.height)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
