#!/usr/bin/env python3
"""
Object detection relay with lightweight tracking for crowd-matching module simulation testing.
Takes raw clustering detections (UNKNOWN labels, zero velocity) and:
  1. Transforms positions from base_link to map frame using ego pose
  2. Tracks detections across frames via nearest-neighbor matching
  3. Estimates per-pedestrian velocity from position deltas
  4. Publishes PredictedObjects with PEDESTRIAN classification and estimated velocities

This is SIMULATION ONLY — on the real bus, the full perception pipeline handles this.
"""

import math

import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import (
    DetectedObjects, PredictedObjects, PredictedObject,
    PredictedPath, ObjectClassification
)
from nav_msgs.msg import Odometry


class ObjectRelay(Node):
    def __init__(self):
        super().__init__('object_relay')

        self.pub = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects',
            10
        )

        self.sub = self.create_subscription(
            DetectedObjects,
            '/perception/object_recognition/detection/clustering/objects',
            self.callback,
            10
        )

        # Ego pose for base_link -> map transform
        self.ego_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.ego_callback,
            10
        )
        self.ego_pose = None

        # Tracking state: list of {pos: (mx, my), vel: (vx, vy), age: int}
        self.prev_tracks = []
        self.prev_stamp = None
        self.match_threshold = 1.0   # max distance (m) to match between frames
        self.vel_alpha = 0.4         # EMA smoothing for velocity (0-1, lower = smoother)
        self.min_track_age = 3       # frames before velocity is published (lets EMA converge)

        # Cache and republish at 20Hz
        self.latest_out = PredictedObjects()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Object relay started — tracking + velocity estimation')
        self.received = False

    def ego_callback(self, msg: Odometry):
        self.ego_pose = msg.pose.pose

    def base_link_to_map(self, bx, by):
        """Transform a point from base_link to map frame using current ego pose."""
        q = self.ego_pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        c, s = math.cos(yaw), math.sin(yaw)
        ex = self.ego_pose.position.x
        ey = self.ego_pose.position.y
        return (ex + c * bx - s * by, ey + s * bx + c * by)

    def callback(self, msg: DetectedObjects):
        if self.ego_pose is None:
            return

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.received and len(msg.objects) > 0:
            self.get_logger().info(f'First detection: {len(msg.objects)} objects')
            self.received = True

        # Transform all detections to map frame, filtering ground-level clusters
        dets = []
        for det in msg.objects:
            p = det.kinematics.pose_with_covariance.pose.position
            # Filter ground plane detections — pedestrians are z > 0.3m in base_link
            if p.z < 0.3:
                continue
            mx, my = self.base_link_to_map(p.x, p.y)
            dets.append({'mx': mx, 'my': my, 'shape': det.shape})

        # Compute dt
        dt = 0.0
        if self.prev_stamp is not None:
            dt = now - self.prev_stamp
        can_track = 0.01 < dt < 1.0 and len(self.prev_tracks) > 0

        # Nearest-neighbor matching + velocity estimation
        new_tracks = []
        if can_track:
            used = [False] * len(self.prev_tracks)
            for d in dets:
                best_dist = self.match_threshold
                best_idx = -1
                for i, pt in enumerate(self.prev_tracks):
                    if used[i]:
                        continue
                    dist = math.hypot(d['mx'] - pt['pos'][0], d['my'] - pt['pos'][1])
                    if dist < best_dist:
                        best_dist = dist
                        best_idx = i

                if best_idx >= 0:
                    used[best_idx] = True
                    pt = self.prev_tracks[best_idx]
                    raw_vx = (d['mx'] - pt['pos'][0]) / dt
                    raw_vy = (d['my'] - pt['pos'][1]) / dt
                    a = self.vel_alpha
                    vx = a * raw_vx + (1.0 - a) * pt['vel'][0]
                    vy = a * raw_vy + (1.0 - a) * pt['vel'][1]
                    new_tracks.append({
                        'pos': (d['mx'], d['my']),
                        'vel': (vx, vy),
                        'age': pt.get('age', 0) + 1,
                        'shape': d['shape']
                    })
                else:
                    # New detection — no velocity yet
                    new_tracks.append({
                        'pos': (d['mx'], d['my']),
                        'vel': (0.0, 0.0),
                        'age': 0,
                        'shape': d['shape']
                    })
        else:
            for d in dets:
                new_tracks.append({
                    'pos': (d['mx'], d['my']),
                    'vel': (0.0, 0.0),
                    'age': 0,
                    'shape': d['shape']
                })

        # Build output
        out = PredictedObjects()
        out.header = msg.header
        out.header.frame_id = 'map'

        for t in new_tracks:
            pred = PredictedObject()
            pred.object_id.uuid = [0] * 16

            cls = ObjectClassification()
            cls.label = ObjectClassification.PEDESTRIAN
            cls.probability = 0.9
            pred.classification = [cls]
            pred.existence_probability = 0.9

            # Map-frame pose
            pred.kinematics.initial_pose_with_covariance.pose.position.x = t['pos'][0]
            pred.kinematics.initial_pose_with_covariance.pose.position.y = t['pos'][1]
            pred.kinematics.initial_pose_with_covariance.pose.position.z = 0.0
            pred.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0

            # Map-frame estimated velocity (only after EMA has converged)
            if t.get('age', 0) >= self.min_track_age:
                pred.kinematics.initial_twist_with_covariance.twist.linear.x = t['vel'][0]
                pred.kinematics.initial_twist_with_covariance.twist.linear.y = t['vel'][1]

            # Trivial predicted path
            path = PredictedPath()
            path.confidence = 1.0
            path.path = [pred.kinematics.initial_pose_with_covariance.pose]
            pred.kinematics.predicted_paths = [path]

            pred.shape = t['shape']
            out.objects.append(pred)

        self.prev_tracks = new_tracks
        self.prev_stamp = now
        self.latest_out = out

    def timer_callback(self):
        self.pub.publish(self.latest_out)


def main():
    rclpy.init()
    node = ObjectRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
