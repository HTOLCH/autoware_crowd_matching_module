#!/usr/bin/env python3
"""
Object detection relay with lightweight tracking for crowd-matching module.
Takes raw clustering detections (UNKNOWN labels, zero velocity) and:
  1. Transforms positions from base_link to map frame using ego pose
  2. Optionally filters detections to a corridor around the planned trajectory
     (used on the real bus to reject walls/furniture; pass-through in sim)
  3. Tracks detections across frames via nearest-neighbor matching
  4. Estimates per-pedestrian velocity from position deltas
  5. Publishes PredictedObjects with PEDESTRIAN classification and estimated velocities
"""

import math

import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import (
    DetectedObjects, PredictedObjects, PredictedObject,
    PredictedPath, ObjectClassification
)
from autoware_planning_msgs.msg import Trajectory
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

        # Trajectory for corridor filter (bus deployment).
        # If no trajectory is ever received, the filter is a pass-through.
        self.declare_parameter('lane_corridor_width', 4.0)
        self.declare_parameter('max_forward_distance', 25.0)
        self.lane_corridor_width = float(
            self.get_parameter('lane_corridor_width').value)
        self.max_forward_distance = float(
            self.get_parameter('max_forward_distance').value)
        self.traj_sub = self.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self.trajectory_callback,
            10
        )
        self.trajectory_points = []   # list of (x, y) in map frame

        # Tracking state: list of {pos: (mx, my), vel: (vx, vy), age: int}
        self.prev_tracks = []
        self.prev_stamp = None
        self.match_threshold = 1.0   # max distance (m) to match between frames
        self.vel_alpha = 0.4         # EMA smoothing for velocity (0-1, lower = smoother)
        self.min_track_age = 3       # frames before velocity is published (lets EMA converge)

        # Throttled filter stats
        self.kept_count = 0
        self.rejected_corridor = 0
        self.rejected_forward = 0
        self.rejected_ground = 0
        self.passthrough_count = 0
        self.last_log_time = 0.0

        # Cache and republish at 20Hz
        self.latest_out = PredictedObjects()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Object relay started — tracking + velocity estimation')
        self.received = False

    def ego_callback(self, msg: Odometry):
        self.ego_pose = msg.pose.pose

    def trajectory_callback(self, msg: Trajectory):
        self.trajectory_points = [
            (pt.pose.position.x, pt.pose.position.y) for pt in msg.points
        ]

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

        # Transform all detections to map frame, applying ground / forward filters.
        # Corridor filter is applied LATER (at publish time) so velocity tracking
        # persists for pedestrians that briefly exit then re-enter the corridor.
        dets = []
        for det in msg.objects:
            p = det.kinematics.pose_with_covariance.pose.position
            # 1. Ground plane filter (pedestrians > 0.3m above base_link)
            if p.z < 0.3:
                self.rejected_ground += 1
                continue
            # 2. Forward distance filter (base_link x is forward)
            if p.x < 0.0 or p.x > self.max_forward_distance:
                self.rejected_forward += 1
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

        # Build output, applying corridor filter at publish-time
        out = PredictedObjects()
        out.header = msg.header
        out.header.frame_id = 'map'

        corridor_active = len(self.trajectory_points) > 0
        corridor_half_sq = (self.lane_corridor_width / 2.0) ** 2

        for t in new_tracks:
            # Corridor gate (publish-time): runs against tracked-and-velocity-estimated
            # pedestrian pose. Tracking continues through brief excursions outside.
            if corridor_active:
                tmx, tmy = t['pos']
                min_dist_sq = min(
                    (tmx - tx) ** 2 + (tmy - ty) ** 2
                    for (tx, ty) in self.trajectory_points
                )
                if min_dist_sq > corridor_half_sq:
                    self.rejected_corridor += 1
                    continue
                self.kept_count += 1
            else:
                self.passthrough_count += 1

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

        # Throttled stats log (~1 Hz)
        log_now = self.get_clock().now().nanoseconds * 1e-9
        if log_now - self.last_log_time > 1.0:
            if corridor_active:
                self.get_logger().info(
                    f'Corridor filter: {self.kept_count} kept, '
                    f'{self.rejected_corridor} corridor-reject, '
                    f'{self.rejected_forward} forward-reject, '
                    f'{self.rejected_ground} ground-reject')
            else:
                self.get_logger().warn(
                    f'No trajectory yet — corridor filter DISABLED, '
                    f'{self.passthrough_count} tracks passed through unfiltered')
            self.kept_count = 0
            self.rejected_corridor = 0
            self.rejected_forward = 0
            self.rejected_ground = 0
            self.passthrough_count = 0
            self.last_log_time = log_now

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
