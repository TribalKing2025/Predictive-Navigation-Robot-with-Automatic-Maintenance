#!/usr/bin/env python3
"""Phase 4 — Predictive human-aware costmap layer.

Pipeline (single node, three separable stages):
  1. LegClusterDetector : /scan -> person candidate positions (map frame)
  2. KalmanTracker      : candidates -> tracks with velocity (CV Kalman filter)
  3. Prediction/grid    : moving tracks -> Gaussian "comet tail" OccupancyGrid

Publishes:
  /predicted_humans_costmap  (nav_msgs/OccupancyGrid, transient local, 2 Hz)
  /human_tracks              (visualization_msgs/MarkerArray, for RViz)

Consumed by Nav2 via a second StaticLayer instance (see nav2_params.yaml):
  predicted_humans_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_topic: /predicted_humans_costmap
    map_subscribe_transient_local: True
    subscribe_to_updates: True
    use_maximum: True

Usage (with Gazebo + Nav2 bringup running):
    ros2 run navigation_manager human_predictor

Team Terminators — navigation_manager package, ROS 2 Jazzy.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------- Tuning constants (see walkthrough) ----------------------
CLUSTER_GAP = 0.13        # m; gap between consecutive scan points that splits clusters
MIN_CLUSTER_PTS = 3       # discard smaller clusters (noise)
MIN_WIDTH = 0.05          # m; cluster width limits for a leg / person
MAX_WIDTH = 0.45
LEG_PAIR_DIST = 0.50      # m; merge two leg clusters closer than this into one person

GATE_DIST = 0.70          # m; association gate track <-> detection
MIN_HITS = 3              # detections before a track is confirmed
MAX_MISSES = 5            # consecutive misses before a track is deleted
HUMAN_SPEED_MIN = 0.15    # m/s; velocity gate: slower tracks are not "humans"

PREDICT_HORIZON = 2.5     # s; how far ahead to project
PREDICT_DT = 0.5          # s; projection step
SIGMA_NOW = 0.25          # m; Gaussian sigma at t=0
SIGMA_GROWTH = 0.12       # m per second of prediction (uncertainty growth)
PEAK_COST = 90            # 0-100; occupancy value at Gaussian center (100 = lethal)
PUBLISH_RATE = 2.0        # Hz; grid publish rate
# ---------------------------------------------------------------------------------


class Track:
    """One tracked person: constant-velocity Kalman filter, state [x y vx vy]."""

    _next_id = 0

    def __init__(self, x, y, stamp_s):
        self.id = Track._next_id
        Track._next_id += 1
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.diag([0.3, 0.3, 1.0, 1.0])
        self.hits = 1
        self.misses = 0
        self.last_stamp = stamp_s

        self.Q_base = np.diag([0.02, 0.02, 0.30, 0.30])  # process noise
        self.R = np.diag([0.05, 0.05])                    # measurement noise
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=float)

    def predict(self, dt):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q_base * max(dt, 1e-3)

    def update(self, zx, zy):
        z = np.array([zx, zy])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.hits += 1
        self.misses = 0

    @property
    def pos(self):
        return self.x[0], self.x[1]

    @property
    def vel(self):
        return self.x[2], self.x[3]

    @property
    def speed(self):
        return math.hypot(self.x[2], self.x[3])

    @property
    def confirmed(self):
        return self.hits >= MIN_HITS

    @property
    def is_human(self):
        return self.confirmed and self.speed >= HUMAN_SPEED_MIN


class HumanPredictor(Node):

    def __init__(self):
        super().__init__('human_predictor')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tracks: list[Track] = []
        self.map_meta: MapMetaData | None = None

        # Static map metadata (grid geometry template). map_server publishes
        # transient-local, so late joining is fine.
        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.grid_pub = self.create_publisher(OccupancyGrid,
                                              '/predicted_humans_costmap',
                                              map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/human_tracks', 10)

        self.create_timer(1.0 / PUBLISH_RATE, self.publish_grid)
        self.get_logger().info('human_predictor up — waiting for /map and /scan')

    # ------------------------------------------------------------- callbacks

    def map_cb(self, msg: OccupancyGrid):
        if self.map_meta is None:
            self.map_meta = msg.info
            self.get_logger().info(
                f'Grid template: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution} m')

    def scan_cb(self, scan: LaserScan):
        points = self.scan_to_map_points(scan)
        if points is None:
            return
        detections = self.detect_people(points)
        stamp_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        self.update_tracks(detections, stamp_s)

    # ------------------------------------------------------- stage 1: detect

    def scan_to_map_points(self, scan: LaserScan):
        """Polar scan -> Nx2 array of points in the map frame (2D transform)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', scan.header.frame_id, scan.header.stamp,
                timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        r, a = ranges[valid], angles[valid]

        xs = r * np.cos(a)
        ys = r * np.sin(a)
        mx = c * xs - s * ys + t.x
        my = s * xs + c * ys + t.y
        return np.column_stack((mx, my))

    def detect_people(self, pts: np.ndarray):
        """Jump-distance clustering + size filter + leg pairing."""
        if len(pts) < MIN_CLUSTER_PTS:
            return []

        clusters, current = [], [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - current[-1]) > CLUSTER_GAP:
                clusters.append(np.array(current))
                current = [p]
            else:
                current.append(p)
        clusters.append(np.array(current))

        # keep leg/person sized clusters
        candidates = []
        for cl in clusters:
            if len(cl) < MIN_CLUSTER_PTS:
                continue
            width = np.linalg.norm(cl[0] - cl[-1])
            if MIN_WIDTH <= width <= MAX_WIDTH:
                candidates.append(cl.mean(axis=0))

        # merge leg pairs into single person detections
        detections, used = [], set()
        for i, ci in enumerate(candidates):
            if i in used:
                continue
            partner = None
            for j in range(i + 1, len(candidates)):
                if j in used:
                    continue
                if np.linalg.norm(ci - candidates[j]) < LEG_PAIR_DIST:
                    partner = j
                    break
            if partner is not None:
                detections.append((ci + candidates[partner]) / 2.0)
                used.update((i, partner))
            else:
                detections.append(ci)
                used.add(i)
        return detections

    # -------------------------------------------------------- stage 2: track

    def update_tracks(self, detections, stamp_s):
        # time update
        for tr in self.tracks:
            tr.predict(max(stamp_s - tr.last_stamp, 1e-3))
            tr.last_stamp = stamp_s

        # greedy nearest-neighbour association
        unmatched = list(range(len(detections)))
        for tr in self.tracks:
            best_j, best_d = None, GATE_DIST
            px, py = tr.pos
            for j in unmatched:
                d = math.hypot(detections[j][0] - px, detections[j][1] - py)
                if d < best_d:
                    best_j, best_d = j, d
            if best_j is not None:
                tr.update(detections[best_j][0], detections[best_j][1])
                unmatched.remove(best_j)
            else:
                tr.misses += 1

        for j in unmatched:
            self.tracks.append(Track(detections[j][0], detections[j][1], stamp_s))

        self.tracks = [t for t in self.tracks if t.misses <= MAX_MISSES]

    # ------------------------------------------------------ stage 3: predict

    def publish_grid(self):
        if self.map_meta is None:
            return

        info = self.map_meta
        grid = np.zeros((info.height, info.width), dtype=np.float32)
        humans = [t for t in self.tracks if t.is_human]

        for tr in humans:
            px, py = tr.pos
            vx, vy = tr.vel
            t = 0.0
            while t <= PREDICT_HORIZON:
                cx = px + vx * t
                cy = py + vy * t
                sigma = SIGMA_NOW + SIGMA_GROWTH * t
                self.splat_gaussian(grid, info, cx, cy, sigma)
                t += PREDICT_DT

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = info
        msg.data = np.clip(grid, 0, 100).astype(np.int8).flatten().tolist()
        self.grid_pub.publish(msg)
        self.publish_markers(humans)

    def splat_gaussian(self, grid, info, cx, cy, sigma):
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        r_cells = int(3 * sigma / res)
        gx = int((cx - ox) / res)
        gy = int((cy - oy) / res)

        x0, x1 = max(gx - r_cells, 0), min(gx + r_cells + 1, info.width)
        y0, y1 = max(gy - r_cells, 0), min(gy + r_cells + 1, info.height)
        if x0 >= x1 or y0 >= y1:
            return

        xs = (np.arange(x0, x1) * res + ox) - cx
        ys = (np.arange(y0, y1) * res + oy) - cy
        XX, YY = np.meshgrid(xs, ys)
        g = PEAK_COST * np.exp(-(XX**2 + YY**2) / (2.0 * sigma**2))
        np.maximum(grid[y0:y1, x0:x1], g, out=grid[y0:y1, x0:x1])

    # ------------------------------------------------------------------ viz

    def publish_markers(self, humans):
        ma = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)

        now = self.get_clock().now().to_msg()
        for tr in humans:
            px, py = tr.pos
            vx, vy = tr.vel

            body = Marker()
            body.header.frame_id = 'map'
            body.header.stamp = now
            body.ns, body.id = 'humans', tr.id
            body.type, body.action = Marker.CYLINDER, Marker.ADD
            body.pose.position.x, body.pose.position.y = px, py
            body.pose.position.z = 0.6
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.4
            body.scale.z = 1.2
            body.color.r, body.color.g, body.color.b, body.color.a = 1.0, 0.4, 0.0, 0.8
            ma.markers.append(body)

            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns, arrow.id = 'velocity', tr.id
            arrow.type, arrow.action = Marker.ARROW, Marker.ADD
            arrow.points = [
                Point(x=px, y=py, z=0.1),
                Point(x=px + vx * PREDICT_HORIZON, y=py + vy * PREDICT_HORIZON, z=0.1),
            ]
            arrow.scale.x, arrow.scale.y = 0.05, 0.12
            arrow.color.g, arrow.color.a = 1.0, 0.9
            ma.markers.append(arrow)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = HumanPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
