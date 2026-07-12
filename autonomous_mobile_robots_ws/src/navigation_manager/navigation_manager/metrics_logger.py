#!/usr/bin/env python3
"""Phase 4 — A/B experiment metrics logger.

Logs one CSV row per navigation run:
    timestamp, predictive, duration_s, min_human_dist_m, mean_close_dist_m, result

Where:
  - predictive       : value of the 'predictive' parameter (tag your condition)
  - duration_s       : time from goal EXECUTING to terminal status (sim time)
  - min_human_dist_m : minimum robot-to-human distance during the run
  - mean_close_dist_m: mean distance over samples where a human was within 3 m
  - result           : SUCCEEDED / ABORTED / CANCELED

Robot pose comes from TF (map -> base_footprint).
Human positions come from /human_tracks markers (ns 'humans') published by
human_predictor. Run boundaries come from the navigate_to_pose action status
topic, so runs are detected automatically whether goals come from RViz or
goal_navigator.

Usage (own terminal, alongside the full stack):
    ros2 run navigation_manager metrics_logger --ros-args -p predictive:=true

Team Terminators — navigation_manager package, ROS 2 Jazzy.
"""

import csv
import math
import os
from datetime import datetime

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

CSV_PATH = os.path.expanduser('~/phase4_metrics.csv')
SAMPLE_RATE = 5.0        # Hz
CLOSE_RANGE = 3.0        # m; "human nearby" threshold for mean_close_dist

TERMINAL = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('predictive', True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.humans: list[tuple[float, float]] = []
        self.create_subscription(MarkerArray, '/human_tracks', self.tracks_cb, 10)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self.status_cb, 10)
        self.create_timer(1.0 / SAMPLE_RATE, self.sample)

        # per-run state
        self.run_active = False
        self.run_start_t = 0.0
        self.min_dist = math.inf
        self.close_samples: list[float] = []

        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'timestamp', 'predictive', 'duration_s',
                    'min_human_dist_m', 'mean_close_dist_m', 'result'])
        self.get_logger().info(f'Logging runs to {CSV_PATH}')

    # ------------------------------------------------------------ callbacks

    def tracks_cb(self, msg: MarkerArray):
        self.humans = [(m.pose.position.x, m.pose.position.y)
                       for m in msg.markers if m.ns == 'humans']

    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]

        if latest.status == GoalStatus.STATUS_EXECUTING and not self.run_active:
            self.run_active = True
            self.run_start_t = self.now_s()
            self.min_dist = math.inf
            self.close_samples = []
            self.get_logger().info('Run started')

        elif latest.status in TERMINAL and self.run_active:
            self.run_active = False
            self.write_row(TERMINAL[latest.status])

    # -------------------------------------------------------------- sampling

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def sample(self):
        if not self.run_active or not self.humans:
            return
        robot = self.robot_xy()
        if robot is None:
            return
        rx, ry = robot
        d = min(math.hypot(hx - rx, hy - ry) for hx, hy in self.humans)
        self.min_dist = min(self.min_dist, d)
        if d <= CLOSE_RANGE:
            self.close_samples.append(d)

    # --------------------------------------------------------------- output

    def write_row(self, result: str):
        duration = self.now_s() - self.run_start_t
        min_d = round(self.min_dist, 3) if math.isfinite(self.min_dist) else ''
        mean_close = (round(sum(self.close_samples) / len(self.close_samples), 3)
                      if self.close_samples else '')
        predictive = self.get_parameter('predictive').value

        with open(CSV_PATH, 'a', newline='') as f:
            csv.writer(f).writerow([
                datetime.now().isoformat(timespec='seconds'),
                predictive, round(duration, 1), min_d, mean_close, result])

        self.get_logger().info(
            f'Run logged: {result} | {duration:.1f} s | '
            f'min human dist: {min_d} m | mean(<{CLOSE_RANGE} m): {mean_close}')


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
