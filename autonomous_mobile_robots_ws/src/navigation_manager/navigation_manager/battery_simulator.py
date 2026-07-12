#!/usr/bin/env python3
"""Phase 5 — Simulated battery for the TurtleBot3 (Gazebo publishes none).

Publishes sensor_msgs/BatteryState on /battery_state at 1 Hz.
Drains linearly from 100% over 'drain_minutes'. Recharges (faster) whenever
the robot is within 'charger_radius' of the charger pose.

Parameters:
    drain_minutes   (default 10.0)  full -> empty time while active
    charge_minutes  (default 2.0)   empty -> full time while at charger
    charger_x, charger_y (default 0.0, 0.0)  charger position in map frame
    charger_radius  (default 0.5)   m

Usage:
    ros2 run navigation_manager battery_simulator --ros-args \
        -p use_sim_time:=true -p drain_minutes:=10.0 \
        -p charger_x:=-2.0 -p charger_y:=1.0

Team Terminators — navigation_manager package, ROS 2 Jazzy.
"""

import math

import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatterySimulator(Node):

    def __init__(self):
        super().__init__('battery_simulator')
        self.declare_parameter('drain_minutes', 10.0)
        self.declare_parameter('charge_minutes', 2.0)
        self.declare_parameter('charger_x', 0.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('charger_radius', 0.5)

        self.percentage = 1.0  # 0.0 .. 1.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_timer(1.0, self.tick)
        self.get_logger().info('Battery simulator up: 100%')

    def at_charger(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        dx = tf.transform.translation.x - self.get_parameter('charger_x').value
        dy = tf.transform.translation.y - self.get_parameter('charger_y').value
        return math.hypot(dx, dy) <= self.get_parameter('charger_radius').value

    def tick(self):
        if self.at_charger():
            rate = 1.0 / (self.get_parameter('charge_minutes').value * 60.0)
            self.percentage = min(1.0, self.percentage + rate)
        else:
            rate = 1.0 / (self.get_parameter('drain_minutes').value * 60.0)
            self.percentage = max(0.0, self.percentage - rate)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = self.percentage
        msg.voltage = 11.1 * (0.85 + 0.15 * self.percentage)
        msg.present = True
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.at_charger()
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.pub.publish(msg)

        pct = int(self.percentage * 100)
        if pct % 10 == 0:
            self.get_logger().info(f'Battery: {pct}%',
                                   throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
