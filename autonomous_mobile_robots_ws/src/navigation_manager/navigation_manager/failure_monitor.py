#!/usr/bin/env python3
"""Phase 5 — Failure-aware supervisor FSM.

Watches odometry, commanded velocity, and battery; publishes /robot_state and
intervenes when Nav2's own behavior tree can't:

  STUCK      commanded motion but <10 cm displacement over 5 s
             -> escalating recovery: clear costmaps -> spin -> backup -> abort
  DEGRADED   actual speed persistently < 50% of commanded -> warn (report metric)
  LOW_BATTERY battery < 20% -> cancel goal, navigate to charger, wait, resume

States: NOMINAL, STUCK, RECOVERING, DEGRADED, LOW_BATTERY, CHARGING, FAILED

Usage (with full stack + battery_simulator running):
    ros2 run navigation_manager failure_monitor --ros-args -p use_sim_time:=true

Team Terminators — navigation_manager package, ROS 2 Jazzy.
"""

import math
from collections import deque
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

# ------------------------------- tuning -----------------------------------
CHECK_PERIOD = 1.0        # s; FSM tick
STUCK_WINDOW = 5.0        # s; observation window
STUCK_MIN_DISP = 0.10     # m; displacement below this = not moving
CMD_ACTIVE_MIN = 0.05     # m/s; commanded speed above this = "trying to move"
DEGRADED_RATIO = 0.5      # actual/commanded speed ratio threshold
DEGRADED_HOLD = 4.0       # s; ratio must stay low this long
BATTERY_LOW = 0.20        # fraction
BATTERY_RESUME = 0.80     # fraction; leave CHARGING above this
RECOVERY_SETTLE = 6.0     # s; wait after each recovery level before re-check
# ---------------------------------------------------------------------------


class RobotState(Enum):
    NOMINAL = 'NOMINAL'
    STUCK = 'STUCK'
    RECOVERING = 'RECOVERING'
    DEGRADED = 'DEGRADED'
    LOW_BATTERY = 'LOW_BATTERY'
    CHARGING = 'CHARGING'
    FAILED = 'FAILED'


class FailureMonitor(Node):

    def __init__(self):
        super().__init__('failure_monitor')
        self.declare_parameter('charger_x', 0.0)
        self.declare_parameter('charger_y', 0.0)

        self.state = RobotState.NOMINAL
        self.odom_window: deque = deque()   # (t, x, y)
        self.actual_speed = 0.0
        self.cmd_speed = 0.0
        self.cmd_stamp = 0.0
        self.battery = 1.0
        self.degraded_since = None
        self.recovery_level = 0
        self.recovery_deadline = 0.0
        self.busy = False                    # an async recovery in flight

        # I/O
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 20)
        self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 5)
        self.state_pub = self.create_publisher(String, '/robot_state', 5)

        # Nav2 interfaces
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self.create_timer(CHECK_PERIOD, self.tick)
        self.get_logger().info('Failure monitor up — state NOMINAL')

    # ------------------------------------------------------------ callbacks

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def odom_cb(self, msg: Odometry):
        t = self.now_s()
        p = msg.pose.pose.position
        self.odom_window.append((t, p.x, p.y))
        while self.odom_window and t - self.odom_window[0][0] > STUCK_WINDOW:
            self.odom_window.popleft()
        v = msg.twist.twist.linear
        self.actual_speed = math.hypot(v.x, v.y)

    def cmd_cb(self, msg: TwistStamped):
        self.cmd_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
        self.cmd_stamp = self.now_s()

    def batt_cb(self, msg: BatteryState):
        self.battery = msg.percentage

    # ------------------------------------------------------------- the FSM

    def set_state(self, new: 'RobotState'):
        if new != self.state:
            self.get_logger().warn(f'STATE: {self.state.value} -> {new.value}')
            self.state = new
        self.state_pub.publish(String(data=self.state.value))

    def tick(self):
        self.state_pub.publish(String(data=self.state.value))

        # Battery has top priority in every state except already-handling ones
        if (self.battery < BATTERY_LOW
                and self.state not in (RobotState.LOW_BATTERY,
                                       RobotState.CHARGING,
                                       RobotState.FAILED)):
            self.enter_low_battery()
            return

        if self.state == RobotState.NOMINAL:
            if self.detect_stuck():
                self.set_state(RobotState.STUCK)
                self.recovery_level = 0
                self.start_next_recovery()
            elif self.detect_degraded():
                self.set_state(RobotState.DEGRADED)

        elif self.state == RobotState.DEGRADED:
            if self.detect_stuck():
                self.set_state(RobotState.STUCK)
                self.recovery_level = 0
                self.start_next_recovery()
            elif not self.detect_degraded():
                self.set_state(RobotState.NOMINAL)

        elif self.state == RobotState.RECOVERING:
            if self.busy or self.now_s() < self.recovery_deadline:
                return
            if not self.detect_stuck(strict=False):
                self.get_logger().info(
                    f'Recovery L{self.recovery_level} succeeded — moving again')
                self.set_state(RobotState.NOMINAL)
            else:
                self.start_next_recovery()

        elif self.state == RobotState.CHARGING:
            if self.battery >= BATTERY_RESUME:
                self.get_logger().info('Recharged — resuming NOMINAL')
                self.set_state(RobotState.NOMINAL)

        elif self.state == RobotState.LOW_BATTERY:
            # waiting for arrival at charger; battery_simulator flips to
            # charging when in radius — detect via rising battery
            if self.battery >= BATTERY_LOW:
                self.set_state(RobotState.CHARGING)

    # ------------------------------------------------------------ detectors

    def detect_stuck(self, strict: bool = True) -> bool:
        if len(self.odom_window) < 5:
            return False
        t0, x0, y0 = self.odom_window[0]
        t1, x1, y1 = self.odom_window[-1]
        if t1 - t0 < STUCK_WINDOW * 0.8:
            return False
        displacement = math.hypot(x1 - x0, y1 - y0)
        commanding = (self.cmd_speed > CMD_ACTIVE_MIN
                      and self.now_s() - self.cmd_stamp < 1.0)
        if strict:
            return commanding and displacement < STUCK_MIN_DISP
        return displacement < STUCK_MIN_DISP

    def detect_degraded(self) -> bool:
        commanding = (self.cmd_speed > 0.10
                      and self.now_s() - self.cmd_stamp < 1.0)
        low_ratio = commanding and (self.actual_speed / self.cmd_speed) < DEGRADED_RATIO
        if low_ratio:
            if self.degraded_since is None:
                self.degraded_since = self.now_s()
            return self.now_s() - self.degraded_since >= DEGRADED_HOLD
        self.degraded_since = None
        return False

    # ------------------------------------------------- recovery escalation

    def start_next_recovery(self):
        self.recovery_level += 1
        self.set_state(RobotState.RECOVERING)
        self.odom_window.clear()          # fresh window for the re-check
        self.recovery_deadline = self.now_s() + RECOVERY_SETTLE

        if self.recovery_level == 1:
            self.get_logger().warn('Recovery L1: clearing costmaps')
            for cli in (self.clear_global, self.clear_local):
                if cli.wait_for_service(timeout_sec=1.0):
                    cli.call_async(ClearEntireCostmap.Request())

        elif self.recovery_level == 2:
            self.get_logger().warn('Recovery L2: spin 90 deg')
            self.send_behavior(self.spin_client,
                               Spin.Goal(target_yaw=1.57))

        elif self.recovery_level == 3:
            self.get_logger().warn('Recovery L3: backup 0.15 m')
            goal = BackUp.Goal()
            goal.target.x = 0.15
            goal.speed = 0.05
            self.send_behavior(self.backup_client, goal)

        else:
            self.get_logger().error(
                'Recovery exhausted — cancelling goal, state FAILED')
            self.nav_client.wait_for_server(timeout_sec=1.0)
            self.nav_client._cancel_goal_async = None  # noop guard
            # cancel all outstanding navigation goals
            try:
                self.nav_client._client.cancel_all_goals_async()  # best effort
            except Exception:
                pass
            self.set_state(RobotState.FAILED)

    def send_behavior(self, client: ActionClient, goal):
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Behavior server unavailable')
            return
        self.busy = True
        fut = client.send_goal_async(goal)
        fut.add_done_callback(self.behavior_accepted)

    def behavior_accepted(self, fut):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self.behavior_done)

    def behavior_done(self, _fut):
        self.busy = False
        self.recovery_deadline = self.now_s() + RECOVERY_SETTLE

    # ---------------------------------------------------------- low battery

    def enter_low_battery(self):
        self.get_logger().warn(
            f'Battery {self.battery*100:.0f}% < {BATTERY_LOW*100:.0f}% — '
            'cancelling goal, heading to charger')
        self.set_state(RobotState.LOW_BATTERY)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.get_parameter('charger_x').value
        goal.pose.pose.position.y = self.get_parameter('charger_y').value
        goal.pose.pose.orientation.w = 1.0

        if self.nav_client.wait_for_server(timeout_sec=2.0):
            # Sending a new goal preempts the current one in Nav2's default
            # single-goal policy — cancel + redirect in one call.
            self.nav_client.send_goal_async(goal)
        else:
            self.get_logger().error('navigate_to_pose unavailable for charger run')


def main():
    rclpy.init()
    node = FailureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
