#!/usr/bin/env python3
"""Phase 3 — Goal navigation with recovery via Nav2.

Sends a NavigateToPose goal through nav2_simple_commander's BasicNavigator,
monitors feedback (including the recovery counter), and reports the result.

Usage (with Gazebo + nav2_bringup already running):
    ros2 run navigation_manager goal_navigator

Team Terminators — navigation_manager package, ROS 2 Jazzy.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


# --- Adjust these for your map -------------------------------------------
INITIAL_POSE_XY_YAW = (0.0, 0.0, 0.0)   # TB3 world default spawn
GOAL_XY_YAW = (3.7, 2.0, 0.0)             # target goal
GOAL_TIMEOUT_S = 180.0                    # cancel if it takes longer


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    """Build a PoseStamped in the map frame. Yaw simplified to 0 or supply quaternion."""
    import math
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def navigate_to(navigator: BasicNavigator, goal: PoseStamped, timeout_s: float = GOAL_TIMEOUT_S) -> TaskResult:
    """Send one goal and block until it finishes, logging feedback.

    Kept as a standalone function on purpose: Phase 6's task executor will
    call this with poses computed from object detections, and Phase 5's
    failure monitor consumes the same feedback signals.
    """
    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            elapsed = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
            navigator.get_logger().info(
                f'dist remaining: {feedback.distance_remaining:.2f} m | '
                f'eta: {eta:.0f} s | '
                f'recoveries: {feedback.number_of_recoveries}'
            )
            # Watchdog: give up (and let recovery/Phase 5 logic take over)
            if elapsed > timeout_s:
                navigator.get_logger().warn('Goal timed out — cancelling task')
                navigator.cancelTask()

    return navigator.getResult()


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Programmatic initial pose — replaces the RViz "2D Pose Estimate" click.
    initial = make_pose(navigator, *INITIAL_POSE_XY_YAW)
    navigator.setInitialPose(initial)

    # Blocks until the full Nav2 stack (amcl, planner, controller, BT) is active.
    navigator.waitUntilNav2Active()

    goal = make_pose(navigator, *GOAL_XY_YAW)
    navigator.get_logger().info(
        f'Navigating to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
    )

    result = navigate_to(navigator, goal)

    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal reached!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Goal was canceled (timeout or external cancel).')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error(
            'Goal failed after exhausting recoveries — '
            'Phase 5 failure handling would trigger here.'
        )

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
