#!/usr/bin/env python3

"""Send one Nav2 goal and report its final settled pose error."""

import argparse
import csv
from datetime import datetime
import math
import os
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


class NavGoalEvaluator(Node):
    def __init__(self, args):
        super().__init__(
            'nav_goal_evaluator',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time)
            ],
        )
        self.goal_x = args.x
        self.goal_y = args.y
        self.goal_yaw = args.yaw
        self.settle_seconds = args.settle_seconds
        self.linear_threshold = args.linear_threshold
        self.angular_threshold = args.angular_threshold
        self.csv_path = args.csv
        self.start_time = time.monotonic()
        self.timeout_seconds = args.timeout
        self.last_linear_speed = float('inf')
        self.last_angular_speed = float('inf')
        self.received_odom = False
        self.last_motion_time = None
        self.action_complete = False
        self.result_status = None
        self.recoveries = None
        self.navigation_time = None
        self.reported = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Odometry, 'odom', self.odom_callback, 20)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 20)
        self.timer = self.create_timer(0.1, self.tick)

    def odom_callback(self, message):
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        self.last_linear_speed = math.hypot(linear.x, linear.y)
        self.last_angular_speed = abs(angular.z)
        self.received_odom = True
        if (self.last_linear_speed > self.linear_threshold or
                self.last_angular_speed > self.angular_threshold):
            self.last_motion_time = time.monotonic()

    def cmd_vel_callback(self, message):
        linear_speed = math.hypot(message.linear.x, message.linear.y)
        angular_speed = abs(message.angular.z)
        if (linear_speed > self.linear_threshold or
                angular_speed > self.angular_threshold):
            self.last_motion_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if not self.action_complete:
            if now - self.start_time > self.timeout_seconds:
                self.finish_with_error('Timed out waiting for navigation result.')
                return
            if self.action_client.server_is_ready():
                self.send_goal()
            return

        if self.last_motion_time is None:
            self.last_motion_time = now
        if now - self.last_motion_time >= self.settle_seconds:
            self.report()

    def send_goal(self):
        self.timer.cancel()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        self.get_logger().info(
            'Sending goal: x=%.3f m, y=%.3f m, yaw=%.3f rad' %
            (self.goal_x, self.goal_y, self.goal_yaw))
        self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback).add_done_callback(
                self.goal_response_callback)
        self.timer = self.create_timer(0.1, self.tick)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.finish_with_error('Nav2 rejected the goal.')
            return
        self.get_logger().info('Goal accepted. Waiting for robot to arrive.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.recoveries = feedback.number_of_recoveries
        self.navigation_time = (
            feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9)

    def result_callback(self, future):
        result = future.result()
        self.result_status = result.status
        self.action_complete = True
        self.last_motion_time = time.monotonic()
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(result.status, str(result.status))
        self.get_logger().info(
            'Navigation result: %s. Waiting %.1f s for a settled pose.' %
            (status_name, self.settle_seconds))

    def report(self):
        if self.reported:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except TransformException as error:
            self.finish_with_error(
                'Cannot read map -> base_footprint transform: %s' % error)
            return

        self.reported = True

        translation = transform.transform.translation
        final_yaw = quaternion_to_yaw(transform.transform.rotation)
        position_error = math.hypot(
            translation.x - self.goal_x, translation.y - self.goal_y)
        yaw_error = abs(normalize_angle(final_yaw - self.goal_yaw))
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(self.result_status, str(self.result_status))

        self.get_logger().info('--- Navigation evaluation ---')
        self.get_logger().info('Result: %s' % status_name)
        self.get_logger().info(
            'Final map pose: x=%.3f m, y=%.3f m, yaw=%.3f rad (%.1f deg)' %
            (translation.x, translation.y, final_yaw, math.degrees(final_yaw)))
        self.get_logger().info('Position error: %.3f m' % position_error)
        self.get_logger().info(
            'Yaw error: %.3f rad (%.1f deg)' %
            (yaw_error, math.degrees(yaw_error)))
        self.get_logger().info(
            'Settled odom speed: linear=%.4f m/s, angular=%.4f rad/s' %
            (self.last_linear_speed, self.last_angular_speed))
        if self.navigation_time is not None:
            self.get_logger().info(
                'Navigation time: %.2f s, recoveries: %d' %
                (self.navigation_time, self.recoveries))
        if not self.received_odom:
            self.get_logger().warn(
                'No /odom message was received; settled speed was not verified.')
        if self.csv_path:
            self.write_csv(
                status_name, translation.x, translation.y, final_yaw,
                position_error, yaw_error)
        self.destroy_node()
        rclpy.shutdown()

    def write_csv(self, status_name, final_x, final_y, final_yaw,
                  position_error, yaw_error):
        file_exists = os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            if not file_exists or os.path.getsize(self.csv_path) == 0:
                writer.writerow([
                    'timestamp', 'result', 'goal_x_m', 'goal_y_m', 'goal_yaw_rad',
                    'final_x_m', 'final_y_m', 'final_yaw_rad', 'position_error_m',
                    'yaw_error_rad', 'navigation_time_s', 'recoveries'])
            writer.writerow([
                datetime.now().isoformat(timespec='seconds'), status_name,
                self.goal_x, self.goal_y, self.goal_yaw, final_x, final_y,
                final_yaw, position_error, yaw_error, self.navigation_time,
                self.recoveries])
        self.get_logger().info('Appended evaluation row to %s' % self.csv_path)

    def finish_with_error(self, message):
        if self.reported:
            return
        self.reported = True
        self.get_logger().error(message)
        self.destroy_node()
        rclpy.shutdown()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Send a Nav2 goal and report final pose error after settling.')
    parser.add_argument('x', type=float, help='Goal x coordinate in map frame (m).')
    parser.add_argument('y', type=float, help='Goal y coordinate in map frame (m).')
    parser.add_argument('yaw', type=float, help='Goal yaw in map frame (rad).')
    parser.add_argument('--settle-seconds', type=float, default=1.5,
                        help='Required continuous stopped time (default: 1.5).')
    parser.add_argument('--linear-threshold', type=float, default=0.01,
                        help='Stopped linear speed threshold in m/s (default: 0.01).')
    parser.add_argument('--angular-threshold', type=float, default=0.02,
                        help='Stopped angular speed threshold in rad/s (default: 0.02).')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='Navigation timeout in wall-clock seconds (default: 180).')
    parser.add_argument('--csv', default='',
                        help='Optional CSV file to append one evaluation result.')
    parser.add_argument('--use-sim-time', action='store_true', default=True,
                        help='Use /clock from Gazebo (default: true).')
    return parser.parse_args()


def main():
    args = parse_arguments()
    rclpy.init()
    node = NavGoalEvaluator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
