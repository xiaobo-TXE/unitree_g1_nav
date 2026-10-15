#!/usr/bin/env python3
"""Select local navigation until a one-way terminal handoff to path_follow."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from path_follow.goal_utils import goal_identity, same_goal, valid_path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import SetBool


def finite_twist(command):
    return all(math.isfinite(value) for value in (
        command.linear.x, command.linear.y, command.linear.z,
        command.angular.x, command.angular.y, command.angular.z))


class SelectionState:
    """State machine independent of ROS executors; all ages use monotonic seconds."""

    def __init__(self, switch_distance=2.0, cmd_timeout=0.3, odom_timeout=0.3,
                 goal_yaw_mode='path'):
        for name, value in (('switch_distance', switch_distance),
                            ('cmd_timeout', cmd_timeout), ('odom_timeout', odom_timeout)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if goal_yaw_mode not in ('path', 'segment'):
            raise ValueError('goal_yaw_mode must be path or segment')
        self.switch_distance = switch_distance
        self.cmd_timeout = cmd_timeout
        self.odom_timeout = odom_timeout
        self.goal_yaw_mode = goal_yaw_mode
        self.goal = None
        self.position = None
        self.odom_error = 'waiting_valid_odom'
        self.odom_time = None
        self.source = None
        self.ready = False
        self.generation = 0
        self.commands = {}
        self.command_times = {}
        self.distance = None
        self.reason = 'no_path'

    def clear_commands(self):
        self.commands.clear()
        self.command_times.clear()

    def reset(self):
        self.source = None
        self.ready = False
        self.distance = None
        self.clear_commands()
        self.generation += 1

    def receive_path(self, path):
        if not valid_path(path):
            if self.goal is not None:
                self.reset()
            self.goal = None
            self.clear_commands()
            self.reason = 'no_path' if not path.poses else 'invalid_path_or_frame'
            return
        goal = goal_identity(path, self.goal_yaw_mode)
        if not same_goal(self.goal, goal):
            self.reset()
            self.goal = goal
            self.reason = 'new_goal'

    def receive_odom(self, odom, now):
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        valid = (odom.header.frame_id == 'map' and
                 all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and
                 math.hypot(q.x, q.y, q.z, q.w) > 1e-6)
        if not valid:
            self.position = None
            self.odom_time = None
            self.clear_commands()
            self.reason = 'invalid_odom_or_frame'
            self.odom_error = self.reason
            return
        self.position = (p.x, p.y)
        self.odom_time = now
        self.odom_error = None

    def receive_command(self, source, command, now):
        # Accept only messages received after activation acknowledgement.
        if source != self.source or not self.ready:
            return
        if not finite_twist(command):
            self.commands.pop(source, None)
            self.command_times.pop(source, None)
            return
        self.commands[source] = command
        self.command_times[source] = now

    def update_selection(self, now):
        if self.goal is None:
            return False
        if self.position is None or self.odom_time is None:
            self.reason = self.odom_error or 'waiting_valid_odom'
            self.clear_commands()
            return False
        if now - self.odom_time > self.odom_timeout:
            self.reason = 'odom_timeout'
            self.clear_commands()
            return False
        self.distance = math.hypot(self.position[0] - self.goal[1],
                                   self.position[1] - self.goal[2])
        if self.source is None or (self.source == 'local' and
                                   self.distance <= self.switch_distance):
            self.source = 'global' if self.distance <= self.switch_distance else 'local'
            self.ready = False
            self.clear_commands()
            self.generation += 1
            self.reason = 'terminal_handoff' if self.source == 'global' else 'local_tracking'
        return True

    def output(self, now):
        if not self.update_selection(now):
            return Twist()
        if not self.ready:
            self.reason = 'waiting_activation'
            return Twist()
        command_time = self.command_times.get(self.source)
        if command_time is None or now - command_time > self.cmd_timeout:
            self.reason = 'command_timeout'
            return Twist()
        self.reason = 'terminal_tracking' if self.source == 'global' else 'local_tracking'
        return self.commands[self.source]


class VelocitySelector(Node):
    def __init__(self):
        super().__init__('velocity_selector')
        self.state = SelectionState(
            self.declare_parameter('switch_distance', 2.0).value,
            self.declare_parameter('cmd_timeout', 0.3).value,
            self.declare_parameter('odom_timeout', 0.3).value,
            self.declare_parameter('goal_yaw_mode', 'path').value)
        path_topic = self.declare_parameter('path_topic', '/global_path').value
        odom_topic = self.declare_parameter('odom_topic', '/loc/robo/odom').value
        local_topic = self.declare_parameter('local_cmd_topic', '/nav_cmd/local').value
        global_topic = self.declare_parameter('global_cmd_topic', '/nav_cmd/global').value
        output_topic = self.declare_parameter('cmd_vel_topic', '/nav_cmd').value
        service = self.declare_parameter('enable_service', '/path_follower/set_enabled').value
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(Path, path_topic, self.path_callback, path_qos)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.local_sub = self.create_subscription(
            Twist, local_topic, lambda msg: self.command_callback('local', msg), 1)
        self.global_sub = self.create_subscription(
            Twist, global_topic, lambda msg: self.command_callback('global', msg), 1)
        self.command_pub = self.create_publisher(Twist, output_topic, 1)
        self.enable_client = self.create_client(SetBool, service)
        self.pending = None
        self.pending_generation = None
        self.pending_time = None
        self.ack_generation = None
        self.last_diag = None
        self.last_diag_time = 0.0
        self.timer = self.create_timer(0.05, self.control_callback)
        self.get_logger().info(
            f'[SELECTOR] switch_distance={self.state.switch_distance} '
            f'cmd_timeout={self.state.cmd_timeout} odom_timeout={self.state.odom_timeout}; '
            f'{local_topic} / {global_topic} -> {output_topic}; terminal obstacle avoidance disabled')

    def path_callback(self, message):
        generation = self.state.generation
        self.state.receive_path(message)
        if generation != self.state.generation or self.state.goal is None:
            self.command_pub.publish(Twist())

    def odom_callback(self, message):
        self.state.receive_odom(message, time.monotonic())

    def command_callback(self, source, message):
        self.state.receive_command(source, message, time.monotonic())

    def update_activation(self, now):
        if self.pending is not None:
            if self.pending.done():
                pending, generation = self.pending, self.pending_generation
                self.pending = None
                try:
                    response = pending.result()
                    if generation == self.state.generation and response.success:
                        # Prevent enabling a follower that has not received the new path yet.
                        if self.state.source != 'global' or same_goal(
                                self.state.goal, json.loads(response.message)):
                            self.state.clear_commands()
                            self.state.ready = self.state.source is not None
                            self.ack_generation = generation
                except Exception as error:
                    self.get_logger().warning(f'[SELECTOR] activation failed: {error}')
            elif now - self.pending_time > 0.3:
                self.pending.cancel()
                self.pending = None
            else:
                return
        if self.ack_generation == self.state.generation:
            return
        if not self.enable_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = self.state.source == 'global' and self.state.goal is not None
        self.pending_generation = self.state.generation
        self.pending_time = now
        self.pending = self.enable_client.call_async(request)

    def control_callback(self):
        now = time.monotonic()
        previous_source = self.state.source
        self.state.update_selection(now)
        self.update_activation(now)
        command = self.state.output(now)
        self.command_pub.publish(command)
        diagnostic = (self.state.source, self.state.reason)
        if diagnostic != self.last_diag or now - self.last_diag_time >= 0.5:
            command_time = self.state.command_times.get(self.state.source)
            age = 'NA' if command_time is None else f'{now - command_time:.3f}'
            distance = 'NA' if self.state.distance is None else f'{self.state.distance:.3f}'
            self.get_logger().info(
                f'[SELECTOR] source={self.state.source} previous={previous_source} '
                f'reason={self.state.reason} goal_dist_m={distance} cmd_age_s={age} '
                f'cmd=({command.linear.x:.3f},{command.linear.y:.3f},{command.angular.z:.3f})')
            self.last_diag = diagnostic
            self.last_diag_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocitySelector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.command_pub.publish(Twist())
            except RuntimeError:
                # When the process is asked to stop (SIGINT/SIGTERM) the rclpy
                # context is already invalid, so the command cannot be sent.
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
