"""Forward ROS 2 Twist commands to the Unitree G1 locomotion service."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from unitree_api.msg import Request


SET_VELOCITY_API_ID = 7105


class CmdVelBridge(Node):
    """Translate ``/cmd_vel`` messages into G1 SetVelocity requests."""

    def __init__(self):
        super().__init__('g1_cmd_vel_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('request_topic', '/api/sport/request')
        self.declare_parameter('motion_enable_topic', '/motion_enabled')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('command_duration', 1.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        request_topic = self.get_parameter('request_topic').value
        motion_enable_topic = self.get_parameter('motion_enable_topic').value
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._command_duration = float(self.get_parameter('command_duration').value)
        if self._command_duration <= 0.0:
            raise ValueError('command_duration must be greater than zero')

        self._request_publisher = self.create_publisher(Request, request_topic, 10)
        self._cmd_vel_subscription = self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        motion_enable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._motion_enable_subscription = self.create_subscription(
            Bool,
            motion_enable_topic,
            self._on_motion_enable,
            motion_enable_qos,
        )

        # Require an explicit START press before forwarding velocity commands.
        self._motion_enabled = False
        self._last_command_time = None
        self._motion_command_active = False
        self._watchdog_timer = self.create_timer(0.05, self._watchdog)

        self.get_logger().info(
            'Forwarding %s to %s when enabled by %s '
            '(timeout %.3f s, duration %.3f s)'
            % (cmd_vel_topic, request_topic, motion_enable_topic,
               self._cmd_vel_timeout, self._command_duration))

    @staticmethod
    def _valid_command(vx, vy, vyaw):
        return all(math.isfinite(value) for value in (vx, vy, vyaw))

    def _velocity_request(self, vx, vy, vyaw):
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = SET_VELOCITY_API_ID
        request.parameter = json.dumps(
            {
                'velocity': [vx, vy, vyaw],
                'duration': self._command_duration,
            },
            separators=(',', ':'),
        )
        return request

    def _on_cmd_vel(self, message):
        vx = float(message.linear.x)
        vy = float(message.linear.y)
        vyaw = float(message.angular.z)

        if not self._valid_command(vx, vy, vyaw):
            self.get_logger().error('Rejected non-finite /cmd_vel command')
            self.stop()
            return

        if not self._motion_enabled:
            return

        self._request_publisher.publish(
            self._velocity_request(vx, vy, vyaw))

        self._last_command_time = time.monotonic()
        self._motion_command_active = any(abs(value) > 1.0e-6 for value in (vx, vy, vyaw))

    def _on_motion_enable(self, message):
        enabled = bool(message.data)
        state_changed = enabled != self._motion_enabled
        self._motion_enabled = enabled

        if enabled and state_changed:
            self.get_logger().info(
                'Motion enabled by controller START button')
        elif not enabled:
            if state_changed:
                self.get_logger().warning(
                    'Motion disabled by controller B button')
            # Always honor a stop request, including repeated B presses.
            self.stop()

    def _watchdog(self):
        if self._cmd_vel_timeout <= 0.0:
            return
        if not self._motion_command_active or self._last_command_time is None:
            return
        if time.monotonic() - self._last_command_time > self._cmd_vel_timeout:
            self.get_logger().warning('/cmd_vel timed out; stopping the robot')
            self.stop()

    def stop(self):
        """Send one zero-velocity request."""
        self._request_publisher.publish(self._velocity_request(0.0, 0.0, 0.0))
        self._motion_command_active = False
        self._last_command_time = None

    def destroy_node(self):
        if self._motion_command_active:
            self.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
