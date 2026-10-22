#!/usr/bin/env python3
"""Exercise the g1_bridge watchdog and command duration on the real robot.

The bridge forwards each /cmd_vel message as one SetVelocity request with
``command_duration`` seconds of validity, and sends a zero request when no
message has arrived for ``cmd_vel_timeout`` seconds. This script drives a
timed velocity profile so both behaviours can be observed without the planner
running: start only g1_bridge, then hold the robot's remote and press START.

The script prints a wall-clock timeline of what to watch for. It publishes to
the topic the bridge subscribes to, which in this workspace is /nav_cmd (the
planner and the bridge default to different names; see README.md).

Exit status is 0 when the profile ran and 1 on a bad configuration.

Usage:
    # terminal 1
    ros2 run g1_bridge cmd_vel_bridge --ros-args -p cmd_vel_topic:=/nav_cmd
    # terminal 2
    python3 tools/a2_bringup/a2_bridge_watchdog_test.py --vx 0.2 --hold 3
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# The bridge stops the robot 0.5 s after the stream stops, by default.
DEFAULT_TIMEOUT = 0.5
# Below this the bridge's own timeout fires between messages.
MIN_RATE_HZ = 2.0


def stamp(start):
    return f'{time.monotonic() - start:6.2f}s'


class WatchdogTester(Node):
    """Publish Twist messages on a fixed schedule."""

    def __init__(self, topic):
        super().__init__('a2_bridge_watchdog_test')
        self.publisher = self.create_publisher(Twist, topic, 10)

    def publish_velocity(self, vx, vy, vyaw):
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = vyaw
        self.publisher.publish(message)


def hold(node, rate, seconds, vx, vy, vyaw):
    """Publish the same velocity at a fixed rate for a while."""
    period = 1.0 / rate
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        node.publish_velocity(vx, vy, vyaw)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def run_profile(node, args, start):
    """Drive the timeline and narrate what should happen on the robot."""
    print(f'{stamp(start)} press START on the remote to enable motion, '
          f'then wait for the run to begin')

    hold(node, args.rate, args.enable_wait, 0.0, 0.0, 0.0)
    print(f'{stamp(start)} baseline: zeros only; the robot must not move')

    print(f'{stamp(start)} streaming vx={args.vx} vy={args.vy} '
          f'vyaw={args.vyaw} at {args.rate:.0f} Hz for {args.hold:.1f} s')
    hold(node, args.rate, args.hold, args.vx, args.vy, args.vyaw)
    print(f'{stamp(start)} stop publishing: the robot keeps the last command '
          f'for at most command_duration (1.0 s), then the bridge must log '
          f'"timed out; stopping the robot" within cmd_vel_timeout '
          f'({args.timeout:.1f} s) and the robot must stop')

    time.sleep(args.timeout + 1.0)
    print(f'{stamp(start)} watchdog window over')

    print(f'{stamp(start)} sending one explicit zero command as a safety stop')
    hold(node, args.rate, 1.0, 0.0, 0.0, 0.0)
    print(f'{stamp(start)} done')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Exercise the bridge watchdog on the real robot.')
    parser.add_argument('--topic', default='/nav_cmd',
                        help='topic the bridge subscribes to')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='publish rate in Hz (must exceed 2)')
    parser.add_argument('--hold', type=float, default=3.0,
                        help='seconds to stream the test velocity')
    parser.add_argument('--enable-wait', type=float, default=2.0,
                        help='seconds of baseline zeros before the test')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT,
                        help='configured cmd_vel_timeout, for the narration')
    parser.add_argument('--vx', type=float, default=0.2)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--vyaw', type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rate < MIN_RATE_HZ:
        print(f'FAIL: --rate {args.rate:.1f} Hz is below the {MIN_RATE_HZ:.0f} Hz '
              f'minimum; the bridge would time out between messages')
        return 1

    rclpy.init()
    node = WatchdogTester(args.topic)
    start = time.monotonic()
    try:
        run_profile(node, args, start)
    except KeyboardInterrupt:
        print('interrupted; sending a stop')
        node.publish_velocity(0.0, 0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
