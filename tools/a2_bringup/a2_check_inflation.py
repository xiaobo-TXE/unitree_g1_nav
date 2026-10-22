#!/usr/bin/env python3
"""Decode the live twin-cylinder footprint and compare it with the config.

``scan_planner`` publishes the two collision cylinders on /self_inflation as
CYLINDER markers (radius = double_cylinder_radius, height = z_up + z_down).
This script reads them back together with the odometry, reconstructs
d_xy / d_off / d_up / d_down, and checks them against planner.yaml and the
measured robot dimensions.

Run it with the full stack up (scan_planner_node publishing, robot standing):

    python3 tools/a2_bringup/a2_check_inflation.py

Manual companion test: put the robot next to a table (~0.7 m), a 25-30 cm step
and a 35-40 cm obstacle, and confirm the planner passes the first two and
avoids the third. See README.md.

Exit status is 0 when every check passes and 1 otherwise.

Usage:
    python3 tools/a2_bringup/a2_check_inflation.py
    python3 tools/a2_bringup/a2_check_inflation.py --repeat
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
DEFAULT_ROBOT = HERE / 'a2_robot.json'
PLANNER_CONFIG = (
    'scan_planner/planner/plan_manage/config/planner.yaml'
)

TOL = 0.02
FAIL = 0.05


def load_parameters(path):
    with open(path) as handle:
        document = yaml.safe_load(handle)
    node_name = next(iter(document))
    return document[node_name]['ros__parameters']


def number(value):
    return f'{value:.3f}'


class InflationChecker(Node):
    """Reconstruct the footprint from /self_inflation plus the body odometry."""

    def __init__(self, marker_topic, odom_topic, repeat):
        super().__init__('a2_check_inflation')
        self.repeat = repeat
        self.markers = {}
        self.odom_z = None
        self.odom_frame = None
        self.stamp = None
        self.done = False
        self.create_subscription(Marker, marker_topic, self._on_marker, 20)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.get_logger().info(
            f'Watching {marker_topic} and {odom_topic}')

    def _on_marker(self, message):
        if message.type != Marker.CYLINDER:
            return
        self.markers[message.id] = message
        self.stamp = self.get_clock().now()

    def _on_odom(self, message):
        self.odom_z = message.pose.pose.position.z
        self.odom_frame = message.child_frame_id

    def ready(self):
        return 0 in self.markers and 1 in self.markers and self.odom_z is not None


def check(configured, measured, name, unit='m'):
    """Grade one reconstructed value against its configured counterpart."""
    delta = abs(configured - measured)
    status = 'PASS' if delta <= TOL else ('FAIL' if delta > FAIL else 'WARN')
    print(f'[{status}] {name}: config {number(configured)} '
          f'vs marker {number(measured)} {unit} (delta {number(delta)})')
    return status != 'FAIL'


def report(node, planner, robot):
    """Print one reconstruction round and return True when nothing failed."""
    front = node.markers[0].pose.position
    rear = node.markers[1].pose.position
    radius = 0.5 * node.markers[0].scale.x
    height = node.markers[0].scale.z
    offset = 0.5 * ((front.x - rear.x) ** 2 + (front.y - rear.y) ** 2) ** 0.5
    center_z = 0.5 * (front.z + rear.z)
    top = center_z + 0.5 * height
    bottom = center_z - 0.5 * height
    d_up = top - node.odom_z
    d_down = node.odom_z - bottom
    base = robot['measured'].get('base_height_m')
    step = robot['limits']['step_height_m']
    margin = robot['margins']['up_margin_m']

    print()
    print(f'== footprint reconstruction (odom frame "{node.odom_frame}", '
          f'z={number(node.odom_z)}) ==')
    print(f'cylinder : r={number(radius)} off={number(offset)} '
          f'height={number(height)}')
    print(f'vertical : d_up={number(d_up)} d_down={number(d_down)} '
          f'(top={number(top)}, bottom={number(bottom)})')

    ok = True
    ok &= check(planner['grid_map.double_cylinder_radius'], radius, 'd_xy')
    ok &= check(planner['grid_map.double_cylinder_offset'], offset, 'd_off')
    ok &= check(planner['grid_map.obstacles_inflation_z_up'], d_up, 'd_up')
    ok &= check(planner['grid_map.obstacles_inflation_z_down'], d_down, 'd_down')

    if base is not None:
        want_up = (robot['measured']['height_m'] - base) + margin
        want_down = base - step
        print(f'[INFO] expected from base_height={number(base)}: '
              f'd_up={number(want_up)} d_down={number(max(want_down, 0.0))} '
              f'(step={number(step)})')
        ok &= check(want_up, d_up, 'd_up (vs measured base height)')
        ok &= check(max(want_down, 0.0), d_down,
                    'd_down (vs measured base height)')
    else:
        print('[INFO] base_height_m is unset; run a2_measure_base_height.py '
              'to also check d_up/d_down against the real body')
    return ok


def parse_args():
    parser = argparse.ArgumentParser(
        description='Check the live twin-cylinder footprint.')
    parser.add_argument('--marker-topic', default='/self_inflation')
    parser.add_argument('--odom-topic', default='/loc/robo/odom')
    parser.add_argument('--robot-json', type=Path, default=DEFAULT_ROBOT)
    parser.add_argument('--repeat', action='store_true',
                        help='keep printing instead of reporting once')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='give up after this many seconds (default: %(default)s)')
    return parser.parse_args()


def main():
    args = parse_args()
    with open(args.robot_json) as handle:
        robot = json.load(handle)
    planner = load_parameters(REPO_ROOT / PLANNER_CONFIG)

    rclpy.init()
    node = InflationChecker(args.marker_topic, args.odom_topic, args.repeat)
    deadline = time.monotonic() + args.timeout
    ok = True
    reported = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.ready():
                continue
            if reported and not node.repeat:
                break
            if reported:
                time.sleep(1.0)
            ok &= report(node, planner, robot)
            reported = True
    except KeyboardInterrupt:
        pass

    if not reported:
        print(f'FAIL: no footprint seen on {args.marker_topic} within '
              f'{args.timeout:.0f} s (is scan_planner_node running?)')
        ok = False
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
