#!/usr/bin/env python3
"""Check the height chain of the reference path against the robot base height.

Two independent offsets lift the reference path above the terrain:
``path_z_offset`` in global_planner (applied to /global_path) and
``grid_map.body_height`` in scan_planner (applied again in navi_mode=3, because
/global_path is remapped onto initial_path). This script reports what each topic
actually carries, so the sum can be compared with the measured base height.

Run it with the stack up and a global path published:

    python3 tools/a2_bringup/a2_check_path_height.py

Exit status is 0 when the chain is consistent and 1 otherwise.

Usage:
    python3 tools/a2_bringup/a2_check_path_height.py
    python3 tools/a2_bringup/a2_check_path_height.py --bspline
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
DEFAULT_ROBOT = HERE / 'a2_robot.json'
PLANNER_CONFIG = 'scan_planner/planner/plan_manage/config/planner.yaml'
GLOBAL_CONFIG = 'global_planner/config/global_planner.yaml'

TOL = 0.10
FAIL = 0.25


def load_parameters(path):
    with open(path) as handle:
        document = yaml.safe_load(handle)
    node_name = next(iter(document))
    return document[node_name]['ros__parameters']


def number(value):
    return f'{value:.3f}'


def optional_bspline_type():
    """Return the Bspline message class when the workspace is sourced."""
    try:
        from scan_planner_msgs.msg import Bspline  # noqa: WPS433
        return Bspline
    except Exception:
        return None


class PathHeightChecker(Node):
    """Track the newest global path, local trajectory and body odometry."""

    def __init__(self, path_topic, odom_topic, bspline_type, bspline_topic):
        super().__init__('a2_check_path_height')
        self.path = None
        self.odom_z = None
        self.odom_frame = None
        self.bspline_z = None
        self.create_subscription(PathMessage, path_topic, self._on_path, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        if bspline_type is not None:
            self.create_subscription(
                bspline_type, bspline_topic, self._on_bspline, 10)

    def _on_path(self, message):
        self.path = message

    def _on_odom(self, message):
        self.odom_z = message.pose.pose.position.z
        self.odom_frame = message.child_frame_id

    def _on_bspline(self, message):
        if message.pos_pts:
            self.bspline_z = statistics.fmean(
                point.z for point in message.pos_pts)

    def ready(self):
        return self.path is not None and self.odom_z is not None


def report(node, planner, global_params, robot):
    """Print one height-chain report and return True when nothing failed."""
    zs = [pose.pose.position.z for pose in node.path.poses]
    if not zs:
        print('FAIL: the global path has no poses')
        return False
    offset = global_params['path_z_offset']
    body = planner['grid_map.body_height']
    total = offset + body
    base = robot['measured'].get('base_height_m')

    print()
    print(f'== path height chain (odom frame "{node.odom_frame}") ==')
    print(f'/global_path : {len(zs)} poses, z {number(min(zs))} .. '
          f'{number(max(zs))} (first {number(zs[0])}, last {number(zs[-1])})')
    print(f'/loc/robo/odom z                      : {number(node.odom_z)}')
    print(f'path_z_offset (global)                : {number(offset)}')
    print(f'grid_map.body_height (scan)           : {number(body)}')
    print(f'sum                                   : {number(total)}')
    print(f'inferred terrain z under first pose   : '
          f'{number(zs[0] - offset)}')
    if node.bspline_z is not None:
        print(f'/planning/bspline mean z              : '
              f'{number(node.bspline_z)}')

    ok = True
    delta = zs[0] - node.odom_z
    status = 'PASS' if abs(delta) <= TOL else (
        'FAIL' if abs(delta) > FAIL else 'WARN')
    print(f'[{status}] first pose z minus odom z: {number(delta)} m')
    ok &= status != 'FAIL'

    if node.bspline_z is not None:
        delta_bs = node.bspline_z - node.odom_z
        status_bs = 'PASS' if abs(delta_bs) <= TOL else (
            'FAIL' if abs(delta_bs) > FAIL else 'WARN')
        print(f'[{status_bs}] bspline mean z minus odom z: '
              f'{number(delta_bs)} m (a large value suggests a double lift)')
        ok &= status_bs != 'FAIL'

    if base is not None:
        status_sum = 'PASS' if abs(total - base) <= TOL else (
            'FAIL' if abs(total - base) > FAIL else 'WARN')
        print(f'[{status_sum}] sum versus base_height {number(base)}: '
              f'{number(abs(total - base))} m')
        ok &= status_sum != 'FAIL'
    else:
        print('[INFO] base_height_m is unset; compare the sum by hand once '
              'it is measured')

    if offset > 0.0 and body > 0.0:
        print('[WARN] both offsets are non-zero, so the path is lifted twice; '
              'keep their sum equal to the base height, not each of them')
    return ok


def parse_args():
    parser = argparse.ArgumentParser(
        description='Check the reference-path height chain.')
    parser.add_argument('--path-topic', default='/global_path')
    parser.add_argument('--odom-topic', default='/loc/robo/odom')
    parser.add_argument('--bspline-topic', default='/planning/bspline')
    parser.add_argument('--bspline', action='store_true',
                        help='force the bspline subscription (needs the workspace sourced)')
    parser.add_argument('--robot-json', type=Path, default=DEFAULT_ROBOT)
    parser.add_argument('--planner-yaml', type=Path,
                        default=REPO_ROOT / PLANNER_CONFIG,
                        help='scan_planner parameters (default: %(default)s)')
    parser.add_argument('--global-yaml', type=Path,
                        default=REPO_ROOT / GLOBAL_CONFIG,
                        help='global_planner parameters (default: %(default)s)')
    parser.add_argument('--timeout', type=float, default=30.0)
    return parser.parse_args()


def main():
    args = parse_args()
    with open(args.robot_json) as handle:
        robot = json.load(handle)
    planner = load_parameters(args.planner_yaml)
    global_params = load_parameters(args.global_yaml)

    bspline_type = optional_bspline_type()
    if bspline_type is None and args.bspline:
        print(f'note: scan_planner_msgs is not importable, skipping '
              f'{args.bspline_topic}')

    rclpy.init()
    node = PathHeightChecker(
        args.path_topic, args.odom_topic, bspline_type, args.bspline_topic)
    deadline = time.monotonic() + args.timeout
    ok = True
    reported = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.ready() and not reported:
                ok = report(node, planner, global_params, robot)
                reported = True
                break
    except KeyboardInterrupt:
        pass

    if not reported:
        print(f'FAIL: no path on {args.path_topic} within '
              f'{args.timeout:.0f} s (is a goal published?)')
        ok = False
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
