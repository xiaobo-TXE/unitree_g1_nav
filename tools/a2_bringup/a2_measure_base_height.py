#!/usr/bin/env python3
"""Measure the A2 base height above the ground from localization and the cloud.

Stand the robot still on flat ground, make sure localization is up, then run:

    python3 tools/a2_bringup/a2_measure_base_height.py

``/loc/robo/odom`` is a pose in the odometry frame while the cloud arrives in
its own frame, and the two differ by a rotation and a translation. A z taken
from one frame and a floor measured in the other are therefore not a height
difference at all, so the script transforms the robot pose into the cloud frame,
measures the floor around it there, and reports ``h_b`` as the difference
between the two inside that one frame. The raw odometry z is printed next to it
so a tape measurement can confirm the result.

The measured value is the base height ``h_b`` that feeds d_up, d_down,
path_z_offset and body_height.

Exit status is 0 when the measurement is usable and 1 otherwise.

Usage:
    python3 tools/a2_bringup/a2_measure_base_height.py --duration 15
    python3 tools/a2_bringup/a2_measure_base_height.py --tape-height 0.35
    python3 tools/a2_bringup/a2_measure_base_height.py --write   # store h_b
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_point

HERE = Path(__file__).resolve().parent
DEFAULT_ROBOT = HERE / 'a2_robot.json'

DEFAULT_ODOM_TOPIC = '/loc/robo/odom'
DEFAULT_CLOUD_TOPIC = '/lio/cloud_world'
# A standing robot still breathes; anything above these means it moved.
MAX_Z_STD = 0.010
MAX_XY_DRIFT = 0.050
EXPECTED_FRAME = 'base'
# The floor is the dominant low surface in the ring around the robot: the body
# hides the ground directly underneath it, so sample a ring instead of a disc.
DEFAULT_FLOOR_RADIUS = 0.7
DEFAULT_FLOOR_PERCENTILE = 5.0
MIN_FLOOR_POINTS = 200
# Enough floor returns to be robust, bounded so a long run cannot grow forever.
MAX_FLOOR_POINTS = 200000
# A tape measurement is the ground truth for a physical distance.
TAPE_TOL = 0.050
# h_b outside this band means the map datum is probably not the floor.
PLAUSIBLE_MIN, PLAUSIBLE_MAX = 0.15, 0.80
# One wording for every path that must not produce a value or touch the file.
UNUSABLE = ('measurement is not usable: the derived parameters and --write are '
            'skipped. Fix the notes above and repeat the run.')


def number(value):
    return f'{value:.3f}'


class BaseHeightMeasurer(Node):
    """Collect odometry and the surrounding floor until the caller stops."""

    def __init__(self, odom_topic, cloud_topic, floor_radius):
        super().__init__('a2_measure_base_height')
        self.samples = []
        self.frames = set()
        self.odom_frame = None
        self.base_zs = []
        self.floor_chunks = []
        self.floor_count = 0
        self.cloud_frames = set()
        self.cloud_count = 0
        self.tf_error = None
        self._floor_radius = floor_radius
        self._latest_position = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f'Waiting for {odom_topic} and {cloud_topic}')

    def _on_odom(self, message):
        self.frames.add(message.child_frame_id)
        self.odom_frame = message.header.frame_id
        position = message.pose.pose.position
        self._latest_position = (position.x, position.y, position.z)
        self.samples.append(
            (time.monotonic(), position.x, position.y, position.z))

    def _robot_pose_in(self, frame):
        """The latest robot pose in ``frame``, or None when it cannot be done.

        The whole pose is transformed, not just xy: the odometry frame and the
        cloud frame differ by a rotation and a translation, so a z taken from
        one and subtracted from the other is not a height difference at all.
        """
        if not frame or frame == self.odom_frame:
            return self._latest_position
        point = PointStamped()
        point.header.frame_id = self.odom_frame
        point.point.x, point.point.y, point.point.z = self._latest_position
        try:
            transform = self._tf_buffer.lookup_transform(
                frame, self.odom_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception as error:  # tf2 raises several wrapper types
            self.tf_error = str(error)
            return None
        transformed = do_transform_point(point, transform)
        return (transformed.point.x, transformed.point.y, transformed.point.z)

    def _on_cloud(self, message):
        if self._latest_position is None or self.floor_count >= MAX_FLOOR_POINTS:
            return
        self.cloud_count += 1
        self.cloud_frames.add(message.header.frame_id)
        pose = self._robot_pose_in(message.header.frame_id)
        if pose is None:
            return
        self.base_zs.append((time.monotonic(), pose[2]))

        points = point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.size == 0:
            return
        dx = points['x'] - pose[0]
        dy = points['y'] - pose[1]
        ring = (dx * dx + dy * dy <= self._floor_radius ** 2) & np.isfinite(points['z'])
        selected = points['z'][ring].astype(float)
        self.floor_chunks.append(selected)
        self.floor_count += int(selected.size)

    def floor_zs(self):
        if not self.floor_chunks:
            return np.empty(0)
        return np.concatenate(self.floor_chunks)


def collect(node, settle, duration):
    """Spin for settle + duration seconds and return the steady samples."""
    start = time.monotonic()
    end = start + settle + duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    window_start = start + settle
    return [sample for sample in node.samples if sample[0] >= window_start]


def base_in_cloud_frame(node, samples):
    """The robot z in the cloud frame, and how many samples it used.

    /loc/robo/odom is a pose in the odometry frame while the cloud arrives in
    its own frame; the floor ring is measured in the cloud frame, so the
    subtraction has to happen there too. The window matches the odometry
    samples, whose first timestamp is the start of the measurement window.
    """
    window_start = samples[0][0]
    values = [z for stamp, z in node.base_zs if stamp >= window_start]
    if not values:
        return None, 0
    return statistics.fmean(values), len(values)


def report(node, samples, args, robot):
    """Print the measurement and the parameters derived from it."""
    if len(samples) < args.min_samples:
        print(f'FAIL: only {len(samples)} samples in the window '
              f'(need {args.min_samples}); is localization publishing?')
        return 1, None

    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    base = statistics.fmean(zs)
    z_std = statistics.pstdev(zs)
    drift = max(
        math.hypot(x - xs[0], y - ys[0]) for x, y in zip(xs, ys))

    status = 0
    usable = True
    notes = []
    if node.frames and node.frames != {EXPECTED_FRAME}:
        notes.append(
            f'WARN: child_frame_id is {sorted(node.frames)}, not '
            f'"{EXPECTED_FRAME}" - z may be the sensor height, not the base')
        status = 1
        usable = False
    if z_std > MAX_Z_STD:
        notes.append(f'WARN: z std {number(z_std)} m is too large; '
                     'the robot was not standing still')
        status = 1
        usable = False
    if drift > MAX_XY_DRIFT:
        notes.append(f'WARN: the robot drifted {number(drift)} m; '
                     'repeat the measurement on flat ground')
        status = 1
        usable = False

    print()
    print('== A2 base height ==')
    print(f'samples   : {len(samples)} over {number(args.duration)} s')
    print(f'frames    : {sorted(node.frames) or ["unknown"]}'
          f' in "{node.odom_frame}", cloud '
          f'{sorted(node.cloud_frames) or ["none"]}')
    print(f'odom z    : {number(base)} m   (std {number(z_std)}, '
          f'z {number(min(zs))} .. {number(max(zs))})')
    print(f'xy drift  : {number(drift)} m')

    cloud_frame = sorted(node.cloud_frames)[0] if node.cloud_frames else None
    base_cloud, base_cloud_count = base_in_cloud_frame(node, samples)
    floor_zs = node.floor_zs()
    z_floor = None
    if base_cloud is None or floor_zs.size < MIN_FLOOR_POINTS:
        notes.append(
            f'WARN: no h_b: {base_cloud_count} robot z samples and '
            f'{floor_zs.size} floor points from {node.cloud_count} clouds '
            f'(need {MIN_FLOOR_POINTS} points); is the cloud publishing and is '
            'the TF up?')
        status = 1
        if node.tf_error:
            notes.append(f'  tf: {node.tf_error}')
    else:
        z_floor = float(np.percentile(floor_zs, args.floor_percentile))
        if abs(base_cloud - base) > 0.005:
            print(f'base z    : {number(base_cloud)} m in "{cloud_frame}"'
                  '   (h_b uses this one, not the odom z above)')
        print(f'floor z   : {number(z_floor)} m   '
              f'(p{args.floor_percentile:g} of {floor_zs.size} points within '
              f'{number(args.floor_radius)} m, {node.cloud_count} clouds)')

    for note in notes:
        print(note)

    if z_floor is None:
        print()
        print(f'raw odom z is {number(base)} m; without the floor there is '
              'no h_b')
        print(UNUSABLE)
        return status, None

    base_height = base_cloud - z_floor
    print(f'base_h h_b: {number(base_height)} m'
          f'   (= base z in "{cloud_frame}" - floor z)')
    if abs(base_cloud - base_height) > 0.02:
        print(f'  note: base z and h_b differ by '
              f'{number(abs(base_cloud - base_height))} m, so the frame z datum'
              ' is not the floor; h_b is the difference, not the raw z')
    if not PLAUSIBLE_MIN <= base_height <= PLAUSIBLE_MAX:
        print(f'WARN: h_b is outside the plausible band '
              f'[{number(PLAUSIBLE_MIN)}, {number(PLAUSIBLE_MAX)}] m; check the '
              'map datum and the frame the cloud arrives in')
        status = 1
        usable = False

    if args.tape_height is not None:
        delta = abs(base_height - args.tape_height)
        verdict = 'PASS' if delta <= TAPE_TOL else 'FAIL'
        print(f'[{verdict}] tape height {number(args.tape_height)} m vs h_b '
              f'{number(base_height)} m (delta {number(delta)})')
        if verdict == 'FAIL':
            status = 1
            usable = False

    if not usable:
        print()
        print(UNUSABLE)
        return 1, base_height

    measured = robot['measured']
    step = robot['limits']['step_height_m']
    margin = robot['margins']['up_margin_m']
    height = measured['height_m']
    want_up = (height - base_height) + margin
    want_down = base_height - step

    print()
    print('== derived parameters ==')
    print(f'grid_map.obstacles_inflation_z_up   : {want_up:.2f}')
    print(f'grid_map.obstacles_inflation_z_down : {max(want_down, 0.0):.2f}')
    print(f'path_z_offset (recommended)         : {base_height:.2f}')
    print('grid_map.body_height                : 0.0')
    print(f'  (d_up = (H {number(height)} - h_b) + margin {number(margin)}; '
          f'd_down = h_b - step {number(step)} = {number(want_down)})')
    print('  (keep path_z_offset + body_height = h_b, never h_b for both)')

    if args.write:
        measured['base_height_m'] = round(base_height, 3)
        with open(args.robot_json, 'w') as handle:
            json.dump(robot, handle, indent=2, ensure_ascii=False)
            handle.write('\n')
        print()
        print(f'wrote base_height_m={round(base_height, 3)} to {args.robot_json}')
        print('now re-run a2_preflight.py to check the clearances')

    return status, base_height


def parse_args():
    parser = argparse.ArgumentParser(description='Measure the A2 base height.')
    parser.add_argument('--odom-topic', default=DEFAULT_ODOM_TOPIC)
    parser.add_argument('--cloud-topic', default=DEFAULT_CLOUD_TOPIC)
    parser.add_argument('--floor-radius', type=float, default=DEFAULT_FLOOR_RADIUS,
                        help='ring radius for the floor samples (default: %(default)s)')
    parser.add_argument('--floor-percentile', type=float, default=DEFAULT_FLOOR_PERCENTILE,
                        help='z percentile taken as the floor (default: %(default)s)')
    parser.add_argument('--tape-height', type=float, default=None,
                        help='tape-measured base height, for a cross-check')
    parser.add_argument('--duration', type=float, default=15.0,
                        help='averaging window in seconds (default: %(default)s)')
    parser.add_argument('--settle', type=float, default=2.0,
                        help='seconds to discard before averaging')
    parser.add_argument('--min-samples', type=int, default=30)
    parser.add_argument('--robot-json', type=Path, default=DEFAULT_ROBOT)
    parser.add_argument('--write', action='store_true',
                        help='store the measured value in a2_robot.json')
    return parser.parse_args()


def main():
    args = parse_args()
    with open(args.robot_json) as handle:
        robot = json.load(handle)

    rclpy.init()
    node = BaseHeightMeasurer(args.odom_topic, args.cloud_topic, args.floor_radius)
    try:
        samples = collect(node, args.settle, args.duration)
    except KeyboardInterrupt:
        print('interrupted')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    status, _ = report(node, samples, args, robot)
    node.destroy_node()
    rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
