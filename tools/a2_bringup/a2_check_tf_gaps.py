#!/usr/bin/env python3
"""Measure how continuously the transform scan_planner depends on arrives.

scan_planner's FSM stops the robot when the transform that places the reference
path (grid_map.frame_id <- the path's own frame) is not fresh: it looks the
transform up as usual, then requires

    node_->now() - transform.header.stamp  <=  fsm.reference_tf_max_age

and after fsm.reference_tf_grace seconds of that failing it calls an emergency
stop, recovering only once the robot has come to rest. A transform that arrives
in bursts, or that arrives steadily with an old header stamp, therefore shows up
on the robot as walk, stop, walk.

This watches /tf for a while and reports both, using the node clock exactly as
the FSM does: the gaps between arrivals, and the age of the stamp each arrival
carries. They are different faults - only the second one imitates a healthy
10 Hz stream.

    python3 tools/a2_bringup/a2_check_tf_gaps.py --duration 40

Exit status is 0 when neither measure exceeds the FSM's age limit.
"""

from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

# The limits scan_planner applies, so the report can be read against them.
MAX_AGE = 0.3
GRACE = 0.5
DEFAULT_TARGET = 'world'
DEFAULT_SOURCE = 'map'


def number(value):
    return f'{value:.3f}'


class TfGapWatcher(Node):
    """Record arrival times and header stamps for one transform pair."""

    def __init__(self, target, source, topics, settle):
        super().__init__('a2_check_tf_gaps')
        self._pairs = {(target, source), (source, target)}
        self._settle = settle
        self._deadline = time.monotonic() + settle
        self.arrivals = []
        self.stamps = []
        for topic in topics:
            self.create_subscription(
                TFMessage, topic, self._on_tf, qos_profile_sensor_data)

    def _on_tf(self, message):
        if time.monotonic() < self._deadline:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for transform in message.transforms:
            header = transform.header
            if (header.frame_id, transform.child_frame_id) not in self._pairs:
                continue
            self.arrivals.append(now)
            self.stamps.append(header.stamp.sec + header.stamp.nanosec * 1e-9)


def report(node, args):
    arrivals = node.arrivals
    if not arrivals:
        print(f'FAIL: no {args.target} <-> {args.source} transform on '
              f'{", ".join(args.topics)} in {number(args.duration)} s')
        print('      (is the stack running, and is the frame name right?)')
        return 1

    span = arrivals[-1] - arrivals[0]
    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    ages = [a - s for a, s in zip(arrivals, node.stamps) if s > 0.0]

    print()
    print(f'== reference transform "{args.source}" <-> "{args.target}" ==')
    print(f'samples   : {len(arrivals)} over {number(span)} s '
          f'({len(arrivals) / span if span > 0 else float("inf"):.1f} Hz)')
    if gaps:
        print(f'arrivals  : median gap {number(statistics.median(gaps))} s, '
              f'max {number(max(gaps))} s')
    if ages:
        print(f'stamp age : median {number(statistics.median(ages))} s, '
              f'max {number(max(ages))} s')

    status = 0
    long_gaps = [gap for gap in gaps if gap > MAX_AGE]
    if long_gaps:
        print(f'[FAIL] {len(long_gaps)} arrival gaps exceed {MAX_AGE} s '
              f'(max {number(max(long_gaps))} s): the publisher stutters, and '
              'a gap over the grace time stops the robot')
        for index, gap in enumerate(gaps):
            if gap > MAX_AGE:
                print(f'         t+{number(arrivals[index] - arrivals[0])} s: '
                      f'gap {number(gap)} s')
        status = 1
    else:
        print(f'[PASS] arrivals are continuous (no gap over {MAX_AGE} s)')

    stale = [age for age in ages if age > MAX_AGE]
    if stale:
        print(f'[FAIL] {len(stale)} of {len(ages)} samples carry a stamp older '
              f'than {MAX_AGE} s (max {number(max(stale))} s): the stream is '
              'steady but the stamp lags, and the FSM cannot tell that apart '
              'from a missing transform')
        status = 1
    elif ages:
        print(f'[PASS] stamps are within {MAX_AGE} s of arrival')

    print()
    print(f'(scan_planner stops the robot after {GRACE} s of the above failing)')
    return status


def parse_args():
    parser = argparse.ArgumentParser(
        description='Measure the continuity of the reference transform.')
    parser.add_argument('--target', default=DEFAULT_TARGET,
                        help='grid_map.frame_id (default: %(default)s)')
    parser.add_argument('--source', default=DEFAULT_SOURCE,
                        help="the reference path's frame (default: %(default)s)")
    parser.add_argument('--topics', nargs='+', default=['/tf'],
                        help='transform topics to watch (default: %(default)s)')
    parser.add_argument('--duration', type=float, default=40.0)
    parser.add_argument('--settle', type=float, default=1.0,
                        help='seconds to ignore while the buffer fills')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TfGapWatcher(args.target, args.source, args.topics, args.settle)
    deadline = time.monotonic() + args.duration + args.settle
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    status = report(node, args)
    node.destroy_node()
    rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
