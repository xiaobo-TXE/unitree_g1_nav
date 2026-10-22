#!/usr/bin/env python3
"""Tests for a2_measure_base_height.py.

These run without a robot and without a running stack: ``report()`` is driven
with synthetic odometry and floor samples, and the cloud callback is fed a
hand-built PointCloud2. ROS 2 must be sourced (the tool imports rclpy), and the
robot JSON is always a temporary copy, never tools/a2_bringup/a2_robot.json.

    python3 -m pytest tools/a2_bringup/test_a2_measure_base_height.py
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'a2_measure_base_height', HERE / 'a2_measure_base_height.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def robot():
    """A fresh measurement file whose base height is still unknown."""
    return {
        'model': 'Unitree A2',
        'measured': {'length_m': 0.86, 'width_m': 0.46, 'height_m': 0.65,
                     'base_height_m': None},
        'limits': {'step_height_m': 0.30, 'max_slope_deg': 45.0},
        'margins': {'up_margin_m': 0.05},
    }


def args(monkeypatch, **overrides):
    """Real parsed arguments, so a new option cannot silently go missing.

    None means "keep the default"; True means "pass this flag".
    """
    argv = ['measure']
    for key, value in overrides.items():
        flag = '--' + key.replace('_', '-')
        if value is None:
            continue
        if isinstance(value, bool):
            if value:
                argv.append(flag)
        else:
            argv += [flag, str(value)]
    monkeypatch.setattr(sys, 'argv', argv)
    return mod.parse_args()


def node(frames=('base',), odom_frame='map', cloud_frames=('world',),
         floor_zs=(), cloud_count=10, tf_error=None, base_z=0.35):
    """A stub node.

    ``base_z`` is the robot z already expressed in the cloud frame, which is
    what the tool measures the floor against; it may differ from the odometry z
    when the two frames do.
    """
    stub = SimpleNamespace(
        frames=set(frames), odom_frame=odom_frame,
        cloud_frames=set(cloud_frames), cloud_count=cloud_count,
        tf_error=tf_error, base_zs=[(0.0, float(base_z))] * 200,
    )
    values = np.asarray(floor_zs, dtype=float)
    stub.floor_zs = lambda: values
    return stub


def odom_samples(z, count=200, jitter=0.002):
    noise = np.random.default_rng(0).normal(0.0, jitter, count)
    return [(i * 0.01, 0.0, 0.0, float(z) + float(d)) for i, d in enumerate(noise)]


def moving_samples(count=200):
    return [(i * 0.01, 0.5 * i, 0.2 * i, 0.35 + 0.05 * i) for i in range(count)]


def report(stub, samples, parsed, measurement):
    with contextlib.redirect_stdout(io.StringIO()) as buffer:
        status, base_height = mod.report(stub, samples, parsed, measurement)
    return status, base_height, buffer.getvalue()


def flat_floor(z=0.0, count=9000):
    return np.full(count, z)


def test_h_b_when_the_map_datum_is_the_floor(monkeypatch):
    status, base_height, out = report(
        node(floor_zs=flat_floor()), odom_samples(0.35), args(monkeypatch),
        robot())
    assert status == 0
    assert base_height == pytest.approx(0.35, abs=0.01)
    assert 'path_z_offset (recommended)         : 0.35' in out
    assert 'obstacles_inflation_z_up   : 0.35' in out
    assert 'obstacles_inflation_z_down : 0.05' in out


def test_h_b_uses_the_robot_z_in_the_cloud_frame(monkeypatch):
    """The odometry frame and the cloud frame need not share a z datum."""
    status, base_height, out = report(
        node(floor_zs=flat_floor(0.0), base_z=0.30), odom_samples(0.35),
        args(monkeypatch), robot())
    assert status == 0
    assert base_height == pytest.approx(0.30, abs=0.01)
    assert 'h_b uses this one' in out


def test_h_b_is_invariant_to_the_frame_datum(monkeypatch):
    """Both the robot z and the floor shifted by 1 m is the same height."""
    status, base_height, out = report(
        node(floor_zs=flat_floor(1.0), base_z=1.35), odom_samples(1.35),
        args(monkeypatch), robot())
    assert status == 0
    assert base_height == pytest.approx(0.35, abs=0.01)
    assert 'is not the floor' in out


def test_floor_percentile_ignores_overhead_returns(monkeypatch):
    """A tabletop or ceiling in the ring must not move the floor."""
    floor_zs = np.concatenate([flat_floor(0.0, 9000), np.full(400, 1.2)])
    _, base_height, _ = report(
        node(floor_zs=floor_zs), odom_samples(0.35),
        args(monkeypatch, floor_percentile=5.0), robot())
    assert base_height == pytest.approx(0.35, abs=0.01)


def test_write_stores_h_b(monkeypatch, tmp_path):
    path = tmp_path / 'a2_robot.json'
    path.write_text(json.dumps(robot()))
    measurement = robot()
    status, _, _ = report(
        node(floor_zs=flat_floor()), odom_samples(0.35),
        args(monkeypatch, write=True, robot_json=path), measurement)
    assert status == 0
    assert json.loads(path.read_text())['measured']['base_height_m'] == 0.35
    assert robot()['measured']['base_height_m'] is None


@pytest.mark.parametrize('label, stub_kwargs, sample_z, tape', [
    ('wrong frame', {'frames': ('base', 'imu')}, 0.35, None),
    ('robot moved', {}, 'moving', None),
    ('no floor points', {'floor_zs': np.zeros(10)}, 0.35, None),
    ('tape mismatch', {}, 0.35, 0.60),
    ('implausible h_b', {'base_z': 15.0}, 15.0, None),
])
def test_unusable_measurements_never_touch_the_file(
        monkeypatch, tmp_path, label, stub_kwargs, sample_z, tape):
    path = tmp_path / 'a2_robot.json'
    path.write_text(json.dumps(robot()))
    parsed = args(monkeypatch, write=True, robot_json=path, tape_height=tape)
    odom = moving_samples() if sample_z == 'moving' else odom_samples(sample_z)
    status, _, out = report(
        node(**{'floor_zs': flat_floor(), **stub_kwargs}), odom, parsed, robot())

    assert status == 1, label
    assert 'measurement is not usable' in out, label
    assert json.loads(path.read_text())['measured']['base_height_m'] is None, label


def test_tape_agreement_is_reported(monkeypatch):
    status, _, out = report(
        node(floor_zs=flat_floor()), odom_samples(0.35),
        args(monkeypatch, tape_height=0.35), robot())
    assert status == 0
    assert '[PASS] tape height 0.350 m' in out


def test_collect_discards_settle_samples(ros, monkeypatch):
    monkeypatch.setattr(mod.rclpy, 'spin_once', lambda *a, **k: None)
    now = time.monotonic()
    stub = SimpleNamespace(samples=[
        (now - 1000.0, 0.0, 0.0, 1.0),
        (now + 1000.0, 0.0, 0.0, 2.0),
    ])
    assert [sample[3] for sample in mod.collect(stub, settle=0.05, duration=0.0)] == [2.0]


def ring_cloud():
    """A world-frame cloud: floor ring, a ceiling, the body, and NaNs."""
    angles = np.linspace(0.0, 2.0 * np.pi, 2000)
    points = [(0.5 + 0.6 * np.cos(t), 0.6 * np.sin(t), 0.0) for t in angles]
    points += [(x, y, 2.5)
               for x in np.linspace(-3.0, 3.0, 50)
               for y in np.linspace(-3.0, 3.0, 50)]
    points += [(0.5, 0.0, 0.35)] * 5
    points += [(float('nan'), 0.0, 0.35)] * 3
    cloud = point_cloud2.create_cloud_xyz32(
        PointCloud2().header,
        [(float(x), float(y), float(z)) for x, y, z in points])
    cloud.header.frame_id = 'world'
    return cloud


def test_cloud_ring_keeps_the_floor(ros):
    node_ = mod.BaseHeightMeasurer('/loc/robo/odom', '/lio/cloud_world', 0.7)
    try:
        node_.odom_frame = 'world'
        node_._latest_position = (0.5, 0.0, 0.0)
        node_._on_cloud(ring_cloud())
        zs = node_.floor_zs()
        recorded = list(node_.base_zs)
    finally:
        node_.destroy_node()
    assert zs.size > 1500
    assert float(np.percentile(zs, 5.0)) == pytest.approx(0.0, abs=1e-6)
    assert (zs < 0.05).mean() > 0.9
    assert [z for _, z in recorded] == [0.0]


def test_cloud_is_ignored_before_odometry(ros):
    node_ = mod.BaseHeightMeasurer('/loc/robo/odom', '/lio/cloud_world', 0.7)
    try:
        node_._on_cloud(ring_cloud())
        assert node_.cloud_count == 0
        assert node_.floor_zs().size == 0
    finally:
        node_.destroy_node()


def test_floor_collection_stops_at_the_cap(ros):
    node_ = mod.BaseHeightMeasurer('/loc/robo/odom', '/lio/cloud_world', 0.7)
    try:
        node_.odom_frame = 'world'
        node_._latest_position = (0.5, 0.0, 0.0)
        node_._on_cloud(ring_cloud())
        collected = node_.floor_count
        node_.floor_count = mod.MAX_FLOOR_POINTS
        node_._on_cloud(ring_cloud())
        assert node_.floor_count == mod.MAX_FLOOR_POINTS
        assert collected < mod.MAX_FLOOR_POINTS
    finally:
        node_.destroy_node()
