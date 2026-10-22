#!/usr/bin/env python3
"""Tests for a2_preflight.py.

The grader must not let a milder result hide a worse one: the status names are
plain strings, and 'FAIL' sorts *below* 'PASS' and 'WARN', so grading two
deviations with max() silently downgrades a failure. These tests pin the
severity order and the two checks that grade more than one deviation.

    python3 -m pytest tools/a2_bringup/test_a2_preflight.py

The grader is normally exercised against the shipped configuration files. The
real robot keeps scan_planner and global_planner in two separate workspaces, so
a test that cannot find a file skips instead of failing; point A2_PLANNER_YAML,
A2_GLOBAL_YAML or A2_REFERENCE_YAML at it to run those too.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'a2_preflight', HERE / 'a2_preflight.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def shipped_path(name):
    """A shipped configuration path, or skip when it is not in this checkout.

    Look where the tool looks by default, then one level down for the ``src/``
    layout the robot uses. A2_<NAME>_YAML takes precedence, which is how the
    robot can supply global_planner from its other workspace.
    """
    override = os.environ.get(f'A2_{name.upper()}_YAML')
    if override:
        return Path(override)
    relative = mod.CONFIG_FILES[name]
    for root in (mod.REPO_ROOT, mod.REPO_ROOT / 'src'):
        path = root / relative
        if path.is_file():
            return path
    pytest.skip(f'{relative} is not under {mod.REPO_ROOT} or its src/')


def shipped(name):
    """The same configuration, loaded."""
    return mod.load_parameters(shipped_path(name))


def planner(**overrides):
    """The shipped scan_planner configuration, with optional overrides."""
    params = dict(shipped('planner'))
    params.update(overrides)
    return params


def measurement():
    """The shipped measurement file, exactly as committed."""
    with open(mod.DEFAULT_ROBOT) as handle:
        return json.load(handle)


def robot(base_height_m=0.35):
    """The same file with a base height, for the checks that need one."""
    value = measurement()
    value['measured']['base_height_m'] = base_height_m
    return value


def grade(check, params, value):
    report = mod.Report()
    check(params, value, report)
    return report.checks[0]['status'], report


def test_strictest_ranks_by_severity_not_alphabetically():
    assert mod.FAIL < mod.PASS < mod.WARN, 'the trap this helper exists for'
    assert mod.strictest(mod.PASS, mod.FAIL) == mod.FAIL
    assert mod.strictest(mod.FAIL, mod.PASS) == mod.FAIL
    assert mod.strictest(mod.WARN, mod.FAIL) == mod.FAIL
    assert mod.strictest(mod.PASS, mod.WARN) == mod.WARN
    assert mod.strictest(mod.PASS, mod.PASS) == mod.PASS


def test_footprint_accepts_the_shipped_configuration():
    status, report = grade(mod.check_footprint, planner(), robot())
    assert status == mod.PASS
    assert not report.failed()


def test_footprint_fails_when_only_the_radius_is_off():
    """The offset still matches, and that must not downgrade the failure."""
    status, report = grade(
        mod.check_footprint, planner(**{'grid_map.double_cylinder_radius': 0.99}),
        robot())
    assert status == mod.FAIL
    assert report.failed()


def test_footprint_fails_when_only_the_offset_is_off():
    status, report = grade(
        mod.check_footprint, planner(**{'grid_map.double_cylinder_offset': 0.99}),
        robot())
    assert status == mod.FAIL
    assert report.failed()


def test_clearances_fail_when_only_the_upward_clearance_is_off():
    """d_down grades WARN here, and must not downgrade d_up's FAIL."""
    status, report = grade(
        mod.check_clearances,
        planner(**{'grid_map.obstacles_inflation_z_up': 0.1,
                   'grid_map.obstacles_inflation_z_down': 0.1}),
        robot(base_height_m=0.35))
    assert status == mod.FAIL
    assert report.failed()


def test_clearances_accept_the_derived_values():
    """d_up = (H - h_b) + margin, d_down = h_b - step, for h_b = 0.35."""
    status, report = grade(
        mod.check_clearances,
        planner(**{'grid_map.obstacles_inflation_z_up': 0.35,
                   'grid_map.obstacles_inflation_z_down': 0.05}),
        robot(base_height_m=0.35))
    assert status == mod.PASS
    assert not report.failed()


@pytest.mark.parametrize('name', ['planner', 'global', 'reference'])
def test_every_config_has_its_own_option(monkeypatch, tmp_path, name):
    """They need not share a workspace, so each one is addressable."""
    target = tmp_path / f'{name}.yaml'
    monkeypatch.setattr(sys, 'argv', ['preflight', f'--{name}-yaml', str(target)])
    assert getattr(mod.parse_args(), f'{name}_yaml') == target


@pytest.mark.parametrize('name', ['planner', 'global', 'reference'])
def test_every_shipped_config_loads(name):
    assert shipped(name)


def test_all_checks_run_on_the_shipped_configuration():
    """With the base height still unmeasured, nothing may grade as FAIL."""
    params = {name: shipped(name) for name in mod.CONFIG_FILES}
    value = measurement()
    report = mod.Report()
    mod.check_footprint(params['planner'], value, report)
    mod.check_inflation_radius(params['planner'], params['global'], report)
    mod.check_clearances(params['planner'], value, report)
    mod.check_path_heights(
        params['planner'], params['global'], params['reference'], value, report)
    mod.check_reference_comment(shipped_path('reference'), params['planner'], report)
    mod.check_traversability_limits(params['global'], value, report)
    assert [check['status'] for check in report.checks].count(mod.FAIL) == 0
