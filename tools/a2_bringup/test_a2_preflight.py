#!/usr/bin/env python3
"""Tests for a2_preflight.py.

The grader must not let a milder result hide a worse one: the status names are
plain strings, and 'FAIL' sorts *below* 'PASS' and 'WARN', so grading two
deviations with max() silently downgrades a failure. These tests pin the
severity order and the two checks that grade more than one deviation.

    python3 -m pytest tools/a2_bringup/test_a2_preflight.py
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'a2_preflight', HERE / 'a2_preflight.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def planner(**overrides):
    """The shipped scan_planner configuration, with optional overrides."""
    with open(mod.REPO_ROOT / mod.CONFIG_FILES['planner']) as handle:
        params = yaml.safe_load(handle)['scan_planner_node']['ros__parameters']
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


@pytest.mark.parametrize('name, expected', [
    ('planner', mod.CONFIG_FILES['planner']),
    ('global', mod.CONFIG_FILES['global']),
    ('reference', mod.CONFIG_FILES['reference']),
])
def test_every_config_file_exists_and_is_readable(name, expected):
    """The robot keeps these in separate workspaces, so all are overridable."""
    assert expected == mod.CONFIG_FILES[name]
    assert mod.load_parameters(mod.REPO_ROOT / expected)


def test_all_checks_run_on_the_shipped_configuration():
    """With the base height still unmeasured, nothing may grade as FAIL."""
    params = {
        'planner': planner(),
        'global': mod.load_parameters(mod.REPO_ROOT / mod.CONFIG_FILES['global']),
        'reference': mod.load_parameters(
            mod.REPO_ROOT / mod.CONFIG_FILES['reference']),
    }
    value = measurement()
    report = mod.Report()
    mod.check_footprint(params['planner'], value, report)
    mod.check_inflation_radius(params['planner'], params['global'], report)
    mod.check_clearances(params['planner'], value, report)
    mod.check_path_heights(
        params['planner'], params['global'], params['reference'], value, report)
    mod.check_reference_comment(
        mod.REPO_ROOT / mod.CONFIG_FILES['reference'], params['planner'], report)
    mod.check_traversability_limits(params['global'], value, report)
    assert [check['status'] for check in report.checks].count(mod.FAIL) == 0
