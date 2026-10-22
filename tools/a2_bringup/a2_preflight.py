#!/usr/bin/env python3
"""Offline preflight checks for migrating the body parameters from G1 to A2.

No ROS and no hardware are needed: the script reads the planner YAML files
together with the measured robot dimensions in ``a2_robot.json`` and reports
every inconsistency it can detect between them. It never writes anything.

Exit status is 0 when no check fails and 1 otherwise, so it can gate a commit.

Usage:
    python3 tools/a2_bringup/a2_preflight.py
    python3 tools/a2_bringup/a2_preflight.py --robot-json /tmp/a2.json
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]

PASS, WARN, FAIL, SKIP, INFO = 'PASS', 'WARN', 'FAIL', 'SKIP', 'INFO'
FAILING = (FAIL,)
# The status names do not sort by severity: 'FAIL' < 'PASS' < 'WARN'. Grading a
# check by two deviations must therefore rank them explicitly, otherwise the
# milder of the two hides the worse one.
SEVERITY = {PASS: 0, WARN: 1, FAIL: 2}

# Default locations inside this repository. The real robot keeps scan_planner
# and global_planner in two separate workspaces, so every file can be pointed at
# explicitly with the matching --*-yaml option.
CONFIG_FILES = {
    'planner': 'scan_planner/planner/plan_manage/config/planner.yaml',
    'global': 'global_planner/config/global_planner.yaml',
    'reference': 'scan_planner/planner/plan_manage/config/reference_path.map.yaml',
}
DEFAULT_ROBOT = HERE / 'a2_robot.json'

# Deviations from the measured envelope, in metres. A footprint that is off by
# more than FOOTPRINT_FAIL no longer models the real body.
FOOTPRINT_TOL = 0.02
FOOTPRINT_FAIL = 0.05
# Deviations of the vertical clearances and of the height offsets.
CLEARANCE_TOL = 0.03
CLEARANCE_FAIL = 0.08
OFFSET_TOL = 0.10
OFFSET_FAIL = 0.25


def load_parameters(path):
    """Return the ``ros__parameters`` block of the single node in a YAML file."""
    with open(path) as handle:
        document = yaml.safe_load(handle)
    node_name = next(iter(document))
    return document[node_name]['ros__parameters']


def number(value):
    """Format a float for the report, keeping it short and readable."""
    return f'{value:.3f}'.rstrip('0').rstrip('.')


def worst(delta, tol, fail):
    """Grade an absolute deviation against the tolerance bands."""
    if delta > fail:
        return FAIL
    if delta > tol:
        return WARN
    return PASS


def strictest(*statuses):
    """The most severe of several statuses, which do not sort themselves."""
    return max(statuses, key=SEVERITY.__getitem__)


class Report:
    """Collects check outcomes and renders them as a plain-text report."""

    def __init__(self):
        self.checks = []

    def add(self, status, title, details=(), fixes=()):
        self.checks.append({
            'status': status,
            'title': title,
            'details': list(details),
            'fixes': list(fixes),
        })

    def counts(self):
        tally = {}
        for check in self.checks:
            tally[check['status']] = tally.get(check['status'], 0) + 1
        return tally

    def render(self, header):
        lines = list(header)
        for check in self.checks:
            lines.append(f"[{check['status']}] {check['title']}")
            for detail in check['details']:
                lines.append(f'       {detail}')
            for fix in check['fixes']:
                lines.append(f'       fix -> {fix}')
        tally = self.counts()
        summary = ', '.join(
            f'{tally[status]} {status.lower()}'
            for status in (PASS, WARN, FAIL, SKIP, INFO)
            if status in tally
        )
        lines.append('')
        lines.append(f'summary: {summary or "no checks ran"}')
        return '\n'.join(lines)

    def failed(self):
        return any(check['status'] in FAILING for check in self.checks)


def check_footprint(planner, robot, report):
    """The two cylinders must cover the measured body rectangle.

    SCAN-Planner places the cylinder centres symmetrically along the body
    axis, so the tightest two equal circles that cover the L x W rectangle sit
    at L/4 and have radius hypot(L/4, W/2). No body corner is left outside the
    collision model, at the cost of an envelope larger than the body itself.
    """
    length = robot['measured']['length_m']
    width = robot['measured']['width_m']
    radius = planner['grid_map.double_cylinder_radius']
    offset = planner['grid_map.double_cylinder_offset']

    want_offset = length / 4.0
    want_radius = math.hypot(want_offset, width / 2.0)
    delta_radius = abs(radius - want_radius)
    delta_offset = abs(offset - want_offset)
    details = [
        f'configured: r={number(radius)} off={number(offset)}'
        f' -> envelope {number(2 * radius)} x {number(2 * (offset + radius))} m',
        f'measured  : r={number(want_radius)} off={number(want_offset)}'
        f' -> envelope {number(2 * want_radius)} x'
        f' {number(2 * (want_offset + want_radius))} m'
        f' covering body {number(width)} x {number(length)} m',
        f'delta     : r={number(delta_radius)} off={number(delta_offset)} m',
    ]
    fixes = []
    if delta_radius > FOOTPRINT_TOL:
        fixes.append(
            'scan_planner/.../config/planner.yaml  '
            f'grid_map.double_cylinder_radius: {want_radius:.2f}'
        )
    if delta_offset > FOOTPRINT_TOL:
        fixes.append(
            'scan_planner/.../config/planner.yaml  '
            f'grid_map.double_cylinder_offset: {want_offset:.2f}'
        )
    status = strictest(
        worst(delta_radius, FOOTPRINT_TOL, FOOTPRINT_FAIL),
        worst(delta_offset, FOOTPRINT_TOL, FOOTPRINT_FAIL),
    )
    report.add(status, 'twin-cylinder footprint', details, fixes)


def check_inflation_radius(planner, global_params, report):
    """The global obstacle inflation should match the local cylinder radius."""
    cylinder = planner['grid_map.double_cylinder_radius']
    radius = global_params['robot_radius']
    if radius <= 0.0:
        report.add(
            FAIL, 'global obstacle inflation',
            [f'robot_radius={number(radius)} disables obstacle inflation entirely',
             'paths can graze obstacles even though the local planner keeps clearance'],
            [f'global_planner/config/global_planner.yaml  robot_radius: {cylinder:.2f}'],
        )
        return
    delta = abs(radius - cylinder)
    report.add(
        worst(delta, FOOTPRINT_TOL, FOOTPRINT_FAIL),
        'global obstacle inflation',
        [f'robot_radius={number(radius)} vs double_cylinder_radius={number(cylinder)}'
         f' (delta {number(delta)} m)'],
        [f'global_planner/config/global_planner.yaml  robot_radius: {cylinder:.2f}']
        if delta > FOOTPRINT_TOL else [],
    )


def check_clearances(planner, robot, report):
    """d_up / d_down follow from the base height, per the paper's derivation."""
    base = robot['measured'].get('base_height_m')
    height = robot['measured']['height_m']
    step = robot['limits']['step_height_m']
    margin = robot['margins']['up_margin_m']
    configured_up = planner['grid_map.obstacles_inflation_z_up']
    configured_down = planner['grid_map.obstacles_inflation_z_down']

    if base is None:
        report.add(
            SKIP, 'vertical clearances (d_up / d_down)',
            ['needs base_height_m in a2_robot.json; run a2_measure_base_height.py',
             f'd_up   = (height {number(height)} - base) + margin {number(margin)}',
             f'd_down = base - step {number(step)}'],
        )
        return

    want_up = (height - base) + margin
    want_down = base - step
    delta_up = abs(configured_up - want_up)
    delta_down = abs(configured_down - want_down)
    status = strictest(
        worst(delta_up, CLEARANCE_TOL, CLEARANCE_FAIL),
        worst(delta_down, CLEARANCE_TOL, CLEARANCE_FAIL),
    )
    details = [
        f'configured: d_up={number(configured_up)} d_down={number(configured_down)}',
        f'derived   : d_up={number(want_up)} d_down={number(want_down)}',
        f'  (base_height={number(base)} step={number(step)} margin={number(margin)})',
    ]
    fixes = []
    if delta_up > CLEARANCE_TOL:
        fixes.append(
            'scan_planner/.../config/planner.yaml  '
            f'grid_map.obstacles_inflation_z_up: {want_up:.2f}'
        )
    if delta_down > CLEARANCE_TOL:
        if want_down < 0.0:
            details.append(
                'd_down would be negative: the body centre sits below the step height'
            )
            status = FAIL
        fixes.append(
            'scan_planner/.../config/planner.yaml  '
            f'grid_map.obstacles_inflation_z_down: {max(want_down, 0.0):.2f}'
        )
    report.add(status, 'vertical clearances (d_up / d_down)', details, fixes)


def check_path_heights(planner, global_params, reference_params, robot, report):
    """path_z_offset and body_height both lift the reference path; they add up."""
    offset = global_params['path_z_offset']
    body = planner['grid_map.body_height']
    total = offset + body
    base = robot['measured'].get('base_height_m')
    waypoint_z = reference_params['waypoints'][2::3]

    details = [
        f'global path_z_offset={number(offset)} + body_height={number(body)}'
        f' = {number(total)} m above terrain',
        f'reference_path.map.yaml waypoint z: {number(min(waypoint_z))}'
        f' .. {number(max(waypoint_z))}',
        'published /global_path z = terrain z + path_z_offset, and scan_planner'
        ' adds body_height again in navi_mode=3',
    ]
    fixes = []
    if base is None:
        details.append('compare the total against base_height once it is measured')
        report.add(INFO, 'path height offsets', details, fixes)
        return

    delta = abs(total - base)
    if offset > 0.0 and body > 0.0:
        details.append(
            'both offsets are non-zero, so the path is lifted twice'
        )
    if delta > OFFSET_TOL:
        details.append(f'total is {number(delta)} m away from base_height={number(base)}')
        fixes.append(
            'global_planner/config/global_planner.yaml  path_z_offset: '
            f'{base - body:.2f}   (keep path_z_offset + body_height = {base:.2f})'
        )
    report.add(
        worst(delta, OFFSET_TOL, OFFSET_FAIL),
        'path height offsets', details, fixes,
    )


def check_reference_comment(reference_path, planner, report):
    """The comment in reference_path.map.yaml may document a stale body_height."""
    text = reference_path.read_text()
    claims = re.findall(r'body_height[^0-9]{0,20}([0-9]+(?:\.[0-9]+)?)\s*m', text)
    body = planner['grid_map.body_height']
    if not claims:
        return
    claim = float(claims[0])
    if abs(claim - body) < 1e-6:
        report.add(INFO, 'reference_path.map.yaml comment',
                   [f'comment and config agree on body_height={number(body)}'])
        return
    report.add(
        WARN, 'reference_path.map.yaml comment',
        [f'comment claims a {number(claim)} m lift,',
         f'but grid_map.body_height is {number(body)} -> the comment is stale'],
        ['update the comment, or set body_height to the value that actually applies'],
    )


def check_traversability_limits(global_params, robot, report):
    """The step and slope limits must not exceed the vendor specification."""
    step = global_params['max_step_height']
    slope = global_params['max_slope_degrees']
    spec_step = robot['limits']['step_height_m']
    spec_slope = robot['limits']['max_slope_deg']

    details = [
        f'max_step_height={number(step)} vs spec {number(spec_step)} m',
        f'max_slope_degrees={number(slope)} vs spec {number(spec_slope)} deg',
    ]
    fixes = []
    status = PASS
    if step > spec_step + 1e-6:
        status = WARN
        fixes.append(
            'global_planner/config/global_planner.yaml  '
            f'max_step_height: {spec_step:.2f}'
        )
    if slope > spec_slope + 1e-6:
        status = WARN
        fixes.append(
            'global_planner/config/global_planner.yaml  '
            f'max_slope_degrees: {spec_slope:.1f}'
        )
    report.add(status, 'traversability limits', details, fixes)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Offline A2 body-parameter preflight checks.')
    parser.add_argument('--robot-json', type=Path, default=DEFAULT_ROBOT,
                        help='measured robot dimensions (default: %(default)s)')
    parser.add_argument('--planner-yaml', type=Path,
                        default=REPO_ROOT / CONFIG_FILES['planner'],
                        help='scan_planner parameters (default: %(default)s)')
    parser.add_argument('--global-yaml', type=Path,
                        default=REPO_ROOT / CONFIG_FILES['global'],
                        help='global_planner parameters (default: %(default)s)')
    parser.add_argument('--reference-yaml', type=Path,
                        default=REPO_ROOT / CONFIG_FILES['reference'],
                        help='reference path parameters (default: %(default)s)')
    return parser.parse_args()


def main():
    args = parse_args()
    with open(args.robot_json) as handle:
        robot = yaml.safe_load(handle)

    configs = {
        'planner': args.planner_yaml,
        'global': args.global_yaml,
        'reference': args.reference_yaml,
    }
    params = {name: load_parameters(path) for name, path in configs.items()}

    measured = robot['measured']
    header = [
        '== A2 body-parameter preflight ==',
        f"robot : {robot['model']}  length={number(measured['length_m'])}"
        f" width={number(measured['width_m'])} height={number(measured['height_m'])}"
        f" base_height={measured.get('base_height_m') or 'unknown'}",
        *(f'{name:<9}: {path}' for name, path in configs.items()),
        '',
    ]

    report = Report()
    check_footprint(params['planner'], robot, report)
    check_inflation_radius(params['planner'], params['global'], report)
    check_clearances(params['planner'], robot, report)
    check_path_heights(
        params['planner'], params['global'], params['reference'], robot, report)
    check_reference_comment(args.reference_yaml, params['planner'], report)
    check_traversability_limits(params['global'], robot, report)

    print(report.render(header))
    return 1 if report.failed() else 0


if __name__ == '__main__':
    raise SystemExit(main())
