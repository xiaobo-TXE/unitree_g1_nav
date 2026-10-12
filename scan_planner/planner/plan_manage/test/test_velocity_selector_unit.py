"""Exercise handoff, goal identity and stale/invalid input without DDS."""

import importlib.util
import math
from pathlib import Path
import sys

import pytest
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parents[2] / 'path_follow'))
spec = importlib.util.spec_from_file_location('velocity_selector', PACKAGE / 'scripts/velocity_selector.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
SelectionState = module.SelectionState


def path(x=5.0, yaw=0.0, frame='map'):
    result = NavPath()
    result.header.frame_id = frame
    for px in (0.0, x):
        pose = PoseStamped()
        pose.pose.position.x = px
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        result.poses.append(pose)
    return result


def odom(x=0.0, frame='map'):
    result = Odometry()
    result.header.frame_id = frame
    result.pose.pose.position.x = x
    result.pose.pose.orientation.w = 1.0
    return result


def command(v=0.25):
    result = Twist()
    result.linear.x = v
    return result


def ready_state(x=0.0):
    state = SelectionState()
    state.receive_path(path())
    state.receive_odom(odom(x), 1.0)
    state.update_selection(1.0)
    state.ready = True
    return state


@pytest.mark.parametrize('x,source', [(2.999, 'local'), (3.0, 'global'), (4.0, 'global')])
def test_initial_distance_and_inclusive_boundary(x, source):
    assert ready_state(x).source == source


def test_handoff_stops_clears_cache_and_only_accepts_new_selected_commands():
    state = ready_state()
    state.receive_command('local', command(), 1.0)
    assert state.output(1.0).linear.x == 0.25
    state.receive_command('global', command(0.4), 1.0)
    state.receive_odom(odom(3.0), 1.1)
    assert state.output(1.1).linear.x == 0.0
    assert state.source == 'global' and not state.commands and not state.ready
    state.receive_command('global', command(), 1.1)
    assert not state.commands
    state.ready = True
    state.receive_command('global', command(0.3), 1.15)
    state.receive_command('local', command(0.5), 1.15)
    assert state.output(1.15).linear.x == 0.3
    state.receive_command('local', Twist(), 1.16)
    assert state.output(1.16).linear.x == 0.3


def test_latch_survives_distance_drift_and_same_goal_republication():
    state = ready_state(4.0)
    generation = state.generation
    state.receive_odom(odom(0.0), 1.1)
    changed = path()
    changed.header.stamp.sec = 100
    changed.poses[0].pose.position.y = 3.0
    changed.poses[-1].pose.orientation.w = -1.0
    state.receive_path(changed)
    state.update_selection(1.1)
    assert state.source == 'global' and state.generation == generation


@pytest.mark.parametrize('new_path', [path(6.0), path(5.0, 0.5)])
def test_new_goal_clears_latch_and_commands(new_path):
    state = ready_state(4.0)
    state.receive_command('global', command(), 1.0)
    state.receive_odom(odom(0.0), 1.1)
    state.receive_path(new_path)
    assert state.source is None and not state.commands
    state.update_selection(1.1)
    assert state.source == 'local' and not state.ready


@pytest.mark.parametrize('time,reason', [(1.31, 'odom_timeout'), (1.2, 'command_timeout')])
def test_timeout_stops_without_fallback(time, reason):
    state = ready_state(4.0)
    if reason == 'command_timeout':
        state.receive_command('global', command(), 0.8)
    else:
        state.receive_command('global', command(), 1.0)
    assert state.output(time).linear.x == 0.0
    assert state.source == 'global' and state.reason == reason
    state.receive_odom(odom(0.0), time)
    state.receive_command('global', command(0.1), time)
    assert state.output(time).linear.x == 0.1


@pytest.mark.parametrize('bad_path', [NavPath(), path(frame='world')])
def test_empty_or_wrong_frame_path_stops(bad_path):
    state = ready_state()
    state.receive_command('local', command(), 1.0)
    state.receive_path(bad_path)
    assert state.output(1.0).linear.x == 0.0 and state.source is None
    state.receive_path(path())
    state.update_selection(1.0)
    assert state.source == 'local' and not state.ready


def test_nonfinite_path_pose_and_mixed_frames_are_invalid():
    for value in (float('nan'), float('inf')):
        state = ready_state()
        invalid = path()
        invalid.poses[0].pose.position.y = value
        state.receive_path(invalid)
        assert state.goal is None
    state = ready_state()
    invalid = path()
    invalid.poses[0].header.frame_id = 'world'
    state.receive_path(invalid)
    assert state.goal is None


def test_invalid_odom_or_speed_stops_and_keeps_source():
    state = ready_state(4.0)
    state.receive_command('global', command(), 1.0)
    state.receive_command('global', command(float('nan')), 1.1)
    assert state.output(1.1).linear.x == 0.0
    state.receive_command('global', command(), 1.1)
    state.receive_odom(odom(frame='world'), 1.1)
    assert state.output(1.1).linear.x == 0.0 and state.source == 'global'


def test_segment_fallback_matches_follower_with_duplicate_endpoints():
    state = SelectionState()
    message = path()
    for pose in message.poses:
        pose.pose.orientation.w = 0.0
    message.poses.append(message.poses[-1])
    state.receive_path(message)
    assert state.goal[3] == pytest.approx(0.0)


@pytest.mark.parametrize('name', ['switch_distance', 'cmd_timeout', 'odom_timeout'])
@pytest.mark.parametrize('value', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_parameters_rejected(name, value):
    with pytest.raises(ValueError):
        SelectionState(**{name: value})
