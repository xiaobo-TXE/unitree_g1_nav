import importlib.util
import math
import sys
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from std_srvs.srv import SetBool

MODULE_PATH = Path(__file__).resolve().parents[1] / 'path_follow' / 'path_follow_py_node.py'
sys.path.insert(0, str(MODULE_PATH.parents[1]))

spec = importlib.util.spec_from_file_location('path_follow_py_node', MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

PathFollower = module.PathFollower


def make_pose(x, y, yaw=0.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )
    return pose


def test_goal_heading_must_be_aligned_before_goal_is_reached():
    follower = PathFollower.__new__(PathFollower)
    follower.path = NavPath()
    follower.path.poses = [
        PoseStamped(pose=make_pose(0.0, 0.0, 0.0)),
        PoseStamped(pose=make_pose(1.0, 0.0, math.pi / 2.0)),
    ]
    follower.robot_x = 0.98
    follower.robot_y = 0.00
    follower.robot_yaw = 0.0
    follower.goal_tolerance = 0.2
    follower.goal_yaw_tolerance = 0.15
    follower.goal_yaw_mode = 'path'

    assert follower.is_goal_reached() is False


def test_goal_heading_is_accepted_when_within_yaw_tolerance():
    follower = PathFollower.__new__(PathFollower)
    follower.path = NavPath()
    follower.path.poses = [
        PoseStamped(pose=make_pose(0.0, 0.0, 0.0)),
        PoseStamped(pose=make_pose(1.0, 0.0, 0.0)),
    ]
    follower.robot_x = 0.98
    follower.robot_y = 0.00
    follower.robot_yaw = 0.10
    follower.goal_tolerance = 0.2
    follower.goal_yaw_tolerance = 0.15
    follower.goal_yaw_mode = 'path'

    assert follower.is_goal_reached() is True


def make_follower(goal_yaw=math.pi / 2.0):
    follower = PathFollower.__new__(PathFollower)
    values = dict(
        path=None, active_goal=None, control_state='tracking', odom_received=True, enabled=True,
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
        goal_tolerance=0.2, goal_align_distance=0.12,
        goal_exit_tolerance=0.35, goal_yaw_tolerance=0.15,
        goal_yaw_mode='path', goal_slowdown_distance=0.8,
        angular_kp=1.5, max_angular_vel=0.5, linear_vel=0.5, max_linear_vel=0.5,
        slow_down_angle=0.6, rotate_in_place_angle=math.pi / 3.0,
        lookahead_distance=0.8, diagnostic_enabled=False, last_warn_time=0.0,
    )
    for name, value in values.items():
        setattr(follower, name, value)
    follower.commands = []
    follower.cmd_vel_pub = SimpleNamespace(publish=follower.commands.append)
    follower.get_logger = lambda: SimpleNamespace(
        info=lambda *args: None, debug=lambda *args: None, warning=lambda *args: None)
    path = NavPath()
    path.header.frame_id = 'map'
    path.poses = [PoseStamped(pose=make_pose(0.0, 0.0)),
                  PoseStamped(pose=make_pose(1.0, 0.0, goal_yaw))]
    follower.path_callback(path)
    return follower


def test_reached_goal_stays_stopped_after_logged_position_drift_and_republish():
    follower = make_follower(math.radians(-5.6539))
    follower.path.poses[-1].pose.position.x = -0.3833
    follower.path.poses[-1].pose.position.y = 3.1674
    follower.path_callback(follower.path)
    follower.robot_x, follower.robot_y = -0.2423, 3.2750
    follower.robot_yaw = math.radians(2.88)
    follower.control_callback()
    assert follower.control_state == 'goal_reached'

    follower.robot_x, follower.robot_y = -0.2188, 3.2978
    follower.robot_yaw = math.radians(-0.40)
    follower.path.header.stamp.sec = 123
    follower.path.poses[0].pose.position.y = 0.5
    follower.path_callback(follower.path)
    follower.control_callback()
    assert follower.control_state == 'goal_reached'
    assert follower.commands[-1].linear.x == follower.commands[-1].angular.z == 0.0


@pytest.mark.parametrize('change', ['position', 'yaw', 'frame'])
def test_new_goal_releases_completion_latch(change):
    follower = make_follower(0.0)
    follower.robot_x = 1.0
    follower.control_callback()
    assert follower.control_state == 'goal_reached'
    if change == 'position':
        follower.path.poses[-1].pose.position.x = 2.0
    elif change == 'yaw':
        follower.path.poses[-1].pose = make_pose(1.0, 0.0, math.pi / 2.0)
    else:
        follower.path.header.frame_id = 'world'
    follower.path_callback(follower.path)
    assert follower.control_state == 'tracking'


def test_alignment_hysteresis_and_large_drift_recovery():
    follower = make_follower()
    follower.robot_x = 0.89
    follower.control_callback()
    assert follower.control_state == 'align_goal_yaw'
    follower.robot_x = 0.79
    follower.control_callback()
    assert follower.control_state == 'align_goal_yaw'
    assert follower.commands[-1].linear.x == 0.0
    follower.robot_x = 0.64
    follower.control_callback()
    assert follower.control_state == 'recover_goal_position'
    follower.robot_x = 0.89
    follower.control_callback()
    assert follower.control_state == 'align_goal_yaw'


def test_aligned_heading_outside_position_tolerance_requires_recovery():
    follower = make_follower()
    follower.robot_x = 0.89
    follower.control_callback()
    follower.robot_x = 0.79
    follower.robot_yaw = math.pi / 2.0
    follower.control_callback()
    assert follower.control_state == 'recover_goal_position'
    assert follower.commands[-1].linear.x == 0.0


def test_alignment_uses_inner_distance_and_completion_keeps_outer_tolerance():
    follower = make_follower()
    follower.robot_x = 0.85
    follower.control_callback()
    assert follower.control_state == 'tracking'
    assert follower.commands[-1].linear.x > 0.0
    follower.robot_x = 0.90
    follower.control_callback()
    assert follower.control_state == 'align_goal_yaw'
    assert follower.commands[-1].linear.x == 0.0
    follower.robot_x = 0.83
    follower.robot_yaw = math.pi / 2.0
    follower.control_callback()
    assert follower.control_state == 'goal_reached'
    assert follower.commands[-1].linear.x == follower.commands[-1].angular.z == 0.0


def test_near_goal_slows_down_and_large_bearing_error_stops_translation():
    follower = make_follower()
    follower.robot_x = 0.6
    follower.control_callback()
    assert follower.commands[-1].linear.x == pytest.approx(0.25)
    for error in (math.pi / 3.0, math.pi / 2.0, math.pi):
        follower.robot_yaw = error
        follower.control_callback()
        assert follower.commands[-1].linear.x == 0.0


def test_exact_tolerance_boundaries_finish_instead_of_tracking():
    follower = make_follower(0.0)
    follower.robot_x = 1.0
    follower.robot_y = follower.goal_tolerance
    follower.robot_yaw = -follower.goal_yaw_tolerance
    follower.control_callback()
    assert follower.control_state == 'goal_reached'


def test_empty_path_cancels_and_allows_same_goal_to_restart():
    follower = make_follower(0.0)
    follower.robot_x = 1.0
    follower.control_callback()
    path = follower.path
    follower.path_callback(NavPath())
    follower.control_callback()
    assert follower.commands[-1].linear.x == follower.commands[-1].angular.z == 0.0
    follower.path_callback(path)
    assert follower.control_state == 'tracking'


def test_segment_heading_skips_duplicate_terminal_points():
    follower = make_follower()
    follower.goal_yaw_mode = 'segment'
    follower.path.poses.append(PoseStamped(pose=make_pose(1.0, 0.0)))
    assert follower.get_goal_yaw() == pytest.approx(0.0)


def test_disabled_follower_does_not_finish_and_activation_resets_terminal_state():
    follower = make_follower(0.0)
    follower.enabled = False
    follower.robot_x = 1.0
    follower.control_callback()
    assert follower.control_state == 'tracking' and not follower.commands
    follower.control_state = 'goal_reached'
    request = SetBool.Request()
    request.data = True
    response = follower.set_enabled_callback(request, SetBool.Response())
    assert response.success and json.loads(response.message) == list(follower.active_goal)
    assert follower.control_state == 'tracking'
    follower.control_callback()
    assert follower.control_state == 'goal_reached'
    follower.set_enabled_callback(request, SetBool.Response())
    assert follower.control_state == 'goal_reached'
    request.data = False
    follower.set_enabled_callback(request, SetBool.Response())
    assert follower.commands[-1].linear.x == follower.commands[-1].angular.z == 0.0
    count = len(follower.commands)
    follower.control_callback()
    assert len(follower.commands) == count


@pytest.mark.parametrize('goal_yaw', [0.0, math.pi / 2.0, math.pi])
@pytest.mark.parametrize('response_time', [0.0, 0.4])
def test_closed_loop_converges_from_side_with_heading_change_and_motion_lag(
        goal_yaw, response_time):
    follower = make_follower(goal_yaw)
    follower.robot_x, follower.robot_y = 1.0, -0.6
    actual_v = actual_w = 0.0
    dt = 0.05
    for step in range(2400):
        if step % 20 == 0:
            follower.path_callback(follower.path)
        follower.control_callback()
        cmd = follower.commands[-1]
        gain = 1.0 if response_time == 0.0 else 1.0 - math.exp(-dt / response_time)
        actual_v += gain * (cmd.linear.x - actual_v)
        actual_w += gain * (cmd.angular.z - actual_w)
        follower.robot_x += actual_v * math.cos(follower.robot_yaw) * dt
        follower.robot_y += actual_v * math.sin(follower.robot_yaw) * dt
        follower.robot_yaw = follower.normalize_angle(follower.robot_yaw + actual_w * dt)
        if follower.control_state == 'goal_reached':
            break
    assert follower.control_state == 'goal_reached', 'Failed to converge within 120 seconds'
    assert follower.is_goal_reached()
    follower.robot_x += 0.1
    follower.control_callback()
    assert follower.commands[-1].linear.x == follower.commands[-1].angular.z == 0.0
