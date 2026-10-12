"""Check launch wiring without running localization, xacro or the simulator."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('run_launch', PACKAGE / 'launch/run.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def nodes(monkeypatch, **overrides):
    values = dict(is_real_world='true', use_sim_time='false', sensor_type='lidar',
                  controller_mode='closed_loop', keypoints_file='', reference_path_file='',
                  navi_mode='3', enable_terminal_handoff='true', init_x='', init_y='', init_z='')
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: '/tmp/' + package)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: SimpleNamespace(**kwargs))
    return [action for action in module._setup(context) if isinstance(action, SimpleNamespace)]


def test_real_reference_mode_has_one_navigation_output(monkeypatch):
    launched = nodes(monkeypatch)
    by_name = {node.name: node for node in launched}
    assert ('cmd_vel', '/nav_cmd/local') in by_name['closed_loop_controller'].remappings
    follower = by_name['path_follower']
    settings = follower.parameters[-1]
    assert settings['enabled'] is False
    assert settings['cmd_vel_topic'] == '/nav_cmd/global'
    assert settings['odom_topic'] == '/loc/robo/odom'
    assert settings['goal_align_distance'] == 0.12 and settings['goal_tolerance'] == 0.2
    assert by_name['velocity_selector'].executable == 'velocity_selector'
    assert len([node for node in launched if node.name == 'velocity_selector']) == 1


@pytest.mark.parametrize('overrides,controller,topic', [
    ({'enable_terminal_handoff': 'false'}, 'closed_loop_controller', '/nav_cmd'),
    ({'navi_mode': '1'}, 'closed_loop_controller', '/nav_cmd'),
    ({'is_real_world': 'false'}, 'closed_loop_controller', '/quad_0/cmd_vel'),
    ({'controller_mode': 'open_loop'}, 'open_loop_controller', None),
])
def test_other_modes_preserve_original_launch(monkeypatch, overrides, controller, topic):
    launched = nodes(monkeypatch, **overrides)
    by_name = {node.name: node for node in launched}
    assert 'path_follower' not in by_name and 'velocity_selector' not in by_name
    assert controller in by_name
    if topic is not None:
        assert ('cmd_vel', topic) in by_name[controller].remappings
