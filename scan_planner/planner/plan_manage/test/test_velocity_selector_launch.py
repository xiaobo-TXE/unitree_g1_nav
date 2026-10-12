"""Run the installed selector and follower with live ROS path/odom/velocity inputs."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


@pytest.mark.launch_test
def generate_test_description():
    follower = launch_ros.actions.Node(
        package='path_follow', executable='path_follow_py_node', name='path_follower',
        parameters=[{'enabled': False, 'cmd_vel_topic': '/nav_cmd/global',
                     'diagnostic_enabled': False}], output='screen')
    selector = launch_ros.actions.Node(
        package='scan_planner', executable='velocity_selector', name='velocity_selector',
        output='screen')
    return (launch.LaunchDescription([
        follower, selector, launch_testing.actions.ReadyToTest()]),
        {'follower': follower, 'selector': selector})


class TestTerminalHandoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def test_live_handoff_and_single_output(self, proc_info, follower, selector):
        proc_info.assertWaitForStartup(process=follower, timeout=10)
        proc_info.assertWaitForStartup(process=selector, timeout=10)
        node = rclpy.create_node('terminal_handoff_test', context=self.context)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        path_pub = node.create_publisher(Path, '/global_path', qos)
        odom_pub = node.create_publisher(Odometry, '/loc/robo/odom', qos_profile_sensor_data)
        local_pub = node.create_publisher(Twist, '/nav_cmd/local', 1)
        outputs = []
        subscription = node.create_subscription(Twist, '/nav_cmd', outputs.append, 100)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base'
        odom.pose.pose.orientation.w = 1.0
        local = Twist()
        local.linear.x = 0.123

        def goal(x, yaw=0.0):
            message = Path()
            message.header.frame_id = 'map'
            for px in (0.0, x):
                pose = PoseStamped()
                pose.pose.position.x = px
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                message.poses.append(pose)
            return message

        def drive(duration, publish_odom=True, publish_local=True):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if publish_odom:
                    odom_pub.publish(odom)
                if publish_local:
                    local_pub.publish(local)
                executor.spin_once(timeout_sec=0.02)
                time.sleep(0.01)

        def wait_for(predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                drive(0.05)
                if outputs and predicate(outputs[-1]):
                    return
            self.fail('Timed out waiting for selected velocity')

        try:
            drive(0.7)
            path_pub.publish(goal(5.0))
            wait_for(lambda cmd: abs(cmd.linear.x - 0.123) < 1e-6)
            publishers = node.get_publishers_info_by_topic('/nav_cmd')
            self.assertEqual([pub.node_name for pub in publishers], ['velocity_selector'])
            drive(0.5, publish_local=False)
            self.assertEqual(outputs[-1].linear.x, 0.0)
            wait_for(lambda cmd: abs(cmd.linear.x - 0.123) < 1e-6)

            outputs.clear()
            odom.pose.pose.position.x = 3.5
            wait_for(lambda cmd: cmd.linear.x > 0.2)
            self.assertTrue(any(cmd.linear.x == 0.0 for cmd in outputs))
            drive(0.5, publish_odom=False)
            self.assertEqual(outputs[-1].linear.x, 0.0)
            wait_for(lambda cmd: cmd.linear.x > 0.2)

            # Moving outside 2 m and local stop commands cannot undo the handoff.
            odom.pose.pose.position.x = 0.0
            local.linear.x = 0.0
            path_pub.publish(goal(5.0))
            wait_for(lambda cmd: cmd.linear.x > 0.2)
            odom.pose.pose.position.x = 5.0
            wait_for(lambda cmd: cmd.linear.x == cmd.angular.z == 0.0)
            drive(0.3)
            odom.pose.pose.position.x = 4.7
            path_pub.publish(goal(5.0))
            drive(0.5)
            self.assertEqual(outputs[-1].linear.x, 0.0)

            # A new goal releases the latch; subsequent terminal entry activates again.
            local.linear.x = 0.123
            path_pub.publish(goal(8.0))
            wait_for(lambda cmd: abs(cmd.linear.x - 0.123) < 1e-6)
            odom.pose.pose.position.x = 7.0
            wait_for(lambda cmd: cmd.linear.x > 0.2)
            # Yaw-only goal replacement must also be acknowledged before moving.
            path_pub.publish(goal(8.0, math.pi / 2.0))
            drive(0.4)
            odom.header.frame_id = 'world'
            drive(0.4)
            self.assertEqual(outputs[-1].linear.x, 0.0)
            odom.header.frame_id = 'map'
            wait_for(lambda cmd: cmd.linear.x > 0.2)
            path_pub.publish(Path())
            drive(0.4)
            self.assertEqual(outputs[-1].linear.x, 0.0)
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_subscription(subscription)
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
