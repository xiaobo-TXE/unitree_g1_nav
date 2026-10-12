"""Exercise reference-path planning with a changing map -> world transform."""

import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml"))
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config, {"fsm.navi_mode": 3}],
        output="screen",
    )
    return (
        launch.LaunchDescription([
            planner,
            launch_testing.actions.ReadyToTest(),
            launch.actions.TimerAction(
                period=18.0,
                actions=[launch.actions.Shutdown(reason="dynamic TF test complete")],
            ),
        ]),
        {"planner": planner},
    )


class TestDynamicReferenceTf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def test_timeout_recovery_and_jump(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=10)
        node = rclpy.create_node("dynamic_reference_tf_test", context=self.context)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        odom_pub = node.create_publisher(Odometry, "body_pose", qos_profile_sensor_data)
        path_pub = node.create_publisher(Path, "initial_path", 10)
        tf_pub = TransformBroadcaster(node)
        received = []
        node.create_subscription(Bspline, "planning/bspline", received.append, 10)

        path = Path()
        path.header.frame_id = "map"
        for x in (0.0, 2.0):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.z = 1.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        def pump(seconds, tf_y=None, send_path=False):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                odom = Odometry()
                odom.header.frame_id = "world"
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.pose.pose.position.z = 1.0
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                if send_path:
                    path.header.stamp = node.get_clock().now().to_msg()
                    path_pub.publish(path)
                if tf_y is not None:
                    tf = TransformStamped()
                    tf.header.frame_id = "map"
                    tf.child_frame_id = "world"
                    tf.header.stamp = node.get_clock().now().to_msg()
                    tf.transform.translation.y = tf_y
                    tf.transform.rotation.w = 1.0
                    tf_pub.sendTransform(tf)
                executor.spin_once(timeout_sec=0.04)

        def stationary(msg):
            return bool(msg.pos_pts) and all(
                math.isclose(p.x, msg.pos_pts[0].x, abs_tol=1e-6) and
                math.isclose(p.y, msg.pos_pts[0].y, abs_tol=1e-6)
                for p in msg.pos_pts
            )

        try:
            pump(0.3, send_path=True)
            pump(0.8)
            self.assertTrue(any(stationary(msg) for msg in received),
                            "stale/missing TF did not stop the robot")

            pump(2.0, tf_y=0.0)
            moving = [msg for msg in received if not stationary(msg)]
            self.assertTrue(moving, "fresh TF did not resume local planning")
            before_jump = max(msg.traj_id for msg in moving)

            # A short dropout must not replace the still-valid world trajectory.
            before_dropout = len(received)
            pump(0.2)
            self.assertFalse(any(stationary(msg) for msg in received[before_dropout:]))
            pump(0.2, tf_y=0.0)
            before_jump = max(msg.traj_id for msg in received if not stationary(msg))

            pump(1.0, tf_y=0.3)
            after_jump = [msg for msg in received if msg.traj_id > before_jump]
            self.assertTrue(after_jump, "map -> world jump did not trigger a new trajectory")
            self.assertTrue(any(
                not stationary(msg) and msg.pos_pts[-1].y < -0.1
                for msg in after_jump
            ), "new local trajectory did not follow the shifted map-frame route")

            # Lose TF while moving: retain briefly, then stop and recover.
            before_timeout = max(msg.traj_id for msg in received)
            pump(0.95)
            self.assertTrue(any(
                stationary(msg) and msg.traj_id > before_timeout for msg in received
            ), "active-trajectory TF timeout did not stop")
            pump(0.8, tf_y=0.3)
            resumed = [msg for msg in received if not stationary(msg)]
            self.assertGreater(max(msg.traj_id for msg in resumed), before_timeout)

            # Move the map path wholly outside the local world grid. Replanning
            # after the jump must fail closed, publishing a stop trajectory.
            before_failure = max(msg.traj_id for msg in received)
            pump(0.35, tf_y=10.0)
            self.assertTrue(any(
                stationary(msg) and msg.traj_id > before_failure for msg in received
            ), "failed jump replan continued to execute the old trajectory")
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
