#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import SetBool
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from path_follow.goal_utils import goal_identity, goal_yaw, same_goal


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ==============================
        # 参数
        # ==============================
        self.path_topic = self.declare_parameter('path_topic', '/global_path').value
        self.odom_topic = self.declare_parameter('odom_topic', '/loc/robo/odom').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/nav_cmd').value
        self.enabled = self.declare_parameter('enabled', True).value

        # 最大线速度
        self.max_linear_vel = self.declare_parameter('max_linear_vel', 0.5).value

        # 最大角速度
        self.max_angular_vel = self.declare_parameter('max_angular_vel', 0.5).value

        # 基础线速度
        self.linear_vel = self.declare_parameter('linear_vel', 0.5).value

        # 前视距离
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 0.8).value

        # 角速度比例系数
        self.angular_kp = self.declare_parameter('angular_kp', 1.5).value

        # 到达终点判定距离
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.2).value

        # 开始终点姿态调整的距离，小于最终位置容差，预留转向漂移余量
        self.goal_align_distance = self.declare_parameter('goal_align_distance', min(0.12, self.goal_tolerance)).value
        if not (0.0 < self.goal_align_distance <= self.goal_tolerance):
            raise ValueError('Require 0 < goal_align_distance <= goal_tolerance')

        # 终点姿态判定误差（rad）
        self.goal_yaw_tolerance = self.declare_parameter('goal_yaw_tolerance', 0.15).value

        # 终点姿态来源：path 表示使用路径最后一个姿态，segment 表示使用最后一段路径方向
        self.goal_yaw_mode = self.declare_parameter('goal_yaw_mode', 'path').value

        # 大角度误差时是否降低速度
        self.slow_down_angle = self.declare_parameter('slow_down_angle', 0.6).value
        self.rotate_in_place_angle = self.declare_parameter('rotate_in_place_angle', math.pi / 3.0).value
        self.goal_slowdown_distance = self.declare_parameter('goal_slowdown_distance', 0.8).value
        self.goal_exit_tolerance = self.declare_parameter('goal_exit_tolerance', max(0.35, self.goal_tolerance + 0.15)).value
        if not (0.0 <= self.slow_down_angle < self.rotate_in_place_angle <= math.pi / 2.0):
            raise ValueError('Require 0 <= slow_down_angle < rotate_in_place_angle <= pi/2')
        if not (0.0 < self.goal_tolerance < self.goal_exit_tolerance < float('inf')):
            raise ValueError('Require 0 < goal_tolerance < goal_exit_tolerance < infinity')
        if not (self.goal_tolerance < self.goal_slowdown_distance < float('inf')):
            raise ValueError('goal_slowdown_distance must be finite and greater than goal_tolerance')

        # 控制频率
        self.control_frequency = self.declare_parameter('control_frequency', 20.0).value

        # 诊断日志：默认每 0.5 秒记录一次，控制分支变化时立即记录
        self.diagnostic_enabled = self.declare_parameter('diagnostic_enabled', True).value
        self.diagnostic_interval = self.declare_parameter('diagnostic_interval', 0.5).value
        if not math.isfinite(self.diagnostic_interval) or self.diagnostic_interval < 0.0:
            raise ValueError('diagnostic_interval must be finite and non-negative')

        # ==============================
        # 数据
        # ==============================
        self.path = None
        self.control_state = 'tracking'
        self.active_goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.last_warn_time = 0.0
        self.last_goal_info_time = 0.0
        self.last_diagnostic_time = None
        self.last_diagnostic_state = None
        self.last_odom_time = None
        self.odom_frame = ''
        self.odom_child_frame = ''

        # ==============================
        # Subscriber
        # ==============================
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, path_qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.enable_service = self.create_service(SetBool, '~/set_enabled', self.set_enabled_callback)

        # ==============================
        # Publisher
        # ==============================
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ==============================
        # Timer
        # ==============================
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_callback)

        self.get_logger().info('Path follower initialized.')
        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Odom topic: {self.odom_topic}')
        self.get_logger().info(f'Cmd vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'Control parameters: linear_vel={self.linear_vel}, '
            f'max_linear_vel={self.max_linear_vel}, max_angular_vel={self.max_angular_vel}, '
            f'angular_kp={self.angular_kp}, lookahead_distance={self.lookahead_distance}, '
            f'goal_tolerance={self.goal_tolerance}, '
            f'goal_align_distance={self.goal_align_distance}, '
            f'goal_yaw_tolerance_deg={math.degrees(self.goal_yaw_tolerance):.2f}, '
            f'goal_yaw_mode={self.goal_yaw_mode}, '
            f'slow_down_angle_deg={math.degrees(self.slow_down_angle):.2f}, '
            f'rotate_in_place_angle_deg={math.degrees(self.rotate_in_place_angle):.2f}, '
            f'goal_slowdown_distance={self.goal_slowdown_distance}, '
            f'goal_exit_tolerance={self.goal_exit_tolerance}, '
            f'control_frequency={self.control_frequency}, '
            f'diagnostic_enabled={self.diagnostic_enabled}, '
            f'diagnostic_interval={self.diagnostic_interval}')

    def path_callback(self, msg):
        """
        路径回调
        """
        if len(msg.poses) == 0:
            self.path = None
            self.active_goal = None
            self.control_state = 'tracking'
            if time.monotonic() - self.last_warn_time > 2.0:
                self.get_logger().warning('Received empty path.')
                self.last_warn_time = time.monotonic()
            return

        new_goal = goal_identity(msg, self.goal_yaw_mode)
        unchanged = same_goal(self.active_goal, new_goal)
        self.path = msg
        if not unchanged:
            self.active_goal = new_goal
            self.control_state = 'tracking'
            self.get_logger().info('New goal received; reset terminal control state.')

    def set_enabled_callback(self, request, response):
        """Enable arbitration with an acknowledgement of the current goal identity."""
        if request.data != self.enabled:
            self.enabled = request.data
            if self.enabled:
                self.control_state = 'tracking'
            else:
                self.stop_robot('disabled')
        response.success = True
        response.message = json.dumps(self.active_goal)
        return response

    def odom_callback(self, msg):
        """
        里程计回调
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.robot_yaw = yaw
        self.odom_received = True
        self.last_odom_time = time.monotonic()
        self.odom_frame = msg.header.frame_id
        self.odom_child_frame = msg.child_frame_id

    def normalize_angle(self, angle):
        """
        将角度归一化到 [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def get_distance(self, x1, y1, x2, y2):
        """
        计算二维距离
        """
        return math.hypot(x2 - x1, y2 - y1)

    def find_nearest_point(self):
        """
        找到轨迹中距离机器人最近的点

        返回：
            nearest_index
        """

        if self.path is None:
            return None

        min_distance = float('inf')
        nearest_index = 0

        for i, pose_stamped in enumerate(self.path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            distance = self.get_distance(self.robot_x, self.robot_y, px, py)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index

    def find_lookahead_point(self, nearest_index):
        """
        从最近点向前寻找前视距离 lookahead_distance 对应的目标点
        """

        if self.path is None:
            return None

        path_size = len(self.path.poses)
        if path_size == 0:
            return None

        accumulated_distance = 0.0

        for i in range(nearest_index, path_size - 1):
            current_pose = self.path.poses[i].pose.position
            next_pose = self.path.poses[i + 1].pose.position
            segment_distance = self.get_distance(current_pose.x, current_pose.y, next_pose.x, next_pose.y)
            accumulated_distance += segment_distance

            if accumulated_distance >= self.lookahead_distance:
                return self.path.poses[i + 1].pose.position

        return self.path.poses[-1].pose.position

    def get_goal_yaw(self, path=None):
        """
        获取终点期望航向角。优先使用路径末端姿态，若无有效姿态则使用最后一段路径方向。
        """

        return goal_yaw(self.path if path is None else path, self.goal_yaw_mode)

    def is_goal_reached(self):
        """
        判断是否到达路径终点（位置 + 姿态都满足条件）
        """

        if self.path is None or len(self.path.poses) == 0:
            return False

        goal = self.path.poses[-1].pose.position
        distance_to_goal = self.get_distance(self.robot_x, self.robot_y, goal.x, goal.y)
        if distance_to_goal > self.goal_tolerance:
            return False

        goal_yaw = self.get_goal_yaw()
        if goal_yaw is None:
            return True

        yaw_error = self.normalize_angle(goal_yaw - self.robot_yaw)
        return abs(yaw_error) <= self.goal_yaw_tolerance

    def log_control_diagnostic(self, state, cmd=None, nearest_index=None, target=None):
        """记录控制分支、几何误差和本节点发布的速度；角度单位为度。"""
        if not self.diagnostic_enabled:
            return

        now = time.monotonic()
        if (state == self.last_diagnostic_state and self.last_diagnostic_time is not None
                and now - self.last_diagnostic_time < self.diagnostic_interval):
            return
        self.last_diagnostic_time = now
        self.last_diagnostic_state = state

        def number(value):
            return 'NA' if value is None else f'{value:.4f}'

        goal_distance = goal_yaw = goal_yaw_error = goal_bearing_error = None
        target_yaw_error = None
        goal_xy = target_xy = 'NA'
        path_frame = 'NA'
        if self.path is not None and self.path.poses:
            path_frame = self.path.header.frame_id
            goal = self.path.poses[-1].pose.position
            goal_xy = f'({goal.x:.4f},{goal.y:.4f})'
            if self.odom_received:
                goal_distance = self.get_distance(self.robot_x, self.robot_y, goal.x, goal.y)
                if goal_distance > 1e-6:
                    goal_bearing_error = math.degrees(self.normalize_angle(
                        math.atan2(goal.y - self.robot_y, goal.x - self.robot_x)
                        - self.robot_yaw))
                goal_yaw = self.get_goal_yaw()
                if goal_yaw is not None:
                    goal_yaw_error = math.degrees(self.normalize_angle(goal_yaw - self.robot_yaw))
                if nearest_index is None:
                    nearest_index = self.find_nearest_point()
                if target is None and nearest_index is not None:
                    target = self.find_lookahead_point(nearest_index)
                if target is not None:
                    target_xy = f'({target.x:.4f},{target.y:.4f})'
                    dx, dy = target.x - self.robot_x, target.y - self.robot_y
                    if math.hypot(dx, dy) > 1e-6:
                        target_yaw_error = math.degrees(self.normalize_angle(
                            math.atan2(dy, dx) - self.robot_yaw))

        robot_pose = (f'({self.robot_x:.4f},{self.robot_y:.4f},'
                      f'{math.degrees(self.robot_yaw):.2f}deg)' if self.odom_received else 'NA')
        odom_age = None if self.last_odom_time is None else now - self.last_odom_time
        self.get_logger().info(
            f'[PF_DIAG] state={state} robot={robot_pose} goal={goal_xy} '
            f'goal_dist_m={number(goal_distance)} '
            f'goal_bearing_error_deg={number(goal_bearing_error)} '
            f'goal_yaw_deg={number(None if goal_yaw is None else math.degrees(goal_yaw))} '
            f'goal_yaw_error_deg={number(goal_yaw_error)} '
            f'nearest={nearest_index} target={target_xy} '
            f'target_yaw_error_deg={number(target_yaw_error)} '
            f'cmd_v_mps={number(None if cmd is None else cmd.linear.x)} '
            f'cmd_w_radps={number(None if cmd is None else cmd.angular.z)} '
            f'path_frame={path_frame!r} odom_frame={self.odom_frame!r} '
            f'odom_child_frame={self.odom_child_frame!r} odom_age_s={number(odom_age)}')

    def stop_robot(self, state='stopped'):
        """
        停止机器人
        """

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.log_control_diagnostic(state, cmd)

    def control_callback(self):
        """
        主控制循环
        """

        if not self.enabled:
            self.log_control_diagnostic('disabled')
            return

        if not self.odom_received:
            self.log_control_diagnostic('waiting_odom')
            return

        if self.path is None:
            self.stop_robot('no_path')
            return

        if len(self.path.poses) == 0:
            self.stop_robot('empty_path')
            return

        if self.control_state == 'goal_reached':
            self.stop_robot('goal_reached')
            return

        goal_distance = self.get_distance(
            self.robot_x,
            self.robot_y,
            self.path.poses[-1].pose.position.x,
            self.path.poses[-1].pose.position.y,
        )
        goal_yaw = self.get_goal_yaw()

        if self.is_goal_reached():
            self.control_state = 'goal_reached'
            self.get_logger().info('Goal reached; holding stop until a new goal arrives.')
            self.stop_robot('goal_reached')
            return

        if goal_distance <= self.goal_align_distance and goal_yaw is not None:
            self.control_state = 'align_goal_yaw'

        if self.control_state == 'align_goal_yaw':
            yaw_error = self.normalize_angle(goal_yaw - self.robot_yaw)
            # 小幅位置漂移继续原地调姿态；明显偏离或姿态已对齐时恢复位置。
            if (goal_distance > self.goal_exit_tolerance or
                    abs(yaw_error) <= self.goal_yaw_tolerance):
                self.control_state = 'recover_goal_position'
            else:
                angular_vel = self.angular_kp * yaw_error
                angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))

                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd)
                self.log_control_diagnostic('align_goal_yaw', cmd)
                return

        nearest_index = self.find_nearest_point()
        if nearest_index is None:
            self.stop_robot('no_nearest_point')
            return

        target = (self.path.poses[-1].pose.position
                  if self.control_state == 'recover_goal_position'
                  else self.find_lookahead_point(nearest_index))
        if target is None:
            self.stop_robot('no_target')
            return

        dx = target.x - self.robot_x
        dy = target.y - self.robot_y
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.robot_yaw)

        angular_vel = self.angular_kp * yaw_error
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))

        # 终点附近减速；大方位误差先原地转向，消除约 90 度时的绕圈平衡。
        distance_scale = min(1.0, goal_distance / self.goal_slowdown_distance)
        angle_scale = 1.0
        if abs(yaw_error) >= self.rotate_in_place_angle:
            angle_scale = 0.0
        elif abs(yaw_error) > self.slow_down_angle:
            ratio = ((abs(yaw_error) - self.slow_down_angle) /
                     (self.rotate_in_place_angle - self.slow_down_angle))
            angle_scale = math.cos(ratio * math.pi / 2.0)
        linear_vel = max(0.0, min(self.linear_vel, self.max_linear_vel)) * distance_scale * angle_scale

        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_vel

        self.cmd_vel_pub.publish(cmd)
        self.log_control_diagnostic(self.control_state, cmd, nearest_index, target)

        self.get_logger().debug(
        f'nearest={nearest_index}, target=({target.x}, {target.y}),'
        f'yaw_error={yaw_error}, v={linear_vel}, w={angular_vel}')


def main(args=None):
    rclpy.init(args=args)
    controller = PathFollower()
    try:
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            controller.stop_robot()
        except RuntimeError:
            # When the process is asked to stop (SIGINT/SIGTERM) the rclpy context
            # is already invalid, so the zero velocity command cannot be sent.
            pass
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
