"""Shared terminal-goal identity for tracking and velocity arbitration."""

import math

from tf_transformations import euler_from_quaternion


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def goal_yaw(path, mode='path'):
    if path is None or not path.poses:
        return None
    goal = path.poses[-1].pose
    q = goal.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    if mode == 'path' and math.sqrt(sum(v * v for v in quaternion)) > 1e-6:
        return euler_from_quaternion(quaternion)[2]
    for previous in reversed(path.poses[:-1]):
        dx = goal.position.x - previous.pose.position.x
        dy = goal.position.y - previous.pose.position.y
        if math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
    return None


def goal_identity(path, mode='path'):
    if path is None or not path.poses:
        return None
    point = path.poses[-1].pose.position
    return (path.header.frame_id, point.x, point.y, goal_yaw(path, mode))


def same_goal(first, second):
    if first is None or second is None:
        return False
    frame, x, y, yaw = first
    other_frame, other_x, other_y, other_yaw = second
    same_yaw = ((yaw is None and other_yaw is None) or
                (yaw is not None and other_yaw is not None and
                 abs(normalize_angle(other_yaw - yaw)) <= 1e-4))
    return (frame == other_frame and math.hypot(other_x - x, other_y - y) <= 1e-4
            and same_yaw)


def valid_path(path, frame='map'):
    if path is None or not path.poses or path.header.frame_id != frame:
        return False
    for stamped in path.poses:
        if stamped.header.frame_id and stamped.header.frame_id != frame:
            return False
        p, q = stamped.pose.position, stamped.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return False
    return True
