#!/usr/bin/env python3
"""
SICNAV Controller Module

Cung cấp các hàm để tính toán các lệnh tốc độ dựa trên lực hút và lực đẩy
theo phương pháp trường thế (potential field).
"""

import numpy as np
import math
from geometry_msgs.msg import PoseStamped, TwistStamped

# Biến toàn cục để lưu trữ goal pose và danh sách các waypoint
goal_pose = PoseStamped()
position_all = []


def quaternion_multiply(q, r):
    """
    Nhân hai quaternion.
    
    Args:
        q (array-like): Quaternion đầu tiên [w, x, y, z].
        r (array-like): Quaternion thứ hai [w, x, y, z].

    Returns:
        np.ndarray: Kết quả của phép nhân quaternion.
    """
    return np.array([
        q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
        q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2],
        q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1],
        q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0]
    ])


def rotate_vector_by_quaternion(q, v):
    """
    Quay một vector bằng cách sử dụng quaternion.
    
    Args:
        q (array-like): Quaternion [w, x, y, z].
        v (array-like): Vector dưới dạng quaternion thuần (0, v_x, v_y, 0).

    Returns:
        np.ndarray: Vector sau khi quay (chỉ gồm thành phần x và y).
    """
    # Tính quaternion liên hợp: [q0, -q1, -q2, -q3]
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    rotated = quaternion_multiply(quaternion_multiply(q, v), q_conj)
    return np.array(rotated[1:3])


def get_ranges(occupancy_grid, pose, q_star):
    """
    Tính khoảng cách đến vật cản xung quanh robot dựa trên occupancy grid.
    
    Args:
        occupancy_grid: Thông điệp occupancy grid chứa thông tin bản đồ.
        pose (PoseStamped): Vị trí hiện tại của robot.
        q_star (float): Khoảng cách ngưỡng để xem xét vật cản.

    Returns:
        np.ndarray: Mảng gồm 360 giá trị khoảng cách, mỗi giá trị ứng với một góc (độ).
    """
    origin = occupancy_grid.info.origin.position
    resolution = occupancy_grid.info.resolution
    grid_x = int((pose.pose.position.x - origin.x) / resolution)
    grid_y = int((pose.pose.position.y - origin.y) / resolution)

    min_x = max(0, grid_x - int(q_star))
    min_y = max(0, grid_y - int(q_star))
    max_x = min(occupancy_grid.info.width, grid_x + int(q_star))
    max_y = min(occupancy_grid.info.height, grid_y + int(q_star))

    # Khởi tạo mảng 360 giá trị với giá trị mặc định là vô cực
    ranges = np.full(360, np.inf)
    base_vector = np.array([1, 0])

    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            dx = x - grid_x
            dy = y - grid_y
            distance = np.hypot(dx, dy)
            if 0 < distance < q_star:
                cost = occupancy_grid.data[y * occupancy_grid.info.width + x]
                if cost >= 100:
                    # Tính góc giữa base_vector và vector (dx, dy)
                    cos_angle = np.clip(np.dot(base_vector, [dx, dy]) / distance, -1.0, 1.0)
                    angle_deg = int(np.rad2deg(np.arccos(cos_angle))) % 360
                    measured_distance = distance * resolution
                    if measured_distance < ranges[angle_deg]:
                        ranges[angle_deg] = measured_distance
    return ranges


def compute_repulsive_force(occupancy_grid, pose):
    """
    Tính lực đẩy (repulsive force) từ vật cản dựa trên occupancy grid.
    
    Args:
        occupancy_grid: Thông điệp occupancy grid.
        pose (PoseStamped): Vị trí hiện tại của robot.
    
    Returns:
        np.ndarray: Lực đẩy (2 chiều).
    """
    eta = 4.0      # Hệ số tỉ lệ cho lực đẩy
    q_star = 1.0   # Khoảng cách ngưỡng

    force_sum = np.array([0.0, 0.0])
    count = 0

    ranges = get_ranges(occupancy_grid, pose, q_star)

    for angle_deg, distance in enumerate(ranges):
        if 0 < distance < q_star:
            mag = abs(eta * (1/q_star - 1/distance) * (1/distance**2))
            # Chuyển đổi góc từ độ sang radian
            angle_rad = np.deg2rad(angle_deg)
            force = -mag * np.array([np.cos(angle_rad), np.sin(angle_rad)])
            force_sum += force
            count += 1

    if count > 0:
        return force_sum / count
    return force_sum


def get_closest_waypoint(point, waypoints):
    """
    Tìm waypoint gần nhất với điểm hiện tại.
    
    Args:
        point (list): [x, y] của điểm hiện tại.
        waypoints (list): Danh sách các waypoint dạng [x, y].
    
    Returns:
        tuple: (index, waypoint gần nhất)
    """
    closest_index = 0
    min_dist = math.inf
    closest_wp = None
    for i, wp in enumerate(waypoints):
        dist = math.dist(wp, point)
        if dist < min_dist:
            min_dist = dist
            closest_wp = wp
            closest_index = i
    return closest_index, closest_wp


def compute_attractive_force(pose, goal_pose, waypoints):
    """
    Tính lực hút (attractive force) về phía goal, sử dụng danh sách waypoint để định hướng.
    
    Args:
        pose (PoseStamped): Vị trí hiện tại của robot.
        goal_pose (PoseStamped): Vị trí goal.
        waypoints (list): Danh sách waypoint từ global plan.
    
    Returns:
        np.ndarray: Lực hút (2 chiều).
    """
    current_pos = np.array([pose.pose.position.x, pose.pose.position.y])
    goal_pos = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y])
    zeta = 5.0    # Hệ số tỉ lệ cho lực hút
    d_star = 1.5  # Khoảng cách ngưỡng

    closest_index, _ = get_closest_waypoint(current_pos.tolist(), waypoints)
    dist_to_goal = np.linalg.norm(current_pos - goal_pos)

    # Nếu waypoint gần nhất là waypoint cuối cùng thì dùng goal trực tiếp
    if closest_index == len(waypoints) - 1:
        angle = np.arctan2(goal_pos[1] - current_pos[1], goal_pos[0] - current_pos[0])
        mag = zeta * dist_to_goal if dist_to_goal <= d_star else zeta * d_star
        return mag * np.array([np.cos(angle), np.sin(angle)])

    # Sử dụng lookahead: chọn waypoint cách xa 10 đơn vị nếu có thể
    if closest_index + 10 < len(waypoints) - 1:
        next_wp = waypoints[closest_index + 10]
        angle = np.arctan2(next_wp[1] - current_pos[1], next_wp[0] - current_pos[0])
    else:
        angle = np.arctan2(goal_pos[1] - current_pos[1], goal_pos[0] - current_pos[0])

    mag = zeta * dist_to_goal if dist_to_goal <= d_star else zeta * d_star
    return mag * np.array([np.cos(angle), np.sin(angle)])


def extract_waypoints_from_global_plan(global_plan):
    """
    Trích xuất danh sách waypoint (dạng [x, y]) từ global plan.
    
    Args:
        global_plan: Global plan chứa danh sách PoseStamped.
    
    Returns:
        list: Danh sách waypoint.
    """
    waypoints = []
    for pose_stamped in global_plan.poses:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        waypoints.append([x, y])
    return waypoints


def compute_velocity(obstacle_force, pose, current_twist):
    """
    Tính toán lệnh tốc độ dựa trên lực tổng hợp từ lực đẩy và lực hút.
    
    Args:
        obstacle_force (np.ndarray): Lực tổng hợp (2 chiều).
        pose (PoseStamped): Vị trí hiện tại của robot.
        current_twist: Thông điệp twist hiện tại.
    
    Returns:
        TwistStamped: Lệnh tốc độ mới.
    """
    cmd = TwistStamped()
    cmd.header = pose.header

    min_vel_x = -0.2
    max_vel_x = 0.2
    min_vel_y = -0.2
    max_vel_y = 0.2
    min_ang_z = -1.0
    max_ang_z = 1.0
    drive_scale = 0.03  # Hệ số tỉ lệ cho lực

    if obstacle_force is None:
        return cmd

    vel_x = obstacle_force[0] * drive_scale
    vel_y = obstacle_force[1] * drive_scale

    # Chuyển vector tốc độ theo frame của robot
    q = np.array([
        pose.pose.orientation.w,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z
    ])
    # Biến vector thành quaternion thuần: [0, vel_x, -vel_y, 0]
    v = np.array([0, vel_x, -vel_y, 0])
    f_v = rotate_vector_by_quaternion(q, v)

    mag = np.hypot(f_v[0], f_v[1])
    if mag == 0:
        mag = 1e-6  # Tránh chia cho 0
    cross_prod = np.cross(np.array([1.0, 0.0, 0.0]), np.array([f_v[0], f_v[1], 0.0])) / mag

    mag_twist = abs(current_twist.linear.x) / max_vel_x if max_vel_x != 0 else 0

    cmd.twist.linear.x = np.clip(f_v[0] * abs(1.0 - cross_prod[2]), min_vel_x, max_vel_x)
    cmd.twist.linear.y = np.clip(f_v[1] * abs(1.0 - cross_prod[2]), min_vel_y, max_vel_y)
    cmd.twist.angular.z = np.clip(-cross_prod[2] * abs(1.0 - mag_twist), min_ang_z, max_ang_z)

    return cmd


def compute_velocity_commands(occupancy_grid, pose, current_twist):
    """
    Tính toán lệnh tốc độ bằng cách kết hợp lực đẩy và lực hút.
    
    Args:
        occupancy_grid: Occupancy grid của bản đồ.
        pose (PoseStamped): Vị trí hiện tại của robot.
        current_twist: Lệnh twist hiện tại.
    
    Returns:
        TwistStamped: Lệnh tốc độ mới.
    """
    global goal_pose, position_all
    repulsive = compute_repulsive_force(occupancy_grid, pose)
    attractive = compute_attractive_force(pose, goal_pose, position_all)
    net_force = repulsive + attractive
    return compute_velocity(net_force, pose, current_twist)


def set_path(global_plan):
    """
    Thiết lập global plan: cập nhật goal_pose và danh sách waypoint.
    
    Args:
        global_plan: Global plan chứa danh sách PoseStamped.
    """
    global goal_pose, position_all
    if not global_plan.poses:
        return
    goal_pose = global_plan.poses[-1]
    position_all = extract_waypoints_from_global_plan(global_plan)


def set_speed_limit(speed_limit, is_percentage):
    """
    Đặt giới hạn tốc độ.
    
    Hiện tại là hàm placeholder.
    
    Args:
        speed_limit: Giá trị giới hạn tốc độ.
        is_percentage: Cờ xác định xem speed_limit có là phần trăm hay không.
    """
    pass
