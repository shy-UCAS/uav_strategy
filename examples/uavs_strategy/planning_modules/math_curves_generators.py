import json
import matplotlib.pyplot as plt
import numpy as np
import random
from scipy.interpolate import CubicSpline


def interpolate_z_coordinates(trajectory):
    """
        高度插值，并填充轨迹长度
        输入：
            trajectory: 三维点的列表，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
        返回：
            高度插值后的轨迹，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
    """
    # 确保轨迹是列表格式
    trajectory = [list(point) if isinstance(point, (list, tuple)) else point
                  for point in trajectory]

    # 收集已知z值的点
    known_z = []
    for i, point in enumerate(trajectory):
        if len(point) >= 3 and point[2] is not None:
            known_z.append((i, point[2]))

    # 处理特殊情况
    if len(trajectory) <= 1:
        return trajectory

    if len(known_z) == 0:
        # 如果没有已知的z值，使用默认值
        print("Warning: No known z coordinates, using default value 100")
        return [list(point[:2]) + [100.0] if len(point) == 2 else point
                for point in trajectory]

    if len(known_z) == 1:
        # 只有一个已知z值，所有点使用相同高度
        z_value = known_z[0][1]
        return [list(point[:2]) + [z_value] if len(point) == 2 else point
                for point in trajectory]

    # 创建完整的轨迹副本
    completed = [list(point) for point in trajectory]

    # 分段插值
    for seg_idx in range(len(known_z) - 1):
        start_idx, start_z = known_z[seg_idx]
        end_idx, end_z = known_z[seg_idx + 1]

        # 插值中间点
        for i in range(start_idx, end_idx + 1):
            if len(completed[i]) == 2:
                # 线性插值
                if end_idx > start_idx:
                    ratio = (i - start_idx) / (end_idx - start_idx)
                else:
                    ratio = 0
                z = start_z + (end_z - start_z) * ratio
                completed[i] = completed[i] + [z]
            elif len(completed[i]) >= 3 and completed[i][2] is None:
                # 替换None值
                if end_idx > start_idx:
                    ratio = (i - start_idx) / (end_idx - start_idx)
                else:
                    ratio = 0
                completed[i][2] = start_z + (end_z - start_z) * ratio

    return completed


def generate_breakthrough_flight(traj, direction_range=None, num_points=15):
    """
    参数：
        traj: 三维点的列表，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
        direction_range: 偏转角度范围，默认值为 [-15, 15]
        num_points: 内部插值点数，默认值为 5

    返回：
        内部插值后的轨迹，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
    """
    if len(traj) < 2:
        return traj

    startpoint = np.array(traj[0])
    endpoint = np.array(traj[-1])

    # 计算三维位移向量
    displacement = endpoint - startpoint
    total_distance = np.linalg.norm(displacement)

    # 处理重合点
    if total_distance < 1e-6:
        return [startpoint.tolist() for _ in range(num_points)]

    # 计算水平距离
    dx, dy = displacement[0], displacement[1]
    L_horizontal = np.sqrt(dx ** 2 + dy ** 2)

    # 确定偏转角度范围
    if direction_range is not None:
        theta_min = np.deg2rad(direction_range[0])
        theta_max = np.deg2rad(direction_range[1])
    else:
        theta_min = np.deg2rad(-15)  # 默认小幅偏转
        theta_max = np.deg2rad(15)

    # 修改：确保 t 参数不包含起始点和终止点
    # 对于内部插值，t 应该在 (0, 1) 开区间内
    if num_points <= 2:
        # 如果点数太少，直接返回起始点和终止点
        return [startpoint.tolist(), endpoint.tolist()]

    # 生成内部点的 t 值，排除 0 和 1
    t_internal = np.linspace(0, 1, num_points)[1:-1]  # 去掉首尾点

    # 随机选取偏转幅度
    theta_peak = np.random.uniform(theta_min, theta_max)

    # 定义偏转幅度曲线：在 t=0,1 时为 0，在 t=0.5 时为 theta_peak
    deflection_magnitude = 4 * theta_peak * t_internal * (1 - t_internal)

    # 判断是垂直方向还是水平方向
    if L_horizontal < 1e-6:
        # 垂直方向：在水平面内偏转
        random_direction = np.random.uniform(0, 2 * np.pi)
        deflection_distance = deflection_magnitude * total_distance * 0.3

        x_offset = deflection_distance * np.cos(random_direction)
        y_offset = deflection_distance * np.sin(random_direction)

        x_internal = startpoint[0] + t_internal * dx + x_offset
        y_internal = startpoint[1] + t_internal * dy + y_offset

    else:
        # 水平方向：基于baseline angle偏转
        baseline_angle = np.arctan2(dy, dx)
        theta_t = deflection_magnitude
        s = t_internal * L_horizontal

        x_internal = startpoint[0] + s * np.cos(baseline_angle + theta_t)
        y_internal = startpoint[1] + s * np.sin(baseline_angle + theta_t)

    # z坐标使用现有的随机生成函数生成内部点
    z_all = generate_zline_random_segment(traj, num_points)
    z_internal = z_all[1:-1]  # 只取内部点的z坐标

    # 组装结果：起始点 + 内部插值点 + 终止点
    segment = [startpoint.tolist()]  # 保持原始起始点

    # 添加内部插值点
    for x, y, z in zip(x_internal, y_internal, z_internal):
        segment.append([float(x), float(y), float(z)])

    segment.append(endpoint.tolist())  # 保持原始终止点

    return segment


def generate_zline_random_segment(traj, num_points=3):
    """
        生成随机的 z 线段，包括直线、下坡、上坡
        参数：
            traj: 三维点的列表，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
            num_points: 生成曲线上点的数量
        返回：
            shape=(num_points, ) 的数组，表示插值曲线上的点

    """
    start = traj[0]
    end = traj[-1]
    up_limit = 8.0
    down_limit = 1.5
    p_ratio = random.uniform(down_limit, up_limit)
    n_ratio = random.uniform(-down_limit, -up_limit)
    t = np.linspace(0, 1, num_points)
    z_straight_values = np.linspace(start[2], end[2], num_points)
    z_dive_values = start[2] + (end[2] - start[2]) * np.exp(p_ratio * (t - 1))
    z_climb_values = start[2] + (end[2] - start[2]) * (1 - np.exp(n_ratio * t))
    return random.choice([z_straight_values, z_dive_values, z_climb_values])


def cubic_interpolation_3d(traj, bc_type='not-a-knot', num_points=15):
    """
    使用三次样条插值拟合三维曲线数据

    参数：
        traj: 三维点的列表，格式为 [[x1, y1, z1], [x2, y2, z2], ...]
        bc_type: 边界条件设置，默认使用'not-a-knot'，可根据需要设置为'periodic'等

    返回：
        三个 CubicSpline 对象，分别对应 x, y, z 的插值函数
    """
    traj = np.array(traj)
    n = traj.shape[0]
    # 使用均匀参数 t 表示数据点
    t = np.linspace(0, 1, n)

    cs_x = CubicSpline(t, traj[:, 0], bc_type=bc_type)
    cs_y = CubicSpline(t, traj[:, 1], bc_type=bc_type)
    cs_z = CubicSpline(t, traj[:, 2], bc_type=bc_type)

    return evaluate_3d_curve(cs_x, cs_y, cs_z, num_points)


def evaluate_3d_curve(cs_x, cs_y, cs_z, num_points=3):
    """
    根据三次样条插值函数计算曲线上对应的三维坐标

    参数：
        cs_x, cs_y, cs_z: CubicSpline 插值函数
        num_points: 生成曲线上点的数量

    返回：
        shape=(num_points, 3) 的数组，表示插值曲线上的点
    """
    t_new = np.linspace(0, 1, num_points)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)
    z_new = cs_z(t_new)
    # 垂直堆叠并转置
    stacked_array = np.vstack((x_new, y_new, z_new)).T

    # 将 ndarray 转换为列表
    stacked_list = stacked_array.tolist()
    return stacked_list
