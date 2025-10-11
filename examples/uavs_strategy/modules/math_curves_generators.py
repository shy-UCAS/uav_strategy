# TODO: 将基于数学公式的曲线生成函数放在这里
import copy
import json

import numpy as np
import random

from imagecodecs import NoneError
from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline

import math
from math import comb

from collections import defaultdict
from shapely.geometry import Polygon, LineString, Point
from shapely.affinity import rotate

import matplotlib.pyplot as plt

from modules import single_basic_behaviors as sbb
from modules import global_configs as gcfg
from modules import formation_generator as fg

from thirdparty.RRT import rrt

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']  # 设置中文字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示异常

class ReconSCurveGenerator:
    """
    生成S型侦察轨迹
    """
    def __init__(self, traj, area, recon_radius=50, vis_check=False):
        self._trajectory = traj
        self._area = area
        self._search_step = recon_radius * 2
        
        # 进行一些通用参数的初始化
        self._traj_start = self._trajectory[0]
        self._traj_end = self._trajectory[-1]
        
        self._traj_length = np.sqrt((self._traj_end[0] - self._traj_start[0]) ** 2 + (self._traj_end[1] - self._traj_start[1]) ** 2)
        self._traj_ux = (self._traj_end[0] - self._traj_start[0]) / self._traj_length
        self._traj_uy = (self._traj_end[1] - self._traj_start[1]) / self._traj_length

        self._traj_line = LineString([Point(self._traj_start), Point(self._traj_end)])
        self._area_poly = Polygon(self._area)
        
        self.vis_check = vis_check

    def _parse_intersect_points(self, res_intersects):
        """
        计算交点数量
        :param res_intersects: 交点数组
        :return: 交点数量
        """
        if res_intersects.is_empty:
            return []
        elif res_intersects.geom_type == 'Point':
            return [res_intersects.coords[0]]
        else:
            return [_point.coords[0] for _point in res_intersects.geoms]

    def _traj2area_intersects(self, vis_check=False):
        """
        判断轨迹是否与指定区域相交
        :param traj: 轨迹点列表
        :param area: 区域列表
        :return: 交点数组（轨迹和多边形区域的交点）[start_point, end_point]
        """
        if self._traj_length <= 0:
            return []
        
        startpoint = self._traj_start
        endpoint = self._traj_end
        
        # 1. 创建原始线段和区域
        _line = self._traj_line
        _area_poly = self._area_poly
        
        # 2. 计算多边形的对角线长度，用于确定延长距离
        _area_bounds = _area_poly.bounds
        _area_diag_len = np.sqrt((_area_bounds[2] - _area_bounds[0]) ** 2 + (_area_bounds[3] - _area_bounds[1]) ** 2)
        
        # 3. 计算方向向量        
        _ux = self._traj_ux
        _uy = self._traj_uy
        
        # 4. 获取和当前vector向量最近的area多边形的边缘点
        _end2vertices_dists = np.linalg.norm(np.array(endpoint).reshape(1, -1) - np.array(self._area).reshape(-1, 2), axis=1)
        _end2nearest_vertex_dist = _end2vertices_dists[np.argmin(_end2vertices_dists)]

        # 5. 计算延长距离
        _extend_dist = _area_diag_len * 2 + _end2nearest_vertex_dist
        _extended_end = np.array(endpoint) + _extend_dist * np.array([_ux, _uy])
        _extended_line = LineString([Point(startpoint), Point(_extended_end)])

        # 6. 计算原始的交点和延长的交点
        _orig_intersections = _line.intersection(_area_poly.boundary)
        _orig_inter_points = self._parse_intersect_points(_orig_intersections)
        _orig_num_inters = len(_orig_inter_points)
        
        _extended_intersections = _extended_line.intersection(_area_poly.boundary)
        _ext_inter_points = self._parse_intersect_points(_extended_intersections)
        _ext_num_inters = len(_ext_inter_points)

        # 7. 根据原始和延长线的交点数量，分析开始与结束搜索的点
        _intersect_points = []
        _intersect_mode = 'unknown'
        if _ext_num_inters <= 0:
            # 如果原始轨迹和区域没有交点，则返回空
            _intersect_points = []
            _intersect_mode = 'none'
        elif _ext_num_inters == 1:
            # 则开始和结束搜索的都是同一个点
            _intersect_points = [_ext_inter_points[0], _ext_inter_points[0]]
            _intersect_mode = 'same_point'
        else: # _ext_num_inters >= 2
            # 如果延长线有交点，则开始搜索的点为延长线交点中离原点最近的点，结束搜索的点为离终点最近的点
            if _orig_num_inters == 0:
                # 如果原始轨迹和区域没有交点，则开始、结束搜索的点为延长线交点中离原点最近的点
                _nearest_inter_point_index = np.argmin(np.linalg.norm(np.array(_ext_inter_points).reshape(-1, 2) - np.array(startpoint), axis=1))
                _start_search_point = _ext_inter_points[int(_nearest_inter_point_index)]
                _intersect_points = [_start_search_point, _start_search_point]
                _intersect_mode = 'same_point'
            elif _orig_num_inters == 1:
                # 如果原始轨迹和区域有1个交点，则开始、结束搜索的点为该交点
                _intersect_points = [_orig_inter_points[0], _orig_inter_points[0]]
                _intersect_mode = 'same_point'
            else: # _orig_num_inters == 2
                # 如果原始轨迹和区域有2个或两个以上交点，则开始点为最近的交点，结束点为最远的交点
                _nearest_inter_point_index = np.argmin(np.linalg.norm(np.array(_ext_inter_points).reshape(-1, 2) - np.array(startpoint), axis=1))
                _farest_inter_point_index = np.argmax(np.linalg.norm(np.array(_ext_inter_points).reshape(-1, 2) - np.array(startpoint), axis=1))
                
                _intersect_points = [_ext_inter_points[int(_nearest_inter_point_index)], 
                                    _ext_inter_points[int(_farest_inter_point_index)]]
                _intersect_mode = 'diff_point'
        
        if vis_check:
            fig, ax = plt.subplots(1, 1, figsize=(10, 10))
            
            ax.plot(_extended_line.xy[0], _extended_line.xy[1], color='red', linestyle='--')
            ax.plot(_line.xy[0], _line.xy[1], color='blue')
            ax.plot(_area_poly.exterior.xy[0], _area_poly.exterior.xy[1], color='green')
            
            for _inter_point in _ext_inter_points:
                ax.plot(_inter_point[0], _inter_point[1], 'ro')
            
            for _inter_point in _orig_inter_points:
                ax.plot(_inter_point[0], _inter_point[1], 'go')
            
            ax.scatter(startpoint[0], startpoint[1], color='red', edgecolors='black', marker='s')
            ax.scatter(endpoint[0], endpoint[1], color='red', edgecolors='black', marker='o')
                
            ax.set_title('Num orig-inters: {}, Num ext-inters: {}'.format(_orig_num_inters, _ext_num_inters))
            ax.set_aspect('equal')
            plt.show()

        return _intersect_points, _intersect_mode
    
    def _rand_rotate_vector(self, vector, angle_range):
        """
        将向量逆时针旋转angle度
        """
        theta_rad = math.radians(angle_range)
        angle = random.uniform(-theta_rad, theta_rad)  # 随机生成偏转角度
        
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        x_new = vector[0] * cos_a - vector[1] * sin_a
        y_new = vector[0] * sin_a + vector[1] * cos_a
        
        return np.array([x_new, y_new])
            
    def _build_area_rasters(self, start_stop_points, deviate_angle=30, mode='return', vis_check=False):
        """
        生成覆盖搜索区域的栅格，栅格方向为飞机飞行方向叠加一个小的偏离theta_angle
        """
        _recon_direct_vector = self._rand_rotate_vector([self._traj_ux, self._traj_uy], deviate_angle)
        
        # 根据飞行方向旋转侦察区域，并由此得到bounding box用于生成侦察搜索的栅格
        _area_centroid = self._area_poly.centroid
        _area_rotate_angle = 90 - math.degrees(math.atan2(_recon_direct_vector[1], _recon_direct_vector[0]))
        _area_rotated = rotate(self._area_poly, angle=_area_rotate_angle, origin=_area_centroid, use_radians=False)
        _area_rotated_bbox = _area_rotated.bounds
        
        # 根据搜索间隔，生成搜索区域航线
        _search_rect_height = _area_rotated_bbox[3] - _area_rotated_bbox[1]
        if _search_rect_height <= self._search_step:
            # 如果搜索区域的宽度小于等于搜索无人机观测直径
            _interm_heights = [np.mean([_area_rotated_bbox[1], _area_rotated_bbox[3]])]
        else:
            # 否则，根据搜索间隔生成搜索栅格
            _lowest_search_height = _area_rotated_bbox[1] + self._search_step * 0.6
            _highest_search_height = _area_rotated_bbox[3] - self._search_step * 0.6
            
            _num_interm_heights = int(np.ceil(((_highest_search_height - _lowest_search_height) / (self._search_step))))
            _interm_heights = np.linspace(_lowest_search_height, _highest_search_height, _num_interm_heights + 2)
            
        # 根据与area polygon的交点，裁剪intermediate search raster line长度
        _raster2poly_lines = []
        _raster2poly_interptrs = []
        
        for _height in _interm_heights:
            _raster2poly_lines.append([(_area_rotated_bbox[0], _height), (_area_rotated_bbox[2], _height)])
            
            _cur_raster_line = LineString([Point(_area_rotated_bbox[0], _height), Point(_area_rotated_bbox[2], _height)])
            
            _cur_r2p_intersects = _cur_raster_line.intersection(_area_rotated.boundary)
            _cur_r2p_interptrs = self._parse_intersect_points(_cur_r2p_intersects)
            
            _intersect_x_min = np.min([_inter_point[0] for _inter_point in _cur_r2p_interptrs])
            _intersect_x_max = np.max([_inter_point[0] for _inter_point in _cur_r2p_interptrs])

            _raster2poly_interptrs.append([[_intersect_x_min, _height], [_intersect_x_max, _height]])
        
        if mode == 'through':
            _lines_iter_orders = np.arange(len(_raster2poly_lines))
            
        else: # mode == 'return'
            _forward_lines_indexes = np.sort(np.random.choice(len(_raster2poly_lines), int(len(_raster2poly_lines) / 2), replace=False))
            _backward_lines_indexes = np.sort(np.setdiff1d(np.arange(len(_raster2poly_lines)), _forward_lines_indexes))
            
            _lines_iter_orders = np.concatenate([_forward_lines_indexes, _backward_lines_indexes])
        
        # 遍历raster line，连接生成侦察轨迹
        _recon_traj_intermediate = []
        _cur_connect_side = np.random.choice(['left', 'right'])
        
        for _line_iter in _lines_iter_orders:
            _cur_line = _raster2poly_lines[_line_iter]
            
            if _cur_connect_side == 'left':
                _cur_connect_point = _raster2poly_interptrs[_line_iter][0]
                _nxt_connect_point = _raster2poly_interptrs[_line_iter][1]
                
                _cur_connect_side = 'right'
            else:
                _cur_connect_point = _raster2poly_interptrs[_line_iter][1]
                _nxt_connect_point = _raster2poly_interptrs[_line_iter][0]
                
                _cur_connect_side = 'left'

            _recon_traj_intermediate.append(_cur_connect_point)
            _recon_traj_intermediate.append(_nxt_connect_point)
        
        # 然后将所有的点转回到之前的角度上
        _rotated_recon_traj = LineString([Point(_point) for _point in _recon_traj_intermediate])
        _recon_traj = rotate(_rotated_recon_traj, angle=-_area_rotate_angle, origin=_area_centroid, use_radians=False)
        
        # import pdb; pdb.set_trace()
        _recon_traj = [start_stop_points[0]] + list(_recon_traj.coords) + [start_stop_points[1]]
        
        if vis_check:
            fig, axs = plt.subplots(1, 2, figsize=(18, 10))
            
            for _iter in range(2):
                axs[_iter].arrow(self._traj_start[0], self._traj_start[1], 
                            self._traj_end[0] - self._traj_start[0], 
                            self._traj_end[1] - self._traj_start[1], 
                            head_width=15, head_length=20, fc='red', ec='red', alpha=0.7)
                
                axs[_iter].arrow(self._traj_start[0], self._traj_start[1], 
                                _recon_direct_vector[0] * self._traj_length, 
                                _recon_direct_vector[1] * self._traj_length, 
                                head_width=15, head_length=20, fc='blue', ec='blue', alpha=0.7)
            
                axs[_iter].plot(self._area_poly.exterior.xy[0], self._area_poly.exterior.xy[1], color='lightgreen')
                
            axs[0].plot(_area_rotated.exterior.xy[0], _area_rotated.exterior.xy[1], color='green', linestyle='--')
            axs[0].plot([_area_rotated_bbox[_iter] for _iter in [0, 2, 2, 0, 0]], [_area_rotated_bbox[_iter] for _iter in [1, 1, 3, 3, 1]], color='green', linestyle='--')
            
            # for _inter_line, _inter_ptrs in zip(_raster2poly_lines, _raster2poly_interptrs):
            #     ax.plot([_inter_line[0][0], _inter_line[1][0]], [_inter_line[0][1], _inter_line[1][1]], color='blue', linestyle='--', alpha=0.3)
            #     ax.scatter([_inter_ptrs[0][0], _inter_ptrs[1][0]], [_inter_ptrs[0][1], _inter_ptrs[1][1]], color='red', marker='*', alpha=0.5)
            
            axs[0].plot([_ptr[0] for _ptr in _recon_traj_intermediate], [_ptr[1] for _ptr in _recon_traj_intermediate], color='blue', linestyle='--', alpha=0.5)
            axs[1].plot([_ptr[0] for _ptr in _recon_traj], [_ptr[1] for _ptr in _recon_traj], color='blue', linestyle='-.', alpha=0.5)
            
            axs[0].set_aspect('equal')
            plt.show()
        
        return _recon_traj
    
    def generate(self):
        """
        生成S型侦察轨迹
        :return: S型侦察轨迹
        """
        _traj2area_intersects, _traj2area_intermode = self._traj2area_intersects(vis_check=False)
        if len(_traj2area_intersects) == 0:
            return []

        # 然后根据分析得到的开始与结束点，以及区域的大小，构建搜索S轨迹
        _trajectory = self._build_area_rasters(_traj2area_intersects, 
                                               mode="through" if _traj2area_intermode == "diff_point" else "return",
                                               vis_check=True)
        
        return _trajectory

class FastpassCurveGenerator:
    def __init__(self,traj, obstacles=None, area=None, avoid_radius=50, step_len=10, vis_check=True):
        self.traj = traj
        
        self.area = area
        
        self.obstacles = obstacles
        self.avoid_radius = avoid_radius
        
        self.obs_array = self._build_obstacles_graph(obstacles, avoid_radius) if obstacles is not None else None
        
        self.step_len = step_len
        self.vis_check = vis_check
    
    def _build_obstacles_graph(self, obstacles, obs_radius):
        _obstacles_array = []
        
        for _obs_pos in obstacles:
            _obstacles_array.append([_obs_pos[0], _obs_pos[1], obs_radius])

        return np.array(_obstacles_array)

    def generate(self):
        """
        生成快速通过的飞行轨迹轨迹
        :return: 快速通过飞行轨迹
        """
        _router = rrt.SingleRRT(2, np.array(self.traj[0]), np.array(self.traj[-1]), self.obs_array, stepsize=self.step_len)
        _router.run()
        
        if self.vis_check:
            _router.plot()
        
        return _router.path

def cubic_interpolation_3d(traj, bc_type='not-a-knot', num_points=3):
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


def n_order_bezier(points, t):
    """
    计算n阶贝塞尔曲线上某一点的坐标
    :param points: 控制点列表（包含起点、终点和中间点）
    :param t: 插值参数（0 ≤ t ≤ 1）
    :return: 插值点坐标 [x,y,z]
    """
    n = len(points) - 1  # 阶数 = 控制点数 -1
    result = np.zeros(3)
    for i in range(n + 1):
        # Bernstein多项式系数计算[5,8](@ref)
        coeff = comb(n, i) * (1 - t) ** (n - i) * t ** i
        result += coeff * np.array(points[i])
    return result.tolist()


def bezier_trajectory(trajectory, num_points=3):
    """
    贝塞尔曲线轨迹生成函数
    :param trajectory: 轨迹点列表 [[x,y,z], ...]
    :param num_points: 输出轨迹点数
    :return: 插值后的轨迹列表
    """
    if len(trajectory) < 2:
        return trajectory  # 无需插值

    # 分段策略判断
    if len(trajectory) <= 4:
        # 单次贝塞尔插值[4](@ref)
        t_vals = np.linspace(0, 1, num_points)
        return [n_order_bezier(trajectory, t) for t in t_vals]
    else:
        # 分段三次贝塞尔曲线[7](@ref)
        segments = []
        for i in range(0, len(trajectory) - 3, 3):
            # 每4个点作为一段（含前一段的终点）
            seg_points = trajectory[i:i + 4]
            t_vals = np.linspace(0, 1, num_points // (len(trajectory) // 3))
            seg_curve = [n_order_bezier(seg_points, t) for t in t_vals]
            segments.extend(seg_curve[:-1])  # 避免重复端点
        return segments


def generate_3d_climb_segment(traj, ratio=-4.5, num_points=3):
    """指数曲线爬升"""
    ratio = random.uniform(-8.5, -4.5)
    start = traj[0]
    end = traj[-1]
    t = np.linspace(0, 1, num_points)
    # 计算 x, y, z 坐标
    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)
    z_values = start[2] + (end[2] - start[2]) * (1 - np.exp(ratio * t))

    # 将坐标合并成列表的每一个元素为 [x, y, z]
    segment = [[x, y, z] for x, y, z in zip(x_values, y_values, z_values)]

    return segment


def generate_3d_dive_segment(traj, ratio=None, num_points=3):
    """指数曲线俯冲"""
    ratio = random.uniform(4.5, 8.5) if ratio is None else ratio
    start = traj[0]
    end = traj[-1]
    t = np.linspace(0, 1, num_points)
    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)
    z_values = start[2] + (end[2] - start[2]) * np.exp(ratio * (t - 1))
    segment = [[x, y, z] for x, y, z in zip(x_values, y_values, z_values)]

    return segment


def generate_3d_straight_segment(traj, num_points=3):
    """直线段"""
    start = traj[0]
    end = traj[-1]
    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)
    z_values = np.linspace(start[2], end[2], num_points)
    segment = [[x, y, z] for x, y, z in zip(x_values, y_values, z_values)]
    return segment


def generate_zline_random_segment(traj, num_points=3):
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


def generate_breakthrough_flight(traj, direction_range=None, num_points=3):
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
        segment.append([x, y, z])

    segment.append(endpoint.tolist())  # 保持原始终止点

    return segment


def generate_random_ratio(upper_limit, down_limit):
    return random.uniform(down_limit, upper_limit)


def generate_random_trajectory(traj_type, traj, num_points=3):
    if traj_type == 'breakthrough':
        method = random.choice([generate_3d_dive_segment, generate_3d_straight_segment, generate_3d_climb_segment])
        # 使用选择的函数生成轨迹
        trajectory = method(traj, num_points=num_points)
        return trajectory
    else:
        method = random.choice([generate_3d_dive_segment, generate_3d_straight_segment, generate_3d_climb_segment])
        # 使用选择的函数生成轨迹
        trajectory = method(traj, num_points=num_points)
        return trajectory

def decorative_trajectory(traj, offset_amplitude=50.0, num_waves=3, eps=1e-8):
    # 提取 x, y, z
    xy_values = np.array([[p[0], p[1]] for p in traj])
    z_values  = np.array([p[2] for p in traj])

    # 计算切向量并归一化
    dx = np.gradient(xy_values[:,0])
    dy = np.gradient(xy_values[:,1])
    tangents = np.column_stack((dx, dy))
    norm_t = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangents_unit = tangents / (norm_t + eps)   # 在分母加 eps

    # 其余与原来一致
    normals_unit = np.column_stack((-tangents_unit[:,1], tangents_unit[:,0]))
    t_vals = np.linspace(0, 1, len(traj))
    offsets = offset_amplitude * np.sin(2*np.pi*num_waves * t_vals)[:,None]
    s_arc = xy_values + normals_unit * offsets

    # 合并回三维
    stack_np = np.hstack((s_arc, z_values[:,None]))
    return stack_np.tolist()

def interpolate_z_coordinates(trajectory):
    """
    修复后的z坐标插值函数
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


def generate_speed_profile(speed_profile=None, speed_mode=None, num_points=3):
    start = speed_profile[0]
    end = speed_profile[1]
    up_limit = 8.0
    down_limit = 3.5
    p_ratio = random.uniform(down_limit, up_limit)
    n_ratio = random.uniform(-down_limit, -up_limit)
    t = np.linspace(0, 1, num_points)
    straight_speed = np.linspace(start[2], end[2], num_points)
    dive_speed = start[2] + (end[2] - start[2]) * np.exp(p_ratio * (t - 1))
    climb_speed = start[2] + (end[2] - start[2]) * (1 - np.exp(n_ratio * t))
    if speed_mode is None:
        return random.choice([straight_speed, dive_speed, climb_speed])
    elif speed_mode == 'straight':
        return straight_speed
    elif speed_mode == 'dive':
        return dive_speed
    elif speed_mode == 'climb':
        return climb_speed


def generate_speed_interpolation(traj, speed_profile=None, num_points=3):
    """
    根据速度变化参数对轨迹进行动态插值（支持多段速度变化）

    输入：
        x, y, z：原始轨迹坐标
        speed_profile：速度变化参数列表（如[1,2,5]表示速度从1→2→5分段变化）
        num_points：输出点数

    输出：
        new_x, new_y, new_z：结合速度变化的插值轨迹
    """
    x = [_traj[0] for _traj in traj]
    y = [_traj[1] for _traj in traj]
    z = [_traj[2] for _traj in traj]
    # 1. 计算原始轨迹累积距离
    if speed_profile is None:
        speed_profile = [1, 5]
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    segment_distances = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
    accumulated_distances = np.insert(np.cumsum(segment_distances), 0, 0)
    total_distance = accumulated_distances[-1]

    # 2. 生成速度曲线（支持任意长度的speed_profile）
    if len(speed_profile) == 1:
        # 恒定速度
        speed_curve = np.full_like(x, speed_profile[0])
    else:
        # 关键点位置：将累积距离均匀分配给speed_profile中的速度点
        key_positions = np.linspace(0, total_distance, len(speed_profile))
        # 创建分段线性插值函数（网页6的分段插值思想）
        speed_interp = interp1d(
            key_positions, speed_profile,
            kind='linear', fill_value='extrapolate'
        )
        # 对每个原始轨迹点进行速度插值（网页7的区间划分策略）
        speed_curve = speed_interp(accumulated_distances)

    # 3. 计算时间参数化（积分求累积时间）
    time_per_segment = segment_distances / speed_curve[:-1]
    accumulated_times = np.insert(np.cumsum(time_per_segment), 0, 0)
    total_time = accumulated_times[-1]

    # 4. 生成等时间间隔采样点
    desired_times = np.linspace(0, total_time, num_points)

    # 5. 构建时间-位置插值函数（网页3的调整旋转弧度思想）
    fx = interp1d(accumulated_times, x, kind='linear', fill_value="extrapolate")
    fy = interp1d(accumulated_times, y, kind='linear', fill_value="extrapolate")
    fz = interp1d(accumulated_times, z, kind='linear', fill_value="extrapolate")

    # 6. 执行插值
    x_new = fx(desired_times)
    y_new = fy(desired_times)
    z_new = fz(desired_times)

    return [[x, y, z] for x, y, z in zip(x_new, y_new, z_new)]


def coord_extention(path_pp, orders, height_value_set):
    _last_coord = None
    start = 0
    end = -1
    for _id, path in enumerate(path_pp):
        path['trajectory'] = [list(_loc) for _loc in list(path['trajectory'])]
        if path['type'] == 'breakthrough':
            bt_z_start = height_value_set[path['type']][start]
            if _id == 0:
                path['trajectory'][0].append(bt_z_start)
            else:
                path['trajectory'][0] = _last_coord
            _cur_bt_z_end = bt_z_start * (1 - orders[_id]['routed_ratio'])
            path['trajectory'][-1].append(round(_cur_bt_z_end, 3))
            # path['trajectory'] = generate_random_trajectory(path['type'],path['trajectory'])
            _last_coord = path['trajectory'][-1]

        elif path['type'] == 'escape':
            escape_z_end = height_value_set[path['type']][end]
            path['trajectory'][0] = _last_coord
            _cur_escape_z_end = escape_z_end * orders[_id]['routed_ratio']
            path['trajectory'][-1].append(_cur_escape_z_end)
            _last_coord = path['trajectory'][-1]
        elif path['type'] == 'detour':
            detour_z_end = height_value_set[path['type']][end]
            path['trajectory'][0] = _last_coord
            _cur_detour_z_end = detour_z_end * orders[_id]['routed_ratio']
            path['trajectory'][-1].append(_cur_detour_z_end)
            path['trajectory'] = interpolate_z_coordinates(path['trajectory'])
            _last_coord = path['trajectory'][-1]

    return path_pp


def fleets_coord_extention(path_pp, orders, height_value_set, direction_range_set=None,traj_points = 6):
    _last_coord = {}
    start = 0
    _aggregate = 1
    end = -1

    for fleets_step, path in enumerate(path_pp):
        if path["type"] == 'independent':
            for fleet_traj in path["trajectories"]:
                for fleet_step, _traj in enumerate(fleet_traj['trajectory']):
                    _traj['trajectory'] = [list(_loc) for _loc in list(_traj['trajectory'])]
                    _altitude_start = height_value_set[_traj['type']][start]
                    _altitude_end = height_value_set[_traj['type']][end]
                    height_ratio = [_order['orders'][fleet_step]['routed_ratio'] for _order in
                                    orders[fleets_step]['plans'] if
                                    _order['fleet'] == fleet_traj["fleet"]]
                    if fleet_traj["fleet"] not in _last_coord:
                        _traj['trajectory'][start].append(_altitude_start)
                    else:
                        _traj['trajectory'][start] = _last_coord[fleet_traj["fleet"]]
                        _altitude_start = _traj['trajectory'][start][2]
                    if _traj['type'] == 'breakthrough':
                        _cur_altitude_end = _altitude_start * (1 - height_ratio[0])
                    else:
                        _cur_altitude_end = _altitude_end * height_ratio[0]

                    _traj['trajectory'][end].append(round(_cur_altitude_end, 3))
                    _last_coord[fleet_traj["fleet"]] = _traj['trajectory'][end]
                    _traj['trajectory'] = interpolate_z_coordinates(_traj['trajectory'])
                    
        elif path["type"] == 'aggregate':
            for _item,(fleet, fleet_traj) in enumerate(path["aggregate_trajectories"].items()):

                fleet_traj['trajectory'] = [list(_loc) for _loc in list(fleet_traj['trajectory'])]
                _altitude_start = height_value_set[fleet_traj['type']][start]
                _altitude_end = height_value_set[fleet_traj['type']][end]
                height_ratio = orders[fleets_step]['order']['routed_ratio']
                if fleet not in _last_coord:
                    fleet_traj['trajectory'].append(_altitude_start)
                else:
                    # * (1 - orders[fleets_step]['aggregate_ratio'])
                    _aggregate_height = list(_last_coord[fleet])
                    _aggregate_height[2] = _aggregate_height[2] * (1 - orders[fleets_step]['aggregate_ratio'])
                    fleet_traj['trajectory'][start] = _last_coord[fleet]
                    fleet_traj['trajectory'][_aggregate].append(_aggregate_height[2])
                    _altitude_start = _aggregate_height[2]
                if fleet_traj['type'] == 'breakthrough':
                    _cur_altitude_end = _altitude_start * (1 - height_ratio)
                else:
                    _cur_altitude_end = _altitude_end * height_ratio
                fleet_traj['trajectory'][end].append(round(_cur_altitude_end, 3))
                _last_coord[fleet] = fleet_traj['trajectory'][end]
                _last_coord[path["fleet"]] = fleet_traj['trajectory'][end]
                if _item == 0:
                    gathering_point = fleet_traj['trajectory'][_aggregate]
                    fleet_traj['trajectory'] = interpolate_z_coordinates(fleet_traj['trajectory'])
                    _pre_gathering_traj = fleet_traj['trajectory'][:_aggregate+1]
                    gathering_traj = fleet_traj['trajectory'][_aggregate:]
                else:
                    fleet_traj['trajectory'][_aggregate] = gathering_point
                    _pre_gathering_traj = interpolate_z_coordinates(fleet_traj['trajectory'][:_aggregate])
                    fleet_traj['trajectory'] =  _pre_gathering_traj + gathering_traj
        elif path["type"] == 'disperse':
            for _fleet in path['fleets']:
                _last_coord[_fleet] = _last_coord[orders[fleets_step - 1]['fleet']]
            for fleet, fleet_traj in path["disperse_trajectories"].items():
                for fleet_step, _traj in enumerate(fleet_traj):
                    _traj['trajectory'] = [list(_loc) for _loc in list(_traj['trajectory'])]
                    _altitude_start = height_value_set[_traj['type']][start]
                    _altitude_end = height_value_set[_traj['type']][end]
                    height_ratio = [_order['orders'][fleet_step]['routed_ratio'] for _order in
                                    orders[fleets_step]['plans'] if
                                    _order['fleet'] == fleet]
                    if fleet not in _last_coord:
                        _traj['trajectory'].append(_altitude_start)
                    else:
                        _traj['trajectory'][start] = _last_coord[fleet]
                        _altitude_start = _last_coord[fleet][2]
                    if _traj['type'] == 'breakthrough':
                        _cur_altitude_end = _altitude_start * (1 - height_ratio[0])
                    else:
                        _cur_altitude_end = _altitude_end * height_ratio[0]
                    _traj['trajectory'][end].append(round(_cur_altitude_end, 3))
                    _last_coord[fleet] = _traj['trajectory'][end]
                    _traj['trajectory'] = interpolate_z_coordinates(_traj['trajectory'])
    for _plan in path_pp:
        if _plan['type'] == 'independent':
            for _sub_plan in _plan['trajectories']:
                _fleet_name = _sub_plan['fleet']
                _fleet_traj = _sub_plan['trajectory']
                for _p_iter, _path in enumerate(_fleet_traj):
                    _path['trajectory'] = sbb.generate_basic_behavior(_path['type'], _plan['type'], _path['trajectory'],
                                                                      height_value_set[_path['type']],
                                                                      direction_range_set[_path['type']],traj_points)
        elif _plan['type'] == 'aggregate':
            _agg_paths = _plan['aggregate_trajectories']
            _agg_fleet_name = _plan['fleet']
            rand_breakthrough_ratio = random.randint(direction_range_set['breakthrough'][0], direction_range_set['breakthrough'][1])
            rand_detour_ratio = random.randint(direction_range_set['detour'][0], direction_range_set['detour'][1])
            direction_range_set['aggregate_ratio'] = {
                'breakthrough':[rand_breakthrough_ratio,rand_breakthrough_ratio],
                'detour':[rand_detour_ratio,rand_detour_ratio],
            }
            for _item,(_fleet, _path) in enumerate(_agg_paths.items()):
                if _item == 0:
                    _path['trajectory'] = sbb.generate_basic_behavior(_path['type'], _plan['type'], _path['trajectory'],
                                                                      height_value_set[_path['type']],
                                                                      direction_range_set['aggregate_ratio'][_path['type']],traj_points)
                    _aft_traj = _path['trajectory'][traj_points:]
                else:
                    _pre_gathering_traj = sbb.generate_basic_behavior(_path['type'], _plan['type'], _path['trajectory'][:_aggregate+1],
                                                                      height_value_set[_path['type']],
                                                                      direction_range_set['aggregate_ratio'][_path['type']],traj_points)

                    _path['trajectory'] = _pre_gathering_traj + _aft_traj

        elif _plan['type'] == 'disperse':
            _disp_paths = _plan['disperse_trajectories']
            for _fleet, _fleet_traj in _disp_paths.items():
                for _p_iter, _path in enumerate(_fleet_traj):
                    _path['trajectory'] = sbb.generate_basic_behavior(_path['type'],_plan['type'] ,_path['trajectory'],
                                                                      height_value_set[_path['type']],
                                                                      direction_range_set[_path['type']],traj_points)
    return path_pp


def fleets_coord_extention_v2(graph_data, key_paths, height_range_value_set, aggreated_height_value_set,
                              direction_range_set=None, traj_points=6):
    """
    修复后的 fleets_coord_extention_v2 函数
    """
    full_path_data = [[] for _ in range(len(key_paths))]
    start_point = 0
    end_point = -1

    def find_edge_index(edges, a, b):
        """在 edges 列表中查找第一个 from==a 且 to==b 的元素的下标"""
        for idx, edge in enumerate(edges):
            if edge.get("from") == a and edge.get("to") == b:
                return idx
        return -1

    for _path_num, _key_path in enumerate(key_paths):
        _last_coord = None
        for _step, _node in enumerate(_key_path):
            if _step < len(_key_path) - 1:
                _current_node = _key_path[_step]
                _next_node = _key_path[_step + 1]
                _current_node_index = find_edge_index(graph_data, _current_node, _next_node)

                if _current_node_index == -1:
                    print(f"Warning: Edge ({_current_node}, {_next_node}) not found in graph_data")
                    continue

                _edge_info = graph_data[_current_node_index]
                _order_mode = _edge_info['attrs']['order_mode']
                _order_type = _edge_info['attrs']['order_type']
                _orig_traj = _edge_info['attrs']['plan']['trajectory']

                _traj = copy.deepcopy(_orig_traj)


                # 确保轨迹是列表格式
                _traj = [list(coord) if isinstance(coord, (list, tuple)) else coord for coord in _traj]

                # 设置高度值
                if _last_coord is None:
                    _altitude_start = random.randint(
                        height_range_value_set[_order_type][0][0],
                        height_range_value_set[_order_type][0][1]
                    )
                else:
                    _altitude_start = _last_coord[2]

                if _order_mode == 'aggregate':
                    _altitude_end = aggreated_height_value_set.get(_next_node,
                                                                   random.randint(
                                                                       height_range_value_set[_order_type][1][0],
                                                                       height_range_value_set[_order_type][1][1]))
                else:
                    _altitude_end = random.randint(
                        height_range_value_set[_order_type][1][0],
                        height_range_value_set[_order_type][1][1]
                    )

                # 确保轨迹点有足够的维度
                if len(_traj) >= 2:
                    # 添加起始高度
                    if len(_traj[start_point]) == 2:
                        _traj[start_point].append(_altitude_start)
                    elif len(_traj[start_point]) == 3:
                        _traj[start_point][2] = _altitude_start

                    # 添加结束高度
                    if len(_traj[end_point]) == 2:
                        _traj[end_point].append(_altitude_end)
                    elif len(_traj[end_point]) == 3:
                        _traj[end_point][2] = _altitude_end

                    _last_coord = list(_traj[end_point])
                    # 插值z坐标
                    _traj = interpolate_z_coordinates(_traj)
                    # 生成基础行为轨迹
                    try:
                        _traj = sbb.generate_basic_behavior(
                            _order_type, _order_mode, _traj,
                            height_range_value_set[_order_type],
                            direction_range_set[_order_type],
                            traj_points
                        )
                        full_path_data[_path_num].append(_traj)
                        _edge_info['attrs']['plan']['trajectory'] = _traj
                    except Exception as e:
                        print(f"Error generating trajectory: {e}")
                        # 使用原始轨迹作为备份
                        # full_path_data[_path_num].append(_traj)
                        _edge_info['attrs']['plan']['trajectory'] = _traj
                else:
                    print(f"Warning: Trajectory too short for edge ({_current_node}, {_next_node})")

    return graph_data

def visualize_trajectories_from_json(json_data, mode='all', figsize=(12, 10), show=True):
    """
    从JSON数据中提取轨迹并进行3D可视化

    参数:
        json_data: 轨迹数据，可以是:
            - JSON字符串
            - 字典列表（已解析的JSON）
            - 文件路径
        mode: 可视化模式
            - 'all': 显示所有轨迹
            - 'by_fleet': 按舰队分组显示
            - 'by_type': 按任务类型分组显示
            - 'single': 显示单条轨迹（需要指定索引）
        figsize: 图像大小
        show: 是否自动显示图像，默认为True

    返回:
        fig, ax: matplotlib图像对象（如果show=False）
        None: 如果show=True，直接显示图像
    """
    # 数据预处理
    if isinstance(json_data, str):
        if json_data.endswith('.json'):
            # 文件路径
            with open(json_data, 'r', encoding='utf-8') as f:
                data = json.load(f)
        else:
            # JSON字符串
            data = json.loads(json_data)
    else:
        # 已经是列表/字典
        data = json_data

    # 提取轨迹数据
    trajectories = []
    metadata = []

    for edge in data:
        if 'attrs' in edge and 'plan' in edge['attrs']:
            plan = edge['attrs']['plan']
            if 'trajectory' in plan:
                traj = plan['trajectory']

                # 确保每个点都有3个坐标
                processed_traj = []
                for point in traj:
                    if len(point) >= 3:
                        processed_traj.append([point[0], point[1], point[2]])
                    elif len(point) == 2:
                        processed_traj.append([point[0], point[1], 0])  # 默认高度

                if len(processed_traj) > 0:
                    trajectories.append(processed_traj)

                    # 提取元数据
                    meta = {
                        'from': edge.get('from', 'unknown'),
                        'to': edge.get('to', 'unknown'),
                        'fleet': edge['attrs'].get('fleet_no', 'unknown'),
                        'type': edge['attrs'].get('order_type', 'unknown'),
                        'mode': edge['attrs'].get('order_mode', 'unknown'),
                        'target': edge['attrs'].get('target', 'unknown')
                    }
                    metadata.append(meta)

    # 创建图像
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    # 获取颜色映射
    cmap = plt.get_cmap('tab10')

    # 用于记录每个点已绘制标签的次数，决定偏移方向
    label_counts = defaultdict(int)

    def offset_position_3d(xyz, count):
        """根据同一点被标注次数，选用不同的偏移模式（3D版本）"""
        # 3D偏移量 (dx, dy, dz)
        offsets = [
            (35, 35, 10), (-35, 35, 10), (35, -35, 10), (-35, -35, 10),
            (40, 0, 15), (0, 40, 15), (-40, 0, 15), (0, -40, 15),
            (55, 45, 20), (-55, 45, 20), (55, -45, 20), (-55, -45, 20),
            (0, 0, 50), (0, 0, -50)  # 纯垂直偏移
        ]
        dx, dy, dz = offsets[count % len(offsets)]
        return xyz[0] + dx, xyz[1] + dy, xyz[2] + dz

    # 根据模式设置颜色
    if mode == 'by_fleet':
        # 按舰队分组
        fleet_colors = {}
        for i, meta in enumerate(metadata):
            fleet = meta['fleet']
            if fleet not in fleet_colors:
                fleet_colors[fleet] = cmap(len(fleet_colors) % 10)
        colors = [fleet_colors[meta['fleet']] for meta in metadata]

    elif mode == 'by_type':
        # 按任务类型分组
        type_colors = {'breakthrough': 'red', 'detour': 'blue', 'escape': 'green'}
        colors = [type_colors.get(meta['type'], 'gray') for meta in metadata]

    else:  # 'all' 或其他模式
        # 使用tab10颜色映射，与二维版本保持一致
        colors = [cmap(i % 10) for i in range(len(trajectories))]

    # 绘制轨迹
    for i, (traj, meta, color) in enumerate(zip(trajectories, metadata, colors)):
        traj_arr = np.array(traj)
        x, y, z = traj_arr[:, 0], traj_arr[:, 1], traj_arr[:, 2]

        # 1. 绘制轨迹线 - 使用与二维版本相同的标签格式
        label = f"{meta['from']}→{meta['to']} [{meta['type']}]"
        ax.plot(x, y, z, linestyle='-', linewidth=2, color=color,
                alpha=0.8, label=label)

        # # 2. 添加方向箭头（3D版本）
        # # 在轨迹上选择几个点添加方向箭头
        # arrow_indices = np.linspace(0, len(traj_arr) - 2, min(5, len(traj_arr) - 1), dtype=int)
        # for idx in arrow_indices:
        #     start_point = traj_arr[idx]
        #     end_point = traj_arr[idx + 1]
        #     direction = end_point - start_point
        #
        #     # 归一化方向向量并缩放
        #     if np.linalg.norm(direction) > 0:
        #         direction = direction / np.linalg.norm(direction) * 50  # 箭头长度
        #         ax.quiver(start_point[0], start_point[1], start_point[2],
        #                   direction[0], direction[1], direction[2],
        #                   color=color, alpha=0.7, arrow_length_ratio=0.2,
        #                   linewidth=1.5)

        # 3. 起点和终点标记 - 与二维版本保持一致的形状
        start = traj_arr[0]
        end = traj_arr[-1]

        # 起点标记（圆形）
        ax.scatter(start[0], start[1], start[2],
                   marker='o', s=60, edgecolor='k', facecolor=color, linewidth=1)

        # 终点标记（X形）
        ax.scatter(end[0], end[1], end[2],
                   marker='X', s=80, edgecolor='k', facecolor=color, linewidth=1)

        # 4. 起点标签（偏移） - 使用与二维版本相同的节点ID
        start_key = (round(start[0]), round(start[1]), round(start[2]))
        sc = label_counts[start_key]
        sx, sy, sz = offset_position_3d(start, sc)
        ax.text(sx, sy, sz, str(meta['from']),
                fontsize=8, color='black',
                verticalalignment='center',
                horizontalalignment='center')
        label_counts[start_key] += 1

        # 5. 终点标签（偏移） - 使用与二维版本相同的节点ID
        end_key = (round(end[0]), round(end[1]), round(end[2]))
        ec = label_counts[end_key]
        ex, ey, ez = offset_position_3d(end, ec)
        ax.text(ex, ey, ez, str(meta['to']),
                fontsize=8, color='black',
                verticalalignment='center',
                horizontalalignment='center')
        label_counts[end_key] += 1

        # 可选：添加中间路径点
        if len(traj_arr) > 2:
            ax.scatter(x[1:-1], y[1:-1], z[1:-1], marker='.', s=20,
                       color=color, alpha=0.6)

    # 设置坐标轴标签 - 与二维版本保持一致的中文标签风格
    ax.set_xlabel('X 坐标 (m)', fontsize=12)
    ax.set_ylabel('Y 坐标 (m)', fontsize=12)
    ax.set_zlabel('高度 Z (m)', fontsize=12)

    # 设置标题 - 与二维版本保持一致的标题风格
    title_map = {
        'all': '各任务轨迹三维示意图（含方向箭头与偏移标签）',
        'by_fleet': '按舰队分组的三维轨迹',
        'by_type': '按任务类型分组的三维轨迹'
    }
    ax.set_title(title_map.get(mode, f'三维轨迹可视化 - {mode}'),
                 fontsize=14, fontweight='bold')

    # 添加网格 - 与二维版本保持一致的网格样式
    ax.grid(True, linestyle='--', alpha=0.5)

    # 添加图例 - 显示所有轨迹
    ax.legend(loc='upper right', fontsize='small', ncol=2)

    # 设置视角
    ax.view_init(elev=20, azim=45)

    # 调整布局
    plt.tight_layout()

    # 根据show参数决定是否显示
    if show:
        plt.show()
        return None
    else:
        return fig, ax


def visualize_3d_paths(path_pp):
    """
    在三维坐标系中可视化多条轨迹（由若干段组成），
    每段用折线连接，并标出起点（o）与终点（^）。
    参数:
      path_pp: List[List[List[List[float]]]]
        外层列表是多条轨迹，每条轨迹又由若干段组成，
        每段是若干 [x,y,z] 点的列表。
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for path_idx, path in enumerate(path_pp):
        for seg in path:
            seg_arr = np.array(seg)       # 转成 NumPy 数组 (N×3)
            x, y, z = seg_arr[:,0], seg_arr[:,1], seg_arr[:,2]
            # 画线
            ax.plot(x, y, z, linestyle='-', linewidth=2)
            # 标记起点和终点
            ax.scatter(x[0], y[0], z[0], marker='o', s=40)
            ax.scatter(x[-1], y[-1], z[-1], marker='^', s=40)

    ax.set_xlabel('X 坐标')
    ax.set_ylabel('Y 坐标')
    ax.set_zlabel('高度 Z')
    ax.set_title('三维轨迹示意图')
    plt.tight_layout()
    plt.show()


def random_point_on_boundary(vertices):
    """
    给定多边形顶点列表 vertices = [[x0,y0], [x1,y1], ..., [xN,yN]]
    （假定已闭合或函数会自动闭合最后一条边），
    返回边界上的一个随机点 [x, y]。
    """
    # 1. 构造边列表（自动闭合最后一条边）
    edges = []
    for i in range(len(vertices)):
        p0 = np.array(vertices[i])
        p1 = np.array(vertices[(i+1) % len(vertices)])
        edges.append((p0, p1))

    # 2. 计算每条边的长度
    lengths = [np.linalg.norm(p1 - p0) for p0, p1 in edges]
    total_length = sum(lengths)
    if total_length == 0:
        raise ValueError("多边形边长总和为0，可能顶点都相同")

    # 3. 构造累积权重并随机选择一条边
    cum_lengths = np.cumsum(lengths)
    r = random.uniform(0, total_length)
    # 找到 r 落在哪段累积区间
    edge_idx = np.searchsorted(cum_lengths, r)
    p0, p1 = edges[edge_idx]

    # 4. 在这条边上按比例插值
    # 计算这条边在总长中的起始位置
    prev_cum = cum_lengths[edge_idx-1] if edge_idx > 0 else 0
    frac = (r - prev_cum) / lengths[edge_idx]  # 在该边上的相对位置 [0,1]
    pt = p0 + frac * (p1 - p0)
    return pt.tolist()


def segment_length(seg: list[list[float]]) -> float:
    """计算一段 3D 轨迹的总欧氏长度"""
    L = 0.0
    for i in range(len(seg) - 1):
        L += math.dist(seg[i], seg[i + 1])
    return L


def add_time_to_paths(
    path_pp: list,
    key_paths: list,
    agg_nodes: set,
    speed: float,
    is_print: bool = False
) -> list:
    '''
    为 path_pp 中的每条轨迹打时间戳，并保证在 agg_nodes 处同步到达。
    聚合段不再等待，而是按各自段长调整速度，让它们都在相同时间完成。
    返回与原来相同的 path_pp_time 结构。
    '''
    num_drones = len(path_pp)

    # 1. 计算每段长度和初始时长（基于统一 speed）
    lengths = [
        [ segment_length(path_pp[i][s]) for s in range(len(path_pp[i])) ]
        for i in range(num_drones)
    ]
    durations = [
        [ lengths[i][s] / speed for s in range(len(path_pp[i])) ]
        for i in range(num_drones)
    ]

    # 2. 找出每个聚合节点对应的段 (到达 node 时段索引 = k-1)
    agg_map = defaultdict(list)
    for i, kp in enumerate(key_paths):
        for k, node in enumerate(kp):
            if node in agg_nodes and k > 0 and (k-1) < len(durations[i]):
                agg_map[node].append((i, k-1))

    # 3. 对每个聚合段，取最大时长并更新 durations
    #    同时计算该段的新速度（可选在仿真中使用）
    #    但不改变返回格式，只用 durations 做插值
    for node, pairs in agg_map.items():
        t_max = max(durations[i][s] for i, s in pairs)
        for i, s in pairs:
            durations[i][s] = t_max
            # new_speed = lengths[i][s] / t_max  # 如果需要可记录这行

    # 4. 均匀插值打时间戳
    path_pp_time: list = []
    for i in range(num_drones):
        t_cursor = 0.0
        traj_time: list = []
        for s, seg in enumerate(path_pp[i]):
            t_seg = durations[i][s]
            n = len(seg)
            if n < 2:
                traj_time.append([[seg[0][0], seg[0][1], seg[0][2], t_cursor]])
            else:
                dt = t_seg / (n - 1)
                seg_pts = [
                    [x, y, z, t_cursor + j * dt]
                    for j, (x, y, z) in enumerate(seg)
                ]
                traj_time.append(seg_pts)
            t_cursor += t_seg
        path_pp_time.append(traj_time)

    # 5. 可选打印聚合节点到达时间
    if is_print:
        agg_times = defaultdict(list)
        for i, traj in enumerate(path_pp_time):
            for s, seg in enumerate(traj):
                if key_paths[i][s] in agg_nodes:
                    agg_times[key_paths[i][s]].append((i, seg[-1][3]))
        for node, arrivals in agg_times.items():
            print(f'agg node {node}:')
            for i, t in arrivals:
                print(f'  drone #{i} at t = {t:.3f} s')
            print()

    return path_pp_time