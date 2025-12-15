""" TODO: 包含一些基本的时间、空间等信息的转换与处理工具 """
import sys
import os, os.path as osp

import pickle as pkl
import json
import re
import configparser

import math
import numpy as np
import pyproj
import random
import networkx as nx
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
from shapely.geometry import Polygon
from shapely.ops import unary_union

from colorsys import hls_to_rgb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

WS_ROOT = osp.dirname(osp.dirname(osp.abspath(__file__)))
if not WS_ROOT in sys.path:
    sys.path.append(WS_ROOT)

BASIC_CONFIGS_FILE = osp.join(WS_ROOT, 'configs.ini')

def generate_circle_positions_from_diameter(num, p1, p2):
    """以两个坐标点为直径的圆内生成指定数量的随机位置
    """
    # 1. 计算圆心 (中点)
    center_lon = (p1[0] + p2[0]) / 2
    center_lat = (p1[1] + p2[1]) / 2
    center_alt = (p1[2] + p2[2]) / 2  

    # 2. 计算半径 (平面欧氏距离的一半)
    # 注意：在小范围内直接用经纬度差值计算是可行的
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    radius = math.sqrt(dx**2 + dy**2) / 2

    positions = []
    for _ in range(num):
        # 3. 极坐标生成随机点 
        # 使用 sqrt(random()) 是为了消除聚集在圆心的现象，保证在圆面积上均匀分布
        r = radius * math.sqrt(random.random())
        theta = random.random() * 2 * math.pi

        new_lon = center_lon + r * math.cos(theta)
        new_lat = center_lat + r * math.sin(theta)
        
        positions.append([new_lon, new_lat, center_alt])

    return positions


class BasicConfigs:
    def __init__(self, ini_file=BASIC_CONFIGS_FILE):
        self.cfg_file = ini_file

        self.ESCAPE_DEVIATE_ANGLE_MAX = None

        self.AVOID_ANTIAIR_DISTANCE = None
        self.AVOID_RADAR_DISTANCE = None
        self.AVOID_HQ_DISTANCE = None
        self.AVOID_AVERAGE_DISTANCE = None

        self.PLANNING_ROUTE_TYPES = None
        self.PLANNING_BREAKTHROUGH_FACILITY_TYPES = None
        self.PLANNING_ESCAPE_FACILITY_TYPES = None

        self._load_config(self.cfg_file)

    def _load_config(self, cfg_file):
        _config = configparser.ConfigParser()

        try:
            _config.read(cfg_file, encoding='utf-8')

            self.ESCAPE_DEVIATE_ANGLE_MAX = float(_config['ESCAPING']['ESCAPE_ROUTE_DEVIATE_ANGLE_MAX'])

            self.AVOID_ANTIAIR_DISTANCE = float(_config['DETOURING']['AVOIDE_ANTIAIR_DISTANCE'])
            self.AVOID_RADAR_DISTANCE = float(_config['DETOURING']['AVOIDE_RADAR_DISTANCE'])
            self.AVOID_HQ_DISTANCE = float(_config['DETOURING']['AVOIDE_HQ_DISTANCE'])

            self.AVOID_AVERAGE_DISTANCE = np.mean(
                [self.AVOID_ANTIAIR_DISTANCE, self.AVOID_RADAR_DISTANCE, self.AVOID_HQ_DISTANCE])

            self.PLANNING_ROUTE_TYPES = [_type.strip() for _type in _config['PATH_PLANNING']['ROUTE_TYPES'].split(',')]
            self.PLANNING_BREAKTHROUGH_FACILITY_TYPES = [_type.strip() for _type in _config['PATH_PLANNING'][
                'BREAKTHROUGH_FACILITIES_TYPES'].split(',')]
            self.PLANNING_ESCAPE_FACILITY_TYPES = [_type.strip() for _type in
                                                   _config['PATH_PLANNING']['ESCAPE_FACILITIES_TYPES'].split(',')]

        except Exception as e:
            print(e)

    def retrieve_avoid_radius(self):
        return {'AVOIDE_ANTIAIR_DISTANCE': self.AVOID_ANTIAIR_DISTANCE,
                'AVOIDE_RADAR_DISTANCE': self.AVOID_RADAR_DISTANCE,
                'AVOIDE_HQ_DISTANCE': self.AVOID_HQ_DISTANCE}

    def update_avoid_radius(self, radius_info, write_back=False):
        '''更新所有的避障半径'''
        self.AVOID_ANTIAIR_DISTANCE = radius_info['AVOIDE_ANTIAIR_DISTANCE']
        self.AVOID_RADAR_DISTANCE = radius_info['AVOIDE_RADAR_DISTANCE']
        self.AVOID_HQ_DISTANCE = radius_info['AVOIDE_HQ_DISTANCE']

        if write_back:
            _config = configparser.ConfigParser()
            _config.read(self.cfg_file, encoding='utf-8')
            _config['DETOURING']['AVOIDE_ANTIAIR_DISTANCE'] = str(self.AVOID_ANTIAIR_DISTANCE)
            _config['DETOURING']['AVOIDE_RADAR_DISTANCE'] = str(self.AVOID_RADAR_DISTANCE)
            _config['DETOURING']['AVOIDE_HQ_DISTANCE'] = str(self.AVOID_HQ_DISTANCE)

            with open(self.cfg_file, 'w', encoding='utf-8') as f:
                _config.write(f)


GlobalBasicConfigs = BasicConfigs()


class LngLat2UTM(object):
    """ 专门将经纬度轨迹转换为UTM坐标的工具类"""

    def __init__(self, zone_number=51, north_or_south='N'):
        self.zone_number = zone_number
        self.north_or_south = north_or_south

        if self.north_or_south == 'N':
            self.utm_epsg = f"EPSG:326{self.zone_number:02d}"  # 北半球
        else:
            self.utm_epsg = f"EPSG:327{self.zone_number:02d}"  # 南半球

        self.utm_transformer = pyproj.Transformer.from_crs("EPSG:4326", self.utm_epsg, always_xy=True)

    def lon_lat_to_utm(self, lng, lat):
        """ 将经纬度坐标转换为UTM坐标"""
        x, y = self.utm_transformer.transform(lng, lat)
        return x, y

    def utm_to_lng_lat(self, x, y):
        """ 将UTM坐标转换为经纬度坐标"""
        lng, lat = self.utm_transformer.transform(x, y, direction='INVERSE')
        return lng, lat

    def lng_lat_to_utm_array(self, lng_lat_array):
        """ 将经纬度坐标数组转换为UTM坐标数组"""
        x, y = self.utm_transformer.transform(lng_lat_array[:, 0], lng_lat_array[:, 1])
        return np.vstack((x, y)).T

    def utm_to_lng_lat_array(self, utm_array):
        """ 将UTM坐标数组转换为经纬度坐标数组"""
        lng, lat = self.utm_transformer.transform(utm_array[:, 0], utm_array[:, 1], direction='INVERSE')
        return np.vstack((lng, lat)).T

    def euavs_lnglats_trajs_to_utm(self, euavs_trajs_llts):
        """ 将所有无人机的经纬度轨迹转换为UTM坐标"""
        if isinstance(euavs_trajs_llts, str):
            euavs_trajs_llts = json.loads(euavs_trajs_llts)

        euavs_trajs_utm = {}
        for _key, _val in euavs_trajs_llts.items():
            _utm_xys = self.lng_lat_to_utm_array(np.array([_val['lngs'], _val['lats']]).T)
            euavs_trajs_utm[_key] = {
                'xs': _utm_xys[:, 0].tolist(),
                'ys': _utm_xys[:, 1].tolist(),
                'ts': _val['ts']
            }

        return euavs_trajs_utm


class DroneTrajectory:
    def __init__(self, uav_id, lnglats=[], alts=[], utm_xys=[], ts=[], time_step=1):
        self.uav_id = uav_id

        self.lnglats = lnglats
        self.alts = alts
        self.utm_xys = utm_xys
        self.ts = ts
        self.time_step = time_step

        self.llt2utm_cnvrtr = LngLat2UTM()

    def set_longlats(self, lngs, lats):
        self.lnglats = np.array([lngs, lats]).T

    def set_alts(self, alts):
        self.alts = alts

    def set_utm_xys(self, utm_xys):
        self.utm_xys = utm_xys

    def set_ts(self, ts):
        self.ts = ts

    def __len__(self):
        return len(self.alts)

    def append_utmxy_alt(self, utm_xy, alt):
        if len(self.utm_xys) == 0:
            self.utm_xys = np.array([utm_xy])
        else:
            self.utm_xys = np.append(self.utm_xys, [utm_xy], axis=0)

        if len(self.lnglats) == 0:
            self.lnglats = np.array([self.llt2utm_cnvrtr.utm_to_lng_lat(utm_xy[0], utm_xy[1])])
        else:
            self.lnglats = np.append(self.lnglats, [self.llt2utm_cnvrtr.utm_to_lng_lat(utm_xy[0], utm_xy[1])], axis=0)

        if len(self.alts) == 0:
            self.alts = np.array([alt])
        else:
            self.alts = np.append(self.alts, [alt])

        if len(self.ts) == 0:
            self.ts = np.array([0])
        else:
            self.ts = np.append(self.ts, [self.ts[-1] + self.time_step])

    def append_lnglat_alt(self, lnglat, alt):
        if len(self.lnglats) == 0:
            self.lnglats = np.array([lnglat])
        else:
            self.lnglats = np.append(self.lnglats, [lnglat], axis=0)

        if len(self.utm_xys) == 0:
            self.utm_xys = np.array([self.llt2utm_cnvrtr.lng_lat_to_utm(lnglat[0], lnglat[1])])
        else:
            self.utm_xys = np.append(self.utm_xys, [self.llt2utm_cnvrtr.lng_lat_to_utm(lnglat[0], lnglat[1])], axis=0)

        if len(self.alts) == 0:
            self.alts = np.array([alt])
        else:
            self.alts = np.append(self.alts, [alt])

        if len(self.ts) == 0:
            self.ts = np.array([0])
        else:
            self.ts = np.append(self.ts, [self.ts[-1] + self.time_step])

    def location_at_step(self, step):
        _cur_utm_xy = self.utm_xys[step]
        _cur_lnglat = self.lnglats[step]
        _cur_alt = self.alts[step]

        return _cur_utm_xy, _cur_lnglat, _cur_alt

    def time_at_step(self, step):
        return self.ts[step]

    def show(self):
        print(f"Drone {self.uav_id} Trajectory:")
        print("<long>, <lat>, <alt>, <x>, <y>")
        for _i in range(len(self.lnglats)):
            print(
                f"{self.lnglats[_i][0]:.3f}, {self.lnglats[_i][1]:.3f}, {self.alts[_i]:.3f}, {self.utm_xys[_i][0]:.3f}, {self.utm_xys[_i][1]:.3f}")


class Facilities:
    def __init__(self, facilities_info, defend_rings_info, convert_to_utm=True):
        self.facilities_info = facilities_info

        self.antiairs = {}  # 防空设施
        self.headquartors = {}  # 指挥所
        self.probers = {}  # 探测设施

        self.defend_rings_info = defend_rings_info
        self.defend_rings = {}  # 防御圈层

        self.convert_to_utm = convert_to_utm
        self.lnglat_converter = LngLat2UTM()

        self._parse_facilities_categories(self.facilities_info)
        self._convert_defence_rings(self.defend_rings_info)

    def _make_polygon_from_circle(self, center_xy, radius, n_points=None):
        _border_points = []

        if n_points is None:
            _point_spacing = 80
            n_points = int(2 * np.pi * radius / _point_spacing)

        for _iter in range(n_points):
            _angle = 2 * math.pi * _iter / n_points
            _border_points.append((center_xy[0] + radius * math.cos(_angle),
                                   center_xy[1] + radius * math.sin(_angle)))

        return Polygon(_border_points)

    def _parse_facilities_categories(self, facilities_info):
        self.antiairs.clear()
        self.headquartors.clear()
        self.probers.clear()

        for _fac, _lnglat in facilities_info.items():
            if self.convert_to_utm:
                _utm_x, _utm_y = self.lnglat_converter.lon_lat_to_utm(_lnglat[0], _lnglat[1])
            else:
                _utm_x, _utm_y = _lnglat[0], _lnglat[1]

            if 'ua_' in _fac:
                self.antiairs[_fac] = [_utm_x, _utm_y]
            elif 'hq_' in _fac:
                self.headquartors[_fac] = [_utm_x, _utm_y]
            elif 'radar_' in _fac:
                self.probers[_fac] = [_utm_x, _utm_y]

    def get_target_location(self, facility_name, utm=True):
        if not utm:
            return self.facilities_info[facility_name]
        else:
            return self.lnglat_converter.lon_lat_to_utm(self.facilities_info[facility_name][0],
                                                        self.facilities_info[facility_name][1])

    def get_facilities_names(self):
        return list(self.facilities_info.keys())

    def pick_random_target(self, facility_type, utm=True):
        if facility_type == 'antiair':
            _fac = np.random.choice(list(self.antiairs.keys()))
        elif facility_type == 'headquarter':
            _fac = np.random.choice(list(self.headquartors.keys()))
        elif facility_type == 'prober':
            _fac = np.random.choice(list(self.probers.keys()))

        if not utm:
            return {'facility': _fac, 'location': self.get_target_location(_fac, utm=False)}
        else:
            return {'facility': _fac, 'location': self.get_target_location(_fac, utm=True)}

    def _convert_defence_rings(self, defend_rings_info):
        self.defend_rings.clear()

        if self.convert_to_utm:
            for _ring, _lnglats in defend_rings_info.items():
                _utm_xys = self.lnglat_converter.lng_lat_to_utm_array(np.array([_lnglats['lngs'], _lnglats['lats']]).T)
                self.defend_rings[_ring] = _utm_xys
        else:
            for _ring, _lnglats in defend_rings_info.items():
                self.defend_rings[_ring] = np.array([_lnglats['lngs'], _lnglats['lats']]).T

    def _get_union_polygons_border(self, polygons):
        _union_polygon = unary_union(polygons)
        return _union_polygon.exterior.coords.xy

    def get_spec_facility_polyborder(self, center_location, radius, ll2utm=True):
        if ll2utm:
            _polygon = self._make_polygon_from_circle(
                self.lnglat_converter.lon_lat_to_utm(center_location[0], center_location[1]), radius)
        else:
            _polygon = self._make_polygon_from_circle(np.array(center_location), radius)

        return _polygon.exterior.coords.xy

    def get_defence_facilities_polyborder(self, radius=GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE, union_polygons=False):
        _facs_polygons = []

        for _fac, _utm_xy in self.antiairs.items():
            _polygon = self._make_polygon_from_circle(_utm_xy, radius)
            _facs_polygons.append(_polygon)

        if len(_facs_polygons) > 1:
            if union_polygons:
                return self._get_union_polygons_border(_facs_polygons)
            else:
                return [_poly.exterior.coords.xy for _poly in _facs_polygons]
        else:
            return [_facs_polygons[0].exterior.coords.xy]

    def get_probe_facilities_polyborder(self, radius=GlobalBasicConfigs.AVOID_RADAR_DISTANCE, union_polygons=False):
        _facs_polygons = []

        for _fac, _utm_xy in self.probers.items():
            _polygon = self._make_polygon_from_circle(_utm_xy, radius)
            _facs_polygons.append(_polygon)

        if len(_facs_polygons) > 1:
            if union_polygons:
                return self._get_union_polygons_border(_facs_polygons)
            else:
                return [_poly.exterior.coords.xy for _poly in _facs_polygons]
        else:
            return [_facs_polygons[0].exterior.coords.xy]

    def get_defence_rings_polyborder(self, radius=GlobalBasicConfigs.AVOID_HQ_DISTANCE, union_polygons=False):
        _ext_defend_rings = []

        for _ring, _utm_xys in self.defend_rings.items():
            _cur_polygon = Polygon(_utm_xys)
            _ext_polygon = _cur_polygon.buffer(radius, join_style=2)
            _ext_defend_rings.append(_ext_polygon)

        if len(_ext_defend_rings) > 1:
            if union_polygons:
                return self._get_union_polygons_border(_ext_defend_rings)
            else:
                return [_poly.exterior.coords.xy for _poly in _ext_defend_rings]
        else:
            return [_ext_defend_rings[0].exterior.coords.xy]

    def visualize(self, start_point=None, show_defend_rings=True, show_borders=False, show_ppath=None, show_mode='3D',
                  block=False):
        fig = plt.figure(figsize=(10, 10))

        if show_mode == '3D':
            ax = fig.add_subplot(111, projection='3d', proj_type='ortho')
        else:
            ax = fig.add_subplot(111)

        # 绘制防御圈
        if show_defend_rings:
            for _ring, _lnglats in self.defend_rings_info.items():
                if self.convert_to_utm:
                    _utm_xys = self.lnglat_converter.lng_lat_to_utm_array(
                        np.array([_lnglats['lngs'], _lnglats['lats']]).T)
                else:
                    _utm_xys = np.array([_lnglats['lngs'], _lnglats['lats']]).T

                if show_mode == '3D':
                    ax.plot_trisurf(_utm_xys[:, 0], _utm_xys[:, 1], np.zeros_like(_utm_xys[:, 0]), color='red',
                                    alpha=0.3, label=_ring)
                else:
                    ax.fill(_utm_xys[:, 0], _utm_xys[:, 1], color='red', alpha=0.3, label=_ring)

        # 绘制设施点
        for _fac, _lnglat in self.facilities_info.items():
            if self.convert_to_utm:
                _utm_x, _utm_y = self.lnglat_converter.lon_lat_to_utm(_lnglat[0], _lnglat[1])
            else:
                _utm_x, _utm_y = _lnglat[0], _lnglat[1]

            if 'hq_' in _fac:
                _color = 'green';
                _marker = '^'
            elif 'ua_' in _fac:
                _color = 'red';
                _marker = 'o'
            elif 'radar_' in _fac:
                _color = 'blue';
                _marker = 's'

            if show_mode == '3D':
                ax.scatter(xs=[_utm_x], ys=[_utm_y], zs=[0], color=_color, marker=_marker, label=_fac)
                ax.text(_utm_x + 50, _utm_y + 50, 0, _fac, color='black')
            else:
                ax.scatter(x=[_utm_x], y=[_utm_y], color=_color, marker=_marker, label=_fac)
                ax.text(_utm_x + 50, _utm_y + 50, _fac, color='black')

        if show_borders:
            _probe_polyborder = self.get_probe_facilities_polyborder(union_polygons=False)

            if show_mode == '3D':
                for _iter, _poly_border in enumerate(_probe_polyborder):
                    ax.plot_trisurf(_poly_border[0], _poly_border[1], np.zeros_like(_poly_border[0]),
                                    color='blue', alpha=0.3, label='Radar')
            else:
                for _iter, _poly_border in enumerate(_probe_polyborder):
                    ax.fill(_poly_border[0], _poly_border[1], color='blue', alpha=0.3, label='Radar')

            _ext_rings_polyborder = self.get_defence_rings_polyborder(union_polygons=False)

            if show_mode == '3D':
                for _iter, _poly_border in enumerate(_ext_rings_polyborder):
                    ax.plot_trisurf(_poly_border[0], _poly_border[1], np.zeros_like(_poly_border[0]), color='red',
                                    alpha=0.3, label='Ext. Ring')
            else:
                for _iter, _poly_border in enumerate(_ext_rings_polyborder):
                    ax.fill(_poly_border[0], _poly_border[1], color='red', alpha=0.3, label='Ext. Ring')

            _antiair_polyborder = self.get_defence_facilities_polyborder(union_polygons=False)

            if show_mode == '3D':
                for _iter, _poly_border in enumerate(_antiair_polyborder):
                    ax.plot_trisurf(_poly_border[0], _poly_border[1], np.zeros_like(_poly_border[0]),
                                    color='yellow', alpha=0.3, label='Antiair')
            else:
                for _iter, _poly_border in enumerate(_antiair_polyborder):
                    ax.fill(_poly_border[0], _poly_border[1], color='yellow', alpha=0.3, label='Antiair')

        if show_ppath is not None:
            _paths_colors = [hls_to_rgb(i / len(show_ppath), 0.5, 0.8) for i in range(len(show_ppath))]

            if show_mode == '3D':
                for _p_iter, _path in enumerate(show_ppath):
                    _cur_xs = [_loc[0] for _loc in _path['trajectory']]
                    _cur_ys = [_loc[1] for _loc in _path['trajectory']]
                    _cur_zs = [_loc[2] for _loc in _path['trajectory']]

                    ax.plot(_cur_xs, _cur_ys, _cur_zs, linestyle='--', color=plt.cm.tab10(_p_iter % 10), linewidth=1,
                            label='Path')

                    if len(_cur_xs) > 2:
                        _cur_mid_idx = len(_cur_xs) // 2
                        x_mid = _cur_xs[_cur_mid_idx]
                        y_mid = _cur_ys[_cur_mid_idx]
                        z_mid = _cur_zs[_cur_mid_idx]
                    else:
                        x_mid = (_cur_xs[0] + _cur_xs[-1]) / 2
                        y_mid = (_cur_ys[0] + _cur_ys[-1]) / 2
                        z_mid = (_cur_zs[0] + _cur_zs[-1]) / 2

                    # 添加标签
                    ax.text(x_mid, y_mid, z_mid,
                            _path['type'],
                            color='black',
                            fontsize=5,
                            bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))
                    ax.set_zlim(-100, 300)

            elif show_mode == '2D':
                for _p_iter, _path in enumerate(show_ppath):
                    _cur_xs = [_loc[0] for _loc in _path['trajectory']]
                    _cur_ys = [_loc[1] for _loc in _path['trajectory']]
                    ax.plot(_cur_xs, _cur_ys, linestyle='--', color=plt.cm.tab10(_p_iter % 10), linewidth=1,
                            label='Path')

                    if len(_cur_xs) > 2:
                        _cur_mid_idx = len(_cur_xs) // 2
                        x_mid = _cur_xs[_cur_mid_idx]
                        y_mid = _cur_ys[_cur_mid_idx]
                    else:
                        x_mid = (_cur_xs[0] + _cur_xs[-1]) / 2
                        y_mid = (_cur_ys[0] + _cur_ys[-1]) / 2

                    ax.text(x_mid, y_mid,
                            _path['type'],
                            color='black',
                            fontsize=5,
                            bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

        if start_point is not None:
            if show_mode == '3D':
                ax.scatter(start_point[0], start_point[1], start_point[2], color='b', marker=_marker, label='start')
            else:
                ax.scatter(start_point[0], start_point[1], color='b', marker=_marker, label='start')

        def on_mouse_move(event):
            if event.inaxes:
                x = event.xdata
                y = event.ydata

                formatted_x = f"{x:.2f}" if x else "N/A"
                formatted_y = f"{y:.2f}" if y else "N/A"

                ax.set_title(f"X: {formatted_x}, Y: {formatted_y}")
                print(f"Current Point: ({x}, {y})")

                fig.canvas.draw_idle()

        # 调整边距，减小空白部分
        plt.tight_layout(pad=0)  # 自动紧凑布局

        if show_mode == '2D':
            ax.legend()

        ax.set_aspect('equal')
        ax.grid(True)
        fig.canvas.mpl_connect('button_press_event', on_mouse_move)

        plt.show()

        return "finished plot"

    def _plot_plan_path_2d(self, plans_paths, fleet_name, ax):
        for _p_iter, _path in enumerate(plans_paths):
            _cur_xs = [_loc[0] for _loc in _path['trajectory']]
            _cur_ys = [_loc[1] for _loc in _path['trajectory']]

            if self.convert_to_utm:
                _utm_xys = self.lnglat_converter.lng_lat_to_utm_array(np.array([_cur_xs, _cur_ys]).T)
                _cur_xs = _utm_xys[:, 0]
                _cur_ys = _utm_xys[:, 1]

            ax.plot(_cur_xs, _cur_ys, linestyle='--', color=plt.cm.tab10(_p_iter % 10), linewidth=1, label='Path')

            if len(_cur_xs) > 2:
                _cur_mid_idx = len(_cur_xs) // 2
                x_mid = _cur_xs[_cur_mid_idx]
                y_mid = _cur_ys[_cur_mid_idx]
            else:
                x_mid = (_cur_xs[0] + _cur_xs[-1]) / 2
                y_mid = (_cur_ys[0] + _cur_ys[-1]) / 2

            ax.text(x_mid, y_mid,
                    fleet_name + ':' + _path['type'],
                    color='black',
                    fontsize=5,
                    bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

    def _plot_plan_path_3d(self, plans_paths, fleet_name, ax):
        for _p_iter, _path in enumerate(plans_paths):
            _cur_xs = [_loc[0] for _loc in _path['trajectory']]
            _cur_ys = [_loc[1] for _loc in _path['trajectory']]
            _cur_zs = [_loc[2] for _loc in _path['trajectory']]

            if self.convert_to_utm:
                _utm_xys = self.lnglat_converter.lng_lat_to_utm_array(np.array([_cur_xs, _cur_ys]).T)
                _cur_xs = _utm_xys[:, 0]
                _cur_ys = _utm_xys[:, 1]

            ax.plot(_cur_xs, _cur_ys, _cur_zs, linestyle='--', color=plt.cm.tab10(_p_iter % 10), linewidth=1,
                    label='Path')

            if len(_cur_xs) > 2:
                _cur_mid_idx = len(_cur_xs) // 2
                x_mid = _cur_xs[_cur_mid_idx]
                y_mid = _cur_ys[_cur_mid_idx]
                z_mid = _cur_zs[_cur_mid_idx]
            else:
                x_mid = (_cur_xs[0] + _cur_xs[-1]) / 2
                y_mid = (_cur_ys[0] + _cur_ys[-1]) / 2
                z_mid = (_cur_zs[0] + _cur_zs[-1]) / 2

            ax.text(x_mid, y_mid, z_mid,
                    fleet_name + ':' + _path['type'],
                    color='black',
                    fontsize=5,
                    bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))

    def visualize_twisted_paths(self, twisted_paths, show_defend_rings=True, show_borders=False,
                                show_mode='3D', show_formation=False):
        fig = plt.figure(figsize=(10, 10))

        if show_mode == '3D':
            ax = fig.add_subplot(111, projection='3d', proj_type='ortho')
        else:
            ax = fig.add_subplot(111)

        # 绘制防御圈层
        if show_defend_rings:
            for _ring, _lnglats in self.defend_rings_info.items():
                if self.convert_to_utm:
                    _utm_xys = self.lnglat_converter.lng_lat_to_utm_array(
                        np.array([_lnglats['lngs'], _lnglats['lats']]).T)
                else:
                    _utm_xys = np.array([_lnglats['lngs'], _lnglats['lats']]).T

                if show_mode == '3D':
                    ax.plot_trisurf(_utm_xys[:, 0], _utm_xys[:, 1], np.zeros_like(_utm_xys[:, 0]), color='red',
                                    alpha=0.1, label=_ring)
                else:
                    ax.fill(_utm_xys[:, 0], _utm_xys[:, 1], color='red', alpha=0.1, label=_ring)

        # 绘制设施点
        for _fac, _lnglat in self.facilities_info.items():
            if self.convert_to_utm:
                _utm_x, _utm_y = self.lnglat_converter.lon_lat_to_utm(_lnglat[0], _lnglat[1])
            else:
                _utm_x, _utm_y = _lnglat[0], _lnglat[1]

            if _fac.startswith('hq_'):
                _color = 'green';
                _marker = '^'
                _border_radius = GlobalBasicConfigs.AVOID_HQ_DISTANCE

            elif _fac.startswith('ua_'):
                _color = 'red';
                _marker = 'o'
                _border_radius = GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE

            elif _fac.startswith('radar_'):
                _color = 'blue';
                _marker = 's'
                _border_radius = GlobalBasicConfigs.AVOID_RADAR_DISTANCE

            if show_borders:  # 绘制主要设施周围的躲避区域（圆形）
                _border_xys = self.get_spec_facility_polyborder([_utm_x, _utm_y], _border_radius, ll2utm=False)

            if show_mode == '3D':
                ax.scatter(xs=[_utm_x], ys=[_utm_y], zs=[0], s=30, color=_color, marker=_marker, edgecolors='black',
                           linewidth=1, label=_fac)
                ax.text(_utm_x + 25, _utm_y + 25, 0, _fac, color='black', fontsize=9,
                        bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))
                ax.plot_trisurf(_border_xys[0], _border_xys[1], np.zeros_like(_border_xys[:, 0]), color='green',
                                alpha=0.1)
            else:
                ax.scatter(x=[_utm_x], y=[_utm_y], s=30, color=_color, marker=_marker, edgecolors='black', linewidth=1,
                           label=_fac)
                ax.text(_utm_x + 25, _utm_y + 25, _fac, color='black', fontsize=9,
                        bbox=dict(facecolor='white', alpha=0.8, boxstyle='round'))
                ax.fill(_border_xys[0], _border_xys[1], color='green', alpha=0.1)

        for _plan in twisted_paths:
            if show_mode == '3D':
                if _plan['type'] == 'independent':
                    for _sub_plan in _plan['trajectories']:
                        _fleet_name = _sub_plan['fleet']
                        self._plot_plan_path_3d(_sub_plan['trajectory'], _fleet_name, ax)

                elif _plan['type'] == 'aggregate':
                    _agg_paths = _plan['aggregate_trajectories']
                    _agg_fleet_name = _plan['fleet']

                    for _fleet, _path in _agg_paths.items():
                        self._plot_plan_path_3d([_path], _agg_fleet_name, ax)

                elif _plan['type'] == 'disperse':
                    _disp_paths = _plan['disperse_trajectories']

                    for _fleet, _path in _disp_paths.items():
                        self._plot_plan_path_3d(_path, _fleet, ax)
            else:
                if _plan['type'] == 'independent':
                    for _sub_plan in _plan['trajectories']:
                        _fleet_name = _sub_plan['fleet']
                        self._plot_plan_path_2d(_sub_plan['trajectory'], _fleet_name, ax)

                elif _plan['type'] == 'aggregate':
                    _agg_paths = _plan['aggregate_trajectories']
                    _agg_fleet_name = _plan['fleet']

                    for _fleet, _path in _agg_paths.items():
                        self._plot_plan_path_2d([_path], _agg_fleet_name, ax)

                elif _plan['type'] == 'disperse':
                    _disp_paths = _plan['disperse_trajectories']

                    for _fleet, _path in _disp_paths.items():
                        self._plot_plan_path_2d(_path, _fleet, ax)

        if show_formation:
            for _plan in twisted_paths:
                if _plan['type'] == 'independent':
                    for _sub_plan in _plan['trajectories']:
                        for _sub_traj in _sub_plan['trajectory']:
                            if _sub_traj['member_paths'] is not None:
                                for _member_traj in _sub_traj['member_paths']:
                                    for _loc in _member_traj:
                                        ax.scatter(_loc[0], _loc[1], _loc[2], color='red', marker='o', s=1)

                elif _plan['type'] == 'aggregate':
                    _agg_paths = _plan['aggregate_trajectories']
                    for _fleet, _path in _agg_paths.items():
                        if _path['member_paths'] is not None:
                            for _member_traj in _path['member_paths']:
                                for _loc in _member_traj:
                                    ax.scatter(_loc[0], _loc[1], _loc[2], color='green', marker='o', s=1)

                elif _plan['type'] == 'disperse':
                    _disp_paths = _plan['disperse_trajectories']
                    for _fleet, _path in _disp_paths.items():
                        for _sub_traj in _path:
                            if _sub_traj['member_paths'] is not None:
                                for _member_traj in _sub_traj['member_paths']:
                                    for _loc in _member_traj:
                                        ax.scatter(_loc[0], _loc[1], _loc[2], color='blue', marker='o', s=1)

        plt.tight_layout(pad=1)  # 自动紧凑布局
        # ax.legend()
        ax.set_aspect('equal')
        ax.grid(True)
        plt.show()

    def animation_visualize_twisted_paths(self, twisted_paths, show_defend_rings=True, show_borders=False,
                                          show_mode='3D', show_formation=False):
        fig = plt.figure(figsize=(10, 10))

        if show_mode == '3D':
            ax = fig.add_subplot(111, projection='3d', proj_type='ortho')
        else:
            ax = fig.add_subplot(111)

        # 绘制防御圈层
        if show_defend_rings:
            for _ring, _lnglats in self.defend_rings_info.items():
                if self.convert_to_utm:
                    _utm_xys = self.lnglat_converter.lng_lat_to_utm_array(
                        np.array([_lnglats['lngs'], _lnglats['lats']]).T)
                else:
                    _utm_xys = np.array([_lnglats['lngs'], _lnglats['lats']]).T

                if show_mode == '3D':
                    ax.plot_trisurf(_utm_xys[:, 0], _utm_xys[:, 1], np.zeros_like(_utm_xys[:, 0]), color='red',
                                    alpha=0.1, label=_ring)
                else:
                    ax.fill(_utm_xys[:, 0], _utm_xys[:, 1], color='red', alpha=0.1, label=_ring)

        # 绘制设施点
        for _fac, _lnglat in self.facilities_info.items():
            if self.convert_to_utm:
                _utm_x, _utm_y = self.lnglat_converter.lon_lat_to_utm(_lnglat[0], _lnglat[1])
            else:
                _utm_x, _utm_y = _lnglat[0], _lnglat[1]

            if _fac[:3] == 'hq_':
                _color = 'green';
                _marker = '^'
            elif _fac[:3] == 'ua_':
                _color = 'red';
                _marker = 'o'
            elif _fac[:6] == 'radar_':
                _color = 'blue';
                _marker = 's'

            ax.scatter(_utm_x, _utm_y, 0, color=_color, marker=_marker, edgecolors='black', linewidth=1, label=_fac)

        members_num = 0
        agg_members_num = 0
        _pre_agg_num = 0
        _atf_aff_num = 0
        _pre_num_time_steps = 0
        indenpendent_paths = []
        aggregate_paths = []
        disperse_paths = []
        members_path = []
        agg_members_path = []
        disperse_paths = []
        for _plan in twisted_paths:
            if _plan['type'] == 'independent':
                for _sub_plan in _plan['trajectories']:
                    _fleet_name = _sub_plan['fleet']
                    full_path = []
                    members_full_path = []
                    for _idx, _traj in enumerate(_sub_plan['trajectory']):
                        if _idx == 0:
                            members_num = len(_traj['member_paths'])
                            full_path.extend(_traj['trajectory'])
                        else:
                            full_path.extend(_traj['trajectory'])

                        # 获取轨迹点数量（假设所有从机轨迹长度一致）
                        num_time_steps = len(_traj['member_paths'][0])  # 每架从机的轨迹点数量
                        num_drones = len(_traj['member_paths'])  # 从机数量
                        # 构造新的轨迹格式
                        uav_traj = []
                        for t in range(num_time_steps):
                            time_step_positions = []
                            for d in range(num_drones):
                                time_step_positions.append(_traj['member_paths'][d][t])
                            members_full_path.append(time_step_positions)

                        # members_full_path.append(uav_traj)
                    indenpendent_paths.append(
                        {
                            _fleet_name: full_path
                        })
                    members_path.append(
                        {
                            _fleet_name: members_full_path
                        })
            elif _plan['type'] == 'aggregate':
                _agg_paths = _plan['aggregate_trajectories']
                _agg_fleet_name = _plan['fleets']
                for _idx, (_fleet, _path) in enumerate(_agg_paths.items()):
                    members_full_path = []
                    if _idx == 0:
                        agg_members_num = len(_path['member_paths'])
                        _pre_agg_num = agg_members_num // 3
                        _atf_aff_num = agg_members_num - _pre_agg_num
                        print(
                            f"agg_members_num: {agg_members_num}, _pre_agg_num: {_pre_agg_num}, _atf_aff_num: {_atf_aff_num}")
                        _pre_num_time_steps = len(_path['member_paths'][0])  # 每架从机的轨迹点数量
                        _pre_num_drones = _pre_agg_num  # 从机数量
                        # 构造新的轨迹格式
                        for t in range(_pre_num_time_steps):
                            time_step_positions = []
                            for d in range(_pre_num_drones):
                                time_step_positions.append(_path['member_paths'][d][t])
                            members_full_path.append(time_step_positions)
                        _atf_num_time_steps = len(_path['member_paths'][-1])  # 每架从机的轨迹点数量
                        _atf_num_drones = _atf_aff_num  # 从机数量
                        for t in range(_atf_num_time_steps):
                            time_step_positions = []
                            for d in range(_pre_num_drones, agg_members_num):
                                time_step_positions.append(_path['member_paths'][d][t])
                            members_full_path.append(time_step_positions)
                        aggregate_paths.append({
                            _fleet: members_full_path
                        })

                    else:
                        num_time_steps = len(_path['member_paths'][0])  # 每架从机的轨迹点数量
                        num_drones = len(_path['member_paths'])  # 从机数量
                        # 构造新的轨迹格式
                        uav_traj = []
                        for t in range(num_time_steps):
                            time_step_positions = []
                            for d in range(num_drones):
                                time_step_positions.append(_path['member_paths'][d][t])
                            members_full_path.append(time_step_positions)
                        aggregate_paths.append({
                            _fleet: members_full_path
                        })

                    for _traj in indenpendent_paths:
                        for fleet_name, path in _traj.items():
                            if fleet_name == _fleet:
                                for sublist in _path['trajectory']:
                                    path.append(sublist)

            elif _plan['type'] == 'disperse':
                _disp_paths = _plan['disperse_trajectories']
                for _fleet_idx, (_fleet, _path) in enumerate(_disp_paths.items()):
                    full_path = []
                    members_full_path = []
                    for _idx, _traj in enumerate(_path):
                        if len(_path) > 1:
                            if _idx == 0:
                                _path_list = _traj['trajectory']
                            else:
                                _path_list = _traj['trajectory']
                            full_path.extend(_path_list)
                        else:
                            full_path.extend(_traj['trajectory'])

                        # 获取轨迹点数量（假设所有从机轨迹长度一致）
                        num_time_steps = len(_traj['member_paths'][0])  # 每架从机的轨迹点数量
                        num_drones = len(_traj['member_paths'])  # 从机数量
                        for t in range(num_time_steps):
                            time_step_positions = []
                            for d in range(num_drones):
                                time_step_positions.append(_traj['member_paths'][d][t])
                            members_full_path.append(time_step_positions)
                    disperse_paths.append({
                        _fleet: members_full_path
                    })
                    for _idx, _traj in enumerate(indenpendent_paths):
                        for fleet_name, path in _traj.items():
                            if _fleet_idx == _idx:
                                for sublist in full_path:
                                    path.append(sublist)

        print(f"Independent paths: {json.dumps(indenpendent_paths, indent=4)}")

        print(f"members_path: {json.dumps(members_path, indent=4)}")

        print(f"aggregate_paths: {json.dumps(aggregate_paths, indent=4)}")

        print(f"disperse_paths: {json.dumps(disperse_paths, indent=4)}")
        # # 提取三维轨迹数据
        fleet1_path = np.array(indenpendent_paths[0]["fleet1"])
        fleet2_path = np.array(indenpendent_paths[1]["fleet2"])

        # 初始化轨迹线和当前位置标记
        line1, = ax.plot([], [], [], 'r--', lw=1, label='Fleet1 Path')
        line2, = ax.plot([], [], [], 'b--', lw=1, label='Fleet2 Path')
        point1, = ax.plot([], [], [], 'ro', markersize=3, label='Fleet1 Current')
        point2, = ax.plot([], [], [], 'bo', markersize=3, label='Fleet2 Current')
        members1_points = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(members_num)]
        members2_points = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(members_num)]
        agg_members_line = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(_atf_aff_num)]
        agg_members_line1 = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(_pre_agg_num)]
        agg_members_line2 = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(_pre_agg_num)]
        disperse_members1_line = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(members_num)]
        disperse_members2_line = [ax.plot([], [], [], 'o', markersize=2)[0] for _ in range(members_num)]

        #
        # 初始化函数
        def init():
            line1.set_data_3d([], [], [])
            line2.set_data_3d([], [], [])
            point1.set_data_3d([], [], [])
            point2.set_data_3d([], [], [])
            for point in members1_points:
                point.set_data_3d([], [], [])
            for point in members2_points:
                point.set_data_3d([], [], [])
            for line in agg_members_line:
                line.set_data_3d([], [], [])
            for line in agg_members_line1:
                line.set_data_3d([], [], [])
            for line in agg_members_line2:
                line.set_data_3d([], [], [])
            for line in disperse_members1_line:
                line.set_data_3d([], [], [])
            for line in disperse_members2_line:
                line.set_data_3d([], [], [])
            return line1, line2, point1, point2, members1_points, members2_points, agg_members_line, agg_members_line1, agg_members_line2, disperse_members1_line, disperse_members2_line

        #
        # 更新函数（每一帧调用）
        def update(frame):
            # 更新轨迹线（三维数据）
            line1.set_data_3d(fleet1_path[:frame + 1, 0], fleet1_path[:frame + 1, 1], fleet1_path[:frame + 1, 2])
            line2.set_data_3d(fleet2_path[:frame + 1, 0], fleet2_path[:frame + 1, 1], fleet2_path[:frame + 1, 2])

            # 更新当前位置标记
            point1.set_data_3d([fleet1_path[frame, 0]], [fleet1_path[frame, 1]], [fleet1_path[frame, 2]])
            point2.set_data_3d([fleet2_path[frame, 0]], [fleet2_path[frame, 1]], [fleet2_path[frame, 2]])
            if frame < len(members_path[0]["fleet1"]):
                for _traj, _path in enumerate(members_path):
                    for fleet_name, path in _path.items():
                        for _idx, _step in enumerate(path):
                            if _idx == frame:
                                for _num, _uav in enumerate(_step):
                                    if _traj == 0:
                                        members1_points[_num].set_data_3d([round(_uav[0], 2)], [round(_uav[1], 2)],
                                                                          [round(_uav[2], 2)])
                                    else:
                                        members2_points[_num].set_data_3d([round(_uav[0], 2)], [round(_uav[1], 2)],
                                                                          [round(_uav[2], 2)])
            elif frame < len(members_path[0]["fleet1"]) + len(aggregate_paths[0]["fleet1"]):
                for point in members1_points:
                    point.set_data_3d([], [], [])
                for point in members2_points:
                    point.set_data_3d([], [], [])
                for _traj, _path in enumerate(aggregate_paths):
                    for fleet_name, path in _path.items():
                        for _idx, _step in enumerate(path[: _pre_num_time_steps]):
                            if _idx == frame - len(members_path[0]["fleet1"]):
                                for _num, _uav in enumerate(_step):
                                    if _traj == 0:
                                        agg_members_line1[_num].set_data_3d([round(_uav[0], 2)], [round(_uav[1], 2)],
                                                                            [round(_uav[2], 2)])
                                    else:
                                        agg_members_line2[_num].set_data_3d([round(_uav[0], 2)], [round(_uav[1], 2)],
                                                                            [round(_uav[2], 2)])
                for _traj, _path in enumerate(aggregate_paths):
                    for fleet_name, path in _path.items():
                        for _idx, _step in enumerate(path[_pre_num_time_steps:]):
                            if _idx == frame - len(members_path[0]["fleet1"]) - _pre_num_time_steps:
                                for line in agg_members_line1:
                                    line.set_data_3d([], [], [])
                                for line in agg_members_line2:
                                    line.set_data_3d([], [], [])
                                for _num, _uav in enumerate(_step):
                                    if _traj == 0:
                                        agg_members_line[_num].set_data_3d([round(_uav[0], 2)], [round(_uav[1], 2)],
                                                                           [round(_uav[2], 2)])
            else:
                for line in agg_members_line:
                    line.set_data_3d([], [], [])
                for line in agg_members_line1:
                    line.set_data_3d([], [], [])
                for line in agg_members_line2:
                    line.set_data_3d([], [], [])
                for _traj, _path in enumerate(disperse_paths):
                    for fleet_name, path in _path.items():
                        for _idx, _step in enumerate(path):
                            if _idx == frame - len(members_path[0]["fleet1"]) - len(aggregate_paths[0]["fleet1"]):
                                for _num, _uav in enumerate(_step):
                                    if _traj == 0:
                                        disperse_members1_line[_num].set_data_3d([round(_uav[0], 2)],
                                                                                 [round(_uav[1], 2)],
                                                                                 [round(_uav[2], 2)])
                                    else:
                                        disperse_members2_line[_num].set_data_3d([round(_uav[0], 2)],
                                                                                 [round(_uav[1], 2)],
                                                                                 [round(_uav[2], 2)])

            # 添加步数标注（显示在3D坐标系中）
            ax.text2D(0.02, 0.95, f'Step: {frame}', transform=ax.transAxes, fontsize=12,
                      bbox=dict(boxstyle="round", fc="white"))

            # # 动态调整视角（可选，模拟飞行视角）
            # ax.view_init(elev=30, azim=frame * 2)  # 每帧旋转2度

            return line1, line2, point1, point2, members1_points, members2_points, agg_members_line, agg_members_line1, agg_members_line2

        # 创建动画
        ani = FuncAnimation(
            fig=fig,
            func=update,
            frames=len(fleet1_path),  # 总帧数
            init_func=init,
            blit=False,
            interval=100,  # 每帧1秒
            repeat=False
        )

        # if show_formation:
        #     for _plan in twisted_paths:
        #         if _plan['type'] == 'independent':
        #             for _sub_plan in _plan['trajectories']:
        #                 for _sub_traj in _sub_plan['trajectory']:
        #                     if _sub_traj['member_paths'] is not None:
        #                         for _member_traj in _sub_traj['member_paths']:
        #                             for _loc in _member_traj:
        #                                 ax.scatter(_loc[0], _loc[1], _loc[2], color='red', marker='o', s=1)
        #
        #         elif _plan['type'] == 'aggregate':
        #             _agg_paths = _plan['aggregate_trajectories']
        #             for _fleet, _path in _agg_paths.items():
        #                 if _path['member_paths'] is not None:
        #                     for _member_traj in _path['member_paths']:
        #                         for _loc in _member_traj:
        #                             ax.scatter(_loc[0], _loc[1], _loc[2], color='green', marker='o', s=1)
        #
        #         elif _plan['type'] == 'disperse':
        #             _disp_paths = _plan['disperse_trajectories']
        #             for _fleet, _path in _disp_paths.items():
        #                 for _sub_traj in _path:
        #                     if _sub_traj['member_paths'] is not None:
        #                         for _member_traj in _sub_traj['member_paths']:
        #                             for _loc in _member_traj:
        #                                 ax.scatter(_loc[0], _loc[1], _loc[2], color='blue', marker='o', s=1)

        plt.tight_layout(pad=1)  # 自动紧凑布局
        # ax.legend()
        ax.set_aspect('equal')
        ax.grid(True)
        plt.show()
        ani.save(
            "animation.gif",  # 保存路径
            writer="pillow",  # 指定GIF写入器
            fps=10,  # 帧率（每秒帧数，需与interval对应）
            dpi=100  # 分辨率
        )

    @staticmethod
    def convert_api_facilities_str(facs_str_in):
        if facs_str_in is None:
            return None

        if isinstance(facs_str_in, str):
            facs_info = json.loads(facs_str_in)
        else:
            facs_info = facs_str_in

        _convert_info = {'facilities_str': {},
                         'defence_rings': {}}

        for _fac in facs_info:
            if 'ring' in _fac.lower():
                _convert_info['defence_rings'][_fac] = {'lngs': facs_info[_fac][:-1:2],
                                                        'lats': facs_info[_fac][1::2]}
            else:
                _convert_info['facilities_str'][_fac] = facs_info[_fac]

        return _convert_info


class PlanGraphAnalyzer:
    ''' 用于分析轨迹路径的图结构，包括路径长度、路径覆盖率、路径连通性等
    '''

    def __init__(self, plan_graph, star_nodes, stop_nodes, crucial_routes=None):
        self.plan_graph = plan_graph
        self.star_nodes = star_nodes
        self.stop_nodes = stop_nodes
        self.crucial_routes = crucial_routes

    def save_data(self, file_path):
        _save_data = {'graph': self.plan_graph,
                      'start_nodes': self.star_nodes,
                      'stop_nodes': self.stop_nodes,
                      'crucial_routes': self.crucial_routes}

        with open(file_path, 'wb') as f:
            pkl.dump(_save_data, f)

    def load_data(self, file_path):
        with open(file_path, 'rb') as f:
            _save_data = pkl.load(f)
            self.plan_graph = _save_data['graph']
            self.star_nodes = _save_data['start_nodes']
            self.stop_nodes = _save_data['stop_nodes']
            self.crucial_routes = _save_data['crucial_routes']

    def plot_graph(self):
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        _draw_pos = nx.spring_layout(self.plan_graph)

        _draw_labels = {}
        for _node in self.plan_graph.nodes:
            if _node in self.star_nodes:
                _draw_labels[_node] = '%d:Beg' % _node
            elif _node in self.stop_nodes:
                _draw_labels[_node] = '%d:End' % _node
            else:
                _draw_labels[_node] = '%d:Nrm' % _node

        nx.draw(self.plan_graph, pos=_draw_pos, node_color='orange', node_size=200, ax=ax)
        nx.draw_networkx_labels(self.plan_graph, pos=_draw_pos, labels=_draw_labels, font_size=10, font_color='black',
                                ax=ax)

        plt.show()

    def count_crucial_routes(self, return_routes=False):
        if self.crucial_routes is None:
            return {}

        _routes_sorted = {}
        _routes_count = {}

        for _route in self.crucial_routes:
            _cur_start_node = _route[0]
            _cur_stop_node = _route[-1]

            if not (f"{_cur_start_node}-{_cur_stop_node}") in _routes_sorted:
                _routes_sorted[f"{_cur_start_node}-{_cur_stop_node}"] = [_route]
                _routes_count[f"{_cur_start_node}-{_cur_stop_node}"] = 1
            else:
                _routes_sorted[f"{_cur_start_node}-{_cur_stop_node}"].append(_route)
                _routes_count[f"{_cur_start_node}-{_cur_stop_node}"] += 1

        if not return_routes:
            return _routes_count
        else:
            return _routes_count, _routes_sorted
