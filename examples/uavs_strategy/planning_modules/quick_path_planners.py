import os, os.path as osp
import json
from functools import cmp_to_key
import numpy as np

from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points
from shapely.geometry.polygon import orient

import matplotlib.pyplot as plt
import matplotlib.cm as cm


from examples.uavs_strategy.planning_modules import basic_functions as bfunc
import copy
from collections import defaultdict

class SimpleBorders:
    ''' 一个基于防御圈层等各种典型区域多边形，生成轨迹路径的辅助功能类
    '''

    def __init__(self, border_poly):
        self.border_xs = border_poly[0]
        self.border_ys = border_poly[1]

        self.border_poly = Polygon([(_x, _y) for _x, _y in zip(self.border_xs, self.border_ys)])

    def is_inside_border(self, point):
        ''' 判断给定点是否在边界内
        '''
        return self.border_poly.contains(Point(point))

    def get_nearest_bordpoint(self, point):
        ''' 获取给定点的最近边界点
        '''
        _nearest_point = nearest_points(self.border_poly, Point(point))[1]
        return _nearest_point.coords[0]

    def get_nearest_border_vertex(self, point, border_xys=None, return_index=False):
        ''' 获取给定点的最近边界顶点
        '''
        if border_xys is None:
            border_xys = self.border_poly.exterior.xy

        _nearest_index = np.argmin(
            np.linalg.norm(np.array(border_xys).T - np.array(point), axis=1))
        _nearest_vertex = Point((border_xys[0][_nearest_index], border_xys[1][_nearest_index]))

        if not return_index:
            return _nearest_vertex
        else:
            return _nearest_vertex, _nearest_index

    def move_along_border(self, point, steps=3, direction='clockwise', vis_check=False):
        ''' 沿着边界移动，返回移动后的点
        '''
        if direction == 'clockwise':
            _oriented_border = orient(self.border_poly, sign=-1)
        else:
            _oriented_border = orient(self.border_poly, sign=1)

        _intercept_point, _vertex_index = self.get_nearest_border_vertex(point, _oriented_border.exterior.xy, return_index=True)

        _oriented_border_coords = list(_oriented_border.exterior.coords)
        _poly_vertices = _oriented_border_coords[:-1]

        _detour_vertices_indexes = [(_vertex_index + _iter) % len(_poly_vertices) for _iter in range(steps)]
        _detour_vertices = [_oriented_border_coords[_index] for _index in _detour_vertices_indexes]

        vis_check = False
        if vis_check:
            fig, axis = plt.subplots(1, 1, figsize=(10, 10))
            axis.plot(self.border_xs, self.border_ys, color='blue', linestyle='--')
            axis.plot([_coord[0] for _coord in _detour_vertices], [_coord[1] for _coord in _detour_vertices],
                      color='red', linestyle='-')
            plt.show()

        _detour_coords = [point] + _detour_vertices
        vis_check = False
        if vis_check:
            fig, axis = plt.subplots(1, 1, figsize=(10, 10))
            axis.plot(self.border_xs, self.border_ys, color='blue', linestyle='--')
            axis.plot([_coord[0] for _coord in _detour_coords], [_coord[1] for _coord in _detour_coords], color='red', linestyle='-')
            plt.show()

        return _detour_coords


class PlanPathTrimmer:
    ''' 用于对生成的轨迹路径进行修剪，使其符合实际飞行路径（例如：只输出规划轨迹的前60%分段）
    '''

    def __init__(self, plan_path):
        self.plan_path = plan_path

    def trim_plan_path(self, trim_ratio=0.6, trim_mode='from_head', vis=False):
        ''' 修剪轨迹路径，仅保留当前路径的前trim_ratio部分
        '''
        _path_coords = self.plan_path['trajectory']

        if len(_path_coords) <= 2:
            # 当前路径为一条直线
            _trimmed_coords = self._cut_line_segment(_path_coords[0], _path_coords[1], trim_ratio, trim_mode)
        else:
            # 当前路径为多端折线
            _trimmed_coords = self._cut_polyline_segment(_path_coords, trim_ratio, trim_mode)

        if vis:
            fig, axis = plt.subplots(1, 1, figsize=(10, 10))
            axis.plot([_coord[0] for _coord in _path_coords], [_coord[1] for _coord in _path_coords], color='blue', linestyle='--')
            axis.plot([_coord[0] for _coord in _trimmed_coords], [_coord[1] for _coord in _trimmed_coords], color='red', linestyle='-')
            axis.set_aspect('equal')
            plt.show()

        return _trimmed_coords

    def _cut_line_segment(self, start_point, end_point, cut_ratio, trim_mode='from_head'):
        ''' 切割线段
        '''
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]

        if trim_mode == 'from_head':
            return start_point, (start_point[0] + dx * cut_ratio, start_point[1] + dy * cut_ratio)
        elif trim_mode == 'to_tail':
            return (start_point[0] + dx * cut_ratio, start_point[1] + dy * cut_ratio), end_point

    def _cut_polyline_segment(self, polyline_coords, cut_ratio, trim_mode='from_head'):
        ''' 切割折线段
        '''
        _segments = list(zip(polyline_coords[:-1], polyline_coords[1:]))
        _lengths = np.linalg.norm(np.array(_segments)[:, 0, :] - np.array(_segments)[:, 1, :], axis=1)

        _cumulative_lengths = np.cumsum(_lengths)
        _cut_length = _cumulative_lengths[-1] * cut_ratio  # 需要切割的总里程长度

        _accum_length = 0.0
        _accum_polyline = [polyline_coords[0]]

        for _i in range(len(_segments)):
            if (_accum_length <= _cut_length) and (_cut_length <= _accum_length + _lengths[_i]):
                _cur_cut_ratio = (_cut_length - _accum_length) / _lengths[_i]
                _, _cut_endpoint = self._cut_line_segment(_segments[_i][0], _segments[_i][1], _cur_cut_ratio)
                _cut_segment_index = _i
                _accum_polyline.append(_cut_endpoint)
                break
            else:
                _accum_polyline.append(_segments[_i][1])
                _accum_length += _lengths[_i]

        if trim_mode == 'from_head':
            return _accum_polyline
        elif trim_mode == 'to_tail':
            _tt_accum_polyline = [_accum_polyline[-1]]

            for _i in range(_cut_segment_index, len(_segments)):
                _tt_accum_polyline.append(_segments[_i][1])

            return _tt_accum_polyline

class SimpleOrdersPlanner:
    ''' 用于生成突防、迂回的轨迹规划点
        输入的order序列中每个order的参数如下：
        order = {'type': 'breakthrough/escape/detour', 'target': 'ua_x'}
        [order, order, order]
        输出的plan序列中每个plan包含的参数如下：
        plan = {'type': 'breakthrough/escape/detour', 'target': 'ua_x', 'trajectory': [keypoint, keypoint, keypoint, ...]}
        [plan, plan, plan]
    '''

    def __init__(self, init_location, orders, facilities=None):
        self.init_location = init_location
        self.orders = orders
        self.plans = []

        if facilities is None:
            self.facilities = self._default_facilities()
        else:
            self.facilities = facilities

    def _default_facilities(self):
        _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')

        with open(_facilities_info_json, 'r') as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(_facilities_info['facilities_str'], _facilities_info['defence_rings'])

    def generate_trajectory_plans(self, init_location=None, orders=None, vis_check=False):
        if init_location is None:
            _cur_location = tuple(self.init_location)
        else:
            _cur_location = tuple(init_location)

        if orders is None:
            _orders = self.orders
        else:
            _orders = orders

        _plans = []

        for _order in _orders:
            if _order['type'] == 'breakthrough':
                if isinstance(_order['target'], str) and \
                    (_order['target'] in bfunc.GlobalBasicConfigs.PLANNING_BREAKTHROUGH_FACILITY_TYPES):
                    _plan = self._plan_breakthrough_targettype(_cur_location, _order['target'])
                elif isinstance(_order['target'], str) and (_order['target'] in self.facilities.get_facilities_names()):
                    _plan = self._plan_breakthrough_target(_cur_location, _order['target'])
                elif isinstance(_order['target'], tuple) or isinstance(_order['target'], list):
                    _plan = self._plan_breakthrough_location(_cur_location, _order['target'])

            elif _order['type'] == 'escape':
                _plan = self._plan_escape(_cur_location, _order['target'])

            elif _order['type'] == 'detour':
                _plan = self._plan_detour(_cur_location, _order['target'])

            if 'routed_ratio' in _order:  # 如果需要对规划的航路进行切割
                _path_trimmer = PlanPathTrimmer(_plan)
                _plan['trajectory'] = _path_trimmer.trim_plan_path(_order['routed_ratio'])
            if 'formation' in _order.keys():
                _plan["formation"] = _order["formation"]
            _cur_location = _plan['trajectory'][-1]
            _plans.append(_plan)

        self.plans = _plans
        return _plans

    def _plan_breakthrough_target(self, init_location, target):
        _target_location = self.facilities.get_target_location(target)
        return {'type': 'breakthrough', 'target': target, 'trajectory': [init_location, _target_location]}

    def _plan_breakthrough_targettype(self, init_location, facility_type):
        _target_info = self.facilities.pick_random_target(facility_type)
        return {'type': 'breakthrough', 'target': facility_type,
                'trajectory': [init_location, _target_info['location']]}

    def _plan_breakthrough_location(self, init_location, location):
        return {'type': 'breakthrough', 'target': location, 'trajectory': [init_location, location]}

    def _plan_escape(self, init_location, target='defence_rings'):
        # 逃逸主要是逃出下面几种典型的空域范围，逃到下列空域范围最近的边界点
        if target in self.facilities.facilities_info.keys() \
            or target in self.facilities.defend_rings.keys():

            if target in self.facilities.antiairs:
                _escape_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.antiairs[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE, ll2utm=False)
            elif target in self.facilities.headquartors:
                _escape_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.headquartors[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_HQ_DISTANCE, ll2utm=False)
            elif target in self.facilities.probers:
                _escape_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.probers[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_RADAR_DISTANCE, ll2utm=False)
            elif target in self.facilities.defend_rings.keys():
                _escape_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'defence_rings':
            _escape_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'probe_facilities':
            _escape_polygon_xys = self.facilities.get_probe_facilities_polyborder()

        elif target == 'antiair_facilities':
            _escape_polygon_xys = self.facilities.get_defence_facilities_polyborder()

        _border = SimpleBorders(_escape_polygon_xys[0])

        if not _border.is_inside_border(init_location):
            # 如果当前坐标本来就在逃逸范围之外，那么则直接返回当前坐标
            return {'type': 'escape', 'target': target, 'trajectory': [init_location, init_location]}
        else:
            # 否则，则计算当前坐标到逃逸范围的最近边界点，并返回
            # _nearest_point = _border.get_nearest_border_point(init_location)
            _nearest_point = _border.get_nearest_border_vertex(init_location)
            return {'type': 'escape', 'target': target, 'trajectory': [init_location, _nearest_point.coords[0]]}

    def _plan_detour(self, init_location, target='defence_rings', detour_steps=5):
        if target in self.facilities.facilities_info.keys() \
            or target in self.facilities.defend_rings.keys():

            if target in self.facilities.antiairs:
                _detour_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.antiairs[target],
                                                                               bfunc.GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE, ll2utm=False)
            elif target in self.facilities.headquartors:
                _detour_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.headquartors[target],
                                                                               bfunc.GlobalBasicConfigs.AVOID_HQ_DISTANCE, ll2utm=False)
            elif target in self.facilities.probers:
                _detour_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.probers[target],
                                                                               bfunc.GlobalBasicConfigs.AVOID_RADAR_DISTANCE, ll2utm=False)
            elif target in self.facilities.defend_rings.keys():
                _detour_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'defence_rings':
            _detour_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'probe_facilities':
            _detour_polygon_xys = self.facilities.get_probe_facilities_polyborder()

        elif target == 'antiair_facilities':
            _detour_polygon_xys = self.facilities.get_defence_facilities_polyborder()
        print(f"detour_polygon_xys: {_detour_polygon_xys}")
        _border = SimpleBorders(_detour_polygon_xys[0])
        _traj_locations = _border.move_along_border(init_location, steps=detour_steps,
                                                    direction=np.random.choice(['clockwise', 'anticlockwise']))

        return {'type': 'detour', 'target': target, 'trajectory': _traj_locations}

class SimpleOrdersTwistor(SimpleOrdersPlanner):
    ''' 综合性行动计划的路径规划功能类，可以为涉及多个编队的任务生成轨迹
    '''
    def __init__(self, init_location, orders, facilities=None):
        super().__init__(init_location, orders, facilities)

    def twist_trajectory_plans(self, init_location=None, orders=None, vis_check=False, convert_to_llt=False):
        if init_location is not None:
            _cur_location = init_location
        else:
            _cur_location = self.init_location

        if orders is not None:
            _orders = orders
        else:
            _orders = self.orders

        _plans = []
        for _order in _orders:
            if _order['type'] == 'independent':
                _cur_indep_plan = {'type': 'independent', 'trajectories': []}

                for _plan in _order['plans']:
                    _cur_fleet_name = _plan['fleet']
                    _cur_init_location = _cur_location[_cur_fleet_name]
                    _cur_sub_orders = _plan['orders']

                    _cur_sub_plans = self.generate_trajectory_plans(_cur_init_location, _cur_sub_orders, vis_check=vis_check)
                    _cur_indep_plan['trajectories'].append({'fleet': _cur_fleet_name, 'trajectory': _cur_sub_plans})

                    _cur_location[_cur_fleet_name] = _cur_sub_plans[-1]['trajectory'][-1]

                _plans.append(_cur_indep_plan)

            elif _order['type'] == 'aggregate':
                _cur_init_fleets = _order['fleets']
                _cur_aggregate_fleet = _order['fleet']
                _cur_agg_plan = {'type': 'aggregate',
                                 'fleets': _cur_init_fleets,
                                 'fleet': _cur_aggregate_fleet,
                                 'aggregate_trajectories': {}}

                _cur_init_locations = {_fleet:self.init_location[_fleet] for _fleet in _cur_init_fleets}

                _aggr_section, _tgtr_section = \
                    self._plan_aggregate_trajectories(_cur_init_locations, _order['order'], aggr_ratio=_order['aggregate_ratio'], vis_check=vis_check)

                _cur_aggr_trajs = {_fleet: {'type': _order['order']['type'],
                                            'target': _order['order']['target'],
                                            "formation": _order['order']["formation"] if "formation" in _order['order'].keys() else [],
                                            'trajectory': _aggr_section[_fleet]} for _fleet in _aggr_section}

                for _fleet in _cur_aggr_trajs.keys():
                    _cur_aggr_trajs[_fleet]['trajectory'].extend(_tgtr_section[1:])
                _cur_agg_plan['aggregate_trajectories'] = _cur_aggr_trajs

                _cur_location[_cur_aggregate_fleet] = _tgtr_section[-1]

                _plans.append(_cur_agg_plan)

            elif _order['type'] == 'disperse':
                _cur_aggregate_fleet = _order['fleet']
                _cur_dispr_plan = {'type': 'disperse',
                                   'fleet': _cur_aggregate_fleet,
                                   'fleets': _order['fleets'],
                                   'disperse_trajectories': {}}

                for _plan in _order['plans']:
                    _cur_fleet_name = _plan['fleet']
                    _cur_init_location = _cur_location[_cur_aggregate_fleet]
                    _cur_sub_orders = _plan['orders']

                    _cur_sub_plans = self.generate_trajectory_plans(_cur_init_location, _cur_sub_orders, vis_check=vis_check)
                    _cur_dispr_plan['disperse_trajectories'][_cur_fleet_name] = _cur_sub_plans

                    _cur_location[_cur_fleet_name] = _cur_sub_plans[-1]['trajectory'][-1]

                _plans.append(_cur_dispr_plan)

        self.plans = _plans

        return _plans

    def _plan_aggregate_trajectories(self, init_locations, order, aggr_ratio=0.5, vis_check=False):
        # 首先生成从多机集合中心出发的航路规划点
        _geometry_center_location = np.mean(np.array([_coords for _coords in init_locations.values()]), axis=0)
        _cur_center2end_fullpath = self.generate_trajectory_plans(_geometry_center_location, [order], vis_check=vis_check)

        # 根据任务重的routed_ratio对生成的轨迹进行裁剪
        _trimmed_center2end_coords = PlanPathTrimmer(_cur_center2end_fullpath[0]).trim_plan_path(order['routed_ratio'], trim_mode='from_head')
        _trimmed_center2end_fullpath = _cur_center2end_fullpath[0].copy()
        _trimmed_center2end_fullpath['trajectory'] = [_coord for _coord in _trimmed_center2end_coords]

        # 然后在上面根据设置聚合位置占航程的比例获取聚合航路点
        _sparse2aggr_ppath = PlanPathTrimmer(_trimmed_center2end_fullpath).trim_plan_path(aggr_ratio, trim_mode='to_tail')
        _aggr_paths = {_fleet: [init_locations[_fleet], _sparse2aggr_ppath[0]] for _fleet in init_locations.keys()}
        _together_path = [_loc for _loc in _sparse2aggr_ppath]

        return _aggr_paths, _together_path

class GraphOrdersPlanner(SimpleOrdersTwistor):
    def __init__(self, init_location, orders_graph, key_paths, facilities=None):
        super().__init__(init_location, orders_graph, facilities)
        self.key_paths = key_paths

    def _joint_nodes_compare(self, lhs_node, rhs_node, mode='ratio'):
        # 比较两个节点在各个关键路径中的位置，以确定它们的先后顺序
        if mode == 'sum':
            _lhs_keypath_locs = lhs_node['key_path_locs']
            _rhs_keypath_locs = rhs_node['key_path_locs']

            if np.sum(_lhs_keypath_locs) > np.sum(_rhs_keypath_locs):
                return 1
            elif np.sum(_lhs_keypath_locs) < np.sum(_rhs_keypath_locs):
                return -1
            else:
                return 0
        elif mode == 'ratio':
            _lhs_keypath_ratios = lhs_node['key_path_ratios']
            _rhs_keypath_ratios = rhs_node['key_path_ratios']

            if np.max(_lhs_keypath_ratios) > np.max(_rhs_keypath_ratios):
                return 1
            elif np.max(_lhs_keypath_ratios) < np.max(_rhs_keypath_ratios):
                return -1
            else:
                return 0

    def _assort_task_joints(self, plan_graph):
        # 提取当前plan graph中的所有的“协同执行节点”，排列他们的先后顺序
        _graph_in_degs = dict(plan_graph.in_degree())
        _mult_in_nodes = [{'node': _node, 'degree': _deg} for _node, _deg in _graph_in_degs.items() if _deg > 1]

        for _node_item in _mult_in_nodes:
            _node_item['key_path_locs'] = [-1 for _iter in range(len(self.key_paths))]
            _node_item['key_path_ratios'] = [0 for _iter in range(len(self.key_paths))]

            for _p_iter, _path in enumerate(self.key_paths):
                if _node_item['node'] in _path:
                    _node_item['key_path_locs'][_p_iter] = _path.index(_node_item['node'])
                    _cur_path_ratio = _node_item['key_path_locs'][_p_iter] / len(_path)
                    _node_item['key_path_ratios'][_p_iter] = np.round(_cur_path_ratio, 3)

        # 根据所有“多入”节点在各个关键节点上面位置进行排序，得到各“多入节点”的处理顺序
        _sorted_mult_in_nodes = sorted(_mult_in_nodes, key=cmp_to_key(self._joint_nodes_compare))

        return _sorted_mult_in_nodes

    def _estimate_trajectory_timesteps(self, trajectory):
        pass

    def _connect_to_nearest_trajectory(self, cur_location, trajectories):
        # 找到和当前位置最近的一组轨迹中的轨迹点，并连接生成新的轨迹
        _cur_loc2trajs_mindists = [0 for _iter in range(len(trajectories))]
        _cur_loc2trajs_minelidxs = [0 for _iter in range(len(trajectories))]

        for _traj_i, _traj in enumerate(trajectories):
            _cur_ptr2traj_dists = np.linalg.norm(np.array(_traj).reshape(-1, 2) - np.array(cur_location).flatten(), axis=1)
            _cur_ptr2traj_minidx = np.argmin(_cur_ptr2traj_dists)

            _cur_loc2trajs_minelidxs[_traj_i] = _cur_ptr2traj_minidx
            _cur_loc2trajs_mindists[_traj_i] = _cur_ptr2traj_dists[_cur_ptr2traj_minidx]

        _nearest_traj_idx = np.argmin(_cur_loc2trajs_mindists)
        _nearest_traj_elidx = _cur_loc2trajs_minelidxs[_nearest_traj_idx]

        # 连同当前的起点位置，生成新的轨迹
        _new_traj = [cur_location] + trajectories[_nearest_traj_idx][_nearest_traj_elidx:]

        return _new_traj

    def _pathwise_plan_generate(self, plan_graph, key_paths, init_locations):
        _marked_plan_graph = plan_graph.copy()

        for _path in key_paths:
            _cur_location = init_locations[_path[0]]

            for _sect_i in range(len(_path) - 1):
                _cur_sect = (_path[_sect_i], _path[_sect_i + 1])
                _edge_data = _marked_plan_graph.edges[_cur_sect]
                _order_mode = _edge_data['order_mode']

                if 'plan' in _edge_data:
                    traj = _edge_data['plan'].get('trajectory', [])
                    if len(traj) == 0:
                        raise ValueError(f"Empty trajectory on edge {_cur_sect}")
                    _cur_location = traj[-1]
                    continue

                _order_type = _edge_data['order_type']
                _target = _edge_data['target']
                _routed_ratio = _edge_data.get('routed_ratio', None)

                if _order_mode in ['singleton', 'disperse']:
                    plan = self.generate_trajectory_plans(_cur_location, [{'type': _order_type, 'target': _target}])[0]
                    _edge_data['plan'] = plan
                    _cur_location = plan['trajectory'][-1]

                elif _order_mode == 'aggregate':
                    _other_in_sect = [
                        edge for edge in _marked_plan_graph.edges
                        if edge[1] == _cur_sect[1] and edge[0] != _cur_sect[0]
                    ]
                    existing_plans = [copy.deepcopy(_marked_plan_graph.edges[edge]['plan'])
                                      for edge in _other_in_sect if 'plan' in _marked_plan_graph.edges[edge]]

                    if not existing_plans:
                        plan = \
                        self.generate_trajectory_plans(_cur_location, [{'type': _order_type, 'target': _target}])[0]
                        _edge_data['plan'] = plan
                        _cur_location = plan['trajectory'][-1]
                    else:
                        if _order_type in ['breakthrough', 'escape']:
                            copied_plan = copy.deepcopy(existing_plans[0])
                            copied_plan['trajectory'][0] = _cur_location
                            _edge_data['plan'] = copied_plan
                            _cur_location = copied_plan['trajectory'][-1]
                        else:
                            other_trajs = [plan['trajectory'] for plan in existing_plans]
                            new_traj = self._connect_to_nearest_trajectory(_cur_location, other_trajs)
                            copied_plan = copy.deepcopy(existing_plans[0])
                            copied_plan['trajectory'] = new_traj
                            _edge_data['plan'] = copied_plan
                            _cur_location = new_traj[-1]

        return _marked_plan_graph

    def generate_trajectories(self, vis_check=False,info_check=False):
        # 为每个关键路径生成具体的目的地和大致时间
        _plans_descs = []

        # 获取可并行执行节点的序列
        _ordered_joint_nodes = self._assort_task_joints(self.orders)

        # 然后根据获取的“联合执行序列”，反推所有节点上面的联合执行时间点
        for _item in _ordered_joint_nodes: print(_item)

        _trajs_graph = self._pathwise_plan_generate(self.orders, self.key_paths, self.init_location)

        graph_data = {
            "nodes": [],
            "edges": []
        }

        for node, attrs in _trajs_graph.nodes(data=True):
            graph_data["nodes"].append({"id": int(node), "attrs": attrs})

        for u, v, attrs in _trajs_graph.edges(data=True):
            edge_info = {
                "from": int(u),
                "to": int(v),
                "attrs": {}
            }
            for k, v_attr in attrs.items():
                if isinstance(v_attr, dict):  # deep copy for plan
                    edge_info["attrs"][k] = json.loads(json.dumps(v_attr))
                else:
                    edge_info["attrs"][k] = v_attr
            graph_data["edges"].append(edge_info)

        if info_check:
            print(json.dumps(graph_data["edges"], indent=4))

        if vis_check:
            self.show_trajectories_graph(_trajs_graph)

        return _trajs_graph

    def show_trajectories_graph(self, trajs_graph):

        edges = list(trajs_graph.edges(data=True))
        cmap = plt.get_cmap('tab10')

        fig, ax = plt.subplots(figsize=(10, 10))

        # 用于记录每个点已绘制标签的次数，决定偏移方向
        label_counts = defaultdict(int)

        def offset_position(xy, count):
            """根据同一点被标注次数，选用不同的偏移模式"""
            offsets = [
                (35, 35), (-35, 35), (35, -35), (-35, -35),
                (40, 0), (0, 40), (-40, 0), (0, -40),
                (55, 45), (55, 45), (-55, -45), (-55, -45)
            ]
            dx, dy = offsets[count % len(offsets)]
            return xy[0] + dx, xy[1] + dy

        for idx, (u, v, data) in enumerate(edges):
            plan = data.get('plan')
            if not plan:
                continue

            traj = np.array(plan['trajectory'])
            color = cmap(idx % 10)

            # 1. 绘制轨迹线
            ax.plot(traj[:, 0], traj[:, 1],
                    linestyle='-', linewidth=2, color=color,
                    label=f"{u}→{v} [{plan['type']}]")

            # 2. 添加方向箭头
            dx = traj[1:, 0] - traj[:-1, 0]
            dy = traj[1:, 1] - traj[:-1, 1]
            ax.quiver(traj[:-1, 0], traj[:-1, 1],
                      dx, dy,
                      angles='xy', scale_units='xy', scale=1,
                      width=0.003, headwidth=3, headlength=5,
                      color=color)

            # 3. 起点和终点标记
            start = traj[0]
            end = traj[-1]
            ax.scatter(start[0], start[1],
                       marker='o', s=60, edgecolor='k', facecolor=color)
            ax.scatter(end[0], end[1],
                       marker='X', s=80, edgecolor='k', facecolor=color)

            # 4. 起点标签（偏移）
            sc = label_counts[(start[0], start[1])]
            sx, sy = offset_position(start, sc)
            ax.text(sx, sy, str(u),
                    fontsize=8, color='black',
                    verticalalignment='center',
                    horizontalalignment='center')
            label_counts[(start[0], start[1])] += 1

            # 5. 终点标签（偏移）
            ec = label_counts[(end[0], end[1])]
            ex, ey = offset_position(end, ec)
            ax.text(ex, ey, str(v),
                    fontsize=8, color='black',
                    verticalalignment='center',
                    horizontalalignment='center')
            label_counts[(end[0], end[1])] += 1

        # 坐标轴、标题、网格、图例（英文也可根据需要改）
        ax.set_xlabel('X 坐标')
        ax.set_ylabel('Y 坐标')
        ax.set_title('各任务轨迹示意图（含方向箭头与偏移标签）')
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.legend(loc='upper right', fontsize='small', ncol=2)

        plt.tight_layout()
        plt.show()
