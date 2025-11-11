import sys
import os, os.path as osp
import random


from sqlalchemy import false

from modules import basic_functions as bfunc
from modules import quick_path_planners as qpp
from modules import math_curves_generators as curve_gen

import numpy as np
import json
import asyncio
from datetime import datetime, timedelta

import redis # 共享一个全局的蓝方分布地图，直接提取其中他机位置信息 （方法1）
# key(blueuav_xxx), value (json - 坐标或状态信息)

SPADE_PKGS_ROOT = osp.abspath(osp.join(osp.abspath(__file__), "../../../.."))
# import pdb; pdb.set_trace()
SPADE_DIR = osp.join(SPADE_PKGS_ROOT, "spade-master")
SPADE_BDI_DIR = osp.join(SPADE_PKGS_ROOT, "spade_bdi-master")

# import pdb; pdb.set_trace()
if not SPADE_DIR in sys.path:
    sys.path.insert(0, SPADE_DIR)
if not SPADE_BDI_DIR in sys.path:
    sys.path.insert(0, SPADE_BDI_DIR)

import agentspeak
import spade
from spade.behaviour import PeriodicBehaviour, TimeoutBehaviour
from spade.template import Template

from spade_bdi.bdi import BDIAgent

fleet1 = [
    122.09686551225596,
    37.56536338371065
]

fleet2 = [
    122.10258217246229,
    37.56342057758475
]

height_value_set = {
    'breakthrough': [200, 0],
    'escape': [0, 200],
    'detour': [0, 200]
}

height_range_value_set = {
    'breakthrough': [[250, 400], [0, 100]],
    'escape': [[0, 100], [250, 400]],
    'detour': [[0, 100], [200, 400]]
}
direction_range_set = {
    'breakthrough': [-20, 20],
    'escape': [-20, 20],
    'detour': [0, 360]
}


class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, position=None, facilities=None, height_range_set=None,
                 direction_range_set=None):
        super().__init__(jid, password, asl_file)
        self.position = position if position else None
        self.facilities = self._default_facilities() if facilities is None else facilities
        self.height_range_set = height_range_set if height_range_set else height_range_value_set
        self.direction_range_set = direction_range_set if direction_range_set else direction_range_set
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()

        self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([self.position])).tolist()

    def _default_facilities(self, default_json_path=None):
        if default_json_path is None:
            _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')
        else:
            _facilities_info_json = default_json_path

        with open(_facilities_info_json, 'r') as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(_facilities_info['facilities_str'], _facilities_info['defence_rings'])

    async def setup(self):
        _template = Template(metadata={"performative": "RedUavAlert"})
        # 读取task_sequence里面的任务列表

        self.add_behaviour(self.RedUavAlert(period=5, start_at=datetime.now()), _template)

    class RedUavAlert(PeriodicBehaviour): # 检查红方位置
        # 这里进行红方uav是否在威胁范围内的周期性检查
        # 周期性获取红方uav的坐标，蓝方规划好预选的轨迹后，根据轨迹和红方uav的坐标，判断是否有必要进行轨迹调整
        async def run(self):
            print(f"{self.agent.name} checking alter from red uavs ...")
            if len(self.agent.bdi_intention_buffer) > 0:
                self.agent.belief_update(self.agent.bdi_intention_buffer.popleft()) # 在置信空间里面添加发现红方的fact
                self.agent.beliefs.insert("red_alert")
            
            if red_detected:
                self.agent.beliefs.insert("!avoid_red_enemy")

    class BlueUavsCheck(PeriodicBehaviour): # 检查其他蓝方的位置和状态信息
        async def run(self):
            print(f"{self.agent.name} checking blue uavs ...")
            self.agent.beliefs.insert("blue_collide")

            if blue_in_range:
                self.agent.beliefs.insert("!avoid_blue_friend")
    
    class ExecutStep(PeriodicBehaviour): # 执行规划好的轨迹
        async def run(self):
            print(f"{self.agent.name} executing plan trajectory ...")
    
    class StateCheck(PeriodicBehaviour): # 检查当前状态
        async def run(self):
            print(f"{self.agent.name} checking current state ...")

    def add_custom_actions(self, actions):
        @actions.add(".act_breakthrough", 3)
        def _action_breakthrough(agent, term, intention):
            # 获取三个参数：第一个是目标类型，第二个是起点高度，第三个是终点高度
            _arg_target = str(agentspeak.grounded(term.args[0], intention.scope))
            _arg_start_height = int(agentspeak.grounded(term.args[1], intention.scope))
            _arg_end_height = int(agentspeak.grounded(term.args[2], intention.scope))

            # 计算轨迹
            if _arg_target in bfunc.GlobalBasicConfigs.PLANNING_BREAKTHROUGH_FACILITY_TYPES:
                _traj = self._plan_breakthrough_targettype(self.traj[-1], _arg_target)
            elif _arg_target in self.facilities.get_facilities_names():
                _traj = self._plan_breakthrough_target(self.traj[-1], _arg_target)

            # 执行三维坐标转换并且插值高度
            _traj = self._insert_height_val('breakthrough', _traj, _arg_start_height, _arg_end_height)
            self.traj.extend(_traj[1:])

            print(f"{agent.name} is breaking through {str(_arg_target)},breakthrough trajectory is: \n {_traj}")
            print(f"cur full trajectory: {self.traj}")

            # blueavoid 
            
            yield

        @actions.add(".act_escape", 3)
        def _action_escape(agent, term, intention):
            _arg_target = str(agentspeak.grounded(term.args[0], intention.scope))
            _arg_start_height = int(agentspeak.grounded(term.args[1], intention.scope))
            _arg_end_height = int(agentspeak.grounded(term.args[2], intention.scope))

            _traj = self._insert_height_val(
                'escape',
                self._plan_escape(self.traj[-1], _arg_target),
                _arg_start_height,
                _arg_end_height
            )

            self.traj.extend(_traj[1:])
            print(f"{agent.name} is escaping {_arg_target},escape trajectory is: \n {_traj}")
            print(f"cur full trajectory: {self.traj}")
            yield

        @actions.add(".act_attack", 1)
        def _action_attack(agent, term, intention):
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            print(f"{agent.name} is attacking {_arg} ...")

            yield

        @actions.add(".act_detour", 3)
        def _action_detour(agent, term, intention):
            _arg_target = str(agentspeak.grounded(term.args[0], intention.scope))
            _arg_start_height = int(agentspeak.grounded(term.args[1], intention.scope))
            _arg_end_height = int(agentspeak.grounded(term.args[2], intention.scope))
            _traj = self._insert_height_val(
                'detour',
                self._plan_detour(self.traj[-1], _arg_target),
                _arg_start_height,
                _arg_end_height
            )
            self.traj.extend(_traj[1:])
            print(f"{agent.name} is detouring {_arg_target},detour trajectory is: \n{_traj}")
            print(f"cur full trajectory: {self.traj}")
            yield

        @actions.add(".act_get_position", 1)
        def _action_get_position(agent, term, intention):
            # 获取 UAV 当前的位置
            position = self.traj[-1]
            print(f"{agent.name} current position: {position}")

            # 将位置绑定到 X 变量
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            _arg.set_value(position)  # 将当前位置赋给 X 变量
            yield

        @actions.add(".act_visualize", 0)
        def _act_visualize(agent, term, intention):
            self.facilities.visualize(show_mode = "2D")
            yield
        
        @actions.add(".check_join_status", 1)
        def _act_check_joint_status(agent, term, intention):
            _arg = str(agentspeak.grounded(term.args[0], intention.scope))


    def _insert_height_val(self, order_type, traj, start_height, end_height):
        # 传入的是只有二维坐标的轨迹点，先给起点和终点添加高度，然后插值得到中间点的高度，返回一个新的三维轨迹点列表
        _default_height_range = self.height_range_set[order_type]

        # 生成随机的起点和终点高度
        _altitude_start_random = random.randint(
            _default_height_range[0][0],
            _default_height_range[0][1]
        )
        _altitude_end_random = random.randint(
            _default_height_range[1][0],
            _default_height_range[1][1]
        )

        # 修改起点高度
        if len(traj[0]) == 2:
            # 传入的值是二维坐标，则给起点添加高度，且值是-1的时候随机生成高度
            traj[0].append(start_height if start_height != -1 else _altitude_start_random)
        elif len(traj[0]) == 3:
            pass

        # 修改终点高度
        if len(traj[-1]) == 2:
            traj[-1].append(end_height if end_height != -1 else _altitude_end_random)
        elif len(traj[-1]) == 3:
            pass

        # 此时轨迹点列表中只有起点和终点是三维的，需要插值得到中间点的高度
        _3dim_traj = curve_gen.interpolate_z_coordinates(traj)

        if order_type == 'detour':
            # escape类型的数据轨迹本身就有很多个点，用样条曲线方法拟合就够了
            return curve_gen.cubic_interpolation_3d(_3dim_traj)
        else:
            # 其他类型的轨迹都是只有两个轨迹点，可以套用breakthrough的方法
            return curve_gen.generate_breakthrough_flight(_3dim_traj)

    def _plan_breakthrough_targettype(self, start_location, facility_type, utm=True):
        # 寻找可以突破的目标
        _target_location = self.facilities.pick_random_target(facility_type, utm)
        return [start_location,
                list(_target_location['location']) if isinstance(_target_location['location'], tuple) else
                _target_location['location']]

    def _plan_breakthrough_target(self, start_location, target, utm=True):
        _target_location = self.facilities.get_target_location(target, utm)
        return [start_location, list(_target_location) if isinstance(_target_location, tuple) else _target_location]

    def _plan_detour(self, start_location, target, detour_steps=5, utm=True):
        if target in self.facilities.facilities_info.keys() \
            or target in self.facilities.defend_rings.keys():

            if target in self.facilities.antiairs:
                _detour_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.antiairs[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE,
                                                                                   ll2utm=utm)
            elif target in self.facilities.headquartors:
                _detour_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.headquartors[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_HQ_DISTANCE,
                                                                                   ll2utm=utm)
            elif target in self.facilities.probers:
                _detour_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.probers[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_RADAR_DISTANCE,
                                                                                   ll2utm=utm)

            elif target in self.facilities.defend_rings.keys():
                _detour_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'defence_rings':
            _detour_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'probe_facilities':
            _detour_polygon_xys = self.facilities.get_probe_facilities_polyborder()

        elif target == 'antiair_facilities':
            _detour_polygon_xys = self.facilities.get_defence_facilities_polyborder()

        _border = qpp.SimpleBorders(_detour_polygon_xys[0])
        _traj_locations = _border.move_along_border(start_location if len(start_location) == 2 else start_location[:2],
                                                    steps=detour_steps,
                                                    direction=np.random.choice(['clockwise', 'anticlockwise']))

        return [list(item) if isinstance(item, tuple) else item for item in _traj_locations]

    def _plan_escape(self, start_location, target='defence_rings', utm=True):
        if target in self.facilities.facilities_info.keys() \
            or target in self.facilities.defend_rings.keys():

            if target in self.facilities.antiairs:
                _escape_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.antiairs[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE,
                                                                                   ll2utm=utm)
            elif target in self.facilities.headquartors:
                _escape_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.headquartors[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_HQ_DISTANCE,
                                                                                   ll2utm=utm)
            elif target in self.facilities.probers:
                _escape_polygon_xys = self.facilities.get_spec_facility_polyborder(self.facilities.probers[target],
                                                                                   bfunc.GlobalBasicConfigs.AVOID_RADAR_DISTANCE,
                                                                                   ll2utm=utm)
            elif target in self.facilities.defend_rings.keys():
                _escape_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'defence_rings':
            _escape_polygon_xys = self.facilities.get_defence_rings_polyborder()

        elif target == 'probe_facilities':
            _escape_polygon_xys = self.facilities.get_probe_facilities_polyborder()

        elif target == 'antiair_facilities':
            _escape_polygon_xys = self.facilities.get_defence_facilities_polyborder()

        _border = qpp.SimpleBorders(_escape_polygon_xys[0])
        if not _border.is_inside_border(start_location):
            # 如果当前坐标本来就在逃逸范围之外，那么则直接返回当前坐标
            _border_traj = [start_location, start_location]
            print(f"_outside_border_traj:{_border_traj}")
            return _border_traj
        else:
            # 否则，则计算当前坐标到逃逸范围的最近边界点，并返回
            # _nearest_point = _border.get_nearest_border_point(init_location)
            _nearest_point = _border.get_nearest_border_vertex(start_location if len(start_location) == 2 else start_location[:2]).coords[0]
            _border_traj = [start_location, list(_nearest_point) if isinstance(_nearest_point, tuple) else _nearest_point]
            print(f"_inside_border_traj:{_border_traj}")
            return _border_traj


async def main(server, password):
    uav_blue01 = BlueUAVAgent(f"blue01@{server}", password, "uav_blue_01.asl", position=fleet1)
    uav_blue01.bdi.set_belief("my_friend", f"blue02@{server}")

    uav_blue02 = BlueUAVAgent(f"blue02@{server}", password, "uav_blue_02.asl", position=fleet2)
    uav_blue02.bdi.set_belief("my_friend", f"blue01@{server}")

    await uav_blue01.start()
    await uav_blue02.start()

    await asyncio.sleep(4)

    await uav_blue01.stop()
    await uav_blue02.stop()


if __name__ == "__main__":
    spade.run(main("127.0.0.1", "202127"))
