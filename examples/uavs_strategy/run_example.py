import sys
import os, os.path as osp

from sqlalchemy import false

from modules import basic_functions as bfunc
from modules import quick_path_planners as qpp

import numpy as np
import json
import asyncio
from datetime import datetime, timedelta

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


class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, position=None, facilities=None):
        super().__init__(jid, password, asl_file)
        self.position = position if position else None
        if facilities is None:
            self.facilities = self._default_facilities()
        else:
            self.facilities = facilities
        self.traj = [self.position]

    def _default_facilities(self, default_json_path = None):
        if default_json_path is None:
            _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')
        else:
            _facilities_info_json = default_json_path

        with open(_facilities_info_json, 'r') as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(_facilities_info['facilities_str'], _facilities_info['defence_rings'])

    async def setup(self):
        _template = Template(metadata={"performative": "RedUavAlert"})
        self.add_behaviour(self.RedUavAlert(period=5, start_at=datetime.now()), _template)

    class RedUavAlert(PeriodicBehaviour):
        # 这里进行红方uav是否在威胁范围内的周期性检查
        async def run(self):
            print(f"{self.agent.name} checking alter from red uavs ...")

    def add_custom_actions(self, actions):
        @actions.add(".act_breakthrough", 1)
        def _action_breakthrough(agent, term, intention):
            _arg = str(agentspeak.grounded(term.args[0], intention.scope))
            if  _arg in bfunc.GlobalBasicConfigs.PLANNING_BREAKTHROUGH_FACILITY_TYPES:
                _traj =  self._plan_breakthrough_targettype(self.traj[-1], _arg)
            elif _arg in self.facilities.get_facilities_names():
                _traj = self._plan_breakthrough_target(self.traj[-1], _arg)
            self.traj.extend(_traj[1:])
            print(f"{agent.name} is breaking through {str(_arg)},breakthrough trajectory is: {_traj}",end=' ')
            print(f"full trajectory: {self.traj}")
            yield

        @actions.add(".act_escape", 1)
        def _action_escape(agent, term, intention):
            _arg = str(agentspeak.grounded(term.args[0], intention.scope))
            _traj = self._plan_escape(self.traj[-1], _arg)
            self.traj.extend(_traj[1:])
            print(f"{agent.name} is escaping {_arg},escape trajectory is: {_traj}",end=' ')
            print(f"full trajectory: {self.traj}")
            yield

        @actions.add(".act_attack", 1)
        def _action_attack(agent, term, intention):
            _arg = agentspeak.grounded(term.args[0], intention.scope)
            print(f"{agent.name} is attacking {_arg} ...",end=' ')

            yield

        @actions.add(".act_detour", 1)
        def _action_detour(agent, term, intention):
            _arg = str(agentspeak.grounded(term.args[0], intention.scope))
            _traj = self._plan_detour(self.traj[-1], _arg)
            self.traj.extend(_traj[1:])
            print(f"{agent.name} is detouring {_arg},detour trajectory is: {_traj}",end=' ')
            print(f"full trajectory: {self.traj}")
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

    def _plan_breakthrough_targettype(self, start_location, facility_type, utm = False):
        # 寻找可以突破的目标
        _target_location = self.facilities.pick_random_target(facility_type,utm)
        return [start_location, _target_location['location']]

    def _plan_breakthrough_target(self, start_location, target):
        _target_location = self.facilities.get_target_location(target)
        return [start_location, _target_location]

    def _plan_detour(self, start_location, target , detour_steps=5):
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

        _border = qpp.SimpleBorders(_detour_polygon_xys[0])
        _traj_locations = _border.move_along_border(start_location, steps=detour_steps,
                                                    direction=np.random.choice(['clockwise', 'anticlockwise']))

        return _traj_locations


    def _plan_escape(self, start_location, target = 'defence_rings'):
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

        _border = qpp.SimpleBorders(_escape_polygon_xys[0])
        if not _border.is_inside_border(start_location):
            # 如果当前坐标本来就在逃逸范围之外，那么则直接返回当前坐标
            return [start_location, start_location]
        else:
            # 否则，则计算当前坐标到逃逸范围的最近边界点，并返回
            # _nearest_point = _border.get_nearest_border_point(init_location)
            _nearest_point = _border.get_nearest_border_vertex(start_location)
            return [start_location, _nearest_point.coords[0]]


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
