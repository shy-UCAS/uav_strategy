# examples/uavs_strategy/BlueSingleAgent.py
# -*- coding: utf-8 -*-

import asyncio
import argparse
import getpass
import os, os.path as osp
import sys
import redis
import json
import numpy as np
from typing import Dict, List, Optional, Iterable, Tuple, Any

from time import time
from datetime import datetime

from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour


from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import register_planning_actions
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import FormationAPFStep, APFStep, FetchWorldState ,DT

fleet1 = [
    122.15451569813476,
    37.50781897194055,
    165
]

fleet2 = [
    122.16533086650654,
    37.52305286868604,
    220
]

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


# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, position=None, facilities=None,
                 height_range_set=None, direction_range_set=None, **kwargs):
        super().__init__(jid, password, asl_file)

        # ---- Redis I/O ----
        self.io = UavRedisIO(**kwargs.get("redis_cfg", {}))

        # ---- 初始位置（经纬度），轨迹起点 ----
        self.position = position  # 可以先传 None，后面再补

        # 默认设施：如果外部没传，就从 json 加载
        self.facilities = self._default_facilities() if facilities is None else facilities

        # 周期性行为
        self.APFStep = APFStep
        self.FormationAPFStep = FormationAPFStep
        self.FetchWorldState = FetchWorldState

        self.height_range_set = height_range_set if height_range_set else height_range_value_set
        self.direction_range_set = direction_range_set if direction_range_set else direction_range_set

        # 经纬度 -> UTM
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()

        if self.position is not None:
            # 如果传入了经纬度初始点，用它初始化轨迹
            self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(
                np.array([self.position])
            ).tolist()

        else:
            self.traj = [[0, 0, 100]]
        print(f"Agent {jid} initialized at position: {self.position}, traj: {self.traj}")
        self.self_uid = jid.split("@")[0]
        self.world = {"blue_pos": {}, "red_pos": {}}
        # 当前参考轨迹
        self.cur_reference_traj = []
        self.blue_ids = []
        self.red_ids = []

        # redis初始化数据
        self.io.add_uav_id(self.self_uid, blue=True)
        self.io.set_pos(self.self_uid, self.traj[0][0], self.traj[0][1], self.position[2])
        self.io.set_traj(self.self_uid, [[self.traj[0][0], self.traj[0][1], self.position[2]]])
        self.io.set_lookahead(self.self_uid, 0)

        # 初始化编队信息
        self.formation_state = {
                    "role": "independent",  # 可选: independent, aggregate
                    "follower_num": None,      # 设定从机数量 
                    "formation_type": "v_shape", # 队形可选的有：'circular', 'vertical', 'horizontal', 'vshape','arc' 
                    "cluster_id": None,    # 只有当role不是independent时才有用，用于标识自己属于哪个集群，比如说蓝方blue01和blue02汇合后，cluster_id为blue01_02
                }

    def _default_facilities(self, default_json_path=None):
        if default_json_path is None:
            _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')
        else:
            _facilities_info_json = default_json_path

        with open(_facilities_info_json, 'r', encoding="utf-8") as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(
            _facilities_info['facilities_str'],
            _facilities_info['defence_rings']
        )

    # =============================
    # 2. 注册自定义动作
    # =============================
    def add_custom_actions(self, actions):
        # 注册所有轨迹规划/高度插值相关动作
        register_planning_actions(self, actions)

    # =============================
    # 3. 周期任务：刷新红蓝态势
    # 定义在behaviors_modules/uav_periodic_behaviours.py中
    # =============================



    # =============================
    # 4. BDI Agent 初始化
    # =============================
    async def setup(self):
        # 注册周期任务
        self.add_behaviour(self.FetchWorldState(period=DT))
        #APFStep不涉及编队合并之类的任务
        self.add_behaviour(self.APFStep(period=DT))
        #FormationAPFStep涉及编队合并之类的任务
        # self.add_behaviour(self.FormationAPFStep(period=DT))



# =============================
# 5. main() 入口函数
# =============================
async def main(server: str, password: str):
    """
    SPADE 框架的标准启动方式：
    python -m examples.uavs_strategy.BlueSingleAgent --server 127.0.0.1 --password 202127
    """
    fleet_config = [
            {
                "id": "blue01",
                "asl": "uav_blue_redis_01.asl",
                "pos": fleet1 # Leader
            },
            {
                "id": "blue02",
                "asl": "uav_blue_redis_02.asl",
                "pos": fleet2 # Potential Follower
            }
        ]

    agents = []
    current_dir = os.path.dirname(__file__)
    for cfg in fleet_config:
        jid = f"{cfg['id']}@{server}"
        asl_path = os.path.join(current_dir, cfg["asl"])

        ag = BlueUAVAgent(jid, password, asl_path, position=cfg["pos"])

        # 初始化 BDI 信念
        ag.bdi.set_belief("start_height", cfg["pos"][2])
        ag.bdi.set_belief("if_set_ref_traj", "False")
        ag.bdi.set_belief("my_id", cfg["id"]) # 告诉 ASL 自己的名字

        await ag.start()
        agents.append(ag)
        print(f"✅ Agent {jid} initialized.")


    # 关闭前保持运行
    await asyncio.sleep(99999)



# =============================
# 6. 命令行处理
# =============================
if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--server", help="XMPP Server (host)", required=False)
    # parser.add_argument("--password", help="Password", required=False)
    # args = parser.parse_args()
    #
    # # server
    # if args.server is None:
    #     server = input("XMPP Server> ")
    # else:
    #     server = args.server
    #
    # # password
    # if args.password is None:
    #     passwd = getpass.getpass("Password> ")
    # else:
    #     passwd = args.password

    # 启动
    import spade

    spade.run(main("127.0.0.1", "202127"))
