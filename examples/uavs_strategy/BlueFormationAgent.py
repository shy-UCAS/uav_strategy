# examples/uavs_strategy/BlueFormationAgent.py
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

# === 项目内部模块（统一写成 from planning_modules.xxx 导入） ===
from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import register_planning_actions
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.planning_modules import avoidance_agents as a_agents
# 周期性执行的行为
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import FormationAPFStep, APFStep,DT

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

# ==== 势场与步进参数（可改为 configs.ini 里读取）====
DT = 1.0  # 与 PeriodicBehaviour 的 period 对齐（秒）
STEP = 8.0  # 每步“最大位移”/速度上限（米/步）
K_ATT = 0.95  # 引力系数
K_REP = 2.5  # 斥力系数
R_INF = 150.0  # 斥力影响半径（米）
CLOSE_TH = 10000.0  # 预瞄点“到点”判定阈值（米）


# ==== 简单向量函数 ====
def v_sub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def v_add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def v_scale(a, s): return [a[0] * s, a[1] * s, a[2] * s]


def v_norm(a):
    import math
    n = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
    return n if n > 1e-9 else 1e-9


def v_unit(a):
    n = v_norm(a)
    return [a[0] / n, a[1] / n, a[2] / n]


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

        self.APFStep = APFStep
        self.FormationAPFStep = FormationAPFStep

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
            # 没有就先用一个占位点
            self.traj = [[0, 0, 100]]

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
        self.io.set_lookahead(self.self_uid, 0)  # 初始化 lookahead 为 0（指向参考轨迹的第一个目标点）

        # 初始化编队状态：
        self.formation_state = {
                    "role": "independent",  # 可选: independent, follower, leader
                    "leader_id": None,      # 仅 follower 需要
                    "offset": np.array([0.0, 0.0, 0.0]) # 相对 Leader 的 XYZ 偏移
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
        # 注册所有轨迹规划/高度插值相关动作（act_breakthrough/act_escape/...）
        register_planning_actions(self, actions)

        # 可在这里扩展 redis IO 动作等

    # =============================
    # 3. 周期任务：刷新红蓝态势
    # =============================
    class FetchWorldState(PeriodicBehaviour):
        async def run(self):
            agent = self.agent  # BlueUAVAgent 实例
            io = agent.io

            # 查询蓝红方 ID
            agent.blue_ids = io.get_ids(blue=True) or io.scan_ids_by_key("uav")
            agent.red_ids = io.get_ids(blue=False) or io.scan_ids_by_key("red")

            # 批量读取蓝方和红方的位置信息
            blue_all = io.mget_pos(agent.blue_ids, blue=True)
            red_all = io.mget_pos(agent.red_ids, blue=False)

            # 写入到 agent.world 中
            agent.world["blue_pos"] = {k: v for k, v in blue_all.items() if v}
            agent.world["red_pos"] = {k: v for k, v in red_all.items() if v}


    # =============================
    # 4. BDI Agent 初始化
    # =============================
    async def setup(self):
        print(f"{self.name} started at {datetime.now()}")

        # 每 1 秒读取一次 Redis
        self.add_behaviour(self.FetchWorldState(period=1.0))
        self.add_behaviour(self.APFStep(period=DT))



# =============================
# 5. main() 入口函数
# =============================
async def main(server: str, password: str):
    """
    SPADE 框架的标准启动方式：
    python -m examples.uavs_strategy.test_example --server 127.0.0.1 --password 202127
    """

    # blue01 agent
    jid_01 = f"blue01@{server}"
    asl_path_01 = os.path.join(os.path.dirname(__file__), "uav_blue_redis_01.asl")  # blue01的ASL文件路径
    # 构造 blue01 UAV Agent
    ag_01 = BlueUAVAgent(
        jid_01,
        password,
        asl_path_01,
        redis_cfg={"host": "127.0.0.1", "port": 6379},
        position=fleet1,
    )
    ag_01.bdi.set_belief("start_height", fleet1[2])
    ag_01.bdi.set_belief("if_set_ref_traj", "False")

    # blue02 agent
    jid_02 = f"blue02@{server}"
    asl_path_02 = os.path.join(os.path.dirname(__file__), "uav_blue_redis_02.asl")  # blue02的ASL文件路径
    # 构造 blue02 UAV Agent
    ag_02 = BlueUAVAgent(
        jid_02,
        password,
        asl_path_02,
        redis_cfg={"host": "127.0.0.1", "port": 6379},
        position=fleet2,
    )
    ag_02.bdi.set_belief("start_height", fleet2[2])
    ag_02.bdi.set_belief("if_set_ref_traj", "False")

    # 启动代理
    await ag_01.start()
    await ag_02.start()
    print(f"Agent {jid_01} started.")
    print(f"Agent {jid_02} started.")

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
