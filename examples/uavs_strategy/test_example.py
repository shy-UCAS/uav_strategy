# examples/uavs_strategy/test_example.py
# -*- coding: utf-8 -*-

import asyncio
import argparse
import getpass
import os, os.path as osp
import sys
import redis
import json
import numpy as np

from datetime import datetime, time

from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour

# === 项目内部模块（统一写成 from planning_modules.xxx 导入） ===
from .redis_modules.uav_redis_io import UavRedisIO
from .planning_modules.uav_planning_actions import register_planning_actions
from .planning_modules import basic_functions as bfunc

fleet1 = [
    122.09686551225596,
    37.56536338371065,
    165
]

fleet2 = [
    122.10258217246229,
    37.56342057758475
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
K_ATT = 0.5  # 引力系数
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

        # redis初始化数据
        self.io.add_uav_id(self.self_uid, blue=True)
        self.io.set_pos(self.self_uid, self.traj[0][0], self.traj[0][1], self.position[2])
        self.io.set_traj(self.self_uid, [[self.traj[0][0], self.traj[0][1], self.position[2]]])
        self.io.set_lookahead(self.self_uid, 0)  # 初始化 lookahead 为 0（指向参考轨迹的第一个目标点）

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

            # 查询蓝方 ID
            blue_ids = io.get_ids(blue=True)
            if not blue_ids:
                blue_ids = io.scan_ids_by_key("uav")

            # 查询红方 ID
            red_ids = io.get_ids(blue=False)
            if not red_ids:
                red_ids = io.scan_ids_by_key("red")

            # 批量读取蓝方和红方的位置信息
            blue_all = io.mget_pos(blue_ids, blue=True)
            red_all = io.mget_pos(red_ids, blue=False)

            # 写入到 agent.world 中
            agent.world["blue_pos"] = {k: v for k, v in blue_all.items() if v}
            agent.world["red_pos"] = {k: v for k, v in red_all.items() if v}
            print(f"[FetchWorldState] blue={list(agent.world['blue_pos'].keys())}, "
                  f"red={list(agent.world['red_pos'].keys())}")

    class APFStep(PeriodicBehaviour):
        async def run(self):
            agent = self.agent
            io = agent.io
            current_time = time()  # 获取当前时间

            # 获取蓝方（当前 Agent）的 ID 和位姿
            me = io.get_pos(agent.self_uid, blue=True)
            if not me:
                return  # 如果没有当前无人机位置，跳过

            # 获取当前的参考轨迹 (_cur_reference_traj)
            traj = agent.cur_reference_traj
            if not traj:
                return  # 如果没有参考轨迹，跳过
            if agent.bdi.get_belief("if_set_ref_traj"):
                # 如果当前轨迹没有写入redis，则写入
                io.set_ref_traj(agent.self_uid,traj)
                agent.bdi.set_belief("if_set_ref_traj", "False")
                print(f"ref_traj add to redis")

            # 获取当前预瞄点的位置（从参考轨迹中）
            lookahead = io.get_lookahead(agent.self_uid)
            lookahead = max(1, min(lookahead, len(traj) - 1))
            goal = traj[lookahead]  # 当前目标点（预瞄点）
            print(f"Current goal: {goal}")
            # 获取当前无人机的位置
            p_me = [me["x"], me["y"], me["z"]]

            # ---- 引力：朝向目标（预瞄点） ----
            F_att = v_scale(v_sub(goal, p_me), K_ATT)

            # ---- 斥力：来自其他无人机的位置 ----
            F_rep = [0.0, 0.0, 0.0]

            def acc_rep(others: dict):
                nonlocal F_rep
                for k, p in (others or {}).items():
                    if not p: continue
                    if k == agent.self_uid: continue  # 排除自己
                    d = v_sub(p_me, [float(p["x"]), float(p["y"]), float(p["z"])])
                    dist = v_norm(d)
                    if 0.0 < dist < R_INF:
                        mag = K_REP * (1.0 / dist - 1.0 / R_INF) * (1.0 / (dist * dist))
                        F_rep = v_add(F_rep, v_scale(v_unit(d), mag))

            # 计算蓝方和红方的斥力（使用已经存储的位姿数据）
            acc_rep(agent.world.get("blue_pos", {}))
            acc_rep(agent.world.get("red_pos", {}))

            # ---- 合力：引力和斥力合成 ----
            F = v_add(F_att, F_rep)

            # 步进：每 period 向目标迈进 STEP 米
            step_vec = v_scale(v_unit(F), STEP)  # 每步最大位移
            # nxt = [p_me[0] + step_vec[0], p_me[1] + step_vec[1], p_me[2] + step_vec[2]]
            nxt = [p_me[0] + F_att[0], p_me[1] + F_att[1], p_me[2] + F_att[2]]

            # ---- 如果到达预瞄点，推进到下一个点 ----
            if v_norm(v_sub(goal, nxt)) <= CLOSE_TH and lookahead < len(traj) - 1:
                io.set_lookahead(agent.self_uid, lookahead + 1)

            # ---- 回写新的无人机位置到 Redis ----
            io.set_pos(agent.self_uid, nxt[0], nxt[1], nxt[2])
            io.append_traj_points(agent.self_uid, [nxt[0], nxt[1], nxt[2]])

            print(f"New position for {agent.self_uid}: {nxt}")

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

    jid = f"blue01@{server}"
    asl_path = os.path.join(os.path.dirname(__file__), "uav_redis_io.asl")
    # === 构造 UAV Agent ===
    ag = BlueUAVAgent(
        jid,
        password,
        asl_path,
        redis_cfg={"host": "127.0.0.1", "port": 6379},
        position=fleet1,
    )
    ag.bdi.set_belief("start_height", fleet1[2])
    ag.bdi.set_belief("if_set_ref_traj", "False")

    # 启动代理
    await ag.start()
    print(f"Agent {jid} started.")

    # 关闭前保持运行
    await asyncio.sleep(99999)


# =============================
# 6. 命令行处理
# =============================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", help="XMPP Server (host)", required=False)
    parser.add_argument("--password", help="Password", required=False)
    args = parser.parse_args()

    # server
    if args.server is None:
        server = input("XMPP Server> ")
    else:
        server = args.server

    # password
    if args.password is None:
        passwd = getpass.getpass("Password> ")
    else:
        passwd = args.password

    # 启动
    import spade

    spade.run(main(server, passwd))
