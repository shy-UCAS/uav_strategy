# examples/uavs_strategy/test_example.py
# -*- coding: utf-8 -*-

import asyncio
import argparse
import getpass
import os

from datetime import datetime

from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour

# === 项目内部模块（统一写成 from modules.xxx 导入） ===
from redis_modules.uav_redis_io import UavRedisIO
from modules.uav_planning_actions import register_planning_actions


# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, **kwargs):
        super().__init__(jid, password, asl_file)

        # === Redis I/O ===
        self.io = UavRedisIO(**kwargs.get("redis_cfg", {}))

        # === UAV 状态 ===
        # 初始轨迹点，用于规划函数拼接
        self.traj = [[0, 0, 100]]

        # 当前 Agent 名字，用于 redis 键名
        self.self_uid = jid.split("@")[0]

        # === 设施信息（你自己提供的）===
        # 测试阶段可以给一个占位对象
        # 若已有 real facilities，可替换成真实对象
        self.facilities = kwargs.get("facilities", None)

        # === 高度区间配置（从你原代码复制）===
        self.height_range_set = {
            "breakthrough": ((80, 120), (80, 120)),
            "escape":       ((80, 120), (80, 120)),
            "detour":       ((80, 120), (80, 120)),
        }

        # === 周期性缓存 ===
        self.world = {"blue_pos": {}, "red_pos": {}}

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
            agent = self.agent       # BlueUAVAgent 实例
            io    = agent.io

            # 蓝方 ID
            blue_ids = io.get_ids(blue=True)
            if not blue_ids:
                blue_ids = io.scan_ids_by_key("uav")

            # 红方 ID
            red_ids = io.get_ids(blue=False)
            if not red_ids:
                red_ids = io.scan_ids_by_key("red")

            # 批量读取位姿
            blue_all = io.mget_pos(blue_ids, blue=True)
            red_all  = io.mget_pos(red_ids,  blue=False)

            # 非空过滤（不做过期过滤）
            agent.world["blue_pos"] = {k: v for k, v in blue_all.items() if v}
            agent.world["red_pos"]  = {k: v for k, v in red_all.items() if v}

            # 打印看看
            print(f"[FetchWorldState] blue={list(agent.world['blue_pos'].keys())}, "
                  f"red={list(agent.world['red_pos'].keys())}")

    # =============================
    # 4. BDI Agent 初始化
    # =============================
    async def setup(self):
        print(f"{self.name} started at {datetime.now()}")

        # 每 1 秒读取一次 Redis
        self.add_behaviour(self.FetchWorldState(period=1.0))


# =============================
# 5. main() 入口函数
# =============================
async def main(server: str, password: str):
    """
    SPADE 框架的标准启动方式：
    python -m examples.uavs_strategy.test_example --server 127.0.0.1 --password 202127
    """

    jid = f"blue01@{server}"
    asl_path = os.path.join(os.path.dirname(__file__), "uav_blue_01.asl")
    # === 构造 UAV Agent ===
    ag = BlueUAVAgent(
        jid,
        password,
        asl_path,
        redis_cfg={"host": "127.0.0.1", "port": 6379},
    )

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
