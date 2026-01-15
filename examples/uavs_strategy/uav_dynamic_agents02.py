# 动态创建 / 删除 BlueUAVAgent 
# 结合key_paths处理提取出的数据,数据导入redis服务器
# 使用一个固定通用的asl文件处理uav_key_path.asl

import asyncio
import argparse
import getpass
import os, os.path as osp
import sys
import redis
import json
import numpy as np
import spade
import agentspeak
import time
import collections
import random
import math


from typing import Dict, List, Optional, Iterable, Tuple, Any
from matplotlib.animation import FuncAnimation
# from time import time
from datetime import datetime
from sympy import N


from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import PlanningLib
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.planning_modules.formation_generator import FormationGenerator3D, Formation_Elements
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import FormationAPFStep, APFStep, FetchWorldState ,DT
from examples.uavs_strategy.key_path_analyzer import KeyPathAnalyzer

init_loc1 = [
    122.09686551225597,
    37.56536338371063,
    200.0
]

init_loc2 = [
    122.10258217246229,
    37.56342057758475,
    200.0
]
init_locs = [
    # [122.06711375, 37.57744204,200],
    # [122.11945753, 37.57340029,180],
    # [122.12628947, 37.52707223,190],
    # [122.07039604, 37.52213903,220]
    [
        122.10258217246229,
        37.56342057758475,
        200.0
    ],
    [
        122.09686551225597,
        37.56536338371063,
        200.0
    ]
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

current_dir = os.path.dirname(__file__)
digraph_attrs_reference_path = os.path.join(current_dir, "data", "digraph_with_attrs.json")
key_path_instructions_path = os.path.join(current_dir,"data" ,"key-path-analyzer02.json")
asl_file = os.path.join(current_dir, "uav_key_path.asl")

digraph_attrs = json.load(open(digraph_attrs_reference_path, "r"))
key_path_instructions = json.load(open(key_path_instructions_path, "r"))
bdi_instructions = key_path_instructions["bdi_instructions"]
# facilities_file = os.path.join(current_dir,"data" ,"facilities.json")
facilities_file = os.path.join(current_dir,"data" ,"test_facilities_locations.json")
# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, flight_plan, orchestrator, init_pos=None, facilities=None, **kwargs):
        super().__init__(jid, password, asl_file)
        self.flight_plan = flight_plan  # 航段列表: [{'segment': (u, v), 'coords': []}, ...]
        self.orchestrator = orchestrator
        self.path_index = 0
        self.is_finished = False
        
        # 确定起始节点
        if self.flight_plan:
            self.current_node = self.flight_plan[0]['segment'][0]
            self.next_node = self.flight_plan[0]['segment'][1]
        
        # 初始化设施
        self.facilities = self._default_facilities(facilities)
        # 初始化位置
        self.position = init_pos
        # 经纬度 -> UTM
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        
        # 处理初始位置 (如果为 None, 使用第一段的起点或随机生成)
        if self.position is None:
             # 尝试使用第一段航段的第一个坐标 (如果可用)
             # 但目前 extract_uav_trajectories 设置的 'coords' 是空的 (占位符)
             # 所以我们使用传入的 init_pos 或默认的随机逻辑
             print("No initial position provided, generating random position.")
             _rdm_init_pos = bfunc.generate_circle_positions_from_diameter(1, init_loc1, init_loc2)
             self.position = _rdm_init_pos[0]
        
        self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([self.position])).tolist()
        self.traj[0].append(self.position[2])
        print(f"{self.jid} initialized at position: {self.position}")

        # Redis I/O 模块
        self.io = UavRedisIO(**kwargs.get("redis_cfg", {}))
        self.self_uid = jid.split("@")[0] 
        self.io.add_uav_id(self.self_uid, blue=True)
        self.io.set_pos(self.self_uid, self.traj[0][0], self.traj[0][1], self.position[2])
        self.io.set_traj(self.self_uid, [[self.traj[0][0], self.traj[0][1], self.position[2]]])
        self.io.set_lookahead(self.self_uid, 0)

        self.APFStep = APFStep
        self.FetchWorldState = FetchWorldState
        self.planning_lib = PlanningLib(self)
        self.cur_reference_traj = []
        self.members_cur_reference_traj = []
        self.height_range_set = height_range_value_set
        self.direction_range_set = direction_range_set

        # 初始化 BDI Beliefs
        if self.flight_plan:
             self.bdi.set_belief("cur_nodes", self.current_node, self.next_node)
        self.bdi.set_belief("if_set_ref_traj", "False")
        self.bdi.set_belief('my_id', self.self_uid)

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
    
    async def setup(self):
        # 注册周期任务
        self.add_behaviour(FormationAPFStep(period=DT))
        
    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            """与 Orchestrator 同步, 获取角色, 生成/获取轨迹"""
            
            if self.is_finished:
                yield
                return
                
            cur_start_node = str(agentspeak.grounded(term.args[0], intention.scope))
            cur_end_node = str(agentspeak.grounded(term.args[1], intention.scope))
            

            yield

class MissionOrchestrator:
    """结合key_path_analyzer.log数据, 生成 Persistent Agents 并管理生命周期"""
    def __init__(self, json_data, key_paths, server: str, password: str, asl_file: str, BlueBDIAgentTemplate: BlueUAVAgent):
        self.server = server
        self.password = password
        self.asl_file = asl_file
        self.BlueBDIAgentTemplate = BlueBDIAgentTemplate
        
        # 1. 生成全局飞行计划
        self.uav_flight_plans = self.extract_uav_trajectories(json_data, key_paths)
        print(f"Generated {len(self.uav_flight_plans)} flight plans.")

        self.active_agents: Dict[str, BlueUAVAgent] = {}
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        self.all_trajectories = {}
        
        # 同步队列
        # Key: (from_node, to_node) -> Value: List[agent_id] waiting
        self.edge_queues = collections.defaultdict(list)
        # Key: (from_node, to_node) -> Value: Required count
        self.edge_requirements = {}
        
        # 从 json_data 预填充需求
        for item in json_data:
             u, v = str(item["from"]), str(item["to"])
             # members_num + 1 Leader
             self.edge_requirements[(u, v)] = item["members_num"] + 1



    def extract_uav_trajectories(self, json_data, key_paths):
        # 1. 构建图结构和属性索引
        edge_attrs = {}
        graph = collections.defaultdict(list)
        
        for item in json_data:
            u, v = str(item["from"]), str(item["to"]) # 确保键是字符串
            # members_num + 1 (1个主机 + N个从机)
            total_drones = item["members_num"] + 1 
            edge_attrs[(u, v)] = {
                "count": total_drones,
                "trajectory": item["attrs"]["plan"]["trajectory"]
            }
            graph[u].append(v)

        # 2. 统计所有可能的路径片段并进行路径拆分
        # uav_paths 存储格式: { uav_id: [ [coord1, coord2...], [coord1... ] ] }
        uav_trajectories = []
        
        # 我们需要跟踪每一条边剩余的“可用名额”
        remaining_flow = {edge: attr["count"] for edge, attr in edge_attrs.items()}
        # print("Initial remaining flow:", json.dumps({str(k): v for k, v in remaining_flow.items()}, indent=2))
        
        # 找到所有的起点 (这里根据 key_paths 的第一个元素确定)
        # key_paths 的项类似于 "1_0" (节点名称)
        # 我们需要起始节点。
        starts = set(path[0] for path in key_paths)
        
        for start_node in starts:
            # 查找从该起点出发的总流量
            start_edges = [e for e in remaining_flow if e[0] == start_node]
            total_at_start = sum(remaining_flow[e] for e in start_edges)
            
            for i in range(total_at_start):
                current_node = start_node
                single_uav_path = []
                
                # 随机游走直到没有出边或流量耗尽
                while True:
                    possible_next = [v for v in graph[current_node] if remaining_flow.get((current_node, v), 0) > 0]
                    
                    if not possible_next:
                        break
                    
                    # 随机选择一个还有剩余流量的分支
                    next_node = random.choice(possible_next)
                    
                    # 记录该片段的轨迹
                    edge = (current_node, next_node)
                    single_uav_path.append({
                        "segment": edge,
                        "coords": [] # 占位符
                    })
                    
                    # 消耗一个流量
                    remaining_flow[edge] -= 1
                    current_node = next_node
                
                if single_uav_path:
                    uav_trajectories.append({
                        "id": f'agent_{start_node}_{i}',
                        "path": single_uav_path
                    })

        return uav_trajectories

    async def run(self):
        print("Mission Orchestrator Started (Persistent Mode).")
        
        for plan in self.uav_flight_plans:
            agent_id = plan['id']
            flight_plan = plan['path']
            # 开始位置? 可以在此处指定, 也可以随机
            await self._spawn_persistent_agent(agent_id, flight_plan)

        # 监控任务
        while self.active_agents:
            current_ids = list(self.active_agents.keys())
            all_done = True
            for aid in current_ids:
                agent = self.active_agents[aid]
                if not agent.is_finished:
                    all_done = False
                else:
                    # 如果已完成, 进行清理 (占位)
                    pass 
            
            if all_done:
                break
            await asyncio.sleep(1.0)
            
        print("All persistent missions completed.")
        self.save_trajectories()

    async def _spawn_persistent_agent(self, agent_name, flight_plan):
        jid = f"{agent_name}@{self.server}"
        print(f"\nSpawning persistent agent: {jid} with {len(flight_plan)} segments")
        agent = self.BlueBDIAgentTemplate(jid, self.password, self.asl_file, flight_plan, self)
        await agent.start()
        self.active_agents[agent_name] = agent

    def save_trajectories(self):
        # 从 Redis 提取轨迹
        for name, agent in self.active_agents.items():
            traj_utm = agent.io.get_traj(agent.self_uid)
            if traj_utm:
                traj_np = np.array(traj_utm)
                if traj_np.shape[0]>0:
                    ll = self._lnglat2utm_convertor.utm_to_lng_lat_array(traj_np)
                    self.all_trajectories[name] = {
                        'lats': ll[:,1].tolist(),
                        'lngs': ll[:,0].tolist()
                    }
        # with open('uav_trajectories_persistent.json', 'w') as f:
        #     json.dump(self.all_trajectories, f)
        print("Trajectories collected.")





async def start_agent(server, password):
    # 清空 Redis 数据库，防止历史数据干扰
    try:
        # 假设 Redis 运行在本地默认端口
        r_conn = redis.Redis(host='127.0.0.1', port=6379, db=0)
        r_conn.flushdb()
        print("[System] agent02 code with Redis database flushed successfully.")
    except Exception as e:
        print(f"[System] Warning: Failed to flush Redis: {e}")

    # 从key_paths解析bdi指令
    # key_paths = [
    #     [0, 1, 4, 5, 2, 14],
    #     [3, 4, 5, 2, 14],
    #     [6, 7, 8, 9, 10, 14],
    #     [11, 12, 13, 14]        
    # ]
    key_paths = [
        ["1_0","1_1","1_2","3_0","3_1","4_1","4_2"],
        ["2_0","2_1","2_2","3_0","3_1","5_1","5_2"],
        ["1_0","1_1","1_2","3_0","3_1","6_1","6_2"]
    ]
    # bdi_instructions = KeyPathAnalyzer(key_paths).generate_bdi_instructions()
    # print(f"BDI instructions: {json.dumps(bdi_instructions, indent=2)}")
    orchestrator = MissionOrchestrator(
        json_data=digraph_attrs,
        key_paths=key_paths,
        server=server,
        password=password,
        asl_file=asl_file,
        BlueBDIAgentTemplate=BlueUAVAgent
    )
    
    await orchestrator.run()

if __name__ == "__main__":
    # 启动代码：python -m examples.uavs_strategy.uav_dynamic_agents02
    server = "127.0.0.1"
    passwd = "202127"
    spade.run(start_agent(server, passwd))