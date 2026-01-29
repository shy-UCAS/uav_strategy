# agent_dynamic_agents02将所有无人机看做一个独立的agent，不再区分每个任务阶段设置一个agent
# 所有agent任务完成后不停止,等待下一个任务指令
# 通过MissionOrchestrator统一管理所有Persistent Agents的生命周期
# 每个agent根据key_paths生成的bdi_instructions执行任务
# 每个agent根据当前航段的属性决定是否需要与同航段其他agent同步
# 如果需要同步，则等待其他agent准备完毕后再开始当前航段任务
# 每个agent在执行航段任务前，先检查当前航段是否已有基准参考轨迹
# 如果没有，则生成基准参考轨迹，并根据编队参数生成所有成员的轨迹，存储到redis服务器
# 如果有，则直接从redis服务器获取自己的轨迹
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
import networkx as nx

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
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import FormationAPFStep, Single_APFStep, FetchWorldState, SyncAPFStep ,DT
from examples.uavs_strategy.key_path_analyzer import KeyPathAnalyzer

init_loc1 = [
    122.18105710089186,
    37.51299467977935,
    200.0
]

init_loc2 = [
    122.16096051695042,
    37.497235513573486,
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
    def __init__(self, jid, password, asl_file, flight_plan, siblings_ref ,orchestrator, init_pos=None, facilities=None, **kwargs):
        super().__init__(jid, password, asl_file)
        self.flight_plan = flight_plan  # 航段列表: [{'segment': (u, v), 'coords': []}, ...]
        self.siblings_ref = siblings_ref
        self.orchestrator = orchestrator
        self.path_index = 0
        self.is_finished = False
        self._need_wait_siblings = False
        self._is_alive = True
        self.is_final_task = False
        
        # 确定起始节点
        if self.flight_plan:
            self.current_node = self.flight_plan[0][0]
            self.next_node = self.flight_plan[0][1]
        
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

        # 汇聚时sibling无人机列表
        self.merge_peers = []
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
        
        # Sync attributes
        self.current_segment_siblings = []
        self.current_segment_key = None
        self.has_synced_segment = False

        self.APFStep = SyncAPFStep
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
        
        self.formation_type = "unknown" # 初始化队形类型

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
    
    def add_achievement_goal(self, name, *args):
        """添加一个成就目标到意图缓冲区
        """
        new_args = ()
        for x in args:
            if type(x) == str:
                new_args += (agentspeak.Literal(x),)
            else:
                new_args += (x,)
        term = agentspeak.Literal(name, tuple(new_args))
        self.bdi_intention_buffer.append((agentspeak.Trigger.addition, agentspeak.GoalType.achievement, term, agentspeak.runtime.Intention()))
 

    async def setup(self):
        # 注册周期任务
        self.add_behaviour(SyncAPFStep(period=DT))
        
    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            """与 Orchestrator 同步, 获取角色, 生成/获取轨迹"""
            
            if self.is_finished:
                yield
                return
                
            cur_start_node = str(agentspeak.grounded(term.args[0], intention.scope))
            cur_end_node = str(agentspeak.grounded(term.args[1], intention.scope))
            
            # Reset Sync Flag for new segment
            self.has_synced_segment = False
            self.current_segment_key = f"{cur_start_node}_{cur_end_node}"
            
            print(f"[{self.self_uid}] Planning path from node {cur_start_node} to node {cur_end_node}")
            for digraph_attr in digraph_attrs:
                if str(digraph_attr["from"]) == cur_start_node and str(digraph_attr["to"]) == cur_end_node:
                    _order_mode = digraph_attr['attrs']['order_mode']
                    _order_target = digraph_attr['attrs']['target']
                    if _order_mode == 'aggregate' and _order_target == 'aggregate_point':
                        self._merge_ready_flag = False
                        # 先清空或初始化一个临时列表
                        all_merge_peers = []
                        for k, v in self.siblings_ref.items():
                            # 找到所有以当前目标点为终点的航段（入边）
                            if k[1] == cur_end_node:
                                # 累加这些航段上的所有无人机（排除自己）
                                peers = [_uav for _uav in v['uav_ids'] if _uav != self.self_uid]
                                all_merge_peers.extend(peers)
                        
                        # 去重
                        self.merge_peers = list(set(all_merge_peers))
                        print(f"[{self.self_uid}] Merge peers for aggregation: {self.merge_peers}")
                    else:
                        self.merge_peers = []

                    # 获取当前航段的所有成员ID
                    cur_siblings_ids = self.siblings_ref.get((cur_start_node, cur_end_node), {}).get("uav_ids", [])
                    self.current_segment_siblings = cur_siblings_ids
                    print(f"[{self.self_uid}] Current segment siblings: {cur_siblings_ids}")
                    # 检索当前航段的基准参考轨迹是否存在，如果不存在说明当前agent是第一个执行该航段的，需要设定参考基准轨迹，后续agent就可以直接获取
                    base_ref_traj = self.io.get_nodes_pair_base_ref_traj(cur_start_node, cur_end_node)
                    
                    if not base_ref_traj or len(base_ref_traj) == 0:    
                        print(f"[{self.self_uid}] No base reference traj found for segment {cur_start_node}->{cur_end_node}. Now generate it.")
                        # 1. 生成基准参考轨迹
                        base_ref_traj = self.planning_lib.execute_path_planning_from_digraph(digraph_attr, -1, -1)
                        # 存储基准参考轨迹
                        self.io.set_nodes_pair_base_ref_traj(cur_start_node, cur_end_node, base_ref_traj)
                        # 2. 生成编队配置参数
                        _member_num = digraph_attr['members_num']
                        _radius = random.randint(20,30)
                        _angle = random.randint(30,60)
                        _max_offset = random.uniform(30,50)
                        _noise_scale = random.uniform(0.00001,0.00005)
                        _angle_noise_scale = random.uniform(1.0,5.0)
                        _formation_type = random.choice(['circular', 'vertical', 'horizontal', 'vshape', 'arc'])
                        
                        # 保存 formation_type 到 Redis 以供后续加入的成员查询
                        self.io.r.set(f"formation_type:{cur_start_node}:{cur_end_node}", _formation_type)
                        
                        # 处理集群从机队形轨迹
                        fleet_formation_config = Formation_Elements(
                            member_num=_member_num+1,
                            radius=_radius,
                            angle=_angle,
                            traj=base_ref_traj,
                            max_offset=_max_offset,
                            noise_scale=_noise_scale,
                            angle_noise_scale=_angle_noise_scale,
                            formation_type=_formation_type,
                        ) 
                        
                        # 3. 生成并存储所有成员的轨迹,只执行一次，后续成员从redis获取
                        members_traj_map = FormationGenerator3D(formation_elements=fleet_formation_config).generate_members_formation_map(cur_siblings_ids)
                        
                        for m_uid, m_traj in members_traj_map.items():
                            self.io.set_nodes_pair_member_traj(cur_start_node, cur_end_node, m_uid, m_traj)
                        
                        print(f"[{self.self_uid}] Generated and saved trajectories for {len(members_traj_map)} members.")

                    else:
                        print(f"[{self.self_uid}] Found existing base reference trajectory for segment {cur_start_node} -> {cur_end_node}.")
                        # 尝试获取此航段的 formation_type
                        _ft = self.io.r.get(f"formation_type:{cur_start_node}:{cur_end_node}")
                        # 兼容 decode_responses=True 和 False 的情况
                        if isinstance(_ft, bytes):
                             _formation_type = _ft.decode('utf-8')
                        else:
                             _formation_type = _ft if _ft else "unknown"

                    self.formation_type = _formation_type
                    
                    # 4. 获取属于自己的那条轨迹
                    my_traj = []
                    my_traj = self.io.get_nodes_pair_member_traj(cur_start_node, cur_end_node, self.self_uid)
                    
                    if my_traj:
                         print(f"[{self.self_uid}] Successfully retrieved my formation trajectory (len={len(my_traj)}).")
                         self.cur_reference_traj = my_traj
                         
                         #  保证轨迹是连续拼接
                         if not self.traj:
                             self.traj.extend(my_traj)
                         else:
                             last_pt = np.array(self.traj[-1][:2]) # xy only
                             first_pt = np.array(my_traj[0][:2])
                             dist = np.linalg.norm(last_pt - first_pt)
                             

                             if dist < 1.0:
                                 self.traj.extend(my_traj[1:])
                             else:
                                 print(f"[{self.self_uid}] Detect gap ({dist:.2f}m) between segments. Keeping full trajectory.")
                                 self.traj.extend(my_traj)

                         self.bdi.set_belief("if_set_ref_traj", "true")
                    else:
                         print(f"[{self.self_uid}] FAILED to retrieve formation trajectory after retries!")

            # 更新 path_index 以指向下一段航程
            current_idx = self.path_index
            if current_idx + 1 < len(self.flight_plan):
                next_idx = current_idx + 1
                next_start = self.flight_plan[next_idx][0]
                next_end = self.flight_plan[next_idx][1]
                
                # 更新 BDI 信念，以便 agent 能够触发对下一段的处理
                print(f"[{self.self_uid}] Segment planned. Prepared next belief: cur_nodes({next_start}, {next_end})")
                self.bdi.set_belief("cur_nodes", next_start, next_end)
                self.path_index = next_idx
            else:
                 # 已经是最后一段
                 print(f"[{self.self_uid}] All segments in flight plan are planned. Marking as final task.")
                 self.is_final_task = True
            
            # 将自己的状态设置为 ready，表示已经准备好执行任务（或者已经开始）
            # 注意：实际任务开始是在 APFStep 中判断 can_task_start
            # 这里先不设置 True，因为还要等 can_task_start 真正变成 True（即上一段飞完）
            # 或者我们可以认为 "Plan" 完这一步就代表我想进入下一阶段的状态了
            # 更好的做法是在 APFStep 结束上一段时设置 "ready_for_next"
            # 暂时我们只在这里处理路径规划本身的逻辑。

            yield

class MissionOrchestrator:
    """结合key_path_analyzer.log数据, 生成 Persistent Agents 并管理生命周期"""
    def __init__(self, json_data, key_paths, server: str, password: str, asl_file: str, BlueBDIAgentTemplate: BlueUAVAgent):
        self.server = server
        self.password = password
        self.asl_file = asl_file
        self.BlueBDIAgentTemplate = BlueBDIAgentTemplate
        
        # 1. 生成全局飞行计划
        self.uav_flight_plans, self.edge_attrs = self.extract_uav_trajectories(json_data, key_paths)
        print(f"Generated {len(self.uav_flight_plans)} flight plans.")
        print(json.dumps(self.uav_flight_plans, indent=2))
        print("Edge attributes with assigned UAV IDs:")
        print(json.dumps({f"{k[0]}->{k[1]}": v for k, v in self.edge_attrs.items()}, indent=2))
        self.active_agents: Dict[str, BlueUAVAgent] = {}
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        self.all_trajectories = {}
        self._plan_graph = nx.DiGraph()
        
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

    def _init_DAG_structure(self):
        for k, v in self.edge_attrs.items():
            self._plan_graph.add_edge(k[0], k[1], count=v["count"], uav_ids=v["uav_ids"])


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
                "uav_ids": []
            }
            graph[u].append(v)

        # 2. 统计所有可能的路径片段并进行路径拆分
        # uav_paths 存储格式: { uav_id: [ [coord1, coord2...], [coord1... ] ] }
        uav_trajectories = []
        
        # 我们需要跟踪每一条边剩余的“可用名额”
        remaining_flow = {edge: attr["count"] for edge, attr in edge_attrs.items()}
        
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
                    single_uav_path.append(edge)
                    
                    # 消耗一个流量
                    remaining_flow[edge] -= 1
                    current_node = next_node
                
                if single_uav_path:
                    sorted_starts = sorted(list(starts))
                    _idx = sorted_starts.index(start_node)
                    uav_trajectories.append({
                        "id": f'agent_{_idx+1}_{i}',
                        "path": single_uav_path
                    })
        
        for _traj in uav_trajectories:
            for seg in _traj['path']:
                if (seg[0], seg[1]) in edge_attrs.keys():
                    edge_attrs[(seg[0], seg[1])]["uav_ids"].append(_traj["id"])


        return uav_trajectories, edge_attrs

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
        agent = self.BlueBDIAgentTemplate(jid, self.password, self.asl_file, flight_plan, self.edge_attrs, self)
        await agent.start()
        self.active_agents[agent_name] = agent

    def save_trajectories(self):
        print("Collecting trajectories and facility info...")
        
        # 1. Load Facilities Info
        facilities_data = {}
        if os.path.exists(facilities_file):
            with open(facilities_file, 'r', encoding='utf-8') as f:
                facilities_data = json.load(f)
        
        facilities_str = facilities_data.get('facilities_str', {})
        defence_rings = facilities_data.get('defence_rings', {})

        for _ring_name, _ring_llgs in defence_rings.items():
            flat = []
            for _lng, _lat in zip(_ring_llgs['lngs'], _ring_llgs['lats']):
                flat.extend([_lng, _lat])
            facilities_str[_ring_name.upper()] = flat
        
        # 2. Collect UAV Trajectories
        uavs_coords = {}
        
        for name, agent in self.active_agents.items():
            # Get trajectory from Redis
            traj_utm = agent.io.get_traj(agent.self_uid)
            traj_extra = agent.io.get_traj_extra(agent.self_uid)
            
            if traj_utm:
                traj_np = np.array(traj_utm)
                if traj_np.shape[0] > 0:
                    # Convert to Lat/Lon
                    ll = self._lnglat2utm_convertor.utm_to_lng_lat_array(traj_np)
                    lats = ll[:, 1].tolist()
                    lngs = ll[:, 0].tolist()
                    
                    # Extra Info (now synced via atomic write)
                    aligned_extras = traj_extra
                    
                    # 3. Generate Timesteps
                    # Assuming DT seconds per step
                    ts = [i  for i in range(len(lats))]

                    uavs_coords[name] = {
                        "lats": lats,
                        "lngs": lngs,
                        "ts": ts,
                        "extras": aligned_extras
                    }
        
        # 3. Construct Final Data Structure
        final_data = {
            "uavs_coords_str": uavs_coords,
            "facilities_str": facilities_str,
            "defence_rings": defence_rings
        }
        
        # 4. Save to File
        output_file = "uav_trajectories_persistent.json"
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(final_data, f, indent=4)
            
        print(f"All data saved to {output_file}")





async def start_agent(server, password):
    # 清空 Redis 数据库，防止历史数据干扰
    try:
        # 假设 Redis 运行在本地默认端口
        r_conn = redis.Redis(host='127.0.0.1', port=6379, db=0)
        r_conn.flushdb()
        print("[System] uav_dynamic_agents02 with Redis database flushed successfully.")
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