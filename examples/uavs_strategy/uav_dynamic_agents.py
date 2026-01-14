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
    def __init__(self, jid, password, asl_file, task_lists, init_pos=None, facilities=None, merge_peers = None, **kwargs):
        super().__init__(jid, password, asl_file)
        # 读取任务列表
        self.task = task_lists[0]
        self.key_path = self.task['path']
        # 读取汇合队友列表 (包含自己)
        self.merge_peers = merge_peers if merge_peers else []
        # 记录当前在key_path路径中的索引
        self.path_index = 0
        # 任务完成标志
        self.is_finished = False
        self.is_final_task = False

        # 初始化设施
        self.facilities = self._default_facilities(facilities)
        # 初始化位置
        self.position = init_pos
        # 经纬度 -> UTM
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        if self.position:
            self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([self.position])).tolist()
            self.traj[0].append(self.position[2])  # 添加高度信息
        else:
            print("No initial position provided, generating random position.")
            _rdm_init_pos = bfunc.generate_circle_positions_from_diameter(1, init_loc1, init_loc2)
            self.position = _rdm_init_pos[0] # 更新 self.position
            self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array(_rdm_init_pos)).tolist()
            self.traj[0].append(self.position[2])  # 添加高度信息
        print(f"{self.jid} initialized at position: {self.position}, traj: {self.traj}")

        # Redis I/O 模块
        self.io = UavRedisIO(**kwargs.get("redis_cfg", {}))
        # Redis 初始化数据
        self.self_uid = jid.split("@")[0] 
        self.io.add_uav_id(self.self_uid, blue=True)
        self.io.set_pos(self.self_uid, self.traj[0][0], self.traj[0][1], self.position[2])
        self.io.set_traj(self.self_uid, [[self.traj[0][0], self.traj[0][1], self.position[2]]])
        self.io.set_lookahead(self.self_uid, 0)

        # 周期性行为
        self.APFStep = APFStep
        self.FetchWorldState = FetchWorldState

        # 轨迹规划类函数
        self.planning_lib = PlanningLib(self)
        # 轨迹相关
        self.cur_reference_traj = []
        # 从机集群参考轨迹
        self.members_cur_reference_traj = []
        self.blue_ids = []
        self.red_ids = []
        self.height_range_set = height_range_value_set
        self.direction_range_set = direction_range_set

        #初始化节点信念
        self.bdi.set_belief("cur_nodes", self.key_path[0], self.key_path[1])
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
        # self.add_behaviour(self.APFStep(period=DT))
        self.add_behaviour(FormationAPFStep(period=DT))
    
    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            """根据当前节点提取路径规划的信息用于后续的轨迹处理"""
            
            # 如果任务已完成，直接跳过
            if self.is_finished:
                yield
                return
            cur_start_node = str(agentspeak.grounded(term.args[0], intention.scope))
            cur_end_node = str(agentspeak.grounded(term.args[1], intention.scope))
            print(f"[{self.jid}] is act_digraph_path_planning from {cur_start_node} to {cur_end_node}")

            current_idx = self.path_index
            for digraph_attr in digraph_attrs:
                if str(digraph_attr['from']) == cur_start_node and str(digraph_attr['to']) == cur_end_node:
                    # 读取当前片段的轨迹规划参数，并设定参考轨迹 
                    print(f"[{self.jid}] cur order_mode:{digraph_attr['attrs']['order_mode']},order_type:{digraph_attr['attrs']['order_type']}")
                    self.planning_lib.execute_path_planning_from_digraph(digraph_attr, -1, -1)
                    _member_num = digraph_attr['members_num']
                    _radius = random.randint(20,30)
                    _angle = random.randint(30,60)
                    _max_offset = random.uniform(30,50)
                    _noise_scale = random.uniform(0.00001,0.00005)
                    _angle_noise_scale = random.uniform(1.0,5.0)
                    _formation_type = random.choice(['circular', 'vertical', 'horizontal', 'vshape', 'arc'])
                    # 处理集群从机队形轨迹
                    fleet_formation_config = Formation_Elements(
                        member_num=_member_num,
                        radius=_radius,
                        angle=_angle,
                        traj=self.cur_reference_traj,
                        max_offset=_max_offset,
                        noise_scale=_noise_scale,
                        angle_noise_scale=_angle_noise_scale,
                        formation_type=_formation_type,
                    ) 
                    members_traj = FormationGenerator3D(formation_elements=fleet_formation_config).generate_members_formation_3d()
                    self.members_cur_reference_traj = members_traj
                    print(f"[{self.jid}] Generated {_formation_type} formation trajectories for {_member_num} members")
                    
                    # 立即为生成的从机初始化 Redis 状态（位置与轨迹起点）
                    # 防止首次 APF 步进时缺失起点导致轨迹滞后 (Master初始化时已有p0, Sub无)
                    for m_i, m_traj in enumerate(members_traj):
                        if m_traj and len(m_traj) > 0:
                            s_uid = f"{self.self_uid}_sub_{m_i}"
                            # 仅当 Redis 中没有该从机历史时才初始化 (避免覆盖多段任务的历史)
                            exist_traj = self.io.get_traj(s_uid)
                            if not exist_traj:
                                s_start = m_traj[0]
                                self.io.add_uav_id(s_uid, blue=True)
                                self.io.set_pos(s_uid, s_start[0], s_start[1], s_start[2])
                                self.io.set_traj(s_uid, [[s_start[0], s_start[1], s_start[2]]])
                                print(f"[{self.jid}] Initialized Redis for new sub-agent: {s_uid}")

                    # 规划完成后，设置标志位通知 APFStep 将新轨迹写入 Redis
                    # self.bdi.set_belief("if_set_ref_traj", "true")
                    print(f"[{self.jid}] cur if_set_ref_traj flag is:{self.bdi.get_belief_value('if_set_ref_traj')[0]}")
            # 在 key_path 中查找当前节点的位置，并更新为下一段路径
            try:
                if current_idx + 1 < len(self.key_path):
                    # key_path[current_idx] -> key_path[current_idx+1]
                    next_idx = current_idx + 1
                    if next_idx + 1 < len(self.key_path):
                        next_start = self.key_path[next_idx]
                        next_end = self.key_path[next_idx + 1]
                        # 更新belief和索引
                        self.bdi.set_belief("cur_nodes", next_start, next_end)
                        self.path_index = next_idx 
                    else:
                        print(f"{self.jid} Reached end of path.")
                        self.is_final_task = True
                
            except ValueError:
                print(f"Error: Could not find nodes {cur_start_node} or {cur_end_node} in key_path {self.key_path}")

            yield 

class MissionOrchestrator:
    """结合key_path_analyzer.log数据,生成bdi agent并管理生命周期"""
    def __init__(self, bdi_instructions: Dict[str, Any], server: str, password: str, asl_file: str, BlueBDIAgentTemplate: BlueUAVAgent):
        self.bdi_instructions = bdi_instructions
        self.server = server
        self.password = password
        self.asl_file = asl_file
        self.BlueBDIAgentTemplate = BlueBDIAgentTemplate

        self.all_trajectories = {}
        # parent -> list of children
        self.agent_lineage = collections.defaultdict(list) 
        self.active_agents: Dict[str, BlueUAVAgent] = {}
        # 经纬度 -> UTM 转换器
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        
        # Merge synchronization
        # key: next_segment_hint (e.g., 'seg_1'), value: set of agent_names ready to merge
        self.pending_merges = collections.defaultdict(set) 
        # key: next_segment_hint, value: total number of agents expected
        self.merge_requirements = self._calculate_merge_requirements()
        # key: next_segment_hint, value: list of agent_names involved
        self.merge_groups = self._calculate_merge_groups()

    def _calculate_merge_groups(self):
        """
        预计算每个汇合点涉及的 agent 列表
        Returns:
            dict: { 'seg_4': ['agent_1', 'agent_2', ...], ... }
        """
        groups = collections.defaultdict(list)
        for agent_name, tasks in self.bdi_instructions.items():
            for task in tasks:
                if task.get('action_at_end') == 'merge_and_terminate':
                    target = task.get('next_segment_hint')
                    groups[target].append(agent_name)
        return groups

    def _calculate_merge_requirements(self):
        """预计算每个汇合点需要多少个agent"""
        reqs = collections.defaultdict(int)
        for agent_name, tasks in self.bdi_instructions.items():
            for task in tasks:
                if task.get('action_at_end') == 'merge_and_terminate':
                    target = task.get('next_segment_hint')
                    reqs[target] += 1
        return reqs

    def _find_agent_name_by_segment_id(self, segment_id):
        """根据segment_id找到对应的agent配置名"""
        for name, tasks in self.bdi_instructions.items():
            # 假设每个agent配置的第一条任务定义了它的起始段
            if tasks and tasks[0]['segment_id'] == segment_id:
                return name
        return None

    async def _spawn_agent(self, agent_name ,_init_pos = None):
        if agent_name not in self.bdi_instructions:
            print(f"Error: No instructions for {agent_name}")
            return

        tasks = self.bdi_instructions[agent_name]
        # 目前假设每个agent主要执行第一个任务段配置
        task = tasks[0] 
        path = task['path']

        # 获取当前任务的汇合队友列表
        merge_peers = []
        if task.get('action_at_end') == 'merge_and_terminate':
            target_seg = task.get('next_segment_hint')
            # self.merge_groups 需要在 __init__ 中初始化: self.merge_groups = self._calculate_merge_groups()
            merge_peers = self.merge_groups.get(target_seg, [])
        
        # 构建JID
        jid = f"{agent_name}@{self.server}"
        print(f"Spawning agent: {jid} with path len {len(path)}")
        
        """
            注册运行bdi智能体，根据nodes查询任务列表并执行轨迹规划与位置
            依次执行每一段的任务规划，每当执行完当前阶段的规划与推理后，才更改标志位agent.is_finished=True(以及其他标志位)
            再返回到MissionOrchestrator中的run进行下一步处理
            队形数据目前的方法是添加一点随机数据
        """
        agent = self.BlueBDIAgentTemplate(jid, self.password, self.asl_file, tasks, _init_pos, facilities=facilities_file, merge_peers=merge_peers)
        
        # 初始化信念
        if len(path) >= 2:
            agent.bdi.set_belief("cur_nodes", path[0], path[1])
        
        await agent.start()
        self.active_agents[agent_name] = agent

    def _reconstruct_full_trajectories(self):
        # 1. 准备工作：构建子节点到父节点的反向映射，用于确定合并时的顺序
        child_to_parents = collections.defaultdict(list)
        for parent, children in self.agent_lineage.items():
            for child in children:
                child_to_parents[child].append(parent)

        # 按照字符串顺序对父节点排序 (确保 Determinism: 字母序最小的父节点在合并时被视为 Master 身份的继承者)
        for child in child_to_parents:
            child_to_parents[child].sort()
        print(f"Child to Parents Mapping: {json.dumps(dict(child_to_parents), indent=2)}")
        print(f"Agent Lineage Mapping: {json.dumps(dict(self.agent_lineage), indent=2)}")

        # 统计每个主从机编队（Agent）实际上拥有多少个从机（Sub-agents）
        _agent_sub_counts = collections.defaultdict(int)
        for key in self.all_trajectories.keys():
            if "_sub_" in key:
                # 解析 "agent_name_sub_idx"
                try:
                    parts = key.rsplit("_sub_", 1)
                    if len(parts) == 2:
                        agent_name = parts[0]
                        idx = int(parts[1])
                        # 记录 count 为 max_index + 1
                        if idx + 1 > _agent_sub_counts[agent_name]:
                            _agent_sub_counts[agent_name] = idx + 1
                except ValueError:
                    continue
        
        def get_sub_count(agent_name):
            return _agent_sub_counts.get(agent_name, 0)

        # 3. 识别 Roots (没有父节点的 Master)
        all_children = set()
        for children in self.agent_lineage.values():
            all_children.update(children)
        
        executed_masters = [k for k in self.all_trajectories.keys() if "_sub_" not in k]
        roots = [a for a in executed_masters if a not in all_children]
        
        reconstructed = {}
        
        for root in roots:
            paths = self._trace_paths(root)
            print(f"Reconstructing trajectories for root {root} with path: {json.dumps(paths, indent=2)}.")
            for i, path in enumerate(paths):
                root_sub_count = get_sub_count(root)
                
                # 初始化容器
                # 'master' 存储 Root Master 的轨迹
                # 数字键存储 Root Sub k 的轨迹
                path_trajs = {'master': {'lats': [], 'lngs': []}}
                for idx in range(root_sub_count):
                    path_trajs[idx] = {'lats': [], 'lngs': []}
                
                # 追踪当前的逻辑角色 (Logical Role) 映射到 物理实体 (Physical Entity)
                # 初始状态下（在 Root 节点）：
                # Root Master -> Physical Master ('master')
                # Root Sub k  -> Physical Sub k  (k)
                # 随着只有 step 向前推进，我们更新这个映射关系。
                
                # mapping: { logical_id (str/int) -> physical_role (str/int) }
                # logical_id: 'master' or int (sub index)
                # physical_role: 'master' or int (sub index in current agent) 或 None (如果该角色在当前层级消失/被挤出)
                
                current_mapping = {'master': 'master'}
                for idx in range(root_sub_count):
                    current_mapping[idx] = idx
                    
                for step_idx, agent_name in enumerate(path):
                    # 如果不是起点，计算从 prev -> curr 的映射变换
                    if step_idx > 0:
                        prev_name = path[step_idx - 1]

                        # --- 1. Split Logic (Prev -> (..., Curr, ...)) ---
                        # 判断 prev 是否分裂为多个子节点，如果是，计算当前 child (curr) 继承的分片
                        prev_siblings = self.agent_lineage[prev_name]
                        split_offset = 0
                        split_capacity = 999999 

                        if len(prev_siblings) > 1:
                            try:
                                # 计算在分裂中的位次
                                my_rank_in_split = prev_siblings.index(agent_name)
                                # 累加前面的兄弟占据的份额
                                for sib in prev_siblings[:my_rank_in_split]:
                                    split_offset += (1 + get_sub_count(sib))
                                
                                # 当前节点的“接收容量”
                                split_capacity = 1 + get_sub_count(agent_name)
                            except ValueError:
                                pass 

        #                 # --- 2. Merge Logic ((..., Prev, ...) -> Curr) ---
        #                 # 确定 prev 在 curr 的合并列表中的位置
        #                 parents = child_to_parents[agent_name]
                        
        #                 try:
        #                     parent_rank = parents.index(prev_name)
        #                 except ValueError:
        #                     # 理论上不应发生，除非 lineage 数据不一致
        #                     print(f"Warning: {prev_name} is not in parents of {agent_name}")
        #                     parent_rank = 0

        #                 # 计算 prev 在 curr 中的“起始 Sub 偏移量”
        #                 # 规则：
        #                 # Initiator (rank 0): Master->Master, Subs->Subs [0...N]
        #                 # Follower (rank > 0): Master->Sub [offset], Subs->Subs [offset+1...offset+1+N]
        #                 # offset 累加之前所有 sibling 的 (1(Master) + Sub_Count)
                        
        #                 base_sub_offset = 0
        #                 is_initiator = (parent_rank == 0)
                        
        #                 if not is_initiator:
        #                     # 累加前面的兄弟占用的位置
        #                     # 优化循环：直接利用 enumerate 判断是否为 initiator，避免重复查找
        #                     for idx, sibling in enumerate(parents):
        #                         if idx == parent_rank:
        #                             break
        #                         sibling_sub_count = get_sub_count(sibling)
        #                         sibling_is_initiator = (idx == 0)
        #                         if sibling_is_initiator:
        #                             base_sub_offset += sibling_sub_count
        #                         else:
        #                             base_sub_offset += (1 + sibling_sub_count)
                        
        #                 # --- 3. Execute Mapping Update ---
        #                 next_mapping = {}
                        
        #                 for logical_id, phys_role in current_mapping.items():
        #                     if phys_role is None:
        #                         next_mapping[logical_id] = None
        #                         continue

        #                     # Step A: Convert phys_role (in Prev) to Linear Index
        #                     curr_linear_in_prev = 0 if phys_role == 'master' else (phys_role + 1)

        #                     # Step B: Apply Split Filter
        #                     # 相对索引 = 绝对索引 - 分裂偏移
        #                     rel_idx = curr_linear_in_prev - split_offset
                            
        #                     # check bounds
        #                     if rel_idx < 0 or rel_idx >= split_capacity:
        #                         # 这个实体被分给了其他分裂分支，不在当前路径中
        #                         next_mapping[logical_id] = None
        #                         continue

        #                     # Step C: Convert Back to "Transfer Role" (as if 1-to-1 transfer)
        #                     transfer_phys_role = 'master' if rel_idx == 0 else (rel_idx - 1)
                            
        #                     # Step D: Apply Merge Offset to land in Curr
        #                     new_phys_role = None
                            
        #                     if is_initiator:
        #                         # Initiator: 角色保持不变 (Master->Master, Sub k->Sub k) (相对于 Transfer Role)
        #                         new_phys_role = transfer_phys_role 
        #                     else:
        #                         # Follower: 全部降级为 Sub
        #                         if transfer_phys_role == 'master':
        #                             new_phys_role = base_sub_offset
        #                         elif isinstance(transfer_phys_role, int):
        #                             new_phys_role = base_sub_offset + 1 + transfer_phys_role
        #                         else:
        #                             new_phys_role = None
                                    
        #                     next_mapping[logical_id] = new_phys_role
                        
        #                 current_mapping = next_mapping

        #             # --- 数据提取 ---
        #             # 根据 current_mapping 提取当前 agent_name 下的数据
        #             for logical_id, phys_role in current_mapping.items():
        #                 # logical_id: 'master' (Root Master) or int (Root Sub k)
                        
        #                 target_key = None
        #                 if phys_role == 'master':
        #                     target_key = agent_name
        #                 elif isinstance(phys_role, int):
        #                     target_key = f"{agent_name}_sub_{phys_role}"
                        
        #                 if target_key:
        #                     data = self.all_trajectories.get(target_key)
        #                     if data:
        #                         path_trajs[logical_id]['lats'].extend(data.get('lats', []))
        #                         path_trajs[logical_id]['lngs'].extend(data.get('lngs', []))

        #         # 5. 保存结果
        #         branch_suffix = f"_branch_{i}" if len(paths) > 1 else ""
                
        #         # 保存 Master
        #         if path_trajs['master']['lats']:
        #             reconstructed[f"{root}{branch_suffix}"] = {
        #                 'lats': path_trajs['master']['lats'],
        #                 'lngs': path_trajs['master']['lngs'],
        #                 'ts': list(range(len(path_trajs['master']['lats'])))
        #             }
                
        #         # 保存 Subs
        #         for idx in range(root_sub_count):
        #             if not path_trajs[idx]['lats']: continue
        #             reconstructed[f"{root}_sub_{idx}{branch_suffix}"] = {
        #                 'lats': path_trajs[idx]['lats'],
        #                 'lngs': path_trajs[idx]['lngs'],
        #                 'ts': list(range(len(path_trajs[idx]['lats'])))
        #             }
        
        # print(f"Traj Reconstruction Report:")
        # print(f"  - Identifying independent drone entities from {len(roots)} root groups.")
        # print(f"  - Total independent trajectories reconstructed: {len(reconstructed)}")
        # for key in reconstructed:
        #     points = len(reconstructed[key]['ts'])
        #     print(f"    * Entity '{key}': {points} points")
        _dict_str = {}
        # _dict_str = {
        #     "uavs_coords_str": reconstructed
        # }
        return _dict_str if _dict_str else {}

    def _trace_paths(self, current_node):
        children = self.agent_lineage.get(current_node, [])
        if not children:
            return [[current_node]]
        
        paths = []
        for child in children:
            child_paths = self._trace_paths(child)
            for cp in child_paths:
                paths.append([current_node] + cp)
        return paths

    async def run(self):
        print("Mission Orchestrator Started.")
        
        # 1. 启动初始 Agents (名字不包含 _sub_ 或 _merged_ 的)
        initial_agents = [name for name in self.bdi_instructions.keys() 
                            if "_sub_" not in name and "_merged_" not in name]
        
        for _idx, name in enumerate(initial_agents):
            if _idx < len(init_locs):
                await self._spawn_agent(name,init_locs[_idx])
            else:
                await self._spawn_agent(name)

        # 2. 监控循环
        while self.active_agents:
            # 复制 keys 以便在循环中修改字典
            current_agent_names = list(self.active_agents.keys())
            
            for name in current_agent_names:
                if name not in self.active_agents:
                    continue
                agent = self.active_agents[name]
                
                if agent.is_finished:
                    await self._handle_agent_completion(name, agent)
            
            await asyncio.sleep(0.5)
        
        print("All missions completed.")
        
        reconstructed_trajs = self._reconstruct_full_trajectories()
        # reconstructed_trajs.update(json.load(open(facilities_file, "r")))
        # with open('uav_trajectories_from_BDI.json', 'w') as f:
        #     json.dump(reconstructed_trajs, f, indent=2)
        # print("Reconstructed trajectories saved to uav_trajectories_from_BDI.json")

    async def _handle_agent_completion(self, agent_name, agent):
        # Extract and store trajectory data
        traj_utm = agent.io.get_traj(agent.self_uid)
        if traj_utm:
            traj_utm_np = np.array(traj_utm)
            if traj_utm_np.shape[0] > 0:
                lng_lat = self._lnglat2utm_convertor.utm_to_lng_lat_array(traj_utm_np)
                self.all_trajectories[agent_name] = {
                    # 'uav_name': agent_name,
                    'lats': lng_lat[:, 1].tolist(),
                    'lngs': lng_lat[:, 0].tolist(),
                    'ts': list(range(len(traj_utm)))
                }

        # Extract sub-agent trajectories
        sub_idx = 0
        while True:
            sub_uid = f"{agent.self_uid}_sub_{sub_idx}"
            sub_traj_utm = agent.io.get_traj(sub_uid)
            if not sub_traj_utm:
                break
            
            sub_traj_np = np.array(sub_traj_utm)
            if sub_traj_np.shape[0] > 0:
                sub_agent_key = f"{agent_name}_sub_{sub_idx}"
                lng_lat_sub = self._lnglat2utm_convertor.utm_to_lng_lat_array(sub_traj_np)
                self.all_trajectories[sub_agent_key] = {
                    'lats': lng_lat_sub[:, 1].tolist(),
                    'lngs': lng_lat_sub[:, 0].tolist(),
                    'ts': list(range(len(sub_traj_utm)))
                }
            sub_idx += 1

        # 获取该agent的任务配置
        tasks = self.bdi_instructions[agent_name]
        task = tasks[0]
        action = task['action_at_end']
        _last_pos = agent.traj[-1] if agent.traj else None
        if action == "finish":
            print(f"Agent {agent_name} finished mission. Terminating.")
            await agent.stop()
            del self.active_agents[agent_name]
            
        elif action == "split_and_terminate":
            print(f"Agent {agent_name} splitting.")

            await agent.stop()
            del self.active_agents[agent_name]

            for branch in task['branches']:
                child_name = branch['new_agent_hint']
                self.agent_lineage[agent_name].append(child_name)
                _last_pos_llt = self._lnglat2utm_convertor.utm_to_lng_lat_array(np.array([_last_pos]))[0].tolist() + [_last_pos[2]]
                await self._spawn_agent(child_name, _init_pos=_last_pos_llt)
            

                
        elif action == "merge_and_terminate":
            target_seg = task['next_segment_hint']
            
            # 只有当该agent还没在等待列表中时才添加 (防止重复处理)
            if agent_name not in self.pending_merges[target_seg]:
                print(f"Agent {agent_name} ready to merge into {target_seg}. Waiting for others...")
                self.pending_merges[target_seg].add(agent_name)
            
            # 检查是否满足合并条件
            required_count = self.merge_requirements[target_seg]
            current_count = len(self.pending_merges[target_seg])
            
            if current_count >= required_count:
                print(f"Merge condition met for {target_seg} ({current_count}/{required_count}). Merging...")
                
                # Capture participants before clearing
                participants = list(self.pending_merges[target_seg])
                
                # 1. 停止所有参与合并的 agent
                for participant in participants:
                    if participant in self.active_agents:
                        p_agent = self.active_agents[participant]
                        await p_agent.stop()
                        del self.active_agents[participant]
                
                # 2. 清理等待列表
                del self.pending_merges[target_seg]
                
                # 3. 启动合并后的新 agent
                new_agent_name = self._find_agent_name_by_segment_id(target_seg)
                if new_agent_name:
                    # Update lineage
                    for participant in participants:
                        self.agent_lineage[participant].append(new_agent_name)
                        
                    _last_pos_llt = self._lnglat2utm_convertor.utm_to_lng_lat_array(np.array([_last_pos]))[0].tolist() + [_last_pos[2]]
                    await self._spawn_agent(new_agent_name, _init_pos=_last_pos_llt)
                else:
                    print(f"Error: Could not find agent definition for segment {target_seg}")
            else:
                # 尚未就绪，Agent 保持运行状态 (is_finished=True)，实际上是在"原地等待"
                pass



async def start_agent(server, password):
    # 清空 Redis 数据库，防止历史数据干扰
    try:
        # 假设 Redis 运行在本地默认端口
        r_conn = redis.Redis(host='127.0.0.1', port=6379, db=0)
        r_conn.flushdb()
        print("[System] Redis database flushed successfully.")
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
    bdi_instructions = KeyPathAnalyzer(key_paths).generate_bdi_instructions()
    print(f"BDI instructions: {json.dumps(bdi_instructions, indent=2)}")
    orchestrator = MissionOrchestrator(
        bdi_instructions=bdi_instructions,
        server=server,
        password=password,
        asl_file=asl_file,
        BlueBDIAgentTemplate=BlueUAVAgent
    )
    
    await orchestrator.run()

if __name__ == "__main__":
    # 启动代码：python -m examples.uavs_strategy.uav_dynamic_agents
    server = "127.0.0.1"
    passwd = "202127"
    spade.run(start_agent(server, passwd))