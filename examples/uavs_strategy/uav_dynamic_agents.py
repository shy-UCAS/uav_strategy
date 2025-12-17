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

# from time import time
from datetime import datetime

from sympy import N

from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import PlanningLib
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import FormationAPFStep, APFStep, FetchWorldState ,DT

complex_key_paths = [
    [20, 21, 5, 27], 
    [28, 3, 21, 5, 27], 
    [1, 2, 10, 26], 
    [30, 2, 10, 26], 
    [29, 22, 2, 10, 26], 
    [20, 21, 5, 7, 25], 
    [1, 2, 10, 12, 4, 8], 
    [1, 2, 10, 12, 11, 24, 16]
]

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
digraph_attrs = json.load(open(digraph_attrs_reference_path, "r"))

# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, task_lists, init_pos=None, facilities=None, merge_peers = None  ,**kwargs):
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
        self.facilities = self._default_facilities() if facilities is None else facilities
        # 初始化位置
        self.position = init_pos
        # 经纬度 -> UTM
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        if self.position:
            self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([self.position])).tolist()
        else:
            _rdm_init_pos = bfunc.generate_circle_positions_from_diameter(1, init_loc1, init_loc2)
            self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([_rdm_init_pos])).tolist()

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
        self.planning_lib = PlanningLib
        # 轨迹相关
        self.cur_reference_traj = []
        self.blue_ids = []
        self.red_ids = []
        self.height_range_set = height_range_value_set
        self.direction_range_set = direction_range_set

        #初始化节点信念
        self.bdi.set_belief("cur_nodes", self.key_path[0], self.key_path[1])

    
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
        
    
    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            """根据当前节点提取路径规划的信息用于后续的轨迹处理"""
            
            # 如果任务已完成，直接跳过
            if self.is_finished:
                # print(f"{self.jid} has already finished. Skipping planning.")
                yield
                return
            cur_start_node = str(agentspeak.grounded(term.args[0], intention.scope))
            cur_end_node = str(agentspeak.grounded(term.args[1], intention.scope))
            print(f"{self.jid} is act_digraph_path_planning from {cur_start_node} to {cur_end_node}")
            current_idx = self.path_index
            for digraph_attr in digraph_attrs:
                if str(digraph_attr['from']) == cur_start_node and str(digraph_attr['to']) == cur_end_node:
                    # 读取当前片段的轨迹规划参数，并设定参考轨迹 
                    print(f"{self.jid} cur order_mode:{digraph_attr['attrs']['order_mode']},order_type:{digraph_attr['attrs']['order_type']}")
                    
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

            
            # 休眠暂时代替规划过程
            waiting_sec = random.uniform(1, 3)
            print(f"{self.jid} planning... will take {waiting_sec:.2f} seconds.")
            time.sleep(waiting_sec)
            self.bdi.set_belief("can_task_start", True)

            if self.is_final_task:
                self.is_finished = True
                print(f"{self.jid} has completed its final task.")
            else:
                # 添加新的成就目标以继续任务，替代掉原先的while循环触发asl目标的方式
                self.add_achievement_goal("task_digraph")

            yield 
    def path_planning_from_digraph(self,digraph):
        # 根据 digraph_attr 进行路径规划
        # 目前只考虑了 order_mode : independent\aggreagate\disperse三种
        digraph_attr = digraph['attrs']
        order_mode = digraph_attr['order_mode']
        order_type = digraph_attr['order_type']
        cur_target = digraph_attr['target']
        formation = digraph_attr['formation']
        fleet_no = digraph_attr['fleet_no']
        # 读取当前片段的轨迹规划参数
        if order_mode == 'independent':
            pass






class MissionOrchestrator:
    """结合key_path_analyzer.log数据,生成bdi agent并管理生命周期"""
    def __init__(self, bdi_instructions: Dict[str, Any], server: str, password: str, asl_file: str, BlueBDIAgentTemplate: BlueUAVAgent):
        self.bdi_instructions = bdi_instructions
        self.server = server
        self.password = password
        self.asl_file = asl_file
        self.BlueBDIAgentTemplate = BlueBDIAgentTemplate

        self.active_agents: Dict[str, BlueUAVAgent] = {}
        
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

        """
        agent = self.BlueBDIAgentTemplate(jid, self.password, self.asl_file, tasks, _init_pos, merge_peers=merge_peers)
        
        # 初始化信念
        if len(path) >= 2:
            agent.bdi.set_belief("cur_nodes", path[0], path[1])
        
        await agent.start()
        self.active_agents[agent_name] = agent

    async def run(self):
        print("Mission Orchestrator Started.")
        
        # 1. 启动初始 Agents (名字不包含 _sub_ 或 _merged_ 的)
        initial_agents = [name for name in self.bdi_instructions.keys() 
                            if "_sub_" not in name and "_merged_" not in name]
        
        for name in initial_agents:
            await self._spawn_agent(name)

        # 2. 监控循环
        while self.active_agents:
            # 复制 keys 以便在循环中修改字典
            current_agent_names = list(self.active_agents.keys())
            
            for name in current_agent_names:
                agent = self.active_agents[name]
                
                if agent.is_finished:
                    await self._handle_agent_completion(name, agent)
            
            await asyncio.sleep(0.5)
        
        print("All missions completed.")

    async def _handle_agent_completion(self, agent_name, agent):
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
                await self._spawn_agent(child_name, _init_pos=_last_pos)
            

                
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
                # 1. 停止所有参与合并的 agent
                for participant in self.pending_merges[target_seg]:
                    if participant in self.active_agents:
                        p_agent = self.active_agents[participant]
                        await p_agent.stop()
                        del self.active_agents[participant]
                
                # 2. 清理等待列表
                del self.pending_merges[target_seg]
                
                # 3. 启动合并后的新 agent
                new_agent_name = self._find_agent_name_by_segment_id(target_seg)
                if new_agent_name:
                    await self._spawn_agent(new_agent_name, _init_pos=_last_pos)
                else:
                    print(f"Error: Could not find agent definition for segment {target_seg}")
            else:
                # 尚未就绪，Agent 保持运行状态 (is_finished=True)，实际上是在"原地等待"
                pass



async def start_agent(server, password):
    current_dir = os.path.dirname(__file__)
    key_path_instructions_path = os.path.join(current_dir,"data" ,"key-path-analyzer.log")
    asl_file = os.path.join(current_dir, "uav_key_path.asl")
    digraph_attrs_reference = os.path.join(current_dir, "data", "digraph_with_attrs.json")
    key_path_instructions = json.load(open(key_path_instructions_path, "r"))
    bdi_instructions = key_path_instructions["bdi_instructions"]
    
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