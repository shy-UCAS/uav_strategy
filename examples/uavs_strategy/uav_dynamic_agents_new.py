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

from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour, OneShotBehaviour
from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import register_planning_actions
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
    122.15451569813476,
    37.50781897194055,
    165
]

init_loc2 = [
    122.16533086650654,
    37.52305286868604,
    220
]

current_dir = os.path.dirname(__file__)
digraph_attrs_reference_path = os.path.join(current_dir, "data", "digraph_with_attrs.json")
digraph_attrs = json.load(open(digraph_attrs_reference_path, "r"))

class PlanningDelayBehaviour(OneShotBehaviour):
    """后台行为：模拟规划耗时，完成后更新信念触发下一步"""
    def __init__(self, agent_obj, next_start, next_end, delay, is_final):
        super().__init__()
        self.agent_obj = agent_obj
        self.next_start = next_start
        self.next_end = next_end
        self.delay = delay
        self.is_final = is_final

    async def run(self):
        # 非阻塞等待
        await asyncio.sleep(self.delay)
        
        # 更新信念，这将触发 ASL 中的 +cur_nodes 事件
        if self.next_start and self.next_end:
            print(f"{self.agent_obj.jid} finished planning. Updating belief to {self.next_start}->{self.next_end}")
            self.agent_obj.bdi.set_belief("cur_nodes", self.next_start, self.next_end)
        
        if self.is_final:
            self.agent_obj.is_finished = True
            print(f"{self.agent_obj.jid} has completed its final task.")

# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file, task_lists, init_pos=None, facilities=None, **kwargs):
        super().__init__(jid, password, asl_file)
        # 读取任务列表
        self.task = task_lists[0]
        self.key_path = self.task['path']

        # 记录当前在key_path路径中的索引
        self.path_index = 0
        # 任务完成标志
        self.is_finished = False
        self.is_final_task = False
        self.io = UavRedisIO(**kwargs.get("redis_cfg", {}))
        self.position = init_pos

        # 周期性行为
        self.APFStep = APFStep
        # self.FormationAPFStep = FormationAPFStep
        self.FetchWorldState = FetchWorldState

        # 经纬度 -> UTM
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()

        if self.position is not None:
            # 如果传入了经纬度初始点，用它初始化轨迹
            self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(
                np.array([self.position])
            ).tolist()
        else:
            # 否则，基于init_loc1和init_loc2生成一个随机位置
            self.traj = bfunc.generate_circle_positions_from_diameter(1, init_loc1, init_loc2)  

        #初始化节点信念
        # 注意：这里设置信念会立即触发 ASL 中的 +cur_nodes 事件
        self.bdi.set_belief("cur_nodes", self.key_path[0], self.key_path[1])
        
    
    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            cur_start_node = str(agentspeak.grounded(term.args[0], intention.scope))
            cur_end_node = str(agentspeak.grounded(term.args[1], intention.scope))
            print(f"{self.jid} is act_digraph_path_planning from {cur_start_node} to {cur_end_node}")
            
            current_idx = self.path_index
            
            # 预计算下一段路径
            next_start = None
            next_end = None
            is_final = False
            
            try:
                if current_idx + 1 < len(self.key_path):
                    next_idx = current_idx + 1
                    if next_idx + 1 < len(self.key_path):
                        next_start = self.key_path[next_idx]
                        next_end = self.key_path[next_idx + 1]
                        self.path_index = next_idx # 立即更新索引
                    else:
                        print(f"{self.jid} Reached end of path.")
                        is_final = True
            except ValueError:
                print(f"Error processing path indices.")

            # 启动后台延时任务
            waiting_sec = random.uniform(1, 3)
            print(f"{self.jid} planning... will take {waiting_sec:.2f} seconds (non-blocking).")
            
            # 创建并添加行为
            beh = PlanningDelayBehaviour(self, next_start, next_end, waiting_sec, is_final)
            self.add_behaviour(beh)

            yield        

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
        
        # 构建JID
        jid = f"{agent_name}@{self.server}"
        print(f"Spawning agent: {jid} with path len {len(path)}")
        
        """
            注册运行bdi智能体，根据nodes查询任务列表并执行轨迹规划与位置
            依次执行每一段的任务规划，每当执行完当前阶段的规划与推理后，才更改标志位agent.is_finished=True(以及其他标志位)
            再返回到MissionOrchestrator中的run进行下一步处理

        """
        agent = self.BlueBDIAgentTemplate(jid, self.password, self.asl_file, tasks, _init_pos)
        
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
            for branch in task['branches']:
                child_name = branch['new_agent_hint']
                await self._spawn_agent(child_name, _init_pos=_last_pos)
            
            await agent.stop()
            del self.active_agents[agent_name]
                
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
    asl_file = os.path.join(current_dir, "uav_key_path_new.asl")
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
    server = "127.0.0.1"
    passwd = "202127"
    spade.run(start_agent(server, passwd))
