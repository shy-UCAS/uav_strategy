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

from typing import Dict, List, Optional, Iterable, Tuple, Any

# from time import time
from datetime import datetime

from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour

complex_key_paths = [
    [1,2,3,4,5,6],
    [7,8,9,10,11,12],

    # [20, 21, 5, 27], 
    # [28, 3, 21, 5, 27], 
    # [1, 2, 10, 26], 
    # [30, 2, 10, 26], 
    # [29, 22, 2, 10, 26], 
    # [20, 21, 5, 7, 25], 
    # [1, 2, 10, 12, 4, 8], 
    # [1, 2, 10, 12, 11, 24, 16]
]
complex_key_paths_stage2 = [
    [20, 21, 5, 27], 
    [28, 3, 21, 5, 27], 
]

# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file,key_path ,**kwargs):
        super().__init__(jid, password, asl_file)
        self.key_path = key_path
        self.is_finished = False  # 任务完成标志

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
            cur_start_node = agentspeak.grounded(term.args[0], intention.scope)
            cur_end_node = agentspeak.grounded(term.args[1], intention.scope)
            print(f"act_digraph_path_planning from {cur_start_node} to {cur_end_node}")

            # next node 获取
            # 在 key_path 中查找当前节点的位置，并更新为下一段路径
            try:
                # 假设 cur_start_node 和 cur_end_node 是整数
                for i, node in enumerate(self.key_path):
                    # 比较时转为int以防类型不匹配
                    if int(node) == int(cur_start_node) and i + 1 < len(self.key_path) and int(self.key_path[i + 1]) == int(cur_end_node):
                        # 检查是否存在下一个节点 (i+2) 防止越界
                        if i + 2 < len(self.key_path):
                            next_start = self.key_path[i + 1]
                            next_end = self.key_path[i + 2]
                            self.bdi.set_belief("cur_nodes", next_start, next_end)
                        else:
                            print("Reached end of path.")
                            self.is_finished = True # 标记任务完成
                        break
                
            except ValueError:
                print(f"Error: Could not find nodes {cur_start_node} or {cur_end_node} in key_path {self.key_path}")

            self.bdi.set_belief("can_task_start", True)
            # 休眠暂时代替规划过程
            time.sleep(0.5)
            
            yield

async def main(server, password):
    current_dir = os.path.dirname(__file__)
    asl_file = os.path.join(current_dir, "uav_key_path.asl")
    
    # 定义所有阶段的任务列表
    # 如果有更多阶段，只需添加到此列表中
    all_stages = [complex_key_paths, complex_key_paths_stage2]
    
    global_agent_idx = 0
    
    for stage_num, stage_paths in enumerate(all_stages, 1):
        print(f"=== Stage {stage_num} Start: {len(stage_paths)} agents ===")
        current_stage_agents = []
        
        for i, key_path in enumerate(stage_paths):
            # 使用全局索引确保JID唯一，避免XMPP连接冲突
            jid = f"blue_{global_agent_idx}_uav@{server}"
            global_agent_idx += 1
            
            print(f"Stage {stage_num} - Agent {i} JID: {jid}")
            agent = BlueUAVAgent(jid, password, asl_file, key_path)
            
            if len(key_path) >= 2:
                agent.bdi.set_belief("cur_nodes", key_path[0], key_path[1])
            
            current_stage_agents.append(agent)
            
        # 并发启动当前阶段的所有 agent
        if current_stage_agents:
            await asyncio.gather(*(agent.start() for agent in current_stage_agents))
        
        # 监控当前阶段完成情况
        while True:
            all_finished = True
            for agent in current_stage_agents:
                if not agent.is_finished:
                    all_finished = False
                    break
            
            if all_finished:
                print(f"=== Stage {stage_num} Completed: All agents reached destination ===")
                break
            
            await asyncio.sleep(1)
            
        # 停止当前阶段 agents
        print(f"Stopping Stage {stage_num} agents...")
        for agent in current_stage_agents:
            await agent.stop()
            
        # 阶段间缓冲
        await asyncio.sleep(2)

    print("All stages finished.")
if __name__ == "__main__":
    server = "127.0.0.1"
    passwd = "202127"
    spade.run(main(server, passwd))