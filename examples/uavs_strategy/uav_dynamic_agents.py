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

# =============================
# 1. Blue UAV Agent
# =============================
class BlueUAVAgent(BDIAgent):
    def __init__(self, jid, password, asl_file,key_path ,**kwargs):
        super().__init__(jid, password, asl_file)
        self.key_path = key_path

    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            cur_start_node = agentspeak.grounded(term.args[0], intention.scope)
            cur_end_node = agentspeak.grounded(term.args[1], intention.scope)
            print(f"act_digraph_path_planning from {cur_start_node} to {cur_end_node}")
            # 休眠1s暂时代替规划过程
            time.sleep(2)
            # self.bdi.set_belief("can_task_start", True)
            yield

async def main(server, password):
    current_dir = os.path.dirname(__file__)
    asl_file = os.path.join(current_dir, "uav_key_path.asl")
    print(f"key paths num:{len(complex_key_paths)}")
    agents = []
    for blue_idx, key_path in enumerate(complex_key_paths):
        jid = f"blue_{blue_idx}_uav@{server}"
        print(f"Blue UAV {blue_idx} jid: {jid}")
        agent = BlueUAVAgent(jid, password, asl_file,key_path)
        
        if len(key_path) >= 2:
            # 设置初始阶段 belief: cur_nodes(Start, End)
            agent.bdi.set_belief("cur_nodes", key_path[0], key_path[1])
            
            # 设置后续阶段 belief: next_node(Current, Next)
            # for i in range(1, len(key_path) - 1):
            #     agent.bdi.set_belief("next_node", key_path[i], key_path[i+1])
            #     print(f"Blue UAV {blue_idx} set_belief: next_node({key_path[i]}, {key_path[i+1]})")
        agents.append(agent)
    
    # 并发启动所有 agent
    await asyncio.gather(*(agent.start() for agent in agents))

    await asyncio.sleep(99999)
if __name__ == "__main__":
    server = "127.0.0.1"
    passwd = "202127"
    spade.run(main(server, passwd))