import os, os.path as osp
import json
import numpy as np
import random
import argparse

import networkx as nx
import matplotlib.pyplot as plt

from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.planning_modules import quick_path_planners as quickpp


# --- 为每条路径分配初始从机数量，利用 in_degree/out_degree 处理汇聚与分离 ---
def compute_edge_members(paths, min_split_num=2):
    """
    根据路径列表计算每条边的从机数量。
    逻辑更新：
      全程跟踪“总机数”（Total UAVs = 1 Leader + N Members）。
      1. 汇聚：所有入边的总机数相加。新集群自动只保留1个Leader，其余变Member。（数值上等于总数不变）
      2. 分离：总机数拆分。每条出边必须至少分配 1个Leader + min_split_num个Member。
      3. 输出：边的从机数 = 该边总机数 - 1。
    """
    # 1) 从 paths 构建有向图
    _paths_graph = nx.DiGraph()
    for _path in paths:
        for u, v in zip(_path[:-1], _path[1:]):
            _paths_graph.add_edge(u, v)

    # 2) 初始生成（这里生成的是“从机”数量，所以总数要+1）
    _path_init_members = []
    for _path in paths:
        _init_members_num = random.randint(3, 4)
        _path_init_members.append(_init_members_num)
    print(f"各路径初始从机数量: {_path_init_members} (总机数对应+1)")

    # 3) 初始化起点的总机数
    _node_totals = {} # 存储节点上的总机数
    for _path_idx, _path in enumerate(paths):
        _start_node = _path[0]
        # 初始总数 = 从机 + 1(主机)
        _init_total = _path_init_members[_path_idx] + 1
        _node_totals[_start_node] = _node_totals.get(_start_node, 0) + _init_total

    # 4) 按拓扑序遍历，计算流
    _edge_totals = {} # 存储边上的总机数
    
    for _node in nx.topological_sort(_paths_graph):
        # 汇聚：若是中间节点，累加入边流量
        if _paths_graph.in_degree(_node) > 0:
            _incoming_total = sum(
                _edge_totals.get((pred, _node), 0)
                for pred in _paths_graph.predecessors(_node)
            )
            # 叠加到可能存在的初始值上（虽然通常中间节点不是起点）
            _node_totals[_node] = _node_totals.get(_node, 0) + _incoming_total

        _total = _node_totals.get(_node, 0)
        _out_deg = _paths_graph.out_degree(_node)

        if _out_deg == 0:
            continue
        elif _out_deg == 1:
            # 直通：总量直接传递
            _succ = list(_paths_graph.successors(_node))[0]
            _edge_totals[(_node, _succ)] = _total
        else:
            # 分离：拆分总量
            # 每个分支至少需要：1个主机 + min_split_num个从机
            _min_uavs_per_branch = 1 + min_split_num
            
            _succs = list(_paths_graph.successors(_node))
            
            # 检查是否有足够的飞机
            if _total >= _min_uavs_per_branch * _out_deg:
                # 能够满足最小分配
                # 1. 先每条边分配最小值
                _splits = [_min_uavs_per_branch] * _out_deg
                _remaining = _total - (_min_uavs_per_branch * _out_deg)
                
                # 2. 剩余的随机分配
                for _ in range(_remaining):
                    _idx = random.randint(0, _out_deg - 1)
                    _splits[_idx] += 1
            else:
                # 飞机不够分！这在逻辑设计上属于边缘情况。
                # 策略：即使不够从机，也必须保证至少有主机（1架）。
                
                if _total < _out_deg:
                    # 情况A: 极度短缺，连每条路分一架主机都不够。必然有路径断流。
                    print(f"[CRITICAL] Node {_node} 飞机总数({_total}) < 分支数({_out_deg})，部分路径将无飞机(断流)！")
                else:
                    # 情况B: 数量较少，无法满足 min_split_num，但每条路至少能分到主机。属于允许的“降级运行”。
                    print(f"[INFO] Node {_node} 飞机总数({_total}) 较少，无法满足最小从机数({min_split_num})限制。执行均分策略（每分支约 {_total/_out_deg:.1f} 架）。")

                # 尽量均分
                _base = _total // _out_deg
                _remainder = _total % _out_deg
                _splits = [_base] * _out_deg
                for i in range(_remainder):
                    _splits[i] += 1
                random.shuffle(_splits)
                
            for i, _succ in enumerate(_succs):
                _edge_totals[(_node, _succ)] = _splits[i]

    # 5) 转换为“从机数量”返回
    _edge_members = {}
    for (u, v), total_val in _edge_totals.items():
        # 从机数 = 总数 - 1
        # 如果 total_val 是 0 (异常情况)，从机也是 0
        attrs_members = max(0, total_val - 1)
        _edge_members[(u, v)] = attrs_members

    return _edge_members, _paths_graph

if __name__ == '__main__':
    # python -m examples.uavs_strategy.uav_manual_path_designer
    _plan_graph = nx.DiGraph()
    switch_case = 1
    _digraph_with_attrs = []
    _natural_language_description = ""
    descriptions_with_digraphplan = {}
    if switch_case == 1:

        _natural_language_description = """蓝方兵力分为四个集群，
        1号集群首先独立进攻hq_mark6、随后飞往hq_mark7与2号集群会合形成大部队，然后一起共同执行突破hq_mark1，然后突破hq_mark5，最后与其他编队汇聚到hq_2后，所有人飞往hq_mark4完成最后的突破。
        2号集群独立飞往hq_mark7，与1号集群会合后执行同样的后续行动。
        3号集群独立依次飞往hq_mark9、hq_mark8、hq_mark2、hq_mark4完成突破，随后前往hq_2与1、2、4号集群会合，后续共同行动。
        4号集群独立飞往hq_mark10、hq_mark3执行突破任务后，汇聚到hq_2与1、3号集群会合后，所有集群一起飞往hq_mark4完成突破。
        """
        _start_nodes = {
            0: {'target': 'hq_mark11'},
            3: {'target': 'hq_mark13'},
            6: {'target': 'hq_mark14'},
            11: {'target': 'hq_mark15'}
        }
        
        # start from hq_mark11                       
        _plan_graph.add_edge(0, 1, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark6", "fleet_no": "f1.1"})

        _plan_graph.add_edge(1, 4, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark7", "fleet_no": "f1.2"})
        
        _plan_graph.add_edge(4, 5, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark1", "fleet_no": "f1.3"})
        
        _plan_graph.add_edge(5, 2, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark5", "fleet_no": "f1.4"})

        # start from hq_mark13
        _plan_graph.add_edge(3, 4, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark7", "fleet_no": "f2.1"})
        
        # start from hq_mark14
        _plan_graph.add_edge(6, 7, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark9", "fleet_no": "f3.1"})
        
        _plan_graph.add_edge(7, 8, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark8", "fleet_no": "f3.2"})
        
        _plan_graph.add_edge(8, 9, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark2", "fleet_no": "f3.3"})
        
        _plan_graph.add_edge(9, 10, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark4", "fleet_no": "f3.4"})
        
        # start from hq_mark15
        _plan_graph.add_edge(11, 12, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark10", "fleet_no": "f4.1"})
        
        _plan_graph.add_edge(12, 13, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark3", "fleet_no": "f4.2"})

        # final aggregation
        _plan_graph.add_edge(2, 14, **{"order_mode": "aggregate", "order_type": "breakthrough", "target": "hq_2", "fleet_no": "f5.1"})
        
        _plan_graph.add_edge(10, 14, **{"order_mode": "aggregate", "order_type": "breakthrough", "target": "hq_2", "fleet_no": "f5.2"})
        
        _plan_graph.add_edge(13, 14, **{"order_mode": "aggregate", "order_type": "breakthrough", "target": "hq_2", "fleet_no": "f5.3"})

        _plan_graph.add_edge(14, 15, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark4", "fleet_no": "f6.1"})

        # 打印所有边的起点、终点以及附带的属性
        print("--- _plan_graph 所有的边和属性 ---")
        for u, v, attrs in _plan_graph.edges(data=True):
            print(f"起点: {u}, 终点: {v}, 属性: {attrs}")
        print("---------------------------------")

        _key_paths = [[0, 1, 4, 5, 2, 14, 15],
                        [3, 4, 5, 2, 14, 15],
                        [6, 7, 8, 9, 10, 14, 15],
                        [11, 12, 13, 14, 15]]
    elif switch_case == 2:    
        _key_paths = [
            [0,1,2,3],
            [0,2,3]
        ]
        _plan_graph.add_edge(0, 1, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark6", "fleet_no": "f1.1"})
        _plan_graph.add_edge(1, 2, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark7", "fleet_no": "f1.2"})
        _plan_graph.add_edge(2, 3, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark1", "fleet_no": "f1.3"})
        _plan_graph.add_edge(0, 2, **{"order_mode": "singleton", "order_type": "breakthrough", "target": "hq_mark7", "fleet_no": "f2.1"})


    # 对 key_paths（含分离情况）计算
    _edge_members, _kp_graph = compute_edge_members(_key_paths, min_split_num=2)
    print(f"key_paths 各边从机数量: {_edge_members}")
    print(f"key_paths 各节点 in/out degree:")
    for _n in _kp_graph.nodes():
        print(f"  {_n}: in={_kp_graph.in_degree(_n)}, out={_kp_graph.out_degree(_n)}")

    for u, v, attrs in _plan_graph.edges(data=True):
        _digraph_with_attrs.append(
            {
                "from": int(u),
                "to": int(v),
                "attrs": attrs,
                "members_num": _edge_members.get((u, v), 0)  # 添加从机数量信息 
            }
        )
    print(f"_digraph_with_attrs:{json.dumps(_digraph_with_attrs, indent=4)}")  

    descriptions_with_digraphplan['_natural_language_description'] = [
        line.strip() for line in _natural_language_description.strip().split('\n')
    ]
    descriptions_with_digraphplan['_digraph_with_attrs'] = _digraph_with_attrs
    current_dir = osp.dirname(os.path.abspath(__file__))
    with open(osp.join(current_dir, "data", 'manual_plan_graph', "manual_plan_graph01.json"), "w") as _f:
        json.dump(descriptions_with_digraphplan, _f, indent=4, ensure_ascii=False)

