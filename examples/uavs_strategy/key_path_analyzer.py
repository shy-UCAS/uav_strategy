import networkx as nx
from collections import defaultdict
import json
import sys
import threading
import time

sys.stdout = open('key-path-analyzer.log', 'w') # Optional: Uncomment to log to file

class KeyPathAnalyzer:
    def __init__(self, key_paths):
        """
        :param key_paths: List of lists, e.g. [['1_0', '1_1', '3_0'], ['2_0', '2_1', '3_0']]
        """
        self.key_paths = key_paths
        self.graph = nx.DiGraph()
        self.split_nodes = set()
        self.merge_nodes = set()
        
        # 构建图并分析
        self._build_graph()
        self._analyze_topology()
        
        # 提取路径片段
        self.segments = self._extract_segments()
        
        # 构建片段图（Segment Graph），用于分析片段间的关系
        self.segment_graph = self._build_segment_graph()

    def _build_graph(self):
        """构建有向图"""
        for path in self.key_paths:
            # 添加节点和边
            if len(path) == 1:
                self.graph.add_node(path[0])
            else:
                for i in range(len(path) - 1):
                    u, v = path[i], path[i+1]
                    self.graph.add_edge(u, v)

    def _analyze_topology(self):
        """识别汇聚点和分离点"""
        for node in self.graph.nodes():
            in_degree = self.graph.in_degree(node)
            out_degree = self.graph.out_degree(node)
            
            if in_degree > 1:
                self.merge_nodes.add(node)
            if out_degree > 1:
                self.split_nodes.add(node)

    def _extract_segments(self):
        """
        将图切分为线性片段 (Segments)
        一个 Segment 是一个节点序列，中间没有分叉或汇聚
        """
        segments = {} # ID -> [nodes]
        seg_id_counter = 0
        
        # 关键点 = 起点 + 终点 + 分离点 + 汇聚点
        # 注意：分离点是当前段的终点，也是下一段的起点
        # 汇聚点是上一段的终点，也是当前段的起点
        
        # 遍历图中的每一条边，将其归类到某个 Segment
        # 我们使用 DFS/BFS 遍历，但在关键点处截断
        
        visited_edges = set()
        
        # 找到所有可能的 Segment 起点：
        # 1. 整个图的起点 (in_degree=0)
        # 2. 分离点 (Split Node)
        # 3. 汇聚点 (Merge Node)
        start_candidates = {n for n in self.graph.nodes() if self.graph.in_degree(n) == 0} | \
                           self.split_nodes | self.merge_nodes
        
        for start_node in start_candidates:
            # 从这个起点出发的所有边
            for next_node in self.graph.successors(start_node):
                if (start_node, next_node) in visited_edges:
                    continue
                
                # 开始一个新的 Segment
                current_seg = [start_node, next_node]
                visited_edges.add((start_node, next_node))
                
                curr = next_node
                # 继续延伸，直到遇到关键点或没有后继
                while True:
                    # 如果当前点是关键点（Split/Merge），则 Segment 结束
                    if curr in self.split_nodes or curr in self.merge_nodes:
                        break
                    
                    if self.graph.out_degree(curr) == 0:
                        break
                        
                    # 获取下一个节点（因为不是 Split，out_degree 必然为 1）
                    succs = list(self.graph.successors(curr))
                    if not succs:
                        break
                    
                    succ = succs[0]
                    current_seg.append(succ)
                    visited_edges.add((curr, succ))
                    curr = succ
                
                segments[f"seg_{seg_id_counter}"] = current_seg
                seg_id_counter += 1
                
        return segments

    def _build_segment_graph(self):
        """构建 Segment 之间的依赖关系图"""
        seg_graph = nx.DiGraph()
        
        # 建立 Segment ID 到 (StartNode, EndNode) 的映射
        seg_endpoints = {sid: (path[0], path[-1]) for sid, path in self.segments.items()}
        
        for sid in self.segments:
            seg_graph.add_node(sid)
            
        # 连接 Segment
        for sid1, (start1, end1) in seg_endpoints.items():
            for sid2, (start2, end2) in seg_endpoints.items():
                if sid1 == sid2:
                    continue
                # 如果 Seg1 的终点是 Seg2 的起点，则连接
                if end1 == start2:
                    seg_graph.add_edge(sid1, sid2)
                    
        return seg_graph

    def generate_bdi_instructions(self):
        """
        生成 BDI 智能体指令 (包含初始 Agent 和衍生 Agent)
        """
        instructions = {}
        processed_segments = set()
        
        # 1. 识别初始任务 (没有前驱 Segment 的 Segment)
        initial_segments = [n for n in self.segment_graph.nodes() if self.segment_graph.in_degree(n) == 0]
        
        # 队列用于处理衍生 Agent
        # (agent_name, start_seg_id)
        agent_queue = []
        
        for i, seg_id in enumerate(initial_segments):
            agent_name = f"agent_{i+1}"
            agent_queue.append((agent_name, seg_id))
            
        # 2. 广度优先处理所有 Agent 任务链
        while agent_queue:
            agent_name, start_seg_id = agent_queue.pop(0)
            
            # 生成当前 Agent 的任务链，并收集衍生的子任务
            task_chain, spawned_tasks = self._trace_agent_task(agent_name, start_seg_id)
            instructions[agent_name] = task_chain
            
            # 将衍生的新 Agent 加入队列
            for new_agent_name, new_seg_id in spawned_tasks:
                agent_queue.append((new_agent_name, new_seg_id))
            
        return instructions

    def _trace_agent_task(self, agent_name, start_seg_id):
        """
        递归生成单个 Agent 的任务链
        返回: (task_chain, list_of_spawned_agents)
        """
        task_chain = []
        spawned_agents = []
        curr_seg_id = start_seg_id
        
        while curr_seg_id:
            path = self.segments[curr_seg_id]
            start_node = path[0]
            end_node = path[-1]
            
            task_info = {
                "segment_id": curr_seg_id,
                "path": path,
                "action_at_end": "finish"
            }
            
            successors = list(self.segment_graph.successors(curr_seg_id))
            
            if not successors:
                task_info["action_at_end"] = "finish"
                task_chain.append(task_info)
                break
            
            # 检查终点类型
            if end_node in self.split_nodes:
                task_info["action_at_end"] = "split_and_terminate"
                task_info["branches"] = []
                
                # 策略：母体消亡模式 (Mitosis)
                for succ_seg_id in successors:
                    new_agent_name = f"{agent_name}_sub_{succ_seg_id}"
                    branch_task = {
                        "new_agent_hint": new_agent_name,
                        "segment_id": succ_seg_id,
                        "path": self.segments[succ_seg_id]
                    }
                    task_info["branches"].append(branch_task)
                    # 记录需要新生成的 Agent
                    spawned_agents.append((new_agent_name, succ_seg_id))
                
                task_chain.append(task_info)
                break
                
            elif end_node in self.merge_nodes:
                task_info["action_at_end"] = "merge_and_terminate"
                
                if successors:
                    next_seg_id = successors[0]
                    task_info["next_segment_hint"] = next_seg_id
                    
                    # Merge 特殊处理：
                    # 多个 Agent 会汇入同一个 next_seg_id。
                    # 我们需要确保只生成一个新 Agent 来接管 next_seg_id，而不是每个汇入者都生成一个。
                    # 这里我们采用一种简单的命名约定：merge_at_{node_id}
                    # 实际运行时，由到达的 Agent 协商谁变身成这个新 Agent。
                    
                    merged_agent_name = f"agent_merged_at_{end_node}"
                    
                    # 只有当这个 merged_agent 还没被加入队列时才添加
                    # 但在这里我们无法全局去重，所以我们先添加，在外层去重
                    # 或者，我们只让 "字典序最小" 的前驱 Segment 负责生成新 Agent (确定性规则)
                    predecessors = list(self.segment_graph.predecessors(next_seg_id))
                    predecessors.sort()
                    if curr_seg_id == predecessors[0]:
                        spawned_agents.append((merged_agent_name, next_seg_id))
                        task_info["role_in_merge"] = "initiator" # 负责生成新 Agent
                    else:
                        task_info["role_in_merge"] = "participant" # 只是参与汇合
                
                task_chain.append(task_info)
                break
                
            else:
                # 普通连接
                curr_seg_id = successors[0]
        
        return task_chain, spawned_agents

class BDISimulator:
    """
    模拟 BDI 系统的并行执行器
    """
    def __init__(self, instructions):
        self.instructions = instructions
        self.print_lock = threading.Lock()
        
    def log(self, agent_name, message):
        with self.print_lock:
            print(f"[{time.strftime('%H:%M:%S')}][{agent_name}] {message}")

    def run(self):
        self.log("SYSTEM", "=== Simulation Started ===")
        
        # 1. 找出初始 Agent (名字里不带 sub 或 merged 的)
        initial_agents = [name for name in self.instructions.keys() 
                          if "_sub_" not in name and "_merged_" not in name]
        
        threads = []
        for agent_name in initial_agents:
            t = threading.Thread(target=self._agent_lifecycle, args=(agent_name,))
            t.start()
            threads.append(t)
            
        # 等待所有线程完成（包括衍生线程）
        # 注意：Python 的 join() 只能等待已启动的线程。
        # 对于动态生成的线程，只要它们是非 daemon 的，主程序就会自动等待它们结束。
        for t in threads:
            t.join()
            
        self.log("SYSTEM", "=== Initial Agents Threads Joined (Simulation continues if children are running) ===")

    def _agent_lifecycle(self, agent_name):
        self.log(agent_name, "Born/Started.")
        
        if agent_name not in self.instructions:
            self.log(agent_name, "No instructions found!")
            return

        my_tasks = self.instructions[agent_name]
        
        for task in my_tasks:
            seg_id = task['segment_id']
            path = task['path']
            action = task['action_at_end']
            
            # 模拟飞行耗时 (每个节点 0.2 秒)
            flight_time = len(path) * 0.2
            self.log(agent_name, f"Flying segment {seg_id} (Nodes: {len(path)})...")
            time.sleep(flight_time)
            
            # 到达终点，执行动作
            if action == "finish":
                self.log(agent_name, "Reached destination. Mission Complete.")
                return
                
            elif action == "split_and_terminate":
                self.log(agent_name, f"Reached Split Point. Spawning {len(task['branches'])} children...")
                
                child_threads = []
                for branch in task['branches']:
                    new_agent_name = branch['new_agent_hint']
                    t = threading.Thread(target=self._agent_lifecycle, args=(new_agent_name,))
                    t.start()
                    child_threads.append(t)
                
                self.log(agent_name, "Split complete. Terminating.")
                return
                
            elif action == "merge_and_terminate":
                role = task.get('role_in_merge', 'participant')
                if role == 'initiator':
                    # 推断合并后的 Agent 名字
                    merged_agent_name = f"agent_merged_at_{path[-1]}"
                    self.log(agent_name, f"Reached Merge Point (Initiator). Spawning {merged_agent_name}...")
                    
                    t = threading.Thread(target=self._agent_lifecycle, args=(merged_agent_name,))
                    t.start()
                    
                    self.log(agent_name, "Merge handover complete. Terminating.")
                else:
                    self.log(agent_name, "Reached Merge Point (Participant). Merging into swarm. Terminating.")
                return

# ==========================================
# Demo 使用
# ==========================================

if __name__ == "__main__":
    # 场景：
    # 用户提供的测试数据
    
    complex_key_paths = [
        [
            "1_0",
            "1_1",
            "1_2",
            "3_0",
            "3_1",
            "4_1",
            "4_2"
        ],
        [
            "2_0",
            "2_1",
            "2_2",
            "3_0",
            "3_1",
            "5_1",
            "5_2"
        ],
        [
            "1_0",
            "1_1",
            "1_2",
            "3_0",
            "3_1",
            "6_1",
            "6_2"
        ]
    ]
    
    # print("=== 1. 输入 Key Paths ===")
    # print(json.dumps(complex_key_paths, indent=2))
    
    analyzer = KeyPathAnalyzer(complex_key_paths)
    
    # print("\n=== 2. 拓扑分析结果 ===")
    # print(f"Split Nodes (分离点): {analyzer.split_nodes}")
    # print(f"Merge Nodes (汇聚点): {analyzer.merge_nodes}")
    
    # print("\n=== 3. 提取的路径片段 (Segments) ===")
    # for sid, path in analyzer.segments.items():
    #     print(f"{sid}: {path}")
    instructions_dict = {}    
    print("\n=== 4. 生成 BDI 任务指令 ===")
    bdi_instructions = analyzer.generate_bdi_instructions()
    instructions_dict["bdi_instructions"] = bdi_instructions
    print(json.dumps(instructions_dict, indent=2))
    
    print("\n=== 5. 开始并行仿真 (BDI Simulator) ===")
    simulator = BDISimulator(bdi_instructions)
    simulator.run()

