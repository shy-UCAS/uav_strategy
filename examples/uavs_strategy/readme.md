# uav_dynamics_agents.py 功能说明
## 核心功能
- 先介绍一下程序利用了之前做的哪些工程：
  - BlueBehaviorsGenerator：
    - `basic_functions.py`:包含一些基本的时间、空间等信息的转换与处理工具
    - `math_curves_generators.py`+`quick_path_planner.py`:轨迹规划、坐标插值算法
    - `formation_generator_v2.py`集群编队队形生成方法
    - `test_client01_basic_multiplanner.py`生成的数据json文件：
      - 调用`json_orders2plan_graph`方法：
        - 从各种json形式的orders提取多条离散的`key_paths`用于后续生成`bdi_instructions`：
            ```
            key_paths = [
                ["1_0","1_1","1_2","3_0","3_1","4_1","4_2"
                ],
                ["2_0","2_1","2_2","3_0","3_1","5_1","5_2"
                ],
                ["1_0","1_1","1_2","3_0","3_1","6_1","6_2"
                ]
            ]          
           
        - 将json数据转换为`networkx.DiGraph`形式的图，每一对节点上包含了当前任务片段的任务信息,用于后续轨迹规划时检索信息：
            ```
            [
                {
                    "from": 0,
                    "to": 1,
                    "attrs": {
                    "order_mode": "singleton",
                    "order_type": "breakthrough",
                    "target": "hq_mark6",
                    "fleet_no": "f1.1",
                    "routed_ratio": 0.6,
                    "plan": {
                        "type": "breakthrough",
                        "target": "hq_mark6",
                        "trajectory": [],
                        "member_paths": []
                    }
                    }
                },                
            ]
  - `BlueAvoidance`中使用到的人工势场避障算法

- 该程序主要负责将key_paths解析成可执行的任务链`bdi_instructions`，并在BDI框架中根据任务链动态生成\终止无人机智能体集群，实现按照任务序列执行轨迹规划、避障等功能，并且将位置与轨迹等数据写入 `Redis`
- 无人机agent之间可以同步进行独立、合并、分裂等形式的任务编排。
## 主要模块与职责
## 1. BlueUAVAgent：
- 继承 BDIAgent，启动时写入自身 ID、位置、轨迹到 `Redis`。
- 通过外部动作:`.act_digraph_path_planning` 调用轨迹规划库，将参考轨迹写回 `Redis`。
- 通过周期任务`APFStep`执行人工势场法的避障算法
## 2. **MissionOrchestrator**：
- 解析 `bdi_instructions`（来自`key_path_analyzer生成的 key-path-analyzer02.json`）数据，按运行顺序提取出每一批`BDI Agent`
- 管理 `BlueUAVAgent` 的生命周期，循环监听当前所有活跃的Agent的任务完成状态
- 触发任务完成状态，并根据bdi_instructions指令做 `split/merge/finish`操作,协调智能体之间的分裂（Split）与汇合（Merge）操作并注册新批次的`BDI Agent`。
- 任务流转控制：
  - 分裂 (Split): 当一个智能体完成当前任务段，且后续动作为 `split_and_terminate` 时，调度器会暂停父智能体，并根据配置生成多个子智能体`Branches`，继承父智能体的位置继续执行。
  - 汇合 (Merge): 当多个智能体需要汇合`merge_and_terminate`时，调度器会：
    - 将到达汇合点的智能体加入` pending_merges `等待队列。
    - 检查是否满足汇合条件（到达数量 == `merge_requirements`）。
    - 一旦条件满足，销毁所有参与汇合的子智能体。
    - 生成一个新的合并后智能体，继续执行下一阶段任务。

## 3. UavRedisIO：
- redis读写key-value数据：
  - uav:ids：集合，所有蓝方 ID。
  - uav:{id}:pos：当前位置 {"x","y","z","ts"}。
  - uav:{id}:traj：历史轨迹列表 [[x,y,z], ...]。
  - uav:{id}:ref_traj：参考轨迹 [[x,y,z], ...]。
  - uav:{id}:lookahead：参考轨迹的索引（int）。
- 主要方法：
  - ID 管理：add_uav_id / remove_uav_id / get_ids / scan_ids_by_key.
  - 位置：set_pos（带时间戳）/ get_pos / mget_pos（pipeline 批量）. 
  - 轨迹：set_traj（覆盖）/ append_traj_points / get_traj / mget_traj / clear_traj.参考轨迹：set_ref_traj / get_ref_traj.
  - 预瞄索引：set_lookahead / get_lookahead.
  - 辅助：mget_speed_from_traj（用前两帧算速度）、get_dist_2d（当前位置到参考终点的平面距离）、get_rendezvous_point（一组 ID 的质心）、filter_stale（过滤超时位置）。
- BlueUAVAgent与Redis的交互：
  - |数据|Redis Key 模式|读/写|作用|
    |:---:|:---:|:---:|:---:|
    |Position|`uav:{id}:pos`|读+写|当前实时坐标 (x, y, z, ts)|
    |Trajectory|`uav:{id}:traj`|读+写|历史飞行轨迹 (用于显示和算速)|
    |Lookahead|`uav:{id}:lookahead`|读+写|当前在参考轨迹上的进度索引|
    |Ref Trajectory|`uav:{id}:ref_traj`|写|当前任务的规划路径|
    |IDs|`uav:ids`|读+写|在线agent列表|


## 4. 数据配置：
- `data/digraph_with_attrs02.json`（有向图及规划属性）、`data/key-path-analyzer02.json`（任务段、合并/分裂规则）、`data/facilities.json`（设施信息）


# key_path_analyzer.py 功能说明
## 核心功能
- 该程序主要负责将多条离散路径key_paths解析成可执行的任务链`bdi_instructions`，并交给uav_dynamics_agents.py的BDI框架动态生成智能体。
## 主要模块与职责
### 1. KeyPathAnalyzer
负责对输入的多条路径进行拓扑分析与分段处理，并生成可用于 BDI 系统的任务指令,例:
```
             
            "bdi_instructions": {
                "agent_1": [
                {
                    "segment_id": "seg_0",
                    "path": [
                    "1_0",
                    "1_1",
                    "1_2",
                    "3_0"
                    ],
                    "action_at_end": "merge_and_terminate",
                    "next_segment_hint": "seg_4",
                    "role_in_merge": "initiator"
                }
                ],   
            }            
```

- **构建有向图**：将 `key_paths` 中的节点序列转换为 `networkx.DiGraph`。
- **识别关键节点**：
  - 入度 > 1：汇合节点（merge node）
  - 出度 > 1：分裂节点（split node）
- **路径分段（Segment）**：
  - 在起点 / 分裂 / 汇合处切断，得到线性片段
  - 每个片段是一段无分叉/无汇合的连续节点序列
- **构建 Segment Graph**：
  - 以片段为节点，片段末尾连接片段起点作为有向边
- **使用广度优先算法 生成 BDI 指令**：
  - 从无前驱片段生成初始 Agent
  - 在分裂点生成子 Agent
  - 在汇合点生成合并后的新 Agent（仅由一个“发起者”生成）

### 2. BDISimulator
基于生成的 BDI 指令进行多线程仿真。

- 为初始 Agent 启动线程
- 每个 Agent 按任务链飞行（sleep 模拟耗时）
- 在分裂点生成子线程
- 在汇合点进行合并逻辑（只有发起者生成新 Agent）
## 输出与日志
- 默认将 `stdout` 重定向到 `key-path-analyzer.log`
  - 调用这个log文件中的**BDI 指令结构**:`bdi_instructions`
