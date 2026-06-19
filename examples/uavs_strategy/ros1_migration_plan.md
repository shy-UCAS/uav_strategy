# uav_dynamic_agents02 → ROS1 渐进移植计划

> 目标：把当前基于 **SPADE + spade_bdi(BDI) + Redis** 的 `uav_dynamic_agents02.py`，
> 渐进迁移到 **ROS1 (Noetic) + PX4 SITL + Gazebo Classic + MAVROS**，最终接入真实无人机动力学仿真。
>
> 本文是**可勾选的执行清单**，每个阶段都带验收判据。建议严格按阶段推进，不要大爆炸式重写。

---

## 0. 背景与定位

- 现状主程序：`examples/uavs_strategy/uav_dynamic_agents02.py`
- BDI 实际只用到极薄一层「逐航段推进 + `can_task_start` 闸门」状态机（见 `uav_key_path.asl`），
  其余全是 Python 函数 + Redis 通信 → 迁移成本主要在**通信层和执行层**，决策层很轻。
- 选择 ROS1 而非 ROS2 的理由：无人机集群顶尖开源（ego-planner / Fast-Planner / XTDrone /
  PX4-Avoidance / MAVROS offboard）几乎都在 ROS1，生态适配成熟。
- **已知代价**：ROS Noetic 已于 2025-05 EOL，锁定 Ubuntu 20.04 + Python 3.8，上游不再修复。
  科研/仿真可接受，不建议在其上长期建生产系统。

---

## 1. 总体策略

1. **先纯逻辑上 ROS，后接 PX4**：阶段 1–2 仍沿用 `nxt=target` 瞬移运动学模型，只把通信从
   Redis 换成 ROS 话题、把 ASL 换成 FSM；确认行为与旧版一致后，再在阶段 3+ 引入真实动力学。
2. **集中式协同优先**：起步用一个 `mission_coordinator` 节点统管依赖闸 / barrier / 放行，
   简单可控；后期需要去中心化再演进。
3. **站在 XTDrone 上起步**：多机 PX4 SITL、MAVROS 命名空间、Gazebo 挂载这些管道直接复用，
   不要从零搭。

---

## 2. 目标架构

```
mission_coordinator (1 个节点)
  ├─ 载入项目A导出三件套 (digraph_attrs / key_paths / facilities) —— 逻辑不变
  ├─ 流量随机游走 → 给每架机分配 flight_plan
  ├─ 一次性预算每段的基准轨迹 + 编队成员轨迹 (替代 Redis "谁先生成谁写")
  ├─ 维护 depends_on/seg_done DAG + 起点 barrier → 发布 "放行" 话题
  └─ 监控任务完成 / 落盘轨迹

uav_i agent 节点 (每机一个, namespace /uav_i)
  ├─ FSM(SMACH): 检查依赖 → 飞向段起点 → barrier 同步 → 跟踪轨迹 → 标记完成 → 下一段
  ├─ offboard 控制器(定时器): 参考轨迹流式发成 MAVROS setpoint
  ├─ 订阅: 自己的 MAVROS 里程计 / coordinator 放行 / (可选)队友状态
  └─ 发布: setpoint → PX4 / 自己的 arrived、segment_done 状态

PX4 SITL + Gazebo Classic (每机一个实例, 经 MAVROS ↔ ROS1 话题)
```

---

## 3. 现状 → ROS1 映射

| 现状 (agent02) | 作用 | ROS1 对应 |
|---|---|---|
| SPADE `BDIAgent`(每机一个) | agent 容器/生命周期 | 每机一个 rospy 节点 |
| ASL 规则 + `can_task_start`/`cur_nodes` 信念 | 逐航段推进状态机 | **SMACH**(首选) / py_trees / BT.CPP |
| `.act_digraph_path_planning` 自定义动作 | 依赖闸/角色分配/取轨迹 | FSM 状态 + 规划库(纯 Python 直接移植) |
| `add_achievement_goal` 回注目标 | 执行层→决策层触发 | FSM 状态转移，无需手动回注 |
| 周期行为 `SyncAPFStepEnhance` | `DT` 步进执行 | rospy `Timer` 回调 → MAVROS setpoint |
| Redis 位置/轨迹/lookahead | 共享最新值 | MAVROS 里程计话题 + 节点内状态，**不再镜像** |
| Redis `current_segment_sync`/`release` | 起点 barrier | coordinator 节点 + 话题/服务 |
| Redis `seg_done` + `depends_on` | 先侦察后突击依赖闸 | coordinator 维护 DAG，发"放行"事件 |
| `GlobalRoundCoordinator`/`sim_round`/`round_done` | 自制离散时钟 | **删除** → Gazebo `/clock` + `use_sim_time` |
| `MissionOrchestrator`(流量游走分配+监控) | 任务编排 | `mission_coordinator` 节点 |
| `save_trajectories`(读 Redis 存 JSON) | 录制/可视化 | **rosbag** 或 logging 节点产同格式 JSON |
| `PlanningLib` / `FormationGenerator3D` | 纯算法 | 算法主体复用，但接口要去 `agent/Redis` 化 |
| `nxt = target` 瞬移 | 假物理 | **替换** → `/mavros/setpoint_raw/local` + PX4 飞控 |
| 配置类持久键(`formation_type` 等) | 持久最新值 | **latched topic**(`latch=True`) |

---

## 4. 关键设计决策（已确认共识）

- [ ] **决策层用 SMACH**（ROS1 原生 FSM，最贴合现有线性状态机）；若后续要"受威胁重规划"
      这类反应式扩展，再考虑 py_trees / BT.CPP。
- [ ] **彻底丢掉 Redis**：位置来自 PX4 里程计；协同走 coordinator；持久最新值用 latched topic。
- [ ] **删除 round 锁步**：`GlobalRoundCoordinator`/`sim_round`/`round_done` 整套作废，
      改 `rosparam set /use_sim_time true` + Gazebo `/clock`。（迁移最大的一块净简化）
- [ ] **coordinator 预算所有轨迹**：分配阶段一次算好各段基准轨迹 + 编队成员轨迹，各机直接取，
      消除原 Redis 里"谁先到谁负责生成"的竞态。
- [ ] **协同先集中后分布**：起步集中式 coordinator。
- [ ] **`sync_group` 升级为一等同步键**：有 `attrs.sync_group` 时按跨航段同步组收齐成员；
      没有 `sync_group` 时才退回同航段 `segment_key` barrier，避免把汇聚点相同但语义不同的任务混在一起。
- [ ] **算法库去上下文依赖**：把 `PlanningLib` 从 `agent.traj` / `agent.facilities` /
      `agent.io` 改成显式参数输入，形成可单测的纯函数或无状态服务。
- [ ] **坐标系边界前置定稿**：明确 `facilities` 经纬度 → UTM → ROS local ENU →
      MAVROS/PX4 的转换链，统一原点、单位、高度正方向和 yaw 约定。

---

## 5. 分阶段执行清单

### 阶段 -1 — 迁移接口定稿（先于编码）
- [ ] 定义 ROS 版任务包格式：保留 `digraph_attrs.json` / `key_paths.json` / `facilities.json`，
      但新增可选 `agent_plans.json` 作为每机执行真值，避免每次运行靠随机游走临时分配。
- [ ] 明确 `digraph_attrs.json` 仍必须是顶层 list，并在 coordinator 启动时做 schema 校验。
- [ ] 明确每条航段的最小字段：`from` / `to` / `members_num` /
      `attrs.action_class` / `attrs.target` / `attrs.segment_id` / `attrs.depends_on` /
      `attrs.sync_group`。
- [ ] 定义 ROS 消息/服务草案：`SegmentPlan`、`AgentPlan`、`SegmentStatus`、
      `SyncRelease`、`MissionEvent`、`GetAgentPlan`。
- [ ] 定义状态枚举：`WAIT_PLAN`、`WAIT_DEPS`、`FLY_TO_START`、`WAIT_SYNC`、
      `TRACK_TRAJ`、`HOLD`、`DONE`、`FAILED`。
- [ ] 固定随机种子和轨迹生成参数来源；后续实验报告必须能复现同一组 agent 分配和编队轨迹。
- [ ] 明确验收基准：先比较 agent 数、航段序列、航段成员集合、目标设施和完成顺序；
      数值轨迹只在固定随机种子和同一参数下比较。
- **验收**：在不启动 ROS 的情况下，用一个离线脚本能加载任务包、生成 `agent_plans`，
  并输出 schema 校验报告。

### 阶段 0 — 环境与基线
- [ ] 准备 Ubuntu 20.04 + ROS Noetic 环境
- [ ] 安装 PX4-Autopilot（配 Gazebo Classic）+ MAVROS + Gazebo Classic 11
- [ ] 跑通官方单机 SITL + MAVROS offboard 示例（确认环境健康）
- [ ] 安装 / 跑通 XTDrone 多机 demo（确认多机管道可用）
- [ ] 在新建 catkin workspace 下建包：`uav_swarm_strategy`（含 `msg/`、`scripts/`、`launch/`）
- **验收**：单机能 arm + 进 OFFBOARD 飞一条直线；XTDrone 多机能在 Gazebo 里起飞。

### 阶段 1 — 纯逻辑上 ROS（仍瞬移，话题代替 Redis）
- [ ] 移植纯算法库：`planning_modules/`（`PlanningLib`、`FormationGenerator3D`、
      `basic_functions`、`avoidance_agents`）原样拷入新包，去掉 Redis 依赖
- [ ] 实现 `mission_coordinator` 节点：
  - [ ] 载入 `data/nl_export/<case>/` 三件套（沿用 `switch_config==5` 的读取逻辑）
  - [ ] 优先读取 `agent_plans.json`；没有时才移植 `extract_uav_trajectories` 的流量随机游走分配
        （**加固定随机种子**，便于复现）
  - [ ] 分配阶段一次性预算各段基准轨迹 + 编队成员轨迹
  - [ ] 通过服务/话题把每机的 `flight_plan` + 各段轨迹下发给对应 agent 节点
- [ ] 重构 `PlanningLib` 接口：输入 `start_pose`、`segment_attrs`、`facilities`、
      `rendezvous_state`，输出 `trajectory`，不再读取 `agent.io`。
- [ ] 重构 `FormationGenerator3D` 调用入口：输入航段成员列表和明确的 formation 参数，
      不再依赖"首个到达者随机生成并写 Redis"。
- [ ] 实现 `uav_i` agent 节点骨架（rospy）：订阅下发、维护本机状态、Timer 步进
- [ ] 用 `nxt=target` 瞬移模型驱动（不接 PX4），位置发布到话题
- [ ] 定义自定义消息：航段、放行信号、agent 状态（`msg/`）
- **验收**：跑同一份 `nl_export` 输入，新版与旧版在 agent 数、航段序列、航段成员集合、
  目标设施、完成顺序上完全一致；固定随机种子后再比较逐段轨迹浮点误差。

### 阶段 2 — 决策层换 FSM + 删 round 锁步
- [ ] 用 SMACH 重写每机状态机：`CHECK_DEPS → FLY_TO_START → BARRIER_SYNC → TRACK_TRAJ →
      MARK_DONE → NEXT_SEGMENT`（对应原 ASL + `SyncAPFStepEnhance` 的逻辑）
- [ ] coordinator 实现依赖闸（`depends_on`/`seg_done`）：收齐前置段完成 → 发放行
- [ ] coordinator 实现起点 barrier：按 `sync_group` 收齐跨航段成员；无 `sync_group` 时按同段成员收齐；
      收齐后发 `release`（替换原 `current_segment_release`/`release_round` 那套绕的时序）
- [ ] 每个 release 带 `sync_key`、`segment_ids`、`member_ids`、`deadline` 和 `reason`，
      日志里能还原为什么放行或为什么超时。
- [ ] 删除 `round_done`/`sim_round` 全部逻辑，启用 `/use_sim_time` + Gazebo `/clock`
- [ ] barrier 等待改为带**超时**，避免单机卡死拖垮全队
- [ ] 超时策略分级：单段失败进入 `FAILED`；可选任务跳过；关键任务触发全局 abort 或 hold。
- **验收**：多机在纯 ROS 仿真下能正确"先侦察后突击"（依赖闸生效）；
  同段 barrier 和跨航段 `sync_group` barrier 都能按预期放行或超时。

### 阶段 3 — 接单架 PX4 + Gazebo + MAVROS
- [ ] agent 节点接入 MAVROS：订阅 `/mavros/local_position/odom`。
- [ ] 先用 `/mavros/setpoint_position/local` 跑通纯位置控制；需要速度/加速度/yaw 控制时再切到
      `/mavros/setpoint_raw/local`，并明确 `type_mask`。
- [ ] 实现坐标转换模块：设施经纬度/UTM 统一转换到 ROS local ENU，所有 setpoint 只发布 local 坐标。
- [ ] 实现 offboard 进入时序：**先 >2Hz 持续发 setpoint**，再 `/mavros/set_mode` 切 OFFBOARD
      + `/mavros/cmd/arming` 解锁
- [ ] 增加任务生命周期动作：连接等待、预发送 setpoint、arm、takeoff 到安全高度、进入任务、
      任务完成后 hold/land。
- [ ] 把 `nxt=target` 瞬移替换为**真实轨迹跟踪**：参考轨迹流式发 setpoint，PX4 位置控制器跟随
- [ ] 把"到达判定 / lookahead 推进"从"瞬间到点"改为**基于容差 + 速度**的判据
- [ ] 增加 offboard 失联/failsafe 处理：setpoint 发布中断、模式切换失败、arming 失败时进入 `HOLD` 或 `FAILED`。
- **验收**：单架真能在 Gazebo 里起飞、进入 OFFBOARD、沿参考轨迹飞完一条 `key_path`、
  悬停或降落，跟踪误差在可接受范围。

### 阶段 4 — 多机 SITL + 真动力学下的协同
- [ ] 在 XTDrone 多机框架下扩展到 N 架（命名空间 `/uav0../uavN`）
- [ ] 先按 1 → 2 → 4 → 8 → 16 架逐级扩容，每一级记录 CPU、实时率、topic 频率和丢包/延迟。
- [ ] **重新启用避障斥力**（`compute_dynamic_repulsive_force`）——真动力学下才有碰撞风险
- [ ] 验证编队保持（`FormationGenerator3D` 生成的成员轨迹）在真飞行下成立
- [ ] barrier / 依赖闸在连续时间 + 真到达误差下复测（用 PX4 loiter/hold 在航点盘旋等待）
- [ ] 汇聚航段必须单独压测：同目标不同 `sync_group` 不互相等待；同 `sync_group` 不同入边能正确等待。
- **验收**：多机能完成完整任务序列（含编队、汇聚、依赖顺序），无碰撞；
  `sync_group` 日志和 rosbag 能解释每次等待与放行。

### 阶段 5 — 录制与可视化
- [ ] 用 rosbag 录制全程；或保留一个 logging 节点，订阅各机状态产出与现有可视化兼容的 JSON
- [ ] 帧对齐改为**按 sim-time 时间戳**（替代原离散 round 帧号），rosbag/sim time 天然支持
- [ ] logging 节点保留旧版字段映射：`segment_key`、`sync_key`、`lookahead`、`formation_type`、
      `phase_state`、`dist_to_target`，便于和旧可视化/旧 JSON 做对比。
- [ ] 增加任务级摘要：每段开始/结束时间、等待时长、最大跟踪误差、最小机间距、失败原因。
- [ ] 跑通"仿真 → 录制 → 复用现有可视化"全链路
- **验收**：能完整回放一次任务，可视化与录制数据对齐。

---

## 6. ROS1 特有的坑（务必注意）

- **MAVROS offboard 时序**：必须先以 **>2Hz** 持续发 setpoint 流，再切 OFFBOARD 模式 + arm，
  否则 PX4 直接拒绝进入 offboard。XTDrone 脚本有现成处理可参考。
- **rospy 规模化**：几十架 agent 各一个 rospy 节点 + 回调线程，Python(GIL) 吞吐有限；
  真要上规模考虑关键节点用 roscpp 或降低话题频率。
- **没有 QoS / lifecycle**：配置类"最新值"用 **latched topic**(`latch=True`) 替代原 Redis 持久键。
- **瞬移→真动力学的连锁影响**：原代码大量逻辑假设"瞬间到点"（lookahead 推进、到达判定、
  编队跟随），接 PX4 后全部要改成基于容差/速度，阶段 3 要专门排查。
- **多机 SITL 算力**：每架 PX4+Gazebo 实例都吃 CPU，规模大时考虑分布式多机或轻量动力学。
- **坐标系混乱风险**：ROS local ENU、PX4 NED、经纬度、UTM 不要在 executor 里混用；
  建议 coordinator 下发前统一到 local ENU。
- **`sync_group` 死锁风险**：不能只按目标点或终点节点自动合并等待组；必须优先使用显式
  `attrs.sync_group`，并为每个 barrier 设置超时和日志。
- **SMACH 依赖状态维护**：SMACH 只是状态机容器，不负责分布式一致性；全局依赖和同步仍应由
  coordinator 维护，避免每机各自判断造成竞态。
- **MAVROS setpoint 类型选择**：`setpoint_position/local` 简单但控制能力弱；
  `setpoint_raw/local` 能带速度/加速度/yaw，但 `type_mask` 配错会导致 PX4 忽略字段。

---

## 7. 参考资源

- **XTDrone**：多机 PX4 + Gazebo + MAVROS 仿真平台（多机脚手架直接复用）
- **PX4 用户手册**：ROS/MAVROS offboard control、multi-vehicle SITL 章节
- **MAVROS wiki**：plugin 列表、setpoint / 模式切换 / arming 话题
- **ego-planner / EGO-Swarm（ZJU FAST Lab）、Fast-Planner（HKUST）**：集群轨迹规划参考（均 ROS1）
- **PX4-Avoidance**：避障参考（ROS1）

---

## 8. 进度速览

| 阶段 | 主题 | 状态 |
|---|---|---|
| 0 | 环境与基线 | ☐ 未开始 |
| 1 | 纯逻辑上 ROS（瞬移） | ☐ 未开始 |
| 2 | FSM + 删 round 锁步 | ☐ 未开始 |
| 3 | 单机 PX4 + MAVROS | ☐ 未开始 |
| 4 | 多机 + 真动力学协同 | ☐ 未开始 |
| 5 | 录制与可视化 | ☐ 未开始 |
