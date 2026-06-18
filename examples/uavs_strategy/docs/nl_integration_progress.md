# NL 编排接入与语义丰富 — 改进记录

记录本项目（uav_strategy）消费 NLTaskOrchestration 任务编排输出的接入改动与后续计划。

## 1. 背景

NLTaskOrchestration 把自然语言作战指令编排成任务 DAG，经转换器输出成本项目能直接消费的**航段图**数据（`digraph_attrs` / `key_paths` / `facilities`）。

- **数据位置**：`data/nl_export/<sample_id>/`，当前样本 `gen_aggregate_disperse_6fd84c95`（1 路侦察 + 3 路突击，汇聚到 hq_mark7）。
- **数据契约**：`digraph_attrs.json` 顶层是 list，每条航段 `attrs` 含 `order_mode` / `order_type` / `target` / `action` / `action_class` / `segment_id` / `depends_on` / `sync_group`，外加顶层 `members_num`。
- **配套文档**（本目录同级 `examples/uavs_strategy/`）：
  - `uav_strategy_bside_task_brief.md` — 任务书（总览）
  - `uav_strategy_integration_guide.md` — 接入细节 + 排错
  - `uav_strategy_enrichment_guide.md` — L1/L2 完整方案

---

## 2. 已完成改动（2026-06-12）

### 阶段 0 — 接入跑通

| 位置 | 改动 |
|---|---|
| [../uav_dynamic_agents02.py:84](../uav_dynamic_agents02.py#L84) | `switch_config = 5` |
| [../uav_dynamic_agents02.py:120](../uav_dynamic_agents02.py#L120) | 新增 `elif switch_config == 5:` 分支，`digraph_attrs` / `facilities` / `key_paths` 指向 `data/nl_export/gen_aggregate_disperse_6fd84c95/` |

### 阶段 1 — L1 动作 4 大类差异化

把 9 种 NL 动作按 4 大类映射到不同飞行模式，可视化上能区分：

| action_class | 动作 | 飞行模式 | 落地 |
|---|---|---|---|
| assault | strike / breakthrough / intercept | 直插突防 | `execute_breakthrough`（现成） |
| recon | reconnaissance / track | 抵近盘旋 | **新增** `execute_orbit` |
| support | jam / standby | jam 绕目标盘旋 / standby 悬停 | `execute_orbit` / **新增** `execute_loiter` |
| maneuver | fly_to / rendezvous | 直飞 / 汇合 | `execute_breakthrough` |

具体改动：

| 位置 | 改动 |
|---|---|
| [../planning_modules/uav_planning_actions.py](../planning_modules/uav_planning_actions.py) `execute_path_planning_from_digraph` | 分发从 `order_type` 改为**优先按 `action_class`**，无该字段时回落旧 `order_type`（向后兼容 switch_config 1~4） |
| 同文件 `execute_orbit`（新增） | 抵近 target 后绕其画圆盘旋；自带几何，不依赖 facilities 分类（`plan_detour` 对 hq_markN/ua_N 会 raise）。参数 `radius=80m` / `steps=8`，可按战场尺度调 |
| 同文件 `execute_loiter`（新增） | standby 原地最小停留轨迹（当前样本无 standby，未被触发，先就位） |
| 同文件 `insert_height_val` | 三次样条平滑分支纳入 `orbit` / `loiter` |
| [../uav_dynamic_agents02.py:73](../uav_dynamic_agents02.py#L73) `height_range_value_set` | 新增 `orbit` `[[150,300],[150,300]]`、`loiter` `[[200,350],[200,350]]` 高度区间 |

**兼容性**：以上对 switch_config 1~4 无影响（无 `action_class` 时走旧分发）。两个文件 `py_compile` 通过。

---

## 3. 验证状态

- ✅ 语法检查（`py_compile`）通过。
- ✅ **端到端仿真已跑通**（2026-06-13，run `uav_trajectories_persistent_20260613_010313.json`）：16 个 agent（4 编队 × 4 架）全部正常 spawn 并完成，每条轨迹 ~34–35 点。
- ✅ **L1 动作差异化生效**（从输出轨迹几何确认）：
  - **侦察盘旋**：recon 编队（`agent_1_*`，段 `0_1` track radar_2）距 radar_2 最近 ~67–95m（≈ orbit 半径 80m）、绕飞方位角跨度 ~331°——确认绕 radar_2 盘旋一圈；
  - **打击直插 + 汇聚**：三路 strike/intercept/breakthrough 编队（`agent_2/3/4_*`，段 `x_y`+`y_4`）末点距 hq_mark7 **≤ 32m**，三路均收拢到汇合点。
- 备注：recon 编队从随机初始阵位飞入，到 radar_2 最远 ~8km（初始阵位散布，见 §5 观察项）；本次 `uavs_coords_str` 同步视图非空（16 agent 都有），未触发 §5 的退化。
- 跑前可把 `BlueUAVAgent.VERBOSE` / `SyncAPFStepEnhance.VERBOSE` 设 `True` 看日志。

---

## 4. 阶段 2 — L2 跨 agent 协同

转换器已在 `attrs` 里输出 `depends_on` / `sync_group`，B 侧实现等待逻辑即可启用。

### 4.1 航段依赖闸（先侦察后突击 / condition_trigger）— ✅ 已实现并验证（2026-06-13）

- **机制**：航段飞完在 Redis 置 `seg_done:<segment_id>="1"`；依赖它的航段规划前检查 `attrs['depends_on']` 每个 flag，未满足则挂起（不规划、不推进 `cur_nodes`）；挂起态每 round 重检，满足后重新触发规划。
- **落地（4 处）**：
  - `BlueUAVAgent.__init__`：初始化 `current_segment_id`/`current_depends_on`/`_waiting_for_deps`。
  - `act_digraph_path_planning`：进规划前依赖闸（未就绪 `yield/return`，置 `_waiting_for_deps`）。
  - `SyncAPFStepEnhance.run`：挂起态重检；满足则 `set_belief("can_task_start", True)` + `add_achievement_goal("task_digraph")`。
  - `_check_task_completion`：段完成置 `seg_done`。
- **踩坑（已修）**：重触发必须先 `set_belief("can_task_start", True)`，否则 ASL 落到 `can_task_start(false)` 的 `.wait(100)` 分支、永不重规划 → strike 永不动 → orchestrator 空转（实测 round 飙到 1018）。对照 `_check_task_completion` 既有写法补这一行即可。
- **验收结果**：recon round 1 起飞、~round 16 飞完置 `seg_done:seg_t0`；三路 strike 被摁到 **round 17** 才解锁（对比无闸时三路 round 1 齐飞），sim 正常退出。"先侦察后突击"生效。

### 4.2 汇聚同步双模式 — ⏸ 已决定暂缓（2026-06-13）

> **决定**：选 A 暂不做——接受当前 facility 模式下三路独立收拢到 hq_mark7（≤32m）的汇聚效果；待"严格同步到达"成为硬需求时，再做下面的 barrier 改造。

- **原计划**：放宽 [../uav_dynamic_agents02.py:305](../uav_dynamic_agents02.py#L305) 触发条件为 `if _order_mode=='aggregate'`，让 facility/geometric 两种汇合点都进 merge 同步等待。
- **阻碍（读码发现）**：当前生效的 `SyncAPFStepEnhance` **根本没调用 merge 逻辑**——`_sync_bdi_state`/`merge_peers` 是死代码，实际同步走 `_sync_state_checkpoint`（按 `segment_key` 的**同边**编队 barrier）。三路 rdv 是不同边（`3_4`/`6_4`/`8_4`）、`segment_key` 各异，naive 放宽会让跨编队 barrier 永远等不齐 → **死锁**。
- **真正要做**：把 barrier 的同步键从"逐边 `segment_key`"改成"共享汇聚键（`sync_group` 或终点节点）"，并让 group 跨边覆盖所有汇聚 agent。是有死锁风险的 barrier 改造。
- **现状可接受性**：facility 模式下三路已**独立收拢到 hq_mark7 ≤32m**（实测），只是不同 round 抵达、非严格同步。若同步到达不是硬需求，可暂不做。

### 其他增强（按需）

- **队形按动作固定**：当前 `_formation_type = random.choice(...)`（[../uav_dynamic_agents02.py:339](../uav_dynamic_agents02.py#L339)）。可改读 `attrs['formation_type']`（需转换器侧也输出该字段）。推荐映射：assault→vshape、recon→circular、support→horizontal、maneuver→arc。暂保持随机。
- **修复 `save_trajectories` 同步视图**：见 §5。

### 暂不做

- **L3 时间/资源**：deadline / 弹药 / 能量 / 能力真正影响仿真——本项目是无时间轴的几何仿真，需大改架构。这些约束留在 NL 侧由 Z3 验证。

---

## 5. 关键约束与已知问题

- **`digraph_attrs` 必须顶层 list**：`extract_uav_trajectories` 直接 `for item in json_data`。转换器输出已是顶层 list（手工图 config 3 的 `_digraph_with_attrs` 字典包装会让 `MissionOrchestrator.__init__` 崩）。
- **`members_num` = 实际 spawn 的 agent 数**（= SPADE agent 数 + 编队轨迹数），不是资源上限。当前固定 3（每航段 4 架），别调到几百几千会崩 XMPP/Redis。
- **`save_trajectories` 同步视图恒空（已知 bug）**：`is_waiting` 写入的是 `True` / `"False"` / `"flying to start"` 等 truthy 值，导致 `segment_common_frames` 恒空、`uavs_coords_str` 退化为空。当前做可视化用 `uavs_coords_raw`；修复时把 `is_waiting` 判断改成只认明确的等待值。
- **round 全局同步**：`GlobalRoundCoordinator` 要求所有 blue agent `round_done==current_round` 才推进；已完成 agent 仍 mark round_done，不死锁。改 L2 等待逻辑时勿破坏这点。
- **观察项**：初始阵位随机聚集、航段拼接处间隙检测被注释（无条件 `extend`）可能有几何跳变、防御圈避障可能让某段绕远——均为现有行为，跑时留意。
