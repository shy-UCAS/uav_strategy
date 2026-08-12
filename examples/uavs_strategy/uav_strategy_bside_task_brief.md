# 任务书：接入 NL 编排输出 + 语义丰富（本项目 = uav_strategy）

> 把这份文件拷到 **uav_strategy 项目根**（或 `examples/uavs_strategy/`），在那边的 Claude Code 会话开头让它读这份 + 随附的两份 guide。本文从**本项目（uav_strategy）视角**写，路径都是本项目内的相对路径。

## 0. 背景（你需要知道的最小集）

另一个项目（NLTaskOrchestration）把自然语言作战指令编排成任务 DAG，再经一个转换器输出成**本项目能直接消费**的航段图数据。**你不需要读那个项目的任何代码**——你只需知道：数据契约长什么样（§2）、数据已经放在哪（§1）、你要改本项目的哪些代码（§3-§5）。

整条链路：`自然语言 → DAG → 转换器 → 三件套 JSON → 你的 uav_dynamic_agents02 仿真`。你负责最后一棒。

## 1. 数据已就位

首个样本的三件套已放在：
```
examples/uavs_strategy/data/nl_export/gen_aggregate_disperse_6fd84c95/
  digraph_attrs.json   航段图（边=航段，节点=整数航路点）
  key_paths.json       每个编队的航路点序列
  facilities.json      设施经纬度 {facilities_str, defence_rings}
```
这个样本语义是"1 路侦察 + 3 路突击，汇聚到 hq_mark7"。

## 2. 数据契约（digraph_attrs.json，顶层是 list）

每条航段：
```jsonc
{
  "from": 2, "to": 3,                     // 整数航路点端点
  "attrs": {
    "order_mode": "singleton",            // singleton 独立 / aggregate 汇聚同步组
    "order_type": "breakthrough",         // 旧分发兜底；本项目现有 execute_path_planning_from_digraph 按它分发
    "target": "hq_mark14",                // 飞向的设施名（在 facilities.json 内）；汇聚组可能是 "aggregate_point"
    "fleet_no": "f2.1",                   // 元数据，可不读
    "action": "breakthrough",             // 原始动作（9 种之一）
    "action_class": "assault",            // 4 大类：assault/recon/support/maneuver  ← L1 据此选飞行模式
    "segment_id": "seg_t1_strike",        // 航段唯一 id
    "depends_on": ["seg_t0"],             // 本航段开始前须完成的 segment_id  ← L2 跨编队时序 + 条件触发
    "sync_group": null                    // 同步组 id；同组航段汇聚时互相等待  ← L2
  },
  "members_num": 3                        // 该航段从机数；实际飞 members_num+1 架
}
```
> 这些字段本项目现有代码**只读了一部分**（order_mode/order_type/target/members_num）；`action_class`/`depends_on`/`sync_group` 等是给你做 L1/L2 时启用的，现在不读也不报错（向后兼容）。

## 3. 阶段 0 — 接入跑通（必做，最先做）

让 `examples/uavs_strategy/uav_dynamic_agents02.py` 消费上面的数据。

在该文件的 `switch_config` 链（现有 `if switch_config == 1: ... elif == 4:`）后加一个分支：
```python
elif switch_config == 5:
    _export_dir = os.path.join(current_dir, "data", "nl_export", "gen_aggregate_disperse_6fd84c95")
    digraph_attrs_reference_path = os.path.join(_export_dir, "digraph_attrs.json")
    facilities_file = os.path.join(_export_dir, "facilities.json")
    key_paths = json.load(open(os.path.join(_export_dir, "key_paths.json"), "r", encoding="utf-8"))
```
再把文件顶部 `switch_config = 3` 改成 `= 5`。

**验收**：仿真正常 spawn 16 个 agent、打印 "All persistent missions completed" 退出，`data/raw_data/uav_trajectories_persistent_*.json` 里四个编队轨迹汇聚到 hq_mark7 附近。先开 `BlueUAVAgent.VERBOSE=True` 看日志。

> 此阶段汇聚航段 `order_mode=aggregate` + 真实设施名,现有逻辑会走 `else` 退化成各自到达(不同步等待),**这是预期的**,先把链路跑通。

## 4. 阶段 1 — L1 动作行为差异化

让 9 种动作按 **4 大类**呈现不同飞行模式（详细映射与代码骨架见随附 `uav_strategy_enrichment_guide.md` §3）：

| action_class | 飞行模式 | 落地 |
|---|---|---|
| assault | 直插突防 | 复用 `execute_breakthrough` |
| recon | 抵近盘旋 | **新增** `execute_orbit`（绕 target 画圆，别复用 detour——它对 hq_markN 会 raise） |
| support | jam 绕目标盘旋 / standby 悬停 | `execute_orbit` / **新增** `execute_loiter` |
| maneuver | 直飞 / 汇合 | 复用 `execute_breakthrough` |

改 `planning_modules/uav_planning_actions.py` 的 `execute_path_planning_from_digraph`：分发从 `order_type` 改为读 `attrs['action_class']`（保留 order_type 兜底）。配套：`uav_dynamic_agents02.py` 的 `height_range_value_set` 加 `orbit`/`loiter` 高度区间；`insert_height_val` 的平滑分支纳入 orbit/loiter。

**验收**：可视化上能区分"侦察盘旋 vs 打击直插"。

## 5. 阶段 2 — L2 协同（详见 enrichment_guide §4）

- **航段依赖**（跨编队时序 + 条件触发）：航段规划前轮询等待 `attrs['depends_on']` 里每个 `seg_done:<id>` Redis flag；航段飞完置位自己的 flag。
- **汇聚同步双模式**：把同步触发条件从 `order_mode=='aggregate' and target=='aggregate_point'` 放宽成 `order_mode=='aggregate'`——于是 `target=hq_mark7` 同步飞真实坐标、`target=aggregate_point` 同步飞几何中心，两种都支持。`execute_breakthrough` 本就同时处理两种 target。

**验收**：打击航段确实等到侦察航段完成才出发；汇聚组同步抵达。

## 6. 关键约束与坑（务必注意）

- **digraph_attrs 必须是顶层 list**：本项目 `extract_uav_trajectories` 直接 `for item in json_data`。转换器输出已是顶层 list；不要套成 `{"...": [...]}` 字典（手工图 config 3 那种 `_digraph_with_attrs` 包装会让 `MissionOrchestrator.__init__` 崩）。
- **members_num = 实际 spawn 的 agent 数**，不是资源上限。固定 3（每航段 4 架）。别调到几百几千，会让 XMPP/Redis 崩。
- **`save_trajectories` 的同步过滤已修复**：`is_waiting` 统一为 JSON 布尔值，`waiting_reason/flight_phase` 承载详细阶段语义；`uavs_coords_str` 只输出任务飞行帧，`uavs_coords_raw` 保留完整原始记录。
- **已启用有界物理步进**：DT=0.5s，默认水平速度上限 16m/s，爬升/下降率上限 5m/s，避免 80m/500ms 强同步瞬移。
- **round 全局同步**：`GlobalRoundCoordinator` 要求所有 blue agent `round_done==current_round` 才推进 round；已完成 agent 仍会 mark round_done，不会死锁。改动等待逻辑时别破坏这点。
- **运行前置**：Redis `127.0.0.1:6379`、XMPP server `127.0.0.1` 必须先起；入口 `python -m examples.uavs_strategy.uav_dynamic_agents02`。

## 7. 随附文档怎么用

- **`uav_strategy_integration_guide.md`** —— 阶段 0 接入的完整细节 + 排错清单。
- **`uav_strategy_enrichment_guide.md`** —— 阶段 1/2 的完整方案、逐函数改法、队形/高度建议。

> ⚠️ 这两份是从 **NLTaskOrchestration 项目视角**写的，里面的相对路径 `../../uav_strategy/...` 指**本项目**，`../tools/`、`../configs/` 指那个项目（你不用读）。看代码引用时以**本项目实际文件**为准，行号可能有出入、自己确认。

## 8. 你不需要做的

- 不用读/改 NLTaskOrchestration 项目。
- 不用碰时间轴/弹药/能量/截止（L3）——本阶段范围只到 L1+L2。
- 队形暂保持现有 `random.choice`，不强制按动作固定。
