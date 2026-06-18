# uav_strategy 接入指南：把 01l 的 DAG 输出灌进无人机仿真

本文说明如何把本项目（NLTaskOrchestration，下称**项目A**）的实验输出，接到无人机集群策略仿真（uav_strategy，下称**项目B**，位于同级目录 `../uav_strategy`，**只读**）的输入上，跑通一次端到端：

> 自然语言指令 → GCJP 任务图（01l）→ 航段图（转换器）→ 多智能体编队仿真（项目B）

## 1. 数据流总览

```
项目A (NLTaskOrchestration)                                  项目B (uav_strategy, 只读)
┌──────────────────────────────────────────────┐          ┌─────────────────────────────────┐
│ 自然语言 case (datasets/.../*.env.jsonl)       │          │                                 │
│        │                                       │          │                                 │
│        ▼  exp_01l (生成→执行→验证→修复)         │          │                                 │
│ final_code/<sid>.py   final_dag/<sid>.json     │          │                                 │
│ reports/<sid>.json                             │          │                                 │
│        │                                       │          │                                 │
│        ▼  tools/export_uav_strategy_inputs.py  │          │                                 │
│        │  (任务图 → 航段图的线图变换)            │          │                                 │
│        ▼                                       │   拷贝    │                                 │
│ out/uav_export/<sid>/                          │ ───────► │ examples/uavs_strategy/data/    │
│   digraph_attrs.json  ← 航段图（边=航段）        │          │   nl_export/<sid>/              │
│   key_paths.json      ← 每编队航路点序列         │          │        │                        │
│   facilities.json     ← 设施经纬度               │          │        ▼  uav_dynamic_agents02  │
│   meta.json           ← 旁路存档（不进 B）       │          │   多智能体 BDI + APF 编队仿真    │
└──────────────────────────────────────────────┘          │        │                        │
                                                            │        ▼                        │
                                                            │   data/raw_data/*.json (轨迹)   │
                                                            └─────────────────────────────────┘
```

**为什么需要转换器**：两个项目的"图"不是同一种。
- 项目A 的 `final_dag`：**节点 = 任务**（actor/action/target），**边 = 依赖关系**（sequence/parallel/…）。
- 项目B 的 `digraph_attrs`：**边 = 航段**（带战术属性），**节点 = 匿名航路点**（整数序号，自身无属性）。

所以接入是一次 **任务图 → 航段图的线图（line-graph）变换**，由 [tools/export_uav_strategy_inputs.py](../tools/export_uav_strategy_inputs.py) 完成（规则见 §5）。

## 2. 前置条件

| 依赖 | 说明 |
|---|---|
| conda 环境 `llm` | 所有 Python 命令通过 `conda run -n llm` 执行 |
| `pyproj` | 转换器用它把 env 配置的 UTM 坐标换算成经纬度（已在 llm 环境） |
| Redis `127.0.0.1:6379` | **项目B运行时**需要；启动即 `flushdb` 清库 |
| XMPP server `127.0.0.1`（Prosody/Openfire） | **项目B运行时**需要；每个 agent 以 `agent_x@127.0.0.1` 登录 |

## 3. 三步操作

### Step 1 — 在项目A生成 DAG（01l）

```bash
conda run -n llm python -m experiments.exp_01l_standard_nl_to_gcjp_with_repair \
  --provider-profile anthropic_Bailian \
  --dataset datasets/generated/_trial/phase1_standard_nl_cases.v2.env.jsonl \
  --workers 16 --max-repair-rounds 2 --limit 32
```

> **务必用带真实地理信息的 env 数据集**（`*.env.jsonl`，由 `generate_cases.py --target-source environment --environment-config configs/environment_facilities_v2.yaml` 生成）。否则 target 是 `target_42` 这类合成名，项目B无法解析坐标。

输出目录形如 `out/phase1_generation/<provider>__<时间戳>/exp_01l_standard_nl_to_gcjp_with_repair/`，内含 `final_code/`、`final_dag/`、`reports/`。记下这个目录。

### Step 2 — 运行转换器

```bash
conda run -n llm python -m tools.export_uav_strategy_inputs \
  --run-dir "out/phase1_generation/<provider>__<时间戳>/exp_01l_standard_nl_to_gcjp_with_repair" \
  --env-config configs/environment_facilities_v2.yaml \
  --members-num 3 \
  --out-dir out/uav_export
```

- `--run-dir` 省略时自动选 `out/phase1_generation` 下**最新**的 01l run。
- 只转换 `expected_result == sat` 且 `final_pass == true` 的样本（unsat 族是故意不可行的，不能拿去飞）。
- 单独转换一个样本：追加 `--only <sample_id 子串>`。

每个样本输出到 `out/uav_export/<sample_id>/`，含 `digraph_attrs.json` / `key_paths.json` / `facilities.json` / `meta.json`，外加一份 `out/uav_export/_summary.json` 汇总（节点/航段/key_path 数与告警）。**有告警的样本先别灌进B**，按 §6 排查。

### Step 3 — 灌进项目B并运行

> 项目B只读，以下文件拷贝与代码改动都在**你手动**完成。转换器不写项目B。

**3a. 拷贝三件套**到项目B（`meta.json` 不用拷，它是旁路存档）：

```
out/uav_export/<sid>/{digraph_attrs,key_paths,facilities}.json
        ↓ 拷到
../uav_strategy/examples/uavs_strategy/data/nl_export/<sid>/
```

`<sid>` 是 `out/uav_export/` 下的样本目录名（即 sample_id），从转换出的样本里**任选一个**——目录名就是 `_summary.json` 里列的那些，例如 `gen_single_5616207f`（最简单，1 航段，适合先验证链路是否通）、`gen_binary_sync_192c7603`（两机汇聚）、`gen_aggregate_disperse_6fd84c95`（分路突击→汇聚，结构最贴近项目B手工图、已端到端验证）。**首次跑通推荐 `gen_aggregate_disperse_6fd84c95`**。

> §3a 与 §3b 必须用**同一个 `<sid>`**：下面 §3b 的示例代码即以 `gen_aggregate_disperse_6fd84c95` 为例，换样本时把 §3b 里 `_export_dir` 末尾的目录名一并改掉。

**3b. 在 `../uav_strategy/examples/uavs_strategy/uav_dynamic_agents02.py` 增加一个 `switch_config` 分支**（紧接现有 `elif switch_config == 4:` 之后）：

```python
elif switch_config == 5:
    # 接入 NLTaskOrchestration 导出的样本
    _export_dir = os.path.join(current_dir, "data", "nl_export", "gen_aggregate_disperse_6fd84c95")
    digraph_attrs_reference_path = os.path.join(_export_dir, "digraph_attrs.json")
    facilities_file = os.path.join(_export_dir, "facilities.json")
    key_paths = json.load(open(os.path.join(_export_dir, "key_paths.json"), "r", encoding="utf-8"))
```

然后把文件顶部的 `switch_config = 3` 改成 `switch_config = 5`。

> - `key_paths` 现有分支是内联 Python 列表；这里改为从 `key_paths.json` 读，等价。
> - `key_path_instructions_path`（`key-path-analyzer02.json` → `bdi_instructions`）实际**未被使用**，新分支无需为它准备文件，保持原样即可。
> - 转换器导出的 `facilities.json` 比项目B自带的更全（含 `radar_2`/`radar_3`），用导出的这份最稳。

**3c. 启动仿真**（先确保 Redis 与 XMPP 已运行）：

```bash
# 在 ../uav_strategy 目录下
python -m examples.uavs_strategy.uav_dynamic_agents02
```

结束后轨迹落在 `examples/uavs_strategy/data/raw_data/uav_trajectories_persistent_<时间戳>.json`，可喂给项目B的查看器/可视化。

## 4. 数据契约

转换器输出严格对齐项目B的消费格式。

### digraph_attrs.json —— 顶层 **list**，每项一条航段

```json
[
  {
    "from": 2, "to": 4,
    "attrs": {
      "order_mode": "singleton",
      "order_type": "breakthrough",
      "target": "hq_mark7",
      "fleet_no": "f2.2"
    },
    "members_num": 3
  }
]
```

| 字段 | 项目B消费点 | 取值 |
|---|---|---|
| `from`/`to` | 航段端点、汇聚判断、随机游走流量 | 整数航路点 ID |
| `attrs.order_mode` | 自定义动作（`aggregate`+`aggregate_point` 才触发 merge 等待） | 固定 `singleton` |
| `attrs.order_type` | `PlanningLib.execute_path_planning_from_digraph` 分发 | 固定 `breakthrough` |
| `attrs.target` | 飞向的设施（须在 facilities 内） | 真实设施名 |
| `attrs.fleet_no` | 元数据，B 不读 | `f<fleet号>.<段序>` |
| `members_num` | 边容量 = `members_num+1`，决定 spawn 的无人机数 | 固定 3 |

### key_paths.json —— 每个编队的航路点序列

```json
[[0, 1], [2, 3, 4], [5, 6, 4], [7, 8, 4]]
```

项目B仅用每条的**首元素**确定起点，随机游走自行重建路径。

### facilities.json —— 设施经纬度（对齐项目B自带格式）

```json
{
  "facilities_str": { "hq_mark7": [122.107, 37.563], "radar_2": [122.097, 37.553] },
  "defence_rings": { "RING1": { "lngs": [...], "lats": [...] } }
}
```

经纬度由 `pyproj` 从 `configs/environment_facilities_v2.yaml` 的 `source_utm_m` 按 **EPSG:32651（UTM 51N）** 换算，与项目B自带 `facilities.json` 逐位吻合。

## 5. 变换规则（参考）

`transform_builtgraph()` 把任务图按 **actor 感知的线图变换**还原为航段图：

1. 每个任务 → 一条航段边，先各分配 `in`/`out` 两个端点。
2. **同 actor** 的 `sequence`/`condition_trigger` 边 → `out(u) ≡ in(v)`（同一编队连续飞两段）。
3. **group_sync 组**（`binary_sync` 在 GCJP 里也编码成 group_sync）→ 组内所有任务的 `out` 端点合并为一个**汇聚航路点**（多路汇聚到一点，与项目B手工图"多路并入节点 14"同构）。
4. **跨 actor 依赖、parallel 边** → 不并点（项目B的流量模型无法表达跨编队时序/并行），记入 `meta.json` 旁路，不丢信息。
5. 端点等价类 → 整数节点 ID；每个 actor 的任务链 → 一条 `key_path`。

变换后强制校验：DAG 无环、流量守恒、`order_type ∈ {breakthrough,escape,detour}`、每个 target 可解析、`key_path` 起点为源节点。任一不过即在 `_summary.json` 告警。

## 6. 排错清单

| 现象 | 原因 / 处理 |
|---|---|
| 项目B在 `MissionOrchestrator.__init__` 崩，`TypeError: string indices must be integers` | `digraph_attrs` 不是顶层 list。转换器输出的就是顶层 list；若是手工图的 `{_digraph_with_attrs: [...]}` 字典包装会崩——别套字典。 |
| `UnicodeDecodeError` 读 json | 项目B多处 `open(path,"r")` 未指定编码。转换器已用纯 ASCII 写出（`ensure_ascii=True`），不会触发；若手改过文件注意保持 ASCII 或 UTF-8 无 BOM。 |
| 无人机原地不动 | 某 `target` 不在 facilities 内 → `execute_breakthrough` 走兜底原地。查 `_summary.json` 的 target 告警；用导出的 `facilities.json`（含全部 25 设施）。 |
| XMPP/Redis 崩、agent 注册失败 | `members_num` 过大。它 = 每航段实际 spawn 的 SPADE agent 数，**不是**资源上限。保持 `--members-num 3`（每航段 4 架），别设 999。 |
| 某条航段没有无人机经过 | 单源分散结构下随机游走流量不足。本批样本（独立链 + 末端汇聚）不会出现；若 `_summary.json` 报 flow imbalance 再处理。 |
| 转换器输出 0 个样本 | 该 run 没有 `sat && final_pass` 的样本，或 `--run-dir` 指错了 exp 目录（应指向 `…/exp_01l_standard_nl_to_gcjp_with_repair`）。 |

## 7. 旁路信息（meta.json，不进项目B）

`meta.json` 存档了变换中**未接入项目B的语义**，供后续增强参考：

- `sync_groups`：group_sync 的"同步到达"语义与真实汇聚设施。当前用真实设施名 + `singleton`，各机各飞向汇聚点、**不互相等待**。若要还原同步等待，可改用 `order_mode=aggregate` + `target=aggregate_point`（触发项目B动态几何中心 + merge_peers 等待），属后续增强。
- `soft_deps_dropped`：跨 actor 依赖（如 lead 侦察先于打击）与 `condition_trigger` 条件内容（如 `ua_3_confirmed`）——项目B无对应语义，已丢弃但存档。
- `parallel_edges_dropped`：显式并行边（跨 fleet 并行天然是两条独立路径，无需显式边）。
- `suggested_initial_positions`：env 配置里各 fleet 的真实阵位。**当前不使用**——项目B沿用自身 `init_loc` 范围生成随机阵位。若要用真实阵位，需在项目B侧 spawn agent 时传 `init_pos`（B 侧改动）。

## 8. 关键决策备忘（2026-06-11）

| 决策点 | 选择 | 理由 |
|---|---|---|
| 汇聚点 target | 真实设施名 + `singleton` | 坐标确定、可复现、可对照验证；先跑通，同步语义留待增强 |
| `members_num` | 固定 3 | = 实际 spawn 无人机数，与真实编成解耦；env quantity 视作不限量库存 |
| 初始阵位 | 沿用项目B `init_loc` 范围 | env 阵位是随机生成的，不如 B 侧规定范围好用 |
| `order_type` | 默认全 `breakthrough` | PlanningLib 对"设施名 target"的安全路径；escape/detour 路径不同 |
