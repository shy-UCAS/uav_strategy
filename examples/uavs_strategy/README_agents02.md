# uav_dynamic_agents02 使用与维护说明

> 本文档记录 `uav_dynamic_agents02.py`（下称 **agent02**）主程序及其配套脚本的架构、运行方式与数据约定。[§6](#6-shaoxing-数据格式适配记录重点) 记录 **shaoxing 数据格式** 适配，[§7](#7-统一仿真时钟物理步进与-latest-轨迹接口) 记录统一仿真时钟、物理步进和面向 SituationAwareness latest 的轨迹导出改造。
>
> 配套文档：`uav_strategy_bside_task_brief.md`（任务书）、`uav_strategy_integration_guide.md`（NL 接入指南）、`uav_strategy_enrichment_guide.md`（语义丰富指南）、`ros1_migration_plan.md`（ROS1 移植计划）。

---

## 1. 概览

agent02 是**多无人机集群任务仿真主程序**：所有无人机作为独立 BDI Agent（SPADE + spade_bdi + agentspeak），按航段图（digraph）拆分为航段任务，经 Redis 共享状态并同步编队飞行。

- 生命周期：由 `MissionOrchestrator` 统一生成/管理 Persistent Agents
- 任务驱动：ASL 文件（`uav_key_path.asl`）触发 BDI 规划 → `PlanningLib` 生成轨迹 → Redis 存储 → `SyncAPFStepEnhance` 周期行为推进飞行
- 同步机制：round（`GlobalRoundCoordinator`）+ 航段起点 barrier + `seg_done` 航段依赖闸
- 时间机制：全部 Agent 共用 `sim_round` 仿真时钟，默认每 round 代表 500ms
- 运动机制：参考轨迹点之间执行有界物理步进，不再直接瞬移到下一个点
- 导出机制：同时提供完整原始轨迹 `uavs_coords_raw` 和 latest 可直接消费的任务轨迹 `uavs_coords_str`

## 2. 运行环境与启动

**依赖环境**：conda 环境 `study`（PyQt5、spade、spade_bdi、pyproj、shapely、scipy、matplotlib 等）。

**前置服务**：
- Redis（agent02 与两个可视化器均读写 `127.0.0.1:6379`；WSL2 下 `sudo service redis-server start`）
- Openfire XMPP 服务器（SPADE 需要，`server = "127.0.0.1"`, 密码 `202127`，见主程序 `__main__`）

**启动主程序**（在 `uav_strategy` 根目录）：

```bash
python -m examples.uavs_strategy.uav_dynamic_agents02
```

**配套脚本**：

```bash
# 实时可视化（从 Redis 读取，随仿真实时刷新）
python -m examples.uavs_strategy.visualize.redis_data_visualize

# 离线回放（加载 save_trajectories 导出的 JSON：data/raw_data/uav_trajectories_persistent_*.json）
python -m examples.uavs_strategy.visualize.pyqt_visualize

# 人工设计航段图（生成 manual_plan_graph 系列 JSON）
python -m examples.uavs_strategy.uav_manual_path_designer

# 批量跑多次实验
python -m examples.uavs_strategy.run_batch
```

## 3. 目录结构与职责

```
examples/uavs_strategy/
├── uav_dynamic_agents02.py        # 主程序：BlueUAVAgent、MissionOrchestrator、switch_config、导出逻辑
├── uav_key_path.asl               # BDI 规划 ASL 文件（触发 .act_digraph_path_planning）
├── uav_manual_path_designer.py    # 人工航段图设计器（生成 manual_plan_graph JSON）
├── planning_modules/
│   ├── uav_planning_actions.py    # PlanningLib：航段规划动作（detour/escape/breakthrough/orbit/loiter）
│   ├── basic_functions.py         # LngLat2UTM（经纬度↔UTM）、Facilities（设施/防御圈）、全局配置
│   ├── math_curves_generators.py  # 轨迹插值/采样（cubic_interpolation_3d、linear_densify_3d 等）
│   ├── quick_path_planners.py     # SimpleBorders（沿多边形边界绕行）等快速规划器
│   └── formation_generator.py     # FormationGenerator3D：编队成员轨迹生成
├── behaviors_modules/
│   └── uav_periodic_behaviours.py # SyncAPFStep / SyncAPFStepEnhance / FetchWorldState / GlobalRoundCoordinator / DT
├── redis_modules/
│   └── uav_redis_io.py            # UavRedisIO：所有 Redis 读写封装
├── visualize/
│   ├── redis_data_visualize.py    # 实时可视化（PyQt5 + matplotlib，读 Redis）
│   └── pyqt_visualize.py          # 离线回放可视化（加载导出 JSON，支持 UTM/经纬度切换）
├── data/                          # 各 switch_config 的航段图/设施/导出数据
└── ../../tests/
    └── test_agents02_trajectory_contract.py # 时钟、轨迹过滤与物理步进回归测试
```

## 4. 数据流与坐标系约定

**坐标系策略：全程 UTM 内部自洽，输入/输出为经纬度，转换全自动，无需手工脚本。**

| 环节 | 坐标系 | 说明 |
|---|---|---|
| 输入：`init_loc1~4`、设施/防御圈 JSON | 经纬度 | 代码内写死/JSON 原始数据 |
| ① Agent 初始化位置 | 经纬度 → UTM | `BlueUAVAgent.__init__` 中 `LngLat2UTM().lng_lat_to_utm_array` |
| ② 设施/防御圈 | 经纬度 → UTM | `Facilities(..., convert_to_utm=True)` 默认转换 |
| ③ 轨迹规划 | UTM | 起点 `agent.traj[-1]`、终点 `get_target_location(utm=True)` |
| ④ Redis / 物理计算 / 实时可视化 | UTM | 可视化器坐标轴标注 UTM |
| ⑤ 导出 | UTM → 经纬度 | `save_trajectories` 中 `utm_to_lng_lat_array` |

**轨迹时间与高度策略**：Redis 中的轨迹点为 `[x, y, z]`，导出后拆为等长的 `lngs/lats/alts/ts`。`alts` 目前沿用仿真中的绝对高度语义；latest 接口若严格要求 AGL，仍需在地形数据可用后进行高程基准换算。时间字段详见 §7.1。

**转换器**：`basic_functions.LngLat2UTM`（pyproj，EPSG:4326 ↔ UTM，`always_xy=True`）。⚠️ zone 硬编码 51（`EPSG:32651`）：威海（122°E）属 zone 51 正确；config 6 的 116°E 区域标准上属 zone 50，但全链路用同一 converter，相对关系自洽，导出往返转换误差抵消——**只要所有输入都是经纬度就不会错位**；切勿混入真实 GIS 的 UTM 坐标。

## 5. switch_config 配置说明

| config | 航段图 | 设施文件 | 说明 |
|---|---|---|---|
| 1 | `digraph_with_attrs02.json` | `facilities.json` | 默认演示（仅 breakthrough） |
| 2 | `digraph_with_attrs.json` | `test_facilities_locations.json` | 含 detour/escape（target 为集合名 `antiair_facilities`） |
| 3 | `manual_plan_graph01.json` | `facilities.json` | NL 任务书生成 |
| 4 | `digraph_with_attrs03.json` | `facilities.json` | 简化示例 |
| 5 | `nl_export/gen_aggregate_disperse_6fd84c95/` | 同目录 | NLTaskOrchestration 导出样本 |
| **6** | `manual_plan_graph01_digraph_attrs.json` | **`facilities_shaoxing.json`** | **shaoxing 地图（见 §6）** |

> 注：config 6 代码注释写"绍兴空域"，但 `init_loc3/4` 与 `facilities_shaoxing.json` 的实际坐标在 116.38°E / 39.90°N（北京附近）——仅注释与数据不符，不影响运行。

## 6. shaoxing 数据格式适配记录（重点）

### 6.1 问题背景

config 6 引入了一套新的地图数据（`facilities_shaoxing.json` + `manual_plan_graph01_digraph_attrs.json`），设施命名为 `shaoxing_1/2/3`。旧代码对设施的处理存在多处**隐含假设**，对新命名全部失效，导致运行崩溃或显示缺失。以下是逐一适配过程。

### 6.2 坐标系：无需转换脚本

shaoxing 的 `init_loc3/4`、设施、防御圈均为经纬度，进入系统后由既有逻辑**自动**转 UTM（agent 初始化 `lng_lat_to_utm_array`、`Facilities(convert_to_utm=True)`），规划/存储/可视化全链路 UTM 自洽，导出时再转回经纬度。**未新增任何坐标转换脚本**。

### 6.3 设施分类缺失 → plan_detour/plan_escape 兜底（修 UnboundLocalError）

**问题**：`Facilities._parse_facilities_categories` 只按名字前缀分类——`ua_`→防空、`hq_`→指挥所、`radar_`→探测。`shaoxing_*` 无前缀，三个分类字典全空，设施只存在于 `facilities_info`。而 `plan_detour`/`plan_escape` 的内层分支只查三个分类字典 + `defend_rings`，导致 `detour_polygon_xys`/`escape_polygon_xys` 从未赋值 → `UnboundLocalError`（`agentspeak` 报 `cannot access local variable`）。

**修复**（`uav_planning_actions.py`）：两个函数各加 `else` 兜底分支——未分类设施绕**设施中心**、以 `AVOID_AVERAGE_DISTANCE`（配置默认 900m）为半径生成绕行/逃逸多边形，传原始经纬度 `fac.facilities_info[target]` 配合 `ll2utm=True`。

### 6.4 坐标二次转换 bug（传参坐标系不匹配）

**问题**：`plan_detour`/`plan_escape` 的单设施分支把**已是 UTM** 的 `fac.antiairs[target]` 等传给 `get_spec_facility_polyborder(..., ll2utm=True)`，该函数内部会按**经纬度**再转一次 → 二次转换，绕行多边形落在错误位置。此 bug 原本休眠（无配置使用"单设施名 + detour/escape"组合），shaoxing 兜底分支激活后暴露。

**修复**（`uav_planning_actions.py` 6 处 + `run_example.py` 6 处）：统一改为传原始经纬度 `fac.facilities_info[target]`（`antiairs`/`headquartors`/`probers` 均为 `facilities_info` 子集，替换安全）。`quick_path_planners.py` 的 6 处是正确写法（UTM + `ll2utm=False`），未改动。

### 6.5 SimpleBorders 返回契约统一（修 'float' is not iterable）

**问题**：`get_spec_facility_polyborder` 返回**裸元组** `(xs, ys)`，而它的三个姊妹函数（`get_defence_facilities_polyborder` 等）返回**单元素列表** `[(xs, ys)]`。所有调用方统一用 `[0]` 解包——对姊妹函数恰好拿到 `(xs, ys)`，对裸元组则解出 xs 数组，`SimpleBorders` 内再 `[0]`/`[1]` 就取到两个 float → `TypeError: 'float' object is not iterable`。此函数此前从未被真正调用成功过。

**修复**（`basic_functions.py`）：`get_spec_facility_polyborder` 改为返回 `[coords.xy]`，与姊妹函数一致；外部 20 处 `[0]` 调用点零改动全部自动修复。内部唯一裸元组调用方（3D/2D 可视化辅助）同步适配，顺带修掉其 `_border_xys[:, 0]` 对元组索引必崩的隐性 bug。

### 6.6 蛇形轨迹修复（三次样条 → 线性采样）

**问题**：detour 折线是"超长直线进入段（~1.9km）+ 短圆弧（80m 间距顶点）"的极端非均匀几何，`cubic_interpolation_3d` 用**按点序号均匀参数化**的全局三次样条（`not-a-knot`），C2 连续性迫使样条先在进入段外凸（实测 +864m）、再在圆上 -94m ~ +24m 间反复过冲 → 轨迹呈蛇形。

**修复**：
- `math_curves_generators.py` 新增 `linear_densify_3d(traj, step=80.0)`：按**水平距离**线性采样，严格贴合折线
- `uav_planning_actions.py` `insert_height_val`：detour 分支改走线性采样；orbit/loiter 保持样条（几何均匀无此问题）

**验证**（同一场景实测）：圆弧段最大横向偏差从 **94.0m 降至 0.91m**（后者即 80m 弦在 900m 圆上的弓高，几何必然值）。

### 6.7 绕行步数调整

`plan_detour` 的 `detour_steps` 默认 5 → **10**：圆弧绕行约 727m（46°），最终参考轨迹 35 个采样点（约每 80m 一个）。执行期间使用 0.5s round 和 8m 水平步长上限在参考点之间插入物理帧，不再按“一个参考点/帧”瞬移。整圈约 78 顶点，需要更多绕行直接改该参数。

### 6.8 可视化适配（两个可视化器）

| 文件 | 适配内容 |
|---|---|
| `visualize/redis_data_visualize.py`（实时） | ① 未分类设施以黑点 `ko` 绘制（`plot_facilities`）；② 加入悬浮提示（`_collect_facility_points`）；③ 计入自动视野范围（`compute_static_range`）；④ `dist_to_target` 为字符串 `'initializing'` 时的格式化防护；⑤ 兼容新布尔值和历史字符串版 `is_waiting`，面板显示 `waiting_reason/flight_phase` |
| `visualize/pyqt_visualize.py`（离线回放） | ① 未分类设施绘制（`init_draw_map` + `draw_plot` 两处）；② 坐标系开关：按钮改为 **QCheckBox"UTM 坐标"**（勾选=UTM/取消=经纬度），点击立即重绘并刷新数字面板；③ 修复视野与首启空白问题；④ 离线状态面板显示 `waiting_reason/flight_phase` |

### 6.9 改动文件清单（本适配全部改动）

| 文件 | 改动 |
|---|---|
| `planning_modules/uav_planning_actions.py` | detour/escape 兜底 + 6 处二次转换修复 + detour 线性采样 + `detour_steps` 10 |
| `planning_modules/basic_functions.py` | `get_spec_facility_polyborder` 返回契约统一 + 内部调用方适配 |
| `planning_modules/math_curves_generators.py` | 新增 `linear_densify_3d` |
| `visualize/redis_data_visualize.py` | 未分类设施绘制/悬浮/视野 + dist 类型防护 |
| `visualize/pyqt_visualize.py` | 未分类设施绘制 + UTM 坐标系开关（QCheckBox）+ 视野逻辑修复 |
| `run_example.py` | 6 处二次转换修复（legacy 演示脚本，一并修防踩坑） |

## 7. 统一仿真时钟、物理步进与 latest 轨迹接口

本章记录 agent02 作为 `situationawareness latest` 轨迹样本生产者的完整改造。目标是让多 Agent 的位置、时间、高度和阶段语义具有统一数据契约，并避免不真实速度扭曲 SituationEngine 的轨迹动向、快速接近和 ETA 判断。

### 7.1 统一仿真时钟

过去每个 Agent 在写 Redis 时使用各自的墙钟，同一仿真帧会因调度和 Redis 延迟产生不同时间戳。现在由全局 round 作为唯一仿真时间基准：

```text
simTimeMs = sim_start_time_ms + round_id × sim_dt_ms
```

默认 `DT=0.5s`，因此 `sim_dt_ms=500`。启动时在 Redis 初始化：

| Redis 世界状态 | 含义 |
|---|---|
| `sim_round` | 当前全局仿真轮次 |
| `sim_start_time_ms` | 仿真的 epoch 毫秒基准 |
| `sim_dt_ms` | 每个 round 代表的仿真毫秒数，默认 500 |

每个 `traj_extra` 记录的时间字段：

| 字段 | 单位 | 用途 |
|---|---:|---|
| `round_id` | round | 定位仿真帧 |
| `simTimeMs` | epoch ms | 标准仿真时间，导出 `ts` 的数据源 |
| `timestamp` | epoch s | 兼容旧可视化器 |
| `recordedAtMs` | epoch ms | Agent 实际写入 Redis 的墙钟，仅用于延迟/过期诊断 |

Redis 当前位置中的 `ts` 也使用 `simTimeMs`，而 `recordedAtMs` 保留墙钟。`save_trajectories` 导出的 `ts[]` 为 epoch 毫秒，不再是帧序号或每个 Agent 的写入时刻。导出顶层同时包含：

```json
"simulationMeta": {
  "startTimeMs": 1786453580000,
  "dtMs": 500,
  "timeBasis": "SIMULATION_ROUND",
  "kinematics": {
    "maxHorizontalSpeedMps": 16.0,
    "maxClimbRateMps": 5.0,
    "maxDescentRateMps": 5.0
  }
}
```

### 7.2 有界物理步进

参考轨迹中的相邻点可能间隔 80m甚至更远。旧实现 `nxt = target` 会让 UAV 每 500ms 直接跳到下一点，80m/0.5s 就会被 latest 正确但不合理地解释为 160m/s。现在 `bounded_motion_step()` 按仿真 round 进行有界移动：

| 参数 | 默认值 | DT=0.5s 时每帧上限 |
|---|---:|---:|
| `MAX_HORIZONTAL_SPEED_MPS` | 16m/s | 8m |
| `MAX_CLIMB_RATE_MPS` | 5m/s | 2.5m |
| `MAX_DESCENT_RATE_MPS` | 5m/s | 2.5m |

水平位移和垂直位移独立限幅，短于最大步长时直接落到目标而不超调。到达误差改为三维距离；即使 `lookahead` 已指向最后一个参考点，也只有真正进入 `CLOSE_TH_SYNC` 阈值后才完成航段，避免物理步进引入“索引到终点但飞机仍在远处”的提前完成。

### 7.3 等待状态与飞行阶段契约

`is_waiting` 原先混用 `True`、`"False"`、`"flying to start"` 和 `"initializing"`。Python 会把非空字符串 `"False"` 判为真，导致同步轨迹过滤把正常飞行帧也全部丢弃。新契约将它拆分为：

| 字段 | 类型 | 含义 |
|---|---|---|
| `is_waiting` | JSON boolean | 只表示当前帧是否处于停住等待 |
| `waiting_reason` | string/null | 等待或特殊阶段原因 |
| `flight_phase` | enum string | 该帧在任务生命周期中的阶段 |

| `flight_phase` | `is_waiting` | 说明 | 进入 `uavs_coords_str` |
|---|---:|---|---:|
| `initializing` | `true` | Agent 初始位置 | 否 |
| `positioning` | `false` | 向当前航段起点飞行 | 否 |
| `sync_wait` | `true` | 已到起点，等待 barrier release | 否 |
| `task_flight` | `false` | 正常执行任务航段 | 是 |

该改动不影响 SPADE-BDI/AgentSpeak 规划。`uav_key_path.asl` 中没有 `is_waiting(...)` 信念，它只根据 `can_task_start(true/false)`、`if_set_ref_traj(...)` 和 `cur_nodes(...)` 控制规划。`is_waiting` 仅是 Redis/JSON 轨迹元数据。注意不要因此全局替换 BDI 层的所有 `"False"`：BDI Literal 与 JSON boolean 是两套不同语义。

### 7.4 raw 与 latest 分析轨迹的分层导出

`save_trajectories()` 现在明确区分两种用途：

| 字段 | 用途 | 数据范围 |
|---|---|---|
| `uavs_coords_raw` | 审计、排错、完整回放 | 保留初始化、定位、barrier 等待和任务飞行的每个物理帧 |
| `uavs_coords_str` | SituationAwareness latest 轨迹样本 | 只保留同步对齐的 `task_flight` 帧 |

`uavs_coords_str` 的选择逻辑：

1. 排除 `segment_key=initializing`、非整数 `frame_id`、`frame_id<=0`、定位和等待帧。
2. 新数据只接受 `flight_phase=task_flight && is_waiting=false`。
3. 兼容旧导出数据：显式解析字符串 `"False"`，不再使用 Python 非空字符串的默认真值。
4. 对每个航段求参与 Agent 的公共 `frame_id` 交集，保证多目标使用同一组参考帧。
5. 物理步进期间同一 `frame_id` 会产生多个中间点；分析轨迹取该帧最后一个物理样本，即接近当前参考点的位置。

这样初始位置到航段起点的跨阶段距离不会进入 latest，不会在首段制造几千 m/s 的假速度；raw 中仍然保留这些记录供排查。

每个导出目标的结构为：

```json
{
  "lngs": [116.36, 116.3601],
  "lats": [39.87, 39.8701],
  "alts": [200.0, 200.5],
  "ts": [1786453581500, 1786453582000],
  "extras": [
    {
      "round_id": 3,
      "simTimeMs": 1786453581500,
      "recordedAtMs": 1786453579123,
      "segment_key": "0_3",
      "frame_id": 1,
      "is_waiting": false,
      "waiting_reason": null,
      "flight_phase": "task_flight"
    }
  ]
}
```

`lngs/lats/alts/ts/extras` 必须等长，`ts` 必须严格递增。SituationEngine 对单个目标默认至少要求 6 个轨迹点；航段过短或同步交集不足 6 点时，仍需在样本组装层合并连续任务帧或跳过该目标。

### 7.5 完整 flight_plan 与计划航迹导出

`cur_reference_traj` 和 Redis 的 `uav:{uid}:ref_traj` 都只保存当前航段，进入下一航段时会被覆盖。`save_trajectories()` 现在新增 `plannedRoutes`：按 `agent.flight_plan` 的航段顺序读取每个 `(fromNode,toNode,memberId)` 对应的 `nodes_pair_member_traj`，重建每架无人机的完整计划航迹。

```json
{
  "plannedRoutes": {
    "agent_1_0": {
      "source": "nodes_pair_member_traj",
      "altitudeReference": "AMSL",
      "complete": true,
      "flightPlan": [
        {"order": 0, "segmentKey": "0_3", "fromNode": "0", "toNode": "3"}
      ],
      "segmentCount": 1,
      "routePointCount": 80,
      "flightRoute": [
        {"lng": 116.36, "lat": 39.87, "alt": 200.0}
      ],
      "segments": [],
      "missingSegments": []
    }
  }
}
```

拼接只删除两个航段完全相同的公共端点；如果换队形等原因造成航段边界不连续，会保留两侧端点，不会静默丢掉下一航段起点。某段参考轨迹缺失时不影响原有轨迹文件保存，当前目标标记 `complete=false` 并在 `missingSegments` 中给出原因。

该逻辑只在仿真结束导出时读取 Redis，不修改 `cur_reference_traj`、lookahead、同步状态或 Agent 飞行过程。`flightRoute` 是计划轨迹，`uavs_coords_str/uavs_coords_raw` 仍是实际执行轨迹，两者保持独立。

### 7.6 与 latest 的字段对应

| agent02 导出 | latest `uavs_coords_str.targetId` | 说明 |
|---|---|---|
| Agent 名，如 `agent_1_0` | 对象 key / `targetId` | 稳定 ID，可直接沿用 |
| `lngs/lats` | `lngs/lats` | 已从 UTM 转回经纬度 |
| `alts` | `alts` | 等长高度序列，当前为仿真绝对高度语义 |
| `ts` | `ts` | epoch 毫秒，由统一仿真时钟生成 |
| `extras` | latest 非必填扩展 | 用于溯源、阶段过滤和排错 |

设施、圈层和目标属性由 latest 侧离线适配器补齐，agent02 主程序继续只负责仿真和导出，避免把 SituationEngine 耦合进 SPADE/Redis 运行循环。完整映射清单见 `situationawareness latest/agents02_to_situation_judgment_接入清单.md`。

### 7.7 latest 离线适配器

已实现 `situationawareness latest/tools/agents02_export_to_payload.py`，负责：

- 读取 `uav_trajectories_persistent_*.json`，默认消费 `uavs_coords_str`；
- 校验并兼容 `ts/simTimeMs/round_id`，过滤非 `task_flight`、等待和初始化样本；
- 生成默认 12 点、步幅 6 的滑动窗口，确保单目标至少 6 点；
- 用共同快照截止时间组装多目标请求，并排除超过 `simulationMeta.dtMs` 的陈旧位置；
- 映射 `facilities_str/defence_rings`，补齐 `baseData` 六个数组和默认 `targetAttributes`；
- 输出 `adapterMeta + payloads[]`，也可直接逐项调用 `SituationEngine.analyze()`。

在 latest 根目录、`study` 环境运行：

```powershell
conda run -n study python tools/agents02_export_to_payload.py "../uav_strategy/examples/uavs_strategy/data/raw_data/uav_trajectories_persistent_20260812_001009.json" "../uav_strategy/examples/uavs_strategy/data/facilities_shaoxing.json" --output outputs/agents02_analyze_payloads.json --require-all-targets --call-engine
```

将占位符替换成一个明确的 `uav_trajectories_persistent_*.json` 文件名。若需要每个时刻尽量保留当前活跃目标，可去掉 `--require-all-targets`；若需要放宽跨 Agent 的时钟偏差，可显式设置 `--max-snapshot-skew-ms`，但不建议用过大值把陈旧位置拼成一个蜂群快照。

### 7.8 可视化兼容

- 实时可视化优先使用新布尔值 `is_waiting`，同时兼容历史字符串数据，避免 `"False"` 被显示为等待中。
- 实时面板和离线回放面板增加 `waiting_reason` 和 `flight_phase`。
- 实时活跃性使用 `recordedAtMs`，不使用可能跑在真实墙钟前后的 `simTimeMs`。
- 离线回放可根据用途选择 `uavs_coords_raw` 或 `uavs_coords_str`：前者展示完整生命周期，后者展示 latest 实际消费的任务轨迹。

### 7.9 回归测试与验证

新增 `tests/test_agents02_trajectory_contract.py`，覆盖：

1. `initializing/positioning/sync_wait` 不进入分析轨迹，`task_flight` 可进入。
2. 历史字符串 `"False"` 可兼容读取，`"flying to start"` 仍被排除。
3. 初始点被排除，多 Agent 公共帧交集和同帧最后物理样本选择正确。
4. 统一时钟每 round 精确增加 500ms。
5. 水平位移、爬升和下降不超过配置上限，短距离不超调。
6. 最后一个参考点未真正到达时不提前完成任务。
7. 多航段完整 `flight_plan` 按顺序拼接，重复公共端点只保留一次。
8. 航段边界不一致时保留两侧点，缺失航段标记为不完整但不丢失其他航段。

在 `study` 环境运行：

```bash
conda run -n study python -m unittest discover -s tests -p "test*.py" -v
```

当前新增的 11 项契约、物理和完整计划航迹测试全部通过，相关 Python 文件也已通过 `py_compile`。这些是不依赖 Redis/Openfire 的逻辑回归；每次改动 round/barrier 或 Redis 写入逻辑后，仍应补跑一次完整 SPADE + Redis + XMPP 仿真。

latest 侧另有 `tests/test_agents02_export_adapter.py`，覆盖设施/圈层映射、阶段过滤、历史时间兼容、滑窗、多目标快照、陈旧目标剔除和 SituationEngine 直连。当前 7 项测试全部通过；改造后的实际样本以 `--require-all-targets` 转换后，13 个目标直连引擎得到 `code=0`、`skippedTargets=[]`、3 个集群。

### 7.10 本轮改造文件清单

| 文件 | 改动 |
|---|---|
| `uav_dynamic_agents02.py` | 统一仿真时钟字段、初始阶段元数据、raw/分析轨迹分层、旧数据兼容过滤、公共帧交集、高度与毫秒 `ts`、`simulationMeta.kinematics`、完整 `plannedRoutes` 重建导出 |
| `behaviors_modules/uav_periodic_behaviours.py` | 有界水平/垂直物理步进、三维到达判定、终点完成防提前、布尔 `is_waiting`、`waiting_reason/flight_phase` |
| `redis_modules/uav_redis_io.py` | 当前位置 `ts` 使用 `simTimeMs`，保留 `recordedAtMs` 诊断墙钟 |
| `visualize/redis_data_visualize.py` | 新旧等待值兼容，显示等待原因和飞行阶段 |
| `visualize/pyqt_visualize.py` | 离线面板显示等待原因和飞行阶段 |
| `tests/test_agents02_trajectory_contract.py` | 新增 11 项轨迹契约、时钟、物理步进和完整计划航迹回归测试 |
| `situationawareness latest/tools/agents02_export_to_payload.py` | agent02 导出到 `SituationEngine.analyze()` 请求的离线适配器和可选引擎回放 CLI |
| `situationawareness latest/tests/test_agents02_export_adapter.py` | 新增 7 项适配器契约及引擎直连测试 |
| `situationawareness latest/agents02_to_situation_judgment_接入清单.md` | 更新生产者当前能力、已完成项和风险边界 |

## 8. 已知问题与后续建议

- **死代码导入**：`behaviors_modules/uav_periodic_behaviours.py` 顶部 `from math import dist` / `from turtle import st` / `from pyparsing import C` / `from ray import state` / `from regex import F` / `from sympy import true` 均为无用导入（部分被局部变量遮蔽），建议删除；`from regex import F` 曾导致 `ModuleNotFoundError`。
- **zone 51 硬编码**：见 §4，内部自洽，但跨区带混入真实 UTM 数据会错位。
- **orbit/loiter 仍走样条**：若日后出现长进入段 + 盘旋的组合产生蛇形，可用与 detour 相同的方式处理。
- **config 6 注释**：`switch_config == 6` 注释写"绍兴空域"，实际坐标在 116.38°E（北京附近），建议修正注释。
- **高度口径**：agent02 导出绝对高度，latest 协议中 `alts` 定义为 AGL；当前可用于仿真样本，接入真实地形后需增加高程基准换算。
- **latest 最小点数**：分析轨迹经阶段过滤和多 Agent 交集后可能不足 6 点，应在滑动窗口/样本组装层检查，不应回退到 raw 初始化帧凑点数。
- **仿真倍速尚未实现**：当前 Agent 墙钟调度周期仍为 `DT=0.5s`。后续若需缩短实际运行时间，应新增独立 `SIM_SPEEDUP`，只把行为的墙钟周期改为 `DT/SIM_SPEEDUP`；不要改变仿真 `DT`、`sim_dt_ms`或每帧物理步长，否则会再次改变 latest 计算的速度和 ETA。
- **完整联调**：当前契约、物理逻辑、实际导出转换和 SituationEngine 本地直连已通过；HTTP 服务回放及 I-01~I-30 意图结果验收仍待后续联调。
