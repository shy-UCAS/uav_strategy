# uav_strategy 语义丰富指南：让仿真忠实还原 NL 任务

把 NLTaskOrchestration 的任务语义更忠实地还原到项目B（uav_strategy）仿真里，消除 [uav_strategy_integration_guide.md](uav_strategy_integration_guide.md) §一 提到的"语义降级"。

**范围（2026-06-11 确认）**：
- **L1 几何行为差异化** —— 9 种动作按 **4 大类**给不同飞行模式/队形/高度。
- **L2 关键协同** —— condition_trigger 触发等待、跨 fleet 时序、汇聚同步到达。
- **不做 L3** —— 不引入时间轴/弹药/能量/截止（与"先跑通"基调一致；这些留在 NL 侧由 Z3 验证）。

责任边界：**转换器侧改动**（数据契约扩展）在项目A，可由本项目实现；**B 侧改动**在 `../uav_strategy`，需你手动（B 只读）。本文每条都标注归属。

---

## 0. NLTaskOrchestration 会生成什么（动作 / 关系 / 约束全集）

来自 [configs/action_templates.yaml](../configs/action_templates.yaml) 与 [tools/dataset/generate_cases.py](../tools/dataset/generate_cases.py)。

**9 种动作**（按本指南的 4 大类归并）：

| 大类 | NL 动作 | NL 语义 | 能力需求 |
|---|---|---|---|
| **打击类** | strike / breakthrough / intercept | 精确打击 / 强行突破防御 / 拦截移动目标 | strike_capable |
| **侦察类** | reconnaissance / track | 侦察目标点区域 / 持续跟踪移动目标 | recon_capable |
| **支援类** | jam / standby | 电子干扰压制雷达 / 原地待命 | jamming_capable / 无 |
| **机动汇合类** | fly_to / rendezvous | 飞行转移 / 多集群同步会合 | 无 |

**4 种关系**：sequence（顺序）、parallel（并行）、condition_trigger（条件触发）、sync（同步，GCJP 里编码为 group_sync）。

**6 类约束**：time_order、time_window(deadline)、group_sync、physical_feasibility、capability、resource(ammo/energy)。其中 **time_window / resource / capability / physical_feasibility 属 L3**，本指南不接入。

---

## 1. 现状 → 目标对照

| 语义 | 现状（接入指南交付的版本） | 本指南目标 |
|---|---|---|
| 动作行为 | 9 种全压成 `breakthrough` 直插 | 4 大类各自飞行模式（直插/盘旋/悬停/汇合） |
| 队形 | B 侧 `random.choice` 每段随机 | 由动作大类决定（可复现） |
| condition_trigger | 丢进 meta.soft_deps | 侦察确认后才触发打击（Redis flag 等待） |
| 跨 fleet 时序 | 丢进 meta.soft_deps | 先侦察后突击（航段依赖等待） |
| 汇聚同步 | singleton，各自到达不等待 | 同步组到齐再继续 |

---

## 2. 数据契约扩展（转换器侧，项目A）

当前 `digraph_attrs[].attrs` 只有 `{order_mode, order_type, target, fleet_no}`。L1+L2 需要在**每条航段**补这些字段（动作语义现在只存在 `meta.task_to_segment`，要提升进 `attrs` 供 B 直接读）：

```jsonc
{
  "from": 2, "to": 3,
  "attrs": {
    "order_mode": "singleton",
    "order_type": "breakthrough",     // 保留：B 现有分发仍可用作兜底
    "target": "hq_mark14",
    "fleet_no": "f2.1",
    // —— L1 新增 ——
    "action": "strike",               // 原始 NL 动作（B 据此选飞行模式）
    "action_class": "assault",        // 4 大类：assault/recon/support/maneuver
    "formation_type": "vshape",       // 由动作类映射的队形（替代 B 的 random.choice）
    // —— L2 新增 ——
    "segment_id": "seg_t1_strike",    // 航段唯一 id（供依赖引用）
    "depends_on": [],                 // 开始前需完成的 segment_id（承载跨 fleet 时序 + condition）
    "sync_group": null                // 汇聚同步组 id；同组航段互相等待
  },
  "members_num": 3
}
```

**字段来源（转换器如何填）**：
- `action` / `action_class`：从 BuiltGraph 的 TaskNode.action 直接取 + 一张 `action → 4 大类`映射。
- `formation_type`：一张 `action_class → 队形`映射（assault→vshape，recon→circular，support→horizontal，maneuver→arc）。
- `segment_id`：每条航段（即每个 task）已有稳定 task_id，直接用 `seg_<task_id>`。
- `depends_on`：把现在丢进 `soft_deps` 的**跨 actor 依赖**和 **condition_trigger** 都还原成航段依赖——
  - 跨 fleet sequence `t0→t1_strike`：`seg_t1_strike.depends_on += [seg_t0]`。
  - condition_trigger `condition="ua_3_confirmed"`：找到产出该条件的侦察航段（target==`ua_3` 的 recon/track），`seg_consumer.depends_on += [seg_producer]`。
- `sync_group`：现在 `meta.sync_groups` 已有 group_sync 组，给每组一个 id，组内各 rendezvous 航段写同一 `sync_group`。

> 这部分（转换器扩展）可由本项目直接实现——它不动项目B，只是让导出的 `digraph_attrs` 多带语义。**B 侧旧逻辑对未知字段无感**，所以可以先加字段、B 暂不读，渐进迁移。

---

## 3. L1：4 大类飞行行为差异化（B 侧）

核心思路：把 [execute_path_planning_from_digraph](../../uav_strategy/examples/uavs_strategy/planning_modules/uav_planning_actions.py#L254) 的分发**从 `order_type` 改为 `action_class`**，复用现有 3 个规划器 + 新增 2 个。

### 3.1 映射设计

| action_class | 飞行模式 | B 落地 | 高度剖面（height_range_set） | 队形 |
|---|---|---|---|---|
| **assault**（打击） | 直插突防 + 末端攻击 | `execute_breakthrough`（现成）→ 到点后 `execute_attack` 置标记 | `[[250,400],[0,100]]` 高进低出俯冲 | vshape |
| **recon**（侦察） | 抵近后绕目标盘旋 | **新增** `execute_orbit`（绕 target 画圆） | `[[150,300],[150,300]]` 中低空保持 | circular |
| **support**（支援） | jam→绕雷达盘旋；standby→原地悬停 | jam 用 `execute_orbit`；standby **新增** `execute_loiter` | `[[200,350],[200,350]]` 中空保持 | horizontal |
| **maneuver**（机动汇合） | fly_to 直飞；rendezvous 汇合 | `execute_breakthrough`；rendezvous 见 §4.3 | 巡航高度 `[[200,300],[200,300]]` | arc |

> **关于队形列**：暂保持 B 现有的 `random.choice`（你的决定）；上表"队形"列是**推荐值**，待想固定时按改动 5 启用。推荐依据 B 的 [generate_formation_offsets](../../uav_strategy/examples/uavs_strategy/planning_modules/formation_generator.py#L55) 真实几何：`vshape` 楔形尖端朝前（集中突防）、`circular` 360° 环绕（盘旋覆盖）、`horizontal` 沿航向侧方一字排开（横向搜索）、`arc` 半圆扇面（多向收拢汇合）、`vertical` 垂直于航向的纵列（高度错开、穿越防撞）。

### 3.2 B 侧改动清单

**改动 1 — 分发逻辑** [uav_planning_actions.py:254](../../uav_strategy/examples/uavs_strategy/planning_modules/uav_planning_actions.py#L254)

```python
def execute_path_planning_from_digraph(self, digraph, start_h, end_h):
    attrs = digraph['attrs']
    cur_target = attrs['target']
    action_class = attrs.get('action_class')          # 新增：优先按动作类分发
    if action_class == 'recon':
        return self.execute_orbit(cur_target, start_h, end_h)
    elif action_class == 'support':
        action = attrs.get('action')
        if action == 'standby':
            return self.execute_loiter(start_h, end_h)
        return self.execute_orbit(cur_target, start_h, end_h)   # jam 绕目标
    elif action_class in ('assault', 'maneuver'):
        return self.execute_breakthrough(cur_target, start_h, end_h)
    # 兜底：保留旧的 order_type 分发（向后兼容）
    order_type = attrs.get('order_type', 'breakthrough')
    ...
```

**改动 2 — 新增 `execute_orbit`** —— 不依赖 facilities 分类（`plan_detour` 要求 target 归 antiair/hq/prober，hq_markN/ua_N 会 `raise`，所以侦察不能直接复用 detour，需自带画圆几何）：

```python
def execute_orbit(self, target, start_h, end_h, radius=80.0, steps=8):
    """抵近 target 后绕其画一圈（侦察/跟踪/干扰的盘旋）。"""
    tgt = self.agent.facilities.get_target_location(target, utm=True)  # 现成
    start = self.agent.traj[-1]
    # 先直飞到目标外缘，再绕一圈（用现成 SimpleBorders 或自写圆周采样）
    import numpy as np
    cx, cy = tgt[0], tgt[1]
    orbit = [[cx + radius*np.cos(2*np.pi*k/steps),
              cy + radius*np.sin(2*np.pi*k/steps)] for k in range(steps+1)]
    traj_2d = [start[:2]] + orbit
    traj_3d = self.insert_height_val("orbit", traj_2d, start_h, end_h)   # 见改动 4
    return traj_3d
```

**改动 3 — 新增 `execute_loiter`**（standby 原地悬停）：

```python
def execute_loiter(self, start_h, end_h, hold_steps=6):
    pos = self.agent.traj[-1]
    traj_2d = [pos[:2] for _ in range(hold_steps)]   # 原地保持
    return self.insert_height_val("loiter", traj_2d, start_h, end_h)
```

**改动 4 — 高度区间 + 插值** [uav_dynamic_agents02.py:73](../../uav_strategy/examples/uavs_strategy/uav_dynamic_agents02.py#L73) 的 `height_range_value_set` 加 `orbit`/`loiter` 两个 key（值见 §3.1 表）；并让 [insert_height_val:101](../../uav_strategy/examples/uavs_strategy/planning_modules/uav_planning_actions.py#L101) 的 `if order_type == "detour"`（用 cubic 平滑）扩成 `if order_type in ("detour", "orbit", "loiter")`，让盘旋轨迹也走样条平滑。

**改动 5 — 队形改读字段**（替代随机）[uav_dynamic_agents02.py:339](../../uav_strategy/examples/uavs_strategy/uav_dynamic_agents02.py#L339)：把 `_formation_type = random.choice([...])` 改成 `_formation_type = digraph_attr['attrs'].get('formation_type') or random.choice([...])`。有字段用字段，没有才随机（向后兼容）。

---

## 4. L2：关键协同（B 侧）

三项协同的**共同基础**是一套"航段完成信号"：每条航段飞完时在 Redis 置 `seg_done:<segment_id>=1`；依赖它的航段开始前轮询等待。复用 B 现成的 `io.set_uav_state` / `io.get_uav_state` 与 round 节拍。

### 4.1 跨 fleet 时序 + condition_trigger（统一为航段依赖）

二者本质相同——"航段 B 必须等航段 A 完成"，都由 §2 的 `depends_on` 承载。

**B 侧改动** —— 在 [act_digraph_path_planning](../../uav_strategy/examples/uavs_strategy/uav_dynamic_agents02.py#L274) 进入航段规划**之前**加一道依赖闸：

```python
# 航段开始前：等待所有前置航段完成
_deps = digraph_attr['attrs'].get('depends_on') or []
for dep_seg in _deps:
    if self.io.r.get(f"seg_done:{dep_seg}") != b"1":
        self.bdi.set_belief("can_task_start", "false")   # 未就绪，下个 round 再来
        yield
        return
```

航段**飞完时**（终点判定处，[uav_periodic_behaviours.py:1096](../../uav_strategy/examples/uavs_strategy/behaviors_modules/uav_periodic_behaviours.py#L1096) 附近 `is_finished/进入下一段`时）置位：

```python
io.r.set(f"seg_done:{agent.current_segment_key_id}", "1")  # current_segment 的 segment_id
```

> 这样 condition_trigger（侦察→打击）与跨 fleet sequence（先侦察后突击）都生效：消费航段会一直在依赖闸前等待，直到生产航段置位 `seg_done`。

### 4.2 汇聚同步到达（group_sync）—— 两种模式都支持

把"**同步等待**"与"**汇合点坐标来源**"解耦，两种汇合点由 `target` 字段自然区分：

- **模式 a — 固定设施名**（如 `target=hq_mark7`）：各机同步等待后飞向该**真实坐标**。
- **模式 b — 编队几何中心**（`target=aggregate_point`）：各机同步等待后飞向 `merge_peers` 的**动态几何中心**（B 现成 [io.get_rendezvous_point](../../uav_strategy/examples/uavs_strategy/uav_dynamic_agents02.py#L286)）。

**B 侧只需一处放宽**：把 [act_digraph_path_planning:299](../../uav_strategy/examples/uavs_strategy/uav_dynamic_agents02.py#L299) 的同步触发条件从
`if _order_mode=='aggregate' and _order_target=='aggregate_point'` 改成 `if _order_mode=='aggregate'`（或 `if attrs.get('sync_group')`）——让"是否同步等待"不再绑定 target。`execute_breakthrough` 本就同时处理两种 target（[:283](../../uav_strategy/examples/uavs_strategy/planning_modules/uav_planning_actions.py#L283) 的 `aggregate_point`→几何中心、[:281](../../uav_strategy/examples/uavs_strategy/planning_modules/uav_planning_actions.py#L281) 的设施名→飞向设施），无需再改。

**转换器侧**：汇聚组航段统一写 `order_mode=aggregate` + `sync_group=<id>`；`target` 取 NL 给的设施名（模式 a，默认，信息更具体）或字面量 `aggregate_point`（模式 b）。用 `--rendezvous-mode {facility,geometric}` 开关切默认，或在样本级指定。

### 4.3 关系 parallel

无需改动：generator 的 single-fleet exclusion 保证同 fleet 任务必串行，跨 fleet parallel 在 B 里天然是两条独立 key_path 同时推进（round 节拍本就并行）。转换器丢弃 parallel 边即可。

---

## 5. 改动清单汇总

**转换器侧（项目A，可由本项目实现）**
1. `attrs` 增 `action` / `action_class` / `formation_type`（L1）。
2. `attrs` 增 `segment_id` / `depends_on` / `sync_group`（L2）——把现在 `soft_deps`/`sync_groups` 里的依赖与同步组提升为结构化字段。
3. 两张映射表：`action→action_class`、`action_class→formation_type`。

**B 侧（`../uav_strategy`，你手动）**
1. `execute_path_planning_from_digraph` 改按 `action_class` 分发（§3.2 改动 1）。
2. 新增 `execute_orbit` / `execute_loiter`（改动 2/3）。
3. `height_range_value_set` 加 `orbit`/`loiter`；`insert_height_val` 平滑分支扩容（改动 4）。
4. `formation_type` 改读字段（改动 5）。
5. `act_digraph_path_planning` 加 `depends_on` 依赖闸 + 航段完成置 `seg_done`（§4.1）。
6. 汇聚同步按选定路线调整（§4.2）。

---

## 6. 分阶段实施建议

**阶段 1（L1，先验证可视化区分）**：只做转换器字段 1 + B 改动 1~5。跑 aggregate_disperse，看四路是否呈现"侦察盘旋 / 打击直插 / 不同队形"。这一步零协同、风险最低。

**阶段 2（L2 依赖）**：加转换器字段 2 + B 改动 5。跑 condition_trigger 样本，看打击航段是否真的等侦察完成。

**阶段 3（L2 同步）**：选定 §4.2 路线，跑 group_sync 样本，看多机是否同步抵达。

每阶段都用 `BlueUAVAgent.VERBOSE=True` 看航段规划与等待日志，确认 orchestrator 正常退出。

---

## 7. 决策记录（2026-06-11）

- **汇聚同步**：两种模式都支持（§4.2）——`target` 是设施名走模式 a（飞真实坐标）、是 `aggregate_point` 走模式 b（飞几何中心）。B 侧只需放宽一处同步触发条件，二者共存。
- **队形**：暂保持 B 的 `random.choice`，不进数据契约（§3.1）；上表队形列为推荐值，待想固定时再启用。
- **jam**：归 support 大类，用 `execute_orbit` 绕目标盘旋。
- **orbit/loiter 参数**：`radius`/`steps`/`hold_steps` 是飞行形态旋钮（§3.2 改动 2/3）。战场约 5km 尺度下，默认 80m 盘旋圈紧贴目标；想要可视化上明显的盘旋可调 200~500m。这些不阻塞实现，跑时再调。

**下一步**：转换器侧字段扩展（§2 + `formation_type` 暂缓）可由本项目直接实现，让导出数据先带上 L1/L2 语义（不动 B）；B 侧改动按 §5 清单逐项落地，或一阶段一阶段配着调。
