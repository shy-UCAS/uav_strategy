# agent_dynamic_agents02将所有无人机看做一个独立的agent，不再区分每个任务阶段设置一个agent
# 所有agent任务完成后不停止,等待下一个任务指令
# 通过MissionOrchestrator统一管理所有Persistent Agents的生命周期
# 每个agent根据key_paths生成的bdi_instructions执行任务
# 每个agent根据当前航段的属性决定是否需要与同航段其他agent同步
# 如果需要同步，则等待其他agent准备完毕后再开始当前航段任务
# 每个agent在执行航段任务前，先检查当前航段是否已有基准参考轨迹
# 如果没有，则生成基准参考轨迹，并根据编队参数生成所有成员的轨迹，存储到redis服务器
# 如果有，则直接从redis服务器获取自己的轨迹
# 结合key_paths处理提取出的数据,数据导入redis服务器
# 使用一个固定通用的asl文件处理uav_key_path.asl

import asyncio
import getpass
import os, os.path as osp
import argparse
import sys
import redis
import json
import numpy as np
import spade
import agentspeak
import time
import collections
import random
import math
import networkx as nx

from typing import Callable, Dict, List, Optional, Iterable, Tuple, Any
from matplotlib.animation import FuncAnimation
# from time import time
from datetime import datetime
from sympy import N

from spade.agent import Agent
from spade_bdi.bdi import BDIAgent
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO
from examples.uavs_strategy.planning_modules.uav_planning_actions import PlanningLib
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.planning_modules.formation_generator import FormationGenerator3D, Formation_Elements
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import (
    DT,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
    MAX_HORIZONTAL_SPEED_MPS,
    FetchWorldState,
    GlobalRoundCoordinator,
    SyncAPFStep,
    SyncAPFStepEnhance,
)
from examples.uavs_strategy.key_path_analyzer import KeyPathAnalyzer

SIM_CLOCK_START_KEY = "sim_start_time_ms"
SIM_CLOCK_DT_KEY = "sim_dt_ms"

# 仿真墙钟加速倍率。只缩短 PeriodicBehaviour 的真实调度周期，不改变 DT 所
# 代表的仿真时间步长，也不改变每个 round 的物理位移上限，因此最终轨迹与
# 时间戳保持不变，只是生成相同仿真数据所需真实墙钟时间缩短约 SIM_SPEEDUP 倍。
# 默认 1.0 表示 1:1 实时；设为 10.0 表示约 10 倍加速。实际加速上限受 Redis
# 往返、每轮物理计算和 agent 数量影响，不等于严格的线性倍率。
SIM_SPEEDUP = 20.0


def simulation_time_ms(start_time_ms: int, round_id: int, dt_ms: int) -> int:
    """Return the shared simulation timestamp for one global round."""
    return int(start_time_ms) + int(round_id) * int(dt_ms)


def trajectory_timestamps_ms(extras: List[Dict[str, Any]]) -> List[int]:
    """Extract latest-compatible Unix millisecond timestamps from trajectory metadata.

    New traces carry ``simTimeMs``.  The timestamp fallback keeps older trace
    metadata readable while treating the historical ``timestamp`` value as
    Unix seconds when necessary.
    """
    timestamps = []
    for extra in extras or []:
        value = extra.get("simTimeMs")
        if value is None:
            value = extra.get("timestamp")
            if value is None:
                raise ValueError("trajectory metadata is missing simTimeMs/timestamp")
            value = float(value)
            if abs(value) < 10_000_000_000:
                value *= 1000.0
        timestamps.append(int(round(float(value))))
    return timestamps


def _coerce_legacy_waiting(value: Any) -> bool:
    """Interpret historical trajectory metadata without Python truthiness bugs."""
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    if isinstance(value, str):
        return value.strip().lower() not in {"", "false", "0", "no", "none", "null"}
    return bool(value)


def is_analysis_ready_trajectory_point(extra: Dict[str, Any]) -> bool:
    """Return whether one metadata record belongs in the latest-facing track.

    New records use ``flight_phase`` and a real JSON boolean.  The legacy
    fallback keeps older exports readable while excluding initialization,
    positioning, barrier waits, and malformed/string frame identifiers.
    """
    if not isinstance(extra, dict):
        return False

    segment_key = extra.get("segment_key")
    frame_id = extra.get("frame_id")
    if segment_key in (None, "", "initializing"):
        return False
    if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id <= 0:
        return False

    flight_phase = extra.get("flight_phase")
    if flight_phase is not None:
        return flight_phase == "task_flight" and extra.get("is_waiting") is False

    if _coerce_legacy_waiting(extra.get("is_waiting")):
        return False
    lookahead = extra.get("lookahead", frame_id)
    return not isinstance(lookahead, bool) and isinstance(lookahead, int) and lookahead > 0


def build_segment_common_frames(
    raw_trajs: Dict[str, Tuple[List[List[float]], List[Dict[str, Any]]]]
) -> Dict[str, set]:
    """Build the common task-flight waypoint IDs for each participating group."""
    segment_agents = collections.defaultdict(set)
    segment_frames = collections.defaultdict(lambda: collections.defaultdict(set))
    for name, (_, extras) in raw_trajs.items():
        for extra in extras or []:
            if not is_analysis_ready_trajectory_point(extra):
                continue
            segment_key = extra["segment_key"]
            segment_agents[segment_key].add(name)
            segment_frames[segment_key][name].add(extra["frame_id"])

    common_frames = {}
    for segment_key, agent_names in segment_agents.items():
        frame_sets = [segment_frames[segment_key][name] for name in agent_names]
        common_frames[segment_key] = set.intersection(*frame_sets) if frame_sets else set()
    return common_frames


def select_analysis_trajectory(
    trajectory: List[List[float]],
    extras: List[Dict[str, Any]],
    segment_common_frames: Dict[str, set],
) -> Tuple[List[List[float]], List[Dict[str, Any]]]:
    """Select one final physical sample for every common task-flight waypoint."""
    aligned_count = min(len(trajectory or []), len(extras or []))
    last_index = {}
    for index in range(aligned_count):
        extra = extras[index]
        if is_analysis_ready_trajectory_point(extra):
            last_index[(extra["segment_key"], extra["frame_id"])] = index

    selected_trajectory = []
    selected_extras = []
    for index in range(aligned_count):
        extra = extras[index]
        if not is_analysis_ready_trajectory_point(extra):
            continue
        key = (extra["segment_key"], extra["frame_id"])
        if last_index.get(key) != index:
            continue
        if extra["frame_id"] not in segment_common_frames.get(extra["segment_key"], set()):
            continue
        selected_trajectory.append(trajectory[index])
        selected_extras.append(extra)
    return selected_trajectory, selected_extras


def _flight_plan_segment_nodes(segment: Any) -> Tuple[str, str]:
    """Return one ``(from_node, to_node)`` pair from a flight-plan entry."""
    raw_pair = segment.get("segment") if isinstance(segment, dict) else segment
    if not isinstance(raw_pair, (list, tuple)) or len(raw_pair) < 2:
        raise ValueError("flight_plan segment must contain from/to nodes")
    return str(raw_pair[0]), str(raw_pair[1])


def _route_points_from_utm(
    trajectory: List[List[float]],
    lnglat_converter: Any,
    segment_key: str,
) -> List[Dict[str, float]]:
    """Convert a 3-D UTM reference trajectory into FlightPlanDto route points."""
    try:
        trajectory_np = np.asarray(trajectory, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("{} contains non-numeric reference points".format(segment_key)) from exc
    if trajectory_np.ndim != 2 or trajectory_np.shape[0] == 0 or trajectory_np.shape[1] < 3:
        raise ValueError("{} requires non-empty [x,y,z] reference points".format(segment_key))
    if not np.isfinite(trajectory_np[:, :3]).all():
        raise ValueError("{} contains non-finite reference points".format(segment_key))

    lnglat = np.asarray(lnglat_converter.utm_to_lng_lat_array(trajectory_np), dtype=float)
    if lnglat.ndim != 2 or lnglat.shape[0] != trajectory_np.shape[0] or lnglat.shape[1] < 2:
        raise ValueError("{} UTM conversion returned an invalid shape".format(segment_key))
    return [
        {
            "lng": float(lnglat[index, 0]),
            "lat": float(lnglat[index, 1]),
            "alt": float(trajectory_np[index, 2]),
        }
        for index in range(trajectory_np.shape[0])
    ]


def build_complete_flight_plan_export(
    flight_plan: Iterable[Any],
    member_id: str,
    segment_trajectory_getter: Callable[[str, str, str], List[List[float]]],
    lnglat_converter: Any,
) -> Dict[str, Any]:
    """Rebuild one member's complete planned route from per-segment Redis data.

    ``cur_reference_traj`` and ``uav:{uid}:ref_traj`` are overwritten whenever
    the Agent enters a new segment.  The per-node-pair member trajectories use
    distinct Redis keys, so walking ``flight_plan`` in order is the reliable
    way to preserve the complete pre-planned route without changing execution.
    """
    ordered_plan = []
    exported_segments = []
    missing_segments = []
    complete_utm: List[List[float]] = []

    for order, raw_segment in enumerate(flight_plan or []):
        from_node, to_node = _flight_plan_segment_nodes(raw_segment)
        segment_key = "{}_{}".format(from_node, to_node)
        ordered_plan.append(
            {
                "order": order,
                "segmentKey": segment_key,
                "fromNode": from_node,
                "toNode": to_node,
            }
        )
        try:
            raw_trajectory = segment_trajectory_getter(from_node, to_node, member_id)
            if not raw_trajectory:
                missing_segments.append(
                    {"segmentKey": segment_key, "reason": "member reference trajectory is missing"}
                )
                continue
            trajectory_np = np.asarray(raw_trajectory, dtype=float)
            route_points = _route_points_from_utm(raw_trajectory, lnglat_converter, segment_key)
        except (TypeError, ValueError) as exc:
            missing_segments.append({"segmentKey": segment_key, "reason": str(exc)})
            continue

        segment_utm = trajectory_np[:, :3].tolist()
        exported_segments.append(
            {
                "order": order,
                "segmentKey": segment_key,
                "fromNode": from_node,
                "toNode": to_node,
                "routePointCount": len(route_points),
                "flightRoute": route_points,
            }
        )

        # Drop only a genuinely duplicated segment boundary.  If formation
        # changes introduce a gap, keep both points so the declared plan does
        # not silently lose the next segment's start position.
        start_index = 0
        if complete_utm and np.allclose(
            np.asarray(complete_utm[-1], dtype=float),
            np.asarray(segment_utm[0], dtype=float),
            rtol=0.0,
            atol=1e-6,
        ):
            start_index = 1
        complete_utm.extend(segment_utm[start_index:])

    complete_route = (
        _route_points_from_utm(complete_utm, lnglat_converter, "complete_flight_plan")
        if complete_utm
        else []
    )
    return {
        "source": "nodes_pair_member_traj",
        "altitudeReference": "AMSL",
        "complete": bool(ordered_plan) and not missing_segments,
        "flightPlan": ordered_plan,
        "segmentCount": len(ordered_plan),
        "routePointCount": len(complete_route),
        "flightRoute": complete_route,
        "segments": exported_segments,
        "missingSegments": missing_segments,
    }


def build_mission_meta(digraph_attrs, planned_routes, raw_trajs):
    """Rebuild cluster-level mission semantics for the latest-side generator.

    agents02 plans missions at swarm level: each digraph edge carries an
    ``order_type`` (detour/breakthrough/...), a target facility and a fleet
    number, while per-frame extras record the runtime formation
    (``cur_siblings_ids``/``leader_id``).  ``plannedRoutes`` alone keeps only
    the segment key, so this function re-attaches the design semantics:

    - ``orderType/target/fleetNo`` come from the digraph edge attributes
      (mission-design truth, never fabricated from trajectories);
    - ``memberIds`` come from grouping ``plannedRoutes[].flightPlan`` by
      ``segmentKey`` (export-time fact, more authoritative than the digraph
      ``members_num`` which historically excluded the leader);
    - ``leaderId`` comes from the runtime extras ``leader_id`` of any member
      frame, falling back to the first sorted member.

    Multi-segment missions produce one swarm entry per edge sharing the same
    ``fleetNo``; consumers may group by ``fleetNo`` when needed.  Targets whose
    segment keys are absent from the design graph are collected into a final
    fallback entry with ``orderType=None`` instead of being silently dropped.
    """
    member_by_segment = {}
    for name, route in (planned_routes or {}).items():
        for segment in (route or {}).get("flightPlan") or []:
            segment_key = segment.get("segmentKey")
            if segment_key:
                member_by_segment.setdefault(segment_key, []).append(name)

    def _leader_of(members):
        for name in sorted(members):
            _, traj_extra = (raw_trajs or {}).get(name, ([], []))
            for extra in traj_extra or []:
                if not isinstance(extra, dict):
                    continue
                leader = extra.get("leader_id")
                if isinstance(leader, str) and leader and leader != "initializing":
                    return leader
        return sorted(members)[0] if members else None

    swarms = []
    seen_members = set()
    for index, edge in enumerate(digraph_attrs or []):
        if not isinstance(edge, dict):
            continue
        attrs = edge.get("attrs") or {}
        segment_key = "{}_{}".format(edge.get("from"), edge.get("to"))
        members = sorted(set(member_by_segment.get(segment_key, [])))
        swarms.append(
            {
                "swarmId": "swarm%d" % (index + 1),
                "fleetNo": attrs.get("fleet_no"),
                "orderType": attrs.get("order_type"),
                "target": attrs.get("target"),
                "segmentKey": segment_key,
                "leaderId": _leader_of(members),
                "memberIds": members,
            }
        )
        seen_members.update(members)

    unassigned = sorted(set((planned_routes or {}).keys()) - seen_members)
    if unassigned:
        swarms.append(
            {
                "swarmId": "swarm%d" % (len(swarms) + 1),
                "fleetNo": None,
                "orderType": None,
                "target": None,
                "segmentKey": None,
                "leaderId": _leader_of(unassigned),
                "memberIds": unassigned,
                "note": "targets without a matching digraph edge",
            }
        )
    return {"source": "digraph_attrs", "swarms": swarms}


init_loc1 = [
    122.18105710089186,
    37.51299467977935,
    200.0
]

init_loc2 = [
    122.16096051695042,
    37.497235513573486,
    200.0
]

# 绍兴空域 (switch_config == 6) 专用起始区：位于三个设施西偏南约 2~4km
# 设施中心约 (116.3830°E, 39.9013°N)，两点为直径的圆即随机起飞区（跨度约 2km）
init_loc3 = [
    116.3480,
    39.8720,
    200.0
]

init_loc4 = [
    116.3680,
    39.8840,
    200.0
]
init_locs = [
    # [122.06711375, 37.57744204,200],
    # [122.11945753, 37.57340029,180],
    # [122.12628947, 37.52707223,190],
    # [122.07039604, 37.52213903,220]
    [
        122.10258217246229,
        37.56342057758475,
        200.0
    ],
    [
        122.09686551225597,
        37.56536338371063,
        200.0
    ]
]

height_range_value_set = {
    'breakthrough': [[250, 400], [0, 100]],
    'escape': [[0, 100], [250, 400]],
    'detour': [[0, 100], [200, 400]],
    'orbit': [[150, 300], [150, 300]],     # 侦察盘旋：中低空保持
    'loiter': [[200, 350], [200, 350]],    # 待命悬停
    'routine': [[200, 300], [200, 300]]    # 常规机动：平飞保持
}
direction_range_set = {
    'breakthrough': [-20, 20],
    'escape': [-20, 20],
    'detour': [0, 360]
}

switch_config = 6

current_dir = os.path.dirname(__file__)

if switch_config == 1:
    digraph_attrs_reference_path = os.path.join(current_dir, "data", "digraph_with_attrs02.json")
    facilities_file = os.path.join(current_dir,"data" ,"facilities.json")
    key_paths = [
        [0, 1, 4, 5, 2, 14],
        [3, 4, 5, 2, 14],
        [6, 7, 8, 9, 10, 14],
        [11, 12, 13, 14]        
    ]

elif switch_config == 2:
    digraph_attrs_reference_path = os.path.join(current_dir, "data", "digraph_with_attrs.json")
    facilities_file = os.path.join(current_dir,"data" ,"test_facilities_locations.json")
    key_paths = [
        ["1_0","1_1","1_2","3_0","3_1","4_1","4_2"],
        ["2_0","2_1","2_2","3_0","3_1","5_1","5_2"],
        ["1_0","1_1","1_2","3_0","3_1","6_1","6_2"]
    ]
elif switch_config == 3:
    digraph_attrs_reference_path = os.path.join(current_dir, "data",'manual_plan_graph' ,"manual_plan_graph01.json")
    facilities_file = os.path.join(current_dir,"data" ,"facilities.json")
    key_paths = [[0, 1, 4, 5, 2, 14, 15],
                    [3, 4, 5, 2, 14, 15],
                    [6, 7, 8, 9, 10, 14, 15],
                    [11, 12, 13, 14, 15]]
elif switch_config == 4:
    digraph_attrs_reference_path = os.path.join(current_dir, "data","digraph_with_attrs03.json")
    facilities_file = os.path.join(current_dir,"data" ,"facilities.json")
    key_paths = [
                [0,1,2,3],
                [0,2,3]
            ]
elif switch_config == 5:
    # 接入 NLTaskOrchestration 导出的样本
    _export_dir = os.path.join(current_dir, "data", "nl_export", "gen_aggregate_disperse_6fd84c95")
    digraph_attrs_reference_path = os.path.join(_export_dir, "digraph_attrs.json")
    facilities_file = os.path.join(_export_dir, "facilities.json")
    key_paths = json.load(open(os.path.join(_export_dir, "key_paths.json"), "r", encoding="utf-8"))

elif switch_config == 6:
    # 绍兴空域 (id=2086742451469029376)：三个设施分别执行 侦查-侦查-突破
    # 航段图由 uav_manual_path_designer.py switch_case 3 生成（顶层 list 格式）
    # 设施与防御圈由 data/gen_facilities_shaoxing.py 生成（三级半径共用 facilityList[0] 圆心）
    digraph_attrs_reference_path = os.path.join(current_dir, "data", "manual_plan_graph", "manual_plan_graph_shaoxing_digraph_attrs.json")
    facilities_file = os.path.join(current_dir, "data", "facilities_shaoxing.json")
    key_paths = [[0, 3], [1, 4], [2, 5]]

key_path_instructions_path = os.path.join(current_dir,"data" ,"key-path-analyzer02.json")
asl_file = os.path.join(current_dir, "uav_key_path.asl")
digraph_attrs = json.load(open(digraph_attrs_reference_path, "r"))
key_path_instructions = json.load(open(key_path_instructions_path, "r"))
bdi_instructions = key_path_instructions["bdi_instructions"]


# =============================
# 1. 蓝方无人机智能体
# =============================
class BlueUAVAgent(BDIAgent):
    VERBOSE = False  # 添加日志控制开关

    def log(self, msg):
        """可控的日志输出方法"""
        if self.VERBOSE:
            print(msg)
        if not hasattr(self, "step_logs"):
            self.step_logs = []
        self.step_logs.append(msg)

    def simulation_time_fields(self, round_id: Optional[int] = None) -> Dict[str, Any]:
        """Build shared simulation-clock fields plus a diagnostic wall clock.

        ``timestamp`` remains Unix seconds for compatibility with existing
        trajectory viewers.  ``simTimeMs`` is the canonical timestamp consumed
        by exported ``ts`` arrays, while ``recordedAtMs`` records when this
        agent actually wrote the Redis entry.
        """
        if round_id is None:
            raw_round = self.io.get_world_state("sim_round")
            if raw_round is None:
                raise RuntimeError("sim_round is not initialized")
            round_id = int(raw_round)

        raw_start = self.io.get_world_state(SIM_CLOCK_START_KEY)
        raw_dt = self.io.get_world_state(SIM_CLOCK_DT_KEY)
        if raw_start is None or raw_dt is None:
            raise RuntimeError("simulation clock is not initialized")

        sim_time_ms = simulation_time_ms(int(raw_start), int(round_id), int(raw_dt))
        return {
            "timestamp": sim_time_ms / 1000.0,
            "simTimeMs": sim_time_ms,
            "recordedAtMs": int(time.time() * 1000),
            "round_id": int(round_id),
        }

    def __init__(self, jid, password, asl_file, flight_plan, siblings_ref ,orchestrator, init_pos=None, facilities=facilities_file, **kwargs):
        super().__init__(jid, password, asl_file)
        self.flight_plan = flight_plan  # 航段列表: [{'segment': (u, v), 'coords': []}, ...]
        self.siblings_ref = siblings_ref
        self.orchestrator = orchestrator
        self.path_index = 0
        self.is_finished = False
        self._need_wait_siblings = False
        self._is_alive = True
        self.is_final_task = False
        self.waiting_next_segment = True
        
        # 确定起始节点
        if self.flight_plan:
            self.current_node = self.flight_plan[0][0]
            self.next_node = self.flight_plan[0][1]
        
        # 初始化设施
        self.facilities = self._default_facilities(facilities)
        # 初始化位置
        self.position = init_pos
        # 经纬度 -> UTM
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        
        # 处理初始位置 (如果为 None, 使用第一段的起点或随机生成)
        if self.position is None:
             # 尝试使用第一段航段的第一个坐标 (如果可用)
             # 但目前 extract_uav_trajectories 设置的 'coords' 是空的 (占位符)
             # 所以我们使用传入的 init_pos 或默认的随机逻辑

             _start_pair = (init_loc3, init_loc4) if switch_config == 6 else (init_loc1, init_loc2)
             _rdm_init_pos = bfunc.generate_circle_positions_from_diameter(1, _start_pair[0], _start_pair[1])
             self.position = _rdm_init_pos[0]
             self.log(f"{self.jid} no initial position provided, generated random position: {self.position}")

        # 汇聚时sibling无人机列表
        self.merge_peers = []
        self.traj = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([self.position])).tolist()
        self.traj[0].append(self.position[2])
        self.log(f"{self.jid} initialized at position: {self.position}")

        # Redis I/O 模块
        self.io = UavRedisIO(**kwargs.get("redis_cfg", {}))
        self.self_uid = jid.split("@")[0] 
        self.io.add_uav_id(self.self_uid, blue=True)
        initial_time_fields = self.simulation_time_fields(round_id=0)
        self.io.set_pos(
            self.self_uid,
            self.traj[0][0],
            self.traj[0][1],
            self.position[2],
            ts_ms=initial_time_fields["simTimeMs"],
            recorded_at_ms=initial_time_fields["recordedAtMs"],
        )
        self.io.set_traj(self.self_uid, [[self.traj[0][0], self.traj[0][1], self.position[2]]])
        self.io.set_lookahead(self.self_uid, 0)

        self.io.set_uav_state(self.self_uid, "round_done", "-1")
        self.io.set_uav_state(self.self_uid, "current_segment_sync", "")
        self.io.set_uav_state(self.self_uid, "can_task_start", "false")

        # 同步属性
        self.current_segment_siblings = []
        self.current_segment_key = None
        self.has_synced_segment = False
        self.io.set_uav_sync_state(self.self_uid, False)
        # 阶段2.1：航段依赖闸状态
        self.current_segment_id = None      # 当前航段 segment_id
        self.current_depends_on = []        # 当前航段前置依赖（segment_id 列表）
        self._waiting_for_deps = False      # 是否在等待前置航段完成

        self.APFStep = SyncAPFStepEnhance
        self.FetchWorldState = FetchWorldState
        self.planning_lib = PlanningLib(self)
        self.cur_reference_traj = []
        self.members_cur_reference_traj = []
        self.height_range_set = height_range_value_set
        self.direction_range_set = direction_range_set

        # 初始化 BDI Beliefs
        if self.flight_plan:
             self.bdi.set_belief("cur_nodes", self.current_node, self.next_node)
        self.bdi.set_belief("if_set_ref_traj", "False")
        self.bdi.set_belief('my_id', self.self_uid)
        
        self.formation_type = "unknown" # 初始化队形类型
        self.global_step_id = 0 # 全局步数ID，不随航段清零
        self.segment_step_id = 0 # 航段内步数ID，只能由leader在确认所有人执行了一次操作后更新，每个航段开始时清零
        self.my_ack = -1 # 当前航段内自身的确认状态


        extra_info = {
            "cur_siblings_ids": 'initializing',  # 初始状态没有兄弟无人机
            "formation_type": "unknown",  # 初始状态未知编队类型
            "my_ack": f"{self.my_ack} initializing",
            "frame_id": f"{self.segment_step_id} initializing",
            'lookahead': f"0",
            "global_id": f"{self.global_step_id} initializing",
            "segment_key": 'initializing',
            "is_waiting": True,
            "waiting_reason": "initializing",
            "flight_phase": "initializing",
            'dist_to_target': 'initializing',
            'lookahead_coord': None,
            'phase_state': 'initializing',
            'leader_id': 'initializing',
            "logs": getattr(self, "step_logs", []).copy(),
            **initial_time_fields,
        }
        if hasattr(self, "step_logs"):
            self.step_logs.clear()
        # 与 set_traj 写入的初始轨迹点对齐，保证 traj 和 traj_extra 长度一致
        self.io.set_traj_extra(self.self_uid, [extra_info])
        

    def _default_facilities(self, default_json_path=None):
        if default_json_path is None:
            _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')
        else:
            _facilities_info_json = default_json_path

        with open(_facilities_info_json, 'r', encoding="utf-8") as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(
            _facilities_info['facilities_str'],
            _facilities_info['defence_rings']
        )
    
    def add_achievement_goal(self, name, *args):
        """添加一个成就目标到意图缓冲区
        """
        new_args = ()
        for x in args:
            if type(x) == str:
                new_args += (agentspeak.Literal(x),)
            else:
                new_args += (x,)
        term = agentspeak.Literal(name, tuple(new_args))
        self.bdi_intention_buffer.append((agentspeak.Trigger.addition, agentspeak.GoalType.achievement, term, agentspeak.runtime.Intention()))
 

    async def setup(self):
        # 注册周期任务
        self.add_behaviour(self.APFStep(period=DT / SIM_SPEEDUP))
        
    def add_custom_actions(self, actions):
        @actions.add(".act_digraph_path_planning", 2)
        def act_digraph_path_planning(agent, term, intention):
            """与 Orchestrator 同步, 获取角色, 生成/获取轨迹"""
            
            if self.is_finished:
                yield
                return
                
            cur_start_node = str(agentspeak.grounded(term.args[0], intention.scope))
            cur_end_node = str(agentspeak.grounded(term.args[1], intention.scope))
            
            # 为新航段重置同步标志位
            self.has_synced_segment = False
            self.io.set_uav_state(self.self_uid, "current_segment_sync", "")
            self.io.set_uav_state(self.self_uid, "can_task_start", "false")
            self.io.set_lookahead(self.self_uid, 0)
            self.current_segment_key = f"{cur_start_node}_{cur_end_node}"

            # === 阶段2.1：航段依赖闸（先侦察后突击 / condition_trigger）===
            for _da in digraph_attrs:
                if str(_da["from"]) == cur_start_node and str(_da["to"]) == cur_end_node:
                    _a = _da["attrs"]
                    self.current_segment_id = _a.get("segment_id")
                    self.current_depends_on = _a.get("depends_on") or []
                    break
            _unmet = [d for d in self.current_depends_on
                      if self.io.r.get(f"seg_done:{d}") != "1"]
            if _unmet:
                self.log(f"[{self.self_uid}] segment {self.current_segment_id} waiting on deps {_unmet}")
                self._waiting_for_deps = True
                yield
                return
            self._waiting_for_deps = False
            # === /阶段2.1 ===
            
            self.log(f"[{self.self_uid}] Planning path from node {cur_start_node} to node {cur_end_node}")
            for digraph_attr in digraph_attrs:
                if str(digraph_attr["from"]) == cur_start_node and str(digraph_attr["to"]) == cur_end_node:
                    _order_mode = digraph_attr['attrs']['order_mode']
                    _order_target = digraph_attr['attrs']['target']

                    if _order_mode == 'aggregate' and _order_target == 'aggregate_point':
                        self._merge_ready_flag = False
                        # 先清空或初始化一个临时列表
                        all_merge_peers = []
                        for k, v in self.siblings_ref.items():
                            # 找到所有以当前目标点为终点的航段（入边）
                            if k[1] == cur_end_node:
                                # 累加这些航段上的所有无人机（排除自己）
                                peers = [_uav for _uav in v['uav_ids'] if _uav != self.self_uid]
                                all_merge_peers.extend(peers)
                        
                        # 去重并固定顺序，保证日志与行为可复现
                        self.merge_peers = sorted(list(set(all_merge_peers)))
                        self.log(
                            f"[{self.self_uid}] Merge peers for aggregation "
                            f"(target={_order_target}): {self.merge_peers}"
                        )
                    else:
                        self.merge_peers = []

                    # 获取当前航段的所有成员ID
                    cur_siblings_ids = self.siblings_ref.get((cur_start_node, cur_end_node), {}).get("uav_ids", [])
                    self.current_segment_siblings = cur_siblings_ids
                    self.log(f"[{self.self_uid}] Current segment siblings: {cur_siblings_ids}")
                    # 检索当前航段的基准参考轨迹是否存在，如果不存在说明当前agent是第一个执行该航段的，需要设定参考基准轨迹，后续agent就可以直接获取
                    base_ref_traj = self.io.get_nodes_pair_base_ref_traj(cur_start_node, cur_end_node)
                    
                    if not base_ref_traj or len(base_ref_traj) == 0:    
                        self.log(f"[{self.self_uid}] No base reference traj found for segment {cur_start_node}->{cur_end_node}. Now generate it.")
                        # 1. 生成基准参考轨迹
                        base_ref_traj = self.planning_lib.execute_path_planning_from_digraph(digraph_attr, -1, -1)
                        # 存储基准参考轨迹
                        self.io.set_nodes_pair_base_ref_traj(cur_start_node, cur_end_node, base_ref_traj)
                        # 2. 生成编队配置参数
                        _member_num = digraph_attr['members_num']
                        _radius = random.randint(20,30)
                        _angle = random.randint(30,60)
                        _max_offset = random.uniform(30,50)
                        _noise_scale = random.uniform(0.00001,0.00005)
                        _angle_noise_scale = random.uniform(1.0,5.0)
                        _formation_type = random.choice(['circular', 'vertical', 'horizontal', 'vshape', 'arc'])
                        
                        # 保存 formation_type 到 Redis 以供后续加入的成员查询
                        self.io.r.set(f"formation_type:{cur_start_node}:{cur_end_node}", _formation_type)
                        
                        # 处理集群从机队形轨迹
                        fleet_formation_config = Formation_Elements(
                            member_num=_member_num+1,
                            radius=_radius,
                            angle=_angle,
                            traj=base_ref_traj,
                            max_offset=_max_offset,
                            noise_scale=_noise_scale,
                            angle_noise_scale=_angle_noise_scale,
                            formation_type=_formation_type,
                        ) 
                        
                        # 3. 生成并存储所有成员的轨迹,只执行一次，后续成员从redis获取
                        members_traj_map = FormationGenerator3D(formation_elements=fleet_formation_config).generate_members_formation_map(cur_siblings_ids)
                        
                        for m_uid, m_traj in members_traj_map.items():
                            self.io.set_nodes_pair_member_traj(cur_start_node, cur_end_node, m_uid, m_traj)
                        
                        self.log(f"[{self.self_uid}] Generated and saved trajectories for {len(members_traj_map)} members.")

                    else:
                        self.log(f"[{self.self_uid}] Found existing base reference trajectory for segment {cur_start_node} -> {cur_end_node}.")
                        # 尝试获取此航段的 formation_type
                        _ft = self.io.r.get(f"formation_type:{cur_start_node}:{cur_end_node}")
                        # 兼容 decode_responses=True 和 False 的情况
                        if isinstance(_ft, bytes):
                             _formation_type = _ft.decode('utf-8')
                        else:
                             _formation_type = _ft if _ft else "unknown"

                    self.formation_type = _formation_type
                    
                    # 4. 获取属于自己的那条轨迹
                    my_traj = []
                    my_traj = self.io.get_nodes_pair_member_traj(cur_start_node, cur_end_node, self.self_uid)
                    
                    if my_traj:
                         self.log(f"[{self.self_uid}] Successfully retrieved my formation trajectory (len={len(my_traj)}).")
                         self.cur_reference_traj = my_traj
                         
                         #  保证轨迹是连续拼接
                         if not self.traj:
                             self.traj.extend(my_traj)
                         else:
                             last_pt = np.array(self.traj[-1][:2]) # 仅 xy
                             first_pt = np.array(my_traj[0][:2])
                             dist = np.linalg.norm(last_pt - first_pt)
                             
                             self.traj.extend(my_traj[1:])
                            #  if dist < 1.0:
                            #      self.traj.extend(my_traj[1:])
                            #  else:
                            #      print(f"[{self.self_uid}] Detect gap ({dist:.2f}m) between segments. Keeping full trajectory.")
                            #      self.traj.extend(my_traj)

                         if self.cur_reference_traj:
                            self.io.set_ref_traj(self.self_uid, self.cur_reference_traj)
                            self.io.set_lookahead(self.self_uid, 0)
                            # if self.APFStep != SyncAPFStep or self.APFStep != SyncAPFStepEnhance:
                            #     self.io.set_uav_state(self.self_uid, "current_segment_sync", self.current_segment_key)
                            self.io.set_uav_state(self.self_uid, "can_task_start", "false")
                            self.bdi.set_belief("if_set_ref_traj", "false")
                            self.waiting_next_segment = False
                            self.log(f"[{self.self_uid}] Trajectory synced to Redis.")
                            self.my_ack = -1 # 重置确认状态
                            self.segment_step_id = 0 # 重置航段内步数
                            self.io.set_uav_state(self.self_uid, f"{self.current_segment_key}_segment_step_id", f"{self.segment_step_id}")
                            self.io.set_uav_state(self.self_uid, f"{self.current_segment_key}_ack", f"{self.my_ack}")
                             
                    else:
                         self.log(f"[{self.self_uid}] FAILED to retrieve formation trajectory after retries!")

            # 更新 path_index 以指向下一段航程
            current_idx = self.path_index
            if current_idx + 1 < len(self.flight_plan):
                next_idx = current_idx + 1
                next_start = self.flight_plan[next_idx][0]
                next_end = self.flight_plan[next_idx][1]
                
                # 更新 BDI 信念，以便 agent 能够触发对下一段的处理
                self.log(f"[{self.self_uid}] Segment planned. Prepared next belief: cur_nodes({next_start}, {next_end})")
                self.bdi.set_belief("cur_nodes", next_start, next_end)
                self.path_index = next_idx
            else:
                 # 已经是最后一段
                 self.log(f"[{self.self_uid}] All segments in flight plan are planned. Marking as final task.")
                 self.is_final_task = True
            
            # 将自己的状态设置为 ready，表示已经准备好执行任务（或者已经开始）
            # 注意：实际任务开始是在 APFStep 中判断 can_task_start
            # 这里先不设置 True，因为还要等 can_task_start 真正变成 True（即上一段飞完）
            # 或者我们可以认为 "Plan" 完这一步就代表我想进入下一阶段的状态了
            # 更好的做法是在 APFStep 结束上一段时设置 "ready_for_next"
            # 暂时我们只在这里处理路径规划本身的逻辑。

            yield

class RoundCoordinatorAgent(Agent):
    def __init__(self, jid, password, redis_cfg=None):
        super().__init__(jid, password)
        self.redis_cfg = redis_cfg or {}

    async def setup(self):
        self.io = UavRedisIO(**self.redis_cfg)
        self.add_behaviour(GlobalRoundCoordinator(period=max(0.01, DT / 4.0 / SIM_SPEEDUP)))

class MissionOrchestrator:
    """结合key_path_analyzer.log数据, 生成 Persistent Agents 并管理生命周期"""
    def __init__(self, json_data, key_paths, server: str, password: str, asl_file: str, BlueBDIAgentTemplate: BlueUAVAgent):
        self.server = server
        self.password = password
        self.asl_file = asl_file
        self.BlueBDIAgentTemplate = BlueBDIAgentTemplate
        
        # 1. 生成全局飞行计划
        self.uav_flight_plans, self.edge_attrs = self.extract_uav_trajectories(json_data, key_paths)
        print(f"Generated {len(self.uav_flight_plans)} flight plans.")
        print(json.dumps(self.uav_flight_plans, indent=2))
        print("Edge attributes with assigned UAV IDs:")
        print(json.dumps({f"{k[0]}->{k[1]}": v for k, v in self.edge_attrs.items()}, indent=2))
        self.active_agents: Dict[str, BlueUAVAgent] = {}
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()
        self.all_trajectories = {}
        self._plan_graph = nx.DiGraph()
        
        # 同步队列
        # Key: (from_node, to_node) -> Value: List[agent_id] waiting
        self.edge_queues = collections.defaultdict(list)
        # Key: (from_node, to_node) -> Value: Required count
        self.edge_requirements = {}
        
        # 从 json_data 预填充需求
        for item in json_data:
             u, v = str(item["from"]), str(item["to"])
             # members_num + 1 (包含主机)
             self.edge_requirements[(u, v)] = item["members_num"] + 1

    def _init_DAG_structure(self):
        for k, v in self.edge_attrs.items():
            self._plan_graph.add_edge(k[0], k[1], count=v["count"], uav_ids=v["uav_ids"])


    def extract_uav_trajectories(self, json_data, key_paths):
        # 1. 构建图结构和属性索引
        edge_attrs = {}
        graph = collections.defaultdict(list)
        # 如果 json_data 是一个列表，逐项处理，如果是一个字典，则查找是否包含了key为'digraph_attrs'的项
        if isinstance(json_data, dict) and "digraph_attrs" in json_data:
            json_data = json_data["digraph_attrs"]

        for item in json_data:
            u, v = str(item["from"]), str(item["to"]) # 确保键是字符串
            # members_num + 1 (1个主机 + N个从机)
            total_drones = item["members_num"] + 1 
            edge_attrs[(u, v)] = {
                "count": total_drones,
                "uav_ids": []
            }
            graph[u].append(v)

        # 2. 统计所有可能的路径片段并进行路径拆分
        # uav_paths 存储格式: { uav_id: [ [coord1, coord2...], [coord1... ] ] }
        uav_trajectories = []
        
        # 我们需要跟踪每一条边剩余的“可用名额”
        remaining_flow = {edge: attr["count"] for edge, attr in edge_attrs.items()}
        
        # 找到所有的起点 (这里根据 key_paths 的第一个元素确定)
        # key_paths 的项类似于 "1_0" (节点名称)
        # 我们需要起始节点。
        starts = set(str(path[0]) for path in key_paths)
        
        for start_node in starts:
            # 查找从该起点出发的总流量
            start_edges = [e for e in remaining_flow if e[0] == start_node]
            total_at_start = sum(remaining_flow[e] for e in start_edges)
            
            for i in range(total_at_start):
                current_node = start_node
                single_uav_path = []
                
                # 随机游走直到没有出边或流量耗尽
                while True:
                    possible_next = [v for v in graph[current_node] if remaining_flow.get((current_node, v), 0) > 0]
                    
                    if not possible_next:
                        break
                    
                    # 随机选择一个还有剩余流量的分支
                    next_node = random.choice(possible_next)
                    
                    # 记录该片段的轨迹
                    edge = (current_node, next_node)
                    single_uav_path.append(edge)
                    
                    # 消耗一个流量
                    remaining_flow[edge] -= 1
                    current_node = next_node
                
                if single_uav_path:
                    sorted_starts = sorted(list(starts))
                    _idx = sorted_starts.index(start_node)
                    uav_trajectories.append({
                        "id": f'agent_{_idx+1}_{i}',
                        "path": single_uav_path
                    })
        
        for _traj in uav_trajectories:
            for seg in _traj['path']:
                if (seg[0], seg[1]) in edge_attrs.keys():
                    edge_attrs[(seg[0], seg[1])]["uav_ids"].append(_traj["id"])


        return uav_trajectories, edge_attrs

    async def run(self):
        print("Mission Orchestrator Started (Persistent Mode).")
        
        for plan in self.uav_flight_plans:
            agent_id = plan['id']
            flight_plan = plan['path']
            # 开始位置? 可以在此处指定, 也可以随机
            await self._spawn_persistent_agent(agent_id, flight_plan)

        try:
            # 监控任务
            while self.active_agents:
                current_ids = list(self.active_agents.keys())
                all_done = True
                for aid in current_ids:
                    agent = self.active_agents[aid]
                    if not agent.is_finished:
                        all_done = False
                    else:
                        # 如果已完成, 进行清理 (占位)
                        pass 
                
                if all_done:
                    print("Stopping all agents...")
                    for agent in self.active_agents.values():
                        await agent.stop()
                    break
                await asyncio.sleep(1.0)
                
            print("All persistent missions completed.")
        except BaseException as e:
            print(f"Mission interrupted: {e}")
        finally:
            print("Saving trajectories before exiting...")
            self.save_trajectories()

    async def _spawn_persistent_agent(self, agent_name, flight_plan):
        jid = f"{agent_name}@{self.server}"
        print(f"\nSpawning persistent agent: {jid} with {len(flight_plan)} segments")
        agent = self.BlueBDIAgentTemplate(jid, self.password, self.asl_file, flight_plan, self.edge_attrs, self)
        await agent.start()
        self.active_agents[agent_name] = agent

    def save_trajectories(self):
        print("Collecting trajectories and facility info...")
        
        # 1. 加载设施信息
        facilities_data = {}
        if os.path.exists(facilities_file):
            with open(facilities_file, 'r', encoding='utf-8') as f:
                facilities_data = json.load(f)
        
        facilities_str = facilities_data.get('facilities_str', {})
        defence_rings = facilities_data.get('defence_rings', {})
        airspaces = facilities_data.get('airspaces', [])

        for _ring_name, _ring_llgs in defence_rings.items():
            flat = []
            for _lng, _lat in zip(_ring_llgs['lngs'], _ring_llgs['lats']):
                flat.extend([_lng, _lat])
            facilities_str[_ring_name.upper()] = flat
        
        # 2. 收集无人机轨迹
        uavs_coords = {}

        raw_trajs = {}

        for name, agent in self.active_agents.items():
            traj_utm = agent.io.get_traj(agent.self_uid)
            traj_extra = agent.io.get_traj_extra(agent.self_uid)
            raw_trajs[name] = (traj_utm, traj_extra)

        # 2.1 完整计划航迹：cur_reference_traj/ref_traj 只保存当前航段，
        # 因此按每架无人机的 flight_plan 顺序读取分航段成员参考轨迹并拼接。
        # 这里只增加导出数据，不修改 Agent 执行状态、lookahead 或 Redis 内容。
        planned_routes = {}
        for name, agent in self.active_agents.items():
            try:
                planned_routes[name] = build_complete_flight_plan_export(
                    agent.flight_plan,
                    agent.self_uid,
                    agent.io.get_nodes_pair_member_traj,
                    self._lnglat2utm_convertor,
                )
            except Exception as exc:
                planned_routes[name] = {
                    "source": "nodes_pair_member_traj",
                    "altitudeReference": "AMSL",
                    "complete": False,
                    "flightPlan": [],
                    "segmentCount": len(agent.flight_plan or []),
                    "routePointCount": 0,
                    "flightRoute": [],
                    "segments": [],
                    "missingSegments": [
                        {"segmentKey": None, "reason": "export failed: {}".format(exc)}
                    ],
                }
            if not planned_routes[name]["complete"]:
                print(
                    "[save_trajectories] Warning: incomplete planned route for {}: {}".format(
                        name,
                        planned_routes[name]["missingSegments"],
                    )
                )

        # 2.2 集群级任务语义：从任务设计图边属性和运行期编队事实重建，
        # 供 latest 侧生成器做语义一致的合规数据分配（只导出，不改仿真逻辑）。
        mission_meta = build_mission_meta(digraph_attrs, planned_routes, raw_trajs)

        segment_common_frames = build_segment_common_frames(raw_trajs)
        print(
            "segment_common_frames: "
            + json.dumps(
                {key: sorted(value) for key, value in segment_common_frames.items()},
                indent=2,
            )
        )

        for name, agent in self.active_agents.items():
            traj_utm, traj_extra = raw_trajs.get(name, ([], []))
            if traj_utm:
                traj_utm, traj_extra = select_analysis_trajectory(
                    traj_utm,
                    traj_extra or [],
                    segment_common_frames,
                )
                if not traj_utm:
                    print(f"[save_trajectories] Warning: no analysis-ready synced frames for {name}")
                    continue

                traj_np = np.array(traj_utm)
                if traj_np.shape[0] > 0:
                    # Convert to Lat/Lon (utm_to_lng_lat_array 只转换前两列，高度需单独取)
                    ll = self._lnglat2utm_convertor.utm_to_lng_lat_array(traj_np)
                    lats = ll[:, 1].tolist()
                    lngs = ll[:, 0].tolist()
                    alts = traj_np[:, 2].tolist()  # 高度列（米）

                    # 额外信息 (现在通过 frame_id/segment_key 同步)
                    aligned_extras = traj_extra

                    # 3. 使用全局 round 生成的统一仿真时钟（Unix 毫秒）
                    ts = trajectory_timestamps_ms(aligned_extras)

                    uavs_coords[name] = {
                        "lats": lats,
                        "lngs": lngs,
                        "alts": alts,
                        "ts": ts,
                        "extras": aligned_extras
                    }


        # 3. 采集原始轨迹（不做同步过滤）
        uavs_coords_raw = {}
        for name, (traj_utm, traj_extra) in raw_trajs.items():
            if traj_utm:
                traj_np = np.array(traj_utm)
                if traj_np.shape[0] > 0:
                    ll = self._lnglat2utm_convertor.utm_to_lng_lat_array(traj_np)
                    lats = ll[:, 1].tolist()
                    lngs = ll[:, 0].tolist()
                    alts = traj_np[:, 2].tolist()  # 高度列（米）
                    ts = trajectory_timestamps_ms(traj_extra)
                    uavs_coords_raw[name] = {
                        "lats": lats,
                        "lngs": lngs,
                        "alts": alts,
                        "ts": ts,
                        "extras": traj_extra
                    }

        # 4. 构建最终数据结构
        simulation_meta = {
            "startTimeMs": None,
            "dtMs": int(round(DT * 1000)),
            "timeBasis": "SIMULATION_ROUND",
            "kinematics": {
                "maxHorizontalSpeedMps": MAX_HORIZONTAL_SPEED_MPS,
                "maxClimbRateMps": MAX_CLIMB_RATE_MPS,
                "maxDescentRateMps": MAX_DESCENT_RATE_MPS,
            },
        }
        if self.active_agents:
            sample_io = next(iter(self.active_agents.values())).io
            raw_start = sample_io.get_world_state(SIM_CLOCK_START_KEY)
            raw_dt = sample_io.get_world_state(SIM_CLOCK_DT_KEY)
            if raw_start is not None:
                simulation_meta["startTimeMs"] = int(raw_start)
            if raw_dt is not None:
                simulation_meta["dtMs"] = int(raw_dt)

        final_data = {
            "simulationMeta": simulation_meta,
            "uavs_coords_str": uavs_coords,
            "uavs_coords_raw": uavs_coords_raw,
            "plannedRoutes": planned_routes,
            "facilities_str": facilities_str,
            "defence_rings": defence_rings,
            "airspaces": airspaces,
            "missionMeta": mission_meta
        }
        
        # 5. 保存到文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(current_dir, "data", "raw_data")
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f"uav_trajectories_persistent_{timestamp}.json")
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(final_data, f, indent=4)
            
        print(f"All data saved to {output_file}")

async def start_agent(server, password):
    try:
        r_conn = redis.Redis(host='127.0.0.1', port=6379, db=0)
        r_conn.flushdb()
        print("[System] uav_dynamic_agents02 with Redis database flushed successfully.")
    except Exception as e:
        print(f"[System] Warning: Failed to flush Redis: {e}")

    # 初始化全局 round 和统一仿真时钟。所有 Agent 都只读取这组世界状态，
    # 不再以各自写 Redis 的墙钟作为轨迹时间。
    io = UavRedisIO()
    io.set_world_state("sim_round", 0)
    simulation_start_time_ms = int(time.time() * 1000)
    simulation_dt_ms = int(round(DT * 1000))
    io.set_world_state(SIM_CLOCK_START_KEY, simulation_start_time_ms)
    io.set_world_state(SIM_CLOCK_DT_KEY, simulation_dt_ms)
    print(
        "[System] Simulation clock initialized: "
        f"startTimeMs={simulation_start_time_ms}, dtMs={simulation_dt_ms}"
    )

    # 启动 round coordinator
    coordinator = RoundCoordinatorAgent(f"round_coordinator@{server}", password)
    await coordinator.start()

    try:
        orchestrator = MissionOrchestrator(
            json_data=digraph_attrs,
            key_paths=key_paths,
            server=server,
            password=password,
            asl_file=asl_file,
            BlueBDIAgentTemplate=BlueUAVAgent
        )
        await orchestrator.run()
    finally:
        await coordinator.stop()



# async def start_agent(server, password):
#     # 清空 Redis 数据库，防止历史数据干扰
#     try:
#         # 假设 Redis 运行在本地默认端口
#         r_conn = redis.Redis(host='127.0.0.1', port=6379, db=0)
#         r_conn.flushdb()
#         print("[System] uav_dynamic_agents02 with Redis database flushed successfully.")
#     except Exception as e:
#         print(f"[System] Warning: Failed to flush Redis: {e}")


#     # bdi_instructions = KeyPathAnalyzer(key_paths).generate_bdi_instructions()
#     # print(f"BDI instructions: {json.dumps(bdi_instructions, indent=2)}")
#     orchestrator = MissionOrchestrator(
#         json_data=digraph_attrs,
#         key_paths=key_paths,
#         server=server,
#         password=password,
#         asl_file=asl_file,
#         BlueBDIAgentTemplate=BlueUAVAgent
#     )
    
#     await orchestrator.run()

if __name__ == "__main__":
    # 启动代码：python -m examples.uavs_strategy.uav_dynamic_agents02
    server = "127.0.0.1"
    passwd = "202127"
    spade.run(start_agent(server, passwd))
