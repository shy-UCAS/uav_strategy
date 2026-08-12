# -*- coding: utf-8 -*-

from math import dist
from turtle import st
import numpy as np
from time import time, sleep
from typing import TYPE_CHECKING
from pyparsing import C
from ray import state
# from regex import F
from spade.behaviour import PeriodicBehaviour
from sympy import true
from examples.uavs_strategy.planning_modules import avoidance_agents as a_agents

if TYPE_CHECKING:
    from examples.uavs_strategy.uav_dynamic_agents02 import BlueUAVAgent
    from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO

# ==== 势场与步进参数 ====
DT = 0.5       # 周期（秒）
STEP = 8.0     # 每步最大水平位移（米/步）
MAX_HORIZONTAL_SPEED_MPS = STEP / DT
MAX_CLIMB_RATE_MPS = 5.0
MAX_DESCENT_RATE_MPS = 5.0
K_ATT = 0.85   # 引力系数 (独立飞行时)
K_ATT_FORM = 1.5 # 引力系数 (编队飞行时，需要更强的跟随力)
K_REP = 2.5    # 斥力系数
CLOSE_TH = 3.0 # 到达判定阈值
CLOSE_TH_SYNC = 3.0 # 起点同步阈值

# ==== 简单向量函数 (从原文件移动过来) ====
def v_sub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def v_add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def v_scale(a, s): return [a[0] * s, a[1] * s, a[2] * s]
def v_norm(a):
    n = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
    return n if n > 1e-9 else 1e-9
def v_unit(a):
    n = v_norm(a)
    return [a[0] / n, a[1] / n, a[2] / n]

def v_sub_2d(a, b):
    return [a[0] - b[0], a[1] - b[1]]
def v_norm_2d(a):
    n = (a[0] ** 2 + a[1] ** 2) ** 0.5
    return n if n > 1e-9 else 1e-9


def bounded_motion_step(
    current,
    target,
    dt=DT,
    max_horizontal_speed_mps=MAX_HORIZONTAL_SPEED_MPS,
    max_climb_rate_mps=MAX_CLIMB_RATE_MPS,
    max_descent_rate_mps=MAX_DESCENT_RATE_MPS,
):
    """Move toward ``target`` without exceeding per-round kinematic limits.

    Horizontal motion and vertical motion are limited independently.  The
    function never overshoots the target on either axis, so it is deterministic
    and safe to use with the global simulation-round clock.
    """
    if dt <= 0:
        raise ValueError("dt must be positive")
    if min(max_horizontal_speed_mps, max_climb_rate_mps, max_descent_rate_mps) < 0:
        raise ValueError("speed and climb/descent limits must be non-negative")

    dx = float(target[0]) - float(current[0])
    dy = float(target[1]) - float(current[1])
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    max_horizontal_step = float(max_horizontal_speed_mps) * float(dt)
    if horizontal_distance <= max_horizontal_step or horizontal_distance <= 1e-9:
        next_x = float(target[0])
        next_y = float(target[1])
    else:
        scale = max_horizontal_step / horizontal_distance
        next_x = float(current[0]) + dx * scale
        next_y = float(current[1]) + dy * scale

    dz = float(target[2]) - float(current[2])
    max_up_step = float(max_climb_rate_mps) * float(dt)
    max_down_step = float(max_descent_rate_mps) * float(dt)
    bounded_dz = max(-max_down_step, min(max_up_step, dz))
    next_z = float(current[2]) + bounded_dz

    return [next_x, next_y, next_z]


class FormationAPFStep(PeriodicBehaviour):
    """
    集群编队飞行控制 (Master-Slave 模式)
    - 主机 (BlueUAVAgent) 负责计算自己和所有从机 (Sub-Agents) 的位置更新
    - 统一使用主机的 lookahead 索引来同步推进
    - 区分 队内斥力 (弱) 和 队间斥力 (强)
    """
    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io
        
        # 1. 轨迹同步 (如果 BDI 刚刚生成了新轨迹)
        if_set_ref = agent.bdi.get_belief("if_set_ref_traj")
        if if_set_ref:
            val_str = if_set_ref[len("if_set_ref_traj("):-1]
            if val_str == "true" and agent.cur_reference_traj:
                io.set_ref_traj(agent.self_uid, agent.cur_reference_traj)
                if agent.members_cur_reference_traj:
                    io.set_members_ref_traj(agent.self_uid, agent.members_cur_reference_traj)
                agent.bdi.set_belief("if_set_ref_traj", "false")
                print(f"[{agent.self_uid}] Cluster trajectories synced.")

        # 2. 准备数据
        master_traj = agent.cur_reference_traj
        if not master_traj: return

        slave_trajs = agent.members_cur_reference_traj or []
        
        # 定义集群成员 ID
        cluster_ids = [agent.self_uid] + [f"{agent.self_uid}_sub_{i}" for i in range(len(slave_trajs))]
        
        # 获取环境信息
        # 使用 agent.blue_ids (由 FetchWorldState 更新) 或直接读 Redis
        # 为了确保获取到从机位置，最好重新 scan 或读 ids
        all_ids = io.get_ids(blue=True)
        all_pos_map = io.mget_pos(all_ids, blue=True)
        
        # 获取进度
        lookahead = io.get_lookahead(agent.self_uid) or 0
        max_idx = len(master_traj) - 1
        lookahead = max(0, min(lookahead, max_idx))

        # 3. 物理计算循环
        next_positions = {}
        master_reached = False
        
        # 斥力参数
        K_REP_INTRA = 0.000008   # 队内弱斥力
        K_REP_INTER = 0.00001   # 队间强斥力
        DIST_SAFE_INTRA = 8.0
        DIST_SAFE_INTER = 20.0

        for i, uid in enumerate(cluster_ids):
            # --- A. 当前位置 ---
            pos_data = all_pos_map.get(uid)
            if pos_data:
                curr = [pos_data['x'], pos_data['y'], pos_data['z']]
            else:
                # 初始化位置 fallback
                if uid == agent.self_uid: curr = master_traj[0]
                elif (i-1) < len(slave_trajs): curr = slave_trajs[i-1][0]
                else: continue

            # --- B. 目标引力 ---
            if uid == agent.self_uid:
                target = master_traj[lookahead]
            else:
                # 从机使用对应的轨迹，但索引与主机同步
                s_traj = slave_trajs[i-1]
                s_idx = min(lookahead, len(s_traj)-1)
                target = s_traj[s_idx]
            
            F_att = v_scale(v_sub(target, curr), K_ATT)

            # --- C. 群体斥力 ---
            F_rep = [0.0, 0.0, 0.0]
            for other_uid, other_data in all_pos_map.items():
                if other_uid == uid: continue
                if not other_data: continue
                
                other_pos = [other_data['x'], other_data['y'], other_data['z']]
                dist = v_norm(v_sub(curr, other_pos))
                
                # 区分队内/队间
                is_teammate = (other_uid in cluster_ids)
                
                k_r = K_REP_INTRA if is_teammate else K_REP_INTER
                d_safe = DIST_SAFE_INTRA if is_teammate else DIST_SAFE_INTER
                
                if dist < d_safe:
                    # 简易斥力模型
                    rep_mag = k_r * (1.0/dist - 1.0/d_safe) * 100.0
                    rep_dir = v_unit(v_sub(curr, other_pos))
                    F_rep = v_add(F_rep, v_scale(rep_dir, rep_mag))

            # --- D. 合成与移动 ---
            F_total = v_add(F_att, F_rep)
            # if v_norm(F_total) > STEP:
            #     F_total = v_scale(v_unit(F_total), STEP)
            
            nxt = v_add(curr, F_total)
            next_positions[uid] = nxt
            
            # 判定主机是否到达
            if uid == agent.self_uid:
                if v_norm(v_sub(target, nxt)) <= CLOSE_TH:
                    master_reached = True

        # 4. 状态更新
        # 推进 Lookahead
        if master_reached and lookahead < max_idx:
            io.set_lookahead(agent.self_uid, lookahead + 1)
            
        # 检查终点
        dist_end = io.get_dist_2d(agent.self_uid)
        if dist_end is not None and dist_end <= 50:
            io.set_lookahead(agent.self_uid, 0)
            agent.bdi.set_belief("can_task_start", True)
            if agent.is_final_task:
                agent.is_finished = True
            else:
                if hasattr(agent, "add_achievement_goal"):
                    agent.add_achievement_goal("task_digraph")
            print(f"[{agent.self_uid}] Segment completed.")
            return

        # 写入 Redis
        for uid, pos in next_positions.items():
            io.set_pos(uid, pos[0], pos[1], pos[2])
            io.append_traj_points(uid, pos)

# 单机APF步进控制行为
class APFStep(PeriodicBehaviour):
    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io
        current_time = time()  # 获取当前时间

        # 获取本机数据
        me = io.get_pos(agent.self_uid, blue=True)
        if not me:
            return  # 如果没有当前无人机位置，跳过

        # 获取当前的参考轨迹 (_cur_reference_traj)
        traj = agent.cur_reference_traj
        members_traj = agent.members_cur_reference_traj
        if not traj:
            return  # 如果没有主机参考轨迹，跳过（没有主机则从机也不会生成）
        if_set_ref_traj_belief = agent.bdi.get_belief("if_set_ref_traj")[len("if_set_ref_traj("):-1]

        if if_set_ref_traj_belief == "true":
            # 如果当前轨迹没有写入redis，则写入
            io.set_ref_traj(agent.self_uid, traj)
            io.set_members_ref_traj(agent.self_uid, members_traj)
            agent.bdi.set_belief("if_set_ref_traj", "false")
            print(f"[{agent.self_uid}] ref_traj and members ref_traj add to redis")

        # 获取当前预瞄点的位置（从参考轨迹中）
        lookahead = io.get_lookahead(agent.self_uid)
        lookahead = max(1, min(lookahead, len(traj) - 1))
        goal = traj[lookahead]  # 当前目标点（预瞄点）

        # 查询蓝红方 ID
        agent.blue_ids = io.get_ids(blue=True) or io.scan_ids_by_key("uav")
        agent.red_ids = io.get_ids(blue=False) or io.scan_ids_by_key("red")

        # 获取无人机的位置及速度
        all_blue_speed = io.mget_speed_from_traj(agent.blue_ids, blue=True, dt=DT)
        all_blue_pos = io.mget_pos(agent.blue_ids, blue=True)

        self_pos = [me["x"], me["y"], me["z"]]
        self_vel = all_blue_speed.get(agent.self_uid, [0.0, 0.0, 0.0])

        # 结束当前task的标志位
        dist_to_end = io.get_dist_2d(agent.self_uid)
        print(f"[{agent.self_uid}] current_dist_to_end: {dist_to_end}")
        if dist_to_end is not None and dist_to_end <= 50:
            # 重置当任务状态
            io.set_lookahead(agent.self_uid, 0)
            agent.bdi.set_belief("can_task_start", True)
            if agent.is_final_task:
                agent.is_finished = True
            else:
                if hasattr(agent, "add_achievement_goal"):
                    agent.add_achievement_goal("task_digraph")
                else:
                    print(f"[{agent.self_uid}] Warning: add_achievement_goal method not found.")
            print(f"[{agent.self_uid}] near end")

            return

        obstacle_positions = []
        obstacle_vels = []

        for uid, pos in all_blue_pos.items():
            if uid == agent.self_uid:
                continue  # 排除自己

            # 位置：[x, y, z]
            obstacle_positions.append([pos["x"], pos["y"], pos["z"]])

            # 速度：从 all_blue_speed 里取，如果还没算出就给个 0
            vel = all_blue_speed.get(uid, [0.0, 0.0, 0.0])
            obstacle_vels.append(vel)

        # # ---- 斥力：来自其他无人机的位置 ----
        # 如果当前只有一架机，障碍数组为空，就没必要算
        if not obstacle_positions:
            F_rep = np.zeros(3)
        else:
            F_rep = a_agents.compute_dynamic_repulsive_force(
                agent_pos=self_pos,
                agent_vel=self_vel,
                obstacle_positions=obstacle_positions,
                obstacle_vels=obstacle_vels,
            )
        print(f"[{agent.self_uid}] obs_pos: {obstacle_positions}, obs_vel: {obstacle_vels}, F_rep: {F_rep}")

        # ---- 引力：朝向目标（预瞄点） ----
        F_att = v_scale(v_sub(goal, self_pos), K_ATT)
        print(f"[{agent.self_uid}] F_att: {F_att}")
        # ---- 合力：引力和斥力合成 ----
        F = v_add(F_att, F_rep)

        # # 步进：每 period 向目标迈进 STEP 米
        # step_vec = v_scale(v_unit(F), STEP)  # 每步最大位移
        nxt = [self_pos[0] + F[0], self_pos[1] + F[1], self_pos[2] + F[2]]
        # nxt = [self_pos[0] + F_att[0], self_pos[1] + F_att[1], self_pos[2] + F_att[2]]

        # ---- 如果到达预瞄点，推进到下一个点 ----
        if v_norm(v_sub(goal, nxt)) <= CLOSE_TH and lookahead < len(traj) - 1:
            io.set_lookahead(agent.self_uid, lookahead + 1)

        # ---- 回写新的无人机位置到 Redis ----
        io.set_pos(agent.self_uid, nxt[0], nxt[1], nxt[2])
        io.append_traj_points(agent.self_uid, [nxt[0], nxt[1], nxt[2]])

        print(f"[{agent.self_uid}] New position for {agent.self_uid}: {nxt}, target_loc: {traj[-1]}")

# 世界状态查询行为
class FetchWorldState(PeriodicBehaviour):
    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io

        # 查询蓝红方 ID
        agent.blue_ids = io.get_ids(blue=True) or io.scan_ids_by_key("uav")
        agent.red_ids = io.get_ids(blue=False) or io.scan_ids_by_key("red")

        # 批量读取蓝方和红方的位置信息
        blue_all = io.mget_pos(agent.blue_ids, blue=True)
        red_all = io.mget_pos(agent.red_ids, blue=False)

        # 写入到 agent.world 中
        agent.world["blue_pos"] = {k: v for k, v in blue_all.items() if v}
        agent.world["red_pos"] = {k: v for k, v in red_all.items() if v}

class SyncAPFStep(PeriodicBehaviour):
    VERBOSE = False  # Set False to disable print logs

    def log(self, *args):
        from datetime import datetime
        ms_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{ms_time}] " + " ".join(map(str, args))
        if self.VERBOSE:
            print(msg)
        if hasattr(self, "agent"):
            if not hasattr(self.agent, "step_logs"):
                self.agent.step_logs = []
            self.agent.step_logs.append(msg)

    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io
        # 向redis写入参考轨迹
        # self._sync_trajectory(agent, io)

        # 获取状态
        state = self._get_agent_state(agent, io)
        if not state:
            self.log(f"[{agent.self_uid}] Cannot get agent state, skipping step.")
            return
        me, traj, lookahead, max_idx, target, self_pos, _ = state

        # 检查同步检查点，返回True表示需要等待
        if self._sync_state_checkpoint(agent, io):
            self.log(f"[{agent.self_uid}] Waiting at sync checkpoint.")
            self._record_wait_state(agent, io, target=target)
            return

        if not state:
            raise ValueError("Cannot get agent state for checkpoint sync.")
        me, traj, lookahead, max_idx, target, self_pos, dist2target = state
        # 物理计算
        nxt = self._calculate_physics(agent, io, target, self_pos)
        # 计算移动后的真实误差 (决定是否到达/同步)
        dist_after_move = v_norm(v_sub(target, nxt))

        # 状态更新写入 Redis, 包括Lookahead推进 logic
        self._update_status_and_redis(agent, io, nxt, lookahead, max_idx, dist_after_move, target)
        
        # 检查当前任务是否结束
        # self._check_task_completion(agent, io, lookahead, max_idx, dist_after_move)



    def _record_wait_state(self, agent: "BlueUAVAgent", io: "UavRedisIO", pos=None, target=None):
        """辅助函数：记录当前位置为等待状态（悬停）"""
        if pos is None:
             me = io.get_pos(agent.self_uid, blue=True)
             if me:
                 pos = [me['x'], me['y'], me['z']]
             else:
                 return

        f_type = getattr(agent, "formation_type", "unknown")
        s_ids = getattr(agent, "current_segment_siblings", [])
        
        lookahead = io.get_lookahead(agent.self_uid) or 0
        is_gathering = (lookahead == 0 and not getattr(agent, "has_synced_segment", False))
        
        if is_gathering:
             f_type = "is_gathering"
             s_ids = []

        # 等待状态不应参与帧级同步
        leader_id = self._get_leader_id(agent)
        frame_id = None
        segment_key = getattr(agent, "current_segment_key", None)
        
        # global_step_id 自增
        agent.global_step_id += 1

        current_logs = getattr(agent, "step_logs", []).copy()
        if hasattr(agent, "step_logs"):
            agent.step_logs.clear()

        extra_info = {
            "cur_siblings_ids": s_ids,
            "formation_type": f_type,
            "frame_id": frame_id,
            "lookahead": lookahead,
            "global_id": agent.global_step_id,
            "segment_key": segment_key,
            "is_waiting": True,
            "waiting_reason": "segment_start_barrier",
            "flight_phase": "sync_wait",
            "leader_id": leader_id,
            'lookahead_coord': target if target else None,
            "logs": current_logs,
            **agent.simulation_time_fields(),
        }

        # 使用组合原子更新
        io.append_pos_traj_with_extra(agent.self_uid, pos, extra_info, blue=True)

    def _sync_state_checkpoint(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        """ 起始同步状态检查,False表示可以继续运行,True表示需要等待 """
        my_can_start = agent.bdi.get_belief("can_task_start", "false")
        state_val = "true" if "true" in str(my_can_start) else "false"
        io.set_uav_state(agent.self_uid, "can_task_start", state_val)

        _self_lookahead = io.get_lookahead(agent.self_uid)
        if _self_lookahead == 0:
            # 还未开始，判断当前位置是否在可以开始航迹的位置
            state = self._get_agent_state(agent, io)
            if not state:
                raise ValueError("Cannot get agent state for checkpoint sync.")
            me, traj, lookahead, max_idx, target, self_pos, dist2target = state
            if dist2target <= CLOSE_TH_SYNC:
                agent.has_synced_segment = True
                target_sync_key = getattr(agent, "current_segment_key", "unknown")
                io.set_uav_state(agent.self_uid, "current_segment_sync", target_sync_key)
                
                # 记录自己到达同步点的时间戳（仅首次写入）
                # arrival_key = f"segment_arrival:{target_sync_key}"
                # if not getattr(agent, "_arrival_ts_recorded", False) or \
                #    getattr(agent, "_arrival_ts_segment", None) != target_sync_key:
                #     arrival_ts = time()
                #     io.r.hset(arrival_key, agent.self_uid, arrival_ts)
                #     agent._arrival_ts_recorded = True
                #     agent._arrival_ts_segment = target_sync_key

                if agent.current_segment_siblings:
                    _leader_id = self._get_leader_id(agent)
                    self.log(f"[{agent.self_uid}] Checking sync with siblings: {agent.current_segment_siblings}")
                    peer_states = io.mget_uav_states(agent.current_segment_siblings, "current_segment_sync")
                    all_synced = True
                    for pid, seg_key in peer_states.items():
                        if seg_key != target_sync_key:
                            all_synced = False
                            self.log(f"[{agent.self_uid}] Peer {pid} at segment {seg_key} not synced, waiting for {target_sync_key}.")
                            break
                    if all_synced:
                        self.log(f"[{agent.self_uid}] All siblings are SYNCED for segment start.")
                        for pid in agent.current_segment_siblings:
                            io.set_lookahead(pid, 1)
                        return True     
                    else:
                        self.log(f"[{agent.self_uid}] Waiting for peers to sync for segment start.")
                        return True
            else:
                # 推进自身位置去往航迹段的start位置附近,利用has_synced_segment为false不推进lookahead
                return False   
        else:
            # 已经开始飞行，不需要等待
            self.log(f"[{agent.self_uid}] Already started flight, no checkpoint wait needed.")
            return False   
                      


    def _sync_bdi_state(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        """0. BDI 状态同步: 检查任务完成状态及汇聚等待"""
        my_can_start = agent.bdi.get_belief("can_task_start", "false")
        state_val = "true" if "true" in str(my_can_start) else "false"
        io.set_uav_state(agent.self_uid, "can_task_start", state_val)

        if agent.merge_peers and state_val == "true":
            states = io.mget_uav_states(agent.merge_peers, "can_task_start")
            all_ready = True
            for pid, status in states.items():
                if status != "true":
                    all_ready = False
                    break
            
            if all_ready:
                 if not hasattr(agent, "_merge_ready_flag") or not agent._merge_ready_flag:
                     self.log(f"[{agent.self_uid}] All merge peers are READY! Proceeding.")
                     agent._merge_ready_flag = True
                     # 触发下一个BDI规划
                     agent.add_achievement_goal("task_digraph")
            
            # 如果处于 merge_peers 且完成状态，无论是否 ready 都 return
            return False
            
        return True

    def _sync_trajectory(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        """1. 轨迹同步: 检查并更新新轨迹"""
        if_set_ref = agent.bdi.get_belief("if_set_ref_traj")
        if if_set_ref:
             val_str = if_set_ref[len("if_set_ref_traj("):-1]
             if val_str == "true" and agent.cur_reference_traj:
                 io.set_ref_traj(agent.self_uid, agent.cur_reference_traj)
                 io.set_lookahead(agent.self_uid, 0)  # 新轨迹，重置预瞄点
                 agent.bdi.set_belief("if_set_ref_traj", "false")
                 
                 # === 修复：重置任务完成状态，防止误判为已到达汇聚点 ===
                 io.set_uav_state(agent.self_uid, "can_task_start", "false")
                 self.log(f"[{agent.self_uid}] Trajectory synced to Redis.")
                 
                 # 收到新轨迹，不再等待
                 agent.waiting_next_segment = False

    def _get_leader_id(self, agent: "BlueUAVAgent"):
        """辅助函数：确定当前段的领队（最小ID）"""
        if hasattr(agent, "current_segment_siblings") and agent.current_segment_siblings:
            # 垂直排序以确保确定性
            return sorted(agent.current_segment_siblings)[0]
        return agent.self_uid

    def _get_agent_state(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        """2. 获取状态: 位置、轨迹、进度等"""
        me = io.get_pos(agent.self_uid, blue=True)
        if not me: return None

        traj = agent.cur_reference_traj
        if not traj: return None

        # --- 预瞄同步逻辑 ---
        # 识别领队
        # leader_id = self._get_leader_id(agent)
        
        # 预瞄同步：
        # 所有人（领队和跟随者）都从领队的记录中读取进度索引。
        # 这确保整个编队步伐一致。
        # target_uid_for_lookahead = leader_id 
        
        # lookahead = io.get_lookahead(target_uid_for_lookahead)
        lookahead = io.get_lookahead(agent.self_uid)
        
        # 初始化回退：如果领队尚未开始，默认为开始 (0)。
        if lookahead is None:
             self.log(f"[{agent.self_uid}] Leader lookahead not set, defaulting to 0.")
             lookahead = io.get_lookahead(agent.self_uid) or 0
             
        max_idx = len(traj) - 1
        lookahead = max(0, min(lookahead, max_idx))
        
        target = traj[lookahead]
        self_pos = [me["x"], me["y"], me["z"]]
        dist_to_target = v_norm(v_sub(target, self_pos))
        self.log(f"[{agent.self_uid}] lookahead: {lookahead}, target: {target}, dist_to_target: {dist_to_target}")
        
        return me, traj, lookahead, max_idx, target, self_pos, dist_to_target

    def _check_task_completion(self, agent: "BlueUAVAgent", io: "UavRedisIO", lookahead, max_idx, dist_to_target):
        """4. 检查任务结束"""
        if lookahead >= max_idx and dist_to_target <= CLOSE_TH_SYNC:
            # 清理状态与重置
            # io.set_uav_state(agent.self_uid, "current_segment_sync", "finished")
            # 是否所有任务彻底结束
            if agent.is_final_task:
                agent.is_finished = True
                self.log(f"[{agent.self_uid}] Final task completed.")
            else:
                # 准备下一段：重置Lookahead并等待新指令
                agent.waiting_next_segment = True
                # io.set_lookahead(agent.self_uid, 0)
                # 触发下一个BDI规划
                agent.add_achievement_goal("task_digraph")
                self.log(f"[{agent.self_uid}] Segment completed. Waiting next.")

            # 通用信号：通知ASL层任务完成（触发can_task_start -> !task_digraph）
            agent.bdi.set_belief("can_task_start", True)
            return True
        return False
    
    # def _check_task_completion(self, agent: "BlueUAVAgent", io: "UavRedisIO", lookahead, max_idx, dist_to_target):
    #     """4. 检查任务结束"""
    #     # 清理状态与重置
    #     io.set_uav_state(agent.self_uid, f"{agent.current_segment_key}_current_segment_sync", "finished")
    #     is_all_finished = True
    #     for pid in agent.current_segment_siblings:
    #         peer_sync_state = io.get_uav_state(pid, f"{agent.current_segment_key}_current_segment_sync")
    #         if peer_sync_state != "finished":
    #             is_all_finished = False
    #             self.log(f"[{agent.self_uid}] Peer {pid} not finished yet (state: {peer_sync_state}).")
    #             break
    #     if is_all_finished:
    #         agent.has_synced_segment = False
    #         io.set_uav_sync_state(agent.self_uid, agent.has_synced_segment)
    #         # 是否所有任务彻底结束
    #         if agent.is_final_task:
    #             agent.is_finished = True
    #             self.log(f"[{agent.self_uid}] Final task completed.")
    #         else:
    #             # 准备下一段：重置Lookahead并等待新指令
    #             agent.waiting_next_segment = True
    #             # io.set_lookahead(agent.self_uid, 0)
    #             # 触发下一个BDI规划
    #             agent.add_achievement_goal("task_digraph")
    #             self.log(f"[{agent.self_uid}] Segment completed. Waiting next.")

    #         # 通用信号：通知ASL层任务完成（触发can_task_start -> !task_digraph）
    #         agent.bdi.set_belief("can_task_start", True)
    #         return True
    #     else:
    #         return False


    def _calculate_physics(self, agent: "BlueUAVAgent", io: "UavRedisIO", target, self_pos):
        """5. 物理计算：按水平速度和升降率做有界步进。"""
        return bounded_motion_step(
            self_pos,
            target,
            dt=DT,
            max_horizontal_speed_mps=getattr(
                agent, "max_horizontal_speed_mps", MAX_HORIZONTAL_SPEED_MPS
            ),
            max_climb_rate_mps=getattr(
                agent, "max_climb_rate_mps", MAX_CLIMB_RATE_MPS
            ),
            max_descent_rate_mps=getattr(
                agent, "max_descent_rate_mps", MAX_DESCENT_RATE_MPS
            ),
        )

    def _update_status_and_redis(self, agent: "BlueUAVAgent", io: "UavRedisIO", nxt, lookahead, max_idx, dist_to_target,target=None):
        """6. 状态更新与 Redis 写入: 推进 lookahead 及记录轨迹"""
        # 记录额外信息
        f_type = getattr(agent, "formation_type", "unknown")
        s_ids = getattr(agent, "current_segment_siblings", [])
        
        is_gathering = False
        if lookahead == 0:
             if not getattr(agent, "has_synced_segment", False):
                 is_gathering = True
        is_waiting = False
        waiting_reason = None
        flight_phase = "task_flight"
        if is_gathering:
             f_type = "unknown"
             waiting_reason = "flying_to_segment_start"
             flight_phase = "positioning"
             
        # global_step_id 自增
        agent.global_step_id += 1
        self.log(
            f"[{agent.self_uid}] Step {agent.global_step_id}: lookahead={lookahead}, "
            f"dist_to_target={dist_to_target:.2f}, is_waiting={is_waiting}, "
            f"flight_phase={flight_phase}"
        )

        if dist_to_target <= CLOSE_TH_SYNC and lookahead < max_idx:
            if getattr(agent, "has_synced_segment", False):
                self.log(f"[{agent.self_uid}] move forward {dist_to_target} meters [DEBUG_ADVANCE] Increasing lookahead {lookahead} -> {lookahead + 1}")
                # 只有在已经同步了当前段的情况下才推进lookahead,否则说明agent还在飞往起点，不能推进lookahead
                io.set_lookahead(agent.self_uid, lookahead + 1)

        self._check_task_completion(agent, io, lookahead, max_idx, dist_to_target)

        current_logs = getattr(agent, "step_logs", []).copy()
        if hasattr(agent, "step_logs"):
            agent.step_logs.clear()

        extra_info = {
            "cur_siblings_ids": s_ids,
            "formation_type": f_type,
            "frame_id": lookahead,
            "lookahead": lookahead,
            "global_id": agent.global_step_id,
            "segment_key": getattr(agent, "current_segment_key", None),
            "is_waiting": is_waiting,
            "waiting_reason": waiting_reason,
            "flight_phase": flight_phase,
            "dist_to_target": dist_to_target,
            "leader_id": self._get_leader_id(agent),
            'lookahead_coord': target if target else None,
            'phase_state': agent.has_synced_segment,
            "logs": current_logs,
            **agent.simulation_time_fields(),
        }
        
        # [DEBUG PRINT]
        # print(f"[{agent.self_uid}] [DEBUG_WRITE] Writing Frame {lookahead}. NewPoS: {[round(x,2) for x in nxt]}. Dist(New): {dist_to_target:.2f}")

        # 使用组合原子更新
        io.append_pos_traj_with_extra(agent.self_uid, nxt, extra_info, blue=True)

class GlobalRoundCoordinator(PeriodicBehaviour):
    VERBOSE = False

    def log(self, *args):
        from datetime import datetime
        ms_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{ms_time}] " + " ".join(map(str, args))
        if self.VERBOSE:
            print(msg)

    def _ensure_round_initialized(self, io: "UavRedisIO"):
        cur = io.get_world_state("sim_round")
        if cur is None:
            io.set_world_state("sim_round", 0)
            return 0
        return int(cur)

    async def run(self):
        io = self.agent.io
        current_round = self._ensure_round_initialized(io)

        all_blue_ids = sorted(io.get_ids(blue=True))
        if not all_blue_ids:
            return

        round_done_map = io.mget_uav_states(all_blue_ids, "round_done")

        all_done = True
        for uid in all_blue_ids:
            v = round_done_map.get(uid)
            if v is None or int(v) != current_round:
                all_done = False
                break

        if all_done:
            io.set_world_state("sim_round", current_round + 1)

class SyncAPFStepEnhance(PeriodicBehaviour):
    VERBOSE = False  # Set False to disable print logs

    # =========================
    # 基础日志
    # =========================
    def log(self, *args):
        from datetime import datetime
        ms_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{ms_time}] " + " ".join(map(str, args))
        if self.VERBOSE:
            print(msg)
        if hasattr(self, "agent"):
            if not hasattr(self.agent, "step_logs"):
                self.agent.step_logs = []
            self.agent.step_logs.append(msg)

    # =========================
    # 运行主入口
    # =========================
    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io

        # 1) 确保 world round 已初始化
        current_round = self._ensure_world_round_initialized(io)

        # 2) 防止同一无人机在同一 round 内重复执行
        last_done = self._get_agent_round_done(agent, io)
        if last_done is not None and last_done == current_round:
            return

        # 2.5) 阶段2.1：航段依赖闸——在等前置航段时，满足后重新触发规划
        if getattr(agent, "_waiting_for_deps", False):
            _deps = getattr(agent, "current_depends_on", []) or []
            if all(io.r.get(f"seg_done:{d}") == "1" for d in _deps):
                agent._waiting_for_deps = False
                # 关键：复位 can_task_start，否则 ASL 走 can_task_start(false) 的 .wait 分支、不重规划
                agent.bdi.set_belief("can_task_start", True)
                agent.add_achievement_goal("task_digraph")
                self.log(f"[{agent.self_uid}] deps satisfied, re-arm planning.")
            self._mark_agent_round_done(agent, io, current_round)
            return

        # 3) 获取状态
        state = self._get_agent_state(agent, io)
        if not state:
            self.log(f"[{agent.self_uid}] Cannot get agent state, skipping round {current_round}.")
            self._mark_agent_round_done(agent, io, current_round)
            return

        me, traj, lookahead, max_idx, target, self_pos, _ = state

        # 4) 检查同步检查点，返回 True 表示需要等待
        if self._sync_state_checkpoint(agent, io, current_round):
            self.log(f"[{agent.self_uid}] Waiting at sync checkpoint. round={current_round}")
            self._record_wait_state(agent, io, target=target, current_round=current_round)
            self._mark_agent_round_done(agent, io, current_round)
            return

        # 5) 再读一次状态，因为 checkpoint 中可能把 lookahead 从 0 -> 1
        state = self._get_agent_state(agent, io)
        if not state:
            self.log(f"[{agent.self_uid}] Cannot get agent state after checkpoint, skipping round {current_round}.")
            self._mark_agent_round_done(agent, io, current_round)
            return

        me, traj, lookahead, max_idx, target, self_pos, dist2target = state

        # 6) 物理计算
        nxt = self._calculate_physics(agent, io, target, self_pos)

        # 7) 计算移动后的真实误差
        dist_after_move = v_norm(v_sub(target, nxt))

        # 8) 写回状态
        self._update_status_and_redis(
            agent=agent,
            io=io,
            nxt=nxt,
            lookahead=lookahead,
            max_idx=max_idx,
            dist_to_target=dist_after_move,
            target=target,
            current_round=current_round,
        )

        # 9) 标记本 round 已完成
        self._mark_agent_round_done(agent, io, current_round)

    # =========================
    # world round / round_done
    # =========================
    def _ensure_world_round_initialized(self, io: "UavRedisIO"):
        cur = io.get_world_state("sim_round")
        if cur is None:
            io.set_world_state("sim_round", 0)
            return 0
        return int(cur)

    def _get_current_round(self, io: "UavRedisIO"):
        cur = io.get_world_state("sim_round")
        if cur is None:
            raise ValueError("sim_round is not initialized in Redis/world state.")
        return int(cur)

    def _get_agent_round_done(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        val = self._get_single_uav_state(io, agent.self_uid, "round_done")
        if val is None or val == "":
            return None
        return int(val)

    def _mark_agent_round_done(self, agent: "BlueUAVAgent", io: "UavRedisIO", current_round: int):
        io.set_uav_state(agent.self_uid, "round_done", str(current_round))

    # =========================
    # 基础状态读写辅助
    # =========================
    def _get_single_uav_state(self, io: "UavRedisIO", uid, key):
        if hasattr(io, "get_uav_state"):
            return io.get_uav_state(uid, key)
        data = io.mget_uav_states([uid], key)
        return data.get(uid)

    def _get_segment_group_ids(self, agent: "BlueUAVAgent"):
        peers = list(getattr(agent, "current_segment_siblings", []) or [])
        return sorted(set(peers + [agent.self_uid]))

    def _get_leader_id(self, agent: "BlueUAVAgent"):
        group_ids = self._get_segment_group_ids(agent)
        return group_ids[0] if group_ids else agent.self_uid

    # =========================
    # 带 round 戳的 release
    # =========================
    def _get_segment_release_info(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        leader_id = self._get_leader_id(agent)

        release_key = self._get_single_uav_state(io, leader_id, "current_segment_release")
        release_round = self._get_single_uav_state(io, leader_id, "current_segment_release_round")

        if release_round is not None and release_round != "":
            release_round = int(release_round)
        else:
            release_round = None

        return release_key, release_round

    def _set_segment_release_info(
        self,
        agent: "BlueUAVAgent",
        io: "UavRedisIO",
        segment_key: str,
        round_id: int,
    ):
        leader_id = self._get_leader_id(agent)
        io.set_uav_state(leader_id, "current_segment_release", segment_key)
        io.set_uav_state(leader_id, "current_segment_release_round", str(round_id))

    def _mark_arrived_at_segment_start(self, agent: "BlueUAVAgent", io: "UavRedisIO", segment_key: str):
        agent.has_synced_segment = True
        io.set_uav_state(agent.self_uid, "current_segment_sync", segment_key)
        io.set_uav_state(agent.self_uid, f"{segment_key}_segment_arrive", "arrived")

    def _all_group_arrived(self, agent: "BlueUAVAgent", io: "UavRedisIO", segment_key: str):
        group_ids = self._get_segment_group_ids(agent)
        states = io.mget_uav_states(group_ids, "current_segment_sync")

        for pid in group_ids:
            if states.get(pid) != segment_key:
                self.log(
                    f"[{agent.self_uid}] Peer {pid} current_segment_sync={states.get(pid)} "
                    f"!= target segment {segment_key}"
                )
                return False
        return True

    # =========================
    # 记录等待状态
    # =========================
    def _record_wait_state(self, agent: "BlueUAVAgent", io: "UavRedisIO", pos=None, target=None, current_round=None):
        """辅助函数：记录当前位置为等待状态（悬停）"""
        if pos is None:
            me = io.get_pos(agent.self_uid, blue=True)
            if me:
                pos = [me["x"], me["y"], me["z"]]
            else:
                return

        f_type = getattr(agent, "formation_type", "unknown")
        s_ids = getattr(agent, "current_segment_siblings", [])

        lookahead = io.get_lookahead(agent.self_uid) or 0
        is_gathering = (lookahead == 0 and not getattr(agent, "has_synced_segment", False))

        if is_gathering:
            f_type = "is_gathering"
            s_ids = []

        leader_id = self._get_leader_id(agent)
        frame_id = None
        segment_key = getattr(agent, "current_segment_key", None)

        agent.global_step_id += 1

        current_logs = getattr(agent, "step_logs", []).copy()
        if hasattr(agent, "step_logs"):
            agent.step_logs.clear()

        extra_info = {
            "cur_siblings_ids": s_ids,
            "formation_type": f_type,
            "frame_id": frame_id,
            "lookahead": lookahead,
            "global_id": agent.global_step_id,
            "segment_key": segment_key,
            "is_waiting": True,
            "waiting_reason": "segment_start_barrier",
            "flight_phase": "sync_wait",
            "leader_id": leader_id,
            "lookahead_coord": target if target else None,
            "logs": current_logs,
            **agent.simulation_time_fields(round_id=current_round),
        }

        io.append_pos_traj_with_extra(agent.self_uid, pos, extra_info, blue=True)

    # =========================
    # 起始 barrier（最终版）
    # =========================
    def _sync_state_checkpoint(self, agent: "BlueUAVAgent", io: "UavRedisIO", current_round: int):
        """
        返回:
            False -> 本 round 可以继续执行
            True  -> 本 round 需要等待

        核心规则：
        1. 到达起点，只登记 arrived，不直接飞
        2. 全员 arrived 后，只写 release(segment_key, release_round=current_round)，本 round 仍等待
        3. 只有在后续 round，且满足 release_round < current_round 时，才允许 lookahead 0 -> 1
        """

        # 同步 BDI can_task_start
        my_can_start = agent.bdi.get_belief("can_task_start", "false")
        state_val = "true" if "true" in str(my_can_start) else "false"
        io.set_uav_state(agent.self_uid, "can_task_start", state_val)

        lookahead = io.get_lookahead(agent.self_uid) or 0

        # 已经开始飞，不再参与起始 barrier
        if lookahead > 0:
            self.log(f"[{agent.self_uid}] Already started flight, no checkpoint wait needed.")
            return False

        state = self._get_agent_state(agent, io)
        if not state:
            raise ValueError("Cannot get agent state for checkpoint sync.")

        me, traj, lookahead, max_idx, target, self_pos, dist2target = state
        segment_key = getattr(agent, "current_segment_key", "unknown")

        my_sync_key = self._get_single_uav_state(io, agent.self_uid, "current_segment_sync")
        release_key, release_round = self._get_segment_release_info(agent, io)

        # -------------------------------------------------------
        # A. 消费旧 release
        # -------------------------------------------------------
        # 只有 release_round < current_round 才允许启动。
        # 同 round 新写入的 release，本 round 不可消费。
        # -------------------------------------------------------
        if (
            my_sync_key == segment_key
            and release_key == segment_key
            and release_round is not None
            and release_round < current_round
        ):
            io.set_lookahead(agent.self_uid, 1)
            self.log(
                f"[{agent.self_uid}] Consume release for segment={segment_key}, "
                f"release_round={release_round}, current_round={current_round}, "
                f"lookahead 0 -> 1"
            )
            return False

        # -------------------------------------------------------
        # B. 还没到起始点：继续向起始点推进
        # -------------------------------------------------------
        # 此时不能等待 barrier，因为自己还没进入 barrier
        # -------------------------------------------------------
        if my_sync_key != segment_key and dist2target > CLOSE_TH_SYNC:
            self.log(
                f"[{agent.self_uid}] Not at segment start yet. "
                f"dist2target={dist2target:.3f} > CLOSE_TH_SYNC={CLOSE_TH_SYNC}"
            )
            return False

        # -------------------------------------------------------
        # C. 到了起始点，但还没登记：只登记，不起飞
        # -------------------------------------------------------
        if my_sync_key != segment_key and dist2target <= CLOSE_TH_SYNC:
            self._mark_arrived_at_segment_start(agent, io, segment_key)
            self.log(
                f"[{agent.self_uid}] Arrived at start and marked sync. "
                f"segment={segment_key}, round={current_round}"
            )
            return True

        # 单机段不需要 barrier，但仍然保持 round 规则一致
        group_ids = self._get_segment_group_ids(agent)
        if len(group_ids) <= 1:
            io.set_lookahead(agent.self_uid, 1)
            self.log(f"[{agent.self_uid}] Single-agent segment, lookahead 0 -> 1")
            return False

        # -------------------------------------------------------
        # D. 已经登记 arrived，检查是不是全员都 arrived
        # -------------------------------------------------------
        if not self._all_group_arrived(agent, io, segment_key):
            self.log(
                f"[{agent.self_uid}] Waiting for all peers to arrive. "
                f"group={group_ids}, segment={segment_key}"
            )
            return True

        # -------------------------------------------------------
        # E. 全员已到，但 release 不存在：本 round 只写 release，不起飞
        # -------------------------------------------------------
        if release_key != segment_key:
            self._set_segment_release_info(agent, io, segment_key, current_round)
            self.log(
                f"[{agent.self_uid}] All peers arrived. "
                f"Release latched for segment={segment_key}, release_round={current_round}. "
                f"This round still waits."
            )
            return True

        # -------------------------------------------------------
        # F. release 已存在，但如果 release_round == current_round，
        #    说明它就是本 round 刚写进去的，本 round 仍不能消费
        # -------------------------------------------------------
        self.log(
            f"[{agent.self_uid}] Release exists but not consumable yet. "
            f"segment={segment_key}, release_round={release_round}, current_round={current_round}"
        )
        return True

    # =========================
    # BDI merge 检查（保留）
    # =========================
    def _sync_bdi_state(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        my_can_start = agent.bdi.get_belief("can_task_start", "false")
        state_val = "true" if "true" in str(my_can_start) else "false"
        io.set_uav_state(agent.self_uid, "can_task_start", state_val)

        if agent.merge_peers and state_val == "true":
            states = io.mget_uav_states(agent.merge_peers, "can_task_start")
            all_ready = True
            for pid, status in states.items():
                if status != "true":
                    all_ready = False
                    break

            if all_ready:
                if not hasattr(agent, "_merge_ready_flag") or not agent._merge_ready_flag:
                    self.log(f"[{agent.self_uid}] All merge peers are READY! Proceeding.")
                    agent._merge_ready_flag = True
                    agent.add_achievement_goal("task_digraph")

            return False

        return True

    # =========================
    # 获取状态
    # =========================
    def _get_agent_state(self, agent: "BlueUAVAgent", io: "UavRedisIO"):
        me = io.get_pos(agent.self_uid, blue=True)
        if not me:
            return None

        traj = agent.cur_reference_traj
        if not traj:
            return None

        lookahead = io.get_lookahead(agent.self_uid)
        if lookahead is None:
            self.log(f"[{agent.self_uid}] lookahead not set, defaulting to 0.")
            lookahead = 0

        max_idx = len(traj) - 1
        lookahead = max(0, min(lookahead, max_idx))

        target = traj[lookahead]
        self_pos = [me["x"], me["y"], me["z"]]
        dist_to_target = v_norm(v_sub(target, self_pos))
        self.log(
            f"[{agent.self_uid}] lookahead: {lookahead}, target: {target}, dist_to_target: {dist_to_target}"
        )

        return me, traj, lookahead, max_idx, target, self_pos, dist_to_target

    # =========================
    # 任务完成检查
    # =========================
    def _check_task_completion(self, agent: "BlueUAVAgent", io: "UavRedisIO", lookahead, max_idx, dist_to_target):
        if lookahead >= max_idx and dist_to_target <= CLOSE_TH_SYNC:
            # 阶段2.1：本航段完成 → 置 seg_done 供依赖闸消费（幂等，同段多机均写 "1"）
            _seg_id = getattr(agent, "current_segment_id", None)
            if _seg_id:
                io.r.set(f"seg_done:{_seg_id}", "1")
            if agent.is_final_task:
                agent.is_finished = True
                self.log(f"[{agent.self_uid}] Final task completed.")
            else:
                agent.waiting_next_segment = True
                agent.add_achievement_goal("task_digraph")
                self.log(f"[{agent.self_uid}] Segment completed. Waiting next.")

            agent.bdi.set_belief("can_task_start", True)
            return True
        return False

    # =========================
    # 物理计算
    # =========================
    def _calculate_physics(self, agent: "BlueUAVAgent", io: "UavRedisIO", target, self_pos):
        return bounded_motion_step(
            self_pos,
            target,
            dt=DT,
            max_horizontal_speed_mps=getattr(
                agent, "max_horizontal_speed_mps", MAX_HORIZONTAL_SPEED_MPS
            ),
            max_climb_rate_mps=getattr(
                agent, "max_climb_rate_mps", MAX_CLIMB_RATE_MPS
            ),
            max_descent_rate_mps=getattr(
                agent, "max_descent_rate_mps", MAX_DESCENT_RATE_MPS
            ),
        )

    # =========================
    # 状态更新与 Redis 写入
    # =========================
    def _update_status_and_redis(
        self,
        agent: "BlueUAVAgent",
        io: "UavRedisIO",
        nxt,
        lookahead,
        max_idx,
        dist_to_target,
        target=None,
        current_round=None,
    ):
        """
        注意：
        - lookahead == 0 表示仍在“飞向段起点 / 起始 barrier”
        - 这一阶段即使已经到达，也只能登记 arrived，不能自己推进到 1
        - 只有 _sync_state_checkpoint() 消费旧 release 时，才允许 0 -> 1
        """
        f_type = getattr(agent, "formation_type", "unknown")
        s_ids = getattr(agent, "current_segment_siblings", [])

        is_gathering = False
        if lookahead == 0 and not getattr(agent, "has_synced_segment", False):
            is_gathering = True

        is_waiting = False
        waiting_reason = None
        flight_phase = "task_flight"
        if is_gathering:
            f_type = "unknown"
            waiting_reason = "flying_to_segment_start"
            flight_phase = "positioning"

        agent.global_step_id += 1
        self.log(
            f"[{agent.self_uid}] Step {agent.global_step_id}: "
            f"lookahead={lookahead}, dist_to_target={dist_to_target:.2f}, "
            f"is_waiting={is_waiting}, flight_phase={flight_phase}, "
            f"round={current_round}"
        )

        if dist_to_target <= CLOSE_TH_SYNC and lookahead <= max_idx:
            if lookahead == 0:
                # barrier 阶段：只登记 arrived，不推进 lookahead
                if not getattr(agent, "has_synced_segment", False):
                    target_sync_key = getattr(agent, "current_segment_key", "unknown")
                    self._mark_arrived_at_segment_start(agent, io, target_sync_key)
                    self.log(
                        f"[{agent.self_uid}] Reached gathering start point at segment={target_sync_key}. "
                        f"Arrival marked only, waiting for release."
                    )
            elif lookahead < max_idx:
                # 正常飞行阶段：才允许推进 lookahead
                if getattr(agent, "has_synced_segment", False):
                    self.log(
                        f"[{agent.self_uid}] move forward {dist_to_target:.2f} meters "
                        f"[DEBUG_ADVANCE] Increasing lookahead {lookahead} -> {lookahead + 1}"
                    )
                    io.set_lookahead(agent.self_uid, lookahead + 1)

        self._check_task_completion(agent, io, lookahead, max_idx, dist_to_target)

        current_logs = getattr(agent, "step_logs", []).copy()
        if hasattr(agent, "step_logs"):
            agent.step_logs.clear()

        release_key, release_round = self._get_segment_release_info(agent, io)

        extra_info = {
            "cur_siblings_ids": s_ids,
            "formation_type": f_type,
            "frame_id": lookahead,
            "lookahead": lookahead,
            "global_id": agent.global_step_id,
            "segment_key": getattr(agent, "current_segment_key", None),
            "is_waiting": is_waiting,
            "waiting_reason": waiting_reason,
            "flight_phase": flight_phase,
            "dist_to_target": dist_to_target,
            "leader_id": self._get_leader_id(agent),
            "lookahead_coord": target if target else None,
            "phase_state": {
                "has_synced_segment": getattr(agent, "has_synced_segment", False),
                "release_key": release_key,
                "release_round": release_round,
            },
            "logs": current_logs,
            **agent.simulation_time_fields(round_id=current_round),
        }

        io.append_pos_traj_with_extra(agent.self_uid, nxt, extra_info, blue=True)
