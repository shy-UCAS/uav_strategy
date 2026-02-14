# -*- coding: utf-8 -*-

from math import dist
from turtle import st
import numpy as np
from time import time, sleep
from typing import TYPE_CHECKING
from pyparsing import C
from ray import state
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.planning_modules import avoidance_agents as a_agents

if TYPE_CHECKING:
    from examples.uavs_strategy.uav_dynamic_agents02 import BlueUAVAgent
    from examples.uavs_strategy.redis_modules.uav_redis_io import UavRedisIO

# ==== 势场与步进参数 ====
DT = 3.0       # 周期（秒）
STEP = 8.0     # 每步“最大位移”/速度上限（米/步）
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

class Single_APFStep(PeriodicBehaviour):
    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io
        
        # 将自己的 BDI 状态同步到 Redis (can_task_start)
        my_can_start = agent.bdi.get_belief("can_task_start", "false") # 如果没获取到，默认为 "false"
        # 注意：get_belief 返回的可能是 Tuple 或 None，需要解析一下
        # 你的 ASL 定义: can_task_start(true). -> 所以这里获取的是 "can_task_start(true)" 这样的结构
        # 简单起见，我们根据是否包含 "true" 来判断
        state_val = "true" if "true" in str(my_can_start) else "false"
        io.set_uav_state(agent.self_uid, "can_task_start", state_val)

        if agent.merge_peers and state_val == "true":
            # print(f"[{agent.self_uid}] Checking sync with peers: {agent.merge_peers}")
            states = io.mget_uav_states(agent.merge_peers, "can_task_start")
            all_ready = True
            for pid, status in states.items():
                if status != "true":
                    all_ready = False
                    # print(f"[{agent.self_uid}] Peer {pid} is NOT ready.")
                    break
            
            if not all_ready:
                # 如果同伴没准备好，自己就先原地等待，不要执行后面的运动计算
                print(f"[{agent.self_uid}] Waiting for peers to be ready...")
                return 
            else:
                 print(f"[{agent.self_uid}] All merge peers are READY!")
                 # 只有首次满足时打印一下，避免刷屏
                 if not hasattr(agent, "_merge_ready_flag") or not agent._merge_ready_flag:
                     print(f"[{agent.self_uid}] All merge peers are READY! Proceeding.")
                     agent._merge_ready_flag = True
                     # 【补全逻辑】汇聚完成，统一触发下一段任务
                     if hasattr(agent, "add_achievement_goal"):
                        agent.add_achievement_goal("task_digraph")
                     # 重置 merge_peer 防止重复进入
                     # agent.merge_peers = []
                 return # 【关键修复】触发后依然要等待BDI更新状态，防止飞回旧轨迹起点

        # # 如果没有汇聚同伴，但状态是 "true" (单机任务完成)，也应该等待
        # if state_val == "true":
        #      return

        # 0. 轨迹同步 (与 Formation 保持一致的这种写法更干净)
        if_set_ref = agent.bdi.get_belief("if_set_ref_traj")
        if if_set_ref:
             val_str = if_set_ref[len("if_set_ref_traj("):-1]
             if val_str == "true" and agent.cur_reference_traj:
                 io.set_ref_traj(agent.self_uid, agent.cur_reference_traj)
                 agent.bdi.set_belief("if_set_ref_traj", "false")
                 print(f"[{agent.self_uid}] Trajectory synced to Redis.")

        # 1. 获取状态与预瞄
        # ----------------------------------------------------
        me = io.get_pos(agent.self_uid, blue=True)
        if not me: return

        traj = agent.cur_reference_traj
        if not traj: return

        lookahead = io.get_lookahead(agent.self_uid)
        max_idx = len(traj) - 1
        lookahead = max(0, min(lookahead, max_idx)) # 修正索引边界
        
        target = traj[lookahead]
        self_pos = [me["x"], me["y"], me["z"]]
        
        # 2. 检查任务结束 (借鉴 Formation 的逻辑，放在计算前或后都可以，放在前效率高)
        # ----------------------------------------------------
        # 优先使用 redis 里的距离计算（假设 io.get_dist_2d 已经在某处更新，或者我们自己算）
        dist_to_target = v_norm(v_sub(target, self_pos))
        
        # 如果已经到了轨迹末端，且距离很近，认为段结束
        if lookahead >= max_idx and dist_to_target <= CLOSE_TH:
            # 或者使用 io.get_dist_2d(agent.self_uid) 如果它是全局终点距离
            io.set_lookahead(agent.self_uid, 0)
            agent.bdi.set_belief("can_task_start", True)

            if agent.is_final_task:
                agent.is_finished = True
            else:
                # 触发 BDI 下一步
                if not agent.is_finished: # 防止重复触发
                    if hasattr(agent, "add_achievement_goal") and not agent.merge_peers:
                        agent.add_achievement_goal("task_digraph")
                    print(f"[{agent.self_uid}] Segment completed (reached end).")
            return

        # 3. 物理计算 (改进斥力)
        # ----------------------------------------------------
        # 获取周围环境
        agent.blue_ids = io.get_ids(blue=True)
        all_blue_pos = io.mget_pos(agent.blue_ids, blue=True)
        
        # 引力
        F_att = v_scale(v_sub(target, self_pos), K_ATT)
        
        # 斥力 (这里可以改进：区分不同类型的邻居)
        F_rep = [0.0, 0.0, 0.0]
        
        # 简单的改进版斥力计算，不依赖复杂的外部库，保持轻量
        for other_uid, other_data in all_blue_pos.items():
            if other_uid == agent.self_uid or not other_data: continue
            
            other_pos = [other_data['x'], other_data['y'], other_data['z']]
            dist = v_norm(v_sub(self_pos, other_pos))
            
            # 使用较大的安全距离，因为是独立飞行
            d_safe = 15.0  
            if dist < d_safe:
                # 越近斥力指数级增大
                rep_mag = K_REP * (1.0/dist - 1.0/d_safe) * 50.0 
                rep_dir = v_unit(v_sub(self_pos, other_pos))
                F_rep = v_add(F_rep, v_scale(rep_dir, rep_mag))

        # 合成
        # F_total = v_add(F_att, F_rep)
        F_total = F_att  # 先只用引力测试效果
        # 速度限制/归一化 (Formation 里用了 nxt = curr + Force，这里建议加一个限幅)
        # 这样能防止斥力过大时飞出地球
        # force_mag = v_norm(F_total)
        # if force_mag > STEP:
        #     F_total = v_scale(v_unit(F_total), STEP)

        nxt = v_add(self_pos, F_total)

        # 4. 状态更新写入 Redis
        # ----------------------------------------------------
        # 推进 Lookahead (阈值判定)
        if dist_to_target <= CLOSE_TH and lookahead < max_idx:
            io.set_lookahead(agent.self_uid, lookahead + 1)
            # 这种快速推进可以让飞机在直线上飞得更快，转弯时自动减速

        # 写入
        io.set_pos(agent.self_uid, nxt[0], nxt[1], nxt[2])
        io.append_traj_points(agent.self_uid, nxt)
        
        # print (可选，减少日志垃圾)
        # print(f"[{agent.self_uid}] Pos: {[round(x,1) for x in nxt]} -> Target: {lookahead}")
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

    async def run(self):
        agent: "BlueUAVAgent" = self.agent
        io = agent.io
        # 向redis写入参考轨迹
        self._sync_trajectory(agent, io)

        # 获取状态
        state = self._get_agent_state(agent, io)
        if not state:
            return
        me, traj, lookahead, max_idx, target, self_pos, _ = state

        # 检查同步检查点，返回True表示需要等待
        if self._sync_state_checkpoint(agent, io):
            print(f"[{agent.self_uid}] Waiting at sync checkpoint.")
            self._record_wait_state(agent, io)
            return

        # 等待下一段任务
        if getattr(agent, "waiting_next_segment", False):
             print(f"[{agent.self_uid}] Waiting for next segment.")
             self._record_wait_state(agent, io, self_pos)
             return

        # 物理计算
        nxt = self._calculate_physics(agent, io, target, self_pos)
        # 计算移动后的真实误差 (决定是否到达/同步)
        dist_after_move = v_norm_2d(v_sub_2d(target, nxt))

        # 状态更新写入 Redis, 包括Lookahead推进 logic
        self._update_status_and_redis(agent, io, nxt, lookahead, max_idx, dist_after_move)
        
        # 检查当前任务是否结束
        self._check_task_completion(agent, io, lookahead, max_idx, dist_after_move)





    def _record_wait_state(self, agent: "BlueUAVAgent", io: "UavRedisIO", pos=None):
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

        extra_info = {
            "cur_siblings_ids": s_ids,
            "formation_type": f_type,
            "frame_id": frame_id,
            "global_id": agent.global_step_id,
            "segment_key": segment_key,
            "is_waiting": True,
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
                if agent.current_segment_siblings:
                    _leader_id = self._get_leader_id(agent)
                    print(f"[{agent.self_uid}] Checking sync with siblings: {agent.current_segment_siblings}")
                    peer_states = io.mget_uav_states(agent.current_segment_siblings, "current_segment_sync")
                    # target_sync_key = getattr(agent, "current_segment_key", "unknown")
                    all_synced = True
                    for pid, seg_key in peer_states.items():
                        if seg_key != target_sync_key:
                            all_synced = False
                            print(f"[{agent.self_uid}] Peer {pid} at segment {seg_key} not synced, waiting for {target_sync_key}.")
                            break
                    if all_synced:
                        print(f"[{agent.self_uid}] All siblings are SYNCED for segment start.")
                        for pid in agent.current_segment_siblings:
                            io.set_lookahead(pid, 1)
                        return False # 修改为False, 允许当前帧继续运行不等待

                    else:
                        print(f"[{agent.self_uid}] Waiting for peers to sync for segment start.")
                        return True
            else:
                # 推进自身位置去往航迹段的start位置附近,利用has_synced_segment为false不推进lookahead
                return False   
        else:
            # 已经开始飞行，不需要等待
            print(f"[{agent.self_uid}] Already started flight, no checkpoint wait needed.")
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
                     print(f"[{agent.self_uid}] All merge peers are READY! Proceeding.")
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
                 print(f"[{agent.self_uid}] Trajectory synced to Redis.")
                 
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
             print(f"[{agent.self_uid}] Leader lookahead not set, defaulting to 0.")
             lookahead = io.get_lookahead(agent.self_uid) or 0
             
        max_idx = len(traj) - 1
        lookahead = max(0, min(lookahead, max_idx))
        
        target = traj[lookahead]
        self_pos = [me["x"], me["y"], me["z"]]
        dist_to_target = v_norm_2d(v_sub_2d(target, self_pos)) # 暂时只计算2D距离
        print(f"[{agent.self_uid}] lookahead: {lookahead}, target: {target}, dist_to_target: {dist_to_target}")
        
        return me, traj, lookahead, max_idx, target, self_pos, dist_to_target

    def _sync_segment_start(self, agent: "BlueUAVAgent", io: "UavRedisIO", lookahead, dist_to_target, self_pos):
        """ 3. 段起点同步：起点协同 """
        if lookahead == 0 and dist_to_target <= CLOSE_TH_SYNC and agent.current_segment_siblings:
             # 检查是否已为当前段同步
            if not getattr(agent, "has_synced_segment", False):
                # A. 标记自己正在等待当前特定的 Segment
                target_sync_key = getattr(agent, "current_segment_key", "unknown")
                io.set_uav_state(agent.self_uid, "current_segment_sync", target_sync_key)
                
                # B. 检查队友状态
                peers_to_check = [p for p in agent.current_segment_siblings if p != agent.self_uid]
                
                if peers_to_check:
                    peer_states = io.mget_uav_states(peers_to_check, "current_segment_sync")
                    all_arrived = True
                    for p, s in peer_states.items():
                        if s != target_sync_key:
                            all_arrived = False
                            break
                    
                    if not all_arrived:
                        # 还没齐，继续执行后续的物理循环，在起点附近动态盘旋
                        return False
                    else:
                        print(f"[{agent.self_uid}] All siblings synced at start of {target_sync_key}. Starting!")
                        agent.has_synced_segment = True
                        # io.set_lookahead(agent.self_uid, 1)
                else:
                    agent.has_synced_segment = True
                    # io.set_lookahead(agent.self_uid, 1)
        
        return True

    def _check_task_completion(self, agent: "BlueUAVAgent", io: "UavRedisIO", lookahead, max_idx, dist_to_target):
        """4. 检查任务结束"""
        if lookahead >= max_idx:
            # 清理状态与重置
            io.set_uav_state(agent.self_uid, "current_segment_sync", "finished")
            agent.has_synced_segment = False
            io.set_uav_sync_state(agent.self_uid, agent.has_synced_segment)
            # 是否所有任务彻底结束
            if agent.is_final_task:
                agent.is_finished = True
                print(f"[{agent.self_uid}] Final task completed.")
            else:
                # 准备下一段：重置Lookahead并等待新指令
                agent.waiting_next_segment = True
                # io.set_lookahead(agent.self_uid, 0)
                # 触发下一个BDI规划
                agent.add_achievement_goal("task_digraph")
                print(f"[{agent.self_uid}] Segment completed. Waiting next.")

            # 通用信号：通知ASL层任务完成（触发can_task_start -> !task_digraph）
            agent.bdi.set_belief("can_task_start", True)
            return True
        return False

    def _calculate_physics(self, agent: "BlueUAVAgent", io: "UavRedisIO", target, self_pos):
        """5. 物理计算: 引力 + 斥力"""
        # 优化：缓存 IDs，不要每帧 scan，因为 ID 列表变化不频繁
        if not hasattr(agent, "cached_blue_ids") or not agent.cached_blue_ids:
            agent.cached_blue_ids = io.get_ids(blue=True)
        all_blue_pos = io.mget_pos(agent.cached_blue_ids, blue=True)

        F_att = v_scale(v_sub(target, self_pos), K_ATT)

        F_total = F_att # 回到简单的物理模型，不计算斥力了，因为可能会造成死锁
        nxt = target # 直接设定为目标点（类似瞬移/强同步），确保位置严格跟随 Lookahead，消除物理滞后
        # nxt = v_add(self_pos, F_total) 
        return nxt

    def _update_status_and_redis(self, agent: "BlueUAVAgent", io: "UavRedisIO", nxt, lookahead, max_idx, dist_to_target):
        """6. 状态更新与 Redis 写入: 推进 lookahead 及记录轨迹"""
        # 记录额外信息
        f_type = getattr(agent, "formation_type", "unknown")
        s_ids = getattr(agent, "current_segment_siblings", [])
        
        is_gathering = False
        if lookahead == 0:
             if not getattr(agent, "has_synced_segment", False):
                 is_gathering = True
        
        if is_gathering:
             f_type = "unknown"
             s_ids = []
             
        # global_step_id 自增
        agent.global_step_id += 1

        extra_info = {
            "cur_siblings_ids": s_ids,
            "formation_type": f_type,
            "frame_id": lookahead,
            "global_id": agent.global_step_id,
            "segment_key": getattr(agent, "current_segment_key", None),
            "is_waiting": False,
        }
        
        # [DEBUG PRINT]
        # print(f"[{agent.self_uid}] [DEBUG_WRITE] Writing Frame {lookahead}. NewPoS: {[round(x,2) for x in nxt]}. Dist(New): {dist_to_target:.2f}")

        # 使用组合原子更新
        io.append_pos_traj_with_extra(agent.self_uid, nxt, extra_info, blue=True)

        if dist_to_target <= CLOSE_TH and lookahead <= max_idx:


            if getattr(agent, "has_synced_segment", False):
                print(f"[{agent.self_uid}] move forward {dist_to_target} meters [DEBUG_ADVANCE] Increasing lookahead {lookahead} -> {lookahead + 1}")
                # 只有在已经同步了当前段的情况下才推进lookahead,否则说明agent还在飞往起点，不能推进lookahead
                io.set_lookahead(agent.self_uid, lookahead + 1)
