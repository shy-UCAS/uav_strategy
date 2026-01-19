# -*- coding: utf-8 -*-

from turtle import st
import numpy as np
from time import time
from typing import TYPE_CHECKING
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.planning_modules import avoidance_agents as a_agents

if TYPE_CHECKING:
    from examples.uavs_strategy.uav_dynamic_agents02 import BlueUAVAgent

# ==== 势场与步进参数 ====
DT = 0.3       # 周期（秒）
STEP = 8.0     # 每步“最大位移”/速度上限（米/步）
K_ATT = 0.85   # 引力系数 (独立飞行时)
K_ATT_FORM = 1.5 # 引力系数 (编队飞行时，需要更强的跟随力)
K_REP = 2.5    # 斥力系数
CLOSE_TH = 30.0 # 到达判定阈值

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

        # 4. 状态更新与因为
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
        
        # 0. BDI 状态同步
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
            if not all_ready:
                return 
            else:
                 if not hasattr(agent, "_merge_ready_flag") or not agent._merge_ready_flag:
                     print(f"[{agent.self_uid}] All merge peers are READY! Proceeding.")
                     agent._merge_ready_flag = True
                     if hasattr(agent, "add_achievement_goal"):
                        agent.add_achievement_goal("task_digraph")
                 return 

        # 1. 轨迹同步
        if_set_ref = agent.bdi.get_belief("if_set_ref_traj")
        if if_set_ref:
             val_str = if_set_ref[len("if_set_ref_traj("):-1]
             if val_str == "true" and agent.cur_reference_traj:
                 io.set_ref_traj(agent.self_uid, agent.cur_reference_traj)
                 agent.bdi.set_belief("if_set_ref_traj", "false")
                 print(f"[{agent.self_uid}] Trajectory synced to Redis.")

        # 2. 获取状态
        me = io.get_pos(agent.self_uid, blue=True)
        if not me: return

        traj = agent.cur_reference_traj
        if not traj: return

        lookahead = io.get_lookahead(agent.self_uid)
        max_idx = len(traj) - 1
        lookahead = max(0, min(lookahead, max_idx))
        
        target = traj[lookahead]
        self_pos = [me["x"], me["y"], me["z"]]
        dist_to_target = v_norm(v_sub(target, self_pos))

        # ==== 3. Segment Start Synchronization ====
        # 只有在轨迹起点 (lookahead=0) 且已经到达该点附近时，触发等待
        if lookahead == 0 and dist_to_target <= CLOSE_TH and hasattr(agent, "current_segment_siblings") and agent.current_segment_siblings:
             # Check if already synced for THIS segment
            if not getattr(agent, "has_synced_segment", False):
                # A. 标记自己正在等待当前特定的 Segment
                # 使用 current_segment_key 作为值，防止不同步的 Agent (还在上一段的) 干扰
                target_sync_key = getattr(agent, "current_segment_key", "unknown")
                io.set_uav_state(agent.self_uid, "current_segment_sync", target_sync_key)
                
                # B. 检查队友状态
                peers_to_check = [p for p in agent.current_segment_siblings if p != agent.self_uid]
                
                if peers_to_check:
                    peer_states = io.mget_uav_states(peers_to_check, "current_segment_sync")
                    all_arrived = True
                    for p, s in peer_states.items():
                        # 必须所有人都声明自己在同一个 Segment Key 上
                        if s != target_sync_key:
                            all_arrived = False
                            break
                    
                    if not all_arrived:
                        # 还没齐，维持位置，不推进 lookahead
                        # 修复：只更新位置，不要使用 set_traj 会清空历史轨迹
                        io.set_pos(agent.self_uid, self_pos[0], self_pos[1], self_pos[2])
                        return 
                    else:
                        print(f"[{agent.self_uid}] All siblings synced at start of {target_sync_key}. Starting!")
                        agent.has_synced_segment = True
                        io.set_lookahead(agent.self_uid, 1)
                else:
                    agent.has_synced_segment = True
                    io.set_lookahead(agent.self_uid, 1)

        # 4. 检查任务结束
        if lookahead >= max_idx and dist_to_target <= CLOSE_TH:
            io.set_lookahead(agent.self_uid, 0)
            
            # 清理状态：标记已完成
            io.set_uav_state(agent.self_uid, "current_segment_sync", "finished")

            agent.bdi.set_belief("can_task_start", True)
            if agent.is_final_task:
                agent.is_finished = True
            else:
                if not agent.is_finished: 
                    if hasattr(agent, "add_achievement_goal") and not agent.merge_peers:
                        agent.add_achievement_goal("task_digraph")
                    print(f"[{agent.self_uid}] Segment completed (reached end).")
            return

        # 5. 物理计算 (Attraction + Repulsion)
        agent.blue_ids = io.get_ids(blue=True)
        all_blue_pos = io.mget_pos(agent.blue_ids, blue=True)
        
        F_att = v_scale(v_sub(target, self_pos), K_ATT)
        
        F_rep = [0.0, 0.0, 0.0]
        for other_uid, other_data in all_blue_pos.items():
            if other_uid == agent.self_uid or not other_data: continue
            other_pos = [other_data['x'], other_data['y'], other_data['z']]
            dist = v_norm(v_sub(self_pos, other_pos))
            d_safe = 15.0  
            if dist < d_safe:
                rep_mag = K_REP * (1.0/dist - 1.0/d_safe) * 50.0 
                rep_dir = v_unit(v_sub(self_pos, other_pos))
                F_rep = v_add(F_rep, v_scale(rep_dir, rep_mag))

        # F_total = v_add(F_att, F_rep)
        F_total = F_att  # 先只用引力测试效果
        nxt = v_add(self_pos, F_total)

        # 6. 状态更新
        if dist_to_target <= CLOSE_TH and lookahead < max_idx:
             # 只有在已同步 (或不需要同步) 后才推进
            if lookahead > 0 or getattr(agent, "has_synced_segment", False) or not agent.current_segment_siblings:
                 io.set_lookahead(agent.self_uid, lookahead + 1)

        io.set_pos(agent.self_uid, nxt[0], nxt[1], nxt[2])
        io.append_traj_points(agent.self_uid, nxt)