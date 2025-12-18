# -*- coding: utf-8 -*-

import numpy as np
from time import time
from spade.behaviour import PeriodicBehaviour
from examples.uavs_strategy.planning_modules import avoidance_agents as a_agents

# ==== 势场与步进参数 ====
DT = 0.30       # 周期（秒）
STEP = 8.0     # 每步“最大位移”/速度上限（米/步）
K_ATT = 0.95   # 引力系数 (独立飞行时)
K_ATT_FORM = 1.5 # 引力系数 (编队飞行时，需要更强的跟随力)
K_REP = 2.5    # 斥力系数
CLOSE_TH = 100.0 # 到达判定阈值

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
    """根据 agent.formation_state 决定是执行轨迹跟随(Independent)还是编队跟随(Follower)"""
    async def run(self):
        agent = self.agent
        io = agent.io
        
        # 1. 获取本机实时位置
        me = io.get_pos(agent.self_uid, blue=True)
        if not me: return
        self_pos = [me["x"], me["y"], me["z"]]
        
        # 2. 获取态势感知数据 (速度用于动态避障)
        # 这里的 blue_ids 由 agent.FetchWorldState 更新
        all_blue_speed = io.mget_speed_from_traj(agent.blue_ids, blue=True, dt=DT)
        all_blue_pos = io.mget_pos(agent.blue_ids, blue=True)
        self_vel = all_blue_speed.get(agent.self_uid, [0.0, 0.0, 0.0])

        # 3. 处理参考轨迹写入 (ASL 触发的)
        if_set_ref = agent.bdi.get_belief("if_set_ref_traj")
        if if_set_ref:
            val_str = if_set_ref[len("if_set_ref_traj("):-1]
            if val_str == "true" and agent.cur_reference_traj:
                io.set_ref_traj(agent.self_uid, agent.cur_reference_traj)
                agent.bdi.set_belief("if_set_ref_traj", "false")
                # print(f"[{agent.name}] Reference trajectory updated in Redis.")

        # ==========================================
        # 核心状态机: Independent vs Follower
        # ==========================================
        formation_role = agent.formation_state.get("role", "independent")
        F_att = [0.0, 0.0, 0.0]
        current_goal = None

        if formation_role == "follower":
            # --- 僚机模式 ---
            leader_id = agent.formation_state.get("leader_id")
            offset = agent.formation_state.get("offset", np.array([0,0,0]))
            
            # 从世界观缓存中获取 Leader 位置
            leader_data = agent.world["blue_pos"].get(leader_id)
            
            if leader_data:
                l_pos = np.array([leader_data["x"], leader_data["y"], leader_data["z"]])
                # 目标点 = 领长位置 + 设定偏移量
                formation_target = l_pos + offset
                current_goal = formation_target.tolist()
                
                # 僚机引力：强力指向编队阵位
                F_att = v_scale(v_sub(current_goal, self_pos), K_ATT_FORM)
            else:
                # 丢失 Leader 信号，悬停或维持惯性
                F_att = [0, 0, 0]

        else:
            # --- 独立/领长模式 ---
            traj = agent.cur_reference_traj
            if traj:
                lookahead = io.get_lookahead(agent.self_uid)
                if lookahead is None: lookahead = 0
                # 防止索引越界
                lookahead = max(0, min(lookahead, len(traj) - 1))
                
                current_goal = traj[lookahead]
                
                # 独立引力：指向轨迹上的预瞄点
                F_att = v_scale(v_sub(current_goal, self_pos), K_ATT)
                
                # 到达终点检测 (可选)
                dist_to_end = io.get_dist_2d(agent.self_uid)
                if dist_to_end is not None and dist_to_end <= 50:
                    io.set_lookahead(agent.self_uid, 0)
                    agent.bdi.set_belief("can_task_start", "true")

        # ==========================================
        # 通用逻辑: 避障与位置更新
        # ==========================================
        
        # 1. 构建障碍物列表 (排除自己)
        obstacle_positions = []
        obstacle_vels = []
        for uid, pos in all_blue_pos.items():
            if uid == agent.self_uid: continue
            
            # 策略：僚机不避让自己的 Leader (防止为了避障而掉队)，信任编队保持力
            if formation_role == "follower" and uid == agent.formation_state.get("leader_id"):
                continue

            obstacle_positions.append([pos["x"], pos["y"], pos["z"]])
            vel = all_blue_speed.get(uid, [0.0, 0.0, 0.0])
            obstacle_vels.append(vel)

        # 2. 计算人工势场斥力
        if not obstacle_positions:
            F_rep = np.zeros(3)
        else:
            F_rep = a_agents.compute_dynamic_repulsive_force(
                agent_pos=self_pos,
                agent_vel=self_vel,
                obstacle_positions=obstacle_positions,
                obstacle_vels=obstacle_vels,
                target_pos=current_goal if current_goal else None
            )

        # 3. 合成最终力矢量
        F_total = v_add(F_att, F_rep)

        # 4. 限制最大速度 (步长)
        move_vec = F_total
        if v_norm(move_vec) > STEP:
            move_vec = v_scale(v_unit(move_vec), STEP)
        
        nxt = v_add(self_pos, move_vec)

        # 5. 更新预瞄点 (仅独立模式需要更新进度)
        if formation_role != "follower" and current_goal:
             if v_norm(v_sub(current_goal, nxt)) <= CLOSE_TH:
                 if lookahead < len(traj) - 1:
                     io.set_lookahead(agent.self_uid, lookahead + 1)

        # 6. 写入 Redis
        io.set_pos(agent.self_uid, nxt[0], nxt[1], nxt[2])
        io.append_traj_points(agent.self_uid, nxt)

# 单机APF步进控制行为
class APFStep(PeriodicBehaviour):
    async def run(self):
        agent = self.agent
        io = agent.io
        current_time = time()  # 获取当前时间
        
        # 获取本机数据
        me = io.get_pos(agent.self_uid, blue=True)
        if not me:
            return  # 如果没有当前无人机位置，跳过

        # 获取当前的参考轨迹 (_cur_reference_traj)
        traj = agent.cur_reference_traj
        if not traj:
            return  # 如果没有参考轨迹，跳过
        if_set_ref_traj_belief = agent.bdi.get_belief("if_set_ref_traj")[len("if_set_ref_traj("):-1]

        if if_set_ref_traj_belief == "true":
            # 如果当前轨迹没有写入redis，则写入
            io.set_ref_traj(agent.self_uid, traj)
            agent.bdi.set_belief("if_set_ref_traj", "false")
            print(f"[{agent.self_uid}] ref_traj add to redis")

        # 获取当前预瞄点的位置（从参考轨迹中）
        lookahead = io.get_lookahead(agent.self_uid)
        lookahead = max(1, min(lookahead, len(traj) - 1))
        goal = traj[lookahead]  # 当前目标点（预瞄点）

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
        agent = self.agent  # BlueUAVAgent 实例
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