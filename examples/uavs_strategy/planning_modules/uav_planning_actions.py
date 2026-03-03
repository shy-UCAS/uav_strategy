# -*- coding: utf-8 -*-
"""
uav_planning_actions.py

把 BlueUAVAgent 中的轨迹规划与 BDI 动作抽出来，做成可复用模块。
第一种使用方式：
    from planning_modules.uav_planning_actions import register_planning_actions

    class BlueUAVAgent(BDIAgent):
        ...
        def add_custom_actions(self, actions):
            # 先注册 redis 相关动作（.set_traj/.get_traj 等），然后：
            register_planning_actions(self, actions)

第二种使用方式：
    agent = MyAgent(...) 
    # 初始化规划库
    lib = ASLPlanningLib(agent6)
    # 直接调用规划方法，跳过bdi的行为
    lib.execute_breakthrough(target="hq_01", start_h=100, end_h=150)
    lib.execute_escape(target="defence_rings", start_h=100, end_h=150)
"""

import random
from typing import Any, Dict, List, TYPE_CHECKING

import numpy as np
import agentspeak

import examples.uavs_strategy.planning_modules.basic_functions as bfunc
import examples.uavs_strategy.planning_modules.quick_path_planners as qpp
import examples.uavs_strategy.planning_modules.math_curves_generators as curve_gen

if TYPE_CHECKING:
    from examples.uavs_strategy.uav_dynamic_agents import BlueUAVAgent

def _ground(term, intention):
    """简化 agentspeak.grounded 的调用。"""
    return agentspeak.grounded(term, intention.scope)


# ------------------------ 轨迹规划工具类 ------------------------ #

class PlanningLib:
    """
    轨迹规划与高度插值工具类，内部持有 BlueUAVAgent 的 self。
    通过 self.agent 访问原来的：
        - self.traj
        - self.facilities
        - self.height_range_set
        - 等属性。
    通过asl的BDI 动作触发调用。
    """
    
    def __init__(self, self_agent: "BlueUAVAgent"):
        self.self_agent = self_agent
        self.VERBOSE = False # 控制日志输出

    # 为了书写方便，提供一个别名
    @property
    def agent(self):
        return self.self_agent

    def log(self, msg):
        """可控的日志输出方法"""
        if self.VERBOSE:
            print(msg)

    # ---------- 高度插值 ---------- #
    def insert_height_val(self, order_type: str, traj: List[List[float]],
                          start_height: int, end_height: int) -> List[List[float]]:
        """
        复制自 BlueUAVAgent._insert_height_val：
        - 传入二维轨迹 [ [x,y], [x,y], ... ]
        - 起点/终点附加高度（-1 表示用随机高度）
        - 中间点用 math_curves_generators.interpolate_z_coordinates 插值
        - detour 用 cubic_interpolation_3d，其他用 generate_breakthrough_flight
        """
        # 默认高度范围从 agent.height_range_set 中取
        default_range = self.agent.height_range_set[order_type]

        # 生成随机起终高度
        alt_start_rand = random.randint(default_range[0][0], default_range[0][1])
        alt_end_rand = random.randint(default_range[1][0], default_range[1][1])
        self.log(f"[{self.agent.jid}] Default 2d traj: {traj}")
        # 起点高度
        if len(traj[0]) == 2:

            self.log(f"[{self.agent.jid}] Insert start height: {start_height} (rand: {alt_start_rand})")
            traj[0].append(start_height if start_height != -1 else alt_start_rand)

        # 终点高度
        if len(traj[-1]) == 2:
            # print(f"[{self.agent.jid}] Insert end height: {end_height} (rand: {alt_end_rand})")
            self.log(f"[{self.agent.jid}] Insert end height: {end_height} (rand: {alt_end_rand})")
            traj[-1].append(end_height if end_height != -1 else alt_end_rand)

        # 此时只有首尾是三维，中间点需要补 z
        traj_3d = curve_gen.interpolate_z_coordinates(traj)

        if order_type == "detour":
            # detour：本身点较多，用三次样条平滑
            return curve_gen.cubic_interpolation_3d(traj_3d)
        else:
            # breakthrough/escape：只有两个关键点，套突破专用插值
            return curve_gen.generate_breakthrough_flight(traj_3d)

    # ---------- 突防规划 ---------- #
    def plan_breakthrough_targettype(self, start_location, facility_type: str,
                                     utm: bool = True) -> List[List[float]]:
        """
        按设施类型（antiair/headquarter/prober）随机挑一个目标，构成 [start, target] 二维轨迹。
        """
        target_info = self.agent.facilities.pick_random_target(facility_type, utm)
        loc = target_info["location"]
        return [
            list(start_location) if not isinstance(start_location, list) else start_location,
            list(loc) if isinstance(loc, tuple) else loc,
        ]

    def plan_breakthrough_target(self, start_location, target: str,
                                 utm: bool = True) -> List[List[float]]:
        """
        按设施名（hq_xxx/ua_xxx/radar_xxx）获取坐标，构成 [start, target]。
        """
        target_loc = self.agent.facilities.get_target_location(target, utm)
        return [
            list(start_location) if not isinstance(start_location, list) else start_location,
            list(target_loc) if isinstance(target_loc, tuple) else target_loc,
        ]
    def planbreakthrough_target_location(self, start_location, target_location: List[float],
                                 utm: bool = True) -> List[List[float]]:
        """按照给定设施的经纬坐标构成 [start, target]。"""
        return [
            list(start_location) if not isinstance(start_location, list) else start_location,
            list(target_location) if isinstance(target_location, tuple) else target_location,
        ]

    # ---------- 迂回规划 ---------- #
    def plan_detour(self, start_location, target: str,
                    detour_steps: int = 5, utm: bool = True) -> List[List[float]]:
        """
        复制自 BlueUAVAgent._plan_detour：
        - 根据目标设施/防御圈选出一个多边形边界
        - 用 SimpleBorders 沿边界绕行 detour_steps 个点
        - 返回二维轨迹点列表
        """
        fac = self.agent.facilities

        if target in fac.facilities_info.keys() or target in fac.defend_rings.keys():
            if target in fac.antiairs:
                detour_polygon_xys = fac.get_spec_facility_polyborder(
                    fac.antiairs[target],
                    bfunc.GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE,
                    ll2utm=utm,
                )
            elif target in fac.headquartors:
                detour_polygon_xys = fac.get_spec_facility_polyborder(
                    fac.headquartors[target],
                    bfunc.GlobalBasicConfigs.AVOID_HQ_DISTANCE,
                    ll2utm=utm,
                )
            elif target in fac.probers:
                detour_polygon_xys = fac.get_spec_facility_polyborder(
                    fac.probers[target],
                    bfunc.GlobalBasicConfigs.AVOID_RADAR_DISTANCE,
                    ll2utm=utm,
                )
            elif target in fac.defend_rings.keys():
                detour_polygon_xys = fac.get_defence_rings_polyborder()
        elif target == "defence_rings":
            detour_polygon_xys = fac.get_defence_rings_polyborder()
        elif target == "probe_facilities":
            detour_polygon_xys = fac.get_probe_facilities_polyborder()
        elif target == "antiair_facilities":
            detour_polygon_xys = fac.get_defence_facilities_polyborder()
        else:
            raise ValueError(f"Unknown detour target: {target}")

        border = qpp.SimpleBorders(detour_polygon_xys[0])

        # start_location 可能是 [x,y,z]，这里只用前两维
        xy_start = start_location if len(start_location) == 2 else start_location[:2]
        traj_locations = border.move_along_border(
            xy_start,
            steps=detour_steps,
            direction=np.random.choice(["clockwise", "anticlockwise"]),
        )

        # 转成统一的 list[list]
        return [list(item) if isinstance(item, tuple) else list(item) for item in traj_locations]

    # ---------- 逃逸规划 ---------- #
    def plan_escape(self, start_location, target: str = "defence_rings",
                    utm: bool = True) -> List[List[float]]:
        """
        复制自 BlueUAVAgent._plan_escape：
        - 如果在危险区域内，则连到最近的边界点；
        - 如果本来就在外面，就返回 [start, start]。
        """
        fac = self.agent.facilities

        if target in fac.facilities_info.keys() or target in fac.defend_rings.keys():
            if target in fac.antiairs:
                escape_polygon_xys = fac.get_spec_facility_polyborder(
                    fac.antiairs[target],
                    bfunc.GlobalBasicConfigs.AVOID_ANTIAIR_DISTANCE,
                    ll2utm=utm,
                )
            elif target in fac.headquartors:
                escape_polygon_xys = fac.get_spec_facility_polyborder(
                    fac.headquartors[target],
                    bfunc.GlobalBasicConfigs.AVOID_HQ_DISTANCE,
                    ll2utm=utm,
                )
            elif target in fac.probers:
                escape_polygon_xys = fac.get_spec_facility_polyborder(
                    fac.probers[target],
                    bfunc.GlobalBasicConfigs.AVOID_RADAR_DISTANCE,
                    ll2utm=utm,
                )
            elif target in fac.defend_rings.keys():
                escape_polygon_xys = fac.get_defence_rings_polyborder()
        elif target == "defence_rings":
            escape_polygon_xys = fac.get_defence_rings_polyborder()
        elif target == "probe_facilities":
            escape_polygon_xys = fac.get_probe_facilities_polyborder()
        elif target == "antiair_facilities":
            escape_polygon_xys = fac.get_defence_facilities_polyborder()
        else:
            raise ValueError(f"Unknown escape target: {target}")

        border = qpp.SimpleBorders(escape_polygon_xys[0])

        # shapely 可以接受 2D/3D，这里和你原始代码保持一致
        if not border.is_inside_border(start_location):
            # 当前本来就在逃逸范围之外
            border_traj = [start_location, start_location]
            # print(f"_outside_border_traj: {border_traj}")
            self.log(f"[{self.agent.jid}] is already outside the border, escape traj: {border_traj}")
            return [list(p) if isinstance(p, tuple) else list(p) for p in border_traj]
        else:
            # 计算最近边界点
            xy = start_location if len(start_location) == 2 else start_location[:2]
            nearest_point = border.get_nearest_border_vertex(xy).coords[0]
            border_traj = [
                start_location,
                list(nearest_point) if isinstance(nearest_point, tuple) else nearest_point,
            ]
            self.log(f"_inside_border_traj: {border_traj}")
            return [list(p) if isinstance(p, tuple) else list(p) for p in border_traj]

    # ---------- 执行动作封装 (解耦 BDI) ---------- #
    def execute_path_planning_from_digraph(self, digraph: Dict[str, Any], start_h: int, end_h: int):
        """
        根据 digraph_attr 进行路径规划
        目前逻辑简化为根据 order_type 分发，不再冗余判断 order_mode
        """
        digraph_attr = digraph['attrs']
        order_type = digraph_attr['order_type']
        cur_target = digraph_attr['target']

        if order_type == 'breakthrough' or order_type == 'singleton':
            return self.execute_breakthrough(cur_target, start_h, end_h)
        elif order_type == 'escape':
            return self.execute_escape(cur_target, start_h, end_h)
        elif order_type == 'detour':
            return self.execute_detour(cur_target, start_h, end_h)
        else:
            self.log(f"[{self.agent.name}] Unknown order_type: {order_type}")


    def execute_breakthrough(self, target: str, start_h: int, end_h: int):
        """
        执行突防动作：规划路径并更新 agent 轨迹
        新增一个针对动态汇合点的处理方法
        """
        # 根据 target 类型决定调用哪个 planner
        if target in bfunc.GlobalBasicConfigs.PLANNING_BREAKTHROUGH_FACILITY_TYPES:
            traj_2d = self.plan_breakthrough_targettype(self.agent.traj[-1], target)
        elif target in self.agent.facilities.get_facilities_names():
            traj_2d = self.plan_breakthrough_target(self.agent.traj[-1], target)
        elif target == 'aggregate_point':
            # 多机汇合点
            self.log(f"[{self.agent.name}] is act aggregating to rendezvous point, merge_peers: {self.agent.merge_peers}")
            rendezvous_pos = self.agent.io.get_rendezvous_point(self.agent.merge_peers)
            traj_2d = self.planbreakthrough_target_location(self.agent.traj[-1], rendezvous_pos)
        else:
            # 兜底：目标既不是类型也不是设施名，就保持原地
            traj_2d = [self.agent.traj[-1], self.agent.traj[-1]]

        traj_3d = self.insert_height_val("breakthrough", traj_2d, start_h, end_h)
        # self.agent.cur_reference_traj = traj_3d
        # 追加到当前完整轨迹
        # self.agent.traj.extend(traj_3d[1:])

        # print(f"{self.agent.name} is breaking through {target}, trajectory:\n{traj_3d}")
        return traj_3d


    def execute_escape(self, target: str, start_h: int, end_h: int):
        """
        执行逃逸动作
        """
        traj_2d = self.plan_escape(self.agent.traj[-1], target)
        traj_3d = self.insert_height_val("escape", traj_2d, start_h, end_h)
        # self.agent.cur_reference_traj = traj_3d
        # self.agent.traj.extend(traj_3d[1:])

        self.log(f"{self.agent.name} is escaping {target}, trajectory:\n{traj_3d}")
        return traj_3d


    def execute_detour(self, target: str, start_h: int, end_h: int):
        """
        执行迂回动作
        """
        traj_2d = self.plan_detour(self.agent.traj[-1], target)
        traj_2d[0].append(self.agent.traj[-1][2])  # 保持当前高度不变
        traj_3d = self.insert_height_val("detour", traj_2d, start_h, end_h)

        # self.agent.cur_reference_traj = traj_3d
        # self.agent.traj.extend(traj_3d[1:])

        self.log(f"{self.agent.name} is detouring {target}, trajectory:\n{traj_3d}")
        return traj_3d


    def execute_attack(self, target: str):
        """
        执行攻击动作
        """
        self.log(f"{self.agent.name} is attacking {target} ...")

    def execute_join_formation(self, leader_id: str, off_x: float, off_y: float, off_z: float):
        """
        执行加入编队
        """
        # 切换状态为跟随者
        self.agent.formation_state["role"] = "follower"
        self.agent.formation_state["leader_id"] = leader_id
        self.agent.formation_state["offset"] = np.array([off_x, off_y, off_z])
        
        self.log(f"[{self.agent.name}] Joining Formation -> Leader: {leader_id}, Offset: {self.agent.formation_state['offset']}")

    def execute_leave_formation(self):
        """
        执行离开编队
        """
        # 切换回独立模式
        self.agent.formation_state["role"] = "independent"
        self.agent.formation_state["leader_id"] = None
        
        # 获取当前位置，重置为新轨迹的起点，防止瞬移
        if hasattr(self.agent, "io") and hasattr(self.agent, "self_uid"):
            curr_pos = self.agent.io.get_pos(self.agent.self_uid, blue=True)
            if curr_pos:
                 # 可以在这里清空旧的参考轨迹，强制 BDI 重新规划
                 self.agent.traj = [[curr_pos['x'], curr_pos['y'], curr_pos['z']]]
                 self.agent.io.set_lookahead(self.agent.self_uid, 0)

        self.log(f"[{self.agent.name}] ⚡ LEAVING FORMATION -> Independent Mode.")


# ------------------------ 注册 BDI Action ------------------------ #

def register_planning_actions(self, actions):
    """
    调用之前设计的轨迹规划方法，注册 BDI 动作。
    """
    lib = PlanningLib(self)

    # ---------- 突防 ---------- #
    @actions.add(".act_breakthrough", 3)
    def _action_breakthrough(agent_, term, intention):
        # 目标类型/目标名称；起点高度；终点高度
        arg_target = str(_ground(term.args[0], intention))
        arg_start_h = int(_ground(term.args[1], intention))
        arg_end_h = int(_ground(term.args[2], intention))

        # 根据 target 类型决定调用哪个 planner
        if arg_target in bfunc.GlobalBasicConfigs.PLANNING_BREAKTHROUGH_FACILITY_TYPES:
            traj_2d = lib.plan_breakthrough_targettype(self.traj[-1], arg_target)
        elif arg_target in self.facilities.get_facilities_names():
            traj_2d = lib.plan_breakthrough_target(self.traj[-1], arg_target)
        else:
            # 兜底：目标既不是类型也不是设施名，就保持原地
            traj_2d = [self.traj[-1], self.traj[-1]]

        traj_3d = lib.insert_height_val("breakthrough", traj_2d, arg_start_h, arg_end_h)
        self.cur_reference_traj = traj_3d
        # 追加到当前完整轨迹
        self.traj.extend(traj_3d[1:])

        lib.log(f"{self.name} is breaking through {arg_target}, trajectory:\n{traj_3d}")
        # print(f"cur full trajectory: {self.traj}")

        # 如需写入 redis，可以在这里使用 self.io（如果有）
        # if hasattr(self, "io"):
        #     self.io.set_traj(self.uid, [{"x": x, "y": y, "z": z} for x, y, z in traj_3d])

        yield

    # ---------- 逃逸 ---------- #
    @actions.add(".act_escape", 3)
    def _action_escape(agent_, term, intention):
        arg_target = str(_ground(term.args[0], intention))
        arg_start_h = int(_ground(term.args[1], intention))
        arg_end_h = int(_ground(term.args[2], intention))

        traj_2d = lib.plan_escape(self.traj[-1], arg_target)
        traj_3d = lib.insert_height_val("escape", traj_2d, arg_start_h, arg_end_h)
        self.cur_reference_traj = traj_3d
        self.traj.extend(traj_3d[1:])

        lib.log(f"{self.name} is escaping {arg_target}, trajectory:\n{traj_3d}")
        # print(f"cur full trajectory: {self.traj}")
        yield

    # ---------- 迂回 ---------- #
    @actions.add(".act_detour", 3)
    def _action_detour(agent_, term, intention):
        arg_target = str(_ground(term.args[0], intention))
        arg_start_h = int(_ground(term.args[1], intention))
        arg_end_h = int(_ground(term.args[2], intention))

        traj_2d = lib.plan_detour(self.traj[-1], arg_target)
        traj_3d = lib.insert_height_val("detour", traj_2d, arg_start_h, arg_end_h)

        self.cur_reference_traj = traj_3d
        self.traj.extend(traj_3d[1:])

        lib.log(f"{self.name} is detouring {arg_target}, trajectory:\n{traj_3d}")
        lib.log(f"cur full trajectory: {self.traj}")
        yield

    # ---------- 攻击（占位） ---------- #
    @actions.add(".act_attack", 1)
    def _action_attack(agent_, term, intention):
        arg = _ground(term.args[0], intention)
        lib.log(f"{self.name} is attacking {arg} ...")
        # 这里暂时只打印，后面你可以接火控/打击仿真
        yield

    # ---------- 获取当前位置 ---------- #
    @actions.add(".act_get_position", 1)
    def _action_get_position(agent_, term, intention):
        """
        在 ASL 里类似这样用：
            +!get_pos(X) <- .act_get_position(X).

        会把 self.traj[-1] 绑定到变量 X 上。
        """
        position = self.traj[-1]
        lib.log(f"{self.name} current position: {position}")

        var_term = term.args[0]
        v = _ground(var_term, intention)

        # 尽量兼容不同 agentspeak 版本的变量实现
        try:
            if hasattr(v, "set_value"):
                v.set_value(position)
            else:
                intention.scope._bindings[var_term.name] = position
        except Exception:
            intention.scope._bindings[getattr(var_term, "name", "X")] = position

        yield

    # ---------- 地图可视化 ---------- #
    @actions.add(".act_visualize", 0)
    def _act_visualize(agent_, term, intention):
        """
        ASL:
            +!viz <- .act_visualize.
        """
        self.facilities.visualize(show_mode="2D")
        yield

    # ---------- 多机汇合等待状态检查（占位） ---------- #
    @actions.add(".check_join_status", 1)
    def _act_check_join_status(agent_, term, intention):
        """
        预留给“多机汇合 / 等待盘旋”的逻辑，比如：
            +!task2 <-
                .check_join_status([blue02, blue03], section02_start);
                .act_breakthrough(...).

        目前先简单打印，你后面可以在这里读 redis 状态：
            - self.io.get_pos(...)
            - self.io.mget_pos(...)
        然后根据条件决定是否在 ASL 层触发下一个任务。
        """
        arg = _ground(term.args[0], intention)
        lib.log(f"{self.name} checking join status: {arg}")

        yield

    # ---------- 加入编队 (Join Formation) ---------- #
    @actions.add(".act_join_formation", 4)
    def _act_join_formation(agent_, term, intention):
        leader_id = str(_ground(term.args[0], intention))
        off_x = float(_ground(term.args[1], intention))
        off_y = float(_ground(term.args[2], intention))
        off_z = float(_ground(term.args[3], intention))

        # 切换状态为跟随者
        self.formation_state["role"] = "follower"
        self.formation_state["leader_id"] = leader_id
        self.formation_state["offset"] = np.array([off_x, off_y, off_z])
        
        lib.log(f"[{self.name}] Joining Formation -> Leader: {leader_id}, Offset: {self.formation_state['offset']}")
        yield
        
    # ---------- 离开编队 (Leave Formation) ---------- #
    # ASL调用格式: .act_leave_formation
    @actions.add(".act_leave_formation", 0)
    def _act_leave_formation(agent_, term, intention):
        # 切换回独立模式
        self.formation_state["role"] = "independent"
        self.formation_state["leader_id"] = None
        
        # 获取当前位置，重置为新轨迹的起点，防止瞬移
        curr_pos = self.io.get_pos(self.self_uid, blue=True)
        if curr_pos:
             # 可以在这里清空旧的参考轨迹，强制 BDI 重新规划
             self.traj = [[curr_pos['x'], curr_pos['y'], curr_pos['z']]]
             self.io.set_lookahead(self.self_uid, 0)

        lib.log(f"[{self.name}] ⚡ LEAVING FORMATION -> Independent Mode.")
        yield


