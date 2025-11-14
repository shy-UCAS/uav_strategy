# -*- coding: utf-8 -*-
"""
uav_planning_actions.py

把 BlueUAVAgent 中的轨迹规划与 BDI 动作抽出来，做成可复用模块。
使用方式：
    from planning_modules.uav_planning_actions import register_planning_actions

    class BlueUAVAgent(BDIAgent):
        ...
        def add_custom_actions(self, actions):
            # 先注册 redis 相关动作（.set_traj/.get_traj 等），然后：
            register_planning_actions(self, actions)
"""

import random
from typing import Any, Dict, List

import numpy as np
import agentspeak

import examples.uavs_strategy.planning_modules.basic_functions as bfunc
import examples.uavs_strategy.planning_modules.quick_path_planners as qpp
import examples.uavs_strategy.planning_modules.math_curves_generators as curve_gen


# ------------------------ 小工具 ------------------------ #

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
    """

    def __init__(self, self_agent):
        self.self_agent = self_agent

    # 为了书写方便，提供一个别名
    @property
    def agent(self):
        return self.self_agent

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

        # 起点高度
        if len(traj[0]) == 2:
            traj[0].append(start_height if start_height != -1 else alt_start_rand)

        # 终点高度
        if len(traj[-1]) == 2:
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
            print(f"_outside_border_traj: {border_traj}")
            return [list(p) if isinstance(p, tuple) else list(p) for p in border_traj]
        else:
            # 计算最近边界点
            xy = start_location if len(start_location) == 2 else start_location[:2]
            nearest_point = border.get_nearest_border_vertex(xy).coords[0]
            border_traj = [
                start_location,
                list(nearest_point) if isinstance(nearest_point, tuple) else nearest_point,
            ]
            print(f"_inside_border_traj: {border_traj}")
            return [list(p) if isinstance(p, tuple) else list(p) for p in border_traj]


# ------------------------ 注册 BDI Action ------------------------ #

def register_planning_actions(self, actions):
    """
    在 BlueUAVAgent.add_custom_actions(self, actions) 中调用：

        from planning_modules.uav_planning_actions import register_planning_actions

        def add_custom_actions(self, actions):
            # 先注册 redis IO 动作 ...
            register_planning_actions(self, actions)

    注意：
    - 这里的 self 是 BlueUAVAgent 实例
    - 所有动作内部都用 self.xxx（包括 self.io / self.traj / self.facilities）
    - 回调参数 agent_ 只是为了接口一致，不参与属性访问
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

        # 追加到当前完整轨迹
        self.traj.extend(traj_3d[1:])

        print(f"{self.name} is breaking through {arg_target}, trajectory:\n{traj_3d}")
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

        self.traj.extend(traj_3d[1:])

        print(f"{self.name} is escaping {arg_target}, trajectory:\n{traj_3d}")
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

        print(f"{self.name} is detouring {arg_target}, trajectory:\n{self.cur_reference_traj}")
        # print(f"cur full trajectory: {self.traj}")
        yield

    # ---------- 攻击（占位） ---------- #
    @actions.add(".act_attack", 1)
    def _action_attack(agent_, term, intention):
        arg = _ground(term.args[0], intention)
        print(f"{self.name} is attacking {arg} ...")
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
        print(f"{self.name} current position: {position}")

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
        print(f"{self.name} checking join status: {arg}")

        yield
