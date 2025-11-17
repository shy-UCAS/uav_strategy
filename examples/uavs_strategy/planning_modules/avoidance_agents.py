import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

import basic_functions as bfun
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from itertools import cycle

import configparser

config = configparser.ConfigParser()
config.read('cfg.ini')
# blue_agent_id = config.getint('DEFAULT', 'blue_agent_id')
blue_agent_id = 7
range_radius = config.getfloat('DEFAULT', 'range_radius')
red_range_radius = config.getfloat('DEFAULT', 'red_range_radius')
target_range_radius = config.getfloat('DEFAULT', 'target_range_radius')
dymic_red_krep_xy = config.getfloat('DEFAULT', 'dymic_red_krep_xy')
dymic_red_krep_z = config.getfloat('DEFAULT', 'dymic_red_krep_z')
kattr = config.getfloat('DEFAULT', 'kattr')


class AvoidanceAgent:
    def __init__(self, preset_trajectory: bfun.DroneTrajectory, mass=1, avg_speed=15, max_speed=50, max_acc=20,
                 repulsive_range=50):
        self.preset_trajectory = preset_trajectory
        self.uav_id = preset_trajectory.uav_id

        # basic physical properties of uavs
        self.mass = mass
        self.avg_speed = avg_speed
        self.max_speed = max_speed
        self.max_acc = max_acc
        self.repulsive_range = repulsive_range

        # current location, avoidance trajectory
        self.cur_location = None  # <_cur_utm_xy>,

        self.trajectory_switch = 'preset'  # 默认一开始是走preset轨迹，否则进入到secondary的避障轨迹
        self.cur_preset_frame = 1  # 用于记录当前无人机实际所在的trajectory frame，用于在躲避后矫正航线轨迹
        self.cur_avoid_frame = 0  # 用于基于当前无人机实际traverse的avoidance trajectory的次序
        self.avoidance_trajectory = bfun.DroneTrajectory(self.uav_id)  # 实际飞行过程中产生的轨迹（预设+secondary轨迹形成）
        self.secondary_trajectory = None  # 临机避障时使用的轨迹，如果临机避障的状态满足，则使用该轨迹，否则使用self.avoidance_trajectory

    def infer_preset_speeds_accs(self, step_time=1, vis=False):
        _cur_utm_xys = self.preset_trajectory.utm_xys
        _cur_alts = self.preset_trajectory.alts

        # infer speeds and accs from the trajectory
        _utm_xys_diffs = np.diff(_cur_utm_xys, axis=0)
        _alts_diffs = np.diff(_cur_alts, axis=0)

        _speeds = np.sqrt(_utm_xys_diffs[:, 0] ** 2 + _utm_xys_diffs[:, 1] ** 2 + _alts_diffs ** 2) / step_time
        _accs = np.diff(_speeds, axis=0) / step_time

        if vis:
            _fig, _axs = plt.subplots(1, 2, figsize=(10, 5))
            _axs[0].plot(_speeds);
            _axs[0].set_title('speeds')
            _axs[1].plot(_accs);
            _axs[1].set_title('accs')
            plt.tight_layout()
            plt.show()

    def get_current_location(self):
        _cur_utm_xy = self.avoidance_trajectory.utm_xys[-1]
        _cur_alt = self.avoidance_trajectory.alts[-1]

        return np.concatenate((_cur_utm_xy, np.array([_cur_alt])))

    def infer_last_speed(self, frame=None, scalar=True, step_time=1):
        if frame is None and len(self.avoidance_trajectory) > 1:
            _cur_utm_xy = self.avoidance_trajectory.utm_xys[-1]
            _cur_alt = self.avoidance_trajectory.alts[-1]

            _prv_utm_xy = self.avoidance_trajectory.utm_xys[-2]
            _prv_alt = self.avoidance_trajectory.alts[-2]

            if scalar:
                _estimate_speed = np.sqrt(
                    (_cur_utm_xy[0] - _prv_utm_xy[0]) ** 2 + (_cur_utm_xy[1] - _prv_utm_xy[1]) ** 2 + (
                            _cur_alt - _prv_alt) ** 2) / step_time
            else:
                _estimate_speed = np.concatenate((_cur_utm_xy - _prv_utm_xy, np.array([_cur_alt - _prv_alt])))

        elif frame is not None and frame < len(self.preset_trajectory) and len(self.avoidance_trajectory) > 1:
            _cur_utm_xy = self.avoidance_trajectory.utm_xys[frame, :]
            _cur_alt = self.avoidance_trajectory.alts[frame]

            _prv_utm_xy = self.avoidance_trajectory.utm_xys[frame - 1, :]
            _prv_alt = self.avoidance_trajectory.alts[frame - 1]

            if scalar:
                _estimate_speed = np.sqrt(
                    (_cur_utm_xy[0] - _prv_utm_xy[0]) ** 2 + (_cur_utm_xy[1] - _prv_utm_xy[1]) ** 2 + (
                            _cur_alt - _prv_alt) ** 2) / step_time
            else:
                _estimate_speed = np.concatenate((_cur_utm_xy - _prv_utm_xy, np.array([_cur_alt - _prv_alt])))

        else:
            if scalar:
                _estimate_speed = self.avg_speed
            else:
                _estimate_speed = np.array([0, 0, 0])

        return _estimate_speed

    def trajectory_length(self):
        _preset_utm_xys = self.preset_trajectory.utm_xys
        return len(_preset_utm_xys)

    def avoid_trajectory_length(self):
        _avoid_utm_xys = self.avoidance_trajectory.utm_xys
        return len(_avoid_utm_xys)

    def reset_preset_trajframe(self, frame=0):
        # 重新矫正预设轨迹frame，用于在躲避后矫正航线轨迹
        self.trajectory_switch = 'preset'
        self.cur_preset_frame = frame

    def reset_avoid_trajframe(self, frame=0):
        # 重新矫正预设轨迹frame，用于在躲避后矫正航线轨迹
        self.trajectory_switch = 'avoidance'
        self.cur_avoid_frame = frame

    def step_traj_forward(self, vocal=False):
        if self.trajectory_switch == 'preset':
            self.cur_preset_frame += 1
        elif self.trajectory_switch == 'avoidance':
            self.cur_avoid_frame += 1

        if vocal:
            print(
                f'--> self.uav_id: {self.uav_id}, trajectory_switch: {self.trajectory_switch}, cur_preset_frame: {self.cur_preset_frame}, self.cur_avoid_frame: {self.cur_avoid_frame}')

    def cur_target_bystep(self, step):
        # return <_cur_utm_xy>, <_cur_lnglat>, _cur_alt, _cur_ts
        _cur_utm_xy, _cur_lnglat, _cur_alt = self.preset_trajectory.location_at_step(step)
        _cur_ts = self.preset_trajectory.time_at_step(step)

        return _cur_utm_xy, _cur_lnglat, _cur_alt, _cur_ts

    def append_to_avoidance_trajectory(self, _cur_utm_xy, _cur_alt):
        self.avoidance_trajectory.append_utmxy_alt(_cur_utm_xy, _cur_alt)

    # def repulsive_from_redagents(self, self_location, reds_locations):
    #     ''' 来自红方agent的斥力，应该是距离越近、斥力越大
    #     '''
    #
    #     _loc_delta = np.linalg.norm(self_location - reds_locations)
    #
    #     _equiv_dist = _loc_delta / (self.repulsive_range * 2) * 10
    #     _repulsive_acc = min(self.max_acc / (_equiv_dist ** 2 + 1), self.max_acc)
    #
    #     _repulse_force = _repulsive_acc * (self_location - reds_locations) / max(_loc_delta, 1e-5)
    #
    #     return _repulse_force
    #
    # def respulsive_from_other_blueagents(self, self_location, other_blues_locations):
    #     ''' 来自其他蓝方agent的斥力，应该是距离越近、斥力越大
    #     '''
    #     _loc_delta = np.linalg.norm(self_location - other_blues_locations)
    #     _equiv_dist = _loc_delta / self.repulsive_range * 10
    #     _repulsive_acc = min(self.max_acc / (_equiv_dist ** 2 + 1), self.max_acc)
    #
    #     _repulse_force = _repulsive_acc * (self_location - other_blues_locations) / max(_loc_delta, 1e-5)
    #
    #     return _repulse_force
    #
    # def attractive_to_target(self, self_location, target_location):
    #     ''' 来自目标点的吸引力，应该是距离越远、吸引力越大
    #     '''
    #
    #     _loc_delta = np.linalg.norm(target_location - self_location)
    #     # _attractive_acc = min(2 * np.power(_loc_delta, 1.5) / (self.preset_trajectory.time_step ** 2), self.max_acc * 4.5)
    #     _attractive_acc = min(2 * np.power(_loc_delta, 1.5) / (self.preset_trajectory.time_step ** 2), 100)
    #
    #     _attract_force = _attractive_acc * (target_location - self_location) / max(_loc_delta, 1e-5)
    #
    #     return _attract_force

    def repulsive_from_dymanic_redagents(self, self_location, self_vel_vec,
                                         reds_locations, reds_vels, target_location,
                                         krep_xy=1000.0, krep_z=1.0, rep_dist=500.0,
                                         zeta=1.0, eps=1e-6, sum_limit=None):
        """
        3D 动态障碍斥力（相对速度投影）：逐红机计算再求和
        F_r = (1/ρ - 1/ρ0) * (1/ρ^3) * (1 + ζ|dot_rho|) * (X - X_r)
        （可选）你也可以把 (||X-Xg||^2) 当作额外权重乘进去
        """
        self_location = np.asarray(self_location, float).reshape(3)
        self_vel_vec = np.asarray(self_vel_vec[0], float).reshape(3)
        reds_locations = np.asarray(reds_locations, float).reshape(-1, 3)
        reds_vels = np.asarray(reds_vels, float).reshape(-1, 3)
        target_location = np.asarray(target_location, float).reshape(3)

        _dist_to_target = np.linalg.norm(target_location - self_location)  # 若你坚持按原图加入目标距
        F_total = np.zeros(3, float)

        for r_loc, r_vel in zip(reds_locations, reds_vels):
            _red_vec = self_location - r_loc
            red_dist = max(np.linalg.norm(_red_vec), eps)
            if red_dist > rep_dist:
                continue

            _loc_delta = 1.0 / red_dist
            # 相对速度投影
            v_rel = self_vel_vec - r_vel
            dot_rho = np.dot(v_rel, _red_vec) / red_dist
            dyn_w = 1.0 + zeta * abs(dot_rho)

            # 力（只取障碍梯度项的工程实现）
            _repulse_force = (_loc_delta - 1.0 / rep_dist) * (_loc_delta ** 3) * dyn_w * _red_vec

            # （可选）若想更贴近配图里“乘以目标距”：
            # _repulse_force *= (_dist_to_target**2)

            # 各向异性缩放
            _repulse_force[0] *= krep_xy
            _repulse_force[1] *= krep_xy
            _repulse_force[2] *= krep_z

            F_total += _repulse_force
            print(f"_red_dist: {red_dist},_red_vec: {_red_vec.tolist()},_rel_vel: {v_rel.tolist()}")
            print(
                f"_red_loc: {r_loc.tolist()},_force_xy:{krep_xy * (_loc_delta - 1.0 / rep_dist) * (_loc_delta ** 3) * dyn_w},_force_z:{krep_z * (_loc_delta - 1.0 / rep_dist) * (_loc_delta ** 3) * dyn_w}")
        # 合力限幅（可选）
        if sum_limit is not None:
            n = np.linalg.norm(F_total)
            if n > sum_limit:
                F_total *= sum_limit / n

        return F_total

    def repulsive_from_redagents(self, self_location, reds_locations, target_location, krep_xy=1000.0, krep_z=1.0,
                                 rep_dist=500, rep_dist_z=1.0, is_separate=False):
        ''' 来自红方agent的斥力，距离越近斥力越大 '''

        if is_separate:
            self_location_xy = self_location[:, 0:2]
            reds_locations_xy = reds_locations[:, 0:2]
            reds_locations_z = reds_locations[:, 2]
            print(f"self_xy:{self_location_xy.tolist()},reds_xy:{reds_locations_xy.tolist()}")
            red_dist_xy = np.linalg.norm(self_location_xy - reds_locations_xy)
            red_dist_z = np.abs(self_location[0][2] - reds_locations_z)
            _check_dist = np.sqrt(rep_dist ** 2 + rep_dist_z ** 2)
            red_dist = np.linalg.norm(self_location - reds_locations)
            if red_dist < _check_dist:
                print(f"\n-->_dist_xy:{red_dist_xy},_dist_z:{red_dist_z},_check_dist:{_check_dist},_dist:{red_dist}")
                print(f"red_locs: {reds_locations.tolist()}")
                print(f"vec: {(self_location - reds_locations).tolist()}")
                if red_dist_xy < rep_dist:
                    # 计算当前位置与红方之间的欧氏距离
                    _loc_delta_xy = 1 / red_dist_xy
                    _red_vec_xy = self_location_xy - reds_locations_xy
                    _repulse_force_xy = krep_xy * (_loc_delta_xy - 1 / rep_dist) * (_loc_delta_xy ** 1) * _red_vec_xy
                    print(f"_krep_xy = {krep_xy * (_loc_delta_xy - 1 / rep_dist) * (_loc_delta_xy ** 1)}", end=' ')
                    print(f"_vec_xy:{_red_vec_xy}")
                else:
                    _repulse_force_xy = np.zeros_like(self_location_xy)
                if red_dist_z < rep_dist_z:

                    # 计算当前位置与红方之间的欧氏距离
                    _loc_delta_z = 1 / red_dist_z
                    _red_vec_z = self_location[0][2] - reds_locations_z
                    _repulse_force_z = krep_z * (_loc_delta_z - 1 / rep_dist_z) * (_loc_delta_z ** 1) * _red_vec_z
                    print(f"_dist_z:{red_dist_z}", end=' ')
                    print(f"_krep_z = {krep_z * (_loc_delta_z - 1 / rep_dist_z) * (_loc_delta_z ** 1)}")
                else:
                    _repulse_force_z = np.zeros_like(self_location[0][2])
                _repulse_force = np.concatenate([_repulse_force_xy.reshape(1, -1), _repulse_force_z.reshape(1, -1)],
                                                axis=1)
                print(f"_force_xy:{_repulse_force_xy.tolist()},_force_z:{_repulse_force_z.tolist()}")
                print(f"_repulse_force:{_repulse_force.tolist()}")
            else:
                _repulse_force = np.zeros_like(self_location)
        else:
            red_dist = np.linalg.norm(self_location - reds_locations)
            if red_dist < rep_dist:
                # 计算当前位置与红方之间的欧氏距离
                _loc_delta = 1.0 / red_dist
                _red_vec = self_location - reds_locations
                _dist_to_target = np.linalg.norm(target_location - self_location)

                # _repulse_force = (_loc_delta - 1.0 / rep_dist) * (_loc_delta ** 1) * (self_location - reds_locations)
                # _repulse_force[0][0] *= krep_xy
                # _repulse_force[0][1] *= krep_xy
                # _repulse_force[0][2] *= krep_z

                _repulse_force = ((_loc_delta - 1.0 / rep_dist) ** 2) * _dist_to_target * (
                        self_location - reds_locations)
                _repulse_force[0][0] *= krep_xy
                _repulse_force[0][1] *= krep_xy
                _repulse_force[0][2] *= krep_z

                print(
                    f'-->red_dist: {red_dist},red_krep_xy = {krep_xy * (_loc_delta - 1.0 / rep_dist) * (_loc_delta ** 1)},red_krep_z = {krep_z * (_loc_delta - 1 / rep_dist) * (_loc_delta ** 1)}')
                print(f"red_locs: {reds_locations.tolist()}")
                print(f"force_xy: {_repulse_force[0][0]},force_z: {_repulse_force[0][2]}")
                print(f'vec: {(self_location - reds_locations).tolist()}')
            else:
                _repulse_force = np.zeros_like(self_location)

        return _repulse_force

    def respulsive_from_other_blueagents(self, self_location, other_blues_locations, krep=0.0, rep_dist=4.5,
                                         average=True):
        ''' 来自其他蓝方agent的斥力，距离越近斥力越大，返回所有蓝方的斥力总和 '''

        total_repulse_force = np.zeros_like(self_location)

        for other_blue_location in other_blues_locations:
            _blue_dist = np.linalg.norm(self_location - other_blue_location)
            if _blue_dist < rep_dist:
                # 计算与每一架蓝方的欧几里得距离
                _loc_delta = 1 / _blue_dist

                _repulse_force = krep * (_loc_delta - 1 / rep_dist) * (_loc_delta ** 2) * (
                        self_location - other_blue_location)
                if average:
                    total_repulse_force /= len(other_blues_locations)  # 如果需要平均
            else:
                _repulse_force = np.zeros_like(self_location)
            # 累加到总斥力向量中
            total_repulse_force += _repulse_force

        # 返回所有蓝方的斥力总和

        return total_repulse_force

    def attractive_to_target(self, self_location, target_location, kattr=1.30):
        ''' 来自目标点的吸引力，应该是距离越远、吸引力越大
        '''
        _target_vec = target_location - self_location

        _attr_dist = np.linalg.norm(_target_vec)
        _attract_force = kattr * _target_vec
        # _attract_force[0][0] *= (_attr_dist ** 1)
        # _attract_force[0][1] *= (_attr_dist ** 1)
        # _attract_force[0][2] *= (_attr_dist ** 1)

        print(f'_attr_dist: {_attr_dist}')
        print(f'_attract_force: {_attract_force.tolist()}')
        return _attract_force

    def _closest_presettraj_location(self, frame, prev_loc, max_move_dist):
        # 基于上一时刻所在的位置、当前的timestamp，以及一个timestep里面的最大移动距离，获得preset trajectory上面最近的坐标点
        _preset_utm_xys = self.preset_trajectory.utm_xys[frame, :]
        _preset_alt = self.preset_trajectory.alts[frame]
        _preset_location = np.concatenate([_preset_utm_xys.reshape(-1, 2), _preset_alt.reshape(-1, 1)], axis=1)

    def infer_avoidance_location_1step(self, frame, other_blues_locations, other_reds_locations,
                                       _other_reds_vel,
                                       target_mode='stick_to_preset'):
        if self.trajectory_switch == 'preset':
            _prev_utm_xy = self.avoidance_trajectory.utm_xys[self.cur_preset_frame - 1]
            _prev_alt = self.avoidance_trajectory.alts[self.cur_preset_frame - 1]
        else:
            _prev_utm_xy = self.avoidance_trajectory.utm_xys[self.cur_avoid_frame - 1]
            _prev_alt = self.avoidance_trajectory.alts[self.cur_avoid_frame - 1]

        _prev_xyaloc = np.concatenate([_prev_utm_xy.reshape(-1, 2), _prev_alt.reshape(-1, 1)], axis=1)

        # 计算当前速度
        if self.cur_preset_frame > 1:
            if self.trajectory_switch == 'preset':
                dt = self.avoidance_trajectory.time_step
                _prev_prev_utm_xy = self.avoidance_trajectory.utm_xys[self.cur_preset_frame - 2]
                _prev_prev_alt = self.avoidance_trajectory.alts[self.cur_preset_frame - 2]

                _prev_prev_xyaloc = np.concatenate([_prev_prev_utm_xy.reshape(-1, 2), _prev_prev_alt.reshape(-1, 1)],
                                                   axis=1)
                _prev_vel = (_prev_xyaloc - _prev_prev_xyaloc) / dt
            else:
                dt = self.avoidance_trajectory.time_step
                _prev_prev_utm_xy = self.avoidance_trajectory.utm_xys[self.cur_preset_frame - 2]
                _prev_prev_alt = self.avoidance_trajectory.alts[self.cur_preset_frame - 2]
                _prev_prev_xyaloc = np.concatenate([_prev_prev_utm_xy.reshape(-1, 2), _prev_prev_alt.reshape(-1, 1)],
                                                   axis=1)
                _prev_vel = (_prev_xyaloc - _prev_prev_xyaloc) / dt
        else:
            _prev_vel = np.zeros_like(_prev_xyaloc)
        print(f"_prev_vel: {_prev_vel.tolist()}")
        print(f"_other_reds_vel: {_other_reds_vel.tolist()}")

        if target_mode == 'try_to_escape':
            # 根据一定的距离范围，找到可以到达的蓝方坐标点，主要用于恢复原先的飞行计划
            _last_fly_speed = self.infer_last_speed()
            _maximum_dist_1step = min(self.max_speed,
                                      _last_fly_speed + self.max_acc * self.preset_trajectory.time_step / 2) * self.preset_trajectory.time_step

            # 判断是否有红方无人机在机动躲避范围内
            _reds_in_range = np.linalg.norm(other_reds_locations - _prev_xyaloc, axis=1) <= self.repulsive_range

            if np.any(_reds_in_range):
                # 如果有，则选择距离红方无人机最远的点作为目标点(先随机生成一组点，然后选里面距离最远的)
                pass
            else:
                if self.trajectory_switch == 'preset':
                    _target_utm_xy = self.preset_trajectory.utm_xys[self.cur_preset_frame]
                    _target_alt = self.preset_trajectory.alts[self.cur_preset_frame]
                else:
                    _target_utm_xy = self.preset_trajectory.utm_xys[self.cur_avoid_frame]
                    _target_alt = self.preset_trajectory.alts[self.cur_avoid_frame]

                _target_xyaloc = np.concatenate([_target_utm_xy.reshape(-1, 2), _target_alt.reshape(-1, 1)], axis=1)

        elif target_mode == 'stick_to_preset':
            _target_step = 1
            if self.trajectory_switch == 'preset':
                _traj_len = len(self.preset_trajectory.utm_xys) - 1
                _target_utm_xy = self.preset_trajectory.utm_xys[min(self.cur_preset_frame + _target_step, _traj_len)]
                _target_alt = self.preset_trajectory.alts[min(self.cur_preset_frame + _target_step, _traj_len)]
            else:
                _traj_len = len(self.preset_trajectory.utm_xys) - 1
                _target_utm_xy = self.preset_trajectory.utm_xys[min(self.cur_preset_frame + _target_step, _traj_len)]
                _target_alt = self.preset_trajectory.alts[min(self.cur_preset_frame + _target_step, _traj_len)]

            _target_xyaloc = np.concatenate([_target_utm_xy.reshape(-1, 2), _target_alt.reshape(-1, 1)], axis=1)

        # 斥力计算的初试方法
        # _resps_from_reds_vec = self.repulsive_from_redagents(_prev_xyaloc, other_reds_locations)
        # _resps_from_blues_vec = self.respulsive_from_other_blueagents(_prev_xyaloc, other_blues_locations)

        # 斥力计算的优化方法
        # _resps_from_reds_vec = self.repulsive_from_redagents(_prev_xyaloc, other_reds_locations,target_location=_target_xyaloc, krep_xy=40,
        #                                                      krep_z=3,rep_dist=red_range_radius,rep_dist_z= 250,is_separate=False)
        # _resps_from_blues_vec = self.respulsive_from_other_blueagents(_prev_xyaloc, other_blues_locations)

        # 动态障碍物的斥力计算方法
        _resps_from_reds_vec = self.repulsive_from_dymanic_redagents(_prev_xyaloc, _prev_vel, other_reds_locations,
                                                                     _other_reds_vel,
                                                                     target_location=_target_xyaloc,
                                                                     krep_xy=dymic_red_krep_xy,
                                                                     krep_z=dymic_red_krep_z, rep_dist=red_range_radius,
                                                                     )
        _resps_from_blues_vec = self.respulsive_from_other_blueagents(_prev_xyaloc, other_blues_locations)

        _kattr = kattr
        _attractive_to_target_vec = self.attractive_to_target(_prev_xyaloc, _target_xyaloc, kattr=_kattr)

        # 计算总向量
        w_red, w_blue, w_tar = 1.0, 0.0, 1.0
        _mix_force_vec = w_red * _resps_from_reds_vec + w_blue * _resps_from_blues_vec + w_tar * _attractive_to_target_vec
        _cur_xyaloc = _prev_xyaloc.reshape(-1) + _mix_force_vec
        print(
            f'--> _resps_from_reds_vec: {_resps_from_reds_vec}, _resps_from_blues_vec: {_resps_from_blues_vec}, _attractive_to_target_vec: {_attractive_to_target_vec}')
        print(
            f"_prev_xyaloc: {_prev_xyaloc.tolist()}, _target_xyaloc: {_target_xyaloc.tolist()}, _mix_force_vec: {_mix_force_vec.tolist()}")
        print(f'_cur_xyaloc: {_cur_xyaloc.tolist()}')

        return _cur_xyaloc


class AvoidanceCluster:
    ''' The blue agents should avoid the red agents
    '''

    def __init__(self, blue_agents: list[AvoidanceAgent], red_agents: list[AvoidanceAgent]):
        self.blue_agents = blue_agents
        self.red_agents = red_agents

    def min_total_trajlength(self):
        _blue_traj_lens = [blue_agent.trajectory_length() for blue_agent in self.blue_agents]
        _red_traj_lens = [red_agent.trajectory_length() for red_agent in self.red_agents]

        return min(_blue_traj_lens + _red_traj_lens)

    def reinfer_bluetrajs_bysteps(self, num_steps=None, vis=False):
        if num_steps is None:
            num_steps = self.min_total_trajlength()
        else:
            num_steps = min(num_steps, self.min_total_trajlength())

        for _step_i in range(num_steps):
            print(f"\n-----current step: {_step_i}-----")
            if _step_i <= 0:
                for _blue_agent in self.blue_agents:
                    _cur_utmxy, _cur_lnglat, _cur_alt, _cur_ts = _blue_agent.cur_target_bystep(_step_i)
                    _blue_agent.append_to_avoidance_trajectory(_cur_utmxy, _cur_alt)
                continue

            for _blue_iter, _blue_agent in enumerate(self.blue_agents):
                print(f"\n--->blue agent: {_blue_iter}")
                _other_blues_prev_utmxys = []
                _other_blues_prev_alts = []
                for _other_blue_iter, _other_blue_agent in enumerate(self.blue_agents):
                    if _other_blue_iter == _blue_iter:
                        continue

                    _ob_prev_utmxy, _ob_prev_lnglat, _ob_prev_alt = _other_blue_agent.avoidance_trajectory.location_at_step(
                        _step_i - 1)
                    _other_blues_prev_utmxys.append(_ob_prev_utmxy)
                    _other_blues_prev_alts.append(_ob_prev_alt)

                _other_blues_prev_xyalocs = np.concatenate([np.array(_other_blues_prev_utmxys).reshape(-1, 2),
                                                            np.array(_other_blues_prev_alts).reshape(-1, 1)], axis=1)
                _reds_prev_utmxys = []
                _reds_prev_alts = []
                _reds_prev_prev_utmxys = []
                _reds_prev_prev_alts = []
                for _red_iter, _red_agent in enumerate(self.red_agents):

                    _or_prev_utmxy, _or_prev_lnglat, _or_prev_alt, _or_prev_ts = _red_agent.cur_target_bystep(
                        _step_i - 1)
                    _reds_prev_utmxys.append(_or_prev_utmxy)
                    _reds_prev_alts.append(_or_prev_alt)
                    if _step_i > 1:
                        _or_prev2_utmxy, _, _or_prev2_alt, _or_prev2_ts = \
                            _red_agent.cur_target_bystep(_step_i - 2)
                        _reds_prev_prev_utmxys.append(_or_prev2_utmxy)
                        _reds_prev_prev_alts.append(_or_prev2_alt)
                    else:
                        _reds_prev_prev_utmxys.append(_or_prev_utmxy)
                        _reds_prev_prev_alts.append(_or_prev_alt)

                _reds_prev_xyalocs = np.concatenate([np.array(_reds_prev_utmxys).reshape(-1, 2),
                                                     np.array(_reds_prev_alts).reshape(-1, 1)], axis=1)
                _reds_prev_prev_xyalocs = np.concatenate([np.array(_reds_prev_prev_utmxys).reshape(-1, 2),
                                                          np.array(_reds_prev_prev_alts).reshape(-1, 1)], axis=1)
                dt = _blue_agent.avoidance_trajectory.time_step
                _reds_prev_vels = (_reds_prev_xyalocs - _reds_prev_prev_xyalocs) / dt

                _cur_avoid_xyaloc = _blue_agent.infer_avoidance_location_1step(_step_i, _other_blues_prev_xyalocs,
                                                                               _reds_prev_xyalocs,
                                                                               _reds_prev_vels,
                                                                               target_mode='stick_to_preset')

                _blue_agent.append_to_avoidance_trajectory(_cur_avoid_xyaloc[0, 0:2], _cur_avoid_xyaloc[0, 2])
                _blue_agent.step_traj_forward(vocal=False)

        if vis:

            show_step = 60
            current_list = [np.concatenate([ba.avoidance_trajectory.utm_xys[max(show_step, 0)].tolist(),
                                            [ba.avoidance_trajectory.alts[max(show_step, 0)]]]).tolist() for ba in
                            self.blue_agents]
            last_list = [np.concatenate([ba.avoidance_trajectory.utm_xys[max(show_step - 1, 0)].tolist(),
                                         [ba.avoidance_trajectory.alts[max(show_step - 1, 0)]]]).tolist() for ba in
                         self.blue_agents]

            target_list = [np.concatenate([ba.preset_trajectory.utm_xys[show_step + 1].tolist(),
                                           [ba.preset_trajectory.alts[show_step + 1]]]).tolist() for ba in
                           self.blue_agents]

            all_red_lists = []  # 存放每一步的 red_list
            _blue_agent_traj = []  # 读取一条完整的蓝方轨迹

            # 假设总步数是 num_steps
            for step_i in range(num_steps):
                _reds_prev_utmxys = []
                _reds_prev_alts = []
                for _blue_iter, _blue_agent in enumerate(self.blue_agents):
                    if _blue_iter == blue_agent_id:
                        _prev_utmxy, _prev_lnglat, _prev_alt = _blue_agent.preset_trajectory.location_at_step(
                            step_i)
                        _blue_agent_traj.append([_prev_utmxy[0].tolist(), _prev_utmxy[1].tolist()])
                for _red_iter, _red_agent in enumerate(self.red_agents):
                    _or_prev_utmxy, _or_prev_lnglat, _or_prev_alt, _or_prev_ts = _red_agent.cur_target_bystep(step_i)
                    _reds_prev_utmxys.append(_or_prev_utmxy)
                    _reds_prev_alts.append(_or_prev_alt)

                red_list = np.concatenate([
                    np.array(_reds_prev_utmxys).reshape(-1, 2),
                    np.array(_reds_prev_alts).reshape(-1, 1)
                ], axis=1)
                all_red_lists.append(red_list)
            print(f"blue agent traj:\n {_blue_agent_traj[::-1]},\nlen: {len(_blue_agent_traj)}")
            # 存放单步red_locations
            _reds_prev_utmxys = []
            _reds_prev_alts = []
            for _red_iter, _red_agent in enumerate(self.red_agents):
                _or_prev_utmxy, _or_prev_lnglat, _or_prev_alt, _or_prev_ts = _red_agent.cur_target_bystep(
                    show_step - 1)
                _reds_prev_utmxys.append(_or_prev_utmxy)
                _reds_prev_alts.append(_or_prev_alt)
            red_list = np.concatenate([np.array(_reds_prev_utmxys).reshape(-1, 2),
                                       np.array(_reds_prev_alts).reshape(-1, 1)], axis=1)

            # 弹出一个窗口显示该步
            self.show_blue_positions_step(show_step, current_list, last_list, target_list, red_list, block=True)

            # 创建一个 1x2 的子图，每个子图为三维坐标系
            _fig = plt.figure(figsize=(12, 6))
            _axs = [_fig.add_subplot(121, projection='3d'), _fig.add_subplot(122, projection='3d')]

            # 为轨迹选择不同的颜色
            colors = cycle(['b', 'g', 'r', 'c', 'm', 'y', 'k'])  # 颜色循环
            blue_trajectory_colors = cycle(['b', 'g', 'r', 'c', 'm', 'y', 'k'])
            red_trajectory_colors = cycle(['b', 'g', 'r', 'c', 'm', 'y', 'k'])
            _show_idx = blue_agent_id  # 显示第几个轨迹
            # 绘制蓝方的轨迹
            for idx, _blue_agent in enumerate(self.blue_agents):
                if idx == _show_idx:
                    _preset_utm_xys = _blue_agent.preset_trajectory.utm_xys
                    _preset_alts = _blue_agent.preset_trajectory.alts
                    # 绘制蓝方的预设轨迹（在三维空间）
                    color = next(blue_trajectory_colors)  # 获取蓝方的颜色
                    _axs[1].plot(_preset_utm_xys[:, 0], _preset_utm_xys[:, 1], _preset_alts, f'{color}-',
                                 label=f"Blue Agent {idx} Preset")

                    _avoid_utm_xys = _blue_agent.avoidance_trajectory.utm_xys
                    _avoid_alts = _blue_agent.avoidance_trajectory.alts
                    # 绘制蓝方的避障轨迹（在三维空间）
                    color = next(blue_trajectory_colors)  # 获取蓝方的颜色
                    _axs[1].scatter(_avoid_utm_xys[:, 0], _avoid_utm_xys[:, 1], _avoid_alts, f'{color}-',
                                    label=f"Blue Agent {idx} Avoidance")

            # 绘制红方的轨迹
            for idx, _red_agent in enumerate(self.red_agents):
                _red_utm_xys = _red_agent.preset_trajectory.utm_xys
                _red_alts = _red_agent.preset_trajectory.alts

                # 绘制红方的预设轨迹（在三维空间）
                color = next(red_trajectory_colors)  # 获取红方的颜色
                _axs[0].plot(_red_utm_xys[:, 0], _red_utm_xys[:, 1], _red_alts, f'r-',
                             label=f"Red Agent {idx} Preset")
                _axs[1].scatter(_red_utm_xys[:, 0], _red_utm_xys[:, 1], _red_alts, f'r-',
                                label=f"Red Agent {idx} Preset")

                # self.draw_tube_from_path(_axs[1], all_red_lists, r=red_range_radius, r_minor=None, color=color, alpha=0.12,
                #                     wire=True,
                #                     circle_res=12, step=3)

            # 设置标签
            _axs[0].set_xlabel('X')
            _axs[0].set_ylabel('Y')
            _axs[0].set_zlabel('Altitude')
            _axs[0].set_title('Blue & Red Agents - Preset Trajectories')

            _axs[1].set_xlabel('X')
            _axs[1].set_ylabel('Y')
            _axs[1].set_zlabel('Altitude')
            _axs[1].set_title('Blue Agents - Avoidance Trajectories')

            # 添加图例
            _axs[0].legend()
            _axs[1].legend()

            # 自动调整布局
            plt.tight_layout()
            plt.show()
            anim = self.animate_trajectories(id=_show_idx, save_path=None, fps=20)

    @staticmethod
    def compute_red_velocities(_reds_prev_xyalocs, _reds_prev_prev_xyalocs, dt, speed_limit=None):
        """ 计算红方无人机的速度，并对速度进行限幅和有效性检查。
        """
        # 计算位置差（位置差是红方无人机的速度）
        delta_pos = _reds_prev_xyalocs - _reds_prev_prev_xyalocs

        # 检查除零情况
        if np.any(np.linalg.norm(delta_pos, axis=1) == 0):  # 如果位置差为零
            print("Warning: Some position differences are zero, setting velocity to zero.")
            return np.zeros_like(delta_pos)  # 返回零速度

        # 计算速度
        _reds_prev_vels = delta_pos / dt

        # 速度限幅
        if speed_limit is not None:
            speed_magnitudes = np.linalg.norm(_reds_prev_vels, axis=1)
            exceed_mask = speed_magnitudes > speed_limit  # 找出超出限幅的速度

            # 对超出限幅的速度进行缩放
            _reds_prev_vels[exceed_mask] = (
                    _reds_prev_vels[exceed_mask].T * speed_limit / speed_magnitudes[exceed_mask]).T

        return _reds_prev_vels



