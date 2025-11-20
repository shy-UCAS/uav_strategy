import numpy as np
from matplotlib import pyplot as plt

from modules import math_curves_generators as mcg


class FormationGenerator:
    def __init__(self, member_num, radius, traj, angle, max_offset, noise_scale=0.1, angle_noise_scale=3,
                 formation_type='circular'):
        self.member_num = member_num
        self.radius = radius
        self.traj = traj
        self.angle = angle
        self.max_offset = max_offset
        self.noise_scale = noise_scale
        self.angle_noise_scale = angle_noise_scale
        self.formation_type = formation_type

    def generate_formation(self):
        if self.formation_type == 'circular':
            return self.generate_circular_formation()
        elif self.formation_type == 'vertical':
            return self.generate_vertical_formation()
        elif self.formation_type == 'horizontal':
            return self.generate_horizontal_formation()
        elif self.formation_type == 'vshape':
            return self.generate_vshape_formation()

    def generate_circular_formation(self):
        """生成圆形队形偏移量"""
        # 计算每个点的角度间隔（2π/N 弧度）
        angle_step = 2 * np.pi / self.member_num
        # 生成N个点，范围从0到2π - angle_step（排除完全闭合）
        theta = np.arange(0, 2 * np.pi, angle_step)
        # 确保生成指定数量的点（处理浮点精度问题）
        theta = theta[:self.member_num]
        x = self.radius * np.cos(theta)
        y = self.radius * np.sin(theta)
        return x, y

    def generate_vertical_formation(self):
        """生成垂直队形偏移量"""
        x = np.zeros(self.member_num)
        y = np.linspace(-self.max_offset, self.max_offset, self.member_num)
        return x, y

    def generate_horizontal_formation(self):
        """生成水平队形偏移量"""
        x = np.linspace(-self.max_offset, self.max_offset, self.member_num)
        y = np.zeros(self.member_num)
        return x, y

    def generate_vshape_formation(self):
        """生成V字形队形偏移量
        :param angle_noise_scale: 角度噪声强度（单位：度）
        """
        half_num = self.member_num // 2
        x = []
        y = []
        for i in range(-half_num, half_num + 1):
            # 生成角度噪声（均匀分布）
            noise = np.random.uniform(-self.angle_noise_scale, self.angle_noise_scale)
            current_angle = self.angle + noise

            offset = self.max_offset * abs(i) / half_num
            x.append(offset * np.cos(np.radians(current_angle)) * (1 if i > 0 else -1))
            y.append(offset * np.sin(np.radians(current_angle)) * (-1))
        return np.array(x), np.array(y)

    def generate_arc_formation(self):
        """生成弧形队形偏移量"""
        theta = np.linspace(np.radians(self.angle), np.radians(np.pi - self.angle), self.member_num)
        x = self.radius * np.cos(theta)
        y = self.radius * np.sin(theta)
        return x, y

    def coordinate_rotation(self, x, y, theta):
        # 将角度转换为弧度
        theta_rad = np.radians(theta)
        # 计算旋转矩阵
        rotation_matrix = np.array([[np.cos(theta_rad), -np.sin(theta_rad)],
                                    [np.sin(theta_rad), np.cos(theta_rad)]])
        # 应用旋转矩阵
        x_rotated, y_rotated = np.dot(rotation_matrix, [x, y])
        return x_rotated, y_rotated

    def generate_members_formation(self):
        """生成队形成员轨迹"""
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111)
        for _member in range(self.member_num):
            x, y = self.generate_formation()
            x_offset = x[_member]
            y_offset = y[_member]
            _member_xs = []
            _member_ys = []
            _cur_xs = [_loc[0] for _loc in self.traj]
            _cur_ys = [_loc[1] for _loc in self.traj]
            dx = np.diff(_cur_xs)
            dy = np.diff(_cur_ys)
            for _step, _loc in enumerate(self.traj):
                # 生成随机噪声（高斯分布）
                noise_x = np.random.normal(0, self.noise_scale)
                noise_y = np.random.normal(0, self.noise_scale)
                if _step < len(self.traj) - 1:
                    dx_loc = dx[_step]
                    dy_loc = dy[_step]
                    theta = np.degrees(np.arctan2(dy_loc, dx_loc))
                    rot_x, rot_y = self.coordinate_rotation(x_offset, y_offset, -(90 - theta))
                    # 添加噪声到坐标
                    _member_xs.append(_loc[0] + rot_x + noise_x)
                    _member_ys.append(_loc[1] + rot_y + noise_y)
                else:
                    theta = np.degrees(np.arctan2(dy[-1], dx[-1]))
                    rot_x, rot_y = self.coordinate_rotation(x_offset, y_offset, -(90 - theta))
                    # 添加噪声到坐标
                    _member_xs.append(_loc[0] + rot_x + noise_x)
                    _member_ys.append(_loc[1] + rot_y + noise_y)
            segment = np.array([[x, y] for x, y in zip(_member_xs, _member_ys)])
            ax.scatter(segment[:, 0], segment[:, 1], color=plt.cm.tab10(_member % 10), label='member')
        ax.set_aspect('equal', adjustable='box')
        plt.grid(True)
        plt.show()


def compute_orientation_matrices(traj):
    """
    输入:
      traj: shape (N,3) 的轨迹数组, 每行是 [x,y,z]
    输出:
      Rs: shape (N-1,3,3) 的旋转矩阵列表
    """
    N = len(traj)
    Rs = []
    up_world = np.array([0.0, 0.0, 1.0])  # 全局“上”方向

    for i in range(N - 1):
        # 1. 切向量
        vel = traj[i + 1] - traj[i]
        T = vel / (np.linalg.norm(vel) + 1e-8)  # 防止除零 :contentReference[oaicite:4]{index=4}

        # 2. 右向量：叉乘并归一化
        R = np.cross(up_world, T)
        R /= (np.linalg.norm(R) + 1e-8)  # 边界处理 :contentReference[oaicite:5]{index=5}

        # 3. 上向量
        U = np.cross(T, R)

        # 4. 组旋转矩阵
        mat = np.column_stack((R, U, T))
        Rs.append(mat)

    return np.array(Rs)  # shape (N-1,3,3)


if __name__ == '__main__':
    # # 使用类方法生成队形
    member_num = 7
    # radius = 0.7
    # traj_points = [[0, 0, 0], [-10, -10, -10]]
    # traj = np.array(mcg.generate_breakthrough_flight(traj_points, direction_range=[-40, -20], num_points=5))
    # print(f"Trajectory: {traj}")
    # angle = 20
    # max_offset = 1.0
    # noise_scale = 0.04
    # angle_noise_scale = 3
    # # 队形可选的有：'circular', 'vertical', 'horizontal', 'vshape'
    # formation_type = 'vertical'
    # formation_generator = FormationGenerator(member_num, radius, traj, angle, max_offset, noise_scale,
    #                                          angle_noise_scale, formation_type)
    #
    # # formation_generator.generate_members_formation()
    #
    # # 全局“上”方向
    # up_world = np.array([0.0, 0.0, 1.0])
    #
    # rotation3d_matrices = compute_orientation_matrices(traj)
    # p_body = [0, 0, 1]
    #
    # p_world = rotation3d_matrices[-1].dot(p_body) + traj[-1]
    #
    # # 创建图形与 3D 坐标系
    # fig = plt.figure(figsize=(8, 6))
    # ax = fig.add_subplot(111, projection='3d')
    #
    # # 绘制轨迹线
    # ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], '-k', label='Trajectory')
    #
    # # 箭头长度缩放
    # scale = 0.5
    #
    # # 在每个时间步绘制坐标系
    # for i in range(len(traj)):
    #     if i < len(traj) - 1:
    #         vel = traj[i + 1] - traj[i]
    #     else:
    #         vel = traj[i] - traj[i - 1]
    #     # 切向量（单位切向量）
    #     T = vel / (np.linalg.norm(vel) + 1e-8)  # 防止除零 :contentReference[oaicite:6]{index=6}
    #
    #     # 右向量：叉乘	up_world × T
    #     R = np.cross(up_world, T)
    #     R /= (np.linalg.norm(R) + 1e-8)
    #
    #     # 上向量：T × R
    #     U = np.cross(T, R)
    #
    #     # 当前点
    #     x0, y0, z0 = traj[i]
    #
    #     # 绘制三轴箭头
    #     ax.quiver(x0, y0, z0, R[0], R[1], R[2],
    #               length=scale, color='r', normalize=True)
    #     ax.quiver(x0, y0, z0, U[0], U[1], U[2],
    #               length=scale, color='g', normalize=True)
    #     ax.quiver(x0, y0, z0, T[0], T[1], T[2],
    #               length=scale, color='b', normalize=True)
    #     ax.scatter(p_world[0], p_world[1], p_world[2], color='k', marker='o', label='Drone')
    #
    # # 图例与轴标签
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.legend(['Trajectory', 'Right axis', 'Up axis', 'Forward axis'])
    # plt.title('Drone Orientation Frames Along Trajectory')
    # plt.tight_layout()
    # plt.show()
    uav = [None for i in range(member_num)]
    print(uav)