import numpy as np
import matplotlib.pyplot as plt

class Formation_Elements:
    def __init__(self, member_num = 5, radius=20.0, traj = None, angle=45, max_offset=30.0, noise_scale=0.1, angle_noise_scale=3,
                 formation_type='vshape'):
        self.member_num = member_num
        self.radius = radius
        self.traj = traj
        self.angle = angle
        self.max_offset = max_offset
        self.noise_scale = noise_scale
        self.angle_noise_scale = angle_noise_scale
        self.formation_type = formation_type

class FormationGenerator3D:
    def __init__(self,
                 formation_elements=None,
                 member_num: int = None,
                 radius: float = 20.0,
                 traj: np.ndarray = None,
                 angle: float = 45.0,
                 max_offset: float = 30.0,
                 noise_scale: float = 0.1,
                 angle_noise_scale: float = 3.0,
                 formation_type: str = 'circular'):
        """
        构造函数支持两种方式：
        1. 传入formation_elements实例 + traj参数
        2. 传入所有单独参数（保持向后兼容）
        """
        if formation_elements is not None:
            self.member_num = member_num if member_num is not None else formation_elements.member_num
            self.radius = formation_elements.radius
            self.angle = formation_elements.angle
            self.max_offset = formation_elements.max_offset
            self.noise_scale = formation_elements.noise_scale
            self.angle_noise_scale = formation_elements.angle_noise_scale
            self.formation_type = formation_elements.formation_type
            self.traj = np.asarray(traj if traj is not None else formation_elements.traj)
        else:
            self.member_num = member_num
            self.radius = radius
            self.angle = angle
            self.max_offset = max_offset
            self.noise_scale = noise_scale
            self.angle_noise_scale = angle_noise_scale
            self.formation_type = formation_type
            if traj is None:
                raise ValueError("traj参数必须提供，或者在formation_elements中指定")
            self.traj = np.asarray(traj)

    def generate_formation_offsets(self):
        """生成二维队形偏移 (x_offs, y_offs)"""
        N = self.member_num
        if self.formation_type == 'circular':
            θ = np.linspace(0, 2 * np.pi, N, endpoint=False)
            return self.radius * np.cos(θ), self.radius * np.sin(θ)
        elif self.formation_type == 'vertical':
            return np.zeros(N), np.linspace(-self.max_offset, self.max_offset, N)
        elif self.formation_type == 'horizontal':
            return np.linspace(-self.max_offset, self.max_offset, N), np.zeros(N)
        elif self.formation_type == 'vshape':
            half = N // 2
            xs, ys = [], []
            for i in range(-half, half + 1):
                α = self.angle + np.random.uniform(-self.angle_noise_scale, self.angle_noise_scale)
                off = self.max_offset * abs(i) / half
                xs.append(off * np.cos(np.deg2rad(α)) * np.sign(i))
                ys.append(-off * np.sin(np.deg2rad(α )))
            return np.array(xs), np.array(ys)
        elif self.formation_type == 'arc':
            θ = np.linspace(np.deg2rad(self.angle),
                            np.pi - np.deg2rad(self.angle),
                            N)
            return self.radius * np.cos(θ), self.radius * np.sin(θ)
        else:
            raise ValueError(f"未知队形类型：{self.formation_type}")

    def compute_orientation_matrices(self):
        """计算每个轨迹点的局部坐标系（修复垂直轨迹问题）"""
        pts = self.traj
        N = len(pts)
        Rs = []
        up = np.array([0., 0., 1.])

        # 用于处理连续共线点的前一个有效方向
        last_valid_R = None

        for i in range(N):
            # 计算切向量
            if i < N - 1:
                v = pts[i + 1] - pts[i]
            else:
                v = pts[i] - pts[i - 1]

            v_norm = np.linalg.norm(v)

            # 处理零向量情况（连续相同的点）
            if v_norm < 1e-8:
                if last_valid_R is not None:
                    Rs.append(last_valid_R)
                else:
                    # 如果还没有有效的R，使用默认坐标系
                    Rs.append(np.eye(3))
                continue

            T = v / v_norm  # 单位切向量

            # 检查 T 是否与 up 接近平行（避免万向锁）
            dot_product = abs(np.dot(T, up))

            if dot_product > 0.999:  # 接近平行的阈值
                # 使用备用向量来构建坐标系
                # 如果 T 接近垂直，使用 X 轴或 Y 轴作为参考
                if abs(T[0]) < 0.9:  # T 不平行于 X 轴
                    alt_vec = np.array([1., 0., 0.])
                else:  # T 接近 X 轴，使用 Y 轴
                    alt_vec = np.array([0., 1., 0.])

                R_vec = np.cross(alt_vec, T)
                R_vec = R_vec / (np.linalg.norm(R_vec) + 1e-8)
                U = np.cross(T, R_vec)
            else:
                # 正常情况：T 不平行于 up
                R_vec = np.cross(up, T)
                R_vec = R_vec / (np.linalg.norm(R_vec) + 1e-8)
                U = np.cross(T, R_vec)

            # 确保构成右手坐标系
            U = U / (np.linalg.norm(U) + 1e-8)
            R_mat = np.column_stack((R_vec, U, T))  # 列为 body-x/y/z

            # 验证旋转矩阵的正交性
            if np.abs(np.linalg.det(R_mat) - 1) > 0.1:
                # 如果行列式不接近1，使用 Gram-Schmidt 正交化
                R_vec = R_vec / np.linalg.norm(R_vec)
                T = T / np.linalg.norm(T)
                U = np.cross(T, R_vec)
                U = U / np.linalg.norm(U)
                R_mat = np.column_stack((R_vec, U, T))

            Rs.append(R_mat)
            last_valid_R = R_mat

        return np.array(Rs)  # shape (N,3,3)

    def generate_members_formation_3d(self):
        """
        生成每个从机的世界坐标轨迹列表
        :return: list of lists, each shape (N,3)
        """
        x_offs, y_offs = self.generate_formation_offsets()
        Rs = self.compute_orientation_matrices()
        N = len(self.traj)
        paths = [np.zeros((N, 3)) for _ in range(self.member_num)]

        for i in range(N):
            base = self.traj[i]
            if i < N - 1:
                Rm = Rs[i]
            else:
                Rm = Rs[-1]

            for m in range(self.member_num):
                # 在 body 坐标系中，Right = x_offs, Forward = y_offs
                p_body = np.array([x_offs[m], 0., y_offs[m]])
                # 旋转到 world，再平移，加噪声
                p_w = Rm.dot(p_body) + base \
                      + np.random.normal(scale=self.noise_scale, size=3)
                paths[m][i] = p_w

        return [path.tolist() for path in paths]

    def plot_formation(self):
        """3D 可视化主机与编队从机轨迹与机体坐标系"""
        paths = [np.asarray(p) for p in self.generate_members_formation_3d()]
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # 画主机轨迹
        ax.plot(*self.traj.T, '-k', label='Leader', linewidth=2)

        # 画从机轨迹
        for m, path in enumerate(paths):
            ax.plot(*path.T, color=plt.cm.tab10(m % 10),
                    label=f'Member {m}', alpha=0.8)

        # 可选：在每隔若干帧绘制机体坐标系
        Rs = self.compute_orientation_matrices()
        scale = self.radius * 0.2
        for i in range(0, len(self.traj), max(1, len(self.traj) // 10)):
            base = self.traj[i]
            Rm = Rs[min(i, len(Rs) - 1)]
            right, up_vec, forward = Rm[:, 0], Rm[:, 1], Rm[:, 2]
            ax.quiver(*base, *right, length=scale, color='r',
                      normalize=True, arrow_length_ratio=0.2)
            ax.quiver(*base, *up_vec, length=scale, color='g',
                      normalize=True, arrow_length_ratio=0.2)
            ax.quiver(*base, *forward, length=scale, color='b',
                      normalize=True, arrow_length_ratio=0.2)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.tight_layout()
        plt.show()
