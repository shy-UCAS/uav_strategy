import redis
import matplotlib.pyplot as plt
import json
import numpy as np
from matplotlib.animation import FuncAnimation
import os.path as osp
from examples.uavs_strategy.planning_modules import basic_functions as bfunc


class MapVisualizer:
    """
    地图可视化类，负责从Redis中读取无人机数据并绘制实时地图。
    """

    def __init__(self, redis_host='127.0.0.1', redis_port=6379, facilities_file=None):
        """
        初始化MapVisualizer类。

        :param redis_host: Redis服务器的主机地址，默认为127.0.0.1。
        :param redis_port: Redis服务器的端口，默认为6379。
        :param facilities_file: 设施文件路径，默认为None。
        """
        # Redis配置
        self.redis_host = redis_host
        self.redis_port = redis_port
        self.r = redis.StrictRedis(host=self.redis_host, port=self.redis_port, db=0)
        self.r.flushdb()

        # 加载设施信息
        self.facilities = self._default_facilities(facilities_file)

    def _default_facilities(self, default_json_path=None):
        """
        加载设施的默认数据。

        :param default_json_path: 设施数据的JSON文件路径，默认为None。
        :return: 返回一个设施对象。
        """
        # 默认路径：设施信息数据
        if default_json_path is None:
            _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')
            print(f"Using default facilities file: {_facilities_info_json}")
        else:
            _facilities_info_json = default_json_path

        with open(_facilities_info_json, 'r') as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(_facilities_info['facilities_str'], _facilities_info['defence_rings'])

    def get_drone_positions_and_traj(self, blue=True):
        """
        获取蓝方或红方无人机的当前位置信息和历史轨迹数据。

        :param blue: 是否获取蓝方数据，默认为True。
        :return: 返回无人机的位置信息和轨迹数据。
        """
        # 获取UAV的ID
        ids = self.r.smembers("uav:ids" if blue else "red:ids")
        print(f"UAV {'Blue' if blue else 'Red'} IDs: {ids}")
        ids = [uid.decode('utf-8') for uid in ids]


        # 获取每个UAV的当前位置和轨迹数据
        positions = {}
        traj_data = {}
        for uid in ids:
            pos = self.r.get(f"uav:{uid}:pos")
            traj = self.r.lrange(f"uav:{uid}:traj", 0, -1)  # 获取历史轨迹数据

            if pos:
                positions[uid] = json.loads(pos.decode('utf-8'))  # 解码并存储位置信息
            if traj:
                traj_data[uid] = [json.loads(point.decode('utf-8')) for point in traj]  # 解码并存储轨迹数据
        print(f"UAV {'Blue' if blue else 'Red'} Positions: {positions}")
        return positions, traj_data

    def visualize(self, blue=True):
        """
        可视化当前蓝方或红方无人机的位置信息和轨迹。

        :param blue: 是否显示蓝方数据，默认为True。
        """
        # 获取UAV的当前位置和轨迹数据
        positions, traj_data = self.get_drone_positions_and_traj(blue)

        # 创建绘图
        fig, ax = plt.subplots()

        # 绘制设施信息
        self.plot_facilities(ax)

        # 绘制UAV的位置和轨迹
        for uid, pos in positions.items():
            # 绘制当前位置信息
            ax.plot(pos['x'], pos['y'], 'go', label=f'{uid} Position')

            # 绘制历史轨迹
            traj = traj_data.get(uid, [])
            if traj:
                traj = np.array(traj)
                ax.plot(traj[:, 0], traj[:, 1], label=f'{uid} Trajectory')

        ax.set_title(f"{'Blue' if blue else 'Red'} UAVs - Positions and Trajectories")
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.legend()
        plt.show()

    def plot_facilities(self, ax):
        """
        绘制设施信息和防御圈。

        :param ax: Matplotlib的轴对象，用于绘制设施。
        """
        if self.facilities:
            # 绘制防空设施
            for _fac, _utm_xy in self.facilities.antiairs.items():
                ax.plot(_utm_xy[0], _utm_xy[1], 'ro', label=f'{_fac} Antiair')

            # 绘制指挥所设施
            for _fac, _utm_xy in self.facilities.headquartors.items():
                ax.plot(_utm_xy[0], _utm_xy[1], 'bo', label=f'{_fac} Headquarters')

            # 绘制探测设施
            for _fac, _utm_xy in self.facilities.probers.items():
                ax.plot(_utm_xy[0], _utm_xy[1], 'go', label=f'{_fac} Prober')

            # 绘制防御环（如果需要）
            for ring in self.facilities.defend_rings.values():
                # 防御圈应只包含两个值，确保它们是单一的坐标对
                if isinstance(ring, tuple) and len(ring) == 2:
                    # 绘制每个防御圈
                    circle = plt.Circle((ring[0], ring[1]), 1000, color='r', fill=False)  # radius假设为1000
                    ax.add_artist(circle)

    def update_plot(self, frame, ax, blue=True):
        """
        每次更新时重新绘制地图。

        :param frame: 动画帧数。
        :param ax: Matplotlib的轴对象，用于更新绘图。
        :param blue: 是否绘制蓝方数据，默认为True。
        :return: 更新后的轴对象。
        """
        ax.clear()  # 清除当前的图像
        positions, traj_data = self.get_drone_positions_and_traj(blue)

        # 绘制设施信息
        self.plot_facilities(ax)

        # 绘制UAV的位置和轨迹
        for uid, pos in positions.items():
            # 绘制当前位置信息
            ax.plot(pos['x'], pos['y'], 'go', label=f'{uid} Position')

            # 绘制历史轨迹
            traj = traj_data.get(uid, [])
            if traj:
                traj = np.array(traj)
                ax.plot(traj[:, 0], traj[:, 1], label=f'{uid} Trajectory')

        ax.set_title(f"{'Blue' if blue else 'Red'} UAVs - Positions and Trajectories")
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.legend()

        return ax


# 运行可视化，并实现动态更新
if __name__ == "__main__":
    visualizer = MapVisualizer()

    # 创建绘图和轴
    fig, ax = plt.subplots()

    # 使用FuncAnimation动态更新图像，每秒更新一次
    ani = FuncAnimation(fig, visualizer.update_plot, fargs=(ax, True), interval=1000)  # 每秒更新一次
    plt.show()
