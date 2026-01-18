import redis
import time
import matplotlib.pyplot as plt
import json
import numpy as np
import os.path as osp
import os
from matplotlib.widgets import RectangleSelector
from matplotlib.animation import FuncAnimation
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import DT
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
        self.custom_xlim = None
        self.custom_ylim = None
        self.is_zoomed = False
        self.zoom_selector = None
        self.zoom_margin = 200
        self._fig = None
        self._ax = None


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
            traj_raw = self.r.get(f"uav:{uid}:traj")
            print(f"get uav:{uid}:pos: {pos}, get uav:{uid}:traj: {traj_raw}")
            if pos:
                positions[uid] = json.loads(pos.decode('utf-8'))  # 解码并存储位置信息
            if traj_raw:
                # String -> JSON list，例如 [[x,y,z], [x,y,z], ...]
                traj_list = json.loads(traj_raw.decode('utf-8'))
                traj_data[uid] = traj_list
        print(f"UAV {'Blue' if blue else 'Red'} Positions: {positions}")
        return positions, traj_data

    def get_drone_states(self, blue=True):
        """
        读取 UAV 当前状态：
        - 当前位置 pos
        - 实际轨迹 traj
        - 预设轨迹 ref_traj
        - 预设轨迹当前索引 lookahead
        """
        ids = self.r.smembers("uav:ids" if blue else "red:ids")
        ids = [uid.decode('utf-8') for uid in ids]

        positions = {}
        traj_data = {}
        ref_traj_data = {}
        lookahead_data = {}

        for uid in ids:
            pos = self.r.get(f"uav:{uid}:pos")
            traj_raw = self.r.get(f"uav:{uid}:traj")  # 你现在用 String(JSON) 存轨迹
            ref_raw = self.r.get(f"uav:{uid}:ref_traj")
            lookahead_raw = self.r.get(f"uav:{uid}:lookahead")

            if pos:
                positions[uid] = json.loads(pos.decode('utf-8'))

            if traj_raw:
                traj_data[uid] = json.loads(traj_raw.decode('utf-8'))  # [[x,y,z], ...]

            if ref_raw:
                ref_traj_data[uid] = json.loads(ref_raw.decode('utf-8'))  # [[x,y,z], ...]

            if lookahead_raw:
                try:
                    lookahead_data[uid] = int(lookahead_raw.decode('utf-8'))
                except ValueError:
                    pass

        return positions, traj_data, ref_traj_data, lookahead_data

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
            for _fac, _utm_xy in self.facilities.defend_rings.items():
                ax.fill(_utm_xy[: ,0], _utm_xy[: ,1], alpha=0.2, label=f'{_fac} Defence Ring')

    def update_plot(self, frame, ax, blue=True):
        ax.clear()  # 清除上一帧
        self._restore_zoom_artists(ax)
        
        # 设置标题和坐标轴
        ax.set_title(f"{'Blue' if blue else 'Red'} UAVs - Real-time Monitor")
        ax.set_xlabel('X (UTM)')
        ax.set_ylabel('Y (UTM)')

        positions, traj_data, ref_traj_data, lookahead_data = self.get_drone_states(blue)
        
        # 获取当前时间戳（毫秒）
        now_ms = int(time.time() * 1000)

        # 画设施
        self.plot_facilities(ax)

        for uid, pos in positions.items():
            # 判断该无人机是否“活跃” (2秒内有更新)
            is_active = False
            if 'ts' in pos:
                if abs(now_ms - pos['ts']) < 2000:
                    is_active = True

            # 1. 实际飞行轨迹（历史） - 始终显示
            # 即使是停止的无人机，也保留其轨迹线
            traj = traj_data.get(uid, [])
            if traj:
                traj_arr = np.array(traj)  # [[x,y,z],...]
                # 活跃的用实线，非活跃的可以用虚线或透明度区别，这里统一用实线
                style = '-' 
                alpha = 1.0 if is_active else 0.4  # 非活跃的变淡
                ax.plot(traj_arr[:, 0], traj_arr[:, 1],
                        style, linewidth=1.5, alpha=alpha, label=f'{uid} Path')

            # 2. 当前无人机位置
            # 活跃状态正常显示，不活跃状态变浅
            alpha_val = 1.0 if is_active else 0.4
            text_color = 'black' if is_active else 'gray'
            
            ax.plot(pos['x'], pos['y'], 'go', markersize=8, alpha=alpha_val, label=f'{uid} Pos')
            # 添加文字标签
            ax.text(pos['x'], pos['y'], uid, fontsize=9, color=text_color, fontweight='bold', alpha=alpha_val)

            # 3. 预设轨迹 - 只显示活跃的
            if is_active:
                ref_traj = ref_traj_data.get(uid, [])
                if ref_traj:
                    ref_arr = np.array(ref_traj)  # [[x,y,z],...]
                    ax.plot(ref_arr[:, 0], ref_arr[:, 1],
                            '--', linewidth=1, color='orange', alpha=0.7)

                # 4. 绘制 Lookahead 点
                lh_idx = lookahead_data.get(uid)
                if lh_idx is not None and ref_traj and 0 <= lh_idx < len(ref_traj):
                    lh_pt = ref_traj[lh_idx]  # [x, y, z]
                    # 用紫色星号表示预瞄点
                    ax.plot(lh_pt[0], lh_pt[1], 'm*', markersize=10, label=f'{uid} Lookahead')
        
        # 避免图例过多，可以只显示一部分或者不显示
        # ax.legend(loc='upper right', fontsize='small')


        # 坐标轴信息
        ax.set_title(f"{'Blue' if blue else 'Red'} UAVs - Realtime Map")
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.legend(loc='best')

        if self.is_zoomed and self.custom_xlim and self.custom_ylim:
            ax.set_xlim(*self.custom_xlim)
            ax.set_ylim(*self.custom_ylim)
        else:
            xmin, xmax, ymin, ymax = self.compute_static_range(positions)
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)

        return ax

    def handle_click(self, event):
        """在鼠标点击点位置打印xy坐标，并保留小数位精度。
        """

        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return

        print(f"Clicked coordinates: ({event.xdata:.13f}, {event.ydata:.13f})")

    def init_interaction(self, fig, ax):
        """初始化框选放大与视角重置的交互。"""
        self._fig = fig
        self._ax = ax
        if self.zoom_selector is None:
            self.zoom_selector = RectangleSelector(
                ax,
                self.on_select,
                useblit=True,
                button=[1],
                interactive=True,
                drag_from_anywhere=True,
            )
        fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    def on_select(self, eclick, erelease):
        if eclick.xdata is None or erelease.xdata is None:
            return
        xmin, xmax = sorted([eclick.xdata, erelease.xdata])
        ymin, ymax = sorted([eclick.ydata, erelease.ydata])
        if abs(xmax - xmin) < 1e-6 or abs(ymax - ymin) < 1e-6:
            return
        pad = self.zoom_margin
        self.custom_xlim = (xmin - pad, xmax + pad)
        self.custom_ylim = (ymin - pad, ymax + pad)
        self.is_zoomed = True
        if self._ax is not None:
            self._ax.set_xlim(*self.custom_xlim)
            self._ax.set_ylim(*self.custom_ylim)
            if self._fig is not None:
                self._fig.canvas.draw_idle()

    def on_key_press(self, event):
        if event.key in ('r', 'escape'):
            self.is_zoomed = False
            self.custom_xlim = None
            self.custom_ylim = None
            if self._fig is not None:
                self._fig.canvas.draw_idle()

    def _restore_zoom_artists(self, ax):
        if not self.zoom_selector:
            return
        selection_artist = getattr(self.zoom_selector, "_selection_artist", None)
        if selection_artist is not None and selection_artist not in ax.patches:
            ax.add_patch(selection_artist)
            selection_artist.set_visible(False)
        handles = getattr(self.zoom_selector, "_handles_artists", None)
        if handles:
            for artist in handles:
                if artist not in ax.get_children():
                    ax.add_artist(artist)

    def compute_static_range(self, positions = None , buffer = 3000):
        """
        自动根据设施与无人机位置计算坐标轴范围。
        positions: UAV 的实时位置 dict
        """
        xs = []
        ys = []

        # ① 加入设施坐标
        for d in [
            self.facilities.antiairs,
            self.facilities.headquartors,
            self.facilities.probers,
        ]:
            for _name, utm_xy in d.items():
                xs.append(utm_xy[0])
                ys.append(utm_xy[1])

        # ② 加入防御圈（polygon）
        for _name, poly in self.facilities.defend_rings.items():
            xs.extend(poly[:, 0])
            ys.extend(poly[:, 1])

        # ③ 加入 UAV 位置
        if positions:
            for _uid, pos in positions.items():
                xs.append(pos["x"])
                ys.append(pos["y"])

        # ④ 计算范围 + buffer
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

          # 自定义 buffer，避免贴边

        return xmin - buffer, xmax + buffer, ymin - buffer, ymax + buffer


# 运行可视化，并实现动态更新
if __name__ == "__main__":
	    # 运行：python -m examples.uavs_strategy.redis_data_visualize
    current_dir = os.path.dirname(__file__)
    # facilities_file_name = 'facilities.json'
    facilities_file_name = 'test_facilities_locations.json'
    facilities_file = os.path.join(current_dir,"data" ,facilities_file_name)
    visualizer = MapVisualizer(facilities_file=facilities_file)

    # 创建绘图和轴
    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('button_press_event', visualizer.handle_click)
    visualizer.init_interaction(fig, ax)

    # 使用FuncAnimation动态更新图像，每秒更新一次
    ani = FuncAnimation(fig, visualizer.update_plot, fargs=(ax, True), interval=DT*1000, cache_frame_data=False)  # 每秒更新一次
    plt.show()


