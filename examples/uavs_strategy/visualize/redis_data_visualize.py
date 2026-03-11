import redis
import time
import json
import numpy as np
import os.path as osp
import os
import sys

# matplotlib 必须使用 FigureCanvasQTAgg 嵌入到 PyQt5
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QWidget, QTextEdit)
from PyQt5.QtCore import Qt, QTimer

from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import DT
from examples.uavs_strategy.uav_dynamic_agents02 import switch_config


class MapVisualizer:
    """
    地图可视化核心类，负责从Redis中读取无人机数据并绘制地图。
    移除了原有的自定义框选逻辑，完全交由 Matplotlib 原生 Toolbar 处理缩放和平移。
    """
    def __init__(self, redis_host='127.0.0.1', redis_port=6379, facilities_file=None):
        self.redis_host = redis_host
        self.redis_port = redis_port
        self.r = redis.StrictRedis(host=self.redis_host, port=self.redis_port, db=0)

        self.facilities = self._default_facilities(facilities_file)
        
        # 保存最新获取的数据供 UI 面板显示
        self.latest_positions = {}
        self.latest_extra_info = {}
        
        # 标记是否为第一次绘制，用于初始化坐标系范围
        self.first_draw = True 

    def _default_facilities(self, default_json_path=None):
        if default_json_path is None:
            _facilities_info_json = osp.join(bfunc.WS_ROOT, 'data', 'test_facilities_locations.json')
            print(f"Using default facilities file: {_facilities_info_json}")
        else:
            _facilities_info_json = default_json_path

        with open(_facilities_info_json, 'r') as f:
            _facilities_info = json.load(f)

        return bfunc.Facilities(_facilities_info['facilities_str'], _facilities_info['defence_rings'])

    def get_drone_states(self, blue=True):
        ids = self.r.smembers("uav:ids" if blue else "red:ids")
        ids = [uid.decode('utf-8') for uid in ids]

        positions = {}
        traj_data = {}
        ref_traj_data = {}
        lookahead_data = {}
        extra_info_data = {}

        for uid in ids:
            pos = self.r.get(f"uav:{uid}:pos")
            traj_raw = self.r.get(f"uav:{uid}:traj")
            ref_raw = self.r.get(f"uav:{uid}:ref_traj")
            lookahead_raw = self.r.get(f"uav:{uid}:lookahead")
            extra_raw = self.r.get(f"uav:{uid}:traj_extra")

            if pos:
                positions[uid] = json.loads(pos.decode('utf-8'))
            if traj_raw:
                traj_data[uid] = json.loads(traj_raw.decode('utf-8'))
            if ref_raw:
                ref_traj_data[uid] = json.loads(ref_raw.decode('utf-8'))
            if lookahead_raw:
                try:
                    lookahead_data[uid] = int(lookahead_raw.decode('utf-8'))
                except ValueError:
                    pass
            if extra_raw:
                extra_list = json.loads(extra_raw.decode('utf-8'))
                if extra_list:
                    extra_info_data[uid] = extra_list[-1]  # 取最新一条

        return positions, traj_data, ref_traj_data, lookahead_data, extra_info_data

    def plot_facilities(self, ax):
        if self.facilities:
            for _fac, _utm_xy in self.facilities.antiairs.items():
                ax.plot(_utm_xy[0], _utm_xy[1], 'ro', label=f'{_fac} Antiair')
            for _fac, _utm_xy in self.facilities.headquartors.items():
                ax.plot(_utm_xy[0], _utm_xy[1], 'bo', label=f'{_fac} Headquarters')
            for _fac, _utm_xy in self.facilities.probers.items():
                ax.plot(_utm_xy[0], _utm_xy[1], 'go', label=f'{_fac} Prober')
            for _fac, _utm_xy in self.facilities.defend_rings.items():
                ax.fill(_utm_xy[: ,0], _utm_xy[: ,1], alpha=0.2, label=f'{_fac} Defence Ring')

    def update_plot(self, ax, blue=True):
        """核心绘图函数，由 QTimer 定期调用"""
        
        # 如果不是第一次绘制，先记录当前的缩放/平移视角范围
        if not self.first_draw:
            current_xlim = ax.get_xlim()
            current_ylim = ax.get_ylim()

        ax.clear()
        
        ax.set_title(f"{'Blue' if blue else 'Red'} UAVs - Real-time Monitor")
        ax.set_xlabel('X (UTM)')
        ax.set_ylabel('Y (UTM)')

        positions, traj_data, ref_traj_data, lookahead_data, extra_info_data = self.get_drone_states(blue)
        self.latest_positions = positions  # 保存供 UI 使用
        self.latest_extra_info = extra_info_data  # 保存供 UI 面板使用
        
        now_ms = int(time.time() * 1000)
        self.plot_facilities(ax)

        for uid, pos in positions.items():
            is_active = False
            if 'ts' in pos and abs(now_ms - pos['ts']) < 2000:
                is_active = True

            alpha_val = 1.0 if is_active else 0.4
            text_color = 'black' if is_active else 'gray'
            
            ax.plot(pos['x'], pos['y'], 'go', markersize=8, alpha=alpha_val, label=f'{uid} Pos')

            # 构建地图上的标注文本：uid + extra_info 摘要
            extra = extra_info_data.get(uid, {})
            map_label = uid
            if extra:
                seg = extra.get('segment_key', '')
                fmt = extra.get('formation_type', '')
                fid = extra.get('frame_id', '')
                waiting = extra.get('is_waiting', False)
                status_tag = '[W]' if waiting else ''
                map_label = f"{uid} {status_tag}\nSeg:{seg} F:{fmt}\nFrame:{fid}"
            ax.text(pos['x'], pos['y'], map_label, fontsize=7, color=text_color,
                    fontweight='bold', alpha=alpha_val, va='bottom')

            if is_active:
                ref_traj = ref_traj_data.get(uid, [])
                if ref_traj:
                    ref_arr = np.array(ref_traj)
                    ax.plot(ref_arr[:, 0], ref_arr[:, 1], '--', linewidth=1, color='orange', alpha=0.7)

                lh_idx = lookahead_data.get(uid)
                if lh_idx is not None and ref_traj and 0 <= lh_idx < len(ref_traj):
                    lh_pt = ref_traj[lh_idx]
                    ax.plot(lh_pt[0], lh_pt[1], 'm*', markersize=10, label=f'{uid} Lookahead')

        ax.legend(loc='best')

        # === 恢复视角范围逻辑 ===
        if self.first_draw:
            # 第一次绘制：使用自动计算的全局范围
            xmin, xmax, ymin, ymax = self.compute_static_range(positions)
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)
            self.first_draw = False
        else:
            # 后续绘制：强制应用刚才记录的视角，让 Matplotlib 工具栏的缩放不被 ax.clear() 重置
            ax.set_xlim(current_xlim)
            ax.set_ylim(current_ylim)

    def handle_click(self, event):
        """保留：点击地图输出坐标功能"""
        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return
        print(f"Clicked coordinates: ({event.xdata:.13f}, {event.ydata:.13f})")

    def compute_static_range(self, positions=None, buffer=3000):
        xs, ys = [], []
        for d in [self.facilities.antiairs, self.facilities.headquartors, self.facilities.probers]:
            for _name, utm_xy in d.items():
                xs.append(utm_xy[0])
                ys.append(utm_xy[1])
        for _name, poly in self.facilities.defend_rings.items():
            xs.extend(poly[:, 0])
            ys.extend(poly[:, 1])
        if positions:
            for _uid, pos in positions.items():
                xs.append(pos["x"])
                ys.append(pos["y"])
        
        if not xs or not ys:
            return 0, 10000, 0, 10000
            
        return min(xs) - buffer, max(xs) + buffer, min(ys) - buffer, max(ys) + buffer


class RealTimeRedisVisualizerApp(QMainWindow):
    """
    PyQt5 主窗口类，整合了 MapVisualizer 和 UI 控件
    """
    def __init__(self, facilities_file):
        super().__init__()
        self.setWindowTitle("Redis 实时无人机轨迹监控系统")
        self.setGeometry(100, 100, 1400, 800)

        # 核心可视化逻辑
        self.visualizer = MapVisualizer(facilities_file=facilities_file)
        self.is_playing = True

        # 定时器用于实时刷新
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

        self.init_ui()
        # 启动定时器 (每 500 毫秒更新一次)
        self.timer.start(500)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # --- 左侧：绘图区域 ---
        left_layout = QVBoxLayout()
        
        # 顶部工具栏
        top_bar = QHBoxLayout()
        self.btn_play = QPushButton("暂停实时刷新")
        self.btn_play.clicked.connect(self.toggle_play)
        top_bar.addWidget(self.btn_play)
        top_bar.addStretch()
        left_layout.addLayout(top_bar)

        # Matplotlib 画布
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        
        # 添加标准工具栏 (提供原生的拖拽平移、框选放大、Home复位、保存截图等功能)
        self.toolbar = NavigationToolbar(self.canvas, self)
        left_layout.addWidget(self.toolbar)
        left_layout.addWidget(self.canvas)

        # 仅绑定点击输出坐标事件（移除了自定义的 RectangleSelector 交互绑定）
        self.canvas.mpl_connect('button_press_event', self.visualizer.handle_click)

        layout.addLayout(left_layout, stretch=4)

        # --- 右侧：状态信息面板 ---
        self.info_panel = QVBoxLayout()
        self.info_label = QLabel("实时无人机状态监控")
        self.info_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        
        self.info_display = QTextEdit()
        self.info_display.setReadOnly(True)
        self.info_display.setPlaceholderText("等待数据接入...")
        
        self.info_panel.addWidget(self.info_label)
        self.info_panel.addWidget(self.info_display)
        
        layout.addLayout(self.info_panel, stretch=1)

    def toggle_play(self):
        self.is_playing = not self.is_playing
        if self.is_playing:
            self.btn_play.setText("暂停实时刷新")
            self.timer.start(500)
        else:
            self.btn_play.setText("恢复实时刷新")
            self.timer.stop()

    def update_frame(self):
        """定时器回调，触发重绘和状态面板更新"""
        self.visualizer.update_plot(self.ax, blue=True)
        self.canvas.draw()
        
        # 更新侧边状态栏
        self.update_info_panel()

    def update_info_panel(self):
        positions = self.visualizer.latest_positions
        extra_info_data = getattr(self.visualizer, 'latest_extra_info', {})
        if not positions:
            self.info_display.setText("当前暂无活跃的无人机数据。")
            return

        now_ms = int(time.time() * 1000)
        info_text = f"--- 更新时间: {time.strftime('%H:%M:%S')} ---\n\n"
        
        sorted_uids = sorted(positions.keys())
        
        for uid in sorted_uids:
            pos = positions[uid]
            is_active = False
            if 'ts' in pos and abs(now_ms - pos['ts']) < 2000:
                is_active = True

            status_str = "活跃" if is_active else "离线/静止"
            
            info_text += f"【无人机: {uid}】 ({status_str})\n"
            info_text += f"  位置: ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f})\n"

            # 显示 extra_info 详细字段
            extra = extra_info_data.get(uid, {})
            if extra:
                seg_key = extra.get('segment_key', 'N/A')
                formation = extra.get('formation_type', 'N/A')
                frame_id = extra.get('frame_id', 'N/A')
                lookahead = extra.get('lookahead', 'N/A')
                global_id = extra.get('global_id', 'N/A')
                my_ack = extra.get('my_ack', 'N/A')
                is_waiting = extra.get('is_waiting', False)
                wait_msg = extra.get('wait_message', '')
                dist = extra.get('dist_to_target', None)
                siblings = extra.get('cur_siblings_ids', [])
                lh_coord = extra.get('lookahead_coord', None)
                leader_id = extra.get('leader_id', None)

                info_text += f"  航段: {seg_key}\n"
                info_text += f"  编队类型: {formation}\n"
                info_text += f"  帧ID/ACK: {frame_id} / {my_ack}\n"
                info_text += f"  Lookahead: {lookahead}"
                if lh_coord:
                    info_text += f"  -> ({lh_coord[0]:.1f}, {lh_coord[1]:.1f})"
                info_text += "\n"
                info_text += f"  全局步数: {global_id}\n"
                if dist is not None:
                    info_text += f"  距目标: {dist:.2f}m\n"
                if is_waiting:
                    info_text += f"  ⏳ 等待中: {wait_msg}\n"
                if siblings:
                    info_text += f"  同组成员: {', '.join(str(s) for s in siblings)}\n"
                if leader_id is not None:
                    info_text += f"  领队ID: {leader_id}\n"
            else:
                info_text += "  (暂无额外状态信息)\n"

            info_text += "-"*30 + "\n"

        self.info_display.setText(info_text)


if __name__ == "__main__":
    # python -m examples.uavs_strategy.visualize.redis_data_visualize
    current_dir = os.path.dirname(os.path.dirname(__file__))
    if switch_config == 1:
        facilities_file_name = 'facilities.json'
    elif switch_config == 2:
        facilities_file_name = 'test_facilities_locations.json'
    elif switch_config == 3:
        facilities_file_name = 'facilities.json'
    else:
        facilities_file_name = 'facilities.json'
        
    facilities_file = os.path.join(current_dir, "data", facilities_file_name)
    
    app = QApplication(sys.argv)
    window = RealTimeRedisVisualizerApp(facilities_file=facilities_file)
    window.show()
    sys.exit(app.exec_())