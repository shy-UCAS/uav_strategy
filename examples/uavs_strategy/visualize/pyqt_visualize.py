import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.uav_dynamic_agents02 import facilities_file as _default_facilities_file
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QPushButton, QCheckBox, QSlider, QLabel, QFileDialog, QWidget, QTextEdit, QSplitter)
from PyQt5.QtCore import Qt, QTimer

class UAVVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV 轨迹交互可视化系统")
        self.setGeometry(100, 100, 1200, 800)
        # 经纬度与UTM坐标转换器
        self._lnglat2utm_convertor = bfunc.LngLat2UTM()

        # 数据存储
        self.data = {}
        self.uav_ids = []
        self.current_step = 0
        self.max_steps = 0
        self.is_playing = False
        self.is_utm = False
        self.facilities = None  # 用于存储地图设施数据

        # 定时器用于动画播放
        self.timer = QTimer()
        self.timer.timeout.connect(self.next_step)

        self.init_ui()

        # 自动加载当前 switch_config 对应的默认地图
        self._load_default_map()

    def _load_default_map(self):
        """尝试加载 agent02 的 switch_config 对应的 facilities_file 作为默认地图"""
        try:
            with open(_default_facilities_file, 'r', encoding='utf-8') as f:
                map_data = json.load(f)
            self.load_map_data(map_data)
            print(f"已自动加载默认地图: {_default_facilities_file}")
        except FileNotFoundError:
            print(f"默认地图文件不存在，请手动加载: {_default_facilities_file}")
        except Exception as e:
            print(f"加载默认地图失败: {e}，请手动加载地图数据文件")

    def init_ui(self):
        # 主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # --- 左侧：绘图区域与控制 ---
        left_layout = QVBoxLayout()
        
        # 引入 QSplitter 来支持鼠标拖拽改变上下区域的高度
        left_splitter = QSplitter(Qt.Vertical)

        # 把原本的上半部分（画图、控制栏）打包进一个 Widget
        upper_left_widget = QWidget()
        upper_left_layout = QVBoxLayout(upper_left_widget)
        upper_left_layout.setContentsMargins(0, 0, 0, 0)
        
        # 顶部工具栏
        top_bar = QHBoxLayout()
        self.btn_load = QPushButton("上传可视化文件")
        self.btn_load.clicked.connect(self.open_file)
        # 加载地图json数据（默认已从 agent02 自动加载，可手动切换）
        self.btn_map = QPushButton("切换地图数据文件")
        self.btn_map.clicked.connect(self.open_map_file)
        # 坐标系切换开关：勾选 = UTM 坐标，取消 = 经纬度（点击立即刷新绘图与数字显示）
        self.cb_utm = QCheckBox("UTM 坐标")
        self.cb_utm.stateChanged.connect(lambda _: self.toggle_coord_system())

        top_bar.addWidget(self.btn_load)
        top_bar.addWidget(self.btn_map)
        top_bar.addWidget(self.cb_utm)
        top_bar.addStretch()
        upper_left_layout.addLayout(top_bar)

        # Matplotlib 画布
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        
        # --- 新增的工具栏 ---
        self.toolbar = NavigationToolbar(self.canvas, self)
        upper_left_layout.addWidget(self.toolbar)

        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("UAV Trajectory Visualization")
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.grid(True)
        # 通过设置 stretch=1（权重比例），让画布占据组件伸缩时所有多余的垂直空间
        upper_left_layout.addWidget(self.canvas, stretch=1)

        # 底部控制栏
        controls = QHBoxLayout()
        self.btn_prev = QPushButton("后退")
        self.btn_play = QPushButton("播放")
        self.btn_next = QPushButton("前进")
        
        self.btn_prev.clicked.connect(self.prev_step)
        self.btn_play.clicked.connect(self.toggle_play)
        self.btn_next.clicked.connect(self.next_step)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.slider_moved)
        
        self.step_label = QLabel("Step: 0/0")

        controls.addWidget(self.btn_prev)
        controls.addWidget(self.btn_play)
        controls.addWidget(self.btn_next)
        controls.addWidget(self.slider)
        controls.addWidget(self.step_label)
        upper_left_layout.addLayout(controls)

        # 下半部分：底部日志显示区域（剥离了限定高度，支持缩放）
        lower_left_widget = QWidget()
        self.log_panel = QVBoxLayout(lower_left_widget)
        self.log_panel.setContentsMargins(0, 0, 0, 0)
        
        self.log_label = QLabel("当前帧日志输出 (Log)")
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setPlaceholderText("等待加载包含执行日志的数据...")
        # 移除了 setMaximumHeight，由 Splitter 接管布局高度
        self.log_panel.addWidget(self.log_label)
        self.log_panel.addWidget(self.log_display)

        # 将上下两个 Widget 放入 Splitter 中
        left_splitter.addWidget(upper_left_widget)
        left_splitter.addWidget(lower_left_widget)
        # 设置初始的大概比例 (上侧绘图区占大多数，下侧日志区占少部分)
        left_splitter.setSizes([650, 150]) 

        left_layout.addWidget(left_splitter)
        layout.addLayout(left_layout, stretch=4)

        # --- 右侧：状态信息显示区域 ---
        self.info_panel = QVBoxLayout()
        self.info_label = QLabel("无人机实时状态信息")
        self.info_display = QTextEdit()
        self.info_display.setReadOnly(True)
        self.info_display.setPlaceholderText("等待数据加载...")
        
        self.info_panel.addWidget(self.info_label)
        self.info_panel.addWidget(self.info_display)
        
        layout.addLayout(self.info_panel, stretch=1)

    # ================= 数据加载 =================
    def open_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "选择轨迹数据", "", "JSON Files (*.json)")
        if file_path:
            with open(file_path, 'r', encoding='utf-8') as f:
                raw_data = json.load(f)
                self.load_data(raw_data)
                self.update_info_display()  # 加载地图后也更新信息显示，确保设施信息同步展示
    
    def open_map_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "选择地图数据", "", "JSON Files (*.json)")
        if file_path:
            with open(file_path, 'r', encoding='utf-8') as f:
                map_data = json.load(f)
                self.load_map_data(map_data)
                
    
    def load_map_data(self, json_data):
        if "facilities_str" not in json_data or "defence_rings" not in json_data:
            print("错误：地图数据格式不匹配")
            return
        self.facilities = bfunc.Facilities(json_data['facilities_str'], json_data['defence_rings'], convert_to_utm=False)
        self.init_draw_map()
        

    def load_data(self, json_data):
        """解析符合 _load_data 格式的数据"""
        if "uavs_coords_raw" not in json_data:
            print("错误：数据格式不匹配")
            return

        self.data = json_data["uavs_coords_raw"]
        self.uav_ids = list(self.data.keys())
        
        # 计算最大步数
        self.max_steps = 0
        for uid in self.uav_ids:
            self.max_steps = max(self.max_steps, len(self.data[uid]["lats"]))

        self.current_step = 0
        self._force_autoscale = True  # 标记为新数据，允许画板首次自适应缩放
        self.slider.blockSignals(True) # 暂时屏蔽信号，防止在 draw_plot 之前重复触发导致死循环
        self.slider.setRange(0, self.max_steps - 1)
        self.slider.setValue(0)
        self.slider.blockSignals(False)
        self.update_ui_state()
        self.draw_plot()

    # ================= 逻辑控制 =================
    def toggle_play(self):
        if not self.data: return
        self.is_playing = not self.is_playing
        if self.is_playing:
            self.btn_play.setText("暂停")
            self.timer.start(200) # 每200ms更新一次，对应 DT 逻辑
        else:
            self.btn_play.setText("播放")
            self.timer.stop()

    def next_step(self):
        if self.current_step < self.max_steps - 1:
            self.current_step += 1
            self.slider.setValue(self.current_step)
        else:
            self.is_playing = False
            self.timer.stop()
            self.btn_play.setText("播放")

    def prev_step(self):
        if self.current_step > 0:
            self.current_step -= 1
            self.slider.setValue(self.current_step)

    def slider_moved(self, value):
        self.current_step = value
        self.update_ui_state()
        self.draw_plot()
        self.update_info_display()

    def update_ui_state(self):
        self.step_label.setText(f"Step: {self.current_step}/{self.max_steps - 1}")

    def toggle_coord_system(self):
        """UTM <-> 经纬度 坐标系切换：勾选状态即目标坐标系，切换后立即重绘并自适应视野"""
        self.is_utm = self.cb_utm.isChecked()
        self._force_autoscale = True
        self.init_draw_map()
        self.draw_plot()
        self.update_info_display()

    # ================= 绘图与显示 =================
    def init_draw_map(self):
        self.ax.clear()
        self.ax.set_title("UAV Trajectory Visualization")
        if self.is_utm:
            self.ax.set_xlabel("UTM X")
            self.ax.set_ylabel("UTM Y")
        else:
            self.ax.set_xlabel("Longitude")
            self.ax.set_ylabel("Latitude")

        if self.facilities:
            # 绘制设施点 (facilities 中如果是 convert_to_utm=False，则内部存储的是经纬度)
            for _fac, _lnglat_xy in self.facilities.antiairs.items():
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'ro', label=f'{_fac} Antiair')
            for _fac, _lnglat_xy in self.facilities.headquartors.items():
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'go', label=f'{_fac} Headquarters')
            for _fac, _lnglat_xy in self.facilities.probers.items():
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'bo', label=f'{_fac} Prober')
            # 未分类设施（如 switch_config 6 的 shaoxing_*）：只存在于 facilities_info，需单独绘制
            for _fac, _lnglat_xy in self.facilities.facilities_info.items():
                if _fac in self.facilities.antiairs or _fac in self.facilities.headquartors or _fac in self.facilities.probers:
                    continue
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'ko', markersize=6, label=f'{_fac} Facility')
            for _fac, _lnglat_xy in self.facilities.defend_rings.items():
                if self.is_utm:
                    _utm_xy = self._lnglat2utm_convertor.lng_lat_to_utm_array(_lnglat_xy)
                    self.ax.fill(_utm_xy[:, 0], _utm_xy[:, 1], alpha=0.2, label=f'{_fac} Defence Ring')
                else:
                    self.ax.fill(_lnglat_xy[:, 0], _lnglat_xy[:, 1], alpha=0.2, label=f'{_fac} Defence Ring')        
        # 地图单独加载时（无轨迹数据）自动适配视野；有轨迹数据时视野由 draw_plot 统一管理
        if not self.data:
            self.ax.autoscale(True)
        self.ax.legend(loc='upper right', fontsize='small')
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.canvas.draw()
    
    def draw_plot(self):
        if not self.data: return

        # 记录先前的视图范围
        old_xlim = self.ax.get_xlim()
        old_ylim = self.ax.get_ylim()

        self.ax.clear()
        self.ax.set_title("UAV Trajectory Visualization")
        if self.is_utm:
            self.ax.set_xlabel("UTM X")
            self.ax.set_ylabel("UTM Y")
        else:
            self.ax.set_xlabel("Longitude")
            self.ax.set_ylabel("Latitude")
        
        for uid in self.uav_ids:
            uav_info = self.data[uid]
            lats = uav_info["lats"]
            lngs = uav_info["lngs"]
            
            # 绘制已完成的轨迹线
            idx = min(self.current_step, len(lats) - 1)

            if self.is_utm:
                coords = self._lnglat2utm_convertor.lng_lat_to_utm_array(np.array([lngs[:idx+1], lats[:idx+1]]).T)
                xs, ys = coords[:, 0], coords[:, 1]
                cur_x, cur_y = xs[-1], ys[-1]
            else:
                xs, ys = lngs[:idx+1], lats[:idx+1]
                cur_x, cur_y = lngs[idx], lats[idx]

            self.ax.plot(xs, ys, label=f"Path {uid}", alpha=0.6)
            
            # 绘制当前点位置
            self.ax.scatter(cur_x, cur_y, marker='^', s=100)
            self.ax.text(cur_x, cur_y, uid, fontsize=9)
            
            if "extras" in uav_info and idx < len(uav_info["extras"]):
                extra = uav_info["extras"][idx]
                lookahead_coord = extra.get("lookahead_coord")
                if lookahead_coord and len(lookahead_coord) >= 2:
                    if self.is_utm:
                        lh_x, lh_y = lookahead_coord[0], lookahead_coord[1]
                    else:
                        lh_x, lh_y = self._lnglat2utm_convertor.utm_to_lng_lat(lookahead_coord[0], lookahead_coord[1])
                    self.ax.scatter(lh_x, lh_y, marker='x', s=50, label=f"Lookahead {uid}" if idx == 0 else "")
                    self.ax.plot([cur_x, lh_x], [cur_y, lh_y], linestyle=":", alpha=0.5)

        if self.facilities:
            # 绘制设施点
            for _fac, _lnglat_xy in self.facilities.antiairs.items():
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'ro', label=f'{_fac} Antiair')
            for _fac, _lnglat_xy in self.facilities.headquartors.items():
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'go', label=f'{_fac} Headquarters')
            for _fac, _lnglat_xy in self.facilities.probers.items():
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'bo', label=f'{_fac} Prober')
            # 未分类设施（如 switch_config 6 的 shaoxing_*）：只存在于 facilities_info，需单独绘制
            for _fac, _lnglat_xy in self.facilities.facilities_info.items():
                if _fac in self.facilities.antiairs or _fac in self.facilities.headquartors or _fac in self.facilities.probers:
                    continue
                x, y = self._lnglat2utm_convertor.lon_lat_to_utm(_lnglat_xy[0], _lnglat_xy[1]) if self.is_utm else _lnglat_xy
                self.ax.plot(x, y, 'ko', markersize=6, label=f'{_fac} Facility')
            for _fac, _lnglat_xy in self.facilities.defend_rings.items():
                if self.is_utm:
                    _utm_xy = self._lnglat2utm_convertor.lng_lat_to_utm_array(_lnglat_xy)
                    self.ax.fill(_utm_xy[:, 0], _utm_xy[:, 1], alpha=0.2, label=f'{_fac} Defence Ring')
                else:
                    self.ax.fill(_lnglat_xy[:, 0], _lnglat_xy[:, 1], alpha=0.2, label=f'{_fac} Defence Ring')

            
        # 判断是恢复之前的视口缩放还是重新自适应大小
        if getattr(self, '_force_autoscale', False):
            self.ax.autoscale(True)
            self._force_autoscale = False
        else:
            self.ax.set_xlim(old_xlim)
            self.ax.set_ylim(old_ylim)

        self.ax.legend(loc='upper right', fontsize='small')
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.canvas.draw()

    def update_info_display(self):
        """
        每更新一个 step 就同步更新所有无人机信息。
        这里预留了逻辑，后续可以根据 extras 里的字段进行更复杂的展示。
        """
        if not self.data: return
        
        info_text = f"--- Step {self.current_step} 实时状态 ---\n\n"
        
        for uid in self.uav_ids:
            uav_info = self.data[uid]
            if self.current_step < len(uav_info["extras"]):
                extra = uav_info["extras"][self.current_step]
                
                info_text += f"【无人机: {uid}】\n"
                if self.is_utm:
                    cur_x, cur_y = self._lnglat2utm_convertor.lon_lat_to_utm(uav_info['lngs'][self.current_step], uav_info['lats'][self.current_step])
                    info_text += f"位置 (UTM): ({cur_x:.2f}, {cur_y:.2f})\n"
                else:
                    info_text += f"位置: ({uav_info['lngs'][self.current_step]:.6f}, {uav_info['lats'][self.current_step]:.6f})\n"
                info_text += f"编队类型: {extra.get('formation_type', 'N/A')}\n"
                info_text += f"航段Key: {extra.get('segment_key', 'N/A')}\n"
                info_text += f"等待状态: {extra.get('is_waiting', 'N/A')}\n"
                info_text += f"领队ID\同伴IDs: {extra.get('leader_id', 'N/A')} | {', '.join(extra.get('cur_siblings_ids', []))}\n"
                info_text += f"等待原因: {extra.get('waiting_reason', extra.get('wait_message', 'N/A'))}\n"
                info_text += f"step_id/my_ack: {extra.get('frame_id', 'N/A')}/{extra.get('my_ack', 'N/A')}\n"
                info_text += f"同伴ACK状态: {extra.get('peers_ack_states', 'N/A')}\n"
                info_text += f"距离目标: {extra.get('dist_to_target', 'N/A')}\n"
                info_text += f"预瞄点: {extra.get('lookahead', 'N/A')}\n"
                info_text += f"当前阶段: {extra.get('phase_state', 'N/A')}\n"
                info_text += f"飞行阶段: {extra.get('flight_phase', 'N/A')}\n"
                
                lookahead_coord = extra.get('lookahead_coord')
                if lookahead_coord:
                    if self.is_utm:
                        info_text += f"lookahead坐标: ({lookahead_coord[0]:.2f}, {lookahead_coord[1]:.2f})\n"
                    else:
                        lookahead_lng, lookahead_lat = self._lnglat2utm_convertor.utm_to_lng_lat(lookahead_coord[0], lookahead_coord[1])
                        info_text += f"lookahead坐标: ({lookahead_lng:.6f}, {lookahead_lat:.6f})\n"
                    
                info_text += f"global_id: {extra.get('global_id', 'N/A')}\n"
                info_text += "-"*25 + "\n"
        
        scrollbar = self.info_display.verticalScrollBar()
        old_scroll_pos = scrollbar.value()
        self.info_display.setText(info_text)
        scrollbar.setValue(old_scroll_pos)
        
        # ---------------- 更新日志面板 ----------------
        log_text = ""
        for uid in self.uav_ids:
            uav_info = self.data[uid]
            if self.current_step < len(uav_info["extras"]):
                extra = uav_info["extras"][self.current_step]
                logs = extra.get("logs", [])
                if logs:
                    for log_msg in logs:
                        log_text += f"{log_msg}\n"
                        
        if not log_text.strip():
            log_text = f"Step {self.current_step} 没有输出日志。"
            
        log_scrollbar = self.log_display.verticalScrollBar()
        old_log_scroll_pos = log_scrollbar.value()
        is_at_bottom = old_log_scroll_pos == log_scrollbar.maximum()
        
        self.log_display.setText(log_text)
        
        if is_at_bottom:
            log_scrollbar.setValue(log_scrollbar.maximum())
        else:
            log_scrollbar.setValue(old_log_scroll_pos)

if __name__ == "__main__":
    # python -m examples.uavs_strategy.visualize.pyqt_visualize
    app = QApplication(sys.argv)
    window = UAVVisualizer()
    window.show()
    sys.exit(app.exec_())
