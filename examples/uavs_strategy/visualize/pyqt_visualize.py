import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QSlider, QLabel, QFileDialog, QWidget, QTextEdit)
from PyQt5.QtCore import Qt, QTimer

class UAVVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV 轨迹交互可视化系统")
        self.setGeometry(100, 100, 1200, 800)

        # 数据存储
        self.data = {}
        self.uav_ids = []
        self.current_step = 0
        self.max_steps = 0
        self.is_playing = False

        # 定时器用于动画播放
        self.timer = QTimer()
        self.timer.timeout.connect(self.next_step)

        self.init_ui()

    def init_ui(self):
        # 主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # --- 左侧：绘图区域与控制 ---
        left_layout = QVBoxLayout()
        
        # 顶部工具栏
        top_bar = QHBoxLayout()
        self.btn_load = QPushButton("上传可视化文件")
        self.btn_load.clicked.connect(self.open_file)
        top_bar.addWidget(self.btn_load)
        top_bar.addStretch()
        left_layout.addLayout(top_bar)

        # Matplotlib 画布
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("UAV Trajectory Visualization")
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.grid(True)
        left_layout.addWidget(self.canvas)

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
        left_layout.addLayout(controls)

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
        self.slider.setRange(0, self.max_steps - 1)
        self.slider.setValue(0)
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

    # ================= 绘图与显示 =================
    def draw_plot(self):
        if not self.data: return

        self.ax.clear()
        self.ax.set_title("UAV Trajectory Visualization")
        
        for uid in self.uav_ids:
            uav_info = self.data[uid]
            lats = uav_info["lats"]
            lngs = uav_info["lngs"]
            
            # 绘制已完成的轨迹线
            idx = min(self.current_step, len(lats) - 1)
            self.ax.plot(lngs[:idx+1], lats[:idx+1], label=f"Path {uid}", alpha=0.6)
            
            # 绘制当前点位置
            self.ax.scatter(lngs[idx], lats[idx], marker='^', s=100)
            self.ax.text(lngs[idx], lats[idx], uid, fontsize=9)

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
                info_text += f"位置: ({uav_info['lngs'][self.current_step]:.6f}, {uav_info['lats'][self.current_step]:.6f})\n"
                info_text += f"编队类型: {extra.get('formation_type', 'N/A')}\n"
                info_text += f"航段Key: {extra.get('segment_key', 'N/A')}\n"
                info_text += f"等待状态: {'是' if extra.get('is_waiting') else '否'}\n"
                info_text += f"同伴IDs: {', '.join(extra.get('cur_siblings_ids', []))}\n"
                info_text += "-"*25 + "\n"
        
        self.info_display.setText(info_text)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = UAVVisualizer()
    window.show()
    sys.exit(app.exec_())