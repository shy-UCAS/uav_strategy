# -*- coding: utf-8 -*-
"""
visualize_trajectories_check.py

此脚本读取 uav_trajectories.json 并动态可视化轨迹。
包含播放/暂停、上一帧/下一帧、进度拖拽、跳转指定帧等功能。
"""

import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import font_manager
from matplotlib.widgets import Button, Slider, TextBox
import os
import sys

# 设置中文字体（仅在检测到可用 CJK 字体时启用中文显示）
_cjk_font_candidates = [
    'SimHei', 'Microsoft YaHei', 'Noto Sans CJK SC',
    'WenQuanYi Zen Hei', 'Source Han Sans SC', 'PingFang SC',
    'Heiti SC', 'Arial Unicode MS'
]
_installed_fonts = {f.name for f in font_manager.fontManager.ttflist}
_available_cjk_fonts = [name for name in _cjk_font_candidates if name in _installed_fonts]
HAS_CJK_FONT = bool(_available_cjk_fonts)

if HAS_CJK_FONT:
    plt.rcParams['font.sans-serif'] = _available_cjk_fonts + ['DejaVu Sans']
else:
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class TrajectoryPlayer:
    def __init__(self, json_path=None):
        self.data = self._load_data(json_path)
        if not self.data:
            print("未找到有效数据，程序退出。")
            return

        self.agents = sorted(self.data.keys())
        # 计算最大帧数
        self.max_frame = 0
        if self.agents:
            self.max_frame = max([len(self.data[a].get('lats', [])) for a in self.agents])
        
        if self.max_frame == 0:
            print("轨迹数据为空。")
            return

        # 初始化绘图窗口
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        plt.subplots_adjust(bottom=0.25) # 留出底部控件区域

        self.lines = {}
        self.points = {}
        self.labels = {}
        
        # 预先计算坐标范围，固定视角
        all_lats = []
        all_lngs = []
        for a in self.agents:
            lats = self.data[a].get('lats', [])
            lngs = self.data[a].get('lngs', [])
            # 简单过滤无效点(如0,0)
            valid_lats = [y for y in lats if abs(y) > 0.1]
            valid_lngs = [x for x in lngs if abs(x) > 0.1]
            all_lats.extend(valid_lats)
            all_lngs.extend(valid_lngs)
            
        if all_lats:
            margin_lat = (max(all_lats) - min(all_lats)) * 0.1 if len(all_lats) > 1 else 0.01
            margin_lng = (max(all_lngs) - min(all_lngs)) * 0.1 if len(all_lngs) > 1 else 0.01
            self.ax.set_ylim(min(all_lats) - margin_lat, max(all_lats) + margin_lat)
            self.ax.set_xlim(min(all_lngs) - margin_lng, max(all_lngs) + margin_lng)
        
        self.ax.set_xlabel('Longitude (经度)' if HAS_CJK_FONT else 'Longitude')
        self.ax.set_ylabel('Latitude (纬度)' if HAS_CJK_FONT else 'Latitude')
        self.ax.set_title(
            f'UAV 轨迹动态回放 (Total Frames: {self.max_frame})'
            if HAS_CJK_FONT else
            f'UAV Trajectory Playback (Total Frames: {self.max_frame})'
        )
        self.ax.grid(True, linestyle=':', alpha=0.6)

        # 初始化每个 Agent 的线条和当前的头部点
        for agent in self.agents:
            # 区分 Master 和 Sub 样式
            if "sub" in agent.lower():
                marker_style = '.'
                line_width = 1
                alpha = 0.6
                markersize = 12 # 稍微大一点以便看见
            else:
                marker_style = 'o'
                line_width = 2
                alpha = 0.9
                markersize = 8

            # 轨迹线
            line, = self.ax.plot([], [], label=agent, linewidth=line_width, alpha=alpha)
            # 当前位置点
            point, = self.ax.plot([], [], marker=marker_style, markersize=markersize, color=line.get_color())
            # 当前位置名称（颜色与轨迹一致）
            label = self.ax.annotate(
                agent,
                xy=(0, 0),
                xytext=(6, 6),
                textcoords='offset points',
                color=line.get_color(),
                fontsize=9
            )
            
            self.lines[agent] = line
            self.points[agent] = point
            self.labels[agent] = label

        # 状态控制
        self.current_frame = 0
        self.is_playing = False

        # 初始化动画对象
        # interval=100ms
        self.anim = animation.FuncAnimation(
            self.fig, self.update_animation, 
            frames=self.max_frame, 
            interval=100, 
            blit=False, 
            repeat=True
        )
        self.anim.event_source.stop() # 默认暂停

        self._setup_widgets()
        
        # 初始绘制
        self.update_plot(0)

    def _load_data(self, json_path):
        if not json_path:
             # 尝试自动寻找路径
             current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
             # 假设在 examples/uavs_strategy/ 下，向上找
             # 1. search in current
             # 2. search in parent/parent (root)
             
             possible_paths = [
                 os.path.join(current_dir,'data','raw_data' ,'uav_trajectories_persistent_20260214_200227.json'),
                 os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'uav_trajectories_persistent.json'),
                 os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'uav_trajectories.json'),
                 r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy\uav_trajectories.json',
                 r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy\uav_trajectories_persistent.json'
             ]
             
             for p in possible_paths:
                 if os.path.exists(p):
                     json_path = p
                     break
        
        if not json_path or not os.path.exists(json_path):
             print(f"Error: Cannot find any data file.")
             return {}
        
        print(f"Loading data from: {json_path}")
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                content = json.load(f)
                # 兼容不同的 json 结构
                if 'uavs_coords_str' in content:
                    return content.get('uavs_coords_raw', {})
                else:
                    return content # 假设就是 {agent: {coord...}}
        except Exception as e:
            print(f"Error reading json: {e}")
            return {}

    def _setup_widgets(self):
        # 按钮位置定义 [left, bottom, width, height]
        
        # 播放/暂停
        ax_play = plt.axes([0.1, 0.05, 0.1, 0.05])
        self.btn_play = Button(ax_play, 'Play')
        self.btn_play.on_clicked(self.on_play_clicked)

        # 上一帧
        ax_prev = plt.axes([0.22, 0.05, 0.1, 0.05])
        self.btn_prev = Button(ax_prev, 'Prev <')
        self.btn_prev.on_clicked(self.on_prev_clicked)

        # 下一帧
        ax_next = plt.axes([0.34, 0.05, 0.1, 0.05])
        self.btn_next = Button(ax_next, 'Next >')
        self.btn_next.on_clicked(self.on_next_clicked)

        # 进度条 Slider
        ax_slider = plt.axes([0.1, 0.12, 0.65, 0.03])
        self.slider = Slider(ax_slider, 'Time', 0, self.max_frame - 1, valinit=0, valstep=1, valfmt='%d')
        self.slider.on_changed(self.on_slider_changed)

        # 跳转输入框
        ax_input = plt.axes([0.85, 0.12, 0.08, 0.03])
        self.text_box = TextBox(ax_input, 'Jump:', initial="0")
        self.text_box.on_submit(self.on_text_submit)

    def update_animation(self, frame):
        # 动画回调，自动增加 frame
        if self.is_playing:
            self.current_frame = (self.current_frame + 1) % self.max_frame
            self.update_widget_visuals()
            self.update_plot(self.current_frame)
        return []

    def update_plot(self, frame):
        frame = int(frame)
        for agent in self.agents:
            lats = self.data[agent].get('lats', [])
            lngs = self.data[agent].get('lngs', [])
            
            if not lats:
                continue
            
            # 如果当前帧超出了该 agent 的数据长度，则停留在最后一点
            idx = min(frame, len(lats) - 1)
            
            # 更新轨迹线 (0 -> idx)
            self.lines[agent].set_data(lngs[:idx+1], lats[:idx+1])
            
            # 更新当前点
            self.points[agent].set_data([lngs[idx]], [lats[idx]])
            # 更新名称位置
            self.labels[agent].xy = (lngs[idx], lats[idx])
        
        self.fig.canvas.draw_idle()

    def update_widget_visuals(self):
        # 更新 Slider 显示
        self.slider.eventson = False
        self.slider.set_val(self.current_frame)
        self.slider.eventson = True
        
    def on_play_clicked(self, event):
        if self.is_playing:
            self.anim.event_source.stop()
            self.btn_play.label.set_text('Play')
            self.is_playing = False
        else:
            self.anim.event_source.start()
            self.btn_play.label.set_text('Pause')
            self.is_playing = True

    def on_prev_clicked(self, event):
        self._manual_set_frame(self.current_frame - 1)

    def on_next_clicked(self, event):
        self._manual_set_frame(self.current_frame + 1)

    def on_slider_changed(self, val):
        # 拖拽 Slider 时，如果是播放状态，可以考虑暂停或者继续用新的一帧
        self.current_frame = int(val)
        self.update_plot(self.current_frame)

    def on_text_submit(self, text):
        try:
            val = int(text)
            self._manual_set_frame(val)
        except ValueError:
            pass

    def _manual_set_frame(self, frame):
        # 边界检查
        if frame < 0: frame = 0
        if frame >= self.max_frame: frame = self.max_frame - 1
        
        self.current_frame = frame
        self.update_widget_visuals()
        self.update_plot(self.current_frame)

    def show(self):
        plt.show()

if __name__ == "__main__":
    # python -m examples.uavs_strategy.visualize.visualize_trajectories_check
    player = TrajectoryPlayer()
    if player.max_frame > 0:
        player.show()
