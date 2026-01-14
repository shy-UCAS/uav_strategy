# -*- coding: utf-8 -*-
"""
visualize_trajectories_check.py

此脚本用于读取 uav_trajectories.json 并可视化轨迹，
帮助分析 Master 与 Sub 节点轨迹点数量差异及断裂问题。
"""

import json
import matplotlib.pyplot as plt
import os
import numpy as np
import sys

# 设置中文字体（尝试常见的 Windows 中文字体，防止乱码）
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial'] 
plt.rcParams['axes.unicode_minus'] = False

def visualize():
    # 1. 确定 JSON 文件路径
    # 假设脚本位于 examples/uavs_strategy/ 下，json 在根目录 uav_strategy/ 下
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    json_path = os.path.join(base_dir, 'uav_trajectories_from_BDI.json')

    # 如果相对路径找不到，尝试硬编码的绝对路径（根据你的环境）
    if not os.path.exists(json_path):
        json_path = r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy\uav_trajectories.json'
    
    if not os.path.exists(json_path):
        print(f"Error: Cannot find {json_path}")
        return

    print(f"正在加载数据: {json_path}")
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f).get('uavs_coords_str', {})

    # 2. 打印统计信息
    print("-" * 60)
    print(f"{'Agent Name':<25} | {'Points':<10} | {'Status'}")
    print("-" * 60)

    sorted_keys = sorted(data.keys())
    
    # 准备绘图
    plt.figure(figsize=(14, 10))
    
    #以此区分 Master 和 Sub（简单通过名字长度或后缀判断，这只是可视化辅助）
    # 使用 lat, lng
    
    for agent_name in sorted_keys:
        traj = data[agent_name]
        lats = traj.get('lats', [])
        lngs = traj.get('lngs', [])
        
        count = len(lats)
        
        status = "Normal"
        if count < 100:
            status = "Low Count (Potential Gap)"
        
        print(f"{agent_name:<25} | {count:<10} | {status}")
        
        if count == 0:
            continue
            
        # 绘图 (修改为点显示模式，方便查看点密度)
        # 区分样式：Sub 用密集小点，Master 用显眼大点
        if "sub" in agent_name:
            line_style = 'None' # 关键：不画线，只画点
            marker = '.'
            markersize = 3
            alpha = 0.5
            linewidth = 0
        else:
            # Master / Main agents
            line_style = 'None' # 关键：不画线，只画点
            marker = '.'
            markersize = 6
            alpha = 0.8
            linewidth = 0
            
        p = plt.plot(lngs, lats, 
                 label=f"{agent_name} ({count})", 
                 linestyle=line_style, 
                 marker=marker, 
                 markersize=markersize,
                 alpha=alpha,
                 linewidth=linewidth)
        
        # 标记起点和终点
        color = p[0].get_color()
        plt.plot(lngs[0], lats[0], marker='^', color=color, markersize=8) # 起点
        plt.plot(lngs[-1], lats[-1], marker='s', color=color, markersize=8) # 终点
        
        # 在终点附近标注名字
        plt.text(lngs[-1], lats[-1], agent_name, fontsize=9, color=color, fontweight='bold')

    plt.title('UAV 轨迹重建可视化分析 (Traj Reconstruction Analysis)', fontsize=16)
    plt.xlabel('Longitude (经度)')
    plt.ylabel('Latitude (纬度)')
    plt.legend(bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()

    # 保存图片
    output_file = os.path.join(os.path.dirname(__file__), 'trajectory_analysis.png')
    plt.savefig(output_file, dpi=150)
    print(f"\n可视化图表已保存至: {output_file}")
    
    # 尝试显示（如果在支持 GUI 的环境）
    try:
        plt.show()
    except Exception as e:
        print("无法直接显示窗口 (可能是无头模式)，请查看保存的图片文件。")

if __name__ == "__main__":
    visualize()
