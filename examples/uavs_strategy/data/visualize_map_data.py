# -*- coding: utf-8 -*-
"""
地图数据可视化查看脚本（matplotlib，无 PyQt 依赖）
仅展示地图数据：

- 设施点（facilities_str）：散点 + 名称标注
- 防御圈（defence_rings）：填充多边形；shaoxing_N_countermeasure/alert/warning
                           按三级半径着色，其他环名（RING1 等）灰色

用法（在本目录下执行）：
    python visualize_map_data.py
    python visualize_map_data.py --facilities facilities.json
    python visualize_map_data.py --save out.png
"""
import argparse
import json
import os.path as osp

import matplotlib
import numpy as np

DATA_DIR = osp.dirname(osp.abspath(__file__))

# 三级防御半径配色：countermeasure(内圈) -> alert(中圈) -> warning(外圈)
RING_LEVEL_COLORS = {
    "countermeasure": "#1f77b4",
    "alert": "#ff7f0e",
    "warning": "#d62728",
}
RING_DEFAULT_COLOR = "#7f7f7f"


def load_json(path):
    """按 utf-8 读取，失败时回退 gbk（部分手工数据文件是 GBK 编码）"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except UnicodeDecodeError:
        with open(path, "r", encoding="gbk") as f:
            return json.load(f)


def setup_fonts():
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False


def draw_facilities(ax, facilities_str):
    """设施点散点 + 名称标注"""
    for name, lnglat in facilities_str.items():
        ax.scatter(lnglat[0], lnglat[1], s=60, c="#333333", zorder=5)
        ax.annotate(name, (lnglat[0], lnglat[1]), textcoords="offset points",
                    xytext=(6, 6), fontsize=8, zorder=6)


def draw_rings(ax, defence_rings):
    """防御圈填充多边形；环名含三级半径后缀时按等级着色"""
    for ring_name, ring in defence_rings.items():
        color = RING_DEFAULT_COLOR
        level = "zone"
        for level_name, level_color in RING_LEVEL_COLORS.items():
            if ring_name.endswith("_" + level_name):
                color = level_color
                level = level_name
                break
        ax.fill(ring["lngs"], ring["lats"], color=color, alpha=0.15,
                edgecolor=color, linewidth=1.0, zorder=2,
                label=f"{ring_name} ({level})")


def main():
    parser = argparse.ArgumentParser(description="地图数据可视化查看脚本（仅设施 + 防御圈）")
    default_facilities = "facilities_shaoxing.json" if osp.exists(
        osp.join(DATA_DIR, "facilities_shaoxing.json")) else "facilities.json"
    parser.add_argument("--facilities", default=default_facilities,
                        help="设施文件（facilities_str + defence_rings），默认 facilities_shaoxing.json")
    parser.add_argument("--save", default=None,
                        help="保存 PNG 路径（默认交互显示）")
    args = parser.parse_args()

    if args.save:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    setup_fonts()

    facilities_path = osp.join(DATA_DIR, args.facilities)
    facilities_data = load_json(facilities_path)
    facilities_str = facilities_data.get("facilities_str", {})
    defence_rings = facilities_data.get("defence_rings", {})
    print(f"[viz] 设施文件: {facilities_path}（{len(facilities_str)} 设施, {len(defence_rings)} 防御圈）")

    if not facilities_str and not defence_rings:
        print("[error] 文件中没有可绘制的设施/防御圈数据")
        return

    lngs_all = [v[0] for v in facilities_str.values()]
    lats_all = [v[1] for v in facilities_str.values()]
    for ring in defence_rings.values():
        lngs_all += ring["lngs"]
        lats_all += ring["lats"]
    if not lngs_all:
        print("[error] 没有可绘制的坐标数据")
        return

    fig, ax = plt.subplots(figsize=(12, 9))
    draw_rings(ax, defence_rings)
    draw_facilities(ax, facilities_str)

    mid_lat = np.mean(lats_all)
    ax.set_aspect(1.0 / np.cos(np.radians(mid_lat)))  # 经纬度等比例，保证圆不变形
    ax.set_xlim(min(lngs_all) - 0.01, max(lngs_all) + 0.01)
    ax.set_ylim(min(lats_all) - 0.01, max(lats_all) + 0.01)
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title(f"Map Data View — {osp.basename(args.facilities)}")
    ax.grid(True, linestyle="--", alpha=0.5)

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="upper right", fontsize="small", ncol=2)

    if args.save:
        save_path = osp.join(DATA_DIR, args.save) if not osp.isabs(args.save) else args.save
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[viz] 已保存: {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
