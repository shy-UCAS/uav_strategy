# -*- coding: utf-8 -*-
"""
生成 facilities_shaoxing.json：
从 apitest/samples/airspace_list_sample.json 提取指定空域的设施点与三级防御半径，
把圆形防御圈离散化为正十二边形顶点（经纬度），对齐 facilities.json 的
defence_rings 存储形式 {环名: {"lngs": [...], "lats": [...]}}。

防御圈规则：三级半径（countermeasure/alert/warning）共用同一圆心——
facilityList 中第一个设施的坐标，只生成一套三级环（3 个环）。
facilities_str 仍包含该空域的全部设施点。

用法：python gen_facilities_shaoxing.py
运行后输出 data/facilities_shaoxing.json
"""
import json
import math
import os

# 空域样本文件（相对本脚本所在目录向上找仓库根目录）
_SAMPLE_REL = ["..", "..", "..", "..", "apitest", "samples", "airspace_list_sample.json"]
_OUT_NAME = "facilities_shaoxing.json"
_AIRSPACE_ID = "2086742451469029376"

# 多边形边数（正十二边形）
_N_GON = 12
# 纬度 1 度 ≈ 111.32 km；经度 1 度 ≈ 111.32km * cos(lat)
_M_PER_DEG_LAT = 111320.0

# 三级半径字段名 -> 米
_RING_LEVELS = [("countermeasure", "countermeasureRadius"),
                ("alert", "alertRadius"),
                ("warning", "warningRadius")]

# 兜底数据：找不到样本文件时使用（空域 2086742451469029376 的三个设施）
_FALLBACK_FACILITIES = [
    {"name": "shaoxing_1", "lng": 116.386614, "lat": 39.909373},
    {"name": "shaoxing_2", "lng": 116.376658, "lat": 39.899957},
    {"name": "shaoxing_3", "lng": 116.385745, "lat": 39.894678},
]
_FALLBACK_RADII = {"countermeasureRadius": 1000, "alertRadius": 2000, "warningRadius": 3000}


def find_sample_file():
    """从脚本目录逐级向上找 apitest/samples/airspace_list_sample.json"""
    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        cand = os.path.join(cur, *_SAMPLE_REL)
        if os.path.exists(cand):
            return cand
        cur = os.path.dirname(cur)
    return None


def load_airspace():
    """读取样本文件，返回 (facilities, radii)；文件缺失时用兜底数据"""
    sample_path = find_sample_file()
    if sample_path:
        with open(sample_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for item in data["data"]["list"]:
            if item["id"] == _AIRSPACE_ID:
                facilities = [
                    {"name": f"shaoxing_{i+1}", "lng": fac["lng"], "lat": fac["lat"]}
                    for i, fac in enumerate(item["facilityList"])
                ]
                radii = {k: item[k] for k in item if k in ("countermeasureRadius", "alertRadius", "warningRadius")}
                print(f"[gen] 从样本读取空域 {_AIRSPACE_ID}: {len(facilities)} 个设施, 半径 {radii}")
                return facilities, radii
        raise ValueError(f"样本中找不到空域 id={_AIRSPACE_ID}")
    print("[gen] 样本文件不存在，使用兜底数据")
    return _FALLBACK_FACILITIES, _FALLBACK_RADII


def circle_to_ngon(lng0, lat0, radius_m, n=_N_GON):
    """
    将圆形防御圈（半径米，圆心经纬度）离散化为正 n 边形顶点（经纬度）。
    近似换算：1° 纬度 ≈ 111.32km；1° 经度 ≈ 111.32km * cos(lat0)。
    顶点首尾不闭合（与现有 RING1/RING2 一致，shapely 自动闭合）。
    """
    cos_lat = math.cos(math.radians(lat0))
    lngs, lats = [], []
    for k in range(n):
        theta = 2 * math.pi * k / n
        lngs.append(round(lng0 + radius_m * math.cos(theta) / _M_PER_DEG_LAT / cos_lat, 6))
        lats.append(round(lat0 + radius_m * math.sin(theta) / _M_PER_DEG_LAT, 6))
    return lngs, lats


def main():
    facilities, radii = load_airspace()
    if not facilities:
        raise ValueError("空域没有设施点")

    # 三级防御圈共用圆心：facilityList 的第一个设施
    center = facilities[0]
    print(f"[gen] 防御圈共用圆心 = facilityList[0]（{center['name']}）: ({center['lng']}, {center['lat']})")

    facilities_str = {fac["name"]: [fac["lng"], fac["lat"]] for fac in facilities}

    defence_rings = {}
    for level_name, radius_key in _RING_LEVELS:
        radius_m = radii[radius_key]
        ring_name = f"{center['name']}_{level_name}"
        lngs, lats = circle_to_ngon(center["lng"], center["lat"], radius_m)
        defence_rings[ring_name] = {"lngs": lngs, "lats": lats}
        print(f"[gen] {ring_name}: 圆心=({center['lng']}, {center['lat']}) r={radius_m}m, {len(lngs)} 个顶点")

    out_data = {
        "facilities_str": facilities_str,
        "defence_rings": defence_rings,
    }
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), _OUT_NAME)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out_data, f, indent=4, ensure_ascii=False)
    print(f"[gen] 已生成: {out_path}")
    print(f"[gen] facilities_str {len(facilities_str)} 项, defence_rings {len(defence_rings)} 项")


if __name__ == "__main__":
    main()
