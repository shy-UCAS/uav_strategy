#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import examples.uavs_strategy.planning_modules.basic_functions as bfunc
fleet1 = [
    122.09686551225596,
    37.56536338371065,
    165
]

fleet2 = [
    122.10258217246229,
    37.56342057758475
]
fleet1_utm = [425274.66488500574, 4151544.508846587]
rings = {
    "defence_rings": {
        "ring1": {
            "lngs": [122.1798974, 122.1850643, 122.1886306, 122.183215, 122.179864],
            "lats": [37.50401, 37.5014229, 37.4981283, 37.4975882, 37.5003623]
        },
        "ring2": {
            "lngs": [122.1772284, 122.1885838, 122.1980009, 122.1929069, 122.186779, 122.1785874],
            "lats": [37.5045976, 37.5091325, 37.5026949, 37.4982665, 37.4945731, 37.4985812]
        }
    }
}
ring1 = {"ring1": {
    "lngs": [122.1798974, 122.1850643, 122.1886306, 122.183215, 122.179864],
    "lats": [37.50401, 37.5014229, 37.4981283, 37.4975882, 37.5003623]
}}

print(np.array([ring1["ring1"]['lngs'], ring1["ring1"]['lats']]).T)


def _default_facilities(self, default_json_path=None):
    import json
    import os.path as osp
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

    with open(_facilities_info_json, 'r', encoding='utf-8') as f:
        _facilities_info = json.load(f)

    return bfunc.Facilities(_facilities_info['facilities_str'], _facilities_info['defence_rings'])


facilities = _default_facilities(None)
for ring in facilities.defend_rings.values():
    print(ring, len(ring))

lnglat2utm_convertor = bfunc.LngLat2UTM()
print(f"{lnglat2utm_convertor.lon_lat_to_utm(fleet1[0], fleet1[1])}")
print(f"{lnglat2utm_convertor.utm_to_lng_lat(426245.5405112152, 4153226.1147590093)}")
