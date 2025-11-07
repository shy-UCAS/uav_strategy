# map_reader.py
# -*- coding: utf-8 -*-
"""
从 Redis 读取地图信息（分段重组为原始结构）：
- load_map_segmented   读取整份地图（facilities_str + defence_rings）
- list_facilities      列出所有设施名
- get_facility         获取单个设施详情
- get_ring             获取单个防区环坐标
- search_nearby        基于 GEO 的“附近设施”查询（若写入时启用了 GEO）
"""

from typing import Dict, List, Optional, Tuple, Set
import redis
import json


def connect_redis(host: str = "127.0.0.1",
                  port: int = 6379,
                  password: Optional[str] = None,
                  db: int = 0,
                  decode_responses: bool = True) -> redis.Redis:
    return redis.Redis(
        host=host, port=port, password=password, db=db, decode_responses=decode_responses
    )


def _k_fac_set(scene_id: str) -> str:
    return f"map:{{{scene_id}}}:facilities"


def _k_fac_hash(scene_id: str, name: str) -> str:
    return f"map:{{{scene_id}}}:facility:{name}"


def _k_geo(scene_id: str) -> str:
    return f"map:{{{scene_id}}}:geo:facilities"


def _k_ring_set(scene_id: str) -> str:
    return f"map:{{{scene_id}}}:rings"


def _k_ring_lngs(scene_id: str, ring_name: str) -> str:
    return f"map:{{{scene_id}}}:ring:{ring_name}:lngs"


def _k_ring_lats(scene_id: str, ring_name: str) -> str:
    return f"map:{{{scene_id}}}:ring:{ring_name}:lats"


def list_facilities(r: redis.Redis, scene_id: str) -> Set[str]:
    return r.smembers(_k_fac_set(scene_id))


def get_facility(r: redis.Redis, scene_id: str, name: str) -> Optional[Dict[str, float]]:
    info = r.hgetall(_k_fac_hash(scene_id, name))
    if not info or "lng" not in info or "lat" not in info:
        return None
    try:
        return {"lng": float(info["lng"]), "lat": float(info["lat"]), "type": info.get("type", "")}
    except ValueError:
        return None


def get_ring(r: redis.Redis, scene_id: str, ring_name: str) -> Optional[Dict[str, List[float]]]:
    lngs = r.lrange(_k_ring_lngs(scene_id, ring_name), 0, -1)
    lats = r.lrange(_k_ring_lats(scene_id, ring_name), 0, -1)
    if not lngs and not lats:
        return None
    return {"lngs": [float(x) for x in lngs], "lats": [float(x) for x in lats]}


def load_map_segmented(r: redis.Redis, scene_id: str) -> Dict:
    # 1) 设施
    facilities: Dict[str, List[float]] = {}
    for name in list_facilities(r, scene_id):
        fac = get_facility(r, scene_id, name)
        if fac:
            facilities[name] = [fac["lng"], fac["lat"]]

    # 2) 防区环
    rings: Dict[str, Dict[str, List[float]]] = {}
    for ring_name in r.smembers(_k_ring_set(scene_id)):
        ring = get_ring(r, scene_id, ring_name)
        if ring:
            rings[ring_name] = ring

    return {"facilities_str": facilities, "defence_rings": rings}


def search_nearby(r: redis.Redis,
                  scene_id: str,
                  lng: float,
                  lat: float,
                  radius_m: int = 500,
                  count: int = 10) -> List[Tuple[str, float]]:
    """
    基于 GEO 的附近检索（需要写入端构建过 GEO 索引）。
    返回 [(member, distance_m), ...]，按距离升序。
    """
    res = r.geosearch(
        _k_geo(scene_id),
        longitude=float(lng), latitude=float(lat),
        radius=int(radius_m), unit="m",
        withdist=True, count=int(count), sort="ASC"
    )
    # redis-py 返回 [(member, dist), ...]
    return [(member, float(dist)) for member, dist in res]


if __name__ == "__main__":
    # 简单自测（按需修改）
    r = connect_redis()
    scene = "demoo"
    data = load_map_segmented(r, scene)
    print("地图数据：", json.dumps(data,indent=4))
    print("设施数量：", len(data["facilities_str"]))
    print("ring1 经度长度：", len((data["defence_rings"].get("ring1") or {}).get("lngs", [])))
    near = search_nearby(r, scene, 122.181, 37.501, 500, 5)
    print("500m 内设施：", near)
    print(f"{{{scene}}}")
