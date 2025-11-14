# map_writer.py
# -*- coding: utf-8 -*-
"""
将地图信息“分段”写入普通 Redis（无需 Redis Stack）：
- 设施：Set + Hash（可选同步到 GEO）
- 防区环：Set + List（lngs / lats 分别为一个列表）
并提供：
- 批量写入 save_map_segmented
- 单点更新设施 upsert_facility
- 向某个防区环追加点 upsert_ring_append_points
- 清空一个场景的所有相关 key clear_scene
"""

from typing import Dict, List, Tuple, Optional
import redis


def connect_redis(host: str = "127.0.0.1",
                  port: int = 6379,
                  password: Optional[str] = None,
                  db: int = 0,
                  decode_responses: bool = True) -> redis.Redis:
    return redis.Redis(
        host=host, port=port, password=password, db=db, decode_responses=decode_responses
    )

# 使用 {scene_id} 便于 Cluster 哈希标签归槽
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


def save_map_segmented(r: redis.Redis,
                       scene_id: str,
                       data: Dict,
                       build_geo: bool = True,
                       reset_existing: bool = True) -> None:
    """
    批量写入整份地图（分段落盘）。
    data 结构示例：
    {
        "facilities_str": {"radar_1": [lng, lat], ...},
        "defence_rings": {"ring1": {"lngs": [...], "lats": [...]}, ...}
    }
    """
    facs: Dict[str, List[float]] = data.get("facilities_str", {}) or {}
    rings: Dict[str, Dict[str, List[float]]] = data.get("defence_rings", {}) or {}

    with r.pipeline(transaction=True) as pipe:
        # 1) 设施索引 & 设施 Hash
        fac_set_key = _k_fac_set(scene_id)
        if reset_existing:
            pipe.delete(fac_set_key)
        for name, coords in facs.items():
            if not isinstance(coords, (list, tuple)) or len(coords) != 2:
                continue
            lng, lat = coords
            pipe.hset(_k_fac_hash(scene_id, name), mapping={
                "lng": float(lng), "lat": float(lat), "type": name.split("_")[0]
            })
            pipe.sadd(fac_set_key, name)

        # 2) 可选 GEO 索引
        if build_geo:
            geo_key = _k_geo(scene_id)
            if reset_existing:
                pipe.delete(geo_key)
            for name, (lng, lat) in facs.items():
                pipe.geoadd(geo_key, (float(lng), float(lat), name))

        # 3) 防区环 Set & 每个环的 List
        ring_set_key = _k_ring_set(scene_id)
        if reset_existing:
            pipe.delete(ring_set_key)
        for ring_name, ring in rings.items():
            lngs = ring.get("lngs") or []
            lats = ring.get("lats") or []
            # 清理旧列表
            pipe.delete(_k_ring_lngs(scene_id, ring_name), _k_ring_lats(scene_id, ring_name))
            # 逐列表写入
            if len(lngs) > 0:
                pipe.rpush(_k_ring_lngs(scene_id, ring_name), *[float(x) for x in lngs])
            if len(lats) > 0:
                pipe.rpush(_k_ring_lats(scene_id, ring_name), *[float(x) for x in lats])
            pipe.sadd(ring_set_key, ring_name)

        pipe.execute()


def upsert_facility(r: redis.Redis,
                    scene_id: str,
                    name: str,
                    lng: float,
                    lat: float,
                    ftype: Optional[str] = None,
                    build_geo: bool = True) -> None:
    """新增/更新一个设施点（Hash + 可选 GEO + 加入 facilities 索引）"""
    if ftype is None:
        ftype = name.split("_")[0]
    with r.pipeline(transaction=True) as pipe:
        pipe.hset(_k_fac_hash(scene_id, name), mapping={
            "lng": float(lng), "lat": float(lat), "type": ftype
        })
        pipe.sadd(_k_fac_set(scene_id), name)
        if build_geo:
            pipe.geoadd(_k_geo(scene_id), (float(lng), float(lat), name))
        pipe.execute()


def upsert_ring_append_points(r: redis.Redis,
                              scene_id: str,
                              ring_name: str,
                              lngs: Optional[List[float]] = None,
                              lats: Optional[List[float]] = None) -> None:
    """向指定防区环尾部追加一批经纬度点（分别写入两个 List）。"""
    lngs = lngs or []
    lats = lats or []
    if not lngs and not lats:
        return
    with r.pipeline(transaction=True) as pipe:
        # 确保 ring 被索引
        pipe.sadd(_k_ring_set(scene_id), ring_name)
        if lngs:
            pipe.rpush(_k_ring_lngs(scene_id, ring_name), *[float(x) for x in lngs])
        if lats:
            pipe.rpush(_k_ring_lats(scene_id, ring_name), *[float(x) for x in lats])
        pipe.execute()


def clear_scene(r: redis.Redis, scene_id: str) -> int:
    """
    清空一个场景的所有相关 key（按前缀扫描+批量删除）。
    返回删除 key 的数量。
    """
    prefix = f"map:{{{scene_id}}}:"
    to_del, count = [], 0
    with r.pipeline(transaction=False) as p:
        for k in r.scan_iter(f"{prefix}*"):
            to_del.append(k)
            if len(to_del) >= 512:
                p.delete(*to_del); count += len(to_del); to_del.clear()
        if to_del:
            p.delete(*to_del); count += len(to_del)
        p.execute()
    return count


if __name__ == "__main__":
    # 简单自测（按需修改）
    r = connect_redis()
    demo = {
        "facilities_str": {
            "radar_3": [122.18911, 37.50317],
            "hq_1": [122.18066, 37.50153],
        },
        "defence_rings": {
            "ring1": {
                "lngs": [122.1798974, 122.1850643],
                "lats": [37.50401, 37.5014229],
            }
        }
    }
    map_name = "demoo"
    save_map_segmented(r, map_name, demo, build_geo=True)
    upsert_facility(r, map_name, "ua_1", 122.18096, 37.50254)
    upsert_ring_append_points(r, map_name, "ring1", lngs=[122.1886306], lats=[37.4981283])
    print("写入完成。")
