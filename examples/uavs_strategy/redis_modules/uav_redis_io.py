# uav_redis_io.py
# -*- coding: utf-8 -*-
import time
import json
from typing import Dict, List, Optional, Iterable, Tuple, Any

import redis


# ======== Redis 键位约定（与前文一致） ========
# 蓝方ID集合:          "uav:ids"                       -> set
# 蓝方单机位置:        "uav:{id}:pos"                  -> {"x","y","z","ts"}  (json str)
# 蓝方轨迹:            "uav:{id}:traj"                 -> [{"x","y","z"}, ...] (json str, list)
# 蓝方预瞄点索引:      "uav:{id}:lookahead"            -> int
#
# 红方ID集合:          "red:ids"                       -> set
# 红方单机位置:        "red:{id}:pos"                  -> {"x","y","z","ts"}  (json str)

def _now_ms() -> int:
    return int(time.time() * 1000)


class UavRedisIO:
    """统一封装无人机（蓝/红）位姿与轨迹的读写操作。"""

    def __init__(self, host="127.0.0.1", port=6379, db=0, decode_responses=True):
        self.r = redis.Redis(host=host, port=port, db=db, decode_responses=decode_responses)

    # ---------- 通用：ID 集合 ----------
    def add_uav_id(self, uid: str, blue: bool = True) -> None:
        key = "uav:ids" if blue else "red:ids"
        self.r.sadd(key, uid)

    def remove_uav_id(self, uid: str, blue: bool = True) -> None:
        key = "uav:ids" if blue else "red:ids"
        self.r.srem(key, uid)

    def get_ids(self, blue: bool = True) -> Iterable[str]:
        key = "uav:ids" if blue else "red:ids"
        return self.r.smembers(key) or set()

    def scan_ids_by_key(self, prefix: str) -> Iterable[str]:
        """兜底：通过扫描键名推断 ID（比如没有维护集合时）"""
        ids, cursor = set(), 0
        while True:
            cursor, keys = self.r.scan(cursor=cursor, match=f"{prefix}:*:pos", count=200)
            ids |= {k.split(":")[1] for k in keys}
            if cursor == 0:
                break
        return ids

    # ---------- 单机：位置读写 ----------
    def set_pos(self, uid: str, x: float, y: float, z: float, ts_ms: Optional[int] = None, blue: bool = True) -> None:
        key = f"{'uav' if blue else 'red'}:{uid}:pos"
        js = json.dumps({"x": x, "y": y, "z": z, "ts": _now_ms() if ts_ms is None else int(ts_ms)})
        self.r.set(key, js)

    def get_pos(self, uid: str, blue: bool = True) -> Optional[Dict[str, Any]]:
        key = f"{'uav' if blue else 'red'}:{uid}:pos"
        js = self.r.get(key)
        return json.loads(js) if js else None

    def mget_pos(self, ids: Iterable[str], blue: bool = True) -> Dict[str, Optional[Dict[str, Any]]]:
        """批量获取位置（pipeline）"""
        p = self.r.pipeline()
        keys = [f"{'uav' if blue else 'red'}:{uid}:pos" for uid in ids]
        for k in keys:
            p.get(k)
        vals = p.execute()
        out = {}
        for uid, js in zip(ids, vals):
            out[uid] = json.loads(js) if js else None
        return out

    # ---------- 蓝方：轨迹读写 ----------
    def set_traj(self, uid: str, points: List[Dict[str, float]]) -> None:
        """覆盖写入轨迹：points = [{x,y,z}, ...]"""
        key = f"uav:{uid}:traj"
        self.r.set(key, json.dumps(points))

    def append_traj_points(self, uid: str, points: List[Dict[str, float]]) -> None:
        """追加轨迹点"""
        key = f"uav:{uid}:traj"
        cur = self.r.get(key)
        cur_list = json.loads(cur) if cur else []
        cur_list.extend(points)
        self.r.set(key, json.dumps(cur_list))

    def get_traj(self, uid: str) -> List[Dict[str, float]]:
        key = f"uav:{uid}:traj"
        js = self.r.get(key)
        return json.loads(js) if js else []

    def clear_traj(self, uid: str) -> None:
        self.r.delete(f"uav:{uid}:traj")

    # ---------- 蓝方：预瞄点 ----------
    def set_lookahead(self, uid: str, idx: int) -> None:
        self.r.set(f"uav:{uid}:lookahead", int(idx))

    def get_lookahead(self, uid: str) -> int:
        v = self.r.get(f"uav:{uid}:lookahead")
        return int(v) if v is not None else 0

    # ---------- 工具：过滤过期 ----------
    @staticmethod
    def filter_stale(pos_map: Dict[str, Optional[Dict[str, Any]]], stale_ms: int) -> Dict[str, Dict[str, Any]]:
        now_ms = _now_ms()
        out: Dict[str, Dict[str, Any]] = {}
        for uid, p in pos_map.items():
            if not p:
                continue
            ts = int(p.get("ts", 0))
            if abs(now_ms - ts) <= stale_ms:
                out[uid] = p
        return out
