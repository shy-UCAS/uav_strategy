# uav_redis_io.py
# -*- coding: utf-8 -*-
import time
import json
from typing import Dict, List, Optional, Iterable, Tuple, Any
import numpy as np
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

    def mget_speed_from_traj(self, ids, blue=True, dt: float = 1.0):
        """
        使用“滞后一帧”的方式计算速度：本轮用的是上一轮的速度，避免半步更新不对称。
        """
        trajs = self.mget_traj(ids, blue=blue)
        speeds = {}

        for uid, traj in trajs.items():
            n = len(traj)
            if n >= 3:
                # 用 t-2 → t-1 之间的位移作为本轮速度
                p1 = np.asarray(traj[-2], dtype=float)
                p0 = np.asarray(traj[-3], dtype=float)
                v = (p1 - p0) / float(dt)
                speeds[uid] = v.tolist()
            elif n == 2:
                # 只有两个点，只好用这两个算一次
                p1 = np.asarray(traj[-1], dtype=float)
                p0 = np.asarray(traj[-2], dtype=float)
                v = (p1 - p0) / float(dt)
                speeds[uid] = v.tolist()
            else:
                speeds[uid] = [0.0, 0.0, 0.0]

        return speeds

    # ---------- 蓝方：轨迹读写 ----------
    def set_traj(self, uid: str, points: List[List[float]]) -> None:
        """覆盖写入轨迹：points = [{x,y,z}, ...]"""
        key = f"uav:{uid}:traj"
        self.r.set(key, json.dumps(points))

    def append_traj_points(self, uid: str, points: List[float]) -> None:
        """追加轨迹点"""
        key = f"uav:{uid}:traj"
        cur = self.r.get(key)
        cur_list = json.loads(cur) if cur else []
        cur_list.append(points)
        self.r.set(key, json.dumps(cur_list))

    def get_traj(self, uid: str) -> List[List[float]]:
        key = f"uav:{uid}:traj"
        js = self.r.get(key)
        return json.loads(js) if js else []

    def mget_traj(self, ids: Iterable[str],blue: bool = True) -> Dict[str, List[List[float]]]:
        """批量获取轨迹（pipeline）"""
        p = self.r.pipeline()
        keys = [f"{'uav' if blue else 'red'}:{uid}:traj" for uid in ids]
        # 批量读取
        for k in keys:
            p.get(k)

        vals = p.execute()

        out = {}
        for uid, js in zip(ids, vals):
            out[uid] = json.loads(js) if js else []

        return out

    def clear_traj(self, uid: str) -> None:
        self.r.delete(f"uav:{uid}:traj")

    # ---------- 蓝方：参考轨迹读写 ----------
    def set_ref_traj(self, uid: str, points: List[List[float]]) -> None:
        """覆盖写入参考轨迹：points = [{x,y,z}, ...]"""
        key = f"uav:{uid}:ref_traj"
        self.r.set(key, json.dumps(points))

    def get_ref_traj(self, uid: str) -> List[List[float]]:
        """获取参考轨迹"""
        key = f"uav:{uid}:ref_traj"
        raw = self.r.get(key)
        if not raw:
            return []
        else:
            return json.loads(raw)


    # ---------- 蓝方：预瞄点 ----------
    def set_lookahead(self, uid: str, idx: int) -> None:
        """
        当前在预设轨迹上走到第几个点（索引）
        """
        key = f"uav:{uid}:lookahead"
        self.r.set(key, str(int(idx)))

    def get_lookahead(self, uid: str) -> Optional[int]:
        key = f"uav:{uid}:lookahead"
        raw = self.r.get(key)
        if raw is None:
            return None

        # 兼容 decode_responses=True/False 两种情况
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8")

        try:
            return int(raw)
        except ValueError:
            return None

    def get_dist_2d(self, uid: str, blue: bool = True):
        """获取当前位置，距离当前参考轨迹终点的距离"""
        pos = self.get_pos(uid, blue=blue)
        ref_traj = self.get_ref_traj(uid)

        if not pos or not ref_traj:
            print("Warning: no valid pos or ref_traj for uav", pos, ref_traj)
            return None

        # 当前位置 (x, y)
        x1, y1 = pos["x"], pos["y"]

        # 参考轨迹终点 (x, y)
        x2, y2 = ref_traj[-1][0], ref_traj[-1][1]

        # 平面距离 √((x1-x2)^2 + (y1-y2)^2)
        dist = np.hypot(x1 - x2, y1 - y2)

        return dist

    def get_rendezvous_point(self, ids: List[str], blue: bool = True) -> Optional[List[float]]:
        """
        计算一组无人机的中心汇合点
        """
        if not ids:
            return None
        
        positions_map = self.mget_pos(ids, blue=blue)
        valid_points = []
        for uid, pos_data in positions_map.items():
            if pos_data:
                valid_points.append([pos_data['x'], pos_data['y'], pos_data['z']])
        
        if not valid_points:
            return None

        points_np = np.array(valid_points)
        centroid = np.mean(points_np, axis=0)
        return centroid.tolist()

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
