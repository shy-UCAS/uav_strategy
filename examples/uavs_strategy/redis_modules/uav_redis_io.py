# uav_redis_io.py
# -*- coding: utf-8 -*-
from doctest import master
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

    def set_nodes_pair_base_ref_traj(self, uid_from: str, uid_to: str, traj: List[List[float]]) -> None:
        '''为digraph_attrs的一对节点定义基准参考轨迹，无人机agent基于这个参考轨迹计算集群内的各个从机轨迹'''
        # 定义唯一的 master_uid
        master_uid = f"{uid_from}_to_{uid_to}_base"
        self.r.set(master_uid, json.dumps(traj))

    # ---------- 协同：状态同步 ----------
    def set_uav_state(self, uid: str, state_key: str, state_value: str) -> None:
        """设置无人机的某个状态值 (用于多机协同)"""
        key = f"uav:{uid}:state:{state_key}"
        self.r.set(key, state_value)

    def get_uav_state(self, uid: str, state_key: str) -> Optional[str]:
        """获取无人机的某个状态值"""
        key = f"uav:{uid}:state:{state_key}"
        return self.r.get(key)
    
    def mget_uav_states(self, uids: List[str], state_key: str) -> Dict[str, str]:
        """批量获取多个无人机的状态"""
        p = self.r.pipeline()
        keys = [f"uav:{uid}:state:{state_key}" for uid in uids]
        for k in keys:
            p.get(k)
        vals = p.execute()
        
        result = {}
        for uid, val in zip(uids, vals):
            result[uid] = val if val else None
        return result

    def set_members_ref_traj(self, master_uid: str, members_traj: List[List[List[float]]]) -> None:
        """
        为集群内的每一架从机定义uid并写入参考轨迹
        members_traj: List of trajectories, where each trajectory is a list of points [x, y, z]
        """
        # 假设 members_traj 是一个列表，每个元素是一条轨迹
        for i, traj in enumerate(members_traj):
            # 定义从机UID: master_uid + "_sub_" + index
            sub_uid = f"{master_uid}_sub_{i}"
            # 写入参考轨迹
            self.set_ref_traj(sub_uid, traj)
            # 将从机ID加入到 uav:ids 集合中，以便被感知
            self.add_uav_id(sub_uid, blue=True)
            # 初始化从机位置（可选，设为轨迹起点）
            if traj and len(traj) > 0:
                start_pt = traj[0]
                # 检查是否已有位置，如果没有则初始化
                if not self.get_pos(sub_uid, blue=True):
                    self.set_pos(sub_uid, start_pt[0], start_pt[1], start_pt[2])
                    self.set_lookahead(sub_uid, 0)


    def get_ref_traj(self, uid: str) -> List[List[float]]:
        """获取参考轨迹"""
        key = f"uav:{uid}:ref_traj"
        js = self.r.get(key)
        # return json.loads(js) if js else []
        # 改为使用 json.loads 的容错
        try:
           return json.loads(js) if js else []
        except:
           return []

    def get_nodes_pair_base_ref_traj(self, uid_from: str, uid_to: str) -> List[List[float]]:
        '''获取digraph_attrs的一对节点定义基准参考轨迹'''
        master_uid = f"{uid_from}_to_{uid_to}_base"
        js = self.r.get(master_uid)
        return json.loads(js) if js else []
    
    def set_nodes_pair_member_traj(self, uid_from: str, uid_to: str, Member_Id: str, traj: List[List[float]]) -> None:
        '''为digraph_attrs的一对节点定义成员参考轨迹，无人机agent基于这个参考轨迹计算集群内的各个从机轨迹'''
        # 定义唯一的 master_uid
        master_uid = f"{uid_from}_to_{uid_to}_{Member_Id}"
        self.r.set(master_uid, json.dumps(traj))
    
    def get_nodes_pair_member_traj(self, uid_from: str, uid_to: str, Member_Id: str) -> List[List[float]]:
        '''获取digraph_attrs的一对节点定义成员参考轨迹'''
        master_uid = f"{uid_from}_to_{uid_to}_{Member_Id}"
        js = self.r.get(master_uid)
        return json.loads(js) if js else []

    def set_lookahead(self, uid: str, idx: int) -> None:
        key = f"uav:{uid}:lookahead"
        self.r.set(key, str(idx))

    def get_lookahead(self, uid: str) -> int:
        key = f"uav:{uid}:lookahead"
        v = self.r.get(key)
        return int(v) if v else 0

    def get_dist_2d(self, uid: str) -> float:
        '''获取当前位置到终点的距离'''
        # 1. 获取当前位置
        pos = self.get_pos(uid)
        if not pos:
             return None
        # 2. 获取当前轨迹
        traj = self.get_traj(uid)
        if not traj:
             return None
        # 3. 计算距离 
        # 终点
        end_pt = traj[-1]
        dist = ((pos['x'] - end_pt[0])**2 + (pos['y'] - end_pt[1])**2)**0.5
        return dist
    
    # ---------- 扩展：轨迹附加信息读写 ----------
    def set_traj_extra(self, uid: str, extra_list: List[Dict[str, Any]]) -> None:
        """覆盖写入轨迹的附加信息"""
        key = f"uav:{uid}:traj_extra"
        self.r.set(key, json.dumps(extra_list))

    def append_traj_extra(self, uid: str, extra_info: Dict[str, Any]) -> None:
        """追加轨迹附加信息"""
        key = f"uav:{uid}:traj_extra"
        cur = self.r.get(key)
        cur_list = json.loads(cur) if cur else []
        cur_list.append(extra_info) 
        self.r.set(key, json.dumps(cur_list))

    def get_traj_extra(self, uid: str) -> List[Dict[str, Any]]:
        """获取轨迹附加信息"""
        key = f"uav:{uid}:traj_extra"
        js = self.r.get(key)
        return json.loads(js) if js else []



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
