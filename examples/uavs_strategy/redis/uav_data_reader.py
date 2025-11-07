# map_reader.py
import redis

# 连接到 Redis
r = redis.Redis(host='127.0.0.1', port=6379, decode_responses=True)

def get_uav_position(uav_id: str):
    position_key = f"uav:{uav_id}:position"
    position = r.hgetall(position_key)
    if position:
        return {
            "lat": float(position["lat"]),
            "lng": float(position["lng"]),
            "timestamp": int(position["timestamp"])
        }
    return None

def get_uav_trajectory(uav_id: str, start_time: int = None, end_time: int = None, count: int = 10):
    trajectory_key = f"uav:{uav_id}:trajectory"
    if start_time and end_time:
        trajectory = r.zrangebyscore(trajectory_key, start_time, end_time, withscores=True, num=10)
    else:
        trajectory = r.zrevrange(trajectory_key, 0, count - 1, withscores=True)
    return [{"position": pos.decode("utf-8"), "timestamp": int(score)} for pos, score in trajectory]

# 获取 UAV1 的最新位置
uav1_position = get_uav_position("uav1")
print(f"UAV1 current position: {uav1_position}")

# 获取 UAV1 的最新10条轨迹（按时间倒序）
uav1_trajectory = get_uav_trajectory("uav1", count=10)
print("UAV1 trajectory:", uav1_trajectory)

