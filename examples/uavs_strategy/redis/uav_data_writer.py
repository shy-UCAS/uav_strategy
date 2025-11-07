# map_writer.py
import redis
import time

# 连接到 Redis
r = redis.Redis(host='127.0.0.1', port=6379, decode_responses=True)

def update_uav_position(uav_id: str, lat: float, lng: float, timestamp: int):
    """
    更新 UAV 的实时位置，并将其添加到轨迹历史
    :param uav_id: 无人机ID（uav1, uav2, ...）
    :param lat: 纬度
    :param lng: 经度
    :param timestamp: 时间戳（由用户指定）
    """
    # 更新实时位置（Hash）
    position_key = f"uav:{{{uav_id}}}:position"
    r.hset(position_key, mapping={"lat": lat, "lng": lng, "timestamp": timestamp})

    # 更新历史轨迹（Sorted Set）
    trajectory_key = f"uav:{uav_id}:trajectory"
    r.zadd(trajectory_key, {f"{lat},{lng}": timestamp})

    print(f"UAV {uav_id} position updated: lat={lat}, lng={lng}, timestamp={timestamp}")

def simulate_uav_movements():
    # 假设 UAV1 到 UAV5 的位置在不断更新，并且每次指定一个时间步
    base_timestamp = 1638001200  # 起始时间戳（例如 2021-11-26 00:00:00）
    
    for uav_id in ["uav1", "uav2", "uav3", "uav4", "uav5"]:
        lat = 37.50 + 0.001 * (int(uav_id[-1]))  # 假设逐步增加纬度
        lng = 122.18 + 0.001 * (int(uav_id[-1]))  # 假设逐步增加经度
        update_uav_position(uav_id, lat, lng, base_timestamp)
        
        # 增加时间步
        base_timestamp += 60  # 假设每次位置更新，时间步增加 60 秒

# 模拟 UAV1 到 UAV5 的位置更新
simulate_uav_movements()
