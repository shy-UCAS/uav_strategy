import redis


r = redis.Redis(host="127.0.0.1",db=0 ,port=6379, decode_responses=True)

data = {
    "facilities_str": {
        "radar_1": [122.18911, 37.50317],
        "hq_1":    [122.18066, 37.50153],
        "hq_2":    [122.18446, 37.49944],
        "ua_1":    [122.18096, 37.50254],
        "ua_2":    [122.18751, 37.50112],
    },
    "defence_rings": {
        "ring1": {
            "lngs": [122.1798974, 122.1850643, 122.1886306, 122.183215, 122.179864],
            "lats": [37.50401, 37.5014229, 37.4981283, 37.4975882, 37.5003623],
        },
        "ring2": {
            "lngs": [122.1772284, 122.1885838, 122.1980009, 122.1929069, 122.186779, 122.1785874],
            "lats": [37.5045976, 37.5091325, 37.5026949, 37.4982665, 37.4945731, 37.4985812],
        },
    },
}

scene_id = "demo"

def save_map_segmented(r, scene_id, data, build_geo=True):
    pipe = r.pipeline(transaction=True)

    # 1) 设施
    fac_set_key = f"map:{scene_id}:facilities"
    pipe.delete(fac_set_key)
    for name, (lng, lat) in data["facilities_str"].items():
        hkey = f"map:{scene_id}:facility:{name}"
        pipe.hset(hkey, mapping={"lng": lng, "lat": lat, "type": name.split("_")[0]})
        pipe.sadd(fac_set_key, name)
    # 可选：写入 GEO 索引，便于邻近查询
    if build_geo:
        geo_key = f"map:{scene_id}:geo:facilities"
        pipe.delete(geo_key)
        for name, (lng, lat) in data["facilities_str"].items():
            pipe.geoadd(geo_key, (lng, lat, name))

    # 2) 防区环
    ring_set_key = f"map:{scene_id}:rings"
    pipe.delete(ring_set_key)
    for ring_name, ring in data["defence_rings"].items():
        lng_key = f"map:{scene_id}:ring:{ring_name}:lngs"
        lat_key = f"map:{scene_id}:ring:{ring_name}:lats"
        pipe.delete(lng_key, lat_key)
        if ring["lngs"]:
            pipe.rpush(lng_key, *ring["lngs"])
        if ring["lats"]:
            pipe.rpush(lat_key, *ring["lats"])
        pipe.sadd(ring_set_key, ring_name)

    pipe.execute()


def load_map_segmented(r, scene_id):
    # 设施
    fac_set_key = f"map:{scene_id}:facilities"
    facilities = {}
    for name in r.smembers(fac_set_key):
        hkey = f"map:{scene_id}:facility:{name}"
        info = r.hgetall(hkey)
        if "lng" in info and "lat" in info:
            facilities[name] = [float(info["lng"]), float(info["lat"])]

    # 防区环
    ring_set_key = f"map:{scene_id}:rings"
    rings = {}
    for ring_name in r.smembers(ring_set_key):
        lng_key = f"map:{scene_id}:ring:{ring_name}:lngs"
        lat_key = f"map:{scene_id}:ring:{ring_name}:lats"
        lngs = [float(x) for x in r.lrange(lng_key, 0, -1)]
        lats = [float(x) for x in r.lrange(lat_key, 0, -1)]
        rings[ring_name] = {"lngs": lngs, "lats": lats}

    return {"facilities_str": facilities, "defence_rings": rings}


# —— 演示：写入 + 读取 ——
save_map_segmented(r, scene_id, data, build_geo=True)
recovered = load_map_segmented(r, scene_id)
print("radar_1 ->", recovered["facilities_str"]["radar_1"])
print("ring1 lngs len ->", len(recovered["defence_rings"]["ring1"]["lngs"]))
