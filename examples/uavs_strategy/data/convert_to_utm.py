import sys
sys.path.insert(0, r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy')
sys.path.insert(0, r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy\examples\uavs_strategy\planning_modules')
from basic_functions import LngLat2UTM
import json

converter = LngLat2UTM(zone_number=51, north_or_south='N')

with open(r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy\examples\uavs_strategy\data\facilities.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

utm_facilities = {}
for name, lnglat in data['facilities_str'].items():
    x, y = converter.lon_lat_to_utm(lnglat[0], lnglat[1])
    utm_facilities[name] = [round(x, 2), round(y, 2)]

utm_rings = {}
for ring_name, ring in data['defence_rings'].items():
    xs, ys = [], []
    for lng, lat in zip(ring['lngs'], ring['lats']):
        x, y = converter.lon_lat_to_utm(lng, lat)
        xs.append(round(x, 2))
        ys.append(round(y, 2))
    utm_rings[ring_name] = {'xs': xs, 'ys': ys}

result = {'facilities_utm': utm_facilities, 'defence_rings_utm': utm_rings}

out_path = r'f:\CASIA\Drone Swarm Situational Awareness Algorithm\uav_strategy\examples\uavs_strategy\data\facilities_utm.json'
with open(out_path, 'w', encoding='utf-8') as f:
    json.dump(result, f, ensure_ascii=False, indent=2)

print(json.dumps(result, ensure_ascii=False, indent=2))
