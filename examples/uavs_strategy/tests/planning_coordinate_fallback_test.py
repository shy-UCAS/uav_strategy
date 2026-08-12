# -*- coding: utf-8 -*-
"""
planning_coordinate_fallback_test.py

黄金对比 + 新行为测试（idea 2 统一坐标兜底 / idea 1 plan_routine）。

用法（顺序重要）：
    1) 改造代码前先跑 --snapshot，固化现有行为：
        python planning_coordinate_fallback_test.py --snapshot
    2) 改造代码后跑 --legacy，验证现有行为逐点不变：
        python planning_coordinate_fallback_test.py --legacy
    3) 验证坐标兜底 + routine 新行为：
        python planning_coordinate_fallback_test.py --new

设计说明：
- 所有用例共用固定随机种子（random + np.random），保证 snapshot 与
  后续对比运行的 RNG 序列完全一致；
- "现有行为不变"的判定是逐点严格相等（np.array_equal）：改造是纯增量
  分支，现有分支代码未被触碰，RNG 对齐后输出应当逐点一致；
- 坐标等价用例（--new）在 golden 相同的 RNG 位置调用，因此坐标 target
  与同坐标设施名的轨迹可逐点相等比对。
"""

import argparse
import json
import os
import random
import sys

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_UAV_STRATEGY_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", ".."))
if _UAV_STRATEGY_ROOT not in sys.path:
    sys.path.insert(0, _UAV_STRATEGY_ROOT)

from examples.uavs_strategy.planning_modules import basic_functions as bfunc
from examples.uavs_strategy.planning_modules.uav_planning_actions import PlanningLib

GOLDEN_FILE = os.path.join(_THIS_DIR, "golden_planning_legacy.json")
FACILITIES_FILE = os.path.abspath(os.path.join(_THIS_DIR, "..", "data", "facilities_shaoxing.json"))

RENDEZVOUS_UTM = [427000.0, 4150000.0]  # 与任何设施无关的固定汇合点
SHAOXING_1_LNGLAT = [116.376658, 39.899957]  # 设施 shaoxing_1 中心（坐标兜底的等价目标）
FAR_POINT_LNGLAT = [116.3300, 39.8500]  # 距所有设施 ~2.8km 的任意点

# 与 uav_dynamic_agents02.py 的 height_range_value_set 一致（含 idea 1 新增 routine）
HEIGHT_RANGE_VALUE_SET = {
    "breakthrough": [[250, 400], [0, 100]],
    "escape": [[0, 100], [250, 400]],
    "detour": [[0, 100], [200, 400]],
    "orbit": [[150, 300], [150, 300]],
    "loiter": [[200, 350], [200, 350]],
    "routine": [[200, 300], [200, 300]],
}


class StubIO:
    """aggregate_point 分支需要的极简 io。"""

    def get_rendezvous_point(self, peers):
        return list(RENDEZVOUS_UTM)


class StubAgent:
    """PlanningLib 的最小依赖桩：traj / facilities / io / height_range_set。"""

    def __init__(self, facilities, start_utm):
        self.facilities = facilities
        self.traj = [list(start_utm) + [200.0]]
        self.jid = "golden_test@127.0.0.1"
        self.name = "golden_test"
        self.merge_peers = ["uav_a", "uav_b"]
        self.io = StubIO()
        self.height_range_set = {
            key: [list(lo_hi), list(lo_hi2)] for key, (lo_hi, lo_hi2) in HEIGHT_RANGE_VALUE_SET.items()
        }


def build_fixture():
    """构造 Facilities（shaoxing 空域）+ 起降点（outside 远离设施 / inside 位于合成圆内部）。"""
    with open(FACILITIES_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    facilities = bfunc.Facilities(data["facilities_str"], data["defence_rings"])

    converter = bfunc.LngLat2UTM()
    out_x, out_y = converter.lon_lat_to_utm(116.3500, 39.8700)
    in_x, in_y = converter.lon_lat_to_utm(SHAOXING_1_LNGLAT[0], SHAOXING_1_LNGLAT[1])
    half = bfunc.GlobalBasicConfigs.AVOID_AVERAGE_DISTANCE / 2.0  # 900/2=450m

    starts = {
        "outside": [out_x, out_y],
        "inside": [in_x + half, in_y],  # 严格位于 shaoxing_1 合成圆（r=900m）内部
    }
    lib = PlanningLib(StubAgent(facilities, starts["outside"]))
    return lib, starts


# 现有行为用例（改造前/后都应逐点一致）
LEGACY_CASES = [
    ("breakthrough", "shaoxing_1", "outside"),
    ("breakthrough", "aggregate_point", "outside"),
    ("breakthrough", "unknown_fac", "outside"),
    ("escape", "shaoxing_1", "outside"),
    ("escape", "shaoxing_1", "inside"),
    ("escape", "defence_rings", "outside"),
    ("escape", "unknown_fac", "outside"),
    ("detour", "shaoxing_1", "outside"),
    ("detour", "shaoxing_1_countermeasure", "outside"),
    ("detour", "defence_rings", "outside"),
    ("detour", "unknown_fac", "outside"),
    ("detour", "probe_facilities", "outside"),
    ("orbit", "shaoxing_1", "outside"),
    ("loiter", None, "outside"),
]

# 坐标等价用例：在 golden 相同 RNG 位置调用，结果必须与对应设施名用例逐点相等
MIRROR_COORD_INDEX = {0, 3, 4, 7, 12}


def run_case(lib, starts, mode, target, start_key):
    lib.agent.traj = [starts[start_key] + [200.0]]
    if mode == "breakthrough":
        return lib.execute_breakthrough(target, -1, -1)
    if mode == "escape":
        return lib.execute_escape(target, -1, -1)
    if mode == "detour":
        return lib.execute_detour(target, -1, -1)
    if mode == "orbit":
        return lib.execute_orbit(target, -1, -1)
    if mode == "loiter":
        return lib.execute_loiter(-1, -1)
    raise ValueError("unknown mode: {}".format(mode))


def capture(lib, starts, cases):
    """固定种子跑一遍用例，返回 [{case, ok|raise}]。"""
    random.seed(42)
    np.random.seed(42)
    results = []
    for mode, target, start_key in cases:
        try:
            traj = run_case(lib, starts, mode, target, start_key)
            results.append({"case": [mode, target, start_key], "ok": traj})
        except Exception as exc:
            results.append({
                "case": [mode, target, start_key],
                "raise": type(exc).__name__,
                "msg": str(exc),
            })
    return results


def load_golden():
    with open(GOLDEN_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


def eq_traj(a, b):
    return np.array_equal(np.asarray(a, dtype=float), np.asarray(b, dtype=float))


def _check(label, ok, detail="", failures=None):
    if failures is None:
        failures = []
    status = "PASS" if ok else "FAIL"
    print("[{}] {}".format(status, label))
    if not ok:
        failures.append(label)
        if detail:
            print("        -> {}".format(detail))
    return failures


def cmd_snapshot():
    if os.path.exists(GOLDEN_FILE):
        print("ERROR: golden file already exists, delete it to re-snapshot: {}".format(GOLDEN_FILE))
        return 1
    lib, starts = build_fixture()
    results = capture(lib, starts, LEGACY_CASES)
    with open(GOLDEN_FILE, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=1)
    ok_count = sum(1 for r in results if "ok" in r)
    raise_count = len(results) - ok_count
    print("Snapshot saved: {} cases ({} ok, {} raised)".format(len(results), ok_count, raise_count))
    return 0


def cmd_legacy():
    golden = load_golden()
    lib, starts = build_fixture()
    results = capture(lib, starts, LEGACY_CASES)
    failures = []
    for idx, new_result in enumerate(results):
        old_result = golden[idx]
        mode, target, start_key = LEGACY_CASES[idx]
        label = "legacy[{:02d}] {} target={!r} start={}".format(idx, mode, target, start_key)
        if "ok" in new_result and "ok" in old_result:
            if not eq_traj(new_result["ok"], old_result["ok"]):
                _check(label, False, "trajectory differs from golden (len {} vs {})".format(
                    len(new_result["ok"]), len(old_result["ok"])), failures)
            else:
                _check(label, True, failures=failures)
        elif "raise" in new_result and "raise" in old_result:
            same = (new_result["raise"] == old_result["raise"] and new_result["msg"] == old_result["msg"])
            _check(label, same, "raise mismatch: {!r} vs golden {!r}".format(
                new_result.get("raise"), old_result.get("raise")), failures)
        else:
            _check(label, False, "result kind mismatch: new={} golden={}".format(
                "ok" if "ok" in new_result else "raise", "ok" if "ok" in old_result else "raise"), failures)
    print("\nlegacy: {} cases, {} failures".format(len(results), len(failures)))
    return len(failures)


def cmd_new():
    golden = load_golden()
    lib, starts = build_fixture()
    failures = []

    # ---- 1) 坐标等价：在 golden 相同 RNG 位置调用，须与设施名用例逐点相等 ----
    random.seed(42)
    np.random.seed(42)
    for idx, (mode, target, start_key) in enumerate(LEGACY_CASES):
        use_target = SHAOXING_1_LNGLAT if idx in MIRROR_COORD_INDEX else target
        label = "mirror[{:02d}] {} target=coords(sx1) start={}".format(idx, mode, start_key)
        try:
            traj = run_case(lib, starts, mode, use_target, start_key)
        except Exception as exc:
            old_result = golden[idx]
            if "raise" in old_result:
                same = (type(exc).__name__ == old_result["raise"] and str(exc) == old_result["msg"])
                _check(label, same, "raise mismatch: {!r} vs golden {!r}".format(
                    type(exc).__name__, old_result["raise"]), failures)
            else:
                _check(label, False, "unexpected raise {}: {}".format(type(exc).__name__, exc), failures)
            continue
        old_result = golden[idx]
        if "raise" in old_result:
            _check(label, False, "golden raised but new ok", failures)
        elif not eq_traj(traj, old_result["ok"]):
            _check(label, False, "trajectory differs from golden", failures)
        else:
            _check(label, True, failures=failures)

    # ---- 2) 任意点几何正确性（与实现无关的几何断言） ----
    lib, starts = build_fixture()  # 重置 traj，几何断言不依赖 RNG 位置
    converter = lib.agent.facilities.lnglat_converter
    far_x, far_y = converter.lon_lat_to_utm(FAR_POINT_LNGLAT[0], FAR_POINT_LNGLAT[1])
    start = starts["outside"] + [200.0]
    r = bfunc.GlobalBasicConfigs.AVOID_AVERAGE_DISTANCE

    traj = run_case(lib, starts, "breakthrough", FAR_POINT_LNGLAT, "outside")
    _check("breakthrough -> far coordinate: endpoint == converted UTM point",
           eq_traj(traj[0], start) and np.array_equal(traj[-1][:2], [far_x, far_y])
           and len(traj) >= 2 and np.isfinite(np.asarray(traj, dtype=float)).all(),
           "traj[0]={} traj[-1]={}".format(traj[0], traj[-1]), failures)

    traj = run_case(lib, starts, "detour", FAR_POINT_LNGLAT, "outside")
    # move_along_border 返回 [起始点] + 圆周顶点：起点保持当前位置，终点落在圆上
    detour_last = np.hypot(traj[-1][0] - far_x, traj[-1][1] - far_y)
    _check("detour -> far coordinate: starts at start point, ends on circle r={} +/- 5m".format(r),
           eq_traj(traj[0], start) and abs(detour_last - r) <= 5.0,
           "traj[0]={} traj[-1]={}".format(traj[0], traj[-1]), failures)

    traj = run_case(lib, starts, "escape", FAR_POINT_LNGLAT, "outside")
    _check("escape -> far coordinate: already outside -> stay in place",
           eq_traj(traj[0], start) and eq_traj(traj[-1], start),
           "traj[0]={} traj[-1]={}".format(traj[0], traj[-1]), failures)

    traj = run_case(lib, starts, "orbit", FAR_POINT_LNGLAT, "outside")
    # 起点 z 由高度剖面随机补全，只比 XY；终点是圆上最后一个顶点（角 2π）
    orbit_last = np.hypot(traj[-1][0] - far_x, traj[-1][1] - far_y)
    _check("orbit -> far coordinate: starts at start (xy), ends on 80m circle around point",
           np.array_equal(np.asarray(traj[0][:2], dtype=float), np.asarray(start[:2], dtype=float))
           and abs(orbit_last - 80.0) <= 1e-3,
           "traj[0]={} traj[-1]={}".format(traj[0], traj[-1]), failures)

    # ---- 3) 非法/垃圾输入处理 ----
    lib, starts = build_fixture()
    lib.agent.traj = [starts["outside"] + [200.0]]
    traj = lib.execute_breakthrough(["a", "b"], -1, -1)
    _check("breakthrough non-numeric list -> keep in place (no crash)",
           eq_traj(traj[0], traj[-1]), failures=failures)
    for mode in ("detour", "escape"):
        try:
            run_case(lib, starts, mode, ["a", "b"], "outside")
            _check("{} non-numeric list -> raises ValueError".format(mode), False, "no raise", failures)
        except ValueError:
            _check("{} non-numeric list -> raises ValueError".format(mode), True, failures=failures)
    try:
        run_case(lib, starts, "orbit", ["a", "b"], "outside")
        _check("orbit non-numeric list -> raises KeyError", False, "no raise", failures)
    except KeyError:
        _check("orbit non-numeric list -> raises KeyError", True, failures=failures)
    try:
        run_case(lib, starts, "breakthrough", [999.0, 999.0], "outside")
        _check("breakthrough out-of-range coordinate -> raises ValueError", False, "no raise", failures)
    except ValueError:
        _check("breakthrough out-of-range coordinate -> raises ValueError", True, failures=failures)

    # ---- 4) 任务图接线：digraph_attrs 直接携带坐标 target（execute_path_planning_from_digraph） ----
    random.seed(42)
    np.random.seed(42)
    digraph_coord = {"attrs": {"order_type": "breakthrough", "target": SHAOXING_1_LNGLAT}}
    traj = lib.execute_path_planning_from_digraph(digraph_coord, -1, -1)
    _check("digraph breakthrough target=coords == golden facility trajectory",
           eq_traj(traj, golden[0]["ok"]), failures=failures)

    # ---- 5) idea 1: plan_routine（若已实现） ----
    if not hasattr(PlanningLib, "execute_routine"):
        print("NOTE: execute_routine not present yet, skipping routine assertions")
        print("\nnew: done, {} failures".format(len(failures)))
        return len(failures)

    lib, starts = build_fixture()
    random.seed(42)
    np.random.seed(42)
    # RNG 位置 0：与 golden[0] 对齐 —— xy 应逐点等于 golden breakthrough，z 落在 routine 平飞带
    traj_routine = lib.execute_routine("shaoxing_1", -1, -1)
    golden_xy = np.asarray(golden[0]["ok"], dtype=float)[:, :2]
    traj_routine_np = np.asarray(traj_routine, dtype=float)
    _check("routine(shaoxing_1): xy == golden breakthrough xy",
           np.array_equal(traj_routine_np[:, :2], golden_xy), failures=failures)
    _check("routine(shaoxing_1): z stays in [200, 300]",
           all(200.0 <= z <= 300.0 for z in traj_routine_np[:, 2])
           and np.isfinite(traj_routine_np).all(), failures=failures)

    # generate_breakthrough_flight 的偏转吃 RNG：以下两对比较各自重新播种，
    # 使坐标/设施、分发/直调在相同 RNG 位置运行，可逐点（含高度）相等
    digraph_routine = {"attrs": {"order_type": "routine", "target": "shaoxing_1"}}
    random.seed(42)
    np.random.seed(42)
    traj_routine_coord = lib.execute_routine(SHAOXING_1_LNGLAT, -1, -1)
    _check("routine(coords) == routine(facility) at same RNG position",
           eq_traj(traj_routine_coord, traj_routine), failures=failures)

    random.seed(42)
    np.random.seed(42)
    traj_dispatch = lib.execute_path_planning_from_digraph(digraph_routine, -1, -1)
    _check("digraph order_type=routine dispatch == execute_routine at same RNG position",
           eq_traj(traj_dispatch, traj_routine), failures=failures)

    print("\nnew: done, {} failures".format(len(failures)))
    return len(failures)


def main():
    # conda run 在 Windows 上会吞掉命令行参数，阶段改用环境变量传入
    cmd = os.environ.get("PLANNING_TEST_PHASE")
    if not cmd:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("cmd", choices=["snapshot", "legacy", "new"])
        cmd = parser.parse_args().cmd
    if cmd == "snapshot":
        return cmd_snapshot()
    if cmd == "legacy":
        return cmd_legacy()
    return cmd_new()


if __name__ == "__main__":
    sys.exit(main())
