#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import unittest

from examples.uavs_strategy.behaviors_modules.uav_periodic_behaviours import (
    CLOSE_TH_SYNC,
    DT,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
    MAX_HORIZONTAL_SPEED_MPS,
    bounded_motion_step,
    SyncAPFStepEnhance,
)
from examples.uavs_strategy.uav_dynamic_agents02 import (
    build_complete_flight_plan_export,
    build_mission_meta,
    build_segment_common_frames,
    is_analysis_ready_trajectory_point,
    select_analysis_trajectory,
    simulation_time_ms,
    trajectory_timestamps_ms,
)


class IdentityLngLatConverter:
    """Treat test x/y values as lng/lat so route stitching can be tested alone."""

    @staticmethod
    def utm_to_lng_lat_array(trajectory):
        return [[point[0], point[1]] for point in trajectory]


class TrajectoryPhaseContractTests(unittest.TestCase):
    def test_only_task_flight_is_analysis_ready(self):
        initializing = {
            "segment_key": "initializing",
            "frame_id": "0 initializing",
            "lookahead": "0",
            "is_waiting": True,
            "flight_phase": "initializing",
        }
        positioning = {
            "segment_key": "0_3",
            "frame_id": 0,
            "lookahead": 0,
            "is_waiting": False,
            "flight_phase": "positioning",
        }
        sync_wait = {
            "segment_key": "0_3",
            "frame_id": None,
            "lookahead": 0,
            "is_waiting": True,
            "flight_phase": "sync_wait",
        }
        task_flight = {
            "segment_key": "0_3",
            "frame_id": 1,
            "lookahead": 1,
            "is_waiting": False,
            "flight_phase": "task_flight",
        }

        self.assertFalse(is_analysis_ready_trajectory_point(initializing))
        self.assertFalse(is_analysis_ready_trajectory_point(positioning))
        self.assertFalse(is_analysis_ready_trajectory_point(sync_wait))
        self.assertTrue(is_analysis_ready_trajectory_point(task_flight))

    def test_legacy_false_string_remains_readable(self):
        legacy_task_point = {
            "segment_key": "0_3",
            "frame_id": 1,
            "lookahead": 1,
            "is_waiting": "False",
        }
        legacy_positioning = {
            "segment_key": "0_3",
            "frame_id": 0,
            "lookahead": 0,
            "is_waiting": "flying to start",
        }
        self.assertTrue(is_analysis_ready_trajectory_point(legacy_task_point))
        self.assertFalse(is_analysis_ready_trajectory_point(legacy_positioning))

    def test_synced_selection_drops_initial_and_keeps_last_physical_sample(self):
        init = {
            "segment_key": "initializing",
            "frame_id": "0 initializing",
            "lookahead": "0",
            "is_waiting": True,
            "flight_phase": "initializing",
        }

        def task(frame_id, sim_time_ms):
            return {
                "segment_key": "0_3",
                "frame_id": frame_id,
                "lookahead": frame_id,
                "is_waiting": False,
                "flight_phase": "task_flight",
                "simTimeMs": sim_time_ms,
            }

        raw = {
            "a": (
                [[0.0, 0.0, 0.0], [8.0, 0.0, 0.0], [16.0, 0.0, 0.0], [24.0, 0.0, 0.0]],
                [init, task(1, 1_000), task(1, 1_500), task(2, 2_000)],
            ),
            "b": (
                [[0.0, 1.0, 0.0], [8.0, 1.0, 0.0], [16.0, 1.0, 0.0]],
                [init, task(1, 1_000), task(2, 2_000)],
            ),
        }

        common = build_segment_common_frames(raw)
        self.assertEqual(common, {"0_3": {1, 2}})

        selected_points, selected_extras = select_analysis_trajectory(
            raw["a"][0], raw["a"][1], common
        )
        self.assertEqual(selected_points, [[16.0, 0.0, 0.0], [24.0, 0.0, 0.0]])
        self.assertEqual([item["frame_id"] for item in selected_extras], [1, 2])
        self.assertNotIn("initializing", [item["segment_key"] for item in selected_extras])


class SimulationClockTests(unittest.TestCase):
    def test_round_clock_is_exactly_500_ms(self):
        start = 1_786_453_580_000
        timestamps = [simulation_time_ms(start, round_id, 500) for round_id in range(4)]
        self.assertEqual(
            timestamps,
            [start, start + 500, start + 1_000, start + 1_500],
        )
        self.assertEqual(
            trajectory_timestamps_ms([{"simTimeMs": value} for value in timestamps]),
            timestamps,
        )


class CompleteFlightPlanExportTests(unittest.TestCase):
    def test_two_segments_are_stitched_and_duplicate_boundary_is_removed(self):
        trajectories = {
            ("0", "1", "agent_1_0"): [[1.0, 2.0, 100.0], [3.0, 4.0, 110.0]],
            ("1", "2", "agent_1_0"): [[3.0, 4.0, 110.0], [5.0, 6.0, 120.0]],
        }

        exported = build_complete_flight_plan_export(
            [("0", "1"), ("1", "2")],
            "agent_1_0",
            lambda start, end, member: trajectories.get((start, end, member), []),
            IdentityLngLatConverter(),
        )

        self.assertTrue(exported["complete"])
        self.assertEqual(exported["segmentCount"], 2)
        self.assertEqual(exported["routePointCount"], 3)
        self.assertEqual(
            exported["flightRoute"],
            [
                {"lng": 1.0, "lat": 2.0, "alt": 100.0},
                {"lng": 3.0, "lat": 4.0, "alt": 110.0},
                {"lng": 5.0, "lat": 6.0, "alt": 120.0},
            ],
        )
        self.assertEqual(
            [item["segmentKey"] for item in exported["flightPlan"]],
            ["0_1", "1_2"],
        )

    def test_non_matching_segment_boundary_is_preserved(self):
        trajectories = {
            ("0", "1", "agent"): [[1.0, 2.0, 100.0], [3.0, 4.0, 110.0]],
            ("1", "2", "agent"): [[30.0, 40.0, 115.0], [5.0, 6.0, 120.0]],
        }
        exported = build_complete_flight_plan_export(
            [("0", "1"), ("1", "2")],
            "agent",
            lambda start, end, member: trajectories.get((start, end, member), []),
            IdentityLngLatConverter(),
        )

        self.assertEqual(exported["routePointCount"], 4)
        self.assertEqual(exported["flightRoute"][2]["lng"], 30.0)

    def test_missing_segment_marks_export_incomplete_without_losing_other_segments(self):
        trajectories = {
            ("0", "1", "agent"): [[1.0, 2.0, 100.0], [3.0, 4.0, 110.0]],
        }
        exported = build_complete_flight_plan_export(
            [("0", "1"), ("1", "2")],
            "agent",
            lambda start, end, member: trajectories.get((start, end, member), []),
            IdentityLngLatConverter(),
        )

        self.assertFalse(exported["complete"])
        self.assertEqual(exported["routePointCount"], 2)
        self.assertEqual(exported["missingSegments"][0]["segmentKey"], "1_2")


class MissionMetaExportTests(unittest.TestCase):
    def _planned_routes(self):
        return {
            name: {"flightPlan": [{"order": 0, "segmentKey": key, "fromNode": key[0], "toNode": key[-1]}]}
            for name, key in (
                ("agent_1_0", "0_3"), ("agent_1_1", "0_3"), ("agent_1_2", "0_3"),
                ("agent_2_0", "1_4"), ("agent_2_1", "1_4"),
                ("agent_3_0", "2_5"),
            )
        }

    def _digraph_attrs(self):
        return [
            {"from": 0, "to": 3, "members_num": 2,
             "attrs": {"order_mode": "singleton", "order_type": "detour",
                       "target": "shaoxing_1", "fleet_no": "sx1.1"}},
            {"from": 1, "to": 4, "members_num": 1,
             "attrs": {"order_mode": "singleton", "order_type": "detour",
                       "target": "shaoxing_2", "fleet_no": "sx2.1"}},
            {"from": 2, "to": 5, "members_num": 0,
             "attrs": {"order_mode": "singleton", "order_type": "breakthrough",
                       "target": "shaoxing_3", "fleet_no": "sx3.1"}},
        ]

    def test_swarms_group_members_by_segment_and_keep_design_attrs(self):
        # 每集群首个成员的 extras 携带 leader_id，其他成员无 leader_id，
        # 用于验证 leaderId 来自运行期 extras 而不是按名称猜测。
        raw_trajs = {
            name: ([[0.0, 0.0, 100.0]], [{"leader_id": "nobody"}])
            for name in self._planned_routes()
        }
        raw_trajs["agent_1_0"][1][0]["leader_id"] = "agent_1_0"
        raw_trajs["agent_2_0"][1][0]["leader_id"] = "agent_2_0"
        raw_trajs["agent_3_0"][1][0]["leader_id"] = "agent_3_0"

        meta = build_mission_meta(self._digraph_attrs(), self._planned_routes(), raw_trajs)
        swarms = {item["segmentKey"]: item for item in meta["swarms"]}
        self.assertEqual(meta["source"], "digraph_attrs")
        self.assertEqual(sorted(swarms["0_3"]["memberIds"]), ["agent_1_0", "agent_1_1", "agent_1_2"])
        self.assertEqual(swarms["0_3"]["orderType"], "detour")
        self.assertEqual(swarms["0_3"]["target"], "shaoxing_1")
        self.assertEqual(swarms["0_3"]["fleetNo"], "sx1.1")
        self.assertEqual(swarms["0_3"]["leaderId"], "agent_1_0")
        self.assertEqual(swarms["2_5"]["orderType"], "breakthrough")
        self.assertEqual(swarms["2_5"]["memberIds"], ["agent_3_0"])

    def test_leader_falls_back_to_first_member_when_extras_lack_leader_id(self):
        raw_trajs = {name: ([], [{"flight_phase": "task_flight"}]) for name in self._planned_routes()}
        meta = build_mission_meta(self._digraph_attrs(), self._planned_routes(), raw_trajs)
        by_segment = {item["segmentKey"]: item for item in meta["swarms"]}
        self.assertEqual(by_segment["0_3"]["leaderId"], "agent_1_0")
        self.assertEqual(by_segment["2_5"]["leaderId"], "agent_3_0")

    def test_targets_without_digraph_edge_go_to_fallback_swarm(self):
        planned = self._planned_routes()
        planned["agent_9_9"] = {"flightPlan": [{"order": 0, "segmentKey": "7_8", "fromNode": "7", "toNode": "8"}]}
        raw_trajs = {name: ([], []) for name in planned}
        meta = build_mission_meta(self._digraph_attrs(), planned, raw_trajs)
        fallback = [item for item in meta["swarms"] if item["segmentKey"] is None]
        self.assertEqual(len(fallback), 1)
        self.assertEqual(fallback[0]["memberIds"], ["agent_9_9"])
        self.assertIsNone(fallback[0]["orderType"])
        self.assertIn("note", fallback[0])

    def test_missing_digraph_attrs_are_tolerated(self):
        raw_trajs = {name: ([], []) for name in self._planned_routes()}
        meta = build_mission_meta([{"from": 0, "to": 3}], self._planned_routes(), raw_trajs)
        self.assertEqual(meta["swarms"][0]["orderType"], None)
        self.assertEqual(meta["swarms"][0]["segmentKey"], "0_3")


class PhysicalStepTests(unittest.TestCase):
    def test_horizontal_and_vertical_motion_are_bounded(self):
        current = [0.0, 0.0, 0.0]
        target = [80.0, 0.0, 20.0]
        nxt = bounded_motion_step(current, target)

        horizontal_step = math.hypot(nxt[0] - current[0], nxt[1] - current[1])
        self.assertAlmostEqual(horizontal_step / DT, MAX_HORIZONTAL_SPEED_MPS)
        self.assertAlmostEqual((nxt[2] - current[2]) / DT, MAX_CLIMB_RATE_MPS)

    def test_long_jump_is_replaced_by_physical_round_steps(self):
        current = [0.0, 0.0, 200.0]
        target = [4_000.0, 3_000.0, 100.0]

        for _ in range(20):
            nxt = bounded_motion_step(current, target)
            horizontal_step = math.hypot(nxt[0] - current[0], nxt[1] - current[1])
            vertical_step = nxt[2] - current[2]
            self.assertLessEqual(horizontal_step, MAX_HORIZONTAL_SPEED_MPS * DT + 1e-9)
            self.assertLessEqual(vertical_step, MAX_CLIMB_RATE_MPS * DT + 1e-9)
            self.assertGreaterEqual(vertical_step, -MAX_DESCENT_RATE_MPS * DT - 1e-9)
            current = nxt

    def test_short_step_does_not_overshoot(self):
        current = [0.0, 0.0, 10.0]
        target = [3.0, 4.0, 9.0]
        self.assertEqual(bounded_motion_step(current, target), target)

    def test_final_waypoint_does_not_complete_while_still_far_away(self):
        class DummyAgent:
            current_segment_id = "segment-1"
            is_final_task = True
            is_finished = False

        class DummyRedis:
            def set(self, *_args, **_kwargs):
                raise AssertionError("completion state must not be written while target is far away")

        class DummyIO:
            r = DummyRedis()

        behaviour = SyncAPFStepEnhance(period=DT)
        completed = behaviour._check_task_completion(
            DummyAgent(), DummyIO(), lookahead=5, max_idx=5, dist_to_target=CLOSE_TH_SYNC + 1.0
        )
        self.assertFalse(completed)


if __name__ == "__main__":
    unittest.main()
