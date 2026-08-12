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
    build_segment_common_frames,
    is_analysis_ready_trajectory_point,
    select_analysis_trajectory,
    simulation_time_ms,
    trajectory_timestamps_ms,
)


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
