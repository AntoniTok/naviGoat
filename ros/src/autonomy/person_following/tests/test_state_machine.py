"""Simulated-clock checks; run python3 -B -m unittest discover -s tests -v."""

import math
import unittest

from state_machine import (FollowingConfig, FollowingState, FollowingStateMachine,
                           PerceptionFrame, PersonObservation)
from velocity_controller import VelocityCommand


def frame(stamp, track_id=7, forward=2.0, lateral=0.3,
          observed=True, confirmed=True):
    return PerceptionFrame(stamp, (PersonObservation(
        track_id, forward, lateral, observed, confirmed),))


class StateMachineTests(unittest.TestCase):
    def setUp(self):
        self.machine = FollowingStateMachine()

    def start_moving(self):
        self.machine.enable()
        self.machine.update(0, frame(0))
        self.machine.update(0.25, frame(0.25))
        result = self.machine.update(0.5, frame(0.5))
        self.assertGreater(result.command.linear_mps, 0)
        self.assertGreater(result.command.angular_rps, 0)
        return result.command

    def test_idle_ignores_people_until_explicit_enable(self):
        result = self.machine.update(0, frame(0))
        self.assertEqual(result.state, FollowingState.IDLE)
        self.assertEqual(result.command, VelocityCommand())
        self.assertIsNone(result.target_id)

    def test_acquisition_requires_confirmed_observation(self):
        self.machine.enable()
        for stamp, observation in ((0, frame(0, confirmed=False)),
                                   (0.1, frame(0.1, observed=False))):
            result = self.machine.update(stamp, observation)
            self.assertEqual(result.state, FollowingState.ACQUIRING)
            self.assertEqual(result.command, VelocityCommand())
        result = self.machine.update(0.2, frame(0.2))
        self.assertEqual(result.state, FollowingState.FOLLOWING)
        self.assertEqual(result.target_id, 7)
        self.assertEqual(result.event, "target_acquired")

    def test_startup_and_repeated_enable_preserve_target(self):
        self.machine.enable()
        first = self.machine.update(0, frame(0))
        self.assertEqual(first.command, VelocityCommand())
        second = self.machine.update(0.1, frame(0.1))
        self.assertAlmostEqual(second.command.linear_mps, 0.04)
        self.assertEqual(self.machine.enable().target_id, 7)

    def test_loss_ramps_both_velocities_to_zero_in_half_second(self):
        moving = self.start_moving()
        missing = self.machine.update(0.6, PerceptionFrame(0.6))
        self.assertEqual(missing.event, "target_missing")
        self.assertEqual(missing.command, moving)
        halfway = self.machine.update(0.85, PerceptionFrame(0.85))
        self.assertAlmostEqual(halfway.command.linear_mps, moving.linear_mps / 2)
        self.assertAlmostEqual(halfway.command.angular_rps, moving.angular_rps / 2)
        stopped = self.machine.update(1.1, PerceptionFrame(1.1))
        self.assertEqual(stopped.command, VelocityCommand())
        self.assertEqual(stopped.state, FollowingState.TEMPORARILY_LOST)

    def test_prediction_does_not_refresh_sighting(self):
        self.start_moving()
        missing = self.machine.update(0.6, frame(0.6, observed=False))
        self.assertEqual(missing.state, FollowingState.TEMPORARILY_LOST)
        for stamp in (1.0, 1.4, 1.8, 2.2):
            self.machine.update(stamp, frame(stamp, observed=False))
        lost = self.machine.update(2.5, frame(2.5, observed=False))
        self.assertEqual(lost.event, "target_lost")

    def test_same_id_returns_and_accelerates_smoothly(self):
        self.start_moving()
        self.machine.update(0.6, PerceptionFrame(0.6))
        self.machine.update(1.1, PerceptionFrame(1.1))
        result = self.machine.update(1.2, frame(1.2))
        self.assertEqual(result.event, "target_reacquired")
        self.assertEqual(result.state, FollowingState.FOLLOWING)
        self.assertAlmostEqual(result.command.linear_mps, 0.04)

    def test_different_id_cannot_replace_selected_target(self):
        self.start_moving()
        result = self.machine.update(0.6, frame(0.6, track_id=8))
        self.assertEqual(result.state, FollowingState.TEMPORARILY_LOST)
        self.assertEqual(result.target_id, 7)

    def test_timeout_event_is_once_and_requires_reenable(self):
        self.start_moving()
        for stamp in (0.6, 1.0, 1.4, 1.8, 2.2):
            self.machine.update(stamp, PerceptionFrame(stamp))
        # Even the same ID returning at the deadline must not resume following.
        lost = self.machine.update(2.5, frame(2.5))
        self.assertEqual(lost.state, FollowingState.IDLE)
        self.assertEqual(lost.event, "target_lost")
        self.assertIsNone(lost.target_id)
        self.assertEqual(lost.command, VelocityCommand())
        self.assertIsNone(self.machine.update(2.6, frame(2.6)).event)
        self.assertEqual(self.machine.state, FollowingState.IDLE)
        self.machine.enable()
        self.assertEqual(self.machine.update(2.7, frame(2.7)).event, "target_acquired")

    def test_no_new_frame_watchdog_stops_without_further_callbacks(self):
        self.start_moving()
        self.machine.update(0.7)
        result = self.machine.update(1.01)
        self.assertEqual(result.event, "input_stale")
        self.assertEqual(result.state, FollowingState.IDLE)
        self.assertEqual(result.command, VelocityCommand())

    def test_duplicate_and_out_of_order_frames_do_not_refresh_watchdog(self):
        self.start_moving()
        self.machine.update(0.8, frame(0.5))
        result = self.machine.update(1.01, frame(0.4))
        self.assertEqual(result.event, "input_stale")
        self.assertEqual(result.command, VelocityCommand())

    def test_disable_and_camera_failure_stop_immediately(self):
        for failure in ("disable", "camera"):
            with self.subTest(failure=failure):
                self.setUp()
                self.start_moving()
                result = (self.machine.disable() if failure == "disable" else
                          self.machine.update(0.6, input_healthy=False))
                self.assertEqual(result.state, FollowingState.IDLE)
                self.assertEqual(result.command, VelocityCommand())

    def test_invalid_depth_stops_immediately(self):
        for forward, lateral in ((math.nan, 0), (0, 0), (-1, 0), (2, math.inf)):
            with self.subTest(position=(forward, lateral)):
                self.setUp()
                self.start_moving()
                result = self.machine.update(0.6, frame(0.6, forward=forward, lateral=lateral))
                self.assertEqual(result.event, "input_fault")
                self.assertEqual(result.command, VelocityCommand())

    def test_additional_person_does_not_interrupt_selected_target(self):
        self.start_moving()
        people = (PersonObservation(7, 2, 0),
                  PersonObservation(8, 3, 0.1, confirmed=False))
        result = self.machine.update(0.6, PerceptionFrame(0.6, people))
        self.assertEqual(result.state, FollowingState.FOLLOWING)
        self.assertEqual(result.target_id, 7)
        self.assertGreater(result.command.linear_mps, 0)

    def test_acquires_nearest_by_horizontal_range_regardless_of_list_order(self):
        # ID 8 has smaller forward depth but greater horizontal range.
        people = (PersonObservation(8, 2, 2), PersonObservation(7, 2.5, 0))
        for ordered in (people, tuple(reversed(people))):
            with self.subTest(order=ordered):
                machine = FollowingStateMachine()
                machine.enable()
                result = machine.update(0, PerceptionFrame(0, ordered))
                self.assertEqual(result.target_id, 7)
                self.assertEqual(result.event, "target_acquired")

    def test_nearest_prediction_or_tentative_person_is_not_selected(self):
        self.machine.enable()
        people = (PersonObservation(8, 0.5, 0, observed=False),
                  PersonObservation(9, 0.7, 0, confirmed=False),
                  PersonObservation(7, 2, 0))
        result = self.machine.update(0, PerceptionFrame(0, people))
        self.assertEqual(result.target_id, 7)

    def test_equal_distance_tie_break_does_not_depend_on_input_order(self):
        people = (PersonObservation(8, 2, -1), PersonObservation(7, 2, 1))
        for ordered in (people, tuple(reversed(people))):
            with self.subTest(order=ordered):
                machine = FollowingStateMachine()
                machine.enable()
                self.assertEqual(machine.update(0, PerceptionFrame(0, ordered)).target_id, 7)

    def test_closer_person_cannot_steal_target_lock(self):
        self.start_moving()
        for stamp, selected_distance in ((0.6, 2), (0.8, 3)):
            people = (PersonObservation(8, 0.8, 0),
                      PersonObservation(7, selected_distance, 0))
            result = self.machine.update(stamp, PerceptionFrame(stamp, people))
            self.assertEqual(result.state, FollowingState.FOLLOWING)
            self.assertEqual(result.target_id, 7)
            self.assertIsNone(result.event)
            self.assertGreater(result.command.linear_mps, 0)

    def test_missing_target_is_not_replaced_by_nearest_remaining_person(self):
        self.start_moving()
        people = (PersonObservation(8, 0.8, 0), PersonObservation(9, 1.5, 0))
        result = self.machine.update(0.6, PerceptionFrame(0.6, people))
        self.assertEqual(result.state, FollowingState.TEMPORARILY_LOST)
        self.assertEqual(result.target_id, 7)
        self.assertEqual(result.event, "target_missing")

    def test_exit_and_new_enable_allow_new_nearest_selection(self):
        self.start_moving()
        exited = self.machine.disable()
        self.assertEqual(exited.command, VelocityCommand())
        self.assertIsNone(exited.target_id)
        self.machine.enable()
        people = (PersonObservation(7, 2, 0), PersonObservation(8, 1, 0))
        result = self.machine.update(0.6, PerceptionFrame(0.6, people))
        self.assertEqual(result.target_id, 8)
        self.assertEqual(result.event, "target_acquired")

    def test_stale_or_future_frame_cannot_start_movement(self):
        for stamp, reason in ((0, "input_stale"), (2, "input_fault")):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.machine.enable()
                result = self.machine.update(1, frame(stamp))
                self.assertEqual(result.event, reason)
                self.assertEqual(result.command, VelocityCommand())

    def test_backward_or_nonfinite_clock_stops_and_raises(self):
        for stamp in (0.4, math.nan, math.inf):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.start_moving()
                with self.assertRaises(ValueError):
                    self.machine.update(stamp)
                self.assertEqual(self.machine.state, FollowingState.IDLE)
                self.assertEqual(self.machine.command, VelocityCommand())

    def test_invalid_config_rejected(self):
        for settings in ({"loss_ramp_s": 0}, {"input_timeout_s": math.nan},
                         {"loss_timeout_s": 0.5}, {"linear_acceleration_mps2": -1}):
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    FollowingConfig(**settings)


if __name__ == "__main__":
    unittest.main()
