"""Run from person_following/: python3 -m unittest discover -s tests -v."""

import math
import unittest

from velocity_controller import ControllerConfig, VelocityCommand, VelocityController


class VelocityControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = VelocityController()

    def test_stops_forward_motion_inside_band_and_when_too_close(self):
        for distance in (0.2, 0.85, 1.0, 1.15):
            with self.subTest(distance=distance):
                self.assertEqual(self.controller.compute(distance, 0), VelocityCommand())

    def test_forward_speed_increases_smoothly_and_is_capped(self):
        edge = self.controller.compute(1.150001, 0).linear_mps
        near = self.controller.compute(1.25, 0).linear_mps
        far = self.controller.compute(10, 0).linear_mps
        self.assertGreater(edge, 0)
        self.assertLess(edge, 0.000001)
        self.assertAlmostEqual(near, 0.05)
        self.assertEqual(far, 0.2)

    def test_left_and_right_steering_are_symmetric(self):
        left = self.controller.compute(2, 0.5)
        right = self.controller.compute(2, -0.5)
        self.assertGreater(left.angular_rps, 0)
        self.assertAlmostEqual(left.angular_rps, -right.angular_rps)
        self.assertAlmostEqual(left.linear_mps, right.linear_mps)
        self.assertLess(left.linear_mps, self.controller.compute(2, 0).linear_mps)

    def test_can_align_without_moving_forward_inside_distance_band(self):
        command = self.controller.compute(1, 0.1)
        self.assertEqual(command.linear_mps, 0)
        self.assertGreater(command.angular_rps, 0)

    def test_large_offset_turns_in_place_with_bounded_angular_speed(self):
        for lateral in (-10, 10):
            with self.subTest(lateral=lateral):
                command = self.controller.compute(2, lateral)
                self.assertEqual(command.linear_mps, 0)
                self.assertEqual(abs(command.angular_rps), 0.5)

    def test_invalid_or_behind_camera_positions_stop(self):
        for position in ((None, 0), ("invalid", 0), (0, 1), (-1, 0),
                         (math.nan, 0), (math.inf, 0), (1, math.nan), (1, math.inf)):
            with self.subTest(position=position):
                self.assertEqual(self.controller.compute(*position), VelocityCommand())

    def test_custom_following_distance(self):
        controller = VelocityController(ControllerConfig(following_distance_m=2))
        self.assertEqual(controller.compute(2.1, 0).linear_mps, 0)
        self.assertGreater(controller.compute(2.3, 0).linear_mps, 0)

    def test_invalid_configuration_is_rejected(self):
        for settings in ({"max_linear_mps": -1}, {"max_angular_rps": math.nan},
                         {"distance_gain": 0}, {"distance_tolerance_m": 1},
                         {"turn_in_place_angle_rad": math.pi / 2}):
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    ControllerConfig(**settings)


if __name__ == "__main__":
    unittest.main()
