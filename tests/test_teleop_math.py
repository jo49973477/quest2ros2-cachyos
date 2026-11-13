import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from teleop_math import (  # noqa: E402
    GateInputs,
    anchor_calibration,
    gate_reason,
    map_anchored_pose,
    matrix_to_quaternion,
    pose_error_to_twist,
    rotation_vector,
)


class TeleopMathTests(unittest.TestCase):
    def test_zero_pose_error_produces_zero_twist(self):
        linear, angular = pose_error_to_twist(
            [0.5, -0.2, 0.3], [0, 0, 0, 1],
            [0.5, -0.2, 0.3], [0, 0, 0, 1],
        )
        np.testing.assert_allclose(linear, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(angular, np.zeros(3), atol=1e-12)

    def test_xyz_direction_mapping(self):
        # +90 degrees about robot Z: VR +X -> robot +Y, VR +Y -> robot -X.
        rotation = np.array([[0.0, -1.0, 0.0],
                             [1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0]])
        identity = [0, 0, 0, 1]
        for delta, expected in (
            ([1, 0, 0], [0, 1, 0]),
            ([0, 1, 0], [-1, 0, 0]),
            ([0, 0, 1], [0, 0, 1]),
        ):
            target, _ = map_anchored_pose(
                delta, identity, [0, 0, 0], identity,
                [0, 0, 0], identity, rotation, position_scale=0.5,
            )
            np.testing.assert_allclose(target, 0.5 * np.asarray(expected), atol=1e-9)

    def test_anchor_calibration_and_rotation_axis(self):
        theta = math.pi / 2
        robot_anchor = matrix_to_quaternion([
            [0, -1, 0], [1, 0, 0], [0, 0, 1]
        ])
        rotation = anchor_calibration([0, 0, 0, 1], robot_anchor)
        np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 1, 0], atol=1e-9)

        vr_x_rotation = [math.sin(theta / 2), 0, 0, math.cos(theta / 2)]
        _, target_q = map_anchored_pose(
            [0, 0, 0], vr_x_rotation,
            [0, 0, 0], [0, 0, 0, 1],
            [0, 0, 0], robot_anchor, rotation,
        )
        # Conjugating VR X by Rz(+90) yields robot Y rotation.
        np.testing.assert_allclose(
            rotation_vector(target_q, robot_anchor), [0, theta, 0], atol=1e-8
        )

    def test_twist_is_bounded(self):
        linear, angular = pose_error_to_twist(
            [10, 0, 0], [0, 0, 1, 0],
            [0, 0, 0], [0, 0, 0, 1],
            max_linear=0.025, max_angular=0.1,
        )
        self.assertAlmostEqual(np.linalg.norm(linear), 0.025)
        self.assertAlmostEqual(np.linalg.norm(angular), 0.1)

    def test_angular_output_can_be_disabled(self):
        linear, angular = pose_error_to_twist(
            [0.01, 0, 0], [0, 0, 1, 0],
            [0, 0, 0], [0, 0, 0, 1], angular_enabled=False,
        )
        self.assertGreater(linear[0], 0)
        np.testing.assert_array_equal(angular, np.zeros(3))

    def test_nan_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            pose_error_to_twist(
                [float("nan"), 0, 0], [0, 0, 0, 1],
                [0, 0, 0], [0, 0, 0, 1],
            )
        with self.assertRaises(ValueError):
            pose_error_to_twist(
                [0, 0, 0], [0, 0, 0, 0],
                [0, 0, 0], [0, 0, 0, 1],
            )

    def test_deadman_and_tracking_loss_stop(self):
        self.assertEqual(gate_reason(GateInputs(10.0, 9.9, 9.9, False, 0.25)),
                         "deadman released")
        self.assertEqual(gate_reason(GateInputs(10.0, 9.0, 9.9, True, 0.25)),
                         "target pose stale")
        self.assertEqual(gate_reason(GateInputs(10.0, 9.9, 9.0, True, 0.25)),
                         "controller input stale")
        self.assertIsNone(gate_reason(GateInputs(10.0, 9.9, 9.9, True, 0.25)))


if __name__ == "__main__":
    unittest.main()
