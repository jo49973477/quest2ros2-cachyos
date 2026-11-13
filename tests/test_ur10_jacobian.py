from pathlib import Path
import math
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ur10_jacobian import singularity_metrics  # noqa: E402


class UR10JacobianTests(unittest.TestCase):
    def test_mock_yaml_matches_validated_pose_and_nominal_joint_limits(self):
        config = Path(__file__).resolve().parents[1] / "config" / \
            "ur10_mock_initial_positions.yaml"
        values = {}
        for raw_line in config.read_text().splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if line:
                name, value = line.split(":", 1)
                values[name.strip()] = float(value)
        ordered = [values[name] for name in (
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        )]
        np.testing.assert_allclose(
            ordered,
            [0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0],
            atol=1e-5,
        )
        self.assertTrue(all(-2 * math.pi <= value <= 2 * math.pi
                            for value in ordered))

    def test_default_elbow_and_wrist_zero_pose_is_singular(self):
        values, condition = singularity_metrics(
            [0, -math.pi / 2, 0, -math.pi / 2, 0, 0]
        )
        self.assertLess(values[-1], 1e-12)
        self.assertTrue(np.isinf(condition))

    def test_motion_away_from_singularity_improves_condition(self):
        _, near_hard_stop = singularity_metrics(
            [0, -math.pi / 2, 0.05, -math.pi / 2, -0.05, 0]
        )
        _, farther = singularity_metrics(
            [0, -math.pi / 2, 0.20, -math.pi / 2, -0.20, 0]
        )
        self.assertGreater(near_hard_stop, 200.0)
        self.assertLess(farther, 100.0)
        self.assertLess(farther, near_hard_stop)

    def test_configured_mock_pose_is_well_conditioned(self):
        values, condition = singularity_metrics(
            [0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0]
        )
        np.testing.assert_allclose(
            values,
            [1.95951358, 1.51136045, 1.00422453,
             0.56631282, 0.44848241, 0.31901760],
            rtol=1e-6,
        )
        self.assertAlmostEqual(condition, 6.1423369, places=6)
        self.assertLess(condition, 100.0)


if __name__ == "__main__":
    unittest.main()
