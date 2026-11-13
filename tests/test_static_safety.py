from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StaticSafetyTests(unittest.TestCase):
    def test_real_robot_and_servo_output_default_off(self):
        env = (ROOT / ".env").read_text()
        self.assertIn("ALLOW_REAL_ROBOT=NO", env)
        self.assertIn("ENABLE_SERVO_OUTPUT=NO", env)

    def test_docker_context_excludes_local_secrets(self):
        dockerignore = (ROOT / ".dockerignore").read_text()
        self.assertIn("**", dockerignore)
        self.assertIn("!Dockerfile", dockerignore)
        self.assertNotIn("!.env", dockerignore)
        self.assertNotIn("!.aws", dockerignore)

    def test_mock_initial_pose_is_mounted_read_only(self):
        compose = (ROOT / "compose.yaml").read_text()
        self.assertIn(
            "./config/ur10_mock_initial_positions.yaml:"
            "/opt/ros/humble/share/ur_description/config/initial_positions.yaml:ro",
            compose,
        )
        self.assertIn("use_fake_hardware:=true", compose)

    def test_control_cli_is_installed(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        self.assertIn("ros-humble-ros2controlcli", dockerfile)

    def test_humble_uses_fake_hardware_launch_argument(self):
        compose = (ROOT / "compose.yaml").read_text()
        real_launcher = (ROOT / "scripts/launch_real_ur.sh").read_text()
        self.assertIn("use_fake_hardware:=true", compose)
        self.assertIn("use_fake_hardware:=false", real_launcher)
        self.assertNotIn("use_mock_hardware", compose + real_launcher)

    def test_real_driver_requires_calibration_and_starts_without_motion_controller(self):
        compose = (ROOT / "compose.yaml").read_text()
        launcher = (ROOT / "scripts/launch_real_ur.sh").read_text()
        self.assertIn("UR_CALIBRATION_FILE", compose)
        self.assertIn("kinematics_params_file", launcher)
        self.assertIn("activate_joint_controller:=false", launcher)

    def test_mock_integration_test_has_multiple_guards(self):
        compose = (ROOT / "compose.yaml").read_text()
        test = (ROOT / "scripts/mock_servo_integration_test.py").read_text()
        self.assertIn('Q2R_MOCK_HARDWARE: "YES"', compose)
        self.assertIn('ALLOW_REAL_ROBOT: "NO"', compose)
        self.assertIn('"CONFIRM_MOCK_ONLY": "YES"', test)
        self.assertIn("mock_components/GenericSystem", test)

    def test_host_validation_keeps_bridge_output_off(self):
        script = (ROOT / "scripts/rebuild_and_validate_mock.sh").read_text()
        self.assertIn("ENABLE_SERVO_OUTPUT=NO", script)
        self.assertIn("ALLOW_REAL_ROBOT=NO", script)
        self.assertIn("ur_real is running", script)


if __name__ == "__main__":
    unittest.main()
