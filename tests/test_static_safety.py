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

    def test_calibration_overlay_is_pinned_and_checked_at_build_time(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        self.assertIn(
            "UR_CALIBRATION_COMMIT=b8e342428839ca8bcc4a49caf25633a712f9cd2f",
            dockerfile,
        )
        self.assertIn("grep -Fq 'pipeline.init();'", dockerfile)
        self.assertIn("--packages-select ur_calibration", dockerfile)
        self.assertIn(
            "test \"$(ros2 pkg prefix ur_calibration)\" = "
            "'/opt/ur_calibration_ws/install'",
            dockerfile,
        )

    def test_entrypoint_prefers_calibration_overlay(self):
        entrypoint = (ROOT / "scripts/entrypoint.sh").read_text()
        ros_index = entrypoint.index("/opt/ros/humble/setup.bash")
        quest_index = entrypoint.index("/opt/quest_ws/install/setup.bash")
        calibration_index = entrypoint.index(
            "/opt/ur_calibration_ws/install/setup.bash"
        )
        self.assertLess(ros_index, quest_index)
        self.assertLess(quest_index, calibration_index)

    def test_calibration_service_is_read_only_motion_wise(self):
        compose = (ROOT / "compose.yaml").read_text()
        runner = (ROOT / "scripts/run_ur_calibration.sh").read_text()
        host_runner = (ROOT / "scripts/extract_ur_calibration.sh").read_text()
        self.assertIn('ALLOW_REAL_ROBOT: "NO"', compose)
        self.assertIn('ENABLE_SERVO_OUTPUT: "NO"', compose)
        self.assertIn("calibration_correction.launch.py", runner)
        self.assertNotIn("ur_control.launch.py", runner)
        self.assertNotIn("start_servo", runner)
        self.assertIn("timeout --signal=INT", runner)
        self.assertIn("ros2 pkg prefix ur_calibration", runner)
        self.assertIn("ur_real|ur_mock|ur_moveit|ur_target|ur_bridge", host_runner)
        self.assertIn("^ALLOW_REAL_ROBOT=NO$", host_runner)
        self.assertIn("^ENABLE_SERVO_OUTPUT=NO$", host_runner)

    def test_robot_calibration_artifacts_are_ignored(self):
        gitignore = (ROOT / ".gitignore").read_text()
        self.assertIn(".env", gitignore.splitlines())
        self.assertIn("bags/*.yaml", gitignore.splitlines())
        self.assertIn("bags/*.log", gitignore.splitlines())
        self.assertIn("config/ur_calibration.yaml", gitignore.splitlines())

    def test_alias_node_does_not_replace_rclpy_read_only_properties(self):
        alias_node = (ROOT / "scripts/alias_topics.py").read_text()
        self.assertNotIn("self.subscriptions =", alias_node)
        self.assertNotIn("self.publishers =", alias_node)
        self.assertIn("self._alias_subscriptions", alias_node)
        self.assertIn("self._alias_publishers", alias_node)


if __name__ == "__main__":
    unittest.main()
