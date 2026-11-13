#!/usr/bin/env python3
"""Guarded end-to-end MoveIt Servo smoke test for UR fake hardware only."""
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from teleop_math import normalize_quaternion, rotation_vector


HARD_STOP_CODES = {2, 4, 5}


def preflight():
    required = {
        "Q2R_MOCK_HARDWARE": "YES",
        "ALLOW_REAL_ROBOT": "NO",
        "CONFIRM_MOCK_ONLY": "YES",
    }
    for name, expected in required.items():
        if os.environ.get(name) != expected:
            raise RuntimeError(f"refusing test: {name} must equal {expected}")

    description = subprocess.check_output(
        ["ros2", "param", "get", "/robot_state_publisher", "robot_description"],
        text=True,
        timeout=10,
    )
    if "mock_components/GenericSystem" not in description:
        raise RuntimeError("refusing test: robot_description is not GenericSystem mock hardware")

    topic_info = subprocess.check_output(
        ["ros2", "topic", "info", "/servo_node/delta_twist_cmds"],
        text=True,
        timeout=10,
    )
    for line in topic_info.splitlines():
        if line.startswith("Publisher count:") and int(line.split(":", 1)[1]) != 0:
            raise RuntimeError("refusing test: another Servo Twist publisher is active")


class MockServoValidator(Node):
    def __init__(self):
        super().__init__("ur10_mock_servo_integration_test")
        self.joints = None
        self.command_count = 0
        self.statuses = []
        self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
        self.create_subscription(
            Float64MultiArray,
            "/forward_position_controller/commands",
            self.on_command,
            20,
        )
        self.create_subscription(Int8, "/servo_node/status", self.on_status, 20)
        self.publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.start_client = self.create_client(Trigger, "/servo_node/start_servo")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_joints(self, message):
        self.joints = np.asarray(message.position, dtype=float)

    def on_command(self, _message):
        self.command_count += 1

    def on_status(self, message):
        self.statuses.append(int(message.data))

    def spin_for(self, duration, linear=None, angular=None):
        linear = np.zeros(3) if linear is None else np.asarray(linear, dtype=float)
        angular = np.zeros(3) if angular is None else np.asarray(angular, dtype=float)
        end = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                message = TwistStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = "base_link"
                (message.twist.linear.x,
                 message.twist.linear.y,
                 message.twist.linear.z) = map(float, linear)
                (message.twist.angular.x,
                 message.twist.angular.y,
                 message.twist.angular.z) = map(float, angular)
                self.publisher.publish(message)
                next_publish = now + 1.0 / 30.0
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_ready(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joints is not None:
                try:
                    self.tf_buffer.lookup_transform("base_link", "tool0", Time())
                    return
                except Exception:
                    pass
        raise RuntimeError("joint states or base_link->tool0 TF unavailable")

    def tool_pose(self):
        transform = self.tf_buffer.lookup_transform(
            "base_link", "tool0", Time()
        ).transform
        return (
            np.array([transform.translation.x,
                      transform.translation.y,
                      transform.translation.z]),
            normalize_quaternion([transform.rotation.x,
                                  transform.rotation.y,
                                  transform.rotation.z,
                                  transform.rotation.w]),
        )

    def start_servo(self):
        if not self.start_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/servo_node/start_servo unavailable")
        future = self.start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None or not future.result().success:
            raise RuntimeError("MoveIt Servo did not start")

    def assert_no_hard_stop(self, label):
        hard = sorted(set(self.statuses) & HARD_STOP_CODES)
        if hard:
            raise AssertionError(f"{label}: Servo hard-stop status observed: {hard}")


def main():
    preflight()
    rclpy.init()
    node = MockServoValidator()
    try:
        node.wait_ready()
        node.start_servo()

        # Zero Twist must not move fake hardware.
        before = node.joints.copy()
        node.statuses.clear()
        node.spin_for(0.4)
        if np.max(np.abs(node.joints - before)) > 1e-5:
            raise AssertionError("zero Twist changed joint state")
        node.assert_no_hard_stop("zero Twist")
        print("PASS zero Twist")

        # Small Cartesian commands must move the TCP in the commanded base axis.
        for index, name in enumerate("XYZ"):
            before_position, _ = node.tool_pose()
            before_joints = node.joints.copy()
            node.statuses.clear()
            command = np.zeros(3)
            command[index] = 0.005
            node.spin_for(0.6, linear=command)
            after_position, _ = node.tool_pose()
            displacement = after_position - before_position
            node.assert_no_hard_stop(f"+{name}")
            if displacement[index] <= 1e-5:
                raise AssertionError(
                    f"+{name}: wrong TCP direction, displacement={displacement}"
                )
            if np.max(np.abs(node.joints - before_joints)) <= 1e-6:
                raise AssertionError(f"+{name}: joint state did not change")
            print(f"PASS +{name} displacement={displacement}")
            node.spin_for(0.6, linear=-command)

        # Positive base-Z angular command must produce positive base-Z rotation.
        _, before_orientation = node.tool_pose()
        node.statuses.clear()
        node.spin_for(0.5, angular=[0.0, 0.0, 0.02])
        _, after_orientation = node.tool_pose()
        delta_rotation = rotation_vector(after_orientation, before_orientation)
        node.assert_no_hard_stop("+Rz")
        if delta_rotation[2] <= 1e-4:
            raise AssertionError(f"+Rz: wrong rotation direction: {delta_rotation}")
        print(f"PASS +Rz rotation={delta_rotation}")
        node.spin_for(0.5, angular=[0.0, 0.0, -0.02])

        # Publishing loss must leave the joints stationary after Servo timeout.
        node.spin_for(0.2)
        first = node.joints.copy()
        end = time.monotonic() + 0.35
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        second = node.joints.copy()
        if np.max(np.abs(second - first)) > 1e-5:
            raise AssertionError("joint state moved after command stream stopped")
        if node.command_count == 0:
            raise AssertionError("Servo produced no controller commands")
        print("PASS command loss halt and controller output")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise
