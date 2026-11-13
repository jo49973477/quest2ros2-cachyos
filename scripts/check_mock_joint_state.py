#!/usr/bin/env python3
"""Compare one live /joint_states sample with the mock startup YAML by name."""
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


REFERENCE = Path("/opt/quest-tools/ur10_mock_initial_positions.yaml")


def load_simple_yaml(path):
    values = {}
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        name, value = line.split(":", 1)
        values[name.strip()] = float(value.strip())
    return values


class JointStateOnce(Node):
    def __init__(self):
        super().__init__("check_ur10_mock_initial_joint_state")
        self.message = None
        self.create_subscription(JointState, "/joint_states", self.callback, 10)

    def callback(self, message):
        self.message = message


def main():
    expected = load_simple_yaml(REFERENCE)
    rclpy.init()
    node = JointStateOnce()
    try:
        deadline = time.monotonic() + 10.0
        while node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            raise RuntimeError("no /joint_states message within 10 seconds")
        actual = dict(zip(node.message.name, node.message.position))
        missing = sorted(set(expected) - set(actual))
        if missing:
            raise RuntimeError(f"joint_states missing joints: {missing}")
        errors = {
            name: (expected[name], actual[name])
            for name in expected
            if not np.isclose(expected[name], actual[name], atol=1e-3)
        }
        if errors:
            raise RuntimeError(f"mock initial joint mismatch: {errors}")
        print("PASS live /joint_states matches mock initial-position YAML")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise
