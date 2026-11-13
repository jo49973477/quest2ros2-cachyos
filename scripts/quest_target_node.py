#!/usr/bin/env python3
"""Calibrated Quest controller pose -> UR10 target pose producer.

Unlike Taokt's generic BaseArmController, this robot-specific node does not
assume Quest tracking axes already equal UR base_link axes. On every enable
edge it anchors the controller pose to the current tool0 pose and computes an
explicit VR-to-robot rotation. No joint or Servo command is published here.
"""
from collections import deque
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from quest2ros.msg import OVR2ROSInputs
from tf2_ros import Buffer, TransformListener

from teleop_math import (
    anchor_calibration,
    average_quaternions,
    finite_vector,
    map_anchored_pose,
    normalize_quaternion,
    validate_rotation_matrix,
)


def parse_fixed_rotation():
    text = os.environ.get("VR_TO_ROBOT_ROTATION", "").strip()
    if not text:
        return None
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) != 9:
        raise ValueError("VR_TO_ROBOT_ROTATION must contain 9 comma-separated values")
    return validate_rotation_matrix(np.asarray(values).reshape(3, 3))


class UR10QuestTarget(Node):
    def __init__(self):
        super().__init__("right_ur10_arm_controller")
        self.base_frame = "base_link"
        self.tool_frame = "tool0"
        self.position_scale = float(os.environ.get("VR_POSITION_SCALE", "1.0"))
        if not np.isfinite(self.position_scale) or self.position_scale <= 0:
            raise ValueError("VR_POSITION_SCALE must be finite and positive")
        self.window_size = int(os.environ.get("QUEST_FILTER_WINDOW", "6"))
        if self.window_size < 1 or self.window_size > 60:
            raise ValueError("QUEST_FILTER_WINDOW must be between 1 and 60")
        self.fixed_rotation = parse_fixed_rotation()
        self.position_history = deque(maxlen=self.window_size)
        self.orientation_history = deque(maxlen=self.window_size)

        self.enabled = False
        self.lower_button_was_pressed = False
        self.pending_anchor = True
        self.vr_anchor_position = None
        self.vr_anchor_orientation = None
        self.robot_anchor_position = None
        self.robot_anchor_orientation = None
        self.vr_to_robot_rotation = None
        self.last_warning = 0.0

        self.target_publisher = self.create_publisher(
            PoseStamped, "/ur10_quest/target_frame", 10
        )
        self.create_subscription(
            PoseStamped, "/q2r_right_hand_pose", self.on_pose, 10
        )
        self.create_subscription(
            OVR2ROSInputs, "/q2r_right_hand_inputs", self.on_inputs, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        mode = "fixed matrix" if self.fixed_rotation is not None else "anchor orientation"
        self.get_logger().warn(
            "TARGET PRODUCER ONLY; pose streaming starts DISABLED. Hold the Quest "
            "controller aligned with tool0 and press button_lower once to calibrate. "
            f"VR->robot calibration mode: {mode}; position scale={self.position_scale:.3f}."
        )

    def warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 2.0:
            self.last_warning = now
            self.get_logger().warn(message)

    def on_inputs(self, message):
        pressed = bool(message.button_lower)
        if pressed and not self.lower_button_was_pressed:
            self.enabled = not self.enabled
            self.position_history.clear()
            self.orientation_history.clear()
            self.pending_anchor = self.enabled
            if self.enabled:
                self.get_logger().warn(
                    "Pose streaming ENABLED; collecting samples and recalibrating "
                    "Quest tracking axes to the current tool0 pose."
                )
            else:
                self.get_logger().warn("Pose streaming DISABLED; target output stopped.")
        self.lower_button_was_pressed = pressed

    def current_tool_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, self.tool_frame, Time()
        ).transform
        position = finite_vector(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            3,
            "tool position",
        )
        orientation = normalize_quaternion([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ])
        return position, orientation

    def on_pose(self, message):
        try:
            position = finite_vector([
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ], 3, "Quest position")
            orientation = normalize_quaternion([
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ])
        except ValueError as error:
            self.warn_throttled(f"Rejected invalid Quest pose: {error}")
            return

        if not self.enabled:
            return
        self.position_history.append(position)
        self.orientation_history.append(orientation)
        if len(self.position_history) < self.window_size:
            return

        filtered_position = np.mean(self.position_history, axis=0)
        try:
            filtered_orientation = average_quaternions(list(self.orientation_history))
            if self.pending_anchor:
                tool_position, tool_orientation = self.current_tool_pose()
                self.vr_anchor_position = filtered_position.copy()
                self.vr_anchor_orientation = filtered_orientation.copy()
                self.robot_anchor_position = tool_position.copy()
                self.robot_anchor_orientation = tool_orientation.copy()
                self.vr_to_robot_rotation = (
                    self.fixed_rotation.copy()
                    if self.fixed_rotation is not None
                    else anchor_calibration(filtered_orientation, tool_orientation)
                )
                self.pending_anchor = False
                self.get_logger().info(
                    "Quest->UR anchor calibrated. R_base_vr="
                    + np.array2string(self.vr_to_robot_rotation, precision=4)
                )

            target_position, target_orientation = map_anchored_pose(
                filtered_position,
                filtered_orientation,
                self.vr_anchor_position,
                self.vr_anchor_orientation,
                self.robot_anchor_position,
                self.robot_anchor_orientation,
                self.vr_to_robot_rotation,
                self.position_scale,
            )
        except Exception as error:
            self.warn_throttled(f"Target pose not published: {error}")
            return

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.base_frame
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(
            float, target_position
        )
        (target.pose.orientation.x,
         target.pose.orientation.y,
         target.pose.orientation.z,
         target.pose.orientation.w) = map(float, target_orientation)
        self.target_publisher.publish(target)


def main():
    rclpy.init()
    node = UR10QuestTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
