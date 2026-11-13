#!/usr/bin/env python3
"""Bounded, fail-closed PoseStamped -> MoveIt Servo Twist bridge.

Default ENABLE_SERVO_OUTPUT=NO means no command ever goes to Servo. VR frame
calibration is performed upstream by quest_target_node.py. This implementation
is for mock validation and is not approved for a physical UR10.
"""
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TwistStamped
from quest2ros.msg import OVR2ROSInputs
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener

from teleop_math import (
    GateInputs,
    finite_vector,
    gate_reason,
    normalize_quaternion,
    pose_error_to_twist,
)


class Bridge(Node):
    def __init__(self):
        super().__init__('ur10_quest_pose_bridge')
        self.preview = self.create_publisher(TwistStamped, '/ur10_quest/twist_preview', 10)
        self.enabled = os.environ.get('ENABLE_SERVO_OUTPUT') == 'YES'
        self.angular_enabled = os.environ.get('ENABLE_ANGULAR_SERVO', 'YES') == 'YES'
        self.timeout = float(os.environ.get('TELEOP_INPUT_TIMEOUT', '0.25'))
        self.tf_timeout = float(os.environ.get('TELEOP_TF_TIMEOUT', '0.50'))
        self.max_linear = float(os.environ.get('MAX_LINEAR_SPEED', '0.025'))
        self.max_angular = float(os.environ.get('MAX_ANGULAR_SPEED', '0.10'))
        if not (0.05 <= self.timeout <= 1.0):
            raise ValueError('TELEOP_INPUT_TIMEOUT must be in [0.05, 1.0] seconds')
        if not (0.05 <= self.tf_timeout <= 1.0):
            raise ValueError('TELEOP_TF_TIMEOUT must be in [0.05, 1.0] seconds')
        if not (0.0 < self.max_linear <= 0.10):
            raise ValueError('MAX_LINEAR_SPEED must be in (0, 0.10] m/s')
        if not (0.0 <= self.max_angular <= 0.50):
            raise ValueError('MAX_ANGULAR_SPEED must be in [0, 0.50] rad/s')
        self.servo_pub = (self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
                          if self.enabled else None)
        self.create_subscription(PoseStamped, '/ur10_quest/target_frame', self.on_pose, 10)
        self.create_subscription(OVR2ROSInputs, '/q2r_right_hand_inputs', self.on_input, 10)
        self.create_subscription(Int8, '/servo_node/status', self.on_servo_status, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal = None
        self.goal_received = None
        self.deadman = False
        self.inputs_received = None
        self.index_press = 0.0
        self.grip_press = 0.0
        self.servo_status = None
        self.last_diagnostic = 0.0
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().warn('Bridge in ' + ('SERVO OUTPUT mode' if self.enabled else 'PREVIEW ONLY mode') +
                               f'; angular commands={"ON" if self.angular_enabled else "OFF"}; '
                               'requires both index and grip triggers continuously held. '
                               'Pose input/TF/timeouts or deadman failure yields zero twist.')

    def on_pose(self, msg):
        if msg.header.frame_id != 'base_link':
            self.get_logger().warn('Target pose not in base_link: ignored')
            return
        try:
            finite_vector([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], 3)
            normalize_quaternion([
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w,
            ])
        except ValueError as error:
            self.get_logger().warn(f'Invalid target pose ignored: {error}')
            return
        self.goal = msg
        self.goal_received = time.monotonic()

    def on_input(self, msg):
        self.index_press = float(msg.press_index)
        self.grip_press = float(msg.press_middle)
        if not np.isfinite(self.index_press) or not np.isfinite(self.grip_press):
            self.index_press = 0.0
            self.grip_press = 0.0
        self.deadman = self.index_press > 0.8 and self.grip_press > 0.8
        self.inputs_received = time.monotonic()

    def on_servo_status(self, msg):
        status = int(msg.data)
        if status != self.servo_status and status != 0:
            descriptions = {
                1: 'approaching singularity',
                2: 'HALT_FOR_SINGULARITY',
                3: 'approaching collision',
                4: 'HALT_FOR_COLLISION',
                5: 'joint bound halt',
                6: 'leaving singularity with deceleration',
            }
            self.get_logger().warn(
                f'MoveIt Servo status={status}: {descriptions.get(status, "unknown")}'
            )
        self.servo_status = status

    def diagnostic(self, now, message, active=False):
        if now - self.last_diagnostic < 2.0:
            return
        self.last_diagnostic = now
        if active:
            self.get_logger().info(message)
        else:
            self.get_logger().warn('Publishing zero preview: ' + message)

    def tick(self):
        now = time.monotonic()
        reason = gate_reason(GateInputs(
            now=now,
            goal_received=self.goal_received,
            inputs_received=self.inputs_received,
            deadman=self.deadman,
            timeout=self.timeout,
        ))
        if reason == 'no target pose':
            reason += '; press button_lower once while aligned with tool0'
        elif reason == 'deadman released':
            reason += (f' (press_index={self.index_press:.2f}, '
                       f'press_middle/grip={self.grip_press:.2f})')
        active = reason is None
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        if active:
            try:
                t = self.tf_buffer.lookup_transform('base_link', 'tool0', Time())
                transform_time = Time.from_msg(t.header.stamp)
                transform_age = (
                    self.get_clock().now() - transform_time
                ).nanoseconds / 1e9
                if transform_age > self.tf_timeout:
                    raise RuntimeError(f'TF is stale ({transform_age:.3f}s old)')
                current = t.transform.translation
                target = self.goal.pose.position
                actual_rot = t.transform.rotation
                target_rot = self.goal.pose.orientation
                linear, angular = pose_error_to_twist(
                    [target.x, target.y, target.z],
                    [target_rot.x, target_rot.y, target_rot.z, target_rot.w],
                    [current.x, current.y, current.z],
                    [actual_rot.x, actual_rot.y, actual_rot.z, actual_rot.w],
                    max_linear=self.max_linear,
                    max_angular=self.max_angular,
                    angular_enabled=self.angular_enabled,
                )
                out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = map(float, linear)
                out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = map(float, angular)
            except Exception as exc:
                # Missing/stale TF; no robot movement command when unavailable.
                reason = f'base_link -> tool0 TF unavailable: {exc}'
                active = False
        if active:
            self.diagnostic(
                now,
                'Preview ACTIVE: '
                f'linear=({out.twist.linear.x:.4f}, {out.twist.linear.y:.4f}, '
                f'{out.twist.linear.z:.4f}), '
                f'angular=({out.twist.angular.x:.4f}, {out.twist.angular.y:.4f}, '
                f'{out.twist.angular.z:.4f})',
                active=True,
            )
        else:
            self.diagnostic(now, reason)
        self.preview.publish(out)
        if self.servo_pub is not None:
            self.servo_pub.publish(out)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
