#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ "${Q2R_MOCK_HARDWARE:-}" == "YES" ]] || fail "run this inside ur_mock"
[[ "${ALLOW_REAL_ROBOT:-}" == "NO" ]] || fail "ALLOW_REAL_ROBOT must be NO"
cmp -s /opt/quest-tools/ur10_mock_initial_positions.yaml \
  /opt/ros/humble/share/ur_description/config/initial_positions.yaml || \
  fail "container initial_positions.yaml does not match the host-mounted reference"
echo "PASS mock initial-position file mount"

/opt/quest-tools/check_mock_joint_state.py

check_param() {
  local name="$1"
  local expected="$2"
  local output
  output="$(ros2 param get /servo_node "moveit_servo.${name}")" || \
    fail "cannot read moveit_servo.${name}"
  [[ "$output" == *"$expected"* ]] || \
    fail "moveit_servo.${name}: expected ${expected}, got ${output}"
  echo "PASS param ${name}=${expected}"
}

controllers="$(ros2 control list_controllers)" || fail "controller_manager unavailable"
grep -Eq '^forward_position_controller[[:space:]].*[[:space:]]active[[:space:]]*$' \
  <<<"$controllers" || fail "forward_position_controller is not active"
echo "PASS controller forward_position_controller=active"

check_param move_group_name ur_manipulator
check_param planning_frame base_link
check_param ee_frame_name tool0
check_param robot_link_command_frame tool0
check_param cartesian_command_in_topic delta_twist_cmds
check_param command_out_topic /forward_position_controller/commands
check_param lower_singularity_threshold 100
check_param hard_stop_singularity_threshold 200
check_param incoming_command_timeout 0.1

[[ "$(ros2 topic type /servo_node/delta_twist_cmds)" == \
  "geometry_msgs/msg/TwistStamped" ]] || fail "unexpected Servo input type"
[[ "$(ros2 topic type /forward_position_controller/commands)" == \
  "std_msgs/msg/Float64MultiArray" ]] || fail "unexpected controller command type"
echo "PASS Servo/controller topic types"

