#!/usr/bin/env bash
set -euo pipefail

if [[ "${ALLOW_REAL_ROBOT:-}" != "I_UNDERSTAND" ]]; then
  echo >&2 '[STOP] This is a physical UR robot. Set ALLOW_REAL_ROBOT=I_UNDERSTAND in .env only after safety checks.'
  exit 2
fi
if [[ ! "${UR_TYPE:-}" =~ ^(ur10|ur10e)$ ]]; then
  echo >&2 '[STOP] UR_TYPE must be ur10 or ur10e.'
  exit 2
fi
if [[ ! "${UR_IP:-}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || [[ "${UR_IP}" == '127.0.0.1' ]]; then
  echo >&2 '[STOP] Set UR_IP to the real robot controller IPv4 address.'
  exit 2
fi
if [[ "${UR_CALIBRATION_FILE:-}" != /config/*.yaml ]] || \
   [[ ! -r "${UR_CALIBRATION_FILE:-}" ]]; then
  echo >&2 '[STOP] UR_CALIBRATION_FILE must name a readable /config/*.yaml file extracted from this robot.'
  exit 2
fi
if ! grep -q 'kinematics' "$UR_CALIBRATION_FILE" || \
   ! grep -q 'hash' "$UR_CALIBRATION_FILE"; then
  echo >&2 '[STOP] Calibration file does not contain expected kinematics/hash data.'
  exit 2
fi

printf 'Starting read-only UR driver bringup for %s at %s; no motion controller is activated.\n' "$UR_TYPE" "$UR_IP"
exec ros2 launch ur_robot_driver ur_control.launch.py \
  "ur_type:=${UR_TYPE}" \
  "robot_ip:=${UR_IP}" \
  "kinematics_params_file:=${UR_CALIBRATION_FILE}" \
  'use_fake_hardware:=false' \
  'activate_joint_controller:=false' \
  'headless_mode:=false' \
  'launch_rviz:=false'
