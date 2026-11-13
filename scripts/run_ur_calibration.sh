#!/usr/bin/env bash
set -euo pipefail

readonly overlay=/opt/ur_calibration_ws
readonly expected_commit=b8e342428839ca8bcc4a49caf25633a712f9cd2f
readonly timeout_seconds="${CALIBRATION_TIMEOUT:-45}"

fail() {
  printf '[FAIL] %s\n' "$*" >&2
  exit 2
}

[[ "${ALLOW_REAL_ROBOT:-NO}" == NO ]] || fail 'ALLOW_REAL_ROBOT must remain NO for calibration extraction.'
[[ "${ENABLE_SERVO_OUTPUT:-NO}" == NO ]] || fail 'ENABLE_SERVO_OUTPUT must remain NO for calibration extraction.'
[[ "${UR_IP:-}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || fail 'UR_IP must be an IPv4 address.'
[[ "${timeout_seconds}" =~ ^[1-9][0-9]*$ ]] || fail 'CALIBRATION_TIMEOUT must be a positive integer.'

actual_prefix="$(ros2 pkg prefix ur_calibration)"
[[ "${actual_prefix}" == "${overlay}/install" ]] || \
  fail "Wrong ur_calibration prefix: ${actual_prefix}"
[[ "$(<"${overlay}/UPSTREAM_COMMIT")" == "${expected_commit}" ]] || \
  fail 'Unexpected ur_calibration source commit.'
grep -Fq 'pipeline.init();' \
  "${overlay}/src/Universal_Robots_ROS2_Driver/ur_calibration/src/calibration_correction.cpp" || \
  fail 'The pipeline.init() fix is absent from the source used to build the overlay.'

python3 - "${UR_IP}" <<'PY'
import socket
import sys

robot_ip = sys.argv[1]
try:
    with socket.create_connection((robot_ip, 30001), timeout=3):
        pass
except OSError as exc:
    raise SystemExit(f"[FAIL] Primary Interface {robot_ip}:30001 is unavailable: {exc}")
print(f"[OK] Primary Interface reachable at {robot_ip}:30001")
PY

mkdir -p /bags /config
readonly temp_file="/bags/.ur_calibration.$$.yaml"
cleanup() {
  rm -f -- "${temp_file}"
}
trap cleanup EXIT INT TERM

printf '[INFO] Extracting factory calibration only; no robot motion stack is started.\n'
set +e
timeout --signal=INT --kill-after=5s "${timeout_seconds}s" \
  ros2 launch ur_calibration calibration_correction.launch.py \
    "robot_ip:=${UR_IP}" "target_filename:=${temp_file}"
status=$?
set -e
if (( status == 124 || status == 137 )); then
  fail "Calibration extraction timed out after ${timeout_seconds}s."
fi
(( status == 0 )) || fail "Calibration extraction exited with status ${status}."

python3 - "${temp_file}" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
if not path.is_file() or path.stat().st_size == 0:
    raise SystemExit("[FAIL] Calibration output is missing or empty.")
with path.open("r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream)
if not isinstance(data, dict) or not isinstance(data.get("kinematics"), dict):
    raise SystemExit("[FAIL] Calibration YAML has no kinematics mapping.")
checksum = data["kinematics"].get("hash")
if not isinstance(checksum, str) or not checksum.startswith("calib_"):
    raise SystemExit("[FAIL] Calibration YAML has no valid kinematics/hash.")
print(f"[OK] Valid calibration YAML ({path.stat().st_size} bytes, hash={checksum})")
PY

install -m 0644 "${temp_file}" /bags/ur_calibration.yaml
install -m 0644 "${temp_file}" /config/ur_calibration.yaml
printf '[OK] Saved /bags/ur_calibration.yaml and /config/ur_calibration.yaml\n'
