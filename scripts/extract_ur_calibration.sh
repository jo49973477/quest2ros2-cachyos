#!/usr/bin/env bash
set -euo pipefail

readonly project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_dir}"

robot_ip="${1:-${UR_IP:-}}"
if [[ ! "${robot_ip}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
  printf 'Usage: %s ROBOT_IP\n' "$0" >&2
  exit 2
fi

if ! grep -Eq '^ALLOW_REAL_ROBOT=NO$' .env 2>/dev/null; then
  printf '[STOP] ALLOW_REAL_ROBOT=NO is required in .env.\n' >&2
  exit 2
fi
if ! grep -Eq '^ENABLE_SERVO_OUTPUT=NO$' .env 2>/dev/null; then
  printf '[STOP] ENABLE_SERVO_OUTPUT=NO is required in .env.\n' >&2
  exit 2
fi

running="$(docker compose ps --services --status running)"
if grep -Eq '^(ur_real|ur_mock|ur_moveit|ur_target|ur_bridge)$' <<<"${running}"; then
  printf '[STOP] A robot or teleoperation service is running:\n%s\n' "${running}" >&2
  exit 2
fi

docker compose config --quiet
docker compose build q2r_endpoint

UR_IP="${robot_ip}" docker compose --profile calibration run --rm --no-deps ur_calibration \
  2>&1 | tee bags/ur_calibration.log

test -s bags/ur_calibration.yaml
test -s config/ur_calibration.yaml
grep -q '^kinematics:' config/ur_calibration.yaml
grep -q '^[[:space:]]*hash:[[:space:]]*calib_' config/ur_calibration.yaml
printf '[OK] Host calibration files validated.\n'
