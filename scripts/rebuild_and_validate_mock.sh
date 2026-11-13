#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -f .env ]] || fail ".env is missing"
grep -Eq '^ALLOW_REAL_ROBOT=NO$' .env || fail "ALLOW_REAL_ROBOT must be NO"
grep -Eq '^ENABLE_SERVO_OUTPUT=NO$' .env || \
  fail "ENABLE_SERVO_OUTPUT must be NO during automated mock validation"

real_id="$(docker compose --profile robot ps -q --status running ur_real)"
[[ -z "$real_id" ]] || fail "ur_real is running"

python3 -m unittest discover -s tests -v
docker compose --profile mock --profile servo --profile teleop config --quiet
docker compose build q2r_endpoint

docker compose --profile mock --profile servo --profile teleop stop \
  ur_bridge ur_target ur_moveit ur_mock
docker compose up -d --force-recreate q2r_endpoint
docker compose --profile mock up -d --force-recreate ur_mock

for _ in $(seq 1 30); do
  if docker compose exec -T ur_mock /opt/quest-tools/entrypoint.sh bash -lc \
      'ros2 service list | grep -qx /controller_manager/list_controllers'; then
    break
  fi
  sleep 1
done
docker compose exec -T ur_mock /opt/quest-tools/entrypoint.sh bash -lc \
  'ros2 service list | grep -qx /controller_manager/list_controllers' || \
  fail "mock controller_manager did not become ready"

docker compose --profile servo up -d --force-recreate ur_moveit
for _ in $(seq 1 30); do
  if docker compose exec -T ur_mock /opt/quest-tools/entrypoint.sh bash -lc \
      'ros2 service list | grep -qx /servo_node/start_servo'; then
    break
  fi
  sleep 1
done
docker compose exec -T ur_mock /opt/quest-tools/entrypoint.sh bash -lc \
  'ros2 service list | grep -qx /servo_node/start_servo' || \
  fail "MoveIt Servo did not become ready"

docker compose --profile teleop up -d --force-recreate ur_target ur_bridge
docker compose exec -T ur_mock /opt/quest-tools/entrypoint.sh \
  /opt/quest-tools/check_mock_servo.sh
docker compose exec -T -e CONFIRM_MOCK_ONLY=YES ur_mock \
  /opt/quest-tools/entrypoint.sh /opt/quest-tools/mock_servo_integration_test.py

echo "PASS: mock stack and direct Servo smoke tests completed"
echo "Bridge remains in PREVIEW ONLY mode (ENABLE_SERVO_OUTPUT=NO)."
