# V4: MoveIt Servo integration scaffold (mock first)

**Not tested with a real UR10 or a Docker Engine in this workspace**. Host-side
unit, syntax and Compose configuration tests are provided below; Docker runtime
checks must still be run on the CachyOS host with Docker socket access.
All commands below run from the Compose folder after building the updated image.
Keep the same `.env` and bags; merge new settings manually.

## What is already available upstream?

ROS2 Humble's Universal Robots `ur_moveit_config` exposes `launch_servo:=true`.
That launch starts `move_group` plus a `servo_node` configured by
`ur_moveit_config/config/ur_servo.yaml` (Humble branch).
The default Servo input is `/servo_node/delta_twist_cmds` (`TwistStamped`),
and output is `/forward_position_controller/commands` (`Float64MultiArray`).
The default UR driver usually activates a *trajectory* controller, therefore
it must be switched to `forward_position_controller` for this setup.

Taokt's BaseArmController produces *target poses*, not joint commands, but also
assumes VR axes already match robot axes. `quest_target_node.py` replaces that
robot-specific path: button_lower anchors the controller to the current tool0
pose and computes an explicit VR-to-base_link rotation. `pose_to_twist_bridge.py`
compares the calibrated target and current tool pose.
It REQUIRES live UR TF (`base_link` -> `tool0`) and both controller triggers.

## Step-by-step ONLY with mock hardware

1. `docker compose build q2r_endpoint`
2. `docker compose up -d q2r_endpoint`
3. `docker compose --profile compat up -d q2r_alias` (only if Quest emits `/right_hand/pose`)
4. `docker compose --profile mock up -d ur_mock`
   The mock service mounts `config/ur10_mock_initial_positions.yaml` so the arm
   starts away from the UR default elbow/wrist singularity.
5. Verify `/joint_states` contains the configured non-singular pose, then run
   `docker compose --profile servo up -d ur_moveit`. Starting or recreating
   MoveIt only after the mock state is available prevents Servo from caching a
   zero/singular initial robot state.
6. `docker compose --profile teleop up -d ur_target ur_bridge`
7. `docker compose ps --all` — verify `ur_mock`, `ur_moveit`, `ur_target`, and
   `ur_bridge` are all running. A stopped `ur_bridge` means
   `/ur10_quest/twist_preview` cannot exist.
8. `docker compose logs --tail=80 ur_moveit ur_target ur_bridge` — verify no
   missing packages, model errors, or Python tracebacks.
9. `docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 control list_controllers`
10. `docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 service list | grep servo`
11. `docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic list -t`
12. If Quest publishes `/right_hand/*`, verify both the source and compatibility
    relay with `ros2 topic hz /right_hand/pose` and
    `ros2 topic hz /q2r_right_hand_pose`. The second requires `q2r_alias`.
13. Hold the controller aligned with the intended tool0 axes and press
    `button_lower` once to anchor/calibrate, then hold BOTH index/grip triggers,
    then `docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic echo /ur10_quest/twist_preview`.
    This should show bounded twist in correct direction — **no robot commands yet**.

The bridge publishes preview messages continuously, including zero twists while
the deadman is released. Therefore `topic echo` reporting that the preview topic
has no publisher means `ur_bridge` is not running or exited; inspect its Compose
status and logs before debugging Quest buttons.

## Enabling Servo (MOCK TEST ONLY)

First verify active controller with `ros2 control list_controllers`.
The Compose mock service starts `forward_position_controller` directly, so it
should already be `active`. If it is not active, inspect `ur_mock` logs instead
of starting Servo against an incomplete controller stack.

For a manually launched driver that instead has `joint_trajectory_controller`
active, switch it before starting Servo:

```
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh \
  ros2 control switch_controllers --deactivate joint_trajectory_controller \
  --activate forward_position_controller --strict
```

Then start Servo:

```
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh \
  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'
```

To make bridge issue actual Servo commands, set `ENABLE_SERVO_OUTPUT=YES` in `.env`
**ONLY AFTER** checking pose frames, axis signs, correct feedback, zero-on-loss,
slow speed caps, collision scene, E-stop and robot isolation. Recreate ONLY
`ur_bridge` to pick up the env change; do not enable this against a real robot.

## Verification

Recommended one-shot rebuild and guarded mock validation:

```
./scripts/rebuild_and_validate_mock.sh
```

This deliberately requires `ALLOW_REAL_ROBOT=NO` and
`ENABLE_SERVO_OUTPUT=NO`, refuses to run while `ur_real` is running, then tests
MoveIt Servo directly against fake hardware. Quest bridge output stays disabled.

Host-only deterministic tests (no ROS or robot motion):

```
python3 -m unittest discover -s tests -v
python3 scripts/ur10_jacobian.py 0 -1.5708 1.5708 -1.5708 -1.5708 0
docker compose --profile mock --profile servo --profile teleop config --quiet
```

Runtime configuration check after mock and MoveIt are running:

```
docker compose exec ur_mock /opt/quest-tools/entrypoint.sh \
  /opt/quest-tools/check_mock_servo.sh
```

Guarded mock motion test (requires `ENABLE_SERVO_OUTPUT=NO`, `ur_real` absent,
and no other publisher on `/servo_node/delta_twist_cmds`):

```
docker compose exec -e CONFIRM_MOCK_ONLY=YES ur_mock \
  /opt/quest-tools/entrypoint.sh /opt/quest-tools/mock_servo_integration_test.py
```

## Why this is NOT a production-ready UR10 Teleoperation system

- Anchor calibration removes the old VR-axes-equal-robot-axes assumption, but
  the controller must be deliberately aligned with the intended tool axes at
  button_lower and its direction must be checked in the actual workcell.
- Both-trigger software deadman is NOT a certified safety device.
- Bridge has velocity caps but no acceleration/jerk limit and no collision scene setup.
- UR's default MoveIt environment contains just the arm; workcell/table are absent.
- Quest and UR timeouts, Servo status, actual robot protective stops, and calibration
  need end-to-end tests on mock and real hardware before enabling outputs.
- `ur_moveit.launch.py` `launch_servo:=true` is not itself a command to start servo motion:
  use `/servo_node/start_servo` only after controller switching and checks.

Original docs:
- https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_moveit_config/doc/index.html
- https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html
