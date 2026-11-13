# V3: Quest Connected/Connecting repeated reconnections

This release addresses a concrete upstream bug in `MWelle77/ROS-TCP-Endpoint` (`main-ros2`) that causes the following traceback upon a Quest TCP connection:

```
AttributeError: 'UnityTcpSender' object has no attribute 'start_sender'
```

The repo's `client.py` calls `start_sender`, but its `tcp_sender.py` incorrectly declares `start_`, creates a thread named `_thread` then uses `sender_thread`, and specifies a nonexistent callback `_loop` instead of `sender_loop`.

The Dockerfile now runs `scripts/fix_tcp_sender.py` **on the fork source before `colcon build`**. The script verifies the old/new tokens, runs a Python syntax parse, checks that `UnityTcpSender.start_sender` exists, and fails the build on an unexpected upstream change.

## Update your existing installation

Preserve your existing `.env` and `bags/` folders. Replace `Dockerfile` and copy `scripts/fix_tcp_sender.py` from this archive into the corresponding local project paths. From the project directory:

```bash
docker compose stop q2r_endpoint
docker compose build --no-cache q2r_endpoint
docker compose up -d --force-recreate q2r_endpoint
docker compose logs -f --tail=50 q2r_endpoint
```

With the Quest set to the CachyOS host LAN IP and TCP port `10000`:

```bash
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic list -t
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic echo /right_hand/pose
```

Taokt's `CheckTCPconnection` listens for `/q2r_right_hand_pose` not `/right_hand/pose`; start the optional `q2r_alias` compatibility relay if the official `right_hand` topics are present:

```bash
docker compose --profile compat up -d q2r_alias
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 run q2r2_bringup CheckTCPconnection
```

Stop any real UR10 teleoperation during this verification. TCP handshake success is not yet proof the robot is ready for motion commands. The image was not hardware-tested here.
