# v2: Connecting / Connected 반복 & ROS topic 미발행 수정안

- 최초 Dockerfile의 TCP Endpoint는 Taokt가 사용하던 `guguroro/ros_tcp_communication`이었다. 이 코드는 자체 `ros_msg_converter.py`로 수신 데이터를 해석하며, 최신 Quest2ROS ROS 2 앱이 기대하는 공식 구성과 다르다.
- 이 수정본은 공식 Quest2ROS ROS 2 문서에서 권장하는 `Quest2ROS/quest2ros` (ros2 브랜치) + `MWelle77/ROS-TCP-Endpoint` (main-ros2 브랜치)와 Taokt 제어 코드의 조합이다.
- ROS topic naming shim인 `q2r_alias`는 `/right_hand/pose`를 `/q2r_right_hand_pose`로 relay한다.
- Docker에서 `exec`는 ENTRYPOINT를 거치지 않는다. `docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 ...`를 사용한다.

## 적용 방법

원본 폴더의 `.env`와 `bags/`를 보존한 채 Dockerfile, compose.yaml, scripts 디렉터리, README를 수정본으로 교체한다.

```bash
docker compose down
docker compose build --no-cache
docker compose up -d q2r_endpoint q2r_alias
docker compose logs -f q2r_endpoint
```

Quest 앱에서 ROS 2, CachyOS 호스트 LAN IP, 10000/TCP 입력 후 연결.

```bash
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic list -t
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic echo /right_hand/pose
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 run q2r2_bringup CheckTCPconnection
```

Quest 연결이 여전히 실패하면 `docker compose logs --tail=100 q2r_endpoint`, Quest TCP 패킷, 컨테이너의 실제 토픽 목록을 확인한다. TCP LISTEN이 있다고 Quest 프로토콜이 호환되는 것은 아니다.

Note: Docker 이미지는 여기서 실제 build 테스트되지 않았고 Quest·UR10 하드웨어 테스트도 하지 않았다. 실물 UR10 제어 어댑터는 포함하지 않는다.
