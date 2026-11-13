# CachyOS + Docker + Quest2ROS2 + UR10

**목표:** CachyOS를 재부팅하지 않고 Docker의 Ubuntu 22.04/ROS 2 Humble에서 Meta Quest 2/3 컨트롤러 데이터를 수신·기록하고, 이후 UR10/UR10e 공식 ROS 2 드라이버와 연결할 수 있는 기초 환경을 만든다.

**v2 호환성 수정 (2026-11-13):** 최초 버전은 Taokt가 사용하던 `guguroro/ros_tcp_communication` 서버를 사용했다. 최신 Quest2ROS 앱(ROS 2 모드)과의 직렬화/토픽 호환성이 문제가 될 수 있어, **공식 Quest2ROS ROS 2 설치 가이드에 명시된** `Quest2ROS/quest2ros` (`ros2`) + `MWelle77/ROS-TCP-Endpoint` (`main-ros2`) 조합으로 교체했다. Taokt/Quest2ROS2의 모니터 및 로봇 관련 코드는 유지한다. 빌드·실물 연결은 아직 확인하지 않았다.

**제공 범위:** Dockerfile, Compose, Quest TCP endpoint, Taokt/Quest2ROS2,
구형/신형 토픽 이름 호환용 선택적 relay, ROS bag2 기록, UR10 mock/실물
드라이버의 별도 프로필, 그리고 **Mock Hardware 검증용 캘리브레이션된
MoveIt Servo 어댑터**. 이 어댑터는 실물 UR10 운용 승인을 받은 구현이 아니다.

> 🚧 저장소·문서의 정적 분석에 기반한 구성안이다. 작성 환경에서 Docker Engine과 외부 이미지 네트워크에 접근할 수 없어 전체 `docker compose build` 및 실제 Quest/UR10 연동은 실행 검증하지 못했다.

## 0. 전체 연결

```
Meta Quest 3 (Quest2ROS app)
       │ Wi-Fi TCP :10000
       ▼
CachyOS PC [LAN_IP]
       └─ Docker / host network
            ├─ ROS–TCP Endpoint + Taokt Quest2ROS2
            ├─ /right_hand/pose, /right_hand/inputs, ...
            ├─ [q2r_alias] → /q2r_right_hand_pose, ...
            ├─ rosbag2 (/bags → host ./bags)
            ├─ [ur_target] VR anchor/calibration → base_link 목표 Pose
            ├─ [ur_bridge] bounded 목표 오차 → MoveIt Servo Twist
            └─ [선택] Universal Robots 공식 ROS 2 driver
                      │ Ethernet (UR_IP)
                      ▼
                    UR10 / UR10e
```

**Quest 앱의 IP** = CachyOS PC가 Quest에서 접근 가능한 **LAN IP**, **UR_IP** = 실제 로봇 제어기 IP. 이 둘은 대부분 다르다. Docker 컨테이너 IP는 Quest에 넣지 않는다.

## 1. CachyOS 준비

```bash
sudo pacman -Syu --needed docker docker-compose git
sudo systemctl enable --now docker
docker version
docker compose version
```

Docker 데몬에 접근할 수 없으면 `sudo docker compose ...`를 사용하거나 본인 계정을 `docker` 그룹에 등록 후 **다시 로그인**한다. Docker 그룹은 루트 수준 권한이므로 신뢰할 수 있는 사용자에게만 부여한다.

프로젝트 폴더에서:

```bash
cp .env.example .env
# UR_TYPE=ur10 또는 ur10e; ROS_DOMAIN_ID=30 설정 가능
# 실제 로봇 프로필을 켜지 않을 때 UR_IP는 비워두어도 됨

docker compose config --quiet
docker compose build
docker compose up -d q2r_endpoint q2r_alias
docker compose logs -f q2r_endpoint
```

- 빌드에서는 `Taokt/Quest2ROS2` + 공식 `Quest2ROS/quest2ros` (`ros2`) 메시지 패키지 + `MWelle77/ROS-TCP-Endpoint` (`main-ros2`)를 Git에서 직접 가져온다.
- 원본의 `q2r2_bringup/package.xml`에 잘못 적힌 `quest2ros2_msg` 의존성을 실제 패키지명인 `quest2ros`로 수정한다.
- 공식 `Quest2ROS/quest2ros`의 `ros2` 브랜치를 메시지 패키지로 사용하고 Taokt의 중복 메시지 디렉터리는 `COLCON_IGNORE`로 무시한다.
- `--cmake-args -DBUILD_TESTING=OFF`는 메시지 패키지의 추가 lint/test 빌드 의존성을 피한다.
- `ros-humble-ur`도 이미지에 설치되어 있지만 **실제 UR10 제어 노드는 기본 실행하지 않는다.**

### Docker 네트워크

`compose.yaml`의 `network_mode: host`는 CachyOS **Linux 호스트와 네트워크를 공유**한다. `ports: - 10000:10000` 같은 매핑은 **작성하지 않는다**. 기본 ROS–TCP 수신 주소 `0.0.0.0`은 PC에서 들어오는 TCP 연결을 받아들이는 bind 주소다.

Wi-Fi를 쓰는 Quest와 유선 PC가 **서로 라우팅 가능한 로컬 네트워크**에 있어야 한다. 회사/학교 AP의 Client Isolation, VPN, 방화벽이 포트 10000/TCP를 막으면 연결되지 않는다.

## 2. Meta Quest와 연결 (실제 헤드셋)

1. 헤드셋에 [Quest2ROS 앱](https://mcwelle.com/q2r/) 설치. 컨트롤러를 인식시키고 Meta Quest 자체 OS에서 실행하면 된다. PC에 Meta Quest Link/Unity는 필요 없다.
2. CachyOS PC의 **Quest에서 접속 가능한 IP** 확인:

```bash
ip -br -4 addr
ip route
# 예: wlp...에 192.168.0.42 (예시일 뿐)
```

3. `docker compose up -d q2r_endpoint q2r_alias` 후 `docker compose logs -f q2r_endpoint`를 확인한다.
4. Quest 앱에서 **ROS 2** 모드를 선택하고 **PC IP(예: 192.168.0.42)**, **TCP Port 10000** 입력 → Connect 또는 Apply.
5. 컨테이너에서 수신 토픽 확인 (먼저 공식 `/right_hand/pose` 출력 → 그다음 relay된 `/q2r_right_hand_pose` 확인):

```bash
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic list
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic type /right_hand/pose
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic echo /right_hand/pose
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic hz /right_hand/pose
```

6. Taokt 수신 진단(종료는 Ctrl-C):

```bash
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 run q2r2_bringup CheckTCPconnection
```

**중요: 최신 Quest2ROS 앱과 Taokt 코드 사이 토픽 차이:** Taokt는 `/q2r_right_hand_pose` 등, [공식 Quest2ROS 최신 ROS 2 문서](https://mcwelle.com/q2r/)는 `/right_hand/pose` 등을 설명한다. 토픽 이름은 `q2r_alias` 서비스가 relay한다. 아래처럼 실행하고 `/q2r_right_hand_pose`도 확인할 수 있다.

```bash
docker compose --profile compat up -d q2r_alias
# /right_hand/pose 가 이미 발행되고 있을 때만 의미가 있음.
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic list
```

**v2에서는 이미 공식 ROS 2 조합을 사용한다.** 위 relay는 *토픽 이름만* 바꿔준다. 토픽 타입과 ROS_DOMAIN_ID가 맞는지도 반드시 검사하자. 두 TCP Endpoint를 같은 포트 10000으로 동시에 띄우면 안 된다.

## 3. 헤드셋 없이 ROS 토픽 테스트

```bash
# 터미널 A: 가상의 오른손 컨트롤러 Pose/버튼/Twist 30 Hz 발행
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 run q2r2_bringup SimulationInput

# 터미널 B: Docker 안에서 구독
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic echo /q2r_right_hand_pose
```

테스트 중에는 실제 컨트롤러와 시뮬레이터가 같은 토픽에 동시에 쓰지 않도록 주의한다. `SimulationInput`이 돌아간다는 것은 ROS 2 코드 빌드의 일부 검증이지 Quest 무선 연결 성공을 의미하지 않는다.

## 4. ROS bag2로 데이터 수집

```bash
# Quest를 연결한 상태에서 별도 터미널 실행
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh bash -lc '
ros2 bag record -o "/bags/quest_$(date +%Y%m%d_%H%M%S)" \
  /q2r_left_hand_pose /q2r_left_hand_inputs /q2r_left_hand_twist \
  /q2r_right_hand_pose /q2r_right_hand_inputs /q2r_right_hand_twist
'
```

Ctrl-C로 종료하면 **CachyOS의 `./bags/` 폴더**에 저장된다. 관절 상태·카메라·촉각을 수집하려면 해당 토픽을 추가하고, 공통 시간 기준으로 동기화해야 한다. Bag는 곧바로 LeRobot 포맷이 되지는 않는다.

## 5. UR10 공식 드라이버: 먼저 Mock Hardware

```bash
# .env: UR_TYPE=ur10 또는 ur10e
docker compose --profile mock up -d ur_mock
docker compose logs -f ur_mock

# 출력 확인
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic list
docker compose exec q2r_endpoint /opt/quest-tools/entrypoint.sh ros2 topic echo /joint_states
```

ROS 2 Humble UR 드라이버의 `use_fake_hardware:=true`를 사용하므로 실제
UR10과 통신하지 않는다. 하지만 ROS 드라이버가 성공적으로 시작해도
**Quest 입력이 UR의 관절 명령으로 자동 변환되는 건 아니다.**

## 6. 실제 UR10 연결 (명시적 승인 후)

### 공장 캘리브레이션 추출(움직임 없음)

ROS 2 Humble의 `ur_calibration` 2.15.0에는 Primary Interface pipeline을
초기화하지 않고 실행하는 버그가 있다. 이 이미지는 Universal Robots의
수정 커밋 `b8e342428839ca8bcc4a49caf25633a712f9cd2f`에서
`ur_calibration` 패키지만 빌드해 `/opt/ur_calibration_ws` overlay로 올린다.
APT의 UR 드라이버·MoveIt 패키지는 그대로 유지한다. Docker entrypoint는
이 overlay를 마지막에 source하므로 이미지 재생성 뒤에도 수정본이 우선한다.

캘리브레이션 서비스는 Primary Interface `30001/TCP`에서 공장 데이터를
읽을 뿐 `ur_control`, `ros2_control`, MoveIt Servo 또는 External Control을
시작하지 않는다. 실행 전 `.env`의 다음 값은 반드시 그대로 유지한다.

```ini
ALLOW_REAL_ROBOT=NO
ENABLE_SERVO_OUTPUT=NO
```

CachyOS와 Fish shell에서도 아래 스크립트 하나만 실행하면 된다. 스크립트는
Compose 검사 → 이미지 빌드 → 수정 overlay/포트 확인 → 45초 timeout을 건
추출 → YAML/hash 검증 → 두 위치에 저장까지 수행한다.

```fish
./scripts/extract_ur_calibration.sh 192.168.2.3
```

성공 결과:

- `bags/ur_calibration.yaml`: 원본 보관용
- `config/ur_calibration.yaml`: `ur_real`의 읽기 전용 입력
- `bags/ur_calibration.log`: 추출 및 진단 로그

세 파일 중 로봇별 YAML과 로그는 Git에서 제외된다. 빌드는 고정된 upstream
SHA와 소스의 `pipeline.init();`를 확인하며, 실행 시에도 다음을 다시 검사한다.

```bash
docker compose run --rm --no-deps q2r_endpoint \
  bash -lc 'ros2 pkg prefix ur_calibration; cat /opt/ur_calibration_ws/UPSTREAM_COMMIT'
```

정상 prefix는 `/opt/ur_calibration_ws/install`이다. `Primary Interface ...
unavailable`이면 IP/30001을, `Wrong ur_calibration prefix`이면 entrypoint overlay를,
timeout이면 Primary Interface 데이터 수신 및 PolyScope/펌웨어 호환성을 먼저
확인한다. 실패 파일은 유효한 캘리브레이션으로 복사하지 않는다.

### 네트워크 준비

- UR 컨트롤러 PolyScope에서 UR10(CB) 또는 UR10e 모델과 **로봇 IP** 확인.
- PC ↔ UR 컨트롤러는 가능하면 유선 Ethernet 전용 인터페이스 사용. Quest Wi-Fi와 별도 네트워크여도 되지만 **CachyOS PC가 두 네트워크 모두에 접근할 수 있어야 함**.
- PC에서 로봇 IP까지 라우팅 확인. 예를 들어 `UR_IP=192.168.50.10`이라면:

```bash
ip route get 192.168.50.10
ping -c 2 192.168.50.10
nc -vz -w 3 192.168.50.10 30004
```

`30004`는 RTDE 연결 확인용이다. 로봇이 ICMP를 무시하면 ping이 실패해도 TCP 연결이 될 수 있다.

### URCap 및 안전

- [UR 공식 ROS 2 드라이버 설치·로봇 설정 문서](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)대로 **External Control URCap**을 설치하고 프로그램을 준비한다.
- URCap에서 설정하는 **Host IP는 UR 로봇이 접속할 수 있는 CachyOS PC의 IP(특히 로봇 쪽 NIC 주소)**. Quest 앱에 넣는 PC IP와 다른 NIC일 수 있다.
- 컨트롤러의 모델, 작업 공간, 속도/가속도, TCP/툴 오프셋, E-stop, 충돌 및 관절 한계, 데드맨, 통신 끊김 정지 로직 검증.
- Docker는 RT kernel / 안전 인증을 제공하지 않는다. 고속·실시간 제어 용도로 그대로 신뢰하면 안 된다.
- Quest 입력과 UR10 드라이버를 **각각 먼저 검증**한다. 기존 RTDE 직접제어와 ROS 드라이버로 동시에 로봇에 명령하지 않는다.

설정값이 확실한 뒤 `.env` 예시(주소는 실제 값으로 교체):

```ini
UR_TYPE=ur10
UR_IP=192.168.50.10
ALLOW_REAL_ROBOT=I_UNDERSTAND
```

**운영자 승인 및 안전 검증 후에만**:

```bash
docker compose --profile robot up -d ur_real
docker compose logs -f ur_real
```

이 프로필은 `ALLOW_REAL_ROBOT=I_UNDERSTAND`가 아니면 중단된다. **`headless_mode:=false`**로 켜며, 실제 동작을 위해서는 UR teach pendant의 준비된 External Control 프로그램을 실행해야 한다. 로봇/드라이버 상태가 정상이고 작업공간이 안전한지 확인하기 전에는 프로그램을 실행하지 말 것.

### Mock 전용 Quest → UR10 제어 어댑터

Taokt의 일반 `BaseArmController`가 VR 축과 로봇 축이 같다고 가정하는
부분은 사용하지 않는다. `quest_target_node.py`가 `button_lower` 활성화 시
Quest controller와 현재 `tool0`를 anchor로 삼아 명시적인 VR→`base_link`
회전을 계산하고, `pose_to_twist_bridge.py`가 제한된 Twist를 생성한다.

```
/q2r_right_hand_pose + /q2r_right_hand_inputs
   → [상대 Pose 기준점 확보 + 축 변환]
   → [버튼 활성화/데드맨, tracking watchdog]
   → [속도·가속도·작업공간 제한]
   → [현재 TCP Pose와 비교, 오차→TwistStamped]
   → [UR10에 맞게 설정한 MoveIt Servo]
   → [UR ROS 2 Driver 관절 컨트롤러]
   → UR10
```

권장: 상대 이동(컨트롤러 최초 위치와 현재 위치 차이)부터 구현 → 목표 pose와 실제 pose 기록 → Mock Hardware로 단방향·과속·끊김 시험 → 실물은 저속/소범위에서 테스트. 회전은 frame/quaternion 매핑을 따로 검증해야 한다. 컨트롤러 버튼과 `button_lower` 토글이 **연속 누름 데드맨과 같은 안전 보장인지** 별도 검증해야 한다.

실물 UR10의 관절각(`/joint_states`)과 캘리브레이션 기준 TCP 위치를 기록하는 것은 드라이버만 구동해도 가능하다. Quest와 로봇 데이터를 동일 ROS_DOMAIN_ID의 ROS 2 그래프에서 통합한다.

## 7. 문제 해결

| 증상 | 확인 |
| --- | --- |
| Docker `permission denied` | docker 그룹 재로그인/`sudo docker compose` |
| `ros2 pkg prefix quest2ros` 실패 | Docker 빌드 로그의 메시지 패키지 및 colcon 상태 확인 |
| Quest 접속 실패 | Quest에 PC LAN IP, TCP 10000, 방화벽, AP client isolation, ROS 2 모드 여부 확인 |
| 연결됐지만 `/q2r_*` 없음 | `/right_hand/pose` 토픽 여부, Taokt ↔ 공식 앱 버전/포크 호환성 |
| `Cannot bind 10000` | 다른 TCP Endpoint / 기존 프로세스가 이미 10000 점유 (`ss -ltnp | grep 10000`) |
| `ros2 topic list`에 타 컨테이너 토픽 없음 | 같은 ROS_DOMAIN_ID/RMW, `network_mode: host`, multicast 방화벽 및 DDS 확인 |
| Servo status `2` | `/joint_states` 초기 자세, mock 초기 YAML mount, Jacobian 조건수, Quest anchor 재설정 확인 |
| UR 연결 안 됨 | UR_IP/PC 로봇 쪽 NIC, External Control, teach pendant, 로봇/PC 방화벽 및 reverse TCP 확인 |

## 원본

- [Taokt/Quest2ROS2](https://github.com/Taokt/Quest2ROS2)
- [guguroro/ros_tcp_communication](https://github.com/guguroro/ros_tcp_communication)
- [최신 Quest2ROS ROS 2 앱·설정](https://mcwelle.com/q2r/)
- [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)
- [MoveIt Servo (ROS 2 Humble)](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
