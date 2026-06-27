# VR 텔레오퍼레이션 (robotis_vuer)

Meta Quest 3 헤드셋으로 AI Worker 로봇을 원격 조작(teleoperation)하기 위한 가이드입니다.
Quest의 WebXR 화면에서 컨트롤러/핸드 트래킹 입력을 받아 ROS 2 토픽으로 변환·발행하는
`robotis_vuer` 패키지를 다룹니다.

> 이 문서는 현재 브랜치의 실제 코드
> ([robotis_vuer/launch/vr.launch.py](../../robotis_applications/robotis_vuer/launch/vr.launch.py),
> [robotis_vuer/robotis_vuer/vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py))
> 를 기준으로 작성되었습니다. 다른 브랜치에서는
> launch 인자나 토픽이 다를 수 있으니 해당 브랜치의 코드를 직접 확인하세요.

---

## 1. 전체 구조

```
Meta Quest 3  ──(HTTPS/WSS, 8012)──►  Vuer 서버 (vr_publisher_sg2 노드 내장)
   WebXR                                      │
   컨트롤러/바디 트래킹 입력                    ▼
                                       ROS 2 토픽 발행 (head/lift/gripper/cmd_vel/pose …)
                                              │
                                              ▼
                                       Leader/Follower 제어 스택
```

| 구성 요소 | 설명 |
|-----------|------|
| **헤드셋** | Meta Quest 3 |
| **VR 클라이언트** | [Vuer](https://github.com/vuer-ai/vuer) 기반 WebXR 웹앱. Quest 브라우저에서 페이지를 열어 세션 시작 |
| **Vuer 버전** | README 기준 검증 버전은 **v0.1.5**, 현재 설치 환경은 **v0.1.6**. [공식 문서](https://docs.vuer.ai) |
| **서버 노드** | `vr_publisher_sg2` (SG2 모델). Vuer 서버를 별도 스레드로 띄우고, WebXR 이벤트를 ROS 2 토픽으로 변환 |
| **연결 방식** | WebXR은 secure context를 요구 → **HTTPS + WSS** 필수. 자기서명 인증서 사용 |

---

## 2. 실행

### 2.1 빌드

```bash
cd ~/AI_Worker_HC
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select robotis_vuer
source install/setup.bash
```

> **중요**: 컨테이너/도커 안에서 돌릴 때도 코드 수정 후에는 반드시 다시 `colcon build` 하세요.
> 노드는 `build/`(또는 `install/`)에 복사된 파일을 실행하므로, 소스만 고치고 재빌드하지 않으면
> 반영되지 않습니다.

### 2.2 ROS 환경 (모든 PC 공통)

```bash
ros2 daemon stop
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset CYCLONEDDS_URI
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_STRATEGY
unset ROS_SECURITY_KEYSTORE
ros2 daemon start
```

로봇 PC · Main PC · Quest가 같은 공유기/LAN에 있어야 합니다. Main PC IP 확인:

```bash
hostname -I
```

### 2.3 런처 실행

현재 `integration/mission`의 `vr.launch.py`는 SG2 노드에 head-tracking·VR 영상·제어 모드
파라미터를 전달하는 인자들을 노출합니다. launch 기본값은 노드 `declare_parameter` 기본값과
일치합니다.

아래 인자들은 모두 **`model:=sg2`일 때만** SG2 노드로 전달됩니다 (`sh5`/`hx5` 노드는 무시).

```bash
ros2 launch robotis_vuer vr.launch.py model:=sg2 \
  view_only_mode:=true \
  enable_vr_image:=true \
  enable_vr_head_tracking:=false \
  enable_leader_control:=false
```

**제어 모드 / 영상 플래그**

| launch 인자 | 기본값 | 설명 |
|-------------|--------|------|
| `model` | `sh5` | 실행할 모델 (`sg2`에서만 아래 인자들이 노드로 전달됨) |
| `view_only_mode` | `true` | 일반 VR 로봇 명령 발행 차단 (보기 전용) |
| `enable_vr_head_tracking` | `false` | VR 헤드셋 방향으로 로봇 머리(`head_joint1/2`) 추종 발행 |
| `enable_leader_control` | `false` | 실제 매니퓰레이션을 Leader가 담당함을 표시하는 마커 |
| `enable_vr_robot_control` | `false` | 원래 VR 컨트롤러 로봇 명령을 명시적으로 허용 |
| `enable_vr_image` | `false` | 압축 카메라 토픽을 Vuer VR 배경으로 스트리밍 |
| `vr_image_left_topic` | `/zed/zed_node/left/image_rect_color/compressed` | 왼쪽 VR 배경 영상 토픽 |
| `vr_image_right_topic` | `/zed/zed_node/right/image_rect_color/compressed` | 오른쪽 VR 배경 영상 토픽 |
| `vr_image_fps` | `15.0` | 카메라 배경 최대 갱신 주기(Hz) |

**머리 추종(head tracking) 튜닝**

| launch 인자 | 기본값 | 설명 |
|-------------|--------|------|
| `vr_head_tracking_hz` | `10.0` | 머리 추종 명령 최대 발행 주기(Hz). 첫 테스트는 낮게(예: `5.0`) 권장 |
| `vr_head_tracking_deadband_rad` | `0.01` | 머리 관절 움직임 전 헤드셋 데드밴드(rad) |
| `vr_head_tracking_smoothing_alpha` | `0.25` | 머리 추종 저역통과(LPF) 평활 계수 |
| `vr_head_tracking_max_delta_per_update` | `0.03` | 1회 갱신당 머리 관절 최대 변화량(rad). 작을수록 천천히/안전 |
| `vr_head_tracking_pitch_scale` | `1.0` | 헤드셋 pitch → `head_joint1` 스케일 |
| `vr_head_tracking_yaw_scale` | `-1.0` | 헤드셋 우측 yaw → `head_joint2` 스케일 |
| `vr_head_joint1_min` / `vr_head_joint1_max` | `-0.20` / `0.50` | `head_joint1` 명령 하한/상한(보수적 안전 한계) |
| `vr_head_joint2_min` / `vr_head_joint2_max` | `-0.28` / `0.28` | `head_joint2` 명령 하한/상한 |

> `host` 인자는 launch에 없습니다. Vuer 서버는 노드 내부에서 `host='0.0.0.0'`로 고정합니다.

#### 단계별 실행

- **Step 0 — VR 영상만 보기**: follower 스택을 먼저 띄운 뒤
  `view_only_mode:=true enable_vr_image:=true enable_vr_head_tracking:=false enable_leader_control:=false`.
- **Step 1 — 영상 + 머리 추종**: 위에서 `enable_vr_head_tracking:=true`로 켜고,
  처음엔 `vr_head_tracking_hz:=5.0 vr_head_tracking_max_delta_per_update:=0.015`로 느리게 시작.
  검증: `ros2 topic echo /leader/joystick_controller_left/joint_trajectory`에
  `joint_names: [head_joint1, head_joint2]`가 보이면 머리 명령이 나가는 상태.
- **Step 2 — 영상/머리 + Leader 조작**: 로봇 PC에서 follower + Leader(no-head config)를 띄우고,
  Main PC는 `enable_leader_control:=true enable_vr_robot_control:=false`로 실행.

> **로봇 PC(ai_worker_1.3.0, `feature/vr-leader-no-head-sg2` 브랜치) Leader no-head 실행 예**:
> ```bash
> ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py \
>   leader_controller_config:=ffw_lg2_leader_ai_hardware_controller_no_head.yaml
> ```
> Leader head 차단 확인: `timeout 5 ros2 topic echo /leader/joystick_controller_left/joint_trajectory`
> 에서 Leader 왼쪽 스틱을 움직여도 메시지가 안 나오면 정상.
> (단, VR head tracking을 켜면 이 토픽에 VR 명령이 나오는 것이 정상입니다.)

### 2.4 Quest 접속

런처 실행 시 노드가 다음 로그를 출력합니다:

```
VR Trajectory server available at: https://<IP>:8012
```

Quest 브라우저에서 접속:

```
https://<Main_PC_LAN_IP>:8012?ws=wss://<Main_PC_LAN_IP>:8012
```

자기서명 인증서이므로 브라우저 보안 경고가 뜹니다 → "고급 / 계속 진행"으로 통과한 뒤
**Enter VR** 버튼으로 WebXR 세션을 시작합니다.

---

## 3. HTTPS 인증서 (cert.pem / key.pem)

`vr_publisher_sg2` 노드는 Vuer를 HTTPS로 띄우기 위해 패키지 디렉터리의
`cert.pem` / `key.pem`을 사용합니다
([vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py)).

- **컨테이너 자동 생성**: `robotis_applications/docker/container.sh start`는 인증서 파일이 없으면
  호스트 IP를 감지해 컨테이너 안에서 `gen_cert.sh`를 실행합니다.
- **직접 실행**: `ros2 launch robotis_vuer vr.launch.py ...`로 바로 실행하는 경우에는
  `cert.pem`/`key.pem`이 패키지 디렉터리에 이미 있어야 합니다. 없으면 Docker 스크립트나
  `robotis_applications/docker/gen_cert.sh`로 먼저 생성하세요.
- 자기서명 인증서이므로 Quest 브라우저에서 보안 경고를 한 번 수동으로 통과해야 합니다.
- 신뢰된 인증서를 쓰고 싶으면 직접 만든 `cert.pem`/`key.pem`을 패키지 디렉터리
  (`robotis_applications/robotis_vuer/robotis_vuer/`)에 넣으면 자동 생성을 건너뜁니다.

---

## 4. 발행/구독 토픽

`vr_publisher_sg2` 노드 기준
([vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py)).

### 4.1 발행 (Publishers)

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/leader/joystick_controller_left/joint_trajectory` | `JointTrajectory` | 머리 제어 (`head_joint1`, `head_joint2`) |
| `/leader/joystick_controller_right/joint_trajectory` | `JointTrajectory` | 리프트 제어 (`lift_joint`) |
| `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory` | `JointTrajectory` | 왼손 그리퍼 (`gripper_l_joint1`) |
| `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory` | `JointTrajectory` | 오른손 그리퍼 (`gripper_r_joint1`) |
| `/cmd_vel` | `Twist` | 베이스 주행 (cmd_vel 모드일 때) |
| `/vr_controller/left_squeeze` | `Float32` | 왼쪽 그립(squeeze) 값 |
| `/vr_controller/right_squeeze` | `Float32` | 오른쪽 그립(squeeze) 값 |
| `/l_wrist_pose`, `/r_wrist_pose` | `PoseStamped` | 손목 포즈 (RViz 시각화용) |
| `/l_elbow_pose`, `/r_elbow_pose` | `PoseStamped` | 팔꿈치 포즈 |
| `/l_shoulder_pose`, `/r_shoulder_pose` | `PoseStamped` | 어깨 포즈 |
| `/reactivate` | `Bool` | 재활성화 신호 (파라미터 `reactivate_topic`로 변경 가능) |

### 4.2 구독 (Subscribers)

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/joint_states` | `JointState` | 현재 `lift_joint`, `head_joint1`, `head_joint2` 위치 읽기 (상대 제어 기준점) |

> `/joint_states`가 안 들어오면 머리/리프트 명령이 기준점을 못 잡습니다.
> → follower 쪽 스택이 먼저 떠 있어야 합니다.

---

## 5. 조작 매핑

[vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py) 기준.

| 입력 | 동작 |
|------|------|
| **오른쪽 스틱 (X축)** | 리프트 상하 조그. 스틱을 **끝까지**(\|x\|>0.95) 밀 때만 동작하며, `목표 += x * right_jog_scale`로 누적·클램프 |
| **왼쪽 스틱** | `LIFT+HEAD` 모드: 머리 제어 / `LIFT+CMD_VEL` 모드: 베이스 주행(`cmd_vel`) |
| **스틱 클릭(모드 토글)** | `joystick_mode` 전환 (`LIFT+HEAD` ↔ `LIFT+CMD_VEL`). cmd_vel 모드를 나갈 때 베이스 정지 명령 발행 |
| **트리거 (좌/우)** | 그리퍼 개폐. 캘리브레이션(offset/scale) 적용 후 `gripper_*_joint1` 위치로 변환 (최대 `1.3`) |
| **그립(squeeze, 좌/우 동시)** | goal pose 발행 게이팅. `goal_pose_squeeze_threshold`(기본 `0.8`) 이상으로 **양손 동시**일 때만 포즈 추종 활성 |
| **A 버튼 (양손 동시)** | `/reactivate` = `True` 발행 (rising edge) |
| **B 버튼 (양손 동시)** | `/reactivate` = `False` 발행 (rising edge) |

- **모드 구분**: `joystick_mode = True` → `LIFT+HEAD`, `False` → `LIFT+CMD_VEL`
  ([vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py)).
- 컨트롤러 입력 로그는 `Controller data received | ... mode=LIFT+HEAD/LIFT+CMD_VEL` 형태로 주기 출력됩니다.

---

## 6. 주요 파라미터

[vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py) 등에서 선언.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `stream_fps` | `30` | 영상 스트림 FPS |
| `pose_publish_hz` | `30.0` | 포즈 발행 주기(Hz) |
| `goal_pose_position_scale` | `1.1` | goal pose 위치 스케일 |
| `goal_pose_squeeze_threshold` | `0.8` | goal pose 활성화 그립 임계값 |
| `left/right_gripper_max_position` | `1.3` | 그리퍼 최대 위치 |
| `left/right_trigger_offset` | `0.0` | 트리거 캘리브레이션 오프셋 |
| `left/right_trigger_scale` | `1.0` | 트리거 캘리브레이션 스케일 |
| `apply_lift_to_arm_z` | `True` | 리프트→팔 Z 좌표 커플링 활성화 |
| `lift_to_arm_z_scale` | `1.0` | 리프트→팔 Z 커플링 스케일 |
| `left/right_wrist_offset_{x,y,z}` | `0.0 / 0.0 / EYE_NECK_OFFSET_Z-0.1` | 손목 위치 오프셋(m) |
| `left/right_wrist_{roll,pitch,yaw}_offset_deg` | `90.0 / 0.0 / 0.0` | 손목 회전 오프셋(deg) |
| `left/right_elbow_offset_{x,y,z}` | `0.0 / 0.0 / EYE_NECK_OFFSET_Z` | 팔꿈치 오프셋(m) |
| `left/right_shoulder_offset_{x,y,z}` | `0.0 / 0.0 / EYE_NECK_OFFSET_Z` | 어깨 오프셋(m) |
| `reactivate_topic` | `/reactivate` | 재활성화 토픽명 |

> `EYE_NECK_OFFSET_Z = -0.25` (헤드셋을 목에 걸 때 발생하는 Z 오프셋 보정값).

파라미터 변경 예시:

```bash
ros2 run robotis_vuer vr_publisher_sg2 --ros-args \
  -p stream_fps:=20 -p goal_pose_position_scale:=1.0
```

---

## 7. 트러블슈팅

| 증상 / 로그 | 원인 | 조치 |
|-------------|------|------|
| `Error in VR server thread: [Errno 2] No such file or directory` | `cert.pem`/`key.pem` 누락으로 Vuer HTTPS 서버 기동 실패 → Quest 접속 불가 | `docker/container.sh start` 또는 `docker/gen_cert.sh`로 인증서를 만든 뒤 재빌드 |
| `'Vuer' object has no attribute 'stop'` (종료 시 cleanup 에러) | vuer 0.1.6에는 `stop()` 메서드가 없음 | `hasattr` 가드가 적용된 버전으로 갱신(종료만의 문제로, 기능엔 영향 없음) |
| Ctrl-C 시 `rcl_shutdown already called` | 종료 경로에서 `rclpy.shutdown()` 중복 호출 | 종료 시 발생하는 무해한 경고. 재실행에 영향 없음 |
| Quest에서 접속이 안 됨 / `no active Vuer session` | 8012 포트·방화벽 문제, 또는 Quest 브라우저 미접속 | 같은 LAN 확인, `https://<IP>:8012` 직접 접속, 보안 경고 통과 |
| `/joint_states not received` (머리/리프트 명령 안 나감) | follower 스택 미실행 또는 `ROS_DOMAIN_ID`/DDS 불일치 | follower 먼저 실행, §2.2 ROS 환경 재확인 |
| `command clamped` | 명령이 안전 범위를 초과 | 정상 보호 동작 |
| 컨트롤러 입력은 들어오는데 로봇이 안 움직임 | squeeze 미활성 / 모드 불일치 | 양손 그립으로 추종 활성, 스틱 클릭으로 모드 확인 |

> **WSL2 사용 시**: Quest가 WSL 내부 IP에 직접 접속하지 못하는 경우가 많습니다.
> Windows 관리자 PowerShell에서 포트포워딩/방화벽 설정 후, Quest 접속 주소는
> WSL IP가 아니라 **Windows LAN IP**를 사용하세요.
> ```powershell
> wsl hostname -I
> netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8012 connectaddress=<WSL_IP> connectport=8012
> New-NetFirewallRule -DisplayName "WSL2 Vuer 8012" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8012
> ```

---

## 8. 참고

- 패키지 개요: [robotis_vuer/README.md](../../robotis_applications/robotis_vuer/README.md)
- 노드 소스: [vr_publisher_sg2.py](../../robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py)
  (동일 패키지에 `vr_publisher_hx5.py`, `vr_publisher_sh5.py`도 있으며 같은 인증서 경로 로직을 사용)
- Vuer 공식: [docs.vuer.ai](https://docs.vuer.ai) · [github.com/vuer-ai/vuer](https://github.com/vuer-ai/vuer)
