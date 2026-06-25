> ⚠️ **아카이브됨 (2026-06-25 재구조화)** — 이 파일은 [../REAL_ROBOT_TEST_PROCEDURE.md](../REAL_ROBOT_TEST_PROCEDURE.md)와 **바이트 동일**한 중복본입니다. 정본은 그쪽을 유지. 참고용으로 보존.

# 실물 로봇 테스트 프로시져 (Mission A)

## 사전 확인

- [ ] 로봇 컴퓨터와 내 노트북 같은 네트워크 연결 확인
- [ ] systems 팀에 크로스 머신 ROS2 통신 설정 확인 (`ROS_DOMAIN_ID=30` 맞는지)
- [ ] 테이블 위치 확인 — 로봇 몸체 기준 250mm 진입 (테이블 앞 엣지 x=0.050)
- [ ] yellow_box 위치 확인 — center_x=0.320, center_y=-0.295
- [ ] GraspAssessment bypass (`success = True`) 복원 여부 결정

---

## 로봇 컴퓨터 세팅

```bash
# Terminal 1 — 로봇 bringup
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py

# Terminal 2 — TRAC-IK 설치 (매번 필요)
docker exec -it ai_worker bash
apt update && apt install -y ros-jazzy-trac-ik-kinematics-plugin ros-jazzy-trac-ik-lib

# Terminal 3 — MoveIt (bringup 뜨고 나서)
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ffw_moveit_config moveit.launch.py
```

---

## 내 노트북 세팅

```bash
docker exec -it humanoid_challenge bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
```

통신 확인 (joint_states 수신되는지):
```bash
timeout 3 ros2 topic echo /joint_states \
  --qos-reliability reliable --qos-durability transient_local --once
```

데이터 안 오면 → `ROS2_COMMUNICATION_ISSUES.md` 시나리오 B 참고

---

## 테스트 순서

### Step 1 — 그리퍼 확인 (가장 먼저)

```bash
ros2 run manipulation test_gripper
```

- `o` → 열리는지 확인
- `c` → 닫히는지 확인
- 동작 안 하면 그리퍼 토픽 문제 → 진행 불가

---

### Step 2 — Zone A collision object 확인

```bash
ros2 run manipulation test_zone_a
```

RViz에서 테이블/박스 위치가 실제와 맞는지 눈으로 확인.
안 맞으면 `config/zone_a.yaml` 수치 조정.

---

### Step 3 — Capture pose 이동

```bash
ros2 run manipulation test_move_to_capture_pose
```

> ⚠️ 현재 joint 값은 시뮬 기준 임시값
> 실물에서 wrist cam이 yellow_box를 내려다보는지 확인
> 아니면 텔레옵으로 자세 잡고 joint 값 추출해서 `CAPTURE_JOINTS` 교체

joint 값 추출:
```bash
# 텔레옵으로 자세 잡은 후
ros2 topic echo /joint_states --once
# arm_r_joint1~7 값 기록 → test_move_to_capture_pose.py CAPTURE_JOINTS 교체
```

---

### Step 4 — Pick 테스트

```bash
ros2 run manipulation test_pick
```

테스트 좌표 (test_pick.py GRASP_X/Y 수정):

| 순서 | GRASP_X | GRASP_Y | 비고 |
|---|---|---|---|
| 1 | 0.320 | -0.250 | 중앙 (기본값) |
| 2 | 0.250 | -0.250 | 앞쪽 |
| 3 | 0.400 | -0.250 | 깊숙이 |
| 4 | 0.320 | -0.350 | 옆쪽 |

성공 기준: pick SUCCEEDED + carry 상승(z=1.020) SUCCEEDED

---

### Step 5 — Place 테스트

```bash
ros2 run manipulation test_place
```

pick 완료 후 팔이 carry 위치(z=1.020)에서 멈춘 상태에서 실행.
성공 기준: place SUCCEEDED (hover 하강 → 그리퍼 열림 → 상승)

---

## 주의사항

- capture pose 이동 시 사람이 옆에서 지켜볼 것 (joint 값 임시)
- pick 첫 시도는 반드시 천천히 (velocity=0.1 유지)
- 팔이 예상 밖 방향으로 움직이면 즉시 E-stop
- GraspAssessment가 bypass 상태(`success = True`)면 실제로 잡았는지 눈으로 확인 필요
