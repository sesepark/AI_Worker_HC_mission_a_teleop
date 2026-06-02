# mission/

System 팀 휴머노이드 챌린지 미션 시나리오 코드 (ament_python 패키지 `mission`).

## 구조

```
mission/
├── package.xml / setup.py / setup.cfg
├── resource/mission
└── mission/                  # python 모듈
    ├── mission_a.py          # Mission A 상태 기계 (rclpy Node)
    ├── task_list.py          # OCR parts → {class_name: 잔여수량} (ROS 무관)
    └── sim_driver.py         # --sim 모드 fake service/topic 주입기
```

## 빌드

```bash
# 통합 워크스페이스(~/robotis_ros2_ws)로 동기화 후 컨테이너에서
colcon build --packages-select mission
source install/setup.bash
```

## 실행

### sim 모드 — 풀스택 없이 FSM 전이 검증 (권장 첫 테스트)
fake 토픽으로 INIT→A1→A2→A3→VERIFY→DONE 루프를 자동 구동.
**로봇/perception 네트워크 오염을 막기 위해 격리 도메인에서 실행:**
```bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
ros2 run mission mission_a --ros-args -p sim_mode:=true
```
→ task_list(총 3개) 기준 3회 pick-place 루프 후 `[DONE] mission A 완료` 출력.

### 실 스택 연동 (perception/manipulation 라이브)
```bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 run mission mission_a
```
Service client: `/mission_a/task_list` (`mission_interfaces/srv/GetTaskList`)
구독: `/manipulator_state`, `/detections`,
`/perception/wrist/target_one_pose`, `/attached_object`
발행: `/active_mission`, `/attach_cmd`, `/detach_cmd`

> task list는 `/mission_a/task_list` 서비스 응답만 사용한다.

## task_list 단독 테스트
```bash
python3 -c "from mission.task_list import TaskList; \
print(TaskList().build_from_ocr_parts([{'name':'육각 너트','count':2}]))"
```

## 구현 현황 (P0 + service 기반 OCR 연동)

- [x] `package.xml`+`setup.py` 추가 → `ros2 run mission mission_a`
- [x] `task_list.py` — OCR 한국어 부품명↔class_name 매핑 + 잔여 수량 관리 (단위 테스트 통과)
- [x] state별 timeout (`STATE_TIMEOUT`) + `_elapsed()`/`_timed_out()`
- [x] `--sim` 모드 + 전체 루프 검증 (DONE 도달 확인)
- [x] `/perception/wrist/target_one_pose` 구독 (구 `/target_pose` 폐기)
- [x] **`/mission_a/task_list` service client** — monitor_ocr OCR 결과를 단일 경로로 수신

## TODO (Phase 2~3 — MISSION_A_SCENARIO_PLAN.md "초안 작성 계획")

- [ ] A3_PICK: `bin_pick` Action client (Calib 전 `/attach_cmd` 우회)
- [ ] A3_PLACE: `tray_place` Action client + `/tray_region`
- [ ] `/manipulator_state` GRASPING→ATTACHED / RELEASING→IDLE 모니터
- [ ] VERIFY: `/tray_region` 재스캔 실측 (현재는 성공 가정)
- [ ] CM 토픽명(`/active_mission`,`/manipulator_state`) 전 팀 합의
