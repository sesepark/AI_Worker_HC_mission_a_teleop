> ⚠️ **아카이브됨 (2026-06-25 재구조화)** — 시점 스냅샷(2026-05-31, 구 `feature/mission-a` 브랜치)입니다. §1 "문서 가이드"는 현행 [../README.md](../README.md) INDEX로 대체됨. 당시 진행 상황 기록으로 보존.

# 진행 상황 요약 (System 팀 — Perception 통합 / Mission A / VR)

> **최종 업데이트**: 2026-05-31 · **브랜치**: `feature/mission-a`
> **담당 4개 역할**: ① Perception 노드 6개 인터페이스 정리 ② 로컬 구동 검증 ③ Mission A 코드 작성 ④ VR 텔레오퍼레이션

---

## 0. 한눈에 보기 (먼저 읽어주세요)

이번 작업은 **Perception 팀이 만든 노드들을 System 팀 입장에서 정리·검증하고, 그 위에서 Mission A 자율 동작 코드를 작성**한 것입니다. Perception 코드 내부를 모르더라도, 어떤 노드가 무슨 토픽을 내보내고 그걸 미션 코드가 어떻게 쓰는지만 알면 됩니다.

**① Perception 노드 정리** — 현재 Mission A 핵심 흐름에 필요한 노드만 남겼습니다. 흐름은 `monitor_ocr_node → detector_node → tray_manage_node → wrist_task_grasp_planner_node`이고, 디버그용 `monitor_ocr_viewer`만 추가로 유지합니다. 미션 코드가 직접 쓰는 핵심 출력은 **부품 1개의 집을 위치 `/perception/wrist/target_one_pose`, 남은 작업 수량 `/perception/task_list`** 두 토픽입니다.

**② 로컬 구동 검증** — 실제 로봇(bringup)을 켠 상태에서 로컬 PC 도커로 Perception 노드들을 직접 띄워봤고, **모니터 OCR 노드 1개를 제외한 나머지는 정상 작동**을 확인했습니다. OCR 노드는 코드 자체는 멀쩡하나 파이썬 가상환경(ocr_venv) 의존성이 깨져 있어 재설치가 필요한 상태에서 멈춰 있습니다.

**③ Mission A 코드** — Perception 코드를 **분석해서 나온 실제 토픽·메시지 구조**를 입력으로 받는 **상태 기계(state machine)** 를 새로 작성했습니다(`mission` 패키지). 모니터에서 목표 수량을 읽고 → 부품을 하나씩 집어 → 트레이에 놓고 → 트레이 비전이 수량을 차감하는 흐름이 끝까지 돌아가는 것을, 하드웨어 없이 검증하는 `--sim` 모드로 **전체 루프 통과**를 확인했습니다. 다만 이는 **인터페이스 분석 + sim 검증**까지이고, **실제 perception 스택과 같이 띄운 라이브 연동은 아직 미검증**입니다(블로커: monitor_ocr 가상환경, 트레이 모델 미배치, Manipulation 팔 동작 미연동).

**④ VR 텔레오퍼레이션** — Meta Quest 3로 로봇을 원격 조작하는 `robotis_vuer` 사용법을 정리했고, **HTTPS 인증서 자동 생성**까지 적용해 Quest 접속 단계를 통과했습니다. 컨트롤러→로봇(머리/리프트/그리퍼/팔 포즈) 매핑은 동작하며, 머리 추종·Leader 제어는 별도 브랜치에 있습니다. 주로 막히는 지점은 **Quest↔PC 네트워크(HTTPS/WSS·방화벽·WSL2)와 Vuer 라이브러리 버전 차이**입니다.

> **요약 한 줄**: Perception은 OCR 노드 빼고 로컬 검증 완료, 그 토픽 위에 Mission A 상태기계를 만들어 sim 통과, VR은 접속·기본 조작까지 동작.

---

## 1. 문서 가이드 (어떤 파일에 무엇이 있나)

| 문서 | 담은 내용 | 언제 보면 되나 |
|------|-----------|----------------|
| **PROGRESS_SUMMARY.md** (본 문서) | 전체 진행상황 요약 + 문서 가이드 | **가장 먼저** |
| [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) | Perception **노드별 입출력 토픽·메시지·역할** (인터페이스 사전) | 어떤 토픽을 구독/발행할지 찾을 때 |
| [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md) | 로컬 도커 **셋업·노드 실행 런북·트러블슈팅** + 구동 검증 현황 | Perception 노드를 직접 띄울 때 |
| [MISSION_A_SCENARIO_PLAN.md](./MISSION_A_SCENARIO_PLAN.md) | Mission A **시나리오·상태기계 설계·구현 계획** | 미션 로직을 이해/확장할 때 |
| [../mission/README.md](../mission/README.md) | `mission` 패키지 **빌드·실행법**(sim/실연동) | mission_a 를 돌릴 때 |
| [VR_TELEOPERATION.md](./VR_TELEOPERATION.md) | VR **실행·토픽·조작 매핑·트러블슈팅** | Quest 텔레오퍼레이션 할 때 |

---

## 2. 역할 ① — Perception 핵심 노드 인터페이스 정리
📄 출처: [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) (상세), 실행법은 [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md)

Perception 팀 기능을 Mission A 실행에 필요한 5개 실행 노드로 정리. **굵은 토픽**이 미션 코드가 실제로 쓰는 것.

| # | 노드 | 핵심 역할 (한 줄) | 주요 출력 |
|---|------|------------------|-----------|
| ① | `monitor_ocr_node` | 지령 모니터를 읽어 **부품별 목표 수량** 파싱 | `/monitor_ocr/result` (JSON) |
| ② | `detector_node` | 카메라 영상에서 **5종 부품 검출·분류**(YOLO) | `/detections` |
| ③ | `tray_manage_node` | OCR 기반 task list + 파란 트레이 ROI 발행 | **`/perception/task_list`**, `/perception/tray_roi` |
| ④ | `wrist_task_grasp_planner_node` | task list와 detection을 이용해 **집을 부품 1개 선택** | **`/perception/wrist/target_one_pose`** |
| ⑤ | `monitor_ocr_viewer` | OCR 디버그 overlay viewer | OpenCV window |

- 부품 5종: `flange_nut / gear_ring / spacer_ring / hex_nut / dome_nut`.
- `tray_manage_node`는 typed `GetTaskList_Response` 계약으로 `/perception/task_list`를 발행한다.
- 실행 순서·명령은 LOCAL_SETUP "부록 A 런북" 참고.

## 3. 역할 ② — 로컬 PC 구동 검증
📄 출처: [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md) "실행 검증 현황"

실로봇 bringup(카메라/TF) + 로컬 도커로 Perception 노드 구동.

| 노드 | 결과 |
|------|------|
| detector / tray_manage / **wrist_task_grasp_planner** | ✅ 정상 작동 |
| `monitor_ocr_node` | ⚠️ **블로커** — 노드 로직은 정상, `ocr_venv` 의존성 누락(가상환경 재생성 필요) |

- 환경: 도커 이미지 `ros2_jazzy_robotis_perception` 직접 빌드 + venv + colcon. 실행 워크스페이스는 `~/robotis_ros2_ws`(= 컨테이너 `/ws`).
- 실행 중 정리한 두 함정: ① `ros2 run` 이 가상환경 파이썬을 안 씀(launch `prefix=` 또는 venv 파이썬 직접 실행으로 해결) ② ocr_venv 의존성 오염(깨끗이 재생성).

## 4. 역할 ③ — Mission A 코드 작성
📦 [humanoid_challenge/mission/](../mission/) · 📄 설계 [MISSION_A_SCENARIO_PLAN.md](./MISSION_A_SCENARIO_PLAN.md)

검증된 Perception 토픽을 입력으로 받는 상태 기계 패키지 `mission` 신규 작성.

- **흐름**: `INIT → A1_MONITOR(목표 수량 확정) → A2_SCAN(집을 부품 선택) → A3_PICK → A3_PLACE → VERIFY(트레이 차감 확인) → DONE`. 실패 시 RECOVERY/MANUAL_WAIT.
- **입력**: 남은 수량 `/perception/task_list`(우선), 집을 위치 `/perception/wrist/target_one_pose`. (OCR 직접 파싱은 폴백.)
- **구성**: `mission_a.py`(상태기계), `task_list.py`(부품명 매핑·잔여 관리, 단위 테스트 통과), `sim_driver.py`(`--sim` 가짜 토픽 주입).

**Perception 분석이 어떻게 반영됐나** — 토픽명·메시지 구조를 **추측이 아니라 Perception 실제 소스에서 추출**해 입력으로 삼음:
- planner 출력 `/perception/wrist/target_one_pose`(base_link PoseStamped) → A2_SCAN 이 구독 + `frame_id` 검증 (구 `/target_pose` 는 실제 없어서 제거).
- `tray_manage_node` 의 `/perception/task_list`(`GetTaskList_Response`, canonical 부품명 'dom nut' 등) → 구독 + `CANONICAL_TO_CLASS` 매핑 추가.
- `/monitor_ocr/result` JSON 구조 + "OCR 실패 10초 강제 OK" 정책 → 폴백 경로로 반영.
- (상세 매핑 표: [MISSION_A_SCENARIO_PLAN.md](./MISSION_A_SCENARIO_PLAN.md) "Perception 코드 분석 → mission_a 반영 매핑")

**연동 검증 상태 (정직한 현황)**
| 항목 | 상태 |
|------|------|
| 인터페이스 정합성(토픽/타입/JSON을 실제 소스에서 추출) | ✅ |
| `--sim` 전 루프 통과 (목표 3개 → 트레이 차감 3→2→1→0 → DONE) | ✅ |
| **라이브 end-to-end** (perception 풀스택 + mission_a 동시 구동) | ⬜ **미검증** |

> ⚠️ 즉, mission_a 는 **작동하는 perception 코드의 인터페이스 분석에 기반**해 만들어졌고 sim 으론 통과했으나,
> **두 스택을 같이 띄운 라이브 연동은 아직 안 했다.** 블로커: ① monitor_ocr ocr_venv(→task_list 라이브 미발행)
> ② 트레이 모델 미배치 ③ Manipulation Action(실제 팔 동작) 미연동. 라이브 검증은 monitor_ocr venv 해소 후 진행.
- **실행**: `export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 && ros2 run mission mission_a --ros-args -p sim_mode:=true`

## 5. 역할 ④ — VR 텔레오퍼레이션
📄 [VR_TELEOPERATION.md](./VR_TELEOPERATION.md) · 📦 [robotis_vuer/](../../robotis_applications/robotis_vuer/)

Meta Quest 3 → WebXR(Vuer) → ROS 2 토픽으로 로봇 원격 조작(`vr_publisher_sg2`).

- **진행된 것**: Quest 접속(HTTPS/WSS 8012) + **자기서명 인증서 자동 생성** 적용, 컨트롤러→로봇 매핑(머리·리프트·그리퍼·팔 포즈) 동작. 실행: `ros2 launch robotis_vuer vr.launch.py model:=sg2`.
- **막히는 지점**: ① Quest↔PC **네트워크**(HTTPS/WSS, 방화벽 8012, WSL2 포트포워딩) ② **Vuer 버전 차이**(검증 v0.1.5 vs 설치 v0.1.6 — 종료 시 `stop()` 없음) ③ `/joint_states` 선행 필요(follower 먼저 실행).
- **범위 주의**: 현재 `feature/mission-a` 의 `vr.launch.py` 는 `model` 인자뿐. **머리 추종·Leader 제어는 별도 브랜치** `feature/vr-head-tracking-leader-sg2`.

---

## 6. 남은 작업
- [ ] `monitor_ocr` ocr_venv 재생성 → 실 OCR → `/perception/task_list` 라이브 검증
- [x] 트레이 YOLO 모델 `tray_best.pt` 배치 (2026-06-01, segment·class `blue_tray`, 로드 검증 완료) — 단 라이브 검출 검증은 미완
- [ ] A3_PLACE용 트레이 base_link place 좌표 인터페이스 협의 (현재 tray_contents 는 2D 카운트만)
- [ ] Phase 2 Manipulation 연동 (`bin_pick`/`tray_place` Action), CM 토픽명 합의
- [ ] VR: Vuer 버전 정합 / Quest 접속 환경(방화벽·WSL2) 안정화
