# Mission A 데모 & 검증 매뉴얼 (SDR v2.3)

> 대상: System 팀 mission_a FSM + Mission A mock 3종. **로컬(mock)** 과 **실로봇** 두 환경에서
> 데모·검증하는 절차. SDR v2.3은 navigation 연동을 **Service(`MoveBaseLateral.srv`)** 로 바꾸고
> **`nav_mode={stub|service}`(기본 stub)** 로 검증을 단계화했다.

---

## 1. 두 환경 정의·전제
| 환경 | 구성 | 전제 |
|---|---|---|
| **로컬(mock)** | mission_a + mock_manipulation_a + mock_navigation_a(Service) + mock_perception_a (또는 sim_mode SimDriver) | 로봇 불필요. 컨테이너 `humanoid_challenge`(image shpark1104/humanoid_challenge:jazzy), repo bind-mount `/ws/src/humanoid_challenge` |
| **실로봇** | 로봇 bringup → perception 실노드 → mission_a → (추후) 실 manipulation/navigation | 로봇 PC + 캘리브레이션 |

빌드(공통): `cd /ws && colcon build --symlink-install --packages-up-to mission`

---

## 2. 로컬(mock) 데모 & 정상동작 검증

> 매 실행 전 잔존 노드 정리: `pkill -9 -f "lib/mission/[m]ission_a"; pkill -9 -f "lib/mission/[m]ock_"`
> 격리: `export ROS_DOMAIN_ID=<n>; export ROS_LOCALHOST_ONLY=1`

### 단계0 — 무회귀(sim, 신규 액션/서비스 우회) — ✅ 검증됨
```
ros2 launch mission mission_a.launch.py sim_mode:=true use_mocks:=false use_task_list_service:=true
```
기대(검증 결과): INIT→A1_MONITOR→A2_SCAN_POSE→A2_SCAN→A3_PICK→A3_MOVE_TO_TRAY→A3_PLACE→VERIFY→A3_RETURN_TO_BOX→…→**DONE (적재 3개)**. **차감이 A3_PLACE에서 발생**(detach 3 = 적재 3), VERIFY 라우팅 전용, RECOVERY 0. (SimDriver SIM_INITIAL: flange_nut×1, hex_nut×2.) → 기존 파이프라인·consume-once·sim_mode 무회귀 확인.

### 단계1 — nav=stub, 전용 mock (로봇/실서비스 없이 전 구간) — ✅ 검증됨 (**SDR 1차 완료 기준**)
```
ros2 launch mission mission_a.launch.py            # nav_mode 기본 stub, 기본 parts 총 5
```
기대(검증 결과): 5사이클 전 흐름 **DONE (적재 5개)**. `MoveToScanPose -> success` ×5(scan 액션 실연동), `nav stub instant success (left/right 675mm)`, A3_PLACE `/detach_cmd` 발행·차감, **준비대기(서버 미발견) 0**. (v2.2의 nav 액션 EDP 병목 없음 — nav=Service + scan 단일 ActionClient.)

#### 단계1 주입 시험 (오선언 0 — 게이트 안전성) — ✅ 검증됨
| 시험 | 명령 인자 | 기대 / 검증 결과 |
|---|---|---|
| **C3 유효** | `use_place_pose_check:=true` | 게이트 통과·release·**DONE 적재 5** ✅ |
| **C3 무효** | `use_place_pose_check:=true place_pose_invalid:=true` | release 안함·**적재 0·detach 0**(무차감) ✅ |
| **C3 플랩(디바운스)** | `use_place_pose_check:=true place_pose_flap:=true` | 디바운스로 조기 release 없음·**적재 0** ✅ |
| **C2 드롭(release 전)** | `use_place_pose_check:=true place_pose_flap:=true drop_during_move:=true drop_after_attach_sec:=0.5` | A3_PLACE C3 dwell 중 드롭 → `release 전 파지 손실(드롭) -> RECOVERY (무차감)`·**적재 0**·재시도→MANUAL_WAIT ✅ |
| **nav 실패(서비스)** | `nav_mode:=service fail_arrive:=true` | A3_MOVE_TO_TRAY 실패 → RECOVERY (서비스 모드 한정, §아래 주의) |

> **오선언 0 입증**: C2 드롭·C3 무효·C3 플랩 모두 **실제 적재(차감)=0** — 미충족 시 크레딧 없음.
> A3_MOVE_TO_TRAY 이동-중 C2 모니터(이동에 duration 필요)는 nav=service에서만 의미가 있어
> §2-주의의 디스커버리 제약을 받는다(코드 존재; release-전 C2 branch-0는 위 표로 런타임 검증).

### 단계2 — nav=service (실 서비스 연동) — ⚠️ 컨테이너 디스커버리 제약
```
ros2 launch mission mission_a.launch.py nav_mode:=service
```
- 의도: A3_MOVE_TO_TRAY/RETURN에서 `MoveBaseLateral.srv` 실호출 → arrived 반환으로 전 구간 DONE.
- **현 컨테이너(WSL2/Fast-DDS) 제약**: **동시 기동 시 mission_a 가 mock_navigation_a(서비스 전용·토픽 트래픽 없는 최경량 참가자) 의 서비스 매칭을 완료하지 못하는** 디스커버리 quirk 관측(`[A3_MOVE_TO_TRAY] nav 서비스 준비 대기 중` 반복). scan 액션·perception 토픽은 정상 매칭됨.
- **이는 환경 이슈(코드/계약 아님)**: 동일 컨테이너에서 (a) 최소 노드(scan ActionClient+nav ServiceClient+subs)는 nav 서비스를 ~2s에 발견, (b) nav 서비스 서버는 단독 호출에 정상 응답. 실로봇/실 navigation 노드(순차 bringup·다른 DDS 설정)에서는 해당 없음.
- **회피책**(필요 시): 실 navigation 노드를 mission_a 보다 먼저 기동(순차 bringup), 또는 단계1(stub)로 전 로직 검증 후 실노드로 단계2 수행.

---

## 3. 실제 로봇 데모 & 정상동작 검증
1. 로봇 bringup → perception 실노드(검증된 토픽 파이프라인) → `ros2 run mission mission_a --ros-args -p nav_mode:=service`(실 navigation 노드 준비 후).
2. 정상동작 확인점: 스캔 포즈 FOV(박스 전체), 675mm 좌/우 왕복 실정밀도, ±적재 위치, C3 `place_pose_valid` 실검출, A-1 OK 사인·A-2 인식/bbox 미러.
3. 안전: E-Stop 등 비상정지 경로, 충돌 여유. (정밀도·FOV·왕복오차는 캘리브 후 별도 측정.)

---

## 4. 미구현 기능 맵 (mock 현황 → 실구현 교체점)
| 팀 | 기능 | 인터페이스 | 현재 | 실구현 교체점 |
|---|---|---|---|---|
| Manipulation | 스캔 포즈 형성 | `move_to_scan_pose` **action** | mock_manipulation_a(success) | 동일 액션 제공 실노드 기동 후 mock 종료 |
| Manipulation | carry 자세(내부) | (인터페이스 없음) | 파지 후 carry-safe 마무리(요건) | manipulation 내부 구현 |
| Manipulation | 파지/적재 + grip(C2) | `/attach_cmd`,`/detach_cmd`,`/attached_object` | mock 반응형 | 실 파지/적재 + `/attached_object`(또는 `/manipulation/grip_state`) |
| Navigation | 측방 675 이동 | `move_base_lateral` **service** | mock_navigation_a(Service) | 동일 서비스 제공 실노드, `nav_mode:=service` |
| Perception | place 위치 유효성(C3) | `/perception/place_pose_valid` | mock_perception_a + guard | 실검출, `use_place_pose_check:=true` |
| Perception | task_list/detections/wrist | (기존) | 실/통합 | 변경 없음 |

---

## 5. 통합 & 정상작동 시험 (실구현 도착 시)
- mock→실노드 **점진 교체**: 해당 mock만 종료하고 동일 인터페이스 실노드 기동, 동일 launch 재실행.
- nav 활성화: `nav_mode:=service`(실 navigation 노드 준비 후). C3 활성화: `use_place_pose_check:=true`.
- batch 옵션(`rescan_each_cycle:=false`)은 nav 675 왕복 정밀도 검증 후 조건부 활성(복귀 누적오차 위험).

---

## 6. SDR §8 체크리스트 매핑 (로컬 가능 vs 실로봇 필요)
| 항목 | 로컬(mock) | 실로봇 |
|---|---|---|
| 신규 흐름 5사이클·스캔포즈·좌우675·DONE | ✅ 단계1(stub) | FOV·675 실정밀도 |
| place 3조건 차감(C1∧C2∧C3) | ✅ 단계1 주입 시험 | C3 실검출 |
| C2 드롭·C3 무효 무크레딧(오선언 0) | ✅ 단계1 주입 시험 | — |
| nav 실서비스 연동(단계2) | ⚠️ §2 디스커버리 제약(컴포넌트 검증) | ✅ 실 navigation 노드 |
| 무회귀(기존 파이프라인·sim_mode) | ✅ 단계0 | — |
| RECOVERY/MANUAL_WAIT | ✅ (C2/C3 시험에서 관측) | — |

> 정본 SDR(`docs/MISSION_A_SDR_v2.md`)은 외부 보유본으로 repo 미포함. 본 매뉴얼은 v2.3 기준.
