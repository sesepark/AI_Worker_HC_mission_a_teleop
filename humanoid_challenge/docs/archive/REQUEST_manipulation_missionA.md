> ⚠️ **아카이브됨 (2026-06-25 재구조화)** — 충족 완료된 요청 스펙입니다. 확정 계약은 [../INTERFACES.md](../INTERFACES.md) 및 [../MISSION_A_PHASE2_RESULT.md](../MISSION_A_PHASE2_RESULT.md) §1. 원 요청 기록으로 보존.

# [요청서] Manipulation — 미션 A (SDR v2.3)

> System(FSM)팀 → Manipulation팀. 본 문서는 **인터페이스 계약**이다. System 은 mock 으로
> 선검증을 마쳤고, 실 manipulation 노드가 동일 계약을 만족하면 mock 만 끄고 교체하면 된다.
> 참고: manipulation 미션 A 방식은 `feature/manipulation` 브랜치 **읽기 참고**(병합 금지).
>
> (v2.3에서 scan 연동은 **Action 유지** — 통합 검증에서 정상 동작. navigation만 Service로 변경됨.)

## 1. 구현 기능
1. **스캔 초기 포즈 형성 (MoveToScanPose Action)** — 오른팔 wrist 카메라가 블루박스(760×480)
   전체를 내려다보도록 상체/팔을 3-waypoint + 상체 높이로 이동. **궤적·높이 preset 은 manipulation 내부 소유.**
2. **carry 자세 내부 형성** — 파지 동작이 **carry-safe 자세로 마무리**(낙하·충돌 방지)되어야 한다.
   **별도 System 호출 인터페이스 없음**(SetCarryPose 등 없음). FSM 은 base 측방 이동만 관리.
3. **파지/적재 + grip 유지 상태(C2)** — 파지 시 `/attached_object` 에 파지 class 발행, 해제 시 `""` 발행.

## 2. 인터페이스
| 이름 | 종류 | 정의 |
|---|---|---|
| `move_to_scan_pose` | action `mission_interfaces/action/MoveToScanPose` | Goal `string preset_id`("" =기본) / Result `bool success, string message` / Feedback `float32 progress` |
| `/attach_cmd` | sub `std_msgs/String` | FSM 이 A3_PICK 진입 시 파지 트리거(data="pick") 발행 → manipulation 이 파지 수행 |
| `/attached_object` | pub `std_msgs/String` | **C2 grip 유지 소스**: 비어있지 않음=파지 class 보유, `""`=해제 완료 |
| `/detach_cmd` | sub `std_msgs/String` | FSM 이 A3_PLACE 게이트 통과 시 해제 트리거(data=pick class) 발행 → manipulation 이 release 수행 |
| `/manipulator_state` | pub `std_msgs/String` | "IDLE" 발행 시 FSM INIT 통과(준비 완료 신호) |

> **C2 향후 승격(선택)**: 전용 `/manipulation/grip_state` (`{holding:bool, object:str}`) 토픽 제공 시
> System 이 그쪽을 C2 소스로 전환 가능. **과도기에는 `/attached_object` 재사용으로 충분**(현 검증 기준).

## 3. 입출력 / 의미·동작
- **MoveToScanPose**: System 이 A2_SCAN_POSE 진입 시 1회 goal 송신 → 완료(success=true)까지 대기 후 A2_SCAN.
  매 사이클(박스 복귀 후) 재형성. 충돌 없이 박스 전체 FOV 확보가 목표.
- **파지**: `/attach_cmd` 수신 → 박스에서 부품 1개 파지 → 파지 class 를 `/attached_object` 로 보고 →
  **carry-safe 자세로 마무리**. (파지 대상 pose 는 perception `/perception/wrist/target_one_pose` 기반.)
- **해제**: `/detach_cmd` 수신 → 트레이에 적재(release) → `/attached_object=""` 보고.
- **grip 유지(C2)**: 파지~해제 사이 `/attached_object` 가 파지 class 를 유지해야 한다. 이동 중 손실(드롭) 시
  `""` 로 바뀌면 FSM 이 무차감 RECOVERY 처리한다.

## 4. success / failure
- success: MoveToScanPose `success=true` + 박스 전체 FOV / 파지 후 class 보고 + carry-safe / 해제 후 `""` 보고.
- failure: 스캔 포즈 도달 실패(success=false 또는 timeout) → FSM RECOVERY. 이동 중 파지 손실(`""`) → 무차감 RECOVERY.

## 5. mock 계약 (현재 = `mock_manipulation_a`)
- MoveToScanPose 액션 서버: `scan_delay_sec`(0.5) 후 success.
- `/attach_cmd`→ task_list 미러의 next-available class 를 `/attached_object` 로 발행.
- `/detach_cmd`→ `/attached_object=""` + 미러 차감.
- `/manipulator_state=IDLE` 주기 발행.
- 드롭 주입: `drop_during_move:=true`(+`drop_after_attach_sec`) → 파지 후 N초 뒤 `""`(C2 검증용).

## 6. 검증법
- 전체 mock: `ros2 launch mission mission_a.launch.py` → A2_SCAN_POSE→A2_SCAN 전이, 파지·해제·DONE 확인.
- 드롭: `... drop_during_move:=true` → A3_MOVE_TO_TRAY 무차감 RECOVERY.
- 실 노드 교체: `mock_manipulation_a` 종료 후 동일 4개 토픽/액션 제공 노드 기동, 동일 launch 재검증(계약 불변).
  실 스캔 FOV·파지 정밀도·carry-safe 안정성은 로봇/Hand-Eye 캘리브 후 별도 확인.
