> ⚠️ **아카이브됨 (2026-06-25 재구조화)** — 충족 완료된 요청 스펙입니다. 확정 계약은 [../INTERFACES.md](../INTERFACES.md), 실 구현 결과는 [../MISSION_A_NAV_RESULT.md](../MISSION_A_NAV_RESULT.md). 원 요청 기록으로 보존.

# [요청서] Navigation (모바일 베이스) — 미션 A (SDR v2.3)

> System(FSM)팀 → Navigation팀. 본 문서는 **인터페이스 계약**이다. System 은 mock 으로
> 선검증을 마쳤고, 실 navigation 노드가 동일 계약을 만족하면 mock 만 끄고 교체하면 된다.
>
> **★ v2.3 변경: 연동을 Action → Service 로 변경.** "좌표/방향+거리 전달 → 이동 → 성공 반환"은
> 본질적으로 request/response 이며, Action 의 다중 엔드포인트가 통합 멀티노드 디스커버리에서
> 병목을 유발했다. Service 로 단순화해 병목 원인을 제거하고 구현도 단순해진다(피드백/취소 불필요).

## 1. 구현 기능
- **측방 dead-reckon 이동 (MoveBaseLateral Service)** — 박스(우)↔트레이(좌) 중심 간 측방 strafe.
  - A3_MOVE_TO_TRAY: 좌 `distance_mm`(규격 근거 675) → 트레이 정면 정렬.
  - A3_RETURN_TO_BOX: 우 `distance_mm` → 박스 정면 복귀.
- 장애물 회피·SLAM 불필요(rule-based, 캘리브 고정 이동량 재생).
- **carry 자세와 무관** — carry 자세는 manipulation 내부 로직, navigation 은 base 이동만.

## 2. 인터페이스
| 이름 | 종류 | 정의 |
|---|---|---|
| `move_base_lateral` | **service** `mission_interfaces/srv/MoveBaseLateral` | Request `string direction`("left"\|"right"), `float32 distance_mm` / Response `bool arrived, float32 lateral_error_mm, string message` |

> 서비스 이름은 FSM 파라미터 `nav_service_name`(기본 `move_base_lateral`)로 재설정 가능.

## 3. 입출력 / 의미·동작
- System 이 A3_MOVE_TO_TRAY 에서 `Request{direction:"left", distance_mm:base_shift_mm}`로 호출 →
  `arrived=true` 응답 시 A3_PLACE 로 진행.
- A3_RETURN_TO_BOX 에서 `Request{direction:"right", distance_mm:base_shift_mm}` 호출 → `arrived=true` 후 A2_SCAN_POSE.
- `lateral_error_mm`: 도착 측방 잔차(부호). 로그·모니터용(현재 게이트 아님; 캘리브 후 ±적재 정밀도 판단 활용).
- 이동 중 파지 유지는 perception/manipulation 책임(navigation 은 base 만).

## 4. success / failure
- success: Response `arrived=true`(목표 측방 위치 도달).
- failure: `arrived=false` 또는 응답 지연/`base_move_timeout_sec` 초과 → FSM RECOVERY.
  - 주의: 드롭 후 RECOVERY 시 base 가 트레이(좌)에 있을 수 있음 — 박스 복귀 선행 RECOVERY 서브스텝은
    실로봇 캘리브 후 추가 예정(현 v2.3 범위 외).

## 5. mock 계약 (현재 = `mock_navigation_a`, **Service 서버**)
- `MoveBaseLateral` 서비스 서버: `travel_sec`(1.0) 동안 모사 후 `arrived` 반환, `lateral_error_mm`(2.0) 주입.
- `fail_arrive:=true` → `arrived=false`(→ FSM RECOVERY) 검증.

## 6. 검증법
- **단계1(nav=stub)**: navigation 미구현이어도 `nav_mode:=stub`(기본)으로 FSM 전 구간 검증 가능(서비스 호출 없음).
- **단계2(nav=service)**: `ros2 launch mission mission_a.launch.py nav_mode:=service` → 매 사이클
  `[A3_MOVE_TO_TRAY] MoveBaseLateral.srv 호출 (left 675mm)` / `right 675mm` 로그, mock 응답 `arrived=true`로 DONE.
- 실패: `... nav_mode:=service fail_arrive:=true` → A3_MOVE_TO_TRAY RECOVERY.
- 단독 테스트: `ros2 service call /move_base_lateral mission_interfaces/srv/MoveBaseLateral "{direction: left, distance_mm: 675.0}"`.
- 실 노드 교체: `mock_navigation_a` 종료 후 동일 서비스 제공 노드 기동, `nav_mode:=service`로 동일 검증(계약 불변).
  실 675mm 왕복 정밀도·복귀 누적오차는 로봇/캘리브 후 별도 확인.
