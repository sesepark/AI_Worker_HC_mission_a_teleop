# Mission A 시나리오 개요 (SCENARIOS)

> Mission A 시나리오의 **개요·포인터**. 정본 설계 문서는 보호문서 [MISSION_A_SCENARIO_PLAN.md](MISSION_A_SCENARIO_PLAN.md)(SDR, 26KB)이며, 본 문서는 중복을 만들지 않고 흐름 요약 + 링크만 제공한다.
> 코드 기준 상태머신 명세는 [MISSION_A_STATE_MACHINE_MANUAL.md](MISSION_A_STATE_MACHINE_MANUAL.md) §2.

---

## 한 줄 시나리오
박스의 부품(너트·링 5종)을 **인식 → 파지 → 트레이로 측방 이동 → 적재**하고, 잔여가 있으면 박스로 복귀해 반복, 모두 적재하면 완료(DONE).

## 5단계 흐름(정상 1사이클)
1. **목표 확정 (A1_MONITOR)** — perception `task_list`로 부품 종류·수량 결정.
2. **스캔 (A2_SCAN_POSE → A2_SCAN)** — manipulation 스캔 포즈 형성 후 wrist 카메라로 부품 1개의 `target_one_pose`(base_link) 수신.
3. **파지 (A3_PICK)** — `/attach_cmd` → 성공 시 `/attached_object=class` 래치(C2).
4. **이동·적재 (A3_MOVE_TO_TRAY → A3_PLACE)** — 베이스 좌 675mm → C1∧C2∧C3 게이트 통과 → `/detach_cmd` → 해제 확정 시 잔여 차감.
5. **루프/완료 (A3_RETURN_TO_BOX → VERIFY)** — 잔여>0이면 베이스 우 675mm 복귀 후 2번으로, 잔여 0이면 **DONE**.

대상 부품 5종: `flange_nut`(플랜지 너트), `gear_ring`(기어 링), `spacer_ring`(스페이서 링), `hex_nut`(육각 너트), `dome_nut`(돔 너트). (`mission/mission/task_list.py:34-39`)

## 정본 설계 문서 안내
| 절 | 내용 |
|---|---|
| [SCENARIO_PLAN §State Machine 전체 구조](MISSION_A_SCENARIO_PLAN.md) | 상태 구조 설계 |
| §Step별 로직 상세 | 단계별 의사코드/판정 |
| §Blackboard 키 | 팀 공통 데이터 키 |
| §전제 조건 / 블로커 | 의존성·블로커 |

> 데모/검증 시나리오(mock vs 실로봇, 주입 시험)는 [MISSION_A_DEMO_VERIFICATION.md](MISSION_A_DEMO_VERIFICATION.md)와 [RUNBOOK.md](RUNBOOK.md) §2.
