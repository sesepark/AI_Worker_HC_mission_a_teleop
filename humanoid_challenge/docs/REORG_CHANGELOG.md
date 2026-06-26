# docs 재구조화 변경 이력 (REORG CHANGELOG)

> 일자: 2026-06-25 · 브랜치: `integration/mission-a` · 범위: `humanoid_challenge/docs/` 문서 정리(문서만, 코드 무변경).
> 원칙: **정보 손실 0** — 삭제 없음, 아카이브 이동(`git mv`, 히스토리 보존) + 리다이렉트 메모. 보호문서 본문 불변.

---

## 1. 신규 생성
| 문서 | 역할 | 출처(흡수/통합) |
|---|---|---|
| [README.md](README.md) | 문서 색인(INDEX) | PROGRESS_SUMMARY §1 문서가이드 대체 |
| [MISSION_A_STATE_MACHINE_MANUAL.md](MISSION_A_STATE_MACHINE_MANUAL.md) | FSM 실행 매뉴얼(코드 근거) | 신규(소스 직접 검증) |
| [RUNBOOK.md](RUNBOOK.md) | 실행/검증/셋업/트러블슈팅 허브 | OPERATION_MANUAL STEP A~E 흡수 + 잔류 문서 링크 + RESULT §6.2 링크 |
| [INTERFACES.md](INTERFACES.md) | 계약 통합 레퍼런스 | OPERATION_MANUAL §5 흡수 + RESULT §1 + 코드 계약표 |
| [SCENARIOS.md](SCENARIOS.md) | 시나리오 개요 + 포인터 | SCENARIO_PLAN(보호) 요약 링크 |
| [REORG_CHANGELOG.md](REORG_CHANGELOG.md) | 본 이력 | — |

## 2. 아카이브 이동 (`docs/` → `docs/archive/`)
모두 상단에 리다이렉트 메모 추가(본문 보존). 정보는 신규/보호 문서에 흡수됨.

| 원본 | 신위치 | 사유 | 정보 보존처 |
|---|---|---|---|
| `MISSION_A_OPERATION_MANUAL .md`(공백 결함) | `archive/MISSION_A_OPERATION_MANUAL.md`(공백 제거) | 불완전·새 완전판에 대체 | STEP A~E→RUNBOOK, §5→INTERFACES, (정본 RESULT §6.2에도 존재) |
| `MANIPULATION_REAL_ROBOT_TEST_PROCEDURE.md` | `archive/…` | `REAL_ROBOT_TEST_PROCEDURE.md`와 바이트 동일 중복 | 동일본 [REAL_ROBOT_TEST_PROCEDURE.md](REAL_ROBOT_TEST_PROCEDURE.md) 유지 |
| `REQUEST_manipulation_missionA.md` | `archive/…` | 충족된 요청 스펙 | INTERFACES / RESULT §1 |
| `REQUEST_navigation_missionA.md` | `archive/…` | 〃 | INTERFACES / NAV_RESULT |
| `REQUEST_perception_missionA.md` | `archive/…` | 〃 | INTERFACES / PERCEPTION_INTERFACE |
| `PROGRESS_SUMMARY.md` | `archive/…` | 시점 스냅샷(2026-05-31, 구 브랜치) | 문서가이드→README, 진행기록 보존 |

## 3. 보호 (본문 불변 — INDEX 링크만)
`MISSION_A_PHASE2_RESULT.md`, `MISSION_A_INTEGRATION_RESULT.md`, `MISSION_A_NAV_RESULT.md`, `MISSION_A_SCENARIO_PLAN.md`, `PERCEPTION_INTERFACE.md`, `MANIPULATION_INTEGRATION_ANALYSIS.md`(+ `analysis/` 덤프).

## 4. 잔류 (이동 안 함 — RUNBOOK/INDEX에서 링크·요약)
`MISSION_A_DEMO_VERIFICATION.md`, `04_INTEGRATION_VERIFICATION_GUIDE.md`, `REAL_ROBOT_TEST_PROCEDURE.md`, `MANIPULATION_SETUP_NEW_MACHINE.md`, `PERCEPTION_LOCAL_SETUP.md`, `PERCEPTION_NODE_VERIFICATION.md`, `MANIPULATION_ROS2_COMMUNICATION_ISSUES.md`.

## 5. 범위 밖 (그대로)
`VR_TELEOPERATION.md`, `../docker/README.md`, `../mission/README.md`, `perception/docs/*`.

## 6. 정정 사항
- 프롬프트가 정본 후보로 가정한 `MISSION_A_PHASE2_STATUS.md`·`MISSION_A_PHASE2_HANDOFF.md`는 **레포에 존재하지 않음**(생성 이력 없음). 실제 Phase 2 정본은 `MISSION_A_PHASE2_RESULT.md` 하나이며 "FIX-1~6"은 그 §6.2 본문에 인라인.
- 미정리 `*.bak`(`../manipulation/setup.py.bak`, `setup.py.bak2`, `../scripts/run_integration_demo.sh.bak`)은 본 재구조화 범위 밖(미이동).

---

## 결과 요약
- 신규 6 · 아카이브 6(리다이렉트 메모 포함) · 보호 6(불변) · 잔류 7 · 범위밖 4.
- **사라진 원본 문서 0**. 코드 변경 0.
