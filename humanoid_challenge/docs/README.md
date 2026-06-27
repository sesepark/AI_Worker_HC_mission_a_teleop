# Mission A 문서 색인 (INDEX)

`humanoid_challenge` 문서 모음. **무슨 문서가 무슨 내용인지 / 언제 읽어야 하는지**를 한눈에.
재구조화 이력은 [REORG_CHANGELOG.md](REORG_CHANGELOG.md).

---

## 🚀 어디서부터 읽나
| 목적 | 문서 |
|---|---|
| 상태머신이 어떻게 동작/실행되나 (정밀, 코드 근거) | **[MISSION_A_STATE_MACHINE_MANUAL.md](MISSION_A_STATE_MACHINE_MANUAL.md)** |
| 지금 당장 실행하고 싶다 (명령 모음) | **[RUNBOOK.md](RUNBOOK.md)** |
| 토픽/액션/서비스 계약이 궁금하다 | **[INTERFACES.md](INTERFACES.md)** |
| 시나리오 큰 그림 | **[SCENARIOS.md](SCENARIOS.md)** |
| 실로봇 전체 사이클 + 근본원인(FIX) | **[MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md)** (정본) |

## 📘 정본 실행 문서 (이번 재구조화 산출)
| 문서 | 내용 | 언제 |
|---|---|---|
| [MISSION_A_STATE_MACHINE_MANUAL.md](MISSION_A_STATE_MACHINE_MANUAL.md) | FSM 12상태 명세·전이·계약·게이트·실행파일 맵·불일치 (모두 `파일:라인`) | 구조 파악·추가 구현 기준 |
| [RUNBOOK.md](RUNBOOK.md) | 빌드/환경, 헤드리스 s0·s1·g5·주입, 실로봇 STEP A~E, 검증·셋업·트러블슈팅 링크 | 실행할 때 |
| [INTERFACES.md](INTERFACES.md) | 토픽·액션·서비스·메시지 + 생산자/소비자 + 불변식·스키마 | 계약 확인 |
| [SCENARIOS.md](SCENARIOS.md) | 5단계 흐름 요약 + 정본 설계 링크 | 큰 그림 |

## 🔒 보호 문서 (정본 — 본문 불변, 링크만)
| 문서 | 내용 |
|---|---|
| [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) | Phase 2 mock→실 전환 결과·확정 계약 7항목·실로봇 §6.2 절차·FIX-1~6 |
| [MISSION_A_INTEGRATION_RESULT.md](MISSION_A_INTEGRATION_RESULT.md) | 3브랜치 통합 결과·G4 무회귀·G5 라이브 데모 |
| [MISSION_A_NAV_RESULT.md](MISSION_A_NAV_RESULT.md) | 실 `MoveBaseLateral` 서버(odom 폐루프·크로스-PC·콜드 디스커버리) |
| [MISSION_A_SCENARIO_PLAN.md](MISSION_A_SCENARIO_PLAN.md) | 시나리오/FSM 설계 정본(SDR) |
| [PERCEPTION_INTERFACE.md](PERCEPTION_INTERFACE.md) | perception 노드 I/O 계약 |
| [MANIPULATION_INTEGRATION_ANALYSIS.md](MANIPULATION_INTEGRATION_ANALYSIS.md) | manipulation 통합 토폴로지 감사(+ [analysis/](analysis/) git 덤프) |

## 🛠️ 운용·검증·셋업 (잔류 — RUNBOOK에서 연결)
| 문서 | 내용 |
|---|---|
| [MISSION_A_DEMO_VERIFICATION.md](MISSION_A_DEMO_VERIFICATION.md) | mock/실 데모 & 검증, SDR §8 매핑 |
| [04_INTEGRATION_VERIFICATION_GUIDE.md](04_INTEGRATION_VERIFICATION_GUIDE.md) | 통합 스모크(빌드/실행파일/토픽 파이프라인/충돌 점검) |
| [REAL_ROBOT_TEST_PROCEDURE.md](REAL_ROBOT_TEST_PROCEDURE.md) | 실물 로봇 테스트 프로시져(로봇/노트북 세팅·순서·주의) |
| [PERCEPTION_NODE_VERIFICATION.md](PERCEPTION_NODE_VERIFICATION.md) | perception 노드별 검증 |
| [MANIPULATION_SETUP_NEW_MACHINE.md](MANIPULATION_SETUP_NEW_MACHINE.md) | 신규 머신 구축(TRAC-IK 포함) |
| [PERCEPTION_LOCAL_SETUP.md](PERCEPTION_LOCAL_SETUP.md) | perception 로컬 셋업 |
| [MANIPULATION_ROS2_COMMUNICATION_ISSUES.md](MANIPULATION_ROS2_COMMUNICATION_ISSUES.md) | 크로스 컨테이너/머신 통신 트러블슈팅 |
| [REMOTE_TELEOP_TRANSITION.md](REMOTE_TELEOP_TRANSITION.md) | 자율 Mission A 종료 후 원격 teleop 전환 절차 |

## 📦 범위 밖 / 기타
| 문서 | 내용 |
|---|---|
| [VR_TELEOPERATION.md](VR_TELEOPERATION.md) | VR 텔레오퍼레이션(Mission A 외 기능) |
| `../docker/README.md` | docker 이미지/컨테이너 |
| `../mission/README.md` | mission 패키지 빌드/sim |
| `perception/docs/monitor_ocr.md`, `part_detector.md` | perception 노드 레퍼런스 |

## 🗄️ 아카이브
대체·중복·스냅샷 문서는 [archive/](archive/)로 이동(삭제 아님, 리다이렉트 메모 포함). 매핑은 [REORG_CHANGELOG.md](REORG_CHANGELOG.md).

> 참고: 프롬프트에서 언급된 `MISSION_A_PHASE2_STATUS.md`·`MISSION_A_PHASE2_HANDOFF.md`는 **레포에 없다**. 해당 정본은 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md)이며 FIX-1~6은 그 §6.2에 인라인.
