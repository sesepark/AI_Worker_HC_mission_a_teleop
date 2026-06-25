> ⚠️ **아카이브됨 (2026-06-25 재구조화)** — 충족 완료된 요청 스펙입니다. 확정 계약은 [../INTERFACES.md](../INTERFACES.md), perception I/O 정본은 [../PERCEPTION_INTERFACE.md](../PERCEPTION_INTERFACE.md). 원 요청 기록으로 보존.

# [요청서] Perception — 미션 A (SDR v2.3)

> System(FSM)팀 → Perception팀. **기존 검증 파이프라인은 불변**이며, 본 요청의 신규 항목은
> C3(트레이 place 위치 유효성) 1건뿐이다. System 은 mock + guard 로 선검증했고, 실 검출 준비 시
> `use_place_pose_check:=true` 로 활성화한다.

## 1. 구현 기능
- **(기존, 불변)** `/perception/task_list`(부품 목록 JSON), `/detections`(+bbox),
  `/perception/wrist/target_one_pose`(base_link, 최종 1개 grasp target).
- **(신규) C3 — 트레이 place 위치 유효성** — gripper(또는 파지 부품)가 blue tray overlay(좌표) 유효
  범위 안에 정렬되었는지 연속 발행. 실제 적재 차감의 3번째 조건.

## 2. 인터페이스
| 이름 | 종류 | 정의 |
|---|---|---|
| `/perception/task_list` | pub `std_msgs/String`(JSON) | `{"parts":[{"name":"flange nut","count":1}, ...]}` (기존) |
| `/perception/wrist/target_one_pose` | pub `geometry_msgs/PoseStamped` | base_link, 최종 grasp target (기존) |
| `/detections` | pub `perception/PartDetectionArray` | bbox 포함 (기존) |
| **`/perception/place_pose_valid`** | pub `std_msgs/String`(JSON) | **신규 C3**: `{"valid":bool, "dx":float, "dy":float, "confidence":float}` |

> C3 는 기존 관례(JSON-on-String)를 따른다 — 신규 .msg 파일 불필요.

## 3. 입출력 / 의미·동작
- `valid=true`: gripper/부품이 트레이 유효 place 영역 내 정렬됨 → System A3_PLACE 게이트의 C3 통과 허용.
- `dx, dy`: 트레이 중심(또는 목표 셀) 대비 잔차(m, 선택; 미세 정렬 참고용).
- `confidence`: 검출 신뢰도(선택).
- System 은 **디바운스**(연속 valid 가 `place_pose_valid_debounce_sec`(0.3s) 지속) + **신선도**(<=1s)를
  적용해 떨림(flapping)에 의한 조기 release 를 방지한다. valid 전에는 release/차감하지 않는다.

## 4. success / failure
- success(C3 통과): valid 가 디바운스 시간 이상 지속 → A3_PLACE 가 release 발행.
- failure: valid 가 끝내 안 옴 → A3_PLACE timeout → RECOVERY(release/차감 없음).
- **guard**: `use_place_pose_check`(기본 False). False 면 C3 무시(C1∧C2 만으로 차감) → perception 미완 시에도
  무회귀로 동작. perception 실검출 준비 후 True 로 전환.

## 5. mock 계약 (현재 = `mock_perception_a`)
- `/perception/task_list`(설정형 `parts_json`; 기본 총 5개), `/perception/wrist/target_one_pose`(base_link),
  `/perception/place_pose_valid` 발행.
- C3 주입: `place_pose_invalid:=true`(항상 invalid), `place_pose_flap:=true`(0.2s 토글 → 디바운스 검증).

## 6. 검증법
- C3 활성: `ros2 launch mission mission_a.launch.py use_place_pose_check:=true` → 정상 valid 시 통과·release.
- 무효: `... use_place_pose_check:=true place_pose_invalid:=true` → A3_PLACE timeout → RECOVERY(무차감).
- 떨림: `... use_place_pose_check:=true place_pose_flap:=true` → 디바운스로 조기 release 없음 → 결국 timeout RECOVERY.
- 실 노드 교체: `mock_perception_a` 종료(또는 기존 perception 스택 사용) 후 `/perception/place_pose_valid` 실검출
  제공, `use_place_pose_check:=true` 로 활성화. 실 overlay 검출 정확도는 캘리브 후 별도 확인.
