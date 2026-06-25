# manipulation `feature/mission-a` 정밀 분석 + 통합·분리 계획

> 작성: system팀 통합 담당. 기준일 2026-06-25. **이번 라운드 = 분석 + 계획만**(merge/rebase/push 등 파괴적 git 미수행).
> 모든 결론은 read-only git 출력 근거를 첨부했다(보조 덤프: [`docs/analysis/`](analysis/)). 추정은 "확인필요"로 표기.
> 대상: 통합 `integration/mission-a`(tip `885663a`) ↔ 분석 `origin/feature/mission-a`(tip `bcc8b04`).

---

## 1. 요약 (TL;DR)

- **토폴로지 보정**: 분석 대상은 **`origin/feature/mission-a`**다. 로컬 `feature/mission-a`(`9eb3b2c`)는 **stale 조상**이라 쓰면 안 된다(merge-base가 자기 HEAD로 잡힘). merge-base = **`56ee558`** ✅(컨텍스트와 일치). manip 추가 커밋은 **7개**(컨텍스트 6개 + `e65cf34`,`bcc8b04` 신규).
- **가장 큰 반전(§3)**: `integration/mission-a`는 자기 커밋 **`141c84c`**("manipulation 팀 ai_worker_final 통합")으로 **manip의 옛 B/dual + Mission C arm-selector를 이미 흡수**했다. 그래서 "A 개선 통합"의 net-new는 매우 작고, B/C 일부는 이미 (옛 버전으로) integration에 섞여 있다.
- **통합 가능(녹색)**: A 개선 잔여 델타는 **`gripper_command.py`(+2/−19)** 와 `pick_skill.py`의 `wait_motion()` 2줄 제거뿐. 나머지 `525546d`는 이미 반영됨. `mission_c_arm_selector.py`는 이미 동일본 존재.
- **주의(황색)**: 실충돌 **3건**(`moveit_dual.py` modify/delete, `planning_scene.py` config경로, `setup.py`). `moveit_client.py`는 integration이 **오히려 최신**(FIX 커밋) → feature본 도입 금지.
- **차단(적색)**: **R4** Zone C peg/pipe Y 좌표 3중 불일치(HIGH) — 하드웨어 실측 단일화 전 실삽입 불가. **R2** 성공판정 반전(dry-run 스캐폴딩) 실동작 전 교정 필수.
- **A↔C 재사용**: A 서버는 **단일팔(`Arm.RIGHT` 하드코딩)** → C 양팔 확장이 최대 작업. perception/action/토픽 계약·primitive·`MoveBaseLateral`은 그대로 재사용, A3_PLACE만 peg 삽입으로 교체.
- **카메라**: `a212bbc`(ZED-M)는 **벤더 파일** `ffw_bringup/config/common/zedm.yaml` 수정 → 무수정 원칙상 통합 제외(이번 라운드 보류).

---

## 2. 토폴로지 (검증 완료)

근거: [`analysis/01_topology.txt`](analysis/01_topology.txt), [`analysis/04_commit_stats.txt`](analysis/04_commit_stats.txt).

```
git merge-base integration/mission-a origin/feature/mission-a → 56ee558  ✅ (컨텍스트 지목과 일치)
```

| 구분 | 브랜치 | tip | 비고 |
|---|---|---|---|
| 통합 | `integration/mission-a` | `885663a` | FSM/perception/T1-T3/nav. 56ee558 이후 37커밋 |
| 분석 | **`origin/feature/mission-a`** | `bcc8b04` | manip 팀 활성 브랜치. 56ee558 이후 **7커밋** |
| ⚠️ 함정 | 로컬 `feature/mission-a` | `9eb3b2c` | **stale 조상**(integration에 이미 포함) — 분석에 쓰지 말 것 |

**manip 7커밋(오래된→최신)** `56ee558..origin/feature/mission-a`:

| SHA | 메시지 | 미션 분류(확정) |
|---|---|---|
| `525546d` | pick_skill 버그픽스, config 경로 이동, robot_interface/tests | **A 개선** (+ config rename) |
| `3c417df` | Mission C 테스트 파일 및 arm selector 추가 | **Mission C** |
| `a212bbc` | Set ZED-M grab resolution to HD720 | **카메라(벤더 파일)** |
| `e8a14b6` | Mission B filter 적용 및 pose 탐색 | **Mission B** |
| `9ba1341` | mission B motion save | **Mission B** |
| `e65cf34` | Mission B joint move | **Mission B** (컨텍스트 미기재) |
| `bcc8b04` | Mission B wait_until_ready 변경 | **Mission B** (컨텍스트 미기재) |

---

## 3. 핵심 반전 — integration이 manip 작업을 이미 부분 흡수

근거: [`analysis/03_filelist_diff.txt`](analysis/03_filelist_diff.txt), `git ls-tree` 대조.

`integration/mission-a`는 `141c84c`("manipulation 팀 ai_worker_final 통합")·`d8de46a`·`a4820df`로 manip의 **옛 스냅샷**을 이미 보유한다. 따라서 "B/C는 feature에만 있다"는 전제는 **부분적으로 거짓**이다.

**integration·feature 양쪽에 존재(이미 흡수, 단 drift):**
- `robot_interface/moveit_dual_client.py` — 양쪽, **+295/−38 drift**(integration이 옛 버전)
- `robot_interface/planning_scene_b*.py` — 양쪽
- `tests/test_dual_{pick,home,place}.py`, `tests/test_zone_b*.py` — 양쪽
- `skill_primitives/mission_c_arm_selector.py` — 양쪽, **동일**

**integration에만 존재(정리 후보):**
- `robot_interface/moveit_dual.py` — feature가 rename으로 삭제한 **중복 stale본** (→ §4 modify/delete 충돌의 원인)
- `tests/test_dual_box.py` — feature가 `bcc8b04`에서 삭제
- `config/{desk,poses,zone_a}.yaml`(top-level) — feature가 `525546d`에서 nested로 이동하며 삭제

**무해성(inert) 확인**: A 런타임 `mission_a_manipulation_server.py`는 dual/B 모듈(`*dual*`,`zone_b`,`planning_scene_b`,`filter_dual`,`torque`,`motion_io`)을 **import하지 않는다**(grep 무결과). → 이미 섞인 옛 B/dual 코드는 **동작상 무해**하나 stale·중복·빌드 entry 혼란 존재. **[팀장 결정2] inert로 두되 정리대상으로 문서화**(이번 라운드 제거 안 함).

---

## 4. 충돌 분석 (authoritative)

근거: [`analysis/02_merge_tree_conflicts.txt`](analysis/02_merge_tree_conflicts.txt) (`git merge-tree --write-tree integration/mission-a origin/feature/mission-a`, exit≠0).

전체 3-way 머지 시 **실충돌 3건**, `planning_scene_b*`·`pick_skill.py`는 자동머지("Auto-merging").

| 파일 | 충돌유형 | 근거 | 해소(결정 반영) | 위험 |
|---|---|---|---|---|
| `manipulation/robot_interface/moveit_dual.py` | **modify/delete** | feature 삭제(rename→client) vs integration 수정 | integration 중복 stale본 → **정리대상**(결정2). 머지 시 삭제 채택 권장 | 중 |
| `manipulation/robot_interface/planning_scene.py` | **content** | config 로드 경로전략 상이(R6) | **하이브리드 폴백**(결정3): `get_package_share_directory` 시도→실패시 `__file__` 상대 | 중 |
| `manipulation/setup.py` | **content** | data_files glob + console_scripts 양쪽 상이 | glob **union**(`config/*` + `manipulation/config/*`), entry_points는 **A전용만 선별**(test_*_c/dual 엔트리는 B·C 브랜치로) | 중 |
| `planning_scene_b{,_pick,_place}.py`, `pick_skill.py` | 자동머지(충돌無) | merge-tree "Auto-merging" | integration superset 채택 | 낮 |
| `perception/`, `mission/`, `mission_interfaces/` | **변경 無(feature측)** | `git diff 56ee558 origin/feature/mission-a -- <pkg>` 빈 출력 | integration 작업 그대로 | 없음 |

### 4.1 A 개선 잔여 델타 (net-new, integration ← feature)
근거: [`analysis/05_a_improvement_deltas.txt`](analysis/05_a_improvement_deltas.txt). `525546d`는 대부분 이미 integration에 반영됨(141c84c). 남은 것:

- **`gripper_command.py` (+2/−19)** — `_send_single`이 arm joint를 동봉 전송 → **gripper joint만** 전송으로 단순화. **실질적 A 개선**(팔 관절 재명령 방지). 통합 권장.
- **`pick_skill.py` (+0/−2)** — `open_to(0.5)` 후 `wait_motion()` 2곳 제거. 개선이나 **실 그리퍼 정착 타이밍에 영향** 가능 → 실로봇 검증 후 채택(§8).
- **`moveit_client.py` (+16/−94)** — integration이 **최신**(FIX-1/2/3 `9b6ce6b`, bad-merge 복구 `ecb4be3`). feature본은 옛 버전 → **도입 금지**.
- **config 3파일** — `525546d`는 순수 rename(내용 0): `manipulation/config/*.yaml` → `manipulation/manipulation/config/*.yaml`. 경로전략은 R6 하이브리드 폴백으로 흡수.

---

## 5. Mission C 위험신호 검증 (R1~R5 + nav)

근거: `git show origin/feature/mission-a:humanoid_challenge/manipulation/manipulation/tests/MissionC/*.py` 및 `.../robot_interface/planning_scene.py` 직접 열람.

| R | 판정 | 근거(파일:라인) | 교정안 |
|---|---|---|---|
| **R1** orientation | **오탐(naming)** | `test_pick_C.py` `_QUAT_YAW90=(0.0,0.0,0.0,1.0)`=**항등**(변수명·독스트링은 yaw90/0.707). 비교: `mission_a_grasp_adapter.py` `_QUAT_YAW90=(0,0,0.7071,0.7071)` 실제 90° | C가 top-down 의도면 변수명 `_QUAT_TOPDOWN`으로 정정, 독스트링 일치 |
| **R2** 성공판정 반전 | **사실** | `test_pick_C.py:81 if result != PickResult.SUCCESS:` + 주석 "실제 로봇 작동 시 != → == 로 변경". 현재 **실패 시 carry 상승**(dry-run) | 실동작 전 `==`로(또는 `sim_mode` 플래그로 분기) — **반드시** |
| **R3** 오프셋 축 | **오탐(comment)** | `GRASP_Y_OFFSET=-0.045`가 `pose.position.y = cy + GRASP_Y_OFFSET`로 **y에 정상 적용**. 주석만 "x 오프셋" 오기 | 주석 수정 |
| **R4** peg/pipe Y 3중 불일치 | **사실 (HIGH 차단)** | `planning_scene.py ZONE_C_PEG_Y_POSITIONS=[-0.225,-0.075,0.075,0.225]` vs `test_place_C.py` 독스트링(+0.225…−0.225) vs 실제 `PIPE_POSITIONS{pipe1:+0.272, pipe2:+0.100, pipe3:−0.079, pipe4:−0.264}`. **인덱싱 방향도 반대**(peg 0→3=−y→+y, pipe1→4=+y→−y) | **규정집/실측 단일화 필요(§8)** — 그래스프(peg 좌표)와 place(pipe 좌표)가 어긋나면 충돌·오삽입 |
| **R5** peg vs pipe 명칭 | **사실** | tests="pipe"(`PIPE_POSITIONS`,`place_on_pipe`), planning_scene="peg"(`zone_c_peg_*`,`ZONE_C_PEG_*`) | 명칭 통일(peg 권장) |
| **nav 횡이동** | **stub** | `test_place_C.place_on_pipe`가 도달불가 시 `nav_y_offset`을 **목표 pose에 더하기만**(실 base 이동 없음). `test_capture_to_pick_C` 복귀는 pseudo 주석 | C FSM에서 **`MoveBaseLateral` service 재사용**(A의 A3_MOVE_TO_TRAY 패턴) |

> 추가 확인필요: `GRASP_ASSESSMENT_ENABLED=False`(integration)에서도 PickSkill→GraspSkill의 `assess_stable()`은 항상 호출됨 — 플래그는 현재 **로그용**(게이트 미연결). C도 동일 동작.

---

## 6. Mission C ↔ A 재사용 매핑 (확정)

근거: `mission_a_manipulation_server.py`(`Arm.RIGHT` 하드코딩: scan/pick/place), `mission_c_arm_selector.py`(`select_arm(y): y≥0→LEFT else RIGHT`), `mission_a.py` State enum, `place_pose_valid_node.py`(place_x/place_y/xy_tol 파라미터).

| A 자산(integration) | C 재사용 | 작업/근거 |
|---|---|---|
| perception `/perception/wrist/target_one_pose` | **그대로** | 동일 계약(PoseStamped). C 픽도 tray 부품 |
| `MoveToScanPose.action` + std_msgs/String 토픽 계약(`/attach_cmd`,`/attached_object`,`/detach_cmd`,`/manipulator_state`) | **그대로** | 동일 |
| Pick/Place/Grasp/PlanningFilter primitive | **그대로**(arm 파라미터) | C는 `arm` 전달만 |
| `MoveBaseLateral` nav(FSM 레벨) | **그대로** | C 횡이동 stub 대체 |
| `task_list.py` | **그대로** | 부품 인벤토리 |
| `mission_c_arm_selector.select_arm` | **이미 존재·동일** | 추가 작업 없음 |
| `mission_a_manipulation_server.py` (scan/pick) | **확장** | **단일팔→양팔**: `Arm.RIGHT` 하드코딩 제거, FSM이 arm 지정. CAPTURE_JOINTS_L/R(테스트에 존재) |
| `A3_PLACE`(트레이 drop) | **교체** | `C3_INSERT`: peg hover→Cartesian 하강→gripper open. 참고=`test_place_C.place_on_pipe` |
| C3 게이트 `place_pose_valid_node`(place_x/y/xy_tol) | **부분 교체** | peg 좌표로 파라미터화. 좌/우 팔 분기 시 노드 2개 또는 우측만 MVP |
| FSM `mission_a.py` 상태 | **변형 재사용** | `mission_c.py`: A 상태 복제, A3_PLACE→C3_INSERT |
| `mission_a_grasp_adapter.py` | **신규/재사용** | C는 top-down 단순 자세. 상수(0.83/1.150/±0.045) 재사용 |

**4대 통합난제**: ①단일팔→양팔 서버 확장, ②C3 게이트 tray→peg 삽입 기준 교체, ③nav 횡이동 stub→`MoveBaseLateral` 연동, ④좌표·orientation 정합(R1·R4).

---

## 7. 통합·분리 실행계획 (다음 라운드 — 이번 라운드 실행 X)

원본 무변경·새 브랜치 작업·각 단계 colcon build RC=0 + 무회귀 + 실로봇 감독.

### 7.1 A 개선 통합 → `integration/mission-a`
- **파일단위 적용**(cherry-pick 금지: `525546d`는 B/C·config rename 혼재): ① `gripper_command.py` 단순화 ② (실로봇 검증 후) `pick_skill.py` `wait_motion()` 제거 ③ `planning_scene.py` config 경로 **하이브리드 폴백**.
- 검증: `colcon build`, `run_integration_demo.sh s0`(DONE 3)/`s1`(DONE 5) 무회귀, 실 그리퍼 정착(감독).

### 7.2 Mission B 분리 → **신규 `integration/mission-b-manip`** [결정1]
- `integration/mission-a`에서 파생. feature-only B **13파일** 적용(`planning_filter_dual.py`,`dual_motion_io.py`,`left_arm_torque_pose.py`,`planning_scene_b_pick_pose.py`,`test_dual_*` 신규,`dual_motion_records/*.json`,`config/zone_b_pick_pose.yaml`) + 이미 있는 옛 B를 feature 최신으로 갱신 + `moveit_dual.py` 중복 정리.
- 기존 `integration/mission-b`(system팀 nav/LiDAR B, **별개 lineage**, merge-base `f8ed899`)와 **분리 유지**. 두 B 통합 여부는 별도 결정.
- 검증: build, test_dual_* 단독 실행(감독).

### 7.3 Mission C 신규 → **신규 `integration/mission-c`** (미존재 확인됨)
- `integration/mission-a`에서 파생. MissionC **5테스트** 적용 + R1~R5 교정(특히 **R2 실동작 전, R4 좌표 단일화**) + 서버 dual-arm 확장 + C3 게이트 peg화 + nav `MoveBaseLateral` 연동.
- 검증: build, C 단독 스크립트, 실 삽입(감독·저속·E-stop).

### 7.4 롤백
모든 작업은 새 파생 브랜치에서만. 원본 3브랜치(`integration/mission-a` 통합분 제외)·벤더 무변경.

---

## 8. 미해결·질문 (팀장 결정 필요)

| # | 항목 | 상태/필요 |
|---|---|---|
| Q1 | **R4 Zone C peg Y 좌표 정본** | planning_scene(±0.225) vs 실측 `PIPE_POSITIONS`(+0.272…) — **규정집/하드웨어 계측**으로 단일화. 기본값 임의지정 불가. **실삽입 전 차단요소** |
| Q2 | **카메라 스택 정책**(보류) | RealSense 단일 유지 vs ZED-M(`a212bbc`) 도입. ZED-M은 **벤더 `ffw_bringup` 수정**이라 예외 승인 필요 + 카메라 단일소유(USB) 정책 재검토 |
| Q3 | **nav-for-C 연동 시점** | `MoveBaseLateral` 재사용을 C 1차부터 vs stub 후속 |
| Q4 | **A 개선 `wait_motion()` 제거** 영향 | 실 그리퍼 정착 타이밍에 영향 가능 — 실로봇 검증 후 채택 |
| Q5 | **integration 옛 B/dual 정리 시점** | [결정2] 이번 라운드 inert 유지. 향후 `moveit_dual.py` 중복 등 정리 라운드 별도 |
| Q6 | **두 mission-b 브랜치 통합 여부** | [결정1] 신규 `integration/mission-b-manip` 분리. system `integration/mission-b`와 최종 합류 정책 미정 |

### 확정된 결정(이번 분석 반영)
1. Mission B → 신규 `integration/mission-b-manip`(feature 파생).
2. integration의 옛 B/dual → inert 유지 + 정리대상 문서화.
3. `planning_scene.py` config 경로 → 하이브리드 폴백(share dir→`__file__` 폴백).
4. ZED-M/카메라 → 이번 라운드 보류, `a212bbc` 통합 제외(벤더 무수정).

---

### 부록 — 보조 덤프(read-only git 출력)
- [`analysis/01_topology.txt`](analysis/01_topology.txt) — merge-base, 7커밋, stale 브랜치 경고
- [`analysis/02_merge_tree_conflicts.txt`](analysis/02_merge_tree_conflicts.txt) — `merge-tree --write-tree` 실충돌 3건
- [`analysis/03_filelist_diff.txt`](analysis/03_filelist_diff.txt) — manipulation/ 파일집합 대조(pending/integration-only)
- [`analysis/04_commit_stats.txt`](analysis/04_commit_stats.txt) — 7커밋 파일별 stat
- [`analysis/05_a_improvement_deltas.txt`](analysis/05_a_improvement_deltas.txt) — pick_skill/gripper_command/moveit_client 실제 diff
