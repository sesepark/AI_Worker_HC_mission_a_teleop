# Mission A — Nav(MoveBaseLateral) 통합 + cross-PC 콜드 디스커버리 결과

> 브랜치 `integration/mission-a-nav`(← `integration/mission-a` 파생). 원본 3브랜치 무변경(drop-in).
> 작성 기준일 2026-06-24. 지시 정본: `~/docs_integration/MISSION_A_NAV_INTEGRATION_{PROMPT,BRIEF}.md`.
> **정직성 원칙**: 아래는 **확정(✅ 본 세션 검증)**과 **추정/미검(🟡 사용자·실로봇 필요)**을 분리해 기록한다.

---

## 0. 한 줄 요약

mission-a FSM은 이미 `MoveBaseLateral` **service client** 를 완비하고 있었고 `.srv` 도 이미 존재했다.
실제로 필요했던 것은 **실 service 서버** 였다. mission-b의 `sg2_lateral_jog`(odom 폐루프 측방 strafe)
로직을 `mission` 패키지의 **새 service 서버**(`move_base_lateral`)로 재구현했다. cross-PC 통신 문제의
핵심으로 지목된 "nav 콜드 디스커버리"는 **service 디스커버리가 stale 데몬/참가자 캐시 때문에 첫
호출에서 실패**하는 것으로 재현·규명했고, ① FSM INIT 비차단 워밍업 ② `wait_for_service` 타임아웃
파라미터화(2s→10s) ③ 런북(데몬 정지·env/RMW 패리티)으로 해소했다.

---

## 1. 측량으로 드러난 핵심(요청서 전제와 달랐던 점) — 확정 ✅

| # | 사실 | 근거 |
|---|---|---|
| 1 | **FSM nav seam은 이미 완성된 service client** (`_nav_step`/`_on_nav_result`/`_nav_cli`, `nav_mode={stub\|service}`). "stub→service 호출 교체"는 사실상 완료 상태였다. | `mission/mission/mission_a.py:167,392-398,400-434` |
| 2 | **`mission_interfaces/srv/MoveBaseLateral.srv` 이미 존재·등록.** 신규 `.srv` 불필요. | `mission_interfaces/srv/MoveBaseLateral.srv`, `CMakeLists.txt:14` |
| 3 | **mission-b에 MoveBaseLateral '실 서버' 없음.** `create_service(MoveBaseLateral,…)`는 *mock* 한 곳뿐. | `git grep create_service integration/mission-b` |
| 4 | mission-b "실 nav"(`ffw_mission_b_nav`: `sg2_mission_b_system_nav`+`sg2_mission_b_route`)는 **Mission B용 다른 아키텍처**(String 토픽+subprocess+A↔B 전후/우 세그먼트+LiDAR). drop-in 불가, 수정 허용 패키지도 아님. | `integration/mission-b:humanoid_challenge/navigation/ffw_mission_b_nav/` |
| 5 | **이식 자산은 `ffw_manual_tools/sg2_lateral_jog.py`**(Apache-2.0/ROBOTIS): odom 폐루프 측방 strafe 단발 노드 → **service 서버로 재구현**. | `integration/mission-b:…/manual_tools/…/sg2_lateral_jog.py` |
| 6 | 베이스는 **SG2 swerve(홀로노믹)** — `Twist.linear.y` 측방 네이티브 지원, `/cmd_vel` 소비·`/odom` 발행은 bringup(로봇 PC). 따라서 nav 서버는 로봇 PC, FSM은 desktop → **cross-PC service**. | `ai_worker/ffw_swerve_drive_controller/…`, bringup controller yaml |
| 7 | **검증된 cross-PC RPC 선례 부재(콜드 디스커버리가 진짜 핵심).** commit `073d280`/§6.2가 "manip 서버=메인 PC"로 정정 → `MoveToScanPose` action·perception은 desktop 로컬. request/reply service가 cross-PC로 매칭된 사내 선례 없음. (단 `mission_a_real.launch.py` docstring은 'manip=로봇 PC'로 남아 모순 — 🟡 실제 배치는 사용자 확인). | `MISSION_A_PHASE2_RESULT.md` §6.2, `mission_a_real.launch.py` docstring |

---

## 2. 변경/생성 (모두 `mission` 패키지 — 허용 범위)

| 경로 | 동작 | 요지 |
|---|---|---|
| `mission/mission/move_base_lateral_node.py` | **신규** | 실 `MoveBaseLateral` 서버. odom 폐루프 strafe(`sg2_lateral_jog` 재구현), fail-safe, distance=0 무이동 |
| `mission/launch/move_base_lateral.launch.py` | **신규** | 로봇 PC 단독 기동 launch(파라미터 노출). 기존 mock/stub 회귀 무영향 |
| `mission/setup.py` | 수정 | `move_base_lateral` console_scripts 추가 |
| `mission/package.xml` | 수정 | `<depend>nav_msgs</depend>` 추가(`Odometry`) |
| `mission/mission/mission_a.py` | 수정(국소) | C1 INIT nav 비차단 워밍업 + C2 `nav_service_wait_sec`(기본10) 파라미터화 |
| `mission/launch/mission_a.launch.py` | 수정(소) | `nav_service_wait_sec` arg 노출 |

> **무변경**: `MoveBaseLateral.srv`(이미 존재), FSM nav seam 본체, mock 3종, 원본 3브랜치, 벤더 패키지.
> FSM diff = 16라인(3 hunk)으로 국소화. `git diff --stat integration/mission-a` = 위 4개 수정 + 2개 신규뿐.

### 2.1 실 서버 설계 요점
- **콜백 내 블로킹(동기 전체 이동)** — mock 계약·FSM async-latch(한 `call_async`=한 이동) 일치.
  `MultiThreadedExecutor`+`ReentrantCallbackGroup`(service/odom 분리)로 블로킹 중에도 odom 갱신.
- **모션**: `sign=+1(left)/−1(right)`, `cmd.linear.y=sign*speed`@`rate_hz`, 로봇프레임 측방 델타
  `left=-sin(yaw0)·dx+cos(yaw0)·dy`, `sign*left>=distance_m` 시 정지. wrong-direction/max_duration 가드.
- **fail-safe(사용자 요구)**: odom 신선도 가드(미수신→`arrived=False,"no odom"`, **무이동**) / 전 종료경로
  zero-Twist(try-finally) / `fail_inject` / `distance_mm=0` 즉시 무이동 성공(콜드콜 안전).
- **타임아웃 정합**: 서버 `max_duration_sec`(12) < FSM `base_move_timeout_sec`(30) → 서버가 항상 먼저 응답.

---

## 3. cross-PC 콜드 디스커버리 — 진단·규명·해결

### 3.1 증상 재현 (확정 ✅)
`nav_mode=service` + mock 을 **동일 호스트 동일 launch** 로 띄웠는데, mock 이 `ready`(t=565.2) 후에도
FSM의 `wait_for_service()` 가 **매번 타임아웃** → `nav 서비스 준비 대기 중` 반복 → 30s 후 RECOVERY.
반면 같은 실행에서 **scan ACTION 은 정상 디스커버리**(A2 통과). 즉 **service 만 디스커버리 실패**.

### 3.2 근본원인 규명 (확정 ✅ — 최소 재현으로 격리)
독립 rclpy 클라이언트로 mock service 를 호출하는 최소 재현을 만들어 도메인별로 반복:
- **오염된 도메인(직전 테스트 잔여 프로세스·stale ros2 daemon 존재)**: `wait_for_service → False`(15s).
- **`ros2 daemon stop` + 잔여 프로세스 정리 후**: 도메인 30·95 **모두 `wait_for_service → True`**(4/4 반복).

→ **원인 = service request/reply 엔드포인트가 stale 데몬/참가자 캐시 때문에 첫(콜드) 매칭에 실패.**
pub/sub·action 보다 service 엔드포인트 매칭이 이 상태에 더 민감했다. **코드 결함이 아니라 디스커버리
환경(데몬/캐시) 문제**임을 격리 입증. (cross-PC 환경에서는 여기에 RMW/env 비대칭이 더해지면 악화.)

### 3.3 적용한 해결 (확정 ✅)
1. **C0(런북, 1순위)** — nav 기동 전 양 PC `ros2 daemon stop` + 잔여 프로세스 정리, env/RMW 패리티
   (`ROS_DOMAIN_ID=30`/`ROS_LOCALHOST_ONLY=0`/`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, **RMW 동일**).
   ⚠️ `scripts/run_integration_demo.sh`(domain 90·LOCALHOST_ONLY=1)를 cross-PC 에 소스 금지.
2. **C1(코드, 비차단)** — `_servers_ready()` INIT 에서 `nav_mode=='service'`면
   `self._nav_cli.wait_for_service(timeout_sec=0.0)` 그래프 nudge(게이트 아님 → INIT 무차단). cross-PC
   request/reply 매칭을 ~1 사이클 일찍 시작. (`mission_a.py:355-372`)
3. **C2(코드, 파라미터화)** — 콜드 `wait_for_service` 타임아웃 `2.0`→`self.nav_service_wait_sec`(기본10),
   launch arg 노출. timeout 시 `pending` 반환→매 틱 재시도, 상태 timeout `base_move_timeout_sec`(30)로
   유계(무한 대기 없음). (`mission_a.py:417,148-153` + `mission_a.launch.py`)
4. **서버측(C3, 본 설계 충족)** — `create_service` 를 `__init__` 에서(무거운 init 없이) 생성해 그래프 조기 광고.

> 적용 후, **데몬 정리된 깨끗한 실행에서 nav_mode=service 가 콜드 첫 호출부터 성공**함을 §4-T6/T7 로 입증.

---

## 4. 검증 결과 (본 세션, humanoid_challenge 컨테이너, 로봇 無)

| ID | 시험 | 방법 | 결과 |
|---|---|---|---|
| **T1 빌드** | colcon RC=0 | `colcon build --packages-select mission_interfaces mission` | ✅ 2 pkg finished, RC=0. executable `mission move_base_lateral` 설치, 모듈 import OK |
| **T2 무이동 안전경로** | distance=0 / invalid dir / no-odom (rclpy 직접 클라) | 단일 서버 domain30 | ✅ `dist=0→arrived=True "no-op cold-call ok"`, `invalid dir→arrived=False`, `no-odom 675→arrived=False "refuse blind move"` (서버 로그 일치) |
| **T3 fail_inject** | 강제 실패 | `-p fail_inject:=true` | ✅ `arrived=False "navigate injected-failure"` |
| **T4 폐루프 strafe** | 합성 odom(좌진행) + distance=300 | odom sim + 서버 | ✅ `arrived=True odom_distance_reached left_delta=+0.366m err=+66mm`(20Hz 체크 오버슈트 정상), `linear.y=+0.12` 발행 후 정지 |
| **T5 무회귀(stub/sim)** | `run_integration_demo.sh s0/s1` | domain91/92 | ✅ s0=**DONE 적재 3**, s1=**DONE 적재 5**(매 사이클 `nav stub instant success`) — 무회귀 |
| **T6 service+mock E2E** | `mission_a.launch.py nav_mode:=service`(데몬 정리) | domain30 | ✅ 5사이클 `MoveBaseLateral.srv 호출(left/right 675mm)`→`[mock_nav] arrived=True`→**DONE 적재 5** |
| **T7 실 서버 드롭인** | mock 대신 실 `move_base_lateral`(개루프) + FSM service | domain30 | ✅ 5사이클 `MoveBaseLateral.srv 호출`→실서버 `MoveBaseLateral move … arrived=True`→**DONE 적재 5** |
| **T8 콜드 디스커버리 격리** | rclpy 클라 mock 호출, 오염 vs 데몬정리 | 도메인 반복 | ✅ 오염=`False`, 데몬정리 후=`True`(30·95 4/4) → §3.2 |

> **콜드 첫 service 호출 성공(통신 문제 해소 증거)** = T2(실 서버 distance=0 콜드콜) + T6(FSM service 콜드 사이클).

---

## 5. 미검·다음 단계 (🟡 사용자 감독·실로봇 필요)

- **실 베이스 측방 이동**: 보조 AI는 **코드/통신 검증까지만**(안전 경계). 실 strafe(675mm 왕복)는 사용자가
  저속·E-stop 준비 하에 트리거. 절차:
  - `[robot PC]` `ros2 launch mission move_base_lateral.launch.py`
  - `[desktop]` `ros2 launch mission mission_a.launch.py use_mocks:=false nav_mode:=service`
    (필요 시 `nav_service_wait_sec` 조정). `A3_MOVE_TO_TRAY`(left)/`A3_RETURN_TO_BOX`(right) 실 strafe,
    `fail_inject:=true` 시 RECOVERY 라우팅 확인.
- **cross-PC 실측**: 동일 호스트에서 콜드 디스커버리·해결을 입증했으나 **로봇 PC↔desktop 실 cross-PC
  타이밍**은 미측정. §3.1-3.2 진단(데몬 정지·env/RMW 패리티·콜드/웜 `ros2 service call` 타이밍,
  `distance_mm=0` 무이동)을 양 PC 에서 수행해 첫 호출 지연을 실측하고 `nav_service_wait_sec` 를 보정.
- **675mm 정밀도·복귀 누적오차**: odom 캘리브 후 실로봇에서 확인(`lateral_error_mm` 텔레메트리 활용).
- **`/cmd_vel` `linear.y` 실수용 확인**: swerve 컨트롤러가 측방 strafe 를 의도대로 실행하는지 실차 확인
  (홀로노믹 가정은 컨트롤러 config 로 확인했으나 실모션 미검).
- **manip/scan 서버 실제 호스트**(§1-7 모순) 확정 → cross-PC 선례 유무 정정.

---

## 6. 재현 명령 (요약)

```bash
# 빌드 (컨테이너)
docker exec -it humanoid_challenge bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
cd /ws && colcon build --packages-select mission_interfaces mission

# 무회귀(stub/sim)
/ws/src/humanoid_challenge/scripts/run_integration_demo.sh s0   # DONE 적재 3
/ws/src/humanoid_challenge/scripts/run_integration_demo.sh s1   # DONE 적재 5

# service 경로 E2E (mock) — 콜드 디스커버리 회피: 먼저 데몬 정리
ros2 daemon stop
ros2 launch mission mission_a.launch.py nav_mode:=service travel_sec:=0.3   # DONE 적재 5

# 실 서버 단독 + 무이동 콜드콜(베이스 안움직임)
ros2 launch mission move_base_lateral.launch.py
ros2 service call /move_base_lateral mission_interfaces/srv/MoveBaseLateral "{direction: left, distance_mm: 0.0}"
```

## 7. 수용 기준 대비

1. ✅ 새 파생 브랜치 `integration/mission-a-nav`, 원본 3브랜치 무변경.
2. ✅ mission-b 자산(`sg2_lateral_jog`) 이식·재구현, **colcon RC=0**.
3. ✅ nav stub→실 service: FSM seam은 이미 service client(무변경 본체) + **실 서버 신규 제공**, 무회귀(s0/s1, service E2E).
4. ✅ cross-PC 통신 문제 **원인 규명(stale 데몬/참가자 캐시 → service 콜드 매칭 실패) + 해결(C0/C1/C2)**,
   **콜드 첫 service 호출 성공** 로그 입증(T2/T6/T8). 🟡 실 cross-PC 타이밍 실측은 사용자 단계.
5. ✅ 본 문서로 추정/확정 분리 기록.
