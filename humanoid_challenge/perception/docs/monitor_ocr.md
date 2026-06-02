# monitor_ocr

ZED 카메라로 대시보드 모니터를 인식하여 부품 수량 task list를 ROS2 서비스로 반환하는 패키지.

## 동작 흐름

```
ZED 카메라 (로봇)
  └─ ROS2 Image 토픽
       └─ YOLO → 모니터 정면화
            └─ PaddleOCR 숫자 인식
                 └─ 10프레임 다수결 안정화
                      └─ ROS2 서비스 응답
```

## 서비스

| Service | 타입 | 내용 |
|--------|------|------|
| `/mission_a/task_list` | `mission_interfaces/srv/GetTaskList` | 요청 시점부터 프레임을 누적해 한 번 OCR을 수행하고 부품명/수량 배열을 반환 |

---

## 실행 방법

### 사전 조건

- 노트북과 로봇이 같은 네트워크에 연결되어 있어야 함
- 로봇 SSH 접속 정보는 환경변수 또는 실행 인자로 전달
  - 예: `ROBOT=robotis@<robot-host-or-ip>`
  - 예: `ROBOT_PW=<password>`
- 로봇에 Docker 컨테이너 (`ai_worker`) 가 실행 중이어야 함

---

### 1단계: 배포 (노트북에서 한 번만)

로봇과 같은 네트워크에 연결한 뒤 **노트북**에서 실행:

```bash
ROBOT=robotis@<robot-host-or-ip> ROBOT_PW=<password> \
  bash ~/ai_worker/monitor_ocr/deploy.sh
```

이 스크립트가 자동으로:
1. `monitor_ocr/` 코드를 로봇으로 복사 (`scp`)
2. ROS2 패키지 빌드 (`colcon build`)

> **처음 실행 시** PaddleOCR 모델 다운로드로 수 분 소요될 수 있음

---

### 2단계: 로봇 bringup (로봇 터미널 1)

```bash
ssh robotis@<robot-host-or-ip>
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash
ros2 launch ffw_bringup ffw_sg2_ai.launch.py
```

---

### 3단계: OCR 노드 실행 (로봇 터미널 2)

```bash
ssh robotis@<robot-host-or-ip>
bash ~/ai_worker/monitor_ocr/run_ocr.sh
```

---

### 결과 확인 (노트북 또는 로봇 터미널 3)

```bash
ros2 service call /mission_a/task_list mission_interfaces/srv/GetTaskList \
  "{timeout_sec: 20.0, frame_count: 3}"
```

---

## 파일 구조

```
monitor_ocr/
├── monitor_ocr/
│   ├── paddle_ocr.py         PaddleOCR 3.x 헬퍼
│   ├── ocr_pipeline_parts.py 부품 수량 테이블 OCR
│   ├── frame_aggregator.py   부품 수량 10프레임 안정화
│   └── monitor_ocr_node.py   ROS2 메인 노드
├── deploy.sh                 로봇 배포 스크립트 (노트북에서 실행)
├── run_ocr.sh                OCR 노드 실행 스크립트 (로봇에서 실행)
└── README.md
```

## 파라미터

```bash
# 카메라 토픽 변경 (기본: /zed/zed_node/rgb/image_rect_color)
ros2 run perception monitor_ocr_node --ros-args -p image_topic:=/your/topic

# OCR 처리 주기 변경 (기본: 2.0초)
ros2 run perception monitor_ocr_node --ros-args -p process_interval:=1.0

# task_list service 호출
ros2 service call /mission_a/task_list mission_interfaces/srv/GetTaskList "{timeout_sec: 20.0, frame_count: 3}"
```

## 검증 결과

현재 OCR 출력은 부품명/수량 테이블만 대상으로 한다.

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `paddlepaddle` import 오류 | 컨테이너 이미지 불일치 | PaddleOCR 3.x가 포함된 고정 이미지 사용 |
| `numpy` 관련 에러 | numpy 2.x 비호환 | `pip install 'numpy<2'` |
| `cv_bridge` 변환 실패 | encoding 불일치 | `bgra8`/`rgba8` 모두 `bgr8`로 변환 처리됨 |
| 모니터 감지 실패 | 조명 조건 | `find_display_parts()` 감지 임계값 조정 필요 |
