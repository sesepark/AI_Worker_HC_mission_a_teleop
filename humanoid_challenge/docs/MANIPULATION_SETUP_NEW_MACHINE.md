# 신규 머신 환경 구축 프로시져

## 1. 레포 클론

```bash
cd ~
mkdir ai_worker_dev && cd ai_worker_dev

git clone https://github.com/snu-shape/ai_worker.git AI_WORKER_ROBOT_FINAL
git clone -b feature/mission-a https://github.com/snu-shape/AI_Worker_HC.git AI_Worker_Final
```

---

## 2. 컨테이너 시작

```bash
# ai_worker (이미지 없으면 자동 pull — 약 10GB, 시간 걸림)
cd ~/ai_worker_dev/AI_WORKER_ROBOT_FINAL/docker
./container.sh start

# humanoid_challenge (이미지 없으면 자동 pull — 약 12GB)
cd ~/ai_worker_dev/AI_Worker_Final/humanoid_challenge/docker
./container.sh start
```

---

## 3. TRAC-IK 설치 (ai_worker 컨테이너, 매번 필요)

> 컨테이너 재시작할 때마다 다시 설치해야 함 (이미지에 포함 안 됨)

```bash
docker exec -it ai_worker bash
apt update
apt install -y ros-jazzy-trac-ik-kinematics-plugin ros-jazzy-trac-ik-lib
```

설치 확인:
```bash
# 아래 명령 실행 후 kinematics_solver가 trac_ik_kinematics_plugin 이면 정상
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 param get /move_group robot_description_kinematics.arm_r.kinematics_solver
```

> MoveIt이 뜨기 전에는 확인 불가 — Terminal 2(MoveIt launch) 실행 후 확인할 것
> 결과가 `kdl_kinematics_plugin` 이면 설치 안 된 것

---

## 4. ai_worker 워크스페이스 빌드 확인

이미지에 워크스페이스가 미리 빌드되어 있음. config yaml 파일은 심링크라 재빌드 불필요.

만약 빌드가 안 되어 있거나 launch가 실패하면:

```bash
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 5. humanoid_challenge 전체 빌드 (최초 1회)

```bash
docker exec -it humanoid_challenge bash

source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

> 이후 컨테이너 재시작 시 재빌드 불필요 (named volume으로 build/install 유지)

---

## 6. 실행 순서

```bash
# Terminal 1 — ai_worker: 로봇 bringup
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py

# Terminal 2 — ai_worker: MoveIt
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ffw_moveit_config moveit.launch.py

# Terminal 3 — humanoid_challenge: 테스트
docker exec -it humanoid_challenge bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 run manipulation test_pick
```

---

## 7. 같은 머신에서 통신 안 될 때

증상: `ros2 node list`는 보이는데 테스트 실행 시 joint states 타임아웃

`AI_Worker_Final/humanoid_challenge/docker/docker-compose.yml` 수정 후 `./container.sh start`:
```yaml
# shm_size: "1gb"  ← 제거
ipc: host           ← 추가
volumes:
  - /dev/shm:/dev/shm  ← 추가 (기존 volumes 아래에)
```
