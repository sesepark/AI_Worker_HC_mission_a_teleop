# ROS2 크로스 컨테이너/머신 통신 문제 정리

## 증상

- `ros2 node list` / `ros2 topic list` 는 정상 (양쪽 노드 보임)
- 실제 데이터 수신 안 됨 — 테스트 실행 시 `joint_states` 타임아웃 발생

## 원인

FastDDS(ROS2 기본 DDS)는 같은 머신의 프로세스끼리 UDP 대신 **공유 메모리(SHM, /dev/shm)** 로 통신함.
두 컨테이너의 `/dev/shm`이 분리되어 있으면 discovery(node/topic 목록)는 UDP 멀티캐스트로 되지만 실제 데이터는 SHM으로 가서 수신 불가.

현재 컨테이너별 SHM 설정:

| 컨테이너 | docker-compose 설정 | 결과 |
|---|---|---|
| `ai_worker` | `ipc: host` + `/dev/shm:/dev/shm` 마운트 | 호스트 SHM 공유 |
| `humanoid_challenge` | `shm_size: "1gb"` | 독립 SHM — 데이터 못 받음 |

---

## 시나리오별 해결

### A. 같은 머신, 두 컨테이너 (로컬/노트북 개발)

`AI_Worker_Final/humanoid_challenge/docker/docker-compose.yml` 수정 후 `./container.sh start`:

```yaml
# 제거
# shm_size: "1gb"

# 추가
ipc: host

volumes:
  - /dev/shm:/dev/shm   # 기존 volumes 아래에 추가
```

> 팀 공용 파일이므로 push 전 팀 공유 필요

---

### B. 다른 머신 (로봇 컴퓨터 + 내 노트북)

SHM은 물리적으로 불가 — FastDDS를 UDP 전용으로 강제.

**1) FastDDS XML 파일 생성** (양쪽 머신 모두):

```xml
<!-- fastdds_udp.xml -->
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
```

**2) 양쪽 docker-compose에 추가**:

```yaml
environment:
  - FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml
volumes:
  - /path/to/fastdds_udp.xml:/fastdds_udp.xml
```

**3) 공유기가 멀티캐스트 차단 시** 상대 IP 직접 지정:

```yaml
environment:
  - ROS_STATIC_PEERS=192.168.0.XX   # 상대방 IP
```

> systems 팀이 크로스 머신 통신 세팅해뒀을 수 있으므로 먼저 확인

---

## 확인 방법

```bash
# humanoid_challenge 컨테이너에서 joint_states 수신 확인
docker exec humanoid_challenge bash -c "
  source /opt/ros/jazzy/setup.bash &&
  timeout 3 ros2 topic echo /joint_states \
    --qos-reliability reliable \
    --qos-durability transient_local --once"
```

데이터가 출력되면 통신 정상.
