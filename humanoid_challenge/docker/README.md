# humanoid_challenge Docker

This Docker setup runs the local `humanoid_challenge` ROS 2 packages on top of
`ros:jazzy-ros-base`.

## Usage

```bash
cd humanoid_challenge/docker
./container.sh start
./container.sh enter
./container.sh stop
```

`start` builds the Docker image only when the configured image tag is missing,
starts the container, builds the mounted GPD source, then runs
`colcon build --symlink-install` for the main-PC ROS packages:
`perception_part_detector`, `monitor_ocr`, `perception_2d_to_pcd`,
`perception_2d_to_pcd_wrist`, `task_management`, and `mission`.

Use `./container.sh build` to force a local image rebuild after changing the
Dockerfile or image-level dependencies. To run a prebuilt image from a
registry, set `HUMANOID_CHALLENGE_IMAGE`, for example:

```bash
HUMANOID_CHALLENGE_IMAGE=your-dockerhub-id/humanoid_challenge:jazzy ./container.sh pull
HUMANOID_CHALLENGE_IMAGE=your-dockerhub-id/humanoid_challenge:jazzy ./container.sh start
```

`ai_worker_manipulation` is not built by default because several of its console
scripts can command MoveIt/gripper controllers over ROS 2. Build it explicitly
with:

```bash
./container.sh colcon-all
```

The local source tree is mounted at:

```text
/ws/src/humanoid_challenge
```

The image does not copy or build the vendored GPD source. `container.sh` builds
the currently mounted source from `humanoid_challenge/gpd` into:

```text
/ws/src/humanoid_challenge/gpd/build
```

`GPD_DIR=/ws/src/humanoid_challenge/gpd` is exported in the container. The GPD
wrapper runs `detect_grasps` from the `build/` directory so the upstream
`../cfg` and `../models` paths in `cfg/eigen_params.cfg` resolve correctly.
After changing GPD source, run `./container.sh gpd` or `./container.sh start`;
the CMake build is incremental and does not require rebuilding the Docker image.

This container does not mount `/dev` and does not run privileged. The ROS nodes
are intended to consume topics published by the robot over the ROS 2 network,
not to open local camera/USB devices on the main PC.

## Installed In The Image

Base image:

- `ros:jazzy-ros-base`

APT packages:

- Build/dev tools: `build-essential`, `cmake`, `git`, `curl`, `wget`, `vim`,
  `nano`, `sudo`, `python3-dev`, `python3-pip`, `python3-venv`
- ROS build tools: `python3-colcon-common-extensions`, `python3-rosdep`,
  `python3-vcstool`, `python3-argcomplete`
- ROS package dependencies: `ament-cmake`, `rosidl_default_generators`,
  `rosidl_default_runtime`, `cv_bridge`, `message_filters`, `tf2_ros`,
  `tf2_geometry_msgs`, `tf2_sensor_msgs`, `sensor_msgs`, `geometry_msgs`,
  `std_msgs`, `trajectory_msgs`, `moveit_msgs`, `pymoveit2`
- GPD local build/runtime dependencies: `libpcl-dev`, `libeigen3-dev`,
  `libopencv-dev`, `libboost-dev`
- Debug/GUI helpers: `rqt_image_view`, `mesa-utils`, `libgl1`, `libglib2.0-0`,
  `libgomp1`, `libsm6`, `libxext6`, `libxrender1`
- Python numeric/CV packages from apt: `python3-numpy`, `python3-scipy`,
  `python3-opencv`

Python packages:

- System Python: `numpy<2`, `scipy`, `psutil>=7`, `open3d==0.19.0`
- `/ws/yolo_venv`: `numpy<2`, `opencv-python`, `ultralytics`
- `/ws/ocr_venv`: `numpy<2`, `opencv-python`, `ultralytics`,
  `paddlepaddle==3.0.0`, `paddleocr`
- Both venvs install CPU-only `torch` and `torchvision` from
  `https://download.pytorch.org/whl/cpu` before installing `ultralytics`.

`open3d` is installed in system Python because `ai_worker_manipulation`'s
`gpd_dual_view` path imports it directly.

Vendored/runtime source:

- `humanoid_challenge/gpd` is mounted from the local checkout and built by
  `container.sh`, not baked into the image.
- The vendored source was copied from the original `ai-worker-ws` `gpd`
  submodule, commit `6327f20eabfcba41a05fdd2e2ba408153dc2e958`.
- `pymoveit2` is not vendored here because Jazzy provides it as
  `ros-jazzy-pymoveit2`, and `ai_worker_manipulation/package.xml` already
  declares `<depend>pymoveit2</depend>`.

The venv paths match the existing launch files and detector shebangs.

## Hardware Access Audit

- Perception packages subscribe to robot-published image/depth/camera info/TF
  topics such as `/zed/...` and `/camera_*`; they do not open local ZED,
  RealSense, USB, or serial devices.
- `monitor_ocr/test_realtime.py` has a local `cv2.VideoCapture(...)` path for
  webcam testing only. It is not used by the ROS launch path.
- `ai_worker_manipulation` contains MoveIt2 and gripper controller clients. It
  does not use `/dev` directly, but running its console scripts can publish
  robot controller commands over ROS 2 if the robot graph is reachable.

## Model Files

The image does not include model weights. Put them in the local source tree:

```text
humanoid_challenge/perception_part_detector/weights/best.pt
humanoid_challenge/monitor_ocr/best.pt
```

The container script creates the `weights/` directory if it is missing, so the
ROS packages can still build. Runtime YOLO/OCR startup still needs the actual
`.pt` files.
