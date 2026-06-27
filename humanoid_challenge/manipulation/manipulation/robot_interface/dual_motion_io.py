from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from manipulation.robot_interface.moveit_dual_client import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    LIFT_JOINT_NAME,
    duration_from_seconds,
)


DEFAULT_MOTION_DIR_ENV = "DUAL_MOTION_DIR"
DEFAULT_PICK_MOTION_FILE_ENV = "DUAL_PICK_MOTION_FILE"
DEFAULT_PICK_MOTION_FILENAME = "test_dual_pick_motion.json"
DEFAULT_PICK_MOTION_PREFIX = "test_dual_pick_motion"
SEGMENT_Z0_TO_WP1 = "z0_to_wp1"
SEGMENT_LIFT_DOWN_TO_WP2 = "lift_down_to_wp2"


@dataclass(frozen=True)
class JointSample:
    time_sec: float
    positions: dict[str, float]


@dataclass(frozen=True)
class MotionRun:
    start_index: int
    end_index: int

    def duration_sec(self, samples: list[JointSample]) -> float:
        return float(samples[self.end_index].time_sec - samples[self.start_index].time_sec)


def default_motion_dir() -> Path:
    if os.environ.get(DEFAULT_MOTION_DIR_ENV):
        return Path(os.environ[DEFAULT_MOTION_DIR_ENV]).expanduser()
    return Path.cwd() / "dual_motion_records"


def module_motion_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "dual_motion_records"


def motion_search_dirs() -> list[Path]:
    dirs = [default_motion_dir(), module_motion_dir()]
    unique: list[Path] = []
    seen: set[Path] = set()
    for path in dirs:
        resolved = path.expanduser().resolve()
        if resolved not in seen:
            unique.append(resolved)
            seen.add(resolved)
    return unique


def default_pick_motion_path() -> Path:
    if os.environ.get(DEFAULT_PICK_MOTION_FILE_ENV):
        return Path(os.environ[DEFAULT_PICK_MOTION_FILE_ENV]).expanduser()
    return default_motion_dir() / DEFAULT_PICK_MOTION_FILENAME


def unique_pick_motion_path() -> Path:
    motion_dir = default_motion_dir()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = motion_dir / f"{DEFAULT_PICK_MOTION_PREFIX}_{stamp}.json"
    if not candidate.exists():
        return candidate

    for index in range(1, 1000):
        candidate = motion_dir / f"{DEFAULT_PICK_MOTION_PREFIX}_{stamp}_{index:03d}.json"
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"could not allocate unique motion filename in {motion_dir}")


def latest_pick_motion_path() -> Path:
    if os.environ.get(DEFAULT_PICK_MOTION_FILE_ENV):
        return Path(os.environ[DEFAULT_PICK_MOTION_FILE_ENV]).expanduser()

    candidates = sorted(
        (
            path
            for motion_dir in motion_search_dirs()
            for path in motion_dir.glob(f"{DEFAULT_PICK_MOTION_PREFIX}*.json")
        ),
        key=lambda path: path.stat().st_mtime,
    )
    if not candidates:
        searched = ", ".join(str(path) for path in motion_search_dirs())
        raise FileNotFoundError(
            f"no saved motion files found in: {searched}; pass --motion-file explicitly"
        )
    return candidates[-1]


def resolve_pick_motion_path(path: Path) -> Path:
    candidate = path.expanduser()
    if candidate.is_absolute() or candidate.exists():
        return candidate

    if candidate.parent.name == "dual_motion_records":
        relative_candidates = [motion_dir / candidate.name for motion_dir in motion_search_dirs()]
    else:
        relative_candidates = [motion_dir / candidate for motion_dir in motion_search_dirs()]

    for relative_candidate in relative_candidates:
        if relative_candidate.exists():
            return relative_candidate

    return candidate


def duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def trajectory_to_dict(traj: JointTrajectory) -> dict[str, Any]:
    return {
        "joint_names": list(traj.joint_names),
        "points": [
            {
                "time_from_start_sec": duration_to_seconds(point.time_from_start),
                "positions": [float(v) for v in point.positions],
                "velocities": [float(v) for v in point.velocities],
                "accelerations": [float(v) for v in point.accelerations],
            }
            for point in traj.points
        ],
    }


def trajectory_from_dict(data: dict[str, Any]) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = [str(name) for name in data["joint_names"]]

    for point_data in data["points"]:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in point_data["positions"]]
        point.velocities = [float(v) for v in point_data.get("velocities", [])]
        point.accelerations = [float(v) for v in point_data.get("accelerations", [])]
        point.time_from_start = duration_from_seconds(float(point_data["time_from_start_sec"]))
        traj.points.append(point)

    return traj


def write_motion_file(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")


def load_motion_file(path: Path) -> dict[str, Any]:
    with path.expanduser().open("r", encoding="utf-8") as f:
        return json.load(f)


def get_segment(payload: dict[str, Any], name: str) -> dict[str, Any]:
    for segment in payload.get("segments", []):
        if segment.get("name") == name:
            return segment
    names = [segment.get("name", "<unnamed>") for segment in payload.get("segments", [])]
    raise KeyError(f"motion segment {name!r} not found; available={names}")


def get_segment_trajectories(
    payload: dict[str, Any],
    name: str,
) -> tuple[JointTrajectory, JointTrajectory]:
    segment = get_segment(payload, name)
    return trajectory_from_dict(segment["left"]), trajectory_from_dict(segment["right"])


def max_joint_delta(a: JointSample, b: JointSample, joint_names: list[str]) -> float:
    return max(abs(float(b.positions[name]) - float(a.positions[name])) for name in joint_names)


def find_motion_runs(
    samples: list[JointSample],
    joint_names: list[str],
    *,
    active_delta: float,
    merge_idle_sec: float,
    min_duration_sec: float,
) -> list[MotionRun]:
    if len(samples) < 2:
        return []

    runs: list[MotionRun] = []
    run_start: int | None = None
    last_active: int | None = None

    for index in range(1, len(samples)):
        active = max_joint_delta(samples[index - 1], samples[index], joint_names) > active_delta
        if active:
            if run_start is None:
                run_start = index - 1
            last_active = index
            continue

        if run_start is None or last_active is None:
            continue

        idle_sec = float(samples[index].time_sec - samples[last_active].time_sec)
        if idle_sec >= merge_idle_sec:
            run = MotionRun(start_index=run_start, end_index=last_active)
            if run.duration_sec(samples) >= min_duration_sec:
                runs.append(run)
            run_start = None
            last_active = None

    if run_start is not None and last_active is not None:
        run = MotionRun(start_index=run_start, end_index=last_active)
        if run.duration_sec(samples) >= min_duration_sec:
            runs.append(run)

    return runs


def samples_to_trajectory(
    samples: list[JointSample],
    joint_names: list[str],
    *,
    min_point_dt_sec: float,
) -> JointTrajectory:
    if len(samples) < 2:
        raise ValueError("at least two samples are required to build a trajectory")

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    start_time = float(samples[0].time_sec)
    last_added_time = -1.0
    last_relative_time = -1.0

    for index, sample in enumerate(samples):
        is_first = index == 0
        is_last = index == len(samples) - 1
        elapsed_since_added = float(sample.time_sec - last_added_time)
        if not (is_first or is_last or elapsed_since_added >= min_point_dt_sec):
            continue

        point = JointTrajectoryPoint()
        point.positions = [float(sample.positions[name]) for name in joint_names]
        relative_time = max(0.0, float(sample.time_sec - start_time))
        if relative_time <= last_relative_time:
            relative_time = last_relative_time + 0.001
        point.time_from_start = duration_from_seconds(relative_time)
        traj.points.append(point)
        last_added_time = float(sample.time_sec)
        last_relative_time = relative_time

    return traj


def run_samples(samples: list[JointSample], run: MotionRun) -> list[JointSample]:
    return samples[run.start_index : run.end_index + 1]


def build_dual_arm_segment(
    *,
    name: str,
    description: str,
    samples: list[JointSample],
    run: MotionRun,
    min_point_dt_sec: float,
) -> dict[str, Any]:
    selected = run_samples(samples, run)
    left_traj = samples_to_trajectory(
        selected,
        ARM_L_JOINTS,
        min_point_dt_sec=min_point_dt_sec,
    )
    right_traj = samples_to_trajectory(
        selected,
        ARM_R_JOINTS,
        min_point_dt_sec=min_point_dt_sec,
    )
    return {
        "name": name,
        "description": description,
        "source_sample_range": [run.start_index, run.end_index],
        "duration_sec": run.duration_sec(samples),
        "lift_position_start": float(selected[0].positions[LIFT_JOINT_NAME]),
        "lift_position_end": float(selected[-1].positions[LIFT_JOINT_NAME]),
        "left": trajectory_to_dict(left_traj),
        "right": trajectory_to_dict(right_traj),
    }


def max_trajectory_start_error(
    actual_positions: list[float],
    expected_traj: JointTrajectory,
) -> float:
    if not expected_traj.points:
        raise ValueError("expected trajectory is empty")
    expected = list(expected_traj.points[0].positions)
    if len(actual_positions) != len(expected):
        raise ValueError(
            f"position length mismatch: actual={len(actual_positions)}, expected={len(expected)}"
        )
    return max(abs(float(a) - float(b)) for a, b in zip(actual_positions, expected))
