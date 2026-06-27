from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from manipulation.robot_interface.moveit_dual_client import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    LIFT_JOINT_NAME,
)
from manipulation.robot_interface.dual_motion_io import (
    JointSample,
    SEGMENT_LIFT_DOWN_TO_WP2,
    SEGMENT_Z0_TO_WP1,
    build_dual_arm_segment,
    find_motion_runs,
    max_joint_delta,
    unique_pick_motion_path,
    write_motion_file,
)


ARM_MOTION_JOINTS = ARM_L_JOINTS + ARM_R_JOINTS
WATCH_JOINTS = ARM_MOTION_JOINTS + [LIFT_JOINT_NAME]

DEFAULT_ARM_ACTIVE_DELTA = 0.0002
DEFAULT_LIFT_ACTIVE_DELTA = 0.0002
DEFAULT_MERGE_IDLE_SEC = 0.35
DEFAULT_MIN_RUN_DURATION_SEC = 0.30
DEFAULT_STOP_IDLE_SEC = 2.0
DEFAULT_MAX_DURATION_SEC = 120.0
DEFAULT_MIN_POINT_DT_SEC = 0.04


class JointMotionRecorder(Node):
    def __init__(
        self,
        *,
        arm_active_delta: float,
        lift_active_delta: float,
    ) -> None:
        super().__init__("test_dual_motion_save")
        self.arm_active_delta = float(arm_active_delta)
        self.lift_active_delta = float(lift_active_delta)
        self.samples: list[JointSample] = []
        self.started = False
        self.last_motion_time_sec: float | None = None
        self._last_sample: JointSample | None = None
        self._missing_joints_logged = False
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 100)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_position = dict(zip(msg.name, msg.position))
        missing = [name for name in WATCH_JOINTS if name not in name_to_position]
        if missing:
            if not self._missing_joints_logged:
                self.get_logger().warn(f"waiting for joints in /joint_states: {missing}")
                self._missing_joints_logged = True
            return

        sample = JointSample(
            time_sec=self.now_sec(),
            positions={name: float(name_to_position[name]) for name in WATCH_JOINTS},
        )

        if self._last_sample is None:
            self._last_sample = sample
            return

        arm_delta = max_joint_delta(self._last_sample, sample, ARM_MOTION_JOINTS)
        lift_delta = max_joint_delta(self._last_sample, sample, [LIFT_JOINT_NAME])
        moving = arm_delta > self.arm_active_delta or lift_delta > self.lift_active_delta

        if not self.started and moving:
            self.started = True
            self.samples.append(self._last_sample)
            self.get_logger().info("motion detected; recording started")

        if self.started:
            self.samples.append(sample)

        if moving:
            self.last_motion_time_sec = sample.time_sec

        self._last_sample = sample


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description=(
            "Record dual-arm joint motion from /joint_states and save the two "
            "arm segments produced while test_dual_pick is running."
        )
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output JSON file. If omitted, creates a new timestamped file under "
            "dual_motion_records for each run."
        ),
    )
    parser.add_argument(
        "--expected-arm-runs",
        type=int,
        default=2,
        help="Stop after this many arm-motion runs have been recorded.",
    )
    parser.add_argument(
        "--expected-lift-runs",
        type=int,
        default=2,
        help="Stop after this many lift-motion runs have been recorded.",
    )
    parser.add_argument("--max-duration-sec", type=float, default=DEFAULT_MAX_DURATION_SEC)
    parser.add_argument("--stop-idle-sec", type=float, default=DEFAULT_STOP_IDLE_SEC)
    parser.add_argument("--arm-active-delta", type=float, default=DEFAULT_ARM_ACTIVE_DELTA)
    parser.add_argument("--lift-active-delta", type=float, default=DEFAULT_LIFT_ACTIVE_DELTA)
    parser.add_argument("--merge-idle-sec", type=float, default=DEFAULT_MERGE_IDLE_SEC)
    parser.add_argument("--min-run-duration-sec", type=float, default=DEFAULT_MIN_RUN_DURATION_SEC)
    parser.add_argument("--min-point-dt-sec", type=float, default=DEFAULT_MIN_POINT_DT_SEC)
    return parser.parse_known_args(args=args)


def motion_output_path(path: Path | None) -> Path:
    if path is None:
        return unique_pick_motion_path().resolve()

    resolved = path.expanduser()
    if resolved.exists() and resolved.is_dir():
        return (resolved / unique_pick_motion_path().name).resolve()
    return resolved.resolve()


def detect_arm_runs(samples: list[JointSample], ns) -> list:
    return find_motion_runs(
        samples,
        ARM_MOTION_JOINTS,
        active_delta=float(ns.arm_active_delta),
        merge_idle_sec=float(ns.merge_idle_sec),
        min_duration_sec=float(ns.min_run_duration_sec),
    )


def detect_lift_runs(samples: list[JointSample], ns) -> list:
    return find_motion_runs(
        samples,
        [LIFT_JOINT_NAME],
        active_delta=float(ns.lift_active_delta),
        merge_idle_sec=float(ns.merge_idle_sec),
        min_duration_sec=float(ns.min_run_duration_sec),
    )


def should_finish_recording(recorder: JointMotionRecorder, ns) -> bool:
    if not recorder.started or recorder.last_motion_time_sec is None:
        return False

    idle_sec = recorder.now_sec() - recorder.last_motion_time_sec
    if idle_sec < float(ns.stop_idle_sec):
        return False

    arm_runs = detect_arm_runs(recorder.samples, ns)
    lift_runs = detect_lift_runs(recorder.samples, ns)
    return (
        len(arm_runs) >= int(ns.expected_arm_runs)
        and len(lift_runs) >= int(ns.expected_lift_runs)
    )


def build_payload(samples: list[JointSample], ns, *, stopped_by: str) -> dict:
    arm_runs = detect_arm_runs(samples, ns)
    lift_runs = detect_lift_runs(samples, ns)
    if len(arm_runs) < 2:
        raise RuntimeError(
            f"expected at least 2 arm-motion runs, found {len(arm_runs)}; "
            "run test_dual_pick while this recorder is active"
        )

    return {
        "schema_version": 1,
        "source": "test_dual_motion_save",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "stopped_by": stopped_by,
        "watch_joints": WATCH_JOINTS,
        "recording": {
            "sample_count": len(samples),
            "arm_run_count": len(arm_runs),
            "lift_run_count": len(lift_runs),
            "arm_active_delta": float(ns.arm_active_delta),
            "lift_active_delta": float(ns.lift_active_delta),
            "merge_idle_sec": float(ns.merge_idle_sec),
            "min_run_duration_sec": float(ns.min_run_duration_sec),
            "min_point_dt_sec": float(ns.min_point_dt_sec),
        },
        "segments": [
            build_dual_arm_segment(
                name=SEGMENT_Z0_TO_WP1,
                description="Dual-arm motion from lift z=0 start posture to waypoint1.",
                samples=samples,
                run=arm_runs[0],
                min_point_dt_sec=float(ns.min_point_dt_sec),
            ),
            build_dual_arm_segment(
                name=SEGMENT_LIFT_DOWN_TO_WP2,
                description="Dual-arm motion from lift-down posture to waypoint2.",
                samples=samples,
                run=arm_runs[1],
                min_point_dt_sec=float(ns.min_point_dt_sec),
            ),
        ],
    }


def record_motion(ns, ros_args) -> int:
    output_path = motion_output_path(ns.output)
    rclpy.init(args=ros_args)
    recorder = JointMotionRecorder(
        arm_active_delta=float(ns.arm_active_delta),
        lift_active_delta=float(ns.lift_active_delta),
    )
    log = recorder.get_logger()

    exit_code = 1
    stopped_by = "unknown"
    started_at = time.monotonic()

    try:
        log.info(f"Recording /joint_states to {output_path}")
        log.info("Start test_dual_pick in another terminal while this recorder is running.")
        log.info(
            "This node saves after it sees two arm motion segments and two lift motion segments."
        )

        while rclpy.ok():
            rclpy.spin_once(recorder, timeout_sec=0.1)

            if should_finish_recording(recorder, ns):
                stopped_by = "expected_motion_runs_complete"
                log.info("expected arm/lift motion runs recorded; stopping")
                break

            if time.monotonic() - started_at > float(ns.max_duration_sec):
                stopped_by = "max_duration"
                log.warn("max recording duration reached; finalizing with recorded samples")
                break

        if not recorder.samples:
            raise RuntimeError("no motion samples recorded")

        payload = build_payload(recorder.samples, ns, stopped_by=stopped_by)
        write_motion_file(output_path, payload)
        log.info(
            f"saved {len(payload['segments'])} motion segments "
            f"from {payload['recording']['sample_count']} samples"
        )
        exit_code = 0

    except KeyboardInterrupt:
        stopped_by = "keyboard_interrupt"
        if recorder.samples:
            try:
                payload = build_payload(recorder.samples, ns, stopped_by=stopped_by)
            except Exception as exc:
                log.error(f"could not save partial recording: {exc}")
            else:
                write_motion_file(output_path, payload)
                log.info(f"saved partial recording to {output_path}")
                exit_code = 0
        else:
            log.warn("interrupted before any motion samples were recorded")
    except Exception as exc:
        log.error(f"test_dual_motion_save failed: {exc}")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

    return exit_code


def main(args=None) -> None:
    ns, ros_args = parse_args(args=args)
    exit_code = record_motion(ns, ros_args)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
