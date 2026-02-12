#!/usr/bin/env python3
import argparse
import os
import sys
from typing import Iterable, Tuple


def fail(msg: str, code: int = 1) -> int:
    print(msg, file=sys.stderr)
    return code


def unwrap_number(v):
    if hasattr(v, "value"):
        return v.value
    return v


def format_pose(pose) -> str:
    # Works with bosdyn.math_helpers.SE3Pose-like objects.
    if pose is None:
        return "unavailable"

    x = getattr(pose, "x", None)
    y = getattr(pose, "y", None)
    z = getattr(pose, "z", None)

    rot = getattr(pose, "rot", None)
    qw = getattr(rot, "w", None) if rot is not None else None
    qx = getattr(rot, "x", None) if rot is not None else None
    qy = getattr(rot, "y", None) if rot is not None else None
    qz = getattr(rot, "z", None) if rot is not None else None

    if None in (x, y, z, qw, qx, qy, qz):
        return str(pose)

    return (
        f"pos=({x:.4f}, {y:.4f}, {z:.4f}) "
        f"quat=({qw:.5f}, {qx:.5f}, {qy:.5f}, {qz:.5f})"
    )


def pick_joint_states(joint_states: Iterable) -> Tuple[list, list]:
    armish = []
    other = []
    for js in joint_states:
        name = getattr(js, "name", "")
        ln = name.lower()
        if (
            "arm" in ln
            or "shoulder" in ln
            or "elbow" in ln
            or "wrist" in ln
            or "gripper" in ln
            or "wr" in ln
            or "f1x" in ln
        ):
            armish.append(js)
        else:
            other.append(js)
    return armish, other


def print_joint_states(joint_states: Iterable) -> None:
    arm_joints, _ = pick_joint_states(joint_states)
    to_print = arm_joints if arm_joints else list(joint_states)

    print("Arm joint states:")
    for js in to_print:
        name = getattr(js, "name", "<unknown>")
        pos = unwrap_number(getattr(js, "position", float("nan")))
        vel = unwrap_number(getattr(js, "velocity", float("nan")))
        load = unwrap_number(getattr(js, "load", float("nan")))
        print(f"  - {name}: pos={pos:.6f}, vel={vel:.6f}, load={load:.6f}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Print Spot arm state (joints + hand pose).")
    parser.add_argument("--host", default=os.getenv("SPOT_HOST", ""), help="Spot hostname/IP")
    parser.add_argument("--username", default=os.getenv("SPOT_USERNAME", ""), help="Spot username")
    parser.add_argument("--password", default=os.getenv("SPOT_PASSWORD", ""), help="Spot password")
    parser.add_argument(
        "--base-frame",
        default=os.getenv("BASE_FRAME", "body"),
        help="Base frame for hand pose (default: body)",
    )
    args = parser.parse_args()

    if not args.host:
        return fail("Missing Spot host. Set SPOT_HOST or pass --host.")
    if not args.username or not args.password:
        return fail("Missing credentials. Set SPOT_USERNAME/SPOT_PASSWORD or pass flags.")

    try:
        import bosdyn.client
        import bosdyn.client.util
        from bosdyn.client.robot_state import RobotStateClient
        from bosdyn.client.frame_helpers import get_a_tform_b
    except Exception as e:
        return fail(
            "Failed to import Spot Python SDK (bosdyn). "
            "Install the Spot Python client in this environment.\n"
            f"Import error: {e}"
        )

    try:
        sdk = bosdyn.client.create_standard_sdk("formant-spot-adapter-arm-state")
        robot = sdk.create_robot(args.host)
        robot.authenticate(args.username, args.password)

        # Best effort sync; robot state queries generally work without explicit timesync.
        try:
            robot.time_sync.wait_for_sync(timeout_sec=3.0)
        except Exception:
            pass

        state_client = robot.ensure_client(RobotStateClient.default_service_name)
        robot_state = state_client.get_robot_state()

        print(f"Connected to Spot at {args.host}")
        print_joint_states(robot_state.kinematic_state.joint_states)

        snapshot = robot_state.kinematic_state.transforms_snapshot
        hand_frame_candidates = ["hand", "gripper", "arm0.link_wr1", "link_wr1"]

        pose = None
        selected_hand_frame = None
        for hand_frame in hand_frame_candidates:
            try:
                pose = get_a_tform_b(snapshot, args.base_frame, hand_frame)
                if pose is not None:
                    selected_hand_frame = hand_frame
                    break
            except Exception:
                continue

        print()
        if pose is None:
            print(f"Hand pose in '{args.base_frame}' frame: unavailable")
            print(f"Tried hand frames: {', '.join(hand_frame_candidates)}")
        else:
            print(f"Hand pose in '{args.base_frame}' frame (using '{selected_hand_frame}'):")
            print(f"  {format_pose(pose)}")

        return 0
    except Exception as e:
        return fail(f"Failed to query robot arm state: {e}")


if __name__ == "__main__":
    raise SystemExit(main())
