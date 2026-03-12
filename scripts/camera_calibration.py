#!/usr/bin/env python3
import argparse
import os
import sys
import time
from typing import Iterable, List


def fail(msg: str, code: int = 1) -> int:
    print(msg, file=sys.stderr)
    return code


def try_import_sdk():
    try:
        import bosdyn.client
        from bosdyn.api import robot_state_pb2
        from bosdyn.api.spot import spot_check_pb2
        from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
        from bosdyn.client.robot_state import RobotStateClient
        from bosdyn.client.spot_check import SpotCheckClient
    except Exception as exc:
        raise RuntimeError(
            "Failed to import the Spot Python SDK (bosdyn). "
            "Install the Spot SDK in this environment."
        ) from exc

    return {
        "bosdyn": bosdyn.client,
        "robot_state_pb2": robot_state_pb2,
        "spot_check_pb2": spot_check_pb2,
        "LeaseClient": LeaseClient,
        "LeaseKeepAlive": LeaseKeepAlive,
        "ResourceAlreadyClaimedError": ResourceAlreadyClaimedError,
        "RobotStateClient": RobotStateClient,
        "SpotCheckClient": SpotCheckClient,
    }


def enum_name(msg, field_name: str, value: int) -> str:
    field = msg.DESCRIPTOR.fields_by_name[field_name]
    return field.enum_type.values_by_number.get(value, None).name if value in field.enum_type.values_by_number else str(value)


def connect_robot(args, sdk_mod):
    sdk = sdk_mod["bosdyn"].create_standard_sdk("formant-spot-adapter-camera-calibration")
    robot = sdk.create_robot(args.host)
    robot.authenticate(args.username, args.password)
    robot.sync_with_directory()
    robot.time_sync.wait_for_sync(timeout_sec=10.0)
    return robot


def get_robot_state(robot, sdk_mod):
    state_client = robot.ensure_client(sdk_mod["RobotStateClient"].default_service_name)
    return state_client.get_robot_state()


def summarize_faults(faults: Iterable, severity_name_fn) -> List[str]:
    out = []
    for fault in faults:
        severity = severity_name_fn(fault.severity)
        name = fault.name if getattr(fault, "name", "") else getattr(fault, "uuid", "fault")
        out.append(f"{severity}: {name}: {fault.error_message}")
    return out


def print_state_summary(state, sdk_mod) -> None:
    robot_state_pb2 = sdk_mod["robot_state_pb2"]
    motor_power = robot_state_pb2.PowerState.MotorPowerState.Name(state.power_state.motor_power_state)
    shore_power = robot_state_pb2.PowerState.ShorePowerState.Name(state.power_state.shore_power_state)
    robot_power = robot_state_pb2.PowerState.RobotPowerState.Name(state.power_state.robot_power_state)
    behavior = robot_state_pb2.BehaviorState.State.Name(state.behavior_state.state)

    print("Robot state:")
    print(f"  motor_power_state: {motor_power}")
    print(f"  shore_power_state: {shore_power}")
    print(f"  robot_power_state: {robot_power}")
    print(f"  behavior_state: {behavior}")
    print("  estops:")
    for estop in state.estop_states:
        estop_name = sdk_mod["robot_state_pb2"].EStopState.State.Name(estop.state)
        print(f"    - {estop.name}: {estop_name}")

    system_faults = summarize_faults(
        state.system_fault_state.faults,
        sdk_mod["robot_state_pb2"].SystemFault.Severity.Name,
    )
    if system_faults:
        print("  active_system_faults:")
        for fault in system_faults:
            print(f"    - {fault}")
    else:
        print("  active_system_faults: none")


def camera_init_faults(state) -> List[str]:
    patterns = (
        "failed to initialize",
        "enumerated usb2",
        "not sending",
        "module stopped responding",
    )
    hits = []
    for fault in state.system_fault_state.faults:
        if getattr(fault, "name", "") != "camera_server":
            continue
        message = fault.error_message.lower()
        if any(pattern in message for pattern in patterns):
            hits.append(fault.error_message)
    return hits


def is_estopped(state, sdk_mod) -> bool:
    estopped = sdk_mod["robot_state_pb2"].EStopState.STATE_ESTOPPED
    return any(estop.state == estopped for estop in state.estop_states)


def calibration_feedback(spot_check_client, sdk_mod):
    req = sdk_mod["spot_check_pb2"].CameraCalibrationFeedbackRequest()
    return spot_check_client.call(
        spot_check_client._stub.CameraCalibrationFeedback,  # pylint: disable=protected-access
        req,
        error_from_response=None,
    )


def ensure_body_lease(robot, sdk_mod) -> str:
    lease_client = robot.ensure_client(sdk_mod["LeaseClient"].default_service_name)
    try:
        lease_client.acquire("body")
        return "acquired"
    except sdk_mod["ResourceAlreadyClaimedError"]:
        lease_client.take("body")
        return "took_over"


def print_feedback(response, sdk_mod) -> None:
    status_name = sdk_mod["spot_check_pb2"].CameraCalibrationFeedbackResponse.Status.Name(response.status)
    print(f"Calibration feedback: status={status_name} progress={response.progress:.3f}")


def preflight_start(state, sdk_mod) -> List[str]:
    robot_state_pb2 = sdk_mod["robot_state_pb2"]
    problems = []

    if is_estopped(state, sdk_mod):
        problems.append("an E-Stop is asserted")

    if state.power_state.motor_power_state != robot_state_pb2.PowerState.STATE_ON:
        problems.append(
            "motor power is not on; camera calibration expects the robot powered on"
        )

    if state.behavior_state.state != robot_state_pb2.BehaviorState.STATE_STANDING:
        problems.append(
            "robot is not standing; SDK guidance expects camera calibration to start from a standing robot"
        )

    if state.power_state.shore_power_state == robot_state_pb2.PowerState.STATE_ON_SHORE_POWER:
        problems.append(
            "robot is on shore power/docked; calibration may try to move around a target and should not start from the dock"
        )

    init_faults = camera_init_faults(state)
    if init_faults:
        joined = "; ".join(init_faults)
        problems.append(
            "camera_server has active init faults that calibration is unlikely to repair remotely: "
            + joined
        )

    return problems


def run_status(robot, sdk_mod) -> int:
    state = get_robot_state(robot, sdk_mod)
    print_state_summary(state, sdk_mod)

    spot_check_client = robot.ensure_client(sdk_mod["SpotCheckClient"].default_service_name)
    try:
        feedback = calibration_feedback(spot_check_client, sdk_mod)
    except Exception as exc:
        print(f"Calibration feedback unavailable: {exc}")
        return 0

    print_feedback(feedback, sdk_mod)
    return 0


def run_cancel(robot, sdk_mod) -> int:
    spot_check_pb2 = sdk_mod["spot_check_pb2"]
    lease_mode = ensure_body_lease(robot, sdk_mod)
    lease_client = robot.ensure_client(sdk_mod["LeaseClient"].default_service_name)
    spot_check_client = robot.ensure_client(sdk_mod["SpotCheckClient"].default_service_name)

    print(f"Body lease {lease_mode.replace('_', ' ')} for cancel request.")
    with sdk_mod["LeaseKeepAlive"](lease_client, must_acquire=False, return_at_exit=True):
        lease = lease_client.lease_wallet.get_lease("body")
        req = spot_check_pb2.CameraCalibrationCommandRequest()
        req.command = spot_check_pb2.CameraCalibrationCommandRequest.COMMAND_CANCEL
        req.lease.CopyFrom(lease.lease_proto)
        spot_check_client.camera_calibration_command(req)
        feedback = calibration_feedback(spot_check_client, sdk_mod)
        print_feedback(feedback, sdk_mod)
    return 0


def run_start(robot, sdk_mod, args) -> int:
    spot_check_pb2 = sdk_mod["spot_check_pb2"]
    state = get_robot_state(robot, sdk_mod)
    print_state_summary(state, sdk_mod)

    problems = preflight_start(state, sdk_mod)
    if problems and not args.force:
        print()
        print("Refusing to start camera calibration because:")
        for problem in problems:
            print(f"  - {problem}")
        print()
        print("Re-run with --force to bypass this preflight.")
        return 2

    lease_mode = ensure_body_lease(robot, sdk_mod)
    lease_client = robot.ensure_client(sdk_mod["LeaseClient"].default_service_name)
    spot_check_client = robot.ensure_client(sdk_mod["SpotCheckClient"].default_service_name)
    print()
    print(f"Body lease {lease_mode.replace('_', ' ')} for camera calibration.")

    with sdk_mod["LeaseKeepAlive"](lease_client, must_acquire=False, return_at_exit=True):
        lease = lease_client.lease_wallet.get_lease("body")
        req = spot_check_pb2.CameraCalibrationCommandRequest()
        req.command = spot_check_pb2.CameraCalibrationCommandRequest.COMMAND_START
        req.lease.CopyFrom(lease.lease_proto)
        spot_check_client.camera_calibration_command(req)

        deadline = time.time() + args.timeout_sec
        last_status = None
        while time.time() < deadline:
            feedback = calibration_feedback(spot_check_client, sdk_mod)
            status_name = spot_check_pb2.CameraCalibrationFeedbackResponse.Status.Name(feedback.status)
            line = f"Calibration feedback: status={status_name} progress={feedback.progress:.3f}"
            if status_name != last_status or feedback.status != spot_check_pb2.CameraCalibrationFeedbackResponse.STATUS_PROCESSING:
                print(line)
                last_status = status_name

            if feedback.status == spot_check_pb2.CameraCalibrationFeedbackResponse.STATUS_PROCESSING:
                time.sleep(args.poll_sec)
                continue
            if feedback.status == spot_check_pb2.CameraCalibrationFeedbackResponse.STATUS_SUCCESS:
                return 0
            return 3

    return fail(
        f"Timed out waiting for camera calibration after {args.timeout_sec} seconds.",
        code=4,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run or inspect Spot camera calibration using the official SpotCheck service."
    )
    parser.add_argument("--host", default=os.getenv("SPOT_HOST", ""), help="Spot hostname/IP")
    parser.add_argument("--username", default=os.getenv("SPOT_USERNAME", ""), help="Spot username")
    parser.add_argument("--password", default=os.getenv("SPOT_PASSWORD", ""), help="Spot password")

    subparsers = parser.add_subparsers(dest="action", required=True)

    subparsers.add_parser("status", help="Show robot state and camera calibration feedback.")

    start = subparsers.add_parser("start", help="Start camera calibration and wait for completion.")
    start.add_argument(
        "--timeout-sec",
        type=int,
        default=1200,
        help="Maximum seconds to wait for completion (default: 1200).",
    )
    start.add_argument(
        "--poll-sec",
        type=float,
        default=2.0,
        help="Polling interval in seconds while calibration runs (default: 2.0).",
    )
    start.add_argument(
        "--force",
        action="store_true",
        help="Bypass preflight checks and issue the start command anyway.",
    )

    subparsers.add_parser("cancel", help="Cancel a running camera calibration.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if not args.host:
        return fail("Missing Spot host. Set SPOT_HOST or pass --host.")
    if not args.username or not args.password:
        return fail("Missing Spot credentials. Set SPOT_USERNAME/SPOT_PASSWORD or pass flags.")

    try:
        sdk_mod = try_import_sdk()
        robot = connect_robot(args, sdk_mod)
        if args.action == "status":
            return run_status(robot, sdk_mod)
        if args.action == "cancel":
            return run_cancel(robot, sdk_mod)
        if args.action == "start":
            return run_start(robot, sdk_mod, args)
        return fail(f"Unsupported action: {args.action}")
    except Exception as exc:
        return fail(f"Camera calibration utility failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
