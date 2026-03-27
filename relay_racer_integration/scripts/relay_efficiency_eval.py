#!/usr/bin/env python3

import argparse
import json
import os
import select
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


WORKSPACE = Path("/home/zby/relay_explore_ws")
MODE_ALIASES = {
    "original_racer": "original_racer",
    "ns3_no_relay": "ns3_no_relay",
    "ns3_rule_relay": "ns3_rule_relay",
    "no_relay": "ns3_no_relay",
    "rule_relay": "ns3_rule_relay",
}
DEFAULT_MODES = ("ns3_no_relay", "ns3_rule_relay")
SUMMARY_KEYS = (
    "report_finalized",
    "success",
    "task_completed",
    "completion_ratio",
    "completion_elapsed_sec",
    "frontier_remaining",
    "frontier_completion_ratio",
    "relay_event_count",
    "post_startup_event_count",
    "relay_total_occupancy_sec",
    "relay_occupancy_cost",
    "mean_information_age",
    "last_num_components",
    "num_components_peak",
    "runtime_sec",
    "team_total_path_length_m",
    "relay_total_path_length_m",
    "relay_path_fraction",
    "completion_ratio_per_meter",
    "frontier_completion_ratio_per_meter",
    "team_low_speed_ratio",
    "planning_artifact_success",
)
ADVANCE_COMMANDS = {"", "n", "next", "continue", "c"}
QUIT_COMMANDS = {"q", "quit", "exit", "stop"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run original RACER reference and ns3 no-relay / rule-relay experiments."
    )
    parser.add_argument(
        "--modes",
        nargs="+",
        choices=tuple(MODE_ALIASES.keys()),
        default=list(DEFAULT_MODES),
        help="Experiment modes to run sequentially. Legacy no_relay / rule_relay names remain accepted.",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/relay_efficiency_eval",
        help="Directory for per-run logs, reports, and summaries.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=260.0,
        help="Max wall time per mode before graceful shutdown.",
    )
    parser.add_argument(
        "--tag",
        default="",
        help="Optional run tag. Defaults to a timestamp.",
    )
    parser.add_argument(
        "--poll-sec",
        type=float,
        default=1.0,
        help="Polling interval for completion and manual control.",
    )
    parser.add_argument(
        "--completion-hold-sec",
        type=float,
        default=0.0,
        help="Require completion to remain true for this long before stopping the current run.",
    )
    parser.add_argument(
        "--disable-completion-check",
        action="store_true",
        help="Disable automatic stop when the monitor reports task_completed=1.",
    )
    parser.add_argument(
        "--manual-advance",
        action="store_true",
        help="Enable operator-controlled advance. While a run is active, press Enter to stop it and continue, or q to stop and quit.",
    )
    parser.add_argument(
        "--manual-next-file",
        default="",
        help="Optional sentinel file path. Creating the file stops the current run and advances to the next mode.",
    )
    parser.add_argument(
        "--no-build-check",
        action="store_true",
        help="Skip the sourced environment sanity check.",
    )
    parser.add_argument(
        "--allow-existing-output",
        action="store_true",
        help="Allow writing into an existing tag directory. Disabled by default to avoid mixed results.",
    )
    parser.add_argument(
        "--mainline-launch",
        default="relay_real_input_4planner_min.launch",
        help="Mainline roslaunch file for ns3 modes, e.g. relay_real_input_4planner_min.launch or relay_real_input_6planner_min.launch.",
    )
    return parser.parse_args()


def parse_report(report_path: Path):
    data = {}
    if not report_path.exists():
        return data
    for line in report_path.read_text(encoding="ascii").splitlines():
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def maybe_number(value):
    if value is None:
        return None
    if value in ("", "NA"):
        return None
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def summarize_report(mode: str, report_path: Path, log_path: Path, exit_state: str, return_code):
    parsed = parse_report(report_path)
    summary = {
        "mode": mode,
        "exit_state": exit_state,
        "return_code": return_code,
        "report_path": str(report_path),
        "log_path": str(log_path),
        "report_exists": report_path.exists(),
    }
    for key in SUMMARY_KEYS:
        summary[key] = maybe_number(parsed.get(key))
    summary["relay_agent_list_csv"] = parsed.get("relay_agent_list_csv", "")
    summary["planning_artifact_agent_list_csv"] = parsed.get("planning_artifact_agent_list_csv", "")
    summary["per_agent_path_length_m_csv"] = parsed.get("per_agent_path_length_m_csv", "")
    summary["per_agent_relay_path_length_m_csv"] = parsed.get("per_agent_relay_path_length_m_csv", "")
    summary["per_agent_low_speed_ratio_csv"] = parsed.get("per_agent_low_speed_ratio_csv", "")
    return summary


def build_roslaunch_args(mode: str, report_path: Path, mainline_launch: str):
    if mode == "original_racer":
        return [
            "roslaunch",
            "relay_racer_integration",
            "racer_original_baseline.launch",
            f"monitor_report_path:={report_path}",
            "monitor_output:=log",
        ]

    common = [
        "roslaunch",
        "relay_racer_integration",
        mainline_launch,
        "use_local_topic_router:=false",
        "distributed_local_relay_controller:=true",
        "relay_policy_mode:=rule",
        "trigger_required:=false",
        "trigger_call_test_service:=false",
        "trigger_wait_for_relay_cmd:=false",
        "trigger_shutdown_on_finish:=false",
        "trigger_require_planner_ready:=true",
        "trigger_republish_without_relay:=true",
        f"monitor_report_path:={report_path}",
        "platform_output:=log",
        "comm_output:=log",
    ]
    if mode == "ns3_no_relay":
        common[5] = "relay_policy_mode:=off"
    elif mode == "ns3_rule_relay":
        common.append("relay_max_occupancy_ratio:=0.5")
    else:
        raise ValueError(f"unknown mode: {mode}")
    return common


def terminate_process_group(proc: subprocess.Popen):
    if proc.poll() is not None:
        return proc.returncode, "exited"

    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return proc.returncode, "exited"

    try:
        proc.wait(timeout=20.0)
        return proc.returncode, "sigint"
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return proc.returncode, "exited"

    try:
        proc.wait(timeout=10.0)
        return proc.returncode, "sigterm"
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return proc.returncode, "exited"
    proc.wait(timeout=5.0)
    return proc.returncode, "sigkill"


def detect_completion(report_path: Path):
    parsed = parse_report(report_path)
    if not parsed:
        return False, "", parsed

    task_completed = maybe_number(parsed.get("task_completed"))
    if task_completed in (1, True):
        return True, "task_completed", parsed
    return False, "", parsed


def poll_manual_command():
    if not sys.stdin.isatty():
        return ""
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not ready:
        return ""
    line = sys.stdin.readline()
    if line == "":
        return ""
    command = line.strip().lower()
    if command in ADVANCE_COMMANDS:
        return "advance"
    if command in QUIT_COMMANDS:
        return "quit"
    print(
        "[relay_efficiency_eval] unknown manual command. Use Enter/n/next to advance, or q to quit.",
        flush=True,
    )
    return ""


def prompt_before_next(current_mode: str, next_mode: str):
    if not sys.stdin.isatty():
        print(
            f"[relay_efficiency_eval] manual_advance requested, but stdin is not interactive. Auto-continuing {current_mode} -> {next_mode}.",
            flush=True,
        )
        return True

    while True:
        response = input(
            f"[relay_efficiency_eval] mode={current_mode} finished. Press Enter to start {next_mode}, or type q to stop: "
        ).strip().lower()
        if response in ADVANCE_COMMANDS:
            return True
        if response in QUIT_COMMANDS:
            return False
        print("[relay_efficiency_eval] unknown response. Use Enter to continue or q to stop.", flush=True)


def run_mode(
    mode: str,
    workspace: Path,
    output_root: Path,
    timeout_sec: float,
    poll_sec: float,
    completion_check_enabled: bool,
    completion_hold_sec: float,
    manual_advance: bool,
    manual_next_file: str,
    mainline_launch: str,
):
    mode_dir = output_root / mode
    mode_dir.mkdir(parents=True, exist_ok=True)
    report_path = mode_dir / "monitor_report.txt"
    summary_path = mode_dir / "summary.json"
    log_path = mode_dir / "roslaunch.log"

    roslaunch_args = build_roslaunch_args(mode, report_path, mainline_launch)
    launch_cmd = " ".join(shlex.quote(arg) for arg in roslaunch_args)
    shell_cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null && "
        f"cd {shlex.quote(str(workspace))} && "
        "source devel/setup.bash >/dev/null && "
        f"{launch_cmd}"
    )

    sentinel_path = Path(manual_next_file).expanduser() if manual_next_file else None
    if sentinel_path is not None and sentinel_path.exists():
        sentinel_path.unlink()

    with log_path.open("w", encoding="ascii", errors="replace") as log_file:
        proc = subprocess.Popen(
            ["bash", "-lc", shell_cmd],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        if manual_advance:
            if sys.stdin.isatty():
                print(
                    "[relay_efficiency_eval] manual control active: press Enter to stop the current run and continue, or q to stop and quit.",
                    flush=True,
                )
            else:
                print(
                    "[relay_efficiency_eval] manual_advance requested, but stdin is not interactive. Only timeout/completion/file-sentinel control is available.",
                    flush=True,
                )
        if sentinel_path is not None:
            print(
                f"[relay_efficiency_eval] manual next sentinel: touch {sentinel_path} to stop the current run and continue.",
                flush=True,
            )

        start_time = time.monotonic()
        exit_state = "timeout"
        return_code = None
        stop_reason = "process_exit"
        completion_detected_at = None

        while True:
            return_code = proc.poll()
            if return_code is not None:
                exit_state = "exited"
                break

            elapsed = time.monotonic() - start_time
            if elapsed >= timeout_sec:
                return_code, exit_state = terminate_process_group(proc)
                stop_reason = "timeout"
                break

            if completion_check_enabled:
                completed, completion_reason, _parsed = detect_completion(report_path)
                if completed:
                    if completion_detected_at is None:
                        completion_detected_at = time.monotonic()
                        print(
                            f"[relay_efficiency_eval] mode={mode} completion detected via {completion_reason}. Waiting {completion_hold_sec:.1f}s before stopping.",
                            flush=True,
                        )
                    if time.monotonic() - completion_detected_at >= completion_hold_sec:
                        return_code, exit_state = terminate_process_group(proc)
                        stop_reason = completion_reason
                        break
                else:
                    completion_detected_at = None

            if sentinel_path is not None and sentinel_path.exists():
                sentinel_path.unlink()
                return_code, exit_state = terminate_process_group(proc)
                stop_reason = "manual_next_file"
                break

            manual_command = poll_manual_command() if manual_advance else ""
            if manual_command == "advance":
                return_code, exit_state = terminate_process_group(proc)
                stop_reason = "manual_advance"
                break
            if manual_command == "quit":
                return_code, exit_state = terminate_process_group(proc)
                stop_reason = "manual_quit"
                break

            time.sleep(max(0.1, poll_sec))

    summary = summarize_report(mode, report_path, log_path, exit_state, return_code)
    summary["stop_reason"] = stop_reason
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="ascii")
    return summary


def sanity_check(workspace: Path, mainline_launch: str):
    cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null && "
        f"cd {shlex.quote(str(workspace))} && "
        "source devel/setup.bash >/dev/null && "
        f"roslaunch relay_racer_integration {shlex.quote(mainline_launch)} --nodes >/dev/null"
    )
    subprocess.run(["bash", "-lc", cmd], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    args = parse_args()
    run_tag = args.tag or datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = Path(args.output_dir) / run_tag
    if output_root.exists() and any(output_root.iterdir()) and not args.allow_existing_output:
        print(
            f"[relay_efficiency_eval] output directory already exists and is not empty: {output_root}. Use a new --tag or pass --allow-existing-output.",
            file=sys.stderr,
            flush=True,
        )
        return 2
    output_root.mkdir(parents=True, exist_ok=True)

    if not args.no_build_check:
        sanity_check(WORKSPACE, args.mainline_launch)

    aggregate = []
    requested_modes = list(args.modes)
    for index, requested_mode in enumerate(requested_modes):
        mode = MODE_ALIASES[requested_mode]
        print(f"[relay_efficiency_eval] running mode={mode} (requested={requested_mode})", flush=True)
        summary = run_mode(
            mode=mode,
            workspace=WORKSPACE,
            output_root=output_root,
            timeout_sec=args.timeout_sec,
            poll_sec=args.poll_sec,
            completion_check_enabled=not args.disable_completion_check,
            completion_hold_sec=max(0.0, args.completion_hold_sec),
            manual_advance=args.manual_advance,
            manual_next_file=args.manual_next_file,
            mainline_launch=args.mainline_launch,
        )
        summary["requested_mode"] = requested_mode
        aggregate.append(summary)
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)

        if summary.get("stop_reason") == "manual_quit":
            print("[relay_efficiency_eval] manual quit requested. Stopping experiment sequence.", flush=True)
            break

        if args.manual_advance and index + 1 < len(requested_modes):
            next_mode = MODE_ALIASES[requested_modes[index + 1]]
            if summary.get("stop_reason") not in ("manual_advance", "manual_next_file"):
                if not prompt_before_next(mode, next_mode):
                    print("[relay_efficiency_eval] operator stopped before next mode.", flush=True)
                    break

    aggregate_path = output_root / "aggregate_summary.json"
    aggregate_path.write_text(json.dumps(aggregate, indent=2, sort_keys=True) + "\n", encoding="ascii")
    print(f"[relay_efficiency_eval] aggregate summary: {aggregate_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
