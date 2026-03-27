#!/usr/bin/env python3
import argparse
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

RESET = "[0m"
COLORS = {
    "exploration": "[36m",
    "relay": "[33m",
    "error": "[31m",
    "startup": "[35m",
}

PATTERNS = {
    "exploration": [
        re.compile(p)
        for p in [
            r"\[FSM\]",
            r"Find grid tour",
            r"Find frontier tour",
            r"Grid tour t:",
            r"Next view:",
            r"Mid goal",
            r"Traj opt iter num",
            r"Total time:",
            r"Idle since no frontier",
            r"Larger cost after reallocation",
            r"Allocated grid:",
            r"Grid tour:",
        ]
    ],
    "relay": [
        re.compile(p)
        for p in [
            r"\[Relay\]",
            r"\[RelayCtl\]",
            r"relay_validation_monitor",
            r"relay_event",
            r"post_startup",
            r"task_metrics",
            r"swarm_comm_state",
            r"recovery",
            r"relay_total_occupancy",
            r"mean_information_age",
            r"num_components",
        ]
    ],
    "error": [
        re.compile(p)
        for p in [
            r"\[ERROR\]",
            r"\[FATAL\]",
            r"NS_ASSERT",
            r"NS_FATAL",
            r"process\[.*\] has died",
            r"terminated with exception",
            r"Traceback \(most recent call last\):",
            r"Segmentation fault",
        ]
    ],
    "startup": [
        re.compile(p)
        for p in [
            r"process\[.*\]: started with pid",
            r"process\[.*\] process has finished cleanly",
            r"ROS_MASTER_URI=",
            r"done$",
            r"shutting down processing monitor",
            r"killing on exit",
            r"escalating to SIGTERM",
            r"WARNING: disk usage in log directory",
            r"racer_comm_controller_node serving",
        ]
    ],
}

MODE_CHOICES = ("mainline", "exploration", "relay", "all")


def classify_line(line: str):
    categories = []
    for name in ("error", "exploration", "relay", "startup"):
        if any(pattern.search(line) for pattern in PATTERNS[name]):
            categories.append(name)
    return categories


def should_show(mode: str, categories):
    if not categories:
        return False
    if mode == "all":
        return True
    if "error" in categories:
        return True
    if mode == "mainline":
        return any(cat in categories for cat in ("exploration", "relay", "startup"))
    return mode in categories


def format_line(line: str, categories, color: bool):
    if "error" in categories:
        tag = "ERR"
        color_code = COLORS["error"]
    elif "relay" in categories and "exploration" in categories:
        tag = "MAIN"
        color_code = COLORS["relay"]
    elif "relay" in categories:
        tag = "RELAY"
        color_code = COLORS["relay"]
    elif "exploration" in categories:
        tag = "EXP"
        color_code = COLORS["exploration"]
    else:
        tag = "SYS"
        color_code = COLORS["startup"]

    payload = f"[{tag}] {line.rstrip()}"
    if color and sys.stdout.isatty():
        return f"{color_code}{payload}{RESET}"
    return payload


def stream_filtered_lines(stream, raw_log, mode: str, color: bool):
    for line in stream:
        raw_log.write(line)
        raw_log.flush()
        categories = classify_line(line)
        if should_show(mode, categories):
            print(format_line(line, categories, color), flush=True)


def wait_for_file(path: Path, timeout_sec: float):
    deadline = time.time() + timeout_sec
    while not path.exists():
        if time.time() > deadline:
            raise TimeoutError(f"log file not found within {timeout_sec:.1f}s: {path}")
        time.sleep(0.2)


def watch_log(args):
    log_path = Path(args.log_file)
    wait_for_file(log_path, args.wait_timeout)
    print(f"[relay_log_console] watching {log_path} mode={args.mode}", flush=True)

    with log_path.open("r", encoding="utf-8", errors="replace") as fh:
        if not args.from_start:
            fh.seek(0, os.SEEK_END)
        while True:
            line = fh.readline()
            if line:
                categories = classify_line(line)
                if should_show(args.mode, categories):
                    print(format_line(line, categories, args.color), flush=True)
                continue
            time.sleep(0.2)


def launch_and_filter(args):
    log_path = Path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = ["roslaunch", "relay_racer_integration", args.launch_file]
    cmd.append(f"use_local_topic_router:={'true' if args.use_local_topic_router else 'false'}")
    cmd.extend(args.roslaunch_arg)

    print(f"[relay_log_console] mode={args.mode} log={log_path}", flush=True)
    print(f"[relay_log_console] cmd={' '.join(cmd)}", flush=True)
    print(
        f"[relay_log_console] watch existing log: python3 {Path(__file__).resolve()} watch --mode {args.mode} --log-file {log_path}",
        flush=True,
    )

    with log_path.open("w", encoding="utf-8") as raw_log:
        raw_log.write(f"[relay_log_console] cmd={' '.join(cmd)}\n")
        raw_log.flush()
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
            errors="replace",
        )

        def handle_signal(signum, _frame):
            if proc.poll() is None:
                proc.send_signal(signum)

        old_int = signal.signal(signal.SIGINT, handle_signal)
        old_term = signal.signal(signal.SIGTERM, handle_signal)
        try:
            assert proc.stdout is not None
            stream_filtered_lines(proc.stdout, raw_log, args.mode, args.color)
            return proc.wait()
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)


def build_parser():
    parser = argparse.ArgumentParser(description="Launch or watch relay mainline logs with runtime filtering.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    launch = subparsers.add_parser("launch", help="launch roslaunch and only print selected log categories")
    launch.add_argument("--mode", choices=MODE_CHOICES, default="mainline")
    launch.add_argument("--log-file", default="/tmp/relay_mainline_live.log")
    launch.add_argument("--launch-file", default="relay_real_input_4planner_min.launch")
    launch.add_argument("--roslaunch-arg", action="append", default=[], help="extra roslaunch arg, e.g. planner_sensor_gate:=false")
    launch.add_argument("--use-local-topic-router", action="store_true")
    launch.add_argument("--no-color", dest="color", action="store_false")
    launch.set_defaults(color=True)

    watch = subparsers.add_parser("watch", help="tail an existing log file with the same filters")
    watch.add_argument("--mode", choices=MODE_CHOICES, default="mainline")
    watch.add_argument("--log-file", default="/tmp/relay_mainline_live.log")
    watch.add_argument("--from-start", action="store_true")
    watch.add_argument("--wait-timeout", type=float, default=20.0)
    watch.add_argument("--no-color", dest="color", action="store_false")
    watch.set_defaults(color=True)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "launch":
        sys.exit(launch_and_filter(args))
    if args.command == "watch":
        watch_log(args)
        return
    parser.error("unknown command")


if __name__ == "__main__":
    main()
