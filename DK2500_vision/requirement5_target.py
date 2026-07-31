#!/usr/bin/env python3
"""Run blind frequency lock and preserve its exact FTW for target shaping."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time


def run_stage(
    command: list[str],
    marker: str,
    fallback_valid_key: str | None = None,
) -> dict:
    completed = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    print(completed.stdout, end="", flush=True)
    if "MCU_RESET_DETECTED:" in completed.stdout:
        print("REQUIREMENT5_RESET_RETURN_IDLE", flush=True)
        raise SystemExit(3)
    records = [
        line[len(marker) :]
        for line in completed.stdout.splitlines()
        if line.startswith(marker)
    ]
    if not records:
        raise RuntimeError(
            f"stage did not produce {marker.strip()}: exit={completed.returncode}"
        )
    result = json.loads(records[-1])
    accepted = bool(result.get("valid")) or bool(
        fallback_valid_key and result.get(fallback_valid_key)
    )
    if not accepted:
        raise RuntimeError(f"stage failed: {result}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument(
        "--target",
        choices=("line", "circle", "infinity"),
        default="line",
    )
    parser.add_argument("--stable-seconds", type=float, default=5.0)
    parser.add_argument("--display-settle", type=float, default=0.35)
    parser.add_argument("--online-refine", type=int, default=2)
    parser.add_argument(
        "--servo-duration",
        type=float,
        default=0.0,
        help="visual phase servo lifetime; 0 keeps the lock active indefinitely",
    )
    parser.add_argument("--output-dir", default="/home/bupt/vision/target_output")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    started = time.monotonic()
    lock = run_stage(
        [
            sys.executable,
            os.path.join(script_dir, "fast_phase_lock.py"),
            "--config",
            args.config,
            "--serial",
            args.serial,
            "--url",
            args.url,
            "--amplitude",
            str(args.amplitude),
            "--stable-seconds",
            str(args.stable_seconds),
            "--display-settle",
            str(args.display_settle),
            "--online-refine",
            str(args.online_refine),
            "--fast-handoff",
        ],
        "FAST_LOCK_FINAL ",
        fallback_valid_key="visual_servo_ready",
    )

    valid_refinements = [
        item
        for item in lock.get("online_refinement", [])
        if item.get("valid")
    ]
    measured_phase_rate = (
        float(valid_refinements[-1].get("phase_rate_words_s", 0.0))
        if valid_refinements
        else 0.0
    )

    shape_command = [
        sys.executable,
        os.path.join(script_dir, "shape_control.py"),
        "--config",
        args.config,
        "--serial",
        args.serial,
        "--url",
        args.url,
        "--frequency",
        str(lock["frequency_hz"]),
        "--amplitude",
        str(args.amplitude),
        "--locked-ftw",
        str(lock["locked_ftw"]),
        "--target-only",
        args.target,
        "--stable-seconds",
        str(args.stable_seconds),
        "--output-dir",
        args.output_dir,
    ]
    if args.target in ("line", "circle", "infinity"):
        shape_command.extend(
            (
                "--visual-servo",
                "--direct-servo",
                "--servo-duration",
                str(args.servo_duration),
                "--initial-phase",
                str(lock["phase_word"]),
            )
        )
    if args.servo_duration <= 0.0:
        # Production mode never returns: replace this wrapper with the visual
        # servo so its live telemetry is not trapped in subprocess.run.
        print(
            "REQUIREMENT5_SERVO_START "
            + json.dumps(
                {
                    "valid": 1,
                    "frequency_hz": lock["frequency_hz"],
                    "locked_ftw": lock["locked_ftw"],
                    "phase_word": lock["phase_word"],
                    "target": args.target,
                    "elapsed_s": round(time.monotonic() - started, 3),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        os.execv(sys.executable, shape_command)
    shape = run_stage(shape_command, "SHAPE_TARGET_FINAL ")
    print(
        "REQUIREMENT5_FINAL "
        + json.dumps(
            {
                "valid": 1,
                "frequency_hz": lock["frequency_hz"],
                "locked_ftw": lock["locked_ftw"],
                "ftw_offset": lock["trim_ftw_offset"],
                "target": args.target,
                "phase_word": shape["phase_word"],
                "measured_phase_rate_words_s": measured_phase_rate,
                "image": shape["image"],
                "elapsed_s": round(time.monotonic() - started, 3),
                "lock": lock,
                "shape": shape,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
