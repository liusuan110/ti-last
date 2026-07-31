#!/usr/bin/env python3
"""Launch the complete requirement-5 optical pipeline from three MCU keys.

The service owns the CH340 only while idle.  After a key request it closes
the port and starts ``requirement5_target.py``, allowing that process to own
the serial link for the entire blind acquisition and visual servo.  A physical
MCU reset makes the active optical process exit; this service then returns to
the idle key-listening state instead of re-locking the DDS automatically.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time

import serial


REQUEST_PATTERN = re.compile(
    r"\bF_EVENT\s+requirement5_start=(line|circle|infinity)\b"
)


def run_target(args: argparse.Namespace, target: str) -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    command = [
        sys.executable,
        os.path.join(script_dir, "requirement5_target.py"),
        "--serial",
        args.serial,
        "--config",
        args.config,
        "--url",
        args.url,
        "--amplitude",
        str(args.amplitude),
        "--target",
        target,
        "--stable-seconds",
        str(args.stable_seconds),
        "--display-settle",
        str(args.display_settle),
        "--online-refine",
        str(args.online_refine),
        "--servo-duration",
        "0",
        "--output-dir",
        args.output_dir,
    ]
    print(
        f"BUTTON_SERVICE_START target={target} time={time.time():.3f}",
        flush=True,
    )
    completed = subprocess.run(command, check=False)
    print(
        "BUTTON_SERVICE_RETURN "
        f"target={target} exit={completed.returncode} time={time.time():.3f}",
        flush=True,
    )
    return completed.returncode


def listen(args: argparse.Namespace) -> None:
    while True:
        try:
            with serial.Serial(args.serial, 115200, timeout=0.10) as port:
                # Never replay a key event buffered before this service owned
                # the port.  A deliberate new press is required for every run.
                port.reset_input_buffer()
                print(
                    f"BUTTON_SERVICE_IDLE serial={args.serial}",
                    flush=True,
                )
                while True:
                    raw = port.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    if "MCU READY" in line:
                        port.write(b"fmode off\r\n")
                        port.flush()
                        print("BUTTON_SERVICE_MCU_RESET idle=1", flush=True)
                        continue
                    match = REQUEST_PATTERN.search(line)
                    if match is None:
                        continue
                    target = match.group(1)
                    # The context manager closes the CH340 before the child
                    # opens it.  Break out through the local target variable.
                    break
            run_target(args, target)
            # USB/UART and the MCU boot banner need a short quiet interval
            # after reset or an aborted run before the listener reopens.
            time.sleep(0.40)
        except (OSError, serial.SerialException) as error:
            print(
                f"BUTTON_SERVICE_SERIAL_WAIT error={error!s}",
                flush=True,
            )
            time.sleep(0.50)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--config", required=True)
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument("--stable-seconds", type=float, default=5.0)
    parser.add_argument("--display-settle", type=float, default=0.35)
    parser.add_argument("--online-refine", type=int, default=2)
    parser.add_argument(
        "--output-dir", default="/home/bupt/vision/target_output"
    )
    args = parser.parse_args()
    print("BUTTON_SERVICE_READY", flush=True)
    listen(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
