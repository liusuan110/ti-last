#!/usr/bin/env python3
"""Run one MCU windowed ramp and save a synchronized camera frame."""

from __future__ import annotations

import argparse
import time
import urllib.request

import serial


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def read_lines(port: serial.Serial, duration: float) -> list[str]:
    deadline = time.monotonic() + duration
    result: list[str] = []
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            result.append(line.decode("utf-8", "replace").strip())
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--window-us", type=int, required=True)
    parser.add_argument(
        "--url", default="http://127.0.0.1:8080/frame.jpg"
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--settle", type=float, default=1.5)
    args = parser.parse_args()

    with serial.Serial(args.serial, 115200, timeout=0.08) as port:
        port.reset_input_buffer()
        send(port, "fping")
        replies = read_lines(port, 0.4)
        if not any(line.startswith("F_PONG") for line in replies):
            raise RuntimeError(f"MCU handshake failed: {replies}")

        try:
            send(port, f"fwindow {args.window_us}")
            replies = read_lines(port, 0.4)
            if not any(line.startswith("OK fwindow ") for line in replies):
                raise RuntimeError(f"MCU rejected fwindow: {replies}")

            deadline = time.monotonic() + max(0.5, args.settle)
            while time.monotonic() < deadline:
                send(port, "fping")
                read_lines(port, 0.12)
                time.sleep(0.25)

            with urllib.request.urlopen(args.url, timeout=3.0) as response:
                frame = response.read()
            with open(args.output, "wb") as output:
                output.write(frame)
            print(
                f"CAPTURED window_us={args.window_us} "
                f"bytes={len(frame)} output={args.output}"
            )
        finally:
            send(port, "fwindow off")
            print("\n".join(read_lines(port, 0.4)))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
