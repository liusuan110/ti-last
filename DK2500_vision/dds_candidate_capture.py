#!/usr/bin/env python3
"""Set one DDS candidate frequency and save the resulting scope-camera frame."""

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


def expect_prefix(lines: list[str], prefix: str, operation: str) -> None:
    if not any(line.startswith(prefix) for line in lines):
        raise RuntimeError(f"{operation} failed: {lines}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--frequency", type=int, required=True)
    parser.add_argument("--phase", type=int, default=0)
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--output", required=True)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument(
        "--restore-direct",
        action="store_true",
        help="switch back to the direct path after the capture",
    )
    args = parser.parse_args()

    if not 1000 <= args.frequency <= 100000:
        raise ValueError("frequency must be in 1000..100000 Hz")
    if not 0 <= args.phase <= 16383:
        raise ValueError("phase must be in 0..16383")

    with serial.Serial(args.serial, 115200, timeout=0.08) as port:
        port.reset_input_buffer()
        send(port, "fping")
        expect_prefix(read_lines(port, 0.4), "F_PONG", "MCU handshake")

        send(port, f"ffreq {args.frequency}")
        expect_prefix(read_lines(port, 0.3), "OK ffreq ", "ffreq")
        send(port, "fmode same")
        mode_reply = read_lines(port, 0.6)
        expect_prefix(mode_reply, "OK fmode same ", "fmode same")
        print("\n".join(mode_reply))
        if args.phase:
            send(port, f"fphase {args.phase}")
            phase_reply = read_lines(port, 0.4)
            expect_prefix(phase_reply, "OK fphase ", "fphase")
            print("\n".join(phase_reply))

        deadline = time.monotonic() + max(0.2, args.settle)
        while time.monotonic() < deadline:
            send(port, "fping")
            read_lines(port, 0.12)
            time.sleep(0.18)

        with urllib.request.urlopen(args.url, timeout=3.0) as response:
            frame = response.read()
        with open(args.output, "wb") as output:
            output.write(frame)
        print(
            f"CAPTURED frequency_hz={args.frequency} phase={args.phase} "
            f"bytes={len(frame)} output={args.output}"
        )

        if args.restore_direct:
            send(port, "fmode thru")
            print("\n".join(read_lines(port, 0.4)))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
