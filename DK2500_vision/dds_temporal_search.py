#!/usr/bin/env python3
"""Fine-search input frequency from temporal stability of an XY trace.

The unknown signal is on scope X and AD9959 CH0 is on scope Y.  At equal
frequencies the line/ellipse is stationary.  A frequency error rotates the
trace, so the union of several camera masks grows rapidly even when every
individual frame still looks thin.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import time
import urllib.request

import cv2
import numpy as np
import serial

from ramp_frequency import Settings, trace_mask


class MCUResetDetected(RuntimeError):
    """The target MCU rebooted while an optical-control run was active."""


def stop_after_mcu_reset(port: serial.Serial) -> None:
    """Make relay-direct/DDS-off the deterministic post-reset state."""
    try:
        port.write(b"fmode off\r\n")
        port.flush()
        time.sleep(0.03)
    except (OSError, serial.SerialException):
        pass


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def read_lines(port: serial.Serial, duration: float) -> list[str]:
    deadline = time.monotonic() + duration
    lines: list[str] = []
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            decoded = line.decode("utf-8", "replace").strip()
            if "MCU READY" in decoded:
                stop_after_mcu_reset(port)
                raise MCUResetDetected(
                    "MCU_RESET_DETECTED: stop optical control and return idle"
                )
            lines.append(decoded)
    return lines


def fetch_jpeg(url: str) -> np.ndarray:
    # The timestamp query prevents accidental reuse by intermediate caches.
    separator = "&" if "?" in url else "?"
    request_url = f"{url}{separator}t={time.time_ns()}"
    with urllib.request.urlopen(request_url, timeout=3.0) as response:
        payload = response.read()
    frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("camera JPEG decode failed")
    return frame


def evaluate_candidate(
    port: serial.Serial,
    frequency_hz: int,
    amplitude: int,
    camera_url: str,
    settings: Settings,
    observe_seconds: float,
    interval_seconds: float,
    debug_dir: str | None,
) -> dict[str, float | int]:
    # Keep amplitude and phase offset fixed across the search. AD9959 keeps
    # its phase accumulator running when CFTW0 is updated, so this changes the
    # sine-wave slope without introducing a PA15 ramp or a phase reset.
    send(port, f"fset {frequency_hz} {amplitude} 0")
    replies = read_lines(port, 0.28)
    if not any(line.startswith("OK fset ") for line in replies):
        raise RuntimeError(f"fset {frequency_hz} failed: {replies}")

    # Drop the first camera frame after a DDS update.
    time.sleep(0.35)
    fetch_jpeg(camera_url)

    masks: list[np.ndarray] = []
    deadline = time.monotonic() + observe_seconds
    while time.monotonic() < deadline:
        frame = fetch_jpeg(camera_url)
        mask = trace_mask(settings.roi.crop(frame), settings)
        if np.count_nonzero(mask) >= 80:
            masks.append(mask > 0)
        send(port, "fping")
        read_lines(port, 0.04)
        time.sleep(max(0.02, interval_seconds))

    if len(masks) < 3:
        return {
            "frequency_hz": frequency_hz,
            "valid": 0,
            "frames": len(masks),
            "score": 1.0e9,
        }

    union = np.logical_or.reduce(masks)
    counts = [int(np.count_nonzero(mask)) for mask in masks]
    median_pixels = float(statistics.median(counts))
    union_pixels = int(np.count_nonzero(union))
    growth = union_pixels / max(1.0, median_pixels)

    # The union area is the primary discriminator. Do not divide it by the
    # per-frame area: a large mismatch may already fill every individual
    # frame, which would make that incorrect ratio deceptively small.
    area_ratio = union_pixels / float(union.size)
    score = area_ratio
    result: dict[str, float | int] = {
        "frequency_hz": frequency_hz,
        "valid": 1,
        "frames": len(masks),
        "median_pixels": int(round(median_pixels)),
        "union_pixels": union_pixels,
        "growth": round(growth, 6),
        "area_ratio": round(area_ratio, 6),
        "score": round(score, 6),
    }

    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)
        cv2.imwrite(
            os.path.join(debug_dir, f"union_{frequency_hz}.png"),
            union.astype(np.uint8) * 255,
        )
    return result


def parse_candidates(text: str) -> list[int]:
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if not values or any(value < 1000 or value > 100000 for value in values):
        raise argparse.ArgumentTypeError("candidates must be 1000..100000 Hz")
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--config", required=True)
    parser.add_argument("--candidates", type=parse_candidates, required=True)
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument("--observe", type=float, default=1.4)
    parser.add_argument("--interval", type=float, default=0.10)
    parser.add_argument("--debug-dir")
    args = parser.parse_args()

    settings = Settings.load(args.config)
    if not 1 <= args.amplitude <= 1023:
        raise ValueError("amplitude must be in 1..1023")
    results: list[dict[str, float | int]] = []
    with serial.Serial(args.serial, 115200, timeout=0.06) as port:
        port.reset_input_buffer()
        send(port, "fping")
        if not any(line.startswith("F_PONG") for line in read_lines(port, 0.4)):
            raise RuntimeError("MCU handshake failed")

        for candidate in args.candidates:
            result = evaluate_candidate(
                port,
                candidate,
                args.amplitude,
                args.url,
                settings,
                max(0.3, args.observe),
                max(0.02, args.interval),
                args.debug_dir,
            )
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)

        valid = [item for item in results if item["valid"]]
        if not valid:
            raise RuntimeError("no valid candidate")
        best = min(valid, key=lambda item: float(item["score"]))
        send(
            port,
            f"fset {int(best['frequency_hz'])} {args.amplitude} 0",
        )
        read_lines(port, 0.3)
        print(
            "FINAL "
            + json.dumps(
                {
                    "frequency_hz": best["frequency_hz"],
                    "score": best["score"],
                    "tested": len(results),
                },
                ensure_ascii=False,
            )
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except MCUResetDetected as error:
        print(str(error), flush=True)
        raise SystemExit(3) from None
