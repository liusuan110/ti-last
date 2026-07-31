#!/usr/bin/env python3
"""Measure Lissajous rotation rate from an uninterrupted camera sequence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time

import numpy as np
import serial

from dds_temporal_search import fetch_jpeg, send
from optical_phase_lock import expect_reply, wait_for_prefix
from ramp_frequency import Settings, trace_mask


DDS_CLOCK_HZ = 25_000_000
FTW_SCALE = (1 << 32) / DDS_CLOCK_HZ


def shape_feature(frame: np.ndarray, settings: Settings) -> tuple[bool, float, float, int]:
    roi = settings.roi.crop(frame)
    mask = trace_mask(roi, settings)
    ys, xs = np.nonzero(mask)
    if xs.size < 250:
        return False, 0.0, 0.0, int(xs.size)

    x = xs.astype(np.float64)
    y = ys.astype(np.float64)
    correlation = float(np.corrcoef(x, y)[0, 1])
    if not math.isfinite(correlation):
        return False, 0.0, 0.0, int(xs.size)

    covariance = np.cov(np.vstack((x, y)))
    eigenvalues = np.linalg.eigvalsh(covariance)
    axis_ratio = math.sqrt(
        max(float(eigenvalues[0]), 0.0)
        / max(float(eigenvalues[1]), 1.0e-9)
    )
    return True, correlation, axis_ratio, int(xs.size)


def fitted_frequency(
    timestamps: list[float],
    values: list[float],
) -> tuple[float, float, float]:
    if len(values) < 12:
        return 0.0, 0.0, 0.0
    start = timestamps[0]
    relative = np.asarray(timestamps, dtype=float) - start
    duration = float(relative[-1])
    if duration <= 0.4:
        return 0.0, 0.0, 0.0

    signal = np.asarray(values, dtype=float)
    total_variance = float(np.sum((signal - np.mean(signal)) ** 2))
    if total_variance <= 1.0e-9:
        return 0.0, 1.0, 0.0

    def residual_at(frequency: float) -> tuple[float, float]:
        angle = 2.0 * math.pi * frequency * relative
        design = np.column_stack(
            (np.ones(relative.size), np.cos(angle), np.sin(angle))
        )
        coefficients, _, _, _ = np.linalg.lstsq(design, signal, rcond=None)
        residual = signal - design @ coefficients
        amplitude = math.hypot(
            float(coefficients[1]), float(coefficients[2])
        )
        return float(np.sum(residual * residual)), amplitude

    # A direct sinusoid fit avoids the old FFT's 0.15 Hz lower cutoff and
    # quantisation to 1 / observation_time.  Coarse search first, then refine
    # around its minimum.
    minimum_frequency = max(0.015, 0.20 / duration)
    maximum_frequency = 2.0
    coarse_step = max(0.004, 1.0 / (duration * 25.0))
    coarse = np.arange(
        minimum_frequency,
        maximum_frequency + 0.5 * coarse_step,
        coarse_step,
    )
    coarse_errors = np.asarray(
        [residual_at(float(frequency))[0] for frequency in coarse]
    )
    coarse_best = float(coarse[int(np.argmin(coarse_errors))])

    fine_step = max(0.0002, coarse_step / 40.0)
    fine = np.arange(
        max(minimum_frequency, coarse_best - 1.5 * coarse_step),
        min(maximum_frequency, coarse_best + 1.5 * coarse_step)
        + 0.5 * fine_step,
        fine_step,
    )
    fine_results = [residual_at(float(frequency)) for frequency in fine]
    fine_errors = np.asarray([result[0] for result in fine_results])
    best_index = int(np.argmin(fine_errors))
    best_frequency = float(fine[best_index])
    best_error, best_amplitude = fine_results[best_index]
    explained = max(0.0, 1.0 - best_error / total_variance)
    return best_frequency, explained, best_amplitude


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--frequency", type=int, required=True)
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument("--phase", type=int, default=0)
    parser.add_argument("--trim-hz", type=float, default=0.0)
    parser.add_argument(
        "--skip-fset",
        action="store_true",
        help="apply only fopt, preserving the current DDS phase origin",
    )
    parser.add_argument(
        "--keep-output",
        action="store_true",
        help="observe the current DDS output without changing any parameter",
    )
    parser.add_argument(
        "--phase-rate-words-s",
        type=float,
        default=0.0,
        help="continuously ramp DDS phase at this many phase words per second",
    )
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--warmup", type=float, default=0.5)
    parser.add_argument("--interval", type=float, default=0.0)
    args = parser.parse_args()

    settings = Settings.load(args.config)
    timestamps: list[float] = []
    correlations: list[float] = []
    ratios: list[float] = []
    previous_digest: bytes | None = None
    attempted = 0

    with serial.Serial(args.serial, 115200, timeout=0.06) as port:
        port.reset_input_buffer()
        send(port, "fping")
        expect_reply(
            wait_for_prefix(port, "F_PONG", 0.35),
            "F_PONG",
            "MCU handshake",
        )
        phase_word = int(args.phase) % 16384
        if not args.skip_fset and not args.keep_output:
            send(
                port,
                f"fset {args.frequency} {args.amplitude} {phase_word}",
            )
            expect_reply(
                wait_for_prefix(port, "OK fset ", 0.35),
                "OK fset ",
                "initial fset",
            )
        if not args.keep_output and (
            args.skip_fset or args.trim_hz != 0.0
        ):
            ftw = int(round(args.frequency * FTW_SCALE)) + int(
                round(args.trim_hz * FTW_SCALE)
            )
            send(port, f"fopt {ftw} {args.amplitude} {phase_word}")
            expect_reply(
                wait_for_prefix(port, "OK fopt ", 0.35),
                "OK fopt ",
                "trimmed fopt",
            )
        warmup_deadline = time.monotonic() + max(0.12, args.warmup)
        next_keepalive = time.monotonic() + 0.35
        while time.monotonic() < warmup_deadline:
            now = time.monotonic()
            if now >= next_keepalive:
                send(port, "fping")
                next_keepalive = now + 0.35
                time.sleep(0.002)
                while port.in_waiting:
                    port.readline()
            fetch_jpeg(args.url)
            time.sleep(0.025)

        deadline = time.monotonic() + args.duration
        observation_start = time.monotonic()
        next_keepalive = time.monotonic() + 0.35
        next_phase_update = observation_start
        while time.monotonic() < deadline:
            now = time.monotonic()
            if (
                args.phase_rate_words_s != 0.0
                and now >= next_phase_update
            ):
                elapsed_control = now - observation_start
                controlled_phase = int(
                    round(phase_word + args.phase_rate_words_s * elapsed_control)
                ) % 16384
                ftw = int(round(args.frequency * FTW_SCALE)) + int(
                    round(args.trim_hz * FTW_SCALE)
                )
                send(
                    port,
                    f"fopt {ftw} {args.amplitude} {controlled_phase}",
                )
                time.sleep(0.002)
                while port.in_waiting:
                    port.readline()
                next_phase_update = now + 0.20
            if now >= next_keepalive:
                send(port, "fping")
                next_keepalive = now + 0.40
                time.sleep(0.002)
                while port.in_waiting:
                    port.readline()

            attempted += 1
            frame = fetch_jpeg(args.url)
            digest = hashlib.blake2s(
                frame[::8, ::8].tobytes(),
                digest_size=8,
            ).digest()
            if digest == previous_digest:
                time.sleep(0.005)
                continue
            previous_digest = digest
            valid, correlation, axis_ratio, pixels = shape_feature(
                frame, settings
            )
            if valid:
                now = time.monotonic()
                timestamps.append(now)
                correlations.append(correlation)
                ratios.append(axis_ratio)
                print(
                    "DRIFT_FRAME "
                    + json.dumps(
                        {
                            "index": len(correlations) - 1,
                            "time_s": round(now, 6),
                            "correlation": round(correlation, 6),
                            "axis_ratio": round(axis_ratio, 6),
                            "pixels": pixels,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
            if args.interval > 0.0:
                time.sleep(args.interval)

    beat_hz, fit_explained, fit_amplitude = fitted_frequency(
        timestamps, correlations
    )
    elapsed = timestamps[-1] - timestamps[0] if len(timestamps) >= 2 else 0.0
    summary = {
        "valid": int(len(correlations) >= 12),
        "frames": len(correlations),
        "attempted": attempted,
        "elapsed_s": round(elapsed, 4),
        "effective_fps": round(
            (len(correlations) - 1) / elapsed if elapsed > 0.0 else 0.0,
            3,
        ),
        "correlation_min": round(min(correlations), 6)
        if correlations
        else 0.0,
        "correlation_max": round(max(correlations), 6)
        if correlations
        else 0.0,
        "correlation_first": round(correlations[0], 6)
        if correlations
        else 0.0,
        "correlation_last": round(correlations[-1], 6)
        if correlations
        else 0.0,
        "axis_ratio_min": round(min(ratios), 6) if ratios else 0.0,
        "axis_ratio_max": round(max(ratios), 6) if ratios else 0.0,
        "beat_hz_abs": round(beat_hz, 6),
        "fit_explained": round(fit_explained, 6),
        "fit_amplitude": round(fit_amplitude, 6),
        "frequency_trim_hz": round(args.trim_hz, 6),
        "phase_word": phase_word,
        "phase_rate_words_s": round(args.phase_rate_words_s, 6),
    }
    print("DRIFT_FINAL " + json.dumps(summary, ensure_ascii=False))
    return 0 if summary["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
