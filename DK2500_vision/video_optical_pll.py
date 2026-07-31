#!/usr/bin/env python3
"""Camera-based frequency-and-phase lock for a 1:1 Lissajous circle.

The unknown input is connected only to the oscilloscope X input.  AD9959 CH0
drives Y.  A local three-point phase search keeps the photographed ellipse as
round as possible.  The slow slope of the phase word required to maintain that
circle adjusts the raw DDS frequency tuning word.

Unlike a phase-only servo, a locked loop has a near-zero long-term phase slope
and therefore does not need to keep winding the phase word around the circle.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import time

import numpy as np
import serial

from dds_temporal_search import fetch_jpeg, read_lines, send
from optical_phase_lock import (
    Geometry,
    PHASE_MODULUS,
    ellipse_geometry,
    expect_reply,
    wait_for_prefix,
    wrap_phase,
)
from ramp_frequency import Settings


DDS_CLOCK_HZ = 25_000_000
FTW_SCALE = (1 << 32) / DDS_CLOCK_HZ


def signed_word(value: float) -> float:
    return (
        (float(value) + PHASE_MODULUS / 2.0) % PHASE_MODULUS
        - PHASE_MODULUS / 2.0
    )


def set_output(
    port: serial.Serial,
    ftw: int,
    amplitude: int,
    phase_word: int,
) -> None:
    phase_word = wrap_phase(phase_word)
    send(port, f"fopt {int(ftw)} {int(amplitude)} {phase_word}")
    expect_reply(
        wait_for_prefix(port, "OK fopt ", 0.20),
        "OK fopt ",
        f"fopt {ftw} {amplitude} {phase_word}",
    )


def capture_geometry(
    port: serial.Serial,
    ftw: int,
    amplitude: int,
    phase_word: int,
    camera_url: str,
    settings: Settings,
    settle_seconds: float,
    debug_dir: str | None,
    tag: str,
) -> Geometry:
    phase_word = wrap_phase(phase_word)
    set_output(port, ftw, amplitude, phase_word)
    time.sleep(max(0.035, settle_seconds))

    # The endpoint may still expose the frame that was being encoded while
    # the DDS update arrived.  Drop it, then allow a few attempts for a valid
    # complete ellipse.
    fetch_jpeg(camera_url)
    best: Geometry | None = None
    for attempt in range(4):
        frame = fetch_jpeg(camera_url)
        debug_path = (
            os.path.join(
                debug_dir,
                f"{tag}_ftw{ftw}_p{phase_word}_{attempt}.jpg",
            )
            if debug_dir
            else None
        )
        result = ellipse_geometry(frame, settings, phase_word, debug_path)
        if result.valid and (best is None or result.score > best.score):
            best = result
        if result.valid:
            break
        time.sleep(0.018)
    if best is None:
        return Geometry(False, phase_word, reason="no valid ellipse frame")
    return best


def acquire_circle(
    port: serial.Serial,
    ftw: int,
    amplitude: int,
    camera_url: str,
    settings: Settings,
    settle_seconds: float,
    debug_dir: str | None,
) -> Geometry:
    candidates = [0, 4096, 8192, 12288]
    results: list[Geometry] = []
    for phase_word in candidates:
        result = capture_geometry(
            port,
            ftw,
            amplitude,
            phase_word,
            camera_url,
            settings,
            settle_seconds,
            debug_dir,
            "acquire_coarse",
        )
        results.append(result)
        print(
            "ACQUIRE "
            + json.dumps(result.as_dict(), ensure_ascii=False),
            flush=True,
        )
    valid = [item for item in results if item.valid]
    if not valid:
        raise RuntimeError("initial circle acquisition failed")

    coarse = max(valid, key=lambda item: item.score)
    results = []
    for delta in (-1024, 0, 1024):
        result = capture_geometry(
            port,
            ftw,
            amplitude,
            coarse.phase_word + delta,
            camera_url,
            settings,
            settle_seconds,
            debug_dir,
            "acquire_refine",
        )
        results.append(result)
        print(
            "ACQUIRE "
            + json.dumps(result.as_dict(), ensure_ascii=False),
            flush=True,
        )
    valid = [item for item in results if item.valid]
    return max(valid, key=lambda item: item.score) if valid else coarse


def median_phase_rate(
    history: list[tuple[float, float]],
) -> float:
    slopes: list[float] = []
    for first in range(len(history)):
        for second in range(first + 1, len(history)):
            elapsed = history[second][0] - history[first][0]
            if elapsed >= 0.22:
                slopes.append(
                    (history[second][1] - history[first][1]) / elapsed
                )
    return float(statistics.median(slopes)) if slopes else 0.0


def select_local_circle(
    port: serial.Serial,
    ftw: int,
    amplitude: int,
    center_phase: int,
    step_words: int,
    camera_url: str,
    settings: Settings,
    settle_seconds: float,
    debug_dir: str | None,
    tag: str,
) -> Geometry:
    results: list[Geometry] = []
    for phase_word in (
        center_phase - step_words,
        center_phase,
        center_phase + step_words,
    ):
        result = capture_geometry(
            port,
            ftw,
            amplitude,
            phase_word,
            camera_url,
            settings,
            settle_seconds,
            debug_dir,
            tag,
        )
        print(
            "PHASE_PROBE "
            + json.dumps(
                {"cycle": tag, **result.as_dict()},
                ensure_ascii=False,
            ),
            flush=True,
        )
        if result.valid:
            results.append(result)
    if not results:
        raise RuntimeError(f"{tag}: no valid ellipse")
    return max(results, key=lambda item: item.score)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--config", required=True)
    parser.add_argument("--frequency", type=int, required=True)
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument("--initial-phase", type=int, default=0)
    parser.add_argument(
        "--initial-trim-hz",
        type=float,
        default=0.0,
        help="raw DDS frequency offset retained during phase acquisition",
    )
    parser.add_argument(
        "--search-only",
        action="store_true",
        help="acquire the roundest phase once, then hold it",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="lock duration in seconds; 0 runs until interrupted",
    )
    parser.add_argument("--settle", type=float, default=0.045)
    parser.add_argument("--frequency-gain", type=float, default=0.80)
    parser.add_argument("--frequency-window", type=float, default=3.0)
    parser.add_argument("--max-trim-ftw", type=int, default=512)
    parser.add_argument("--debug-dir")
    args = parser.parse_args()

    if not 1000 <= args.frequency <= 100000:
        raise ValueError("frequency must be in 1000..100000 Hz")
    if not 1 <= args.amplitude <= 1023:
        raise ValueError("amplitude must be in 1..1023")
    if args.duration < 0.0:
        raise ValueError("duration must be non-negative")

    settings = Settings.load(args.config)
    nominal_ftw = int(round(args.frequency * FTW_SCALE))
    current_ftw = nominal_ftw + int(
        round(args.initial_trim_hz * FTW_SCALE)
    )
    current_phase = wrap_phase(args.initial_phase)

    with serial.Serial(args.serial, 115200, timeout=0.06) as port:
        port.reset_input_buffer()
        send(port, "fping")
        expect_reply(read_lines(port, 0.4), "F_PONG", "MCU handshake")

        # fset establishes the DDS/relay state.  All following changes use the
        # raw-FTW fast path without resetting the DDS or toggling the relay.
        send(
            port,
            f"fset {args.frequency} {args.amplitude} {current_phase}",
        )
        expect_reply(
            wait_for_prefix(port, "OK fset ", 0.35),
            "OK fset ",
            "initial fset",
        )
        time.sleep(0.10)

        best = acquire_circle(
            port,
            current_ftw,
            args.amplitude,
            args.url,
            settings,
            args.settle,
            args.debug_dir,
        )
        current_phase = best.phase_word
        set_output(port, current_ftw, args.amplitude, current_phase)
        if args.search_only:
            print(
                "PLL_FINAL "
                + json.dumps(
                    {
                        "frequency_hz_nominal": args.frequency,
                        "ftw": current_ftw,
                        "frequency_trim_hz": round(
                            (current_ftw - nominal_ftw) / FTW_SCALE,
                            6,
                        ),
                        "phase_word": current_phase,
                        "axis_ratio": round(best.axis_ratio, 6),
                        "correlation": round(best.correlation, 6),
                        "locked": 0,
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
            return 0

        start = time.monotonic()
        deadline = math.inf if args.duration == 0.0 else start + args.duration
        history: list[tuple[float, float]] = []
        phase_moves: list[int] = []
        unwrapped_phase = float(current_phase)
        cycle = 0
        last_residual_hz = math.inf
        locked_windows = 0

        try:
            while time.monotonic() < deadline:
                cycle += 1
                step_words = 1024 if best.axis_ratio < 0.62 else 384
                try:
                    local = select_local_circle(
                        port,
                        current_ftw,
                        args.amplitude,
                        current_phase,
                        step_words,
                        args.url,
                        settings,
                        args.settle,
                        args.debug_dir,
                        f"pll{cycle}",
                    )
                except RuntimeError as exc:
                    set_output(
                        port,
                        current_ftw,
                        args.amplitude,
                        current_phase,
                    )
                    print(
                        "PLL_RETRY "
                        + json.dumps(
                            {
                                "cycle": cycle,
                                "reason": str(exc),
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                    time.sleep(0.04)
                    continue

                phase_step = int(
                    signed_word(local.phase_word - current_phase)
                )
                current_phase = local.phase_word
                best = local
                unwrapped_phase += float(phase_step)
                now = time.monotonic()
                history.append((now, unwrapped_phase))
                phase_moves.append(phase_step)

                phase_rate = median_phase_rate(history)
                # If DDS is slow, maintaining the same circle requires an
                # increasing phase offset.  Therefore the required DDS
                # frequency correction has the same sign as phase_rate.
                residual_hz = -phase_rate / PHASE_MODULUS
                frequency_delta_ftw = 0
                history_span = (
                    history[-1][0] - history[0][0]
                    if len(history) >= 2
                    else 0.0
                )
                nonzero_moves = [move for move in phase_moves if move != 0]
                direction_consensus = 0.0
                if nonzero_moves:
                    positive = sum(move > 0 for move in nonzero_moves)
                    negative = len(nonzero_moves) - positive
                    direction_consensus = max(positive, negative) / len(
                        nonzero_moves
                    )
                if (
                    len(history) >= 5
                    and history_span >= args.frequency_window
                ):
                    requested_delta = int(
                        round(
                            args.frequency_gain
                            * phase_rate
                            / PHASE_MODULUS
                            * FTW_SCALE
                        )
                    )
                    requested_delta = int(
                        np.clip(requested_delta, -64, 64)
                    )
                    # Reject camera/score noise that looks like an impossible
                    # jump or lacks a persistent direction.  Fine search has
                    # already reduced the true error to below one hertz.
                    if (
                        abs(residual_hz) > 0.50
                        or direction_consensus < 0.65
                    ):
                        requested_delta = 0
                    next_ftw = int(
                        np.clip(
                            current_ftw + requested_delta,
                            nominal_ftw - args.max_trim_ftw,
                            nominal_ftw + args.max_trim_ftw,
                        )
                    )
                    frequency_delta_ftw = next_ftw - current_ftw
                    current_ftw = next_ftw
                    if frequency_delta_ftw != 0:
                        history.clear()
                        phase_moves.clear()
                        unwrapped_phase = float(current_phase)
                    else:
                        # Keep only a rolling window so old score mistakes
                        # cannot dominate indefinitely.
                        cutoff = now - max(5.0, args.frequency_window)
                        history = [
                            item for item in history if item[0] >= cutoff
                        ]
                        if len(phase_moves) > len(history):
                            phase_moves = phase_moves[-len(history) :]
                    if abs(residual_hz) < 0.008:
                        locked_windows += 1
                    else:
                        locked_windows = 0
                    last_residual_hz = residual_hz

                set_output(
                    port,
                    current_ftw,
                    args.amplitude,
                    current_phase,
                )
                print(
                    "PLL "
                    + json.dumps(
                        {
                            "cycle": cycle,
                            "phase_word": current_phase,
                            "phase_step": phase_step,
                            "axis_ratio": round(best.axis_ratio, 6),
                            "correlation": round(best.correlation, 6),
                            "phase_rate_words_s": round(phase_rate, 3),
                            "residual_frequency_hz": round(
                                residual_hz, 6
                            ),
                            "direction_consensus": round(
                                direction_consensus, 3
                            ),
                            "ftw": current_ftw,
                            "ftw_delta": frequency_delta_ftw,
                            "frequency_trim_hz": round(
                                (current_ftw - nominal_ftw) / FTW_SCALE,
                                6,
                            ),
                            "locked": int(
                                locked_windows >= 2
                                and best.axis_ratio >= 0.62
                            ),
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        except KeyboardInterrupt:
            pass
        finally:
            set_output(
                port,
                current_ftw,
                args.amplitude,
                current_phase,
            )

        print(
            "PLL_FINAL "
            + json.dumps(
                {
                    "frequency_hz_nominal": args.frequency,
                    "ftw": current_ftw,
                    "frequency_trim_hz": round(
                        (current_ftw - nominal_ftw) / FTW_SCALE,
                        6,
                    ),
                    "phase_word": current_phase,
                    "last_residual_frequency_hz": (
                        None
                        if not math.isfinite(last_residual_hz)
                        else round(last_residual_hz, 6)
                    ),
                    "locked": int(locked_windows >= 2),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
