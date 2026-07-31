#!/usr/bin/env python3
"""Video optical phase servo for a 1:1 Lissajous circle.

The unknown sine is connected to the oscilloscope X input and AD9959 CH0 is
connected to Y.  Once frequency search has selected the closest DDS frequency,
this program changes only the DDS phase word.  It fits an ellipse to the
camera trace, searches for the largest minor/major axis ratio, and then keeps
the trace near that circular state with a small video feedback servo.

The unwrapped phase correction slope is also reported.  A persistent slope
means that a residual sub-hertz frequency error remains even though the phase
servo can keep the displayed curve close to a circle.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import time
from dataclasses import dataclass

import cv2
import numpy as np
import serial

from dds_temporal_search import fetch_jpeg, read_lines, send
from ramp_frequency import Settings, trace_mask


PHASE_MODULUS = 16384


@dataclass
class Geometry:
    valid: bool
    phase_word: int
    score: float = 0.0
    axis_ratio: float = 0.0
    angular_coverage: float = 0.0
    fit_residual: float = 1.0
    correlation: float = 0.0
    pixels: int = 0
    center_x: float = 0.0
    center_y: float = 0.0
    major_axis: float = 0.0
    minor_axis: float = 0.0
    angle_deg: float = 0.0
    reason: str = ""

    def as_dict(self) -> dict[str, float | int | str]:
        return {
            "valid": int(self.valid),
            "phase_word": self.phase_word,
            "score": round(self.score, 6),
            "axis_ratio": round(self.axis_ratio, 6),
            "angular_coverage": round(self.angular_coverage, 6),
            "fit_residual": round(self.fit_residual, 6),
            "correlation": round(self.correlation, 6),
            "pixels": self.pixels,
            "center_x": round(self.center_x, 2),
            "center_y": round(self.center_y, 2),
            "major_axis": round(self.major_axis, 2),
            "minor_axis": round(self.minor_axis, 2),
            "angle_deg": round(self.angle_deg, 2),
            "reason": self.reason,
        }


def wrap_phase(value: int) -> int:
    return int(value) % PHASE_MODULUS


def signed_phase_delta(new_phase: int, old_phase: int) -> int:
    return (
        (int(new_phase) - int(old_phase) + PHASE_MODULUS // 2)
        % PHASE_MODULUS
        - PHASE_MODULUS // 2
    )


def ellipse_geometry(
    frame: np.ndarray,
    settings: Settings,
    phase_word: int,
    debug_path: str | None = None,
) -> Geometry:
    roi = settings.roi.crop(frame)
    mask = trace_mask(roi, settings)
    # Join the oscilloscope's forced horizontal dot segments without changing
    # the overall ellipse axes.
    joined = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 5)),
        iterations=1,
    )
    ys, xs = np.nonzero(joined)
    if xs.size < 250:
        return Geometry(
            False,
            phase_word,
            pixels=int(xs.size),
            reason="too few trace pixels",
        )

    points = np.column_stack((xs, ys)).astype(np.float32).reshape(-1, 1, 2)
    try:
        (cx, cy), (axis_a, axis_b), angle_deg = cv2.fitEllipse(points)
    except cv2.error as exc:
        return Geometry(False, phase_word, pixels=int(xs.size), reason=str(exc))

    major = float(max(axis_a, axis_b))
    minor = float(min(axis_a, axis_b))
    height, width = joined.shape
    if major < 0.20 * min(width, height) or minor < 8.0:
        return Geometry(
            False,
            phase_word,
            pixels=int(xs.size),
            major_axis=major,
            minor_axis=minor,
            reason="ellipse is too small or line-like",
        )

    # Convert every trace pixel to normalized ellipse coordinates.  Coverage
    # rejects partial arcs; radial residual rejects menus and multi-ellipse
    # persistence images that happen to have a large bounding box.
    angle = math.radians(float(angle_deg))
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    dx = xs.astype(np.float64) - float(cx)
    dy = ys.astype(np.float64) - float(cy)
    xr = cos_a * dx + sin_a * dy
    yr = -sin_a * dx + cos_a * dy
    semia = max(float(axis_a) * 0.5, 1.0)
    semib = max(float(axis_b) * 0.5, 1.0)
    normalized_radius = np.sqrt((xr / semia) ** 2 + (yr / semib) ** 2)
    residual = float(np.median(np.abs(normalized_radius - 1.0)))

    theta = np.mod(np.arctan2(yr / semib, xr / semia), 2.0 * math.pi)
    bins = np.unique(np.floor(theta * 72.0 / (2.0 * math.pi)).astype(int))
    coverage = float(len(bins) / 72.0)
    axis_ratio = minor / max(major, 1.0)
    correlation = float(np.corrcoef(xs.astype(float), ys.astype(float))[0, 1])
    if not math.isfinite(correlation):
        correlation = 0.0

    # Circularity is primary.  Coverage and fit quality prevent a short arc or
    # several persistence trails from winning merely because fitEllipse found
    # nearly equal axes.
    quality = float(np.clip(1.0 - residual / 0.24, 0.0, 1.0))
    score = axis_ratio * (0.45 + 0.55 * coverage) * quality
    valid = coverage >= 0.55 and residual <= 0.30
    result = Geometry(
        valid,
        phase_word,
        score=score if valid else 0.0,
        axis_ratio=axis_ratio,
        angular_coverage=coverage,
        fit_residual=residual,
        correlation=correlation,
        pixels=int(xs.size),
        center_x=float(cx),
        center_y=float(cy),
        major_axis=major,
        minor_axis=minor,
        angle_deg=float(angle_deg),
        reason="" if valid else "incomplete or poor ellipse fit",
    )

    if debug_path:
        os.makedirs(os.path.dirname(debug_path) or ".", exist_ok=True)
        overlay = roi.copy()
        cv2.ellipse(
            overlay,
            ((float(cx), float(cy)), (float(axis_a), float(axis_b)), float(angle_deg)),
            (0, 0, 255),
            2,
        )
        cv2.putText(
            overlay,
            (
                f"p={phase_word} ratio={axis_ratio:.3f} "
                f"cov={coverage:.2f} res={residual:.3f}"
            ),
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(debug_path, overlay)
    return result


def expect_reply(lines: list[str], prefix: str, operation: str) -> None:
    if not any(line.startswith(prefix) for line in lines):
        raise RuntimeError(f"{operation} failed: {lines}")


def wait_for_prefix(
    port: serial.Serial,
    prefix: str,
    timeout_seconds: float,
) -> list[str]:
    """Read only until the expected reply arrives instead of burning a fixed delay."""
    deadline = time.monotonic() + timeout_seconds
    lines: list[str] = []
    while time.monotonic() < deadline:
        line = port.readline()
        if not line:
            continue
        text = line.decode("utf-8", "replace").strip()
        lines.append(text)
        if text.startswith(prefix):
            return lines
    return lines


def set_phase(port: serial.Serial, phase_word: int) -> None:
    phase_word = wrap_phase(phase_word)
    send(port, f"fphase {phase_word}")
    expect_reply(
        wait_for_prefix(port, "OK fphase ", 0.18),
        "OK fphase ",
        f"fphase {phase_word}",
    )


def evaluate_phase(
    port: serial.Serial,
    phase_word: int,
    camera_url: str,
    settings: Settings,
    settle_seconds: float,
    frames: int,
    debug_dir: str | None,
    tag: str,
) -> Geometry:
    phase_word = wrap_phase(phase_word)
    set_phase(port, phase_word)
    time.sleep(max(0.05, settle_seconds))
    # Discard the first endpoint image after changing phase.
    fetch_jpeg(camera_url)

    results: list[Geometry] = []
    wanted_frames = max(1, frames)
    max_attempts = wanted_frames + 3
    for index in range(max_attempts):
        frame = fetch_jpeg(camera_url)
        debug_path = (
            os.path.join(debug_dir, f"{tag}_p{phase_word}_{index}.jpg")
            if debug_dir
            else None
        )
        geometry = ellipse_geometry(frame, settings, phase_word, debug_path)
        if geometry.valid:
            results.append(geometry)
            if len(results) >= wanted_frames:
                break
        time.sleep(0.018)

    if not results:
        return Geometry(False, phase_word, reason="no valid ellipse frame")

    representative = max(
        results,
        key=lambda item: item.score,
    )
    representative.score = float(statistics.median(item.score for item in results))
    representative.axis_ratio = float(
        statistics.median(item.axis_ratio for item in results)
    )
    representative.angular_coverage = float(
        statistics.median(item.angular_coverage for item in results)
    )
    representative.fit_residual = float(
        statistics.median(item.fit_residual for item in results)
    )
    representative.correlation = float(
        statistics.median(item.correlation for item in results)
    )
    return representative


def scan_phases(
    port: serial.Serial,
    candidates: list[int],
    camera_url: str,
    settings: Settings,
    settle_seconds: float,
    frames: int,
    debug_dir: str | None,
    tag: str,
) -> Geometry:
    unique_candidates: list[int] = []
    for phase in candidates:
        wrapped = wrap_phase(phase)
        if wrapped not in unique_candidates:
            unique_candidates.append(wrapped)

    results: list[Geometry] = []
    for phase in unique_candidates:
        result = evaluate_phase(
            port,
            phase,
            camera_url,
            settings,
            settle_seconds,
            frames,
            debug_dir,
            tag,
        )
        results.append(result)
        print(
            json.dumps({"stage": tag, **result.as_dict()}, ensure_ascii=False),
            flush=True,
        )
    valid = [item for item in results if item.valid]
    if not valid:
        raise RuntimeError(f"{tag}: no valid phase candidate")
    return max(valid, key=lambda item: item.score)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--config", required=True)
    parser.add_argument("--frequency", type=int, required=True)
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument("--initial-phase", type=int, default=0)
    parser.add_argument("--settle", type=float, default=0.05)
    parser.add_argument("--frames", type=int, default=1)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--debug-dir")
    parser.add_argument(
        "--search-only",
        action="store_true",
        help="find and hold the circular phase without the continuous servo",
    )
    args = parser.parse_args()

    if not 1000 <= args.frequency <= 100000:
        raise ValueError("frequency must be in 1000..100000 Hz")
    if not 1 <= args.amplitude <= 1023:
        raise ValueError("amplitude must be in 1..1023")

    settings = Settings.load(args.config)
    with serial.Serial(args.serial, 115200, timeout=0.06) as port:
        port.reset_input_buffer()
        send(port, "fping")
        expect_reply(read_lines(port, 0.4), "F_PONG", "MCU handshake")

        initial_phase = wrap_phase(args.initial_phase)
        send(
            port,
            f"fset {args.frequency} {args.amplitude} {initial_phase}",
        )
        expect_reply(
            wait_for_prefix(port, "OK fset ", 0.35),
            "OK fset ",
            "initial fset",
        )
        time.sleep(0.12)

        coarse = scan_phases(
            port,
            [0, 4096, 8192, 12288],
            args.url,
            settings,
            max(0.08, args.settle),
            max(1, args.frames),
            args.debug_dir,
            "coarse",
        )
        refined = scan_phases(
            port,
            [coarse.phase_word + delta for delta in range(-3072, 3073, 1024)],
            args.url,
            settings,
            max(0.08, args.settle),
            max(1, args.frames),
            args.debug_dir,
            "refine",
        )
        best = scan_phases(
            port,
            [refined.phase_word + delta for delta in range(-1024, 1025, 256)],
            args.url,
            settings,
            max(0.08, args.settle),
            max(1, args.frames),
            args.debug_dir,
            "fine",
        )
        set_phase(port, best.phase_word)
        print(
            "SEARCH_FINAL "
            + json.dumps(
                {
                    "frequency_hz": args.frequency,
                    **best.as_dict(),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

        if args.search_only or args.duration <= 0:
            return 0

        deadline = time.monotonic() + args.duration
        current_phase = best.phase_word
        unwrapped_phase = float(current_phase)
        history: list[tuple[float, float]] = [(time.monotonic(), unwrapped_phase)]

        cycle = 0
        while time.monotonic() < deadline:
            cycle += 1
            step = 1024 if best.axis_ratio < 0.55 else 384
            try:
                local = scan_phases(
                    port,
                    [
                        current_phase - step,
                        current_phase,
                        current_phase + step,
                    ],
                    args.url,
                    settings,
                    max(0.06, 0.60 * args.settle),
                    1,
                    args.debug_dir,
                    f"lock{cycle}",
                )
            except RuntimeError as exc:
                set_phase(port, current_phase)
                print(
                    "LOCK_RETRY "
                    + json.dumps(
                        {
                            "cycle": cycle,
                            "phase_word": current_phase,
                            "reason": str(exc),
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                time.sleep(0.04)
                continue

            phase_delta = signed_phase_delta(local.phase_word, current_phase)
            current_phase = local.phase_word
            unwrapped_phase += float(phase_delta)
            best = local
            set_phase(port, current_phase)

            now = time.monotonic()
            history.append((now, unwrapped_phase))
            recent = [item for item in history if now - item[0] <= 5.0]
            phase_rate = 0.0
            if len(recent) >= 2 and recent[-1][0] > recent[0][0]:
                phase_rate = (recent[-1][1] - recent[0][1]) / (
                    recent[-1][0] - recent[0][0]
                )
            print(
                "LOCK "
                + json.dumps(
                    {
                        "cycle": cycle,
                        "phase_word": current_phase,
                        "axis_ratio": round(best.axis_ratio, 6),
                        "correlation": round(best.correlation, 6),
                        "score": round(best.score, 6),
                        "phase_step": phase_delta,
                        "phase_rate_words_s": round(phase_rate, 3),
                        "residual_frequency_hz_abs": round(
                            abs(phase_rate) / PHASE_MODULUS, 6
                        ),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )

        set_phase(port, current_phase)
        print(
            "LOCK_FINAL "
            + json.dumps(
                {
                    "frequency_hz": args.frequency,
                    "phase_word": current_phase,
                    "axis_ratio": round(best.axis_ratio, 6),
                    "correlation": round(best.correlation, 6),
                    "score": round(best.score, 6),
                },
                ensure_ascii=False,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
