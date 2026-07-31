#!/usr/bin/env python3
"""Closed-loop visual control for line, circle and figure-eight Lissajous shapes."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
import statistics
import time

import cv2
import numpy as np
import serial

from dds_temporal_search import (
    MCUResetDetected,
    fetch_jpeg,
    read_lines,
    send,
    stop_after_mcu_reset,
)
from fast_phase_lock import predicted_ftw_offset
from ramp_frequency import Settings, trace_mask
from video_phase_drift import FTW_SCALE


LOCKED_FTW_FREQUENCY: int | None = None
LOCKED_FTW_VALUE: int | None = None
PHASE_RATE_WORDS_S = 0.0
PHASE_RATE_EPOCH = 0.0
FTW_AUTO_ADJUST_COUNT = 0
FTW_AUTO_ADJUST_TOTAL = 0
FTW_DRIFT_WINDOW_S = 3.5
FTW_DRIFT_RATE_WORDS_S = 55.0
FTW_AUTO_ADJUST_LIMIT = 4


def configure_locked_ftw(frequency_hz: int, ftw: int | None) -> None:
    """Preserve the online frequency-lock result during phase-only control."""
    global LOCKED_FTW_FREQUENCY, LOCKED_FTW_VALUE
    global FTW_AUTO_ADJUST_COUNT, FTW_AUTO_ADJUST_TOTAL
    LOCKED_FTW_FREQUENCY = frequency_hz if ftw is not None else None
    LOCKED_FTW_VALUE = int(ftw) if ftw is not None else None
    FTW_AUTO_ADJUST_COUNT = 0
    FTW_AUTO_ADJUST_TOTAL = 0


def signed_phase_delta(new_phase: int, old_phase: int) -> int:
    """Return the shortest signed change between two 14-bit phase words."""
    return ((new_phase - old_phase + 8192) % 16384) - 8192


def auto_adjust_ftw_from_phase_drift(
    samples: deque[tuple[float, int]],
    now: float,
    phase_word: int,
    rate_threshold: float = FTW_DRIFT_RATE_WORDS_S,
) -> tuple[int, float, int | None]:
    """Convert sustained visual phase winding into a one-LSB FTW trim."""
    global LOCKED_FTW_VALUE, FTW_AUTO_ADJUST_COUNT, FTW_AUTO_ADJUST_TOTAL

    samples.append((now, phase_word % 16384))
    while len(samples) > 2 and now - samples[0][0] > 5.0:
        samples.popleft()
    if (
        LOCKED_FTW_VALUE is None
        or FTW_AUTO_ADJUST_COUNT >= FTW_AUTO_ADJUST_LIMIT
        or len(samples) < 3
        or now - samples[0][0] < FTW_DRIFT_WINDOW_S
    ):
        return 0, 0.0, LOCKED_FTW_VALUE

    total_phase = 0
    previous_phase = samples[0][1]
    for _, sample_phase in list(samples)[1:]:
        total_phase += signed_phase_delta(sample_phase, previous_phase)
        previous_phase = sample_phase
    elapsed = now - samples[0][0]
    phase_rate = total_phase / max(0.001, elapsed)
    if abs(phase_rate) < rate_threshold:
        return 0, phase_rate, LOCKED_FTW_VALUE

    delta_ftw = 1 if phase_rate > 0.0 else -1
    LOCKED_FTW_VALUE += delta_ftw
    FTW_AUTO_ADJUST_COUNT += 1
    FTW_AUTO_ADJUST_TOTAL += delta_ftw
    samples.clear()
    return delta_ftw, phase_rate, LOCKED_FTW_VALUE


def effective_ftw_offset(frequency_hz: int) -> int:
    if (
        LOCKED_FTW_VALUE is not None
        and LOCKED_FTW_FREQUENCY == frequency_hz
    ):
        return LOCKED_FTW_VALUE - int(round(frequency_hz * FTW_SCALE))
    if (
        LOCKED_FTW_VALUE is not None
        and LOCKED_FTW_FREQUENCY is not None
        and frequency_hz == 2 * LOCKED_FTW_FREQUENCY
    ):
        doubled_ftw = 2 * LOCKED_FTW_VALUE
        return doubled_ftw - int(round(frequency_hz * FTW_SCALE))
    return predicted_ftw_offset(frequency_hz)


def configure_phase_rate(rate_words_s: float) -> None:
    global PHASE_RATE_WORDS_S, PHASE_RATE_EPOCH
    PHASE_RATE_WORDS_S = float(rate_words_s)
    PHASE_RATE_EPOCH = time.monotonic()


def compensated_phase(phase_word: int) -> int:
    elapsed = max(0.0, time.monotonic() - PHASE_RATE_EPOCH)
    correction = int(round(PHASE_RATE_WORDS_S * elapsed))
    return (phase_word + correction) % 16384


def expect_prefix(lines: list[str], prefix: str, label: str) -> None:
    if not any(line.startswith(prefix) for line in lines):
        raise RuntimeError(f"{label} failed: {lines}")


def read_until_prefix(
    port: serial.Serial,
    prefix: str,
    timeout_s: float,
) -> list[str]:
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        line = port.readline()
        if not line:
            continue
        decoded = line.decode("utf-8", "replace").strip()
        if "MCU READY" in decoded:
            stop_after_mcu_reset(port)
            raise MCUResetDetected(
                "MCU_RESET_DETECTED: release DDS and return button service idle"
            )
        lines.append(decoded)
        if decoded.startswith(prefix):
            break
    return lines


def apply_output(
    port: serial.Serial,
    frequency_hz: int,
    amplitude: int,
    phase_word: int,
) -> int:
    base_ftw = int(round(frequency_hz * FTW_SCALE))
    if (
        LOCKED_FTW_VALUE is not None
        and LOCKED_FTW_FREQUENCY == frequency_hz
    ):
        ftw = LOCKED_FTW_VALUE
        offset = ftw - base_ftw
    elif (
        LOCKED_FTW_VALUE is not None
        and LOCKED_FTW_FREQUENCY is not None
        and frequency_hz == 2 * LOCKED_FTW_FREQUENCY
    ):
        # For a 2:1 Lissajous trace the output must stay exactly twice the
        # already measured input.  Doubling the locked tuning word preserves
        # that ratio and also doubles the calibrated clock correction.
        ftw = 2 * LOCKED_FTW_VALUE
        offset = ftw - base_ftw
    else:
        offset = predicted_ftw_offset(frequency_hz)
        ftw = base_ftw + offset
    controlled_phase = compensated_phase(phase_word)
    send(port, f"fopt {ftw} {amplitude} {controlled_phase}")
    replies = read_until_prefix(port, "OK fopt ", 0.10)
    if not any(line.startswith("OK fopt ") for line in replies):
        send(port, f"fopt {ftw} {amplitude} {phase_word % 16384}")
        replies = read_until_prefix(port, "OK fopt ", 0.16)
    expect_prefix(replies, "OK fopt ", "fopt")
    return offset


def mask_geometry(mask: np.ndarray) -> dict[str, float | int]:
    """Measure the trace in normalized X/Y and +/-45 degree coordinates.

    The camera view is not square and the scope X/Y gains can differ, so the
    trace is first normalized by its robust horizontal and vertical spans.
    In that coordinate system ``u`` and ``v`` are the two diagonal rulers.
    A line must be thin *and* have no gap across one ruler; a circle must have
    equal diagonal diameters and nearly constant radius.
    """
    mask = cv2.morphologyEx(
        (mask > 0).astype(np.uint8),
        cv2.MORPH_CLOSE,
        np.ones((3, 3), np.uint8),
    )
    ys, xs = np.nonzero(mask)
    if xs.size < 250:
        return {"valid": 0, "pixels": int(xs.size)}

    x_values = xs.astype(float)
    y_values = ys.astype(float)
    points = np.vstack((x_values, y_values))
    correlation = float(np.corrcoef(points)[0, 1])
    covariance = np.cov(points)
    eigenvalues = np.linalg.eigvalsh(covariance)
    axis_ratio = math.sqrt(
        max(float(eigenvalues[0]), 0.0)
        / max(float(eigenvalues[1]), 1.0e-9)
    )

    x0, x1 = np.percentile(xs, (1.0, 99.0))
    y0, y1 = np.percentile(ys, (1.0, 99.0))
    center_x = 0.5 * float(x0 + x1)
    center_y = 0.5 * float(y0 + y1)
    scale_x = max(1.0, 0.5 * float(x1 - x0))
    scale_y = max(1.0, 0.5 * float(y1 - y0))

    # Positive Y points upwards here.  In normalized coordinates the two
    # rulers are true +/-45 degree lines even if the photographed screen is
    # slightly stretched.
    x_normalized = (x_values - center_x) / scale_x
    y_normalized = (center_y - y_values) / scale_y
    root_two = math.sqrt(2.0)
    diagonal_plus = (x_normalized + y_normalized) / root_two
    diagonal_minus = (x_normalized - y_normalized) / root_two

    def diagonal_line_error(
        parallel: np.ndarray,
        perpendicular: np.ndarray,
    ) -> tuple[float, float, float, float]:
        p1, p99 = np.percentile(parallel, (1.0, 99.0))
        span = max(1.0e-6, float(p99 - p1))
        centre = 0.5 * float(p1 + p99)
        middle = perpendicular[
            np.abs(parallel - centre) <= 0.32 * span
        ]
        if middle.size < 40:
            middle = perpendicular
        low, high = np.percentile(middle, (5.0, 95.0))
        width90 = float(high - low) / span

        # Split the perpendicular samples into two 1-D clusters.  For one
        # thick line the inner quantiles touch; for a narrow ellipse they are
        # the two visible borders and leave a positive black gap.  This stays
        # correct even when one border contains many more phosphor pixels.
        centre_a, centre_b = [float(v) for v in np.percentile(middle, (25, 75))]
        for _ in range(6):
            threshold = 0.5 * (centre_a + centre_b)
            cluster_a = middle[middle <= threshold]
            cluster_b = middle[middle > threshold]
            if cluster_a.size < 10 or cluster_b.size < 10:
                break
            centre_a = float(np.median(cluster_a))
            centre_b = float(np.median(cluster_b))
        if cluster_a.size >= 10 and cluster_b.size >= 10:
            inner_a = float(np.percentile(cluster_a, 90.0))
            inner_b = float(np.percentile(cluster_b, 10.0))
            cluster_gap = max(0.0, inner_b - inner_a) / span
        else:
            cluster_gap = width90

        centre_value = 0.5 * (float(low) + float(high))
        centre_band = max(0.004 * span, 0.08 * float(high - low))
        centre_fill = float(
            np.mean(np.abs(middle - centre_value) <= centre_band)
        )
        error = width90 + 2.0 * cluster_gap
        return error, width90, cluster_gap, centre_fill

    plus_error, plus_width, plus_gap, plus_fill = diagonal_line_error(
        diagonal_plus, diagonal_minus
    )
    minus_error, minus_width, minus_gap, minus_fill = diagonal_line_error(
        diagonal_minus, diagonal_plus
    )
    if plus_error <= minus_error:
        line_orientation = 1
        line_error = plus_error
        line_width = plus_width
        line_gap = plus_gap
        line_center_fill = plus_fill
    else:
        line_orientation = -1
        line_error = minus_error
        line_width = minus_width
        line_gap = minus_gap
        line_center_fill = minus_fill

    plus_low, plus_high = np.percentile(diagonal_plus, (1.0, 99.0))
    minus_low, minus_high = np.percentile(diagonal_minus, (1.0, 99.0))
    diameter_plus = float(plus_high - plus_low)
    diameter_minus = float(minus_high - minus_low)
    diameter_mean = max(1.0e-6, 0.5 * (diameter_plus + diameter_minus))
    diagonal_thickness = min(diameter_plus, diameter_minus) / max(
        diameter_plus, diameter_minus, 1.0e-6
    )
    # This is the literal two-ruler distance test: a closed diagonal has one
    # full-length diameter and one trace-thickness diameter.  It remains
    # sensitive when the local branch-width estimate becomes nearly flat.
    line_width = diagonal_thickness
    line_error = diagonal_thickness + 2.0 * line_gap
    diameter_error = abs(diameter_plus - diameter_minus) / diameter_mean
    plus_symmetry = abs(float(plus_high + plus_low)) / max(diameter_plus, 1.0e-6)
    minus_symmetry = abs(float(minus_high + minus_low)) / max(diameter_minus, 1.0e-6)
    radial = np.sqrt(x_normalized * x_normalized + y_normalized * y_normalized)
    radial_median = max(1.0e-6, float(np.median(radial)))
    radial_mad = float(np.median(np.abs(radial - radial_median))) / radial_median
    circle_error = (
        diameter_error
        + 0.35 * (plus_symmetry + minus_symmetry)
        + 1.8 * radial_mad
    )

    left = max(0, int(math.floor(x0)) - 3)
    right = min(mask.shape[1], int(math.ceil(x1)) + 4)
    top = max(0, int(math.floor(y0)) - 3)
    bottom = min(mask.shape[0], int(math.ceil(y1)) + 4)
    crop = mask[top:bottom, left:right] > 0
    if crop.size == 0:
        return {"valid": 0, "pixels": int(xs.size)}

    def mirror_iou(other: np.ndarray) -> float:
        # A photographed phosphor trace is quantized into small horizontal
        # steps.  Exact pixel IoU therefore reports almost zero even for a
        # visually symmetric figure-eight.  Score each trace pixel by its
        # distance to the mirrored trace, with about 2% of the screen span as
        # geometric tolerance.  The name is kept for result compatibility.
        inverse = (~other).astype(np.uint8)
        distance = cv2.distanceTransform(inverse, cv2.DIST_L2, 3)
        tolerance = max(4.0, 0.02 * float(min(crop.shape)))
        samples = distance[crop]
        if samples.size == 0:
            return 0.0
        return float(np.mean(np.exp(-np.square(samples / tolerance))))

    horizontal_iou = mirror_iou(np.fliplr(crop))
    vertical_iou = mirror_iou(np.flipud(crop))
    height, width = crop.shape
    half_w = max(2, int(round(width * 0.035)))
    half_h = max(2, int(round(height * 0.035)))
    cx, cy = width // 2, height // 2
    center = crop[
        max(0, cy - half_h) : min(height, cy + half_h + 1),
        max(0, cx - half_w) : min(width, cx + half_w + 1),
    ]
    center_density = float(np.mean(center)) if center.size else 0.0
    crossing_samples = y_normalized[np.abs(x_normalized) <= 0.035]
    if crossing_samples.size >= 20:
        absolute_crossing = np.abs(crossing_samples)
        near_limit = float(np.percentile(absolute_crossing, 20.0))
        near_crossing = crossing_samples[absolute_crossing <= near_limit]
        center_crossing_y = float(np.median(near_crossing))
        # A low quantile rejects isolated threshold noise but follows the
        # actual central crossing instead of the much denser outer branches.
        center_crossing_error = float(
            np.percentile(absolute_crossing, 10.0)
        )
    else:
        center_crossing_y = 0.0
        center_crossing_error = 1.0
    left_pixels = int(np.count_nonzero(crop[:, :cx]))
    right_pixels = int(np.count_nonzero(crop[:, cx:]))
    lobe_imbalance = abs(left_pixels - right_pixels) / float(
        max(1, left_pixels + right_pixels)
    )

    return {
        "valid": 1,
        "pixels": int(xs.size),
        "axis_ratio": axis_ratio,
        "correlation": correlation,
        "center_x": center_x,
        "center_y": center_y,
        "scale_x": scale_x,
        "scale_y": scale_y,
        "diameter_plus": diameter_plus,
        "diameter_minus": diameter_minus,
        "diameter_error": diameter_error,
        "radial_mad": radial_mad,
        "circle_error": circle_error,
        "line_orientation": line_orientation,
        "line_error": line_error,
        "line_width": line_width,
        "diagonal_thickness": diagonal_thickness,
        "line_gap": line_gap,
        "line_center_fill": line_center_fill,
        "line_plus_error": plus_error,
        "line_minus_error": minus_error,
        "width": float(x1 - x0),
        "height": float(y1 - y0),
        "horizontal_iou": horizontal_iou,
        "vertical_iou": vertical_iou,
        "center_density": center_density,
        "center_crossing_y": center_crossing_y,
        "center_crossing_error": center_crossing_error,
        "lobe_imbalance": lobe_imbalance,
    }


def sample_output(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    phase_word: int,
    settle_s: float,
    frame_count: int = 2,
    discard_frames: int = 1,
) -> tuple[dict[str, float | int], np.ndarray]:
    apply_output(port, frequency_hz, amplitude, phase_word)
    time.sleep(settle_s)
    for _ in range(discard_frames):
        fetch_jpeg(camera_url)
    samples: list[tuple[dict[str, float | int], np.ndarray]] = []
    for _ in range(frame_count):
        frame = fetch_jpeg(camera_url)
        mask = trace_mask(settings.roi.crop(frame), settings)
        geometry = mask_geometry(mask)
        if geometry.get("valid"):
            samples.append((geometry, frame))
        time.sleep(0.012)
    if not samples:
        raise RuntimeError("no valid trace after DDS shape update")
    samples.sort(key=lambda item: int(item[0]["pixels"]))
    return samples[len(samples) // 2]


def shape_score(kind: str, geometry: dict[str, float | int]) -> float:
    if not geometry.get("valid"):
        return -1.0e9
    ratio = float(geometry["axis_ratio"])
    pixels = int(geometry["pixels"])
    dense_penalty = max(0.0, (pixels - 100000) / 100000.0)
    if kind == "line":
        return -float(geometry["line_error"]) - 2.0 * dense_penalty
    if kind == "circle":
        return -float(geometry["circle_error"]) - 2.0 * dense_penalty
    if kind == "infinity":
        horizontal = float(geometry["horizontal_iou"])
        vertical = float(geometry["vertical_iou"])
        return (
            -3.0 * float(geometry["center_crossing_error"])
            + 0.20 * (horizontal + vertical)
            + 0.50 * float(geometry["center_density"])
            - 0.50 * float(geometry["lobe_imbalance"])
            - 2.0 * dense_penalty
        )
    raise ValueError(f"unknown shape kind: {kind}")


def scan_diagnostic(
    phase: int,
    score: float,
    geometry: dict[str, float | int],
) -> dict[str, float | int]:
    return {
        "phase_word": phase,
        "score": round(score, 6),
        "axis_ratio": round(float(geometry["axis_ratio"]), 6),
        "line_error": round(float(geometry["line_error"]), 6),
        "line_gap": round(float(geometry["line_gap"]), 6),
        "line_orientation": int(geometry["line_orientation"]),
        "diameter_plus": round(float(geometry["diameter_plus"]), 6),
        "diameter_minus": round(float(geometry["diameter_minus"]), 6),
        "circle_error": round(float(geometry["circle_error"]), 6),
        "center_crossing_error": round(
            float(geometry["center_crossing_error"]), 6
        ),
        "center_density": round(float(geometry["center_density"]), 6),
        "pixels": int(geometry["pixels"]),
    }


def phase_search(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    kind: str,
    center_phase: int | None = None,
    settle_s: float = 0.10,
) -> tuple[int, dict[str, float | int], list[dict[str, float | int]]]:
    if center_phase is None:
        coarse = list(range(0, 16384, 1024))
    else:
        coarse = [
            (center_phase + delta) % 16384
            for delta in range(-1024, 1025, 256)
        ]

    evaluated: dict[int, tuple[float, dict[str, float | int]]] = {}

    def evaluate(phase_word: int) -> None:
        phase_word %= 16384
        if phase_word in evaluated:
            return
        geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            phase_word,
            settle_s,
        )
        evaluated[phase_word] = (shape_score(kind, geometry), geometry)

    for phase in coarse:
        evaluate(phase)
    best_phase = max(evaluated, key=lambda p: evaluated[p][0])
    for delta in range(-384, 385, 64):
        evaluate(best_phase + delta)
    best_phase = max(evaluated, key=lambda p: evaluated[p][0])
    best_score, best_geometry = evaluated[best_phase]
    diagnostics = [
        scan_diagnostic(phase, score, geometry)
        for phase, (score, geometry) in sorted(evaluated.items())
    ]
    best_geometry = dict(best_geometry)
    best_geometry["score"] = best_score
    return best_phase, best_geometry, diagnostics


def fast_phase_search(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    kind: str,
) -> tuple[int, dict[str, float | int], list[dict[str, float | int]]]:
    """Locate either diagonal, then refine using the two-ruler error."""
    if kind not in ("line", "circle"):
        raise ValueError("fast phase search supports line and circle")
    evaluated: dict[int, tuple[float, dict[str, float | int]]] = {}

    def evaluate(phase_word: int) -> None:
        phase_word %= 16384
        if phase_word in evaluated:
            return
        geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            phase_word,
            settle_s=0.16,
            frame_count=3,
            discard_frames=3,
        )
        evaluated[phase_word] = (shape_score(kind, geometry), geometry)

    for phase in (0, 4096, 8192, 12288):
        evaluate(phase)
    best_phase = max(evaluated, key=lambda phase: evaluated[phase][0])
    coarse_geometry = evaluated[best_phase][1]
    coarse_passed = (
        kind == "line" and float(coarse_geometry["line_error"]) <= 0.035
    ) or (
        kind == "circle" and float(coarse_geometry["circle_error"]) <= 0.14
    )
    if not coarse_passed:
        # Four quadrants put the target within 2048 phase words.  Five measured
        # refinements are fast enough that residual phase drift cannot make the
        # winning phase stale before it is applied.
        for step in (2048, 512, 128, 32, 8):
            evaluate(best_phase - step)
            evaluate(best_phase + step)
            best_phase = max(evaluated, key=lambda phase: evaluated[phase][0])

    best_score, best_geometry = evaluated[best_phase]
    best_geometry = dict(best_geometry)
    best_geometry["score"] = best_score
    diagnostics = [
        scan_diagnostic(phase, score, geometry)
        for phase, (score, geometry) in sorted(evaluated.items())
    ]
    return best_phase, best_geometry, diagnostics


def closed_loop_line_search(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    initial_phase: int,
    include_diagonal_seeds: bool = True,
) -> tuple[int, dict[str, float | int], list[dict[str, float | int]]]:
    """Collapse the current ellipse with a measured local phase search.

    Correlation is intentionally not converted into a phase error here.  Near
    a line, ``acos(abs(correlation))`` is both sign-ambiguous and very
    sensitive to a single camera/scope refresh.  A small bracketed search is
    slower by a few frames, but it measures the quantity we actually care
    about: the minor/major axis ratio.
    """
    evaluated: dict[int, tuple[float, dict[str, float | int]]] = {}

    def evaluate(phase_word: int) -> None:
        phase_word %= 16384
        if phase_word in evaluated:
            return
        geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            phase_word,
            settle_s=0.20,
            frame_count=3,
            discard_frames=3,
        )
        evaluated[phase_word] = (shape_score("line", geometry), geometry)

    best_phase = initial_phase % 16384
    evaluate(best_phase)
    if include_diagonal_seeds:
        # The frequency-lock stage normally parks on a sensitive circle.
        evaluate(best_phase - 4096)
        evaluate(best_phase + 4096)
        best_phase = max(evaluated, key=lambda phase: evaluated[phase][0])
        steps = (1024, 256, 64, 16, 4)
    else:
        # A final-frame retry must not jump to the opposite diagonal after a
        # good line has already been found; only repair local display latency.
        steps = (128, 32, 8, 2)
    for step in steps:
        evaluate(best_phase - step)
        evaluate(best_phase + step)
        best_phase = max(evaluated, key=lambda phase: evaluated[phase][0])
    best_score, best_geometry = evaluated[best_phase]
    best_geometry = dict(best_geometry)
    best_geometry["score"] = best_score
    diagnostics = [
        scan_diagnostic(phase, score, geometry)
        for phase, (score, geometry) in sorted(evaluated.items())
    ]
    return best_phase, best_geometry, diagnostics


def visual_line_servo(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    initial_phase: int,
    duration_s: float,
    stable_seconds: float,
    error_limit: float = 0.065,
    gap_limit: float = 0.015,
) -> tuple[int, dict[str, float | int], dict[str, float | int]]:
    """Continuously hold a diagonal line with camera-feedback phase steps.

    A small residual beat below one FTW LSB cannot be removed electrically.
    This loop follows it by probing one phase direction at a time and keeping
    only changes that reduce the two-diagonal line error.
    """
    current_phase = initial_phase % 16384
    current_geometry, _ = sample_output(
        port,
        camera_url,
        settings,
        frequency_hz,
        amplitude,
        current_phase,
        settle_s=0.16,
        frame_count=3,
        discard_frames=3,
    )
    direction = 1
    started = time.monotonic()
    deadline = started + duration_s if duration_s > 0.0 else math.inf
    stable_since: float | None = None
    stable_achieved = False
    best_error = float(current_geometry["line_error"])
    best_gap = float(current_geometry["line_gap"])
    # A duration of zero is the production mode: this loop is expected to run
    # for as long as the target trace is required.  Keep only a recent
    # telemetry window so a resident controller cannot grow without bound.
    error_history: deque[float] = deque(maxlen=600)
    gap_history: deque[float] = deque(maxlen=600)
    phase_history: deque[int] = deque(maxlen=600)
    drift_history: deque[tuple[float, int]] = deque(maxlen=80)
    phase_min = current_phase
    phase_max = current_phase
    accepted = 0
    rejected = 0
    cycles = 0

    while time.monotonic() < deadline:
        cycles += 1
        current_error = float(current_geometry["line_error"])
        if current_error > 0.20:
            step = 256
        elif current_error > 0.10:
            step = 128
        elif current_error > 0.065:
            step = 64
        elif current_error > 0.045:
            step = 16
        else:
            step = 4

        probe_phase = (current_phase + direction * step) % 16384
        probe_geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            probe_phase,
            settle_s=0.10,
            frame_count=3,
            discard_frames=2,
        )
        probe_error = float(probe_geometry["line_error"])
        probe_gap = float(probe_geometry["line_gap"])
        # A small tolerance prevents camera quantization from making the loop
        # chatter while still allowing it to follow slow monotonic phase drift.
        if probe_error <= current_error + 0.0015:
            current_phase = probe_phase
            current_geometry = probe_geometry
            accepted += 1
        else:
            direction *= -1
            rejected += 1
            current_geometry, _ = sample_output(
                port,
                camera_url,
                settings,
                frequency_hz,
                amplitude,
                current_phase,
                settle_s=0.07,
                frame_count=2,
                discard_frames=1,
            )

        now = time.monotonic()
        current_error = float(current_geometry["line_error"])
        current_gap = float(current_geometry["line_gap"])
        if current_error <= 0.10 and current_gap <= gap_limit:
            delta_ftw, phase_rate, adjusted_ftw = (
                auto_adjust_ftw_from_phase_drift(
                    drift_history, now, current_phase
                )
            )
            if delta_ftw != 0:
                apply_output(port, frequency_hz, amplitude, current_phase)
                current_geometry, _ = sample_output(
                    port,
                    camera_url,
                    settings,
                    frequency_hz,
                    amplitude,
                    current_phase,
                    settle_s=0.12,
                    frame_count=3,
                    discard_frames=2,
                )
                current_error = float(current_geometry["line_error"])
                current_gap = float(current_geometry["line_gap"])
                stable_since = None
                stable_achieved = False
                print(
                    "PHASE_SERVO_FTW "
                    + json.dumps(
                        {
                            "delta_ftw": delta_ftw,
                            "phase_rate_words_s": round(phase_rate, 3),
                            "locked_ftw": adjusted_ftw,
                            "ftw_offset": effective_ftw_offset(frequency_hz),
                            "phase_word": current_phase,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        else:
            drift_history.clear()
        best_error = min(best_error, current_error)
        best_gap = min(best_gap, current_gap)
        error_history.append(current_error)
        gap_history.append(current_gap)
        phase_history.append(current_phase)
        phase_min = min(phase_min, current_phase)
        phase_max = max(phase_max, current_phase)
        if current_error <= error_limit and current_gap <= gap_limit:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_seconds:
                if not stable_achieved:
                    stable_achieved = True
                    print(
                        "PHASE_SERVO_LOCKED "
                        + json.dumps(
                            {
                                "valid": 1,
                                "phase_word": current_phase,
                                "line_error": round(current_error, 6),
                                "line_gap": round(current_gap, 6),
                                "stable_s": round(now - stable_since, 3),
                                "elapsed_s": round(now - started, 3),
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
        else:
            stable_since = None

        if cycles == 1 or cycles % 5 == 0:
            print(
                "PHASE_SERVO "
                + json.dumps(
                    {
                        "cycle": cycles,
                        "phase_word": current_phase,
                        "step": step,
                        "direction": direction,
                        "line_error": round(current_error, 6),
                        "line_gap": round(current_gap, 6),
                        "stable_s": round(
                            now - stable_since if stable_since is not None else 0.0,
                            3,
                        ),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
        # For a finite test, keep controlling until the requested duration so
        # the user can observe the held trace after the 5-second criterion.

    apply_output(port, frequency_hz, amplitude, current_phase)
    errors = sorted(error_history) if error_history else [best_error]
    gaps = sorted(gap_history) if gap_history else [best_gap]
    summary = {
        "stable": int(stable_achieved),
        "duration_s": round(time.monotonic() - started, 3),
        "stable_required_s": stable_seconds,
        "cycles": cycles,
        "accepted": accepted,
        "rejected": rejected,
        "best_line_error": best_error,
        "median_line_error": statistics.median(errors),
        "p95_line_error": errors[min(len(errors) - 1, int(0.95 * len(errors)))],
        "best_line_gap": best_gap,
        "median_line_gap": statistics.median(gaps),
        "phase_min": phase_min,
        "phase_max": phase_max,
        "ftw_auto_adjust_count": FTW_AUTO_ADJUST_COUNT,
        "ftw_auto_adjust_total": FTW_AUTO_ADJUST_TOTAL,
        "locked_ftw": LOCKED_FTW_VALUE,
    }
    return current_phase, current_geometry, summary


def visual_circle_servo(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    initial_phase: int,
    duration_s: float,
    stable_seconds: float,
    error_limit: float = 0.22,
) -> tuple[int, dict[str, float | int], dict[str, float | int]]:
    """Continuously hold the quadrature phase that produces a circle."""
    started = time.monotonic()
    current_phase = initial_phase % 16384
    current_geometry, _ = sample_output(
        port,
        camera_url,
        settings,
        frequency_hz,
        amplitude,
        current_phase,
        settle_s=0.16,
        frame_count=3,
        discard_frames=3,
    )

    # Test the two local quadrature brackets before starting the continuous
    # hill climb.  On the live scope, +/-1024 words is large enough to escape
    # a line-like start but small enough that one side normally enters the
    # circle capture region.  This measured bracket is more reliable than
    # converting phosphor-thickened correlation directly into phase.
    bootstrap: list[dict[str, float | int]] = []
    base_phase = current_phase
    base_geometry = current_geometry
    base_error = float(base_geometry["circle_error"])
    correlation = max(-1.0, min(1.0, float(base_geometry["correlation"])))
    # Phosphor/display memory thickens the photographed ellipse and inflates
    # |correlation|.  Measurements on the live scope show that about 55% of
    # the ideal analytic correction lands inside the circle servo's capture
    # region while avoiding the observed twofold overshoot.
    phase_jump = int(
        round(
            0.55
            * abs(math.asin(correlation))
            * 16384.0
            / (2.0 * math.pi)
        )
    )
    phase_jump = min(4096, phase_jump)
    if base_error > error_limit:
        best_phase = base_phase
        best_geometry = base_geometry
        best_error = base_error
        for offset in (1024, -1024):
            candidate_phase = (base_phase + offset) % 16384
            candidate_geometry, _ = sample_output(
                port,
                camera_url,
                settings,
                frequency_hz,
                amplitude,
                candidate_phase,
                settle_s=0.22,
                frame_count=3,
                discard_frames=3,
            )
            candidate_error = float(candidate_geometry["circle_error"])
            bootstrap.append(
                {
                    "phase_word": candidate_phase,
                    "circle_error": round(candidate_error, 6),
                }
            )
            if candidate_error < best_error:
                best_phase = candidate_phase
                best_geometry = candidate_geometry
                best_error = candidate_error
            if best_error <= error_limit:
                break
        current_phase = best_phase
        current_geometry = best_geometry
        apply_output(port, frequency_hz, amplitude, current_phase)
    print(
        "CIRCLE_SERVO_BOOTSTRAP "
        + json.dumps(
            {
                "initial_phase": base_phase,
                "initial_error": round(base_error, 6),
                "correlation": round(correlation, 6),
                "phase_jump": phase_jump,
                "selected_phase": current_phase,
                "selected_error": round(
                    float(current_geometry["circle_error"]), 6
                ),
                "candidates": bootstrap,
                "elapsed_s": round(time.monotonic() - started, 3),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    direction = 1
    deadline = started + duration_s if duration_s > 0.0 else math.inf
    stable_since: float | None = None
    stable_achieved = False
    best_error = float(current_geometry["circle_error"])
    error_history: deque[float] = deque(maxlen=600)
    phase_history: deque[int] = deque(maxlen=600)
    drift_history: deque[tuple[float, int]] = deque(maxlen=80)
    phase_min = current_phase
    phase_max = current_phase
    accepted = 0
    rejected = 0
    cycles = 0

    while time.monotonic() < deadline:
        cycles += 1
        current_error = float(current_geometry["circle_error"])
        if current_error > 0.60:
            step = 512
        elif current_error > 0.35:
            step = 512
        elif current_error > 0.22:
            step = 256
        elif current_error > 0.16:
            step = 64
        else:
            step = 4

        probe_phase = (current_phase + direction * step) % 16384
        probe_geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            probe_phase,
            settle_s=0.10,
            frame_count=3,
            discard_frames=2,
        )
        probe_error = float(probe_geometry["circle_error"])
        # Circle error is noisier than line thickness because it includes a
        # radial statistic.  This tolerance lets the phase centre follow the
        # residual beat without reacting to every camera quantization step.
        if probe_error <= current_error + 0.004:
            current_phase = probe_phase
            current_geometry = probe_geometry
            accepted += 1
        else:
            direction *= -1
            rejected += 1
            current_geometry, _ = sample_output(
                port,
                camera_url,
                settings,
                frequency_hz,
                amplitude,
                current_phase,
                settle_s=0.07,
                frame_count=2,
                discard_frames=1,
            )

        now = time.monotonic()
        current_error = float(current_geometry["circle_error"])
        if current_error <= error_limit:
            delta_ftw, phase_rate, adjusted_ftw = (
                auto_adjust_ftw_from_phase_drift(
                    drift_history, now, current_phase
                )
            )
            if delta_ftw != 0:
                apply_output(port, frequency_hz, amplitude, current_phase)
                current_geometry, _ = sample_output(
                    port,
                    camera_url,
                    settings,
                    frequency_hz,
                    amplitude,
                    current_phase,
                    settle_s=0.12,
                    frame_count=3,
                    discard_frames=2,
                )
                current_error = float(current_geometry["circle_error"])
                stable_since = None
                stable_achieved = False
                print(
                    "CIRCLE_SERVO_FTW "
                    + json.dumps(
                        {
                            "delta_ftw": delta_ftw,
                            "phase_rate_words_s": round(phase_rate, 3),
                            "locked_ftw": adjusted_ftw,
                            "ftw_offset": effective_ftw_offset(frequency_hz),
                            "phase_word": current_phase,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        else:
            drift_history.clear()
        best_error = min(best_error, current_error)
        error_history.append(current_error)
        phase_history.append(current_phase)
        phase_min = min(phase_min, current_phase)
        phase_max = max(phase_max, current_phase)
        if current_error <= error_limit:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_seconds and not stable_achieved:
                stable_achieved = True
                print(
                    "CIRCLE_SERVO_LOCKED "
                    + json.dumps(
                        {
                            "valid": 1,
                            "phase_word": current_phase,
                            "circle_error": round(current_error, 6),
                            "diameter_error": round(
                                float(current_geometry["diameter_error"]), 6
                            ),
                            "radial_mad": round(
                                float(current_geometry["radial_mad"]), 6
                            ),
                            "stable_s": round(now - stable_since, 3),
                            "elapsed_s": round(now - started, 3),
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        else:
            stable_since = None

        if cycles == 1 or cycles % 5 == 0:
            print(
                "CIRCLE_SERVO "
                + json.dumps(
                    {
                        "cycle": cycles,
                        "phase_word": current_phase,
                        "step": step,
                        "direction": direction,
                        "circle_error": round(current_error, 6),
                        "diameter_error": round(
                            float(current_geometry["diameter_error"]), 6
                        ),
                        "radial_mad": round(
                            float(current_geometry["radial_mad"]), 6
                        ),
                        "stable_s": round(
                            now - stable_since if stable_since is not None else 0.0,
                            3,
                        ),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )

    apply_output(port, frequency_hz, amplitude, current_phase)
    errors = sorted(error_history) if error_history else [best_error]
    summary = {
        "stable": int(stable_achieved),
        "duration_s": round(time.monotonic() - started, 3),
        "stable_required_s": stable_seconds,
        "cycles": cycles,
        "accepted": accepted,
        "rejected": rejected,
        "best_circle_error": best_error,
        "median_circle_error": statistics.median(errors),
        "p95_circle_error": errors[
            min(len(errors) - 1, int(0.95 * len(errors)))
        ],
        "phase_min": phase_min,
        "phase_max": phase_max,
        "ftw_auto_adjust_count": FTW_AUTO_ADJUST_COUNT,
        "ftw_auto_adjust_total": FTW_AUTO_ADJUST_TOTAL,
        "locked_ftw": LOCKED_FTW_VALUE,
    }
    return current_phase, current_geometry, summary


def visual_infinity_servo(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    initial_phase: int,
    duration_s: float,
    stable_seconds: float,
    crossing_limit: float = 0.10,
    lobe_limit: float = 0.15,
) -> tuple[int, dict[str, float | int], dict[str, float | int]]:
    """Hold the central crossing of a symmetric 2:1 Lissajous figure."""
    current_phase = initial_phase % 16384
    current_geometry, _ = sample_output(
        port,
        camera_url,
        settings,
        frequency_hz,
        amplitude,
        current_phase,
        settle_s=0.16,
        frame_count=3,
        discard_frames=3,
    )
    direction = 1
    started = time.monotonic()
    deadline = started + duration_s if duration_s > 0.0 else math.inf
    stable_since: float | None = None
    stable_achieved = False
    best_error = float(current_geometry["center_crossing_error"])
    error_history: deque[float] = deque(maxlen=600)
    drift_history: deque[tuple[float, int]] = deque(maxlen=80)
    phase_min = current_phase
    phase_max = current_phase
    accepted = 0
    rejected = 0
    cycles = 0

    while time.monotonic() < deadline:
        cycles += 1
        current_error = float(current_geometry["center_crossing_error"])
        if current_error > 0.60:
            step = 512
        elif current_error > 0.30:
            step = 256
        elif current_error > 0.15:
            step = 128
        elif current_error > crossing_limit:
            step = 64
        elif current_error > 0.04:
            step = 16
        else:
            step = 4

        probe_phase = (current_phase + direction * step) % 16384
        probe_geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            probe_phase,
            settle_s=0.10,
            frame_count=3,
            discard_frames=2,
        )
        probe_error = float(probe_geometry["center_crossing_error"])
        if probe_error <= current_error + 0.008:
            current_phase = probe_phase
            current_geometry = probe_geometry
            accepted += 1
        else:
            direction *= -1
            rejected += 1
            current_geometry, _ = sample_output(
                port,
                camera_url,
                settings,
                frequency_hz,
                amplitude,
                current_phase,
                settle_s=0.07,
                frame_count=2,
                discard_frames=1,
            )

        now = time.monotonic()
        current_error = float(current_geometry["center_crossing_error"])
        current_lobe = float(current_geometry["lobe_imbalance"])
        if current_error <= 0.20 and current_lobe <= 0.25:
            delta_ftw, phase_rate, adjusted_ftw = (
                auto_adjust_ftw_from_phase_drift(
                    drift_history,
                    now,
                    current_phase,
                    rate_threshold=100.0,
                )
            )
            if delta_ftw != 0:
                apply_output(port, frequency_hz, amplitude, current_phase)
                current_geometry, _ = sample_output(
                    port,
                    camera_url,
                    settings,
                    frequency_hz,
                    amplitude,
                    current_phase,
                    settle_s=0.12,
                    frame_count=3,
                    discard_frames=2,
                )
                current_error = float(
                    current_geometry["center_crossing_error"]
                )
                current_lobe = float(current_geometry["lobe_imbalance"])
                stable_since = None
                stable_achieved = False
                print(
                    "INFINITY_SERVO_FTW "
                    + json.dumps(
                        {
                            "delta_input_ftw": delta_ftw,
                            "phase_rate_words_s": round(phase_rate, 3),
                            "input_locked_ftw": adjusted_ftw,
                            "output_ftw_offset": effective_ftw_offset(
                                frequency_hz
                            ),
                            "phase_word": current_phase,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        else:
            drift_history.clear()
        best_error = min(best_error, current_error)
        error_history.append(current_error)
        phase_min = min(phase_min, current_phase)
        phase_max = max(phase_max, current_phase)
        if current_error <= crossing_limit and current_lobe <= lobe_limit:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_seconds and not stable_achieved:
                stable_achieved = True
                print(
                    "INFINITY_SERVO_LOCKED "
                    + json.dumps(
                        {
                            "valid": 1,
                            "phase_word": current_phase,
                            "crossing_error": round(current_error, 6),
                            "lobe_imbalance": round(current_lobe, 6),
                            "stable_s": round(now - stable_since, 3),
                            "elapsed_s": round(now - started, 3),
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        else:
            stable_since = None

        if cycles == 1 or cycles % 5 == 0:
            print(
                "INFINITY_SERVO "
                + json.dumps(
                    {
                        "cycle": cycles,
                        "phase_word": current_phase,
                        "step": step,
                        "direction": direction,
                        "crossing_error": round(current_error, 6),
                        "crossing_y": round(
                            float(current_geometry["center_crossing_y"]), 6
                        ),
                        "lobe_imbalance": round(current_lobe, 6),
                        "stable_s": round(
                            now - stable_since if stable_since is not None else 0.0,
                            3,
                        ),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )

    apply_output(port, frequency_hz, amplitude, current_phase)
    errors = sorted(error_history) if error_history else [best_error]
    summary = {
        "stable": int(stable_achieved),
        "duration_s": round(time.monotonic() - started, 3),
        "stable_required_s": stable_seconds,
        "cycles": cycles,
        "accepted": accepted,
        "rejected": rejected,
        "best_crossing_error": best_error,
        "median_crossing_error": statistics.median(errors),
        "p95_crossing_error": errors[
            min(len(errors) - 1, int(0.95 * len(errors)))
        ],
        "phase_min": phase_min,
        "phase_max": phase_max,
        "ftw_auto_adjust_count": FTW_AUTO_ADJUST_COUNT,
        "ftw_auto_adjust_total": FTW_AUTO_ADJUST_TOTAL,
        "input_locked_ftw": LOCKED_FTW_VALUE,
    }
    return current_phase, current_geometry, summary


def confirm_line_stability(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    phase_word: int,
    duration_s: float,
    line_error_limit: float = 0.040,
    line_gap_limit: float = 0.008,
) -> dict[str, float | int]:
    """Hold one phase and require one gap-free diagonal for five seconds."""
    apply_output(port, frequency_hz, amplitude, phase_word)
    time.sleep(0.30)
    deadline = time.monotonic() + duration_s
    next_phase_update = time.monotonic() + 0.18
    errors: list[float] = []
    gaps: list[float] = []
    ratios: list[float] = []
    valid_frames = 0
    passing_frames = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if PHASE_RATE_WORDS_S != 0.0 and now >= next_phase_update:
            apply_output(port, frequency_hz, amplitude, phase_word)
            next_phase_update = now + 0.18
        frame = fetch_jpeg(camera_url)
        geometry = mask_geometry(trace_mask(settings.roi.crop(frame), settings))
        if geometry.get("valid"):
            valid_frames += 1
            error = float(geometry["line_error"])
            gap = float(geometry["line_gap"])
            ratio = float(geometry["axis_ratio"])
            errors.append(error)
            gaps.append(gap)
            ratios.append(ratio)
            if error <= line_error_limit and gap <= line_gap_limit:
                passing_frames += 1
        time.sleep(0.012)
    if not ratios:
        return {
            "stable": 0,
            "duration_s": duration_s,
            "valid_frames": 0,
        }
    errors.sort()
    gaps.sort()
    ratios.sort()
    p95_index = min(len(errors) - 1, int(0.95 * len(errors)))
    error_p95 = errors[p95_index]
    gap_p95 = gaps[p95_index]
    pass_fraction = passing_frames / float(valid_frames)
    return {
        "stable": int(
            error_p95 <= line_error_limit
            and gap_p95 <= line_gap_limit
            and pass_fraction >= 0.95
        ),
        "duration_s": duration_s,
        "valid_frames": valid_frames,
        "line_error_limit": line_error_limit,
        "line_gap_limit": line_gap_limit,
        "line_error_min": errors[0],
        "line_error_median": statistics.median(errors),
        "line_error_p95": error_p95,
        "line_error_max": errors[-1],
        "line_gap_median": statistics.median(gaps),
        "line_gap_p95": gap_p95,
        "axis_ratio_min": ratios[0],
        "axis_ratio_median": statistics.median(ratios),
        "axis_ratio_max": ratios[-1],
        "pass_fraction": pass_fraction,
    }


def circle_amplitude_search(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    phase_word: int,
    nominal_amplitude: int,
    settle_s: float,
) -> tuple[int, dict[str, float | int]]:
    candidates = {
        max(40, min(1023, value))
        for value in (
            int(round(nominal_amplitude * 0.55)),
            int(round(nominal_amplitude * 0.70)),
            int(round(nominal_amplitude * 0.85)),
            nominal_amplitude,
            int(round(nominal_amplitude * 1.15)),
            int(round(nominal_amplitude * 1.35)),
            int(round(nominal_amplitude * 1.60)),
        )
    }
    results = []
    for amplitude in sorted(candidates):
        geometry, _ = sample_output(
            port,
            camera_url,
            settings,
            frequency_hz,
            amplitude,
            phase_word,
            settle_s,
        )
        results.append((shape_score("circle", geometry), amplitude, geometry))
    _, best_amplitude, best_geometry = max(results, key=lambda item: item[0])
    return best_amplitude, best_geometry


def draw_diagonal_overlay(
    frame: np.ndarray,
    settings: Settings,
    geometry: dict[str, float | int],
    kind: str,
) -> np.ndarray:
    """Draw the two normalized rulers and their measured distances."""
    annotated = frame.copy()
    roi = settings.roi
    center_x = int(round(roi.x + float(geometry["center_x"])))
    center_y = int(round(roi.y + float(geometry["center_y"])))
    scale_x = 0.94 * float(geometry["scale_x"])
    scale_y = 0.94 * float(geometry["scale_y"])

    plus_points = (
        (int(round(center_x - scale_x)), int(round(center_y + scale_y))),
        (int(round(center_x + scale_x)), int(round(center_y - scale_y))),
    )
    minus_points = (
        (int(round(center_x - scale_x)), int(round(center_y - scale_y))),
        (int(round(center_x + scale_x)), int(round(center_y + scale_y))),
    )
    orientation = int(geometry["line_orientation"])
    muted = (70, 150, 70)
    active = (0, 255, 0)
    plus_colour = active if kind == "line" and orientation > 0 else muted
    minus_colour = active if kind == "line" and orientation < 0 else muted
    if kind == "circle":
        plus_colour = active
        minus_colour = active
    cv2.line(annotated, *plus_points, plus_colour, 2, cv2.LINE_AA)
    cv2.line(annotated, *minus_points, minus_colour, 2, cv2.LINE_AA)
    cv2.circle(annotated, (center_x, center_y), 5, (0, 0, 255), -1)

    text_x = max(8, roi.x + 12)
    text_y = max(24, roi.y + 24)
    lines = [
        f"D+ {float(geometry['diameter_plus']):.3f}  "
        f"D- {float(geometry['diameter_minus']):.3f}",
        f"LINE err {float(geometry['line_error']):.4f}  "
        f"gap {float(geometry['line_gap']):.4f}",
        f"CIRCLE err {float(geometry['circle_error']):.4f}  "
        f"rad {float(geometry['radial_mad']):.4f}",
    ]
    for index, label in enumerate(lines):
        position = (text_x, text_y + 25 * index)
        cv2.putText(
            annotated,
            label,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            label,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated


def save_shape(
    port: serial.Serial,
    camera_url: str,
    settings: Settings,
    frequency_hz: int,
    amplitude: int,
    phase_word: int,
    output_path: str,
    kind: str,
) -> dict[str, float | int]:
    geometry, frame = sample_output(
        port,
        camera_url,
        settings,
        frequency_hz,
        amplitude,
        phase_word,
        0.35,
    )
    frame = draw_diagonal_overlay(frame, settings, geometry, kind)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if not cv2.imwrite(output_path, frame):
        raise RuntimeError(f"failed to save {output_path}")
    return geometry


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--url", default="http://127.0.0.1:8080/frame.jpg")
    parser.add_argument("--config", required=True)
    parser.add_argument("--frequency", type=int, required=True)
    parser.add_argument("--amplitude", type=int, default=489)
    parser.add_argument(
        "--locked-ftw",
        type=int,
        help="exact FTW returned by frequency lock; phase search preserves it",
    )
    parser.add_argument(
        "--phase-rate-words-s",
        type=float,
        default=0.0,
        help="continuous phase feed-forward that cancels sub-LSB residual beat",
    )
    parser.add_argument(
        "--visual-servo",
        action="store_true",
        help="continuously hold a line or circle using camera phase feedback",
    )
    parser.add_argument(
        "--direct-servo",
        action="store_true",
        help="skip the duplicate phase scan and let the adaptive servo acquire",
    )
    parser.add_argument(
        "--servo-duration",
        type=float,
        default=10.0,
        help="visual servo runtime; zero keeps the controller running forever",
    )
    parser.add_argument("--servo-error-limit", type=float, default=0.065)
    parser.add_argument("--servo-gap-limit", type=float, default=0.015)
    parser.add_argument("--servo-circle-error-limit", type=float, default=0.22)
    parser.add_argument("--servo-crossing-limit", type=float, default=0.10)
    parser.add_argument("--servo-lobe-limit", type=float, default=0.15)
    parser.add_argument("--output-dir", default="/home/bupt/vision/shapes")
    parser.add_argument("--settle", type=float, default=0.10)
    parser.add_argument(
        "--stable-seconds",
        type=float,
        default=5.0,
        help="continuous visual hold required for a target line",
    )
    parser.add_argument(
        "--initial-phase",
        type=int,
        help="current phase word from the lock stage for a short line loop",
    )
    parser.add_argument(
        "--target-only",
        choices=("line", "circle", "infinity"),
        help="search and leave only one requested shape",
    )
    parser.add_argument(
        "--leave",
        choices=("line", "circle", "infinity"),
        default="line",
    )
    args = parser.parse_args()

    if not 1000 <= args.frequency <= 100000:
        raise ValueError("input frequency must be 1 kHz..100 kHz")
    if not 1 <= args.amplitude <= 1023:
        raise ValueError("amplitude must be 1..1023")

    settings = Settings.load(args.config)
    configure_locked_ftw(args.frequency, args.locked_ftw)
    configure_phase_rate(
        0.0 if args.visual_servo else args.phase_rate_words_s
    )
    started = time.monotonic()
    with serial.Serial(args.serial, 115200, timeout=0.05) as port:
        port.reset_input_buffer()
        send(port, "fping")
        expect_prefix(read_lines(port, 0.30), "F_PONG", "MCU handshake")

        if args.target_only:
            target_frequency = (
                args.frequency * 2
                if args.target_only == "infinity"
                else args.frequency
            )
            direct_servo = bool(
                args.visual_servo
                and args.direct_servo
                and args.initial_phase is not None
                and args.target_only in ("line", "circle", "infinity")
            )
            if direct_servo:
                target_phase = args.initial_phase % 16384
                target_geometry = {"valid": 0}
                target_scan = []
            elif args.target_only == "line" and args.initial_phase is not None:
                target_phase, target_geometry, target_scan = (
                    closed_loop_line_search(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        args.initial_phase,
                    )
                )
            elif args.target_only in ("line", "circle"):
                target_phase, target_geometry, target_scan = fast_phase_search(
                    port,
                    args.url,
                    settings,
                    target_frequency,
                    args.amplitude,
                    args.target_only,
                )
            else:
                target_phase, target_geometry, target_scan = phase_search(
                    port,
                    args.url,
                    settings,
                    target_frequency,
                    args.amplitude,
                    args.target_only,
                    settle_s=max(0.06, args.settle),
                )
            target_path = os.path.join(
                args.output_dir,
                f"{args.target_only}_{args.frequency}_final.jpg",
            )
            if not direct_servo:
                target_geometry = save_shape(
                    port,
                    args.url,
                    settings,
                    target_frequency,
                    args.amplitude,
                    target_phase,
                    target_path,
                    args.target_only,
                )
            stability = None
            if args.visual_servo and args.target_only in (
                "line",
                "circle",
                "infinity",
            ):
                if args.target_only == "line":
                    target_phase, target_geometry, stability = visual_line_servo(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        target_phase,
                        max(0.0, args.servo_duration),
                        max(0.0, args.stable_seconds),
                        args.servo_error_limit,
                        args.servo_gap_limit,
                    )
                elif args.target_only == "circle":
                    target_phase, target_geometry, stability = visual_circle_servo(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        target_phase,
                        max(0.0, args.servo_duration),
                        max(0.0, args.stable_seconds),
                        args.servo_circle_error_limit,
                    )
                else:
                    target_phase, target_geometry, stability = visual_infinity_servo(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        target_phase,
                        max(0.0, args.servo_duration),
                        max(0.0, args.stable_seconds),
                        args.servo_crossing_limit,
                        args.servo_lobe_limit,
                    )
                if args.servo_duration > 0.0:
                    target_geometry = save_shape(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        target_phase,
                        target_path,
                        args.target_only,
                    )
            needs_retry = (
                args.target_only == "line"
                and not args.visual_servo
                and (
                    float(target_geometry["line_error"]) > 0.040
                    or float(target_geometry["line_gap"]) > 0.008
                )
            ) or (
                args.target_only == "circle"
                and float(target_geometry["circle_error"]) > 0.22
            )
            if needs_retry and args.target_only in ("line", "circle"):
                if args.target_only == "line":
                    target_phase, _, retry_scan = closed_loop_line_search(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        target_phase,
                        include_diagonal_seeds=False,
                    )
                else:
                    target_phase, _, retry_scan = phase_search(
                        port,
                        args.url,
                        settings,
                        target_frequency,
                        args.amplitude,
                        args.target_only,
                        settle_s=0.10,
                    )
                target_scan += retry_scan
                target_geometry = save_shape(
                    port,
                    args.url,
                    settings,
                    target_frequency,
                    args.amplitude,
                    target_phase,
                    target_path,
                    args.target_only,
                )
            if (
                stability is None
                and args.target_only == "line"
                and args.stable_seconds > 0.0
            ):
                stability = confirm_line_stability(
                    port,
                    args.url,
                    settings,
                    target_frequency,
                    args.amplitude,
                    target_phase,
                    args.stable_seconds,
                )
            geometry_valid = (
                args.target_only != "line"
                or (
                    args.visual_servo
                    and stability is not None
                    and bool(stability["stable"])
                )
                or (
                    not args.visual_servo
                    and float(target_geometry["line_error"]) <= 0.040
                    and float(target_geometry["line_gap"]) <= 0.008
                )
            ) and (
                args.target_only != "circle"
                or (
                    args.visual_servo
                    and stability is not None
                    and bool(stability["stable"])
                )
                or (
                    not args.visual_servo
                    and float(target_geometry["circle_error"]) <= 0.22
                )
            )
            stability_valid = stability is None or bool(stability["stable"])
            result = {
                "valid": int(geometry_valid and stability_valid),
                "shape": args.target_only,
                "input_frequency_hz": args.frequency,
                "output_frequency_hz": target_frequency,
                "ftw_offset": effective_ftw_offset(target_frequency),
                "locked_ftw": (
                    args.locked_ftw
                    if target_frequency == args.frequency
                    else (
                        2 * args.locked_ftw
                        if (
                            args.locked_ftw is not None
                            and target_frequency == 2 * args.frequency
                        )
                        else None
                    )
                ),
                "phase_rate_words_s": (
                    0.0 if args.visual_servo else args.phase_rate_words_s
                ),
                "visual_servo": int(args.visual_servo),
                "amplitude": args.amplitude,
                "phase_word": target_phase,
                "geometry": target_geometry,
                "stability": stability,
                "image": target_path,
                "elapsed_s": round(time.monotonic() - started, 3),
                "scan": target_scan,
            }
            print("SHAPE_TARGET_FINAL " + json.dumps(result, ensure_ascii=False))
            return 0

        line_phase, line_probe, line_scan = phase_search(
            port,
            args.url,
            settings,
            args.frequency,
            args.amplitude,
            "line",
            settle_s=max(0.06, args.settle),
        )
        line_path = os.path.join(
            args.output_dir, f"line_{args.frequency}.jpg"
        )
        line_geometry = save_shape(
            port,
            args.url,
            settings,
            args.frequency,
            args.amplitude,
            line_phase,
            line_path,
            "line",
        )

        circle_seed = (line_phase + 4096) % 16384
        circle_phase, _, circle_scan = phase_search(
            port,
            args.url,
            settings,
            args.frequency,
            args.amplitude,
            "circle",
            center_phase=circle_seed,
            settle_s=max(0.06, args.settle),
        )
        circle_amplitude, _ = circle_amplitude_search(
            port,
            args.url,
            settings,
            args.frequency,
            circle_phase,
            args.amplitude,
            max(0.06, args.settle),
        )
        circle_phase, _, circle_refine = phase_search(
            port,
            args.url,
            settings,
            args.frequency,
            circle_amplitude,
            "circle",
            center_phase=circle_phase,
            settle_s=max(0.06, args.settle),
        )
        circle_path = os.path.join(
            args.output_dir, f"circle_{args.frequency}.jpg"
        )
        circle_geometry = save_shape(
            port,
            args.url,
            settings,
            args.frequency,
            circle_amplitude,
            circle_phase,
            circle_path,
            "circle",
        )

        infinity_frequency = args.frequency * 2
        infinity_phase, infinity_probe, infinity_scan = phase_search(
            port,
            args.url,
            settings,
            infinity_frequency,
            circle_amplitude,
            "infinity",
            settle_s=max(0.06, args.settle),
        )
        infinity_path = os.path.join(
            args.output_dir, f"infinity_{args.frequency}.jpg"
        )
        infinity_geometry = save_shape(
            port,
            args.url,
            settings,
            infinity_frequency,
            circle_amplitude,
            infinity_phase,
            infinity_path,
            "infinity",
        )

        # Searching the figure-eight temporarily runs the DDS at 2*f.  The
        # phase accumulator keeps advancing during that interval, so an old
        # phase word found at f cannot be reused after returning to f.  Always
        # reacquire the requested final shape at its final frequency and do
        # not change frequency again afterwards.
        if args.leave == "line":
            line_phase, line_probe, final_scan = phase_search(
                port,
                args.url,
                settings,
                args.frequency,
                args.amplitude,
                "line",
                settle_s=max(0.06, args.settle),
            )
            line_scan += final_scan
            line_geometry = save_shape(
                port,
                args.url,
                settings,
                args.frequency,
                args.amplitude,
                line_phase,
                line_path,
                "line",
            )
        elif args.leave == "circle":
            circle_phase, _, final_scan = phase_search(
                port,
                args.url,
                settings,
                args.frequency,
                circle_amplitude,
                "circle",
                settle_s=max(0.06, args.settle),
            )
            circle_scan += final_scan
            circle_geometry = save_shape(
                port,
                args.url,
                settings,
                args.frequency,
                circle_amplitude,
                circle_phase,
                circle_path,
                "circle",
            )
        else:
            infinity_phase, infinity_probe, final_scan = phase_search(
                port,
                args.url,
                settings,
                infinity_frequency,
                circle_amplitude,
                "infinity",
                settle_s=max(0.06, args.settle),
            )
            infinity_scan += final_scan
            infinity_geometry = save_shape(
                port,
                args.url,
                settings,
                infinity_frequency,
                circle_amplitude,
                infinity_phase,
                infinity_path,
                "infinity",
            )

        choices = {
            "line": (args.frequency, args.amplitude, line_phase),
            "circle": (
                args.frequency,
                circle_amplitude,
                circle_phase,
            ),
            "infinity": (
                infinity_frequency,
                circle_amplitude,
                infinity_phase,
            ),
        }
        leave_frequency, leave_amplitude, leave_phase = choices[args.leave]
        leave_offset = apply_output(
            port, leave_frequency, leave_amplitude, leave_phase
        )

    result = {
        "valid": 1,
        "input_frequency_hz": args.frequency,
        "elapsed_s": round(time.monotonic() - started, 3),
        "line": {
            "output_frequency_hz": args.frequency,
            "ftw_offset": effective_ftw_offset(args.frequency),
            "amplitude": args.amplitude,
            "phase_word": line_phase,
            "axis_ratio": round(float(line_geometry["axis_ratio"]), 6),
            "line_error": round(float(line_geometry["line_error"]), 6),
            "line_gap": round(float(line_geometry["line_gap"]), 6),
            "line_orientation": int(line_geometry["line_orientation"]),
            "image": line_path,
            "probe": line_probe,
            "scan": line_scan,
        },
        "circle": {
            "output_frequency_hz": args.frequency,
            "ftw_offset": effective_ftw_offset(args.frequency),
            "amplitude": circle_amplitude,
            "phase_word": circle_phase,
            "axis_ratio": round(float(circle_geometry["axis_ratio"]), 6),
            "diameter_plus": round(
                float(circle_geometry["diameter_plus"]), 6
            ),
            "diameter_minus": round(
                float(circle_geometry["diameter_minus"]), 6
            ),
            "circle_error": round(float(circle_geometry["circle_error"]), 6),
            "image": circle_path,
            "scan": circle_scan + circle_refine,
        },
        "infinity": {
            "output_frequency_hz": infinity_frequency,
            "ftw_offset": predicted_ftw_offset(infinity_frequency),
            "amplitude": circle_amplitude,
            "phase_word": infinity_phase,
            "horizontal_iou": round(
                float(infinity_geometry["horizontal_iou"]), 6
            ),
            "vertical_iou": round(
                float(infinity_geometry["vertical_iou"]), 6
            ),
            "lobe_imbalance": round(
                float(infinity_geometry["lobe_imbalance"]), 6
            ),
            "image": infinity_path,
            "probe": infinity_probe,
            "scan": infinity_scan,
        },
        "left_active": {
            "shape": args.leave,
            "frequency_hz": leave_frequency,
            "ftw_offset": leave_offset,
            "amplitude": leave_amplitude,
            "phase_word": leave_phase,
        },
    }
    print("SHAPE_FINAL " + json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except MCUResetDetected as error:
        print(str(error), flush=True)
        raise SystemExit(3) from None
