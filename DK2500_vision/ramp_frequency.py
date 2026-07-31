#!/usr/bin/env python3
"""Camera-only frequency measurement from the oscilloscope XY ramp trace.

The MSPM0 drives the oscilloscope Y input with a linear ramp.  The unknown
signal is connected only to oscilloscope X.  Inside a ramp window, X as a
function of Y is a sinusoid.  This program segments the green/yellow trace,
removes the nearly vertical ramp-reset line, reconstructs x(y), and fits a
sinusoid to estimate the number of input cycles in the known ramp window.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import time
import urllib.request
from dataclasses import dataclass

import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None


@dataclass
class Roi:
    x: int
    y: int
    w: int
    h: int

    def crop(self, frame: np.ndarray) -> np.ndarray:
        height, width = frame.shape[:2]
        x0 = max(0, min(self.x, width - 1))
        y0 = max(0, min(self.y, height - 1))
        x1 = max(x0 + 1, min(x0 + self.w, width))
        y1 = max(y0 + 1, min(y0 + self.h, height))
        return frame[y0:y1, x0:x1]


@dataclass
class Settings:
    roi: Roi
    hue_min: int = 20
    hue_max: int = 50
    sat_min: int = 40
    val_min: int = 80
    frequency_val_min: int = 40
    min_component_area: int = 18
    frequency_scale: float = 1.0

    @classmethod
    def load(cls, path: str) -> "Settings":
        with open(path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
        roi_raw = raw.get("frequency_roi", raw.get("roi", raw))
        return cls(
            roi=Roi(
                int(roi_raw["x"]),
                int(roi_raw["y"]),
                int(roi_raw["w"]),
                int(roi_raw["h"]),
            ),
            hue_min=int(raw.get("hue_min", 20)),
            hue_max=int(raw.get("hue_max", 50)),
            sat_min=int(raw.get("sat_min", 40)),
            val_min=int(raw.get("val_min", 80)),
            frequency_val_min=int(raw.get("frequency_val_min", 40)),
            min_component_area=int(raw.get("frequency_min_component_area", 18)),
            frequency_scale=float(raw.get("frequency_scale", 1.0)),
        )


@dataclass
class Estimate:
    valid: bool
    frequency_hz: float
    cycles: float
    fit_score: float
    row_coverage: float
    amplitude_px: float
    reset_x: int | None
    reason: str = ""
    method: str = ""

    @property
    def confidence(self) -> float:
        if not self.valid:
            return 0.0
        fit = np.clip((self.fit_score - 0.35) / 0.55, 0.0, 1.0)
        coverage = np.clip((self.row_coverage - 0.35) / 0.55, 0.0, 1.0)
        amplitude = np.clip((self.amplitude_px - 8.0) / 45.0, 0.0, 1.0)
        return float(0.60 * fit + 0.25 * coverage + 0.15 * amplitude)


def fetch_jpeg(url: str, timeout: float = 2.0) -> np.ndarray:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = response.read()
    frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("camera JPEG decode failed")
    return frame


def trace_mask(roi_bgr: np.ndarray, settings: Settings) -> np.ndarray:
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [settings.hue_min, settings.sat_min, settings.val_min], dtype=np.uint8
    )
    upper = np.array([settings.hue_max, 255, 255], dtype=np.uint8)
    raw = cv2.inRange(hsv, lower, upper)

    raw = cv2.morphologyEx(
        raw,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )

    # The configured ROI intentionally contains some scope UI so that it is
    # easy to aim the camera.  The left measurement menu and the bottom channel
    # labels use the same yellow/cyan colours as the trace, so remove those
    # fixed UI strips before component selection.
    height, width = raw.shape
    raw[:, : int(0.16 * width)] = 0
    raw[int(0.88 * height) :, :] = 0

    # Group nearby phosphor dots first, then retain the original (undilated)
    # pixels belonging to the largest trace-shaped group.  This handles the
    # DS1102Z-E's forced dot display without joining distant pieces of UI.
    grouping = cv2.dilate(
        raw,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
        iterations=1,
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (grouping > 0).astype(np.uint8), connectivity=8
    )
    clean = np.zeros_like(raw)
    components = []
    for label in range(1, count):
        x, y, w, h, area = [int(v) for v in stats[label]]
        if area < max(settings.min_component_area, 30):
            continue
        if w < max(12, int(0.05 * width)) or h < 18:
            continue
        components.append((label, area, x, y, w, h))

    if not components:
        return clean

    dominant = max(components, key=lambda item: item[1])
    dominant_label = dominant[0]
    clean[(labels == dominant_label) & (raw > 0)] = 255

    return cv2.morphologyEx(
        clean,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        iterations=1,
    )


def frequency_trace_mask(roi_bgr: np.ndarray, settings: Settings) -> np.ndarray:
    """Keep every dim phosphor segment needed by the coarse-frequency trace."""
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [
            max(0, settings.hue_min - 2),
            max(20, settings.sat_min // 2),
            settings.frequency_val_min,
        ],
        dtype=np.uint8,
    )
    upper = np.array([min(179, settings.hue_max + 5), 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        iterations=1,
    )


def _groups(values: np.ndarray, maximum_gap: int) -> list[list[int]]:
    result: list[list[int]] = []
    for raw_value in values:
        value = int(raw_value)
        if not result or value - result[-1][-1] > maximum_gap:
            result.append([value])
        else:
            result[-1].append(value)
    return result


def _edge_geometry(
    mask: np.ndarray,
) -> tuple[np.ndarray, int, int, float, int | None]:
    """Locate the two idle edge bands and the fixed vertical return."""
    height, width = mask.shape
    x0 = int(0.02 * width)
    x1 = max(x0 + 1, int(0.90 * width))
    row_counts = np.count_nonzero(mask[:, x0:x1], axis=1)
    usable_width = x1 - x0
    dense_threshold = max(
        int(0.45 * usable_width),
        int(0.62 * float(row_counts.max(initial=0))),
    )
    dense_groups = _groups(
        np.flatnonzero(row_counts >= dense_threshold), maximum_gap=4
    )

    top_group: list[int] | None = None
    bottom_group: list[int] | None = None
    if dense_groups:
        top_group = dense_groups[0]
        for group in reversed(dense_groups[1:]):
            if group[0] - top_group[-1] >= int(0.50 * height):
                bottom_group = group
                break

    ys, _ = np.where(mask > 0)
    if ys.size == 0:
        return np.zeros_like(mask), 0, 0, 0.0, None
    if top_group is None or bottom_group is None:
        low = int(np.percentile(ys, 2))
        high = int(np.percentile(ys, 98))
        top_group = [low]
        bottom_group = [high]

    active_y0 = min(height - 1, top_group[-1] + 5)
    active_y1 = max(active_y0, bottom_group[0] - 5)
    top_center = 0.5 * (top_group[0] + top_group[-1])
    bottom_center = 0.5 * (bottom_group[0] + bottom_group[-1])
    full_span = max(1.0, bottom_center - top_center)

    clean = mask.copy()
    clean[:active_y0, :] = 0
    clean[active_y1 + 1 :, :] = 0

    column_counts = np.count_nonzero(clean, axis=0)
    reset_x = int(np.argmax(column_counts))
    if column_counts[reset_x] < max(18, int(0.18 * (active_y1 - active_y0 + 1))):
        reset_x = None
    elif reset_x is not None:
        clean[:, max(0, reset_x - 3) : min(width, reset_x + 4)] = 0

    return clean, active_y0, active_y1, full_span, reset_x


def _reconstruct_all_components(
    mask: np.ndarray, y0: int, y1: int
) -> tuple[np.ndarray, np.ndarray, float]:
    rows: list[float] = []
    values: list[float] = []
    width = mask.shape[1]
    for y in range(y0, y1 + 1):
        xs = np.flatnonzero(mask[y] > 0)
        xs = xs[(xs >= int(0.01 * width)) & (xs <= int(0.96 * width))]
        if xs.size == 0:
            continue
        rows.append(float(y))
        values.append(float(np.median(xs)))

    if len(rows) < 25:
        return np.array([]), np.array([]), 0.0
    full_rows = np.arange(y0, y1 + 1, dtype=np.float64)
    curve = np.interp(
        full_rows,
        np.asarray(rows, dtype=np.float64),
        np.asarray(values, dtype=np.float64),
    )
    curve = cv2.GaussianBlur(
        curve.reshape(-1, 1), (1, 9), sigmaX=0, sigmaY=2.0
    ).reshape(-1)
    coverage = len(rows) / max(1, y1 - y0 + 1)
    return full_rows, curve, float(coverage)


def _midline_spacing_cycles(
    mask: np.ndarray, y0: int, y1: int, full_span: float
) -> tuple[float, float, int | None, np.ndarray]:
    ys, xs = np.where(mask > 0)
    if ys.size < 50:
        return 0.0, 0.0, None, np.array([])
    x_low, x_high = np.percentile(xs, [1, 99])
    x_span = float(x_high - x_low)
    if x_span < 40.0:
        return 0.0, 0.0, None, np.array([])
    x_center = 0.5 * (float(x_low) + float(x_high))

    best_score = float("inf")
    best_cycles = 0.0
    best_x: int | None = None
    best_centers = np.array([])
    for half_width in (4, 8, 16, 24, 32):
        for candidate in np.linspace(
            x_center - 0.18 * x_span,
            x_center + 0.18 * x_span,
            37,
        ):
            x = int(round(candidate))
            xa = max(0, x - half_width)
            xb = min(mask.shape[1], x + half_width + 1)
            hits = (
                np.flatnonzero(
                    np.count_nonzero(mask[y0 : y1 + 1, xa:xb], axis=1) > 0
                )
                + y0
            )
            crossing_groups = _groups(hits, maximum_gap=10)
            centers = np.asarray(
                [
                    float(np.mean(group))
                    for group in crossing_groups
                    if 1 <= len(group) <= 35
                ],
                dtype=np.float64,
            )
            if centers.size < 3:
                continue
            gaps = np.diff(centers)
            median_gap = float(np.median(gaps))
            if median_gap < 4.0:
                continue
            regularity = float(
                np.median(np.abs(gaps - median_gap)) / median_gap
            )
            observed_span = float(centers[-1] - centers[0]) / full_span
            score = regularity + max(0.0, 0.65 - observed_span)
            if score < best_score:
                best_score = score
                best_cycles = full_span / (2.0 * median_gap)
                best_x = x
                best_centers = centers

    if best_x is None:
        return 0.0, 0.0, None, np.array([])
    quality = float(np.clip(1.0 - 4.0 * best_score, 0.0, 1.0))
    return best_cycles, quality, best_x, best_centers


def _find_reset_column(mask: np.ndarray, y0: int, y1: int) -> int | None:
    local = mask[y0 : y1 + 1]
    if local.size == 0:
        return None
    counts = np.count_nonzero(local, axis=0)
    reset_x = int(np.argmax(counts))
    height = max(1, y1 - y0 + 1)
    if counts[reset_x] < max(12, int(0.45 * height)):
        return None
    return reset_x


def _reconstruct_curve(
    mask: np.ndarray, reset_x: int | None
) -> tuple[np.ndarray, np.ndarray, float]:
    ys_all, xs_all = np.where(mask > 0)
    if ys_all.size < 50:
        return np.array([]), np.array([]), 0.0
    y0, y1 = int(ys_all.min()), int(ys_all.max())
    if y1 - y0 < 25:
        return np.array([]), np.array([]), 0.0

    rows = []
    x_values = []
    for y in range(y0, y1 + 1):
        xs = np.flatnonzero(mask[y] > 0)
        if xs.size == 0:
            continue
        if reset_x is None:
            x = float(np.median(xs))
        else:
            keep = xs[np.abs(xs - reset_x) > 3]
            if keep.size == 0:
                continue
            # Dot-mode XY often draws short horizontal runs between adjacent
            # samples.  The endpoint farthest from the reset line follows the
            # sinusoidal envelope much better than the row median.
            x = float(keep[np.argmax(np.abs(keep - reset_x))])
        rows.append(float(y))
        x_values.append(x)

    if len(rows) < 25:
        return np.array([]), np.array([]), 0.0

    rows_arr = np.asarray(rows, dtype=np.float64)
    values_arr = np.asarray(x_values, dtype=np.float64)
    full_rows = np.arange(y0, y1 + 1, dtype=np.float64)
    interp = np.interp(full_rows, rows_arr, values_arr)
    coverage = len(rows) / max(1, y1 - y0 + 1)

    # Remove isolated endpoint jumps and camera pixel noise.
    smooth = cv2.GaussianBlur(
        interp.reshape(-1, 1), (1, 0), sigmaX=0, sigmaY=2.0
    ).reshape(-1)
    return full_rows, smooth, float(coverage)


def _crossing_cycle_seed(values: np.ndarray) -> tuple[float | None, int]:
    """Estimate cycles from robust midline crossings.

    Camera dot mode turns parts of the sinusoid into short horizontal bars.
    Their exact shape hurts a global sine fit, but the spacing between midline
    crossings remains very stable.
    """
    count = values.size
    if count < 25:
        return None, 0
    low_value = float(np.percentile(values, 10))
    high_value = float(np.percentile(values, 90))
    span = high_value - low_value
    if span < 8.0:
        return None, 0
    midline = 0.5 * (low_value + high_value)
    hysteresis = max(1.5, 0.06 * span)

    state = 0
    crossings: list[float] = []
    previous_index = 0
    previous_value = float(values[0] - midline)
    for index, sample in enumerate(values):
        value = float(sample - midline)
        new_state = 1 if value > hysteresis else (-1 if value < -hysteresis else state)
        if state != 0 and new_state != state:
            denominator = value - previous_value
            fraction = (
                -previous_value / denominator if abs(denominator) > 1e-9 else 0.0
            )
            crossing = previous_index + float(np.clip(fraction, 0.0, 1.0))
            if not crossings or crossing - crossings[-1] >= 3.0:
                crossings.append(crossing)
        if abs(value) <= hysteresis:
            previous_index = index
            previous_value = value
        state = new_state

    if len(crossings) < 3:
        return None, len(crossings)
    half_periods = np.diff(np.asarray(crossings))
    median_half_period = float(np.median(half_periods))
    if median_half_period <= 1.0:
        return None, len(crossings)
    return count / (2.0 * median_half_period), len(crossings)


def _fit_cycles(values: np.ndarray) -> tuple[float, float, float]:
    count = values.size
    if count < 25:
        return 0.0, 0.0, 0.0
    t = np.linspace(0.0, 1.0, count)
    centered = values - np.median(values)
    amplitude_px = 0.5 * (
        float(np.percentile(centered, 95)) - float(np.percentile(centered, 5))
    )
    total = float(np.sum((centered - np.mean(centered)) ** 2))
    if total < 1e-6 or amplitude_px < 4.0:
        return 0.0, 0.0, amplitude_px

    # A coarse FFT seed narrows the expensive continuous least-squares search.
    windowed = (centered - np.mean(centered)) * np.hanning(count)
    spectrum = np.abs(np.fft.rfft(windowed))
    spectrum[:1] = 0.0
    fft_index = int(np.argmax(spectrum))
    fft_cycles = float(fft_index)
    maximum_cycles = min(80.0, max(2.0, count / 4.0))
    crossing_seed, crossing_count = _crossing_cycle_seed(values)
    if crossing_seed is not None:
        low = max(0.25, crossing_seed - 0.55)
        high = min(maximum_cycles, crossing_seed + 0.55)
    else:
        low = max(0.25, fft_cycles - 1.5)
        high = min(maximum_cycles, max(2.0, fft_cycles + 1.5))

    best_cycles = 0.0
    best_score = -1.0
    for cycles in np.linspace(low, high, int((high - low) * 80) + 1):
        omega = 2.0 * math.pi * cycles * t
        design = np.column_stack(
            (np.sin(omega), np.cos(omega), np.ones(count), t)
        )
        coeff, _, _, _ = np.linalg.lstsq(design, values, rcond=None)
        residual = values - design @ coeff
        score = 1.0 - float(np.sum(residual * residual)) / max(total, 1e-9)
        if score > best_score:
            best_score = score
            best_cycles = float(cycles)

    # With at least three crossings, their median spacing is more reliable
    # than the exact least-squares optimum on a dotted trace.  Accept the fit
    # refinement only when it stays close to that geometric estimate.
    if crossing_seed is not None and crossing_count >= 3:
        if abs(best_cycles - crossing_seed) > 0.28:
            best_cycles = crossing_seed

    return best_cycles, float(best_score), float(amplitude_px)


def analyze_frame(
    frame: np.ndarray,
    window_ms: float,
    settings: Settings,
    debug_path: str | None = None,
) -> Estimate:
    roi = settings.roi.crop(frame)
    raw_mask = frequency_trace_mask(roi, settings)
    mask, y0, y1, full_span, reset_x = _edge_geometry(raw_mask)
    ys, xs = np.where(mask > 0)
    if ys.size < 80:
        return Estimate(False, 0.0, 0.0, 0.0, 0.0, 0.0, None, "trace not found")

    rows, curve, coverage = _reconstruct_all_components(mask, y0, y1)
    fit_cycles = 0.0
    fit_score = 0.0
    amplitude_px = 0.5 * (
        float(np.percentile(xs, 95)) - float(np.percentile(xs, 5))
    )
    if curve.size:
        fit_cycles, fit_score, fitted_amplitude = _fit_cycles(curve)
        amplitude_px = max(amplitude_px, fitted_amplitude)
        if curve.size > 1:
            fit_cycles *= full_span / float(curve.size - 1)

    crossing_cycles, crossing_quality, crossing_x, crossing_rows = (
        _midline_spacing_cycles(mask, y0, y1, full_span)
    )
    if fit_score >= 0.45 and coverage >= 0.55 and fit_cycles > 0.0:
        cycles = fit_cycles
        method = "curve"
    elif crossing_quality >= 0.50 and crossing_cycles > 0.0:
        cycles = crossing_cycles
        fit_score = crossing_quality
        method = "crossing"
    else:
        return Estimate(
            False,
            0.0,
            0.0,
            max(fit_score, crossing_quality),
            coverage,
            amplitude_px,
            reset_x,
            "curve and crossing estimates rejected",
        )

    frequency_hz = cycles * 1000.0 / window_ms
    valid = (
        0.20 <= cycles <= 80.0
        and fit_score >= 0.30
        and coverage >= 0.25
        and amplitude_px >= 5.0
    )
    result = Estimate(
        valid,
        frequency_hz,
        cycles,
        fit_score,
        coverage,
        amplitude_px,
        reset_x,
        "" if valid else "low fit quality",
        method,
    )

    if debug_path:
        raw_debug_path = os.path.splitext(debug_path)[0] + "_raw.jpg"
        cv2.imwrite(
            raw_debug_path,
            roi,
            [int(cv2.IMWRITE_JPEG_QUALITY), 95],
        )
        debug = roi.copy()
        tint = np.zeros_like(debug)
        tint[:, :, 2] = mask
        debug = cv2.addWeighted(debug, 0.75, tint, 0.45, 0.0)
        if reset_x is not None:
            cv2.line(debug, (reset_x, y0), (reset_x, y1), (255, 0, 255), 2)
        cv2.line(debug, (0, y0), (debug.shape[1] - 1, y0), (0, 128, 255), 1)
        cv2.line(debug, (0, y1), (debug.shape[1] - 1, y1), (0, 128, 255), 1)
        if crossing_x is not None:
            cv2.line(
                debug,
                (crossing_x, y0),
                (crossing_x, y1),
                (255, 128, 0),
                1,
            )
            for crossing_y in crossing_rows:
                cv2.circle(
                    debug,
                    (crossing_x, int(round(crossing_y))),
                    3,
                    (0, 255, 255),
                    -1,
                )
        if rows.size:
            points = np.column_stack((curve, rows)).round().astype(np.int32)
            cv2.polylines(debug, [points], False, (255, 255, 0), 2)
        text = (
            f"{frequency_hz:.1f} Hz  cycles={cycles:.3f}  "
            f"{method} fit={fit_score:.3f} conf={result.confidence:.3f}"
        )
        cv2.putText(
            debug,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        os.makedirs(os.path.dirname(os.path.abspath(debug_path)), exist_ok=True)
        cv2.imwrite(debug_path, debug)

    return result


def print_estimate(result: Estimate, window_ms: float) -> None:
    print(
        json.dumps(
            {
                "valid": result.valid,
                "window_ms": window_ms,
                "frequency_hz": round(result.frequency_hz, 3),
                "cycles": round(result.cycles, 4),
                "fit_score": round(result.fit_score, 4),
                "row_coverage": round(result.row_coverage, 4),
                "amplitude_px": round(result.amplitude_px, 2),
                "confidence": round(result.confidence, 4),
                "reset_x": result.reset_x,
                "reason": result.reason,
                "method": result.method,
            },
            ensure_ascii=False,
        )
    )


def serial_send(port, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def serial_drain(port, duration: float = 0.15) -> list[str]:
    deadline = time.monotonic() + duration
    lines = []
    while time.monotonic() < deadline:
        raw = port.readline()
        if raw:
            lines.append(raw.decode("utf-8", "replace").strip())
    return lines


def measure_live(
    camera_url: str,
    serial_port: str,
    settings: Settings,
    windows_us: list[int],
    frames_per_window: int,
    debug_dir: str,
) -> Estimate:
    if serial is None:
        raise RuntimeError("pyserial is not installed")
    os.makedirs(debug_dir, exist_ok=True)
    candidates: list[Estimate] = []
    confirmation_windows = 0
    with serial.Serial(serial_port, 115200, timeout=0.08) as port:
        port.reset_input_buffer()
        serial_send(port, "fping")
        serial_drain(port, 0.4)
        serial_send(port, "fping")
        serial_drain(port, 0.25)
        try:
            for window_us in windows_us:
                window_ms = window_us / 1000.0
                serial_send(port, f"fwindow {window_us}")
                replies = serial_drain(port, 0.15)
                if not any(line.startswith("OK fwindow ") for line in replies):
                    serial_send(port, f"fwindow {window_us}")
                    replies = serial_drain(port, 0.15)
                if not any(line.startswith("OK fwindow ") for line in replies):
                    raise RuntimeError(
                        f"MCU rejected fwindow {window_us}: {' | '.join(replies)}"
                    )

                serial_send(port, "fping")
                time.sleep(0.50)
                # Do not score the first image after switching the DAC
                # window.  The camera endpoint may already be live while the
                # scope still displays the preceding window for a few frames.
                previous_frame: np.ndarray | None = fetch_jpeg(camera_url)
                time.sleep(0.10)
                window_results: list[Estimate] = []
                required_results = max(2, (frames_per_window + 1) // 2)
                max_attempts = frames_per_window + (4 if candidates else 0)
                for frame_index in range(max_attempts):
                    serial_send(port, "fping")
                    frame = fetch_jpeg(camera_url)
                    if previous_frame is not None and np.array_equal(
                        frame, previous_frame
                    ):
                        time.sleep(0.05)
                        frame = fetch_jpeg(camera_url)
                    previous_frame = frame
                    debug_path = os.path.join(
                        debug_dir, f"window_{window_us}us_{frame_index}.jpg"
                    )
                    result = analyze_frame(
                        frame, window_ms, settings, debug_path
                    )
                    print_estimate(result, window_ms)
                    if result.valid:
                        if candidates:
                            reference_frequency = candidates[-1].frequency_hz
                            stale_tolerance = max(
                                600.0, 0.35 * reference_frequency
                            )
                            if (
                                abs(result.frequency_hz - reference_frequency)
                                > stale_tolerance
                            ):
                                print(
                                    json.dumps(
                                        {
                                            "window_us": window_us,
                                            "discarded": True,
                                            "reason": "previous-window residue",
                                            "frequency_hz": round(
                                                result.frequency_hz, 3
                                            ),
                                        }
                                    )
                                )
                            else:
                                window_results.append(result)
                        else:
                            window_results.append(result)
                        if len(window_results) >= frames_per_window:
                            break
                    elif (
                        result.row_coverage >= 0.90
                        and not candidates
                        and not window_results
                    ):
                        # The trace has filled essentially every row: this
                        # window is too long.  Stop spending frames here, but
                        # still inspect the immediately adjacent 1-2-5 window;
                        # skipping it can miss the ideal five-cycle trace.
                        break
                    elif frame_index >= 1 and not window_results:
                        # Allow one retry for a transition/stale camera frame,
                        # then move on instead of spending the full frame
                        # budget on an unreadable window.
                        break
                    time.sleep(0.05)

                if len(window_results) >= required_results:
                    curve_results = [
                        item
                        for item in window_results
                        if item.method == "curve"
                    ]
                    clustering_results = (
                        curve_results
                        if len(curve_results) >= 2
                        else window_results
                    )
                    ordered = sorted(
                        clustering_results,
                        key=lambda item: item.frequency_hz,
                    )
                    center_frequency = float(
                        statistics.median(
                            item.frequency_hz for item in ordered
                        )
                    )
                    cluster_width = max(100.0, 0.01 * center_frequency)
                    clusters: list[list[Estimate]] = []
                    left = 0
                    for right in range(len(ordered)):
                        while (
                            ordered[right].frequency_hz
                            - ordered[left].frequency_hz
                            > cluster_width
                        ):
                            left += 1
                        group = ordered[left : right + 1]
                        if (
                            len(group) >= 2
                            and float(
                                np.mean([item.fit_score for item in group])
                            )
                            >= 0.90
                        ):
                            clusters.append(group)
                    if clusters:
                        # Persistence and phase smearing bias the fitted cycle
                        # count downward.  Prefer the highest well-supported
                        # cluster; isolated high outliers never form a cluster.
                        agreed = max(
                            clusters,
                            key=lambda group: (
                                statistics.median(
                                    item.frequency_hz for item in group
                                ),
                                len(group),
                                sum(item.fit_score for item in group),
                            ),
                        )
                    else:
                        agreed = []

                else:
                    agreed = []

                if agreed:
                    median_frequency = float(
                        statistics.median(item.frequency_hz for item in agreed)
                    )
                    representative = min(
                        agreed,
                        key=lambda item: abs(item.frequency_hz - median_frequency),
                    )
                    median_cycles = float(
                        statistics.median(item.cycles for item in agreed)
                    )
                    readable = (
                        0.8 <= median_cycles <= 25.0
                        and representative.confidence >= 0.55
                    )
                    if readable:
                        spread = (
                            max(item.frequency_hz for item in agreed)
                            - min(item.frequency_hz for item in agreed)
                        )
                        temporal_score = float(
                            np.clip(
                                1.0 - spread / max(600.0, 0.08 * median_frequency),
                                0.0,
                                1.0,
                            )
                        )
                        candidates.append(
                            Estimate(
                                True,
                                median_frequency,
                                median_cycles,
                                max(representative.fit_score, temporal_score),
                                representative.row_coverage,
                                representative.amplitude_px,
                                representative.reset_x,
                                "",
                                "temporal_median",
                            )
                        )
                        if confirmation_windows == 0:
                            confirmation_windows = 2
                        else:
                            confirmation_windows -= 1
                            if confirmation_windows == 0:
                                break
                elif confirmation_windows > 0:
                    confirmation_windows -= 1
                    if confirmation_windows == 0:
                        break
        finally:
            serial_send(port, "fwindow off")
            serial_drain(port, 0.3)

    if len(candidates) < 2:
        return Estimate(
            False,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
            "fewer than two confirmed windows",
        )

    best_group: list[Estimate] = []
    for anchor in candidates:
        tolerance = max(100.0, 0.05 * anchor.frequency_hz)
        group = [
            item
            for item in candidates
            if abs(item.frequency_hz - anchor.frequency_hz) <= tolerance
        ]
        if len(group) > len(best_group):
            best_group = group
        elif len(group) == len(best_group) and group:
            if sum(item.confidence for item in group) > sum(
                item.confidence for item in best_group
            ):
                best_group = group

    if len(best_group) < 2:
        return Estimate(
            False,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
            "cross-window consensus failed",
        )

    # Cross-window agreement confirms the result.  About five visible cycles
    # gives the most stable phase-independent fit on this oscilloscope.  The
    # small camera/geometry correction is explicit in the camera config so it
    # can be re-calibrated after moving the camera.
    representative = min(
        best_group,
        key=lambda item: abs(math.log(max(item.cycles, 0.1) / 5.0)),
    )
    consensus_frequency = (
        representative.frequency_hz * settings.frequency_scale
    )
    final_method = "temporal+window_calibrated"
    rounded_frequency = math.floor(consensus_frequency / 100.0 + 0.5) * 100.0
    return Estimate(
        True,
        rounded_frequency,
        representative.cycles,
        float(np.mean([item.confidence for item in best_group])),
        representative.row_coverage,
        representative.amplitude_px,
        representative.reset_x,
        "",
        final_method,
    )


def parse_windows_us(text: str) -> list[int]:
    allowed = {20, 50, 100, 200, 500, 1000, 2000, 5000, 10000}
    result = []
    for item in text.split(","):
        value = int(item.strip())
        if value not in allowed:
            raise argparse.ArgumentTypeError(
                "windows must use 20,50,100,200,500,1000,2000,5000,10000 us"
            )
        result.append(value)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="vision_config.json")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image")
    source.add_argument("--url")
    source.add_argument("--live", action="store_true")
    parser.add_argument("--window-ms", type=float, default=2.0)
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument(
        "--windows-us",
        type=parse_windows_us,
        default=[10000, 5000, 2000, 1000, 500, 200, 100],
    )
    parser.add_argument("--frames", type=int, default=7)
    parser.add_argument("--debug-out")
    parser.add_argument("--debug-dir", default="debug_frequency")
    args = parser.parse_args()

    settings = Settings.load(args.config)
    if args.live:
        result = measure_live(
            "http://127.0.0.1:8080/frame.jpg",
            args.serial,
            settings,
            args.windows_us,
            max(3, args.frames),
            args.debug_dir,
        )
        print("FINAL")
        print_estimate(result, 0.0)
        return 0 if result.valid else 2

    frame = (
        cv2.imread(args.image, cv2.IMREAD_COLOR)
        if args.image
        else fetch_jpeg(args.url)
    )
    if frame is None:
        raise RuntimeError("image could not be read")
    result = analyze_frame(frame, args.window_ms, settings, args.debug_out)
    print_estimate(result, args.window_ms)
    return 0 if result.valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
