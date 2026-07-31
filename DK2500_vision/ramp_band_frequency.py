#!/usr/bin/env python3
"""Estimate ramp-test frequency from horizontal XY trace bands.

For the current DS1102Z-E display mode, a sine wave plotted against the
windowed ramp is rendered as horizontal sweeps.  A complete sine period
contains two sweeps, so:

    frequency_hz = band_count * 0.5 / window_seconds

This is deliberately separate from ramp_frequency.py, whose x(y) sinusoid
fit is retained for displays that render a thin continuous curve.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Roi:
    x: int
    y: int
    w: int
    h: int


def load_config(path: str) -> tuple[Roi, np.ndarray, np.ndarray]:
    with open(path, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    roi_raw = raw["roi"]
    roi = Roi(
        int(roi_raw["x"]),
        int(roi_raw["y"]),
        int(roi_raw["w"]),
        int(roi_raw["h"]),
    )
    lower = np.array(
        [
            int(raw.get("hue_min", 20)),
            int(raw.get("sat_min", 40)),
            int(raw.get("val_min", 80)),
        ],
        dtype=np.uint8,
    )
    upper = np.array(
        [int(raw.get("hue_max", 50)), 255, 255], dtype=np.uint8
    )
    return roi, lower, upper


def group_rows(active: np.ndarray, max_gap: int = 2) -> list[tuple[int, int]]:
    groups: list[tuple[int, int]] = []
    start: int | None = None
    last = -1
    for index, value in enumerate(active):
        if value:
            if start is None:
                start = index
            last = index
        elif start is not None and index - last > max_gap:
            groups.append((start, last))
            start = None
    if start is not None:
        groups.append((start, last))
    return groups


def analyze(
    image: np.ndarray, config_path: str, window_ms: float
) -> dict[str, object]:
    roi, lower, upper = load_config(config_path)
    height, width = image.shape[:2]
    x0 = max(0, min(roi.x, width - 1))
    y0 = max(0, min(roi.y, height - 1))
    x1 = max(x0 + 1, min(x0 + roi.w, width))
    y1 = max(y0 + 1, min(y0 + roi.h, height))
    crop = image[y0:y1, x0:x1]

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)),
    )

    # Only the central horizontal span is used for band detection. This drops
    # channel arrows and the vertical reset/turnaround portions at both ends.
    inner_x0 = int(mask.shape[1] * 0.25)
    inner_x1 = int(mask.shape[1] * 0.85)
    row_fill = np.mean(mask[:, inner_x0:inner_x1] > 0, axis=1)
    row_fill = np.convolve(row_fill, np.ones(3) / 3.0, mode="same")
    value_profile = np.mean(
        hsv[:, inner_x0:inner_x1, 2].astype(np.float32), axis=1
    )

    active_rows = np.flatnonzero(row_fill >= 0.5)
    spectral_peaks: list[dict[str, float]] = []
    if active_rows.size >= 40:
        active_start = int(active_rows[0])
        active_end = int(active_rows[-1]) + 1
        profile = value_profile[active_start:active_end]
        baseline = np.convolve(profile, np.ones(31) / 31.0, mode="same")
        centered = (profile - baseline) * np.hanning(profile.size)
        spectrum = np.abs(np.fft.rfft(centered))
        upper_bin = min(spectrum.size, max(3, profile.size // 3))
        indexes = np.argsort(spectrum[2:upper_bin])[-8:][::-1] + 2
        peak_max = float(np.max(spectrum[indexes])) if indexes.size else 1.0
        spectral_peaks = [
            {
                "bands": float(index),
                "period_px": round(float(profile.size / index), 3),
                "strength": round(float(spectrum[index] / max(1.0, peak_max)), 4),
            }
            for index in indexes
        ]
    else:
        active_start = 0
        active_end = 0

    candidates: list[dict[str, object]] = []
    for threshold in (0.20, 0.30, 0.40, 0.50, 0.60):
        raw_groups = group_rows(row_fill >= threshold, max_gap=0)
        groups = raw_groups
        groups = [(start, end) for start, end in groups if end - start + 1 <= 12]
        centers = [int(round((start + end) * 0.5)) for start, end in groups]
        if len(centers) >= 2:
            gaps = np.diff(centers)
            median_gap = float(np.median(gaps))
            regularity = float(
                1.0 - min(1.0, np.median(np.abs(gaps - median_gap)) /
                          max(1.0, median_gap))
            )
        else:
            median_gap = 0.0
            regularity = 0.0
        candidates.append(
            {
                "threshold": threshold,
                "bands": len(centers),
                "centers": centers,
                "raw_widths": [end - start + 1 for start, end in raw_groups],
                "median_gap": round(median_gap, 3),
                "regularity": round(regularity, 4),
            }
        )

    usable = [
        item
        for item in candidates
        if int(item["bands"]) >= 3 and float(item["regularity"]) >= 0.65
    ]
    if not usable:
        return {
            "valid": False,
            "window_ms": window_ms,
            "reason": "no regular horizontal band train",
            "row_fill_quantiles": [
                round(float(value), 4)
                for value in np.quantile(
                    row_fill, [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
                )
            ],
            "active_rows": [active_start, active_end],
            "value_quantiles": [
                round(float(value), 3)
                for value in np.quantile(
                    value_profile, [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
                )
            ],
            "spectral_peaks": spectral_peaks,
            "candidates": candidates,
        }

    # Prefer a threshold in the middle of the tested range, then regularity.
    best = max(
        usable,
        key=lambda item: (
            float(item["regularity"]) - abs(float(item["threshold"]) - 0.4),
            int(item["bands"]),
        ),
    )
    bands = int(best["bands"])
    frequency_hz = bands * 500.0 / window_ms
    return {
        "valid": True,
        "window_ms": window_ms,
        "bands": bands,
        "frequency_hz": frequency_hz,
        "threshold": best["threshold"],
        "median_gap": best["median_gap"],
        "regularity": best["regularity"],
        "candidates": candidates,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--window-ms", type=float, required=True)
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read image: {args.image}")
    result = analyze(image, args.config, args.window_ms)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if bool(result["valid"]) else 2


if __name__ == "__main__":
    raise SystemExit(main())
