#!/usr/bin/env python3
"""Score how compact a same-frequency DDS/unknown-signal XY trace is."""

from __future__ import annotations

import argparse
import json

import cv2
import numpy as np

from ramp_frequency import Settings, trace_mask


def score_frame(frame: np.ndarray, settings: Settings) -> dict[str, float | int]:
    roi = settings.roi.crop(frame)
    mask = trace_mask(roi, settings)
    ys, xs = np.where(mask > 0)
    if xs.size < 80:
        return {
            "valid": 0,
            "trace_pixels": int(xs.size),
            "area_ratio": 1.0,
            "bbox_fill": 1.0,
            "compact_score": 1.0e9,
        }

    x_span = int(xs.max() - xs.min() + 1)
    y_span = int(ys.max() - ys.min() + 1)
    bbox_area = max(1, x_span * y_span)
    trace_pixels = int(xs.size)
    area_ratio = trace_pixels / float(mask.size)
    bbox_fill = trace_pixels / float(bbox_area)

    # The matching-frequency trace is a thin line or ellipse. A mismatch
    # rotates through many phases during one camera exposure and fills most of
    # the XY rectangle. Pixel area is therefore the primary term; bbox fill
    # rejects UI fragments and unusually thick traces.
    compact_score = trace_pixels * (1.0 + 2.0 * bbox_fill)
    return {
        "valid": 1,
        "trace_pixels": trace_pixels,
        "area_ratio": round(area_ratio, 6),
        "bbox_fill": round(bbox_fill, 6),
        "x_span": x_span,
        "y_span": y_span,
        "compact_score": round(compact_score, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    frame = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"cannot read image: {args.image}")
    result = score_frame(frame, Settings.load(args.config))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
