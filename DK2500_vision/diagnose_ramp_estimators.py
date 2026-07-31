#!/usr/bin/env python3
"""Compare coarse-ramp estimators on saved debug frames."""

from __future__ import annotations

import argparse
import glob
import os

import cv2

import ramp_frequency as ramp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--images", required=True)
    args = parser.parse_args()

    settings = ramp.Settings.load(args.config)
    for path in sorted(glob.glob(args.images)):
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            continue
        raw = ramp.frequency_trace_mask(image, settings)
        mask, y0, y1, full_span, reset_x = ramp._edge_geometry(raw)
        _, curve, coverage = ramp._reconstruct_all_components(mask, y0, y1)
        fit_cycles, fit_score, _ = ramp._fit_cycles(curve)
        if curve.size > 1:
            fit_cycles *= full_span / float(curve.size - 1)
        crossing_cycles, crossing_quality, _, _ = (
            ramp._midline_spacing_cycles(mask, y0, y1, full_span)
        )
        print(
            f"{os.path.basename(path)} "
            f"fit={fit_cycles:.4f}/{fit_score:.3f} "
            f"cross={crossing_cycles:.4f}/{crossing_quality:.3f} "
            f"coverage={coverage:.3f} reset={reset_x}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
