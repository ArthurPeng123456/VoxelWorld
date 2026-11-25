"""Measure frame-to-frame shimmer at a fixed camera and frozen simulation time."""
import argparse
import json
import math
from pathlib import Path
import runpy

bitmap = runpy.run_path(str(Path(__file__).with_name("validate-temporal-captures.py")))["bitmap"]


GROUPS = ("clear", "orbit-full", "orbit-checker", "horizon-checker", "horizon-quarter",
          "horizon-no-projection-jitter", "horizon-no-ray-jitter")


def measure(directory, name):
    frames = []
    for frame in range(32):
        stem = directory / f"taa-stability-{name}-{frame}"
        report = dict(line.split("=", 1) for line in stem.with_suffix(".pipeline.txt").read_text().splitlines())
        if int(report["invalid_gpu_values"]) or not int(report["taa_history_valid"]):
            raise ValueError(f"Invalid buffers or reset history: {stem}")
        (width, height), pixels = bitmap(stem.with_suffix(".bmp"))
        # Uniform interior samples retain the high-frequency silhouette signal.
        frames.append([(pixels[p] + 2 * pixels[p + 1] + pixels[p + 2]) / 4
                       for y in range(4, height - 4, 3) for x in range(4, width - 4, 3)
                       for p in ((y * width + x) * 4,)])
    count = len(frames[0])
    delta = sum((b - a) ** 2 for a_frame, b_frame in zip(frames, frames[1:])
                for a, b in zip(a_frame, b_frame)) / (31 * count)
    high = sum((c - 2 * b + a) ** 2 for i in range(1, 31)
               for a, b, c in zip(frames[i - 1], frames[i], frames[i + 1])) / (30 * count)
    ranges = sorted(max(p) - min(p) for p in zip(*frames))
    return {"frame_rms": math.sqrt(delta), "high_frequency_rms": math.sqrt(high),
            "range_p99": ranges[int(count * 0.99)], "unstable_fraction": sum(x > 2 for x in ranges) / count}


def validate(directory, baseline=None, measure_only=False):
    result = {name: measure(directory, name) for name in GROUPS}
    before = {name: measure(baseline, name) for name in GROUPS} if baseline else None
    for name, values in result.items():
        text = f"{name}: frame RMS={values['frame_rms']:.4f}/255, high-frequency RMS={values['high_frequency_rms']:.4f}/255, unstable={values['unstable_fraction']:.2%}"
        if before:
            ratio = values["high_frequency_rms"] / max(1e-9, before[name]["high_frequency_rms"])
            text += f", high-frequency ratio={ratio:.1%}"
        print(text)
    (directory / "stability-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    if not measure_only:
        # Absolute display-space budgets reject the original global jitter and
        # the quarter-rate phase crawl. Frozen views retain stochastic sampling;
        # they are not expected to be bit-identical between successive frames.
        limits = dict(zip(GROUPS, (0.5, 1.5, 1.5, 0.3, 0.3, 0.15, 0.3)))
        for name, limit in limits.items():
            if result[name]["high_frequency_rms"] > limit:
                raise ValueError(f"Excessive fixed-camera shimmer in {name}: {result[name]['high_frequency_rms']:.4f} > {limit}/255")
        print("Fixed-camera temporal stability passed: 7 sequences x 32 frames")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--measure-only", action="store_true", help="Report a pre-fix baseline without applying stability budgets")
    args = parser.parse_args()
    validate(args.directory, args.baseline, args.measure_only)
