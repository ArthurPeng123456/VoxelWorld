"""Validate cloud detail and shadow flight captures and summarize DX12 GPU timings."""
import argparse
import csv
import json
from pathlib import Path
import runpy

import numpy as np


bitmap = runpy.run_path(str(Path(__file__).with_name("validate-temporal-captures.py")))["bitmap"]
MODES = ("detail-off", "enhanced", "full", "full-detail-off", "shadow-off")


def luma(path, stride=1):
    (width, height), pixels = bitmap(path)
    bgr = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 4)[::stride, ::stride, :3]
    return bgr @ np.array([0.0722, 0.7152, 0.2126], dtype=np.float32)


def rms(a):
    return float(np.sqrt(np.mean(np.square(a))))


def high_pass(a):
    return a[..., 1:-1, 1:-1] - 0.25 * (
        a[..., :-2, 1:-1] + a[..., 2:, 1:-1] + a[..., 1:-1, :-2] + a[..., 1:-1, 2:])


def second_difference(a):
    return np.diff(a, n=2, axis=0)


def summarize_gpu(rows):
    values = np.array([[float(row["integration_ms"]), float(row["reconstruction_ms"])] for row in rows])
    if not len(values) or not np.isfinite(values).all() or (values < 0).any() or (values[:, 0] <= 0).any():
        raise ValueError("Missing or invalid D3D12 timestamps")
    combined = np.column_stack((values, values.sum(axis=1)))
    return {stage: dict(zip(("p50", "p95"), percentiles.tolist()))
            for stage, percentiles in zip(("integration", "reconstruction", "total"),
                                          np.percentile(combined, (50, 95), axis=0).T)}


def validate(directory):
    report = dict(line.split("=", 1) for line in (directory / "cloud-flight.txt").read_text().splitlines())
    orbit, flight = (int(report[field]) for field in ("orbit_frames", "flight_frames"))
    count = orbit + flight
    with (directory / "cloud-flight.csv").open(newline="") as source:
        timings = list(csv.DictReader(source))
    modes = {mode: [row for row in timings if row["mode"] == mode] for mode in MODES}
    for mode, rows in modes.items():
        if [int(row["frame"]) for row in rows] != list(range(count)):
            raise ValueError(f"Incomplete timestamps for {mode}")
    distance = np.array([float(row["distance_m"]) for row in modes["enhanced"]])
    if not np.allclose(distance[orbit:], distance[orbit:][::-1], atol=0.01, rtol=0):
        raise ValueError("The descent and ascent do not use matching positions")
    gpu = {mode: summarize_gpu(rows) for mode, rows in modes.items()}
    print(f'{report["renderer"]}, {report["viewport"]}, {count} timed frames per mode')
    for mode, stages in gpu.items():
        print(mode, ", ".join(f'{name} {v["p50"]:.3f}/{v["p95"]:.3f} ms P50/P95' for name, v in stages.items()))

    frames = {mode: np.stack([luma(directory / f"{mode}-{index}.bmp", stride=3) for index in range(count)])
              for mode in ("enhanced", "detail-off", "full", "full-detail-off")}
    enhanced, baseline, full, full_baseline = (frames[mode] for mode in
                                               ("enhanced", "detail-off", "full", "full-detail-off"))
    detail = enhanced - baseline
    full_detail = full - full_baseline
    inside = np.stack([luma(directory / f"inside-{i}.bmp", stride=3) for i in range(orbit)])
    # Frozen frames isolate noise. On the moving camera, subtract the matched
    # detail signal without temporal jitter to account for genuine feature motion.
    transition = (distance[1:-1] > 2000) & (distance[1:-1] < 300000)
    near = distance < 15000
    metrics = {
        "orbit_detail_rms": rms(detail[:orbit]),
        "orbit_shimmer_rms": rms(second_difference(enhanced[:orbit])),
        "inside_shimmer_rms": rms(second_difference(inside)),
        "near_detail_highpass_rms": rms(high_pass(detail[near])),
        "transition_added_shimmer_rms": rms(second_difference(detail - full_detail)[transition]),
        "transition_detail_change_rms": rms(second_difference(detail)[transition]),
        "transition_full_vs_temporal_rms": rms((enhanced - full)[1:-1][transition]),
    }
    shadow_maps = []
    for sun in range(2):
        prefix = directory / f"sun-{sun}"
        on, off = (luma(prefix.with_name(prefix.name + f"-{suffix}.bmp")) for suffix in ("on", "off"))
        alpha_on, alpha_off = (luma(prefix.with_name(prefix.name + f"-{suffix}.cloud-opacity.bmp"))
                               for suffix in ("on", "off"))
        for suffix in ("on", "off"):
            pipeline = prefix.with_name(prefix.name + f"-{suffix}.pipeline.txt")
            values = dict(line.split("=", 1) for line in pipeline.read_text().splitlines())
            if int(values["invalid_gpu_values"]) or int(values["cloud_pixels"]) < 100:
                raise ValueError(f"Invalid shadow test buffers in {pipeline}")
        if np.max(np.abs(alpha_on - alpha_off)) > 1:
            raise ValueError("Changing self-shadow unexpectedly changed cloud opacity")
        shade = off - on
        if float(np.mean(shade > 1)) < 0.01:
            raise ValueError(f"Sun angle {sun} did not produce measurable cloud self-shadow")
        shadow_maps.append(shade)
    metrics["shadow_pattern_change_rms"] = rms(high_pass(shadow_maps[0] - shadow_maps[1]))
    print(json.dumps(metrics, indent=2))
    (directory / "cloud-flight-metrics.json").write_text(json.dumps({"image_metrics": metrics, "gpu_ms": gpu}, indent=2) + "\n")
    if metrics["orbit_detail_rms"] > 0.1 or metrics["orbit_shimmer_rms"] > 1.5:
        raise ValueError("The new details leaked into the orbit view or the orbit view shimmers")
    if metrics["near_detail_highpass_rms"] < 0.07:
        raise ValueError("The replay did not resolve the new detail bands near the clouds")
    if metrics["inside_shimmer_rms"] > 1.5 or metrics["transition_added_shimmer_rms"] > 1.5:
        raise ValueError("The new detail bands shimmer during the descent or ascent")
    if metrics["shadow_pattern_change_rms"] < 0.05:
        raise ValueError("The self-shadow pattern did not respond to the sunlight direction")
    print("Orbit, two-way transition, near detail and self-shadow checks passed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
