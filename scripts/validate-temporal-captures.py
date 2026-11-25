"""Validate scene TAA and depth/wind-aware sparse cloud reconstruction (stdlib only)."""
import argparse
import math
from pathlib import Path
import struct


def bitmap(path):
    data = path.read_bytes()
    offset = struct.unpack_from("<I", data, 10)[0]
    width, height = struct.unpack_from("<ii", data, 18)
    if data[:2] != b"BM" or struct.unpack_from("<H", data, 28)[0] != 32 or width <= 0 or height >= 0:
        raise ValueError(f"Unsupported capture: {path}")
    pixels = data[offset:]
    if len(pixels) != width * -height * 4:
        raise ValueError(f"Incomplete bitmap: {path}")
    return (width, -height), pixels


def difference(a, b):
    if a[0] != b[0]:
        raise ValueError("Capture dimensions differ")
    maximum = total = 0
    for i in range(0, len(a[1]), 4):
        for c in range(3):
            delta = abs(a[1][i + c] - b[1][i + c])
            maximum = max(maximum, delta)
            total += delta
    return maximum, total / (len(a[1]) * 0.75)


def linear(path, resolved=False):
    data = path.read_bytes()
    width, height = struct.unpack_from("<II", data)
    size = width * height * 8
    if len(data) != 8 + 2 * size:
        raise ValueError(f"Incomplete linear HDR region: {path}")
    offset = 8 + size * resolved
    values = [p[:3] for p in struct.iter_unpack("<eeee", data[offset:offset + size])]
    if any(not math.isfinite(c) or c < 0 for pixel in values for c in pixel):
        raise ValueError(f"Invalid linear HDR values: {path}")
    return values


def validate(directory):
    reports = {}
    for path in directory.glob("taa-*.pipeline.txt"):
        report = dict(line.split("=", 1) for line in path.read_text().splitlines())
        if int(report["invalid_gpu_values"]):
            raise ValueError(f"Invalid GPU values in {path}")
        reports[path.name.removesuffix(".pipeline.txt")] = report
    required = ["off", "static", "moving", "moving-reference", "camera-pan", "camera-cut", "cut-reference",
                "resized", "precision-base", "precision-translated", "cloud-no-taa", "cloud-reenabled",
                "cloud-occluded", "cloud-occluded-reference", "cloud-disoccluded", "cloud-disoccluded-reference", "cloud-wind"]
    required += [f"reference-{i}" for i in range(16)] + [f"cloud-pattern-{i}" for i in range(3)]
    for name in required:
        bitmap(directory / f"taa-{name}.bmp")
        if f"taa-{name}" not in reports:
            raise ValueError(f"Missing temporal diagnostics: {name}")
    def image(name):
        return bitmap(directory / f"taa-{name}.bmp")
    def report(name):
        return reports[f"taa-{name}"]

    # The reference is the arithmetic mean of 16 LINEAR HDR jitter samples,
    # before tone mapping. Measure only pixels whose coverage varies with jitter.
    references = [linear(directory / f"taa-reference-{i}.linear.bin") for i in range(16)]
    n = len(references[0])
    mean = [tuple(sum(references[i][p][c] for i in range(16)) / 16 for c in range(3)) for p in range(n)]
    edge = [p for p in range(n) if sum(sum((references[i][p][c] - mean[p][c]) ** 2 for i in range(16)) / 16
                                      for c in range(3)) > 0.0001]
    if len(edge) < 1000:
        raise ValueError("Temporal reference does not contain enough subpixel edges")
    off = linear(directory / "taa-off.linear.bin")
    filtered = linear(directory / "taa-static.linear.bin", True)
    def rms(a):
        return math.sqrt(sum((a[p][c] - mean[p][c]) ** 2 for p in edge for c in range(3)) / (len(edge) * 3))
    error_off, error_taa = rms(off), rms(filtered)
    if error_taa >= error_off * 0.85:
        raise ValueError(f"TAA failed to improve edge error: {error_taa} vs {error_off}")
    if int(report("static")["taa_history_pixels"]) < 1000 or not int(report("camera-pan")["taa_history_valid"]):
        raise ValueError("Static/panning scene did not reuse temporal history")
    if int(report("moving")["moving_object_pixels"]) < 100:
        raise ValueError("Dynamic geometry is missing motion vectors")
    for name in ("camera-cut", "resized", "cloud-reenabled"):
        if int(report(name)["taa_history_valid"]):
            raise ValueError(f"Stale history survived {name}")
    if difference(image("camera-cut"), image("cut-reference"))[0] > 1:
        raise ValueError("Camera-cut result differs from a cold history")
    if image("resized")[0] == image("static")[0]:
        raise ValueError("Resize scenario did not resize the render targets")

    # A saturated moving object must not leave a long colored trail on the floor.
    (width, height), current = image("moving-reference")
    moving = image("moving")[1]
    red = {i // 4 for i in range(0, len(current), 4)
           if current[i + 2] > current[i + 1] + 35 and current[i + 2] > current[i] + 35}
    allowed = {yy * width + xx for p in red for yy in range(max(0, p // width - 3), min(height, p // width + 4))
               for xx in range(max(0, p % width - 3), min(width, p % width + 4))}
    ghosts = sum(p not in allowed and moving[p * 4 + 2] > moving[p * 4 + 1] + 35 and moving[p * 4 + 2] > moving[p * 4] + 35
                 for p in range(width * height))
    if len(red) < 100 or ghosts > 20:
        raise ValueError(f"Moving object left a colored history trail: {ghosts} pixels")
    shift = difference(image("precision-base"), image("precision-translated"))
    if shift[0] > 2 or shift[1] > 0.005:
        raise ValueError(f"TAA is unstable after a common 1e12 m translation: {shift}")

    work = [int(report(f"cloud-pattern-{i}")["cloud_primary_steps"]) for i in range(3)]
    if not work[0] or work[1] >= work[0] * 0.75 or work[2] >= work[0] * 0.55:
        raise ValueError(f"Sparse cloud modes did not reduce actual raymarch work: {work}")
    for i in (1, 2):
        if not int(report(f"cloud-pattern-{i}")["cloud_skipped_pixels"]):
            raise ValueError("Sparse cloud mode never skipped a pixel")
        if difference(image("cloud-pattern-0"), image(f"cloud-pattern-{i}"))[1] > 4:
            raise ValueError("Cloud reconstruction differs excessively from full temporal sampling")
    for name in ("cloud-no-taa", "cloud-reenabled"):
        if int(report(name)["cloud_skipped_pixels"]):
            raise ValueError(f"Sparse clouds used invalid history in {name}")
    if difference(image("cloud-occluded"), image("cloud-occluded-reference"))[0] > 1:
        raise ValueError("Temporal clouds leak through a new opaque foreground")
    disoccluded = report("cloud-disoccluded")
    if int(disoccluded["cloud_fallback_pixels"]) < int(disoccluded["cloud_skipped_pixels"]) * 0.9:
        raise ValueError("Newly uncovered cloud pixels did not use immediate raymarch fallback")
    if difference(image("cloud-disoccluded"), image("cloud-disoccluded-reference"))[1] > 8:
        raise ValueError("Newly visible clouds retain stale foreground history")
    print(f"Temporal validation passed: {len(required)} captures; edge RMSE {error_off:.5f} -> {error_taa:.5f} ({1-error_taa/error_off:.1%} lower)")
    print(f"Moving object: {report('moving')['moving_object_pixels']} motion pixels, {ghosts} trail pixels; cuts/resize/occlusion passed")
    print(f"1e12 m translation: max {shift[0]}/255, mean {shift[1]:.6f}/255")
    print(f"Cloud primary steps: full {work[0]}, checker {work[1]} ({work[1]/work[0]:.1%}), quarter {work[2]} ({work[2]/work[0]:.1%})")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
