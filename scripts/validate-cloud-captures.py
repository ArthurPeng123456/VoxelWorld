"""Check planetary cloud rendering, empty coverage, occlusion and origin shifts."""
import argparse
from collections import deque
from pathlib import Path
import struct


def bitmap(path):
    data = path.read_bytes()
    offset = struct.unpack_from("<I", data, 10)[0]
    width, height = struct.unpack_from("<ii", data, 18)
    if data[:2] != b"BM" or struct.unpack_from("<H", data, 28)[0] != 32 or width <= 0 or height >= 0:
        raise ValueError(f"Expected a top-down 32-bit capture: {path}")
    pixels = data[offset:]
    if len(pixels) != width * -height * 4:
        raise ValueError(f"Incomplete bitmap: {path}")
    return (width, -height), pixels


def difference(a, b):
    if a[0] != b[0]:
        raise ValueError("Capture dimensions differ")
    maximum = changed = total = 0
    for i in range(0, len(a[1]), 4):
        values = [abs(a[1][i + c] - b[1][i + c]) for c in range(3)]
        maximum = max(maximum, *values)
        changed += max(values) > 3
        total += sum(values)
    return maximum, total / (len(a[1]) * 3 / 4), changed


def validate(directory):
    names = ("orbit", "orbit-off", "zero-coverage", "zero-density", "orbit-low", "orbit-high",
             "below", "horizon", "inside", "wind", "precision-base", "precision-translated",
             "occluded-on", "occluded-off")
    images = {name: bitmap(directory / f"cloud-{name}.bmp") for name in names}
    reports = {}
    for name in names:
        report = dict(line.split("=", 1) for line in (directory / f"cloud-{name}.pipeline.txt").read_text().splitlines())
        if int(report["invalid_gpu_values"]) or not 0 <= float(report["cloud_max_opacity"]) <= 1:
            raise ValueError(f"Invalid cloud/HDR/G-buffer data in {name}")
        reports[name] = report
    maximum, mean, changed = difference(images["orbit"], images["orbit-off"])
    if changed < 1000 or mean < 0.2:
        raise ValueError("Enabling clouds did not materially change the orbital view")
    for name in ("zero-coverage", "zero-density"):
        if difference(images[name], images["orbit-off"])[0] > 1:
            raise ValueError(f"{name} leaves residual cloud radiance or ground shadows")
    shift = difference(images["precision-base"], images["precision-translated"])
    if shift[0] > 2 or shift[1] > 0.005:
        raise ValueError(f"Cloud rendering changes under a common 1e12 m translation: {shift}")
    if difference(images["occluded-on"], images["occluded-off"])[0] > 1:
        raise ValueError("Clouds leak through the opaque foreground receiver")
    quality = difference(images["orbit-low"], images["orbit-high"])
    if quality[1] > 12:
        raise ValueError(f"Orbital cloud shape is unstable across sampling quality levels: {quality}")
    wind = difference(images["inside"], images["wind"])
    if wind[2] < images["inside"][0][0] * images["inside"][0][1] * 0.001:
        raise ValueError("Wind did not move the local cloud density field")

    # Test large coherent regions rather than a carpet of isolated speckles.
    # The opacity buffer isolates the cloud shape from surface color/atmosphere.
    (width, height), mask = bitmap(directory / "cloud-orbit.cloud-opacity.bmp")
    cells = {(x // 4, y // 4) for y in range(0, height, 4) for x in range(0, width, 4)
             if mask[(y * width + x) * 4] >= 64}
    occupied = len(cells)
    components = []
    while cells:
        pending = deque([cells.pop()])
        count = 0
        while pending:
            x, y = pending.popleft()
            count += 1
            for point in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if point in cells:
                    cells.remove(point)
                    pending.append(point)
        components.append(count)
    if not occupied or max(components) / occupied < 0.10 or sum(c for c in components if c < 4) / occupied > 0.10:
        raise ValueError("Orbital cloud mask is dominated by small disconnected details")
    disk_pixels = int(reports["orbit"]["gbuffer_geometry_pixels"]) / 4
    cover = occupied * 16 / disk_pixels
    if not 0.03 < cover < 0.85:
        raise ValueError(f"Default weather must retain substantial clear sky: {cover:.1%}")
    print(f"14 cloud views passed; orbital mask coverage ~{cover:.1%}; largest region {max(components)/occupied:.1%}")
    print(f"Cloud toggle: {changed} changed pixels, mean delta {mean:.3f}/255")
    print(f"1e12 m shift: max {shift[0]}/255, mean {shift[1]:.6f}/255; foreground occlusion passed")
    print(f"32 vs 96 samples: mean delta {quality[1]:.3f}/255; wind: {wind[2]} changed pixels")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
