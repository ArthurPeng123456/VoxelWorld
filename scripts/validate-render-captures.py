"""Check DX12 capture coverage and large-world translation invariance (stdlib only)."""
import argparse
from pathlib import Path
import struct


def bitmap(path):
    data = path.read_bytes()
    if data[:2] != b"BM":
        raise ValueError(f"Not a BMP: {path}")
    offset = struct.unpack_from("<I", data, 10)[0]
    width, height = struct.unpack_from("<ii", data, 18)
    bpp = struct.unpack_from("<H", data, 28)[0]
    if bpp != 32 or width <= 0 or height >= 0:
        raise ValueError(f"Expected the viewer's 32-bit top-down BMP: {path}")
    pixels = data[offset:]
    if len(pixels) != width * -height * 4:
        raise ValueError(f"Incomplete capture: {path}")
    return (width, -height), pixels


def orbit_shading_edges(path):
    (width, height), pixels = bitmap(path)
    radius = int(height * 0.26)
    stride = width * 4
    # Sample inside the disk, away from the silhouette. A second difference
    # catches mesh edges and bands while allowing the gradual daylight gradient.
    def luma(index):
        b, g, r = pixels[index:index + 3]
        return (19 * b + 183 * g + 54 * r + 128) >> 8

    edges = 0
    for y in range(height // 2 - radius, height // 2 + radius):
        for x in range(width // 2 - radius, width // 2 + radius):
            index = y * stride + 4 * x
            center = 2 * luma(index)
            for offset in (4, stride):
                edges += abs(luma(index - offset) + luma(index + offset) - center) > 4
    return edges / (8 * radius * radius)


def validate(directory):
    report = (directory / "validation.txt").read_text().splitlines()
    terrain = next((line.split("=", 1)[1] for line in report if line.startswith("terrain=")), "classic")
    for backend in ("mesh", "grid", "dag", "vdb"):
        for view in ("orbit", "orbit-wireframe", "surface", "horizon", "wireframe", "ranges", "free", "global-lod-wireframe"):
            bitmap(directory / f"{backend}-{view}.bmp")
    orbit_file = directory / "grid-orbit-no-effects.bmp"
    orbit_edges = orbit_shading_edges(orbit_file)
    if terrain == "classic" and orbit_edges > 0.005:
        raise ValueError(f"Mesh shading bands visible in the orbit capture: {orbit_edges:.2%}")
    bitmap(directory / "invoker-control.bmp")
    if "invoker_controls=passed" not in report:
        raise ValueError("Invoker movement and camera controls did not finish validation")
    if "terrain=classic" in report and "terrain_controls=passed" not in report:
        raise ValueError("Terrain dropdown did not finish validation")
    if terrain == "mars":
        for region in (1, 2):
            bitmap(directory / f"grid-mars-remote-{region}.bmp")
    size_a, a = bitmap(directory / "precision-base.bmp")
    size_b, b = bitmap(directory / "precision-translated.bmp")
    if size_a != size_b:
        raise ValueError("Precision captures have different dimensions")
    maximum = max(abs(x - y) for x, y in zip(a, b))
    mean = sum(abs(x - y) for x, y in zip(a, b)) / len(a)
    if maximum > 3:
        raise ValueError(f"Trillion-meter translation changed a channel by {maximum}/255")
    on_size, on = bitmap(directory / "pipeline-csm-on.bmp")
    off_size, off = bitmap(directory / "pipeline-csm-off.bmp")
    ev_size, exposed = bitmap(directory / "pipeline-exposure-plus1.bmp")
    resized_size, _ = bitmap(directory / "pipeline-resized.bmp")
    restored_size, _ = bitmap(directory / "pipeline-restored.bmp")
    if on_size != off_size or on_size != ev_size or restored_size != on_size or resized_size == on_size:
        raise ValueError("HDR pipeline did not preserve/recreate the expected target dimensions")
    shadowed = sum(sum(on[i:i+3]) + 15 < sum(off[i:i+3]) for i in range(0, len(on), 4))
    brightened = sum(sum(exposed[i:i+3]) > sum(off[i:i+3]) + 10 for i in range(0, len(off), 4))
    if shadowed < 100 or brightened < 1000:
        raise ValueError(f"Missing visible shadow/exposure response: shadowed={shadowed}, brightened={brightened}")
    def pipeline(name, resolution):
        info = dict(line.split("=", 1) for line in
                    (directory / f"{name}.pipeline.txt").read_text().splitlines())
        if int(info["shadow_resolution"]) != resolution or int(info["invalid_gpu_values"]) != 0:
            raise ValueError(f"Invalid DX12 targets in {name}")
        if int(info["gbuffer_geometry_pixels"]) < 1000 or any(
            int(info[f"cascade_{i}_depth_pixels"]) == 0 for i in range(4)
        ):
            raise ValueError(f"G-buffer or one of the four CSM layers is empty in {name}")
        return info

    diagnostics = pipeline("pipeline-csm-on", 1024)
    if float(diagnostics["hdr_max_component"]) <= 1:
        raise ValueError("HDR target failed float-range/finite-value validation")
    for edge in (2048, 4096):
        name = f"pipeline-shadow-{edge}"
        if bitmap(directory / f"{name}.bmp")[0] != on_size:
            raise ValueError("Changing shadow resolution changed the viewport size")
        pipeline(name, edge)
    for name in ("tone-reinhard", "tone-linear", "atmosphere-tuned"):
        size, configured = bitmap(directory / f"pipeline-{name}.bmp")
        if size != on_size or sum(abs(p - q) > 2 for p, q in zip(on, configured)) < 1000:
            raise ValueError(f"Rendering settings did not reach the GPU in {name}")
    count = 48 if terrain == "mars" else 46
    print(f"{count} required captures verified for {terrain}; orbit shading edges={orbit_edges:.2%}; CSM 1024/2048/4096; "
          f"translation max difference={maximum}/255, mean={mean:.9f}/255; "
          f"shadowed={shadowed}, exposure-brightened={brightened}; HDR max={diagnostics['hdr_max_component']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
