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


def validate(directory):
    for backend in ("mesh", "grid", "dag", "vdb"):
        for view in ("orbit", "orbit-wireframe", "surface", "horizon", "wireframe", "ranges", "free", "global-lod-wireframe"):
            bitmap(directory / f"{backend}-{view}.bmp")
    bitmap(directory / "invoker-control.bmp")
    if "invoker_controls=passed" not in (directory / "validation.txt").read_text().splitlines():
        raise ValueError("Invoker movement and camera controls did not finish validation")
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
    diagnostics = dict(line.split("=", 1) for line in
                       (directory / "pipeline-csm-on.pipeline.txt").read_text().splitlines())
    if float(diagnostics["hdr_max_component"]) <= 1 or int(diagnostics["invalid_gpu_values"]) != 0:
        raise ValueError("HDR target failed float-range/finite-value validation")
    if int(diagnostics["gbuffer_geometry_pixels"]) < 1000 or any(
        int(diagnostics[f"cascade_{i}_depth_pixels"]) == 0 for i in range(4)
    ):
        raise ValueError("G-buffer or one of the four CSM layers is empty")
    print(f"40 required captures verified; translation max difference={maximum}/255, mean={mean:.9f}/255; "
          f"shadowed={shadowed}, exposure-brightened={brightened}; HDR max={diagnostics['hdr_max_component']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
