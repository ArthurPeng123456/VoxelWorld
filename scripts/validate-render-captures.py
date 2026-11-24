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
    size_a, a = bitmap(directory / "precision-base.bmp")
    size_b, b = bitmap(directory / "precision-translated.bmp")
    if size_a != size_b:
        raise ValueError("Precision captures have different dimensions")
    maximum = max(abs(x - y) for x, y in zip(a, b))
    mean = sum(abs(x - y) for x, y in zip(a, b)) / len(a)
    if maximum > 3:
        raise ValueError(f"Trillion-meter translation changed a channel by {maximum}/255")
    print(f"34 required captures verified; translation max difference={maximum}/255, mean={mean:.9f}/255")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
