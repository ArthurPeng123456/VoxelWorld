"""Check human-scale CSM contrast, filtering, TAA and large-world translation (stdlib only)."""
import argparse
import json
from pathlib import Path
import struct


def bitmap(path):
    data = path.read_bytes()
    offset = struct.unpack_from("<I", data, 10)[0]
    width, height = struct.unpack_from("<ii", data, 18)
    bpp = struct.unpack_from("<H", data, 28)[0]
    if data[:2] != b"BM" or bpp != 32 or width <= 0 or height >= 0:
        raise ValueError(f"Unexpected capture format: {path}")
    pixels = data[offset:]
    if len(pixels) != width * -height * 4:
        raise ValueError(f"Incomplete capture: {path}")
    return (width, -height), pixels


def validate(directory):
    def pipeline(name, edge):
        info = dict(line.split("=", 1) for line in
                    (directory / f"shadow-{name}.pipeline.txt").read_text().splitlines())
        if int(info["invalid_gpu_values"]) or int(info["shadow_resolution"]) != edge:
            raise ValueError(f"Invalid GPU targets: {name}")
        if int(info["gbuffer_geometry_pixels"]) < 1000:
            raise ValueError(f"Missing ground and casters: {name}")
        return info

    def contrast(on, off, mask):
        delta = [off[i + 1] - on[i + 1] for i in mask]
        return {"max_delta": max(delta), "strong_pixels": sum(d > 48 for d in delta),
                "partial_pixels": sum(3 < d <= 48 for d in delta)}

    metrics = {}
    for edge in (1024, 2048, 4096):
        size, off = bitmap(directory / f"shadow-off-{edge}.bmp")
        # The unshadowed receiver is bright neutral gray. Exclude the colored
        # objects and sky, so caster shading cannot masquerade as a ground shadow.
        mask = [i for i in range(0, len(off), 4)
                if min(off[i:i + 3]) > 180 and max(off[i:i + 3]) - min(off[i:i + 3]) < 3]
        if not mask:
            raise ValueError("No visible unshadowed receiver")
        for mode in ("wide", "near"):
            name = f"{mode}-{edge}"
            on_size, on = bitmap(directory / f"shadow-{name}.bmp")
            if on_size != size:
                raise ValueError("Shadow setting changed viewport dimensions")
            info = pipeline(name, edge)
            metrics[name] = contrast(on, off, mask)
            metrics[name]["texel_m"] = float(info["cascade_0_texel_m"])
            metrics[name]["far_m"] = float(info["cascade_3_far_m"])
            if mode == "near" and (float(info["cascade_0_far_m"]) > 64.0001 or
                                   int(info["cascade_0_depth_pixels"]) == 0):
                raise ValueError("Near shadow coverage is missing or excessively broad")
        near, wide = metrics[f"near-{edge}"], metrics[f"wide-{edge}"]
        if near["max_delta"] < 100 or near["strong_pixels"] < max(400, wide["strong_pixels"] * 1.5):
            raise ValueError(f"Human-scale shadow contrast did not recover at {edge}: {near}")
        if near["texel_m"] >= wide["texel_m"] * 0.2 or near["far_m"] != wide["far_m"]:
            raise ValueError("Near detail did not improve independently of distant coverage")
        if edge == 2048:
            taa_size, taa = bitmap(directory / "shadow-near-2048-taa.bmp")
            taa_info = pipeline("near-2048-taa", edge)
            metrics["taa"] = contrast(taa, off, mask)
            if taa_size != size or int(taa_info["taa_history_valid"]) != 1 or \
                    metrics["taa"]["strong_pixels"] < near["strong_pixels"] * 0.8:
                raise ValueError("Temporal accumulation lost the human-scale shadows")
    size, base = bitmap(directory / "shadow-precision-base.bmp")
    moved_size, moved = bitmap(directory / "shadow-precision-translated.bmp")
    if size != moved_size:
        raise ValueError("Translation changed viewport dimensions")
    delta = [max(abs(base[i + c] - moved[i + c]) for c in range(3)) for i in range(0, len(base), 4)]
    changed = [pixel for pixel, difference in enumerate(delta) if difference > 3]
    width, height = size
    # DF64 vertex uploads at 1e12m can shift a few un-antialiased silhouette
    # samples by a fraction of a pixel. Permit only sparse edge changes; flat
    # interiors must agree, and coherent shadow drift must fail this comparison.
    def edge(pixel):
        x, y = pixel % width, pixel // width
        neighbors = [ny * width + nx for nx, ny in ((x-1,y), (x+1,y), (x,y-1), (x,y+1))
                     if 0 <= nx < width and 0 <= ny < height]
        return any(abs(base[pixel * 4 + c] - base[other * 4 + c]) > 3
                   for other in neighbors for c in range(3))
    mean = sum(delta) / len(delta)
    if len(changed) > max(16, len(delta) // 20000) or mean > 0.01 or any(not edge(pixel) for pixel in changed):
        raise ValueError(f"Translation drift: changed={len(changed)}, mean={mean:.6f}/255")
    metrics["translation"] = {"max_channel_delta": max(delta), "edge_pixels_over_3": len(changed),
                              "mean_pixel_max_delta": mean}
    coarse, fine = metrics["near-1024"], metrics["near-4096"]
    if fine["partial_pixels"] / fine["strong_pixels"] >= \
            0.5 * coarse["partial_pixels"] / coarse["strong_pixels"]:
        raise ValueError("Increasing map resolution did not narrow the filtered shadow edge")
    for mode in ("wide", "near", "near-taa"):
        bitmap(directory / f"shadow-terrain-{mode}.bmp")
        pipeline(f"terrain-{mode}", 2048)
    (directory / "shadow-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))
    print("Human-scale shadows: 1024/2048/4096, terrain, TAA and trillion-meter translation passed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    validate(parser.parse_args().directory)
