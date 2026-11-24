"""Check an exported global LOD mesh for open seams, allowing collinear T junctions."""
import argparse
from collections import Counter, defaultdict
import heapq
from pathlib import Path


def validate(path, tolerance=0.001):
    vertices = []
    counts = Counter()
    triangles = 0
    with path.open(encoding="utf-8") as source:
        for line in source:
            if line.startswith("v "):
                vertices.append(tuple(map(float, line.split()[1:4])))
            elif line.startswith("f "):
                ids = [int(x.split("/")[0]) - 1 for x in line.split()[1:]]
                if len(ids) != 3:
                    raise ValueError("Expected triangular VoxelWorld OBJ")
                triangles += 1
                for i in range(3):
                    a, b = vertices[ids[i]], vertices[ids[(i + 1) % 3]]
                    if a != b:
                        counts[tuple(sorted((a, b)))] += 1
    if not triangles:
        raise ValueError("Mesh is empty")
    if any(count > 2 for count in counts.values()):
        raise ValueError("Mesh has edges incident to more than two triangles")
    boundary = [edge for edge, count in counts.items() if count == 1]
    adjacent = defaultdict(list)
    for i, (a, b) in enumerate(boundary):
        adjacent[a].append(i)
        adjacent[b].append(i)
    missing = 0
    for i, (a, b) in enumerate(boundary):
        midpoint = tuple((x + y) * 0.5 for x, y in zip(a, b))
        distance = lambda p: sum((x - y) ** 2 for x, y in zip(p, midpoint))
        queue = [(distance(p), p) for p in (a, b)]
        heapq.heapify(queue)
        seen = {i}
        matched = False
        # A fine edge can lie within a coarse edge without sharing both vertices.
        # Search the adjacent boundary chain for another segment covering its midpoint.
        while queue and len(seen) < 512:
            _, p = heapq.heappop(queue)
            for j in adjacent[p]:
                if j in seen:
                    continue
                seen.add(j)
                c, d = boundary[j]
                delta = tuple(y - x for x, y in zip(c, d))
                square = sum(x * x for x in delta)
                t = max(0, min(1, sum((midpoint[k] - c[k]) * delta[k] for k in range(3)) / square))
                error = sum((midpoint[k] - c[k] - t * delta[k]) ** 2 for k in range(3))
                if error <= tolerance * tolerance:
                    matched = True
                    break
                other = d if c == p else c
                heapq.heappush(queue, (distance(other), other))
            if matched:
                break
        missing += not matched
    print(f"{triangles} triangles; {len(boundary)} split boundary edges; {missing} unmatched beyond {tolerance}m")
    if missing:
        raise ValueError("Global mesh has unmatched seam edges")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("obj", type=Path)
    validate(parser.parse_args().obj)
