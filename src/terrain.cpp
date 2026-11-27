#include "terrain.hpp"
#include "../third_party/marching_cases.hpp"

#include <limits>
#include <stdexcept>

namespace voxel {

bool clip(Box box, Vec start, Vec delta, double& first, double& last) {
    for (int i = 0; i < 3; ++i) {
        if (std::abs(delta[i]) < 1e-13) {
            if (start[i] < box.lo[i] || start[i] > box.hi[i]) return false;
            continue;
        }
        double a = (box.lo[i] - start[i]) / delta[i];
        double b = (box.hi[i] - start[i]) / delta[i];
        if (a > b) std::swap(a, b);
        first = std::max(first, a);
        last = std::min(last, b);
        if (first > last) return false;
    }
    return true;
}

Key keyAt(Vec p, int densityShift) {
    p = p / gridChunkSize(densityShift);
    return {int(std::floor(p.x)), int(std::floor(p.y)), int(std::floor(p.z)), 0, densityShift};
}

Box bounds(Key key) {
    const double size = gridChunkSize(key.densityShift);
    const Vec lo = asVec(key) * size;
    return {lo, lo + splat(std::ldexp(size, key.level))};
}

const char* name(TerrainPreset preset) {
    switch (preset) {
        case TerrainPreset::Mars: return "mars";
        case TerrainPreset::Volcanic: return "volcanic";
        default: return "classic";
    }
}

TerrainPreset parseTerrainPreset(std::string_view value) {
    for (const auto preset : terrainPresets) if (value == name(preset)) return preset;
    throw std::invalid_argument("Choose classic, mars or volcanic for --terrain");
}

Planet::Planet(std::uint64_t seed, TerrainPreset preset) : preset_(preset) {
    if (preset_ != TerrainPreset::Classic) {
        buildGeology(seed);
        return;
    }
    auto random = [&seed] {
        seed += 0x9e3779b97f4a7c15ULL;
        auto value = seed;
        value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
        value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
        return ((value ^ (value >> 31)) >> 11) * 0x1p-53;
    };
    constexpr double tau = 6.283185307179586;
    for (auto& phase : phase_) phase = random() * tau;
    craters_.reserve(80);
    for (int i = 0; i < 80; ++i) {
        const double z = random() * 2 - 1;
        const double azimuth = random() * tau;
        const double r = std::sqrt(1 - z * z);
        const Vec axis{r * std::cos(azimuth), r * std::sin(azimuth), z};
        const double diameterRatio = (0.015 + 0.12 * std::pow(random(), 2)) * (25600.0 / planetRadius);
        craters_.push_back({axis, diameterRatio * diameterRatio * 0.5,
                            180 + random() * 560, 40 + random() * 95});
    }
    for (std::size_t i = 0; i < craters_.size(); ++i) {
        const auto& crater = craters_[i];
        // The angular cutoff is equivalent to this chord length on the unit sphere.
        const double reach = std::sqrt(3.92 * crater.width) + 1e-12;
        for (int octant = 0; octant < 8; ++octant) {
            bool overlaps = true;
            for (int axis = 0; axis < 3; ++axis) {
                const double sign = octant & (1 << axis) ? 1 : -1;
                overlaps &= crater.axis[axis] * sign >= -reach;
            }
            if (overlaps) craterOctants_[octant].push_back(i);
        }
    }
}

template<bool withGradient>
double Planet::sampleElevation(Vec d, Vec& gradient) const {
    double bowl = 0;
    double rim = 0;
    Vec bowlSlope{}, rimSlope{};
    const int octant = (d.x >= 0 ? 1 : 0) | (d.y >= 0 ? 2 : 0) | (d.z >= 0 ? 4 : 0);
    for (const auto index : craterOctants_[octant]) {
        const auto& crater = craters_[index];
        const double chord = 1 - dot(d, crater.axis);
        if (chord > 1.96 * crater.width) continue;
        const double radius = std::sqrt(std::max(0.0, chord / crater.width));
        const double depth = std::max(0.0, 1 - radius * radius);
        const double ridge = std::max(0.0, 1 - std::abs(radius - 0.99) / 0.4);
        const double nextBowl = crater.depth * depth * depth;
        const double nextRim = crater.rim * ridge * ridge;
        if (nextBowl > bowl) {
            bowl = nextBowl;
            if constexpr (withGradient) bowlSlope = crater.axis * (2 * crater.depth * depth / crater.width);
        }
        if (nextRim > rim) {
            rim = nextRim;
            if constexpr (withGradient) {
                const double slope = radius < 0.99 ? -1 : radius > 0.99 ? 1 : 0;
                rimSlope = crater.axis * (crater.rim * ridge * slope / (0.4 * crater.width * radius));
            }
        }
    }
    constexpr double frequency = planetRadius / 25600.0;
    constexpr Vec directions[] = {{14, 11, -7}, {29, -19, 25}, {-70, 59, 91}};
    constexpr double amplitudes[] = {85, 50, 20};
    double relief = 0;
    if constexpr (withGradient) gradient = rimSlope - bowlSlope;
    for (int i = 0; i < 3; ++i) {
        const double phase = dot(d, directions[i]) * frequency + phase_[i];
        relief += amplitudes[i] * std::sin(phase);
        if constexpr (withGradient) gradient = gradient + directions[i] * (amplitudes[i] * frequency * std::cos(phase));
    }
    return relief + rim - bowl;
}

double Planet::elevation(Vec direction) const {
    Vec unused;
    if (preset_ != TerrainPreset::Classic) return sampleGeology<false>(direction, unused, nullptr, 0);
    return sampleElevation<false>(direction, unused);
}

PlanetSurfaceSample Planet::sampleSurface(Vec direction, double spacing, bool sampleNormal) const {
    Vec gradient;
    double craterMaterial = 0;
    double height;
    if (preset_ == TerrainPreset::Classic) height = sampleElevation<true>(direction, gradient);
    else if (sampleNormal) height = sampleGeology<true>(direction, gradient, &craterMaterial, spacing);
    else height = sampleGeology<false>(direction, gradient, &craterMaterial, spacing);
    const Vec tangent = gradient - direction * dot(gradient, direction);
    return {height, unit(direction - tangent / (planetRadius + height), direction), craterMaterial};
}

float Planet::sdf(Vec p) const {
    const double r = length(p);
    // Coarse planetary cells need actual radial distances: clamping them to a
    // 2400m band would move roots toward cell midpoints hundreds of km away.
    return float(r - planetRadius - elevation(unit(p)));
}

bool Planet::mayContainSurface(Box box) const {
    const Vec center = box.center();
    const double radius = length(box.hi - center), minimumRadius = length(center) - radius;
    if (minimumRadius <= 0) return true;
    // Conservative interval from the same analytical gradient bound used by
    // certifyUniform. No corner-only test: small craters must not disappear.
    const double variation = radius * (1 + 12 * planetRadius / minimumRadius) + 0.01;
    const double margin = std::max(16.0, 2 * (box.hi.x - box.lo.x) / fineCells);
    return std::abs(sdf(center)) <= variation + margin;
}

Vec Planet::invokerDirection(int index, double frame) const {
    const Vec axis = preset_ == TerrainPreset::Classic ? craters_[std::size_t(index) % craters_.size()].axis
                                                       : impacts_[std::size_t(index) % impacts_.size()].axis;
    const Vec reference = std::abs(axis.z) < 0.9 ? Vec{0, 0, 1} : Vec{0, 1, 0};
    const Vec tangent = unit(cross(reference, axis));
    const double angle = (0.07 * 25600.0 / planetRadius)
                       * std::sin(frame * ((index % 2) == 0 ? 0.039 : -0.051));
    return axis * std::cos(angle) + tangent * std::sin(angle);
}

const char* name(Method method) {
    switch (method) {
        case Method::Mesh: return "mesh";
        case Method::Grid: return "grid";
        case Method::Dag: return "dag";
        case Method::Vdb: return "vdb";
    }
    return "unknown";
}

double Cell::value(Vec p) const {
    const Vec t = (p - origin) / spacing;
    const auto lerp = [](double a, double b, double fraction) { return a + (b - a) * fraction; };
    const double z0 = lerp(lerp(values[0], values[1], t.x), lerp(values[2], values[3], t.x), t.y);
    const double z1 = lerp(lerp(values[4], values[5], t.x), lerp(values[6], values[7], t.x), t.y);
    return lerp(z0, z1, t.z);
}

Vec Cell::gradient(Vec p) const {
    const Vec t = (p - origin) / spacing;
    Vec result{};
    for (int axis = 0; axis < 3; ++axis) {
        const int bit = 1 << axis;
        const int a = (axis + 1) % 3;
        const int b = (axis + 2) % 3;
        for (int i = 0; i < 8; ++i) {
            if (i & bit) continue;
            result[axis] += (values[i | bit] - values[i]) * ((i & (1 << a)) ? t[a] : 1 - t[a])
                                                        * ((i & (1 << b)) ? t[b] : 1 - t[b]);
        }
    }
    return result / spacing;
}

Cell Chunk::cell(Key index) const {
    return {box.lo + asVec(index) * spacing, spacing, field->corners(index)};
}

Key Chunk::cellIndex(Vec p) const {
    const Vec cell = (p - box.lo) / spacing;
    Key result;
    for (int axis = 0; axis < 3; ++axis) {
        result[axis] = std::clamp(int(std::floor(cell[axis])), 0, cells - 1);
    }
    return result;
}

Cell Chunk::cellAt(Vec p) const { return cell(cellIndex(p)); }

bool intersectsPlanetShell(Key key, TerrainElevationRange range) {
    const auto box = bounds(key);
    const double rMin = length(box.clamp({}));
    const double rMax = length(max(abs(box.lo), abs(box.hi)));
    return rMin <= planetRadius + range.upper && rMax >= planetRadius + range.lower;
}

Layout selectChunks(const std::vector<Vec>& invokers, bool global, int densityShift) {
    if (global) return planetLayout(invokers, nullptr, densityShift);
    const double size = gridChunkSize(densityShift);
    Layout result;
    auto offer = [&](Key key) {
        const Box box = bounds(key);
        if (!intersectsPlanetShell(key)) return;
        double distance = std::numeric_limits<double>::infinity();
        for (Vec invoker : invokers) distance = std::min(distance, length(box.clamp(invoker) - invoker));
        if (distance > 2.3 * size) return;
        const int lod = distance < 0.45 * size ? 0 : distance < 1.2 * size ? 1
                       : distance < 2.0 * size ? 2 : 3;
        result.try_emplace(key, lod);
    };
    {
        for (Vec p : invokers) {
            const Key center = keyAt(p, densityShift);
            for (int z = -3; z <= 3; ++z) for (int y = -3; y <= 3; ++y) {
                for (int x = -3; x <= 3; ++x) offer({center.x + x, center.y + y, center.z + z, 0, densityShift});
            }
        }
    }
    return result;
}

std::array<int, 27> neighborhood(const Layout& layout, Key key) {
    std::array<int, 27> result;
    const int lod = layout.at(key);
    int i = 0;
    for (int z = -1; z <= 1; ++z) for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            const Box box = bounds(key);
            Vec p = box.center();
            for (int a = 0; a < 3; ++a) {
                const int side = a == 0 ? x : a == 1 ? y : z;
                if (side) p[a] = side < 0 ? box.lo[a] - 1e-4 : box.hi[a] + 1e-4;
            }
            const auto neighbor = leafAt(layout, p);
            // Only a coarser neighbor constrains the boundary samples or stitching of this chunk.
            result[i++] = neighbor ? std::max(lod, layout.at(*neighbor)) : lod;
        }
    }
    return result;
}

namespace {

int floorTo(int value, int step) { return value - ((value % step + step) % step); }

class BoundarySampler {
public:
    BoundarySampler(const Planet& planet, const Layout& layout)
        : planet_(planet), layout_(layout), finestSpacing_(gridChunkSize(layoutDensityShift(layout)) / fineCells),
          generation_(++cache_.generation) {}

    float value(Key grid, int level) {
        // Neighboring fine face samples interpolate the same coarse lattice
        // points. Their values and owner lookups are immutable within a build.
        const auto hash = std::uint32_t(grid.x) * 0x9e3779b9U ^ std::uint32_t(grid.y) * 0x85ebca77U ^
                          std::uint32_t(grid.z) * 0xc2b2ae3dU ^ std::uint32_t(level) * 0x27d4eb2dU;
        auto& entry = cache_.entries[(hash ^ (hash >> 16)) & (cache_.entries.size() - 1)];
        if (entry.generation == generation_ && entry.grid == grid && entry.level == level) return entry.value;
        int coarsest = level;
        const Vec world = asVec(grid) * finestSpacing_;
        for (int i = 0; i < 8; ++i) {
            const Vec probe = world + Vec{(i & 1) ? 1e-4 : -1e-4, (i & 2) ? 1e-4 : -1e-4, (i & 4) ? 1e-4 : -1e-4};
            const auto neighbor = leafAt(layout_, probe);
            if (neighbor) coarsest = std::max(coarsest, layout_.at(*neighbor));
        }
        const float result = interpolate(grid, coarsest);
        entry = {grid, level, result, generation_};
        return result;
    }

    float interpolate(Key grid, int coarsest) {
        const int stride = 1 << coarsest;
        const Key base{floorTo(grid.x, stride), floorTo(grid.y, stride), floorTo(grid.z, stride)};
        const Vec fraction = (asVec(grid) - asVec(base)) / stride;
        if (fraction.x == 0 && fraction.y == 0 && fraction.z == 0)
            return planet_.sdf(asVec(grid) * finestSpacing_);
        double interpolated = 0;
        for (int i = 0; i < 8; ++i) {
            const double weight = ((i & 1) ? fraction.x : 1 - fraction.x)
                                * ((i & 2) ? fraction.y : 1 - fraction.y)
                                * ((i & 4) ? fraction.z : 1 - fraction.z);
            if (weight == 0) continue;
            const Key point{base.x + (i & 1) * stride, base.y + ((i >> 1) & 1) * stride,
                            base.z + (i >> 2) * stride};
            interpolated += weight * value(point, coarsest);
        }
        return float(interpolated);
    }
private:
    struct Entry { Key grid; int level{}; float value{}; std::uint64_t generation{}; };
    struct Cache { std::array<Entry, 4096> entries{}; std::uint64_t generation{}; };
    static thread_local Cache cache_;
    const Planet& planet_;
    const Layout& layout_;
    double finestSpacing_;
    std::uint64_t generation_;
};

thread_local BoundarySampler::Cache BoundarySampler::cache_{};

int boundaryLod(const Chunk& chunk, int x, int y, int z) {
    const int samples[3]{x, y, z};
    int lo[3], hi[3];
    for (int axis = 0; axis < 3; ++axis) {
        lo[axis] = samples[axis] == 0 ? 0 : 1;
        hi[axis] = samples[axis] == chunk.cells ? 2 : 1;
    }
    int lod = chunk.lod;
    // A coarser neighbor is at least as wide as this chunk and shares the
    // lattice, so it spans the entire touching sector on the face or edge.
    for (int k = lo[2]; k <= hi[2]; ++k) for (int j = lo[1]; j <= hi[1]; ++j)
        for (int i = lo[0]; i <= hi[0]; ++i) lod = std::max(lod, chunk.neighborhood[i + 3 * j + 9 * k]);
    return lod;
}

bool certifyUniform(const Planet& planet, Vec center, double radius, double sign, double margin,
                    int subdivisions, double& closest) {
    const double distance = sign * planet.sdf(center);
    if (distance <= margin) return false;
    // All presets have a relief-gradient bound below 12 m/m on the reference
    // sphere (Classic's maximum bowl/rim slopes and three waves total < 6).
    // Along a segment inside this box,
    // normalization can amplify that bound by at most R / minRadius.
    const double minimumRadius = length(center) - radius;
    if (minimumRadius <= 0) return false;
    const double variation = radius * (1 + 12 * planetRadius / minimumRadius) + 0.01;
    if (distance - variation > margin) {
        closest = std::min(closest, distance - variation);
        return true;
    }
    if (!subdivisions) return false;
    constexpr double inverseSqrt3 = 0.5773502691896258;
    const double offset = radius * (0.5 * inverseSqrt3);
    for (int child = 0; child < 8; ++child) {
        const Vec direction{child & 1 ? 1.0 : -1.0, child & 2 ? 1.0 : -1.0, child & 4 ? 1.0 : -1.0};
        if (!certifyUniform(planet, center + direction * offset, radius * 0.5, sign, margin, subdivisions - 1, closest))
            return false;
    }
    return true;
}

void stitchHierarchicalFaces(Chunk& chunk, BoundarySampler& boundary, const Layout& layout) {
    using Segment = std::array<Vec, 2>;
    using Face = std::tuple<int, int, int, int, int>;
    std::map<Face, std::vector<Segment>> contours;
    for (auto& triangle : chunk.mesh) for (Vec& point : triangle.vertices) {
        for (int axis = 0; axis < 3; ++axis) for (int side : {-1, 1}) {
            const double plane = side < 0 ? chunk.box.lo[axis] : chunk.box.hi[axis];
            if (std::abs(point[axis] - plane) > 1e-5) continue;
            Vec probe = point;
            for (int a = 0; a < 3; ++a)
                probe[a] = a == axis ? plane + side * 1e-4
                                    : std::clamp(point[a], chunk.box.lo[a] + 1e-4, chunk.box.hi[a] - 1e-4);
            const auto neighbor = leafAt(layout, probe);
            if (!neighbor || layout.at(*neighbor) <= chunk.lod) continue;
            const int lod = layout.at(*neighbor), step = 1 << lod;
            const double spacing = std::ldexp(gridChunkSize(chunk.key.densityShift) / fineCells, lod);
            const Box neighborBox = bounds(*neighbor);
            Key grid;
            for (int a = 0; a < 3; ++a) {
                const int cells = chunkCells(*neighbor, lod);
                const int index = std::clamp(int(std::floor((probe[a] - neighborBox.lo[a]) / spacing)), 0, cells - 1);
                grid[a] = (*neighbor)[a] * fineCells + index * step;
            }
            const Face face{axis, side, grid.x, grid.y, grid.z};
            auto found = contours.find(face);
            if (found == contours.end()) {
                const Vec origin = asVec(grid) * (gridChunkSize(chunk.key.densityShift) / fineCells);
                Chunk coarse{Key{}, lod, 1, {origin, origin + splat(spacing)}, spacing, {}, {}, {}, {}, {}};
                std::vector<float> samples;
                for (int i = 0; i < 8; ++i)
                    samples.push_back(boundary.value({grid.x + (i & 1) * step,
                        grid.y + ((i >> 1) & 1) * step, grid.z + (i >> 2) * step}, lod));
                coarse.field = makeStorage(Method::Grid, 2, samples, false, spacing, origin);
                meshChunk(coarse, false);
                std::vector<Segment> segments;
                for (const auto& t : coarse.mesh) for (int i = 0; i < 3; ++i) {
                    const Vec a = t.vertices[i], b = t.vertices[(i + 1) % 3];
                    if (std::abs(a[axis] - plane) < 1e-5 && std::abs(b[axis] - plane) < 1e-5 && length(a - b) > 1e-8)
                        segments.push_back({a, b});
                }
                found = contours.emplace(face, std::move(segments)).first;
            }
            Vec nearest = point;
            double distance = std::numeric_limits<double>::infinity();
            for (const auto& segment : found->second) {
                const Vec delta = segment[1] - segment[0];
                const double square = dot(delta, delta);
                double t = std::clamp(dot(point - segment[0], delta) / square, 0.0, 1.0);
                bool valid = true;
                // At chunk edges, preserve the other shared face as well.
                for (int a = 0; a < 3; ++a) if (a != axis) {
                    if (std::abs(point[a] - chunk.box.lo[a]) < 1e-5 || std::abs(point[a] - chunk.box.hi[a]) < 1e-5) {
                        if (std::abs(delta[a]) > 1e-12) t = (point[a] - segment[0][a]) / delta[a];
                        else if (std::abs(segment[0][a] - point[a]) > 1e-5) valid = false;
                    }
                }
                if (!valid || t < -1e-10 || t > 1 + 1e-10) continue;
                const Vec candidate = segment[0] + delta * std::clamp(t, 0.0, 1.0);
                const double d = dot(candidate - point, candidate - point);
                if (d < distance) { distance = d; nearest = candidate; }
            }
            point = nearest;
        }
    }
    chunk.mesh.erase(std::remove_if(chunk.mesh.begin(), chunk.mesh.end(), [](const Triangle& t) {
        const Vec area = cross(t.vertices[1] - t.vertices[0], t.vertices[2] - t.vertices[0]);
        return dot(area, area) < 1e-16;
    }), chunk.mesh.end());
}

void stitchFaces(Chunk& chunk, BoundarySampler& boundary, const Layout& layout) {
    if (!layout.empty() && layout.rbegin()->first.level > 0) {
        stitchHierarchicalFaces(chunk, boundary, layout);
        return;
    }
    // Collapse the fine contour onto each unambiguous straight coarse face segment.
    // Selection is based only on the layout, so it does not depend on build order.
    for (int axis = 0; axis < 3; ++axis) for (int side : {-1, 1}) {
        Key neighbor = chunk.key;
        neighbor[axis] += side;
        const auto it = layout.find(neighbor);
        if (it == layout.end() || it->second <= chunk.lod) continue;
        const int level = it->second;
        const int step = 1 << level;
        const int count = fineCells / step;
        const double spacing = gridChunkSize(chunk.key.densityShift) / count;
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        const double face = side < 0 ? chunk.box.lo[axis] : chunk.box.hi[axis];
        std::vector<std::optional<std::array<Vec, 2>>> segments(std::size_t(count) * count);
        constexpr int offsets[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        for (int j = 0; j < count; ++j) for (int i = 0; i < count; ++i) {
            std::array<Vec, 4> points;
            std::array<float, 4> values;
            for (int c = 0; c < 4; ++c) {
                Key key{chunk.key.x * fineCells, chunk.key.y * fineCells, chunk.key.z * fineCells};
                key[axis] += side > 0 ? fineCells : 0;
                key[u] += (i + offsets[c][0]) * step;
                key[v] += (j + offsets[c][1]) * step;
                points[c] = asVec(key) * (gridChunkSize(chunk.key.densityShift) / fineCells);
                values[c] = boundary.value(key, level);
            }
            std::array<Vec, 4> crossings;
            int n = 0;
            for (int a = 0; a < 4; ++a) {
                const int b = (a + 1) % 4;
                if ((values[a] < 0) == (values[b] < 0)) continue;
                const double f = values[a] / (double(values[a]) - values[b]);
                crossings[n++] = points[a] + (points[b] - points[a]) * f;
            }
            if (n == 2) segments[std::size_t(j) * count + i] = std::array<Vec, 2>{crossings[0], crossings[1]};
        }
        auto onEdge = [&](Vec p, int a) {
            return std::abs(p[a] - chunk.box.lo[a]) < 1e-7 || std::abs(p[a] - chunk.box.hi[a]) < 1e-7;
        };
        for (auto& triangle : chunk.mesh) {
            auto& points = triangle.vertices;
            const Vec oldNormal = cross(points[1] - points[0], points[2] - points[0]);
            for (Vec& p : points) {
                if (std::abs(p[axis] - face) > 1e-7 || onEdge(p, u) || onEdge(p, v)) continue;
                const int i = std::clamp(int((p[u] - chunk.box.lo[u]) / spacing), 0, count - 1);
                const int j = std::clamp(int((p[v] - chunk.box.lo[v]) / spacing), 0, count - 1);
                const auto& segment = segments[std::size_t(j) * count + i];
                if (!segment) continue;
                const Vec delta = (*segment)[1] - (*segment)[0];
                const double square = dot(delta, delta);
                if (square < 1e-16) continue;
                const double f = std::clamp(dot(p - (*segment)[0], delta) / square, 0.0, 1.0);
                p = (*segment)[0] + delta * f;
            }
            if (dot(oldNormal, cross(points[1] - points[0], points[2] - points[0])) < 0) {
                std::swap(points[1], points[2]);
            }
        }
    }
    chunk.mesh.erase(std::remove_if(chunk.mesh.begin(), chunk.mesh.end(), [](const Triangle& triangle) {
        const auto& p = triangle.vertices;
        const Vec area = cross(p[1] - p[0], p[2] - p[0]);
        return dot(area, area) < 1e-16;
    }), chunk.mesh.end());
}

}  // namespace

void meshChunk(Chunk& chunk, bool indexField) {
    // The table uses VTK's corner order and swaps the final two vertical edges.
    constexpr int corner[8] = {0, 1, 3, 2, 4, 5, 7, 6};
    constexpr int edge[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                                  {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    static_assert(fineCells <= 32);
    if (indexField) chunk.rows.resize(std::size_t(chunk.cells) * chunk.cells);
    for (int z = 0; z < chunk.cells; ++z) for (int y = 0; y < chunk.cells; ++y) {
        auto* row = indexField ? &chunk.rows[std::size_t(z) * chunk.cells + y] : nullptr;
        for (int x = 0; x < chunk.cells; ++x) {
            const Cell cell = chunk.cell({x, y, z});
            int mask = 0;
            for (int i = 0; i < 8; ++i) if (cell.values[corner[i]] < 0) mask |= 1 << i;
            if (row) {
                if (mask == 255 && cell.maximum() < -1e-6) {
                    row->solid |= std::uint32_t(1) << x;
                    continue;
                }
                if (mask == 0 && cell.minimum() > 1e-6) continue;
                row->surface |= std::uint32_t(1) << x;
                const Box active{cell.origin, cell.origin + splat(cell.spacing)};
                if (chunk.surfaceBounds) {
                    auto& prior = *chunk.surfaceBounds;
                    prior = {min(prior.lo, active.lo), max(prior.hi, active.hi)};
                } else chunk.surfaceBounds = active;
            }
            if (mask == 0 || mask == 255) continue;
            const auto& table = marchingCases[mask];
            for (int k = 0; table[k] >= 0; k += 3) {
                Triangle triangle;
                for (int i = 0; i < 3; ++i) {
                    const int a = edge[table[k + i]][0], b = edge[table[k + i]][1];
                    const Vec u{double(a & 1), double((a >> 1) & 1), double(a >> 2)};
                    const Vec v{double(b & 1), double((b >> 1) & 1), double(b >> 2)};
                    const double f = cell.values[a] / (double(cell.values[a]) - cell.values[b]);
                    triangle.vertices[i] = cell.origin + (u + (v - u) * f) * cell.spacing;
                }
                auto& p = triangle.vertices;
                const Vec normal = cross(p[1] - p[0], p[2] - p[0]);
                if (dot(normal, normal) < 1e-16) continue;
                if (dot(normal, cell.gradient((p[0] + p[1] + p[2]) / 3)) < 0) std::swap(p[1], p[2]);
                chunk.mesh.push_back(triangle);
            }
        }
    }
}

std::unique_ptr<Chunk> buildChunk(const Planet& planet, const Layout& layout, Key key,
                                  Method method, bool morton, BuildStats& stats, const Chunk* previous) {
    const int lod = layout.at(key);
    const int cells = chunkCells(key, lod);
    const int count = cells + 1;
    const auto box = bounds(key);
    const bool shell = intersectsPlanetShell(key, planet.elevationRange());
    auto chunk = std::make_unique<Chunk>(Chunk{key, lod, cells, box, (box.hi.x - box.lo.x) / cells,
                                               shell ? neighborhood(layout, key) : std::array<int, 27>{}, {}, {}, {}, {}});
    auto start = Clock::now();
    const auto storeUniform = [&](float value) {
        if (method != Method::Mesh) {
            chunk->field = makeStorage(method, count, {value}, morton, chunk->spacing, box.lo);
            const auto solidMask = value < 0 ? std::uint32_t((std::uint64_t(1) << cells) - 1) : 0;
            chunk->rows.assign(1, CellRow{0, solidMask});
        }
        stats.storageMs += milliseconds(start);
        ++stats.built;
    };
    if (!shell) {
        storeUniform(length(box.center()) < planetRadius ? -background : background);
        return chunk;
    }
    const int stride = 1 << lod;
    BoundarySampler boundary(planet, layout);
    const bool hasCoarserEdge = *std::max_element(chunk->neighborhood.begin(), chunk->neighborhood.end()) > lod;
    const auto sampleBoundary = [&](int x, int y, int z, int coarsest) {
        const Key grid{key.x * fineCells + x * stride, key.y * fineCells + y * stride,
                       key.z * fineCells + z * stride};
        return boundary.interpolate(grid, coarsest);
    };
    const double margin = std::max(16.0, 2 * chunk->spacing);
    {
        const auto center = box.center();
        const double prior = previous && previous->key == key ? previous->certifiedDistance : 0;
        const bool reuseProof = std::abs(prior) > margin;
        const double sign = (reuseProof ? prior : planet.sdf(center)) < 0 ? -1.0 : 1.0;
        double closest = reuseProof ? std::abs(prior) : std::numeric_limits<double>::infinity();
        const int depth = cells >= 16 ? 3 : cells >= 8 ? 2 : 1;
        bool uniform = reuseProof || certifyUniform(planet, center, length(box.hi - center), sign, margin, depth, closest);
        if (uniform) chunk->certifiedDistance = sign * closest;
        if (uniform && hasCoarserEdge) {
            // A coarse seam overrides analytical boundary samples. Check that
            // they also stay on the certified side of the zero surface.
            for (int axis = 0; axis < 3 && uniform; ++axis) for (int face : {0, cells}) {
                for (int i = 0; i <= cells && uniform; ++i) for (int j = 0; j <= cells && uniform; ++j) {
                    Key p;
                    p[axis] = face; p[(axis + 1) % 3] = i; p[(axis + 2) % 3] = j;
                    const int coarsest = boundaryLod(*chunk, p.x, p.y, p.z);
                    if (coarsest == lod) continue;
                    closest = std::min(closest, sign * sampleBoundary(p.x, p.y, p.z, coarsest));
                    uniform = closest > margin;
                }
            }
        }
        if (uniform) {
            stats.sampleMs += milliseconds(start);
            start = Clock::now();
            storeUniform(float(sign * closest));
            return chunk;
        }
    }
    // Only immutable, full fields may supply old samples. OpenVDB's live
    // accessor caches traversal state and is used by collision on the main thread.
    const int priorStride = previous && previous->key == key && previous->field && previous->surfaceBounds && method != Method::Vdb
                         ? 1 << previous->lod : 0;
    std::vector<float> samples;
    samples.reserve(std::size_t(count) * count * count);
    for (int z = 0; z < count; ++z) for (int y = 0; y < count; ++y) {
        for (int x = 0; x < count; ++x) {
            const bool onBoundary = x == 0 || y == 0 || z == 0 || x == cells || y == cells || z == cells;
            const int coarsest = onBoundary && hasCoarserEdge ? boundaryLod(*chunk, x, y, z) : lod;
            if (coarsest != lod) samples.push_back(sampleBoundary(x, y, z, coarsest));
            else if (!onBoundary && priorStride && ((x * stride | y * stride | z * stride) & (priorStride - 1)) == 0)
                samples.push_back(previous->field->get(x * stride / priorStride, y * stride / priorStride, z * stride / priorStride));
            else samples.push_back(planet.sdf(box.lo + Vec{double(x), double(y), double(z)} * chunk->spacing));
        }
    }
    stats.sampleMs += milliseconds(start);
    start = Clock::now();
    {
        // Test the final samples, including the coarser boundary interpolation.
        // Trilinear cells cannot cross zero when all corners are on one side.
        // Keep signed distances in the band used by contact and seam queries.
        const auto [lowest, highest] = std::minmax_element(samples.begin(), samples.end());
        if (*lowest > margin || *highest < -margin) {
            storeUniform(*lowest > margin ? *lowest : *highest);
            return chunk;
        }
    }
    chunk->field = makeStorage(method, count, samples, morton, chunk->spacing, box.lo);
    stats.storageMs += milliseconds(start);
    start = Clock::now();
    meshChunk(*chunk, method != Method::Mesh);
    if (hasCoarserEdge) stitchFaces(*chunk, boundary, layout);
    stats.meshMs += milliseconds(start);
    if (method == Method::Mesh) chunk->field.reset();
    ++stats.built;
    return chunk;
}

}  // namespace voxel
