#pragma once

#include "grid_density.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace voxel {

using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct Vec {
    double x{}, y{}, z{};
    double operator[](int axis) const { return axis == 0 ? x : axis == 1 ? y : z; }
    double& operator[](int axis) { return axis == 0 ? x : axis == 1 ? y : z; }
};
inline Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec operator*(Vec a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec operator/(Vec a, double s) { return a * (1.0 / s); }
inline double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec cross(Vec a, Vec b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(Vec a) { return std::sqrt(dot(a, a)); }
inline Vec unit(Vec a, Vec fallback = {0, 0, 1}) {
    const double size = length(a);
    return size > 1e-10 ? a / size : fallback;
}
inline Vec min(Vec a, Vec b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec max(Vec a, Vec b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline Vec abs(Vec a) { return {std::abs(a.x), std::abs(a.y), std::abs(a.z)}; }
inline Vec splat(double a) { return {a, a, a}; }

struct Box {
    Vec lo, hi;
    bool contains(Vec v) const {
        return v.x >= lo.x && v.y >= lo.y && v.z >= lo.z && v.x <= hi.x && v.y <= hi.y && v.z <= hi.z;
    }
    Box expanded(Vec by) const { return {lo - by, hi + by}; }
    Vec center() const { return (lo + hi) * 0.5; }
    Vec clamp(Vec p) const { return max(lo, min(hi, p)); }
};
bool clip(Box box, Vec start, Vec delta, double& first, double& last);

// World distances are meters. CPU positions, directions and transforms use Vec/double.
constexpr double planetRadius = 6371000.0;
constexpr double atmosphereHeight = 100000.0;
constexpr double chunkSize = 1600.0;
constexpr int fineCells = 32;
constexpr int fieldRootLevel() {
    int level = 0;
    double extent = chunkSize;
    while (extent < planetRadius + 1000) { extent *= 2; ++level; }
    return level;
}
constexpr int rootLevel = fieldRootLevel();
constexpr int coarsestSurfaceLevel = rootLevel > 3 ? rootLevel - 3 : 0;
constexpr int lodCount = rootLevel + 7; // The 0.125x grid requires three additional levels.
inline double gridChunkSize(int densityShift) { return std::ldexp(chunkSize, densityShift); }
inline int gridRootLevel(int densityShift) { return rootLevel - densityShift; }
constexpr float background = 2400.0f;

struct Key {
    int x{}, y{}, z{};
    // Lower corner in base-chunk units; side length is chunkSize * 2^level.
    int level{};
    int densityShift{};
    int operator[](int axis) const { return axis == 0 ? x : axis == 1 ? y : z; }
    int& operator[](int axis) { return axis == 0 ? x : axis == 1 ? y : z; }
    bool operator==(Key b) const { return x == b.x && y == b.y && z == b.z && level == b.level && densityShift == b.densityShift; }
    bool operator<(Key b) const { return std::tie(densityShift, level, x, y, z) < std::tie(b.densityShift, b.level, b.x, b.y, b.z); }
};
inline Vec asVec(Key key) { return {double(key.x), double(key.y), double(key.z)}; }
Key keyAt(Vec p, int densityShift = 0);
Box bounds(Key key);
bool intersectsPlanetShell(Key key);

class Planet {
public:
    explicit Planet(std::uint64_t seed);
    double elevation(Vec direction) const;
    float sdf(Vec p) const;
    Vec surface(Vec direction) const { return direction * (planetRadius + elevation(direction)); }
    Vec invokerDirection(int index, double frame) const;
private:
    struct Crater { Vec axis; double width, depth, rim; };
    std::vector<Crater> craters_;
    std::array<double, 3> phase_{};
};


enum class Method { Mesh, Grid, Dag, Vdb };
const char* name(Method method);

class Storage {
public:
    virtual ~Storage() = default;
    virtual float get(int x, int y, int z) const = 0;
    virtual std::array<float, 8> corners(Key index) const;
    virtual std::size_t bytes() const = 0;
};
std::unique_ptr<Storage> makeStorage(Method method, int count, const std::vector<float>& values,
                                     bool morton, double spacing, Vec origin);

struct Cell {
    Vec origin;
    double spacing;
    std::array<float, 8> values;
    double value(Vec p) const;
    Vec gradient(Vec p) const;
    double minimum() const { return *std::min_element(values.begin(), values.end()); }
    double maximum() const { return *std::max_element(values.begin(), values.end()); }
};

struct Triangle { std::array<Vec, 3> vertices; };
struct CellRow { std::uint32_t surface{}, solid{}; };
struct Chunk {
    Key key;
    int lod;
    int cells;
    Box box;
    double spacing;
    std::array<int, 27> neighborhood;
    std::unique_ptr<Storage> field;
    std::vector<Triangle> mesh;
    std::optional<Box> surfaceBounds;
    std::vector<CellRow> rows;
    const CellRow& row(int y, int z) const { return rows[std::size_t(z) * cells + y]; }
    Cell cell(Key index) const;
    Cell cellAt(Vec p) const;
    Key cellIndex(Vec p) const;
};

struct BuildStats {
    double sampleMs{}, storageMs{}, meshMs{};
    int built{}, removed{};
};

using Layout = std::map<Key, int>;
inline int layoutDensityShift(const Layout& layout) { return layout.empty() ? 0 : layout.begin()->first.densityShift; }
Layout planetLayout(const std::vector<Vec>& invokers, std::size_t* visits = nullptr, int densityShift = 0);
std::optional<Key> leafAt(const Layout& layout, Vec point);
std::vector<Key> intersectingLeaves(const Layout& layout, Box box);
Layout selectChunks(const std::vector<Vec>& invokers, bool global, int densityShift = 0);
std::array<int, 27> neighborhood(const Layout& layout, Key key);
std::unique_ptr<Chunk> buildChunk(const Planet& planet, const Layout& layout, Key key,
                                  Method method, bool morton, BuildStats& stats);
void meshChunk(Chunk& chunk, bool indexField = true);

}  // namespace voxel
