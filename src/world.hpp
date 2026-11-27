#pragma once

#include "collision.hpp"

#include <functional>
#include <future>
#include <set>

namespace voxel {

struct WorldStats {
    BuildStats build;
    double selectMs{}, updateMs{}, physicsLoadMs{};
    std::size_t windowVisits{}, changedOwners{}, stagingBytes{};
    bool windowsChanged{};
};

struct ClipmapForecast {
    std::vector<Vec> displacement;
    std::size_t primary{};
    std::vector<int> minimumLod; // Omitted sources request every level; coarse sources can refine later.
};

class Clipmap {
public:
    Clipmap() = default;
    explicit Clipmap(int densityShift, TerrainElevationRange range = elevationRange(TerrainPreset::Classic), const Planet* planet = nullptr)
        : densityShift_(densityShift), elevationRange_(range), planet_(planet) {}
    void setDensityShift(int shift) { densityShift_ = shift; }
    int densityShift() const { return densityShift_; }
    std::set<Key> update(const std::vector<Vec>& invokers, bool global, std::size_t& visits,
                         std::uint32_t lod0Cells = defaultLod0Cells, bool rolling = false,
                         const ClipmapForecast& forecast = {});
    std::vector<ClipmapWindow> windows() const;
    const Layout& layout() const { return layout_; }
    std::uint32_t lod0Cells() const { return lod0Cells_; }
    std::uint64_t windowRevision() const { return windowRevision_; }
    std::size_t bytes() const;
private:
    struct GlobalAnchor { Vec origin; double step; };
    int densityShift_{};
    TerrainElevationRange elevationRange_{elevationRange(TerrainPreset::Classic)};
    const Planet* planet_{};
    std::uint32_t lod0Cells_{defaultLod0Cells};
    std::vector<Key> anchors_;
    std::map<Key, std::array<unsigned, 4>> coverage_;
    std::vector<GlobalAnchor> globalAnchors_;
    std::vector<ClipmapWindow> globalWindows_;
    Layout idealLayout_;
    Layout layout_;
    std::uint64_t windowRevision_{};
    std::vector<int> nearRolls_;
    bool global_{};
};

struct TerrainMemory {
    std::size_t fields{}, physics{}, meshes{}, metadata{};
    std::size_t total() const { return fields + physics + meshes + metadata; }
};

class World {
public:
    using TerrainPreparation = std::function<void(const Chunks& live, const Chunks& staged, const std::vector<Key>& removed)>;
    World(const Planet& planet, Method method, bool morton, double density = 1, TerrainPreparation prepareTerrain = {});
    void setVoxelDensity(double density) { densityShift_ = voxelDensityShift(density); }
    double voxelDensity() const { return std::ldexp(1.0, publishedDensityShift_); }
    void setLod0Cells(std::uint32_t cells);
    std::uint32_t lod0Cells() const { return publishedLod0Cells_; }
    bool hasFinestLod(Vec point) const;
    WorldStats update(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera = {}, ClipmapForecast forecast = {});
    bool updatePending() const { return building_.valid() || ready_.has_value(); }
    bool requestUpdate(const std::vector<Vec>& invokers, bool global, Vec camera, bool rolling = false,
                       ClipmapForecast forecast = {});
    std::optional<WorldStats> pollUpdate(const std::function<bool()>& uploadReady = {});
    std::array<int, lodCount> lodCounts() const;
    TerrainMemory terrainMemory() const;
    std::size_t meshBytes() const;
    const std::vector<ClipmapWindow>& clipmapWindows() const { return publishedWindows_; }

    Chunks chunks;
    Physics physics;
private:
    struct Batch {
        Clock::time_point start;
        WorldStats stats;
        std::vector<Key> build, remove;
        Chunks staged;
        std::map<Key, Physics::Terrain> colliders;
    };
    Batch prepareUpdate(const std::vector<Vec>& sources, bool global, int densityShift,
                        std::uint32_t lod0Cells, Clock::time_point start, bool rolling, const ClipmapForecast& forecast);
    Batch planUpdate(const std::vector<Vec>& sources, bool global, int densityShift,
                     std::uint32_t lod0Cells, Clock::time_point start, bool rolling, const ClipmapForecast& forecast);
    void build(Batch& batch, const Layout& layout);
    WorldStats publish(Batch batch);
    const Planet& planet_;
    Method method_;
    bool morton_;
    int densityShift_{};
    std::uint32_t lod0Cells_{defaultLod0Cells};
    int publishedDensityShift_{};
    std::uint32_t publishedLod0Cells_{defaultLod0Cells};
    std::size_t publishedClipmapBytes_{};
    std::array<int, lodCount> lodCounts_{};
    std::size_t fieldBytes_{}, meshBytes_{};
    std::uint64_t terrainRevision_{};
    bool recoverPlan_{};
    Clipmap clipmap_;
    std::vector<ClipmapWindow> publishedWindows_;
    TerrainPreparation prepareTerrain_;
    Chunks retiredChunks_;
    std::vector<Physics::Terrain> retiredColliders_;
    std::optional<Batch> ready_;
    std::future<Batch> building_;
};

std::vector<Vec> invokerPositions(const Planet& planet, int count, double frame);
std::array<Vec, 3> frameAt(Vec up);
Shape sampleShape(int kind, Vec up);

}  // namespace voxel
