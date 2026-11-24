#pragma once

#include "collision.hpp"

#include <future>
#include <set>

namespace voxel {

struct WorldStats {
    BuildStats build;
    double selectMs{}, updateMs{}, physicsLoadMs{};
    std::size_t windowVisits{}, changedOwners{}, stagingBytes{};
    bool windowsChanged{};
    std::vector<Key> meshChanges;
};

class Clipmap {
public:
    Clipmap() = default;
    explicit Clipmap(int densityShift, TerrainElevationRange range = elevationRange(TerrainPreset::Classic))
        : densityShift_(densityShift), elevationRange_(range) {}
    void setDensityShift(int shift) { densityShift_ = shift; }
    int densityShift() const { return densityShift_; }
    std::set<Key> update(const std::vector<Vec>& invokers, bool global, std::size_t& visits,
                         std::uint32_t lod0Radius = defaultLod0Radius, bool rolling = false);
    std::vector<ClipmapWindow> windows() const;
    const Layout& layout() const { return layout_; }
    std::uint32_t lod0Radius() const { return lod0Radius_; }
    std::uint64_t windowRevision() const { return windowRevision_; }
    std::size_t bytes() const;
private:
    struct GlobalAnchor { Vec origin; double step; };
    int densityShift_{};
    TerrainElevationRange elevationRange_{elevationRange(TerrainPreset::Classic)};
    std::uint32_t lod0Radius_{defaultLod0Radius};
    std::vector<Key> anchors_;
    std::map<Key, std::array<unsigned, 4>> coverage_;
    std::vector<GlobalAnchor> globalAnchors_;
    std::vector<ClipmapWindow> globalWindows_;
    Layout idealLayout_;
    Layout layout_;
    std::uint64_t windowRevision_{};
    int nearRolls_{};
    bool global_{};
};

struct TerrainMemory {
    std::size_t fields{}, physics{}, meshes{}, metadata{};
    std::size_t total() const { return fields + physics + meshes + metadata; }
};

class World {
public:
    World(const Planet& planet, Method method, bool morton, double density = 1);
    void setVoxelDensity(double density) { densityShift_ = voxelDensityShift(density); }
    double voxelDensity() const { return std::ldexp(1.0, publishedDensityShift_); }
    void setLod0Radius(std::uint32_t radius);
    std::uint32_t lod0Radius() const { return publishedLod0Radius_; }
    bool hasFinestLod(Vec point) const;
    WorldStats update(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera = {});
    bool requestUpdate(const std::vector<Vec>& invokers, bool global, Vec camera, bool rolling = false);
    std::optional<WorldStats> pollUpdate();
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
    };
    Batch prepareUpdate(const std::vector<Vec>& sources, bool global, int densityShift,
                        std::uint32_t lod0Radius, Clock::time_point start, bool rolling);
    Batch planUpdate(const std::vector<Vec>& sources, bool global, int densityShift,
                     std::uint32_t lod0Radius, Clock::time_point start, bool rolling);
    static void build(Batch& batch, const Planet& planet, const Layout& layout, Method method, bool morton);
    WorldStats publish(Batch batch);
    const Planet& planet_;
    Method method_;
    bool morton_;
    int densityShift_{};
    std::uint32_t lod0Radius_{defaultLod0Radius};
    int publishedDensityShift_{};
    std::uint32_t publishedLod0Radius_{defaultLod0Radius};
    std::size_t publishedClipmapBytes_{};
    bool recoverPlan_{};
    Clipmap clipmap_;
    std::vector<ClipmapWindow> publishedWindows_;
    std::future<Batch> building_;
};

std::vector<Vec> invokerPositions(const Planet& planet, int count, double frame);
std::array<Vec, 3> frameAt(Vec up);
Shape sampleShape(int kind, Vec up);

}  // namespace voxel
