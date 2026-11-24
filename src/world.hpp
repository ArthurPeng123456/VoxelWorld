#pragma once

#include "collision.hpp"

#include <future>
#include <set>

namespace voxel {

struct WorldStats {
    BuildStats build;
    double selectMs{}, updateMs{}, physicsLoadMs{};
    std::size_t windowVisits{}, changedOwners{}, stagingBytes{};
};

struct ClipmapWindow {
    std::size_t invoker;
    int lod;
    Box box;
};

class Clipmap {
public:
    Clipmap() = default;
    explicit Clipmap(int densityShift) : densityShift_(densityShift) {}
    void setDensityShift(int shift) { densityShift_ = shift; }
    int densityShift() const { return densityShift_; }
    std::set<Key> update(const std::vector<Vec>& invokers, bool global, std::size_t& visits);
    std::vector<ClipmapWindow> windows() const;
    const Layout& layout() const { return layout_; }
    std::size_t bytes() const;
private:
    int densityShift_{};
    std::vector<Key> anchors_;
    std::map<Key, std::array<unsigned, 4>> coverage_;
    Layout layout_;
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
    double voxelDensity() const { return std::ldexp(1.0, clipmap_.densityShift()); }
    WorldStats update(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera = {});
    bool requestUpdate(const std::vector<Vec>& invokers, bool global, Vec camera);
    std::optional<WorldStats> pollUpdate();
    std::array<int, lodCount> lodCounts() const;
    TerrainMemory terrainMemory() const;
    std::size_t meshBytes() const;
    const std::vector<ClipmapWindow>& clipmapWindows() const { return publishedWindows_; }

    Chunks chunks;
    Physics physics;
private:
    using Prior = std::map<Key, std::pair<int, std::array<int, 27>>>;
    struct Batch {
        Clock::time_point start;
        WorldStats stats;
        std::vector<Key> build, remove;
        Chunks staged;
        Clipmap clipmap;
    };
    Prior snapshot() const;
    static Batch planUpdate(Clipmap planned, const Prior& prior, const std::vector<Vec>& sources,
                            bool global, int densityShift, Clock::time_point start);
    static void build(Batch& batch, const Planet& planet, const Layout& layout, Method method, bool morton);
    WorldStats publish(Batch batch);
    const Planet& planet_;
    Method method_;
    bool morton_;
    int densityShift_{};
    Clipmap clipmap_;
    std::vector<ClipmapWindow> publishedWindows_;
    std::future<Batch> building_;
};

std::vector<Vec> invokerPositions(const Planet& planet, int count, double frame);
std::array<Vec, 3> frameAt(Vec up);
Shape sampleShape(int kind, Vec up);

}  // namespace voxel
