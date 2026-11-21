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
    std::set<Key> update(const std::vector<Vec>& invokers, bool global, std::size_t& visits);
    std::vector<ClipmapWindow> windows() const;
    const Layout& layout() const { return layout_; }
    std::size_t bytes() const;
private:
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
    World(const Planet& planet, Method method, bool morton);
    WorldStats update(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera = {});
    bool requestUpdate(const std::vector<Vec>& invokers, bool global, Vec camera);
    std::optional<WorldStats> pollUpdate();
    std::array<int, 4> lodCounts() const;
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
    Batch planUpdate(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera);
    static void build(Batch& batch, const Planet& planet, const Layout& layout, Method method, bool morton);
    WorldStats publish(Batch batch);
    const Planet& planet_;
    Method method_;
    bool morton_;
    Clipmap clipmap_;
    std::set<Key> pending_;
    std::vector<ClipmapWindow> publishedWindows_;
    std::optional<WorldStats> completed_;
    std::future<Batch> building_;
};

std::vector<Vec> invokerPositions(const Planet& planet, int count, double frame);
std::array<Vec, 3> frameAt(Vec up);
Shape sampleShape(int kind, Vec up);

}  // namespace voxel
