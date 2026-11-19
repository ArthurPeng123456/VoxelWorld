#include "world.hpp"

namespace voxel {

World::World(const Planet& planet, Method method, bool morton)
    : physics(method, chunks), planet_(planet), method_(method), morton_(morton) {}

WorldStats World::update(const std::vector<Vec>& invokers, bool global) {
    WorldStats stats;
    const auto start = Clock::now();
    const auto layout = selectChunks(invokers, global);
    stats.selectMs = milliseconds(start);
    for (auto it = chunks.begin(); it != chunks.end();) {
        const auto found = layout.find(it->first);
        if (found != layout.end() && it->second->neighborhood == neighborhood(layout, it->first)) {
            ++it;
            continue;
        }
        const auto tick = Clock::now();
        physics.removeTerrain(it->first);
        ++stats.build.removed;
        it = chunks.erase(it);
        stats.physicsLoadMs += milliseconds(tick);
    }
    for (const auto& entry : layout) {
        if (chunks.find(entry.first) != chunks.end()) continue;
        auto chunk = buildChunk(planet_, layout, entry.first, method_, morton_, stats.build);
        auto inserted = chunks.emplace(entry.first, std::move(chunk));
        const auto tick = Clock::now();
        physics.addTerrain(*inserted.first->second);
        stats.physicsLoadMs += milliseconds(tick);
    }
    stats.updateMs = milliseconds(start);
    return stats;
}

std::array<int, 4> World::lodCounts() const {
    std::array<int, 4> result{};
    for (const auto& entry : chunks) ++result[entry.second->lod];
    return result;
}

std::size_t World::fieldBytes() const {
    std::size_t result = 0;
    for (const auto& entry : chunks) {
        if (entry.second->field) result += entry.second->field->bytes() + entry.second->rows.capacity() * sizeof(CellRow);
    }
    return result;
}

std::size_t World::meshBytes() const {
    std::size_t result = 0;
    for (const auto& entry : chunks) result += entry.second->mesh.capacity() * sizeof(Triangle);
    return result;
}

std::vector<Vec> invokerPositions(const Planet& planet, int count, double frame) {
    std::vector<Vec> result;
    result.reserve(std::size_t(count));
    for (int i = 0; i < count; ++i) result.push_back(planet.surface(planet.invokerDirection(i, frame)));
    return result;
}

std::array<Vec, 3> frameAt(Vec up) {
    const Vec reference = std::abs(up.z) < 0.9 ? Vec{0, 0, 1} : Vec{0, 1, 0};
    const Vec x = unit(cross(reference, up));
    return {x, cross(up, x), up};
}

Shape sampleShape(int kind, Vec up) {
    if (kind == 1) return {ShapeKind::Sphere, {13, 0, 0}, frameAt(up)};
    if (kind == 2) return {ShapeKind::Capsule, {9, 0, 12}, frameAt(up)};
    return {ShapeKind::Box, {15, 11, 10}, frameAt(up)};
}

}  // namespace voxel
