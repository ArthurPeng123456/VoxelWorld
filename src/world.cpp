#include "world.hpp"

#include <stdexcept>
#include <utility>

namespace voxel {
namespace {

constexpr int radii[] = {1, 2, 3, 4};
constexpr double anchorMargin = 0.125 * chunkSize;
constexpr std::size_t mapNodeOverhead = 4 * sizeof(void*);

struct KeyBox { Key lo, hi; }; // Half-open in chunk coordinates.

KeyBox window(Key anchor, int lod) {
    const int r = radii[lod];
    return {{anchor.x - r, anchor.y - r, anchor.z - r}, {anchor.x + r + 1, anchor.y + r + 1, anchor.z + r + 1}};
}

template<class Visit>
void visitBox(KeyBox box, Visit visit) {
    for (int z = box.lo.z; z < box.hi.z; ++z) for (int y = box.lo.y; y < box.hi.y; ++y) {
        for (int x = box.lo.x; x < box.hi.x; ++x) visit(Key{x, y, z});
    }
}

template<class Visit>
void visitDifference(KeyBox box, KeyBox other, Visit visit) {
    KeyBox overlap;
    for (int axis = 0; axis < 3; ++axis) {
        overlap.lo[axis] = std::max(box.lo[axis], other.lo[axis]);
        overlap.hi[axis] = std::min(box.hi[axis], other.hi[axis]);
        if (overlap.lo[axis] >= overlap.hi[axis]) {
            visitBox(box, visit);
            return;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        KeyBox lower = box, upper = box;
        lower.hi[axis] = overlap.lo[axis];
        upper.lo[axis] = overlap.hi[axis];
        visitBox(lower, visit);
        visitBox(upper, visit);
        box.lo[axis] = overlap.lo[axis];
        box.hi[axis] = overlap.hi[axis];
    }
}

std::size_t fieldSize(const Chunk& chunk) {
    return (chunk.field ? chunk.field->bytes() : 0) + chunk.rows.capacity() * sizeof(CellRow);
}

constexpr std::size_t chunkMetadata = sizeof(Chunk) + sizeof(Chunks::value_type) + mapNodeOverhead;

}  // namespace

std::set<Key> Clipmap::update(const std::vector<Vec>& invokers, bool global, std::size_t& visits) {
    std::set<Key> changed;
    if (global) {
        std::vector<Key> next;
        for (std::size_t i = 0; i < invokers.size(); ++i) {
            Key key = keyAt(invokers[i]);
            if (global_ && i < anchors_.size()) {
                const Box retained = bounds(anchors_[i]).expanded(splat(anchorMargin));
                for (int a = 0; a < 3; ++a)
                    if (invokers[i][a] >= retained.lo[a] && invokers[i][a] < retained.hi[a]) key[a] = anchors_[i][a];
            }
            next.push_back(key);
        }
        visits = 0;
        if (global_ && next == anchors_) return changed;
        std::vector<Vec> centers;
        for (Key key : next) centers.push_back(bounds(key).center());
        auto desired = planetLayout(centers, &visits);
        for (const auto& entry : layout_) {
            const auto found = desired.find(entry.first);
            if (found == desired.end() || found->second != entry.second) changed.insert(entry.first);
        }
        for (const auto& entry : desired) {
            const auto found = layout_.find(entry.first);
            if (found == layout_.end() || found->second != entry.second) changed.insert(entry.first);
        }
        layout_ = std::move(desired);
        anchors_ = std::move(next);
        coverage_.clear();
        global_ = true;
        return changed;
    }
    if (global_) {
        for (const auto& entry : layout_) changed.insert(entry.first);
        anchors_.clear(); coverage_.clear(); layout_.clear(); global_ = false;
    }
    std::set<Key> touched;
    visits = 0;
    for (std::size_t i = 0; i < std::max(anchors_.size(), invokers.size()); ++i) {
        const bool had = i < anchors_.size(), has = i < invokers.size();
        const Key previous = had ? anchors_[i] : Key{};
        Key next = has ? keyAt(invokers[i]) : Key{};
        if (had && has) {
            const Box retained = bounds(previous).expanded(splat(anchorMargin));
            for (int axis = 0; axis < 3; ++axis) {
                if (invokers[i][axis] >= retained.lo[axis] && invokers[i][axis] < retained.hi[axis]) next[axis] = previous[axis];
            }
            if (next == previous) continue;
        }
        for (int lod = 0; lod < 4; ++lod) {
            auto add = [&](Key key) {
                ++visits;
                if (!intersectsPlanetShell(key)) return;
                ++coverage_[key][lod];
                touched.insert(key);
            };
            auto remove = [&](Key key) {
                ++visits;
                const auto found = coverage_.find(key);
                if (found == coverage_.end() || found->second[lod] == 0) return;
                --found->second[lod];
                touched.insert(key);
            };
            if (had && has) {
                visitDifference(window(previous, lod), window(next, lod), remove);
                visitDifference(window(next, lod), window(previous, lod), add);
            } else if (has) visitBox(window(next, lod), add);
            else visitBox(window(previous, lod), remove);
        }
        if (has && had) anchors_[i] = next;
        else if (has) anchors_.push_back(next);
    }
    anchors_.resize(invokers.size());
    for (const Key key : touched) {
        int lod = -1;
        const auto found = coverage_.find(key);
        if (found != coverage_.end()) {
            const auto& counts = found->second;
            auto level = std::find_if(counts.begin(), counts.end(), [](unsigned n) { return n != 0; });
            if (level != counts.end()) lod = int(level - counts.begin());
            else coverage_.erase(found);
        }
        auto prior = layout_.find(key);
        if (prior == layout_.end() ? lod < 0 : prior->second == lod) continue;
        changed.insert(key);
        if (lod < 0) layout_.erase(prior);
        else layout_.insert_or_assign(key, lod);
    }
    return changed;
}

std::vector<ClipmapWindow> Clipmap::windows() const {
    std::vector<ClipmapWindow> result;
    if (global_) {
        for (std::size_t i = 0; i < anchors_.size(); ++i) for (int lod = 0; lod < lodCount; ++lod) {
            const Vec center = bounds(anchors_[i]).center();
            const double radius = lod < 4 ? chunkSize * radii[lod] : 2 * std::ldexp(chunkSize, lod - 3);
            const double limit = std::ldexp(chunkSize, rootLevel);
            result.push_back({i, lod, {max(splat(-limit), center - splat(radius)), min(splat(limit), center + splat(radius))}});
        }
        return result;
    }
    result.reserve(anchors_.size() * 4);
    for (std::size_t i = 0; i < anchors_.size(); ++i) for (int lod = 0; lod < 4; ++lod) {
        const auto box = window(anchors_[i], lod);
        result.push_back({i, lod, {asVec(box.lo) * chunkSize, asVec(box.hi) * chunkSize}});
    }
    return result;
}

std::size_t Clipmap::bytes() const {
    return sizeof(*this) + anchors_.capacity() * sizeof(Key)
         + coverage_.size() * (sizeof(decltype(coverage_)::value_type) + mapNodeOverhead)
         + layout_.size() * (sizeof(Layout::value_type) + mapNodeOverhead);
}

World::World(const Planet& planet, Method method, bool morton)
    : physics(method, chunks), planet_(planet), method_(method), morton_(morton) {}

World::Prior World::snapshot() const {
    Prior prior;
    for (const auto& entry : chunks) prior.emplace(entry.first, std::make_pair(entry.second->lod, entry.second->neighborhood));
    return prior;
}

World::Batch World::planUpdate(Clipmap planned, const Prior& prior, const std::vector<Vec>& sources,
                              bool global, Clock::time_point start) {
    Batch batch{};
    batch.start = start;
    batch.clipmap = std::move(planned);
    const auto changed = batch.clipmap.update(sources, global, batch.stats.windowVisits);
    const auto& layout = batch.clipmap.layout();
    batch.stats.changedOwners = changed.size();
    auto dirty = changed;
    if (!prior.empty()) for (const Key key : changed) {
        if (global) {
            for (Key neighbor : intersectingLeaves(layout, bounds(key).expanded(splat(1e-4)))) dirty.insert(neighbor);
        } else {
            visitBox(window(key, 0), [&](Key neighbor) {
                if (layout.find(neighbor) != layout.end()) dirty.insert(neighbor);
            });
        }
    }
    for (const Key key : dirty) {
        const auto desired = layout.find(key);
        const auto previous = prior.find(key);
        if (desired != layout.end()) {
            if (previous != prior.end() && previous->second.first == desired->second &&
                previous->second.second == neighborhood(layout, key)) continue;
            batch.build.push_back(key);
        }
        if (previous != prior.end()) batch.remove.push_back(key);
    }
    batch.stats.selectMs = milliseconds(batch.start);
    return batch;
}

void World::build(Batch& batch, const Planet& planet, const Layout& layout, Method method, bool morton) {
    for (const Key key : batch.build) {
        auto chunk = buildChunk(planet, layout, key, method, morton, batch.stats.build);
        batch.stats.stagingBytes += chunkMetadata + fieldSize(*chunk) + chunk->mesh.capacity() * sizeof(Triangle);
        batch.staged.emplace(key, std::move(chunk));
    }
}

WorldStats World::publish(Batch batch) {
    const auto activation = Clock::now();
    for (const Key key : batch.remove) {
        physics.removeTerrain(key);
        chunks.erase(key);
        ++batch.stats.build.removed;
    }
    while (!batch.staged.empty()) {
        auto inserted = chunks.insert(batch.staged.extract(batch.staged.begin()));
        physics.addTerrain(*inserted.position->second);
    }
    batch.stats.physicsLoadMs = milliseconds(activation);
    clipmap_ = std::move(batch.clipmap);
    if (batch.stats.windowVisits != 0) publishedWindows_ = clipmap_.windows();
    batch.stats.updateMs = milliseconds(batch.start);
    return batch.stats;
}

WorldStats World::update(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera) {
    if (building_.valid()) throw std::logic_error("An asynchronous Clipmap update is pending");
    const auto start = Clock::now();
    auto sources = invokers;
    if (camera) sources.push_back(*camera);
    auto batch = planUpdate(clipmap_, snapshot(), sources, global, start);
    build(batch, planet_, batch.clipmap.layout(), method_, morton_);
    return publish(std::move(batch));
}

bool World::requestUpdate(const std::vector<Vec>& invokers, bool global, Vec camera) {
    if (building_.valid()) return false;
    const auto start = Clock::now();
    auto sources = invokers;
    sources.push_back(camera);
    // Copy only immutable scheduling metadata. The worker owns the next tree;
    // rendering, queries, memory statistics and physics retain the published tree.
    building_ = std::async(std::launch::async, [planet = &planet_, planned = clipmap_, prior = snapshot(),
            method = method_, morton = morton_, global, start, sources = std::move(sources)]() mutable {
        auto batch = planUpdate(std::move(planned), prior, sources, global, start);
        build(batch, *planet, batch.clipmap.layout(), method, morton);
        return batch;
    });
    return true;
}

std::optional<WorldStats> World::pollUpdate() {
    if (building_.valid() && building_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        return publish(building_.get());
    }
    return std::nullopt;
}

std::array<int, lodCount> World::lodCounts() const {
    std::array<int, lodCount> result{};
    for (const auto& entry : chunks) ++result[entry.second->lod];
    return result;
}

TerrainMemory World::terrainMemory() const {
    TerrainMemory result;
    result.physics = physics.terrainBytes();
    result.metadata = clipmap_.bytes() + chunks.size() * chunkMetadata + publishedWindows_.capacity() * sizeof(ClipmapWindow);
    for (const auto& entry : chunks) {
        result.fields += fieldSize(*entry.second);
        result.meshes += entry.second->mesh.capacity() * sizeof(Triangle);
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
