#include "world.hpp"

#include <stdexcept>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <utility>

namespace voxel {
namespace {

constexpr int radii[] = {1, 2, 3, 4};
double anchorMargin(int densityShift) { return 0.125 * gridChunkSize(densityShift); }
constexpr std::size_t mapNodeOverhead = 4 * sizeof(void*);

struct KeyBox { Key lo, hi; }; // Half-open in chunk coordinates.

double lodAnchorStep(double radius, int lod, int densityShift) {
    const double chunkStride = std::ldexp(gridChunkSize(densityShift), lod - 3);
    return std::min(std::max(128.0, radius / 2), chunkStride);
}

bool hasVolume(Box box) {
    return box.lo.x < box.hi.x && box.lo.y < box.hi.y && box.lo.z < box.hi.z;
}

void appendDifference(const ClipmapWindow& window, Box other, std::vector<ClipmapSlab>& dirty) {
    Box box = window.box;
    const Box intersection{max(box.lo, other.lo), min(box.hi, other.hi)};
    if (!hasVolume(intersection)) {
        dirty.push_back({box, window.lod});
        return;
    }
    for (int axis = 0; axis < 3; ++axis) {
        Box lower = box, upper = box;
        lower.hi[axis] = intersection.lo[axis];
        upper.lo[axis] = intersection.hi[axis];
        if (hasVolume(lower)) dirty.push_back({lower, window.lod});
        if (hasVolume(upper)) dirty.push_back({upper, window.lod});
        box.lo[axis] = intersection.lo[axis];
        box.hi[axis] = intersection.hi[axis];
    }
}

KeyBox window(Key anchor, int lod) {
    const int r = radii[lod];
    return {{anchor.x - r, anchor.y - r, anchor.z - r, 0, anchor.densityShift},
            {anchor.x + r + 1, anchor.y + r + 1, anchor.z + r + 1, 0, anchor.densityShift}};
}

template<class Visit>
void visitBox(KeyBox box, Visit visit) {
    for (int z = box.lo.z; z < box.hi.z; ++z) for (int y = box.lo.y; y < box.hi.y; ++y) {
        for (int x = box.lo.x; x < box.hi.x; ++x) visit(Key{x, y, z, 0, box.lo.densityShift});
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

std::set<Key> Clipmap::update(const std::vector<Vec>& invokers, bool global, std::size_t& visits,
                             std::uint32_t lod0Radius, bool rolling) {
    std::set<Key> changed;
    if (global) {
        const bool rebuild = !global_ || layoutDensityShift(layout_) != densityShift_;
        std::vector<GlobalAnchor> next;
        std::vector<ClipmapWindow> windows;
        std::vector<ClipmapSlab> dirty;
        constexpr int nearLevels = 3;
        const int windowCount = lodWindowCount(lod0Radius);
        if (!windowCount) throw std::invalid_argument("Invalid LOD0 radius");
        const bool sameRadius = lod0Radius_ == lod0Radius;
        const std::size_t noRoll = invokers.size() * windowCount;
        const bool phased = rolling && !rebuild && sameRadius && noRoll == globalWindows_.size();
        double nearOverdue = 0, outerOverdue = 0;
        std::size_t nearWindow = noRoll, outerWindow = noRoll;
        bool fineBuffer = true;
        for (std::size_t i = 0; i < invokers.size(); ++i) for (int lod = 0; lod < windowCount; ++lod) {
            const auto index = next.size();
            const double radius = std::ldexp(double(lod0Radius), lod);
            const double step = lodAnchorStep(radius, lod, densityShift_);
            const double margin = step / 8;
            double overdue = 0;
            int mostOverdueAxis = 0;
            Vec origin;
            for (int axis = 0; axis < 3; ++axis) {
                const double position = invokers[i][axis];
                origin[axis] = std::floor(position / step) * step;
                if (!sameRadius || index >= globalAnchors_.size() || globalAnchors_[index].step != step) continue;
                const double previous = globalAnchors_[index].origin[axis];
                if (position >= previous - margin && position < previous + step + margin) origin[axis] = previous;
                else {
                    const double distance = std::abs(position - (previous + step / 2));
                    if (distance > overdue) { overdue = distance; mostOverdueAxis = axis; }
                }
            }
            // During continuous movement, publish one entering and one leaving
            // slab. A jump beyond the nominal radius recenters its window.
            if (phased && overdue > 0 && overdue < radius) {
                const auto previous = globalAnchors_[index].origin;
                const double delta = origin[mostOverdueAxis] - previous[mostOverdueAxis];
                origin = previous;
                origin[mostOverdueAxis] += std::clamp(delta, -step, step);
            }
            next.push_back({origin, step});
            const auto box = Box{origin, origin + splat(step)}.expanded(splat(radius + margin));
            windows.push_back({i, lod, box});
            if (!phased) continue;
            const double urgency = overdue / radius;
            if (lod < nearLevels) {
                if (urgency > nearOverdue) { nearOverdue = urgency; nearWindow = index; }
            } else if (urgency > outerOverdue) { outerOverdue = urgency; outerWindow = index; }
            if (lod == 0) fineBuffer &= globalWindows_[index].box.expanded(splat(-double(lod0Radius) / 2)).contains(invokers[i]);
        }

        // Prioritize the near field. Give a waiting outer window a turn when
        // frequent near rolls still leave every invoker well inside its LOD0.
        const bool doNear = nearWindow != noRoll && (outerWindow == noRoll || nearRolls_ < 3 || !fineBuffer);
        const auto activeWindow = doNear ? nearWindow : outerWindow;
        nearRolls_ = phased && doNear && outerWindow != noRoll ? std::min(nearRolls_ + 1, 3) : 0;
        for (std::size_t i = 0; i < windows.size(); ++i) {
            if (phased && i != activeWindow) {
                next[i] = globalAnchors_[i];
                windows[i] = globalWindows_[i];
            } else if (!sameRadius || i >= globalWindows_.size()) {
                dirty.push_back({windows[i].box, windows[i].lod});
            } else {
                appendDifference(windows[i], globalWindows_[i].box, dirty);
                appendDifference(globalWindows_[i], windows[i].box, dirty);
            }
        }
        for (std::size_t i = sameRadius ? windows.size() : 0; i < globalWindows_.size(); ++i)
            dirty.push_back({globalWindows_[i].box, globalWindows_[i].lod});
        changed = updatePlanetLayout(idealLayout_, layout_, windows, dirty, densityShift_, rebuild, visits, elevationRange_);
        if (!dirty.empty() || rebuild) ++windowRevision_;
        globalAnchors_ = std::move(next);
        globalWindows_ = std::move(windows);
        lod0Radius_ = lod0Radius;
        anchors_.clear();
        coverage_.clear();
        global_ = true;
        return changed;
    }
    const bool wasGlobal = global_;
    if (global_ || (!layout_.empty() && layoutDensityShift(layout_) != densityShift_) ||
        (!anchors_.empty() && anchors_.front().densityShift != densityShift_)) {
        for (const auto& entry : layout_) changed.insert(entry.first);
        anchors_.clear(); coverage_.clear(); layout_.clear(); global_ = false;
        idealLayout_.clear();
        globalAnchors_.clear();
        globalWindows_.clear();
    }
    std::set<Key> touched;
    visits = 0;
    for (std::size_t i = 0; i < std::max(anchors_.size(), invokers.size()); ++i) {
        const bool had = i < anchors_.size(), has = i < invokers.size();
        const Key previous = had ? anchors_[i] : Key{};
        Key next = has ? keyAt(invokers[i], densityShift_) : Key{};
        if (had && has) {
            const Box retained = bounds(previous).expanded(splat(anchorMargin(densityShift_)));
            for (int axis = 0; axis < 3; ++axis) {
                if (invokers[i][axis] >= retained.lo[axis] && invokers[i][axis] < retained.hi[axis]) next[axis] = previous[axis];
            }
            if (next == previous) continue;
        }
        for (int lod = 0; lod < 4; ++lod) {
            auto add = [&](Key key) {
                ++visits;
                if (!intersectsPlanetShell(key, elevationRange_)) return;
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
    if (visits != 0 || wasGlobal) ++windowRevision_;
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
        const double limit = fieldSideLength / 2;
        for (const auto& w : globalWindows_) {
            const Box box{max(splat(-limit), w.box.lo), min(splat(limit), w.box.hi)};
            if (hasVolume(box)) result.push_back({w.invoker, w.lod, box});
        }
        return result;
    }
    result.reserve(anchors_.size() * 4);
    for (std::size_t i = 0; i < anchors_.size(); ++i) for (int lod = 0; lod < 4; ++lod) {
        const auto box = window(anchors_[i], lod);
        const double size = gridChunkSize(densityShift_);
        result.push_back({i, lod, {asVec(box.lo) * size, asVec(box.hi) * size}});
    }
    return result;
}

std::size_t Clipmap::bytes() const {
    return sizeof(*this) + anchors_.capacity() * sizeof(Key)
         + globalAnchors_.capacity() * sizeof(GlobalAnchor) + globalWindows_.capacity() * sizeof(ClipmapWindow)
         + coverage_.size() * (sizeof(decltype(coverage_)::value_type) + mapNodeOverhead)
         + (layout_.size() + idealLayout_.size()) * (sizeof(Layout::value_type) + mapNodeOverhead);
}

World::World(const Planet& planet, Method method, bool morton, double density, TerrainPreparation prepareTerrain)
    : physics(method, chunks), planet_(planet), method_(method), morton_(morton),
      densityShift_(voxelDensityShift(density)), publishedDensityShift_(densityShift_),
      clipmap_(densityShift_, planet.elevationRange()), prepareTerrain_(std::move(prepareTerrain)) {
    publishedClipmapBytes_ = clipmap_.bytes();
}

void World::setLod0Radius(std::uint32_t radius) {
    if (!validLod0Radius(radius)) throw std::invalid_argument("Invalid LOD0 radius");
    lod0Radius_ = radius;
}

World::Batch World::prepareUpdate(const std::vector<Vec>& sources, bool global, int densityShift,
                                  std::uint32_t lod0Radius, Clock::time_point start, bool rolling) {
    try {
        // Destruction of VDB trees and mesh BVHs can be expensive. Detach them
        // at publication and free them before the next background build.
        retiredColliders_.clear();
        retiredChunks_.clear();
        auto batch = planUpdate(sources, global, densityShift, lod0Radius, start, rolling);
        build(batch, planet_, clipmap_.layout(), method_, morton_);
        if (prepareTerrain_) prepareTerrain_(chunks, batch.staged, batch.remove);
        recoverPlan_ = false;
        return batch;
    } catch (...) {
        clipmap_ = Clipmap(densityShift, planet_.elevationRange());
        recoverPlan_ = true;
        throw;
    }
}

World::Batch World::planUpdate(const std::vector<Vec>& sources, bool global, int densityShift,
                              std::uint32_t lod0Radius, Clock::time_point start, bool rolling) {
    Batch batch{};
    batch.start = start;
    clipmap_.setDensityShift(densityShift);
    const auto windowRevision = clipmap_.windowRevision();
    const auto changed = clipmap_.update(sources, global, batch.stats.windowVisits, lod0Radius, rolling);
    batch.stats.windowsChanged = clipmap_.windowRevision() != windowRevision || recoverPlan_;
    const auto& layout = clipmap_.layout();
    const auto& published = chunks;
    batch.stats.changedOwners = changed.size();
    auto dirty = changed;
    if (recoverPlan_) for (const auto& entry : published) dirty.insert(entry.first);
    if (!published.empty()) for (const Key key : changed) {
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
        const auto previous = published.find(key);
        if (desired != layout.end()) {
            if (previous != published.end() && previous->second->lod == desired->second &&
                (!intersectsPlanetShell(key, planet_.elevationRange()) ||
                 previous->second->neighborhood == neighborhood(layout, key))) continue;
            batch.build.push_back(key);
        }
        if (previous != published.end()) batch.remove.push_back(key);
    }
    batch.stats.selectMs = milliseconds(batch.start);
    return batch;
}

void World::build(Batch& batch, const Planet& planet, const Layout& layout, Method method, bool morton) {
    if (batch.build.empty()) return;
    struct BuiltChunk { std::unique_ptr<Chunk> chunk; BuildStats stats; Physics::Terrain collider; };
    std::vector<BuiltChunk> built(batch.build.size());
    tbb::task_arena arena(8);
    arena.execute([&] {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, built.size(), 4), [&](const auto& range) {
            for (auto i = range.begin(); i != range.end(); ++i) {
                built[i].chunk = buildChunk(planet, layout, batch.build[i], method, morton, built[i].stats);
                built[i].collider = Physics::prepareTerrain(*built[i].chunk, method);
            }
        });
    });
    for (auto& item : built) {
        auto& chunk = item.chunk;
        batch.stats.build.sampleMs += item.stats.sampleMs;
        batch.stats.build.storageMs += item.stats.storageMs;
        batch.stats.build.meshMs += item.stats.meshMs;
        batch.stats.build.built += item.stats.built;
        chunk->fieldBytes = fieldSize(*chunk);
        batch.stats.stagingBytes += chunkMetadata + chunk->fieldBytes + chunk->mesh.capacity() * sizeof(Triangle)
                                 + Physics::preparedTerrainBytes(item.collider);
        if (item.collider) batch.colliders.emplace(chunk->key, std::move(item.collider));
        batch.staged.emplace(chunk->key, std::move(chunk));
    }
}

WorldStats World::publish(Batch batch) {
    const auto activation = Clock::now();
    for (const Key key : batch.remove) {
        const auto old = chunks.find(key);
        --lodCounts_[old->second->lod];
        fieldBytes_ -= old->second->fieldBytes;
        meshBytes_ -= old->second->mesh.capacity() * sizeof(Triangle);
        if (auto collider = physics.removeTerrain(key)) retiredColliders_.push_back(std::move(collider));
        retiredChunks_.insert(retiredChunks_.end(), chunks.extract(old));
        ++batch.stats.build.removed;
    }
    while (!batch.staged.empty()) {
        auto inserted = chunks.insert(batch.staged.extract(batch.staged.begin()));
        const auto& chunk = *inserted.position->second;
        ++lodCounts_[chunk.lod];
        fieldBytes_ += chunk.fieldBytes;
        meshBytes_ += chunk.mesh.capacity() * sizeof(Triangle);
        const auto collider = batch.colliders.find(chunk.key);
        if (collider != batch.colliders.end()) physics.addTerrain(chunk, std::move(collider->second));
    }
    batch.stats.physicsLoadMs = milliseconds(activation);
    publishedDensityShift_ = clipmap_.densityShift();
    publishedLod0Radius_ = clipmap_.lod0Radius();
    publishedClipmapBytes_ = clipmap_.bytes();
    if (batch.stats.windowsChanged) publishedWindows_ = clipmap_.windows();
    batch.stats.updateMs = milliseconds(batch.start);
    return batch.stats;
}

WorldStats World::update(const std::vector<Vec>& invokers, bool global, std::optional<Vec> camera) {
    if (building_.valid() || ready_) throw std::logic_error("An asynchronous Clipmap update is pending");
    const auto start = Clock::now();
    auto sources = invokers;
    if (camera) sources.push_back(*camera);
    return publish(prepareUpdate(sources, global, densityShift_, lod0Radius_, start, false));
}

bool World::requestUpdate(const std::vector<Vec>& invokers, bool global, Vec camera, bool rolling) {
    if (building_.valid() || ready_) return false;
    const auto start = Clock::now();
    auto sources = invokers;
    sources.push_back(camera);
    // Published chunks are read-only until this worker finishes. Only the worker
    // touches the planner; queries and physics continue using the published chunks.
    building_ = std::async(std::launch::async, [this, densityShift = densityShift_, lod0Radius = lod0Radius_,
            global, start, rolling, sources = std::move(sources)] {
        return prepareUpdate(sources, global, densityShift, lod0Radius, start, rolling);
    });
    return true;
}

std::optional<WorldStats> World::pollUpdate(const std::function<bool()>& uploadReady) {
    if (building_.valid() && building_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        ready_.emplace(building_.get());
    }
    if (ready_ && (!uploadReady || uploadReady())) {
        auto batch = std::move(*ready_);
        ready_.reset();
        return publish(std::move(batch));
    }
    return std::nullopt;
}

bool World::hasFinestLod(Vec point) const {
    const auto found = chunks.find(keyAt(point, publishedDensityShift_));
    return found != chunks.end() && found->second->lod == 0;
}

std::array<int, lodCount> World::lodCounts() const {
    return lodCounts_;
}

TerrainMemory World::terrainMemory() const {
    return {fieldBytes_, physics.terrainBytes(), meshBytes_, publishedClipmapBytes_ + chunks.size() * chunkMetadata +
            publishedWindows_.capacity() * sizeof(ClipmapWindow)};
}

std::size_t World::meshBytes() const {
    return meshBytes_;
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
