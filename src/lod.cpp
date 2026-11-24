#include "terrain.hpp"

namespace voxel {
namespace {
int floorTo(int x, int step) { return x - (x % step + step) % step; }
bool overlaps(Box a, Box b) {
    return a.lo.x <= b.hi.x && a.hi.x >= b.lo.x && a.lo.y <= b.hi.y && a.hi.y >= b.lo.y
        && a.lo.z <= b.hi.z && a.hi.z >= b.lo.z;
}
bool overlapsInterior(Box a, Box b) {
    return a.lo.x < b.hi.x && a.hi.x > b.lo.x && a.lo.y < b.hi.y && a.hi.y > b.lo.y
        && a.lo.z < b.hi.z && a.hi.z > b.lo.z;
}
template<class Visit> void children(Key key, Visit visit) {
    const int step = 1 << (key.level - 1);
    for (int i = 0; i < 8; ++i)
        visit(Key{key.x + (i & 1) * step, key.y + ((i >> 1) & 1) * step,
                  key.z + (i >> 2) * step, key.level - 1, key.densityShift});
}
template<class Visit> void roots(Visit visit, int densityShift) {
    const int level = gridRootLevel(densityShift), step = 1 << level;
    for (int i = 0; i < 8; ++i)
        visit(Key{(i & 1) ? 0 : -step, (i & 2) ? 0 : -step, (i & 4) ? 0 : -step, level, densityShift});
}
}

std::optional<Key> leafAt(const Layout& layout, Vec point) {
    const int densityShift = layoutDensityShift(layout);
    const Key base = keyAt(point, densityShift);
    const int limit = layout.empty() ? 0 : layout.rbegin()->first.level;
    for (int level = 0; level <= limit; ++level) {
        const int step = 1 << level;
        const Key key{floorTo(base.x, step), floorTo(base.y, step), floorTo(base.z, step), level, densityShift};
        if (layout.find(key) != layout.end()) return key;
    }
    return {};
}

std::vector<Key> intersectingLeaves(const Layout& layout, Box box) {
    std::vector<Key> result;
    if (layout.empty()) return result;
    const int shift = layoutDensityShift(layout);
    const double size = gridChunkSize(shift);
    const Vec lo{box.lo.x / size, box.lo.y / size, box.lo.z / size};
    const Vec hi{box.hi.x / size, box.hi.y / size, box.hi.z / size};
    const auto intersects = [&](Key key) {
        const int span = 1 << key.level;
        return key.x <= hi.x && key.x + span >= lo.x && key.y <= hi.y && key.y + span >= lo.y &&
               key.z <= hi.z && key.z + span >= lo.z;
    };
    if (layout.rbegin()->first.level == 0) {
        for (const auto& entry : layout) if (intersects(entry.first)) result.push_back(entry.first);
        return result;
    }
    auto visit = [&](const auto& self, Key key) -> void {
        if (!intersects(key)) return;
        const auto found = layout.find(key);
        if (found != layout.end()) { result.push_back(key); return; }
        if (key.level) children(key, [&](Key child) { self(self, child); });
    };
    roots([&](Key key) { visit(visit, key); }, shift);
    return result;
}

std::set<Key> updatePlanetLayout(Layout& ideal, Layout& balanced, const std::vector<ClipmapWindow>& windows,
                                 const std::vector<ClipmapSlab>& dirty, int densityShift, bool rebuild, std::size_t& visits) {
    visits = 0;
    if (!rebuild && dirty.empty()) return {};
    std::array<std::vector<Box>, lodCount - 1> levels;
    for (const auto& window : windows) if (window.lod < int(levels.size())) levels[window.lod].push_back(window.box);
    const auto requestedLod = [&](Box box, bool surface) {
        // Subdividing a uniformly solid or empty node adds no SDF information.
        if (!surface) return lodCount - 1;
        for (int lod = 0; lod < int(levels.size()); ++lod) {
            for (Box window : levels[lod]) if (overlaps(box, window)) return lod;
        }
        return lodCount - 1;
    };

    std::set<Key> idealChanges;
    if (rebuild) ideal.clear();
    const auto eraseSubtree = [&](const auto& self, Key key) -> void {
        if (ideal.erase(key)) idealChanges.insert(key);
        else if (key.level) children(key, [&](Key child) { self(self, child); });
    };
    const auto select = [&](const auto& self, Key key, bool force) -> void {
        const auto box = bounds(key);
        // A change at this node's own resolution or coarser cannot split it
        // or undo an existing split: either decision depends on finer levels.
        if (!force && std::none_of(dirty.begin(), dirty.end(), [&](const ClipmapSlab& slab) {
                return slab.lod < key.level + 3 && overlaps(box, slab.box);
            })) return;
        ++visits;
        const bool surface = intersectsPlanetShell(key);
        const int requested = requestedLod(box, surface);
        const auto previous = ideal.find(key);
        if (key.level && (key.level + 3 > requested ||
                         (key.level > coarsestSurfaceLevel - densityShift && surface))) {
            const bool wasLeaf = previous != ideal.end();
            if (wasLeaf) {
                ideal.erase(previous);
                idealChanges.insert(key);
            }
            children(key, [&](Key child) { self(self, child, force || wasLeaf); });
            return;
        }
        const int lod = key.level ? key.level + 3 : std::min(requested, 3);
        if (previous != ideal.end() && previous->second == lod) return;
        if (previous == ideal.end() && key.level && !force) {
            children(key, [&](Key child) { eraseSubtree(eraseSubtree, child); });
        }
        ideal.insert_or_assign(key, lod);
        idealChanges.insert(key);
    };
    roots([&](Key key) { select(select, key, rebuild); }, densityShift);
    if (!rebuild && idealChanges.empty()) return {};

    // Save only the published leaves touched by the patch or its balance closure.
    std::map<Key, std::optional<int>> previous;
    const auto remember = [&](Key key) {
        const auto it = balanced.find(key);
        previous.try_emplace(key, it == balanced.end() ? std::nullopt : std::optional<int>(it->second));
    };
    std::set<Key> queue, relax;
    const auto enqueueNeighbors = [&](Key key) {
        const auto neighbors = intersectingLeaves(balanced, bounds(key));
        queue.insert(neighbors.begin(), neighbors.end());
    };
    if (rebuild) {
        for (const auto& leaf : balanced) previous.emplace(leaf.first, leaf.second);
        balanced = ideal;
        for (const auto& leaf : ideal) {
            previous.try_emplace(leaf.first, std::nullopt);
            queue.insert(leaf.first);
        }
    } else {
        std::set<Key> reset;
        for (Key key : idealChanges) {
            const auto box = bounds(key);
            for (Key leaf : intersectingLeaves(ideal, box))
                if (overlapsInterior(bounds(leaf), box)) reset.insert(leaf);
        }
        std::set<Key> removed;
        for (Key key : reset) {
            const auto box = bounds(key);
            for (Key actual : intersectingLeaves(balanced, box)) {
                const auto actualBox = bounds(actual);
                if (!overlapsInterior(actualBox, box) || !removed.insert(actual).second) continue;
                const auto neighbors = intersectingLeaves(balanced, actualBox);
                relax.insert(neighbors.begin(), neighbors.end());
            }
        }
        for (Key key : removed) { remember(key); balanced.erase(key); }
        for (Key key : reset) { remember(key); balanced.insert_or_assign(key, ideal.at(key)); }
        for (Key key : reset) enqueueNeighbors(key);
    }

    while (!queue.empty()) {
        const Key key = *queue.begin();
        queue.erase(queue.begin());
        const auto found = balanced.find(key);
        if (found == balanced.end()) continue;
        int target = found->second;
        for (Key neighbor : intersectingLeaves(balanced, bounds(key)))
            target = std::min(target, balanced.at(neighbor) + 1);
        if (target == found->second) continue;
        remember(key);
        if (key.level) {
            balanced.erase(found);
            children(key, [&](Key child) {
                remember(child);
                balanced.emplace(child, child.level ? child.level + 3 :
                    std::min(requestedLod(bounds(child), intersectsPlanetShell(child)), 3));
                ++visits;
            });
            children(key, enqueueNeighbors);
        } else {
            found->second = target;
            enqueueNeighbors(key);
        }
    }

    // A single ideal air or solid node can contain transition leaves from
    // several invokers. Coarsen actual siblings locally, leaving unrelated
    // portions of that same ideal node intact.
    while (!relax.empty()) {
        const Key key = *relax.begin();
        relax.erase(relax.begin());
        const auto found = balanced.find(key);
        if (found == balanced.end()) continue;
        const Box box = bounds(key);
        const auto owner = leafAt(ideal, box.center());
        if (!owner || found->second >= ideal.at(*owner)) continue;
        if (!key.level) {
            int target = std::min(ideal.at(*owner), 3);
            const auto neighbors = intersectingLeaves(balanced, box);
            for (Key neighbor : neighbors) if (!(neighbor == key)) target = std::min(target, balanced.at(neighbor) + 1);
            if (target > found->second) {
                remember(key);
                found->second = target;
                relax.insert(neighbors.begin(), neighbors.end());
            }
        }
        if (key.level >= owner->level) continue;
        const int parentSpan = 1 << (key.level + 1);
        const Key parent{floorTo(key.x, parentSpan), floorTo(key.y, parentSpan), floorTo(key.z, parentSpan),
                         key.level + 1, key.densityShift};
        bool allChildren = true;
        children(parent, [&](Key child) { allChildren &= balanced.find(child) != balanced.end(); });
        if (!allChildren) continue;
        const Box parentBox = bounds(parent);
        const auto neighbors = intersectingLeaves(balanced, parentBox);
        const int parentLod = parent.level + 3;
        if (std::any_of(neighbors.begin(), neighbors.end(), [&](Key neighbor) {
                return !overlapsInterior(bounds(neighbor), parentBox) && balanced.at(neighbor) < parentLod - 1;
            })) continue;
        children(parent, [&](Key child) { remember(child); balanced.erase(child); });
        remember(parent);
        balanced.emplace(parent, parentLod);
        relax.insert(neighbors.begin(), neighbors.end());
        relax.insert(parent);
    }

    std::set<Key> changed;
    for (const auto& item : previous) {
        const auto next = balanced.find(item.first);
        if (next == balanced.end() ? item.second.has_value() : item.second != next->second) changed.insert(item.first);
    }
    return changed;
}

Layout planetLayout(const std::vector<Vec>& invokers, std::size_t* visits, int densityShift, std::uint32_t lod0Radius) {
    std::vector<ClipmapWindow> windows;
    const int windowCount = lodWindowCount(lod0Radius);
    for (std::size_t i = 0; i < invokers.size(); ++i) for (int lod = 0; lod < windowCount; ++lod) {
        const auto extent = splat(std::ldexp(double(lod0Radius), lod));
        windows.push_back({i, lod, {invokers[i] - extent, invokers[i] + extent}});
    }
    Layout ideal, layout;
    std::size_t count = 0;
    updatePlanetLayout(ideal, layout, windows, {}, densityShift, true, count);
    if (visits) *visits = count;
    return layout;
}
} // namespace voxel
