#include "terrain.hpp"

#include <functional>
#include <limits>

namespace voxel {
namespace {
int floorTo(int x, int step) { return x - (x % step + step) % step; }
bool overlaps(Box a, Box b) {
    return a.lo.x <= b.hi.x && a.hi.x >= b.lo.x && a.lo.y <= b.hi.y && a.hi.y >= b.lo.y
        && a.lo.z <= b.hi.z && a.hi.z >= b.lo.z;
}
double distanceTo(Box box, const std::vector<Vec>& invokers) {
    double distance = std::numeric_limits<double>::infinity();
    for (Vec p : invokers) distance = std::min(distance, length(p - box.clamp(p)));
    return distance;
}
int leafLod(Key key, const std::vector<Vec>& invokers) {
    if (key.level) return key.level + 3;
    const double distance = distanceTo(bounds(key), invokers) / gridChunkSize(key.densityShift);
    return distance < 0.45 ? 0 : distance < 1.2 ? 1 : distance < 2 ? 2 : 3;
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
    if (layout.rbegin()->first.level == 0) {
        for (const auto& entry : layout) if (overlaps(bounds(entry.first), box)) result.push_back(entry.first);
        return result;
    }
    std::function<void(Key)> visit = [&](Key key) {
        if (!overlaps(bounds(key), box)) return;
        const auto found = layout.find(key);
        if (found != layout.end()) { result.push_back(key); return; }
        if (key.level) children(key, visit);
    };
    roots(visit, layoutDensityShift(layout));
    return result;
}

Layout planetLayout(const std::vector<Vec>& invokers, std::size_t* visits, int densityShift) {
    Layout layout;
    std::size_t count = 0;
    std::function<void(Key)> select = [&](Key key) {
        ++count;
        const bool surface = intersectsPlanetShell(key);
        if (key.level && ((surface && key.level > coarsestSurfaceLevel - densityShift) ||
                         distanceTo(bounds(key), invokers) < 2 * std::ldexp(gridChunkSize(densityShift), key.level))) {
            children(key, select);
        } else layout.emplace(key, leafLod(key, invokers));
    };
    roots(select, densityShift);

    // Balance sample spacings across faces, edges and corners. This also refines
    // constant solid/air nodes touching a fine surface, without dense allocation.
    for (;;) {
        std::map<Key, int> refine;
        for (const auto& entry : layout) {
            const auto neighbors = intersectingLeaves(layout, bounds(entry.first).expanded(splat(1e-4)));
            for (Key neighbor : neighbors) {
                const int target = layout.at(neighbor) + 1;
                if (entry.second > target) {
                    auto found = refine.find(entry.first);
                    if (found == refine.end()) refine.emplace(entry.first, target);
                    else found->second = std::min(found->second, target);
                }
            }
        }
        if (refine.empty()) break;
        for (const auto& entry : refine) {
            if (entry.first.level) {
                layout.erase(entry.first);
                children(entry.first, [&](Key child) { layout.emplace(child, leafLod(child, invokers)); ++count; });
            } else layout.at(entry.first) = entry.second;
        }
    }
    if (visits) *visits = count;
    return layout;
}
} // namespace voxel
