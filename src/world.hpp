#pragma once

#include "collision.hpp"

namespace voxel {

struct WorldStats {
    BuildStats build;
    double selectMs{}, updateMs{}, physicsLoadMs{};
};

class World {
public:
    World(const Planet& planet, Method method, bool morton);
    WorldStats update(const std::vector<Vec>& invokers, bool global);
    std::array<int, 4> lodCounts() const;
    std::size_t fieldBytes() const;
    std::size_t meshBytes() const;

    Chunks chunks;
    Physics physics;
private:
    const Planet& planet_;
    Method method_;
    bool morton_;
};

std::vector<Vec> invokerPositions(const Planet& planet, int count, double frame);
std::array<Vec, 3> frameAt(Vec up);
Shape sampleShape(int kind, Vec up);

}  // namespace voxel
