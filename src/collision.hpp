#pragma once

#include "terrain.hpp"

namespace voxel {

enum class ShapeKind { Sphere, Capsule, Box };

struct Shape {
    ShapeKind kind;
    // Sphere: radius. Capsule: radius, unused, half length of its cylindrical segment.
    // Box: half extents. The capsule's axis is the third basis vector.
    Vec size;
    std::array<Vec, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Vec direction(Vec local) const;
    Vec extents() const;
    double distance(Vec offset) const;
};

struct Hit {
    enum class State { Miss, Contact, Unresolved };
    State state{State::Miss};
    double fraction{1};
    Vec position{}, normal{};
    bool initial{};
};

using Chunks = std::map<Key, std::unique_ptr<Chunk>>;
Hit fieldRay(const Chunks& chunks, Vec from, Vec to);
Hit fieldSweep(const Chunks& chunks, const Shape& shape, Vec from, Vec to);

class Physics {
public:
    explicit Physics(Method method, const Chunks& chunks);
    ~Physics();
    void addTerrain(const Chunk& chunk);
    void removeTerrain(Key key);
    std::size_t addBody(Shape shape, Vec position, Vec velocity = {});
    Vec bodyPosition(std::size_t index) const;
    double bodySpeed(std::size_t index) const;
    int terrainContacts() const;
    void step(double dt, bool radialGravity, Vec gravity = {0, 0, -9.81});
    Hit ray(Vec from, Vec to) const;
    Hit sweep(const Shape& shape, Vec from, Vec to) const;
    std::size_t terrainBytes() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
