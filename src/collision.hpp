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

struct BodyPose {
    Vec position;
    std::array<Vec, 3> axes;
};

// Full rigid body state, so a benchmark can restore both backends to exactly
// the same pose and velocities at the start of every frame.
struct BodyState {
    BodyPose pose;
    Vec velocity, angularVelocity;
};

using Chunks = std::map<Key, std::unique_ptr<Chunk>>;
Hit fieldRay(const Chunks& chunks, Vec from, Vec to);
Hit fieldSweep(const Chunks& chunks, const Shape& shape, Vec from, Vec to);

// Why a field sweep gave up: the recursion budget ran out, or the remaining
// interval sank below the tolerance that the search promises to resolve. Both
// cases collapse into the same Unresolved answer, so they are counted apart here.
// The counters are process-wide and single threaded.
struct SweepDiagnostics {
    std::size_t budgetStops{};
    std::size_t toleranceStops{};
};
void setSweepBudget(int budget);
int sweepBudget();
// Distance along the cast that the field sweep promises to resolve. A tighter
// value subdivides deeper before it refuses, so it trades time for fewer
// Unresolved answers.
void setSweepTolerance(double meters);
double sweepToleranceMeters();
// Per-cast trace of one field sweep: how deep the interval recursion went, how
// many cells it visited, and where it gave up. Used by --cast-at to explain a
// single Unresolved answer.
struct SweepTrace {
    int visits{};
    int depth{};
    int budgetStops{};
    int toleranceStops{};
    // The earliest give-up is what becomes Hit::fraction on an Unresolved answer,
    // so both the earliest and the latest stop are kept.
    double earliestFirst{}, earliestLast{};
    int earliestLevel{};
    double latestFirst{}, latestLast{};
    int latestLevel{};
    double stopTravel{}, stopDimension{};
    // The contact the search did prove, kept even when the answer is downgraded to
    // Unresolved, so a refusal can be told apart from a real disagreement.
    double provenFraction{-1};
    // Proved contact minus the end of the earliest unresolved interval, in meters.
    // Unresolved is returned only when this exceeds 2 * tolerance.
    double unknownMargin{-1};
};
void beginSweepTrace();
SweepTrace endSweepTrace();

void resetSweepDiagnostics();
SweepDiagnostics sweepDiagnostics();

class Physics {
public:
    explicit Physics(Method method, const Chunks& chunks);
    ~Physics();
    void addTerrain(const Chunk& chunk);
    void removeTerrain(Key key);
    std::size_t addBody(Shape shape, Vec position, Vec velocity = {});
    // Same as addBody, but the body only pairs with the terrain. Used when many
    // independently spawned bodies would otherwise interpenetrate each other.
    std::size_t addTerrainOnlyBody(Shape shape, Vec position, Vec velocity = {});
    Vec bodyPosition(std::size_t index) const;
    BodyPose bodyPose(std::size_t index) const;
    BodyState bodyState(std::size_t index) const;
    void setBodyState(std::size_t index, const BodyState& state);
    double bodySpeed(std::size_t index) const;
    int terrainContacts() const;
    int bodyContacts() const;
    void step(double dt, bool radialGravity, Vec gravity = {0, 0, -9.81});
    Hit ray(Vec from, Vec to) const;
    Hit sweep(const Shape& shape, Vec from, Vec to) const;
    std::size_t terrainBytes() const;
private:
    struct Impl;
    std::size_t addBodyImpl(Shape shape, Vec position, Vec velocity, short mask);
    std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
