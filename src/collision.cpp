#include "collision.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include <limits>
#include <utility>

namespace voxel {

Vec Shape::direction(Vec local) const {
    return axes[0] * local.x + axes[1] * local.y + axes[2] * local.z;
}

Vec Shape::extents() const {
    if (kind == ShapeKind::Sphere) return splat(size.x);
    if (kind == ShapeKind::Capsule) return splat(size.x) + abs(axes[2]) * size.z;
    return abs(axes[0]) * size.x + abs(axes[1]) * size.y + abs(axes[2]) * size.z;
}

double Shape::distance(Vec offset) const {
    if (kind == ShapeKind::Sphere) return length(offset) - size.x;
    if (kind == ShapeKind::Capsule) {
        const double along = std::clamp(dot(offset, axes[2]), -size.z, size.z);
        return length(offset - axes[2] * along) - size.x;
    }
    const Vec local{dot(axes[0], offset), dot(axes[1], offset), dot(axes[2], offset)};
    const Vec d = abs(local) - size;
    return length(max(d, {})) + std::min(0.0, std::max({d.x, d.y, d.z}));
}

namespace {

constexpr double epsilon = 1e-6;
constexpr double sweepTolerance = 0.75;
constexpr int sweepBudget = 12000;

template<class Operation>
void forChunks(const Chunks& chunks, Box area, Operation operation) {
    const Key lo = keyAt(area.lo - splat(epsilon));
    const Key hi = keyAt(area.hi + splat(epsilon));
    for (int z = lo.z; z <= hi.z; ++z) for (int y = lo.y; y <= hi.y; ++y) {
        for (int x = lo.x; x <= hi.x; ++x) {
            auto it = chunks.find({x, y, z});
            if (it != chunks.end()) operation(*it->second);
        }
    }
}

template<class Operation>
void forCells(const Chunk& chunk, Box area, Operation operation) {
    const Key lo = chunk.cellIndex(area.lo - splat(epsilon));
    const Key hi = chunk.cellIndex(area.hi + splat(epsilon));
    for (int z = lo.z; z <= hi.z; ++z) for (int y = lo.y; y <= hi.y; ++y) {
        for (int x = lo.x; x <= hi.x; ++x) operation(chunk.cell({x, y, z}));
    }
}

Vec normalAt(const Cell& cell, Vec point) { return unit(cell.gradient(point), unit(point)); }

double firstRoot(const Cell& cell, Vec from, Vec to) {
    const Vec delta = to - from;
    const double start = cell.value(from);
    if (std::abs(start) <= epsilon) return 0;
    if ((start > 0 && cell.minimum() > epsilon) || (start < 0 && cell.maximum() < -epsilon)) return -1;
    const double sign = start < 0 ? -1 : 1;
    const double f0 = sign * start;
    // A trilinear field restricted to a line is a cubic. Split it at its stationary points.
    const double q = 27 * (sign * cell.value(from + delta / 3) - f0);
    const double r = 27 * (sign * cell.value(from + delta * (2.0 / 3)) - f0);
    const double ab = (r - 2 * q) / 6;
    const double c = sign * cell.value(to) - f0 - ab;
    const double a = (3 * ab + 9 * c - q) / 2;
    const double b = ab - a;
    const auto value = [&](double t) { return ((a * t + b) * t + c) * t + f0; };
    std::array<double, 3> splits{1, 1, 1};
    auto addSplit = [&](double t) {
        if (t > 0 && t < 1) {
            if (splits[0] == 1) splits[0] = t;
            else splits[1] = t;
        }
    };
    if (std::abs(a) > 1e-11) {
        const double discriminant = 4 * b * b - 12 * a * c;
        if (discriminant >= 0) {
            const double root = std::sqrt(discriminant);
            addSplit((-2 * b - root) / (6 * a));
            addSplit((-2 * b + root) / (6 * a));
        }
    } else if (std::abs(b) > 1e-11) {
        addSplit(-c / (2 * b));
    }
    std::sort(splits.begin(), splits.end());
    double lo = 0;
    for (double hi : splits) {
        if (value(hi) > epsilon) {
            lo = hi;
            continue;
        }
        if (value(hi) >= 0) return hi;
        for (int i = 0; i < 28; ++i) {
            const double mid = (lo + hi) * 0.5;
            if (value(mid) > 0) lo = mid;
            else hi = mid;
        }
        return hi;
    }
    return -1;
}

void rayChunk(const Chunk& chunk, Vec from, Vec delta, Hit& hit) {
    double first = 0, last = hit.fraction;
    if (!clip(chunk.box, from, delta, first, last)) return;
    const Vec p = from + delta * first;
    Key index = chunk.cellIndex(p);
    for (int a = 0; a < 3; ++a) {
        const double coord = (p[a] - chunk.box.lo[a]) / chunk.spacing;
        if (delta[a] < 0 && std::abs(coord - index[a]) < 1e-9 && index[a] > 0) --index[a];
    }
    while (first <= last) {
        const Cell cell = chunk.cell(index);
        Vec exits = splat(std::numeric_limits<double>::infinity());
        for (int a = 0; a < 3; ++a) {
            if (std::abs(delta[a]) < 1e-13) continue;
            const double edge = chunk.box.lo[a] + (index[a] + (delta[a] > 0)) * chunk.spacing;
            exits[a] = (edge - from[a]) / delta[a];
        }
        const double end = std::max(first, std::min({last, exits.x, exits.y, exits.z}));
        const Vec cellEntry = from + delta * first;
        const double fraction = first == 0 && cell.value(cellEntry) < 0
            ? 0 : firstRoot(cell, cellEntry, from + delta * end);
        if (fraction >= 0) {
            const double t = first + (end - first) * fraction;
            const Vec point = from + delta * t;
            hit = {Hit::State::Contact, t, point, normalAt(cell, point), t == 0};
            return;
        }
        if (end >= last) return;
        bool advanced = false;
        for (int a = 0; a < 3; ++a) {
            if (exits[a] > end + 1e-12) continue;
            index[a] += delta[a] > 0 ? 1 : -1;
            if (index[a] < 0 || index[a] >= chunk.cells) return;
            advanced = true;
        }
        if (!advanced) return;
        first = end;
    }
}

Vec corner(Box box, int i) {
    return {(i & 1) ? box.hi.x : box.lo.x, (i & 2) ? box.hi.y : box.lo.y, (i & 4) ? box.hi.z : box.lo.z};
}

Vec projectToShape(const Shape& shape, Vec offset) {
    if (shape.kind == ShapeKind::Box) {
        const Vec local{dot(offset, shape.axes[0]), dot(offset, shape.axes[1]), dot(offset, shape.axes[2])};
        return shape.direction(max(shape.size * -1, min(shape.size, local)));
    }
    const Vec axis = shape.kind == ShapeKind::Capsule
        ? shape.axes[2] * std::clamp(dot(offset, shape.axes[2]), -shape.size.z, shape.size.z) : Vec{};
    const Vec radial = offset - axis;
    return length(radial) <= shape.size.x ? offset : axis + unit(radial) * shape.size.x;
}

Vec support(const Shape& shape, Vec direction) {
    auto sign = [](double value) { return (value > 1e-12) - (value < -1e-12); };
    if (shape.kind == ShapeKind::Box) {
        return shape.direction({sign(dot(direction, shape.axes[0])) * shape.size.x,
                                sign(dot(direction, shape.axes[1])) * shape.size.y,
                                sign(dot(direction, shape.axes[2])) * shape.size.z});
    }
    const Vec axis = shape.kind == ShapeKind::Capsule
        ? shape.axes[2] * (sign(dot(direction, shape.axes[2])) * shape.size.z) : Vec{};
    return axis + unit(direction, {}) * shape.size.x;
}

struct SweepSearch {
    const Shape& shape;
    Vec from, delta, extents;
    double speed;
    Hit hit;
    double unknown{std::numeric_limits<double>::infinity()};
    int budget{sweepBudget};

    double cutoff() const {
        return hit.state == Hit::State::Contact && speed > 0
            ? std::max(0.0, hit.fraction - sweepTolerance / speed) : hit.fraction;
    }

    void sample(const Cell& cell, Box box, Vec point, double time) {
        const Vec center = from + delta * time;
        point = center + projectToShape(shape, point - center);
        if (time > hit.fraction || !box.expanded(splat(epsilon)).contains(point) || cell.value(point) > 2 * epsilon) return;
        const Vec n = normalAt(cell, point);
        hit = {Hit::State::Contact, time, point, n, time == 0};
    }

    void visit(const Cell& cell, Box box, double first, double last, bool seed = false) {
        last = std::min(last, cutoff());
        if (first > last || hit.initial || !clip(box.expanded(extents), from, delta, first, last)) return;
        const Vec begin = from + delta * first, end = from + delta * last;
        box.lo = max(box.lo, min(begin, end) - extents - splat(epsilon));
        box.hi = min(box.hi, max(begin, end) + extents + splat(epsilon));
        const Vec center = box.center();
        const Vec dimensions = box.hi - box.lo;
        const double initialMidpoint = (first + last) * 0.5;
        const double initialTravel = speed * (last - first);
        if (shape.distance(center - from - delta * initialMidpoint) > (length(dimensions) + initialTravel) * 0.5 + epsilon) return;
        const Vec gradient = cell.gradient(center);
        const double centerValue = cell.value(center);
        Vec deepest = box.lo;
        double smallest = std::numeric_limits<double>::infinity();
        double residual = smallest;
        for (int i = 0; i < 8; ++i) {
            const Vec point = corner(box, i);
            const double value = cell.value(point);
            residual = std::min(residual, value - centerValue - dot(gradient, point - center));
            if (value < smallest) {
                deepest = point;
                smallest = value;
            }
        }
        if (smallest > epsilon) return;
        // The residual from the tangent plane is trilinear and bounded at box corners.
        // The shape's support point bounds the tangent plane over every pose on the path.
        const Vec extreme = support(shape, gradient * -1);
        const double minAtZero = centerValue + dot(gradient, from + extreme - center) + residual;
        const double slope = dot(gradient, delta);
        if (std::abs(slope) > 1e-12) {
            const double limit = (epsilon - minAtZero) / slope;
            if (slope < 0) first = std::max(first, limit);
            else last = std::min(last, limit);
        } else if (minAtZero > epsilon) return;
        if (first > last) return;
        if (budget-- <= 0) {
            unknown = std::min(unknown, first);
            return;
        }
        sample(cell, box, from + delta * first + extreme, first);
        sample(cell, box, deepest, first);
        sample(cell, box, center, first);
        if (hit.initial || hit.fraction <= first) return;
        if (seed) {
            // Cast a support point to get a confirmed upper bound early. The subdivision
            // still searches for contacts elsewhere on the shape that may occur earlier.
            double enter = first, exit = last;
            const Vec point = from + extreme;
            if (clip(box, point, delta, enter, exit)) {
                const Vec a = point + delta * enter;
                const double root = cell.value(a) <= 0 ? 0 : firstRoot(cell, a, point + delta * exit);
                if (root >= 0) {
                    const double time = enter + (exit - enter) * root;
                    sample(cell, box, point + delta * time, time);
                }
            }
        }
        last = std::min(last, cutoff());
        if (first > last || hit.initial) return;
        const double midpoint = (first + last) * 0.5;
        const double travel = speed * (last - first);
        sample(cell, box, from + delta * midpoint + extreme, midpoint);
        sample(cell, box, deepest, midpoint);
        const int axis = dimensions.x >= dimensions.y && dimensions.x >= dimensions.z ? 0
                       : dimensions.y >= dimensions.z ? 1 : 2;
        if (travel <= sweepTolerance && dimensions[axis] <= sweepTolerance) {
            sample(cell, box, deepest, last);
            sample(cell, box, center, midpoint);
            unknown = std::min(unknown, first);
            return;
        }
        if (travel > sweepTolerance && travel >= dimensions[axis]) {
            visit(cell, box, first, midpoint);
            visit(cell, box, midpoint, last);
        } else {
            Box low = box, high = box;
            low.hi[axis] = high.lo[axis] = center[axis];
            if (delta[axis] < 0) std::swap(low, high);
            visit(cell, low, first, last);
            visit(cell, high, first, last);
        }
    }
};

}  // namespace

Hit fieldRay(const Chunks& chunks, Vec from, Vec to) {
    Hit hit;
    forChunks(chunks, {min(from, to), max(from, to)}, [&](const Chunk& chunk) {
        if (!hit.initial) rayChunk(chunk, from, to - from, hit);
    });
    return hit;
}

Hit fieldSweep(const Chunks& chunks, const Shape& shape, Vec from, Vec to) {
    SweepSearch search{shape, from, to - from, shape.extents(), length(to - from), {}};
    struct Candidate { Cell cell; Box box; double first, last; };
    std::vector<Candidate> candidates;
    const Box sweep = Box{min(from, to), max(from, to)}.expanded(search.extents);
    forChunks(chunks, sweep, [&](const Chunk& chunk) {
        double first = 0, last = search.hit.fraction;
        if (!clip(chunk.box.expanded(search.extents), from, search.delta, first, last)) return;
        const Vec start = from + search.delta * first, end = from + search.delta * last;
        forCells(chunk, Box{min(start, end), max(start, end)}.expanded(search.extents), [&](const Cell& cell) {
            if (cell.minimum() > epsilon) return;
            const Box box{cell.origin, cell.origin + splat(cell.spacing)};
            double enter = first, exit = last;
            if (clip(box.expanded(search.extents), from, search.delta, enter, exit)) {
                candidates.push_back({cell, box, enter, exit});
            }
        });
    });
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.first < b.first;
    });
    for (const auto& candidate : candidates) {
        if (search.hit.initial || candidate.first > search.cutoff()) break;
        search.visit(candidate.cell, candidate.box, candidate.first, candidate.last, true);
    }
    if (!search.hit.initial && std::isfinite(search.unknown)) {
        if (search.hit.state == Hit::State::Miss || (search.hit.fraction - search.unknown) * search.speed > 2 * sweepTolerance) {
            return {Hit::State::Unresolved, search.unknown};
        }
    }
    return search.hit;
}

namespace {

btVector3 bullet(Vec v) { return {btScalar(v.x), btScalar(v.y), btScalar(v.z)}; }
Vec voxelVec(const btVector3& v) { return {v.x(), v.y(), v.z()}; }

constexpr short terrainGroup = 1;
constexpr short bodyGroup = 2;
constexpr double contactMargin = 0.04;

btTransform pose(const Shape& shape, Vec position) {
    btMatrix3x3 basis;
    const auto x = bullet(shape.axes[0]), y = bullet(shape.axes[1]), z = bullet(shape.axes[2]);
    basis.setValue(x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z());
    return btTransform(basis, bullet(position));
}

std::unique_ptr<btConvexShape> bulletShape(const Shape& shape) {
    if (shape.kind == ShapeKind::Sphere) return std::make_unique<btSphereShape>(btScalar(shape.size.x));
    if (shape.kind == ShapeKind::Capsule) {
        return std::make_unique<btCapsuleShapeZ>(btScalar(shape.size.x), btScalar(2 * shape.size.z));
    }
    auto box = std::make_unique<btBoxShape>(bullet(shape.size));
    box->setMargin(btScalar(contactMargin));
    return box;
}

Shape readShape(const btCollisionShape& shape, const btTransform& transform) {
    Shape result{};
    if (shape.getShapeType() == SPHERE_SHAPE_PROXYTYPE) {
        result.kind = ShapeKind::Sphere;
        result.size.x = static_cast<const btSphereShape&>(shape).getRadius();
    } else if (shape.getShapeType() == CAPSULE_SHAPE_PROXYTYPE) {
        const auto& capsule = static_cast<const btCapsuleShape&>(shape);
        result.kind = ShapeKind::Capsule;
        result.size = {capsule.getRadius(), 0, capsule.getHalfHeight()};
    } else {
        result.kind = ShapeKind::Box;
        result.size = voxelVec(static_cast<const btBoxShape&>(shape).getHalfExtentsWithMargin());
    }
    for (int i = 0; i < 3; ++i) result.axes[i] = voxelVec(transform.getBasis().getColumn(i));
    return result;
}

class FieldShape final : public btConcaveShape {
public:
    explicit FieldShape(const Chunk& chunk) : chunk(chunk) { m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE; }
    void getAabb(const btTransform&, btVector3& lo, btVector3& hi) const override {
        lo = bullet(chunk.surfaceBounds->lo - splat(contactMargin));
        hi = bullet(chunk.surfaceBounds->hi + splat(contactMargin));
    }
    void setLocalScaling(const btVector3&) override {}
    const btVector3& getLocalScaling() const override { return scale_; }
    void calculateLocalInertia(btScalar, btVector3& inertia) const override { inertia.setZero(); }
    const char* getName() const override { return "VoxelField"; }
    void processAllTriangles(btTriangleCallback*, const btVector3&, const btVector3&) const override {}
    const Chunk& chunk;
private:
    btVector3 scale_{1, 1, 1};
};

struct Contact { Vec point, normal; double depth; };

class FieldAlgorithm final : public btCollisionAlgorithm {
public:
    FieldAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* a,
                   const btCollisionObjectWrapper* b, bool swapped)
        : btCollisionAlgorithm(ci), manifold_(ci.m_dispatcher1->getNewManifold(a->getCollisionObject(), b->getCollisionObject())),
          swapped_(swapped) {}
    ~FieldAlgorithm() override { m_dispatcher->releaseManifold(manifold_); }

    void processCollision(const btCollisionObjectWrapper* a, const btCollisionObjectWrapper* b,
                          const btDispatcherInfo&, btManifoldResult* result) override {
        const auto* dynamic = swapped_ ? b : a;
        const auto* terrain = swapped_ ? a : b;
        const auto& chunk = static_cast<const FieldShape&>(*terrain->getCollisionShape()).chunk;
        const auto& transform = dynamic->getWorldTransform();
        const Shape shape = readShape(*dynamic->getCollisionShape(), transform);
        const Vec center = voxelVec(transform.getOrigin());
        std::vector<Contact> contacts;
        contacts.reserve(32);
        auto add = [&](Vec point, Vec normal, double depth) {
            if (depth > contactMargin || !chunk.box.expanded(splat(epsilon)).contains(point)) return;
            for (auto& existing : contacts) {
                if (length(existing.point - point) > 0.10) continue;
                if (depth < existing.depth) existing = {point, normal, depth};
                return;
            }
            contacts.push_back({point, normal, depth});
        };
        auto probe = [&](Vec seed) {
            if (!chunk.box.contains(seed)) return;
            const Cell cell = chunk.cellAt(seed);
            const double value = cell.value(seed);
            const Vec gradient = cell.gradient(seed);
            const double magnitude = length(gradient);
            if (magnitude < 1e-6 || value > contactMargin * magnitude) return;
            const Vec normal = gradient / magnitude;
            Vec surface = seed - normal * (value / magnitude);
            for (int i = 0; i < 3 && chunk.box.contains(surface); ++i) {
                const Cell local = chunk.cellAt(surface);
                const Vec g = local.gradient(surface);
                const double square = dot(g, g);
                if (square < 1e-9) break;
                surface = surface - g * (local.value(surface) / square);
            }
            if (chunk.box.contains(surface)) add(surface, normalAt(chunk.cellAt(surface), surface), dot(seed - surface, normal));
        };
        if (shape.kind == ShapeKind::Box) {
            for (int z = -1; z <= 1; ++z) for (int y = -1; y <= 1; ++y) {
                for (int x = -1; x <= 1; ++x) {
                    if (x == 0 && y == 0 && z == 0) continue;
                    probe(center + shape.direction({x * shape.size.x, y * shape.size.y, z * shape.size.z}));
                }
            }
        } else {
            const int last = shape.kind == ShapeKind::Sphere ? 0 : 4;
            for (int i = 0; i <= last; ++i) {
                const Vec axis = center + shape.axes[2] * ((last ? i * 0.5 - 1 : 0) * shape.size.z);
                const Vec p = chunk.box.clamp(axis);
                const Vec n = normalAt(chunk.cellAt(p), p);
                probe(axis - n * shape.size.x);
            }
        }
        const Box overlap = Box{center, center}.expanded(shape.extents() + splat(contactMargin));
        // Recover contacts with features between the shape probes by projecting onto the field.
        forCells(chunk, overlap, [&](const Cell& cell) {
            if (cell.minimum() > 0 || cell.maximum() < 0) return;
            const Box box{cell.origin, cell.origin + splat(cell.spacing)};
            Vec p = box.clamp(center);
            for (int i = 0; i < 6; ++i) {
                const Vec g = cell.gradient(p);
                const double square = dot(g, g);
                if (square < 1e-9) return;
                p = box.clamp(p - g * (cell.value(p) / square));
            }
            if (std::abs(cell.value(p)) < 0.01) add(p, normalAt(cell, p), shape.distance(p - center));
        });

        result->setPersistentManifold(manifold_);
        result->refreshContactPoints();
        // Bullet keeps at most four points and uses their area and depth to reduce the manifold.
        for (const Contact& contact : contacts) {
            const btVector3 n = bullet(contact.normal), point = bullet(contact.point);
            if (swapped_) result->addContactPoint(-n, point + n * btScalar(contact.depth), btScalar(contact.depth));
            else result->addContactPoint(n, point, btScalar(contact.depth));
        }
    }

    btScalar calculateTimeOfImpact(btCollisionObject*, btCollisionObject*, const btDispatcherInfo&,
                                  btManifoldResult*) override { return btScalar(1); }
    void getAllContactManifolds(btManifoldArray& manifolds) override { manifolds.push_back(manifold_); }
private:
    btPersistentManifold* manifold_;
    bool swapped_;
};

struct MakeFieldAlgorithm final : btCollisionAlgorithmCreateFunc {
    explicit MakeFieldAlgorithm(bool swapped) { m_swapped = swapped; }
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* a,
                                                   const btCollisionObjectWrapper* b) override {
        void* memory = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(FieldAlgorithm));
        return new (memory) FieldAlgorithm(ci, a, b, m_swapped);
    }
};

}  // namespace

struct Physics::Impl {
    struct Fixed {
        std::unique_ptr<btTriangleMesh> triangles;
        std::unique_ptr<btCollisionShape> shape;
        std::unique_ptr<btCollisionObject> object;
        std::size_t bytes{};
    };
    struct Body {
        std::unique_ptr<btConvexShape> shape;
        std::unique_ptr<btRigidBody> body;
    };

    Impl(Method method, const Chunks& chunks)
        : method(method), chunks(chunks), dispatcher(&configuration), world(&dispatcher, &broadphase, &solver, &configuration) {
        world.setGravity(btVector3(0, 0, 0));
        world.getSolverInfo().m_numIterations = 12;
        for (int type : {SPHERE_SHAPE_PROXYTYPE, CAPSULE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE}) {
            dispatcher.registerCollisionCreateFunc(type, CUSTOM_CONCAVE_SHAPE_TYPE, &forward);
            dispatcher.registerCollisionCreateFunc(CUSTOM_CONCAVE_SHAPE_TYPE, type, &reverse);
        }
    }
    ~Impl() {
        for (auto& body : bodies) world.removeRigidBody(body.body.get());
        for (auto& item : fixed) world.removeCollisionObject(item.second.object.get());
    }

    Method method;
    const Chunks& chunks;
    MakeFieldAlgorithm forward{false}, reverse{true};
    btDefaultCollisionConfiguration configuration;
    btCollisionDispatcher dispatcher;
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world;
    std::map<Key, Fixed> fixed;
    std::vector<Body> bodies;
};

Physics::Physics(Method method, const Chunks& chunks) : impl_(std::make_unique<Impl>(method, chunks)) {}
Physics::~Physics() = default;

void Physics::addTerrain(const Chunk& chunk) {
    Impl::Fixed fixed;
    btTransform transform;
    transform.setIdentity();
    if (impl_->method == Method::Mesh) {
        if (chunk.mesh.empty()) return;
        fixed.triangles = std::make_unique<btTriangleMesh>();
        const Vec origin = chunk.box.center();
        for (const auto& triangle : chunk.mesh) {
            const auto& p = triangle.vertices;
            fixed.triangles->addTriangle(bullet(p[0] - origin), bullet(p[1] - origin), bullet(p[2] - origin), false);
        }
        auto bvh = std::make_unique<btBvhTriangleMeshShape>(fixed.triangles.get(), true);
        bvh->setMargin(btScalar(contactMargin));
        fixed.bytes = sizeof(btTriangleMesh) + sizeof(btBvhTriangleMeshShape)
                    + chunk.mesh.size() * (3 * sizeof(btVector3) + 3 * sizeof(int))
                    + bvh->getOptimizedBvh()->calculateSerializeBufferSize();
        fixed.shape = std::move(bvh);
        transform.setOrigin(bullet(origin));
    } else {
        if (!chunk.surfaceBounds) return;
        fixed.shape = std::make_unique<FieldShape>(chunk);
        fixed.bytes = sizeof(FieldShape);
    }
    fixed.object = std::make_unique<btCollisionObject>();
    fixed.object->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
    fixed.object->setCollisionShape(fixed.shape.get());
    fixed.object->setWorldTransform(transform);
    fixed.object->setFriction(btScalar(0.8));
    fixed.bytes += sizeof(btCollisionObject) + sizeof(btDbvtProxy);
    impl_->world.addCollisionObject(fixed.object.get(), terrainGroup, bodyGroup);
    impl_->fixed.emplace(chunk.key, std::move(fixed));
}

void Physics::removeTerrain(Key key) {
    auto found = impl_->fixed.find(key);
    if (found == impl_->fixed.end()) return;
    const Box bounds = voxel::bounds(key);
    for (auto& body : impl_->bodies) {
        const Vec pos = voxelVec(body.body->getWorldTransform().getOrigin());
        if (bounds.expanded(splat(body.shape->getAngularMotionDisc())).contains(pos)) body.body->activate();
    }
    impl_->world.removeCollisionObject(found->second.object.get());
    impl_->fixed.erase(found);
}

std::size_t Physics::addBody(Shape shape, Vec position, Vec velocity) {
    Impl::Body value;
    value.shape = bulletShape(shape);
    btVector3 inertia;
    value.shape->calculateLocalInertia(1, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(1, nullptr, value.shape.get(), inertia);
    ci.m_startWorldTransform = pose(shape, position);
    ci.m_friction = btScalar(0.75);
    value.body = std::make_unique<btRigidBody>(ci);
    value.body->setLinearVelocity(bullet(velocity));
    impl_->world.addRigidBody(value.body.get(), bodyGroup, terrainGroup | bodyGroup);
    impl_->bodies.push_back(std::move(value));
    return impl_->bodies.size() - 1;
}

Vec Physics::bodyPosition(std::size_t i) const { return voxelVec(impl_->bodies.at(i).body->getWorldTransform().getOrigin()); }
double Physics::bodySpeed(std::size_t i) const { return impl_->bodies.at(i).body->getLinearVelocity().length(); }

int Physics::terrainContacts() const {
    int count = 0;
    for (int i = 0; i < impl_->dispatcher.getNumManifolds(); ++i) {
        const auto* manifold = impl_->dispatcher.getManifoldByIndexInternal(i);
        if (!manifold->getBody0()->isStaticObject() && !manifold->getBody1()->isStaticObject()) continue;
        for (int c = 0; c < manifold->getNumContacts(); ++c) {
            count += manifold->getContactPoint(c).getDistance() <= contactMargin;
        }
    }
    return count;
}

void Physics::step(double dt, bool radialGravity, Vec gravity) {
    for (auto& value : impl_->bodies) {
        auto& body = *value.body;
        const auto radial = unit(voxelVec(body.getWorldTransform().getOrigin())) * -9.81;
        body.setGravity(bullet(radialGravity ? radial : gravity));
    }
    impl_->world.stepSimulation(btScalar(dt), 0);
}

Hit Physics::ray(Vec from, Vec to) const {
    if (impl_->method != Method::Mesh) return fieldRay(impl_->chunks, from, to);
    btCollisionWorld::ClosestRayResultCallback callback(bullet(from), bullet(to));
    callback.m_collisionFilterGroup = bodyGroup;
    callback.m_collisionFilterMask = terrainGroup;
    impl_->world.rayTest(bullet(from), bullet(to), callback);
    if (!callback.hasHit()) return {};
    return {Hit::State::Contact, callback.m_closestHitFraction, voxelVec(callback.m_hitPointWorld), voxelVec(callback.m_hitNormalWorld)};
}

Hit Physics::sweep(const Shape& shape, Vec from, Vec to) const {
    if (impl_->method != Method::Mesh) return fieldSweep(impl_->chunks, shape, from, to);
    const auto convex = bulletShape(shape);
    btCollisionWorld::ClosestConvexResultCallback callback(bullet(from), bullet(to));
    callback.m_collisionFilterGroup = bodyGroup;
    callback.m_collisionFilterMask = terrainGroup;
    impl_->world.convexSweepTest(convex.get(), pose(shape, from), pose(shape, to), callback);
    if (!callback.hasHit()) return {};
    return {Hit::State::Contact, callback.m_closestHitFraction, voxelVec(callback.m_hitPointWorld), voxelVec(callback.m_hitNormalWorld),
            callback.m_closestHitFraction == 0};
}

std::size_t Physics::terrainBytes() const {
    std::size_t bytes = 0;
    for (const auto& item : impl_->fixed) bytes += item.second.bytes;
    return bytes;
}

}  // namespace voxel
