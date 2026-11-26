#include "collision.hpp"

#include <btBulletDynamicsCommon.h>
static_assert(sizeof(btScalar) == sizeof(double), "Earth-scale physics requires double-precision Bullet");
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include <limits>
#include <stdexcept>
#include <utility>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

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
void forChunks(const Chunks& chunks, Vec from, Vec to, Vec extents, Operation operation) {
    const Vec pad = extents + splat(epsilon);
    const int densityShift = chunks.empty() ? 0 : chunks.begin()->first.densityShift;
    const double size = gridChunkSize(densityShift);
    if (!chunks.empty() && chunks.rbegin()->first.level > 0) {
        const Vec delta = to - from;
        auto visit = [&](auto&& self, Key key) -> void {
            double first = 0, last = 1;
            if (!clip(bounds(key).expanded(pad), from, delta, first, last)) return;
            const auto found = chunks.find(key);
            if (found != chunks.end()) { operation(*found->second); return; }
            if (!key.level) return;
            const int step = 1 << (key.level - 1);
            for (int i = 0; i < 8; ++i)
                self(self, Key{key.x + (i & 1) * step, key.y + ((i >> 1) & 1) * step,
                               key.z + (i >> 2) * step, key.level - 1, densityShift});
        };
        const int level = gridRootLevel(densityShift), step = 1 << level;
        for (int i = 0; i < 8; ++i)
            visit(visit, Key{(i & 1) ? 0 : -step, (i & 2) ? 0 : -step, (i & 4) ? 0 : -step, level, densityShift});
        return;
    }
    const Key lo = keyAt(min(from, to) - pad, densityShift), hi = keyAt(max(from, to) + pad, densityShift);
    auto emit = [&](int x, int y, int z) {
        const auto found = chunks.find({x, y, z, 0, densityShift});
        if (found != chunks.end()) operation(*found->second);
    };
    if (hi.x - lo.x <= 1 && hi.y - lo.y <= 1 && hi.z - lo.z <= 1) {
        for (int z = lo.z; z <= hi.z; ++z) for (int y = lo.y; y <= hi.y; ++y) {
            for (int x = lo.x; x <= hi.x; ++x) emit(x, y, z);
        }
        return;
    }
    // Clip each candidate row to the swept time interval in the preceding axes.
    // This keeps a long diagonal cast proportional to its swept tube.
    const Vec delta = to - from;
    auto slice = [&](int axis, int index, double& first, double& last) {
        const double lower = index * size - pad[axis];
        const double upper = (index + 1) * size + pad[axis];
        if (std::abs(delta[axis]) < 1e-13) return from[axis] >= lower && from[axis] <= upper;
        const double a = (lower - from[axis]) / delta[axis], b = (upper - from[axis]) / delta[axis];
        first = std::max(first, std::min(a, b));
        last = std::min(last, std::max(a, b));
        return first <= last;
    };
    auto range = [&](int axis, double first, double last) {
        const double a = from[axis] + delta[axis] * first, b = from[axis] + delta[axis] * last;
        return std::make_pair(int(std::floor((std::min(a, b) - pad[axis]) / size)),
                              int(std::floor((std::max(a, b) + pad[axis]) / size)));
    };
    for (int z = lo.z; z <= hi.z; ++z) {
        double beforeZ = 0, afterZ = 1;
        if (!slice(2, z, beforeZ, afterZ)) continue;
        const auto ys = range(1, beforeZ, afterZ);
        for (int y = ys.first; y <= ys.second; ++y) {
            double beforeY = beforeZ, afterY = afterZ;
            if (!slice(1, y, beforeY, afterY)) continue;
            const auto xs = range(0, beforeY, afterY);
            for (int x = xs.first; x <= xs.second; ++x) emit(x, y, z);
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

int trailingBit(std::uint32_t bits) {
#if defined(_MSC_VER)
    unsigned long index;
    _BitScanForward(&index, bits);
    return int(index);
#else
    return __builtin_ctz(bits);
#endif
}

std::uint32_t cellMask(int lo, int hi) { return (0xffffffffu << lo) & (0xffffffffu >> (31 - hi)); }

template<class Operation>
void forIndexedCells(const Chunk& chunk, Box area, bool solid, Operation operation) {
    const Key lo = chunk.cellIndex(area.lo - splat(epsilon));
    const Key hi = chunk.cellIndex(area.hi + splat(epsilon));
    const auto xMask = cellMask(lo.x, hi.x);
    for (int z = lo.z; z <= hi.z; ++z) for (int y = lo.y; y <= hi.y; ++y) {
        const auto& row = chunk.row(y, z);
        auto cells = xMask & (row.surface | (solid ? row.solid : 0));
        for (; cells; cells &= cells - 1) operation(Key{trailingBit(cells), y, z});
    }
}

template<class Operation>
void forSweptCells(const Chunk& chunk, Vec from, Vec delta, Vec extents, double first, double last,
                   Operation operation) {
    const Vec a = from + delta * first, b = from + delta * last;
    const Vec pad = extents + splat(epsilon);
    const Key lo = chunk.cellIndex(min(a, b) - pad), hi = chunk.cellIndex(max(a, b) + pad);
    auto slab = [&](int axis, int index, double& low, double& high) {
        const double start = chunk.box.lo[axis] + index * chunk.spacing;
        const double lower = start - pad[axis], upper = start + chunk.spacing + pad[axis];
        if (std::abs(delta[axis]) < 1e-13) return from[axis] >= lower && from[axis] <= upper;
        const double enter = (lower - from[axis]) / delta[axis], exit = (upper - from[axis]) / delta[axis];
        low = std::max(low, std::min(enter, exit));
        high = std::min(high, std::max(enter, exit));
        return low <= high;
    };
    auto coord = [&](int axis, double t) { return from[axis] + delta[axis] * t; };
    auto index = [&](int axis, double value) {
        return std::clamp(int(std::floor((value - chunk.box.lo[axis]) / chunk.spacing)), 0, chunk.cells - 1);
    };
    for (int z = lo.z; z <= hi.z; ++z) {
        double zFirst = first, zLast = last;
        if (!slab(2, z, zFirst, zLast)) continue;
        const int yFirst = index(1, std::min(coord(1, zFirst), coord(1, zLast)) - pad.y);
        const int yLast = index(1, std::max(coord(1, zFirst), coord(1, zLast)) + pad.y);
        for (int y = yFirst; y <= yLast; ++y) {
            const auto& row = chunk.row(y, z);
            if (!row.surface) continue;
            double rowFirst = zFirst, rowLast = zLast;
            if (!slab(1, y, rowFirst, rowLast)) continue;
            const int xFirst = index(0, std::min(coord(0, rowFirst), coord(0, rowLast)) - pad.x);
            const int xLast = index(0, std::max(coord(0, rowFirst), coord(0, rowLast)) + pad.x);
            for (auto mask = row.surface & cellMask(xFirst, xLast); mask; mask &= mask - 1) {
                const int x = trailingBit(mask);
                double before = rowFirst, after = rowLast;
                if (slab(0, x, before, after)) operation(Key{x, y, z}, before, after);
            }
        }
    }
}

Vec normalAt(const Cell& cell, Vec point) { return unit(cell.gradient(point), unit(point)); }

struct Trilinear {
    Vec origin, linear;
    double inverse, constant;
    double xy, xz, yz, xyz;
    explicit Trilinear(const Cell& cell) : origin(cell.origin), inverse(1.0 / cell.spacing), constant(cell.values[0]) {
        const auto v = [&](int index) { return double(cell.values[index]); };
        linear = {v(1) - constant, v(2) - constant, v(4) - constant};
        xy = v(3) - v(2) - v(1) + v(0);
        xz = v(5) - v(4) - v(1) + v(0);
        yz = v(6) - v(4) - v(2) + v(0);
        xyz = v(7) - v(6) - v(5) - v(3) + v(4) + v(2) + v(1) - v(0);
    }
    Vec local(Vec point) const { return (point - origin) * inverse; }
    double value(Vec p) const {
        return constant + linear.y * p.y + p.z * (linear.z + yz * p.y)
             + p.x * (linear.x + xy * p.y + p.z * (xz + xyz * p.y));
    }
    Vec gradient(Vec p) const {
        return Vec{linear.x + xy * p.y + p.z * (xz + xyz * p.y),
                   linear.y + xy * p.x + p.z * (yz + xyz * p.x),
                   linear.z + xz * p.x + p.y * (yz + xyz * p.x)} * inverse;
    }
    double lower(Vec p, Vec half) const {
        const Vec d = half * inverse;
        const double a = (xy + p.z * xyz) * d.x * d.y;
        const double b = (xz + p.y * xyz) * d.x * d.z;
        const double c = (yz + p.x * xyz) * d.y * d.z;
        return std::min({a + b + c, a - b - c, b - a - c, c - a - b}) - std::abs(xyz) * d.x * d.y * d.z;
    }
};

double firstRoot(const Cell& cell, Vec from, Vec to) {
    const Trilinear field(cell);
    const Vec p = field.local(from);
    const Vec worldDelta = to - from;
    const Vec d = worldDelta * field.inverse;
    const double start = field.value(p);
    if (std::abs(start) <= epsilon) return 0;
    const double sign = start < 0 ? -1 : 1;
    const double f0 = sign * start;
    // Substituting the ray in the trilinear polynomial gives a cubic in t.
    const double c = sign * dot(field.gradient(p), worldDelta);
    const double b = sign * ((field.xy + p.z * field.xyz) * d.x * d.y + (field.xz + p.y * field.xyz) * d.x * d.z
                           + (field.yz + p.x * field.xyz) * d.y * d.z);
    const double a = sign * field.xyz * d.x * d.y * d.z;
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
        const double fHi = value(hi);
        if (fHi >= 0) return hi;
        const double fLo = value(lo);
        double trial = lo + (hi - lo) * (fLo / (fLo - fHi));
        for (int i = 0; i < 5; ++i) {
            const double residual = value(trial);
            if (std::abs(residual) < 1e-10) return trial;
            if (residual > 0) lo = trial;
            else hi = trial;
            const double derivative = (3 * a * trial + 2 * b) * trial + c;
            const double next = std::abs(derivative) > 1e-12 ? trial - residual / derivative : lo;
            trial = next > lo && next < hi ? next : (lo + hi) * 0.5;
        }
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
    if (chunk.box.contains(from)) {
        const Key start = chunk.cellIndex(from);
        const auto& row = chunk.row(start.y, start.z);
        const std::uint32_t bit = std::uint32_t(1) << start.x;
        if ((row.surface | row.solid) & bit) {
            const auto cell = chunk.cell(start);
            if (cell.value(from) <= epsilon) {
                hit = {Hit::State::Contact, 0, from, normalAt(cell, from), true};
                return;
            }
        }
    }
    if (!chunk.surfaceBounds) return;
    double first = 0, last = hit.fraction;
    if (!clip(*chunk.surfaceBounds, from, delta, first, last)) return;
    const Vec p = from + delta * first;
    Key index = chunk.cellIndex(p);
    Vec exits = splat(std::numeric_limits<double>::infinity());
    Vec steps = exits;
    for (int a = 0; a < 3; ++a) {
        if (std::abs(delta[a]) < 1e-13) continue;
        const double coord = (p[a] - chunk.box.lo[a]) / chunk.spacing;
        if (delta[a] < 0 && std::abs(coord - index[a]) < 1e-9 && index[a] > 0) --index[a];
        const double edge = chunk.box.lo[a] + (index[a] + (delta[a] > 0)) * chunk.spacing;
        exits[a] = (edge - from[a]) / delta[a];
        steps[a] = chunk.spacing / std::abs(delta[a]);
    }
    while (first <= last) {
        const double end = std::max(first, std::min({last, exits.x, exits.y, exits.z}));
        if (chunk.row(index.y, index.z).surface & (std::uint32_t(1) << index.x)) {
            const auto cell = chunk.cell(index);
            const double fraction = firstRoot(cell, from + delta * first, from + delta * end);
            if (fraction >= 0) {
                const double t = first + (end - first) * fraction;
                const Vec point = from + delta * t;
                hit = {Hit::State::Contact, t, point, normalAt(cell, point), t == 0};
                return;
            }
        }
        if (end >= last) return;
        bool advanced = false;
        for (int a = 0; a < 3; ++a) {
            if (exits[a] > end + 1e-12) continue;
            index[a] += delta[a] > 0 ? 1 : -1;
            if (index[a] < 0 || index[a] >= chunk.cells) return;
            exits[a] += steps[a];
            advanced = true;
        }
        if (!advanced) return;
        first = end;
    }
}

Vec corner(Box box, int i) {
    return {(i & 1) ? box.hi.x : box.lo.x, (i & 2) ? box.hi.y : box.lo.y, (i & 4) ? box.hi.z : box.lo.z};
}

Box zeroBounds(const Cell& cell) {
    Box local{splat(1), {}};
    constexpr Box unitCell{{}, {1, 1, 1}};
    bool found = false;
    for (int i = 0; i < 8; ++i) {
        const double a = cell.values[i];
        if (std::abs(a) < epsilon) {
            const Vec p = corner(unitCell, i);
            local.lo = min(local.lo, p);
            local.hi = max(local.hi, p);
            found = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            const int bit = 1 << axis;
            if (i & bit) continue;
            const double b = cell.values[i | bit];
            if ((a < 0) == (b < 0)) continue;
            Vec p = corner(unitCell, i);
            p[axis] = a / (a - b);
            local.lo = min(local.lo, p);
            local.hi = max(local.hi, p);
            found = true;
        }
    }
    const Box region = found ? local : unitCell;
    return {cell.origin + region.lo * cell.spacing - splat(epsilon),
            cell.origin + region.hi * cell.spacing + splat(epsilon)};
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

double supportRadius(const Shape& shape, Vec axis) {
    if (shape.kind == ShapeKind::Box) {
        return std::abs(dot(axis, shape.axes[0])) * shape.size.x + std::abs(dot(axis, shape.axes[1])) * shape.size.y
             + std::abs(dot(axis, shape.axes[2])) * shape.size.z;
    }
    const double sphere = length(axis) * shape.size.x;
    return shape.kind == ShapeKind::Capsule ? sphere + std::abs(dot(axis, shape.axes[2])) * shape.size.z : sphere;
}

struct SweepSearch {
    struct Axis { Vec direction, absolute; double start, slope, radius; };

    const Shape& shape;
    Vec from, delta, extents;
    double speed;
    Hit hit;
    double unknown{std::numeric_limits<double>::infinity()};
    int budget{sweepBudget};
    std::array<Axis, 12> axes{};
    int axisCount{};

    SweepSearch(const Shape& input, Vec start, Vec end)
        : shape(input), from(start), delta(end - start), extents(input.extents()), speed(length(delta)) {
        constexpr Vec world[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        if (shape.kind == ShapeKind::Box) {
            for (const auto direction : shape.axes) {
                addAxis(direction);
                for (Vec basis : world) addAxis(cross(direction, basis));
            }
        } else if (shape.kind == ShapeKind::Capsule) {
            addAxis(shape.axes[2]);
            for (Vec basis : world) addAxis(cross(shape.axes[2], basis));
        }
    }

    Axis makeAxis(Vec direction) const {
        return {direction, abs(direction), dot(direction, from), dot(direction, delta), supportRadius(shape, direction)};
    }

    void addAxis(Vec direction) {
        if (dot(direction, direction) > 1e-16) axes[axisCount++] = makeAxis(direction);
    }

    bool clipAxis(const Axis& axis, Vec center, Vec half, double& first, double& last) const {
        const double origin = axis.start - dot(axis.direction, center);
        const double range = axis.radius + dot(axis.absolute, half) + epsilon;
        if (std::abs(axis.slope) < 1e-12) return std::abs(origin) <= range;
        const double r = range / std::abs(axis.slope);
        const double middle = -origin / axis.slope;
        first = std::max(first, middle - r);
        last = std::min(last, middle + r);
        return first <= last;
    }

    bool clipRounded(Box box, Vec center, Vec half, double& first, double& last) const {
        if (shape.kind == ShapeKind::Box) return true;
        const double time = speed > 0 ? std::clamp(dot(center - from, delta) / (speed * speed), first, last) : first;
        const Vec pose = from + delta * time;
        Vec point = pose;
        if (shape.kind == ShapeKind::Capsule) {
            const Vec axis = shape.axes[2];
            for (int i = 0; i < 2; ++i) {
                point = pose + axis * std::clamp(dot(box.clamp(point) - pose, axis), -shape.size.z, shape.size.z);
            }
        }
        const Vec separation = point - box.clamp(point);
        if (dot(separation, separation) < 1e-14) return true;
        return clipAxis(makeAxis(unit(separation)), center, half, first, last);
    }

    bool clipPlane(Vec gradient, double threshold, double& first, double& last) const {
        const double minimum = dot(gradient, from) - supportRadius(shape, gradient);
        const double slope = dot(gradient, delta);
        if (std::abs(slope) <= 1e-12) return minimum <= threshold;
        const double limit = (threshold - minimum) / slope;
        if (slope < 0) first = std::max(first, limit);
        else last = std::min(last, limit);
        return first <= last;
    }

    bool clipConstrainedPlane(Vec gradient, double threshold, Box box, Vec extreme, double& first, double& last) const {
        // Combining the tangent plane with a face inequality bounds support outside the cell.
        // A second pass can combine that constraint with a different face at a cell edge.
        for (int pass = 0; pass < 2; ++pass) {
            const Vec center = from + delta * first;
            const Vec witness = center + extreme;
            const double scale = length(gradient);
            Vec next = gradient;
            double nextThreshold = threshold;
            bool tightened = false;
            for (int axis = 0; axis < 3; ++axis) {
                const double face = std::clamp(witness[axis], box.lo[axis], box.hi[axis]);
                const double difference = witness[axis] - face;
                if (std::abs(difference) < 0.01) continue;
                const double sign = difference > 0 ? 1 : -1;
                auto test = [&](double adjustment) {
                    Vec normal = gradient;
                    normal[axis] += adjustment;
                    const double limit = threshold + adjustment * face + epsilon;
                    const double oldFirst = first;
                    if (!clipPlane(normal, limit, first, last)) return false;
                    if (first > oldFirst + 1e-10) {
                        next = normal;
                        nextThreshold = limit;
                        tightened = true;
                    }
                    return true;
                };
                if (shape.kind == ShapeKind::Box) {
                    for (Vec basis : shape.axes) {
                        if (std::abs(basis[axis]) < 1e-8) continue;
                        const double adjustment = -dot(gradient, basis) / basis[axis];
                        if (adjustment * sign > 0 && !test(adjustment)) return false;
                    }
                } else {
                    Vec tangent = gradient;
                    tangent[axis] = 0;
                    const double axial = shape.kind == ShapeKind::Capsule
                        ? extreme[axis] + gradient[axis] * (scale > 1e-10 ? shape.size.x / scale : 0) : 0;
                    const double k = face - center[axis] - axial;
                    if (std::abs(k) < shape.size.x - 1e-7) {
                        const double adjustment = -k * length(tangent) / std::sqrt(shape.size.x * shape.size.x - k * k) - gradient[axis];
                        if (adjustment * sign > 0 && !test(adjustment)) return false;
                    }
                    if (!test(sign * scale * 0.6) || !test(sign * scale * 1.8)) return false;
                    if (shape.kind == ShapeKind::Capsule && std::abs(shape.axes[2][axis]) > 1e-8) {
                        const double adjustment = -dot(gradient, shape.axes[2]) / shape.axes[2][axis];
                        if (adjustment * sign > 0 && !test(adjustment)) return false;
                    }
                }
            }
            if (!tightened || pass == 1) break;
            gradient = next;
            threshold = nextThreshold;
            extreme = support(shape, gradient * -1);
        }
        return true;
    }

    double cutoff() const {
        return hit.state == Hit::State::Contact && speed > 0
            ? std::max(0.0, hit.fraction - sweepTolerance / speed) : hit.fraction;
    }

    void sample(const Trilinear& field, Box box, Vec point, double time) {
        if (time > hit.fraction) return;
        const Vec center = from + delta * time;
        point = center + projectToShape(shape, point - center);
        if (!box.expanded(splat(epsilon)).contains(point)) return;
        const Vec p = field.local(point);
        if (field.value(p) > 2 * epsilon) return;
        const Vec n = unit(field.gradient(p), unit(point));
        hit = {Hit::State::Contact, time, point, n, time == 0};
    }

    void visit(const Cell& cell, const Trilinear& curvature, Box box, double first, double last, bool seed = false) {
        last = std::min(last, cutoff());
        if (first > last || hit.initial || !clip(box.expanded(extents), from, delta, first, last)) return;
        const double priorSpan = last - first;
        const Vec begin = from + delta * first, end = from + delta * last;
        box.lo = max(box.lo, min(begin, end) - extents - splat(epsilon));
        box.hi = min(box.hi, max(begin, end) + extents + splat(epsilon));
        const Vec center = box.center();
        const Vec dimensions = box.hi - box.lo;
        const Vec half = dimensions * 0.5;
        for (int i = 0; i < axisCount; ++i) if (!clipAxis(axes[i], center, half, first, last)) return;
        if (!clipRounded(box, center, half, first, last)) return;
        const Vec local = curvature.local(center);
        const Vec gradient = curvature.gradient(local);
        const double centerValue = curvature.value(local);
        const double residual = curvature.lower(local, half);
        if (centerValue - dot(abs(gradient), half) + residual > epsilon) return;
        const Vec deepest{gradient.x >= 0 ? box.lo.x : box.hi.x, gradient.y >= 0 ? box.lo.y : box.hi.y,
                          gradient.z >= 0 ? box.lo.z : box.hi.z};
        // The residual from the tangent plane is trilinear and bounded at box corners.
        // The shape's support point bounds the tangent plane over every pose on the path.
        const Vec extreme = support(shape, gradient * -1);
        const double threshold = dot(gradient, center) - centerValue - residual + epsilon;
        if (!clipPlane(gradient, threshold, first, last) ||
            !clipConstrainedPlane(gradient, threshold, box, extreme, first, last)) return;
        if (budget-- <= 0) {
            unknown = std::min(unknown, first);
            return;
        }
        sample(curvature, box, from + delta * first + extreme, first);
        sample(curvature, box, deepest, first);
        sample(curvature, box, center, first);
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
                    sample(curvature, box, point + delta * time, time);
                }
            }
        }
        last = std::min(last, cutoff());
        if (first > last || hit.initial) return;
        const double midpoint = (first + last) * 0.5;
        const double travel = speed * (last - first);
        if (speed * priorSpan > 2 * sweepTolerance && last - first < priorSpan * 0.4) {
            visit(cell, curvature, box, first, last);
            return;
        }
        sample(curvature, box, from + delta * midpoint + extreme, midpoint);
        sample(curvature, box, deepest, midpoint);
        const int axis = dimensions.x >= dimensions.y && dimensions.x >= dimensions.z ? 0
                       : dimensions.y >= dimensions.z ? 1 : 2;
        if (travel <= sweepTolerance && dimensions[axis] <= sweepTolerance) {
            sample(curvature, box, deepest, last);
            sample(curvature, box, center, midpoint);
            unknown = std::min(unknown, first);
            return;
        }
        if (travel > sweepTolerance && travel >= dimensions[axis]) {
            visit(cell, curvature, box, first, midpoint);
            visit(cell, curvature, box, midpoint, last);
        } else {
            Box low = box, high = box;
            low.hi[axis] = high.lo[axis] = center[axis];
            if (delta[axis] < 0) std::swap(low, high);
            visit(cell, curvature, low, first, last);
            visit(cell, curvature, high, first, last);
        }
    }
};

}  // namespace

Hit triangleRay(const Triangle& triangle, Vec from, Vec to) {
    Hit hit;
    const Vec delta = to - from;
    const double distance = length(delta);
    if (distance <= 0) return hit;
    const Vec direction = delta / distance;
    const Vec a = triangle.vertices[0], e1 = triangle.vertices[1] - a, e2 = triangle.vertices[2] - a;
    const Vec p = cross(direction, e2);
    const double determinant = dot(e1, p);
    if (std::abs(determinant) <= 1e-12 * length(e1) * length(e2)) return hit;
    const Vec offset = from - a;
    const double u = dot(offset, p) / determinant;
    if (u < -1e-9 || u > 1 + 1e-9) return hit;
    const Vec q = cross(offset, e1);
    const double v = dot(direction, q) / determinant;
    if (v < -1e-9 || u + v > 1 + 1e-9) return hit;
    const double fraction = dot(e2, q) / (determinant * distance);
    if (fraction < 0 || fraction > 1) return hit;
    hit.state = Hit::State::Contact;
    hit.fraction = fraction;
    hit.position = from + delta * fraction;
    hit.normal = unit(cross(e1, e2));
    return hit;
}

Hit meshRay(const Chunks& chunks, Vec from, Vec to, Triangle* support, Key* owner) {
    Hit hit;
    forChunks(chunks, from, to, {}, [&](const Chunk& chunk) {
        for (const auto& triangle : chunk.mesh) {
            const auto candidate = triangleRay(triangle, from, to);
            if (candidate.state != Hit::State::Contact ||
                (hit.state == Hit::State::Contact && candidate.fraction >= hit.fraction)) continue;
            hit = candidate;
            if (support) *support = triangle;
            if (owner) *owner = chunk.key;
        }
    });
    return hit;
}

Hit fieldRay(const Chunks& chunks, Vec from, Vec to) {
    Hit hit;
    const Vec delta = to - from;
    forChunks(chunks, from, to, {}, [&](const Chunk& chunk) {
        if (!hit.initial) rayChunk(chunk, from, delta, hit);
    });
    return hit;
}

Hit fieldSweep(const Chunks& chunks, const Shape& shape, Vec from, Vec to) {
    SweepSearch search(shape, from, to);
    // A ray through any point fixed on the shape supplies a verified upper bound.
    // The interval search below still rules out earlier contacts elsewhere on the shape.
    const Hit center = fieldRay(chunks, from, to);
    if (center.initial) return center;
    if (center.state == Hit::State::Contact) {
        const Vec offset = support(shape, center.normal * -1);
        const Hit extreme = fieldRay(chunks, from + offset, to + offset);
        search.hit = extreme.fraction < center.fraction ? extreme : center;
        if (search.hit.initial) return search.hit;
    }
    struct Candidate { Cell cell; Box box; double first, last; };
    std::vector<Candidate> candidates;
    const Box initial = Box{from, from}.expanded(search.extents);
    const Vec limit = from + search.delta * search.cutoff();
    forChunks(chunks, from, limit, search.extents, [&](const Chunk& chunk) {
        if (search.hit.initial) return;
        double first = 0, last = search.cutoff();
        if (chunk.box.expanded(search.extents).contains(from)) {
            forIndexedCells(chunk, initial, true, [&](Key key) {
                if (search.hit.initial) return;
                const auto cell = chunk.cell(key);
                search.visit(cell, Trilinear(cell), {cell.origin, cell.origin + splat(cell.spacing)}, 0, 0);
            });
        }
        if (search.hit.initial || !chunk.surfaceBounds ||
            !clip(chunk.surfaceBounds->expanded(search.extents), from, search.delta, first, last)) return;
        forSweptCells(chunk, from, search.delta, search.extents, first, last, [&](Key key, double enter, double exit) {
            const Cell cell = chunk.cell(key);
            const Box box = zeroBounds(cell);
            if (clip(box.expanded(search.extents), from, search.delta, enter, exit)) {
                candidates.push_back({cell, box, enter, exit});
            }
        });
    });
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.first < b.first;
    });
    for (const auto& candidate : candidates) {
        if (search.hit.initial || candidate.first > search.cutoff()) break;
        search.visit(candidate.cell, Trilinear(candidate.cell), candidate.box, candidate.first, candidate.last, true);
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

struct Physics::PreparedTerrain {
    std::unique_ptr<btTriangleMesh> triangles;
    std::unique_ptr<btCollisionShape> shape;
    std::unique_ptr<btCollisionObject> object;
    std::size_t bytes{};
};

struct Physics::Impl {
    struct Body {
        std::unique_ptr<btConvexShape> shape;
        std::unique_ptr<btRigidBody> body;
    };

    Impl(Method method, const Chunks& chunks)
        : method(method), chunks(chunks), dispatcher(&configuration), world(&dispatcher, &broadphase, &solver, &configuration) {
        world.setGravity(btVector3(0, 0, 0));
        world.setForceUpdateAllAabbs(false);
        world.getSolverInfo().m_numIterations = 12;
        for (int type : {SPHERE_SHAPE_PROXYTYPE, CAPSULE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE}) {
            dispatcher.registerCollisionCreateFunc(type, CUSTOM_CONCAVE_SHAPE_TYPE, &forward);
            dispatcher.registerCollisionCreateFunc(CUSTOM_CONCAVE_SHAPE_TYPE, type, &reverse);
        }
    }
    ~Impl() {
        for (auto& body : bodies) world.removeRigidBody(body.body.get());
        for (auto& item : fixed) world.removeCollisionObject(item.second->object.get());
    }

    void wakeBodies(Box box) {
        for (auto& body : bodies) {
            const Vec pos = voxelVec(body.body->getWorldTransform().getOrigin());
            if (box.expanded(splat(body.shape->getAngularMotionDisc())).contains(pos)) body.body->activate();
        }
    }

    Method method;
    const Chunks& chunks;
    MakeFieldAlgorithm forward{false}, reverse{true};
    btDefaultCollisionConfiguration configuration;
    btCollisionDispatcher dispatcher;
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world;
    std::map<Key, Terrain> fixed;
    std::size_t terrainBytes{};
    std::vector<Body> bodies;
};

Physics::Physics(Method method, const Chunks& chunks) : impl_(std::make_unique<Impl>(method, chunks)) {}
Physics::~Physics() = default;

Physics::Terrain Physics::prepareTerrain(const Chunk& chunk, Method method) {
    if (method == Method::Mesh ? chunk.mesh.empty() : !chunk.surfaceBounds) return {};
    auto terrain = std::make_shared<PreparedTerrain>();
    auto& fixed = *terrain;
    btTransform transform;
    transform.setIdentity();
    if (method == Method::Mesh) {
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
        fixed.shape = std::make_unique<FieldShape>(chunk);
        fixed.bytes = sizeof(FieldShape);
    }
    fixed.object = std::make_unique<btCollisionObject>();
    fixed.object->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
    // The terrain proxy stays at its insertion bounds until the chunk is replaced.
    fixed.object->setActivationState(ISLAND_SLEEPING);
    fixed.object->setCollisionShape(fixed.shape.get());
    fixed.object->setWorldTransform(transform);
    fixed.object->setFriction(btScalar(0.8));
    fixed.bytes += sizeof(btCollisionObject) + sizeof(btDbvtProxy);
    return terrain;
}

std::size_t Physics::preparedTerrainBytes(const Terrain& terrain) { return terrain ? terrain->bytes : 0; }

void Physics::addTerrain(const Chunk& chunk) { addTerrain(chunk, prepareTerrain(chunk, impl_->method)); }

void Physics::addTerrain(const Chunk& chunk, Terrain terrain) {
    if (!terrain) return;
    const auto [entry, inserted] = impl_->fixed.try_emplace(chunk.key, std::move(terrain));
    if (!inserted) throw std::logic_error("Terrain collider already registered for this chunk");
    impl_->world.addCollisionObject(entry->second->object.get(), terrainGroup, bodyGroup);
    impl_->wakeBodies(chunk.box);
    impl_->terrainBytes += entry->second->bytes;
}

Physics::Terrain Physics::removeTerrain(Key key) {
    auto found = impl_->fixed.find(key);
    if (found == impl_->fixed.end()) return {};
    impl_->wakeBodies(bounds(key));
    impl_->world.removeCollisionObject(found->second->object.get());
    impl_->terrainBytes -= found->second->bytes;
    auto retired = impl_->fixed.extract(found);
    return std::move(retired.mapped());
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

BodyPose Physics::bodyPose(std::size_t index) const {
    const auto& transform = impl_->bodies.at(index).body->getWorldTransform();
    const auto& basis = transform.getBasis();
    return {voxelVec(transform.getOrigin()), {voxelVec(basis.getColumn(0)), voxelVec(basis.getColumn(1)), voxelVec(basis.getColumn(2))}};
}

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
    btCollisionWorld::ClosestConvexResultCallback callback(bullet(from), bullet(to));
    callback.m_collisionFilterGroup = bodyGroup;
    callback.m_collisionFilterMask = terrainGroup;
    const auto start = pose(shape, from), end = pose(shape, to);
    if (shape.kind == ShapeKind::Sphere) {
        const btSphereShape convex{btScalar(shape.size.x)};
        impl_->world.convexSweepTest(&convex, start, end, callback);
    } else if (shape.kind == ShapeKind::Capsule) {
        const btCapsuleShapeZ convex{btScalar(shape.size.x), btScalar(2 * shape.size.z)};
        impl_->world.convexSweepTest(&convex, start, end, callback);
    } else {
        btBoxShape convex{bullet(shape.size)};
        convex.setMargin(btScalar(contactMargin));
        impl_->world.convexSweepTest(&convex, start, end, callback);
    }
    if (!callback.hasHit()) return {};
    return {Hit::State::Contact, callback.m_closestHitFraction, voxelVec(callback.m_hitPointWorld), voxelVec(callback.m_hitNormalWorld),
            callback.m_closestHitFraction == 0};
}

std::size_t Physics::terrainBytes() const {
    return impl_->terrainBytes;
}

}  // namespace voxel
