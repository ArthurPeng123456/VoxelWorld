#include "sweep.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace voxel {
namespace {

constexpr double pi = 3.14159265358979323846;
// Two contact points closer than this along the mesh normal count as the same
// answer: the verdict then calls the cast OK instead of early or penetrating.
constexpr double sweepContactSlack = 0.1;

std::uint32_t sweepColor(Vec rgb, int code) {
    auto byte = [](double value) { return std::uint32_t(std::lround(std::clamp(value, 0.0, 1.0) * 255)); };
    return byte(rgb.x) | (byte(rgb.y) << 8) | (byte(rgb.z) << 16) | (std::uint32_t(code) << 24);
}

Vertex sweepVertex(Vec p, Vec normal, std::uint32_t material) {
    return {{p.x, p.y, p.z}, normal, material};
}

void flatTriangle(std::vector<Vertex>& batch, Vec a, Vec b, Vec c, Vec outward, std::uint32_t material) {
    Vec n = unit(cross(b - a, c - a), outward);
    if (dot(n, outward) < 0) {
        n = n * -1;
        std::swap(b, c);
    }
    for (const Vec p : {a, b, c}) batch.push_back(sweepVertex(p, n, material));
}

void appendBoxShape(std::vector<Vertex>& batch, const Shape& shape, Vec p, std::uint32_t material) {
    const auto transform = [&](Vec v) { return p + shape.direction(v); };
    for (int axis = 0; axis < 3; ++axis) for (int side : {-1, 1}) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        Vec a = shape.size * -1, b = a, c = a, d = a;
        a[axis] = b[axis] = c[axis] = d[axis] = side * shape.size[axis];
        b[u] = c[u] = shape.size[u];
        c[v] = d[v] = shape.size[v];
        const Vec normal = shape.axes[axis] * side;
        flatTriangle(batch, transform(a), transform(b), transform(c), normal, material);
        flatTriangle(batch, transform(a), transform(c), transform(d), normal, material);
    }
}

void appendRoundShape(std::vector<Vertex>& batch, const Shape& shape, Vec p, std::uint32_t material) {
    constexpr int rings = 5;
    constexpr int slices = 12;
    const double segment = shape.kind == ShapeKind::Capsule ? shape.size.z : 0;
    const int rows = 2 * (rings + 1);
    auto at = [&](int row, int column) {
        const bool bottom = row <= rings;
        const double latitude = (bottom ? row - rings : row - rings - 1) * (pi * 0.5 / rings);
        const double angle = column * (2 * pi / slices);
        const Vec n{std::cos(latitude) * std::cos(angle), std::cos(latitude) * std::sin(angle), std::sin(latitude)};
        const Vec offset = n * shape.size.x + Vec{0, 0, bottom ? -segment : segment};
        return sweepVertex(p + shape.direction(offset), shape.direction(n), material);
    };
    for (int row = 0; row + 1 < rows; ++row) {
        if (row == rings && segment == 0) continue;
        for (int i = 0; i < slices; ++i) {
            const auto a = at(row, i), b = at(row, i + 1), c = at(row + 1, i + 1), d = at(row + 1, i);
            if (row > 0) batch.insert(batch.end(), {a, b, c});
            if (row < rows - 2) batch.insert(batch.end(), {a, c, d});
        }
    }
}

// Seven-segment digits, drawn as line segments in the camera plane. Segment
// order: a top, b top right, c bottom right, d bottom, e bottom left, f top left,
// g middle, each as two corners in a 1 x 2 cell.
constexpr double glyphSegments[7][4] = {
    {0, 2, 1, 2}, {1, 2, 1, 1}, {1, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 1, 0, 2}, {0, 1, 1, 1}};

int glyphBits(char c) {
    switch (c) {
        case '0': return 63;
        case '1': return 6;
        case '2': return 91;
        case '3': return 79;
        case '4': return 102;
        case '5': return 109;
        case '6': return 125;
        case '7': return 7;
        case '8': return 127;
        case '9': return 111;
        case '-': return 64;
        default: return 0;
    }
}

void appendLabel(std::vector<Vertex>& lines, const std::string& text, Vec origin, Vec right, Vec up,
                 double height, std::uint32_t material) {
    const double width = 0.55 * height, advance = 0.85 * height;
    auto segment = [&](const Vec& cell, double u0, double v0, double u1, double v1) {
        lines.push_back(sweepVertex(cell + right * (u0 * width) + up * (v0 * 0.5 * height), {0, 0, 1}, material));
        lines.push_back(sweepVertex(cell + right * (u1 * width) + up * (v1 * 0.5 * height), {0, 0, 1}, material));
    };
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const Vec cell = origin + right * (double(i) * advance);
        if (c == '.') {
            segment(cell, 0.30, 0, 0.70, 0);
            continue;
        }
        const int bits = glyphBits(c);
        for (int s = 0; s < 7; ++s) {
            if ((bits & (1 << s)) == 0) continue;
            segment(cell, glyphSegments[s][0], glyphSegments[s][1], glyphSegments[s][2], glyphSegments[s][3]);
        }
    }
}

void appendMarker(std::vector<Vertex>& lines, Vec point, Vec right, Vec up, double size, std::uint32_t material) {
    for (const Vec direction : {right, up, unit(right + up)}) {
        lines.push_back(sweepVertex(point - direction * size, {0, 0, 1}, material));
        lines.push_back(sweepVertex(point + direction * size, {0, 0, 1}, material));
    }
}

const wchar_t* contactTag(Vec delta, Vec normal) {
    const double gap = dot(delta, normal);
    if (gap > sweepContactSlack) return L"EA";
    if (gap < -sweepContactSlack) return L"PT";
    return L"OK";
}

const wchar_t* kindName(ShapeKind kind) {
    switch (kind) {
        case ShapeKind::Sphere: return L"sphere";
        case ShapeKind::Capsule: return L"capsule";
        case ShapeKind::Box: return L"box";
    }
    return L"?";
}

double randomPick(std::mt19937_64& rng) { return (rng() >> 11) * 0x1p-53; }

}  // namespace

SweepInspector::SweepInspector(const Planet& planet, const SweepOptions& options, std::uint64_t seed)
    : planet_(planet), opts_(options), seed_(seed) {}

void SweepInspector::attachBackend(World& backend, Method method, bool morton, double density) {
    backend_ = &backend;
    method_ = method;
    morton_ = morton;
    density_ = density;
}

void SweepInspector::clear() {
    mesh_.reset();
    casts_.clear();
    tally_ = SweepTally{};
    orderByError_.clear();
    orderPos_ = 0;
    invokers_.clear();
    castIndex_ = 0;
    invokerCount_ = 0;
    focusValid_ = false;
    active_ = false;
}

void SweepInspector::begin(const std::vector<Vec>& invokers, Vec cameraEye) {
    active_ = true;
    invokers_ = invokers;
    invokerCount_ = invokers.size();
    ensureMesh(invokers, cameraEye);
    validateSweep(invokers);
    const int perFrame = int(invokerCount_ * opts_.shapesPerInvoker);
    if (opts_.frame >= 0) {
        const auto upper = std::max<std::size_t>(0, casts_.size() - 1);
        castIndex_ = std::clamp(static_cast<std::size_t>(opts_.frame) * std::size_t(perFrame), std::size_t(0), upper);
    } else if (!orderByError_.empty()) {
        // Open on the cast with the largest TOI error.
        castIndex_ = orderByError_.front();
    } else {
        castIndex_ = 0;
    }
    syncOrderPos();
    focusCurrent();
}

void SweepInspector::ensureMesh(const std::vector<Vec>& invokers, Vec eye) {
    if (method_ == Method::Mesh) {
        mesh_.reset();
        return;
    }
    if (mesh_) return;
    mesh_ = std::make_unique<World>(planet_, Method::Mesh, morton_, density_);
    mesh_->update(invokers, true, eye);
}

Shape SweepInspector::castShape(Vec anchor, int index, std::mt19937_64& rng) const {
    const Vec up = unit(anchor);
    const auto axes = frameAt(up);
    const double azimuth = 2 * pi * randomPick(rng);
    const double tilt = 0.45 * (randomPick(rng) - 0.5);
    const Vec tilted = unit(up + (axes[0] * std::cos(azimuth) + axes[1] * std::sin(azimuth)) * tilt);
    Shape shape = sampleShape(index % 3 + 1, tilted);
    shape.size = shape.size * (0.7 + 0.8 * randomPick(rng));
    return shape;
}

std::pair<Vec, Vec> SweepInspector::castPlacement(Vec anchor, const Shape& shape, std::mt19937_64& rng) const {
    const Vec up = unit(anchor);
    const auto axes = frameAt(up);
    const Vec dir = unit(up * planetRadius + axes[0] * (1100 * (randomPick(rng) - 0.5))
                                            + axes[1] * (1100 * (randomPick(rng) - 0.5)));
    const Vec surface = planet_.surface(dir);
    const double support = dot(abs(dir), shape.extents());
    return {surface + dir * (support + 120.0), surface - dir * (support + 60.0)};
}

void SweepInspector::fillFrame(int frame, std::uint64_t salt, std::vector<SweepCast>& out) const {
    std::mt19937_64 rng(seed_ ^ (salt * std::uint64_t(frame + 1)));
    const int perInvoker = opts_.shapesPerInvoker;
    for (const Vec& anchor : invokers_) {
        for (int k = 0; k < perInvoker; ++k) {
            SweepCast cast;
            cast.shape = castShape(anchor, k, rng);
            const auto [from, to] = castPlacement(anchor, cast.shape, rng);
            cast.frame = frame;
            cast.index = int(out.size()) % std::max(1, int(invokerCount_ * perInvoker));
            cast.from = from;
            cast.to = to;
            out.push_back(cast);
        }
    }
}

void SweepInspector::validateSweep(const std::vector<Vec>& invokers) {
    constexpr std::uint64_t measuredSalt = 0x51ed270b1a6c3f27ULL, warmupSalt = 0x2545f4914f6cdd1dULL;
    const int perFrame = int(invokers.size()) * opts_.shapesPerInvoker;
    tally_ = SweepTally{};
    casts_.clear();
    casts_.reserve(std::size_t(perFrame) * std::size_t(opts_.frames));
    for (int frame = 0; frame < opts_.frames; ++frame) fillFrame(frame, measuredSalt, casts_);

    std::vector<SweepCast> warm;
    warm.reserve(std::size_t(perFrame));
    for (int w = 0; w < opts_.warmup; ++w) {
        warm.clear();
        fillFrame(w, warmupSalt, warm);
        for (const SweepCast& cast : warm) {
            backend_->physics.sweep(cast.shape, cast.from, cast.to);
            if (mesh_) mesh_->physics.sweep(cast.shape, cast.from, cast.to);
        }
    }

    double bestError = -1.0;
    for (SweepCast& cast : casts_) {
        const Hit otherHit = backend_->physics.sweep(cast.shape, cast.from, cast.to);
        cast.hit = otherHit;
        cast.hasMesh = mesh_ != nullptr;
        if (!cast.hasMesh) continue;
        cast.meshHit = mesh_->physics.sweep(cast.shape, cast.from, cast.to);
        const Hit& meshHit = cast.meshHit;
        const double span = length(cast.to - cast.from);
        tally_.meshContacts += meshHit.state == Hit::State::Contact;
        tally_.backendContacts += otherHit.state == Hit::State::Contact;
        tally_.meshMiss += meshHit.state == Hit::State::Miss;
        tally_.backendMiss += otherHit.state == Hit::State::Miss;
        tally_.meshUnresolved += meshHit.state == Hit::State::Unresolved;
        tally_.backendUnresolved += otherHit.state == Hit::State::Unresolved;
        tally_.initial += meshHit.initial || otherHit.initial;
        if (meshHit.state != otherHit.state) {
            const bool refusal = meshHit.state == Hit::State::Unresolved || otherHit.state == Hit::State::Unresolved;
            tally_.refusals += refusal;
            tally_.conflicts += !refusal;
            tally_.mismatches.push_back(int(&cast - casts_.data()));
        }
        if (meshHit.state == Hit::State::Contact && otherHit.state == Hit::State::Contact) {
            const double path = std::abs(meshHit.fraction - otherHit.fraction) * span;
            cast.toiError = path;
            tally_.errors.push_back(path);
            if (tally_.worstError < 0 || path > bestError) {
                tally_.worstError = int(&cast - casts_.data());
                bestError = path;
            }
            const double point = length(otherHit.position - meshHit.position);
            ++tally_.paired;
            if (point > tally_.maxPoint) {
                tally_.maxPoint = point;
                tally_.worstPoint = int(&cast - casts_.data());
            }
            tally_.meanPath += path;
            tally_.meanPoint += point;
            const std::wstring tag = contactTag(otherHit.position - meshHit.position, meshHit.normal);
            if (tag == L"EA") ++tally_.early;
            else if (tag == L"PT") ++tally_.penetrating;
            else ++tally_.exact;
        }
    }

    std::sort(tally_.errors.begin(), tally_.errors.end());
    const std::size_t paired = std::max(1, tally_.paired);
    if (!tally_.errors.empty()) {
        tally_.meanError = std::accumulate(tally_.errors.begin(), tally_.errors.end(), 0.0) / double(paired);
        tally_.maxError = tally_.errors.back();
        tally_.medianError = tally_.errors[tally_.errors.size() / 2];
        const double rank = 0.95 * double(tally_.errors.size() - 1);
        const std::size_t lower = std::size_t(rank);
        const double weight = rank - double(lower);
        tally_.p95Error = lower + 1 < tally_.errors.size()
                              ? tally_.errors[lower] + (tally_.errors[lower + 1] - tally_.errors[lower]) * weight
                              : tally_.errors.back();
    }
    tally_.meanPath /= double(paired);
    tally_.meanPoint /= double(paired);

    orderByError_.resize(casts_.size());
    std::iota(orderByError_.begin(), orderByError_.end(), std::size_t(0));
    std::stable_sort(orderByError_.begin(), orderByError_.end(), [&](std::size_t a, std::size_t b) {
        return casts_[a].toiError > casts_[b].toiError;
    });
    orderPos_ = 0;
}

void SweepInspector::onTerrainPublished(bool force, const WorldStats& stats,
                                         const std::vector<Vec>& invokers, Vec cameraEye) {
    if (!mesh_ || !(force || stats.build.built != 0 || stats.build.removed != 0)) return;
    mesh_->update(invokers, true, cameraEye);
    if (active_) {
        resweepCurrent();
        focusCurrent();
    }
}

void SweepInspector::resweepCurrent() {
    if (casts_.empty()) return;
    SweepCast& cast = casts_[castIndex_];
    cast.hit = backend_->physics.sweep(cast.shape, cast.from, cast.to);
    cast.hasMesh = mesh_ != nullptr;
    if (cast.hasMesh) cast.meshHit = mesh_->physics.sweep(cast.shape, cast.from, cast.to);
}

void SweepInspector::focusCurrent() {
    if (casts_.empty()) {
        focusValid_ = false;
        return;
    }
    castIndex_ = std::clamp(castIndex_, std::size_t(0), casts_.size() - 1);
    const SweepCast& cast = casts_[castIndex_];
    focus_ = (cast.from + cast.to) * 0.5;
    focusValid_ = true;
    focusDistance_ = std::clamp(length(cast.to - cast.from) * 1.8, 260.0, 1000.0);
}

void SweepInspector::stepCast(int delta) {
    if (!active_ || orderByError_.empty()) return;
    const int n = int(orderByError_.size());
    int pos = int(orderPos_) + delta;
    pos = (pos % n + n) % n;
    orderPos_ = std::size_t(pos);
    castIndex_ = orderByError_[orderPos_];
    focusCurrent();
}

void SweepInspector::syncOrderPos() {
    orderPos_ = 0;
    for (std::size_t i = 0; i < orderByError_.size(); ++i) {
        if (orderByError_[i] == castIndex_) { orderPos_ = i; return; }
    }
}

void SweepInspector::nextCast() { stepCast(1); }
void SweepInspector::prevCast() { stepCast(-1); }

void SweepInspector::jumpFrame() {
    if (!active_ || casts_.empty()) return;
    const int perFrame = int(invokerCount_ * opts_.shapesPerInvoker);
    if (perFrame <= 0) return;
    castIndex_ = (castIndex_ / std::size_t(perFrame) + 1) * std::size_t(perFrame) % casts_.size();
    syncOrderPos();
    focusCurrent();
}

void SweepInspector::nextMismatch() {
    if (!active_ || casts_.empty()) return;
    for (const int index : tally_.mismatches) {
        if (std::size_t(index) > castIndex_) {
            castIndex_ = std::size_t(index);
            syncOrderPos();
            focusCurrent();
            return;
        }
    }
    if (!tally_.mismatches.empty()) {
        castIndex_ = std::size_t(tally_.mismatches.front());
        syncOrderPos();
        focusCurrent();
    }
}

void SweepInspector::nextWorstError() {
    if (tally_.worstError >= 0) {
        castIndex_ = std::size_t(tally_.worstError);
        syncOrderPos();
        focusCurrent();
    }
}

void SweepInspector::nextWorstPoint() {
    if (tally_.worstPoint >= 0) {
        castIndex_ = std::size_t(tally_.worstPoint);
        syncOrderPos();
        focusCurrent();
    }
}

void SweepInspector::revalidate() {
    if (!active_) return;
    validateSweep(invokers_);
    if (!orderByError_.empty()) castIndex_ = orderByError_.front();
    syncOrderPos();
    focusCurrent();
}

void SweepInspector::appendOverlay(std::vector<Vertex>& lines, const Camera& view) const {
    if (casts_.empty()) return;
    const SweepCast& cast = casts_[castIndex_];
    const auto meshTint = sweepColor(Vec{1.0, 0.10, 0.32}, 255);
    const auto backendTint = sweepColor(Vec{0.05, 1.0, 0.85}, 255);
    const Vec delta = cast.to - cast.from, down = unit(delta);
    const double span = length(delta);
    const auto path = sweepColor(Vec{0.60, 0.68, 0.80}, 255);

    lines.push_back(sweepVertex(cast.from, {0, 0, 1}, path));
    lines.push_back(sweepVertex(cast.to, {0, 0, 1}, path));
    const Vec head = cast.from + down * (0.10 * span);
    for (const Vec side : {unit(down + view.right * 0.22), unit(down - view.right * 0.22),
                           unit(down + view.up * 0.22), unit(down - view.up * 0.22)}) {
        lines.push_back(sweepVertex(head, {0, 0, 1}, path));
        lines.push_back(sweepVertex(head + side * (0.06 * span), {0, 0, 1}, path));
    }

    constexpr double labelHeight = 24.0;
    auto label = [&](const Hit& hit, std::uint32_t tint, double lift) {
        if (hit.state != Hit::State::Contact) return;
        appendMarker(lines, hit.position, view.right, view.up, 9.0, tint);
        std::ostringstream position;
        position << std::llround(hit.position.x) << ' ' << std::llround(hit.position.y) << ' '
                 << std::llround(hit.position.z);
        appendLabel(lines, position.str(), hit.position + view.up * lift, view.right, view.up, labelHeight, tint);
    };
    if (cast.hasMesh) label(cast.meshHit, meshTint, 1.5 * labelHeight);
    label(cast.hit, backendTint, -2.9 * labelHeight);
}

void SweepInspector::appendShapes(std::vector<Vertex>& objects) const {
    if (casts_.empty()) return;
    const SweepCast& cast = casts_[castIndex_];
    const auto meshTint = sweepColor(Vec{1.0, 0.10, 0.32}, 255);
    const auto backendTint = sweepColor(Vec{0.05, 1.0, 0.85}, 255);
    const auto startTint = sweepColor(Vec{0.34, 0.42, 0.52}, 255);
    auto centerOf = [&](const Hit& hit) {
        return hit.state == Hit::State::Contact ? cast.from + (cast.to - cast.from) * hit.fraction : cast.from;
    };
    auto draw = [&](const Shape& shape, Vec center, std::uint32_t material) {
        if (shape.kind == ShapeKind::Box) appendBoxShape(objects, shape, center, material);
        else appendRoundShape(objects, shape, center, material);
    };
    draw(cast.shape, cast.from, startTint);
    if (cast.hasMesh) draw(cast.shape, centerOf(cast.meshHit), meshTint);
    draw(cast.shape, centerOf(cast.hit), cast.hasMesh ? backendTint : meshTint);
}

std::wstring SweepInspector::statusLine(const std::array<const wchar_t*, 4>& methodLabels, std::size_t triangles) const {
    std::wostringstream info;
    info << std::fixed << methodLabels[std::size_t(method_)] << L" vs mesh   |   " << opts_.frames
         << L" frames x " << (invokerCount_ * std::size_t(opts_.shapesPerInvoker)) << L" = " << casts_.size()
         << L" casts   |   cast " << (casts_.empty() ? 0 : castIndex_ + 1) << L"/" << casts_.size();
    if (!casts_.empty()) {
        const SweepCast& cast = casts_[castIndex_];
        const int perFrame = int(invokerCount_ * opts_.shapesPerInvoker);
        info << L" (frame " << cast.frame << L", invoker " << cast.index / std::max(1, perFrame)
             << L", shape " << cast.index % std::max(1, perFrame) << L", " << kindName(cast.shape.kind) << L')';
        if (castIndex_ == std::size_t(tally_.worstError))
            info << L"   WORST toi " << std::setprecision(3) << tally_.maxError << L" m";
        else if (castIndex_ == std::size_t(tally_.worstPoint))
            info << L"   WORST point " << std::setprecision(3) << tally_.maxPoint << L" m";
        info << std::setprecision(0);
    }
    info << L"   |   " << triangles << L" triangles";
    return info.str();
}

std::wstring SweepInspector::castLine() const {
    if (casts_.empty()) return L"";
    const SweepCast& cast = casts_[castIndex_];
    const double span = length(cast.to - cast.from);
    std::wostringstream sweep;
    sweep << std::fixed;
    sweep << L"start (" << std::llround(cast.from.x) << L' ' << std::llround(cast.from.y) << L' '
          << std::llround(cast.from.z) << L") length " << std::setprecision(0) << span << L" m   |   ";
    if (cast.hasMesh) {
        const auto& hit = cast.meshHit;
        if (hit.state == Hit::State::Contact) {
            sweep << L"M (" << std::llround(hit.position.x) << L' ' << std::llround(hit.position.y) << L' '
                  << std::llround(hit.position.z) << L") f " << std::setprecision(3) << hit.fraction << L"   |   ";
        } else sweep << L"M miss   |   ";
    }
    const auto& hit = cast.hit;
    if (hit.state == Hit::State::Contact) {
        sweep << L"S (" << std::llround(hit.position.x) << L' ' << std::llround(hit.position.y) << L' '
              << std::llround(hit.position.z) << L") f " << std::setprecision(3) << hit.fraction << L"   |   ";
        if (cast.hasMesh && cast.meshHit.state == Hit::State::Contact) {
            sweep << L"toi " << std::abs(hit.fraction - cast.meshHit.fraction) * span << L"  point "
                  << length(hit.position - cast.meshHit.position) << L" m   |   "
                  << contactTag(hit.position - cast.meshHit.position, cast.meshHit.normal);
        }
    } else sweep << L"S miss";
    return sweep.str();
}

std::wstring SweepInspector::totalsLine() const {
    std::wostringstream totals;
    totals << std::fixed;
    if (mesh_) {
        totals << L"all " << casts_.size() << L" | contacts mesh/backend " << tally_.meshContacts << L'/'
               << tally_.backendContacts << L", conf " << tally_.conflicts << L", refuse " << tally_.refusals
               << L", init " << tally_.initial << L", paired " << tally_.paired << L" | toi max "
               << std::setprecision(3) << tally_.maxError << L" m | mean path/point " << tally_.meanPath << L'/'
               << tally_.meanPoint << L" m | EA " << tally_.early << L"  PT " << tally_.penetrating << L"  OK "
               << tally_.exact;
    } else {
        totals << L"all " << casts_.size() << L": the active backend is the mesh, nothing to compare";
    }
    return totals.str();
}

std::wstring SweepInspector::controlsHint() const {
    return L"Sweep: Left/Right cast | N next frame | C next conflict | E worst TOI | O worst point | V re-validate | X toggle sweep";
}

}  // namespace voxel
