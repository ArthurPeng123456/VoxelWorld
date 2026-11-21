#include "world.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>

namespace voxel {
namespace {

namespace fs = std::filesystem;

struct Options {
    std::vector<Method> methods{Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
    std::uint64_t seed{42};
    int frames{90};
    int warmup{6};
    int invokers{2};
    int queries{12};
    int bodies{48};
    int streamEvery{6};
    bool morton{}, global{}, obj{}, selfTest{}, help{}, capsuleStress{};
    fs::path output{"results"};
};

void help() {
    std::cout << "voxel_world [--scheme all|mesh|grid|dag|vdb] [--seed N] [--frames N]\n"
              << "  [--invokers N] [--queries N] [--bodies N] [--warmup N] [--stream-every N]\n"
              << "  [--morton] [--global] [--obj] [--out DIRECTORY] [--capsule-stress] [--self-test]\n"
              << "Defaults: 90 frames, 6 warmup, 2 moving invokers plus a camera, 12 queries per type per frame,\n"
              << "48 bodies and a terrain update every 6 frames. Units are meters.\n";
}

Options options(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&] {
            if (++i == argc) throw std::invalid_argument("Missing value for " + arg);
            return std::string(argv[i]);
        };
        auto number = [&] {
            const auto value = next();
            std::size_t count = 0;
            if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
                throw std::invalid_argument("Expected a nonnegative integer for " + arg);
            }
            const auto result = std::stoull(value, &count);
            if (count != value.size()) throw std::invalid_argument("Invalid integer for " + arg);
            return result;
        };
        auto bounded = [&](int lo, int hi) {
            const auto value = number();
            if (value < std::uint64_t(lo) || value > std::uint64_t(hi)) {
                throw std::invalid_argument("Out of range for " + arg + " (" + std::to_string(lo) + ".." + std::to_string(hi) + ")");
            }
            return int(value);
        };
        if (arg == "--scheme") {
            const auto method = next();
            if (method == "all") opts.methods = {Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
            else if (method == "mesh" || method == "1") opts.methods = {Method::Mesh};
            else if (method == "grid" || method == "2") opts.methods = {Method::Grid};
            else if (method == "dag" || method == "3") opts.methods = {Method::Dag};
            else if (method == "vdb" || method == "4") opts.methods = {Method::Vdb};
            else throw std::invalid_argument("Invalid scheme: " + method);
        } else if (arg == "--seed") opts.seed = number();
        else if (arg == "--frames") opts.frames = bounded(1, 100000);
        else if (arg == "--warmup") opts.warmup = bounded(0, 10000);
        else if (arg == "--invokers") opts.invokers = bounded(1, 80);
        else if (arg == "--queries") opts.queries = bounded(1, 10000);
        else if (arg == "--bodies") opts.bodies = bounded(0, 100000);
        else if (arg == "--stream-every") opts.streamEvery = bounded(1, 10000);
        else if (arg == "--out") opts.output = next();
        else if (arg == "--morton") opts.morton = true;
        else if (arg == "--global") opts.global = true;
        else if (arg == "--obj") opts.obj = true;
        else if (arg == "--self-test") opts.selfTest = true;
        else if (arg == "--capsule-stress") opts.capsuleStress = true;
        else if (arg == "--help" || arg == "-h") opts.help = true;
        else throw std::invalid_argument("Unknown argument: " + arg);
    }
    if (std::int64_t(opts.frames) * opts.queries > 1'000'000) throw std::invalid_argument("frames * queries must be at most 1000000");
    return opts;
}

struct Query { int kind; Vec from, to; Shape shape; int scenario{-1}; };

constexpr const char* capsuleCases[] = {"tilted", "grazing", "cross_chunk", "clear_miss", "initial_overlap"};

std::vector<Query> queries(const Planet& planet, const std::vector<Vec>& invokers, int count, int frame,
                           std::uint64_t seed, bool capsuleStress) {
    std::mt19937_64 rng(seed ^ (0xb65944d0259327d3ULL * std::uint64_t(frame + 1)));
    auto random = [&] { return (rng() >> 11) * 0x1p-53; };
    std::vector<Query> list;
    list.reserve(std::size_t(count) * 4);
    for (int i = 0; i < count; ++i) {
        const Vec up = unit(invokers[std::size_t(i) % invokers.size()]);
        const auto axes = frameAt(up);
        const Vec direction = unit(up * planetRadius + axes[0] * (1100 * (random() - 0.5))
                                                    + axes[1] * (1100 * (random() - 0.5)));
        const Vec surface = planet.surface(direction);
        const Vec tangent = frameAt(direction)[0] * (20 + 125 * random());
        if (capsuleStress) {
            const Vec along = unit(tangent);
            const double tilt = (0.14 + 1.40 * random()) * (i % 2 == 0 ? 1 : -1);
            const Vec axis = unit(direction * std::cos(tilt) + along * std::sin(tilt));
            const Shape shape{ShapeKind::Capsule, {6 + 16 * random(), 0, 8 + 41 * random()}, frameAt(axis)};
            const double height = shape.size.x + std::abs(dot(axis, direction)) * shape.size.z;
            const int scenario = (i + frame) % int(std::size(capsuleCases));
            Vec from, to;
            switch (scenario) {
                case 0:
                    from = surface + direction * (height + 85) - tangent;
                    to = surface - direction * 65 + tangent;
                    break;
                case 1:
                    from = surface + direction * (height + 5.0) - along * 175;
                    to = surface + direction * (height - 1.0) + along * 180;
                    break;
                case 2:
                    // The path is longer than a chunk's diagonal.
                    from = surface + direction * 460 - along * 1550;
                    to = surface - direction * 115 + along * 1550;
                    break;
                case 3:
                    from = surface + direction * 240 - tangent;
                    to = surface + direction * 235 + tangent;
                    break;
                default:
                    from = surface + direction * (height * 0.40);
                    to = surface + direction * 150;
                    break;
            }
            list.push_back({2, from, to, shape, scenario});
            continue;
        }
        const double height = i % 5 == 0 ? 750 : 130;
        const Vec from = surface + direction * height;
        const Vec to = (i + frame) % 4 == 0 ? from + tangent * 3 : surface - direction * 85 + tangent;
        for (int kind = 0; kind < 4; ++kind) list.push_back({kind, from, to, sampleShape(kind, direction)});
    }
    return list;
}

void spawn(World& world, const Planet& planet, int count, const std::vector<Vec>& invokers) {
    for (int i = 0; i < count; ++i) {
        const auto group = std::size_t(i) / invokers.size();
        const auto invoker = invokers[std::size_t(i) % invokers.size()];
        const auto axes = frameAt(unit(invoker));
        const double angle = group * 2.39996322972865;
        const double radius = 78 * std::sqrt(double(group));
        const Vec up = unit(invoker + (axes[0] * std::cos(angle) + axes[1] * std::sin(angle)) * radius);
        const int kind = i % 3 + 1;
        const double support = kind == 1 ? 13 : kind == 2 ? 21 : 10;
        world.physics.addBody(sampleShape(kind, up), planet.surface(up) + up * (support + 4 + i % 9), up * -11);
    }
}

double percentile(std::vector<double>& sorted, double percent) {
    if (sorted.empty()) return 0;
    return sorted[std::max<std::size_t>(1, std::size_t(std::ceil(sorted.size() * percent))) - 1];
}

struct QueryMetrics {
    std::vector<double> durations;
    struct Outcome { Hit::State state; double pathPosition; bool initial; };
    std::vector<Outcome> results;
    int hits{}, unresolved{}, initial{};
};

struct Agreement {
    int mismatch{}, paired{};
    double p95Distance{};
};

Agreement agreement(const QueryMetrics& mesh, const QueryMetrics& other) {
    Agreement result;
    std::vector<double> errors;
    for (std::size_t j = 0; j < other.results.size(); ++j) {
        const auto a = mesh.results[j], b = other.results[j];
        if (b.state == Hit::State::Unresolved) continue;
        result.mismatch += a.state != b.state;
        if (a.state == Hit::State::Contact && b.state == Hit::State::Contact) {
            errors.push_back(std::abs(a.pathPosition - b.pathPosition));
        }
    }
    std::sort(errors.begin(), errors.end());
    result.paired = int(errors.size());
    result.p95Distance = percentile(errors, 0.95);
    return result;
}

std::ofstream createFile(const fs::path& path) {
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("Could not create " + path.string());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << std::fixed << std::setprecision(6);
    return stream;
}

void exportMesh(const fs::path& path, const World& world, const std::vector<Vec>& invokers, Vec camera) {
    auto out = createFile(path);
    out << "# VoxelWorld Marching Cubes, world coordinates in meters\n";
    for (const Vec p : invokers) out << "# invoker " << p.x << ' ' << p.y << ' ' << p.z << '\n';
    out << "# camera_invoker " << camera.x << ' ' << camera.y << ' ' << camera.z << '\n';
    for (const auto& window : world.clipmapWindows()) {
        const auto& b = window.box;
        out << "# clipmap " << window.invoker << ' ' << window.lod << ' '
            << b.lo.x << ' ' << b.lo.y << ' ' << b.lo.z << ' ' << b.hi.x << ' ' << b.hi.y << ' ' << b.hi.z << '\n';
    }
    std::size_t index = 1;
    for (const auto& entry : world.chunks) {
        const auto& chunk = *entry.second;
        if (chunk.mesh.empty()) continue;
        out << "o chunk_" << chunk.key.x << '_' << chunk.key.y << '_' << chunk.key.z << "_LOD" << chunk.lod << '\n';
        for (const auto& triangle : chunk.mesh) {
            for (Vec p : triangle.vertices) out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
            out << "f " << index << ' ' << index + 1 << ' ' << index + 2 << '\n';
            index += 3;
        }
    }
}

void logBuild(std::ostream& out, Method method, int frame, const WorldStats& stats, const std::array<int, 4>& lodCounts,
              const TerrainMemory& memory, double stepMs, int contacts, bool scheduled) {
    out << name(method) << ',' << frame << ',' << stats.updateMs << ',' << stats.selectMs << ','
        << stats.build.sampleMs << ',' << stats.build.storageMs << ',' << stats.build.meshMs << ','
        << stats.physicsLoadMs << ',' << stats.build.built << ',' << stats.build.removed;
    for (int count : lodCounts) out << ',' << count;
    out << ',' << memory.fields << ',' << memory.physics << ',' << memory.meshes << ',' << stepMs << ',' << contacts
        << ',' << scheduled << ',' << stats.windowVisits << ',' << stats.changedOwners << ',' << memory.metadata
        << ',' << memory.total() << ',' << stats.stagingBytes << '\n';
}

Vec replayCamera(Vec target, int frame) {
    const Vec up = unit(target);
    return target + up * 1450 + frameAt(up)[0] * (1100 + 850 * std::sin(0.06 * frame));
}

struct QueryResults {
    std::array<QueryMetrics, 4> types;
    std::array<QueryMetrics, std::size(capsuleCases)> capsules;
};

QueryResults run(const Options& opts, Method method, std::ostream& frames, std::ostream& summary, std::ostream& streaming,
                 const QueryResults* meshReference, const QueryResults* fieldReference, std::ostream* capsuleReport) {
    Planet planet(opts.seed);
    World world(planet, method, opts.morton);
    auto invokers = invokerPositions(planet, opts.invokers, 0);
    Vec camera = replayCamera(invokers.front(), 0);
    const WorldStats initial = world.update(invokers, opts.global, camera);
    auto memory = world.terrainMemory();
    auto lodCounts = world.lodCounts();
    logBuild(frames, method, -1, initial, lodCounts, memory, 0, 0, true);
    spawn(world, planet, opts.bodies, invokers);
    for (int i = 0; i < opts.warmup; ++i) {
        for (const Query& q : queries(planet, invokers, opts.queries, i, opts.seed, opts.capsuleStress)) {
            if (q.kind == 0) world.physics.ray(q.from, q.to);
            else world.physics.sweep(q.shape, q.from, q.to);
        }
        world.physics.step(1.0 / 60, true);
    }

    QueryResults metrics;
    std::vector<double> steps, updates, changedUpdates, stationaryUpdates, changedSelect;
    int totalBuilt = 0, totalRemoved = 0, maxContacts = 0;
    std::size_t peakCollision = memory.fields + memory.physics;
    std::size_t peakResident = memory.total(), peakOverlap = memory.total();
    std::size_t peakStaged = initial.stagingBytes, peakFields = memory.fields, peakMeshes = memory.meshes;
    std::size_t visited = 0, changedOwners = 0;
    for (int frame = 0; frame < opts.frames; ++frame) {
        WorldStats stats;
        const auto moving = invokerPositions(planet, opts.invokers, frame);
        const bool scheduled = frame > 0 && frame % opts.streamEvery == 0;
        if (scheduled) {
            camera = replayCamera(moving.front(), frame);
            stats = world.update(moving, opts.global, camera);
            if (stats.build.built || stats.build.removed) {
                changedUpdates.push_back(stats.updateMs);
                changedSelect.push_back(stats.selectMs);
            } else stationaryUpdates.push_back(stats.updateMs);
            updates.push_back(stats.updateMs);
            totalBuilt += stats.build.built;
            totalRemoved += stats.build.removed;
            visited += stats.windowVisits;
            changedOwners += stats.changedOwners;
            peakOverlap = std::max(peakOverlap, memory.total() + stats.stagingBytes);
            memory = world.terrainMemory();
            lodCounts = world.lodCounts();
            peakResident = std::max(peakResident, memory.total());
            peakOverlap = std::max(peakOverlap, memory.total());
            peakStaged = std::max(peakStaged, stats.stagingBytes);
            peakCollision = std::max(peakCollision, memory.fields + memory.physics);
            peakFields = std::max(peakFields, memory.fields);
            peakMeshes = std::max(peakMeshes, memory.meshes);
        }
        const auto workload = queries(planet, moving, opts.queries, frame, opts.seed, opts.capsuleStress);
        for (const auto& q : workload) {
            const auto timer = Clock::now();
            const Hit result = q.kind == 0 ? world.physics.ray(q.from, q.to) : world.physics.sweep(q.shape, q.from, q.to);
            const auto elapsed = milliseconds(timer) * 1000;
            const QueryMetrics::Outcome outcome{result.state, result.fraction * length(q.to - q.from), result.initial};
            auto& m = metrics.types[q.kind];
            m.durations.push_back(elapsed);
            m.results.push_back(outcome);
            m.hits += result.state == Hit::State::Contact;
            m.unresolved += result.state == Hit::State::Unresolved;
            m.initial += result.initial;
            if (q.scenario >= 0) {
                auto& c = metrics.capsules[q.scenario];
                c.durations.push_back(elapsed);
                c.results.push_back(outcome);
                c.hits += result.state == Hit::State::Contact;
                c.unresolved += result.state == Hit::State::Unresolved;
                c.initial += result.initial;
            }
        }
        const auto timer = Clock::now();
        world.physics.step(1.0 / 60, true);
        const auto elapsed = milliseconds(timer);
        const int contacts = world.physics.terrainContacts();
        maxContacts = std::max(maxContacts, contacts);
        steps.push_back(elapsed);
        logBuild(frames, method, frame, stats, lodCounts, memory, elapsed, contacts, scheduled);
    }
    if (opts.obj) {
        exportMesh(opts.output / (std::string(name(method)) + ".obj"), world,
                   invokerPositions(planet, opts.invokers, (opts.frames - 1) / opts.streamEvery * opts.streamEvery), camera);
    }

    int below = 0;
    for (int i = 0; i < opts.bodies; ++i) below += planet.sdf(world.physics.bodyPosition(i)) < -1;
    std::sort(steps.begin(), steps.end());
    std::sort(updates.begin(), updates.end());
    std::sort(changedUpdates.begin(), changedUpdates.end());
    std::sort(stationaryUpdates.begin(), stationaryUpdates.end());
    std::sort(changedSelect.begin(), changedSelect.end());
    streaming << name(method) << ',' << opts.seed << ',' << opts.invokers << ",1," << opts.global
              << ',' << initial.updateMs << ',' << updates.size() << ',' << changedUpdates.size()
              << ',' << percentile(changedUpdates, 0.50) << ',' << percentile(changedUpdates, 0.95)
              << ',' << percentile(changedUpdates, 1.0) << ',' << percentile(changedSelect, 0.95)
              << ',' << percentile(stationaryUpdates, 0.95) << ',' << totalBuilt << ',' << totalRemoved
              << ',' << visited << ',' << changedOwners << ',' << peakResident << ',' << peakOverlap << ',' << peakStaged
              << ',' << peakFields << ',' << peakMeshes << ',' << peakCollision << '\n';
    std::cout << name(method) << " initial " << initial.updateMs << " ms, step P95 " << percentile(steps, 0.95)
              << " ms, peak collision " << (double(peakCollision) / 1048576) << " MiB, max contacts "
              << maxContacts << ", below analytic surface " << below << '\n';
    constexpr const char* labels[] = {"ray", "sphere", "capsule", "box"};
    for (int i = 0; i < 4; ++i) {
        auto& m = metrics.types[i];
        if (m.durations.empty()) continue;
        std::sort(m.durations.begin(), m.durations.end());
        const Agreement comparison = meshReference ? agreement(meshReference->types[i], m) : Agreement{};
        summary << name(method) << ',' << labels[i] << ',' << (method == Method::Grid && opts.morton ? "morton" : "default")
                << ',' << opts.seed << ',' << opts.frames << ',' << opts.invokers << ',' << opts.bodies << ',' << opts.global
                << ',' << opts.warmup << ',' << opts.streamEvery << ',' << m.results.size() << ',' << m.hits << ','
                << m.initial << ',' << m.unresolved << ',';
        if (meshReference) summary << comparison.mismatch << ',' << comparison.paired << ',' << comparison.p95Distance;
        else summary << ",,";
        summary << ',' << percentile(m.durations, 0.50) << ',' << percentile(m.durations, 0.95) << ','
                << percentile(m.durations, 0.99) << ','
                << (m.durations.size() * 1e6 / std::accumulate(m.durations.begin(), m.durations.end(), 0.0))
                << ',' << initial.updateMs << ',' << percentile(updates, 0.95)
                << ',' << percentile(steps, 0.50) << ',' << percentile(steps, 0.95) << ',' << percentile(steps, 0.99)
                << ',' << peakCollision << ',' << totalBuilt << ',' << totalRemoved << ',' << maxContacts << ',' << below << ',';
        if (fieldReference) {
            const auto& reference = fieldReference->types[i].results;
            const auto different = std::inner_product(reference.begin(), reference.end(), m.results.begin(), 0,
                std::plus<int>{}, [](const QueryMetrics::Outcome& a, const QueryMetrics::Outcome& b) {
                    return a.state != b.state || a.initial != b.initial || std::abs(a.pathPosition - b.pathPosition) > 1e-8;
                });
            summary << different;
        }
        summary << '\n';
        std::cout << "  " << labels[i] << " P95 " << percentile(m.durations, 0.95) << " us, hits "
                  << m.hits << ", unresolved " << m.unresolved;
        if (meshReference) std::cout << ", mismatches " << comparison.mismatch << ", paired P95 error " << comparison.p95Distance << " m";
        std::cout << '\n';
    }
    if (capsuleReport) {
        for (std::size_t i = 0; i < metrics.capsules.size(); ++i) {
            auto& c = metrics.capsules[i];
            std::sort(c.durations.begin(), c.durations.end());
            *capsuleReport << name(method) << ',' << capsuleCases[i] << ',' << c.durations.size() << ',' << c.hits << ','
                           << c.initial << ',' << c.unresolved << ',' << percentile(c.durations, 0.50) << ','
                           << percentile(c.durations, 0.95) << ',' << percentile(c.durations, 0.99) << ',';
            if (meshReference) {
                const auto compared = agreement(meshReference->capsules[i], c);
                *capsuleReport << compared.mismatch << ',' << compared.paired << ',' << compared.p95Distance;
            } else *capsuleReport << ",,";
            *capsuleReport << '\n';
        }
    }
    return metrics;
}

void check(bool condition, const std::string& description) {
    if (!condition) throw std::runtime_error(std::string("Self-test failed: ") + description);
}

std::unique_ptr<Chunk> fixture(Method method, bool morton, bool saddle = false, Key key = {}) {
    const int n = saddle ? 1 : 32;
    auto chunk = std::make_unique<Chunk>(Chunk{key, 0, n, bounds(key), chunkSize / n, {}, {}, {}, {}, {}});
    std::vector<float> sample;
    for (int z = 0; z <= n; ++z) for (int y = 0; y <= n; ++y) {
        for (int x = 0; x <= n; ++x) {
            sample.push_back(saddle ? float((x - 0.3) * (y - 0.7)) : float(z * chunk->spacing - 797));
        }
    }
    chunk->field = makeStorage(method, n + 1, sample, morton, chunk->spacing, chunk->box.lo);
    meshChunk(*chunk);
    return chunk;
}

void selfTest() {
    const Planet planet(42);
    const auto first = invokerPositions(planet, 1, 0), both = invokerPositions(planet, 2, 0);
    Clipmap clipmap;
    std::size_t visits = 0;
    clipmap.update(first, false, visits);
    const auto one = clipmap.layout();
    check(visits == 27 + 125 + 343 + 729, "one invoker adds four clipmap windows");
    check(clipmap.update(first, false, visits).empty() && visits == 0, "stationary invoker visits no slabs");
    const Box home = bounds(keyAt(first[0]));
    Vec nearEdge = home.center();
    nearEdge.x = home.hi.x + 100;
    check(clipmap.update({nearEdge}, false, visits).empty() && visits == 0, "anchor hysteresis retains coverage near edge");
    nearEdge.x = home.hi.x + 250;
    clipmap.update({nearEdge}, false, visits);
    check(visits == 2 * (9 + 25 + 49 + 81), "a one-axis move touches only the entering and leaving slabs");
    const auto withOneMoved = clipmap.layout();
    check(clipmap.update({nearEdge, nearEdge}, false, visits).empty() && clipmap.layout() == withOneMoved,
          "overlapping invokers share world chunks");
    check(clipmap.update({nearEdge}, false, visits).empty() && clipmap.layout() == withOneMoved,
          "one invoker leaving does not evict shared coverage");
    clipmap.update(both, false, visits);
    auto fullWindowLayout = [&] {
        Layout expected;
        for (const auto& w : clipmap.windows()) {
            const Key lo = keyAt(w.box.lo), hi = keyAt(w.box.hi);
            for (int z = lo.z; z < hi.z; ++z) for (int y = lo.y; y < hi.y; ++y) for (int x = lo.x; x < hi.x; ++x) {
                const Key key{x, y, z};
                if (!intersectsPlanetShell(key)) continue;
                auto [it, inserted] = expected.try_emplace(key, w.lod);
                if (!inserted) it->second = std::min(it->second, w.lod);
            }
        }
        return expected;
    };
    for (int frame = 5; frame <= 165; frame += 5) {
        clipmap.update(invokerPositions(planet, 2, frame), false, visits);
        check(clipmap.layout() == fullWindowLayout(), "incremental clipmap equals full coverage for multiple moving invokers");
    }
    Clipmap fresh;
    fresh.update(both, false, visits);
    const auto two = fresh.layout();
    check(two.size() > one.size(), "two invokers load additional chunks");
    for (const auto& entry : one) check(two.at(entry.first) <= entry.second, "nearest invoker wins");
    check(two != clipmap.layout(), "moving invokers change LOD");
    fresh.update({}, true, visits);
    check(fresh.layout().size() > two.size() && fresh.layout().at(keyAt(first[0])) == 3, "global shell falls back to LOD 3");
    fresh.update({}, false, visits);
    check(fresh.layout().empty(), "unloaded clipmap releases all owners");
    check(std::abs(planet.sdf(planet.surface(both[0] / length(both[0])))) < 1e-4, "planet surface is zero");

    bool seamTested = false;
    for (const auto& entry : two) {
        for (int axis = 0; axis < 3 && !seamTested; ++axis) {
            Key neighbor = entry.first;
            ++neighbor[axis];
            const auto found = two.find(neighbor);
            if (found == two.end() || found->second == entry.second) continue;
            BuildStats stats;
            const auto a = buildChunk(planet, two, entry.first, Method::Grid, false, stats);
            const auto b = buildChunk(planet, two, neighbor, Method::Grid, false, stats);
            const int u = (axis + 1) % 3, v = (axis + 2) % 3;
            for (int i = 0; i <= 16; ++i) for (int j = 0; j <= 16; ++j) {
                Vec point = a->box.lo;
                point[axis] = a->box.hi[axis];
                point[u] += i * chunkSize / 16;
                point[v] += j * chunkSize / 16;
                check(std::abs(a->cellAt(point).value(point) - b->cellAt(point).value(point)) < 0.002,
                      "mixed LOD shares the field at a chunk face");
            }
            seamTested = true;
        }
        if (seamTested) break;
    }
    check(seamTested, "the test layout contains mixed LOD neighbors");

    const Vec oldPoint = first[0];
    const Vec newPoint = planet.surface(unit(oldPoint) * -1);
    for (Method method : {Method::Grid, Method::Dag, Method::Vdb}) {
        World world(planet, method, false);
        const auto probe = [&](Vec target) {
            const Vec up = unit(target);
            const Vec from = target + up * 500, to = target - up * 500;
            return std::make_pair(world.physics.ray(from, to), world.physics.sweep(sampleShape(2, up), from, to));
        };
        auto before = world.update({}, false, oldPoint);
        check(before.build.built > 0 && world.chunks.at(keyAt(oldPoint))->lod == 0, "the camera alone can load LOD 0");
        const auto a = probe(oldPoint);
        check(a.first.state == Hit::State::Contact && a.second.state == Hit::State::Contact, "camera footprint supports ray and capsule casts");
        WorldStats after;
        if (method == Method::Grid) {
            check(world.requestUpdate({}, false, newPoint), "viewer accepts an asynchronous camera move");
            check(!world.requestUpdate({}, false, oldPoint) && world.chunks.count(keyAt(oldPoint)) != 0 &&
                  world.chunks.count(keyAt(newPoint)) == 0 && world.clipmapWindows()[0].box.contains(oldPoint),
                  "queries and debug windows retain the published world while replacements are built");
            const auto deadline = Clock::now() + std::chrono::seconds(20);
            std::optional<WorldStats> result;
            while (!(result = world.pollUpdate()) && Clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            check(result.has_value(), "background Clipmap batch publishes");
            after = *result;
        } else after = world.update({}, false, newPoint);
        check(after.build.removed > 0 && after.build.built > 0 && world.chunks.count(keyAt(oldPoint)) == 0,
              "camera teleport publishes new chunks and removes expired chunks");
        const auto b = probe(newPoint), expired = probe(oldPoint);
        check(b.first.state == Hit::State::Contact && b.second.state == Hit::State::Contact &&
              expired.first.state == Hit::State::Miss && expired.second.state == Hit::State::Miss,
              "casts use only the published camera coverage after a teleport");
    }

    for (const auto& option : {std::make_pair(Method::Mesh, false), std::make_pair(Method::Grid, false),
                              std::make_pair(Method::Grid, true), std::make_pair(Method::Dag, false),
                              std::make_pair(Method::Vdb, false)}) {
        const auto method = option.first;
        std::cout << "Testing " << name(method) << (option.second ? " Morton" : "") << std::endl;
        constexpr int side = 17;
        std::vector<float> pattern;
        for (int i = 0; i < side * side * side; ++i) pattern.push_back(float((i * 17) % 4801 - 2400));
        const auto lookup = makeStorage(method, side, pattern, option.second, 1, {});
        for (int z = 0; z < side - 1; ++z) for (int y = 0; y < side - 1; ++y) {
            for (int x = 0; x < side - 1; ++x) {
                const auto corners = lookup->corners({x, y, z});
                for (int i = 0; i < 8; ++i) {
                    const int at = ((z + (i >> 2)) * side + y + ((i >> 1) & 1)) * side + x + (i & 1);
                    check(corners[i] == pattern[at], "bulk corner sampling matches across storage boundaries");
                }
            }
        }
        Chunks chunks;
        chunks.emplace(Key{}, fixture(method, option.second));
        const auto& chunk = *chunks.begin()->second;
        check(chunk.mesh.size() == 2 * 32 * 32, "Marching Cubes plane triangle count");
        for (int z = 0; z <= 32; ++z) {
            check(chunk.field->get(0, 32, z) == float(z * 50 - 797), "storage sample matches original");
        }
        Physics physics(method, chunks);
        physics.addTerrain(chunk);
        const Vec origin{400, 400, 900};
        const auto hit = physics.ray(origin, {400, 400, 760});
        check(hit.state == Hit::State::Contact && std::abs(hit.position.z - 797) < 0.02 && hit.normal.z > 0.99,
              "ray hits sampled plane with correct normal");
        check(physics.ray(origin, {700, 500, 900}).state == Hit::State::Miss, "ray misses above plane");
        if (method != Method::Mesh) {
            check(physics.ray({400, 400, 780}, origin).initial, "ray starts in solid field");
            Chunks saddle;
            saddle.emplace(Key{}, fixture(method, option.second, true));
            const auto root = fieldRay(saddle, {0, 0, 400}, {1600, 1600, 400});
            check(root.state == Hit::State::Contact && std::abs(root.position.x - 480) < 0.1,
                  "ray detects first of two crossings in a cell");
        }
        for (int type = 1; type <= 3; ++type) {
            const auto shape = sampleShape(type, {0, 0, 1});
            const auto sweep = physics.sweep(shape, origin, {400, 400, 750});
            const double support = shape.extents().z;
            check(sweep.state == Hit::State::Contact && std::abs(900 - sweep.fraction * 150 - (797 + support)) < 1.6,
                  "shape sweep hits sampled plane, shape=" + std::to_string(type) + ", state="
                  + std::to_string(int(sweep.state)) + ", fraction=" + std::to_string(sweep.fraction));
            check(physics.sweep(shape, origin, {500, 500, 900}).state == Hit::State::Miss, "shape sweep above plane misses");
            if (method != Method::Mesh) {
                check(physics.sweep(shape, {400, 400, 795}, origin).initial, "shape sweep starts overlapped");
            }
            physics.addBody(shape, {double(200 * type), 400, 850});
        }
        Shape rotated = sampleShape(3, {0, 0, 1});
        rotated.axes = frameAt(unit({0, 0.3, 1}));
        const auto tilted = physics.sweep(rotated, origin, {420, 400, 750});
        check(tilted.state == Hit::State::Contact &&
              std::abs(900 - tilted.fraction * 150 - (797 + rotated.extents().z)) < 1.6, "rotated box sweep");
        const Shape capsule{ShapeKind::Capsule, {9, 0, 29}, frameAt(unit({1, 0.5, 0.8}))};
        const auto capsuleHit = physics.sweep(capsule, origin, {460, 370, 750});
        const double contactHeight = 797 + capsule.extents().z;
        check(capsuleHit.state == Hit::State::Contact && std::abs(900 - capsuleHit.fraction * 150 - contactHeight) < 0.9,
              "tilted long capsule diagonal sweep has analytic TOI");
        check(physics.sweep(capsule, {350, 350, contactHeight + 1}, {560, 430, contactHeight + 1}).state == Hit::State::Miss,
              "tilted capsule near-grazing pass stays clear");
        if (method != Method::Mesh) {
            const Vec touching{400, 400, contactHeight};
            const auto grazing = physics.sweep(capsule, touching, {580, 400, touching.z});
            check(grazing.state == Hit::State::Contact && grazing.initial, "grazing capsule starts in contact");
            const Vec overlapping{400, 400, contactHeight - 2};
            check(physics.sweep(capsule, overlapping, overlapping).initial, "stationary tilted capsule starts overlapped");
            chunks.emplace(Key{1, 0, 0}, fixture(method, option.second, false, {1, 0, 0}));
            const auto seam = physics.sweep(capsule, {1575, 400, contactHeight + 25}, {1625, 400, contactHeight - 25});
            check(seam.state == Hit::State::Contact && std::abs(seam.fraction - 0.5) * length(Vec{50, 0, 50}) < 0.9,
                  "tilted capsule hits across a chunk seam");
            const auto diagonal = physics.ray({-6000, -3000, 1500}, {8000, 3000, 200});
            check(diagonal.state == Hit::State::Contact && std::abs(diagonal.position.z - 797) < 0.02,
                  "long diagonal ray hits through sparse chunks");
        }
        physics.addBody(sampleShape(3, {0, 0, 1}), {600, 400, 876});
        for (int i = 0; i < 480; ++i) physics.step(1.0 / 60, false);
        check(physics.terrainContacts() >= 3, "terrain manifolds exist");
        for (int type = 1; type <= 3; ++type) {
            const double z = physics.bodyPosition(std::size_t(type - 1)).z;
            check(std::abs(z - (797 + sampleShape(type, {0, 0, 1}).extents().z)) < 1, "body settles on sampled plane");
            check(physics.bodySpeed(std::size_t(type - 1)) < 0.5, "body comes to rest");
        }
        check(physics.bodyPosition(3).z > 826, "boxes stack");
        physics.removeTerrain({});
        chunks.clear();
        for (int i = 0; i < 40; ++i) physics.step(1.0 / 60, false);
        check(physics.bodyPosition(0).z < 809, "unloading terrain wakes a resting body");
    }
    std::cout << "All self-tests passed.\n";
}

}  // namespace
}  // namespace voxel

int main(int argc, char** argv) {
    using namespace voxel;
    try {
        const auto opts = options(argc, argv);
        if (opts.help) { help(); return 0; }
        if (opts.selfTest) { selfTest(); return 0; }
        fs::create_directories(opts.output);
        auto frames = createFile(opts.output / "frames.csv");
        auto summary = createFile(opts.output / "summary.csv");
        auto streaming = createFile(opts.output / "streaming.csv");
        std::ofstream capsuleReport;
        if (opts.capsuleStress) {
            capsuleReport = createFile(opts.output / "capsule-cases.csv");
            capsuleReport << "scheme,scenario,queries,hits,initial,unresolved,p50_us,p95_us,p99_us,"
                             "vs_mesh_mismatch,paired_hits,vs_mesh_toi_p95_m\n";
        }
        frames << "scheme,frame,update_ms,select_ms,sample_ms,storage_ms,render_mesh_ms,register_remove_ms,built,removed,"
                  "lod0,lod1,lod2,lod3,field_bytes,physics_static_bytes,render_mesh_bytes,step_ms,terrain_contacts,"
                  "scheduled,window_visits,changed_owners,metadata_bytes,resident_terrain_bytes,staged_terrain_bytes\n";
        streaming << "scheme,seed,actor_invokers,camera_invokers,global,initial_ms,scheduled_updates,changed_updates,"
                     "changed_p50_ms,changed_p95_ms,changed_max_ms,changed_select_p95_ms,unchanged_p95_ms,"
                     "built,removed,window_visits,changed_owners,peak_resident_bytes,estimated_peak_overlap_bytes,"
                     "peak_staged_bytes,peak_field_bytes,peak_mesh_bytes,peak_collision_bytes\n";
        summary << "scheme,query,layout,seed,frames,invokers,bodies,global,warmup,stream_every,queries,hits,initial,unresolved,"
                   "vs_mesh_mismatch,paired_hits,vs_mesh_toi_p95_m,p50_us,p95_us,p99_us,queries_per_second,initial_ms,update_p95_ms,step_p50_ms,"
                   "step_p95_ms,step_p99_ms,peak_collision_bytes,stream_built,stream_removed,max_terrain_contacts,body_centers_below_analytic,"
                   "vs_first_field_different\n";
        std::optional<QueryResults> reference, fieldReference;
        std::cout << std::fixed << std::setprecision(3);
        for (Method method : opts.methods) {
            auto result = run(opts, method, frames, summary, streaming, reference ? &*reference : nullptr,
                              fieldReference ? &*fieldReference : nullptr,
                              opts.capsuleStress ? &capsuleReport : nullptr);
            if (method == Method::Mesh) reference = std::move(result);
            else if (!fieldReference) fieldReference = std::move(result);
        }
        frames.close();
        summary.close();
        streaming.close();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
