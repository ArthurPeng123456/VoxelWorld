#include "world.hpp"
#include "free_camera.hpp"
#include "shadow_cascades.hpp"
#include "post_process.hpp"

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
    double voxelDensity{1};
    TerrainPreset terrain{TerrainPreset::Classic};
    bool morton{}, global{}, obj{}, selfTest{}, selfTestGlobal{}, help{}, capsuleStress{};
    fs::path output{"results"};
};

void help() {
    std::cout << "voxel_world [--scheme all|mesh|grid|dag|vdb] [--seed N] [--frames N]\n"
              << "  [--invokers N] [--queries N] [--bodies N] [--warmup N] [--stream-every N]\n"
              << "  [--morton] [--global] [--obj] [--out DIRECTORY] [--capsule-stress]\n"
              << "  [--self-test] [--self-test-global] (full-planet integration tests)\n"
              << "  [--voxel-density 0.125|0.25|0.5|1|2] (sampling-spacing multiplier; default 1 = 50m)\n"
              << "  [--terrain classic|mars|volcanic]\n"
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
        else if (arg == "--terrain") opts.terrain = parseTerrainPreset(next());
        else if (arg == "--frames") opts.frames = bounded(1, 100000);
        else if (arg == "--warmup") opts.warmup = bounded(0, 10000);
        else if (arg == "--invokers") opts.invokers = bounded(1, 80);
        else if (arg == "--queries") opts.queries = bounded(1, 10000);
        else if (arg == "--bodies") opts.bodies = bounded(0, 100000);
        else if (arg == "--stream-every") opts.streamEvery = bounded(1, 10000);
        else if (arg == "--out") opts.output = next();
        else if (arg == "--morton") opts.morton = true;
        else if (arg == "--global") opts.global = true;
        else if (arg == "--voxel-density" || arg == "--density") opts.voxelDensity = parseVoxelDensity(next());
        else if (arg == "--obj") opts.obj = true;
        else if (arg == "--self-test") opts.selfTest = true;
        else if (arg == "--self-test-global") opts.selfTestGlobal = true;
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
    out << "# voxel_density " << world.voxelDensity() << '\n';
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
        out << "o chunk_" << chunk.key.x << '_' << chunk.key.y << '_' << chunk.key.z << "_S" << chunk.key.level << "_LOD" << chunk.lod << '\n';
        for (const auto& triangle : chunk.mesh) {
            for (Vec p : triangle.vertices) out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
            out << "f " << index << ' ' << index + 1 << ' ' << index + 2 << '\n';
            index += 3;
        }
    }
}

void logBuild(std::ostream& out, Method method, int frame, const WorldStats& stats, const std::array<int, lodCount>& lodCounts,
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
    Planet planet(opts.seed, opts.terrain);
    World world(planet, method, opts.morton, opts.voxelDensity);
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

WorldStats waitForPublication(World& world) {
    // Correctness tests wait for completion. CTest supplies the process timeout.
    for (;;) {
        if (auto published = world.pollUpdate()) return *published;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void verifyDensitySwitch(World& world, const std::vector<Vec>& invokers, bool global, Vec point) {
    const Vec up = unit(point);
    const Key oldKey = keyAt(point), newKey = keyAt(point, voxelDensityShift(0.125));
    const auto body = world.physics.addBody(sampleShape(1, up), point + up * 100, up * -10);
    const Vec before = world.physics.bodyPosition(body);
    world.setVoxelDensity(0.125);
    check(world.requestUpdate(invokers, global, point), "runtime density switch starts a background build");
    check(world.voxelDensity() == 1 && world.chunks.count(oldKey) != 0 && world.chunks.count(newKey) == 0,
          "runtime density switch preserves the published field until activation");
    const auto published = waitForPublication(world);
    const auto fine = world.chunks.find(newKey);
    check(published.build.built > 0 && published.build.removed > 0 && world.voxelDensity() == 0.125 &&
          world.chunks.count(oldKey) == 0 && fine != world.chunks.end() &&
          fine->second->spacing == 6.25 && fine->second->field && !fine->second->mesh.empty(),
          "runtime density switch publishes the new field and meshes");
    check(world.physics.ray(point + up * 500, point - up * 500).state == Hit::State::Contact,
          "density replacement publishes queryable terrain");
    check(length(world.physics.bodyPosition(body) - before) == 0, "density replacement retains body transforms");
    world.physics.step(1.0 / 60, true);
    check(world.physics.bodySpeed(body) > 9, "density replacement retains body velocity");
}

void selfTestGlobalTerrain() {
    const Planet planet(42);
    const auto first = invokerPositions(planet, 1, 0);
    {
        Clipmap clipmap(-3);
        check(validLod0Radius(1) && validLod0Radius(512) && !validLod0Radius(0) && !validLod0Radius(500),
              "LOD0 radius must be a positive power of two");
        const int count = lodWindowCount(defaultLod0Radius);
        check(count == 15 && std::ldexp(double(defaultLod0Radius), count - 1) < fieldSideLength &&
              std::ldexp(double(defaultLod0Radius), count) >= fieldSideLength && lodWindowCount(1024) == count - 1,
              "LOD radii double until the field-wide fallback");
        for (double density : voxelDensities) {
            const auto cells = fieldCellsPerAxis(voxelDensityShift(density));
            check((cells & (cells - 1)) == 0 && cells * (chunkSize / fineCells * density) == fieldSideLength,
                  "every density uses a power-of-two cube covering the same spatial bounds");
        }
        std::size_t visits = 0;
        const auto box = bounds(keyAt(first[0], -3));
        Vec edge = box.center();
        edge.x = box.hi.x;
        const Vec before = edge - Vec{1, 0, 0};
        const Vec after = edge + Vec{1, 0, 0};
        clipmap.update({before}, true, visits, 1);
        check(clipmap.update({after}, true, visits, 1).empty() && visits == 0,
              "global anchor hysteresis retains its schedule across a nearby voxel boundary");
        check(clipmap.layout().at(keyAt(after, -3)) == 0 && clipmap.windows().front().box.contains(after),
              "finest LOD and its debug range cover the retained Invoker position");
        const Vec farther = edge + Vec{310, 0, 0};
        check(clipmap.layout().at(*leafAt(clipmap.layout(), farther)) != 0, "small LOD0 range leaves farther chunks coarse");
        check(!clipmap.update({after}, true, visits, defaultLod0Radius).empty() && visits > 0 &&
              clipmap.layout().at(keyAt(farther, -3)) == 0,
              "editing a range while stationary expands the actual fine field");
        check(clipmap.lod0Radius() == defaultLod0Radius && clipmap.windows().front().box.contains(farther),
              "debug ranges use the configured physical distance");

        // Cross an interior LOD0 anchor while staying inside the LOD1 anchor.
        Clipmap rolling(-3);
        const Vec start = edge + Vec{72, 0, 0};
        const Vec next = edge + Vec{80, 0, 0};
        const Vec other = planet.surface(unit(first[0] * -1));
        rolling.update({start, other}, true, visits);
        const auto initialWindows = rolling.windows();
        const auto compareFull = [&](bool checkCoarseTraversal = false) {
            Layout ideal, oracle;
            std::size_t fullVisits = 0;
            const auto windows = rolling.windows();
            updatePlanetLayout(ideal, oracle, windows, {}, -3, true, fullVisits);
            check(rolling.layout() == oracle, "incremental global LOD equals full rebuild including 2:1 balance");
            if (checkCoarseTraversal) {
                const auto half = splat(fieldSideLength / 2);
                std::size_t coarseVisits = 0;
                const auto changes = updatePlanetLayout(ideal, oracle, windows, {{{half * -1, half}, count - 1}},
                                                        -3, false, coarseVisits);
                check(changes.empty() && coarseVisits < fullVisits / 16,
                      "a coarse slab skips the finer subtrees even when it spans the field");
            }
            return fullVisits;
        };
        const auto changed = rolling.update({next, other}, true, visits);
        const auto movedWindows = rolling.windows();
        check(!changed.empty() && movedWindows.front().box.lo.x != initialWindows.front().box.lo.x,
              "crossing the fine anchor rolls LOD0");
        for (std::size_t i = 1; i < movedWindows.size(); ++i) {
            const auto a = movedWindows[i].box, b = initialWindows[i].box;
            check(length(a.lo - b.lo) == 0 && length(a.hi - b.hi) == 0,
                  "rolling LOD0 leaves coarser levels and the second Invoker stationary");
        }
        check(visits < compareFull(true) / 2, "a fine window move visits only a small part of the tree");
        {
            const Vec jump = edge + Vec{730, 167, 43};
            Clipmap phased = rolling, settled = rolling;
            settled.update({jump, other}, true, visits);
            phased.update({jump, other}, true, visits, defaultLod0Radius, true);
            const auto firstBatch = phased.windows();
            check(firstBatch[0].box.contains(jump) && phased.layout().at(keyAt(jump, -3)) == 0,
                  "rolling update publishes the moved invoker's LOD0 first");
            for (std::size_t i = 0; i < firstBatch.size(); ++i) if (firstBatch[i].lod >= 3) {
                check(length(firstBatch[i].box.lo - movedWindows[i].box.lo) == 0 &&
                      length(firstBatch[i].box.hi - movedWindows[i].box.hi) == 0,
                      "the near batch does not roll outer windows");
            }
            for (int i = 0; i < 4 * count; ++i) {
                const auto previous = phased.windows();
                phased.update({jump, other}, true, visits, defaultLod0Radius, true);
                const auto current = phased.windows();
                int moved = 0;
                for (std::size_t w = 0; w < current.size(); ++w) {
                    const Vec delta = abs(current[w].box.center() - previous[w].box.center());
                    const double distance = std::max({delta.x, delta.y, delta.z});
                    if (!distance) continue;
                    ++moved;
                    const double stride = std::ldexp(gridChunkSize(-3), current[w].lod - 3);
                    check(distance <= stride && (delta.x > 0) + (delta.y > 0) + (delta.z > 0) == 1,
                          "a regular roll moves at most one chunk along one axis");
                }
                check(moved <= 1, "only one invoker window rolls in a batch");
                if (!moved) break;
            }
            check(phased.layout() == settled.layout(), "outer batches converge to the complete balanced layout");
        }
        for (Vec p : {edge + Vec{730, 150, 30}, edge + Vec{-300, 30, 50}, start}) {
            rolling.update({p, other}, true, visits);
            compareFull();
        }
        rolling.update({start}, true, visits, 256);
        compareFull();
    }
    for (double density : voxelDensities) {
        const int shift = voxelDensityShift(density);
        const Key key = keyAt(first[0], shift);
        const Box box = bounds(key);
        check(box.contains(first[0]) && box.hi.x - box.lo.x == chunkSize * density,
              "density scales chunk coordinates without scaling the planet");
        World scaled(planet, Method::Grid, false, density);
        scaled.update(first, true);
        check(scaled.voxelDensity() == density && scaled.chunks.at(key)->lod == 0 &&
              scaled.chunks.at(key)->spacing == 50 * density, "all five density levels produce the requested voxel spacing");
        double volume = 0;
        for (const auto& item : scaled.chunks) {
            check(item.first.densityShift == shift, "published field uses one coherent density");
            volume += std::ldexp(density * density * density, 3 * item.first.level);
        }
        check(volume == std::ldexp(8.0, 3 * rootLevel), "density settings preserve full planetary coverage");
        const Vec up = unit(first[0]);
        const auto ray = scaled.physics.ray(first[0] + up * 500, first[0] - up * 500);
        check(ray.state == Hit::State::Contact && std::abs(ray.fraction - 0.5) < 0.02,
              "density-aware field rays hit the local surface");
        const Vec opposite = first[0] * -1, away = unit(opposite);
        check(scaled.physics.ray(opposite + away * 100000, opposite - away * 100000).state == Hit::State::Contact,
              "density settings preserve distant hemisphere queries");
        if (density == 1) verifyDensitySwitch(scaled, first, true, first[0]);
    }

    // Global LOD must retain real field values and meshes away from every invoker.
    for (Method method : {Method::Grid, Method::Dag, Method::Vdb, Method::Mesh}) {
        World globe(planet, method, false);
        globe.update(first, true);
        for (int i = 0; i < 32; ++i) {
            const double z = -0.97 + 1.94 * i / 31;
            const double angle = i * 2.399963229728653;
            const Vec up{std::sqrt(1 - z * z) * std::cos(angle), std::sqrt(1 - z * z) * std::sin(angle), z};
            const Vec center = planet.surface(up);
            const auto ray = globe.physics.ray(center + up * 100000, center - up * 100000);
            check(ray.state == Hit::State::Contact && ray.fraction > 0.4 && ray.fraction < 0.6,
                  "global voxel/mesh ray hits distant terrain across all hemispheres");
        }
        if (method != Method::Mesh)
            check(globe.physics.ray({}, {planetRadius + 100000,0,0}).initial, "constant interior leaves preserve solid queries");
        if (method == Method::Grid) {
            int interfaces = 0;
            Layout layout;
            for (const auto& c : globe.chunks) layout.emplace(c.first, c.second->lod);
            for (const auto& item : globe.chunks) {
                const Chunk& fine = *item.second;
                if (fine.mesh.empty()) continue;
                for (Key key : intersectingLeaves(layout, fine.box.expanded(splat(1e-4)))) {
                    const Chunk& coarse = *globe.chunks.at(key);
                    if (coarse.lod <= fine.lod || coarse.mesh.empty()) continue;
                    for (int axis = 0; axis < 3; ++axis) {
                        if (std::abs(fine.box.hi[axis] - coarse.box.lo[axis]) > 1e-5 &&
                            std::abs(fine.box.lo[axis] - coarse.box.hi[axis]) > 1e-5) continue;
                        const Vec lo = max(fine.box.lo, coarse.box.lo), hi = min(fine.box.hi, coarse.box.hi);
                        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
                        if (hi[u] - lo[u] < 1e-4 || hi[v] - lo[v] < 1e-4) continue;
                        for (double a : {0.25,0.75}) for (double b : {0.25,0.75}) {
                            Vec p = lo; p[u] += (hi[u] - lo[u]) * a; p[v] += (hi[v] - lo[v]) * b;
                            const double x = fine.cellAt(p).value(p), y = coarse.cellAt(p).value(p);
                            check(std::abs(x - y) < coarse.spacing * 2e-6 + 0.001,
                                  "different-sized LOD leaves share a continuous boundary field");
                        }
                        ++interfaces;
                    }
                }
            }
            check(interfaces > 50, "global test exercises many multiscale boundaries");
            const Vec opposite = first[0] * -1;
            globe.setLod0Radius(256);
            check(globe.requestUpdate({}, true, opposite), "global camera teleport starts a background build");
            check(globe.lod0Radius() == defaultLod0Radius, "pending LOD range keeps the previous terrain active");
            const Vec oldUp = unit(first[0]);
            check(globe.physics.ray(first[0] + oldUp * 100000, first[0] - oldUp * 100000).state == Hit::State::Contact,
                  "old global coverage remains queryable while new LOD leaves are built");
            waitForPublication(globe);
            check(globe.chunks.at(keyAt(opposite))->lod == 0, "camera refines the opposite hemisphere");
            check(globe.lod0Radius() == 256 && globe.hasFinestLod(opposite), "LOD range publishes with its refined chunks");
            const auto prior = leafAt(layout, first[0]);
            check(prior.has_value(), "previous invoker was covered");
            const Vec up = unit(first[0]);
            check(globe.physics.ray(first[0] + up * 100000, first[0] - up * 100000).state == Hit::State::Contact,
                  "moving away coarsens terrain without removing global coverage");
        }
    }

    std::cout << "Global terrain self-tests passed.\n";
}

void selfTest() {
    {
        const Vec eye{123.125,-45.5,planetRadius + 200}, right{1,0,0}, up{0,0,1}, forward{0,1,0};
        const auto csm = buildShadowCascades(eye, right, up, forward, 0.3, 10000000, 1.6, {0.45,0.25,0.86});
        check(csm.enabled, "near-surface views enable cascaded shadows");
        double previous = 0.3;
        for (const auto& cascade : csm.cascades) {
            check(cascade.farDistance > previous && cascade.texelSize > 0 && cascade.depthBias > 0,
                  "shadow cascade splits and biases are well ordered");
            for (int i = 0; i < 8; ++i) {
                const double d = i & 4 ? cascade.farDistance : previous;
                const Vec p = forward * d + right * ((i & 1 ? 1 : -1) * d * 1.6 / 1.92)
                            + up * ((i & 2 ? 1 : -1) * d / 1.92);
                const Vec projected = cascade.project(p);
                check(std::abs(projected.x) <= 1.000001 && std::abs(projected.y) <= 1.000001 &&
                      projected.z >= 0 && projected.z <= 1, "CSM projections contain each camera frustum interval");
            }
            previous = cascade.farDistance;
        }
        const Vec translation{1e12,-1e12,1e12};
        const auto translated = buildShadowCascades((eye + translation) - translation, right, up, forward,
            0.3, 10000000, 1.6, {0.45,0.25,0.86});
        for (int i = 0; i < shadowCascadeCount; ++i)
            check(length(csm.cascades[i].project({100,200,30}) - translated.cascades[i].project({100,200,30})) < 1e-10,
                  "CSM fitting is invariant under common trillion-meter translations");
        check(!buildShadowCascades(eye, right, up, forward, 300000, 10000000, 1.6, {0.45,0.25,0.86}).enabled,
              "orbital views outside the shadow range disable local cascades");
        double lastTexel = csm.cascades.front().texelSize;
        for (int edge : {2048, 4096}) {
            RenderSettings settings;
            settings.shadowResolution = edge;
            const auto finer = buildShadowCascades(eye, right, up, forward, 0.3, 10000000, 1.6, {0.45,0.25,0.86}, settings);
            check(finer.cascades.front().texelSize < lastTexel * 0.51 &&
                  finer.cascades.back().farDistance == csm.cascades.back().farDistance,
                  "higher CSM resolution shrinks texels without changing cascade coverage");
            lastTexel = finer.cascades.front().texelSize;
        }
        RenderSettings settings;
        settings.shadowDistanceKm = 4;
        settings.shadowSplitBlend = 0;
        settings.shadowDepthBias = 0;
        const auto adjusted = buildShadowCascades(eye, right, up, forward, 0.3, 10000000, 1.6, {0.45,0.25,0.86}, settings);
        check(adjusted.cascades.back().farDistance == 4000 && adjusted.cascades.front().depthBias == 0 &&
              std::abs(adjusted.cascades.front().farDistance - 1000.225) < 1e-9,
              "CSM distance cap, split weight and depth bias reach the cascade fit");
        check(toneMap(0, 0) == 0 && toneMap(4, 0) > toneMap(1, 0) && toneMap(1, 1) > toneMap(1, 0) &&
              toneMap(1000, 0) <= 1, "HDR exposure and highlight compression are monotonic and bounded");
        check(!readRenderParameter(L"nan", hdrParameters.front()) && !readRenderParameter(L"11", hdrParameters.front()) &&
              readRenderParameter(L"-1.5", hdrParameters.front()) == -1.5 && !validShadowResolution(8192),
              "render arguments validate finite values, bounds and supported shadow sizes");
        const auto referenceAir = atmosphereDimensions(settings);
        settings.atmosphereHeightKm = 300;
        const auto thickerAir = atmosphereDimensions(settings);
        check(referenceAir.thickness == 100000 && referenceAir.rayleighScale == 8000 && referenceAir.mieScale == 1200 &&
              thickerAir.thickness == 300000 && thickerAir.rayleighScale == 24000 && thickerAir.mieScale == 3600,
              "atmosphere thickness stretches both density profiles in meters");
    }
    {
        FreeCamera camera;
        const Vec eye{planetRadius + 0.125, -planetRadius, 200};
        camera.reset(eye, {0, 1, 0}, {0, 0, 1});
        camera.move(1, 0, 0, 0.5, 20);
        check(length(camera.position() - (eye + Vec{0, 10, 0})) < 1e-8, "free camera moves forward at Earth coordinates");
        camera.reset(eye, {0, 1, 0}, {0, 0, 1});
        camera.move(1, 1, 1, 1, 100);
        check(std::abs(length(camera.position() - eye) - 100) < 1e-8, "diagonal camera movement keeps constant speed");
        const Vec before = camera.position();
        camera.move(0, 0, 0, 100, 100);
        check(length(camera.position() - before) == 0, "released camera keys stop movement");
        camera.reset({}, {0, 1, 0}, {0, 0, 1});
        camera.look(1.5707963267948966, 0);
        check(length(camera.forward() - Vec{1, 0, 0}) < 1e-10, "mouse look rotates toward the camera right");
        camera.look(10000, 10000);
        check(std::abs(dot(camera.forward(), camera.right())) < 1e-10 &&
              std::abs(dot(camera.forward(), camera.up())) < 1e-10 &&
              std::abs(length(camera.forward()) - 1) < 1e-10, "camera pitch limit preserves an orthonormal frame");
        FreeCamera subdivided;
        camera.reset({}, {0, 1, 0}, {0, 0, 1});
        subdivided.reset({}, {0, 1, 0}, {0, 0, 1});
        camera.move(1, -1, 0, 1, 123);
        for (int i = 0; i < 100; ++i) subdivided.move(1, -1, 0, 0.01, 123);
        check(length(camera.position() - subdivided.position()) < 1e-9, "camera movement is independent of frame rate");
    }
    static_assert(sizeof(Vec) == 3 * sizeof(double), "World vectors must remain double precision");
    check(planetRadius == 6371000.0 && chunkSize / fineCells == 50.0, "Earth radius retains 50 meter local sampling");
    const Planet planet(42);
    const auto checkSurfaceNormal = [](const Planet& field, Vec direction) {
        const auto axes = frameAt(direction);
        constexpr double delta = 0.2 / planetRadius;
        const Vec dx = field.surface(unit(direction + axes[0] * delta)) -
                       field.surface(unit(direction - axes[0] * delta));
        const Vec dy = field.surface(unit(direction + axes[1] * delta)) -
                       field.surface(unit(direction - axes[1] * delta));
        const auto sample = field.sampleSurface(direction);
        check(std::abs(sample.elevation - field.elevation(direction)) < 1e-9 &&
              dot(sample.normal, unit(cross(dx, dy))) > 0.99999,
              "procedural terrain normal agrees with finite differences of elevation");
    };
    checkSurfaceNormal(planet, unit({-0.4, 1, 0.3}));
    for (int i = 0; i < 3; ++i) {
        const Vec center = planet.invokerDirection(i, 0);
        const Vec tangent = frameAt(center)[0];
        for (double offset : {100.0, 400.0, 900.0})
            checkSurfaceNormal(planet, unit(center + tangent * (offset / planetRadius)));
    }
    for (const auto preset : {TerrainPreset::Mars, TerrainPreset::Volcanic}) {
        const Planet field(42, preset), repeat(42, preset), otherSeed(43, preset);
        const auto range = field.elevationRange();
        const Vec center = field.invokerDirection(0, 0);
        const auto tangent = frameAt(center)[0];
        check(parseTerrainPreset(name(preset)) == preset && field.elevation(center) == repeat.elevation(center) &&
              std::abs(field.elevation(center) - otherSeed.elevation(center)) > 1,
              "terrain presets and seeds reproduce the intended field");
        for (double meters : {85.0, 350.0, 540.0, 1230.0})
            checkSurfaceNormal(field, unit(center + tangent * (meters / planetRadius)));
        double maximumRelief = 0;
        for (int i = 0; i < 960; ++i) {
            const double z = (i + 0.5) / 480 - 1;
            const double r = std::sqrt(1 - z * z), azimuth = i * 2.399963229728653;
            const Vec direction{r * std::cos(azimuth), r * std::sin(azimuth), z};
            const double height = field.elevation(direction);
            check(height >= range.lower && height <= range.upper &&
                  intersectsPlanetShell(keyAt(field.surface(direction), -3), range),
                  "new terrain relief fits the shell used for field and clipmap selection");
            maximumRelief = std::max(maximumRelief, std::abs(height));
        }
        check(maximumRelief > 1000, "regional geology extends beyond the original shell");
        if (preset == TerrainPreset::Mars) {
            for (const auto* globe : {&field, &otherSeed}) {
                for (double z : {-0.88, -0.53, -0.17, 0.17, 0.53, 0.88}) {
                    for (int sector = 0; sector < 4; ++sector) {
                        const double angle = sector * 1.5707963267948966 + 0.47;
                        const double radius = std::sqrt(1 - z * z);
                        const Vec direction{radius * std::cos(angle), radius * std::sin(angle), z};
                        const auto local = frameAt(direction);
                        int craterSamples = 0, smallCraterSamples = 0;
                        for (int row = -7; row <= 7; ++row) for (int col = -7; col <= 7; ++col) {
                            const Vec step = local[0] * (181 * row + 29 * col) + local[1] * (47 * row + 171 * col);
                            const Vec d = unit(direction + step / planetRadius);
                            const double fine = globe->sampleSurface(d, 6.25, false).craterMaterial;
                            const double coarse = globe->sampleSurface(d, 130, false).craterMaterial;
                            craterSamples += fine > 0.13;
                            smallCraterSamples += fine - coarse > 0.08;
                        }
                        check(craterSamples >= 12 && smallCraterSamples >= 2,
                              "remote Mars sector has local and small impacts at z=" + std::to_string(z) +
                              ", sector=" + std::to_string(sector) + ": " + std::to_string(craterSamples) +
                              " crater samples and " + std::to_string(smallCraterSamples) + " small-crater samples");
                        checkSurfaceNormal(*globe, direction);
                    }
                }
            }
        }
        const Vec point = field.surface(center);
        for (double sign : {-1.0, 1.0}) {
            const Key key = keyAt(point + center * (800 * sign), -3);
            BuildStats stats;
            const auto chunk = buildChunk(field, {{key, 2}}, key, Method::Grid, false, stats);
            const float value = chunk->field->get(0, 0, 0);
            check(intersectsPlanetShell(key, range) && !chunk->surfaceBounds && chunk->mesh.empty() &&
                  value * sign > 2 * chunk->spacing && chunk->field->get(8, 8, 8) == value,
                  "blocks safely inside or outside the terrain use constant signed storage");
        }
        for (const auto method : {Method::Mesh, Method::Grid, Method::Dag, Method::Vdb}) {
            World world(field, method, false);
            world.update({point}, false);
            const Vec start = point + center * 400, end = point - center * 400;
            const auto ray = world.physics.ray(start, end);
            const auto capsule = world.physics.sweep(sampleShape(2, center), start, end);
            check(ray.state == Hit::State::Contact && capsule.state == Hit::State::Contact &&
                  std::abs(ray.fraction - 0.5) < 0.075 && capsule.fraction < ray.fraction,
                  "all backends cast against the selected terrain geometry");
        }
    }
    const auto first = invokerPositions(planet, 1, 0), both = invokerPositions(planet, 2, 0);
    for (const auto& text : {"", "0", "0.3", "nan", "inf", "0.5x", "2.0001"}) {
        bool rejected = false;
        try { parseVoxelDensity(std::string(text)); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "unsupported voxel density is rejected");
    }
    check(parseVoxelDensity(std::wstring(L"0.125")) == 0.125, "viewer accepts wide-character density arguments");
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
    check(!fresh.layout().empty() && fresh.layout().size() < 50000, "global field has bounded hierarchical coverage");
    double volume = 0;
    for (const auto& leaf : fresh.layout()) volume += std::ldexp(1.0, 3 * leaf.first.level);
    check(volume == std::ldexp(8.0, 3 * rootLevel), "octree leaves cover the entire field volume");
    for (Vec p : {Vec{}, Vec{planetRadius,0,0}, Vec{-planetRadius,0,0}, Vec{0,planetRadius,0},
                  Vec{0,-planetRadius,0}, Vec{0,0,planetRadius}, Vec{0,0,-planetRadius}})
        check(leafAt(fresh.layout(), p).has_value(), "global field covers core and all six hemispheres without invokers");
    fresh.update(both, true, visits);
    check(fresh.layout().at(keyAt(both[0])) == 0 && fresh.layout().at(keyAt(both[1])) == 0,
          "multiple invokers refine a common global field");
    for (const auto& leaf : fresh.layout()) {
        for (Key neighbor : intersectingLeaves(fresh.layout(), bounds(leaf.first).expanded(splat(1e-4))))
            check(std::abs(leaf.second - fresh.layout().at(neighbor)) <= 1, "global field has 2:1 balanced sample spacings");
    }
    check(fresh.update(both, true, visits).empty() && visits == 0, "stationary global invokers do not rebuild the octree");
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
            bool awaitingUpload = false;
            while (!awaitingUpload) {
                check(!world.pollUpdate([&] { awaitingUpload = true; return false; }),
                      "the prepared field is held until its render pages are uploaded");
                if (!awaitingUpload) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            check(!world.requestUpdate({}, false, oldPoint) && world.chunks.count(keyAt(newPoint)) == 0 &&
                  world.clipmapWindows()[0].box.contains(oldPoint) && probe(oldPoint).first.state == Hit::State::Contact,
                  "waiting for uploads retains the old collision field and clipmap windows");
            after = waitForPublication(world);
        } else after = world.update({}, false, newPoint);
        check(after.build.removed > 0 && after.build.built > 0 && world.chunks.count(keyAt(oldPoint)) == 0,
              "camera teleport publishes new chunks and removes expired chunks");
        const auto b = probe(newPoint), expired = probe(oldPoint);
        check(b.first.state == Hit::State::Contact && b.second.state == Hit::State::Contact &&
              expired.first.state == Hit::State::Miss && expired.second.state == Hit::State::Miss,
              "casts use only the published camera coverage after a teleport");
        if (method == Method::Grid) verifyDensitySwitch(world, {}, false, newPoint);
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

        // Exercise both signs and the Earth-scale Bullet transforms, not just a plane at the origin.
        for (Key key : {Key{3981, -2700, 3100}, Key{-3981, 2700, -3100}}) {
            Chunks distant;
            distant.emplace(key, fixture(method, option.second, false, key));
            Physics farPhysics(method, distant);
            farPhysics.addTerrain(*distant.at(key));
            const Vec base = bounds(key).lo;
            const auto farRay = farPhysics.ray(base + Vec{400.125, 400.0625, 900}, base + Vec{400.125, 400.0625, 760});
            check(farRay.state == Hit::State::Contact && std::abs(farRay.position.z - (base.z + 797)) < 0.001,
                  "Earth-scale ray preserves millimeter precision");
            const auto farShape = sampleShape(1, {0,0,1});
            const auto farSweep = farPhysics.sweep(farShape, base + Vec{400,400,900}, base + Vec{400,400,750});
            check(farSweep.state == Hit::State::Contact && std::abs(farSweep.fraction - 0.6) < 0.01,
                  "Earth-scale sphere sweep has analytic TOI");
            farPhysics.addBody(farShape, base + Vec{400,400,850});
            for (int step = 0; step < 240; ++step) farPhysics.step(1.0 / 60, false);
            check(std::abs(farPhysics.bodyPosition(0).z - (base.z + 810)) < 0.1,
                  "Earth-scale body settles without float coordinate quantization");
        }
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
        if (opts.selfTest || opts.selfTestGlobal) {
            if (opts.selfTest) selfTest();
            if (opts.selfTestGlobal) selfTestGlobalTerrain();
            return 0;
        }
        fs::create_directories(opts.output);
        auto settings = createFile(opts.output / "settings.txt");
        settings << "voxel_density=" << opts.voxelDensity << '\n'
                 << "finest_spacing_m=" << chunkSize / fineCells * opts.voxelDensity << '\n'
                 << "planet_radius_m=" << planetRadius << '\n'
                 << "terrain=" << name(opts.terrain) << '\n';
        std::cout << "Voxel sampling spacing " << opts.voxelDensity << "x (LOD 0: "
                  << chunkSize / fineCells * opts.voxelDensity << " meters)\n";
        auto frames = createFile(opts.output / "frames.csv");
        auto summary = createFile(opts.output / "summary.csv");
        auto streaming = createFile(opts.output / "streaming.csv");
        std::ofstream capsuleReport;
        if (opts.capsuleStress) {
            capsuleReport = createFile(opts.output / "capsule-cases.csv");
            capsuleReport << "scheme,scenario,queries,hits,initial,unresolved,p50_us,p95_us,p99_us,"
                             "vs_mesh_mismatch,paired_hits,vs_mesh_toi_p95_m\n";
        }
        frames << "scheme,frame,update_ms,select_ms,sample_ms,storage_ms,render_mesh_ms,register_remove_ms,built,removed,";
        for (int lod = 0; lod < voxel::lodCount; ++lod) frames << "lod" << lod << ',';
        frames << "field_bytes,physics_static_bytes,render_mesh_bytes,step_ms,terrain_contacts,"
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
