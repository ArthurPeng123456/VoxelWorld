#include "world.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace voxel {
namespace {

namespace fs = std::filesystem;

// Which query the mesh/field comparison issues: a cast, a shape sweep, or a
// rigid body collision step that only pairs bodies with the terrain.
enum class QueryMode { Ray, Sweep, Collision, Trajectory };

struct Options {
    std::vector<Method> methods{Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
    std::uint64_t seed{42};
    int frames{90};
    int warmup{6};
    int invokers{2};
    int queries{12};
    int bodies{48};
    int streamEvery{6};
    bool morton{}, global{}, obj{}, selfTest{}, help{}, capsuleStress{}, rayCompare{};
    int raysPerInvoker{20};
    int shapesPerInvoker{21};
    int spawnEvery{10};
    bool bodyPairs{};
    int settle{240};
    int sweepBudget{48000};
    double sweepTolerance{0.75};
    int mismatchSamples{10};
    int castAtFrame{-1};
    int castAtIndex{0};
    fs::path sweepCsv{};
    fs::path sweepLast{};
    fs::path sweepOutliers{};
    int sweepOutlierLimit{64};
    fs::path castObj{};
    QueryMode mode{QueryMode::Ray};
    std::vector<Method> compare{Method::Grid};
    fs::path output{"results"};
};

void help() {
    std::cout << "voxel_world [--scheme all|mesh|grid|dag|vdb] [--seed N] [--frames N]\n"
              << "  [--invokers N] [--queries N] [--bodies N] [--warmup N] [--stream-every N]\n"
              << "  [--morton] [--global] [--obj] [--out DIRECTORY] [--capsule-stress] [--self-test]\n"
              << "  [--ray-compare] [--rays-per-invoker N] [--sweep] [--shapes-per-invoker N]\n"
              << "  [--collision] [--settle N] [--compare grid|vdb|dag|all] [--sweep-budget N]\n"
              << "  [--sweep-tolerance METERS] [--mismatch-samples N] [--cast-at FRAME,INDEX]\n"
              << "  [--sweep-csv PATH] [--sweep-last PATH] [--sweep-outliers PATH] [--sweep-outlier-limit N]\n"
              << "  [--cast-obj PATH]\n"
              << "  [--trajectory] [--spawn-every N] [--body-pairs]  one backend per run, chosen by --scheme\n"
              << "  --obj dumps the final frame: mesh.obj, <backend>.obj and bodies_<backend>.csv under --out\n"
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
        auto real = [&](double lo, double hi) {
            const auto text = next();
            double value = 0;
            std::size_t count = 0;
            try {
                value = std::stod(text, &count);
            } catch (const std::exception&) {
                throw std::invalid_argument("Expected a decimal number for " + arg);
            }
            if (count != text.size() || !(value >= lo && value <= hi)) {
                throw std::invalid_argument("Out of range for " + arg + " (" + std::to_string(lo) + ".." + std::to_string(hi) + ")");
            }
            return value;
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
        else if (arg == "--ray-compare") opts.rayCompare = true;
        else if (arg == "--rays-per-invoker") opts.raysPerInvoker = bounded(1, 10000);
        else if (arg == "--sweep") opts.mode = QueryMode::Sweep;
        else if (arg == "--collision") opts.mode = QueryMode::Collision;
        else if (arg == "--shapes-per-invoker") opts.shapesPerInvoker = bounded(1, 84);
        else if (arg == "--trajectory") opts.mode = QueryMode::Trajectory;
        else if (arg == "--spawn-every") opts.spawnEvery = bounded(1, 10000);
        else if (arg == "--body-pairs") opts.bodyPairs = true;
        else if (arg == "--settle") opts.settle = bounded(0, 10000);
        else if (arg == "--sweep-budget") opts.sweepBudget = bounded(1, 10000000);
        else if (arg == "--sweep-tolerance") opts.sweepTolerance = real(0.01, 10.0);
        else if (arg == "--mismatch-samples") opts.mismatchSamples = bounded(0, 100000);
        else if (arg == "--sweep-csv") opts.sweepCsv = next();
        else if (arg == "--sweep-last") opts.sweepLast = next();
        else if (arg == "--sweep-outliers") opts.sweepOutliers = next();
        else if (arg == "--sweep-outlier-limit") opts.sweepOutlierLimit = bounded(0, 100000);
        else if (arg == "--cast-obj") opts.castObj = next();
        else if (arg == "--cast-at") {
            const auto text = next();
            const auto comma = text.find(',');
            if (comma == std::string::npos) throw std::invalid_argument("Expected FRAME,INDEX for --cast-at");
            opts.castAtFrame = std::stoi(text.substr(0, comma));
            opts.castAtIndex = std::stoi(text.substr(comma + 1));
            if (opts.castAtFrame < 0 || opts.castAtIndex < 0)
                throw std::invalid_argument("--cast-at needs a nonnegative FRAME,INDEX");
        }
        else if (arg == "--compare") {
            const auto backend = next();
            if (backend == "all") opts.compare = {Method::Grid, Method::Dag, Method::Vdb};
            else if (backend == "grid") opts.compare = {Method::Grid};
            else if (backend == "dag") opts.compare = {Method::Dag};
            else if (backend == "vdb") opts.compare = {Method::Vdb};
            else throw std::invalid_argument("Invalid compare backend: " + backend);
        }
        else if (arg == "--help" || arg == "-h") opts.help = true;
        else throw std::invalid_argument("Unknown argument: " + arg);
    }
    //opts.rayCompare = true;
    //opts.invokers = 100;
    //opts.frames = 1;
    //opts.raysPerInvoker = 20;
    //opts.seed = 42;
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

namespace voxel {

// Side-by-side ray benchmark: the same ray is issued on the mesh backend and on
// one field backend back-to-back, in the same frame. No clock timing; VTune
// attributes the cost. Invokers stay at their frame-0 positions and each
// backend's clipmap is built exactly once, so no chunk is ever rebuilt or
// unloaded: every frame only adds more random rays around the same invokers.
// Bodies are not spawned because only ray detection is compared and rigid
// bodies never block a terrain ray.
void compareRayToMesh(const Options& opts, Method other) {
    Planet planet(opts.seed);
    World meshWorld(planet, Method::Mesh, false);
    World otherWorld(planet, other, false);

    const auto invokers = invokerPositions(planet, opts.invokers, 0.0);
    const WorldStats meshBuild = meshWorld.update(invokers, opts.global, std::nullopt);
    const WorldStats otherBuild = otherWorld.update(invokers, opts.global, std::nullopt);

    std::mt19937_64 rng(opts.seed ^ 0x9e3779b97f4a7c15ULL);
    auto random = [&] { return (rng() >> 11) * 0x1p-53; };

    // One external query around an invoker: a random surface target within
    // ~1100 m of it, crossed from 130 m above to 85 m below the surface.
    auto shot = [&](Vec anchor) {
        const Vec up = unit(anchor);
        const auto axes = frameAt(up);
        const Vec dir = unit(up * planetRadius + axes[0] * (1100 * (random() - 0.5))
                                                + axes[1] * (1100 * (random() - 0.5)));
        const Vec surface = planet.surface(dir);
        return std::make_pair(surface + dir * 130.0, surface - dir * 85.0);
    };

    // The ray list is materialised once, so both backends receive exactly the
    // same queries while each one is timed in its own pass. Interleaving the two
    // inside a single loop would let one backend evict the other's data.
    struct Ray { Vec from, to; };
    std::vector<Ray> rays;
    rays.reserve(std::size_t(opts.frames) * invokers.size() * std::size_t(opts.raysPerInvoker));
    for (int frame = 0; frame < opts.frames; ++frame)
        for (std::size_t i = 0; i < invokers.size(); ++i)
            for (int r = 0; r < opts.raysPerInvoker; ++r) {
                const auto [from, to] = shot(invokers[i]);
                rays.push_back({from, to});
            }

    // Warmup primes caches per backend over a prefix of the same list; nothing
    // here is measured (VTune samples regardless).
    const std::size_t warmupRays =
        std::min(rays.size(), std::size_t(opts.warmup) * invokers.size() * std::size_t(opts.raysPerInvoker));
    auto warmup = [&](World& world) {
        for (std::size_t r = 0; r < warmupRays; ++r) world.physics.ray(rays[r].from, rays[r].to);
    };
    warmup(meshWorld);
    warmup(otherWorld);

    // One timed block per backend. At ~1 us per ray the two steady_clock reads
    // around a single call would eat a few percent of the measurement itself,
    // so the stamp sits outside the loop and only the ray call is inside; the
    // per-ray figure is then total / rays.
    auto timePass = [&](World& world) {
        const auto timer = Clock::now();
        for (const Ray& ray : rays) world.physics.ray(ray.from, ray.to);
        return milliseconds(timer) * 1000;  // microseconds
    };

    // Correctness is collected in a separate untimed pass, so the bookkeeping
    // never lands inside the timed region while both backends still report
    // exact hit counts and per-ray states for the mismatch check.
    struct Outcome { int hits{}, unresolved{}; std::vector<Hit::State> states; };
    auto verify = [&](World& world) {
        Outcome outcome;
        outcome.states.reserve(rays.size());
        for (const Ray& ray : rays) {
            const Hit hit = world.physics.ray(ray.from, ray.to);
            outcome.hits += hit.state == Hit::State::Contact;
            outcome.unresolved += hit.state == Hit::State::Unresolved;
            outcome.states.push_back(hit.state);
        }
        return outcome;
    };

    const double meshMicro = timePass(meshWorld);
    const double fieldMicro = timePass(otherWorld);
    const Outcome meshOutcome = verify(meshWorld);
    const Outcome fieldOutcome = verify(otherWorld);

    int mismatches = 0;
    for (std::size_t i = 0; i < meshOutcome.states.size(); ++i)
        mismatches += meshOutcome.states[i] != fieldOutcome.states[i];

    const double queries = double(std::max<std::size_t>(1, rays.size()));
    auto report = [&](double micro, const char* label) {
        std::cout << std::fixed << std::setprecision(3) << "  " << label << ": total " << micro / 1000 << " ms, mean "
                  << micro / queries << " us, "
                  << (micro > 0 ? queries / (micro / 1e6) : 0) << " rays/s\n";
    };

    std::cout << std::fixed << std::setprecision(3) << name(other) << " vs mesh [ray]: " << opts.invokers
              << " static invokers x " << opts.raysPerInvoker << " rays x " << opts.frames << " frames = " << rays.size()
              << " rays per backend (identical list, " << warmupRays << " warmup rays)\n"
              << "  build mesh " << meshBuild.updateMs << " ms, " << name(other) << ' ' << otherBuild.updateMs << " ms\n";
    report(meshMicro, "mesh");
    report(fieldMicro, name(other));
    std::cout << "  ratio " << name(other) << "/mesh: " << (meshMicro > 0 ? fieldMicro / meshMicro : 0) << "x\n"
              << "  contacts mesh " << meshOutcome.hits << ", " << name(other) << ' ' << fieldOutcome.hits
              << ", state mismatches " << mismatches << ", unresolved " << meshOutcome.unresolved << '/'
              << fieldOutcome.unresolved << '\n';


}

// Side-by-side sweep benchmark (--sweep): every invoker gets
// --shapes-per-invoker shapes per frame (by default 21: seven spheres, seven
// capsules, seven boxes), each with a random tilt and scale, so tilted boxes and
// capsules are exercised and every kind sees its own support height. The cast
// list of a frame is regenerated from a frame-seeded generator, so both backends
// issue exactly the same shapes in the same order. Invokers stay at their frame-0
// positions and the clipmap is built once, so no chunk is rebuilt; each backend
// is timed in its own pass, one timed region per frame that contains nothing but
// the sweep calls (the cast generation sits outside the clock stamps).
// Geometry writer for the sweep outlier dump. One OBJ group per case holds the
// cast path, the shape at the start pose and at each backend's contact pose, plus
// the two reported contact points and the mesh normal, so a single row of
// --sweep-outliers can be loaded into a viewer instead of replayed blind.
namespace obj {

struct Mesh {
    std::ostream& out;
    int next{1};
    int vertex(Vec p) {
        out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
        return next++;
    }
    void tri(int a, int b, int c) { out << "f " << a << ' ' << b << ' ' << c << '\n'; }
    void line(int a, int b) { out << "l " << a << ' ' << b << '\n'; }
};

constexpr double pi = 3.141592653589793;

// Triangulated surface of the shape centred at `center`, rotated by its own axes.
void shape(Mesh& mesh, const Shape& shape, Vec center) {
    const auto place = [&](Vec local) { return center + shape.direction(local); };
    if (shape.kind == ShapeKind::Box) {
        int id[8];
        for (int i = 0; i < 8; ++i) {
            id[i] = mesh.vertex(place({(i & 1) ? shape.size.x : -shape.size.x,
                                       (i & 2) ? shape.size.y : -shape.size.y,
                                       (i & 4) ? shape.size.z : -shape.size.z}));
        }
        const int face[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                                 {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
        for (const auto& f : face) mesh.tri(id[f[0]], id[f[1]], id[f[2]]);
        return;
    }
    // Profile of revolution around axes[2]: (axial offset, radius). A sphere is a
    // half circle, a capsule adds the straight cylindrical segment between the caps.
    std::vector<std::pair<double, double>> profile;
    if (shape.kind == ShapeKind::Sphere) {
        for (int i = 0; i <= 12; ++i) {
            const double phi = -0.5 * pi + pi * i / 12.0;
            profile.emplace_back(shape.size.x * std::sin(phi), shape.size.x * std::cos(phi));
        }
    } else {
        for (int i = 0; i <= 6; ++i) {
            const double phi = -0.5 * pi + 0.5 * pi * i / 6.0;
            profile.emplace_back(-shape.size.z + shape.size.x * std::sin(phi), shape.size.x * std::cos(phi));
        }
        for (int i = 0; i <= 6; ++i) {
            const double phi = 0.5 * pi * i / 6.0;
            profile.emplace_back(shape.size.z + shape.size.x * std::sin(phi), shape.size.x * std::cos(phi));
        }
    }
    constexpr int slices = 24;
    std::vector<int> rings(profile.size() * slices);
    for (std::size_t i = 0; i < profile.size(); ++i) {
        for (int j = 0; j < slices; ++j) {
            const double angle = 2 * pi * j / slices;
            const Vec radial = shape.axes[0] * std::cos(angle) + shape.axes[1] * std::sin(angle);
            rings[i * slices + std::size_t(j)] =
                mesh.vertex(center + shape.axes[2] * profile[i].first + radial * profile[i].second);
        }
    }
    for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
        for (int j = 0; j < slices; ++j) {
            const int a = rings[i * slices + std::size_t(j)];
            const int b = rings[i * slices + std::size_t((j + 1) % slices)];
            const int c = rings[(i + 1) * slices + std::size_t((j + 1) % slices)];
            const int d = rings[(i + 1) * slices + std::size_t(j)];
            mesh.tri(a, b, c);
            mesh.tri(a, c, d);
        }
    }
}

// A small three axis cross marks a point, and an optional segment shows a normal.
void marker(Mesh& mesh, Vec point, double size, Vec normal = {}) {
    mesh.line(mesh.vertex(point - Vec{size, 0, 0}), mesh.vertex(point + Vec{size, 0, 0}));
    mesh.line(mesh.vertex(point - Vec{0, size, 0}), mesh.vertex(point + Vec{0, size, 0}));
    mesh.line(mesh.vertex(point - Vec{0, 0, size}), mesh.vertex(point + Vec{0, 0, size}));
    if (length(normal) > 1e-9) mesh.line(mesh.vertex(point), mesh.vertex(point + normal * (4 * size)));
}

}  // namespace obj

// Closest point on a triangle (Ericson, Real-Time Collision Detection 5.1.5). Used
// by the --cast-at face report to answer whether two reported contact points sit on
// the same terrain triangle, which is what makes a normal projected gap meaningful.
Vec closestOnTriangle(Vec p, Vec a, Vec b, Vec c) {
    const Vec ab = b - a, ac = c - a, ap = p - a;
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec bp = p - b;
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const Vec cp = p - c;
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double denom = 1.0 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

void compareSweepToMesh(const Options& opts, Method other) {
    Planet planet(opts.seed);
    World meshWorld(planet, Method::Mesh, false);
    World otherWorld(planet, other, false);

    const auto invokers = invokerPositions(planet, opts.invokers, 0.0);
    const WorldStats meshBuild = meshWorld.update(invokers, opts.global, std::nullopt);
    const WorldStats otherBuild = otherWorld.update(invokers, opts.global, std::nullopt);

    // Every cast below uses this budget, and the stop counters are cleared here
    // so the tally covers the measured and the validated pass only.
    setSweepBudget(opts.sweepBudget);
    setSweepTolerance(opts.sweepTolerance);
    resetSweepDiagnostics();

    // One shape of the invoker's three kinds: the up axis is tilted by up to
    // ~0.22 rad in a random azimuth and the size is scaled 0.7x..1.5x.
    auto shapeAt = [&](Vec anchor, int index, auto& random) {
        const Vec up = unit(anchor);
        const auto axes = frameAt(up);
        const double azimuth = 6.283185307179586 * random();
        const double tilt = 0.45 * (random() - 0.5);
        const Vec tilted = unit(up + (axes[0] * std::cos(azimuth) + axes[1] * std::sin(azimuth)) * tilt);
        Shape shape = sampleShape(index % 3 + 1, tilted);
        shape.size = shape.size * (0.7 + 0.8 * random());
        return shape;
    };

    // Sweep a random surface target near the invoker, starting clear above the
    // surface and ending well below it, so a contact exists exactly once.
    auto placement = [&](Vec anchor, const Shape& shape, auto& random) {
        const Vec up = unit(anchor);
        const auto axes = frameAt(up);
        const Vec dir = unit(up * planetRadius + axes[0] * (1100 * (random() - 0.5))
                                                + axes[1] * (1100 * (random() - 0.5)));
        const Vec surface = planet.surface(dir);
        const double support = dot(abs(dir), shape.extents());
        return std::make_pair(surface + dir * (support + 120.0), surface - dir * (support + 60.0));
    };

    struct Cast { Shape shape; Vec from, to; };

    // The casts of one frame are regenerated from a frame-seeded generator, so
    // every pass issues exactly the same shapes, placements and order without
    // having to keep a copy of the whole run in memory.
    auto fillFrame = [&](int frame, std::uint64_t salt, std::vector<Cast>& casts) {
        std::mt19937_64 rng(opts.seed ^ (salt * std::uint64_t(frame + 1)));
        auto random = [&] { return (rng() >> 11) * 0x1p-53; };
        casts.clear();
        for (std::size_t i = 0; i < invokers.size(); ++i) {
            for (int k = 0; k < opts.shapesPerInvoker; ++k) {
                const Shape shape = shapeAt(invokers[i], k, random);
                const auto [from, to] = placement(invokers[i], shape, random);
                casts.push_back({shape, from, to});
            }
        }
    };

    constexpr std::uint64_t measuredSalt = 0x51ed270b1a6c3f27ULL;
    constexpr std::uint64_t warmupSalt = 0x2545f4914f6cdd1dULL;
    const std::size_t perFrame = invokers.size() * std::size_t(opts.shapesPerInvoker);
    const double sweeps = double(std::max<std::size_t>(1, std::size_t(opts.frames) * perFrame));

    // Warmup primes caches per backend on its own set of casts; nothing here is
    // measured.
    std::vector<Cast> casts;
    for (int w = 0; w < opts.warmup; ++w) {
        fillFrame(w, warmupSalt, casts);
        for (const Cast& cast : casts) meshWorld.physics.sweep(cast.shape, cast.from, cast.to);
        for (const Cast& cast : casts) otherWorld.physics.sweep(cast.shape, cast.from, cast.to);
    }

    // One timed region per frame per backend: the cast generation sits outside
    // the stamps, so only the sweep calls are inside and the per-frame
    // microseconds are summed. The 2 x frames clock reads (20-40 ns each) are the
    // whole overhead -- about 0.002 us per sweep at the default sizes -- and
    // nothing has to be subtracted afterwards.
    auto timePass = [&](World& world) {
        std::vector<Cast> local;
        local.reserve(perFrame);
        double micro = 0.0;
        for (int frame = 0; frame < opts.frames; ++frame) {
            fillFrame(frame, measuredSalt, local);
            const auto timer = Clock::now();
            for (int i = 0; i < local.size(); ++i) {
                Cast& cast = local[i];
                world.physics.sweep(cast.shape, cast.from, cast.to);
            }
            micro += milliseconds(timer) * 1000;
        }
        return micro;  // microseconds
    };

    // Correctness is checked in a separate untimed pass, which issues both
    // backends on the same cast so it can also compare the times of impact. Two
    // kinds of disagreement are counted apart: a conflict is the two backends
    // answering differently about a contact (contact vs miss), a refusal is the
    // field search declining to decide at all (Unresolved).
    struct Sample { int frame, index; ShapeKind kind; Hit::State mesh, other; double meshFraction, otherFraction, span; };
    struct Verdict {
        int meshContacts{}, otherContacts{}, conflicts{}, refusals{}, initial{};
        int meshMiss{}, otherMiss{}, meshUnresolved{}, otherUnresolved{};
        std::vector<double> errors;
        std::vector<Sample> samples;
    };
    // --sweep-csv PATH streams every cast of the untimed validation pass, so the two
    // backends can be diffed row by row offline. Rows are results, not timings: the
    // reported timing block stays per-frame, this one is per-cast.
    auto csvState = [](Hit::State state) {
        switch (state) {
            case Hit::State::Contact: return "contact";
            case Hit::State::Miss: return "miss";
            case Hit::State::Unresolved: return "unresolved";
        }
        return "?";
    };
    auto csvKind = [](ShapeKind kind) {
        switch (kind) {
            case ShapeKind::Sphere: return "sphere";
            case ShapeKind::Capsule: return "capsule";
            case ShapeKind::Box: return "box";
        }
        return "?";
    };
    std::ofstream sweepCsv;
    std::size_t sweepCsvRows = 0;
    if (!opts.sweepCsv.empty()) {
        sweepCsv.open(opts.sweepCsv, std::ios::trunc);
        if (!sweepCsv) throw std::runtime_error("Cannot open --sweep-csv " + opts.sweepCsv.string());
        sweepCsv << std::fixed << std::setprecision(3)
                 << "frame,index,kind,span_m,mesh_state,mesh_toi_m," << name(other) << "_state," << name(other)
                 << "_toi_m,delta_m\n";
    }
    // --sweep-last PATH dumps the final frame cast by cast, in space rather than in
    // cast parameter: the shape centre each cast started from, and the contact point
    // both backends reported. `point_delta_m` is the plain distance between those two
    // points; `normal_gap_m` projects it onto the mesh normal, so a negative value
    // means the field's contact sits embedded inside the mesh surface (the direction
    // that lets a body sink in) and a positive value means it floats above it.
    std::ofstream sweepLast;
    std::vector<std::string> lastLines;
    std::vector<double> lastPointDelta, lastNormalGap;
    double lastPathDelta = 0;
    std::size_t lastRows = 0, lastNegative = 0, lastBeyond = 0;
    if (!opts.sweepLast.empty()) {
        sweepLast.open(opts.sweepLast, std::ios::trunc);
        if (!sweepLast) throw std::runtime_error("Cannot open --sweep-last " + opts.sweepLast.string());
        sweepLast << std::fixed << std::setprecision(3)
                  << "frame,index,invoker,shape,kind,span_m,center_x,center_y,center_z,"
                  << "mesh_state,mesh_toi_m,mesh_x,mesh_y,mesh_z,"
                  << name(other) << "_state," << name(other) << "_toi_m,"
                  << name(other) << "_x," << name(other) << "_y," << name(other) << "_z,"
                  << "path_delta_m,point_delta_m,normal_gap_m\n";
    }
    // --sweep-outliers PATH writes every cast whose two reported contact points are
    // further apart than the tolerance the sweep promises, over all validated frames.
    // The CSV carries the full cast (size, all three axes, from, to, both answers),
    // which is enough to replay it with --cast-at FRAME,INDEX; the OBJ carries the
    // geometry of the first --sweep-outlier-limit cases (0 = no limit).
    std::ofstream outlierCsv, outlierObj;
    obj::Mesh outlierMesh{outlierObj};
    std::vector<std::string> outlierLines;
    std::size_t outlierRows = 0, outlierObjects = 0, outlierShown = 0;
    if (!opts.sweepOutliers.empty()) {
        outlierCsv.open(opts.sweepOutliers.string() + ".csv", std::ios::trunc);
        if (!outlierCsv) throw std::runtime_error("Cannot open " + opts.sweepOutliers.string() + ".csv");
        outlierCsv << std::fixed << std::setprecision(3)
                   << "frame,index,invoker,shape,kind,size_x,size_y,size_z,"
                   << "a0x,a0y,a0z,a1x,a1y,a1z,a2x,a2y,a2z,"
                   << "from_x,from_y,from_z,to_x,to_y,to_z,span_m,"
                   << "mesh_toi_m," << name(other) << "_toi_m,"
                   << "mesh_x,mesh_y,mesh_z," << name(other) << "_x," << name(other) << "_y," << name(other) << "_z,"
                   << "mesh_nx,mesh_ny,mesh_nz,path_delta_m,point_delta_m,normal_gap_m\n";
        outlierObj.open(opts.sweepOutliers.string() + ".obj", std::ios::trunc);
        if (!outlierObj) throw std::runtime_error("Cannot open " + opts.sweepOutliers.string() + ".obj");
        outlierObj << std::fixed << std::setprecision(3)
                   << "# voxel_world sweep outliers; one group per case: cast path, shape at start,"
                   << " shape at each backend's contact, contact points and the mesh normal\n";
    }
    auto validate = [&]() {
        Verdict verdict;
        std::vector<Cast> local;
        local.reserve(perFrame);
        for (int frame = 0; frame < opts.frames; ++frame) {
            fillFrame(frame, measuredSalt, local);
            int index = 0;
            for (const Cast& cast : local) {
                const Hit meshHit = meshWorld.physics.sweep(cast.shape, cast.from, cast.to);
                const Hit otherHit = otherWorld.physics.sweep(cast.shape, cast.from, cast.to);
                const double span = length(cast.to - cast.from);
                verdict.meshContacts += meshHit.state == Hit::State::Contact;
                verdict.otherContacts += otherHit.state == Hit::State::Contact;
                verdict.meshMiss += meshHit.state == Hit::State::Miss;
                verdict.otherMiss += otherHit.state == Hit::State::Miss;
                verdict.meshUnresolved += meshHit.state == Hit::State::Unresolved;
                verdict.otherUnresolved += otherHit.state == Hit::State::Unresolved;
                verdict.initial += meshHit.initial || otherHit.initial;
                if (meshHit.state != otherHit.state) {
                    const bool refusal = meshHit.state == Hit::State::Unresolved || otherHit.state == Hit::State::Unresolved;
                    verdict.refusals += refusal;
                    verdict.conflicts += !refusal;
                    if (verdict.samples.size() < std::size_t(opts.mismatchSamples) || opts.mismatchSamples == 0)
                        verdict.samples.push_back({frame, index, cast.shape.kind, meshHit.state, otherHit.state,
                                                   meshHit.fraction, otherHit.fraction, length(cast.to - cast.from)});
                }
                if (meshHit.state == Hit::State::Contact && otherHit.state == Hit::State::Contact)
                    verdict.errors.push_back(std::abs(meshHit.fraction - otherHit.fraction) * span);
                // Space level disagreement, both dumps below use it: the path delta is
                // the distance along the cast, the point delta the distance between the
                // two reported contact points, and the gap that distance projected onto
                // the mesh normal (negative = the field contact sits inside the mesh).
                const bool paired = meshHit.state == Hit::State::Contact && otherHit.state == Hit::State::Contact;
                const double path = paired ? std::abs(meshHit.fraction - otherHit.fraction) * span : 0.0;
                const double point = paired ? length(otherHit.position - meshHit.position) : 0.0;
                const double gap = paired ? dot(otherHit.position - meshHit.position, meshHit.normal) : 0.0;
                if (sweepCsv) {
                    sweepCsv << frame << ',' << index << ',' << csvKind(cast.shape.kind) << ',' << span << ','
                             << csvState(meshHit.state) << ',' << meshHit.fraction * span << ','
                             << csvState(otherHit.state) << ',' << otherHit.fraction * span << ',';
                    if (meshHit.state == Hit::State::Contact && otherHit.state == Hit::State::Contact)
                        sweepCsv << std::abs(meshHit.fraction - otherHit.fraction) * span;
                    sweepCsv << '\n';
                    ++sweepCsvRows;
                }
                if (sweepLast && frame + 1 == opts.frames) {
                    const Vec center = cast.from;
                    if (paired) {
                        lastPointDelta.push_back(point);
                        lastNormalGap.push_back(gap);
                        lastPathDelta += path;
                        lastNegative += gap < 0;
                        lastBeyond += std::abs(gap) > sweepToleranceMeters();
                    }
                    sweepLast << frame << ',' << index << ',' << index / opts.shapesPerInvoker << ','
                              << index % opts.shapesPerInvoker << ',' << csvKind(cast.shape.kind) << ',' << span << ','
                              << center.x << ',' << center.y << ',' << center.z << ',' << csvState(meshHit.state)
                              << ',' << meshHit.fraction * span << ',' << meshHit.position.x << ','
                              << meshHit.position.y << ',' << meshHit.position.z << ',' << csvState(otherHit.state)
                              << ',' << otherHit.fraction * span << ',' << otherHit.position.x << ','
                              << otherHit.position.y << ',' << otherHit.position.z << ',' << path << ',' << point
                              << ',' << gap << '\n';
                    std::ostringstream line;
                    line << std::fixed << std::setprecision(3) << "    #" << index << " inv "
                         << index / opts.shapesPerInvoker << " shape " << index % opts.shapesPerInvoker << " "
                         << csvKind(cast.shape.kind) << " span " << span
                         << " | center " << center.x << '/' << center.y << '/' << center.z << " | mesh "
                         << csvState(meshHit.state) << ' ' << meshHit.fraction * span << " pt " << meshHit.position.x
                         << '/' << meshHit.position.y << '/' << meshHit.position.z << " | " << name(other) << ' '
                         << csvState(otherHit.state) << ' ' << otherHit.fraction * span << " pt "
                         << otherHit.position.x << '/' << otherHit.position.y << '/' << otherHit.position.z
                         << " | path " << path << " point " << point << " gap " << gap;
                    lastLines.push_back(line.str());
                    ++lastRows;
                }
                // Anything whose two contact points are further apart than the tolerance
                // goes out with its full cast, so it can be replayed alone with --cast-at
                // and inspected in the OBJ dump.
                if (paired && outlierCsv && point > sweepToleranceMeters()) {
                    const Shape& shape = cast.shape;
                    outlierCsv << frame << ',' << index << ',' << index / opts.shapesPerInvoker << ','
                               << index % opts.shapesPerInvoker << ',' << csvKind(shape.kind) << ',' << shape.size.x
                               << ',' << shape.size.y << ',' << shape.size.z;
                    for (const Vec& axis : shape.axes) outlierCsv << ',' << axis.x << ',' << axis.y << ',' << axis.z;
                    outlierCsv << ',' << cast.from.x << ',' << cast.from.y << ',' << cast.from.z << ',' << cast.to.x
                               << ',' << cast.to.y << ',' << cast.to.z << ',' << span << ',' << meshHit.fraction * span
                               << ',' << otherHit.fraction * span << ',' << meshHit.position.x << ','
                               << meshHit.position.y << ',' << meshHit.position.z << ',' << otherHit.position.x
                               << ',' << otherHit.position.y << ',' << otherHit.position.z << ',' << meshHit.normal.x
                               << ',' << meshHit.normal.y << ',' << meshHit.normal.z << ',' << path << ',' << point
                               << ',' << gap << '\n';
                    ++outlierRows;
                    if (outlierObj && (opts.sweepOutlierLimit == 0 || outlierObjects < std::size_t(opts.sweepOutlierLimit))) {
                        ++outlierObjects;
                        const Vec delta = cast.to - cast.from;
                        const Vec meshCenter = cast.from + delta * meshHit.fraction;
                        const Vec otherCenter = cast.from + delta * otherHit.fraction;
                        outlierObj << "o cast_" << frame << '_' << index << '\n';
                        const int a = outlierMesh.vertex(cast.from), b = outlierMesh.vertex(cast.to);
                        outlierMesh.line(a, b);
                        obj::shape(outlierMesh, shape, cast.from);
                        obj::shape(outlierMesh, shape, meshCenter);
                        obj::shape(outlierMesh, shape, otherCenter);
                        obj::marker(outlierMesh, meshHit.position, 1.0, meshHit.normal);
                        obj::marker(outlierMesh, otherHit.position, 1.0);
                    }
                    if (outlierShown < std::size_t(opts.mismatchSamples) || opts.mismatchSamples == 0) {
                        ++outlierShown;
                        std::ostringstream line;
                        line << std::fixed << std::setprecision(3) << "  outlier frame " << frame << ", index "
                             << index << " (invoker " << index / opts.shapesPerInvoker << ", shape "
                             << index % opts.shapesPerInvoker << "), " << csvKind(shape.kind) << " size "
                             << shape.size.x << '/' << shape.size.y << '/' << shape.size.z << ", axis "
                             << shape.axes[2].x << '/' << shape.axes[2].y << '/' << shape.axes[2].z << '\n'
                             << "    from " << cast.from.x << '/' << cast.from.y << '/' << cast.from.z << "  to "
                             << cast.to.x << '/' << cast.to.y << '/' << cast.to.z << "  span " << span << " m\n"
                             << "    mesh " << meshHit.fraction * span << " m pt " << meshHit.position.x << '/'
                             << meshHit.position.y << '/' << meshHit.position.z << " | " << name(other) << ' '
                             << otherHit.fraction * span << " m pt " << otherHit.position.x << '/'
                             << otherHit.position.y << '/' << otherHit.position.z << '\n'
                             << "    point " << point << " m, gap " << gap << " m | replay: --cast-at " << frame
                             << ',' << index;
                        outlierLines.push_back(line.str());
                    }
                }
                ++index;
            }
        }
        std::sort(verdict.errors.begin(), verdict.errors.end());
        return verdict;
    };

    // --cast-at FRAME,INDEX reproduces exactly one cast and explains both answers,
    // so a reported refusal can be inspected without a full run.
    const bool debugCast = opts.castAtFrame >= 0;
    auto kindName = [](ShapeKind kind) {
        switch (kind) {
            case ShapeKind::Sphere: return "sphere";
            case ShapeKind::Capsule: return "capsule";
            case ShapeKind::Box: return "box";
        }
        return "?";
    };
    if (debugCast) {
        std::vector<Cast> local;
        local.reserve(perFrame);
        fillFrame(opts.castAtFrame, measuredSalt, local);
        if (std::size_t(opts.castAtIndex) >= local.size()) {
            throw std::invalid_argument("--cast-at index out of range for frame " + std::to_string(opts.castAtFrame) +
                                        " (" + std::to_string(local.size()) + " casts)");
        }
        const Cast& cast = local[std::size_t(opts.castAtIndex)];
        const Hit meshHit = meshWorld.physics.sweep(cast.shape, cast.from, cast.to);
        beginSweepTrace();
        const Hit fieldHit = otherWorld.physics.sweep(cast.shape, cast.from, cast.to);
        const SweepTrace trace = endSweepTrace();
        const double span = length(cast.to - cast.from);
        std::cout << std::fixed << std::setprecision(3)
                  << "  cast " << opts.castAtFrame << ',' << opts.castAtIndex << " of " << local.size()
                  << " casts (invoker " << opts.castAtIndex / opts.shapesPerInvoker << ", shape "
                  << opts.castAtIndex % opts.shapesPerInvoker << "), " << kindName(cast.shape.kind)
                  << " half sizes " << cast.shape.size.x << '/' << cast.shape.size.y << '/' << cast.shape.size.z
                  << ", span " << span << " m\n"
                  << "    from " << cast.from.x << '/' << cast.from.y << '/' << cast.from.z << "  to " << cast.to.x << '/'
                  << cast.to.y << '/' << cast.to.z << '\n'
                  << "    mesh " << (meshHit.state == Hit::State::Contact ? "contact" : "no contact") << " at "
                  << meshHit.fraction * span << " m, point " << meshHit.position.x << '/' << meshHit.position.y << '/'
                  << meshHit.position.z << ", normal " << meshHit.normal.x << '/' << meshHit.normal.y << '/'
                  << meshHit.normal.z << '\n'
                  << "    field " << (fieldHit.state == Hit::State::Contact ? "contact"
                                       : fieldHit.state == Hit::State::Unresolved ? "unresolved" : "miss")
                  << " at " << fieldHit.fraction * span << " m (fraction " << fieldHit.fraction << ")\n"
                  << "    field trace: visits " << trace.visits << ", depth " << trace.depth << ", budget stops "
                  << trace.budgetStops << ", tolerance stops " << trace.toleranceStops << '\n';
        if (trace.provenFraction >= 0) {
            std::cout << "    field proved a contact at " << trace.provenFraction * span << " m (mesh "
                      << meshHit.fraction * span << " m, gap " << (meshHit.fraction - trace.provenFraction) * span
                      << " m), unresolved band starts " << trace.unknownMargin << " m before it (limit "
                      << 2 * sweepToleranceMeters() << " m)\n";
        }
        if (trace.toleranceStops > 0 || trace.budgetStops > 0) {
            const int stops = trace.budgetStops + trace.toleranceStops;
            std::cout << "    earliest give up (this is what Unresolved reports): level " << trace.earliestLevel
                      << ", interval " << trace.earliestFirst * span << " .. " << trace.earliestLast * span << " m ("
                      << (trace.earliestLast - trace.earliestFirst) * span << " m long)\n"
                      << "    last give up: level " << trace.latestLevel << ", interval " << trace.latestFirst * span
                      << " .. " << trace.latestLast * span << " m, travel " << trace.stopTravel << " m, cell "
                      << trace.stopDimension << " m, " << stops << " stops in total\n";
        }
        std::cout << "    verdict: " << (meshHit.state == fieldHit.state ? "agree" : "disagree") << '\n';
        // Which terrain triangle did each backend land on? The gap metric only means
        // something when both points sit on the same face, so report the nearest
        // triangle of each point: chunk, index, distance, plane normal and the signed
        // distance along it.
        struct Face {
            double distance;
            Key key;
            int lod;
            std::size_t index;
            Vec normal, closest;
            double spacing;
            int coarseNeighbor;
            Box box;
        };
        auto nearestFace = [&](Vec point) {
            Face best{std::numeric_limits<double>::infinity(), Key{0, 0, 0}, 0, 0, Vec{}, Vec{}, 0, 0, Box{}};
            for (const auto& entry : meshWorld.chunks) {
                const auto& chunk = *entry.second;
                if (chunk.mesh.empty() || !chunk.box.expanded(Vec{200, 200, 200}).contains(point)) continue;
                for (std::size_t i = 0; i < chunk.mesh.size(); ++i) {
                    const auto& triangle = chunk.mesh[i];
                    const Vec closest =
                        closestOnTriangle(point, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]);
                    const double distance = length(point - closest);
                    if (distance >= best.distance) continue;
                    const Vec normal =
                        unit(cross(triangle.vertices[1] - triangle.vertices[0], triangle.vertices[2] - triangle.vertices[0]));
                    const int coarse = *std::max_element(chunk.neighborhood.begin(), chunk.neighborhood.end());
                    best = {distance, chunk.key, chunk.lod, i, normal, closest, chunk.spacing, coarse, chunk.box};
                }
            }
            return best;
        };
        auto faceReport = [&](const char* label, Vec point, Vec reportedNormal) {
            const Face face = nearestFace(point);
            if (!std::isfinite(face.distance)) {
                std::cout << "    " << label << " face: no triangle within the chunk window\n";
                return face;
            }
            const double planeSigned = dot(point - face.closest, face.normal);
            double toFace = std::numeric_limits<double>::infinity();
            for (int axis = 0; axis < 3; ++axis) toFace = std::min(toFace, std::min(point[axis] - face.box.lo[axis], face.box.hi[axis] - point[axis]));
            std::cout << "    " << label << " face: chunk " << face.key.x << '/' << face.key.y << '/' << face.key.z
                      << " lod " << face.lod << " spacing " << face.spacing << " m, coarsest neighbor lod "
                      << face.coarseNeighbor << ", tri " << face.index << ", distance " << face.distance
                      << " m, " << toFace << " m inside its chunk, plane normal " << face.normal.x << '/'
                      << face.normal.y << '/' << face.normal.z << ", signed " << planeSigned << " m";
            if (length(reportedNormal) > 0)
                std::cout << ", dot with reported normal " << dot(face.normal, reportedNormal);
            std::cout << '\n';
            return face;
        };
        Face meshFace{}, fieldFace{};
        if (meshHit.state == Hit::State::Contact) meshFace = faceReport("mesh", meshHit.position, meshHit.normal);
        if (fieldHit.state == Hit::State::Contact) fieldFace = faceReport("field", fieldHit.position, fieldHit.normal);
        if (meshHit.state == Hit::State::Contact && fieldHit.state == Hit::State::Contact) {
            const bool same = meshFace.key == fieldFace.key && meshFace.index == fieldFace.index;
            std::cout << "    same triangle: " << (same ? "yes" : "no");
            if (!same) std::cout << " (field point is " << fieldFace.distance << " m from its own triangle)";
            std::cout << '\n';
        }
        // --cast-obj PATH writes this one cast as an OBJ: the path, the shape at the
        // start pose and at each backend's contact pose, plus the contact points.
        if (!opts.castObj.empty()) {
            std::ofstream file(opts.castObj, std::ios::trunc);
            if (!file) throw std::runtime_error("Cannot open --cast-obj " + opts.castObj.string());
            obj::Mesh mesh{file};
            file << std::fixed << std::setprecision(3) << "# cast " << opts.castAtFrame << ',' << opts.castAtIndex
                 << " " << kindName(cast.shape.kind) << " size " << cast.shape.size.x << ' ' << cast.shape.size.y
                 << ' ' << cast.shape.size.z << "\n# axes\n";
            for (const Vec& axis : cast.shape.axes) file << "# " << axis.x << ' ' << axis.y << ' ' << axis.z << '\n';
            file << "o cast_" << opts.castAtFrame << '_' << opts.castAtIndex << '\n';
            const Vec delta = cast.to - cast.from;
            const int a = mesh.vertex(cast.from), b = mesh.vertex(cast.to);
            mesh.line(a, b);
            obj::shape(mesh, cast.shape, cast.from);
            if (meshHit.state == Hit::State::Contact && meshHit.fraction > 0) {
                obj::shape(mesh, cast.shape, cast.from + delta * meshHit.fraction);
                obj::marker(mesh, meshHit.position, 1.0, meshHit.normal);
            }
            if (fieldHit.state == Hit::State::Contact && fieldHit.fraction > 0) {
                obj::shape(mesh, cast.shape, cast.from + delta * fieldHit.fraction);
                obj::marker(mesh, fieldHit.position, 1.0);
            }
            std::cout << "    cast obj " << opts.castObj.string() << '\n';
        }
        return;
    }

    const double meshMicro = timePass(meshWorld);
    const double fieldMicro = timePass(otherWorld);
    Verdict verdict = validate();
    const SweepDiagnostics stops = sweepDiagnostics();

    auto stateName = [](Hit::State state) {
        switch (state) {
            case Hit::State::Contact: return "contact";
            case Hit::State::Miss: return "miss";
            case Hit::State::Unresolved: return "unresolved";
        }
        return "?";
    };
    auto report = [&](double micro, const char* label) {
        std::cout << std::fixed << std::setprecision(3) << "  " << label << ": total " << micro / 1000 << " ms, mean "
                  << micro / sweeps << " us, " << (micro > 0 ? sweeps / (micro / 1e6) : 0) << " sweeps/s\n";
    };
    std::cout << std::fixed << std::setprecision(3) << name(other) << " vs mesh [sweep]: " << opts.invokers
              << " static invokers x " << opts.shapesPerInvoker << " shapes x " << opts.frames << " frames = "
              << std::size_t(opts.frames) * perFrame << " sweeps per backend (identical list)\n"
              << "  build mesh " << meshBuild.updateMs << " ms, " << name(other) << ' ' << otherBuild.updateMs << " ms\n";
    report(meshMicro, "mesh");
    report(fieldMicro, name(other));
    std::cout << "  ratio " << name(other) << "/mesh: " << (meshMicro > 0 ? fieldMicro / meshMicro : 0) << "x\n"
              << "  contacts mesh " << verdict.meshContacts << ", " << name(other) << ' ' << verdict.otherContacts
              << ", initial " << verdict.initial << ", conflicts " << verdict.conflicts
              << ", refusals " << verdict.refusals << '\n'
              << "  states mesh " << verdict.meshContacts << '/' << verdict.meshMiss << '/' << verdict.meshUnresolved
              << ", " << name(other) << ' ' << verdict.otherContacts << '/' << verdict.otherMiss << '/'
              << verdict.otherUnresolved << " (contact/miss/unresolved)\n"
              << "  sweep stops: budget " << stops.budgetStops << " (limit " << sweepBudget() << "), tolerance "
              << stops.toleranceStops << " (limit " << sweepToleranceMeters() << " m)\n";
    const int disagreements = verdict.conflicts + verdict.refusals;
    if (disagreements > 0) {
        const int printed = int(verdict.samples.size());
        std::cout << "  disagreeing casts " << disagreements;
        if (printed < disagreements)
            std::cout << " (first " << printed << " shown, raise --mismatch-samples to see more)";
        std::cout << '\n';
    }
    if (!verdict.errors.empty()) {
        std::cout << "  paired " << verdict.errors.size() << ", p95 TOI difference " << percentile(verdict.errors, 0.95)
                  << " m, max " << verdict.errors.back() << " m\n";
    }
    for (const Sample& sample : verdict.samples) {
        std::cout << "  mismatch: frame " << sample.frame << ", shape " << sample.index << " ("
                  << (sample.kind == ShapeKind::Sphere ? "sphere" : sample.kind == ShapeKind::Capsule ? "capsule" : "box")
                  << "), mesh " << stateName(sample.mesh) << " at " << sample.meshFraction * sample.span << " m, "
                  << name(other) << ' ' << stateName(sample.other) << " at " << sample.otherFraction * sample.span
                  << " m (span " << sample.span << " m)\n";
    }
    if (sweepCsv.is_open()) {
        sweepCsv.close();
        std::cout << "  sweep csv " << opts.sweepCsv.string() << ": " << sweepCsvRows
                  << " rows (one per cast, from the untimed validation pass)\n";
    }
    if (sweepLast.is_open()) {
        sweepLast.close();
        std::cout << "  last frame " << opts.frames - 1 << ": " << lastRows << " casts, "
                  << lastPointDelta.size() << " paired contacts\n";
        if (!lastPointDelta.empty()) {
            std::sort(lastPointDelta.begin(), lastPointDelta.end());
            std::sort(lastNormalGap.begin(), lastNormalGap.end());
            const double meanPoint = std::accumulate(lastPointDelta.begin(), lastPointDelta.end(), 0.0)
                                   / double(lastPointDelta.size());
            const double meanGap = std::accumulate(lastNormalGap.begin(), lastNormalGap.end(), 0.0)
                                 / double(lastNormalGap.size());
            std::cout << "    path delta: mean " << lastPathDelta / double(lastPointDelta.size()) << " m | point delta: mean "
                      << meanPoint << " m, p95 " << percentile(lastPointDelta, 0.95) << " m, max "
                      << lastPointDelta.back() << " m\n"
                      << "    normal gap: mean " << meanGap << " m, min " << lastNormalGap.front() << " m, max "
                      << lastNormalGap.back() << " m, embedded (gap < 0) " << lastNegative << ", |gap| > tolerance "
                      << sweepToleranceMeters() << " m: " << lastBeyond << '\n';
        }
        for (const std::string& line : lastLines) std::cout << line << '\n';
    }
    if (outlierCsv.is_open()) {
        outlierCsv.close();
        outlierObj.close();
        std::cout << "  sweep outliers " << opts.sweepOutliers.string() << ": " << outlierRows << " casts with point delta > "
                  << sweepToleranceMeters() << " m, " << outlierObjects << " of them in the obj (limit "
                  << opts.sweepOutlierLimit << ", 0 = all)" << '\n';
        if (outlierShown < outlierRows)
            std::cout << "  (first " << outlierShown << " shown, raise --mismatch-samples to see more)\n";
        for (const std::string& line : outlierLines) std::cout << line << '\n';
    }
}

// Side-by-side collision benchmark (--collision): the same shapes are dropped
// onto the terrain in the mesh world and in one field world, then both are
// stepped in lockstep, so every narrowphase and manifold build runs twice per
// frame. After --settle frames the mesh world's body states are captured once
// and re-applied to both worlds at the start of every measured frame, so the
// two backends always process identical inputs and no body drifts into a
// neighbour: the only collision pairs that can appear are body-terrain ones.
// Body-body contacts are counted and reported, so a broken premise is visible.
void compareCollisionToMesh(const Options& opts, Method other) {
    Planet planet(opts.seed);
    World meshWorld(planet, Method::Mesh, false);
    World otherWorld(planet, other, false);

    const auto invokers = invokerPositions(planet, opts.invokers, 0.0);
    const WorldStats meshBuild = meshWorld.update(invokers, opts.global, std::nullopt);
    const WorldStats otherBuild = otherWorld.update(invokers, opts.global, std::nullopt);

    std::mt19937_64 rng(opts.seed ^ 0x2545f4914f6cdd1dULL);
    auto random = [&] { return (rng() >> 11) * 0x1p-53; };

    struct Drop { Shape shape; Vec position; Vec velocity; };
    std::vector<Drop> drops;
    drops.reserve(invokers.size() * std::size_t(opts.shapesPerInvoker));
    for (std::size_t i = 0; i < invokers.size(); ++i) {
        const auto axes = frameAt(unit(invokers[i]));
        for (int k = 0; k < opts.shapesPerInvoker; ++k) {
            // 3 columns x 7 rows for the default 21 shapes, 160 m apart.
            const double column = double(k % 3) - 1.0;
            const double row = double(k / 3) - 3.0;
            const Vec up = unit(invokers[i] + axes[0] * (160.0 * column) + axes[1] * (160.0 * row));
            const auto basis = frameAt(up);
            const double azimuth = 6.283185307179586 * random();
            const double tilt = 0.45 * (random() - 0.5);
            const Vec tilted = unit(up + (basis[0] * std::cos(azimuth) + basis[1] * std::sin(azimuth)) * tilt);
            Shape shape = sampleShape(k % 3 + 1, tilted);
            shape.size = shape.size * (0.7 + 0.8 * random());
            const double support = dot(abs(up), shape.extents());
            const Vec position = planet.surface(up) + up * (support + 40.0 + 25.0 * random());
            drops.push_back({shape, position, up * -18.0});
        }
    }
    for (const Drop& drop : drops) {
        meshWorld.physics.addBody(drop.shape, drop.position, drop.velocity);
        otherWorld.physics.addBody(drop.shape, drop.position, drop.velocity);
    }

    constexpr double dt = 1.0 / 60;
    // Track when the first terrain contact appears, so an undersized --settle is
    // visible instead of silently measuring a free fall.
    int meshLanding = -1, otherLanding = -1;
    for (int i = 0; i < opts.settle; ++i) {
        meshWorld.physics.step(dt, true);
        otherWorld.physics.step(dt, true);
        if (meshLanding < 0 && meshWorld.physics.terrainContacts() > 0) meshLanding = i + 1;
        if (otherLanding < 0 && otherWorld.physics.terrainContacts() > 0) otherLanding = i + 1;
    }
    int meshMoving = 0, otherMoving = 0;
    double meshSpeed = 0, otherSpeed = 0;
    for (std::size_t i = 0; i < drops.size(); ++i) {
        const double a = meshWorld.physics.bodySpeed(i), b = otherWorld.physics.bodySpeed(i);
        meshMoving += a > 0.5;
        otherMoving += b > 0.5;
        meshSpeed += a;
        otherSpeed += b;
    }
    meshSpeed /= std::max<std::size_t>(1, drops.size());
    otherSpeed /= std::max<std::size_t>(1, drops.size());

    // Canonical state captured once after settling; every measured frame starts
    // from it in both worlds.
    std::vector<BodyState> canonical(drops.size());
    for (std::size_t i = 0; i < drops.size(); ++i) canonical[i] = meshWorld.physics.bodyState(i);

    int meshContacts = 0, otherContacts = 0, meshMax = 0, otherMax = 0, worstGap = 0;
    int meshBody = 0, otherBody = 0, bodyFrames = 0;
    for (int frame = 0; frame < opts.frames; ++frame) {
        for (std::size_t i = 0; i < drops.size(); ++i) {
            meshWorld.physics.setBodyState(i, canonical[i]);
            otherWorld.physics.setBodyState(i, canonical[i]);
        }
        meshWorld.physics.step(dt, true);
        otherWorld.physics.step(dt, true);
        const int a = meshWorld.physics.terrainContacts(), b = otherWorld.physics.terrainContacts();
        meshContacts += a;
        otherContacts += b;
        meshMax = std::max(meshMax, a);
        otherMax = std::max(otherMax, b);
        worstGap = std::max(worstGap, std::abs(a - b));
        const int c = meshWorld.physics.bodyContacts(), d = otherWorld.physics.bodyContacts();
        meshBody = std::max(meshBody, c);
        otherBody = std::max(otherBody, d);
        bodyFrames += c > 0 || d > 0;
    }

    int meshBelow = 0, otherBelow = 0;
    double meanDrift = 0, maxDrift = 0, closestGap = 0;
    std::vector<Vec> centers(drops.size()), halfExtents(drops.size());
    for (std::size_t i = 0; i < drops.size(); ++i) {
        const BodyPose meshPose = meshWorld.physics.bodyPose(i);
        const Vec b = otherWorld.physics.bodyPosition(i);
        meshBelow += planet.sdf(meshPose.position) < -1;
        otherBelow += planet.sdf(b) < -1;
        const double drift = length(meshPose.position - b);
        meanDrift += drift;
        maxDrift = std::max(maxDrift, drift);
        Shape posed = drops[i].shape;
        posed.axes = meshPose.axes;                     // current orientation
        centers[i] = meshPose.position;
        halfExtents[i] = posed.extents();               // half extents on world axes
    }
    meanDrift /= std::max<std::size_t>(1, drops.size());
    // The most separated world axis decides whether two AABBs can overlap, so a
    // positive gap for every pair means the only pairs Bullet produced were
    // body-terrain ones.
    bool firstPair = true;
    for (std::size_t i = 0; i < centers.size(); ++i) {
        for (std::size_t j = i + 1; j < centers.size(); ++j) {
            const Vec delta = abs(centers[i] - centers[j]);
            const Vec reach = halfExtents[i] + halfExtents[j];
            const double gap = std::max(std::max(delta.x - reach.x, delta.y - reach.y), delta.z - reach.z);
            closestGap = firstPair ? gap : std::min(closestGap, gap);
            firstPair = false;
        }
    }

    // Final frame snapshot: the terrain mesh of both worlds plus one row per
    // rigid body with its resting center, so the two backends can be diffed
    // offline instead of only through the aggregate contact counts above.
    if (opts.obj) {
        fs::create_directories(opts.output);
        const Vec camera = replayCamera(invokers.front(), 0);
        const fs::path meshPath = opts.output / "mesh.obj";
        const fs::path otherPath = opts.output / (std::string(name(other)) + ".obj");
        const fs::path bodyPath = opts.output / (std::string("bodies_") + name(other) + ".csv");
        exportMesh(meshPath, meshWorld, invokers, camera);
        exportMesh(otherPath, otherWorld, invokers, camera);
        auto bodies = createFile(bodyPath);
        bodies << "# kind is sphere, capsule or box; positions are the final centers of mass in meters\n";
        bodies << "id,kind,size_x,size_y,size_z,mesh_x,mesh_y,mesh_z,other_x,other_y,other_z,drift,mesh_speed,other_speed\n";
        for (std::size_t i = 0; i < drops.size(); ++i) {
            const Shape& shape = drops[i].shape;
            const char* kind = shape.kind == ShapeKind::Sphere ? "sphere"
                             : shape.kind == ShapeKind::Capsule ? "capsule" : "box";
            const Vec a = meshWorld.physics.bodyPose(i).position;
            const Vec b = otherWorld.physics.bodyPosition(i);
            bodies << i << ',' << kind << ',' << shape.size.x << ',' << shape.size.y << ',' << shape.size.z
                   << ',' << a.x << ',' << a.y << ',' << a.z << ',' << b.x << ',' << b.y << ',' << b.z
                   << ',' << length(a - b) << ',' << meshWorld.physics.bodySpeed(i) << ','
                   << otherWorld.physics.bodySpeed(i) << '\n';
        }
        std::cout << "  wrote " << meshPath.string() << ", " << otherPath.string() << ", " << bodyPath.string()
                  << '\n';
    }

    const double steps = std::max(1, opts.frames);
    std::cout << std::fixed << std::setprecision(3) << name(other) << " vs mesh [collision]: " << opts.invokers
              << " static invokers x " << opts.shapesPerInvoker << " shapes = " << drops.size() << " bodies, settle "
              << opts.settle << ", " << opts.frames << " measured steps\n"
              << "  build mesh " << meshBuild.updateMs << " ms, " << name(other) << ' ' << otherBuild.updateMs << " ms\n"
              << "  terrain contacts per step: mesh mean " << meshContacts / steps << " max " << meshMax << ", "
              << name(other) << " mean " << otherContacts / steps << " max " << otherMax << ", worst gap " << worstGap
              << '\n'
              << "  settle " << opts.settle << " steps (" << opts.settle * dt << " s): first terrain contact at step mesh "
              << meshLanding << ", " << name(other) << ' ' << otherLanding << '\n'
              << "  at measurement start: still moving (>0.5 m/s) mesh " << meshMoving << '/' << drops.size()
              << " (mean " << meshSpeed << " m/s), " << name(other) << ' ' << otherMoving << '/' << drops.size()
              << " (mean " << otherSpeed << " m/s); every frame restarts from that captured state\n"
              << "  body-body contacts (max per frame): mesh " << meshBody << ", " << name(other) << ' ' << otherBody
              << " over " << bodyFrames << " frames"
              << ((meshBody || otherBody) ? "  WARNING: a body pair was generated\n" : "  (pairs stayed body-terrain)\n")
              << "  bodies below analytic surface: mesh " << meshBelow << ", " << name(other) << ' ' << otherBelow
              << '\n'
              << "  body drift after " << opts.settle << '+' << opts.frames << " steps: mean " << meanDrift
              << " m, max " << maxDrift << " m\n"
              << "  closest body AABB gap " << closestGap << " m (positive means no body-body pair)\n";
}

// One backend per process: the trajectory, the spawn schedule, the shapes and
// the launch velocities are all prescribed here, so two runs only differ in the
// terrain representation. Nothing in this loop reads the physics state back, so
// the runs are independent and can be launched as separate processes.
void compareTrajectory(const Options& opts, Method method) {
    Planet planet(opts.seed);
    World world(planet, method, opts.morton);
    const auto start = invokerPositions(planet, opts.invokers, 0.0);
    const WorldStats initial = world.update(start, opts.global, std::nullopt);

    struct Spawned { Shape shape; Vec position; Vec velocity; int frame; int invoker; };
    std::vector<Spawned> spawned;
    std::mt19937_64 rng(opts.seed ^ 0x5f2b7cd19e4a83c7ULL);
    auto random = [&] { return (rng() >> 11) * 0x1p-53; };

    constexpr double dt = 1.0 / 60;
    double updateMs = 0;
    int updates = 0, built = 0, contactSum = 0, contactMax = 0, bodyPairMax = 0;

    for (int frame = 0; frame < opts.frames; ++frame) {
        const auto moving = invokerPositions(planet, opts.invokers, double(frame));
        if (frame > 0 && frame % opts.streamEvery == 0) {
            const WorldStats stats = world.update(moving, opts.global, std::nullopt);
            updateMs += stats.updateMs;
            built += stats.build.built;
            ++updates;
        }
        // Terrain first, then the batch, so the new bodies land on coverage
        // that already includes this frame's invoker positions.
        if (frame % opts.spawnEvery == 0) {
            for (std::size_t i = 0; i < moving.size(); ++i) {
                const auto axes = frameAt(unit(moving[i]));
                for (int k = 0; k < opts.shapesPerInvoker; ++k) {
                    // 3 columns x 7 rows, 160 m apart, around the invoker.
                    const double column = double(k % 3) - 1.0;
                    const double row = double(k / 3) - 3.0;
                    const Vec up = unit(moving[i] + axes[0] * (160.0 * column) + axes[1] * (160.0 * row));
                    const auto basis = frameAt(up);
                    const double azimuth = 6.283185307179586 * random();
                    const double tilt = 0.45 * (random() - 0.5);
                    const Vec tilted = unit(up + (basis[0] * std::cos(azimuth) + basis[1] * std::sin(azimuth)) * tilt);
                    Shape shape = sampleShape(k % 3 + 1, tilted);
                    shape.size = shape.size * (0.7 + 0.8 * random());
                    const double support = dot(abs(up), shape.extents());
                    const Vec position = planet.surface(up) + up * (support + 40.0 + 25.0 * random());
                    const Vec velocity = up * -18.0;
                    if (opts.bodyPairs) world.physics.addBody(shape, position, velocity);
                    else world.physics.addTerrainOnlyBody(shape, position, velocity);
                    spawned.push_back({shape, position, velocity, frame, int(i)});
                }
            }
        }
        world.physics.step(dt, true);
        const int contacts = world.physics.terrainContacts();
        contactSum += contacts;
        contactMax = std::max(contactMax, contacts);
        bodyPairMax = std::max(bodyPairMax, world.physics.bodyContacts());
    }

    int below = 0, resting = 0;
    double speedSum = 0, speedMax = 0;
    for (std::size_t i = 0; i < spawned.size(); ++i) {
        below += planet.sdf(world.physics.bodyPosition(i)) < -1;
        const double speed = world.physics.bodySpeed(i);
        speedSum += speed;
        speedMax = std::max(speedMax, speed);
        resting += speed < 0.5;
    }
    const double bodies = double(std::max<std::size_t>(1, spawned.size()));
    const double frames = double(std::max(1, opts.frames));

    if (opts.obj) {
        fs::create_directories(opts.output);
        const Vec camera = replayCamera(start.front(), 0);
        const auto finalInvokers = invokerPositions(planet, opts.invokers, double(opts.frames - 1));
        const fs::path meshPath = opts.output / (std::string(name(method)) + ".obj");
        const fs::path bodyPath = opts.output / (std::string("trajectory_") + name(method) + ".csv");
        exportMesh(meshPath, world, finalInvokers, camera);
        auto out = createFile(bodyPath);
        out << "# one row per spawned body; positions are centers of mass in meters, velocities in m/s\n";
        out << "# spawn_p* is the prescribed launch pose, identical for every backend\n";
        out << "id,spawn_frame,age,invoker,kind,size_x,size_y,size_z,spawn_px,spawn_py,spawn_pz,"
               "px,py,pz,vx,vy,vz,speed\n";
        for (std::size_t i = 0; i < spawned.size(); ++i) {
            const Spawned& body = spawned[i];
            const Vec p = world.physics.bodyPose(i).position;
            const Vec v = world.physics.bodyState(i).velocity;
            const char* kind = body.shape.kind == ShapeKind::Sphere ? "sphere"
                             : body.shape.kind == ShapeKind::Capsule ? "capsule" : "box";
            out << i << ',' << body.frame << ',' << (opts.frames - 1 - body.frame) << ',' << body.invoker << ',' << kind
                << ',' << body.shape.size.x << ',' << body.shape.size.y << ',' << body.shape.size.z
                << ',' << body.position.x << ',' << body.position.y << ',' << body.position.z
                << ',' << p.x << ',' << p.y << ',' << p.z
                << ',' << v.x << ',' << v.y << ',' << v.z << ',' << length(v) << '\n';
        }
        std::cout << "  wrote " << meshPath.string() << ", " << bodyPath.string() << '\n';
    }

    std::cout << std::fixed << std::setprecision(3) << name(method) << " [trajectory]: " << opts.invokers
              << " moving invokers, spawn " << opts.shapesPerInvoker << " shapes every " << opts.spawnEvery
              << " frames for " << opts.frames << " frames (" << opts.frames * dt << " s) -> " << spawned.size()
              << " bodies" << (opts.bodyPairs ? ", body-body pairs enabled\n" : ", terrain-only pairs\n")
              << "  build " << initial.updateMs << " ms, " << updates << " terrain updates " << updateMs << " ms ("
              << built << " chunks built)\n"
              << "  terrain contacts per step: mean " << contactSum / frames << " max " << contactMax
              << ", body-body contacts max " << bodyPairMax << '\n'
              << "  at the end: still moving (>0.5 m/s) " << (int(spawned.size()) - resting) << '/' << spawned.size()
              << " (mean " << speedSum / bodies << " m/s, max " << speedMax << " m/s)\n"
              << "  bodies below analytic surface: " << below << '\n';
}

// Each entry point compares one field backend against the mesh baseline on the
// same queries; queryCompare() dispatches with --compare (default grid).
void gridRayCompare(const Options& opts) { compareRayToMesh(opts, Method::Grid); }
void vdbRayCompare(const Options& opts) { compareRayToMesh(opts, Method::Vdb); }
void dagRayCompare(const Options& opts) { compareRayToMesh(opts, Method::Dag); }
void gridSweepCompare(const Options& opts) { compareSweepToMesh(opts, Method::Grid); }
void vdbSweepCompare(const Options& opts) { compareSweepToMesh(opts, Method::Vdb); }
void dagSweepCompare(const Options& opts) { compareSweepToMesh(opts, Method::Dag); }
void gridCollisionCompare(const Options& opts) { compareCollisionToMesh(opts, Method::Grid); }
void vdbCollisionCompare(const Options& opts) { compareCollisionToMesh(opts, Method::Vdb); }
void dagCollisionCompare(const Options& opts) { compareCollisionToMesh(opts, Method::Dag); }

void queryCompare(const Options& opts) {
    if (opts.mode == QueryMode::Trajectory) {
        // One world at a time, and it is destroyed before the next backend starts,
        // so memory and cache never mix two representations.
        for (const Method method : opts.methods) compareTrajectory(opts, method);
        return;
    }
    for (const Method method : opts.compare) {
        switch (opts.mode) {
            case QueryMode::Ray:
                switch (method) {
                    case Method::Grid: gridRayCompare(opts); break;
                    case Method::Vdb: vdbRayCompare(opts); break;
                    case Method::Dag: dagRayCompare(opts); break;
                    default: break;  // mesh has no field counterpart to compare against
                }
                break;
            case QueryMode::Sweep:
                switch (method) {
                    case Method::Grid: gridSweepCompare(opts); break;
                    case Method::Vdb: vdbSweepCompare(opts); break;
                    case Method::Dag: dagSweepCompare(opts); break;
                    default: break;
                }
                break;
            case QueryMode::Collision:
                switch (method) {
                    case Method::Grid: gridCollisionCompare(opts); break;
                    case Method::Vdb: vdbCollisionCompare(opts); break;
                    case Method::Dag: dagCollisionCompare(opts); break;
                    default: break;
                }
                break;
            case QueryMode::Trajectory:
                break;  // handled above, one backend per run
        }
    }
}

}  // namespace voxel

int main(int argc, char** argv) {
    using namespace voxel;
    try {
        auto opts = options(argc, argv);
        // Fixed benchmark configuration: grid vs mesh, 16 static invokers, 21 shapes
        // each, 20 measured frames after 2 warmup frames, so every run reports the
        // same 6720 sweeps per backend and the numbers stay comparable over time.
        opts.mode = QueryMode::Sweep;
        opts.compare = {Method::Grid};
        opts.invokers = 75;
        opts.shapesPerInvoker = 21;
        opts.frames = 100;
        opts.warmup = 2;
        queryCompare(opts); return 0;
        //if (opts.help) { help(); return 0; }
        //if (opts.selfTest) { selfTest(); return 0; }
        //if (opts.rayCompare) { rayCompare(opts); return 0; }
        //fs::create_directories(opts.output);
        //auto frames = createFile(opts.output / "frames.csv");
        //auto summary = createFile(opts.output / "summary.csv");
        //auto streaming = createFile(opts.output / "streaming.csv");
        //std::ofstream capsuleReport;
        //if (opts.capsuleStress) {
        //    capsuleReport = createFile(opts.output / "capsule-cases.csv");
        //    capsuleReport << "scheme,scenario,queries,hits,initial,unresolved,p50_us,p95_us,p99_us,"
        //                     "vs_mesh_mismatch,paired_hits,vs_mesh_toi_p95_m\n";
        //}
        //frames << "scheme,frame,update_ms,select_ms,sample_ms,storage_ms,render_mesh_ms,register_remove_ms,built,removed,"
        //          "lod0,lod1,lod2,lod3,field_bytes,physics_static_bytes,render_mesh_bytes,step_ms,terrain_contacts,"
        //          "scheduled,window_visits,changed_owners,metadata_bytes,resident_terrain_bytes,staged_terrain_bytes\n";
        //streaming << "scheme,seed,actor_invokers,camera_invokers,global,initial_ms,scheduled_updates,changed_updates,"
        //             "changed_p50_ms,changed_p95_ms,changed_max_ms,changed_select_p95_ms,unchanged_p95_ms,"
        //             "built,removed,window_visits,changed_owners,peak_resident_bytes,estimated_peak_overlap_bytes,"
        //             "peak_staged_bytes,peak_field_bytes,peak_mesh_bytes,peak_collision_bytes\n";
        //summary << "scheme,query,layout,seed,frames,invokers,bodies,global,warmup,stream_every,queries,hits,initial,unresolved,"
        //           "vs_mesh_mismatch,paired_hits,vs_mesh_toi_p95_m,p50_us,p95_us,p99_us,queries_per_second,initial_ms,update_p95_ms,step_p50_ms,"
        //           "step_p95_ms,step_p99_ms,peak_collision_bytes,stream_built,stream_removed,max_terrain_contacts,body_centers_below_analytic,"
        //           "vs_first_field_different\n";
        //std::optional<QueryResults> reference, fieldReference;
        //std::cout << std::fixed << std::setprecision(3);
        //for (Method method : opts.methods) {
        //    auto result = run(opts, method, frames, summary, streaming, reference ? &*reference : nullptr,
        //                      fieldReference ? &*fieldReference : nullptr,
        //                      opts.capsuleStress ? &capsuleReport : nullptr);
        //    if (method == Method::Mesh) reference = std::move(result);
        //    else if (!fieldReference) fieldReference = std::move(result);
        //}
        //frames.close();
        //summary.close();
        //streaming.close();
        //return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
