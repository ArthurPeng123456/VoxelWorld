#include "collision.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>

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
    bool morton{}, global{}, obj{}, selfTest{}, help{};
    fs::path output{"results"};
};

void help() {
    std::cout << "voxel_world [--scheme all|mesh|grid|dag|vdb] [--seed N] [--frames N]\n"
              << "  [--invokers N] [--queries N] [--bodies N] [--warmup N] [--stream-every N]\n"
              << "  [--morton] [--global] [--obj] [--out DIRECTORY] [--self-test]\n"
              << "Defaults: 90 frames, 6 warmup, 2 invokers, 12 queries per type per frame,\n"
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
        else if (arg == "--help" || arg == "-h") opts.help = true;
        else throw std::invalid_argument("Unknown argument: " + arg);
    }
    if (std::int64_t(opts.frames) * opts.queries > 1'000'000) throw std::invalid_argument("frames * queries must be at most 1000000");
    return opts;
}

struct WorldStats {
    BuildStats build;
    double selectMs{}, updateMs{}, physicsLoadMs{};
};

class World {
public:
    World(const Planet& planet, Method method, bool morton)
        : physics(method, chunks), planet_(planet), method_(method), morton_(morton) {}

    WorldStats update(const std::vector<Vec>& invokers, bool global) {
        WorldStats stats;
        const auto start = Clock::now();
        const auto layout = selectChunks(invokers, global);
        stats.selectMs = milliseconds(start);
        for (auto it = chunks.begin(); it != chunks.end();) {
            const auto found = layout.find(it->first);
            if (found != layout.end() && it->second->neighborhood == neighborhood(layout, it->first)) {
                ++it;
                continue;
            }
            const auto tick = Clock::now();
            physics.removeTerrain(it->first);
            ++stats.build.removed;
            it = chunks.erase(it);
            stats.physicsLoadMs += milliseconds(tick);
        }
        for (const auto& entry : layout) {
            if (chunks.find(entry.first) != chunks.end()) continue;
            auto chunk = buildChunk(planet_, layout, entry.first, method_, morton_, stats.build);
            auto inserted = chunks.emplace(entry.first, std::move(chunk));
            const auto tick = Clock::now();
            physics.addTerrain(*inserted.first->second);
            stats.physicsLoadMs += milliseconds(tick);
        }
        stats.updateMs = milliseconds(start);
        return stats;
    }

    std::array<int, 4> lodCounts() const {
        std::array<int, 4> result{};
        for (const auto& entry : chunks) ++result[entry.second->lod];
        return result;
    }
    std::size_t fieldBytes() const {
        std::size_t result = 0;
        for (const auto& entry : chunks) if (entry.second->field) result += entry.second->field->bytes();
        return result;
    }
    std::size_t meshBytes() const {
        std::size_t result = 0;
        for (const auto& entry : chunks) result += entry.second->mesh.capacity() * sizeof(Triangle);
        return result;
    }

    Chunks chunks;
    Physics physics;
private:
    const Planet& planet_;
    Method method_;
    bool morton_;
};

std::vector<Vec> positions(const Planet& planet, int count, int frame) {
    std::vector<Vec> result;
    for (int i = 0; i < count; ++i) result.push_back(planet.surface(planet.invokerDirection(i, frame)));
    return result;
}

std::array<Vec, 3> frameAt(Vec up) {
    const Vec reference = std::abs(up.z) < 0.9 ? Vec{0, 0, 1} : Vec{0, 1, 0};
    const Vec x = unit(cross(reference, up));
    return {x, cross(up, x), up};
}

Shape sampleShape(int kind, Vec up) {
    if (kind == 1) return {ShapeKind::Sphere, {13, 0, 0}, frameAt(up)};
    if (kind == 2) return {ShapeKind::Capsule, {9, 0, 12}, frameAt(up)};
    return {ShapeKind::Box, {15, 11, 10}, frameAt(up)};
}

struct Query { int kind; Vec from, to; Shape shape; };

std::vector<Query> queries(const Planet& planet, const std::vector<Vec>& invokers, int count, int frame,
                           std::uint64_t seed) {
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
    struct Outcome { Hit::State state; double pathPosition; };
    std::vector<Outcome> results;
    int hits{}, unresolved{}, initial{};
};

std::ofstream createFile(const fs::path& path) {
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("Could not create " + path.string());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << std::fixed << std::setprecision(6);
    return stream;
}

void exportMesh(const fs::path& path, const Chunks& chunks) {
    auto out = createFile(path);
    out << "# VoxelWorld Marching Cubes, world coordinates in meters\n";
    std::size_t index = 1;
    for (const auto& entry : chunks) {
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

void logBuild(std::ostream& out, Method method, int frame, const WorldStats& stats, const World& world,
              double stepMs, int contacts) {
    out << name(method) << ',' << frame << ',' << stats.updateMs << ',' << stats.selectMs << ','
        << stats.build.sampleMs << ',' << stats.build.storageMs << ',' << stats.build.meshMs << ','
        << stats.physicsLoadMs << ',' << stats.build.built << ',' << stats.build.removed;
    for (int count : world.lodCounts()) out << ',' << count;
    out << ',' << world.fieldBytes() << ',' << world.physics.terrainBytes() << ',' << world.meshBytes()
        << ',' << stepMs << ',' << contacts << '\n';
}

using QueryResults = std::array<QueryMetrics, 4>;

QueryResults run(const Options& opts, Method method, std::ostream& frames, std::ostream& summary,
                 const QueryResults* meshReference) {
    Planet planet(opts.seed);
    World world(planet, method, opts.morton);
    auto invokers = positions(planet, opts.invokers, 0);
    const WorldStats initial = world.update(invokers, opts.global);
    logBuild(frames, method, -1, initial, world, 0, 0);
    spawn(world, planet, opts.bodies, invokers);
    for (int i = 0; i < opts.warmup; ++i) {
        for (const Query& q : queries(planet, invokers, opts.queries, i, opts.seed)) {
            if (q.kind == 0) world.physics.ray(q.from, q.to);
            else world.physics.sweep(q.shape, q.from, q.to);
        }
        world.physics.step(1.0 / 60, true);
    }

    QueryResults metrics;
    std::vector<double> steps, updates;
    int totalBuilt = 0, totalRemoved = 0, maxContacts = 0;
    std::size_t peakCollision = world.fieldBytes() + world.physics.terrainBytes();
    for (int frame = 0; frame < opts.frames; ++frame) {
        WorldStats stats;
        const auto moving = positions(planet, opts.invokers, frame);
        if (frame > 0 && frame % opts.streamEvery == 0) {
            stats = world.update(moving, opts.global);
            updates.push_back(stats.updateMs);
            totalBuilt += stats.build.built;
            totalRemoved += stats.build.removed;
            peakCollision = std::max(peakCollision, world.fieldBytes() + world.physics.terrainBytes());
        }
        const auto workload = queries(planet, moving, opts.queries, frame, opts.seed);
        for (const auto& q : workload) {
            const auto timer = Clock::now();
            const Hit result = q.kind == 0 ? world.physics.ray(q.from, q.to) : world.physics.sweep(q.shape, q.from, q.to);
            const auto elapsed = milliseconds(timer) * 1000;
            auto& m = metrics[q.kind];
            m.durations.push_back(elapsed);
            m.results.push_back({result.state, result.fraction * length(q.to - q.from)});
            m.hits += result.state == Hit::State::Contact;
            m.unresolved += result.state == Hit::State::Unresolved;
            m.initial += result.initial;
        }
        const auto timer = Clock::now();
        world.physics.step(1.0 / 60, true);
        const auto elapsed = milliseconds(timer);
        const int contacts = world.physics.terrainContacts();
        maxContacts = std::max(maxContacts, contacts);
        steps.push_back(elapsed);
        logBuild(frames, method, frame, stats, world, elapsed, contacts);
    }
    if (opts.obj) exportMesh(opts.output / (std::string(name(method)) + ".obj"), world.chunks);

    int below = 0;
    for (int i = 0; i < opts.bodies; ++i) below += planet.sdf(world.physics.bodyPosition(i)) < -1;
    std::sort(steps.begin(), steps.end());
    std::sort(updates.begin(), updates.end());
    std::cout << name(method) << " initial " << initial.updateMs << " ms, step P95 " << percentile(steps, 0.95)
              << " ms, peak collision " << (double(peakCollision) / 1048576) << " MiB, max contacts "
              << maxContacts << ", below analytic surface " << below << '\n';
    constexpr const char* labels[] = {"ray", "sphere", "capsule", "box"};
    for (int i = 0; i < 4; ++i) {
        auto& m = metrics[i];
        std::sort(m.durations.begin(), m.durations.end());
        int mismatch = 0;
        std::vector<double> hitErrors;
        if (meshReference) {
            for (std::size_t j = 0; j < m.results.size(); ++j) {
                const auto a = (*meshReference)[i].results[j], b = m.results[j];
                if (b.state == Hit::State::Unresolved) continue;
                mismatch += a.state != b.state;
                if (a.state == Hit::State::Contact && b.state == Hit::State::Contact) {
                    hitErrors.push_back(std::abs(a.pathPosition - b.pathPosition));
                }
            }
            std::sort(hitErrors.begin(), hitErrors.end());
        }
        summary << name(method) << ',' << labels[i] << ',' << (method == Method::Grid && opts.morton ? "morton" : "default")
                << ',' << opts.seed << ',' << opts.frames << ',' << opts.invokers << ',' << opts.bodies << ',' << opts.global
                << ',' << opts.warmup << ',' << opts.streamEvery << ',' << m.results.size() << ',' << m.hits << ','
                << m.initial << ',' << m.unresolved << ',';
        if (meshReference) summary << mismatch << ',' << hitErrors.size() << ',' << percentile(hitErrors, 0.95);
        else summary << ",,";
        summary << ',' << percentile(m.durations, 0.50) << ',' << percentile(m.durations, 0.95) << ','
                << percentile(m.durations, 0.99) << ','
                << (m.durations.size() * 1e6 / std::accumulate(m.durations.begin(), m.durations.end(), 0.0))
                << ',' << initial.updateMs << ',' << percentile(updates, 0.95)
                << ',' << percentile(steps, 0.50) << ',' << percentile(steps, 0.95) << ',' << percentile(steps, 0.99)
                << ',' << peakCollision << ',' << totalBuilt << ',' << totalRemoved << ',' << maxContacts << ',' << below << '\n';
        std::cout << "  " << labels[i] << " P95 " << percentile(m.durations, 0.95) << " us, hits "
                  << m.hits << ", unresolved " << m.unresolved;
        if (meshReference) std::cout << ", mismatches " << mismatch << ", paired P95 error " << percentile(hitErrors, 0.95) << " m";
        std::cout << '\n';
    }
    return metrics;
}

void check(bool condition, const std::string& description) {
    if (!condition) throw std::runtime_error(std::string("Self-test failed: ") + description);
}

std::unique_ptr<Chunk> fixture(Method method, bool morton, bool saddle = false) {
    const int n = saddle ? 1 : 32;
    auto chunk = std::make_unique<Chunk>(Chunk{{0, 0, 0}, 0, n, bounds({0, 0, 0}), chunkSize / n, {}, {}, {}, {}});
    std::vector<float> sample;
    for (int z = 0; z <= n; ++z) for (int y = 0; y <= n; ++y) {
        for (int x = 0; x <= n; ++x) {
            sample.push_back(saddle ? float((x - 0.3) * (y - 0.7)) : float(z * chunk->spacing - 797));
        }
    }
    chunk->field = makeStorage(method, n + 1, sample, morton, chunk->spacing, {});
    meshChunk(*chunk);
    return chunk;
}

void selfTest() {
    const Planet planet(42);
    const auto first = positions(planet, 1, 0), both = positions(planet, 2, 0);
    const auto one = selectChunks(first, false), two = selectChunks(both, false);
    check(two.size() > one.size(), "two invokers load additional chunks");
    for (const auto& entry : one) check(two.at(entry.first) <= entry.second, "nearest invoker wins");
    check(neighborhood(two, keyAt(first[0])) != neighborhood(selectChunks(positions(planet, 2, 25), false),
                                                           keyAt(first[0])), "moving invokers change LOD");
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

    for (const auto& option : {std::make_pair(Method::Mesh, false), std::make_pair(Method::Grid, false),
                              std::make_pair(Method::Grid, true), std::make_pair(Method::Dag, false),
                              std::make_pair(Method::Vdb, false)}) {
        const auto method = option.first;
        std::cout << "Testing " << name(method) << (option.second ? " Morton" : "") << std::endl;
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
        frames << "scheme,frame,update_ms,select_ms,sample_ms,storage_ms,render_mesh_ms,register_remove_ms,built,removed,"
                  "lod0,lod1,lod2,lod3,field_bytes,physics_static_bytes,render_mesh_bytes,step_ms,terrain_contacts\n";
        summary << "scheme,query,layout,seed,frames,invokers,bodies,global,warmup,stream_every,queries,hits,initial,unresolved,"
                   "vs_mesh_mismatch,paired_hits,vs_mesh_toi_p95_m,p50_us,p95_us,p99_us,queries_per_second,initial_ms,update_p95_ms,step_p50_ms,"
                   "step_p95_ms,step_p99_ms,peak_collision_bytes,stream_built,stream_removed,max_terrain_contacts,body_centers_below_analytic\n";
        QueryResults reference;
        bool hasMesh = false;
        std::cout << std::fixed << std::setprecision(3);
        for (Method method : opts.methods) {
            auto result = run(opts, method, frames, summary, hasMesh ? &reference : nullptr);
            if (method == Method::Mesh) {
                reference = std::move(result);
                hasMesh = true;
            }
        }
        frames.close();
        summary.close();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
