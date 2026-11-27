#include "world.hpp"
#include "invoker_navigation.hpp"

#include <iostream>
#include <stdexcept>

using namespace voxel;

int main(int argc, char** argv) {
    try {
        const int count = argc > 1 ? parseInvokerCount(std::string(argv[1])) : 256;
        const auto cells = argc > 2 ? parseLod0Cells(std::string(argv[2])) : defaultLod0Cells;
        const Planet planet(42, TerrainPreset::Mars);
        {
            std::vector<Vec> moving;
            for (int i = 0; i < 4; ++i) moving.push_back(planet.surface(initialInvokerDirection(i, 42)));
            Clipmap simultaneous(0, planet.elevationRange(), &planet);
            std::size_t visits = 0;
            simultaneous.update(moving, true, visits, 64);
            for (Vec& point : moving)
                point = planet.surface(unit(point + frameAt(unit(point))[0] * 256));
            simultaneous.update(moving, true, visits, 64, true);
            int covered = 0;
            for (Vec point : moving) {
                const auto leaf = leafAt(simultaneous.layout(), point);
                covered += leaf && leaf->level == 0;
            }
            std::cout << "simultaneous_fine_coverage=" << covered << "/" << moving.size() << std::endl;
            if (covered != int(moving.size()))
                throw std::runtime_error("Moving distributed Invokers starve behind a single global rolling window");
        }
        std::vector<Vec> sources;
        for (int i = 0; i < 2; ++i) sources.push_back(planet.surface(initialInvokerDirection(i, 42)));
        const Vec camera = sources[0] + unit(sources[0]) * 6;
        sources.push_back(camera);
        Clipmap clipmap(0, planet.elevationRange(), &planet);
        std::size_t visits = 0;
        clipmap.update(sources, true, visits, cells);
        sources.clear();
        for (int i = 0; i < count; ++i) sources.push_back(planet.surface(initialInvokerDirection(i, 42)));
        sources.push_back(camera);
        if (count >= 12) {
            Clipmap progressive(0, planet.elevationRange(), &planet);
            ClipmapForecast forecast;
            forecast.minimumLod.assign(sources.size(), gridRootLevel(0) - 2);
            forecast.minimumLod[0] = forecast.minimumLod[1] = forecast.minimumLod.back() = 0;
            progressive.update(sources, true, visits, cells, true, forecast);
            if (leafAt(progressive.layout(), sources[7])->level == 0)
                throw std::runtime_error("A queued source was unexpectedly admitted as a fine region");
            for (std::size_t selected : {7u, 8u, 11u}) {
                forecast.minimumLod[selected] = 0;
                progressive.update(sources, true, visits, cells, true, forecast);
                const auto point = leafAt(progressive.layout(), sources[selected]);
                if (!point || point->level != 0)
                    throw std::runtime_error("Activating a selected Invoker failed to publish its fine region");
                Layout ideal, oracle;
                updatePlanetLayout(ideal, oracle, progressive.windows(), {}, 0, true, visits, planet.elevationRange(), &planet);
                if (oracle != progressive.layout())
                    throw std::runtime_error("Progressive admission differs from a full balanced layout");
            }
            forecast.minimumLod[7] = gridRootLevel(0) - 2;
            progressive.update(sources, true, visits, cells, true, forecast);
            if (leafAt(progressive.layout(), sources[7])->level == 0)
                throw std::runtime_error("Removing fine windows retained stale fine terrain");
            forecast.minimumLod.assign(sources.size(), lodWindowCount(cells));
            progressive.update(sources, true, visits, cells, true, forecast);
            Layout ideal, oracle;
            updatePlanetLayout(ideal, oracle, {}, {}, 0, true, visits, planet.elevationRange(), &planet);
            if (!progressive.windows().empty() || progressive.layout() != oracle)
                throw std::runtime_error("Disabling all source windows lost the full-field coarse fallback");
            std::cout << "progressive_targets_8_9_12=passed" << std::endl;
        }
        auto start = Clock::now();
        clipmap.update(sources, true, visits, cells, true);
        std::cout << "growth_ms=" << milliseconds(start) << " leaves=" << clipmap.layout().size()
                  << " visits=" << visits << std::endl;
        int missed = 0;
        for (int i = 0; i < count; ++i) {
            const auto leaf = leafAt(clipmap.layout(), sources[i]);
            if (!leaf || leaf->level != 0) {
                std::cout << "unrefined_invoker=" << i + 1 << " lod=" << (leaf ? leaf->level : -1) << std::endl;
                ++missed;
            }
        }
        start = Clock::now();
        Clipmap fresh(0, planet.elevationRange(), &planet);
        fresh.update(sources, true, visits, cells);
        std::cout << "fresh_ms=" << milliseconds(start) << " leaves=" << fresh.layout().size()
                  << " equal=" << (clipmap.layout() == fresh.layout()) << " missed=" << missed << std::endl;
        if (missed || clipmap.layout() != fresh.layout())
            throw std::runtime_error("Distributed Invoker clipmaps differ after growing the population");
        double maxMeshError = 0;
        for (int i = 0; i < count; ++i) {
            const Vec point = sources[i], up = unit(point);
            const Box patch{point - splat(2), point + splat(2)};
            Hit nearest;
            for (Key key : intersectingLeaves(clipmap.layout(), patch)) {
                BuildStats stats;
                const auto chunk = buildChunk(planet, clipmap.layout(), key, Method::Grid, false, stats);
                for (const auto& triangle : chunk->mesh) {
                    const auto hit = triangleRay(triangle, point + up * 2, point - up * 2);
                    if (hit.state == Hit::State::Contact &&
                        (nearest.state != Hit::State::Contact || hit.fraction < nearest.fraction)) nearest = hit;
                }
            }
            if (nearest.state != Hit::State::Contact)
                throw std::runtime_error("Distributed Invoker " + std::to_string(i + 1) + " has no actual fine mesh");
            maxMeshError = std::max(maxMeshError, length(nearest.position - point));
        }
        std::cout << "fine_meshes_tested=" << count << " max_surface_error_m=" << maxMeshError << std::endl;
        for (int i = 0; i < count; ++i)
            sources[i] = planet.surface(unit(sources[i] + frameAt(unit(sources[i]))[0] * (4 * cells)));
        start = Clock::now();
        clipmap.update(sources, true, visits, cells, true);
        for (int i = 0; i < count; ++i) {
            const auto leaf = leafAt(clipmap.layout(), sources[i]);
            if (!leaf || leaf->level != 0)
                throw std::runtime_error("Simultaneous global movement left an Invoker without LOD0");
        }
        Layout ideal, oracle;
        updatePlanetLayout(ideal, oracle, clipmap.windows(), {}, 0, true, visits, planet.elevationRange(), &planet);
        if (clipmap.layout() != oracle)
            throw std::runtime_error("Simultaneous rolling slabs do not match a balanced full rebuild");
        std::cout << "moving_invokers=" << count << " fine_coverage=" << count << '/' << count
                  << " balanced_layout_equal=1 moving_check_ms=" << milliseconds(start) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
