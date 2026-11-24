#include "terrain.hpp"

namespace voxel {
namespace {

constexpr int cells = 32;
constexpr int cellCount = cells * cells * cells;
constexpr double ejectaReach = 1.55;
constexpr double shieldReach = 3.25;

int gridIndex(int x, int y, int z) { return (z * cells + y) * cells + x; }
int directionCell(double coordinate) {
    return std::clamp(int((coordinate + 1) * (cells / 2)), 0, cells - 1);
}

template<bool withGradient>
double ramp(double lower, double upper, double value, double& slope) {
    const double t = std::clamp((value - lower) / (upper - lower), 0.0, 1.0);
    if constexpr (withGradient) slope = 6 * t * (1 - t) / (upper - lower);
    return t * t * (3 - 2 * t);
}

std::uint32_t craterHash(std::uint32_t x) {
    x = (x ^ (x >> 16)) * 0x7feb352dU;
    x = (x ^ (x >> 15)) * 0x846ca68bU;
    return x ^ (x >> 16);
}

Vec rotateCraterGrid(Vec p) {
    const double x = 0.8 * p.x - 0.6 * p.y;
    return {0.8 * x + 0.6 * p.z, 0.6 * p.x + 0.8 * p.y, -0.6 * x + 0.8 * p.z};
}

Vec unrotateCraterGradient(Vec p) {
    const double x = 0.8 * p.x - 0.6 * p.z;
    return {0.8 * x + 0.6 * p.y, -0.6 * x + 0.8 * p.y, 0.6 * p.x + 0.8 * p.z};
}

template<bool withGradient, std::size_t n>
double craterTiles(Vec direction, const std::array<int, n>& resolutions, std::uint32_t seed,
                   Vec& gradient, double* material, double spacing) {
    const Vec magnitude = abs(direction);
    int faceAxis = magnitude.x >= magnitude.y ? 0 : 1;
    if (magnitude.z > magnitude[faceAxis]) faceAxis = 2;
    const int u = (faceAxis + 1) % 3, v = (faceAxis + 2) % 3;
    const double inv = 1 / magnitude[faceAxis];
    const double faceU = direction[u] * inv, faceV = direction[v] * inv;
    const double faceSign = direction[faceAxis] < 0 ? -1.0 : 1.0;
    const auto faceSeed = seed ^ (std::uint32_t(faceAxis * 2 + (faceSign < 0)) * 0x94d049bbU);
    double height = 0;
    for (const int resolution : resolutions) {
        const double scale = 0.5 * resolution;
        const double x = std::clamp((faceU + 1) * scale, 0.0, double(resolution));
        const double y = std::clamp((faceV + 1) * scale, 0.0, double(resolution));
        const int column = std::min(int(x), resolution - 1), row = std::min(int(y), resolution - 1);
        const auto key = std::uint32_t(column) * 0x9e3779b9U ^ std::uint32_t(row) * 0x85ebca77U;
        const auto h = craterHash(key ^ faceSeed ^ (std::uint32_t(resolution) * 0xc2b2ae3dU));
        if (h % 13 < 2) continue;
        const double size = double(h & 255) / 255;
        const double radius = 0.145 + 0.225 * size * size;
        const double support = 1.28 * radius;
        const double freeWidth = 1 - 2 * support;
        const double dx = x - column - (support + freeWidth * (double((h >> 8) & 255) / 255));
        const double dy = y - row - (support + freeWidth * (double((h >> 16) & 255) / 255));
        const double d2 = dx * dx + dy * dy;
        if (d2 >= support * support) continue;

        // Each crater's rim fits entirely inside its tile. This also gives the
        // faces of the cube a continuous, zero-height boundary for this layer.
        const double r = std::sqrt(d2) / radius;
        const double radiusMeters = radius * planetRadius / scale * (0.55 + 0.45 * magnitude[faceAxis]);
        const double age = double(h >> 24) / 255;
        const double depth = std::min(128.0, radiusMeters * (0.065 + 0.085 * age));
        const double rim = std::min(53.0, radiusMeters * (0.018 + 0.024 * age));
        double bowlSlope{}, wallSlope{};
        const double bowl = 1 - ramp<withGradient>(0.18, 1.0, r, bowlSlope);
        const double wall = 1 - ramp<withGradient>(0.02, 0.28, std::abs(r - 1), wallSlope);
        height += rim * wall - depth * bowl;
        double unused;
        const double visibility = spacing > 0 ? ramp<false>(1.2 * spacing, 3.6 * spacing, radiusMeters, unused) : 1;
        if (material) {
            const double contribution = visibility * (0.52 * bowl + 0.72 * wall) * (0.55 + 0.45 * age);
            *material += (1 - *material) * contribution;
        }
        if constexpr (withGradient) {
            // Differentiate the normalized face coordinates and the local scale.
            double strength{};
            if (r > 1e-12)
                strength = (depth * bowlSlope + (r < 1 ? 1 : -1) * rim * wallSlope) * scale * inv /
                           (r * radius * radius);
            gradient[u] += visibility * dx * strength;
            gradient[v] += visibility * dy * strength;
            const double depthSlope = depth < 128 ? 0.45 * (radius * planetRadius / scale) * (0.065 + 0.085 * age) : 0;
            const double rimSlope = rim < 53 ? 0.45 * (radius * planetRadius / scale) * (0.018 + 0.024 * age) : 0;
            gradient[faceAxis] += visibility * faceSign *
                                  (rimSlope * wall - depthSlope * bowl - (dx * faceU + dy * faceV) * strength);
        }
    }
    return height;
}

}  // namespace

void Planet::buildGeology(std::uint64_t seed) {
    auto random = [&seed] {
        seed += 0x9e3779b97f4a7c15ULL;
        auto value = seed;
        value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
        value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
        return ((value ^ (value >> 31)) >> 11) * 0x1p-53;
    };
    const auto direction = [&] {
        const double z = random() * 2 - 1;
        const double angle = random() * 6.283185307179586;
        const double radial = std::sqrt(1 - z * z);
        return Vec{radial * std::cos(angle), radial * std::sin(angle), z};
    };
    const bool mars = preset_ == TerrainPreset::Mars;
    craterSeed_ = craterHash(std::uint32_t(seed) ^ craterHash(std::uint32_t(seed >> 32)));
    const auto add = [&](Vec axis, double radius, double depth, double rim, double peak) {
        // Store the angular chord in unit-sphere coordinates. The transverse
        // direction gives the impact footprint a slight ellipticity.
        const double scale = radius / planetRadius;
        impacts_.push_back({axis, unit(cross(axis, direction())), 0.5 * scale * scale, depth, rim, peak});
    };

    // Seed landing sites in view of crater walls or calderas, then populate the
    // rest of the sphere with independent features at several distinct scales.
    constexpr int landingSites = 16;
    for (int i = 0; i < landingSites; ++i) {
        const auto axis = direction();
        const double radius = (mars ? 440 : 530) + 220 * random();
        add(axis, radius, mars ? 125 : 170, mars ? 36 : 72, mars ? 22 : 0);
    }
    if (mars) {
        struct ImpactBand { int count; double minRadius, maxRadius, depth, rim, peak; };
        constexpr ImpactBand bands[] = {
            {18, 390000, 660000, 1650, 430, 140},
            {96, 140000, 390000, 1220, 290, 160},
            {680, 43000, 145000, 590, 128, 110},
            {3850, 9000, 46000, 210, 65, 44}
        };
        for (const auto band : bands) for (int i = 0; i < band.count; ++i) {
            const auto axis = direction();
            const double draw = random();
            const double radius = band.minRadius * std::pow(band.maxRadius / band.minRadius, draw * draw);
            const double age = 0.62 + 0.38 * random();
            const double depth = band.depth * age * (0.83 + 0.37 * random());
            const double rim = band.rim * age * (0.78 + 0.40 * random());
            add(axis, radius, depth, rim, band.peak * age);
        }
    } else {
        for (int i = 0; i < 14; ++i) {
            const auto axis = direction();
            const double radius = 140000 + random() * 260000;
            const double depth = 880 + random() * 680;
            const double rim = 510 + random() * 220;
            add(axis, radius, depth, rim, 0);
        }
        for (int i = 0; i < 76; ++i) {
            const auto axis = direction();
            const double radius = 15000 + random() * 71000;
            const double depth = 440 + random() * 550;
            const double rim = 142 + random() * 155;
            add(axis, radius, depth, rim, 0);
        }
        for (int i = 0; i < 340; ++i) {
            const auto axis = direction();
            const double radius = 900 + std::pow(random(), 2) * 12500;
            add(axis, radius, std::min(radius * 0.088, 320.0), std::min(radius * 0.022, 101.0), 0);
        }
        for (int i = 0; i < landingSites; ++i) {
            const Vec center = impacts_[i].axis;
            const Vec across = unit(cross(center, {0, 0, 1}), {1, 0, 0});
            const Vec along = cross(center, across);
            for (int j = 0; j < 14; ++j) {
                const double angle = random() * 6.283185307179586;
                const double distance = (480 + 2900 * random()) / planetRadius;
                const double radius = 85 + 290 * std::pow(random(), 2);
                const Vec offset = (across * std::cos(angle) + along * std::sin(angle)) * distance;
                add(unit(center + offset), radius, radius * 0.28, radius * 0.075, 0);
            }
        }
    }

    std::vector<std::vector<unsigned>> buckets(cellCount);
    for (unsigned i = 0; i < impacts_.size(); ++i) {
        const auto& impact = impacts_[i];
        const double support = !mars && impact.rim >= 45 ? shieldReach : ejectaReach;
        const double reach = support * std::sqrt(2 * impact.width) + 1e-12;
        const Vec lo = impact.axis - splat(reach), hi = impact.axis + splat(reach);
        for (int z = directionCell(lo.z); z <= directionCell(hi.z); ++z)
            for (int y = directionCell(lo.y); y <= directionCell(hi.y); ++y)
                for (int x = directionCell(lo.x); x <= directionCell(hi.x); ++x)
                    buckets[gridIndex(x, y, z)].push_back(i);
    }
    impactOffsets_.reserve(cellCount + 1);
    for (const auto& bucket : buckets) {
        impactOffsets_.push_back(unsigned(impactIndices_.size()));
        impactIndices_.insert(impactIndices_.end(), bucket.begin(), bucket.end());
    }
    impactOffsets_.push_back(unsigned(impactIndices_.size()));
    noise_.resize(cellCount);
    for (auto& value : noise_) value = float(random() * 2 - 1);
}

template<bool withGradient>
double Planet::geologyNoise(Vec p, Vec& gradient) const {
    const Vec origin{std::floor(p.x), std::floor(p.y), std::floor(p.z)};
    const Vec fraction = p - origin;
    const Vec t{fraction.x * fraction.x * (3 - 2 * fraction.x),
                fraction.y * fraction.y * (3 - 2 * fraction.y),
                fraction.z * fraction.z * (3 - 2 * fraction.z)};
    const auto at = [&](int dx, int dy, int dz) {
        return double(noise_[gridIndex((int(origin.x) + dx) & (cells - 1),
                                       (int(origin.y) + dy) & (cells - 1),
                                       (int(origin.z) + dz) & (cells - 1))]);
    };
    const double a = at(0, 0, 0), b = at(1, 0, 0), c = at(0, 1, 0), d = at(1, 1, 0);
    const double e = at(0, 0, 1), f = at(1, 0, 1), g = at(0, 1, 1), h = at(1, 1, 1);
    const auto mix = [](double a, double b, double t) { return a + (b - a) * t; };
    const double x00 = mix(a, b, t.x), x10 = mix(c, d, t.x);
    const double x01 = mix(e, f, t.x), x11 = mix(g, h, t.x);
    const double y0 = mix(x00, x10, t.y), y1 = mix(x01, x11, t.y);
    if constexpr (withGradient) {
        gradient = {
            mix(mix(b - a, d - c, t.y), mix(f - e, h - g, t.y), t.z) * 6 * fraction.x * (1 - fraction.x),
            mix(x10 - x00, x11 - x01, t.z) * 6 * fraction.y * (1 - fraction.y),
            (y1 - y0) * 6 * fraction.z * (1 - fraction.z)
        };
    }
    return mix(y0, y1, t.z);
}

template<bool withGradient>
double Planet::sampleGeology(Vec d, Vec& gradient, double* craterMaterial, double spacing) const {
    const auto layer = [&](double frequency, Vec offset, double amplitude) {
        Vec slope;
        const double value = geologyNoise<withGradient>(d * frequency + offset, slope);
        if constexpr (withGradient) gradient = gradient + slope * (frequency * amplitude);
        return value * amplitude;
    };
    const auto ridge = [&](double frequency, Vec offset, double amplitude) {
        Vec slope;
        const double value = geologyNoise<withGradient>(d * frequency + offset, slope);
        const double crest = 1 - std::abs(value);
        if constexpr (withGradient) {
            const double sign = value > 0 ? -1 : value < 0 ? 1 : 0;
            gradient = gradient + slope * (sign * 3 * crest * crest * frequency * amplitude);
        }
        return (crest * crest * crest - 0.35) * amplitude;
    };
    const bool mars = preset_ == TerrainPreset::Mars;
    double relief;
    if (mars) {
        relief = layer(11.5, {13, 56, 3}, 1250) + layer(47, {61, 15, 27}, 610)
               + layer(planetRadius / 9500, {7, 9, 20}, 200) + layer(planetRadius / 1500, {49, 19, 72}, 95)
               + layer(planetRadius / 310, {86, 19, 52}, 32);
        constexpr std::array first{337, 5381, 55291};
        constexpr std::array second{1627, 17123};
        Vec rotatedSlope;
        relief += craterTiles<withGradient>(d, first, craterSeed_, gradient, craterMaterial, spacing);
        relief += craterTiles<withGradient>(rotateCraterGrid(d), second, craterSeed_, rotatedSlope, craterMaterial, spacing);
        if constexpr (withGradient) gradient = gradient + unrotateCraterGradient(rotatedSlope);
    } else {
        relief = layer(12, {17, 20, 31}, 860) + ridge(38, {59, 10, 4}, 2250)
               + ridge(planetRadius / 12800, {16, 40, 57}, 610) + ridge(planetRadius / 2950, {42, 13, 93}, -310)
               + ridge(planetRadius / 1320, {93, 19, 57}, -235) + layer(planetRadius / 330, {24, 28, 75}, 55);
    }

    double deepest = 0, highest = 0, basinMaterial = 0;
    Vec bowlSlope, rimSlope;
    const int bucket = gridIndex(directionCell(d.x), directionCell(d.y), directionCell(d.z));
    for (unsigned i = impactOffsets_[bucket]; i < impactOffsets_[bucket + 1]; ++i) {
        const auto& impact = impacts_[impactIndices_[i]];
        const double transverse = dot(d, impact.along);
        const double ellipticity = mars ? 0.18 : 0.56;
        const bool shielded = !mars && impact.rim >= 45;
        const double support = shielded ? shieldReach : ejectaReach;
        // Subtract the unit directions before squaring to retain precision for
        // craters tens of meters across on a multi-million-meter sphere.
        const Vec offset = d - impact.axis;
        const double distanceSquared = (0.5 * dot(offset, offset) + ellipticity * transverse * transverse) / impact.width;
        if (distanceSquared >= support * support) continue;
        const double r = std::sqrt(std::max(0.0, distanceSquared));
        double bowlDerivative{}, wallDerivative{}, ejectaDerivative{}, peakDerivative{};
        const double bowl = impact.depth * (1 - ramp<withGradient>(mars ? 0.27 : 0.47, 1.0, r, bowlDerivative));
        const double wall = impact.rim * (1 - ramp<withGradient>(0.02, 0.32, std::abs(r - 1), wallDerivative));
        const double ejecta = 0.18 * impact.rim * (1 - ramp<withGradient>(1.04, ejectaReach, r, ejectaDerivative));
        const double peak = impact.peak * (1 - ramp<withGradient>(0.06, 0.31, r, peakDerivative));
        double shieldDerivative{};
        const double shieldHeight = shielded ? 1.5 * impact.rim : 0;
        const double shield = shieldHeight * (1 - ramp<withGradient>(0.45, shieldReach, r, shieldDerivative));
        const double raised = wall + ejecta + peak + shield;
        if (mars && craterMaterial) {
            double unused;
            const double radiusMeters = std::sqrt(2 * impact.width) * planetRadius;
            const double visible = spacing > 0 ? ramp<false>(0.45 * spacing, 2.1 * spacing, radiusMeters, unused) : 1;
            const double material = 0.37 * bowl / impact.depth + 0.70 * wall / impact.rim;
            basinMaterial = std::max(basinMaterial, visible * material);
        }
        Vec radial;
        if constexpr (withGradient) if (r > 1e-12)
            radial = (offset + impact.along * (2 * ellipticity * transverse)) / (2 * impact.width * r);
        if (bowl > deepest) {
            deepest = bowl;
            if constexpr (withGradient) bowlSlope = radial * (-impact.depth * bowlDerivative);
        }
        if (raised > highest) {
            highest = raised;
            if constexpr (withGradient) {
                const double wallSign = r < 1 ? 1 : -1;
                rimSlope = radial * (wallSign * impact.rim * wallDerivative -
                                     0.18 * impact.rim * ejectaDerivative - impact.peak * peakDerivative -
                                     shieldHeight * shieldDerivative);
            }
        }
    }
    if (craterMaterial) *craterMaterial += (1 - *craterMaterial) * basinMaterial;
    if constexpr (withGradient) gradient = gradient + rimSlope - bowlSlope;
    return relief + highest - deepest;
}

template double Planet::sampleGeology<false>(Vec, Vec&, double*, double) const;
template double Planet::sampleGeology<true>(Vec, Vec&, double*, double) const;

}  // namespace voxel
