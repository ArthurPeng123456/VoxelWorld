#include "cloud_noise.hpp"

#include <algorithm>
#include <cmath>

namespace voxel {
namespace {

std::uint32_t hash(int x, int y, int z, int period, std::uint32_t seed = 42) {
    const auto wrap = [period](int v) { return std::uint32_t((v % period + period) % period); };
    auto h = wrap(x) * 1597334677u ^ wrap(y) * 3812015801u ^ wrap(z) * 2798796415u ^ seed;
    h = (h ^ (h >> 16)) * 2246822519u;
    h = (h ^ (h >> 13)) * 3266489917u;
    return h ^ (h >> 16);
}

float random(std::uint32_t h) { return float(h & 0x00ffffffu) / 16777215.0f; }
float mix(float a, float b, float t) { return a + (b - a) * t; }

float value(float x, float y, float z, int period, std::uint32_t seed = 42) {
    const int ix = int(std::floor(x)), iy = int(std::floor(y)), iz = int(std::floor(z));
    const auto fade = [](float f) { return f * f * f * (f * (f * 6 - 15) + 10); };
    const float tx = fade(x - ix), ty = fade(y - iy), tz = fade(z - iz);
    float planes[2]{};
    for (int dz = 0; dz < 2; ++dz) {
        const float a = mix(random(hash(ix, iy, iz + dz, period, seed)), random(hash(ix + 1, iy, iz + dz, period, seed)), tx);
        const float b = mix(random(hash(ix, iy + 1, iz + dz, period, seed)), random(hash(ix + 1, iy + 1, iz + dz, period, seed)), tx);
        planes[dz] = mix(a, b, ty);
    }
    return mix(planes[0], planes[1], tz);
}

float worley(float x, float y, float z, int period) {
    const int ix = int(std::floor(x)), iy = int(std::floor(y)), iz = int(std::floor(z));
    float nearest = 3;
    for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        const auto h = hash(ix + dx, iy + dy, iz + dz, period);
        const float a = ix + dx + random(h) - x;
        const float b = iy + dy + random(h * 747796405u + 2891336453u) - y;
        const float c = iz + dz + random(h * 277803737u + 1013904223u) - z;
        nearest = std::min(nearest, a * a + b * b + c * c);
    }
    return 1 - std::clamp(std::sqrt(nearest), 0.0f, 1.0f);
}

} // namespace

std::vector<CloudNoiseMip> makeCloudNoise() {
    constexpr unsigned edge = 64;
    std::vector<CloudNoiseMip> result{{edge, std::vector<std::uint8_t>(edge * edge * edge * 4)}};
    for (unsigned z = 0; z < edge; ++z) for (unsigned y = 0; y < edge; ++y) for (unsigned x = 0; x < edge; ++x) {
        const float u = (x + 0.5f) / edge, v = (y + 0.5f) / edge, w = (z + 0.5f) / edge;
        const float p4 = value(u * 4, v * 4, w * 4, 4), p8 = value(u * 8, v * 8, w * 8, 8);
        const float p16 = value(u * 16, v * 16, w * 16, 16);
        const float channels[]{0.75f * p4 + 0.25f * p8,
            0.65f * (0.7f * p8 + 0.3f * p16) + 0.35f * worley(u * 8, v * 8, w * 8, 8),
            worley(u * 16, v * 16, w * 16, 16), value(u * 4, v * 4, w * 4, 4, 731)};
        const auto index = ((z * edge + y) * edge + x) * 4;
        for (unsigned c = 0; c < 4; ++c) result[0].rgba[index + c] = std::uint8_t(std::lround(channels[c] * 255));
    }
    while (result.back().edge > 1) {
        const auto& previous = result.back();
        const unsigned n = previous.edge / 2;
        CloudNoiseMip next{n, std::vector<std::uint8_t>(n * n * n * 4)};
        for (unsigned z = 0; z < n; ++z) for (unsigned y = 0; y < n; ++y) for (unsigned x = 0; x < n; ++x)
            for (unsigned c = 0; c < 4; ++c) {
                unsigned sum = 0;
                for (unsigned dz = 0; dz < 2; ++dz) for (unsigned dy = 0; dy < 2; ++dy) for (unsigned dx = 0; dx < 2; ++dx)
                    sum += previous.rgba[(((2 * z + dz) * previous.edge + 2 * y + dy) * previous.edge + 2 * x + dx) * 4 + c];
                next.rgba[((z * n + y) * n + x) * 4 + c] = std::uint8_t((sum + 4) / 8);
            }
        result.push_back(std::move(next));
    }
    return result;
}

} // namespace voxel
