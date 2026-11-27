#pragma once
#include "render_settings.hpp"
#include "terrain.hpp"

namespace voxel {
constexpr int shadowCascadeCount = 4;
struct ShadowCascade {
    std::array<std::array<double, 4>, 3> rows{};
    double farDistance{}, texelSize{}, depthBias{};
    Vec project(Vec relative) const {
        Vec result;
        for (int a = 0; a < 3; ++a)
            result[a] = rows[a][0] * relative.x + rows[a][1] * relative.y + rows[a][2] * relative.z + rows[a][3];
        return result;
    }
};
struct ShadowCascades { std::array<ShadowCascade, shadowCascadeCount> cascades{}; bool enabled{}; };

// CPU fitting and stabilization stay in double. Coordinates are relative to the
// camera; texel snapping uses the planet's frame, independent of world translation.
inline ShadowCascades buildShadowCascades(Vec planetEye, Vec right, Vec up, Vec forward,
                                        double nearPlane, double farPlane, double aspect, Vec sun,
                                        const RenderSettings& settings = {}) {
    ShadowCascades result;
    const double altitude = std::max(0.0, length(planetEye) - planetRadius);
    const double farDistance = std::min({farPlane, 10000 + altitude * 4, settings.shadowDistanceKm * 1000});
    if (nearPlane >= farDistance || nearPlane <= 0 || aspect <= 0) return result;
    result.enabled = true;
    const Vec z = unit(sun) * -1;
    const Vec x = unit(cross(std::abs(z.z) < 0.9 ? Vec{0,0,1} : Vec{0,1,0}, z));
    const Vec y = cross(z, x);
    // A small uniform contribution over a planetary shadow distance otherwise
    // consumes the entire first map on hundreds of meters. Reserve a close
    // cascade for meter-scale objects and spread the other ceilings out to far.
    // Keep the interval valid when the camera's near plane is already distant.
    const double nearRange = std::min(farDistance, std::max(settings.shadowNearDistanceM, nearPlane * 2));
    double previous = nearPlane;
    for (int i = 0; i < shadowCascadeCount; ++i) {
        const double t = double(i + 1) / shadowCascadeCount;
        const double practical = settings.shadowSplitBlend * nearPlane * std::pow(farDistance / nearPlane, t)
                               + (1 - settings.shadowSplitBlend) * (nearPlane + (farDistance - nearPlane) * t);
        const double ceiling = nearRange * std::pow(farDistance / nearRange, double(i) / (shadowCascadeCount - 1));
        const double split = i == shadowCascadeCount - 1 ? farDistance : std::min(practical, ceiling);
        // Overlap the fitted intervals for smooth blending at cascade boundaries.
        const double first = i ? previous - (previous - nearPlane) * 0.12 : nearPlane;
        const Vec center = forward * ((first + split) * 0.5);
        std::array<Vec, 8> corners;
        for (int c = 0; c < 8; ++c) {
            const double d = c & 4 ? split : first;
            corners[c] = forward * d + right * ((c & 1 ? 1 : -1) * d * aspect / 1.92)
                         + up * ((c & 2 ? 1 : -1) * d / 1.92);
        }
        double radius = 0;
        for (Vec p : corners) radius = std::max(radius, length(p - center));
        radius = std::ceil(radius * 16) / 16;
        const double border = 2 + 2 * std::max(1.0, settings.shadowNormalBias);
        radius *= 1 + border / settings.shadowResolution; // Reserve a PCF / texel-snapping border.
        const double texel = 2 * radius / settings.shadowResolution;
        const Vec planetCenter = planetEye + center;
        const double cx = std::round(dot(planetCenter, x) / texel) * texel - dot(planetEye, x);
        const double cy = std::round(dot(planetCenter, y) / texel) * texel - dot(planetEye, y);
        const double extent = radius + std::max(10000.0, radius);
        const double lowZ = dot(center, z) - extent;
        auto& cascade = result.cascades[i];
        cascade.rows = {{{x.x / radius, x.y / radius, x.z / radius, -cx / radius},
                         {y.x / radius, y.y / radius, y.z / radius, -cy / radius},
                         {z.x / (2 * extent), z.y / (2 * extent), z.z / (2 * extent), -lowZ / (2 * extent)}}};
        cascade.farDistance = split;
        cascade.texelSize = texel;
        cascade.depthBias = settings.shadowDepthBias * 0.1 * texel / (2 * extent);
        previous = split;
    }
    return result;
}
} // namespace voxel
