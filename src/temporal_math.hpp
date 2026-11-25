#pragma once

#include "render_settings.hpp"
#include "render_coordinates.hpp"

namespace voxel {

inline double radicalInverse(unsigned index, unsigned base) {
    double result = 0, scale = 1;
    while (index) { scale /= base; result += (index % base) * scale; index /= base; }
    return result;
}

inline std::array<double, 2> temporalJitter(unsigned frame, const RenderSettings& settings) {
    if (!settings.taaEnabled) return {};
    const unsigned sample = frame % unsigned(settings.taaSamples) + 1;
    return {(radicalInverse(sample, 2) - 0.5) * settings.taaJitter,
            (radicalInverse(sample, 3) - 0.5) * settings.taaJitter};
}

inline bool temporalCameraCut(Vec eye, Vec previousEye, Vec forward, Vec previousForward,
                               Vec up, Vec previousUp, const RenderSettings& settings) {
    const double cosine = std::cos(settings.taaCutAngle * 0.017453292519943295);
    return length(eye - previousEye) > settings.taaCutDistanceKm * 1000 ||
           dot(forward, previousForward) < cosine || dot(up, previousUp) < cosine;
}

} // namespace voxel
