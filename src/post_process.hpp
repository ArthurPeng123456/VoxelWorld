#pragma once
#include <algorithm>
#include <cmath>

namespace voxel {
// Narkowicz's ACES fit, followed by the sRGB output transfer function.
inline double toneMap(double radiance, double exposureEV) {
    const double x = std::max(0.0, radiance) * std::exp2(exposureEV);
    const double mapped = std::clamp(x * (2.51 * x + 0.03) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
    return mapped <= 0.0031308 ? 12.92 * mapped : 1.055 * std::pow(mapped, 1 / 2.4) - 0.055;
}
} // namespace voxel
