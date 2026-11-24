#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace voxel {
// Narkowicz's ACES fit, followed by the sRGB output transfer function.
inline double toneMap(double radiance, double exposureEV) {
    const double x = std::max(0.0, radiance) * std::exp2(exposureEV);
    const double mapped = std::clamp(x * (2.51 * x + 0.03) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
    return mapped <= 0.0031308 ? 12.92 * mapped : 1.055 * std::pow(mapped, 1 / 2.4) - 0.055;
}
template<class Text> double parseExposure(const Text& text) {
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size() && std::isfinite(value) && value >= -10 && value <= 10) return value;
    } catch (const std::exception&) {}
    throw std::invalid_argument("--exposure requires a finite EV value from -10 to 10");
}
} // namespace voxel
