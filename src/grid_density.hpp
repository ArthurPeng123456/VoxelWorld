#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace voxel {
inline constexpr std::array<double, 5> voxelDensities{{0.125, 0.25, 0.5, 1.0, 2.0}};
inline int voxelDensityShift(double value) {
    for (int i = 0; i < int(voxelDensities.size()); ++i)
        if (value == voxelDensities[i]) return i - 3;
    throw std::invalid_argument("Voxel density must be 0.125, 0.25, 0.5, 1 or 2 (sampling-spacing multiplier)");
}
template<class Text> double parseVoxelDensity(const Text& text) {
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size()) { voxelDensityShift(value); return value; }
    } catch (const std::exception&) {}
    throw std::invalid_argument("--voxel-density requires 0.125, 0.25, 0.5, 1 or 2");
}
} // namespace voxel
