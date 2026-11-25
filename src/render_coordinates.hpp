#pragma once
#include "terrain.hpp"

namespace voxel {
inline Vec lodColor(int lod) {
    if (lod == 0) return {0.19, 0.72, 0.83};
    if (lod == 1) return {0.28, 0.63, 0.38};
    if (lod == 2) return {0.81, 0.63, 0.29};
    if (lod == 3) return {0.53, 0.43, 0.64};
    const double t = std::clamp((lod - 4) / 8.0, 0.0, 1.0);
    return Vec{0.95, 0.35, 0.22} * (1 - t) + Vec{0.12, 0.22, 0.46} * t;
}
// Split only at the GPU boundary. All application geometry remains double.
struct FloatPair { float hi, lo; };
inline FloatPair splitDouble(double value) {
    const float hi = static_cast<float>(value);
    return {hi, static_cast<float>(value - static_cast<double>(hi))};
}
struct Vertex { Vec position, normal; std::uint32_t color; std::uint64_t motionId{}; };
struct GpuVertex { float hi[3], lo[3], normal[3]; std::uint32_t color; };
inline GpuVertex packVertex(const Vertex& vertex) {
    GpuVertex result{};
    for (int i = 0; i < 3; ++i) {
        const auto pair = splitDouble(vertex.position[i]);
        result.hi[i] = pair.hi;
        result.lo[i] = pair.lo;
        result.normal[i] = static_cast<float>(vertex.normal[i]);
    }
    result.color = vertex.color;
    return result;
}
}
