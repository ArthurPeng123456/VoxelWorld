#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace voxel {

inline constexpr std::array<int, 3> shadowResolutions{1024, 2048, 4096};
inline constexpr double referenceAtmosphereHeightKm = 100;
constexpr bool validShadowResolution(int resolution) {
    for (int supported : shadowResolutions) if (resolution == supported) return true;
    return false;
}

enum class ToneMapping { Aces, Reinhard, Linear };

struct RenderSettings {
    bool shadowsEnabled{true};
    int shadowResolution{shadowResolutions.front()};
    double shadowDistanceKm{200}, shadowSplitBlend{0.85};
    double shadowNormalBias{1}, shadowDepthBias{1};
    bool atmosphereEnabled{true};
    double atmosphereHeightKm{referenceAtmosphereHeightKm}, rayleighHeightKm{8}, mieHeightKm{1.2};
    double rayleighStrength{1}, mieStrength{1}, mieAnisotropy{0.76};
    double exposureEV{};
    ToneMapping toneMapping{ToneMapping::Aces};
};

struct AtmosphereDimensions { double thickness, rayleighScale, mieScale; };

constexpr AtmosphereDimensions atmosphereDimensions(const RenderSettings& settings) {
    // Stretch the density profile and its outer boundary together. The scale
    // height controls describe the reference profile at 100 km thickness.
    const double scale = 1000 * settings.atmosphereHeightKm / referenceAtmosphereHeightKm;
    return {1000 * settings.atmosphereHeightKm, scale * settings.rayleighHeightKm, scale * settings.mieHeightKm};
}

struct RenderParameter {
    const wchar_t* option;
    const wchar_t* label;
    double RenderSettings::*value;
    double minimum, maximum;
};

inline constexpr std::array shadowParameters{
    RenderParameter{L"--shadow-distance-km", L"Distance cap (km)", &RenderSettings::shadowDistanceKm, 1, 1000},
    RenderParameter{L"--shadow-split-blend", L"Log split weight", &RenderSettings::shadowSplitBlend, 0, 1},
    RenderParameter{L"--shadow-normal-bias", L"Normal bias multiplier", &RenderSettings::shadowNormalBias, 0, 8},
    RenderParameter{L"--shadow-depth-bias", L"Depth bias multiplier", &RenderSettings::shadowDepthBias, 0, 8}
};
inline constexpr std::array atmosphereParameters{
    RenderParameter{L"--atmosphere-height-km", L"Thickness (km)", &RenderSettings::atmosphereHeightKm, 1, 1000},
    RenderParameter{L"--rayleigh-height-km", L"Rayleigh scale (km at 100 km)", &RenderSettings::rayleighHeightKm, 0.1, 100},
    RenderParameter{L"--mie-height-km", L"Mie scale (km at 100 km)", &RenderSettings::mieHeightKm, 0.1, 100},
    RenderParameter{L"--rayleigh-strength", L"Rayleigh strength", &RenderSettings::rayleighStrength, 0, 8},
    RenderParameter{L"--mie-strength", L"Mie strength", &RenderSettings::mieStrength, 0, 8},
    RenderParameter{L"--mie-anisotropy", L"Mie anisotropy", &RenderSettings::mieAnisotropy, -0.95, 0.95}
};
inline constexpr std::array hdrParameters{
    RenderParameter{L"--exposure", L"Exposure (EV)", &RenderSettings::exposureEV, -10, 10}
};

inline std::optional<double> readRenderParameter(const std::wstring& text, const RenderParameter& parameter) {
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size() && std::isfinite(value) && value >= parameter.minimum && value <= parameter.maximum)
            return value;
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {}
    return {};
}

} // namespace voxel
