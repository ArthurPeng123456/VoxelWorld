#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace voxel {

inline constexpr std::array<int, 3> shadowResolutions{1024, 2048, 4096};
inline constexpr double referenceAtmosphereHeightKm = 100;
inline constexpr std::array<int, 3> cloudStepCounts{32, 64, 96};
inline constexpr std::array<int, 3> taaSampleCounts{8, 16, 32};
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
    bool cloudsEnabled{true};
    int cloudSteps{64};
    double cloudBaseKm{6}, cloudThicknessKm{8}, cloudCoverage{0.5}, cloudDensity{1};
    double cloudWeatherKm{1400}, cloudFormationKm{45}, cloudWindSpeed{25};
    bool taaEnabled{true}, cloudTemporalEnabled{true};
    int taaSamples{16}, cloudSampling{1}; // Full / two-phase checkerboard / four-phase 2x2.
    double taaHistoryWeight{0.9}, taaJitter{1}, taaVarianceGamma{1.25}, taaDepthThreshold{0.01};
    double taaMotionResponse{0.2}, taaSharpen{0.15}, taaCutDistanceKm{200}, taaCutAngle{45};
    double cloudHistoryWeight{0.92}, cloudDepthThreshold{0.2}, cloudRayJitter{0.85};
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
inline constexpr std::array cloudParameters{
    RenderParameter{L"--cloud-base-km", L"Base altitude (km)", &RenderSettings::cloudBaseKm, 0.1, 200},
    RenderParameter{L"--cloud-thickness-km", L"Thickness (km)", &RenderSettings::cloudThicknessKm, 0.2, 100},
    RenderParameter{L"--cloud-coverage", L"Weather coverage", &RenderSettings::cloudCoverage, 0, 1},
    RenderParameter{L"--cloud-density", L"Extinction multiplier", &RenderSettings::cloudDensity, 0, 4},
    RenderParameter{L"--cloud-weather-km", L"Weather scale (km)", &RenderSettings::cloudWeatherKm, 300, 4000},
    RenderParameter{L"--cloud-formation-km", L"Formation scale (km)", &RenderSettings::cloudFormationKm, 5, 150},
    RenderParameter{L"--cloud-wind-speed", L"Wind speed (m/s)", &RenderSettings::cloudWindSpeed, 0, 300}
};
inline constexpr std::array taaParameters{
    RenderParameter{L"--taa-history", L"History weight", &RenderSettings::taaHistoryWeight, 0, 0.98},
    RenderParameter{L"--taa-jitter", L"Jitter spread (pixels)", &RenderSettings::taaJitter, 0, 1.5},
    RenderParameter{L"--taa-variance", L"Variance clip gamma", &RenderSettings::taaVarianceGamma, 0.5, 3},
    RenderParameter{L"--taa-depth-threshold", L"Depth rejection ratio", &RenderSettings::taaDepthThreshold, 0.0005, 0.1},
    RenderParameter{L"--taa-motion-response", L"Motion response", &RenderSettings::taaMotionResponse, 0, 1},
    RenderParameter{L"--taa-sharpen", L"Output sharpening", &RenderSettings::taaSharpen, 0, 1},
    RenderParameter{L"--taa-cut-distance-km", L"Camera cut distance (km)", &RenderSettings::taaCutDistanceKm, 1, 2000},
    RenderParameter{L"--taa-cut-angle", L"Camera cut angle (deg)", &RenderSettings::taaCutAngle, 10, 120}
};
inline constexpr std::array cloudTemporalParameters{
    RenderParameter{L"--cloud-history", L"History weight", &RenderSettings::cloudHistoryWeight, 0, 0.98},
    RenderParameter{L"--cloud-depth-threshold", L"Depth rejection ratio", &RenderSettings::cloudDepthThreshold, 0.01, 0.75},
    RenderParameter{L"--cloud-ray-jitter", L"Ray jitter strength", &RenderSettings::cloudRayJitter, 0, 1}
};

inline bool sameHistorySettings(const RenderSettings& a, const RenderSettings& b) {
    if (a.shadowsEnabled != b.shadowsEnabled || a.shadowResolution != b.shadowResolution ||
        a.atmosphereEnabled != b.atmosphereEnabled || a.cloudsEnabled != b.cloudsEnabled ||
        a.cloudSteps != b.cloudSteps || a.taaEnabled != b.taaEnabled || a.taaSamples != b.taaSamples ||
        a.cloudTemporalEnabled != b.cloudTemporalEnabled || a.cloudSampling != b.cloudSampling) return false;
    const auto equal = [&](const auto& parameters) {
        for (const auto& parameter : parameters) if (a.*parameter.value != b.*parameter.value) return false;
        return true;
    };
    // Histories store unexposed linear HDR; exposure and tone mapping are post effects.
    return equal(shadowParameters) && equal(atmosphereParameters) && equal(cloudParameters) &&
           equal(taaParameters) && equal(cloudTemporalParameters);
}

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
