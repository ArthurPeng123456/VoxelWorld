#pragma once

#include "render_coordinates.hpp"
#include "render_settings.hpp"

#include <windows.h>

#include <filesystem>

namespace voxel {

struct Camera {
    Vec eye, right, up, forward;
    double nearPlane, farPlane;
    bool lodTint, wireframe;
    Vec planetCenter{};
    RenderSettings settings{};
    TerrainPreset terrain{TerrainPreset::Classic};
    std::uint32_t terrainSeed{};
    double timeSeconds{};
    Vec sunDirection{0.45, 0.25, 0.86};
};

struct RenderDiagnostics {
    double maxHdrComponent{};
    std::size_t geometryPixels{}, invalidValues{};
    std::array<std::size_t, 4> shadowPixels{};
    unsigned shadowResolution{};
    bool debugLayer{};
    std::size_t cloudPixels{};
    double maxCloudOpacity{};
    std::size_t taaHistoryPixels{}, cloudTracedPixels{}, cloudSkippedPixels{}, cloudFallbackPixels{};
    std::uint64_t cloudRaySteps{};
    bool temporalHistoryValid{};
    std::size_t movingObjectPixels{};
    double maxObjectMotionPixels{};
};

struct CloudGpuTiming {
    double integrationMs{}, reconstructionMs{};
};

class Dx12Renderer {
public:
    Dx12Renderer(HWND window, bool forceWarp, bool debug = false);
    ~Dx12Renderer();
    void setTerrain(const std::vector<Vertex>& terrain);
    void clearTerrain();
    void stageTerrainPage(Key page, Box bounds, const std::vector<GpuVertex>& vertices);
    bool terrainUploadsReady() const;
    void publishTerrainPages();
    void setClipmapLines(const std::vector<Vertex>& lines);
    void setShadowResolution(int resolution);
    void resetTemporalHistory();
    void enableCloudGpuTiming();
    void draw(const Camera& camera, const std::vector<Vertex>& objects, const std::filesystem::path& capture = {}, bool inspectPipeline = false);
    RenderDiagnostics diagnostics() const;
    CloudGpuTiming cloudGpuTiming() const;
    bool isWarp() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
