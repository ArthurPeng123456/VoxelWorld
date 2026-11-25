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
};

struct RenderDiagnostics {
    double maxHdrComponent{};
    std::size_t geometryPixels{}, invalidValues{};
    std::array<std::size_t, 4> shadowPixels{};
    unsigned shadowResolution{};
    bool debugLayer{};
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
    void draw(const Camera& camera, const std::vector<Vertex>& objects, const std::filesystem::path& capture = {}, bool inspectPipeline = false);
    RenderDiagnostics diagnostics() const;
    bool isWarp() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
