#pragma once

#include "render_coordinates.hpp"

#include <windows.h>

#include <filesystem>

namespace voxel {

struct Camera {
    Vec eye, right, up, forward;
    double nearPlane, farPlane;
    bool lodTint, wireframe;
    Vec planetCenter{};
    bool atmosphereEnabled{true};
    double exposureEV{};
    bool shadowsEnabled{true};
};

struct RenderDiagnostics {
    double maxHdrComponent{};
    std::size_t geometryPixels{}, invalidValues{};
    std::array<std::size_t, 4> shadowPixels{};
    bool debugLayer{};
};

class Dx12Renderer {
public:
    Dx12Renderer(HWND window, bool forceWarp, bool debug = false);
    ~Dx12Renderer();
    void setTerrain(const std::vector<Vertex>& terrain);
    void setClipmapLines(const std::vector<Vertex>& lines);
    void draw(const Camera& camera, const std::vector<Vertex>& objects, const std::filesystem::path& capture = {}, bool inspectPipeline = false);
    RenderDiagnostics diagnostics() const;
    bool isWarp() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
