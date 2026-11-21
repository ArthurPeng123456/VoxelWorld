#pragma once

#include "terrain.hpp"

#include <windows.h>

#include <filesystem>

namespace voxel {

struct Vertex {
    float position[3];
    float normal[3];
    std::uint32_t color;
};

struct Camera {
    Vec eye, right, up, forward;
    double nearPlane, farPlane;
    bool lodTint, wireframe;
};

class Dx12Renderer {
public:
    Dx12Renderer(HWND window, bool forceWarp);
    ~Dx12Renderer();
    void setTerrain(const std::vector<Vertex>& terrain);
    void setClipmapLines(const std::vector<Vertex>& lines);
    void draw(const Camera& camera, const std::vector<Vertex>& objects, const std::filesystem::path& capture = {});
    bool isWarp() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace voxel
