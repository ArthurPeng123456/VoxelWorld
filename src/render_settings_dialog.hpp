#pragma once

#include "render_settings.hpp"

#include <windows.h>

namespace voxel {

bool editRenderSettings(HWND owner, RenderSettings& settings, const RenderSettings& defaults = {});

} // namespace voxel
