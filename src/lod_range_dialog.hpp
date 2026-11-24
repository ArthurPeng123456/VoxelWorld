#pragma once

#include "terrain.hpp"

#include <windows.h>

namespace voxel {

bool editLodRanges(HWND owner, std::uint32_t& lod0Radius, int densityShift);

} // namespace voxel
