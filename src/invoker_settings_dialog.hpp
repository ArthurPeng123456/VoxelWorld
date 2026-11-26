#pragma once

#include "invoker_settings.hpp"
#include <windows.h>

namespace voxel {
bool editInvokerSettings(HWND owner, InvokerSettings& settings);
} // namespace voxel
