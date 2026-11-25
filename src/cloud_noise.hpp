#pragma once

#include <cstdint>
#include <vector>

namespace voxel {

struct CloudNoiseMip {
    unsigned edge;
    std::vector<std::uint8_t> rgba;
};

// Periodic, deterministic volume: weather noise, smooth value/Worley shape,
// fine erosion and an independent low-frequency weather channel.
std::vector<CloudNoiseMip> makeCloudNoise();

} // namespace voxel
