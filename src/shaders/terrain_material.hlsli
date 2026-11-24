float terrainHash(int3 cell, int mask, uint seed) {
    uint3 p = uint3(cell & mask);
    uint state = p.x * 1597334677u ^ p.y * 3812015801u ^ p.z * 2798796415u ^ (seed * 747796405u);
    state = (state ^ (state >> 16)) * 2246822519u;
    state = (state ^ (state >> 13)) * 3266489917u;
    return float((state ^ (state >> 16)) & 0x00ffffffu) / 16777215.0;
}

float terrainNoise(float3 p, int mask, uint seed, out float3 gradient) {
    int3 cell = int3(floor(p));
    float3 f = frac(p);
    float3 t = f * f * (3 - 2 * f);
    float a = terrainHash(cell, mask, seed), b = terrainHash(cell + int3(1,0,0), mask, seed);
    float c = terrainHash(cell + int3(0,1,0), mask, seed), d = terrainHash(cell + int3(1,1,0), mask, seed);
    float e = terrainHash(cell + int3(0,0,1), mask, seed), g = terrainHash(cell + int3(1,0,1), mask, seed);
    float h = terrainHash(cell + int3(0,1,1), mask, seed), i = terrainHash(cell + int3(1,1,1), mask, seed);
    float4 x = lerp(float4(a,c,e,h), float4(b,d,g,i), t.x);
    float2 y = lerp(x.xz, x.yw, t.y);
    gradient = float3(lerp(lerp(b-a, d-c, t.y), lerp(g-e, i-h, t.y), t.z),
                      lerp(x.y-x.x, x.w-x.z, t.z), y.y-y.x) * (6 * f * (1 - f));
    return lerp(y.x, y.y, t.z);
}

float3 geologyMaterial(float3 radial, float3 relative, float3 profile, float metersPerPixel,
                       float surfaceDetail, inout float3 normal, inout float roughness) {
    const bool mars = terrainParams.x < 1.5;
    const uint seed = (uint)terrainParams.y;
    float3 ignored;
    float province = terrainNoise(radial * 13.0 + float3(23, 41, 11), 1023, seed, ignored);
    float region = terrainNoise(radial * 49.0 + float3(7, 37, 19), 1023, seed + 17, ignored);
    float elevation = profile.x;
    float rock = saturate(profile.y * 2.4 * surfaceDetail);
    float mottling = province * 0.68 + region * 0.32;
    float3 albedo;
    if (mars) {
        float3 ferricDust = lerp(float3(0.30, 0.083, 0.037), float3(0.47, 0.205, 0.090), elevation);
        float basalt = smoothstep(0.39, 0.65, mottling * 0.76 + (1 - elevation) * 0.24);
        albedo = lerp(ferricDust, float3(0.205, 0.093, 0.050), basalt * 0.82);
        albedo = lerp(albedo, float3(0.145, 0.074, 0.048), rock * 0.58);
        albedo = lerp(albedo, float3(0.162, 0.070, 0.041), saturate(profile.z) * 0.68);
        roughness = lerp(0.94, 0.72, rock);
    } else {
        float ash = smoothstep(0.31, 0.7, mottling * 0.7 + elevation * 0.3);
        albedo = lerp(float3(0.047, 0.055, 0.067), float3(0.19, 0.151, 0.121), ash);
        float oxidized = smoothstep(0.53, 0.75, province) * (1 - elevation);
        albedo = lerp(albedo, float3(0.235, 0.084, 0.041), oxidized * 0.85);
        albedo = lerp(albedo, float3(0.037, 0.041, 0.053), rock * 0.52);
        roughness = lerp(0.82, 0.58, rock);
    }

    [branch] if (metersPerPixel < 42) {
        // Wrap the high part before adding the nearby relative position. The
        // hash period matches the 4096 m wrap at every scale, including negative coordinates.
        float3 local = fmod(planetEyeHi.xyz, 4096.0) + planetEyeLo.xyz + relative;
        float3 slope;
        float patches = terrainNoise(local / 64, 63, seed + 31, ignored);
        float stones = terrainNoise(local / 16, 255, seed + 41, ignored);
        float grains = terrainNoise(local / 4, 1023, seed + 53, slope);
        float patchWeight = 1 - smoothstep(13, 42, metersPerPixel);
        float stoneWeight = 1 - smoothstep(3.5, 11, metersPerPixel);
        float grainWeight = 1 - smoothstep(0.85, 2.8, metersPerPixel);
        float variation = (patches - 0.5) * 0.46 * patchWeight + (stones - 0.5) * 0.32 * stoneWeight +
                          (grains - 0.5) * 0.23 * grainWeight;
        albedo *= 1 + variation;
        roughness = saturate(roughness - (stones - 0.5) * 0.17 * stoneWeight);
        float3 tangent = slope - normal * dot(normal, slope);
        normal = normalize(normal - tangent * (mars ? 0.095 : 0.155) * grainWeight);
    }
    return albedo;
}
