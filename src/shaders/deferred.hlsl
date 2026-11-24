Texture2D<float4> gAlbedo : register(t0);
Texture2D<float4> gNormal : register(t1);
Texture2D<float> gDepth : register(t2);
Texture2DArray<float> shadowMaps : register(t3);
Texture2D<float4> hdrScene : register(t4);
SamplerComparisonState shadowSampler : register(s0);
cbuffer ShadowPass : register(b1) { uint cascadeIndex; };

struct GBuffer { float4 albedo : SV_Target0; float4 normal : SV_Target1; };
GBuffer surfacePS(Surface input) {
    GBuffer result;
    result.albedo = input.material;
    int code = int(round(input.material.a * 255));
    if (code < 253 && sunlight.w > 0.5) {
        if (code == 0) result.albedo.rgb = float3(0.19, 0.72, 0.83);
        if (code == 1) result.albedo.rgb = float3(0.28, 0.63, 0.38);
        if (code == 2) result.albedo.rgb = float3(0.81, 0.63, 0.29);
        if (code == 3) result.albedo.rgb = float3(0.53, 0.43, 0.64);
        if (code >= 4) result.albedo.rgb = lerp(float3(0.95, 0.35, 0.22), float3(0.12, 0.22, 0.46), saturate((code - 4) / 8.0));
    }
    result.normal = float4(normalize(input.normal), code == 253 ? 0.35 : code == 254 ? 0.5 : 0.85);
    return result;
}

float4 shadowVS(float3 hi : POSITION0, float3 lo : POSITION1) : SV_Position {
    DF3 translated = df3Sub(df3(hi, lo), df3(cameraEyeHi.xyz, cameraEyeLo.xyz));
    float4 relative = float4(translated.hi + translated.lo, 1);
    uint first = cascadeIndex * 3;
    return float4(dot(shadowRows[first], relative), dot(shadowRows[first + 1], relative),
                  dot(shadowRows[first + 2], relative), 1);
}
float shadowAt(uint index, float3 relative, float3 normal) {
    float normalBias = shadowTexels[index] * (0.35 + 0.65 * (1 - saturate(dot(normal, sunlight.xyz))));
    float4 p = float4(relative + normal * normalBias, 1);
    uint first = index * 3;
    float3 shadow = float3(dot(shadowRows[first], p), dot(shadowRows[first + 1], p), dot(shadowRows[first + 2], p));
    float2 uv = shadow.xy * float2(0.5, -0.5) + 0.5;
    if (any(uv < 0) || any(uv > 1) || shadow.z < 0 || shadow.z > 1) return 1;
    float sum = 0;
    [unroll] for (int y = -1; y <= 1; ++y) [unroll] for (int x = -1; x <= 1; ++x)
        sum += shadowMaps.SampleCmpLevelZero(shadowSampler,
            float3(uv + float2(x, y) * shadowSettings.y, index), shadow.z - shadowBiases[index]);
    return sum / 9;
}
float visibility(float3 relative, float3 normal, float depth) {
    if (shadowSettings.x < 0.5 || depth > shadowSplits.w) return 1;
    uint index = depth <= shadowSplits.x ? 0 : depth <= shadowSplits.y ? 1 : depth <= shadowSplits.z ? 2 : 3;
    float value = shadowAt(index, relative, normal);
    float previous = index ? shadowSplits[index - 1] : projection.z;
    float blend = smoothstep(0.9, 1, (depth - previous) / max(1, shadowSplits[index] - previous));
    [branch] if (blend <= 0) return value;
    float next = index < 3 ? shadowAt(min(index + 1, 3u), relative, normal) : 1;
    return lerp(value, next, blend);
}

float3 skyRadiance(float3 ray) {
    float2 ground;
    bool hit = dfRaySphere(planetEye(), ray, float2(atmosphere.x - 1000, 0), ground) && ground.x > 0;
    float3 transmittance;
    float3 inscatter = scatter(planetEye(), ray, hit ? ground.x : projection.w, transmittance);
    float3 background = hit ? 0 : float3(0.0008, 0.0012, 0.002);
    if (!hit) background += 20 * smoothstep(cos(0.0048), cos(0.0044), dot(ray, sunlight.xyz));
    return background * transmittance + inscatter;
}
float4 deferredPS(Sky input) : SV_Target {
    int2 pixel = int2(input.clip.xy);
    float depth = gDepth.Load(int3(pixel, 0));
    float2 ndc = input.clip.xy / renderParams.xy * float2(2, -2) + float2(-1, 1);
    float3 ray = cameraForward.xyz + cameraRight.xyz * (ndc.x / projection.x) + cameraUp.xyz * (ndc.y / projection.y);
    float4 normalData = gNormal.Load(int3(pixel, 0));
    if (depth <= 0 || normalData.w <= 0) return float4(skyRadiance(normalize(ray)), 1);
    float4 material = gAlbedo.Load(int3(pixel, 0));
    int code = int(round(material.a * 255));
    if (code == 255) return float4(material.rgb * 4, 1);
    float viewDepth = projection.z / depth;
    float3 relative = ray * viewDepth;
    float3 normal = normalize(normalData.xyz);
    float diffuse = max(0, dot(normal, sunlight.xyz));
    float shadow = visibility(relative, normal, viewDepth);
    float3 eyeDirection = -normalize(relative);
    float3 halfVector = normalize(sunlight.xyz + eyeDirection);
    float specular = pow(saturate(dot(normal, halfVector)), lerp(96, 8, normalData.w)) * 0.08;
    float3 radiance = material.rgb * (0.12 + 1.8 * diffuse * shadow) + specular * shadow;
    if (code == 253) {
        float facing = saturate(dot(normal, eyeDirection));
        float rim = 1 - facing;
        radiance += material.rgb * (0.22 * facing + 0.09 * rim * rim);
    }
    float distance = length(relative);
    float3 transmittance;
    float3 inscatter = scatter(planetEye(), relative / max(distance, 1e-6), distance, transmittance);
    return float4(radiance * transmittance + inscatter, 1);
}
float4 linePS(Surface input) : SV_Target { return float4(input.material.rgb * 2, 1); }

// ACES filmic fit (Narkowicz), then linear -> sRGB. Applied exactly once.
float toSrgb(float x) { return x <= 0.0031308 ? 12.92 * x : 1.055 * pow(x, 1.0 / 2.4) - 0.055; }
float3 acesToneMap(float3 radiance, float exposure) {
    float3 x = max(radiance, 0) * exposure;
    float3 mapped = saturate((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14));
    return float3(toSrgb(mapped.x), toSrgb(mapped.y), toSrgb(mapped.z));
}
float4 toneMapPS(Sky input) : SV_Target {
    return float4(acesToneMap(hdrScene.Load(int3(int2(input.clip.xy), 0)).rgb, renderParams.z), 1);
}
