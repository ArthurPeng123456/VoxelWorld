cbuffer Camera : register(b0) {
    float4 cameraEyeHi, cameraEyeLo;
    float4 cameraRight, cameraUp, cameraForward; // cameraRight.w: atmosphere enabled
    float4 projection; // x/y scale, near plane, far plane (reverse Z)
    float4 sunlight;
    float4 planetEyeHi, planetEyeLo; // camera minus planet center, split on CPU in double
    float4 atmosphere; // ground radius, top radius, Rayleigh/Mie scale heights (meters)
    float4 shadowRows[12];
    float4 shadowSplits, shadowTexels, shadowBiases;
    float4 shadowSettings; // enabled, inverse resolution
    float4 renderParams; // width, height, exposure multiplier, wireframe
};

DF3 planetEye() { return df3(planetEyeHi.xyz, planetEyeLo.xyz); }
float3 betaR() { return float3(5.802e-6, 13.558e-6, 33.100e-6); }
float betaM() { return 3.996e-6; }
float2 density(DF3 p) {
    // The cancellation must happen before demotion to float.
    float h = max(0, dfAltitude(p, float2(atmosphere.x, 0)));
    return exp(-h / atmosphere.zw);
}
float3 extinction(float2 optical) { return betaR() * optical.x + (betaM() / 0.9) * optical.y; }
float2 sunlightDepth(DF3 p) {
    float2 ground, top;
    if (dfRaySphere(p, sunlight.xyz, float2(atmosphere.x, 0), ground) && ground.x > 0.01)
        return float2(1e20, 1e20);
    if (!dfRaySphere(p, sunlight.xyz, float2(atmosphere.y, 0), top)) return 0;
    float step = max(0, top.y) / 6;
    float2 optical = 0;
    [loop] for (int i = 0; i < 6; ++i)
        optical += density(df3Add(p, df3Scale(sunlight.xyz, (i + 0.5) * step))) * step;
    return optical;
}
// Single scattering with Rayleigh + Cornette-Shanks Mie phase functions.
// Geometry is df64; bounded optical depths, exponentials and radiance use float.
float3 scatter(DF3 eye, float3 ray, float limit, out float3 transmittance) {
    float2 interval;
    transmittance = 1;
    if (cameraRight.w < 0.5) return 0;
    if (!dfRaySphere(eye, ray, float2(atmosphere.y, 0), interval)) return 0;
    float start = max(0, interval.x), end = min(limit, interval.y);
    if (end <= start) return 0;
    float mu = dot(ray, sunlight.xyz), g = 0.76;
    float phaseR = 3.0 / (16.0 * 3.14159265) * (1 + mu * mu);
    float phaseM = 3.0 / (8.0 * 3.14159265) * (1 - g * g) * (1 + mu * mu)
                 / ((2 + g * g) * pow(max(1e-3, 1 + g * g - 2 * g * mu), 1.5));
    float2 optical = 0;
    float3 sumR = 0, sumM = 0;
    [loop] for (int i = 0; i < 16; ++i) {
        // Concentrate at both ends: near-ground density must also be resolved from orbit.
        float u0 = float(i) / 16, u1 = float(i + 1) / 16;
        float s0 = 0.5 - 0.5 * cos(3.14159265 * u0), s1 = 0.5 - 0.5 * cos(3.14159265 * u1);
        float dt = (end - start) * (s1 - s0);
        float t = start + (end - start) * 0.5 * (s0 + s1);
        DF3 p = df3Add(eye, df3Scale(ray, t));
        float2 local = density(p) * dt;
        float3 attenuation = exp(-extinction(optical + local * 0.5 + sunlightDepth(p)));
        sumR += attenuation * local.x;
        sumM += attenuation * local.y;
        optical += local;
    }
    transmittance = exp(-extinction(optical));
    return 20 * (sumR * betaR() * phaseR + sumM * betaM() * phaseM);
}

struct Surface {
    float4 clip : SV_Position;
    float3 normal : NORMAL;
    float4 material : COLOR;
    float3 relative : TEXCOORD0;
    float farClip : SV_ClipDistance0;
};
Surface surfaceVS(float3 hi : POSITION0, float3 lo : POSITION1, float3 normal : NORMAL, float4 material : COLOR) {
    // UE translated-world pattern: subtract split world positions, THEN use float.
    DF3 translated = df3Sub(df3(hi, lo), df3(cameraEyeHi.xyz, cameraEyeLo.xyz));
    float3 relative = translated.hi + translated.lo;
    float depth = dot(relative, cameraForward.xyz);
    Surface result;
    result.clip = float4(dot(relative, cameraRight.xyz) * projection.x,
                         dot(relative, cameraUp.xyz) * projection.y, projection.z, depth);
    result.relative = relative;
    result.farClip = projection.w - depth;
    result.normal = normal;
    result.material = material;
    return result;
}
struct Sky { float4 clip : SV_Position; float2 position : TEXCOORD; };
Sky skyVS(uint id : SV_VertexID) {
    Sky result;
    result.position = float2(id == 1 ? 3.0 : -1.0, id == 2 ? 3.0 : -1.0);
    result.clip = float4(result.position, 0, 1);
    return result;
}
