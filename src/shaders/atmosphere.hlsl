cbuffer Camera : register(b0) {
    float4 cameraEyeHi, cameraEyeLo;
    float4 cameraRight, cameraUp, cameraForward; // cameraRight.w: atmosphere enabled
    float4 projection; // x/y scale, near plane, far plane (reverse Z)
    float4 sunlight;
    float4 planetEyeHi, planetEyeLo; // camera minus planet center, split on CPU in double
    float4 atmosphere; // ground radius, top radius, Rayleigh/Mie scale heights (meters)
    float4 shadowRows[12];
    float4 shadowSplits, shadowTexels, shadowBiases;
    float4 shadowSettings; // enabled, inverse resolution, normal bias multiplier
    float4 renderParams; // width, height, exposure multiplier, tone mapper
    float4 scatteringParams; // Rayleigh strength, Mie strength, Mie anisotropy
    float4 terrainParams; // preset, texture seed, minimum terrain elevation
    float4 cloudLayer; // inner/outer radius, coverage, extinction per meter
    float4 cloudShape; // global weather scale, formation scale (meters)
    float4 cloudRender; // enabled, primary steps, half-resolution width/height
    float4 cloudWind; // periodic planet-relative noise displacement (meters)
    float4 previousRight, previousUp, previousForward; // w: x/y projection scale, previous near
    float4 eyeDeltaHi, eyeDeltaLo; // current planet-relative eye minus previous eye, split on CPU
    float4 temporalJitter; // current/previous NDC offsets
    float4 taaParams; // enabled, history valid, maximum feedback, variance gamma
    float4 taaFilter; // motion response, depth rejection, sharpening, frame index
    float4 cloudTemporal; // enabled, history valid, feedback, sparse sampling pattern
    float4 cloudTemporalFilter; // depth rejection, ray jitter strength
    float4 cloudWindDelta; // unwrapped displacement since the previous frame
};
cbuffer DrawPass : register(b1) { uint cascadeIndex; uint objectPass; };
StructuredBuffer<float4> previousObjectPositions : register(t17);

float3 previousRelative(float3 relative) {
    DF3 p = df3Add(df3(relative, 0), df3(eyeDeltaHi.xyz, eyeDeltaLo.xyz));
    return p.hi + p.lo;
}
float4 previousClip(float3 relative) {
    float z = dot(relative, previousForward.xyz);
    return float4(dot(relative, previousRight.xyz) * previousRight.w + temporalJitter.z * z,
                  dot(relative, previousUp.xyz) * previousUp.w + temporalJitter.w * z, 1, z);
}
float2 clipUv(float4 clip) { return clip.xy / max(1e-6, clip.w) * float2(0.5, -0.5) + 0.5; }
float3 cameraRay(float2 uv) {
    float2 ndc = uv * float2(2, -2) + float2(-1, 1) - temporalJitter.xy;
    return cameraForward.xyz + cameraRight.xyz * (ndc.x / projection.x) + cameraUp.xyz * (ndc.y / projection.y);
}
float3 previousRay(float2 uv) {
    float2 ndc = uv * float2(2, -2) + float2(-1, 1) - temporalJitter.zw;
    return normalize(previousForward.xyz + previousRight.xyz * (ndc.x / previousRight.w) + previousUp.xyz * (ndc.y / previousUp.w));
}

DF3 planetEye() { return df3(planetEyeHi.xyz, planetEyeLo.xyz); }
float3 betaR() {
    float3 spectrum = float3(5.802, 13.558, 33.100);
    if (terrainParams.x == 1) spectrum = float3(2.6, 1.8, 1.15);
    if (terrainParams.x == 2) spectrum = float3(3.9, 3.7, 3.6);
    return (scatteringParams.x * 1e-6) * spectrum;
}
float3 betaM() {
    float3 spectrum = 3.996;
    if (terrainParams.x == 1) spectrum = float3(2.95, 2.15, 1.45);
    if (terrainParams.x == 2) spectrum = float3(4.4, 4.0, 3.55);
    return (scatteringParams.y * 1e-6) * spectrum;
}
float2 density(float radiusDelta) {
    // (r² - R²) / (r + R) avoids subtracting two nearly equal radii.
    float r = sqrt(max(0, atmosphere.x * atmosphere.x + radiusDelta));
    float h = max(0, radiusDelta / (r + atmosphere.x));
    return exp(-h / atmosphere.zw);
}
float3 extinction(float2 optical) { return betaR() * optical.x + (betaM() / 0.9) * optical.y; }
float2 sunlightDepth(float radiusDelta, float sunProjection, float sunLengthSquared, float topDelta) {
    float groundDisc = sunProjection * sunProjection - sunLengthSquared * radiusDelta;
    if (radiusDelta > 0 && sunProjection < 0 && groundDisc >= 0 &&
        radiusDelta > 0.01 * (sqrt(max(0, groundDisc)) - sunProjection)) return float2(1e20, 1e20);
    float toTop = radiusDelta - topDelta;
    float topDisc = sunProjection * sunProjection - sunLengthSquared * toTop;
    if (topDisc < 0) return 0;
    float root = sqrt(topDisc);
    float distance = sunProjection < 0 ? (root - sunProjection) / sunLengthSquared
                                       : -toTop / max(root + sunProjection, 1e-6);
    float step = max(0, distance) / 6;
    float2 optical = 0;
    [unroll] for (int i = 0; i < 6; ++i) {
        float t = (i + 0.5) * step;
        optical += density(radiusDelta + t * (2 * sunProjection + sunLengthSquared * t));
    }
    return optical * step;
}
// Single scattering with Rayleigh + Cornette-Shanks Mie phase functions.
// Intersect and anchor the ray in DF64, then integrate radial deltas from
// that anchor. The inner integrals never reconstruct absolute positions.
float3 scatter(DF3 eye, float3 ray, float limit, out float3 transmittance) {
    float2 interval;
    transmittance = 1;
    if (cameraRight.w < 0.5 || !any(scatteringParams.xy)) return 0;
    if (!dfRaySphere(eye, ray, float2(atmosphere.y, 0), interval)) return 0;
    float start = max(0, interval.x), end = min(limit, interval.y);
    if (end <= start) return 0;
    DF3 anchor = eye;
    float altitude;
    [branch] if (start > 0) {
        anchor = df3Add(eye, df3Scale(ray, start));
        altitude = atmosphere.y - atmosphere.x;
    } else {
        altitude = dfAltitude(eye, float2(atmosphere.x, 0));
    }
    float delta = altitude * (2 * atmosphere.x + altitude);
    float topDelta = (atmosphere.y - atmosphere.x) * (atmosphere.y + atmosphere.x);
    float viewProjection = dot(anchor.hi, ray) + dot(anchor.lo, ray);
    float sunProjection = dot(anchor.hi, sunlight.xyz) + dot(anchor.lo, sunlight.xyz);
    float rayLengthSquared = dot(ray, ray), sunLengthSquared = dot(sunlight.xyz, sunlight.xyz);
    float mu = dot(ray, sunlight.xyz), g = scatteringParams.z;
    float phaseR = 3.0 / (16.0 * 3.14159265) * (1 + mu * mu);
    float phaseM = 3.0 / (8.0 * 3.14159265) * (1 - g * g) * (1 + mu * mu)
                 / ((2 + g * g) * pow(max(1e-3, 1 + g * g - 2 * g * mu), 1.5));
    float2 optical = 0;
    float3 sumR = 0, sumM = 0;
    // Cosine-spaced quadrature resolves near-ground density from either end.
    static const float sampleEdges[17] = {
        0.000000000f, 0.009607360f, 0.038060234f, 0.084265194f, 0.146446609f,
        0.222214883f, 0.308658284f, 0.402454839f, 0.500000000f, 0.597545161f,
        0.691341716f, 0.777785117f, 0.853553391f, 0.915734806f, 0.961939766f,
        0.990392640f, 1.000000000f
    };
    float span = end - start;
    [loop] for (int i = 0; i < 16; ++i) {
        float s0 = sampleEdges[i], s1 = sampleEdges[i + 1];
        float dt = span * (s1 - s0), t = span * 0.5 * (s0 + s1);
        float radial = delta + t * (2 * viewProjection + rayLengthSquared * t);
        float2 local = density(radial) * dt;
        float3 attenuation = exp(-extinction(optical + local * 0.5 +
                                  sunlightDepth(radial, sunProjection + mu * t, sunLengthSquared, topDelta)));
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
    float4 previous : TEXCOORD1;
    float farClip : SV_ClipDistance0;
};
Surface surfaceVS(float3 hi : POSITION0, float3 lo : POSITION1, float3 normal : NORMAL, float4 material : COLOR, uint id : SV_VertexID) {
    // UE translated-world pattern: subtract split world positions, THEN use float.
    DF3 translated = df3Sub(df3(hi, lo), df3(cameraEyeHi.xyz, cameraEyeLo.xyz));
    float3 relative = translated.hi + translated.lo;
    float depth = dot(relative, cameraForward.xyz);
    Surface result;
    result.clip = float4(dot(relative, cameraRight.xyz) * projection.x,
                         dot(relative, cameraUp.xyz) * projection.y, projection.z, depth);
    result.clip.xy += temporalJitter.xy * depth;
    DF3 old = df3Add(translated, df3(eyeDeltaHi.xyz, eyeDeltaLo.xyz));
    result.previous = previousClip(old.hi + old.lo);
    [branch] if (objectPass != 0) {
        float4 previous = previousObjectPositions[id];
        result.previous = previousClip(previous.xyz);
        result.previous.z = previous.w;
    }
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
