// Planetary cloud prototype. Global weather controls where clouds exist;
// local 3D noise only shapes those regions. Each scale is filtered separately.
Texture2D<float> gDepth : register(t2);
Texture3D<float4> cloudNoise : register(t6);
Texture2D<float4> cloudLighting : register(t7);
Texture2D<float4> cloudDistances : register(t8);
SamplerState cloudSampler : register(s1);

float cloudLod(float footprint, float period) {
    return clamp(log2(max(1, footprint * 64 / period)), 0, 6);
}

// Subtract the inner sphere from the outer sphere, preserving BOTH intervals
// for rays that cross the hollow layer. This also handles cameras inside it.
float4 cloudIntervals(DF3 eye, float3 ray, float limit) {
    float2 outer, inner;
    if (!dfRaySphere(eye, ray, float2(cloudLayer.y, 0), outer)) return 0;
    float a = max(0, outer.x), b = min(limit, outer.y);
    if (b <= a) return 0;
    if (!dfRaySphere(eye, ray, float2(cloudLayer.x, 0), inner) || inner.y <= a || inner.x >= b)
        return float4(a, b, 0, 0);
    float2 first = float2(a, min(b, inner.x)), second = float2(max(a, inner.y), b);
    if (first.y <= first.x) return second.y > second.x ? float4(second, 0, 0) : 0;
    return float4(first, second.y > second.x ? second : float2(0, 0));
}

float2 cloudWeather(float3 position, float footprint) {
    float3 direction = normalize(position);
    float3 seed = terrainParams.y * float3(0.137, 0.273, 0.197);
    float3 q = direction * (atmosphere.x / (4 * cloudShape.x)) + seed;
    float lod = cloudLod(footprint, 4 * cloudShape.x);
    float4 a = cloudNoise.SampleLevel(cloudSampler, q, lod);
    float3 warp = float3(a.a - 0.5, a.r - 0.5, a.a - a.r) * 0.24;
    float4 b = cloudNoise.SampleLevel(cloudSampler, q * 2.03 + warp + float3(0.31, 0.67, 0.19), lod + 1);
    float4 c = cloudNoise.SampleLevel(cloudSampler, q * 5.17 + warp * 1.7 + float3(0.73, 0.11, 0.43), lod + 2.4);
    float front = 0.5 + 0.5 * sin(dot(direction, float3(8, 13, 3)) + a.a * 4);
    float weather = a.r * 0.55 + b.r * 0.23 + c.a * 0.15 + front * 0.07;
    float threshold = 1.035 - cloudLayer.z;
    float coverage = smoothstep(threshold - 0.11, threshold + 0.11, weather);
    return float2(coverage, a.a);
}

float cloudSample(float3 position, float height, float footprint, float2 weather, bool fine) {
    float bottom = cloudLayer.x - atmosphere.x, thickness = cloudLayer.y - cloudLayer.x;
    float h = (height - bottom) / thickness;
    if (h <= 0 || h >= 1 || weather.x <= 0.001) return 0;
    float top = lerp(0.62, 1.0, weather.y);
    float profile = smoothstep(0, 0.10, h) * (1 - smoothstep(top * 0.55, top, h));
    if (profile <= 0) return 0;
    float period = cloudShape.y * 8;
    float verticalStretch = clamp(cloudShape.y / (thickness * 0.6), 1, 24);
    float3 shapedPosition = position + normalize(position) * (height - bottom) * (verticalStretch - 1);
    float3 q = (shapedPosition + cloudWind.xyz) / period;
    float shape = cloudNoise.SampleLevel(cloudSampler, q, cloudLod(footprint, period)).g;
    // Remove unresolved billows rather than inventing thousands of tiny clouds
    // at orbital distances. The global weather mask remains unchanged.
    float detail = 1 - smoothstep(3000, 35000, footprint);
    shape = lerp(0.5, shape, detail);
    float amount = saturate((shape - 0.60 + weather.x * 0.22) * 5);
    [branch] if (fine && footprint < 3500 && amount > 0) {
        float finePeriod = cloudShape.y * 0.5;
        float erosion = 1 - cloudNoise.SampleLevel(cloudSampler,
            (position + cloudWind.xyz) / finePeriod, cloudLod(footprint, finePeriod)).b;
        amount = max(0, amount - erosion * 0.19 * (1 - amount) * (1 - smoothstep(500, 3500, footprint)));
    }
    return amount * profile;
}

float cloudPhase(float cosine, float g) {
    return (1 - g * g) / (12.5663706 * pow(max(0.001, 1 + g * g - 2 * g * cosine), 1.5));
}

float cloudSun(float3 position, float height, float footprint, float2 weather) {
    float radial = dot(position, sunlight.xyz);
    float groundDelta = height * (2 * atmosphere.x + height);
    if (radial < 0 && radial * radial > groundDelta) return 0;
    float topHeight = cloudLayer.y - atmosphere.x;
    float c = (height - topHeight) * (2 * atmosphere.x + height + topHeight);
    float root = sqrt(max(0, radial * radial - c));
    float span = radial < 0 ? root - radial : -c / max(1, root + radial);
    span = min(max(0, span), 250000);
    float optical = 0;
    [unroll] for (int i = 0; i < 4; ++i) {
        float a = float(i) / 4, b = float(i + 1) / 4;
        float t = 0.5 * (a * a + b * b) * span;
        float delta = groundDelta + t * (2 * radial + t);
        float h = delta / (sqrt(max(0, atmosphere.x * atmosphere.x + delta)) + atmosphere.x);
        optical += cloudSample(position + sunlight.xyz * t, h,
            max(footprint, (b * b - a * a) * span * 0.4), weather, false) * (b * b - a * a) * span;
    }
    return exp(-cloudLayer.w * optical);
}

struct CloudRay { float3 ray; float opaqueDistance; float4 intervals; float referenceDistance; };
CloudRay makeCloudRay(float2 uv) {
    CloudRay camera;
    camera.intervals = 0; camera.referenceDistance = 0;
    int2 pixel = min(int2(uv * renderParams.xy), int2(renderParams.xy) - 1);
    float depth = gDepth.Load(int3(pixel, 0));
    float3 view = cameraRay(uv);
    float viewLength = length(view);
    float3 ray = view / viewLength;
    float opaqueDistance = depth > 0 ? (projection.z / depth) * viewLength : projection.w;
    camera.ray = ray; camera.opaqueDistance = opaqueDistance;
    if (cloudRender.x < 0.5 || cloudLayer.z <= 0 || cloudLayer.w <= 0) return camera;
    float limit = opaqueDistance;
    // Only limit cloud integration at the planet: this never draws ground or
    // writes scene depth, which remains entirely supplied by actual meshes.
    float2 ground;
    if (dfRaySphere(planetEye(), ray, float2(atmosphere.x + terrainParams.z, 0), ground) && ground.x > 0)
        limit = min(limit, ground.x);
    camera.intervals = cloudIntervals(planetEye(), ray, limit);
    camera.referenceDistance = 0.5 * (camera.intervals.x + camera.intervals.y);
    return camera;
}
struct CloudOutput { float4 light : SV_Target0; float4 distances : SV_Target1; };
CloudOutput traceCloud(CloudRay camera, int2 pixel) {
    CloudOutput result;
    result.light = float4(0, 0, 0, 1);
    result.distances = float4(camera.referenceDistance, camera.opaqueDistance, 0, 0);
    float4 intervals = camera.intervals;
    if (intervals.y <= intervals.x) return result;
    result.distances.z = 1;
    float3 ray = camera.ray;
    uint hash = uint(pixel.x) * 1597334677u ^ uint(pixel.y) * 3812015801u ^ uint(taaFilter.w) * 747796405u;
    hash = (hash ^ (hash >> 16)) * 2246822519u;
    float sampleOffset = 0.5 + (float(hash & 65535u) / 65535 - 0.5) * cloudTemporalFilter.y;
    float transmittance = 1, weightedDistance = 0, weight = 0;
    float3 radiance = 0;
    float mu = dot(ray, sunlight.xyz);
    float phase = 0.8 * cloudPhase(mu, 0.65) + 0.2 * cloudPhase(mu, -0.2);
    float pixelAngle = 2 / max(1, cloudRender.w * projection.y);
    [loop] for (int segment = 0; segment < 2; ++segment) {
        float start = intervals[segment * 2], end = intervals[segment * 2 + 1];
        if (end <= start || transmittance < 0.005) continue;
        int steps = int(cloudRender.y);
        float span = end - start;
        float curve = 1 - smoothstep(30000, 150000, start);
        DF3 anchor = df3Add(planetEye(), df3Scale(ray, start));
        float altitude = dfAltitude(anchor, float2(atmosphere.x, 0));
        float delta = altitude * (2 * atmosphere.x + altitude);
        float radial = dot(anchor.hi, ray) + dot(anchor.lo, ray);
        float raySquared = dot(ray, ray);
        [loop] for (int i = 0; i < steps; ++i) {
            float a = float(i) / steps, b = float(i + 1) / steps;
            float s0 = lerp(a, a * a, curve), s1 = lerp(b, b * b, curve);
            float step = span * (s1 - s0), t = span * lerp(s0, s1, sampleOffset), distance = start + t;
            result.distances.w += 1;
            float localDelta = delta + t * (2 * radial + t * raySquared);
            float height = localDelta / (sqrt(max(0, atmosphere.x * atmosphere.x + localDelta)) + atmosphere.x);
            float3 position = anchor.hi + (anchor.lo + ray * t);
            float footprint = max(distance * pixelAngle, step * 0.5);
            float2 weather = cloudWeather(position, footprint);
            float sampleDensity = cloudSample(position, height, footprint, weather, true);
            if (sampleDensity <= 0.0001) continue;
            float alpha = 1 - exp(-sampleDensity * cloudLayer.w * step);
            float sun = cloudSun(position, height, footprint, weather);
            float h = saturate((height - (cloudLayer.x - atmosphere.x)) / (cloudLayer.y - cloudLayer.x));
            float3 ambient = lerp(float3(0.075, 0.105, 0.155), float3(0.26, 0.32, 0.41), h);
            float day = smoothstep(-0.12, 0.08, dot(normalize(position), sunlight.xyz));
            float multiple = 0.28 * pow(sun, 0.35);
            float3 light = (ambient + (20 * phase * sun + multiple) * float3(1.0, 0.97, 0.91)) * day + 0.002;
            float w = transmittance * alpha;
            radiance += w * light;
            weightedDistance += w * distance;
            weight += w;
            transmittance *= 1 - alpha;
            if (transmittance < 0.005) break;
        }
    }
    if (weight > 0.00001) {
        float meanDistance = weightedDistance / weight;
        float3 airTransmittance;
        float3 air = scatter(planetEye(), ray, meanDistance, airTransmittance);
        // Approximate the atmospheric segment in front of the cloud layer.
        // Premultiplied radiance + remaining transmittance compose into HDR.
        result.light = float4(radiance * airTransmittance + air * (1 - transmittance), transmittance);
        result.distances.x = meanDistance;
    }
    return result;
}

CloudOutput cloudPS(Sky input) {
    int2 pixel = int2(input.clip.xy);
    CloudRay camera = makeCloudRay(input.clip.xy / cloudRender.zw);
    uint phase = uint(taaFilter.w);
    // Rotate the phase between cycles so a pixel is not permanently paired
    // with one subset of the Halton jitter sequence (especially its base-2 X).
    uint checkerPhase = phase + (phase >> 1u), quarterPhase = phase + (phase >> 2u);
    bool active = cloudTemporal.w < 0.5 || (cloudTemporal.w < 1.5
        ? ((uint(pixel.x + pixel.y) + checkerPhase) & 1u) == 0
        : ((uint(pixel.x) & 1u) | ((uint(pixel.y) & 1u) << 1u)) == (quarterPhase & 3u));
    if (cloudTemporal.x > 0.5 && cloudTemporal.y > 0.5 && !active && camera.intervals.y > camera.intervals.x) {
        CloudOutput skipped;
        skipped.light = float4(0, 0, 0, -1); // Never composed directly; temporal resolve fills this pixel.
        skipped.distances = float4(camera.referenceDistance, camera.opaqueDistance, -1, 0);
        return skipped;
    }
    return traceCloud(camera, pixel);
}

float3 compositeClouds(float3 background, float2 uv, float opaqueDistance) {
    if (cloudRender.x < 0.5 || cloudLayer.z <= 0 || cloudLayer.w <= 0) return background;
    float2 p = uv * cloudRender.zw - 0.5;
    int2 first = int2(floor(p));
    float2 fraction = frac(p);
    float4 total = 0;
    float totalWeight = 0;
    [unroll] for (int y = 0; y < 2; ++y) [unroll] for (int x = 0; x < 2; ++x) {
        int2 pixel = clamp(first + int2(x, y), int2(0, 0), int2(cloudRender.zw) - 1);
        float2 distances = cloudDistances.Load(int3(pixel, 0)).xy;
        float w = (x ? fraction.x : 1 - fraction.x) * (y ? fraction.y : 1 - fraction.y);
        // Reject low-resolution samples across foreground/sky discontinuities.
        w *= exp(-abs(distances.y - opaqueDistance) / max(100, opaqueDistance * 0.015));
        if (distances.x > opaqueDistance + max(100, opaqueDistance * 0.01)) w = 0;
        total += cloudLighting.Load(int3(pixel, 0)) * w;
        totalWeight += w;
    }
    if (totalWeight < 0.0001) return background;
    total /= totalWeight;
    return total.rgb + total.a * background;
}

// Broad weather shadows: deliberately use the global mask so unresolved cloud
// billows do not become a flickering carpet of shadow dots in orbital views.
float cloudGroundShadow(float3 position) {
    if (cloudRender.x < 0.5 || cloudLayer.z <= 0 || cloudLayer.w <= 0) return 1;
    float radius = 0.5 * (cloudLayer.x + cloudLayer.y);
    float b = dot(position, sunlight.xyz);
    float c = dot(position, position) - radius * radius;
    float root = sqrt(max(0, b * b - c));
    float t = b < 0 ? root - b : -c / max(1, root + b);
    float3 p = position + sunlight.xyz * t;
    float cover = cloudWeather(p, 5000).x;
    float optical = cover * (cloudLayer.y - cloudLayer.x) * cloudLayer.w * 0.45;
    return 0.25 + 0.75 * exp(-optical);
}
