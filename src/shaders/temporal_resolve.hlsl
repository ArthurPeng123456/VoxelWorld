float3 cloudPreviousPosition(float3 relative) {
    float3 p = planetEyeHi.xyz + (planetEyeLo.xyz + relative);
    return previousRelative(relative + cloudRotationOffset(p, cloudWindDelta.xy));
}

float3 cloudCurrentPosition(float3 oldRelative) {
    DF3 shifted = df3Add(df3(oldRelative, 0), df3(-eyeDeltaHi.xyz, -eyeDeltaLo.xyz));
    float3 relative = shifted.hi + shifted.lo;
    float3 p = planetEyeHi.xyz + (planetEyeLo.xyz + relative);
    return relative + cloudRotationOffset(p, float2(-cloudWindDelta.x, cloudWindDelta.y));
}

struct CloudResolvedOutput { float4 light : SV_Target0; float4 metadata : SV_Target1; };
CloudResolvedOutput cloudResolvePS(Sky input) {
    int2 pixel = int2(input.clip.xy);
    float2 uv = input.clip.xy / cloudRender.zw;
    float4 current = cloudRaw.Load(int3(pixel, 0)), data = cloudRawDepth.Load(int3(pixel, 0));
    bool missing = current.a < 0;
    float3 ray = normalize(cameraRay(uv));
    float distance = data.x;
    float4 oldColor = float4(0, 0, 0, 1), oldData = 0;
    float feedback = 0, fallbackSteps = 0, footprintChange = 0;
    bool accepted = cloudTemporal.x > 0.5 && cloudTemporal.y > 0.5 && distance > 0;
    float2 oldUv = uv;
    [branch] if (accepted) {
        // Refine the initial shell-depth estimate with the reprojected mean
        // scattering depth. Translation is camera/planet relative, never absolute float.
        [unroll] for (int iteration = 0; iteration < 2; ++iteration) {
            float4 clip = previousClip(cloudPreviousPosition(ray * distance));
            oldUv = clipUv(clip);
            if (clip.w <= 0 || !validHistoryUv(oldUv, cloudRender.zw)) { accepted = false; break; }
            oldData = cloudHistoryDepth.Load(int3(int2(oldUv * cloudRender.zw), 0));
            if (oldData.z <= 0 || oldData.x <= 0) { accepted = false; break; }
            if (missing && iteration == 0) {
                float3 previous = previousRay(oldUv) * oldData.x;
                distance = max(0, dot(cloudCurrentPosition(previous), ray));
            }
        }
        if (accepted) {
            float3 oldPosition = cloudPreviousPosition(ray * distance);
            float error = abs(length(oldPosition) - oldData.x) / max(1, oldData.x);
            float footprintRatio = (max(1, distance) * previousUp.w) / (max(1, oldData.x) * projection.y);
            footprintChange = abs(log2(max(0.01, footprintRatio)));
            accepted = error <= cloudTemporalFilter.x && distance < data.y + max(50, data.y * 0.01);
            accepted = accepted && (!missing || footprintChange < 0.32);
            // A history sample hidden behind the old foreground must not leak
            // into newly visible pixels. Such pixels are raymarched below.
            accepted = accepted && oldData.x <= oldData.y + max(50, oldData.y * 0.01);
        }
        if (accepted) {
            oldColor = cloudHistory.SampleLevel(historySampler, oldUv, 0);
            accepted = all(isfinite(oldColor)) && oldColor.a >= 0 && oldColor.a <= 1;
        }
    }
    [branch] if (missing && !accepted) {
        CloudOutput traced = traceCloud(makeCloudRay(uv), pixel);
        current = traced.light; data = traced.distances;
        distance = data.x; fallbackSteps = data.w;
        missing = false;
    }
    float age = 1;
    if (accepted) {
        // Only current, actually traced samples may constrain history. Sparse
        // sentinels are excluded from the neighborhood distribution.
        float3 lower = 1e20, upper = -1e20, sum = 0, square = 0;
        float minT = 1, maxT = 0, count = 0;
        [unroll] for (int y = -1; y <= 1; ++y) [unroll] for (int x = -1; x <= 1; ++x) {
            int2 p = clamp(pixel + int2(x, y), int2(0,0), int2(cloudRender.zw) - 1);
            float4 c = cloudRaw.Load(int3(p, 0));
            float opaque = cloudRawDepth.Load(int3(p, 0)).y;
            if (c.a < 0 || abs(opaque - data.y) > max(100, data.y * 0.02)) continue;
            float3 v = toYCoCg(c.rgb);
            lower = min(lower, v); upper = max(upper, v); sum += v; square += v * v;
            minT = min(minT, c.a); maxT = max(maxT, c.a); count += 1;
        }
        // An untraced pixel has no current center sample. In particular, a
        // quarter-rate 3x3 neighborhood may contain just one traced neighbor;
        // clipping to that degenerate box would replace valid history with a
        // different neighbor on each phase, producing a crawling checkerboard.
        // Trust depth/wind-validated history until this pixel is traced again.
        if (count > 0 && !missing) {
            float3 mean = sum / count, sigma = sqrt(max(0, square / count - mean * mean));
            float3 lo = max(lower, mean - 1.75 * sigma), hi = min(upper, mean + 1.75 * sigma);
            oldColor.rgb = max(0, fromYCoCg(clipHistory(toYCoCg(oldColor.rgb), lo, hi)));
            oldColor.a = clamp(oldColor.a, minT, maxT);
        } else if (count == 0 && missing) {
            CloudOutput traced = traceCloud(makeCloudRay(uv), pixel);
            current = traced.light; data = traced.distances; distance = data.x;
            fallbackSteps = data.w; missing = false; accepted = false;
        }
        if (accepted) {
            float motion = length((oldUv - uv) * cloudRender.zw);
            feedback = min(cloudTemporal.z, oldData.z / (oldData.z + 1)) /
                       (1 + motion * 0.015 + footprintChange * 3);
            age = min(64, oldData.z + (missing ? 0 : 1));
        }
    }
    CloudResolvedOutput result;
    result.light = missing ? oldColor : lerp(current, oldColor, feedback);
    result.light.rgb = max(0, result.light.rgb); result.light.a = saturate(result.light.a);
    result.metadata = float4(distance, data.y, distance > 0 ? age : 0, fallbackSteps);
    return result;
}

struct TemporalOutput { float4 color : SV_Target0; float2 depth : SV_Target1; };
TemporalOutput temporalPS(Sky input) {
    float2 uv = input.clip.xy / renderParams.xy;
    // Store history and present on an UNJITTERED pixel grid. The raster and
    // lighting buffers are jittered, so reconstruct their current sample at
    // uv + current jitter. Resolving directly at uv makes the entire image
    // follow the Halton offset even when reprojection itself is correct.
    float2 jitterUv = temporalJitter.xy * float2(0.5, -0.5);
    float2 previousJitterUv = temporalJitter.zw * float2(0.5, -0.5);
    float2 sourceUv = uv + jitterUv;
    int2 pixel = clamp(int2(sourceUv * renderParams.xy), int2(0,0), int2(renderParams.xy) - 1);
    float2 sampleUv = (float2(pixel) + 0.5) / renderParams.xy;
    float4 current = hdrScene.SampleLevel(historySampler, sourceUv, 0);
    float sceneDepth = gDepth.Load(int3(pixel, 0));
    float depth = sceneDepth > 0 ? projection.z / sceneDepth : 0;
    float4 motion = gMotion.Load(int3(pixel, 0));
    float code = sceneDepth > 0 ? motion.w : -1;
    TemporalOutput result;
    result.color = float4(max(0, current.rgb), 1);
    result.depth = float2(depth, code);
    if (taaParams.x < 0.5 || taaParams.y < 0.5) return result;

    // Motion is evaluated at the selected raster sample. Remove the previous
    // raster jitter and account for the subpixel reconstruction offset. Scene
    // history no longer contains projection jitter; cloud history still does.
    float2 oldUv = motion.xy - previousJitterUv + sourceUv - sampleUv;
    float expectedDepth = motion.z;
    if (sceneDepth <= 0) {
        float4 sky = previousClip(normalize(cameraRay(sourceUv)));
        oldUv = clipUv(sky) - previousJitterUv; expectedDepth = 0;
        if (sky.w <= 0) return result;
    } else if (expectedDepth <= 0) return result;
    if (!validHistoryUv(oldUv, renderParams.xy)) return result;
    float2 oldDepth = taaHistoryDepth.Load(int3(int2(oldUv * renderParams.xy), 0));
    bool depthValid = (depth == 0 && oldDepth.x == 0) ||
        (depth > 0 && oldDepth.x > 0 && abs(oldDepth.x - expectedDepth) <= max(0.05, expectedDepth * taaFilter.y) && abs(oldDepth.y - code) < 0.5);

    float3 lower = 1e20, upper = -1e20, mean = 0, second = 0;
    // A one-pixel neighborhood permits subpixel silhouette coverage, while
    // disocclusions beyond it still reject unrelated foreground history.
    [unroll] for (int y = -1; y <= 1; ++y) [unroll] for (int x = -1; x <= 1; ++x) {
        int2 p = clamp(pixel + int2(x,y), int2(0,0), int2(renderParams.xy) - 1);
        float3 c = toYCoCg(hdrScene.Load(int3(p, 0)).rgb);
        lower = min(lower, c); upper = max(upper, c); mean += c / 9; second += c * c / 9;
        float neighborDepth = gDepth.Load(int3(p, 0));
        float4 neighbor = gMotion.Load(int3(p, 0));
        if (length((oldUv - uv) * renderParams.xy) < 2) {
            depthValid = depthValid || (oldDepth.x == 0 && neighborDepth == 0) ||
                (oldDepth.x > 0 && neighborDepth > 0 && neighbor.z > 0 && abs(oldDepth.y - neighbor.w) < 0.5 &&
                 abs(oldDepth.x - neighbor.z) <= max(0.05, neighbor.z * taaFilter.y));
        }
    }
    if (!depthValid) return result;
    float4 old = taaHistory.SampleLevel(historySampler, oldUv, 0);
    if (!all(isfinite(old)) || old.a < 1) return result;
    float3 sigma = sqrt(max(0, second - mean * mean));
    float3 lo = max(lower, mean - taaParams.w * sigma), hi = min(upper, mean + taaParams.w * sigma);
    float3 clipped = max(0, fromYCoCg(clipHistory(toYCoCg(old.rgb), lo, hi)));
    float motionPixels = length((oldUv - uv) * renderParams.xy);
    float feedback = min(taaParams.z, old.a / (old.a + 1));
    feedback /= 1 + taaFilter.x * motionPixels * 0.1;
    // Clouds already have depth/wind-aware temporal filtering; avoid a second
    // long trail from the underlying opaque surface's different motion.
    if (cloudRender.x > 0.5) {
        float opacity = 1 - cloudLighting.SampleLevel(historySampler, sourceUv, 0).a;
        feedback *= 1 - 0.9 * saturate(opacity * 2);
    }
    feedback *= saturate(current.a);
    result.color = float4(blendHdr(current.rgb, clipped, feedback), feedback > 0.001 ? min(64, old.a + 1) : 1);
    return result;
}
