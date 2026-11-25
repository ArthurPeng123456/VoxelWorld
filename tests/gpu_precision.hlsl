struct TestInput { float4 pHi, pLo, bHi, bLo, ray, params; };
struct TestOutput { float4 relative, geometry, arithmetic0, arithmetic1, scattering, transmittance, tone, clouds, solar, wind, density; };
StructuredBuffer<TestInput> cases : register(t5);
RWStructuredBuffer<TestOutput> results : register(u0);
[numthreads(64, 1, 1)]
void precisionCS(uint id : SV_DispatchThreadID) {
    uint count, stride;
    cases.GetDimensions(count, stride);
    if (id >= count) return;
    TestInput input = cases[id];
    DF3 p = df3(input.pHi.xyz, input.pLo.xyz), b = df3(input.bHi.xyz, input.bLo.xyz);
    DF3 relative = df3Sub(p, b);
    TestOutput result;
    result.relative = float4(relative.hi + relative.lo, 0);
    float2 roots;
    bool hit = dfRaySphere(p, input.ray.xyz, input.params.xy, roots);
    result.geometry = float4(dfAltitude(p, input.params.xy), roots, hit ? 1 : 0);
    float2 a = float2(input.pHi.x, input.pLo.x), bb = float2(input.bHi.x, input.bLo.x);
    result.arithmetic0 = float4(dfAdd(a, bb), dfMul(a, bb));
    result.arithmetic1 = float4(dfDiv(a, bb), dfSqrt(float2(abs(a.x), a.x >= 0 ? a.y : -a.y)));
    float3 transmittance;
    float limit = hit && roots.x > 0 ? roots.x : 5e7;
    result.scattering = float4(scatter(p, input.ray.xyz, limit, transmittance), 0);
    result.transmittance = float4(transmittance, 0);
    float exposure = exp2(int(id % 5) - 2);
    result.tone = float4(acesToneMap(float3(0.18, 1, 4), exposure), acesToneMap(float3(20, 20, 20), exposure).x);
    result.clouds = cloudIntervals(p, input.ray.xyz, 5e7);
    result.solar = cloudSolarIntervals(result.geometry.x, dot(p.hi, input.ray.xyz) + dot(p.lo, input.ray.xyz));
    const float2 wind = float2(-0.0699428473, 0.00244899975); // inverse rotation by 0.07 rad
    float3 position = p.hi + p.lo;
    float3 moved = cloudRotationOffset(position, wind);
    float3 back = cloudRotationOffset(position + moved, float2(-wind.x, wind.y));
    result.wind = float4(moved, length(moved + back));
    result.density = 0;
    if (result.geometry.x > cloudLayer.x - atmosphere.x && result.geometry.x < cloudLayer.y - atmosphere.x) {
        // Reconstruct the same physical point from two cameras with different
        // elevations and bearings. Both rays use the same band limit.
        DF3 observerA = df3Add(p, df3(float3(72000, -420, 10000), 0));
        DF3 observerB = df3Add(p, df3(float3(-4600, 350000, -8500), 0));
        DF3 sampleA = df3Add(observerA, df3(float3(-72000, 420, -10000), 0));
        DF3 sampleB = df3Add(observerB, df3(float3(4600, -350000, 8500), 0));
        result.density.x = cloudSample(sampleA.hi + sampleA.lo, result.geometry.x, 80, 80);
        result.density.y = cloudSample(sampleB.hi + sampleB.lo, result.geometry.x, 80, 80);
    }
    results[id] = result;
}
