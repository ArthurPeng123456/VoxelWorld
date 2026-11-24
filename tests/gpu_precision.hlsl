struct TestInput { float4 pHi, pLo, bHi, bLo, ray, params; };
struct TestOutput { float4 relative, geometry, arithmetic0, arithmetic1, scattering, transmittance; };
StructuredBuffer<TestInput> cases : register(t0);
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
    results[id] = result;
}
