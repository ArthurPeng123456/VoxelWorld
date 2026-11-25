Texture2D<float4> gMotion : register(t9);
Texture2D<float4> taaHistory : register(t10);
Texture2D<float2> taaHistoryDepth : register(t11);
Texture2D<float4> cloudHistory : register(t12);
Texture2D<float4> cloudHistoryDepth : register(t13);
Texture2D<float4> taaResolved : register(t14);
Texture2D<float4> cloudRaw : register(t15);
Texture2D<float4> cloudRawDepth : register(t16);
SamplerState historySampler : register(s2);

float3 toYCoCg(float3 c) { return float3(dot(c, float3(0.25, 0.5, 0.25)), 0.5 * (c.r - c.b), c.g - 0.5 * (c.r + c.b)); }
float3 fromYCoCg(float3 c) { return float3(c.x + c.y - 0.5 * c.z, c.x + 0.5 * c.z, c.x - c.y - 0.5 * c.z); }
float luma(float3 c) { return dot(c, float3(0.2126, 0.7152, 0.0722)); }
float3 clipHistory(float3 c, float3 lower, float3 upper) {
    float3 center = 0.5 * (lower + upper), extent = max(0.00001, 0.5 * (upper - lower));
    float3 offset = c - center;
    float radius = max(abs(offset.x / extent.x), max(abs(offset.y / extent.y), abs(offset.z / extent.z)));
    return center + offset / max(1, radius);
}
bool validHistoryUv(float2 uv, float2 dimensions) {
    return all(uv >= 0.5 / dimensions) && all(uv <= 1 - 0.5 / dimensions);
}
float3 blendHdr(float3 current, float3 history, float feedback) {
    float a = (1 - feedback) / (1 + luma(current)), b = feedback / (1 + luma(history));
    return (a * current + b * history) / max(1e-6, a + b);
}
