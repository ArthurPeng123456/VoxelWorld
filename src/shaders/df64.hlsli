// A. Thall, Extended-Precision Floating-Point Numbers for GPU Computation (2007),
// algorithms 1-4 and section III.A. Unevaluated (hi, lo) sums: ~48 significand bits,
// float exponent range, NOT IEEE binary64. Keep every error transform precise.
float2 twoSum(float a, float b) {
    precise float s = a + b;
    precise float v = s - a;
    precise float e = (a - (s - v)) + (b - v);
    return float2(s, e);
}
float2 quickTwoSum(float a, float b) {
    precise float s = a + b;
    precise float e = b - (s - a);
    return float2(s, e);
}
float2 dfSplit(float a) {
    precise float t = 4097.0 * a;
    precise float hi = t - (t - a);
    precise float lo = a - hi;
    return float2(hi, lo);
}
float2 twoProduct(float a, float b) {
    precise float p = a * b;
    float2 aa = dfSplit(a), bb = dfSplit(b);
    precise float e = ((aa.x * bb.x - p) + aa.x * bb.y + aa.y * bb.x) + aa.y * bb.y;
    return float2(p, e);
}
float2 dfAdd(float2 a, float2 b) {
    precise float2 s = twoSum(a.x, b.x);
    precise float2 t = twoSum(a.y, b.y);
    s.y = s.y + t.x;
    s = quickTwoSum(s.x, s.y);
    s.y = s.y + t.y;
    return quickTwoSum(s.x, s.y);
}
float2 dfSub(float2 a, float2 b) { return dfAdd(a, -b); }
float2 dfMul(float2 a, float2 b) {
    precise float2 p = twoProduct(a.x, b.x);
    p.y = p.y + a.x * b.y;
    p.y = p.y + a.y * b.x;
    return quickTwoSum(p.x, p.y);
}
float2 dfDiv(float2 b, float2 a) {
    precise float q = b.x / a.x;
    float2 r = dfSub(b, dfMul(a, float2(q, 0)));
    return dfAdd(float2(q, 0), float2((r.x + r.y) / a.x, 0));
}
float2 dfSqrt(float2 a) {
    if (a.x <= 0) return 0;
    precise float y = sqrt(a.x);
    float2 r = dfSub(a, twoProduct(y, y));
    return dfAdd(float2(y, 0), float2((r.x + r.y) * (0.5 / y), 0));
}
float dfFloat(float2 a) { return a.x + a.y; }
struct DF3 { float3 hi; float3 lo; };
DF3 df3(float3 hi, float3 lo) { DF3 r; r.hi = hi; r.lo = lo; return r; }
DF3 df3Add(DF3 a, DF3 b) {
    float2 x = dfAdd(float2(a.hi.x, a.lo.x), float2(b.hi.x, b.lo.x));
    float2 y = dfAdd(float2(a.hi.y, a.lo.y), float2(b.hi.y, b.lo.y));
    float2 z = dfAdd(float2(a.hi.z, a.lo.z), float2(b.hi.z, b.lo.z));
    return df3(float3(x.x, y.x, z.x), float3(x.y, y.y, z.y));
}
DF3 df3Sub(DF3 a, DF3 b) { return df3Add(a, df3(-b.hi, -b.lo)); }
DF3 df3Scale(float3 a, float t) {
    float2 x = twoProduct(a.x, t), y = twoProduct(a.y, t), z = twoProduct(a.z, t);
    return df3(float3(x.x, y.x, z.x), float3(x.y, y.y, z.y));
}
float2 dfDot(DF3 a, DF3 b) {
    return dfAdd(dfAdd(dfMul(float2(a.hi.x, a.lo.x), float2(b.hi.x, b.lo.x)),
                       dfMul(float2(a.hi.y, a.lo.y), float2(b.hi.y, b.lo.y))),
                       dfMul(float2(a.hi.z, a.lo.z), float2(b.hi.z, b.lo.z)));
}
float dfAltitude(DF3 p, float2 radius) { return dfFloat(dfSub(dfSqrt(dfDot(p, p)), radius)); }
// General quadratic (direction is only approximately normalized). Stable q roots
// avoid subtracting almost equal b and sqrt(discriminant) at the ground.
bool dfRaySphere(DF3 origin, float3 direction, float2 radius, out float2 roots) {
    DF3 d = df3(direction, 0);
    float2 a = dfDot(d, d), b = dfDot(origin, d);
    float2 c = dfSub(dfDot(origin, origin), dfMul(radius, radius));
    float2 disc = dfSub(dfMul(b, b), dfMul(a, c));
    roots = 0;
    if (disc.x < 0) return false;
    float2 s = dfSqrt(disc);
    float2 q = dfSub(-b, b.x >= 0 ? s : -s);
    if (q.x == 0) return true;
    float r0 = dfFloat(dfDiv(q, a)), r1 = dfFloat(dfDiv(c, q));
    roots = float2(min(r0, r1), max(r0, r1));
    return true;
}
