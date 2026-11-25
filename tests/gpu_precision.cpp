#include "render_coordinates.hpp"
#include "render_settings.hpp"
#include "shaders.hpp"
#include "precision_test_shader.hpp"
#include "post_process.hpp"
#include "cloud_noise.hpp"
#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

using Microsoft::WRL::ComPtr;
using namespace voxel;
namespace {
void require(HRESULT hr, const char* what) {
    if (FAILED(hr)) throw std::runtime_error(std::string(what) + " HRESULT=" + std::to_string(hr));
}
ComPtr<ID3DBlob> compile(const std::string& source, const char* entry, const char* profile) {
    ComPtr<ID3DBlob> code, errors;
    const HRESULT hr = D3DCompile(source.data(), source.size(), "precision-test", nullptr, nullptr, entry, profile,
        D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_IEEE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3,
        0, &code, &errors);
    if (FAILED(hr) && errors) throw std::runtime_error(static_cast<const char*>(errors->GetBufferPointer()));
    require(hr, "Compile shader");
    return code;
}
struct Input { float pHi[4]{}, pLo[4]{}, bHi[4]{}, bLo[4]{}, ray[4]{}, params[4]{}; };
struct Output { float relative[4], geometry[4], arithmetic0[4], arithmetic1[4], scattering[4], transmittance[4], tone[4], clouds[4]; };
void pack(Vec value, float* hi, float* lo) {
    for (int i = 0; i < 3; ++i) { const auto p = splitDouble(value[i]); hi[i] = p.hi; lo[i] = p.lo; }
}
Vec unpack(const float* hi, const float* lo) { return {double(hi[0]) + lo[0], double(hi[1]) + lo[1], double(hi[2]) + lo[2]}; }
void close(double actual, double expected, double tolerance, const std::string& label) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(label + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual)
            + ", tolerance " + std::to_string(tolerance));
}
}
int main(int argc, char** argv) {
    try {
        // Compile the production entry points, including precise transforms, with release flags.
        for (const auto& entry : {std::pair{"surfaceVS", "vs_5_0"}, {"surfacePS", "ps_5_0"},
                                  {"skyVS", "vs_5_0"}, {"deferredPS", "ps_5_0"},
                                  {"shadowVS", "vs_5_0"}, {"toneMapPS", "ps_5_0"}, {"linePS", "ps_5_0"}, {"cloudPS", "ps_5_0"},
                                  {"cloudResolvePS", "ps_5_0"}, {"temporalPS", "ps_5_0"}})
            compile(shaderSource, entry.first, entry.second);
        const auto noise = makeCloudNoise();
        if (noise.size() != 7 || noise.front().edge != 64 || noise.back().edge != 1)
            throw std::runtime_error("Cloud noise mip chain is incomplete");
        for (unsigned channel = 0; channel < 4; ++channel) {
            double previousVariance = 1e9, baseMean = 0;
            for (const auto& mip : noise) {
                double mean = 0, square = 0;
                const auto count = mip.rgba.size() / 4;
                for (std::size_t i = channel; i < mip.rgba.size(); i += 4) {
                    mean += mip.rgba[i]; square += double(mip.rgba[i]) * mip.rgba[i];
                }
                mean /= count;
                const double variance = square / count - mean * mean;
                if (mip.edge == 64) baseMean = mean;
                if (variance > previousVariance + 1 || std::abs(mean - baseMean) > 4)
                    throw std::runtime_error("Cloud mip filtering loses the mean or increases aliasing");
                previousVariance = variance;
            }
        }
        const bool hardware = argc > 1 && std::string(argv[1]) == "--hardware";
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        D3D_FEATURE_LEVEL level;
        require(D3D11CreateDevice(nullptr, hardware ? D3D_DRIVER_TYPE_HARDWARE : D3D_DRIVER_TYPE_WARP,
            nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &device, &level, &context), "Create compute test device");
        std::vector<Input> inputs;
        // Millimeter altitude, horizon hit/miss, ground-down, orbital and large-origin cancellation.
        for (double h : {0.001, 0.1, 1.0, 1200.0, 8000.0, 13000.0, 18000.0, 100000.0, 10000000.0}) {
            for (Vec axis : {Vec{1,0,0}, Vec{0.3,0.4,0.8660254037844386}, Vec{-0.6,0.8,0}}) {
                const Vec up = unit(axis), tangent = unit(cross(up, Vec{0,0,1}), {0,1,0});
                for (double angle : {-1.0, -0.01, 0.0, 0.01, 1.0}) {
                    Input input;
                    pack(up * (planetRadius + h), input.pHi, input.pLo);
                    pack(up * planetRadius, input.bHi, input.bLo);
                    const Vec ray = unit(tangent + up * angle);
                    for (int i = 0; i < 3; ++i) input.ray[i] = float(ray[i]);
                    input.params[0] = float(planetRadius);
                    inputs.push_back(input);
                }
            }
        }
        const auto geometryCases = inputs.size();
        for (double origin : {1e9, -1e9, 1e12, -1e12}) {
            Input input;
            pack({origin + 0.125, origin - 0.0625, origin + 1.25}, input.pHi, input.pLo);
            pack({origin, origin, origin}, input.bHi, input.bLo);
            input.ray[2] = 1;
            input.params[0] = float(planetRadius);
            inputs.push_back(input);
        }
        const auto csCode = compile(std::string(shaderSource) + precisionTestShader, "precisionCS", "cs_5_0");
        ComPtr<ID3D11ComputeShader> cs;
        require(device->CreateComputeShader(csCode->GetBufferPointer(), csCode->GetBufferSize(), nullptr, &cs), "Create CS");
        D3D11_BUFFER_DESC desc{};
        desc.ByteWidth = UINT(inputs.size() * sizeof(Input));
        desc.Usage = D3D11_USAGE_IMMUTABLE;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
        desc.StructureByteStride = sizeof(Input);
        D3D11_SUBRESOURCE_DATA data{inputs.data(), 0, 0};
        ComPtr<ID3D11Buffer> inputBuffer, outputBuffer, staging;
        require(device->CreateBuffer(&desc, &data, &inputBuffer), "Create input");
        desc.ByteWidth = UINT(inputs.size() * sizeof(Output));
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        desc.StructureByteStride = sizeof(Output);
        require(device->CreateBuffer(&desc, nullptr, &outputBuffer), "Create output");
        desc.Usage = D3D11_USAGE_STAGING;
        desc.BindFlags = 0;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        desc.MiscFlags = 0;
        require(device->CreateBuffer(&desc, nullptr, &staging), "Create readback");
        ComPtr<ID3D11ShaderResourceView> srv;
        ComPtr<ID3D11UnorderedAccessView> uav;
        require(device->CreateShaderResourceView(inputBuffer.Get(), nullptr, &srv), "Create input SRV");
        require(device->CreateUnorderedAccessView(outputBuffer.Get(), nullptr, &uav), "Create output UAV");
        context->CSSetShader(cs.Get(), nullptr, 0);
        const Vec sun = unit(Vec{0.45, 0.25, 0.86});
        constexpr RenderSettings defaults;
        constexpr auto air = atmosphereDimensions(defaults);
        float camera[176]{};
        camera[11] = 1; // Atmosphere enabled, matching the viewer default.
        camera[24] = float(sun.x); camera[25] = float(sun.y); camera[26] = float(sun.z);
        camera[36] = float(planetRadius); camera[37] = float(planetRadius + air.thickness);
        camera[38] = float(air.rayleighScale); camera[39] = float(air.mieScale);
        camera[106] = 1;
        camera[108] = float(defaults.rayleighStrength);
        camera[109] = float(defaults.mieStrength);
        camera[110] = float(defaults.mieAnisotropy);
        camera[116] = float(planetRadius + defaults.cloudBaseKm * 1000);
        camera[117] = float(planetRadius + (defaults.cloudBaseKm + defaults.cloudThicknessKm) * 1000);
        D3D11_BUFFER_DESC cbDesc{};
        cbDesc.ByteWidth = sizeof(camera); cbDesc.Usage = D3D11_USAGE_IMMUTABLE;
        cbDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        D3D11_SUBRESOURCE_DATA cbData{camera, 0, 0};
        ComPtr<ID3D11Buffer> constants;
        require(device->CreateBuffer(&cbDesc, &cbData, &constants), "Create atmosphere constants");
        ID3D11Buffer* cbs[]{constants.Get()};
        context->CSSetConstantBuffers(0, 1, cbs);
        ID3D11ShaderResourceView* srvs[]{srv.Get()};
        ID3D11UnorderedAccessView* uavs[]{uav.Get()};
        context->CSSetShaderResources(5, 1, srvs);
        context->CSSetUnorderedAccessViews(0, 1, uavs, nullptr);
        context->Dispatch(UINT((inputs.size() + 63) / 64), 1, 1);
        context->CopyResource(staging.Get(), outputBuffer.Get());
        D3D11_MAPPED_SUBRESOURCE mapped{};
        require(context->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &mapped), "Read compute results");
        std::vector<Output> outputs(inputs.size());
        std::memcpy(outputs.data(), mapped.pData, outputs.size() * sizeof(Output));
        context->Unmap(staging.Get(), 0);
        double worstAltitude = 0, worstRoot = 0, worstRelative = 0, worstNearRelative = 0;
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            const auto& in = inputs[i]; const auto& out = outputs[i];
            const Vec p = unpack(in.pHi, in.pLo), b = unpack(in.bHi, in.bLo);
            const std::string label = "case " + std::to_string(i);
            const double ev = int(i % 5) - 2;
            const double hdrValues[4] = {0.18, 1, 4, 20};
            for (int channel = 0; channel < 4; ++channel)
                close(out.tone[channel], toneMap(hdrValues[channel], ev), 2e-5, label + " HDR tone mapping");
            for (int a = 0; a < 3; ++a) {
                const double error = std::abs(out.relative[a] - (p - b)[a]);
                worstRelative = std::max(worstRelative, error);
                if (length(p - b) < 10000) worstNearRelative = std::max(worstNearRelative, error);
                const double tolerance = (std::abs(p[a]) > 1e11 ? 0.02 : 2e-3) + std::abs((p - b)[a]) * 6e-8;
                close(out.relative[a], (p - b)[a], tolerance, label + " relative");
            }
            for (int channel = 0; channel < 3; ++channel) {
                if (!std::isfinite(out.scattering[channel]) || out.scattering[channel] < 0 ||
                    !std::isfinite(out.transmittance[channel]) || out.transmittance[channel] < 0 || out.transmittance[channel] > 1)
                    throw std::runtime_error(label + " invalid scattering/transmittance");
            }
            if (i < geometryCases) {
                const double altitude = length(p) - planetRadius;
                worstAltitude = std::max(worstAltitude, std::abs(out.geometry[0] - altitude));
                close(out.geometry[0], altitude, 1e-3 + std::abs(altitude) * 6e-8, label + " altitude");
                const Vec d{in.ray[0], in.ray[1], in.ray[2]};
                const double aa = dot(d,d), bb = dot(p,d), cc = dot(p,p) - planetRadius * planetRadius;
                const double disc = bb * bb - aa * cc;
                if (bool(out.geometry[3]) != (disc >= 0)) throw std::runtime_error(label + " sphere hit/miss mismatch");
                if (disc >= 0) {
                    const double q = -bb - std::copysign(std::sqrt(disc), bb);
                    const double r0 = q / aa, r1 = cc / q;
                    const double nearRoot = std::min(r0,r1), farRoot = std::max(r0,r1);
                    worstRoot = std::max(worstRoot, std::abs(out.geometry[1] - nearRoot));
                    close(out.geometry[1], nearRoot, 0.03 + std::abs(nearRoot) * 1e-7, label + " near root");
                    close(out.geometry[2], farRoot, 0.03 + std::abs(farRoot) * 1e-7, label + " far root");
                }
                const auto sphere = [&](double radius) {
                    const double c = dot(p, p) - radius * radius, discriminant = bb * bb - aa * c;
                    if (discriminant < 0) return std::array<double, 2>{1, -1};
                    const double root = std::sqrt(discriminant);
                    return std::array<double, 2>{(-bb - root) / aa, (-bb + root) / aa};
                };
                const auto outer = sphere(camera[117]), inner = sphere(camera[116]);
                const double a = std::max(0.0, outer[0]), end = std::min(5e7, outer[1]);
                std::array<double, 4> expected{};
                if (end > a) {
                    if (inner[1] < inner[0] || inner[1] <= a || inner[0] >= end) expected = {a, end, 0, 0};
                    else {
                        unsigned index = 0;
                        if (inner[0] > a) { expected[index++] = a; expected[index++] = std::min(end, inner[0]); }
                        if (inner[1] < end) { expected[index++] = std::max(a, inner[1]); expected[index] = end; }
                    }
                }
                for (unsigned c = 0; c < 4; ++c)
                    close(out.clouds[c], expected[c], 0.05 + std::abs(expected[c]) * 2e-7, label + " cloud shell interval");
            }
            const auto pair = [](const float* v) { return double(v[0]) + v[1]; };
            for (const auto& op : {std::pair{pair(out.arithmetic0), p.x + b.x},
                                  {pair(out.arithmetic0 + 2), p.x * b.x},
                                  {pair(out.arithmetic1), p.x / b.x},
                                  {pair(out.arithmetic1 + 2), std::sqrt(std::abs(p.x))}})
                close(op.first, op.second, 2e-6 + std::abs(op.second) * 8e-14, label + " arithmetic");
        }
        std::cout << (hardware ? "Hardware" : "WARP") << ": " << inputs.size() << " GPU cases passed; max altitude error="
                  << worstAltitude << "m, root error=" << worstRoot << "m, relative error=" << worstRelative
                  << "m, near relative error=" << worstNearRelative << "m\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
