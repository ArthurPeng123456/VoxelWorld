#include "dx12_renderer.hpp"
#include "shaders.hpp"
#include "shadow_cascades.hpp"
#include "cloud_noise.hpp"

#include <d3d12.h>
#include <d3dcompiler.h>
#include <d3d12sdklayers.h>
#include <dxgi1_4.h>
#include <wrl/client.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <type_traits>
#include <sstream>
#include <stdexcept>

namespace voxel {
namespace {

using Microsoft::WRL::ComPtr;

constexpr UINT buffers = 2;
constexpr DXGI_FORMAT pixelFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
constexpr DXGI_FORMAT depthFormat = DXGI_FORMAT_D32_FLOAT;
constexpr DXGI_FORMAT hdrFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
struct FrameConstants {
    float camera[40]{};
    float shadowRows[4][3][4]{};
    float splits[4]{}, texels[4]{}, biases[4]{}, shadow[4]{}, render[4]{}, scattering[4]{}, terrain[4]{};
    float cloudLayer[4]{}, cloudShape[4]{}, cloudRender[4]{}, cloudWind[4]{};
};
static_assert(sizeof(FrameConstants) == 132 * sizeof(float));

struct RelativeBounds {
    Vec center, extent;
    RelativeBounds(Box box, Vec eye) : center(box.center() - eye), extent((box.hi - box.lo) * 0.5) {
        // Allow for the GPU's final conversion of camera-relative coordinates to float.
        extent = extent + splat(1.0 + 4e-7 * length(center));
    }
    double projectionRadius(Vec axis) const { return dot(extent, abs(axis)); }
    double maximum(Vec axis) const { return dot(center, axis) + projectionRadius(axis); }
    bool visible(const Camera& camera, double xScale, double yScale) const {
        return maximum(camera.forward) >= camera.nearPlane && -maximum(camera.forward * -1) <= camera.farPlane &&
               maximum(camera.forward + camera.right * xScale) >= 0 && maximum(camera.forward - camera.right * xScale) >= 0 &&
               maximum(camera.forward + camera.up * yScale) >= 0 && maximum(camera.forward - camera.up * yScale) >= 0;
    }
    bool visible(const ShadowCascade& cascade) const {
        for (int axis = 0; axis < 3; ++axis) {
            const auto& row = cascade.rows[axis];
            const Vec direction{row[0], row[1], row[2]};
            const double p = dot(center, direction) + row[3], r = projectionRadius(direction);
            if (p - r > 1 || p + r < (axis == 2 ? 0 : -1)) return false;
        }
        return true;
    }
};

double halfValue(std::uint16_t bits) {
    const unsigned exponent = (bits >> 10) & 31, mantissa = bits & 1023;
    const double sign = bits & 0x8000 ? -1.0 : 1.0;
    if (exponent == 31) return mantissa ? std::numeric_limits<double>::quiet_NaN() : sign * std::numeric_limits<double>::infinity();
    return sign * (exponent ? std::ldexp(1.0 + mantissa / 1024.0, int(exponent) - 15)
                           : std::ldexp(double(mantissa), -24));
}

void require(HRESULT result, const char* operation) {
    if (SUCCEEDED(result)) return;
    std::ostringstream error;
    error << operation << " (HRESULT 0x" << std::hex << static_cast<std::uint32_t>(result) << ')';
    throw std::runtime_error(error.str());
}

ComPtr<ID3DBlob> compile(const char* entry, const char* profile) {
    ComPtr<ID3DBlob> shader, message;
    const HRESULT result = D3DCompile(shaderSource, sizeof(shaderSource) - 1, "VoxelWorld", nullptr, nullptr,
                                      entry, profile, D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_IEEE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3,
                                      0, shader.GetAddressOf(), message.GetAddressOf());
    if (FAILED(result) && message) {
        throw std::runtime_error(std::string(static_cast<const char*>(message->GetBufferPointer()), message->GetBufferSize()));
    }
    require(result, entry);
    return shader;
}

D3D12_HEAP_PROPERTIES heap(D3D12_HEAP_TYPE type) {
    return {type, D3D12_CPU_PAGE_PROPERTY_UNKNOWN, D3D12_MEMORY_POOL_UNKNOWN, 1, 1};
}

D3D12_RESOURCE_DESC buffer(UINT64 size) {
    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width = size;
    desc.Height = 1;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    return desc;
}

void transition(ID3D12GraphicsCommandList& list, ID3D12Resource* resource,
                D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
    D3D12_RESOURCE_BARRIER barrier{};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition = {resource, D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES, before, after};
    list.ResourceBarrier(1, &barrier);
}

void writeBitmap(const std::filesystem::path& path, const std::uint8_t* bytes, UINT stride, UINT width, UINT height) {
    BITMAPFILEHEADER header{};
    BITMAPINFOHEADER info{};
    info.biSize = sizeof(info);
    info.biWidth = LONG(width);
    info.biHeight = -LONG(height);
    info.biPlanes = 1;
    info.biBitCount = 32;
    info.biCompression = BI_RGB;
    info.biSizeImage = width * height * 4;
    header.bfType = 0x4d42;
    header.bfOffBits = sizeof(header) + sizeof(info);
    header.bfSize = header.bfOffBits + info.biSizeImage;
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(&header), sizeof(header));
    file.write(reinterpret_cast<const char*>(&info), sizeof(info));
    std::vector<std::uint8_t> row(std::size_t(width) * 4);
    for (UINT y = 0; y < height; ++y) {
        std::memcpy(row.data(), bytes + std::size_t(y) * stride, row.size());
        for (std::size_t x = 0; x < row.size(); x += 4) std::swap(row[x], row[x + 2]);
        file.write(reinterpret_cast<const char*>(row.data()), std::streamsize(row.size()));
    }
}

}  // namespace

struct Dx12Renderer::Impl {
    struct Upload {
        ComPtr<ID3D12Resource> resource;
        UINT64 capacity{};
        D3D12_VERTEX_BUFFER_VIEW view{};
        UINT count{};
    };
    struct TerrainPage {
        Box bounds;
        Upload staging;
        ComPtr<ID3D12Resource> vertices;
        D3D12_VERTEX_BUFFER_VIEW view{};
        bool dirty{};
    };
    struct Readback { ComPtr<ID3D12Resource> resource; D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{}; };

    Impl(HWND target, bool forceWarp, bool debug) : window(target) {
        ComPtr<ID3D12Debug> layer;
        if (debug && SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&layer)))) {
            layer->EnableDebugLayer();
            debugLayer = true;
        }
        ComPtr<IDXGIFactory4> factory;
        require(CreateDXGIFactory2(0, IID_PPV_ARGS(factory.GetAddressOf())), "Create DXGI factory");
        warp = forceWarp || FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device.GetAddressOf())));
        if (warp) {
            ComPtr<IDXGIAdapter> adapter;
            require(factory->EnumWarpAdapter(IID_PPV_ARGS(adapter.GetAddressOf())), "Find Direct3D 12 WARP adapter");
            require(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device.GetAddressOf())), "Create Direct3D 12 WARP device");
        }
        if (debugLayer) require(device.As(&infoQueue), "Get D3D12 validation queue");
        D3D12_COMMAND_QUEUE_DESC queueDesc{};
        require(device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(queue.GetAddressOf())), "Create command queue");
        RECT rect{};
        GetClientRect(window, &rect);
        width = std::max<LONG>(1, rect.right);
        height = std::max<LONG>(1, rect.bottom);
        DXGI_SWAP_CHAIN_DESC1 swapDesc{};
        swapDesc.Width = width;
        swapDesc.Height = height;
        swapDesc.Format = pixelFormat;
        swapDesc.SampleDesc.Count = 1;
        swapDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapDesc.BufferCount = buffers;
        swapDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        ComPtr<IDXGISwapChain1> initialSwap;
        require(factory->CreateSwapChainForHwnd(queue.Get(), window, &swapDesc, nullptr, nullptr, initialSwap.GetAddressOf()), "Create swap chain");
        require(factory->MakeWindowAssociation(GetAncestor(window, GA_ROOT), DXGI_MWA_NO_ALT_ENTER), "Set DXGI window association");
        require(initialSwap.As(&swap), "Get DXGI swap chain interface");

        D3D12_DESCRIPTOR_HEAP_DESC desc{};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        desc.NumDescriptors = buffers + 5;
        require(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(rtv.GetAddressOf())), "Create render target descriptor heap");
        rtvSize = device->GetDescriptorHandleIncrementSize(desc.Type);
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        desc.NumDescriptors = 1 + shadowCascadeCount;
        require(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(dsv.GetAddressOf())), "Create depth descriptor heap");
        dsvSize = device->GetDescriptorHandleIncrementSize(desc.Type);
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        desc.NumDescriptors = 9;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        require(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(srv.GetAddressOf())), "Create shader resource heap");
        srvSize = device->GetDescriptorHandleIncrementSize(desc.Type);
        createTargets();
        createPipelines();
        require(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(allocator.GetAddressOf())), "Create command allocator");
        require(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(list.GetAddressOf())), "Create command list");
        require(list->Close(), "Close initial command list");
        require(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(fence.GetAddressOf())), "Create frame fence");
        completed = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        if (!completed) throw std::runtime_error("Create frame event failed");
        createCloudNoise();
    }

    ~Impl() {
        if (submitted) {
            // A failed Present can leave submitted commands in flight.
            try { wait(); } catch (...) {}
        }
        CloseHandle(completed);
    }

    D3D12_CPU_DESCRIPTOR_HANDLE target(UINT index) const {
        auto handle = rtv->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += SIZE_T(index) * rtvSize;
        return handle;
    }
    D3D12_CPU_DESCRIPTOR_HANDLE depthTarget(UINT index) const {
        auto h = dsv->GetCPUDescriptorHandleForHeapStart(); h.ptr += SIZE_T(index) * dsvSize; return h;
    }
    D3D12_CPU_DESCRIPTOR_HANDLE shaderView(UINT index) const {
        auto h = srv->GetCPUDescriptorHandleForHeapStart(); h.ptr += SIZE_T(index) * srvSize; return h;
    }

    void texture(ComPtr<ID3D12Resource>& resource, UINT w, UINT h, UINT16 layers, DXGI_FORMAT format,
                 D3D12_RESOURCE_FLAGS flags, const D3D12_CLEAR_VALUE& clear) {
        auto desc = buffer(w);
        desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Height = h; desc.DepthOrArraySize = layers; desc.Format = format;
        desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN; desc.Flags = flags;
        const auto properties = heap(D3D12_HEAP_TYPE_DEFAULT);
        require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, &clear, IID_PPV_ARGS(resource.ReleaseAndGetAddressOf())), "Create render texture");
    }

    void createTargets() {
        for (UINT i = 0; i < buffers; ++i) {
            require(swap->GetBuffer(i, IID_PPV_ARGS(back[i].GetAddressOf())), "Get swap chain buffer");
            device->CreateRenderTargetView(back[i].Get(), nullptr, target(i));
        }
        D3D12_CLEAR_VALUE clear{};
        clear.Format = pixelFormat;
        texture(albedo, width, height, 1, pixelFormat, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET, clear);
        device->CreateRenderTargetView(albedo.Get(), nullptr, target(buffers));
        clear.Format = hdrFormat;
        texture(normal, width, height, 1, hdrFormat, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET, clear);
        device->CreateRenderTargetView(normal.Get(), nullptr, target(buffers + 1));
        texture(hdr, width, height, 1, hdrFormat, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET, clear);
        device->CreateRenderTargetView(hdr.Get(), nullptr, target(buffers + 2));
        clear = {}; clear.Format = depthFormat; clear.DepthStencil.Depth = 0;
        texture(depth, width, height, 1, DXGI_FORMAT_R32_TYPELESS, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL, clear);
        D3D12_DEPTH_STENCIL_VIEW_DESC view{};
        view.Format = depthFormat; view.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
        device->CreateDepthStencilView(depth.Get(), &view, depthTarget(0));
        D3D12_SHADER_RESOURCE_VIEW_DESC shader{};
        shader.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        shader.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        shader.Texture2D.MipLevels = 1;
        shader.Format = pixelFormat;
        device->CreateShaderResourceView(albedo.Get(), &shader, shaderView(0));
        shader.Format = hdrFormat;
        device->CreateShaderResourceView(normal.Get(), &shader, shaderView(1));
        device->CreateShaderResourceView(hdr.Get(), &shader, shaderView(4));
        shader.Format = DXGI_FORMAT_R32_FLOAT;
        device->CreateShaderResourceView(depth.Get(), &shader, shaderView(2));
        texture(cloudLight, (width + 1) / 2, (height + 1) / 2, 1, hdrFormat, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                D3D12_CLEAR_VALUE{hdrFormat, {0, 0, 0, 1}});
        device->CreateRenderTargetView(cloudLight.Get(), nullptr, target(buffers + 3));
        D3D12_CLEAR_VALUE cloudClear{}; cloudClear.Format = DXGI_FORMAT_R32G32_FLOAT;
        texture(cloudDepth, (width + 1) / 2, (height + 1) / 2, 1, DXGI_FORMAT_R32G32_FLOAT,
                D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET, cloudClear);
        device->CreateRenderTargetView(cloudDepth.Get(), nullptr, target(buffers + 4));
        shader.Format = hdrFormat;
        device->CreateShaderResourceView(cloudLight.Get(), &shader, shaderView(7));
        shader.Format = DXGI_FORMAT_R32G32_FLOAT;
        device->CreateShaderResourceView(cloudDepth.Get(), &shader, shaderView(8));
        shader.Format = pixelFormat;
        device->CreateShaderResourceView(nullptr, &shader, shaderView(5)); // Reserved by D3D11 precision tests.
    }

    void createCloudNoise() {
        const auto mips = makeCloudNoise();
        auto desc = buffer(mips.front().edge);
        desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
        desc.Height = mips.front().edge; desc.DepthOrArraySize = UINT16(mips.front().edge);
        desc.MipLevels = UINT16(mips.size()); desc.Format = pixelFormat;
        desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        const auto properties = heap(D3D12_HEAP_TYPE_DEFAULT);
        require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &desc,
            D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&cloudNoise)), "Create cloud noise volume");
        std::vector<D3D12_PLACED_SUBRESOURCE_FOOTPRINT> layouts(mips.size());
        UINT64 bytes = 0;
        device->GetCopyableFootprints(&desc, 0, UINT(mips.size()), 0, layouts.data(), nullptr, nullptr, &bytes);
        ComPtr<ID3D12Resource> staging;
        const auto uploadHeap = heap(D3D12_HEAP_TYPE_UPLOAD);
        const auto uploadDesc = buffer(bytes);
        require(device->CreateCommittedResource(&uploadHeap, D3D12_HEAP_FLAG_NONE, &uploadDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&staging)), "Create cloud noise upload");
        void* data = nullptr; const D3D12_RANGE noReads{0, 0};
        require(staging->Map(0, &noReads, &data), "Map cloud noise upload");
        for (std::size_t i = 0; i < mips.size(); ++i) {
            const auto& mip = mips[i]; const auto& layout = layouts[i];
            for (unsigned z = 0; z < mip.edge; ++z) for (unsigned y = 0; y < mip.edge; ++y)
                std::memcpy(static_cast<std::uint8_t*>(data) + layout.Offset +
                    (SIZE_T(z) * mip.edge + y) * layout.Footprint.RowPitch,
                    mip.rgba.data() + (SIZE_T(z) * mip.edge + y) * mip.edge * 4, mip.edge * 4);
        }
        staging->Unmap(0, nullptr);
        require(allocator->Reset(), "Reset cloud upload allocator");
        require(list->Reset(allocator.Get(), nullptr), "Reset cloud upload commands");
        for (UINT i = 0; i < UINT(mips.size()); ++i) {
            D3D12_TEXTURE_COPY_LOCATION from{}, to{};
            from.pResource = staging.Get(); from.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            from.PlacedFootprint = layouts[i];
            to.pResource = cloudNoise.Get(); to.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX; to.SubresourceIndex = i;
            list->CopyTextureRegion(&to, 0, 0, 0, &from, nullptr);
        }
        transition(*list.Get(), cloudNoise.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        require(list->Close(), "Close cloud upload commands");
        ID3D12CommandList* commands[]{list.Get()}; queue->ExecuteCommandLists(1, commands); submitted = true;
        wait();
        D3D12_SHADER_RESOURCE_VIEW_DESC view{};
        view.Format = pixelFormat; view.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
        view.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        view.Texture3D.MipLevels = UINT(mips.size());
        device->CreateShaderResourceView(cloudNoise.Get(), &view, shaderView(6));
    }

    void setShadowResolution(int resolution) {
        if (!validShadowResolution(resolution)) throw std::invalid_argument("Shadow resolution must be 1024, 2048 or 4096");
        if (shadowResolution == UINT(resolution)) return;
        wait();
        D3D12_CLEAR_VALUE clear{};
        clear.Format = depthFormat;
        clear.DepthStencil.Depth = 1;
        ComPtr<ID3D12Resource> next;
        texture(next, UINT(resolution), UINT(resolution), shadowCascadeCount, DXGI_FORMAT_R32_TYPELESS,
                D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL, clear);
        D3D12_DEPTH_STENCIL_VIEW_DESC view{};
        view.Format = depthFormat;
        view.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DARRAY;
        view.Texture2DArray.ArraySize = 1;
        for (UINT i = 0; i < shadowCascadeCount; ++i) {
            view.Texture2DArray.FirstArraySlice = i;
            device->CreateDepthStencilView(next.Get(), &view, depthTarget(i + 1));
        }
        D3D12_SHADER_RESOURCE_VIEW_DESC shader{};
        shader.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        shader.Format = DXGI_FORMAT_R32_FLOAT;
        shader.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
        shader.Texture2DArray.MipLevels = 1;
        shader.Texture2DArray.ArraySize = shadowCascadeCount;
        device->CreateShaderResourceView(next.Get(), &shader, shaderView(3));
        shadows = std::move(next);
        shadowResolution = UINT(resolution);
    }

    void createPipelines() {
        D3D12_DESCRIPTOR_RANGE range{};
        range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV; range.NumDescriptors = 9;
        D3D12_ROOT_PARAMETER parameters[3]{};
        parameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        parameters[0].Descriptor.ShaderRegister = 0;
        parameters[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        parameters[1].Constants.ShaderRegister = 1; parameters[1].Constants.Num32BitValues = 1;
        parameters[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        parameters[2].DescriptorTable.NumDescriptorRanges = 1; parameters[2].DescriptorTable.pDescriptorRanges = &range;
        D3D12_STATIC_SAMPLER_DESC sampler{};
        sampler.Filter = D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
        sampler.AddressU = sampler.AddressV = sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
        sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
        sampler.BorderColor = D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE;
        sampler.MaxLOD = D3D12_FLOAT32_MAX;
        sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        D3D12_ROOT_SIGNATURE_DESC desc{};
        desc.NumParameters = UINT(std::size(parameters)); desc.pParameters = parameters;
        D3D12_STATIC_SAMPLER_DESC samplers[]{sampler, sampler};
        samplers[1].ShaderRegister = 1;
        samplers[1].Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
        samplers[1].AddressU = samplers[1].AddressV = samplers[1].AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
        samplers[1].ComparisonFunc = D3D12_COMPARISON_FUNC_ALWAYS;
        desc.NumStaticSamplers = UINT(std::size(samplers)); desc.pStaticSamplers = samplers;
        desc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
        ComPtr<ID3DBlob> data;
        require(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &data, nullptr), "Serialize root signature");
        require(device->CreateRootSignature(0, data->GetBufferPointer(), data->GetBufferSize(), IID_PPV_ARGS(&root)), "Create root signature");
        const auto properties = heap(D3D12_HEAP_TYPE_UPLOAD);
        const auto constantDesc = buffer((sizeof(FrameConstants) + 255) & ~UINT64(255));
        require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &constantDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&frameUpload)), "Allocate frame constants");
        constexpr D3D12_INPUT_ELEMENT_DESC inputs[] = {
            {"POSITION",0,DXGI_FORMAT_R32G32B32_FLOAT,0,0,D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,0},
            {"POSITION",1,DXGI_FORMAT_R32G32B32_FLOAT,0,12,D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,0},
            {"NORMAL",0,DXGI_FORMAT_R32G32B32_FLOAT,0,24,D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,0},
            {"COLOR",0,pixelFormat,0,36,D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,0}
        };
        static_assert(sizeof(GpuVertex) == 40);
        const auto vs = compile("surfaceVS", "vs_5_0"), ps = compile("surfacePS", "ps_5_0");
        D3D12_GRAPHICS_PIPELINE_STATE_DESC state{};
        state.pRootSignature = root.Get();
        state.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
        state.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
        state.SampleMask = std::numeric_limits<UINT>::max();
        state.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        state.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        state.RasterizerState.DepthClipEnable = TRUE;
        state.BlendState.RenderTarget[0] = {FALSE,FALSE,D3D12_BLEND_ONE,D3D12_BLEND_ZERO,D3D12_BLEND_OP_ADD,
            D3D12_BLEND_ONE,D3D12_BLEND_ZERO,D3D12_BLEND_OP_ADD,D3D12_LOGIC_OP_NOOP,D3D12_COLOR_WRITE_ENABLE_ALL};
        state.BlendState.RenderTarget[1] = state.BlendState.RenderTarget[0];
        state.DepthStencilState.DepthEnable = TRUE;
        state.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        state.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_GREATER_EQUAL;
        state.DepthStencilState.FrontFace = {D3D12_STENCIL_OP_KEEP,D3D12_STENCIL_OP_KEEP,D3D12_STENCIL_OP_KEEP,D3D12_COMPARISON_FUNC_ALWAYS};
        state.DepthStencilState.BackFace = state.DepthStencilState.FrontFace;
        state.InputLayout = {inputs, UINT(std::size(inputs))};
        state.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        state.NumRenderTargets = 2; state.RTVFormats[0] = pixelFormat; state.RTVFormats[1] = hdrFormat;
        state.DSVFormat = depthFormat; state.SampleDesc.Count = 1;
        auto create = [&](const D3D12_GRAPHICS_PIPELINE_STATE_DESC& pso, ComPtr<ID3D12PipelineState>& output, const char* name) {
            require(device->CreateGraphicsPipelineState(&pso, IID_PPV_ARGS(output.ReleaseAndGetAddressOf())), name);
        };
        create(state, surfacePipeline, "Create G-buffer pipeline");
        auto depthOnly = state;
        depthOnly.PS = {}; depthOnly.NumRenderTargets = 0;
        for (auto& format : depthOnly.RTVFormats) format = DXGI_FORMAT_UNKNOWN;
        create(depthOnly, depthPipeline, "Create reverse-Z depth prepass");
        const auto shadowVS = compile("shadowVS", "vs_5_0");
        depthOnly.VS = {shadowVS->GetBufferPointer(), shadowVS->GetBufferSize()};
        depthOnly.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
        depthOnly.RasterizerState.DepthBias = 2; depthOnly.RasterizerState.SlopeScaledDepthBias = 1.5f;
        create(depthOnly, shadowPipeline, "Create CSM depth pipeline");
        auto wire = state;
        wire.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
        wire.RasterizerState.DepthBias = 2; wire.RasterizerState.SlopeScaledDepthBias = 1;
        create(wire, wireframePipeline, "Create G-buffer wireframe pipeline");
        auto overlay = state;
        overlay.NumRenderTargets = 1; overlay.RTVFormats[0] = hdrFormat; overlay.RTVFormats[1] = DXGI_FORMAT_UNKNOWN;
        overlay.DepthStencilState.DepthEnable = FALSE;
        overlay.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
        overlay.DSVFormat = DXGI_FORMAT_UNKNOWN;
        overlay.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
        const auto linePS = compile("linePS", "ps_5_0");
        overlay.PS = {linePS->GetBufferPointer(), linePS->GetBufferSize()};
        create(overlay, linePipeline, "Create HDR line overlay pipeline");
        overlay.InputLayout = {};
        overlay.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        const auto screenVS = compile("skyVS", "vs_5_0"), lightingPS = compile("deferredPS", "ps_5_0");
        overlay.VS = {screenVS->GetBufferPointer(), screenVS->GetBufferSize()};
        overlay.PS = {lightingPS->GetBufferPointer(), lightingPS->GetBufferSize()};
        create(overlay, lightingPipeline, "Create deferred HDR lighting pipeline");
        auto clouds = overlay;
        const auto cloudPS = compile("cloudPS", "ps_5_0");
        clouds.PS = {cloudPS->GetBufferPointer(), cloudPS->GetBufferSize()};
        clouds.NumRenderTargets = 2; clouds.RTVFormats[1] = DXGI_FORMAT_R32G32_FLOAT;
        create(clouds, cloudPipeline, "Create half-resolution planetary cloud pipeline");
        const auto postPS = compile("toneMapPS", "ps_5_0");
        overlay.PS = {postPS->GetBufferPointer(), postPS->GetBufferSize()};
        overlay.RTVFormats[0] = pixelFormat;
        create(overlay, tonePipeline, "Create HDR tone mapping pipeline");
    }

    void wait() {
        if (!submitted) return;
        ++lastFence;
        require(queue->Signal(fence.Get(), lastFence), "Signal frame fence");
        if (fence->GetCompletedValue() < lastFence) {
            require(fence->SetEventOnCompletion(lastFence, completed), "Wait for frame fence");
            if (WaitForSingleObject(completed, 15000) != WAIT_OBJECT_0) throw std::runtime_error("Direct3D 12 frame timed out");
        }
        require(device->GetDeviceRemovedReason(), "Direct3D 12 device was removed");
        submitted = false;
        if (infoQueue) {
            for (UINT64 i = 0; i < infoQueue->GetNumStoredMessagesAllowedByRetrievalFilter(); ++i) {
                SIZE_T length = 0; infoQueue->GetMessage(i, nullptr, &length);
                std::vector<std::uint8_t> storage(length);
                auto* message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
                if (SUCCEEDED(infoQueue->GetMessage(i, message, &length)) &&
                    message->Severity <= D3D12_MESSAGE_SEVERITY_ERROR)
                    throw std::runtime_error(std::string("D3D12 validation: ") + message->pDescription);
            }
        }
    }

    template<class VertexType>
    void upload(Upload& result, const std::vector<VertexType>& vertices) {
        const UINT64 size = UINT64(vertices.size()) * sizeof(GpuVertex);
        if (size > std::numeric_limits<UINT>::max()) throw std::runtime_error("Vertex buffer is too large");
        result.count = UINT(vertices.size());
        if (!size) return;
        if (result.capacity < size) {
            const auto properties = heap(D3D12_HEAP_TYPE_UPLOAD);
            const auto desc = buffer(std::max(size, result.capacity + result.capacity / 2));
            require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_GENERIC_READ,
                                                     nullptr, IID_PPV_ARGS(result.resource.ReleaseAndGetAddressOf())), "Allocate vertex buffer");
            result.capacity = desc.Width;
        }
        void* pointer = nullptr;
        const D3D12_RANGE noReads{0, 0};
        require(result.resource->Map(0, &noReads, &pointer), "Map vertex buffer");
        if constexpr (std::is_same_v<VertexType, GpuVertex>) {
            std::memcpy(pointer, vertices.data(), SIZE_T(size));
        } else {
            auto* packed = static_cast<GpuVertex*>(pointer);
            for (const auto& vertex : vertices) *packed++ = packVertex(vertex);
        }
        const D3D12_RANGE written{0, SIZE_T(size)};
        result.resource->Unmap(0, &written);
        result.view = {result.resource->GetGPUVirtualAddress(), UINT(size), UINT(sizeof(GpuVertex))};
    }

    void copyTerrainPages(std::map<Key, TerrainPage>& pages) {
        for (auto& item : pages) {
            auto& page = item.second;
            if (!page.dirty) continue;
            if (!page.vertices || page.vertices->GetDesc().Width < page.staging.view.SizeInBytes) {
                const auto properties = heap(D3D12_HEAP_TYPE_DEFAULT);
                const auto desc = buffer(page.staging.capacity);
                require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &desc,
                    D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER, nullptr,
                    IID_PPV_ARGS(page.vertices.ReleaseAndGetAddressOf())), "Allocate device terrain buffer");
            }
            const auto size = page.staging.view.SizeInBytes;
            transition(*list.Get(), page.vertices.Get(), D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER, D3D12_RESOURCE_STATE_COPY_DEST);
            list->CopyBufferRegion(page.vertices.Get(), 0, page.staging.resource.Get(), 0, size);
            transition(*list.Get(), page.vertices.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
            page.view = {page.vertices->GetGPUVirtualAddress(), size, UINT(sizeof(GpuVertex))};
            page.dirty = false;
        }
    }

    Readback copyReadback(ID3D12Resource* source, UINT subresource, D3D12_RESOURCE_STATES state) {
        Readback result;
        UINT64 bytes = 0;
        const auto desc = source->GetDesc();
        device->GetCopyableFootprints(&desc, subresource, 1, 0, &result.footprint, nullptr, nullptr, &bytes);
        const auto properties = heap(D3D12_HEAP_TYPE_READBACK);
        const auto destination = buffer(bytes);
        require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &destination,
            D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&result.resource)), "Allocate pipeline readback");
        D3D12_TEXTURE_COPY_LOCATION from{}, to{};
        from.pResource = source; from.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX; from.SubresourceIndex = subresource;
        to.pResource = result.resource.Get(); to.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT; to.PlacedFootprint = result.footprint;
        transition(*list.Get(), source, state, D3D12_RESOURCE_STATE_COPY_SOURCE);
        list->CopyTextureRegion(&to, 0, 0, 0, &from, nullptr);
        transition(*list.Get(), source, D3D12_RESOURCE_STATE_COPY_SOURCE, state);
        return result;
    }

    template<class Operation> void readRows(const Readback& source, Operation operation) {
        void* mapped = nullptr;
        require(source.resource->Map(0, nullptr, &mapped), "Map pipeline readback");
        const auto* bytes = static_cast<const std::uint8_t*>(mapped) + source.footprint.Offset;
        for (UINT y = 0; y < source.footprint.Footprint.Height; ++y)
            operation(y, bytes + SIZE_T(y) * source.footprint.Footprint.RowPitch);
        const D3D12_RANGE noWrites{0,0};
        source.resource->Unmap(0, &noWrites);
    }

    void inspect(const Readback& color, const Readback& sceneDepth, const Readback& normals,
                 const std::array<Readback, 4>& maps, const Readback& clouds, const std::filesystem::path& capture) {
        lastDiagnostics = {}; lastDiagnostics.debugLayer = debugLayer;
        lastDiagnostics.shadowResolution = maps.front().footprint.Footprint.Width;
        readRows(color, [&](UINT, const std::uint8_t* row) {
            const auto* values = reinterpret_cast<const std::uint16_t*>(row);
            for (UINT x = 0; x < width; ++x) for (int c = 0; c < 3; ++c) {
                const double value = halfValue(values[x * 4 + c]);
                if (!std::isfinite(value) || value < 0) ++lastDiagnostics.invalidValues;
                else lastDiagnostics.maxHdrComponent = std::max(lastDiagnostics.maxHdrComponent, value);
            }
        });
        readRows(sceneDepth, [&](UINT, const std::uint8_t* row) {
            const auto* values = reinterpret_cast<const float*>(row);
            for (UINT x = 0; x < width; ++x) {
                if (!std::isfinite(values[x]) || values[x] < 0 || values[x] > 1) ++lastDiagnostics.invalidValues;
                if (values[x] > 0) ++lastDiagnostics.geometryPixels;
            }
        });
        readRows(normals, [&](UINT, const std::uint8_t* row) {
            const auto* values = reinterpret_cast<const std::uint16_t*>(row);
            for (UINT x = 0; x < width * 4; ++x)
                if (!std::isfinite(halfValue(values[x]))) ++lastDiagnostics.invalidValues;
        });
        for (int cascade = 0; cascade < shadowCascadeCount; ++cascade) {
            readRows(maps[cascade], [&](UINT, const std::uint8_t* row) {
                const auto* values = reinterpret_cast<const float*>(row);
                for (UINT x = 0; x < maps[cascade].footprint.Footprint.Width; ++x) {
                    if (!std::isfinite(values[x]) || values[x] < 0 || values[x] > 1) ++lastDiagnostics.invalidValues;
                    if (values[x] < 1) ++lastDiagnostics.shadowPixels[cascade];
                }
            });
        }
        if (clouds.resource) {
            const auto w = clouds.footprint.Footprint.Width, h = clouds.footprint.Footprint.Height;
            std::vector<std::uint8_t> opacity(SIZE_T(w) * h * 4);
            readRows(clouds, [&](UINT y, const std::uint8_t* row) {
                const auto* values = reinterpret_cast<const std::uint16_t*>(row);
                for (UINT x = 0; x < w; ++x) {
                    for (int c = 0; c < 4; ++c) if (!std::isfinite(halfValue(values[x * 4 + c])))
                        ++lastDiagnostics.invalidValues;
                    const double t = halfValue(values[x * 4 + 3]);
                    if (t < 0 || t > 1) ++lastDiagnostics.invalidValues;
                    const double alpha = std::clamp(1 - t, 0.0, 1.0);
                    lastDiagnostics.cloudPixels += alpha > 0.02;
                    lastDiagnostics.maxCloudOpacity = std::max(lastDiagnostics.maxCloudOpacity, alpha);
                    const auto byte = std::uint8_t(std::lround(alpha * 255));
                    const auto i = (SIZE_T(y) * w + x) * 4;
                    opacity[i] = opacity[i + 1] = opacity[i + 2] = byte; opacity[i + 3] = 255;
                }
            });
            auto path = capture; path.replace_extension(".cloud-opacity.bmp");
            writeBitmap(path, opacity.data(), w * 4, w, h);
        }
        auto path = capture; path.replace_extension(".pipeline.txt");
        std::ofstream report(path);
        report << "gbuffer_geometry_pixels=" << lastDiagnostics.geometryPixels
               << "\nhdr_max_component=" << lastDiagnostics.maxHdrComponent
               << "\ninvalid_gpu_values=" << lastDiagnostics.invalidValues
               << "\nshadow_resolution=" << lastDiagnostics.shadowResolution
               << "\nd3d12_debug_layer=" << debugLayer
               << "\ncloud_pixels=" << lastDiagnostics.cloudPixels
               << "\ncloud_max_opacity=" << lastDiagnostics.maxCloudOpacity << '\n';
        for (int i = 0; i < shadowCascadeCount; ++i) report << "cascade_" << i << "_depth_pixels=" << lastDiagnostics.shadowPixels[i] << '\n';
        if (lastDiagnostics.invalidValues) throw std::runtime_error("Pipeline validation found invalid GPU values");
    }

    void draw(const Camera& camera, const std::vector<Vertex>& vertices, const std::filesystem::path& capture, bool inspectPipeline) {
        RECT rect{}; GetClientRect(window, &rect);
        if (rect.right <= 0 || rect.bottom <= 0) return;
        if (width != UINT(rect.right) || height != UINT(rect.bottom)) {
            wait();
            for (auto& resource : back) resource.Reset();
            width = UINT(rect.right); height = UINT(rect.bottom);
            require(swap->ResizeBuffers(buffers, width, height, pixelFormat, 0), "Resize swap chain");
            createTargets();
        }
        if (infoQueue) infoQueue->ClearStoredMessages();
        const auto& settings = camera.settings;
        const auto air = atmosphereDimensions(settings);
        setShadowResolution(settings.shadowResolution);
        upload(objects, vertices);
        FrameConstants frame{};
        const Vec sun = unit(Vec{0.45,0.25,0.86});
        const auto ex = splitDouble(camera.eye.x), ey = splitDouble(camera.eye.y), ez = splitDouble(camera.eye.z);
        const Vec planetEye = camera.eye - camera.planetCenter;
        const auto px = splitDouble(planetEye.x), py = splitDouble(planetEye.y), pz = splitDouble(planetEye.z);
        const float xScale = float(1.92 * double(height) / width), yScale = 1.92f;
        const float constants[] = {
            ex.hi,ey.hi,ez.hi,0,ex.lo,ey.lo,ez.lo,0,
            float(camera.right.x),float(camera.right.y),float(camera.right.z),settings.atmosphereEnabled ? 1.0f : 0.0f,
            float(camera.up.x),float(camera.up.y),float(camera.up.z),0,
            float(camera.forward.x),float(camera.forward.y),float(camera.forward.z),0,
            xScale,yScale,float(camera.nearPlane),float(camera.farPlane),
            float(sun.x),float(sun.y),float(sun.z),camera.lodTint ? 1.0f : 0.0f,
            px.hi,py.hi,pz.hi,0,px.lo,py.lo,pz.lo,0,
            float(planetRadius),float(planetRadius + air.thickness),
            float(air.rayleighScale),float(air.mieScale)
        };
        std::copy(std::begin(constants), std::end(constants), frame.camera);
        const auto csm = buildShadowCascades(planetEye, camera.right, camera.up, camera.forward,
            camera.nearPlane, camera.farPlane, double(width) / height, sun, settings);
        for (int i = 0; i < shadowCascadeCount; ++i) {
            for (int row = 0; row < 3; ++row) for (int c = 0; c < 4; ++c)
                frame.shadowRows[i][row][c] = float(csm.cascades[i].rows[row][c]);
            frame.splits[i] = float(csm.cascades[i].farDistance);
            frame.texels[i] = float(csm.cascades[i].texelSize);
            frame.biases[i] = float(csm.cascades[i].depthBias);
        }
        frame.shadow[0] = settings.shadowsEnabled && csm.enabled ? 1.0f : 0.0f;
        frame.shadow[1] = 1.0f / shadowResolution;
        frame.shadow[2] = float(settings.shadowNormalBias);
        frame.render[0] = float(width); frame.render[1] = float(height);
        frame.render[2] = float(std::exp2(settings.exposureEV));
        frame.render[3] = float(settings.toneMapping);
        frame.scattering[0] = float(settings.rayleighStrength);
        frame.scattering[1] = float(settings.mieStrength);
        frame.scattering[2] = float(settings.mieAnisotropy);
        frame.terrain[0] = float(camera.terrain);
        frame.terrain[1] = float(camera.terrainSeed);
        frame.terrain[2] = float(elevationRange(camera.terrain).lower);
        frame.cloudLayer[0] = float(planetRadius + 1000 * settings.cloudBaseKm);
        frame.cloudLayer[1] = float(planetRadius + 1000 * (settings.cloudBaseKm + settings.cloudThicknessKm));
        frame.cloudLayer[2] = float(settings.cloudCoverage);
        frame.cloudLayer[3] = float(settings.cloudDensity * 0.00035);
        frame.cloudShape[0] = float(settings.cloudWeatherKm * 1000);
        frame.cloudShape[1] = float(settings.cloudFormationKm * 1000);
        frame.cloudRender[0] = settings.cloudsEnabled ? 1.0f : 0.0f;
        frame.cloudRender[1] = float(settings.cloudSteps);
        frame.cloudRender[2] = float((width + 1) / 2); frame.cloudRender[3] = float((height + 1) / 2);
        constexpr double windAxes[]{1, 0.37, 0.13};
        for (int i = 0; i < 3; ++i) frame.cloudWind[i] = float(std::fmod(
            camera.timeSeconds * settings.cloudWindSpeed * windAxes[i], settings.cloudFormationKm * 8000));
        void* mapped = nullptr; const D3D12_RANGE noReads{0,0};
        require(frameUpload->Map(0, &noReads, &mapped), "Map frame constants");
        std::memcpy(mapped, &frame, sizeof(frame));
        frameUpload->Unmap(0, nullptr);
        require(allocator->Reset(), "Reset command allocator");
        require(list->Reset(allocator.Get(), nullptr), "Reset command list");
        if (terrainCopiesPending) {
            copyTerrainPages(terrainPages);
            copyTerrainPages(stagedTerrainPages);
            terrainCopiesPending = false;
        }
        list->SetGraphicsRootSignature(root.Get());
        ID3D12DescriptorHeap* heaps[]{srv.Get()}; list->SetDescriptorHeaps(1, heaps);
        list->SetGraphicsRootConstantBufferView(0, frameUpload->GetGPUVirtualAddress());
        list->SetGraphicsRoot32BitConstant(1, 0, 0);
        list->SetGraphicsRootDescriptorTable(2, srv->GetGPUDescriptorHandleForHeapStart());
        list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        std::vector<const TerrainPage*> visible;
        std::array<std::vector<const TerrainPage*>, shadowCascadeCount> casters;
        for (const auto& item : terrainPages) {
            const RelativeBounds box(item.second.bounds, camera.eye);
            if (box.visible(camera, xScale, yScale)) visible.push_back(&item.second);
            if (frame.shadow[0] > 0) for (int i = 0; i < shadowCascadeCount; ++i)
                if (box.visible(csm.cascades[i])) casters[i].push_back(&item.second);
        }
        auto geometry = [&](const std::vector<const TerrainPage*>& pages) {
            const auto drawBatch = [&](const D3D12_VERTEX_BUFFER_VIEW& view, UINT count) {
                if (!count) return;
                list->IASetVertexBuffers(0, 1, &view);
                list->DrawInstanced(count, 1, 0, 0);
            };
            drawBatch(terrain.view, terrain.count);
            for (const auto* page : pages) drawBatch(page->view, page->staging.count);
            drawBatch(objects.view, objects.count);
        };
        // Four conventional-depth CSM layers; scene depth remains reverse Z.
        if (frame.shadow[0] > 0 || inspectPipeline) {
            transition(*list.Get(), shadows.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
            const D3D12_VIEWPORT shadowViewport{0,0,float(shadowResolution),float(shadowResolution),0,1};
            const D3D12_RECT shadowScissor{0,0,LONG(shadowResolution),LONG(shadowResolution)};
            list->RSSetViewports(1, &shadowViewport); list->RSSetScissorRects(1, &shadowScissor);
            list->SetPipelineState(shadowPipeline.Get());
            for (UINT i = 0; i < shadowCascadeCount; ++i) {
                const auto dsvHandle = depthTarget(i + 1);
                list->OMSetRenderTargets(0, nullptr, FALSE, &dsvHandle);
                list->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1, 0, 0, nullptr);
                list->SetGraphicsRoot32BitConstant(1, i, 0);
                if (frame.shadow[0] > 0) geometry(casters[i]);
            }
            transition(*list.Get(), shadows.Get(), D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        }
        const D3D12_VIEWPORT viewport{0,0,float(width),float(height),0,1};
        const D3D12_RECT scissor{0,0,LONG(width),LONG(height)};
        list->RSSetViewports(1, &viewport); list->RSSetScissorRects(1, &scissor);
        // Geometry/material pass.
        transition(*list.Get(), albedo.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
        transition(*list.Get(), normal.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
        transition(*list.Get(), depth.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
        const D3D12_CPU_DESCRIPTOR_HANDLE targets[]{target(buffers), target(buffers + 1)};
        const auto sceneDsv = depthTarget(0); const float zero[4]{};
        list->ClearRenderTargetView(targets[0], zero, 0, nullptr);
        list->ClearRenderTargetView(targets[1], zero, 0, nullptr);
        list->ClearDepthStencilView(sceneDsv, D3D12_CLEAR_FLAG_DEPTH, 0, 0, 0, nullptr);
        if (camera.wireframe) {
            list->OMSetRenderTargets(0, nullptr, FALSE, &sceneDsv);
            list->SetPipelineState(depthPipeline.Get()); geometry(visible);
        }
        list->OMSetRenderTargets(2, targets, FALSE, &sceneDsv);
        list->SetPipelineState(camera.wireframe ? wireframePipeline.Get() : surfacePipeline.Get()); geometry(visible);
        transition(*list.Get(), albedo.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        transition(*list.Get(), normal.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        transition(*list.Get(), depth.Get(), D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        if (settings.cloudsEnabled) {
            transition(*list.Get(), cloudLight.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
            transition(*list.Get(), cloudDepth.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
            const D3D12_VIEWPORT cloudViewport{0, 0, frame.cloudRender[2], frame.cloudRender[3], 0, 1};
            const D3D12_RECT cloudScissor{0, 0, LONG(frame.cloudRender[2]), LONG(frame.cloudRender[3])};
            list->RSSetViewports(1, &cloudViewport); list->RSSetScissorRects(1, &cloudScissor);
            const D3D12_CPU_DESCRIPTOR_HANDLE cloudTargets[]{target(buffers + 3), target(buffers + 4)};
            list->OMSetRenderTargets(2, cloudTargets, FALSE, nullptr);
            list->SetPipelineState(cloudPipeline.Get()); list->DrawInstanced(3, 1, 0, 0);
            transition(*list.Get(), cloudLight.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            transition(*list.Get(), cloudDepth.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            list->RSSetViewports(1, &viewport); list->RSSetScissorRects(1, &scissor);
        }
        // Linear HDR lighting, sky and aerial perspective share one HDR target.
        transition(*list.Get(), hdr.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
        const auto hdrTarget = target(buffers + 2);
        list->OMSetRenderTargets(1, &hdrTarget, FALSE, nullptr);
        list->SetPipelineState(lightingPipeline.Get()); list->DrawInstanced(3, 1, 0, 0);
        if (clipmapLines.count) {
            list->SetPipelineState(linePipeline.Get());
            list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
            list->IASetVertexBuffers(0, 1, &clipmapLines.view);
            list->DrawInstanced(clipmapLines.count, 1, 0, 0);
            list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        }
        transition(*list.Get(), hdr.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        const UINT index = swap->GetCurrentBackBufferIndex(); const auto backTarget = target(index);
        transition(*list.Get(), back[index].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
        list->OMSetRenderTargets(1, &backTarget, FALSE, nullptr);
        list->SetPipelineState(tonePipeline.Get()); list->DrawInstanced(3, 1, 0, 0);
        Readback screenshot, hdrRead, depthRead, normalRead, cloudRead;
        std::array<Readback, 4> shadowRead;
        if (!capture.empty()) screenshot = copyReadback(back[index].Get(), 0, D3D12_RESOURCE_STATE_RENDER_TARGET);
        if (inspectPipeline) {
            hdrRead = copyReadback(hdr.Get(), 0, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            depthRead = copyReadback(depth.Get(), 0, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            normalRead = copyReadback(normal.Get(), 0, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            if (settings.cloudsEnabled) cloudRead = copyReadback(cloudLight.Get(), 0, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            for (UINT i = 0; i < shadowCascadeCount; ++i)
                shadowRead[i] = copyReadback(shadows.Get(), i, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        }
        transition(*list.Get(), back[index].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
        require(list->Close(), "Close draw command list");
        ID3D12CommandList* commands[]{list.Get()}; queue->ExecuteCommandLists(1, commands); submitted = true;
        const HRESULT presented = swap->Present(1,0); wait(); require(presented, "Present frame");
        if (screenshot.resource) {
            void* bytes = nullptr; require(screenshot.resource->Map(0, nullptr, &bytes), "Read screenshot");
            writeBitmap(capture, static_cast<const std::uint8_t*>(bytes) + screenshot.footprint.Offset,
                        screenshot.footprint.Footprint.RowPitch, width, height);
            const D3D12_RANGE noWrites{0,0}; screenshot.resource->Unmap(0, &noWrites);
        }
        if (inspectPipeline) inspect(hdrRead, depthRead, normalRead, shadowRead, cloudRead, capture);
    }

    HWND window;
    bool warp{}, submitted{}, debugLayer{};
    RenderDiagnostics lastDiagnostics;
    UINT width{}, height{}, rtvSize{}, dsvSize{}, srvSize{}, shadowResolution{};
    UINT64 lastFence{};
    HANDLE completed{};
    ComPtr<ID3D12Device> device;
    ComPtr<ID3D12CommandQueue> queue;
    ComPtr<IDXGISwapChain3> swap;
    ComPtr<ID3D12DescriptorHeap> rtv, dsv, srv;
    ComPtr<ID3D12InfoQueue> infoQueue;
    std::array<ComPtr<ID3D12Resource>, buffers> back;
    ComPtr<ID3D12Resource> depth, albedo, normal, hdr, shadows, frameUpload, cloudLight, cloudDepth, cloudNoise;
    ComPtr<ID3D12RootSignature> root;
    ComPtr<ID3D12PipelineState> lightingPipeline, tonePipeline, surfacePipeline, depthPipeline, shadowPipeline, wireframePipeline, linePipeline, cloudPipeline;
    ComPtr<ID3D12CommandAllocator> allocator;
    ComPtr<ID3D12GraphicsCommandList> list;
    ComPtr<ID3D12Fence> fence;
    Upload terrain, objects, clipmapLines;
    std::map<Key, TerrainPage> terrainPages;
    std::map<Key, TerrainPage> stagedTerrainPages;
    bool terrainCopiesPending{};
};

Dx12Renderer::Dx12Renderer(HWND target, bool forceWarp, bool debug) : impl_(std::make_unique<Impl>(target, forceWarp, debug)) {}
Dx12Renderer::~Dx12Renderer() = default;
void Dx12Renderer::setTerrain(const std::vector<Vertex>& terrain) {
    clearTerrain();
    impl_->upload(impl_->terrain, terrain);
}
void Dx12Renderer::clearTerrain() {
    impl_->terrainPages.clear();
    impl_->stagedTerrainPages.clear();
    impl_->terrainCopiesPending = false;
    impl_->terrain = {};
}
void Dx12Renderer::stageTerrainPage(Key page, Box bounds, const std::vector<GpuVertex>& vertices) {
    auto& entry = impl_->stagedTerrainPages[page];
    impl_->upload(entry.staging, vertices);
    entry.bounds = bounds;
    entry.dirty = !vertices.empty();
    impl_->terrainCopiesPending |= entry.dirty;
}
bool Dx12Renderer::terrainUploadsReady() const { return !impl_->terrainCopiesPending; }
void Dx12Renderer::publishTerrainPages() {
    auto& staged = impl_->stagedTerrainPages;
    while (!staged.empty()) {
        auto page = staged.extract(staged.begin());
        impl_->terrainPages.erase(page.key());
        if (page.mapped().staging.count) impl_->terrainPages.insert(std::move(page));
    }
}
void Dx12Renderer::setClipmapLines(const std::vector<Vertex>& lines) { impl_->upload(impl_->clipmapLines, lines); }
void Dx12Renderer::setShadowResolution(int resolution) { impl_->setShadowResolution(resolution); }
void Dx12Renderer::draw(const Camera& camera, const std::vector<Vertex>& objects, const std::filesystem::path& capture, bool inspectPipeline) {
    impl_->draw(camera, objects, capture, inspectPipeline);
}
RenderDiagnostics Dx12Renderer::diagnostics() const { return impl_->lastDiagnostics; }
bool Dx12Renderer::isWarp() const { return impl_->warp; }

}  // namespace voxel
