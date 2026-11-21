#include "dx12_renderer.hpp"

#include <d3d12.h>
#include <d3dcompiler.h>
#include <dxgi1_4.h>
#include <wrl/client.h>

#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace voxel {
namespace {

using Microsoft::WRL::ComPtr;

constexpr UINT buffers = 2;
constexpr DXGI_FORMAT pixelFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
constexpr DXGI_FORMAT depthFormat = DXGI_FORMAT_D32_FLOAT;

void require(HRESULT result, const char* operation) {
    if (SUCCEEDED(result)) return;
    std::ostringstream error;
    error << operation << " (HRESULT 0x" << std::hex << static_cast<std::uint32_t>(result) << ')';
    throw std::runtime_error(error.str());
}

constexpr char shaderSource[] = R"hlsl(
cbuffer Camera : register(b0) {
    float4 cameraEye, cameraRight, cameraUp, cameraForward;
    float4 projection;
    float4 sunlight;
};

struct Surface { float4 clip : SV_Position; float3 normal : NORMAL; float4 material : COLOR; };

Surface surfaceVS(float3 position : POSITION, float3 normal : NORMAL, float4 material : COLOR) {
    float3 relative = position - cameraEye.xyz;
    float depth = dot(relative, cameraForward.xyz);
    Surface result;
    result.clip = float4(dot(relative, cameraRight.xyz) * projection.x,
                         dot(relative, cameraUp.xyz) * projection.y,
                         (depth - projection.z) * projection.w, depth);
    result.normal = normal;
    result.material = material;
    return result;
}

float4 surfacePS(Surface input) : SV_Target {
    float3 normal = normalize(input.normal);
    float diffuse = max(0, dot(normal, sunlight.xyz));
    int code = int(round(input.material.a * 255));
    float3 color = input.material.rgb;
    if (code <= 3 && sunlight.w > 0.5) {
        if (code == 0) color = float3(0.19, 0.72, 0.83);
        if (code == 1) color = float3(0.28, 0.63, 0.38);
        if (code == 2) color = float3(0.81, 0.63, 0.29);
        if (code == 3) color = float3(0.53, 0.43, 0.64);
    }
    if (code == 255) return float4(color, 1);
    float rim = pow(saturate(1 - abs(dot(cameraForward.xyz, normal))), 3);
    return float4(color * (0.25 + 0.75 * diffuse) + rim * float3(0.030, 0.046, 0.070), 1);
}

struct Sky { float4 clip : SV_Position; float2 position : TEXCOORD; };

Sky skyVS(uint id : SV_VertexID) {
    Sky result;
    result.position = float2(id == 1 ? 3.0 : -1.0, id == 2 ? 3.0 : -1.0);
    result.clip = float4(result.position, 0, 1);
    return result;
}

float4 skyPS(Sky input) : SV_Target {
    float glow = saturate(1 - 0.65 * length(input.position - float2(-0.3, 0.25)));
    float2 tile = floor(input.clip.xy / 6);
    float variation = frac(sin(dot(tile, float2(127.1, 311.7))) * 43758.5453);
    float star = (variation > 0.982 && length(frac(input.clip.xy / 6) - 0.42) < 0.13)
        ? 0.15 + (variation - 0.982) * 12 : 0;
    return float4(float3(0.015, 0.022, 0.040) + glow * float3(0.020, 0.031, 0.055) + star, 1);
}
)hlsl";

ComPtr<ID3DBlob> compile(const char* entry, const char* profile) {
    ComPtr<ID3DBlob> shader, message;
    const HRESULT result = D3DCompile(shaderSource, sizeof(shaderSource) - 1, "VoxelWorld", nullptr, nullptr,
                                      entry, profile, D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3,
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

    Impl(HWND target, bool forceWarp) : window(target) {
        ComPtr<IDXGIFactory4> factory;
        require(CreateDXGIFactory2(0, IID_PPV_ARGS(factory.GetAddressOf())), "Create DXGI factory");
        warp = forceWarp || FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device.GetAddressOf())));
        if (warp) {
            ComPtr<IDXGIAdapter> adapter;
            require(factory->EnumWarpAdapter(IID_PPV_ARGS(adapter.GetAddressOf())), "Find Direct3D 12 WARP adapter");
            require(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(device.GetAddressOf())), "Create Direct3D 12 WARP device");
        }
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
        desc.NumDescriptors = buffers;
        require(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(rtv.GetAddressOf())), "Create render target descriptor heap");
        rtvSize = device->GetDescriptorHandleIncrementSize(desc.Type);
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        desc.NumDescriptors = 1;
        require(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(dsv.GetAddressOf())), "Create depth descriptor heap");
        createTargets();
        createPipelines();
        require(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(allocator.GetAddressOf())), "Create command allocator");
        require(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator.Get(), nullptr, IID_PPV_ARGS(list.GetAddressOf())), "Create command list");
        require(list->Close(), "Close initial command list");
        require(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(fence.GetAddressOf())), "Create frame fence");
        completed = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        if (!completed) throw std::runtime_error("Create frame event failed");
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

    void createTargets() {
        for (UINT i = 0; i < buffers; ++i) {
            require(swap->GetBuffer(i, IID_PPV_ARGS(back[i].GetAddressOf())), "Get swap chain buffer");
            device->CreateRenderTargetView(back[i].Get(), nullptr, target(i));
        }
        auto desc = buffer(width);
        desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        desc.Height = height;
        desc.Format = depthFormat;
        desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
        const auto properties = heap(D3D12_HEAP_TYPE_DEFAULT);
        D3D12_CLEAR_VALUE clear{};
        clear.Format = depthFormat;
        clear.DepthStencil.Depth = 1;
        require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_DEPTH_WRITE,
                                                 &clear, IID_PPV_ARGS(depth.GetAddressOf())), "Create depth buffer");
        device->CreateDepthStencilView(depth.Get(), nullptr, dsv->GetCPUDescriptorHandleForHeapStart());
    }

    void createPipelines() {
        D3D12_ROOT_PARAMETER constants{};
        constants.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        constants.Constants.Num32BitValues = 24;
        constants.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        D3D12_ROOT_SIGNATURE_DESC desc{};
        desc.NumParameters = 1;
        desc.pParameters = &constants;
        desc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
        ComPtr<ID3DBlob> data;
        require(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, data.GetAddressOf(), nullptr), "Serialize root signature");
        require(device->CreateRootSignature(0, data->GetBufferPointer(), data->GetBufferSize(), IID_PPV_ARGS(root.GetAddressOf())), "Create root signature");

        constexpr D3D12_INPUT_ELEMENT_DESC inputs[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"COLOR", 0, pixelFormat, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}
        };
        static_assert(sizeof(Vertex) == 28);
        const auto vs = compile("surfaceVS", "vs_5_0"), ps = compile("surfacePS", "ps_5_0");
        D3D12_GRAPHICS_PIPELINE_STATE_DESC state{};
        state.pRootSignature = root.Get();
        state.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
        state.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
        state.SampleMask = std::numeric_limits<UINT>::max();
        state.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        state.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        state.RasterizerState.DepthClipEnable = TRUE;
        state.BlendState.RenderTarget[0] = {FALSE, FALSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_BLEND_OP_ADD,
                                           D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_BLEND_OP_ADD,
                                           D3D12_LOGIC_OP_NOOP, D3D12_COLOR_WRITE_ENABLE_ALL};
        state.DepthStencilState.DepthEnable = TRUE;
        state.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        state.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
        state.DepthStencilState.FrontFace = {D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS};
        state.DepthStencilState.BackFace = state.DepthStencilState.FrontFace;
        state.InputLayout = {inputs, UINT(std::size(inputs))};
        state.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        state.NumRenderTargets = 1;
        state.RTVFormats[0] = pixelFormat;
        state.DSVFormat = depthFormat;
        state.SampleDesc.Count = 1;
        require(device->CreateGraphicsPipelineState(&state, IID_PPV_ARGS(surfacePipeline.GetAddressOf())), "Create surface pipeline");

        state.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
        require(device->CreateGraphicsPipelineState(&state, IID_PPV_ARGS(wireframePipeline.GetAddressOf())), "Create wireframe pipeline");
        state.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;

        state.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
        state.DepthStencilState.DepthEnable = FALSE;
        state.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
        require(device->CreateGraphicsPipelineState(&state, IID_PPV_ARGS(linePipeline.GetAddressOf())), "Create clipmap line pipeline");

        const auto skyVs = compile("skyVS", "vs_5_0"), skyPs = compile("skyPS", "ps_5_0");
        state.VS = {skyVs->GetBufferPointer(), skyVs->GetBufferSize()};
        state.PS = {skyPs->GetBufferPointer(), skyPs->GetBufferSize()};
        state.InputLayout = {};
        state.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        require(device->CreateGraphicsPipelineState(&state, IID_PPV_ARGS(skyPipeline.GetAddressOf())), "Create sky pipeline");
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
    }

    void upload(Upload& result, const std::vector<Vertex>& vertices) {
        const UINT64 size = UINT64(vertices.size()) * sizeof(Vertex);
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
        std::memcpy(pointer, vertices.data(), std::size_t(size));
        const D3D12_RANGE written{0, SIZE_T(size)};
        result.resource->Unmap(0, &written);
        result.view = {result.resource->GetGPUVirtualAddress(), UINT(size), UINT(sizeof(Vertex))};
    }

    void draw(const Camera& camera, const std::vector<Vertex>& vertices, const std::filesystem::path& capture) {
        RECT rect{};
        GetClientRect(window, &rect);
        if (rect.right <= 0 || rect.bottom <= 0) return;
        if (width != UINT(rect.right) || height != UINT(rect.bottom)) {
            wait();
            for (auto& resource : back) resource.Reset();
            depth.Reset();
            width = UINT(rect.right);
            height = UINT(rect.bottom);
            require(swap->ResizeBuffers(buffers, width, height, pixelFormat, 0), "Resize swap chain");
            createTargets();
        }
        upload(objects, vertices);
        require(allocator->Reset(), "Reset command allocator");
        require(list->Reset(allocator.Get(), skyPipeline.Get()), "Reset command list");
        const UINT frame = swap->GetCurrentBackBufferIndex();
        const auto rtvHandle = target(frame), dsvHandle = dsv->GetCPUDescriptorHandleForHeapStart();
        transition(*list.Get(), back[frame].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
        list->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);
        list->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1, 0, 0, nullptr);
        const D3D12_VIEWPORT viewport{0, 0, float(width), float(height), 0, 1};
        const D3D12_RECT scissor{0, 0, LONG(width), LONG(height)};
        list->RSSetViewports(1, &viewport);
        list->RSSetScissorRects(1, &scissor);
        list->SetGraphicsRootSignature(root.Get());
        list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        const Vec sun = unit(camera.right * -0.68 + camera.up * 0.60 - camera.forward * 0.64);
        constexpr float vertical = 1.92f;
        const float constants[] = {
            float(camera.eye.x), float(camera.eye.y), float(camera.eye.z), 0,
            float(camera.right.x), float(camera.right.y), float(camera.right.z), 0,
            float(camera.up.x), float(camera.up.y), float(camera.up.z), 0,
            float(camera.forward.x), float(camera.forward.y), float(camera.forward.z), 0,
            vertical * float(height) / float(width), vertical, float(camera.nearPlane),
            float(camera.farPlane / (camera.farPlane - camera.nearPlane)),
            float(sun.x), float(sun.y), float(sun.z), camera.lodTint ? 1.0f : 0.0f
        };
        list->SetGraphicsRoot32BitConstants(0, UINT(std::size(constants)), constants, 0);
        list->DrawInstanced(3, 1, 0, 0);
        list->SetPipelineState(camera.wireframe ? wireframePipeline.Get() : surfacePipeline.Get());
        for (const auto* batch : {&terrain, &objects}) {
            if (batch->count == 0) continue;
            list->IASetVertexBuffers(0, 1, &batch->view);
            list->DrawInstanced(batch->count, 1, 0, 0);
        }
        if (clipmapLines.count) {
            list->SetPipelineState(linePipeline.Get());
            list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
            list->IASetVertexBuffers(0, 1, &clipmapLines.view);
            list->DrawInstanced(clipmapLines.count, 1, 0, 0);
        }

        ComPtr<ID3D12Resource> readback;
        D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint{};
        if (!capture.empty()) {
            UINT64 size = 0;
            const auto desc = back[frame]->GetDesc();
            device->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &size);
            const auto properties = heap(D3D12_HEAP_TYPE_READBACK);
            const auto readDesc = buffer(size);
            require(device->CreateCommittedResource(&properties, D3D12_HEAP_FLAG_NONE, &readDesc, D3D12_RESOURCE_STATE_COPY_DEST,
                                                     nullptr, IID_PPV_ARGS(readback.GetAddressOf())), "Allocate screenshot readback");
            D3D12_TEXTURE_COPY_LOCATION source{}, destination{};
            source.pResource = back[frame].Get();
            source.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            destination.pResource = readback.Get();
            destination.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            destination.PlacedFootprint = footprint;
            transition(*list.Get(), back[frame].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
            list->CopyTextureRegion(&destination, 0, 0, 0, &source, nullptr);
            transition(*list.Get(), back[frame].Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_PRESENT);
        } else {
            transition(*list.Get(), back[frame].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
        }
        require(list->Close(), "Close draw command list");
        ID3D12CommandList* commands[]{list.Get()};
        queue->ExecuteCommandLists(1, commands);
        submitted = true;
        const HRESULT result = swap->Present(1, 0);
        wait();
        require(result, "Present frame");
        if (readback) {
            void* bytes = nullptr;
            require(readback->Map(0, nullptr, &bytes), "Read screenshot");
            writeBitmap(capture, static_cast<const std::uint8_t*>(bytes) + footprint.Offset, footprint.Footprint.RowPitch, width, height);
            const D3D12_RANGE noWrites{0, 0};
            readback->Unmap(0, &noWrites);
        }
    }

    HWND window;
    bool warp{}, submitted{};
    UINT width{}, height{}, rtvSize{};
    UINT64 lastFence{};
    HANDLE completed{};
    ComPtr<ID3D12Device> device;
    ComPtr<ID3D12CommandQueue> queue;
    ComPtr<IDXGISwapChain3> swap;
    ComPtr<ID3D12DescriptorHeap> rtv, dsv;
    std::array<ComPtr<ID3D12Resource>, buffers> back;
    ComPtr<ID3D12Resource> depth;
    ComPtr<ID3D12RootSignature> root;
    ComPtr<ID3D12PipelineState> skyPipeline, surfacePipeline, wireframePipeline, linePipeline;
    ComPtr<ID3D12CommandAllocator> allocator;
    ComPtr<ID3D12GraphicsCommandList> list;
    ComPtr<ID3D12Fence> fence;
    Upload terrain, objects, clipmapLines;
};

Dx12Renderer::Dx12Renderer(HWND target, bool forceWarp) : impl_(std::make_unique<Impl>(target, forceWarp)) {}
Dx12Renderer::~Dx12Renderer() = default;
void Dx12Renderer::setTerrain(const std::vector<Vertex>& terrain) { impl_->upload(impl_->terrain, terrain); }
void Dx12Renderer::setClipmapLines(const std::vector<Vertex>& lines) { impl_->upload(impl_->clipmapLines, lines); }
void Dx12Renderer::draw(const Camera& camera, const std::vector<Vertex>& objects, const std::filesystem::path& capture) {
    impl_->draw(camera, objects, capture);
}
bool Dx12Renderer::isWarp() const { return impl_->warp; }

}  // namespace voxel
