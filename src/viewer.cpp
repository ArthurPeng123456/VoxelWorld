#include "dx12_renderer.hpp"
#include "lod_range_dialog.hpp"
#include "render_settings_dialog.hpp"
#include "world.hpp"
#include "free_camera.hpp"
#include "sweep.hpp"

#include <shellapi.h>
#include <psapi.h>
#include <windowsx.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <fstream>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace voxel {
namespace {

constexpr wchar_t windowClass[] = L"VoxelWorldWindow";
constexpr wchar_t viewportClass[] = L"VoxelWorldViewport";
constexpr int footerHeight = 128;
constexpr int cameraPanelWidth = 340;
constexpr int cameraPanelHeight = 90;
constexpr double pi = 3.14159265358979323846;
constexpr double dt = 1.0 / 60.0;
constexpr int maxInvokers = 16;
constexpr double invokerSpeed = 240.0;
constexpr double invokerReturnSpeed = 720.0;
constexpr std::array<Method, 4> methods{Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
constexpr std::array<const wchar_t*, 4> methodLabels{L"Mesh BVH", L"VoxelField", L"HashDAG", L"OpenVDB"};
constexpr std::array terrainLabels{L"Classic", L"Mars Craters", L"Volcanic Highlands"};
static_assert(terrainLabels.size() == terrainPresets.size());

Key terrainPage(Key chunk) {
    // Group adjacent chunks of the same scale to limit both uploads and draw calls.
    const int level = chunk.level + (chunk.level == 0 && chunk.densityShift <= -2 ? 2 : 3);
    const int span = 1 << level;
    const auto align = [span](int coordinate) { return coordinate - (coordinate % span + span) % span; };
    return {align(chunk.x), align(chunk.y), align(chunk.z), level, chunk.densityShift};
}

enum Command { Mesh = 1, Grid, Dag, Vdb, View, Tint, Pause, Drop, Reset, Ranges, Wireframe, Atmosphere, FreeView, Shadows, Clouds,
               Sweep, RangeSettings, GraphicsSettings, Density = 100, InvokerCount, CameraTarget, TerrainStyle };

struct ToolbarButton { const wchar_t* label; int width; };
constexpr std::array<ToolbarButton, GraphicsSettings> toolbarButtons{{
    {L"1  Mesh BVH", 100}, {L"2  VoxelField", 100}, {L"3  HashDAG", 100}, {L"4  OpenVDB", 100},
    {L"Surface [F]", 104}, {L"LOD [L]", 72}, {L"Pause", 68}, {L"Drop [B]", 76}, {L"Reset", 56},
    {L"Ranges [K]", 84}, {L"Wireframe [F2]", 116}, {L"Atmosphere [H]", 128}, {L"Free [C]", 92}, {L"Shadows [J]", 108},
    {L"Clouds [V]", 100}, {L"Sweep [X]", 96},
    {L"LOD ranges...", 112}, {L"Render settings...", 136}
}};

struct Options {
    int method{1};
    std::uint64_t seed{42};
    int invokers{2};
    bool warp{}, morton{}, cloudCaptures{}, temporalCaptures{}, stabilityCaptures{}, lodRangeCaptures{};
    bool cloudFlight{}, cloudFlightCaptures{};
    double voxelDensity{1};
    TerrainPreset terrain{TerrainPreset::Classic};
    RenderSettings render;
    SweepOptions sweep;
    std::filesystem::path captures;
    std::filesystem::path streamProfile;
};

constexpr std::array terrainRenderFields{&RenderSettings::atmosphereHeightKm,
    &RenderSettings::cloudThicknessKm, &RenderSettings::cloudCoverage};

constexpr RenderSettings defaultRenderSettings(TerrainPreset preset) {
    RenderSettings settings;
    if (preset == TerrainPreset::Mars) {
        settings.atmosphereHeightKm = 300;
        settings.cloudThicknessKm = 24;
        settings.cloudCoverage = 0.35;
    }
    return settings;
}

Options readOptions() {
    int count = 0;
    const std::unique_ptr<wchar_t*, decltype(&LocalFree)> args(CommandLineToArgvW(GetCommandLineW(), &count), LocalFree);
    if (!args) throw std::runtime_error("Could not read the command line");
    Options result;
    std::array<bool, terrainRenderFields.size()> renderOverrides{};
    for (int i = 1; i < count; ++i) {
        const std::wstring option = args.get()[i];
        auto next = [&] {
            if (++i >= count) throw std::invalid_argument("Missing option value");
            return std::wstring(args.get()[i]);
        };
        auto integer = [&] {
            const std::wstring text = next();
            if (text.empty() || text.find_first_not_of(L"0123456789") != std::wstring::npos) {
                throw std::invalid_argument("Expected a nonnegative integer");
            }
            return std::stoull(text);
        };
        auto bounded = [&](std::uint64_t min, std::uint64_t max) {
            const auto value = integer();
            if (value < min || value > max) throw std::invalid_argument("Value out of range");
            return int(value);
        };
        auto real = [&](double min, double max) {
            const std::wstring text = next();
            std::size_t consumed = 0;
            const double value = std::stod(text, &consumed);
            if (consumed != text.size() || !std::isfinite(value) || value < min || value > max)
                throw std::invalid_argument("Value out of range");
            return value;
        };
        const auto renderOption = [&](const auto& parameters) {
            for (const auto& parameter : parameters) if (option == parameter.option) {
                const auto value = readRenderParameter(next(), parameter);
                if (!value) {
                    std::string flag(option.size(), ' ');
                    std::transform(option.begin(), option.end(), flag.begin(), [](wchar_t c) { return static_cast<char>(c); });
                    throw std::invalid_argument(flag + " requires a finite value within the supported range");
                }
                result.render.*parameter.value = *value;
                for (std::size_t field = 0; field < terrainRenderFields.size(); ++field)
                    if (parameter.value == terrainRenderFields[field]) renderOverrides[field] = true;
                return true;
            }
            return false;
        };
        if (option == L"--warp") result.warp = true;
        else if (option == L"--no-shadows") result.render.shadowsEnabled = false;
        else if (option == L"--no-atmosphere") result.render.atmosphereEnabled = false;
        else if (option == L"--clouds") result.render.cloudsEnabled = true;
        else if (option == L"--no-clouds") result.render.cloudsEnabled = false;
        else if (option == L"--taa") result.render.taaEnabled = true;
        else if (option == L"--no-taa") result.render.taaEnabled = false;
        else if (option == L"--cloud-temporal") result.render.cloudTemporalEnabled = true;
        else if (option == L"--no-cloud-temporal") result.render.cloudTemporalEnabled = false;
        else if (option == L"--taa-samples") {
            const auto samples = integer();
            if (std::find(taaSampleCounts.begin(), taaSampleCounts.end(), samples) == taaSampleCounts.end())
                throw std::invalid_argument("--taa-samples requires 8, 16 or 32");
            result.render.taaSamples = int(samples);
        } else if (option == L"--cloud-sampling") {
            const auto value = next();
            if (value == L"full") result.render.cloudSampling = 0;
            else if (value == L"checkerboard") result.render.cloudSampling = 1;
            else if (value == L"quarter") result.render.cloudSampling = 2;
            else throw std::invalid_argument("--cloud-sampling requires full, checkerboard or quarter");
        }
        else if (option == L"--cloud-steps") {
            const auto steps = integer();
            if (std::find(cloudStepCounts.begin(), cloudStepCounts.end(), steps) == cloudStepCounts.end())
                throw std::invalid_argument("--cloud-steps requires 32, 64 or 96");
            result.render.cloudSteps = int(steps);
        }
        else if (option == L"--shadow-resolution") {
            const auto resolution = integer();
            if (resolution > shadowResolutions.back() || !validShadowResolution(int(resolution)))
                throw std::invalid_argument("--shadow-resolution requires 1024, 2048 or 4096");
            result.render.shadowResolution = int(resolution);
        } else if (option == L"--tone-map") {
            const auto value = next();
            if (value == L"aces") result.render.toneMapping = ToneMapping::Aces;
            else if (value == L"reinhard") result.render.toneMapping = ToneMapping::Reinhard;
            else if (value == L"linear") result.render.toneMapping = ToneMapping::Linear;
            else throw std::invalid_argument("--tone-map requires aces, reinhard or linear");
        } else if (option == L"--morton") result.morton = true;
        else if (option == L"--capture-dir") result.captures = next();
        else if (option == L"--capture-clouds") { result.captures = next(); result.cloudCaptures = true; }
        else if (option == L"--capture-temporal") { result.captures = next(); result.temporalCaptures = true; }
        else if (option == L"--capture-stability") { result.captures = next(); result.stabilityCaptures = true; }
        else if (option == L"--replay-cloud-flight") result.cloudFlight = true;
        else if (option == L"--capture-cloud-flight") { result.captures = next(); result.cloudFlightCaptures = true; }
        else if (option == L"--capture-lod-ranges") { result.captures = next(); result.lodRangeCaptures = true; }
        else if (option == L"--profile-stream") result.streamProfile = next();
        else if (option == L"--voxel-density" || option == L"--density") result.voxelDensity = parseVoxelDensity(next());
        else if (option == L"--seed") result.seed = integer();
        else if (option == L"--terrain") {
            const auto value = next();
            std::string ascii(value.size(), ' ');
            std::transform(value.begin(), value.end(), ascii.begin(), [](wchar_t c) {
                return c >= L'a' && c <= L'z' ? static_cast<char>(c) : ' ';
            });
            result.terrain = parseTerrainPreset(ascii);
        }
        else if (option == L"--invokers") {
            const auto n = integer();
            if (n < 1 || n > maxInvokers) throw std::invalid_argument("Invoker count must be from 1 to 16");
            result.invokers = int(n);
        } else if (option == L"--sweep") result.sweep.enabled = true;
        else if (option == L"--sweep-invokers") {
            const auto n = integer();
            if (n < 1 || n > 1000) throw std::invalid_argument("Sweep invoker count must be from 1 to 1000");
            result.sweep.invokers = int(n);
        } else if (option == L"--shapes-per-invoker") result.sweep.shapesPerInvoker = bounded(1, 84);
        else if (option == L"--sweep-frames") result.sweep.frames = bounded(1, 400);
        else if (option == L"--sweep-warmup") result.sweep.warmup = bounded(0, 20);
        else if (option == L"--sweep-budget") result.sweep.budget = bounded(1, 10000000);
        else if (option == L"--sweep-tolerance") result.sweep.tolerance = real(0.01, 10.0);
        else if (option == L"--sweep-frame") {
            const std::wstring text = next();
            if (text == L"-1") result.sweep.frame = -1;
            else {
                if (text.empty() || text.find_first_not_of(L"0123456789") != std::wstring::npos)
                    throw std::invalid_argument("Expected -1 or a nonnegative frame");
                result.sweep.frame = int(std::stoull(text));
                if (result.sweep.frame > 100000) throw std::invalid_argument("Sweep frame is out of range");
            }
        } else if (option == L"--scheme") {
            const auto value = next();
            if (value == L"mesh" || value == L"1") result.method = 0;
            else if (value == L"grid" || value == L"2") result.method = 1;
            else if (value == L"dag" || value == L"3") result.method = 2;
            else if (value == L"vdb" || value == L"4") result.method = 3;
            else throw std::invalid_argument("Choose mesh, grid, dag or vdb");
        } else if (!renderOption(shadowParameters) && !renderOption(atmosphereParameters) &&
                   !renderOption(cloudParameters) && !renderOption(taaParameters) &&
                   !renderOption(cloudTemporalParameters) && !renderOption(hdrParameters))
            throw std::invalid_argument("Unknown viewer option");
    }
    // Resolve terrain defaults after parsing so explicit flags win regardless
    // of whether they appear before or after --terrain.
    const auto defaults = defaultRenderSettings(result.terrain);
    for (std::size_t field = 0; field < terrainRenderFields.size(); ++field)
        if (!renderOverrides[field]) result.render.*terrainRenderFields[field] = defaults.*terrainRenderFields[field];
    return result;
}

std::uint32_t color(Vec rgb, int code) {
    auto byte = [](double value) { return std::uint32_t(std::lround(std::clamp(value, 0.0, 1.0) * 255)); };
    return byte(rgb.x) | (byte(rgb.y) << 8) | (byte(rgb.z) << 16) | (std::uint32_t(code) << 24);
}

Vertex vertex(Vec p, Vec normal, std::uint32_t material) {
    return {p, normal, material};
}

void flatTriangle(std::vector<Vertex>& batch, Vec a, Vec b, Vec c, Vec outward, std::uint32_t material) {
    Vec n = unit(cross(b - a, c - a), outward);
    if (dot(n, outward) < 0) {
        n = n * -1;
        std::swap(b, c);
    }
    for (const Vec p : {a, b, c}) batch.push_back(vertex(p, n, material));
}

void appendBox(std::vector<Vertex>& batch, const Shape& shape, Vec p, std::uint32_t material) {
    const auto transform = [&](Vec v) { return p + shape.direction(v); };
    for (int axis = 0; axis < 3; ++axis) for (int side : {-1, 1}) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        Vec a = shape.size * -1, b = a, c = a, d = a;
        a[axis] = b[axis] = c[axis] = d[axis] = side * shape.size[axis];
        b[u] = c[u] = shape.size[u];
        c[v] = d[v] = shape.size[v];
        const Vec normal = shape.axes[axis] * side;
        flatTriangle(batch, transform(a), transform(b), transform(c), normal, material);
        flatTriangle(batch, transform(a), transform(c), transform(d), normal, material);
    }
}

void appendRound(std::vector<Vertex>& batch, const Shape& shape, Vec p, std::uint32_t material) {
    constexpr int rings = 5;
    constexpr int slices = 12;
    const double segment = shape.kind == ShapeKind::Capsule ? shape.size.z : 0;
    // Two hemispheres with an optional cylindrical band between their equators.
    const int rows = 2 * (rings + 1);
    auto at = [&](int row, int column) {
        const bool bottom = row <= rings;
        const double latitude = (bottom ? row - rings : row - rings - 1) * (pi * 0.5 / rings);
        const double angle = column * (2 * pi / slices);
        const Vec n{std::cos(latitude) * std::cos(angle), std::cos(latitude) * std::sin(angle), std::sin(latitude)};
        const Vec offset = n * shape.size.x + Vec{0, 0, bottom ? -segment : segment};
        return vertex(p + shape.direction(offset), shape.direction(n), material);
    };
    for (int row = 0; row + 1 < rows; ++row) {
        if (row == rings && segment == 0) continue;
        for (int i = 0; i < slices; ++i) {
            const auto a = at(row, i), b = at(row, i + 1), c = at(row + 1, i + 1), d = at(row + 1, i);
            if (row > 0) batch.insert(batch.end(), {a, b, c});
            if (row < rows - 2) batch.insert(batch.end(), {a, c, d});
        }
    }
}

void appendBeacon(std::vector<Vertex>& batch, Vec surface, double size, bool selected) {
    const Vec up = unit(surface);
    const auto axes = frameAt(up);
    const auto shell = color(selected ? Vec{0.07, 0.56, 0.77} : Vec{0.8, 0.32, 0.10}, 253);
    const auto belt = color(selected ? Vec{0.39, 0.88, 1.0} : Vec{1.0, 0.72, 0.34}, 253);
    constexpr int sides = 8;
    std::array<Vec, sides> directions;
    for (int i = 0; i < sides; ++i) {
        const double angle = 2 * pi * i / sides;
        directions[i] = axes[0] * std::cos(angle) + axes[1] * std::sin(angle);
    }
    const Vec bottom = surface + up * (size * 0.89), top = surface + up * (size * 1.11);
    const Vec tip = surface + up * (size * 2);
    for (int i = 0; i < sides; ++i) {
        const Vec x = directions[i] * (size * 0.55), y = directions[(i + 1) % sides] * (size * 0.55);
        const Vec normal = unit(x + y);
        flatTriangle(batch, surface, bottom + x, bottom + y, normal, shell);
        flatTriangle(batch, bottom + x, top + x, top + y, normal, belt);
        flatTriangle(batch, bottom + x, top + y, bottom + y, normal, belt);
        flatTriangle(batch, tip, top + x, top + y, normal, shell);
    }
}

Vec moveOnPlanet(const Planet& planet, Vec position, Vec tangent, double meters) {
    const double angle = meters / length(position);
    return planet.surface(unit(unit(position) * std::cos(angle) + tangent * std::sin(angle)));
}

class App {
public:
    explicit App(Options options) : opts_(std::move(options)), planet_(opts_.seed, opts_.terrain), method_(opts_.method) {}
    ~App() {
        world_.reset();
        renderer_.reset();
        if (IsWindow(window_)) DestroyWindow(window_);
        if (font_) DeleteObject(font_);
        if (background_) DeleteObject(background_);
    }

    void create(HINSTANCE instance, int show) {
        background_ = CreateSolidBrush(RGB(17, 23, 33));
        font_ = CreateFontW(-16, 0, 0, 0, FW_MEDIUM, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                            CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
        WNDCLASSEXW parent{};
        parent.cbSize = sizeof(parent);
        parent.hInstance = instance;
        parent.lpfnWndProc = windowProcedure;
        parent.lpszClassName = windowClass;
        parent.hCursor = LoadCursorW(nullptr, IDC_ARROW);
        parent.hIcon = LoadIconW(nullptr, IDI_APPLICATION);
        parent.hbrBackground = background_;
        if (!RegisterClassExW(&parent)) throw std::runtime_error("Register the viewer window failed");
        WNDCLASSEXW viewport = parent;
        viewport.lpfnWndProc = viewportProcedure;
        viewport.lpszClassName = viewportClass;
        viewport.hbrBackground = nullptr;
        viewport.hCursor = LoadCursorW(nullptr, IDC_SIZEALL);
        if (!RegisterClassExW(&viewport)) throw std::runtime_error("Register the Direct3D viewport failed");

        RECT screen{};
        SystemParametersInfoW(SPI_GETWORKAREA, 0, &screen, 0);
        const int width = std::min(1340L, screen.right - screen.left);
        const int height = std::min(850L, screen.bottom - screen.top);
        window_ = CreateWindowExW(0, windowClass, L"VoxelWorld  |  Planet collision viewer", WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
                                   screen.left + (screen.right - screen.left - width) / 2,
                                   screen.top + (screen.bottom - screen.top - height) / 2,
                                   width, height, nullptr, nullptr, instance, this);
        if (!window_) throw std::runtime_error("Create the viewer window failed");
        viewport_ = CreateWindowExW(0, viewportClass, nullptr, WS_CHILD | WS_VISIBLE, 0, 0, 1, 1, window_, nullptr, instance, this);
        if (!viewport_) throw std::runtime_error("Create the Direct3D viewport failed");
        for (int i = 0; i < int(toolbarButtons.size()); ++i) {
            buttons_[i] = CreateWindowExW(0, L"BUTTON", toolbarButtons[i].label, WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_OWNERDRAW,
                                           0, 0, 1, 1, window_, reinterpret_cast<HMENU>(INT_PTR(i + 1)), instance, nullptr);
            if (!buttons_[i]) throw std::runtime_error("Create viewer toolbar failed");
            SendMessageW(buttons_[i], WM_SETFONT, reinterpret_cast<WPARAM>(font_), FALSE);
        }
        createCombo(instance, Density, L"Voxel step", 80, 156);
        createCombo(instance, InvokerCount, L"Invokers", 67, 67);
        createCombo(instance, CameraTarget, L"Camera", 66, 125);
        createCombo(instance, TerrainStyle, L"Terrain", 65, 177);
        for (const auto* label : terrainLabels) addComboItem(TerrainStyle, label);
        SendMessageW(combo(TerrainStyle), CB_SETCURSEL, WPARAM(opts_.terrain), 0);
        for (int i = 0; i < int(voxelDensities.size()); ++i) {
            std::wostringstream label;
            label << voxelDensities[i] << L"x (" << chunkSize / fineCells * voxelDensities[i] << L" m)";
            addComboItem(Density, label.str());
            if (voxelDensities[i] == opts_.voxelDensity) SendMessageW(combo(Density), CB_SETCURSEL, WPARAM(i), 0);
        }
        for (int count = 1; count <= maxInvokers; ++count) addComboItem(InvokerCount, std::to_wstring(count));
        SendMessageW(combo(InvokerCount), CB_SETCURSEL, WPARAM(opts_.invokers - 1), 0);
        arrange();
        ShowWindow(window_, show);
        UpdateWindow(window_);
        renderer_ = std::make_unique<Dx12Renderer>(viewport_, opts_.warp, !opts_.captures.empty());
        renderer_->setShadowResolution(opts_.render.shadowResolution);
        rebuild();
    }

    int run() {
        if (opts_.cloudFlightCaptures) return captureCloudFlight();
        if (opts_.cloudFlight) return replayCloudFlight();
        if (opts_.lodRangeCaptures) return captureLodRanges();
        if (opts_.stabilityCaptures) return captureStability();
        if (opts_.temporalCaptures) return captureTemporal();
        if (opts_.cloudCaptures) return captureClouds();
        if (!opts_.captures.empty()) return captureAll();
        std::ofstream profile;
        int profileFrame = 0;
        double profileSeconds = 0;
        if (!opts_.streamProfile.empty()) {
            profile.open(opts_.streamProfile);
            profile.exceptions(std::ios::badbit | std::ios::failbit);
            profile << "frame,frame_ms,physics_ms,publication_ms,terrain_ms,status_ms,draw_ms,built_chunks,"
                       "lod0_covered,lod_update_ms,plan_ms,sample_cpu_ms,lod0_headroom_m,rolled_invoker,rolled_lod\n";
            selectCameraTarget(1);
        }
        std::vector<ClipmapWindow> profiledWindows;
        MSG message{};
        auto previous = Clock::now();
        double accumulator = 0, statsElapsed = 0, streamElapsed = 0;
        while (!closed_) {
            const auto frameStart = Clock::now();
            while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
                if (message.message == WM_QUIT) return int(message.wParam);
                if (message.message == WM_KEYDOWN && !comboOwnsKeyboard(message.hwnd) &&
                    key(message.wParam, (message.lParam & (1L << 30)) == 0)) continue;
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }
            if (closed_) break;
            if (pending_) {
                const int value = std::exchange(pending_, 0);
                action(value);
            }
            if (IsIconic(window_)) {
                WaitMessage();
                previous = Clock::now();
                continue;
            }
            const auto now = Clock::now();
            const double elapsed = std::clamp(std::chrono::duration<double>(now - previous).count(), 0.0, 0.1);
            previous = now;
            const auto physicsStart = Clock::now();
            if (profile.is_open()) moveInvoker(1, 0.25, elapsed, true);
            else moveControlled(elapsed);
            if (!paused_) {
                accumulator += elapsed;
                while (accumulator >= dt) {
                    tick();
                    accumulator -= dt;
                }
            }
            // The camera keeps moving while simulation is paused.
            const double physicsMs = milliseconds(physicsStart);
            const auto publishStart = Clock::now();
            double uploadMs = 0;
            const auto published = world_->pollUpdate([&] {
                const auto start = Clock::now();
                const bool ready = stagePreparedTerrain(false);
                uploadMs = milliseconds(start);
                return ready;
            });
            const double publicationMs = milliseconds(publishStart) - uploadMs;
            const auto terrainStart = Clock::now();
            if (published) applyTerrain(*published, false);
            const Camera frameCamera = camera();
            trackClipmapMotion(frameCamera.eye, elapsed);
            streamElapsed += elapsed;
            if ((streamElapsed >= dt || profile.is_open()) &&
                world_->requestUpdate(invokers_, true, frameCamera.eye, true, clipmapForecast_)) {
                streamElapsed = 0;
            }
            const double terrainMs = milliseconds(terrainStart) + uploadMs;
            const auto statusStart = Clock::now();
            fps_ = fps_ * 0.94 + 0.06 / std::max(0.001, elapsed);
            statsElapsed += elapsed;
            if (statsElapsed >= 0.25) {
                updateFooter();
                statsElapsed = 0;
            }
            const double statusMs = milliseconds(statusStart);
            updateCameraInfo(frameCamera);
            buildObjects();
            const auto drawStart = Clock::now();
            renderer_->draw(frameCamera, objects_);
            if (profile.is_open()) {
                const auto& windows = world_->clipmapWindows();
                const auto point = invokers_[selected_];
                int rolledInvoker = -1, rolledLod = -1;
                double headroom = 0;
                for (std::size_t i = 0; i < windows.size(); ++i) {
                    const auto& w = windows[i];
                    if (w.invoker == selected_ && w.lod == 0) {
                        const auto gap = min(point - w.box.lo, w.box.hi - point);
                        headroom = std::min({gap.x, gap.y, gap.z});
                    }
                    if (published && windows.size() == profiledWindows.size() &&
                        length(w.box.center() - profiledWindows[i].box.center()) > 0) {
                        rolledInvoker = int(w.invoker);
                        rolledLod = w.lod;
                    }
                }
                if (published || profiledWindows.empty()) profiledWindows = windows;
                profile << profileFrame++ << ',' << milliseconds(frameStart) << ',' << physicsMs << ',' << publicationMs << ','
                        << terrainMs << ',' << statusMs << ',' << milliseconds(drawStart) << ','
                        << (published ? published->build.built : 0) << ',' << world_->hasFinestLod(invokers_[selected_]) << ','
                        << (published ? published->updateMs : 0) << ',' << (published ? published->selectMs : 0) << ','
                        << (published ? published->build.sampleMs : 0) << ',' << headroom << ',' << rolledInvoker << ',' << rolledLod << '\n';
                profileSeconds += elapsed;
                if (profileSeconds >= 8) return 0;
            }
        }
        return 0;
    }

private:
    struct VisibleBody { std::size_t index; Shape shape; std::uint32_t material; };
    struct TerrainPage { std::set<Key> chunks; std::size_t triangles{}; };
    struct PreparedTerrainPage {
        Key key;
        TerrainPage page;
        Box bounds;
        std::vector<GpuVertex> vertices;
    };
    struct ToolbarCombo { HWND label{}, field{}; int labelWidth{}, fieldWidth{}; };

    HWND combo(int command) const { return combos_[command - Density].field; }

    void createCombo(HINSTANCE instance, int command, const wchar_t* label, int labelWidth, int fieldWidth) {
        auto& control = combos_[command - Density];
        control.labelWidth = labelWidth;
        control.fieldWidth = fieldWidth;
        control.label = CreateWindowExW(0, L"STATIC", label, WS_VISIBLE | WS_CHILD,
                                       0, 0, 1, 1, window_, nullptr, instance, nullptr);
        control.field = CreateWindowExW(0, L"COMBOBOX", nullptr,
            WS_VISIBLE | WS_CHILD | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
            0, 0, 1, 220, window_, reinterpret_cast<HMENU>(INT_PTR(command)), instance, nullptr);
        if (!control.label || !control.field) throw std::runtime_error("Create viewer dropdown failed");
        SendMessageW(control.label, WM_SETFONT, reinterpret_cast<WPARAM>(font_), FALSE);
        SendMessageW(control.field, WM_SETFONT, reinterpret_cast<WPARAM>(font_), FALSE);
    }

    void addComboItem(int command, const std::wstring& text) {
        SendMessageW(combo(command), CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(text.c_str()));
    }

    bool comboOwnsKeyboard(HWND window) const {
        for (const auto& control : combos_)
            if (window == control.field || SendMessageW(control.field, CB_GETDROPPEDSTATE, 0, 0)) return true;
        return false;
    }

    void refreshCameraTargets() {
        SendMessageW(combo(CameraTarget), CB_RESETCONTENT, 0, 0);
        addComboItem(CameraTarget, L"Detached");
        for (int i = 1; i <= opts_.invokers; ++i) addComboItem(CameraTarget, L"Invoker " + std::to_wstring(i));
        syncCameraTarget();
    }

    void syncCameraTarget() {
        SendMessageW(combo(CameraTarget), CB_SETCURSEL, controlling_ ? WPARAM(selected_ + 1) : 0, 0);
    }

    void detachInvoker() {
        if (controlling_) returning_[selected_] = true;
        controlling_ = false;
    }

    void selectCameraTarget(std::size_t item) {
        if (item == 0) {
            if (controlling_) {
                detachInvoker();
                surface_ = false;
            }
        } else if (item <= invokers_.size()) {
            detachInvoker();
            selected_ = item - 1;
            returning_[selected_] = false;
            controlling_ = true;
            freeView_ = false;
            surface_ = true;
        }
        syncCameraTarget();
    }

    void setInvokerCount(int count) {
        const auto previous = invokers_.size();
        if (count == int(previous)) return;
        if (selected_ >= std::size_t(count)) {
            if (controlling_) selectCameraTarget(0);
            selected_ = std::size_t(count - 1);
        }
        opts_.invokers = count;
        invokers_.resize(std::size_t(count));
        for (std::size_t i = previous; i < invokers_.size(); ++i) {
            invokers_[i] = planet_.surface(planet_.invokerDirection(int(i), simulationTime_ * 2.0));
            returning_[i] = false;
        }
        refreshCameraTargets();
    }

    void rebuild() {
        world_.reset();
        sweepYaw_ = 0;
        sweepPitch_ = 0.2;
        sweepDistance_ = 0;
        clipmapPositions_.clear();
        clipmapForecast_ = {};
        preparedTerrain_.clear();
        terrainPages_.clear();
        nextTerrainUpload_ = 0;
        simulationTime_ = 0;
        shapes_.clear();
        controlling_ = false;
        returning_.fill(false);
        const int sweepCount = sweepMode_ ? opts_.sweep.invokers : opts_.invokers;
        invokers_ = invokerPositions(planet_, sweepCount, 0);
        refreshCameraTargets();
        world_ = std::make_unique<World>(planet_, methods[method_], opts_.morton, opts_.voxelDensity,
            [this](const Chunks& live, const Chunks& staged, const std::vector<Key>& removed) {
                prepareTerrainPages(live, staged, removed);
            });
        world_->setLod0Radius(lod0Radius_);
        sweep_.attachBackend(*world_, methods[method_], opts_.morton, opts_.voxelDensity);
        sweep_.clear();
        updateTerrain(true);
        if (sweepMode_) {
            sweep_.begin(invokers_, invokers_.front());
        } else {
            for (int i = 0; i < opts_.invokers; ++i) spawn(i);
        }
        updateFooter();
        repaintButtons();
    }

    void spawn(int index) {
        if (shapes_.size() > 347) return;
        const Vec point = invokers_.at(std::size_t(index));
        const auto basis = frameAt(unit(point));
        constexpr Vec palette[] = {{0.98, 0.61, 0.27}, {0.33, 0.84, 0.95}, {0.77, 0.68, 0.99}};
        for (int i = 0; i < 9; ++i) {
            const double angle = i * 2.3999632;
            const double distance = 52 * std::sqrt(double(i));
            const Vec up = unit(point + (basis[0] * std::cos(angle) + basis[1] * std::sin(angle)) * distance);
            Shape shape = sampleShape(i % 3 + 1, up);
            const auto id = world_->physics.addBody(shape, planet_.surface(up) + up * (72 + 8 * i), up * -18);
            shapes_.push_back({id, shape, color(palette[i % 3], 254)});
        }
    }

    void tick() {
        if (sweepMode_) return;
        world_->physics.step(dt, true);
        simulationTime_ += dt;
        for (std::size_t i = 0; i < invokers_.size(); ++i) {
            if (controlling_ && i == selected_) continue;
            const Vec route = planet_.surface(planet_.invokerDirection(int(i), simulationTime_ * 2.0));
            if (returning_[i]) {
                const Vec up = unit(invokers_[i]), goal = unit(route);
                const double cosine = dot(up, goal);
                const Vec tangent = goal - up * cosine;
                const double angle = std::atan2(length(tangent), cosine);
                if (angle * length(invokers_[i]) > invokerReturnSpeed * dt) {
                    invokers_[i] = moveOnPlanet(planet_, invokers_[i], unit(tangent, frameAt(up)[0]), invokerReturnSpeed * dt);
                    continue;
                }
                returning_[i] = false;
            }
            invokers_[i] = route;
        }
    }

    void moveControlled(double elapsed) {
        // Poll current state so held keys move smoothly and focus changes cannot
        // leave a movement key stuck.
        if ((!freeView_ && !controlling_) || GetForegroundWindow() != window_ || GetFocus() != viewport_) return;
        auto down = [](int keyCode) { return (GetAsyncKeyState(keyCode) & 0x8000) != 0; };
        const double ahead = double(down('W')) - double(down('S'));
        const double sideways = double(down('D')) - double(down('A'));
        if (controlling_) {
            moveInvoker(ahead, sideways, elapsed, down(VK_SHIFT));
        } else {
            const double vertical = double(down('E')) - double(down('Q'));
            flight_.move(ahead, sideways, vertical, elapsed, flightSpeed_ * (down(VK_SHIFT) ? 5 : 1));
        }
    }

    void moveInvoker(double ahead, double sideways, double elapsed, bool faster) {
        if ((!ahead && !sideways) || elapsed <= 0) return;
        Vec& position = invokers_[selected_];
        const Vec up = unit(position), view = camera().forward;
        const Vec forward = unit(view - up * dot(view, up), frameAt(up)[0]);
        const Vec right = cross(forward, up);
        const Vec direction = unit(forward * ahead + right * sideways);
        position = moveOnPlanet(planet_, position, direction, invokerSpeed * elapsed * (faster ? 3 : 1));
    }

    void trackClipmapMotion(Vec cameraEye, double elapsed) {
        const std::size_t count = invokers_.size() + 1;
        clipmapForecast_.primary = controlling_ ? selected_ : count - 1;
        const auto positionAt = [&](std::size_t i) { return i < invokers_.size() ? invokers_[i] : cameraEye; };
        if (clipmapPositions_.size() != count || elapsed <= 0) {
            clipmapPositions_.resize(count);
            for (std::size_t i = 0; i < count; ++i) clipmapPositions_[i] = positionAt(i);
            clipmapForecast_.displacement.assign(count, {});
            return;
        }
        const double blend = std::min(1.0, elapsed / 0.10);
        const double lookAhead = std::clamp(0.70 + streamMs_ * 0.0006, 0.70, 1.00);
        const double jump = 2.0 * world_->lod0Radius();
        for (std::size_t i = 0; i < count; ++i) {
            const Vec current = positionAt(i), delta = current - clipmapPositions_[i];
            clipmapPositions_[i] = current;
            auto& forecast = clipmapForecast_.displacement[i];
            forecast = length(delta) > jump ? Vec{} : forecast * (1 - blend) + delta * (lookAhead * blend / elapsed);
        }
    }

    void updateTerrain(bool force) {
        fullTerrainBuild_ = force;
        const auto stats = world_->update(invokers_, true, camera().eye);
        fullTerrainBuild_ = false;
        finishTerrainUpdate(stats, force);
    }

    void finishTerrainUpdate(const WorldStats& stats, bool force) {
        if (force) renderer_->clearTerrain();
        stagePreparedTerrain(true);
        applyTerrain(stats, force);
    }

    void prepareTerrainPages(const Chunks& live, const Chunks& staged, const std::vector<Key>& removed) {
        std::map<Key, TerrainPage> dirty;
        const auto edit = [&](Key key) -> TerrainPage& {
            const auto page = terrainPage(key);
            const auto found = terrainPages_.find(page);
            if (found != terrainPages_.end()) return dirty.try_emplace(page, found->second).first->second;
            return dirty[page];
        };
        if (fullTerrainBuild_) dirty = terrainPages_;
        for (const auto key : removed) if (!live.at(key)->mesh.empty()) edit(key).chunks.erase(key);
        for (const auto& [key, chunk] : staged) if (!chunk->mesh.empty()) edit(key).chunks.insert(key);
        preparedTerrain_.clear();
        preparedTerrain_.reserve(dirty.size());
        for (auto& [key, page] : dirty) preparedTerrain_.push_back({key, std::move(page), {}, {}});
        if (preparedTerrain_.empty()) return;
        tbb::task_arena arena(6);
        arena.execute([&] {
            tbb::parallel_for(std::size_t{0}, preparedTerrain_.size(), [&](std::size_t index) {
                bakeTerrainPage(preparedTerrain_[index], live, staged);
            });
        });
    }

    void bakeTerrainPage(PreparedTerrainPage& prepared, const Chunks& live, const Chunks& staged) const {
        const auto chunkAt = [&](Key key) -> const Chunk& {
            const auto candidate = staged.find(key);
            return *(candidate != staged.end() ? candidate->second : live.at(key));
        };
        auto& page = prepared.page;
        page.triangles = 0;
        for (const auto key : page.chunks) page.triangles += chunkAt(key).mesh.size();
        if (!page.triangles) return;
        const double infinity = std::numeric_limits<double>::infinity();
        prepared.bounds = {splat(infinity), splat(-infinity)};
        prepared.vertices.reserve(page.triangles * 3);
        const bool classic = planet_.preset() == TerrainPreset::Classic;
        const double geologyHalfRange = planet_.preset() == TerrainPreset::Mars ? 4700 : 4500;
        struct CachedVertex { const Chunk* chunk{}; Vec position; GpuVertex vertex; };
        std::array<CachedVertex, 256> cache{};
        for (const auto key : page.chunks) {
            const Chunk& chunk = chunkAt(key);
            const double t = std::clamp((chunk.spacing - 400.0) / 1200.0, 0.0, 1.0);
            const double reliefDetail = 1 - t * t * (3 - 2 * t);
            for (const auto& triangle : chunk.mesh) for (const Vec point : triangle.vertices) {
                std::uint64_t bits[3];
                static_assert(sizeof(point) == sizeof(bits));
                std::memcpy(bits, &point, sizeof(bits));
                const auto hash = bits[0] * 0x9e3779b97f4a7c15ULL ^ bits[1] * 0xbf58476d1ce4e5b9ULL ^
                                  bits[2] * 0x94d049bb133111ebULL;
                auto& cached = cache[(hash ^ (hash >> 32)) & (cache.size() - 1)];
                if (cached.chunk == &chunk && cached.position.x == point.x && cached.position.y == point.y &&
                    cached.position.z == point.z) {
                    prepared.vertices.push_back(cached.vertex);
                    continue;
                }
                prepared.bounds.lo = min(prepared.bounds.lo, point);
                prepared.bounds.hi = max(prepared.bounds.hi, point);
                const Vec up = unit(point);
                Vec normal = up;
                double elevation = 0, craterMaterial = 0;
                if (reliefDetail > 0 || !classic) {
                    const auto sample = planet_.sampleSurface(up, chunk.spacing, reliefDetail > 0);
                    if (reliefDetail > 0) normal = unit(up + (sample.normal - up) * reliefDetail, up);
                    elevation = classic ? sample.elevation * reliefDetail : sample.elevation;
                    craterMaterial = sample.craterMaterial;
                }
                const double height = std::clamp(classic ? (elevation + 660) / 850 :
                    (elevation + geologyHalfRange) / (2 * geologyHalfRange), 0.0, 1.0);
                const double rock = std::clamp(2.4 * (1 - dot(normal, up)), 0.0, 1.0);
                cached = {&chunk, point, packVertex(vertex(point, normal, color({height, rock, craterMaterial}, chunk.lod)))};
                prepared.vertices.push_back(cached.vertex);
            }
        }
    }

    bool stagePreparedTerrain(bool immediate) {
        constexpr std::size_t uploadBytesPerFrame = 8 * 1024 * 1024;
        const auto start = Clock::now();
        std::size_t bytes = 0, pages = 0;
        while (nextTerrainUpload_ < preparedTerrain_.size()) {
            auto& prepared = preparedTerrain_[nextTerrainUpload_];
            const auto size = prepared.vertices.size() * sizeof(GpuVertex);
            if (!immediate && pages && (bytes + size > uploadBytesPerFrame || pages >= 64 || milliseconds(start) >= 1.5))
                return false;
            renderer_->stageTerrainPage(prepared.key, prepared.bounds, prepared.vertices);
            std::vector<GpuVertex>().swap(prepared.vertices);
            bytes += size;
            ++pages;
            ++nextTerrainUpload_;
        }
        return immediate || renderer_->terrainUploadsReady();
    }

    void applyTerrain(const WorldStats& stats, bool force) {
        // The same batch becomes visible to rendering and collision before the next frame.
        renderer_->publishTerrainPages();
        if (force) {
            terrainPages_.clear();
            triangles_ = 0;
        }
        for (auto& prepared : preparedTerrain_) {
            const auto prior = terrainPages_.find(prepared.key);
            if (prior != terrainPages_.end()) {
                triangles_ -= prior->second.triangles;
                terrainPages_.erase(prior);
            }
            triangles_ += prepared.page.triangles;
            if (!prepared.page.chunks.empty()) terrainPages_.emplace(prepared.key, std::move(prepared.page));
        }
        preparedTerrain_.clear();
        nextTerrainUpload_ = 0;
        if (force || stats.windowVisits != 0 || stats.build.built != 0 || stats.build.removed != 0) {
            streamMs_ = stats.updateMs;
            streamBuilt_ = stats.build.built;
        }
        if (showRanges_ && (force || stats.windowsChanged)) refreshRanges();
        sweep_.onTerrainPublished(force, stats, invokers_, camera().eye);
    }

    void refreshRanges() {
        std::vector<Vertex> lines;
        if (showRanges_) {
            for (const auto& w : world_->clipmapWindows()) {
                const auto tint = color(lodColor(w.lod), 255);
                for (int i = 0; i < 8; ++i) for (int axis = 0; axis < 3; ++axis) {
                    if (i & (1 << axis)) continue;
                    const Vec a{(i & 1) ? w.box.hi.x : w.box.lo.x, (i & 2) ? w.box.hi.y : w.box.lo.y,
                                (i & 4) ? w.box.hi.z : w.box.lo.z};
                    Vec b = a;
                    b[axis] = w.box.hi[axis];
                    lines.push_back(vertex(a, {0, 0, 1}, tint));
                    lines.push_back(vertex(b, {0, 0, 1}, tint));
                }
            }
        }
        if (sweepMode_) sweep_.appendOverlay(lines, camera());
        renderer_->setClipmapLines(lines);
    }

    void buildObjects() {
        objects_.clear();
        if (sweepMode_) {
            sweep_.appendShapes(objects_);
            std::vector<Vertex> overlay;
            sweep_.appendOverlay(overlay, camera());
            renderer_->setClipmapLines(overlay);
            return;
        }
        if (surface_ || freeView_) {
            const Vec focus = freeView_ ? flight_.position() : invokers_[selected_];
            for (const auto& body : shapes_) {
                const auto pose = world_->physics.bodyPose(body.index);
                if (!freeView_ && length(pose.position - focus) > (controlling_ ? followDistance_ : surfaceDistance_) * 2.7) continue;
                Shape shape = body.shape;
                shape.axes = pose.axes;
                const auto first = objects_.size();
                if (shape.kind == ShapeKind::Box) appendBox(objects_, shape, pose.position, body.material);
                else appendRound(objects_, shape, pose.position, body.material);
                for (auto i = first; i < objects_.size(); ++i) objects_[i].motionId = body.index + 1;
            }
        }
        for (std::size_t i = 0; i < invokers_.size(); ++i) {
            const auto first = objects_.size();
            appendBeacon(objects_, invokers_[i], surface_ || freeView_ ? 24 : 380, i == selected_);
            for (auto j = first; j < objects_.size(); ++j) objects_[j].motionId = (1ull << 63) | (i + 1);
        }
    }

    Camera camera() const {
        const auto textureSeed = std::uint32_t((opts_.seed ^ (opts_.seed >> 32)) % 1021);
        if (sweepMode_ && sweep_.focusValid()) {
            const Vec target = sweep_.focus();
            const Vec radial = unit(target);
            const auto basis = frameAt(radial);
            const double distance = sweepDistance_ > 0 ? sweepDistance_ : sweep_.focusDistance();
            const Vec dir = radial * std::cos(sweepPitch_)
                          + basis[0] * std::sin(sweepYaw_) * std::cos(sweepPitch_)
                          + basis[1] * std::sin(sweepPitch_);
            const Vec eye = target + dir * distance;
            const Vec forward = unit(target - eye), right = unit(cross(forward, radial), basis[0]);
            return {eye, right, unit(cross(right, forward)), forward, 0.3, 20 * planetRadius,
                    lodTint_, wireframe_, {}, opts_.render, opts_.terrain, textureSeed, simulationTime_};
        }
        if (freeView_) {
            return {flight_.position(), flight_.right(), flight_.up(), flight_.forward(), 0.3, 20 * planetRadius,
                    lodTint_, wireframe_, {}, opts_.render, opts_.terrain, textureSeed, simulationTime_};
        }
        const Vec target = invokers_[selected_];
        const Vec radial = unit(target);
        const auto basis = frameAt(radial);
        Vec eye, center, nominalUp;
        double nearPlane;
        if (surface_) {
            const double distance = controlling_ ? followDistance_ : surfaceDistance_;
            const Vec tangent = basis[0] * std::cos(bearing_) + basis[1] * std::sin(bearing_);
            const Vec offset = radial * std::sin(elevation_) + tangent * std::cos(elevation_);
            center = target + radial * 24;
            eye = center + offset * distance;
            const double clearance = length(planet_.surface(unit(eye))) + 30 - length(eye);
            if (clearance > 0) eye = eye + unit(eye) * clearance;
            nominalUp = radial;
            nearPlane = std::max(0.3, distance * 0.004);
        } else {
            const Vec orbit = radial * (std::cos(yaw_) * std::cos(pitch_)) + basis[0] * (std::sin(yaw_) * std::cos(pitch_))
                              + basis[1] * std::sin(pitch_);
            center = {};
            eye = orbit * orbitDistance_;
            nominalUp = basis[1];
            nearPlane = std::max(12.0, (orbitDistance_ - planetRadius) * 0.03);
        }
        const Vec forward = unit(center - eye), right = unit(cross(forward, nominalUp), basis[0]);
        return {eye, right, unit(cross(right, forward)), forward, nearPlane, 20 * planetRadius,
                lodTint_, wireframe_, {}, opts_.render, opts_.terrain, textureSeed, simulationTime_};
    }

    Vec cloudFlightTarget(const RenderSettings& settings) const {
        // A daylit weather front in the default seed. A fixed planet-space
        // target makes the descent and ascent directly comparable.
        const Vec direction = unit(Vec{0.35485, -0.61883, 0.70081});
        return direction * (planetRadius + (settings.cloudBaseKm + settings.cloudThicknessKm * 0.45) * 1000);
    }

    Camera cloudFlightCamera(double progress) const {
        Camera view = camera();
        view.settings.cloudsEnabled = true;
        // Travel along the same line of sight on both legs so the frames can
        // be compared at matching camera positions.
        const double leg = 1 - std::abs(2 * progress - 1);
        const double eased = leg * leg * (3 - 2 * leg);
        const double distance = 900 * std::pow(4e6 / 900, 1 - eased);
        const Vec target = cloudFlightTarget(view.settings);
        const Vec normal = unit(target);
        const Vec offset = unit(normal * 0.77 + frameAt(normal)[0] * 0.64);
        view.eye = target + offset * distance;
        view.forward = offset * -1;
        view.right = unit(cross(view.forward, normal));
        view.up = cross(view.right, view.forward);
        view.nearPlane = std::max(0.3, distance * 0.0001);
        view.timeSeconds = 0;
        return view;
    }

    int replayCloudFlight() {
        auto start = Clock::now();
        for (;;) {
            MSG message{};
            while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
                if (message.message == WM_QUIT || (message.message == WM_KEYDOWN && message.wParam == VK_ESCAPE)) return 0;
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }
            if (closed_) return 0;
            const double progress = std::chrono::duration<double>(Clock::now() - start).count() / 16;
            auto view = cloudFlightCamera(std::min(1.0, progress));
            view.timeSeconds = std::min(1.0, progress) * 16;
            renderer_->draw(view, {});
            if (progress >= 1) return 0;
        }
    }

    int captureCloudFlight() {
        SetWindowPos(window_, nullptr, 0, 0, 1100, 800, SWP_NOMOVE | SWP_NOZORDER);
        renderer_->setClipmapLines({});
        renderer_->clearTerrain();
        renderer_->enableCloudGpuTiming();
        std::ofstream measurements(opts_.captures / "cloud-flight.csv");
        measurements.exceptions(std::ios::badbit | std::ios::failbit);
        measurements << "mode,frame,phase,distance_m,integration_ms,reconstruction_ms\n";
        constexpr int orbitFrames = 16, flightFrames = 193;
        const auto route = [&](const char* mode, const auto& configure) {
            renderer_->resetTemporalHistory();
            const auto viewAt = [&](double progress) {
                Camera view = cloudFlightCamera(progress);
                view.settings.shadowsEnabled = false;
                view.settings.taaCutDistanceKm = 2000; // The recorded orbit leg advances in large, continuous increments.
                configure(view.settings);
                return view;
            };
            for (int i = 0; i < 24; ++i) renderer_->draw(viewAt(0), {});
            for (int i = 0; i < orbitFrames + flightFrames; ++i) {
                const bool orbit = i < orbitFrames;
                const double progress = orbit ? 0 : double(i - orbitFrames) / (flightFrames - 1);
                const auto view = viewAt(progress);
                const auto stem = std::string(mode) + "-" + std::to_string(i);
                renderer_->draw(view, {}, opts_.captures / (stem + ".bmp"));
                const auto timing = renderer_->cloudGpuTiming();
                const double distance = length(view.eye - cloudFlightTarget(view.settings));
                measurements << mode << ',' << i << ',' << (orbit ? "orbit" : progress <= 0.5 ? "descent" : "ascent") << ','
                             << distance << ',' << timing.integrationMs << ',' << timing.reconstructionMs << '\n';
            }
        };
        route("detail-off", [](RenderSettings& settings) { settings.cloudDetailStrength = 0; });
        route("enhanced", [](RenderSettings&) {});
        route("full", [](RenderSettings& settings) { settings.taaEnabled = false; settings.cloudTemporalEnabled = false; });
        route("full-detail-off", [](RenderSettings& settings) {
            settings.taaEnabled = false; settings.cloudTemporalEnabled = false; settings.cloudDetailStrength = 0;
        });
        route("shadow-off", [](RenderSettings& settings) { settings.cloudSelfShadowStrength = 0; });
        Camera view = cloudFlightCamera(0.5);
        view.settings.shadowsEnabled = false;
        renderer_->resetTemporalHistory();
        for (int i = 0; i < 24; ++i) renderer_->draw(view, {});
        for (int i = 0; i < orbitFrames; ++i)
            renderer_->draw(view, {}, opts_.captures / ("inside-" + std::to_string(i) + ".bmp"));
        view.settings.taaEnabled = false;
        for (int i = 0; i < 2; ++i) {
            view.sunDirection = i == 0 ? Vec{0.45, 0.25, 0.86} : Vec{-0.70, 0.25, 0.68};
            const auto stem = "sun-" + std::to_string(i);
            renderer_->draw(view, {}, opts_.captures / (stem + "-on.bmp"), true);
            Camera clear = view; clear.settings.cloudSelfShadowStrength = 0;
            renderer_->draw(clear, {}, opts_.captures / (stem + "-off.bmp"), true);
        }
        std::ofstream report(opts_.captures / "cloud-flight.txt");
        RECT viewport{};
        GetClientRect(viewport_, &viewport);
        report << "renderer=" << (renderer_->isWarp() ? "WARP" : "hardware") << "\norbit_frames=" << orbitFrames
               << "\nflight_frames=" << flightFrames << "\ncoverage=" << opts_.render.cloudCoverage
               << "\nviewport=" << viewport.right << 'x' << viewport.bottom << '\n';
        return 0;
    }

    void setLod0Radius(std::uint32_t radius) {
        world_->setLod0Radius(radius);
        lod0Radius_ = radius;
        updateFooter();
    }

    int captureLodRanges() {
        std::filesystem::create_directories(opts_.captures);
        paused_ = true;
        lodTint_ = true;
        opts_.render.cloudsEnabled = opts_.render.atmosphereEnabled = opts_.render.shadowsEnabled = false;
        opts_.render.taaEnabled = false;
        buildObjects();
        renderer_->draw(camera(), objects_, opts_.captures / "lod-range-512.bmp", true);
        const auto before = world_->lodCounts();
        std::ofstream report(opts_.captures / "lod-range-validation.txt");
        const auto record = [&](const char* stage) {
            updateFooter();
            report << stage << "_radius=" << world_->lod0Radius() << '\n' << stage << "_counts=";
            for (const auto count : world_->lodCounts()) report << count << ',';
            const int bytes = WideCharToMultiByte(CP_UTF8, 0, status_.data(), int(status_.size()), nullptr, 0, nullptr, nullptr);
            std::string text(std::size_t(bytes), '\0');
            WideCharToMultiByte(CP_UTF8, 0, status_.data(), int(status_.size()), text.data(), bytes, nullptr, nullptr);
            report << '\n' << stage << "_status=" << text << '\n';
        };
        record("before");
        const auto replace = [&](std::uint32_t radius) {
            setLod0Radius(radius);
            if (!world_->requestUpdate(invokers_, true, camera().eye, true))
                throw std::runtime_error("LOD range capture could not schedule the update");
            if (radius == 4096) {
                if (world_->lodCounts() != before || world_->lod0Radius() != 512)
                    throw std::runtime_error("Pending LOD range was published before GPU upload");
                record("pending");
                renderer_->draw(camera(), objects_, opts_.captures / "lod-range-pending.bmp", true);
            }
            const auto start = Clock::now();
            for (;;) {
                if (const auto stats = world_->pollUpdate([&] { return stagePreparedTerrain(false); })) {
                    applyTerrain(*stats, false);
                    report << "update_" << radius << "_ms=" << stats->updateMs << '\n';
                    break;
                }
                if (milliseconds(start) > 120000) throw std::runtime_error("LOD range publication timed out");
                renderer_->draw(camera(), objects_); // Executes staged GPU copies, as in the interactive loop.
            }
            if (world_->lod0Radius() != radius || !renderer_->terrainUploadsReady())
                throw std::runtime_error("LOD range did not publish together with its GPU meshes");
        };
        replace(4096);
        if (world_->lodCounts()[0] <= before[0]) throw std::runtime_error("4096 m range did not expand resident LOD0 blocks");
        record("expanded");
        renderer_->draw(camera(), objects_, opts_.captures / "lod-range-4096.bmp", true);
        replace(512);
        if (world_->lodCounts() != before) throw std::runtime_error("512 m range did not restore resident block counts");
        record("restored");
        renderer_->draw(camera(), objects_, opts_.captures / "lod-range-restored.bmp", true);
        report << "scenarios=passed\n";
        return 0;
    }

    int captureStability() {
        std::filesystem::create_directories(opts_.captures);
        SetWindowPos(window_, nullptr, 0, 0, 960, 720, SWP_NOMOVE | SWP_NOZORDER);
        renderer_->setClipmapLines({});
        const auto sequence = [&](const std::string& name, const Camera& view) {
            renderer_->resetTemporalHistory();
            for (int frame = 0; frame < 64; ++frame) renderer_->draw(view, {});
            for (int frame = 0; frame < 32; ++frame)
                renderer_->draw(view, {}, opts_.captures / ("taa-stability-" + name + "-" + std::to_string(frame) + ".bmp"), true);
        };
        Camera orbit = camera();
        orbit.timeSeconds = 0;
        orbit.eye = Vec{0,-0.3,2.5} * planetRadius;
        orbit.forward = unit(orbit.eye * -1); orbit.right = unit(cross(orbit.forward, {0,1,0}));
        orbit.up = cross(orbit.right, orbit.forward); orbit.nearPlane = 10;
        orbit.settings = opts_.render;
        orbit.settings.shadowsEnabled = false;
        Camera clear = orbit; clear.settings.cloudsEnabled = false;
        sequence("clear", clear);
        Camera full = orbit; full.settings.cloudSampling = 0;
        sequence("orbit-full", full);
        sequence("orbit-checker", orbit);
        Camera horizon = orbit;
        horizon.eye = {0,0,planetRadius + 18000}; horizon.forward = unit(Vec{1,0,-0.14});
        horizon.right = {0,-1,0}; horizon.up = cross(horizon.right,horizon.forward); horizon.nearPlane = 0.3;
        horizon.settings.cloudCoverage = 0.75;
        sequence("horizon-checker", horizon);
        Camera quarter = horizon; quarter.settings.cloudSampling = 2;
        sequence("horizon-quarter", quarter);
        Camera noJitter = horizon; noJitter.settings.taaJitter = 0;
        sequence("horizon-no-projection-jitter", noJitter);
        Camera noRayJitter = horizon; noRayJitter.settings.cloudRayJitter = 0;
        sequence("horizon-no-ray-jitter", noRayJitter);
        return 0;
    }

    int captureTemporal() {
        std::filesystem::create_directories(opts_.captures);
        SetWindowPos(window_, nullptr, 0, 0, 1100, 800, SWP_NOMOVE | SWP_NOZORDER);
        renderer_->setClipmapLines({});
        const Vec origin{0, 0, planetRadius + 3000};
        std::vector<Vertex> floor;
        Shape tile = sampleShape(3, {0,0,1}); tile.size = {6, 6, 0.15};
        for (int y = -12; y <= 12; ++y) for (int x = -12; x <= 12; ++x)
            appendBox(floor, tile, origin + Vec{x * 12.0, y * 12.0, -0.15},
                      color(((x + y) & 1) ? Vec{0.045,0.045,0.045} : Vec{0.8,0.8,0.8}, 254));
        Shape slat = tile; slat.size = {0.12, 65, 0.3};
        const double angle = 0.47;
        slat.axes = {Vec{std::cos(angle),std::sin(angle),0}, Vec{-std::sin(angle),std::cos(angle),0}, Vec{0,0,1}};
        for (int i = -8; i <= 8; ++i) appendBox(floor, slat, origin + Vec{i * 5.0,0,0.3}, color({0.95,0.95,0.95},254));
        renderer_->setTerrain(floor);
        Camera view{};
        view.eye = origin + Vec{3,-150,95};
        view.forward = unit(origin - view.eye); view.right = unit(cross(view.forward, {0,0,1}));
        view.up = cross(view.right, view.forward); view.nearPlane = 0.3; view.farPlane = 100000;
        view.settings.cloudsEnabled = false; view.settings.atmosphereEnabled = false;
        view.settings.shadowsEnabled = false; view.settings.taaSharpen = 0;
        const auto capture = [&](const std::string& name, const Camera& camera, const std::vector<Vertex>& objects = std::vector<Vertex>{}) {
            renderer_->draw(camera, objects, opts_.captures / (name + ".bmp"), true);
        };
        Camera off = view; off.settings.taaEnabled = false;
        capture("taa-off", off);
        renderer_->resetTemporalHistory();
        Camera reference = view; reference.settings.taaHistoryWeight = 0;
        for (int i = 0; i < 16; ++i) capture("taa-reference-" + std::to_string(i), reference);
        renderer_->resetTemporalHistory();
        for (int i = 0; i < 32; ++i) renderer_->draw(view, {});
        capture("taa-static", view);
        if (!renderer_->diagnostics().temporalHistoryValid || renderer_->diagnostics().taaHistoryPixels < 1000)
            throw std::runtime_error("TAA did not accumulate a valid history");
        Shape moving = sampleShape(3, {0,0,1}); moving.size = {7,7,9};
        std::vector<Vertex> body;
        for (int frame = 0; frame < 24; ++frame) {
            body.clear();
            appendBox(body, moving, origin + Vec{-32 + frame * 2.8,0,9}, color({0.8,0.045,0.02},254));
            for (auto& vertex : body) vertex.motionId = 123;
            if (frame == 23) capture("taa-moving", view, body);
            else renderer_->draw(view, body);
        }
        if (renderer_->diagnostics().movingObjectPixels < 100)
            throw std::runtime_error("Moving rigid-body vertices did not produce object motion vectors");
        capture("taa-moving-reference", off, body);
        renderer_->resetTemporalHistory();
        for (int frame = 0; frame < 16; ++frame) {
            Camera pan = view; pan.eye = pan.eye + Vec{frame * 0.6,0,0}; renderer_->draw(pan, {});
            if (frame == 15) capture("taa-camera-pan", pan);
        }
        Camera cut = view; cut.eye = cut.eye + Vec{1000000,0,0};
        capture("taa-camera-cut", cut);
        if (renderer_->diagnostics().temporalHistoryValid) throw std::runtime_error("Camera cut retained stale TAA history");
        renderer_->resetTemporalHistory(); capture("taa-cut-reference", cut);
        for (int i = 0; i < 16; ++i) renderer_->draw(view, {});
        SetWindowPos(window_, nullptr, 0, 0, 1200, 850, SWP_NOMOVE | SWP_NOZORDER);
        capture("taa-resized", view);
        if (renderer_->diagnostics().temporalHistoryValid) throw std::runtime_error("Resize retained stale TAA history");
        SetWindowPos(window_, nullptr, 0, 0, 1100, 800, SWP_NOMOVE | SWP_NOZORDER);
        // Rebase both camera and planet without a fictitious 1e12 m motion.
        // Binary-exact visible geometry isolates temporal reprojection from the
        // source-coordinate quantization of the subpixel diagonal slats at 1e12 m.
        std::vector<Vertex> precisionFloor, precisionBody;
        Shape exactTile = tile; exactTile.size.z = 0.125;
        for (int y = -12; y <= 12; ++y) for (int x = -12; x <= 12; ++x)
            appendBox(precisionFloor, exactTile, origin + Vec{x * 12.0,y * 12.0,-0.125},
                      color(((x + y) & 1) ? Vec{0.045,0.045,0.045} : Vec{0.8,0.8,0.8},254));
        appendBox(precisionBody, moving, origin + Vec{32,0,9}, color({0.8,0.045,0.02},254));
        for (auto& vertex : precisionBody) vertex.motionId = 123;
        renderer_->setTerrain(precisionFloor);
        for (int i = 0; i < 16; ++i) renderer_->draw(view, precisionBody);
        capture("taa-precision-base", view, precisionBody);
        const Vec translation{1e12,-1e12,1e12};
        auto shiftedFloor = precisionFloor, shiftedBody = precisionBody;
        for (auto& vertex : shiftedFloor) vertex.position = vertex.position + translation;
        for (auto& vertex : shiftedBody) vertex.position = vertex.position + translation;
        renderer_->setTerrain(shiftedFloor);
        Camera shifted = view; shifted.eye = shifted.eye + translation; shifted.planetCenter = translation;
        for (int i = 0; i < 16; ++i) renderer_->draw(shifted, shiftedBody);
        capture("taa-precision-translated", shifted, shiftedBody);

        rebuild();
        Camera clouds = camera();
        clouds.eye = Vec{0,-0.3,2.5} * planetRadius;
        clouds.forward = unit(clouds.eye * -1); clouds.right = unit(cross(clouds.forward,{0,1,0}));
        clouds.up = cross(clouds.right,clouds.forward); clouds.nearPlane = 10;
        clouds.settings.taaEnabled = true; clouds.settings.cloudsEnabled = true;
        clouds.settings.cloudTemporalEnabled = true; clouds.settings.taaSharpen = 0;
        std::array<std::uint64_t,3> steps{};
        for (int pattern = 0; pattern < 3; ++pattern) {
            clouds.settings.cloudSampling = pattern;
            renderer_->resetTemporalHistory();
            for (int i = 0; i < 32; ++i) renderer_->draw(clouds, {});
            capture("taa-cloud-pattern-" + std::to_string(pattern), clouds);
            steps[std::size_t(pattern)] = renderer_->diagnostics().cloudRaySteps;
            if (pattern && renderer_->diagnostics().cloudSkippedPixels == 0)
                throw std::runtime_error("Sparse cloud sampling did not skip any raymarches");
        }
        clouds.settings.taaEnabled = false;
        capture("taa-cloud-no-taa", clouds);
        if (renderer_->diagnostics().cloudSkippedPixels || renderer_->diagnostics().cloudFallbackPixels)
            throw std::runtime_error("Disabling TAA left sparse cloud rendering enabled");
        clouds.settings.taaEnabled = true;
        capture("taa-cloud-reenabled", clouds);
        if (renderer_->diagnostics().temporalHistoryValid || renderer_->diagnostics().cloudSkippedPixels)
            throw std::runtime_error("Enabling TAA reused invalid cloud history");
        if (!steps[0] || steps[1] >= steps[0] * 0.75 || steps[2] >= steps[0] * 0.55)
            throw std::runtime_error("Cloud temporal reconstruction did not reduce primary raymarch work");
        // A foreground object is introduced and removed WITHOUT clearing history.
        // Newly visible cloud pixels must trace immediately, even on skipped phases.
        Camera foreground = clouds;
        foreground.eye = {0,0,planetRadius + 2000}; foreground.forward = {0,0,1};
        foreground.right = {1,0,0}; foreground.up = {0,1,0}; foreground.nearPlane = 0.3;
        foreground.settings.cloudCoverage = 1; foreground.settings.shadowsEnabled = false;
        foreground.settings.cloudSampling = 2;
        std::vector<Vertex> blocker;
        Shape plate = sampleShape(3,{0,0,1}); plate.size = {20000,20000,5};
        appendBox(blocker, plate, foreground.eye + Vec{0,0,500}, color({0.65,0.08,0.025},254));
        for (auto& vertex : blocker) vertex.motionId = 888;
        for (int i = 0; i < 16; ++i) renderer_->draw(foreground, {});
        capture("taa-cloud-occluded", foreground, blocker);
        capture("taa-cloud-occluded-reference", [&] { auto v = foreground; v.settings.taaEnabled = false; return v; }(), blocker);
        for (int i = 0; i < 16; ++i) renderer_->draw(foreground, blocker);
        capture("taa-cloud-disoccluded", foreground);
        const auto disoccluded = renderer_->diagnostics();
        if (!disoccluded.cloudSkippedPixels || disoccluded.cloudFallbackPixels < disoccluded.cloudSkippedPixels * 0.9 ||
            disoccluded.maxCloudOpacity < 0.1)
            throw std::runtime_error("Disoccluded sparse cloud pixels did not raymarch missing history");
        renderer_->resetTemporalHistory(); capture("taa-cloud-disoccluded-reference", foreground);
        Camera wind = foreground;
        wind.eye.z = planetRadius + 18000; wind.forward = unit(Vec{1,0,-0.14});
        wind.right = {0,-1,0}; wind.up = cross(wind.right,wind.forward);
        wind.settings.cloudWindSpeed = 250; wind.settings.cloudFormationKm = 18;
        for (int i = 0; i < 32; ++i) { wind.timeSeconds = i / 60.0; renderer_->draw(wind, {}); }
        wind.timeSeconds = 32 / 60.0; capture("taa-cloud-wind", wind);
        std::ofstream report(opts_.captures / "temporal-validation.txt");
        report << "renderer=" << (renderer_->isWarp() ? "WARP" : "hardware") << "\nfull_steps=" << steps[0]
               << "\nchecker_steps=" << steps[1] << "\nquarter_steps=" << steps[2] << "\nscenarios=passed\n";
        return 0;
    }

    int captureClouds() {
        std::filesystem::create_directories(opts_.captures);
        const bool previousClouds = opts_.render.cloudsEnabled;
        key('V', true);
        if (opts_.render.cloudsEnabled == previousClouds) throw std::runtime_error("V did not toggle planetary clouds");
        key('V', true);
        SetWindowPos(window_, nullptr, 0, 0, opts_.warp ? 960 : 1280, opts_.warp ? 720 : 900, SWP_NOMOVE | SWP_NOZORDER);
        renderer_->setClipmapLines({});
        Camera orbit = camera();
        orbit.settings.taaEnabled = false;
        orbit.eye = Vec{0, -0.3, 2.5} * planetRadius;
        orbit.forward = unit(orbit.eye * -1);
        orbit.right = unit(cross(orbit.forward, {0, 1, 0}));
        orbit.up = cross(orbit.right, orbit.forward);
        orbit.nearPlane = 10;
        orbit.settings.cloudsEnabled = true;
        orbit.timeSeconds = 0;
        const auto capture = [&](const char* name, const Camera& view) {
            renderer_->draw(view, {}, opts_.captures / (std::string(name) + ".bmp"), true);
        };
        capture("cloud-orbit", orbit);
        const auto orbitClouds = renderer_->diagnostics().cloudPixels;
        if (orbitClouds < 500) throw std::runtime_error("Planetary cloud capture produced no significant cloud coverage");
        Camera clear = orbit; clear.settings.cloudsEnabled = false;
        capture("cloud-orbit-off", clear);
        clear = orbit; clear.settings.cloudCoverage = 0;
        capture("cloud-zero-coverage", clear);
        clear = orbit; clear.settings.cloudDensity = 0;
        capture("cloud-zero-density", clear);
        Camera quality = orbit; quality.settings.cloudSteps = 32;
        capture("cloud-orbit-low", quality);
        quality.settings.cloudSteps = 96;
        capture("cloud-orbit-high", quality);
        Camera local = orbit;
        local.eye = {0, 0, planetRadius + 2000};
        local.forward = unit(Vec{1, 0, 0.18});
        local.right = {0, -1, 0}; local.up = cross(local.right, local.forward);
        capture("cloud-below", local);
        local.eye.z = planetRadius + 60000;
        local.forward = unit(Vec{1, 0, -0.22}); local.up = cross(local.right, local.forward);
        capture("cloud-horizon", local);
        local.eye.z = planetRadius + 9000;
        local.forward = {1, 0, 0}; local.up = {0, 0, 1};
        local.settings.cloudCoverage = 1;
        local.settings.cloudWindSpeed = 250;
        capture("cloud-inside", local);
        local.timeSeconds = 600;
        capture("cloud-wind", local);
        local.timeSeconds = 0;
        // Cloud-only captures isolate the actual large-coordinate upload path.
        renderer_->setTerrain({});
        capture("cloud-precision-base", local);
        const Vec translation{1e12, -1e12, 1e12};
        local.eye = local.eye + translation; local.planetCenter = translation;
        capture("cloud-precision-translated", local);
        local.planetCenter = {};
        local.eye = {0, 0, planetRadius + 2000};
        local.forward = {0, 0, 1}; local.right = {1, 0, 0}; local.up = {0, 1, 0};
        Shape blocker = sampleShape(3, {0, 0, 1}); blocker.size = {20000, 20000, 5};
        std::vector<Vertex> occluder;
        appendBox(occluder, blocker, local.eye + Vec{0, 0, 500}, color({0.6, 0.12, 0.08}, 254));
        renderer_->setTerrain(occluder);
        capture("cloud-occluded-on", local);
        local.settings.cloudsEnabled = false;
        capture("cloud-occluded-off", local);
        std::ofstream report(opts_.captures / "cloud-validation.txt");
        report << "renderer=" << (renderer_->isWarp() ? "WARP" : "hardware")
               << "\norbit_cloud_pixels=" << orbitClouds
               << "\nprecision_translation=1000000000000\nscenarios=14\n";
        return 0;
    }

    int captureAll() {
        // Keep the existing atmosphere/terrain reference suite independent of
        // the new cloud art direction. --capture-clouds validates clouds.
        opts_.render.cloudsEnabled = false;
        opts_.render.taaEnabled = false;
        const auto referenceSettings = [] {
            RenderSettings settings;
            settings.cloudsEnabled = false;
            settings.taaEnabled = false;
            return settings;
        };
        std::filesystem::create_directories(opts_.captures);
        std::ofstream report(opts_.captures / "validation.txt");
        report.exceptions(std::ios::badbit | std::ios::failbit);
        report << "Direct3D 12 " << (renderer_->isWarp() ? "WARP" : "hardware") << '\n';
        report << "terrain=" << name(opts_.terrain) << '\n';
        for (int i = 0; i < int(methods.size()); ++i) {
            method_ = i;
            selected_ = 0;
            surface_ = false;
            rebuild();
            MSG message{};
            while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
                if (message.message == WM_QUIT) return 1;
                DispatchMessageW(&message);
            }
            if (closed_) return 1;
            const std::string prefix = name(methods[i]);
            surface_ = false;
            lodTint_ = false;
            buildObjects();
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-orbit.bmp"));
            if (methods[i] == Method::Grid) {
                Camera orbitClear = camera();
                orbitClear.settings = referenceSettings();
                orbitClear.settings.atmosphereEnabled = false;
                orbitClear.settings.shadowsEnabled = false;
                renderer_->draw(orbitClear, objects_, opts_.captures / (prefix + "-orbit-no-effects.bmp"));
                Camera atmosphereCheck = orbitClear;
                atmosphereCheck.settings.atmosphereEnabled = true;
                renderer_->draw(atmosphereCheck, objects_, opts_.captures / "grid-atmosphere-100km.bmp");
                atmosphereCheck.settings.atmosphereHeightKm = 300;
                renderer_->draw(atmosphereCheck, objects_, opts_.captures / "grid-atmosphere-300km.bmp");
            }
            Camera orbitWire = camera();
            orbitWire.wireframe = true;
            orbitWire.settings.atmosphereEnabled = false;
            renderer_->draw(orbitWire, objects_, opts_.captures / (prefix + "-orbit-wireframe.bmp"));
            int maxContacts = 0;
            for (int step = 0; step < 190; ++step) {
                tick();
                maxContacts = std::max(maxContacts, world_->physics.terrainContacts());
            }
            updateTerrain(false);
            // Center the surface capture on the test bodies' original drop location.
            invokers_ = invokerPositions(planet_, opts_.invokers, 0);
            surface_ = true;
            updateTerrain(false);
            buildObjects();
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-surface.bmp"));
            const Camera anchored = camera();
            key('C', true);
            if (!freeView_ || length(camera().eye - anchored.eye) > 1e-8 ||
                length(camera().forward - anchored.forward) > 1e-8)
                throw std::runtime_error("Free camera did not preserve the entry view");
            const bool previousWireframe = wireframe_, previousAtmosphere = opts_.render.atmosphereEnabled;
            key('W', true);
            key('A', true);
            if (wireframe_ != previousWireframe || opts_.render.atmosphereEnabled != previousAtmosphere)
                throw std::runtime_error("WASD conflicts with a render shortcut");
            flight_.look(0.12, 0.08);
            flight_.move(1, 1, 0.5, 1, flightSpeed_);
            updateTerrain(false);
            buildObjects();
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-free.bmp"));
            key(VK_F2, true);
            if (wireframe_ == previousWireframe) throw std::runtime_error("F2 did not toggle wireframe");
            key(VK_F2, true);
            key('H', true);
            if (opts_.render.atmosphereEnabled == previousAtmosphere) throw std::runtime_error("H did not toggle atmosphere");
            key('H', true);
            key('C', true);
            if (freeView_ || length(camera().eye - anchored.eye) > 1e-8)
                throw std::runtime_error("Free camera did not restore the anchored view");
            updateTerrain(false);
            buildObjects();
            Camera noAtmosphere = camera();
            noAtmosphere.settings.atmosphereEnabled = false;
            renderer_->draw(noAtmosphere, objects_, opts_.captures / (prefix + "-surface-no-atmosphere.bmp"));
            Camera horizon = camera();
            const Vec up = unit(invokers_[selected_]);
            const auto axes = frameAt(up);
            horizon.eye = up * (planetRadius + 12000);
            horizon.forward = unit(axes[0] - up * 0.04);
            horizon.right = unit(cross(horizon.forward, up));
            horizon.up = cross(horizon.right, horizon.forward);
            horizon.nearPlane = 10;
            renderer_->draw(horizon, objects_, opts_.captures / (prefix + "-horizon.bmp"));
            Camera pullback = horizon;
            pullback.eye = up * (planetRadius + 100000);
            pullback.forward = up * -1;
            pullback.right = axes[0];
            pullback.up = cross(pullback.right, pullback.forward);
            pullback.wireframe = true;
            pullback.settings.atmosphereEnabled = false;
            pullback.lodTint = true;
            renderer_->draw(pullback, objects_, opts_.captures / (prefix + "-global-lod-wireframe.bmp"));
            action(Wireframe);
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-wireframe.bmp"));
            action(Wireframe);
            showRanges_ = true;
            refreshRanges();
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-ranges.bmp"));
            showRanges_ = false;
            refreshRanges();
            report << prefix << " triangles=" << triangles_ << " invokers=" << opts_.invokers << " bodies=" << shapes_.size()
                   << " peak_contacts=" << maxContacts << '\n';
            updateFooter();
            // The diagnostic labels and numeric formatting above are ASCII.
            for (const auto* line : {&processMemory_, &terrainMemory_}) {
                for (const wchar_t ch : *line) report.put(static_cast<char>(ch));
                report.put('\n');
            }
            if (maxContacts == 0) throw std::runtime_error("Capture scenario did not observe terrain contact");
            if (methods[i] == Method::Grid && opts_.terrain == TerrainPreset::Mars) captureRemoteMars();
        }
        // End-to-end check of the actual DX12 vertex/camera/planet upload paths
        // with a common trillion-meter world translation.
        const Vec up = unit(invokers_[selected_]);
        Shape box = sampleShape(3, up);
        box.size = {80, 80, 80};
        std::vector<Vertex> precisionGeometry;
        appendBox(precisionGeometry, box, invokers_[selected_] + up * 100, color({0.9, 0.45, 0.2}, 254));
        renderer_->setClipmapLines({});
        renderer_->setTerrain(precisionGeometry);
        Camera translated = camera();
        renderer_->draw(translated, {}, opts_.captures / "precision-base.bmp");
        const Vec translation{1e12, -1e12, 1e12};
        for (auto& v : precisionGeometry) v.position = v.position + translation;
        translated.eye = translated.eye + translation;
        translated.planetCenter = translation;
        renderer_->setTerrain(precisionGeometry);
        renderer_->draw(translated, {}, opts_.captures / "precision-translated.bmp");
        report << "precision_translation=1000000000000m\n";
        // Known receiver + caster for validating actual CSM/HDR GPU output.
        const Vec origin{0,0,planetRadius + 2000};
        std::vector<Vertex> ground, caster;
        Shape floor = sampleShape(3, {0,0,1}); floor.size = {400,400,2};
        appendBox(ground, floor, origin - Vec{0,0,2}, color({0.85,0.85,0.85}, 254));
        Shape cube = floor; cube.size = {18,18,45};
        appendBox(caster, cube, origin + Vec{0,0,45}, color({0.75,0.3,0.1}, 254));
        Camera validation{};
        validation.settings = referenceSettings();
        validation.eye = origin + Vec{0,-180,145};
        validation.forward = unit(origin + Vec{0,0,20} - validation.eye);
        validation.right = unit(cross(validation.forward, {0,0,1}));
        validation.up = cross(validation.right, validation.forward);
        validation.nearPlane = 0.3; validation.farPlane = 100000;
        renderer_->setTerrain(ground);
        renderer_->draw(validation, caster, opts_.captures / "pipeline-csm-on.bmp", true);
        const auto diagnostic = renderer_->diagnostics();
        std::size_t shadowPixels = 0;
        for (auto pixels : diagnostic.shadowPixels) shadowPixels += pixels;
        if (diagnostic.invalidValues || diagnostic.maxHdrComponent <= 1 || diagnostic.geometryPixels < 1000 || !shadowPixels)
            throw std::runtime_error("Deferred/HDR/CSM validation did not produce expected GPU buffers");
        validation.settings.shadowsEnabled = false;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-csm-off.bmp");
        validation.settings.exposureEV = 1;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-exposure-plus1.bmp");
        RECT saved{}; GetWindowRect(window_, &saved);
        SetWindowPos(window_, nullptr, 0, 0, 1100, 700, SWP_NOMOVE | SWP_NOZORDER);
        renderer_->draw(validation, caster, opts_.captures / "pipeline-resized.bmp", true);
        SetWindowPos(window_, nullptr, 0, 0, saved.right - saved.left, saved.bottom - saved.top, SWP_NOMOVE | SWP_NOZORDER);
        renderer_->draw(validation, caster, opts_.captures / "pipeline-restored.bmp");
        report << "pipeline=deferred HDR_RGBA16F CSM4\nhdr_max_component=" << diagnostic.maxHdrComponent
               << "\nshadow_depth_pixels=" << shadowPixels << "\nd3d12_debug_layer=" << diagnostic.debugLayer << '\n';

        validation.settings = referenceSettings();
        for (int edge : shadowResolutions) {
            if (edge == RenderSettings{}.shadowResolution) continue;
            validation.settings.shadowResolution = edge;
            const auto name = "pipeline-shadow-" + std::to_string(edge) + ".bmp";
            renderer_->draw(validation, caster, opts_.captures / name, true);
        }
        validation.settings = referenceSettings();
        validation.settings.toneMapping = ToneMapping::Reinhard;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-tone-reinhard.bmp");
        validation.settings.toneMapping = ToneMapping::Linear;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-tone-linear.bmp");
        validation.settings = referenceSettings();
        validation.settings.atmosphereHeightKm = 18;
        validation.settings.rayleighHeightKm = 4;
        validation.settings.mieHeightKm = 0.6;
        validation.settings.rayleighStrength = 5;
        validation.settings.mieStrength = 3;
        validation.settings.mieAnisotropy = 0.55;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-atmosphere-tuned.bmp");
        captureInvokerControls();
        report << "invoker_controls=passed\n";
        if (opts_.terrain == TerrainPreset::Classic) {
            for (const auto preset : {TerrainPreset::Mars, TerrainPreset::Volcanic, TerrainPreset::Classic}) {
                selectCaptureItem(TerrainStyle, int(preset));
                const Vec target = invokers_[selected_], direction = unit(target);
                const auto hit = world_->physics.ray(target + direction * 250, target - direction * 250);
                if (planet_.preset() != preset || camera().terrain != preset || triangles_ == 0 ||
                    std::abs(planet_.sdf(target)) > 1e-6 || hit.state != Hit::State::Contact)
                    throw std::runtime_error("Terrain dropdown did not replace both the rendered and collision fields");
            }
            report << "terrain_controls=passed\n";
        }
        return 0;
    }

    void captureRemoteMars() {
        constexpr std::array directions{Vec{0.34, -0.82, 0.46}, Vec{0.83, 0.52, -0.20}};
        for (std::size_t i = 0; i < directions.size(); ++i) {
            const Vec direction = unit(directions[i]);
            const Vec target = planet_.surface(direction);
            const auto local = frameAt(direction);
            Camera remote = camera();
            remote.eye = target + direction * 1760 + local[0] * 950;
            remote.forward = unit(target - remote.eye);
            remote.right = unit(cross(remote.forward, direction));
            remote.up = cross(remote.right, remote.forward);
            remote.nearPlane = 8;
            finishTerrainUpdate(world_->update({target}, true, remote.eye), false);
            if (!world_->hasFinestLod(target) ||
                world_->physics.ray(target + direction * 300, target - direction * 300).state != Hit::State::Contact)
                throw std::runtime_error("Remote Mars terrain did not publish fine collision coverage");
            renderer_->draw(remote, {}, opts_.captures / ("grid-mars-remote-" + std::to_string(i + 1) + ".bmp"));
        }
    }

    void selectCaptureItem(int command, int item) {
        const HWND field = combo(command);
        if (SendMessageW(field, CB_SETCURSEL, WPARAM(item), 0) == CB_ERR)
            throw std::runtime_error("Unable to select a viewer dropdown item");
        SendMessageW(window_, WM_COMMAND, MAKEWPARAM(command, CBN_SELENDOK), reinterpret_cast<LPARAM>(field));
        if (std::exchange(pending_, 0) != command) throw std::runtime_error("Viewer dropdown did not dispatch its selection");
        action(command);
    }

    void captureInvokerControls() {
        const auto bodyCount = shapes_.size();
        const Vec first = invokers_.front();
        selectCaptureItem(InvokerCount, 2);
        if (invokers_.size() != 3 || length(invokers_.front() - first) != 0 || shapes_.size() != bodyCount ||
            SendMessageW(combo(CameraTarget), CB_GETCOUNT, 0, 0) != 4)
            throw std::runtime_error("Changing Invoker count did not preserve the scene and update the camera list");

        selectCaptureItem(CameraTarget, 2);
        if (!controlling_ || !surface_ || freeView_ || selected_ != 1 ||
            (GetActiveWindow() == window_ && GetFocus() != viewport_))
            throw std::runtime_error("Camera selection did not hand control to the selected Invoker");
        const Vec start = invokers_[1];
        moveInvoker(1, 1, 1, false);
        const Vec driven = invokers_[1];
        const Vec startUp = unit(start), drivenUp = unit(driven);
        const double arc = length(start) * std::atan2(length(cross(startUp, drivenUp)), dot(startUp, drivenUp));
        if (std::abs(arc - invokerSpeed) > 0.01 ||
            std::abs(planet_.sdf(driven)) > 1e-6)
            throw std::runtime_error("Invoker movement did not follow the ground at the expected diagonal speed");
        tick();
        if (length(invokers_[1] - driven) != 0 || length(invokers_.front() - first) == 0)
            throw std::runtime_error("Automatic routing did not respect the currently controlled Invoker");

        updateTerrain(true);
        if (!world_->hasFinestLod(driven)) throw std::runtime_error("Controlled Invoker did not drive terrain LOD");
        buildObjects();
        renderer_->draw(camera(), objects_, opts_.captures / "invoker-control.bmp");

        if (!key(VK_ESCAPE, true) || controlling_ || !returning_[1] || length(invokers_[1] - driven) != 0)
            throw std::runtime_error("Detaching did not start the Invoker's return from its current position");
        for (int i = 0; i < 120 && returning_[1]; ++i) tick();
        const Vec route = planet_.surface(planet_.invokerDirection(1, simulationTime_ * 2.0));
        if (returning_[1] || length(invokers_[1] - route) != 0)
            throw std::runtime_error("Detached Invoker did not rejoin its route");
        selectCaptureItem(CameraTarget, 3);
        selectCaptureItem(InvokerCount, 0);
        if (controlling_ || selected_ != 0 || invokers_.size() != 1 || shapes_.size() != bodyCount ||
            SendMessageW(combo(CameraTarget), CB_GETCURSEL, 0, 0) != 0)
            throw std::runtime_error("Removing the controlled Invoker did not safely detach the camera");
    }

    void action(int command) {
        if (command >= Mesh && command <= Vdb) {
            if (method_ != command - Mesh) {
                method_ = command - Mesh;
                rebuild();
            }
        } else if (command == Density) {
            const auto selected = SendMessageW(combo(Density), CB_GETCURSEL, 0, 0);
            if (selected >= 0 && selected < LRESULT(voxelDensities.size())) {
                opts_.voxelDensity = voxelDensities[std::size_t(selected)];
                world_->setVoxelDensity(opts_.voxelDensity);
            }
        } else if (command == InvokerCount) {
            const auto selected = SendMessageW(combo(InvokerCount), CB_GETCURSEL, 0, 0);
            if (selected >= 0 && selected < maxInvokers) setInvokerCount(int(selected + 1));
        } else if (command == CameraTarget) {
            const auto selected = SendMessageW(combo(CameraTarget), CB_GETCURSEL, 0, 0);
            if (selected >= 0) selectCameraTarget(std::size_t(selected));
        } else if (command == TerrainStyle) {
            const auto selected = SendMessageW(combo(TerrainStyle), CB_GETCURSEL, 0, 0);
            if (selected >= 0 && selected < LRESULT(terrainPresets.size())) {
                const auto preset = terrainPresets[std::size_t(selected)];
                if (preset != opts_.terrain) {
                    Planet next(opts_.seed, preset);
                    // The old streamer holds a reference to this planet. Join
                    // its work before installing the new procedural field.
                    world_.reset();
                    planet_ = std::move(next);
                    opts_.terrain = preset;
                    const auto defaults = defaultRenderSettings(preset);
                    for (const auto field : terrainRenderFields) opts_.render.*field = defaults.*field;
                    freeView_ = false;
                    rebuild();
                }
            }
        } else if (command == RangeSettings) {
            if (editLodRanges(window_, lod0Radius_, voxelDensityShift(opts_.voxelDensity))) {
                setLod0Radius(lod0Radius_);
            }
        } else if (command == GraphicsSettings) {
            auto next = opts_.render;
            if (editRenderSettings(window_, next, defaultRenderSettings(opts_.terrain))) {
                try {
                    renderer_->setShadowResolution(next.shadowResolution);
                    renderer_->resetTemporalHistory();
                    opts_.render = next;
                } catch (const std::exception& error) {
                    const auto message = std::string("Could not apply the requested shadow-map size.\n") + error.what();
                    MessageBoxA(window_, message.c_str(), "Render settings", MB_OK | MB_ICONERROR);
                }
            }
        } else if (command == View) {
            freeView_ = false;
            detachInvoker();
            surface_ = !surface_;
        } else if (command == FreeView) {
            if (!freeView_) {
                const Camera current = camera();
                detachInvoker();
                flight_.reset(current.eye, current.forward, current.up);
                flightSpeed_ = std::clamp(std::abs(length(current.eye) - planetRadius) * 0.05, 50.0, 1000000.0);
            }
            freeView_ = !freeView_;
        } else if (command == Tint) lodTint_ = !lodTint_;
        else if (command == Wireframe) wireframe_ = !wireframe_;
        else if (command == Atmosphere) opts_.render.atmosphereEnabled = !opts_.render.atmosphereEnabled;
        else if (command == Clouds) opts_.render.cloudsEnabled = !opts_.render.cloudsEnabled;
        else if (command == Sweep) { sweepMode_ = !sweepMode_; rebuild(); }
        else if (command == Shadows) opts_.render.shadowsEnabled = !opts_.render.shadowsEnabled;
        else if (command == Ranges) {
            showRanges_ = !showRanges_;
            refreshRanges();
        }
        else if (command == Pause) paused_ = !paused_;
        else if (command == Drop) {
            if (!sweepMode_) {
                freeView_ = false;
                surface_ = true;
                spawn(int(selected_));
            }
        } else if (command == Reset) {
            freeView_ = false;
            selected_ = 0;
            yaw_ = 0.13;
            pitch_ = 0.16;
            orbitDistance_ = 2.5 * planetRadius;
            surfaceDistance_ = 1950;
            followDistance_ = 175;
            bearing_ = 0.42;
            elevation_ = 0.90;
            paused_ = false;
            rebuild();
        }
        syncCameraTarget();
        repaintButtons();
        updateFooter();
        SetFocus(viewport_);
    }

    bool key(WPARAM code, bool first) {
        if (code == VK_ESCAPE) {
            if (first) {
                if (controlling_) action(View);
                else PostMessageW(window_, WM_CLOSE, 0, 0);
            }
            return true;
        }
        if ((freeView_ || controlling_) && (code == 'W' || code == 'A' || code == 'S' || code == 'D' || code == VK_SHIFT)) return true;
        if (freeView_ && (code == 'Q' || code == 'E')) return true;
        if (!first) return false;
        if (sweepMode_) {
            if (code == VK_LEFT) sweep_.prevCast();
            else if (code == VK_RIGHT) sweep_.nextCast();
            else if (code == 'N') sweep_.jumpFrame();
            else if (code == 'C') sweep_.nextMismatch();
            else if (code == 'E') sweep_.nextWorstError();
            else if (code == 'O') sweep_.nextWorstPoint();
            else if (code == 'V') sweep_.revalidate();
            else if (code == 'X') action(Sweep);
            else return false;
            updateFooter();
            return true;
        }
        if (code >= '1' && code <= '4') action(int(code - '1' + Mesh));
        else if (code == 'F') action(View);
        else if (code == 'L') action(Tint);
        else if (code == 'K') action(Ranges);
        else if (code == VK_F2) action(Wireframe);
        else if (code == 'H') action(Atmosphere);
        else if (code == 'V') action(Clouds);
        else if (code == 'J') action(Shadows);
        else if (code == VK_OEM_4 || code == VK_OEM_6) {
            opts_.render.exposureEV = std::clamp(opts_.render.exposureEV + (code == VK_OEM_6 ? 0.5 : -0.5),
                                                hdrParameters.front().minimum, hdrParameters.front().maximum);
            updateFooter();
        }
        else if (code == 'C') action(FreeView);
        else if (code == 'B') action(Drop);
        else if (code == 'R') action(Reset);
        else if (code == VK_SPACE) action(Pause);
        else if (code == VK_TAB) {
            SendMessageW(combo(CameraTarget), CB_SETCURSEL, (selected_ + 1) % invokers_.size() + 1, 0);
            action(CameraTarget);
        } else return false;
        return true;
    }

    void arrange() {
        if (!viewport_) return;
        RECT client{};
        GetClientRect(window_, &client);
        int x = 12, y = 9;
        for (int i = 0; i < int(buttons_.size()); ++i) {
            const int width = toolbarButtons[i].width;
            if (x > 12 && x + width > client.right - 12) { x = 12; y += 40; }
            MoveWindow(buttons_[i], x, y, width, 34, TRUE);
            x += width + (i == 3 ? 21 : 6);
        }
        for (const auto& control : combos_) {
            if (!control.field) continue;
            const int width = control.labelWidth + control.fieldWidth;
            if (x > 12 && x + width > client.right - 12) { x = 12; y += 40; }
            MoveWindow(control.label, x, y + 8, control.labelWidth, 24, TRUE);
            MoveWindow(control.field, x + control.labelWidth, y + 4, control.fieldWidth, 220, TRUE);
            x += width + 12;
        }
        const int toolbarHeight = y + 43;
        const int height = std::max(0L, client.bottom - toolbarHeight - footerHeight);
        MoveWindow(viewport_, 0, toolbarHeight, client.right, height, TRUE);
        InvalidateRect(window_, nullptr, TRUE);
    }

    void repaintButtons() {
        SetWindowTextW(buttons_[View - 1], surface_ ? L"Orbit [F]" : L"Surface [F]");
        SetWindowTextW(buttons_[Pause - 1], paused_ ? L"Resume" : L"Pause");
        for (auto button : buttons_) InvalidateRect(button, nullptr, TRUE);
    }

    void updateFooter() {
        updateCameraInfo(camera());
        const auto counts = world_->lodCounts();
        int coarse = 0;
        for (int lod = 4; lod < lodCount; ++lod) coarse += counts[lod];
        std::wostringstream info;
        // Keep the applied/requested range and resident counts visible even
        // when the remaining diagnostic text is ellipsized on narrow windows.
        info << L"LOD blocks 0/1/2/3/4+  " << counts[0] << L" / " << counts[1] << L" / " << counts[2]
             << L" / " << counts[3] << L" / " << coarse
             << L"   |   LOD0 radius " << world_->lod0Radius();
        if (world_->lod0Radius() != lod0Radius_) info << L" -> " << lod0Radius_ << L" m (building)";
        else info << L" m";
        info << L"   |   " << methodLabels[method_] << L"   |   " << terrainLabels[std::size_t(opts_.terrain)];
        info << L"   |   Step " << world_->voxelDensity() << L"x ("
             << chunkSize / fineCells * world_->voxelDensity() << L" m)";
        if (world_->voxelDensity() != opts_.voxelDensity) info << L" -> " << opts_.voxelDensity << L"x building";
        if (freeView_) info << L"   |   Free " << std::fixed << std::setprecision(0) << flightSpeed_ << L" m/s";
        info << L"   |   Invoker " << selected_ + 1 << L"/" << opts_.invokers << L" + camera"
             << (controlling_ ? L" (controlled)" : L"")
             << (world_->hasFinestLod(invokers_[selected_]) ? L"" : L" (LOD0 pending)")
             << L"   |   " << triangles_ << L" triangles   |   " << shapes_.size() << L" bodies   |   "
             << std::fixed << std::setprecision(1) << streamMs_ << L" ms LOD update (" << streamBuilt_ << L" built)";
        if (fps_ > 0) info << L"   |   " << std::setprecision(0) << fps_ << L" FPS";
        if (paused_) info << L"   |   Paused";
        if (renderer_->isWarp()) info << L"   |   WARP";
        status_ = info.str();
        const double spacing = chunkSize / fineCells * world_->voxelDensity();
        std::wostringstream legend;
        legend << L"LOD 0 Cyan " << spacing << L" m / 1 Green " << spacing * 2
               << L" m / 2 Gold " << spacing * 4 << L" m / 3 Violet " << spacing * 8
               << L" m / 4+ Red to Blue, x2 per LOD";
        lodLegend_ = legend.str();

        constexpr double mib = 1024.0 * 1024.0;
        PROCESS_MEMORY_COUNTERS_EX process{};
        process.cb = sizeof(process);
        std::wostringstream processInfo;
        processInfo << std::fixed << std::setprecision(1) << L"Process MiB: ";
        if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&process),
                                 DWORD(sizeof(process)))) {
            processInfo << L"RAM " << process.WorkingSetSize / mib
                        << L"   |   Private commit " << process.PrivateUsage / mib
                        << L"   |   Peak RAM " << process.PeakWorkingSetSize / mib;
        } else {
            processInfo << L"N/A";
        }
        processInfo << L"   |   HDR EV " << opts_.render.exposureEV << L"   |   CSM ";
        if (opts_.render.shadowsEnabled) processInfo << opts_.render.shadowResolution << L" x " << opts_.render.shadowResolution << L" [J]";
        else processInfo << L"Off [J]";
        processMemory_ = processInfo.str();

        // These are resident terrain allocations, not an additional process total.
        // Background builds, dynamic bodies, renderer resources and allocator overhead
        // are represented by the process counters above, not this estimate.
        const auto memory = world_->terrainMemory();
        std::wostringstream terrainInfo;
        terrainInfo << std::fixed << std::setprecision(1)
                    << L"Terrain est. MiB: Total " << memory.total() / mib
                    << L"   |   Fields " << memory.fields / mib
                    << L"   |   Meshes " << memory.meshes / mib
                    << L"   |   Collision " << memory.physics / mib
                    << L"   |   Metadata " << memory.metadata / mib;
        terrainMemory_ = terrainInfo.str();
        if (sweepMode_) {
            status_ = sweep_.statusLine(methodLabels, triangles_);
            processMemory_ = sweep_.castLine();
            terrainMemory_ = sweep_.totalsLine();
        }
        RECT footer{};
        GetClientRect(window_, &footer);
        footer.top = std::max(0L, footer.bottom - footerHeight);
        InvalidateRect(window_, &footer, FALSE);
    }

    RECT cameraPanelRect() const {
        RECT client{};
        GetClientRect(window_, &client);
        return {std::max(13L, client.right - cameraPanelWidth), client.bottom - cameraPanelHeight - 6,
                client.right - 12, client.bottom - 6};
    }

    void updateCameraInfo(const Camera& view) {
        const auto mode = freeView_ ? std::wstring(L"Free") : controlling_
            ? L"Invoker " + std::to_wstring(selected_ + 1) : surface_ ? std::wstring(L"Surface") : std::wstring(L"Orbit");
        const auto title = L"Camera world (m) | " + mode;
        const std::array<double, 4> values{view.eye.x, view.eye.y, view.eye.z,
            length(view.eye - view.planetCenter) - planetRadius};
        std::array<std::wstring, 4> text;
        for (std::size_t i = 0; i < values.size(); ++i) {
            std::wostringstream value;
            value << std::fixed << std::setprecision(3) << values[i];
            text[i] = value.str();
        }
        if (cameraTitle_ == title && cameraValues_ == text) return;
        cameraTitle_ = title;
        cameraValues_ = std::move(text);
        const auto panel = cameraPanelRect();
        InvalidateRect(window_, &panel, FALSE);
    }

    void paint() const {
        PAINTSTRUCT paintInfo{};
        HDC dc = BeginPaint(window_, &paintInfo);
        RECT client{};
        GetClientRect(window_, &client);
        FillRect(dc, &paintInfo.rcPaint, background_);
        SelectObject(dc, font_);
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, RGB(220, 231, 243));
        const auto panel = cameraPanelRect();
        RECT row{13, client.bottom - footerHeight + 9, client.right - 10, client.bottom - footerHeight + 30};
        DrawTextW(dc, status_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        row.right = panel.left - 16;
        row.top += 22;
        row.bottom += 22;
        DrawTextW(dc, processMemory_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        row.top += 22;
        row.bottom += 22;
        DrawTextW(dc, terrainMemory_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        SetTextColor(dc, RGB(152, 170, 193));
        row.top += 22;
        row.bottom += 22;
        std::wstring sweepHint;
        const wchar_t* controls;
        if (sweepMode_) {
            sweepHint = sweep_.controlsHint();
            controls = sweepHint.c_str();
        } else if (freeView_)
            controls = L"Free [C]: WASD move / Q-E down-up / Drag to look / Shift x5 / Wheel speed / F2 wireframe / H atmosphere";
        else if (controlling_)
            controls = L"WASD move / Shift x3 / Drag to orbit / Wheel zoom / Tab next Invoker / Esc detach / C free camera";
        else if (lodTint_ || showRanges_)
            controls = lodLegend_.c_str();
        else
            controls = L"Drag to orbit / Wheel zoom / Tab Invoker / Space pause / B drop / J shadows / [ ] exposure -/+0.5 EV";
        row.bottom = client.bottom - 6;
        DrawTextW(dc, controls, -1, &row, DT_WORDBREAK | DT_NOPREFIX);
        RECT cameraRow{panel.left, panel.top, panel.right, panel.top + 18};
        SetTextColor(dc, RGB(117, 211, 231));
        DrawTextW(dc, cameraTitle_.c_str(), -1, &cameraRow, DT_SINGLELINE | DT_VCENTER | DT_NOPREFIX);
        constexpr const wchar_t* labels[]{L"X", L"Y", L"Z", L"Alt (sphere)"};
        for (std::size_t i = 0; i < cameraValues_.size(); ++i) {
            cameraRow.top += 18; cameraRow.bottom += 18;
            SetTextColor(dc, RGB(152, 170, 193));
            DrawTextW(dc, labels[i], -1, &cameraRow, DT_SINGLELINE | DT_VCENTER | DT_NOPREFIX);
            RECT valueRow = cameraRow;
            valueRow.left += 98;
            SetTextColor(dc, RGB(220, 231, 243));
            DrawTextW(dc, cameraValues_[i].c_str(), -1, &valueRow, DT_SINGLELINE | DT_VCENTER | DT_RIGHT | DT_NOPREFIX);
        }
        EndPaint(window_, &paintInfo);
    }

    void paintButton(const DRAWITEMSTRUCT& item) const {
        const int button = int(item.CtlID);
        const bool active = button == method_ + Mesh || (button == View && surface_) ||
                            (button == Tint && lodTint_) || (button == Ranges && showRanges_) ||
                            (button == Wireframe && wireframe_) || (button == Pause && paused_) ||
                            (button == Atmosphere && opts_.render.atmosphereEnabled) || (button == FreeView && freeView_) ||
                            (button == Shadows && opts_.render.shadowsEnabled) || (button == Clouds && opts_.render.cloudsEnabled) ||
                            (button == Sweep && sweepMode_);
        const bool pressed = (item.itemState & ODS_SELECTED) != 0;
        const HBRUSH fill = CreateSolidBrush(active ? RGB(34, 72, 88) : pressed ? RGB(52, 65, 84) : RGB(31, 42, 58));
        FillRect(item.hDC, &item.rcItem, fill);
        DeleteObject(fill);
        RECT edge = item.rcItem;
        const HBRUSH border = CreateSolidBrush(active ? RGB(85, 200, 222) : RGB(52, 65, 84));
        FrameRect(item.hDC, &edge, border);
        DeleteObject(border);
        SetBkMode(item.hDC, TRANSPARENT);
        SetTextColor(item.hDC, active ? RGB(190, 248, 255) : RGB(213, 224, 239));
        SelectObject(item.hDC, font_);
        wchar_t title[64]{};
        GetWindowTextW(item.hwndItem, title, 64);
        DrawTextW(item.hDC, title, -1, &edge, DT_VCENTER | DT_CENTER | DT_SINGLELINE | DT_NOPREFIX);
        if ((item.itemState & ODS_FOCUS) != 0) {
            InflateRect(&edge, -3, -3);
            DrawFocusRect(item.hDC, &edge);
        }
    }

    static App* attach(HWND window, UINT message, LPARAM parameter) {
        if (message == WM_NCCREATE) {
            auto* instance = static_cast<App*>(reinterpret_cast<CREATESTRUCTW*>(parameter)->lpCreateParams);
            SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(instance));
            return instance;
        }
        return reinterpret_cast<App*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    }

    static LRESULT CALLBACK windowProcedure(HWND window, UINT message, WPARAM w, LPARAM l) {
        auto* app = attach(window, message, l);
        if (!app) return DefWindowProcW(window, message, w, l);
        switch (message) {
            case WM_GETMINMAXINFO: {
                auto* limits = reinterpret_cast<MINMAXINFO*>(l);
                limits->ptMinTrackSize = {1080, 560};
                return 0;
            }
            case WM_COMMAND: {
                const int command = LOWORD(w), notification = HIWORD(w);
                if (command >= Density && command <= TerrainStyle) {
                    if (notification == CBN_SELENDOK || (notification == CBN_SELCHANGE &&
                        !SendMessageW(app->combo(command), CB_GETDROPPEDSTATE, 0, 0))) app->pending_ = command;
                } else if (command >= Mesh && command <= GraphicsSettings && notification == BN_CLICKED) app->pending_ = command;
                return 0;
            }
            case WM_CTLCOLORSTATIC:
                SetTextColor(reinterpret_cast<HDC>(w), RGB(220, 231, 243));
                SetBkMode(reinterpret_cast<HDC>(w), TRANSPARENT);
                return reinterpret_cast<LRESULT>(app->background_);
            case WM_DRAWITEM: app->paintButton(*reinterpret_cast<DRAWITEMSTRUCT*>(l)); return TRUE;
            case WM_PAINT:
                if (!app->window_) return DefWindowProcW(window, message, w, l);
                app->paint();
                return 0;
            case WM_SIZE: app->arrange(); return 0;
            case WM_CLOSE: app->closed_ = true; return 0;
            case WM_DESTROY: PostQuitMessage(0); return 0;
            default: return DefWindowProcW(window, message, w, l);
        }
    }

    static LRESULT CALLBACK viewportProcedure(HWND window, UINT message, WPARAM w, LPARAM l) {
        auto* app = attach(window, message, l);
        if (!app) return DefWindowProcW(window, message, w, l);
        switch (message) {
            case WM_ERASEBKGND: return 1;
            case WM_LBUTTONDOWN:
            case WM_RBUTTONDOWN:
                if (message == WM_RBUTTONDOWN && !app->freeView_) break;
                SetCapture(window);
                SetFocus(window);
                app->dragging_ = true;
                app->mouse_ = {GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                return 0;
            case WM_MOUSEMOVE:
                if (app->dragging_) {
                    const POINT next{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                    const double dx = (next.x - app->mouse_.x) * 0.006, dy = (next.y - app->mouse_.y) * 0.006;
                    if (app->sweepMode_) {
                        app->sweepYaw_ = std::remainder(app->sweepYaw_ - dx, 2 * pi);
                        app->sweepPitch_ = std::clamp(app->sweepPitch_ + dy, -1.35, 1.35);
                    } else if (app->freeView_) {
                        app->flight_.look(dx, -dy);
                    } else if (app->surface_) {
                        app->bearing_ = std::remainder(app->bearing_ - dx, 2 * pi);
                        app->elevation_ = std::clamp(app->elevation_ + dy, 0.24, 1.46);
                    } else {
                        app->yaw_ = std::remainder(app->yaw_ - dx, 2 * pi);
                        app->pitch_ = std::clamp(app->pitch_ + dy, -1.35, 1.35);
                    }
                    app->mouse_ = next;
                }
                return 0;
            case WM_LBUTTONUP:
            case WM_RBUTTONUP: if (GetCapture() == window) ReleaseCapture(); return 0;
            case WM_CAPTURECHANGED: app->dragging_ = false; return 0;
            case WM_MOUSEWHEEL: {
                const double factor = std::pow(0.84, double(GET_WHEEL_DELTA_WPARAM(w)) / WHEEL_DELTA);
                if (app->sweepMode_) {
                    const double base = app->sweepDistance_ > 0 ? app->sweepDistance_ : app->sweep_.focusDistance();
                    app->sweepDistance_ = std::clamp(base * factor, 60.0, 4000.0);
                }
                else if (app->freeView_) app->flightSpeed_ = std::clamp(app->flightSpeed_ / factor, 1.0, 10000000.0);
                else if (app->controlling_) app->followDistance_ = std::clamp(app->followDistance_ * factor, 30.0, 10500.0);
                else if (app->surface_) app->surfaceDistance_ = std::clamp(app->surfaceDistance_ * factor, 110.0, 10500.0);
                else app->orbitDistance_ = std::clamp(app->orbitDistance_ * factor, 1.02 * planetRadius, 8 * planetRadius);
                if (app->freeView_) app->updateFooter();
                return 0;
            }
            default: return DefWindowProcW(window, message, w, l);
        }
        return DefWindowProcW(window, message, w, l);
    }

    Options opts_;
    Planet planet_;
    SweepInspector sweep_{planet_, opts_.sweep, opts_.seed};
    bool sweepMode_{opts_.sweep.enabled};
    int method_, pending_{};
    HWND window_{}, viewport_{};
    std::array<HWND, toolbarButtons.size()> buttons_{};
    std::array<ToolbarCombo, TerrainStyle - Density + 1> combos_{};
    HFONT font_{};
    HBRUSH background_{};
    std::unique_ptr<Dx12Renderer> renderer_;
    std::unique_ptr<World> world_;
    std::vector<Vec> invokers_;
    std::vector<Vec> clipmapPositions_;
    ClipmapForecast clipmapForecast_;
    std::array<bool, maxInvokers> returning_{};
    std::uint32_t lod0Radius_{defaultLod0Radius};
    std::vector<VisibleBody> shapes_;
    std::vector<Vertex> objects_;
    std::map<Key, TerrainPage> terrainPages_;
    std::vector<PreparedTerrainPage> preparedTerrain_;
    std::size_t nextTerrainUpload_{};
    bool fullTerrainBuild_{};
    std::size_t selected_{}, triangles_{};
    std::wstring status_, processMemory_, terrainMemory_, lodLegend_;
    std::wstring cameraTitle_;
    std::array<std::wstring, 4> cameraValues_;
    bool closed_{}, dragging_{}, surface_{}, lodTint_{}, showRanges_{}, wireframe_{}, paused_{};
    bool freeView_{}, controlling_{};
    FreeCamera flight_;
    double flightSpeed_{200};
    POINT mouse_{};
    double simulationTime_{}, streamMs_{}, fps_{};
    int streamBuilt_{};
    double yaw_{0.13}, pitch_{0.16}, orbitDistance_{2.5 * planetRadius}, bearing_{0.42}, elevation_{0.90}, surfaceDistance_{1950};
    double followDistance_{175};
    double sweepYaw_{}, sweepPitch_{0.2}, sweepDistance_{};
};

}  // namespace
}  // namespace voxel

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show) {
    bool capture = false;
    std::filesystem::path errorPath;
    try {
        SetProcessDPIAware();
        auto options = voxel::readOptions();
        capture = !options.captures.empty();
        if (capture) {
            std::filesystem::create_directories(options.captures);
            errorPath = options.captures / "error.txt";
        }
        voxel::App app(std::move(options));
        app.create(instance, show == SW_HIDE ? SW_SHOWNORMAL : show);
        return app.run();
    } catch (const std::exception& e) {
        if (capture && !errorPath.empty()) std::ofstream(errorPath) << e.what() << '\n';
        else MessageBoxA(nullptr, e.what(), "VoxelWorld Viewer", MB_OK | MB_ICONERROR);
        return 1;
    }
}
