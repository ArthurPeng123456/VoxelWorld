#include "dx12_renderer.hpp"
#include "lod_range_dialog.hpp"
#include "render_settings_dialog.hpp"
#include "world.hpp"
#include "free_camera.hpp"

#include <shellapi.h>
#include <psapi.h>
#include <windowsx.h>

#include <fstream>
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
constexpr int footerHeight = 102;
constexpr double pi = 3.14159265358979323846;
constexpr double dt = 1.0 / 60.0;
constexpr int maxInvokers = 16;
constexpr double invokerSpeed = 240.0;
constexpr double invokerReturnSpeed = 720.0;
constexpr std::array<Method, 4> methods{Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
constexpr std::array<const wchar_t*, 4> methodLabels{L"Mesh BVH", L"VoxelField", L"HashDAG", L"OpenVDB"};

Key terrainPage(Key chunk) {
    // Group adjacent chunks of the same scale to limit both uploads and draw calls.
    const int level = chunk.level + 3;
    const int span = 1 << level;
    const auto align = [span](int coordinate) { return coordinate - (coordinate % span + span) % span; };
    return {align(chunk.x), align(chunk.y), align(chunk.z), level, chunk.densityShift};
}

enum Command { Mesh = 1, Grid, Dag, Vdb, View, Tint, Pause, Drop, Reset, Ranges, Wireframe, Atmosphere, FreeView, Shadows,
               RangeSettings, GraphicsSettings, Density = 100, InvokerCount, CameraTarget };

struct ToolbarButton { const wchar_t* label; int width; };
constexpr std::array<ToolbarButton, GraphicsSettings> toolbarButtons{{
    {L"1  Mesh BVH", 100}, {L"2  VoxelField", 100}, {L"3  HashDAG", 100}, {L"4  OpenVDB", 100},
    {L"Surface [F]", 104}, {L"LOD [L]", 72}, {L"Pause", 68}, {L"Drop [B]", 76}, {L"Reset", 56},
    {L"Ranges [K]", 84}, {L"Wireframe [F2]", 116}, {L"Atmosphere [H]", 128}, {L"Free [C]", 92}, {L"Shadows [J]", 108},
    {L"LOD ranges...", 112}, {L"Render settings...", 136}
}};

struct Options {
    int method{1};
    std::uint64_t seed{42};
    int invokers{2};
    bool warp{}, morton{};
    double voxelDensity{1};
    RenderSettings render;
    std::filesystem::path captures;
};

Options readOptions() {
    int count = 0;
    const std::unique_ptr<wchar_t*, decltype(&LocalFree)> args(CommandLineToArgvW(GetCommandLineW(), &count), LocalFree);
    if (!args) throw std::runtime_error("Could not read the command line");
    Options result;
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
        const auto renderOption = [&](const auto& parameters) {
            for (const auto& parameter : parameters) if (option == parameter.option) {
                const auto value = readRenderParameter(next(), parameter);
                if (!value) {
                    std::string flag(option.size(), ' ');
                    std::transform(option.begin(), option.end(), flag.begin(), [](wchar_t c) { return static_cast<char>(c); });
                    throw std::invalid_argument(flag + " requires a finite value within the supported range");
                }
                result.render.*parameter.value = *value;
                return true;
            }
            return false;
        };
        if (option == L"--warp") result.warp = true;
        else if (option == L"--no-shadows") result.render.shadowsEnabled = false;
        else if (option == L"--no-atmosphere") result.render.atmosphereEnabled = false;
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
        else if (option == L"--voxel-density" || option == L"--density") result.voxelDensity = parseVoxelDensity(next());
        else if (option == L"--seed") result.seed = integer();
        else if (option == L"--invokers") {
            const auto n = integer();
            if (n < 1 || n > maxInvokers) throw std::invalid_argument("Invoker count must be from 1 to 16");
            result.invokers = int(n);
        } else if (option == L"--scheme") {
            const auto value = next();
            if (value == L"mesh" || value == L"1") result.method = 0;
            else if (value == L"grid" || value == L"2") result.method = 1;
            else if (value == L"dag" || value == L"3") result.method = 2;
            else if (value == L"vdb" || value == L"4") result.method = 3;
            else throw std::invalid_argument("Choose mesh, grid, dag or vdb");
        } else if (!renderOption(shadowParameters) && !renderOption(atmosphereParameters) && !renderOption(hdrParameters))
            throw std::invalid_argument("Unknown viewer option");
    }
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
    explicit App(Options options) : opts_(std::move(options)), planet_(opts_.seed), method_(opts_.method) {}
    ~App() {
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
        if (!opts_.captures.empty()) return captureAll();
        MSG message{};
        auto previous = Clock::now();
        double accumulator = 0, statsElapsed = 0, streamElapsed = 0;
        while (!closed_) {
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
            moveControlled(elapsed);
            if (!paused_) {
                accumulator += elapsed;
                while (accumulator >= dt) {
                    tick();
                    accumulator -= dt;
                }
            }
            // The camera keeps moving while simulation is paused.
            streamElapsed += elapsed;
            if (const auto published = world_->pollUpdate()) applyTerrain(*published, false);
            if (streamElapsed >= dt && world_->requestUpdate(invokers_, true, camera().eye, true)) {
                streamElapsed = 0;
            }
            fps_ = fps_ * 0.94 + 0.06 / std::max(0.001, elapsed);
            statsElapsed += elapsed;
            if (statsElapsed >= 0.25) {
                updateFooter();
                statsElapsed = 0;
            }
            buildObjects();
            renderer_->draw(camera(), objects_);
        }
        return 0;
    }

private:
    struct VisibleBody { std::size_t index; Shape shape; std::uint32_t material; };
    struct TerrainPage { std::set<Key> chunks; std::size_t triangles{}; };
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
        simulationTime_ = 0;
        shapes_.clear();
        controlling_ = false;
        returning_.fill(false);
        invokers_ = invokerPositions(planet_, opts_.invokers, 0);
        refreshCameraTargets();
        world_ = std::make_unique<World>(planet_, methods[method_], opts_.morton, opts_.voxelDensity);
        world_->setLod0Radius(lod0Radius_);
        updateTerrain(true);
        for (int i = 0; i < opts_.invokers; ++i) spawn(i);
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

    void updateTerrain(bool force) {
        applyTerrain(world_->update(invokers_, true, camera().eye), force);
    }

    void applyTerrain(const WorldStats& stats, bool force) {
        if (force || stats.windowVisits != 0 || stats.build.built != 0 || stats.build.removed != 0) {
            streamMs_ = stats.updateMs;
            streamBuilt_ = stats.build.built;
        }
        if (showRanges_ && (force || stats.windowsChanged)) refreshRanges();
        std::set<Key> dirtyPages;
        if (force) {
            renderer_->clearTerrain();
            terrainPages_.clear();
            triangles_ = 0;
            for (const auto& entry : world_->chunks) if (!entry.second->mesh.empty()) {
                const auto page = terrainPage(entry.first);
                terrainPages_[page].chunks.insert(entry.first);
                dirtyPages.insert(page);
            }
        } else {
            for (Key key : stats.meshChanges) {
                const Key page = terrainPage(key);
                const auto chunk = world_->chunks.find(key);
                auto& keys = terrainPages_[page].chunks;
                if (chunk != world_->chunks.end() && !chunk->second->mesh.empty()) keys.insert(key);
                else keys.erase(key);
                dirtyPages.insert(page);
            }
        }
        std::vector<Vertex> geometry;
        for (const Key page : dirtyPages) {
            auto& data = terrainPages_.at(page);
            triangles_ -= data.triangles;
            data.triangles = 0;
            for (Key key : data.chunks) data.triangles += world_->chunks.at(key)->mesh.size();
            const double infinity = std::numeric_limits<double>::infinity();
            Box pageBounds{splat(infinity), splat(-infinity)};
            geometry.clear();
            geometry.reserve(data.triangles * 3);
            for (Key key : data.chunks) {
                const Chunk& chunk = *world_->chunks.at(key);
                // Fade relief that this mesh's vertex spacing cannot resolve.
                const double t = std::clamp((chunk.spacing - 400.0) / 1200.0, 0.0, 1.0);
                const double reliefDetail = 1 - t * t * (3 - 2 * t);
                for (const auto& triangle : chunk.mesh) {
                    for (const Vec point : triangle.vertices) {
                        pageBounds.lo = min(pageBounds.lo, point);
                        pageBounds.hi = max(pageBounds.hi, point);
                        const Vec up = unit(point);
                        Vec normal = up;
                        double elevation = 0;
                        if (reliefDetail > 0) {
                            const auto sample = planet_.sampleSurface(up);
                            normal = unit(up + (sample.normal - up) * reliefDetail, up);
                            elevation = sample.elevation * reliefDetail;
                        }
                        const double height = std::clamp((elevation + 660) / 850, 0.0, 1.0);
                        const double rock = std::clamp(2.4 * (1 - dot(normal, up)), 0.0, 1.0);
                        geometry.push_back(vertex(point, normal, color({height, rock, 0}, chunk.lod)));
                    }
                }
            }
            triangles_ += data.triangles;
            renderer_->setTerrainPage(page, pageBounds, geometry);
            if (data.chunks.empty()) terrainPages_.erase(page);
        }
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
        renderer_->setClipmapLines(lines);
    }

    void buildObjects() {
        objects_.clear();
        if (surface_ || freeView_) {
            const Vec focus = freeView_ ? flight_.position() : invokers_[selected_];
            for (const auto& body : shapes_) {
                const auto pose = world_->physics.bodyPose(body.index);
                if (!freeView_ && length(pose.position - focus) > (controlling_ ? followDistance_ : surfaceDistance_) * 2.7) continue;
                Shape shape = body.shape;
                shape.axes = pose.axes;
                if (shape.kind == ShapeKind::Box) appendBox(objects_, shape, pose.position, body.material);
                else appendRound(objects_, shape, pose.position, body.material);
            }
        }
        for (std::size_t i = 0; i < invokers_.size(); ++i) {
            appendBeacon(objects_, invokers_[i], surface_ || freeView_ ? 24 : 380, i == selected_);
        }
    }

    Camera camera() const {
        if (freeView_) {
            return {flight_.position(), flight_.right(), flight_.up(), flight_.forward(), 0.3, 20 * planetRadius,
                    lodTint_, wireframe_, {}, opts_.render};
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
                lodTint_, wireframe_, {}, opts_.render};
    }

    int captureAll() {
        std::filesystem::create_directories(opts_.captures);
        std::ofstream report(opts_.captures / "validation.txt");
        report.exceptions(std::ios::badbit | std::ios::failbit);
        report << "Direct3D 12 " << (renderer_->isWarp() ? "WARP" : "hardware") << '\n';
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
                orbitClear.settings = {};
                orbitClear.settings.atmosphereEnabled = false;
                orbitClear.settings.shadowsEnabled = false;
                renderer_->draw(orbitClear, objects_, opts_.captures / (prefix + "-orbit-no-effects.bmp"));
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

        validation.settings = {};
        for (int edge : shadowResolutions) {
            if (edge == RenderSettings{}.shadowResolution) continue;
            validation.settings.shadowResolution = edge;
            const auto name = "pipeline-shadow-" + std::to_string(edge) + ".bmp";
            renderer_->draw(validation, caster, opts_.captures / name, true);
        }
        validation.settings = {};
        validation.settings.toneMapping = ToneMapping::Reinhard;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-tone-reinhard.bmp");
        validation.settings.toneMapping = ToneMapping::Linear;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-tone-linear.bmp");
        validation.settings = {};
        validation.settings.atmosphereHeightKm = 18;
        validation.settings.rayleighHeightKm = 4;
        validation.settings.mieHeightKm = 0.6;
        validation.settings.rayleighStrength = 5;
        validation.settings.mieStrength = 3;
        validation.settings.mieAnisotropy = 0.55;
        renderer_->draw(validation, caster, opts_.captures / "pipeline-atmosphere-tuned.bmp");
        captureInvokerControls();
        report << "invoker_controls=passed\n";
        return 0;
    }

    void captureInvokerControls() {
        const auto selectItem = [&](int command, int item) {
            const HWND field = combo(command);
            if (SendMessageW(field, CB_SETCURSEL, WPARAM(item), 0) == CB_ERR)
                throw std::runtime_error("Unable to select an Invoker control item");
            SendMessageW(window_, WM_COMMAND, MAKEWPARAM(command, CBN_SELENDOK), reinterpret_cast<LPARAM>(field));
            if (std::exchange(pending_, 0) != command) throw std::runtime_error("Invoker dropdown did not dispatch its selection");
            action(command);
        };
        const auto bodyCount = shapes_.size();
        const Vec first = invokers_.front();
        selectItem(InvokerCount, 2);
        if (invokers_.size() != 3 || length(invokers_.front() - first) != 0 || shapes_.size() != bodyCount ||
            SendMessageW(combo(CameraTarget), CB_GETCOUNT, 0, 0) != 4)
            throw std::runtime_error("Changing Invoker count did not preserve the scene and update the camera list");

        selectItem(CameraTarget, 2);
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
        selectItem(CameraTarget, 3);
        selectItem(InvokerCount, 0);
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
        } else if (command == RangeSettings) {
            if (editLodRanges(window_, lod0Radius_, voxelDensityShift(opts_.voxelDensity))) world_->setLod0Radius(lod0Radius_);
        } else if (command == GraphicsSettings) {
            auto next = opts_.render;
            if (editRenderSettings(window_, next)) {
                try {
                    renderer_->setShadowResolution(next.shadowResolution);
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
        else if (command == Shadows) opts_.render.shadowsEnabled = !opts_.render.shadowsEnabled;
        else if (command == Ranges) {
            showRanges_ = !showRanges_;
            refreshRanges();
        }
        else if (command == Pause) paused_ = !paused_;
        else if (command == Drop) {
            freeView_ = false;
            surface_ = true;
            spawn(int(selected_));
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
        if (code >= '1' && code <= '4') action(int(code - '1' + Mesh));
        else if (code == 'F') action(View);
        else if (code == 'L') action(Tint);
        else if (code == 'K') action(Ranges);
        else if (code == VK_F2) action(Wireframe);
        else if (code == 'H') action(Atmosphere);
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
        const auto counts = world_->lodCounts();
        int coarse = 0;
        for (int lod = 4; lod < lodCount; ++lod) coarse += counts[lod];
        std::wostringstream info;
        info << methodLabels[method_];
        info << L"   |   Step " << world_->voxelDensity() << L"x ("
             << chunkSize / fineCells * world_->voxelDensity() << L" m)";
        if (world_->voxelDensity() != opts_.voxelDensity) info << L" -> " << opts_.voxelDensity << L"x building";
        if (freeView_) info << L"   |   Free " << std::fixed << std::setprecision(0) << flightSpeed_ << L" m/s";
        info << L"   |   Invoker " << selected_ + 1 << L"/" << opts_.invokers << L" + camera"
             << (controlling_ ? L" (controlled)" : L"")
             << (world_->hasFinestLod(invokers_[selected_]) ? L"" : L" (LOD0 pending)")
             << (world_->lod0Radius() == lod0Radius_ ? L"" : L"   |   Building new LOD ranges")
             << L"   |   LOD 0/1/2/3/4+  " << counts[0] << L" / " << counts[1] << L" / " << counts[2] << L" / " << counts[3] << L" / " << coarse
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
        RECT footer{};
        GetClientRect(window_, &footer);
        footer.top = std::max(0L, footer.bottom - footerHeight);
        InvalidateRect(window_, &footer, FALSE);
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
        RECT row{13, client.bottom - footerHeight + 9, client.right - 10, client.bottom - footerHeight + 30};
        DrawTextW(dc, status_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        row.top += 22;
        row.bottom += 22;
        DrawTextW(dc, processMemory_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        row.top += 22;
        row.bottom += 22;
        DrawTextW(dc, terrainMemory_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        SetTextColor(dc, RGB(152, 170, 193));
        row.top += 22;
        row.bottom += 22;
        const wchar_t* controls = freeView_
            ? L"Free [C]: WASD move / Q-E down-up / Drag to look / Shift x5 / Wheel speed / F2 wireframe / H atmosphere"
            : controlling_
            ? L"WASD move / Shift x3 / Drag to orbit / Wheel zoom / Tab next Invoker / Esc detach / C free camera"
            : lodTint_ || showRanges_
            ? lodLegend_.c_str()
            : L"Drag to orbit / Wheel zoom / Tab Invoker / Space pause / B drop / J shadows / [ ] exposure -/+0.5 EV";
        DrawTextW(dc, controls, -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS);
        EndPaint(window_, &paintInfo);
    }

    void paintButton(const DRAWITEMSTRUCT& item) const {
        const int button = int(item.CtlID);
        const bool active = button == method_ + Mesh || (button == View && surface_) ||
                            (button == Tint && lodTint_) || (button == Ranges && showRanges_) ||
                            (button == Wireframe && wireframe_) || (button == Pause && paused_) ||
                            (button == Atmosphere && opts_.render.atmosphereEnabled) || (button == FreeView && freeView_) ||
                            (button == Shadows && opts_.render.shadowsEnabled);
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
                if (command >= Density && command <= CameraTarget) {
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
                    if (app->freeView_) {
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
                if (app->freeView_) app->flightSpeed_ = std::clamp(app->flightSpeed_ / factor, 1.0, 10000000.0);
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
    int method_, pending_{};
    HWND window_{}, viewport_{};
    std::array<HWND, toolbarButtons.size()> buttons_{};
    std::array<ToolbarCombo, CameraTarget - Density + 1> combos_{};
    HFONT font_{};
    HBRUSH background_{};
    std::unique_ptr<Dx12Renderer> renderer_;
    std::unique_ptr<World> world_;
    std::vector<Vec> invokers_;
    std::array<bool, maxInvokers> returning_{};
    std::uint32_t lod0Radius_{defaultLod0Radius};
    std::vector<VisibleBody> shapes_;
    std::vector<Vertex> objects_;
    std::map<Key, TerrainPage> terrainPages_;
    std::size_t selected_{}, triangles_{};
    std::wstring status_, processMemory_, terrainMemory_, lodLegend_;
    bool closed_{}, dragging_{}, surface_{}, lodTint_{}, showRanges_{}, wireframe_{}, paused_{};
    bool freeView_{}, controlling_{};
    FreeCamera flight_;
    double flightSpeed_{200};
    POINT mouse_{};
    double simulationTime_{}, streamMs_{}, fps_{};
    int streamBuilt_{};
    double yaw_{0.13}, pitch_{0.16}, orbitDistance_{2.5 * planetRadius}, bearing_{0.42}, elevation_{0.90}, surfaceDistance_{1950};
    double followDistance_{175};
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
