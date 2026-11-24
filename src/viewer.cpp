#include "dx12_renderer.hpp"
#include "world.hpp"
#include "free_camera.hpp"

#include <shellapi.h>
#include <psapi.h>
#include <windowsx.h>

#include <fstream>
#include <iomanip>
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
constexpr std::array<Method, 4> methods{Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
constexpr std::array<const wchar_t*, 4> methodLabels{L"Mesh BVH", L"VoxelField", L"HashDAG", L"OpenVDB"};

enum Command { Mesh = 1, Grid, Dag, Vdb, View, Tint, Pause, Drop, Reset, Ranges, Wireframe, Atmosphere, FreeView };

struct ToolbarButton { const wchar_t* label; int width; };
constexpr std::array<ToolbarButton, FreeView> toolbarButtons{{
    {L"1  Mesh BVH", 100}, {L"2  VoxelField", 100}, {L"3  HashDAG", 100}, {L"4  OpenVDB", 100},
    {L"Surface [F]", 104}, {L"LOD [L]", 72}, {L"Pause", 68}, {L"Drop [B]", 76}, {L"Reset", 56},
    {L"Ranges [K]", 84}, {L"Wireframe [F2]", 116}, {L"Atmosphere [H]", 128}, {L"Free [C]", 92}
}};

struct Options {
    int method{1};
    std::uint64_t seed{42};
    int invokers{2};
    bool warp{}, morton{};
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
        if (option == L"--warp") result.warp = true;
        else if (option == L"--morton") result.morton = true;
        else if (option == L"--capture-dir") result.captures = next();
        else if (option == L"--seed") result.seed = integer();
        else if (option == L"--invokers") {
            const auto n = integer();
            if (n < 1 || n > 16) throw std::invalid_argument("Invoker count must be from 1 to 16");
            result.invokers = int(n);
        } else if (option == L"--scheme") {
            const auto value = next();
            if (value == L"mesh" || value == L"1") result.method = 0;
            else if (value == L"grid" || value == L"2") result.method = 1;
            else if (value == L"dag" || value == L"3") result.method = 2;
            else if (value == L"vdb" || value == L"4") result.method = 3;
            else throw std::invalid_argument("Choose mesh, grid, dag or vdb");
        } else throw std::invalid_argument("Unknown viewer option");
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
    const Vec center = surface + up * (1.8 * size);
    const Vec a = center + up * size, b = center - up * size;
    const std::array<Vec, 4> ring{center + axes[0] * (size * 0.55), center + axes[1] * (size * 0.55),
                                  center - axes[0] * (size * 0.55), center - axes[1] * (size * 0.55)};
    const auto material = color(selected ? Vec{0.32, 0.96, 1.0} : Vec{1.0, 0.61, 0.34}, 255);
    for (int i = 0; i < 4; ++i) {
        const Vec x = ring[i], y = ring[(i + 1) % 4];
        flatTriangle(batch, a, x, y, unit((x + y) * 0.5 - center), material);
        flatTriangle(batch, x, b, y, unit((x + y) * 0.5 - center), material);
    }
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
        arrange();
        ShowWindow(window_, show);
        UpdateWindow(window_);
        renderer_ = std::make_unique<Dx12Renderer>(viewport_, opts_.warp);
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
                if (message.message == WM_KEYDOWN && key(message.wParam, (message.lParam & (1L << 30)) == 0)) continue;
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
            moveFreeCamera(elapsed);
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
            if (streamElapsed >= 0.15 && world_->requestUpdate(invokers_, true, camera().eye)) {
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

    void rebuild() {
        simulationTime_ = 0;
        shapes_.clear();
        world_ = std::make_unique<World>(planet_, methods[method_], opts_.morton);
        refreshTerrain(true);
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
        invokers_ = invokerPositions(planet_, opts_.invokers, simulationTime_ * 2.0);
    }

    void moveFreeCamera(double elapsed) {
        // Poll current state so held keys move smoothly and focus changes cannot
        // leave a movement key stuck. Camera movement is independent of physics.
        if (!freeView_ || GetForegroundWindow() != window_ || GetFocus() != viewport_) return;
        auto down = [](int keyCode) { return (GetAsyncKeyState(keyCode) & 0x8000) != 0; };
        const double ahead = double(down('W')) - double(down('S'));
        const double sideways = double(down('D')) - double(down('A'));
        const double vertical = double(down('E')) - double(down('Q'));
        flight_.move(ahead, sideways, vertical, elapsed, flightSpeed_ * (down(VK_SHIFT) ? 5 : 1));
    }

    void refreshTerrain(bool force) {
        invokers_ = invokerPositions(planet_, opts_.invokers, simulationTime_ * 2.0);
        updateTerrain(force);
    }

    void updateTerrain(bool force) {
        applyTerrain(world_->update(invokers_, true, camera().eye), force);
    }

    void applyTerrain(const WorldStats& stats, bool force) {
        streamMs_ = stats.updateMs;
        if (showRanges_) refreshRanges();
        if (!force && stats.build.built == 0 && stats.build.removed == 0) return;
        std::vector<Vertex> geometry;
        geometry.reserve(world_->meshBytes() / sizeof(Triangle) * 3);
        for (const auto& entry : world_->chunks) {
            const Chunk& chunk = *entry.second;
            for (const auto& triangle : chunk.mesh) {
                const auto& p = triangle.vertices;
                const Vec middle = (p[0] + p[1] + p[2]) / 3;
                const Vec up = unit(middle);
                const Vec n = unit(cross(p[1] - p[0], p[2] - p[0]), up);
                for (const Vec point : p) {
                    const double height = std::clamp((length(point) - planetRadius + 660) / 850, 0.0, 1.0);
                    const double rock = std::clamp(2.4 * (1 - dot(n, up)), 0.0, 1.0);
                    const Vec tone = Vec{0.35, 0.41, 0.47} + Vec{0.31, 0.18, 0.06} * height + Vec{0.09, 0.04, 0.03} * rock;
                    geometry.push_back(vertex(point, n, color(tone, chunk.lod)));
                }
            }
        }
        triangles_ = geometry.size() / 3;
        renderer_->setTerrain(geometry);
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
                if (!freeView_ && length(pose.position - focus) > surfaceDistance_ * 2.7) continue;
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
                    lodTint_, wireframe_, {}, atmosphereEnabled_};
        }
        const Vec target = invokers_[selected_];
        const Vec radial = unit(target);
        const auto basis = frameAt(radial);
        Vec eye, center, nominalUp;
        double nearPlane;
        if (surface_) {
            const Vec tangent = basis[0] * std::cos(bearing_) + basis[1] * std::sin(bearing_);
            const Vec offset = radial * std::sin(elevation_) + tangent * std::cos(elevation_);
            center = target + radial * 24;
            eye = center + offset * surfaceDistance_;
            const double clearance = length(planet_.surface(unit(eye))) + 30 - length(eye);
            if (clearance > 0) eye = eye + unit(eye) * clearance;
            nominalUp = radial;
            nearPlane = std::max(0.3, surfaceDistance_ * 0.004);
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
                lodTint_, wireframe_, {}, atmosphereEnabled_};
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
            Camera orbitWire = camera();
            orbitWire.wireframe = true;
            orbitWire.atmosphereEnabled = false;
            renderer_->draw(orbitWire, objects_, opts_.captures / (prefix + "-orbit-wireframe.bmp"));
            int maxContacts = 0;
            for (int step = 0; step < 190; ++step) {
                tick();
                maxContacts = std::max(maxContacts, world_->physics.terrainContacts());
            }
            refreshTerrain(false);
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
            const bool previousWireframe = wireframe_, previousAtmosphere = atmosphereEnabled_;
            key('W', true);
            key('A', true);
            if (wireframe_ != previousWireframe || atmosphereEnabled_ != previousAtmosphere)
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
            if (atmosphereEnabled_ == previousAtmosphere) throw std::runtime_error("H did not toggle atmosphere");
            key('H', true);
            key('C', true);
            if (freeView_ || length(camera().eye - anchored.eye) > 1e-8)
                throw std::runtime_error("Free camera did not restore the anchored view");
            updateTerrain(false);
            buildObjects();
            action(Atmosphere);
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-surface-no-atmosphere.bmp"));
            action(Atmosphere);
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
            pullback.atmosphereEnabled = false;
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
        return 0;
    }

    void action(int command) {
        if (command >= Mesh && command <= Vdb) {
            if (method_ != command - Mesh) {
                method_ = command - Mesh;
                rebuild();
            }
        } else if (command == View) {
            freeView_ = false;
            surface_ = !surface_;
        } else if (command == FreeView) {
            if (!freeView_) {
                const Camera current = camera();
                flight_.reset(current.eye, current.forward, current.up);
                flightSpeed_ = std::clamp(std::abs(length(current.eye) - planetRadius) * 0.05, 50.0, 1000000.0);
            }
            freeView_ = !freeView_;
        } else if (command == Tint) lodTint_ = !lodTint_;
        else if (command == Wireframe) wireframe_ = !wireframe_;
        else if (command == Atmosphere) atmosphereEnabled_ = !atmosphereEnabled_;
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
            bearing_ = 0.42;
            elevation_ = 0.90;
            paused_ = false;
            rebuild();
        }
        repaintButtons();
        updateFooter();
        SetFocus(viewport_);
    }

    bool key(WPARAM code, bool first) {
        if (code == VK_ESCAPE) {
            PostMessageW(window_, WM_CLOSE, 0, 0);
            return true;
        }
        if (freeView_ && (code == 'W' || code == 'A' || code == 'S' || code == 'D' ||
                          code == 'Q' || code == 'E' || code == VK_SHIFT)) return true;
        if (!first) return false;
        if (code >= '1' && code <= '4') action(int(code - '1' + Mesh));
        else if (code == 'F') action(View);
        else if (code == 'L') action(Tint);
        else if (code == 'K') action(Ranges);
        else if (code == VK_F2) action(Wireframe);
        else if (code == 'H') action(Atmosphere);
        else if (code == 'C') action(FreeView);
        else if (code == 'B') action(Drop);
        else if (code == 'R') action(Reset);
        else if (code == VK_SPACE) action(Pause);
        else if (code == VK_TAB) {
            selected_ = (selected_ + 1) % invokers_.size();
            updateFooter();
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
        if (freeView_) info << L"   |   Free " << std::fixed << std::setprecision(0) << flightSpeed_ << L" m/s";
        info << L"   |   Invoker " << selected_ + 1 << L"/" << opts_.invokers << L" + camera"
             << L"   |   LOD 0/1/2/3/4+  " << counts[0] << L" / " << counts[1] << L" / " << counts[2] << L" / " << counts[3] << L" / " << coarse
             << L"   |   " << triangles_ << L" triangles   |   " << shapes_.size() << L" bodies   |   "
             << std::fixed << std::setprecision(1) << streamMs_ << L" ms LOD update";
        if (fps_ > 0) info << L"   |   " << std::setprecision(0) << fps_ << L" FPS";
        if (paused_) info << L"   |   Paused";
        if (renderer_->isWarp()) info << L"   |   WARP";
        status_ = info.str();

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
            : lodTint_ || showRanges_
            ? L"LOD 0 Cyan 50 m / 1 Green 100 m / 2 Gold 200 m / 3 Violet 400 m / 4-15 Red to Blue, x2 per LOD"
            : L"Drag to orbit   /   Wheel to zoom   /   Tab to select Invoker   /   Space to pause   /   B to drop bodies";
        DrawTextW(dc, controls, -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS);
        EndPaint(window_, &paintInfo);
    }

    void paintButton(const DRAWITEMSTRUCT& item) const {
        const int button = int(item.CtlID);
        const bool active = button == method_ + Mesh || (button == View && surface_) ||
                            (button == Tint && lodTint_) || (button == Ranges && showRanges_) ||
                            (button == Wireframe && wireframe_) || (button == Pause && paused_) ||
                            (button == Atmosphere && atmosphereEnabled_) || (button == FreeView && freeView_);
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
            case WM_COMMAND: if (HIWORD(w) == BN_CLICKED) app->pending_ = LOWORD(w); return 0;
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
    HFONT font_{};
    HBRUSH background_{};
    std::unique_ptr<Dx12Renderer> renderer_;
    std::unique_ptr<World> world_;
    std::vector<Vec> invokers_;
    std::vector<VisibleBody> shapes_;
    std::vector<Vertex> objects_;
    std::size_t selected_{}, triangles_{};
    std::wstring status_, processMemory_, terrainMemory_;
    bool closed_{}, dragging_{}, surface_{}, lodTint_{}, showRanges_{}, wireframe_{}, paused_{};
    bool atmosphereEnabled_{true};
    bool freeView_{};
    FreeCamera flight_;
    double flightSpeed_{200};
    POINT mouse_{};
    double simulationTime_{}, streamMs_{}, fps_{};
    double yaw_{0.13}, pitch_{0.16}, orbitDistance_{2.5 * planetRadius}, bearing_{0.42}, elevation_{0.90}, surfaceDistance_{1950};
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
