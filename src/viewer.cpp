#include "dx12_renderer.hpp"
#include "world.hpp"

#include <shellapi.h>
#include <windowsx.h>

#include <fstream>
#include <iomanip>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace voxel {
namespace {

constexpr wchar_t windowClass[] = L"VoxelWorldWindow";
constexpr wchar_t viewportClass[] = L"VoxelWorldViewport";
constexpr int toolbarHeight = 52;
constexpr int footerHeight = 80;
constexpr double pi = 3.14159265358979323846;
constexpr double dt = 1.0 / 60.0;
constexpr std::array<Method, 4> methods{Method::Mesh, Method::Grid, Method::Dag, Method::Vdb};
constexpr std::array<const wchar_t*, 4> methodLabels{L"Mesh BVH", L"VoxelField", L"HashDAG", L"OpenVDB"};
// Two contact points closer than this along the mesh normal count as the same
// answer: the verdict then calls the cast OK instead of early or penetrating.
constexpr double sweepContactSlack = 0.1;

enum Command { Mesh = 1, Grid, Dag, Vdb, View, Tint, Pause, Drop, Reset, Ranges, Wireframe, Sweep };

struct ToolbarButton { const wchar_t* label; int width; };
constexpr std::array<ToolbarButton, Sweep> toolbarButtons{{
    {L"1  Mesh BVH", 100}, {L"2  VoxelField", 100}, {L"3  HashDAG", 100}, {L"4  OpenVDB", 100},
    {L"Surface [F]", 104}, {L"LOD [L]", 72}, {L"Pause", 68}, {L"Drop [B]", 76}, {L"Reset", 56},
    {L"Ranges [K]", 84}, {L"Wireframe [W]", 110}, {L"Sweep [S]", 88}
}};

struct Options {
    int method{1};
    std::uint64_t seed{42};
    int invokers{75};
    bool warp{}, morton{};
    std::filesystem::path captures;
    // --sweep turns the window into the visual twin of main.cpp's --sweep
    // benchmark: static invokers that only host the cast loop's shapes.
    bool sweep{};
    int shapesPerInvoker{21};
    // --sweep-frames is main.cpp's --frames: the untimed validation pass walks
    // every frame's frame-seeded cast set, exactly like main's validate loop.
    int sweepFrames{100};
    // Negative means "start on the cast that went wrong worst", which is what a
    // comparison is usually opened for; a value >= 0 starts on that frame instead.
    int sweepFrame{-1};
    int sweepWarmup{1};
    int sweepBudget{48000};
    double sweepTolerance{0.5};
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
        auto bounded = [&](std::uint64_t lo, std::uint64_t hi) {
            const auto value = integer();
            if (value < lo || value > hi) throw std::invalid_argument("Option value is out of range");
            return int(value);
        };
        auto real = [&](double lo, double hi) {
            const std::wstring text = next();
            double value = 0;
            std::size_t parsed = 0;
            try {
                value = std::stod(text, &parsed);
            } catch (const std::exception&) {
                throw std::invalid_argument("Expected a decimal number");
            }
            if (parsed != text.size() || !(value >= lo && value <= hi)) throw std::invalid_argument("Option value is out of range");
            return value;
        };
        if (option == L"--warp") result.warp = true;
        else if (option == L"--morton") result.morton = true;
        else if (option == L"--sweep") result.sweep = true;
        else if (option == L"--capture-dir") result.captures = next();
        else if (option == L"--seed") result.seed = integer();
        else if (option == L"--shapes-per-invoker") result.shapesPerInvoker = bounded(1, 84);
        else if (option == L"--sweep-frame") result.sweepFrame = bounded(0, 100000);
        else if (option == L"--sweep-frames") result.sweepFrames = bounded(1, 400);
        else if (option == L"--sweep-warmup") result.sweepWarmup = bounded(0, 20);
        else if (option == L"--sweep-budget") result.sweepBudget = bounded(1, 10000000);
        else if (option == L"--sweep-tolerance") result.sweepTolerance = real(0.01, 10.0);
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
    return {{float(p.x), float(p.y), float(p.z)}, {float(normal.x), float(normal.y), float(normal.z)}, material};
}

// How the backend's contact sits against the mesh surface, as the normal-projected
// gap between the two points: EA means it stopped early and floats above the surface,
// PT that it went through it, OK that the two answers agree to within the slack.
const wchar_t* contactTag(Vec delta, Vec normal) {
    const double gap = dot(delta, normal);
    if (gap > sweepContactSlack) return L"EA";
    if (gap < -sweepContactSlack) return L"PT";
    return L"OK";
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

// Seven-segment digits, drawn as line segments in the camera plane, so a
// coordinate can sit next to its contact point without a font texture. Segment
// order: a top, b top right, c bottom right, d bottom, e bottom left, f top left,
// g middle, each as two corners in a 1 x 2 cell.
constexpr double glyphSegments[7][4] = {
    {0, 2, 1, 2}, {1, 2, 1, 1}, {1, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 1, 0, 2}, {0, 1, 1, 1}};

int glyphBits(char c) {
    switch (c) {
        case '0': return 63;    // a b c d e f
        case '1': return 6;     // b c
        case '2': return 91;    // a b d e g
        case '3': return 79;    // a b c d g
        case '4': return 102;   // b c f g
        case '5': return 109;   // a c d f g
        case '6': return 125;   // a c d e f g
        case '7': return 7;     // a b c
        case '8': return 127;
        case '9': return 111;   // a b c d f g
        case '-': return 64;    // g
        default: return 0;
    }
}

void appendLabel(std::vector<Vertex>& lines, const std::string& text, Vec origin, Vec right, Vec up,
                 double height, std::uint32_t material) {
    const double width = 0.55 * height, advance = 0.85 * height;
    auto segment = [&](const Vec& cell, double u0, double v0, double u1, double v1) {
        lines.push_back(vertex(cell + right * (u0 * width) + up * (v0 * 0.5 * height), {0, 0, 1}, material));
        lines.push_back(vertex(cell + right * (u1 * width) + up * (v1 * 0.5 * height), {0, 0, 1}, material));
    };
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const Vec cell = origin + right * (double(i) * advance);
        if (c == '.') {
            segment(cell, 0.30, 0, 0.70, 0);
            continue;
        }
        const int bits = glyphBits(c);
        for (int s = 0; s < 7; ++s) {
            if ((bits & (1 << s)) == 0) continue;
            segment(cell, glyphSegments[s][0], glyphSegments[s][1], glyphSegments[s][2], glyphSegments[s][3]);
        }
    }
}

// A screen-aligned asterisk marks a point that a sweep reported as its contact.
void appendMarker(std::vector<Vertex>& lines, Vec point, Vec right, Vec up, double size, std::uint32_t material) {
    for (const Vec direction : {right, up, unit(right + up)}) {
        lines.push_back(vertex(point - direction * size, {0, 0, 1}, material));
        lines.push_back(vertex(point + direction * size, {0, 0, 1}, material));
    }
}

class App {
public:
    explicit App(Options options)
        : opts_(std::move(options)), planet_(opts_.seed), method_(opts_.method), sweepMode_(opts_.sweep) {}
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
        window_ = CreateWindowExW(0, windowClass,
                                   sweepMode_ ? L"VoxelWorld  |  Sweep: mesh vs field" : L"VoxelWorld  |  Planet collision viewer",
                                   WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
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
        // Sweep mode has no rigid bodies, so the drop button would do nothing.
        if (sweepMode_) EnableWindow(buttons_[Drop - 1], FALSE);
        arrange();
        ShowWindow(window_, show);
        UpdateWindow(window_);
        renderer_ = std::make_unique<Dx12Renderer>(viewport_, opts_.warp);
        if (sweepMode_) {
            // The cast budget and tolerance are the knobs main.cpp's sweep
            // benchmark exposes, so they are wired straight through.
            setSweepBudget(opts_.sweepBudget);
            setSweepTolerance(opts_.sweepTolerance);
        }
        rebuild();
    }

    int run() {
        if (!opts_.captures.empty()) {
            if (sweepMode_) throw std::invalid_argument("--capture-dir is not available in --sweep mode");
            return captureAll();
        }
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
            // The labels are billboards that follow the camera, so the overlay is
            // rebuilt every frame in sweep mode.
            if (sweepMode_) refreshLines();
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
    // One cast of main.cpp's --sweep loop: the shape, its path, and the answers the
    // mesh reference and the active backend gave for it.
    struct SweepCast {
        Shape shape;
        Vec from{}, to{};
        Hit hit{}, meshHit{};
        int frame{}, index{};
        bool hasMesh{};
    };
    // main.cpp's Verdict, widened with the space metrics of its --sweep-last dump,
    // so the window prints the same numbers the benchmark prints.
    struct SweepTally {
        int meshContacts{}, backendContacts{}, meshMiss{}, backendMiss{};
        int meshUnresolved{}, backendUnresolved{}, initial{}, conflicts{}, refusals{}, paired{};
        // How the two contact points sit relative to the mesh surface: EA = the
        // backend stopped early and floats above it, PT = it penetrated through it,
        // OK = the two agree to within sweepContactSlack metres.
        int early{}, penetrating{}, exact{};
        double meanError{}, maxError{}, medianError{}, p95Error{};
        double meanPath{}, meanPoint{}, maxPoint{};
        // Where the worst cast of the pass lives, so the window can open on it.
        int worstError{-1}, worstPoint{-1};
        std::vector<double> errors;
        // Every cast in walk order: PT first, then EA, then OK, worst toi inside each
        // group, so <-/-> walks the comparison from the most suspicious contact down
        // to the ones both backends agree on.
        std::vector<int> byToi;
        std::vector<int> mismatches;
    };

    void rebuild() {
        simulationTime_ = 0;
        shapes_.clear();
        sweepCasts_.clear();
        if (method_ == 0) sweepMesh_.reset();
        world_ = std::make_unique<World>(planet_, methods[method_], opts_.morton);
        refreshTerrain(true);
        if (sweepMode_) {
            ensureSweepMesh();
            validateSweep();
            // Open on the worst toi difference -- the number the footer reports and
            // the first cast of the <-/-> walk -- unless --sweep-frame named a frame.
            if (opts_.sweepFrame >= 0) {
                castIndex_ = std::clamp(opts_.sweepFrame * int(invokers_.size()) * opts_.shapesPerInvoker, 0,
                                        std::max(0, int(sweepCasts_.size()) - 1));
            } else {
                castIndex_ = tally_.byToi.empty() ? 0 : tally_.byToi.front();
            }
            focusCurrent();
            refreshLines();
        } else {
            for (int i = 0; i < opts_.invokers; ++i) spawn(i);
        }
        updateFooter();
        repaintButtons();
    }

    // The toolbar's sweep switch: the same view --sweep builds at startup, turned
    // on and off at run time. Both directions re-create the world, because sweep
    // mode must run against a world with no rigid bodies in it (a body in the way
    // would answer the cast) and the interactive mode needs them back. Entering
    // pins the invokers to frame 0, builds the mesh reference, runs main.cpp's
    // validate pass and opens on the cast that disagrees worst.
    void setSweep(bool on) {
        if (sweepMode_ == on) return;
        sweepMode_ = on;
        if (!on) {
            sweepMesh_.reset();
            focusValid_ = false;
            surfaceDistance_ = 1950.0;
        }
        // Sweep mode has no rigid bodies, so the drop button would do nothing.
        EnableWindow(buttons_[Drop - 1], on ? FALSE : TRUE);
        SetWindowTextW(window_, on ? L"VoxelWorld  |  Sweep: mesh vs field" : L"VoxelWorld  |  Planet collision viewer");
        rebuild();
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
            shapes_.push_back({id, shape, color(palette[i % 3], 4)});
        }
    }

    void tick() {
        // Sweep mode is the benchmark loop made visible: every invoker stays at its
        // frame-0 position and no rigid body exists, so there is nothing to step.
        if (sweepMode_) return;
        world_->physics.step(dt, true);
        simulationTime_ += dt;
        invokers_ = invokerPositions(planet_, opts_.invokers, simulationTime_ * 2.0);
    }

    // The mesh reference only exists while sweep mode is on and the active backend
    // is a field: the comparison is mesh BVH against the field answer.
    void ensureSweepMesh() {
        if (method_ == 0) {
            sweepMesh_.reset();
            return;
        }
        if (sweepMesh_) return;
        sweepMesh_ = std::make_unique<World>(planet_, Method::Mesh, opts_.morton);
        sweepMesh_->update(invokers_, true, camera().eye);
    }

    static double randomPick(std::mt19937_64& rng) { return (rng() >> 11) * 0x1p-53; }

    // main.cpp's shapeAt: one of the invoker's three kinds, the up axis tilted by
    // up to ~0.22 rad in a random azimuth, size scaled 0.7x..1.5x.
    Shape castShape(Vec anchor, int index, std::mt19937_64& rng) const {
        const Vec up = unit(anchor);
        const auto axes = frameAt(up);
        const double azimuth = 2 * pi * randomPick(rng);
        const double tilt = 0.45 * (randomPick(rng) - 0.5);
        const Vec tilted = unit(up + (axes[0] * std::cos(azimuth) + axes[1] * std::sin(azimuth)) * tilt);
        Shape shape = sampleShape(index % 3 + 1, tilted);
        shape.size = shape.size * (0.7 + 0.8 * randomPick(rng));
        return shape;
    }

    // main.cpp's placement: a random surface target within ~1100 m of the invoker,
    // entered from clear above it and left well below it.
    std::pair<Vec, Vec> castPlacement(Vec anchor, const Shape& shape, std::mt19937_64& rng) const {
        const Vec up = unit(anchor);
        const auto axes = frameAt(up);
        const Vec dir = unit(up * planetRadius + axes[0] * (1100 * (randomPick(rng) - 0.5))
                                                + axes[1] * (1100 * (randomPick(rng) - 0.5)));
        const Vec surface = planet_.surface(dir);
        const double support = dot(abs(dir), shape.extents());
        return {surface + dir * (support + 120.0), surface - dir * (support + 60.0)};
    }

    // The cast set of one frame, frame-seeded exactly like main.cpp's fillFrame, so
    // the same seed and frame always issue the same shapes, in the same order.
    void fillFrame(int frame, std::uint64_t salt, std::vector<SweepCast>& casts) const {
        std::mt19937_64 rng(opts_.seed ^ (salt * std::uint64_t(frame + 1)));
        for (const Vec& anchor : invokers_) {
            for (int k = 0; k < opts_.shapesPerInvoker; ++k) {
                SweepCast cast;
                cast.shape = castShape(anchor, k, rng);
                const auto [from, to] = castPlacement(anchor, cast.shape, rng);
                cast.frame = frame;
                cast.index = int(casts.size()) % std::max(1, int(invokers_.size()) * opts_.shapesPerInvoker);
                cast.from = from;
                cast.to = to;
                casts.push_back(cast);
            }
        }
    }

    // main.cpp's sweep validation, run against the live streaming world instead of
    // an offline pair: warm up on its own cast set, time the backend pass over every
    // frame, then issue both backends on the same cast and tally the verdict --
    // contacts and misses per backend, refusals (a field search declining to decide)
    // kept apart from conflicts (contact vs miss), the per-cast toi error, and the
    // contact-point distance, the normal-projected gap and the path delta.
    void validateSweep() {
        constexpr std::uint64_t measuredSalt = 0x51ed270b1a6c3f27ULL, warmupSalt = 0x2545f4914f6cdd1dULL;
        const int perFrame = int(invokers_.size()) * opts_.shapesPerInvoker;
        setSweepBudget(opts_.sweepBudget);
        setSweepTolerance(opts_.sweepTolerance);
        resetSweepDiagnostics();

        sweepCasts_.clear();
        tally_ = SweepTally{};
        sweepCasts_.reserve(std::size_t(perFrame) * std::size_t(opts_.sweepFrames));
        for (int frame = 0; frame < opts_.sweepFrames; ++frame) fillFrame(frame, measuredSalt, sweepCasts_);

        std::vector<SweepCast> casts;
        casts.reserve(std::size_t(perFrame));
        for (int w = 0; w < opts_.sweepWarmup; ++w) {
            casts.clear();
            fillFrame(w, warmupSalt, casts);
            for (const SweepCast& cast : casts) {
                world_->physics.sweep(cast.shape, cast.from, cast.to);
                if (sweepMesh_) sweepMesh_->physics.sweep(cast.shape, cast.from, cast.to);
            }
        }

        // One pass issues both backends on every cast of every frame.
        std::vector<double> pathByIndex(sweepCasts_.size(), -1.0);
        for (SweepCast& cast : sweepCasts_) {
            const Hit otherHit = world_->physics.sweep(cast.shape, cast.from, cast.to);
            cast.hit = otherHit;
            cast.hasMesh = sweepMesh_ != nullptr;
            if (!cast.hasMesh) continue;
            cast.meshHit = sweepMesh_->physics.sweep(cast.shape, cast.from, cast.to);
            const Hit& meshHit = cast.meshHit;
            const double span = length(cast.to - cast.from);
            tally_.meshContacts += meshHit.state == Hit::State::Contact;
            tally_.backendContacts += otherHit.state == Hit::State::Contact;
            tally_.meshMiss += meshHit.state == Hit::State::Miss;
            tally_.backendMiss += otherHit.state == Hit::State::Miss;
            tally_.meshUnresolved += meshHit.state == Hit::State::Unresolved;
            tally_.backendUnresolved += otherHit.state == Hit::State::Unresolved;
            tally_.initial += meshHit.initial || otherHit.initial;
            if (meshHit.state != otherHit.state) {
                const bool refusal = meshHit.state == Hit::State::Unresolved || otherHit.state == Hit::State::Unresolved;
                tally_.refusals += refusal;
                tally_.conflicts += !refusal;
                tally_.mismatches.push_back(int(&cast - sweepCasts_.data()));
            }
            if (meshHit.state == Hit::State::Contact && otherHit.state == Hit::State::Contact) {
                tally_.errors.push_back(std::abs(meshHit.fraction - otherHit.fraction) * span);
                if (tally_.errors.back() > (tally_.worstError >= 0 ? tally_.errors[std::size_t(tally_.worstError)] : 0.0)) {
                    tally_.worstError = int(&cast - sweepCasts_.data());
                }
                const double path = tally_.errors.back();
                const double point = length(otherHit.position - meshHit.position);
                pathByIndex[std::size_t(&cast - sweepCasts_.data())] = path;
                ++tally_.paired;
                if (point > tally_.maxPoint) {
                    tally_.maxPoint = point;
                    tally_.worstPoint = int(&cast - sweepCasts_.data());
                }
                tally_.meanPath += path;
                tally_.meanPoint += point;
                const std::wstring tag = contactTag(otherHit.position - meshHit.position, meshHit.normal);
                if (tag == L"EA") ++tally_.early;
                else if (tag == L"PT") ++tally_.penetrating;
                else ++tally_.exact;
            }
        }
        std::sort(tally_.errors.begin(), tally_.errors.end());
        const std::size_t paired = std::size_t(std::max(1, tally_.paired));
        if (!tally_.errors.empty()) {
            tally_.meanError = std::accumulate(tally_.errors.begin(), tally_.errors.end(), 0.0) / double(paired);
            tally_.maxError = tally_.errors.back();
            tally_.medianError = tally_.errors[tally_.errors.size() / 2];
            // Same percentile main.cpp prints: linear interpolation between the two
            // neighbours, matching its percentile() helper.
            const double rank = 0.95 * double(tally_.errors.size() - 1);
            const std::size_t lower = std::size_t(rank);
            const double weight = rank - double(lower);
            tally_.p95Error = lower + 1 < tally_.errors.size()
                                  ? tally_.errors[lower] + (tally_.errors[lower + 1] - tally_.errors[lower]) * weight
                                  : tally_.errors.back();
        }
        tally_.meanPath /= double(paired);
        tally_.meanPoint /= double(paired);
        // Penetrating contacts first, then the early ones, then the agreeing ones;
        // inside each group the worst toi difference comes first. Casts without a
        // paired contact (class -1) fall to the end so a walk still visits everything.
        // Worst toi difference first, so <-/-> walks the comparison from the biggest
        // disagreement down to the smallest. Casts without a paired contact fall to
        // the end so a walk still visits everything.
        tally_.byToi.resize(sweepCasts_.size());
        std::iota(tally_.byToi.begin(), tally_.byToi.end(), 0);
        std::sort(tally_.byToi.begin(), tally_.byToi.end(), [&](int a, int b) {
            const double ea = pathByIndex[std::size_t(a)], eb = pathByIndex[std::size_t(b)];
            const bool pairedA = ea >= 0, pairedB = eb >= 0;
            if (pairedA != pairedB) return pairedA;
            if (pairedA && ea != eb) return ea > eb;
            return a < b;
        });
    }

    // Point the camera at the cast under inspection, close enough that the path,
    // the shapes and the coordinate labels all read.
    void focusCurrent() {
        if (sweepCasts_.empty()) {
            focusValid_ = false;
            return;
        }
        castIndex_ = std::clamp(castIndex_, 0, int(sweepCasts_.size()) - 1);
        const SweepCast& cast = sweepCasts_[std::size_t(castIndex_)];
        focus_ = (cast.from + cast.to) * 0.5;
        focusValid_ = true;
        surface_ = true;
        // Frame the whole path: main.cpp's casts run from 120 m above the surface
        // to 60 m below, so long ones need a wider view.
        surfaceDistance_ = std::clamp(length(cast.to - cast.from) * 1.8, 260.0, 1000.0);
    }

    // Re-issue the two backends on the cast on screen only. A terrain publish can
    // change its answer, but re-running the whole validated set on every publish
    // would cost a frame's worth of sweeps at 6720 casts, so the verdict row keeps
    // the numbers of the last full pass (key V re-runs it).
    void resweepCurrent() {
        if (sweepCasts_.empty()) return;
        SweepCast& cast = sweepCasts_[std::size_t(castIndex_)];
        cast.hit = world_->physics.sweep(cast.shape, cast.from, cast.to);
        cast.hasMesh = sweepMesh_ != nullptr;
        if (cast.hasMesh) cast.meshHit = sweepMesh_->physics.sweep(cast.shape, cast.from, cast.to);
    }

    void stepCast(int delta) {
        if (!sweepMode_ || sweepCasts_.empty()) return;
        const int count = int(sweepCasts_.size());
        if (int(tally_.byToi.size()) == count) {
            // Walk the ranking, so -> starts at the worst toi and each press moves to
            // the next smaller disagreement. The current cast may be anywhere in it
            // (E/O/N/C jump by their own keys), so locate it first.
            int at = 0;
            for (int i = 0; i < count; ++i) {
                if (tally_.byToi[std::size_t(i)] == castIndex_) {
                    at = i;
                    break;
                }
            }
            at = ((at + delta) % count + count) % count;
            castIndex_ = tally_.byToi[std::size_t(at)];
        } else {
            castIndex_ = ((castIndex_ + delta) % count + count) % count;
        }
        focusCurrent();
        refreshLines();
        updateFooter();
    }

    void refreshTerrain(bool force) {
        // Sweep mode pins every invoker to its frame-0 position, like the benchmark.
        invokers_ = invokerPositions(planet_, opts_.invokers, sweepMode_ ? 0.0 : simulationTime_ * 2.0);
        updateTerrain(force);
    }

    void updateTerrain(bool force) {
        applyTerrain(world_->update(invokers_, true, camera().eye), force);
    }

    void applyTerrain(const WorldStats& stats, bool force) {
        streamMs_ = stats.updateMs;
        // Keep the mesh reference on the same published terrain, otherwise a cast
        // near an invoker that just streamed in would compare against old chunks.
        // Re-running the sweep pass there also refreshes the verdict, which is what
        // main.cpp's offline worlds can never do.
        if (sweepMesh_ && (force || stats.build.built != 0 || stats.build.removed != 0)) {
            sweepMesh_->update(invokers_, true, camera().eye);
            if (sweepMode_) {
                resweepCurrent();
                focusCurrent();
                refreshLines();
                updateFooter();
            }
        }
        if (showRanges_) refreshLines();
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

    // Clipmap debug windows plus, in sweep mode, the overlay of the cast being
    // inspected: its sweep direction, both contact markers and the coordinate
    // labels, all of which follow the camera so they are rebuilt every frame.
    void refreshLines() {
        lines_.clear();
        if (showRanges_) appendRanges(lines_);
        if (sweepMode_) appendSweepOverlay(lines_);
        renderer_->setClipmapLines(lines_);
    }

    void appendRanges(std::vector<Vertex>& lines) const {
        constexpr Vec shades[] = {{0.25, 0.91, 1}, {0.40, 1, 0.54}, {1, 0.76, 0.28}, {0.81, 0.53, 1}};
        for (const auto& w : world_->clipmapWindows()) {
            const auto tint = color(shades[w.lod], 255);
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

    void appendSweepOverlay(std::vector<Vertex>& lines) const {
        if (sweepCasts_.empty()) return;
        const SweepCast& cast = sweepCasts_[std::size_t(castIndex_)];
        const Camera view = camera();
        const auto meshTint = color(Vec{1.0, 0.62, 0.30}, 255), backendTint = color(Vec{0.30, 0.96, 1.0}, 255);
        const Vec delta = cast.to - cast.from, down = unit(delta);
        const double span = length(delta);

        // The sweep direction: the whole path, with an arrow head where it starts.
        const auto path = color(Vec{0.60, 0.68, 0.80}, 255);
        lines.push_back(vertex(cast.from, {0, 0, 1}, path));
        lines.push_back(vertex(cast.to, {0, 0, 1}, path));
        const Vec head = cast.from + down * (0.10 * span);
        for (const Vec side : {unit(down + view.right * 0.22), unit(down - view.right * 0.22),
                               unit(down + view.up * 0.22), unit(down - view.up * 0.22)}) {
            lines.push_back(vertex(head, {0, 0, 1}, path));
            lines.push_back(vertex(head + side * (0.06 * span), {0, 0, 1}, path));
        }

        // Contact markers and their coordinates, one label above and one below so
        // the two answers stay apart even when they agree to a few centimetres.
        // Seven-segment cells can only draw digits, so the backend is told apart by
        // colour and vertical side, and spelled out in the footer.
        constexpr double labelHeight = 24.0;
        auto label = [&](const Hit& hit, std::uint32_t tint, double lift) {
            if (hit.state != Hit::State::Contact) return;
            appendMarker(lines, hit.position, view.right, view.up, 9.0, tint);
            std::ostringstream position;
            position << std::llround(hit.position.x) << ' ' << std::llround(hit.position.y) << ' '
                     << std::llround(hit.position.z);
            appendLabel(lines, position.str(), hit.position + view.up * lift, view.right, view.up, labelHeight, tint);
        };
        if (cast.hasMesh) label(cast.meshHit, meshTint, 1.5 * labelHeight);
        label(cast.hit, backendTint, -2.9 * labelHeight);
    }

    static const wchar_t* kindName(ShapeKind kind) {
        switch (kind) {
            case ShapeKind::Sphere: return L"sphere";
            case ShapeKind::Capsule: return L"capsule";
            case ShapeKind::Box: return L"box";
        }
        return L"?";
    }

    static void drawShape(std::vector<Vertex>& batch, const Shape& shape, Vec center, std::uint32_t material) {
        if (shape.kind == ShapeKind::Box) appendBox(batch, shape, center, material);
        else appendRound(batch, shape, center, material);
    }

    void buildObjects() {
        objects_.clear();
        if (sweepMode_) {
            appendSweepShapes();
            return;
        }
        if (surface_) {
            const Vec focus = invokers_[selected_];
            for (const auto& body : shapes_) {
                const auto pose = world_->physics.bodyPose(body.index);
                if (length(pose.position - focus) > surfaceDistance_ * 2.7) continue;
                Shape shape = body.shape;
                shape.axes = pose.axes;
                drawShape(objects_, shape, pose.position, body.material);
            }
        }
        for (std::size_t i = 0; i < invokers_.size(); ++i) {
            appendBeacon(objects_, invokers_[i], surface_ ? 24 : 380, i == selected_);
        }
    }

    // The shape of the cast being inspected, drawn at the pose it started from and
    // where each backend stopped: the mesh BVH answer in warm orange, the active
    // backend in cyan on top, so a visible orange rim is a visible disagreement.
    void appendSweepShapes() {
        if (sweepCasts_.empty()) return;
        const SweepCast& cast = sweepCasts_[std::size_t(castIndex_)];
        const auto meshTint = color(Vec{1.0, 0.62, 0.30}, 255), backendTint = color(Vec{0.30, 0.96, 1.0}, 255);
        const auto startTint = color(Vec{0.34, 0.42, 0.52}, 255);
        auto centerOf = [&](const Hit& hit) {
            return hit.state == Hit::State::Contact ? cast.from + (cast.to - cast.from) * hit.fraction : cast.from;
        };
        drawShape(objects_, cast.shape, cast.from, startTint);
        if (cast.hasMesh) drawShape(objects_, cast.shape, centerOf(cast.meshHit), meshTint);
        drawShape(objects_, cast.shape, centerOf(cast.hit), cast.hasMesh ? backendTint : meshTint);
    }

    Camera camera() const {
        // Sweep inspection follows the cast it is showing; the interactive mode
        // follows the selected invoker.
        const Vec target = focusValid_ ? focus_ : invokers_[selected_];
        const Vec radial = unit(target);
        const auto basis = frameAt(radial);
        Vec eye, center, nominalUp;
        double nearPlane;
        if (surface_) {
            const Vec tangent = basis[0] * std::cos(bearing_) + basis[1] * std::sin(bearing_);
            const Vec offset = radial * std::sin(elevation_) + tangent * std::cos(elevation_);
            center = target + radial * 24;
            eye = center + offset * surfaceDistance_;
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
        return {eye, right, unit(cross(right, forward)), forward, nearPlane, 270000.0, lodTint_, wireframe_};
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
            action(Wireframe);
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-wireframe.bmp"));
            action(Wireframe);
            showRanges_ = true;
            refreshLines();
            renderer_->draw(camera(), objects_, opts_.captures / (prefix + "-ranges.bmp"));
            showRanges_ = false;
            refreshLines();
            report << prefix << " triangles=" << triangles_ << " invokers=" << opts_.invokers << " bodies=" << shapes_.size()
                   << " peak_contacts=" << maxContacts << '\n';
            if (maxContacts == 0) throw std::runtime_error("Capture scenario did not observe terrain contact");
        }
        return 0;
    }

    void action(int command) {
        if (command >= Mesh && command <= Vdb) {
            if (method_ != command - Mesh) {
                method_ = command - Mesh;
                rebuild();
            }
        } else if (command == View) surface_ = !surface_;
        else if (command == Tint) lodTint_ = !lodTint_;
        else if (command == Wireframe) wireframe_ = !wireframe_;
        else if (command == Ranges) {
            showRanges_ = !showRanges_;
            refreshLines();
        }
        else if (command == Pause) paused_ = !paused_;
        // The sweep comparison is a whole different world: rebuild on the way in
        // and on the way out.
        else if (command == Sweep) setSweep(!sweepMode_);
        // Sweep mode has no rigid bodies to drop.
        else if (command == Drop && !sweepMode_) {
            surface_ = true;
            spawn(int(selected_));
        } else if (command == Reset) {
            selected_ = 0;
            yaw_ = 0.13;
            pitch_ = 0.16;
            orbitDistance_ = 64100;
            surfaceDistance_ = sweepMode_ ? 2600.0 : 1950.0;
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
        if (!first) return false;
        if (code >= '1' && code <= '4') action(int(code - '1' + Mesh));
        else if (code == 'F') action(View);
        else if (code == 'L') action(Tint);
        else if (code == 'K') action(Ranges);
        else if (code == 'W') action(Wireframe);
        else if (code == 'S') action(Sweep);
        else if (code == 'N' && sweepMode_) {
            // Jump to the first cast of the next frame in the validated set.
            const int perFrame = int(invokers_.size()) * opts_.shapesPerInvoker;
            if (!sweepCasts_.empty()) castIndex_ = (castIndex_ / perFrame + 1) * perFrame % int(sweepCasts_.size());
            focusCurrent();
            refreshLines();
            updateFooter();
        }
        else if (code == 'C' && sweepMode_) {
            // Jump to the next cast whose two backends disagree.
            for (const int index : tally_.mismatches) {
                if (index > castIndex_) {
                    castIndex_ = index;
                    focusCurrent();
                    refreshLines();
                    updateFooter();
                    return true;
                }
            }
            if (!tally_.mismatches.empty()) {
                castIndex_ = tally_.mismatches.front();
                focusCurrent();
                refreshLines();
                updateFooter();
            }
            return true;
        }
        else if (code == 'V' && sweepMode_) {
            // Re-run main.cpp's whole validation pass on the terrain as it is now.
            validateSweep();
            focusCurrent();
            refreshLines();
            updateFooter();
        }
        else if (code == 'E' && sweepMode_) {
            // Jump to the cast with the largest TOI error of the last pass.
            if (tally_.worstError >= 0) {
                castIndex_ = tally_.worstError;
                focusCurrent();
                refreshLines();
                updateFooter();
            }
            return true;
        }
        else if (code == 'O' && sweepMode_) {
            // Jump to the cast whose two contact points are furthest apart.
            if (tally_.worstPoint >= 0) {
                castIndex_ = tally_.worstPoint;
                focusCurrent();
                refreshLines();
                updateFooter();
            }
            return true;
        }
        else if (code == VK_RIGHT && sweepMode_) stepCast(1);
        else if (code == VK_LEFT && sweepMode_) stepCast(-1);
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
        const int height = std::max(0L, client.bottom - toolbarHeight - footerHeight);
        MoveWindow(viewport_, 0, toolbarHeight, client.right, height, TRUE);
        int x = 12;
        for (int i = 0; i < int(buttons_.size()); ++i) {
            const int width = toolbarButtons[i].width;
            MoveWindow(buttons_[i], x, 9, width, 34, TRUE);
            x += width + (i == 3 ? 21 : 6);
        }
        InvalidateRect(window_, nullptr, TRUE);
    }

    void repaintButtons() {
        SetWindowTextW(buttons_[View - 1], surface_ ? L"Orbit [F]" : L"Surface [F]");
        SetWindowTextW(buttons_[Pause - 1], paused_ ? L"Resume" : L"Pause");
        for (auto button : buttons_) InvalidateRect(button, nullptr, TRUE);
    }

    void updateFooter() {
        const auto counts = world_->lodCounts();
        std::wostringstream info;
        if (sweepMode_) {
            info << std::fixed << methodLabels[method_] << L" vs mesh   |   " << opts_.sweepFrames << L" frames x "
                 << (invokers_.size() * std::size_t(opts_.shapesPerInvoker)) << L" = " << sweepCasts_.size()
                 << L" casts   |   cast " << (sweepCasts_.empty() ? 0 : castIndex_ + 1) << L"/" << sweepCasts_.size();
            if (!sweepCasts_.empty()) {
                const SweepCast& cast = sweepCasts_[std::size_t(castIndex_)];
                info << L" (frame " << cast.frame << L", invoker " << cast.index / std::max(1, opts_.shapesPerInvoker)
                     << L", shape " << cast.index % std::max(1, opts_.shapesPerInvoker) << L", "
                     << kindName(cast.shape.kind) << L')';
                // Flag the two casts the pass points at, so the picture is known to
                // be the worst case rather than an arbitrary one.
                if (castIndex_ == tally_.worstError)
                    info << L"   WORST toi " << std::setprecision(3) << tally_.maxError << L" m";
                else if (castIndex_ == tally_.worstPoint)
                    info << L"   WORST point " << std::setprecision(3) << tally_.maxPoint << L" m";
                info << std::setprecision(0);
            }
            info << L"   |   " << triangles_ << L" triangles";
        } else {
            info << methodLabels[method_] << L"   |   Invoker " << selected_ + 1 << L"/" << opts_.invokers << L" + camera"
                 << L"   |   LOD 0/1/2/3  " << counts[0] << L" / " << counts[1] << L" / " << counts[2] << L" / " << counts[3]
                 << L"   |   " << triangles_ << L" triangles   |   " << shapes_.size() << L" bodies   |   "
                 << std::fixed << std::setprecision(1) << streamMs_ << L" ms LOD update";
        }
        if (fps_ > 0) info << L"   |   " << std::setprecision(0) << fps_ << L" FPS";
        if (paused_) info << L"   |   Paused";
        if (renderer_->isWarp()) info << L"   |   WARP";
        status_ = info.str();
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
        RECT row{13, client.bottom - footerHeight + 6, client.right - 10, client.bottom - footerHeight + 27};
        DrawTextW(dc, status_.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
        SetTextColor(dc, RGB(152, 170, 193));
        row.top += 22;
        row.bottom += 22;
        std::wstring controls;
        if (sweepMode_) {
            std::wostringstream sweep;
            sweep << std::fixed;
            if (!sweepCasts_.empty()) {
                const SweepCast& cast = sweepCasts_[std::size_t(castIndex_)];
                const double span = length(cast.to - cast.from);
                sweep << L"start (" << std::llround(cast.from.x) << L' ' << std::llround(cast.from.y) << L' '
                      << std::llround(cast.from.z) << L") length " << std::setprecision(0) << span << L" m   |   ";
                if (cast.hasMesh) {
                    const auto& hit = cast.meshHit;
                    if (hit.state == Hit::State::Contact) {
                        sweep << L"M (" << std::llround(hit.position.x) << L' ' << std::llround(hit.position.y) << L' '
                              << std::llround(hit.position.z) << L") f " << std::setprecision(3) << hit.fraction << L"   |   ";
                    } else sweep << L"M miss   |   ";
                }
                const auto& hit = cast.hit;
                if (hit.state == Hit::State::Contact) {
                    sweep << L"S (" << std::llround(hit.position.x) << L' ' << std::llround(hit.position.y) << L' '
                          << std::llround(hit.position.z) << L") f " << std::setprecision(3) << hit.fraction << L"   |   ";
                    if (cast.hasMesh && cast.meshHit.state == Hit::State::Contact) {
                        sweep << L"toi " << std::abs(hit.fraction - cast.meshHit.fraction) * span << L"  point "
                              << length(hit.position - cast.meshHit.position) << L" m   |   "
                              << contactTag(hit.position - cast.meshHit.position, cast.meshHit.normal);
                    }
                } else sweep << L"S miss";
            }
            controls = sweep.str();
        } else if (lodTint_ || showRanges_) {
            controls = L"LOD 0 Cyan 50 m   /   LOD 1 Green 100 m   /   LOD 2 Gold 200 m   /   LOD 3 Violet 400 m";
        } else {
            controls = L"Drag to orbit   /   Wheel to zoom   /   Tab to select Invoker   /   Space to pause   /   B to drop bodies   /   S to sweep vs mesh";
        }
        DrawTextW(dc, controls.c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS);
        if (sweepMode_) {
            // A third row for the whole-run verdict, so the per-cast numbers above
            // never get ellipsised away.
            row.top += 22;
            row.bottom += 22;
            std::wostringstream totals;
            totals << std::fixed;
            if (sweepMesh_) {
                // The same verdict main.cpp prints, minus the timing block: contacts
                // per backend, refusals kept apart from conflicts, the worst toi
                // error, the mean path and point deltas, and how the two contact
                // points sit against the mesh surface.
                totals << L"all " << sweepCasts_.size() << L" | contacts mesh/backend " << tally_.meshContacts << L'/'
                       << tally_.backendContacts << L", conf " << tally_.conflicts << L", refuse " << tally_.refusals
                       << L", init " << tally_.initial << L", paired " << tally_.paired << L" | toi max "
                       << std::setprecision(3) << tally_.maxError << L" m | mean path/point " << tally_.meanPath << L'/'
                       << tally_.meanPoint << L" m | PT " << tally_.penetrating << L"  EA " << tally_.early << L"  OK "
                       << tally_.exact;
            } else {
                totals << L"all " << sweepCasts_.size() << L": the active backend is the mesh, nothing to compare";
            }
            DrawTextW(dc, totals.str().c_str(), -1, &row, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS);
        }
        EndPaint(window_, &paintInfo);
    }

    void paintButton(const DRAWITEMSTRUCT& item) const {
        const int button = int(item.CtlID);
        const bool active = button == method_ + Mesh || (button == View && surface_) ||
                            (button == Tint && lodTint_) || (button == Ranges && showRanges_) ||
                            (button == Wireframe && wireframe_) || (button == Pause && paused_) ||
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
                limits->ptMinTrackSize = {1200, 560};
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
                SetCapture(window);
                SetFocus(window);
                app->dragging_ = true;
                app->mouse_ = {GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                return 0;
            case WM_MOUSEMOVE:
                if (app->dragging_) {
                    const POINT next{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                    const double dx = (next.x - app->mouse_.x) * 0.006, dy = (next.y - app->mouse_.y) * 0.006;
                    if (app->surface_) {
                        app->bearing_ = std::remainder(app->bearing_ - dx, 2 * pi);
                        app->elevation_ = std::clamp(app->elevation_ + dy, 0.24, 1.46);
                    } else {
                        app->yaw_ = std::remainder(app->yaw_ - dx, 2 * pi);
                        app->pitch_ = std::clamp(app->pitch_ + dy, -1.35, 1.35);
                    }
                    app->mouse_ = next;
                }
                return 0;
            case WM_LBUTTONUP: if (GetCapture() == window) ReleaseCapture(); return 0;
            case WM_CAPTURECHANGED: app->dragging_ = false; return 0;
            case WM_MOUSEWHEEL: {
                const double factor = std::pow(0.84, double(GET_WHEEL_DELTA_WPARAM(w)) / WHEEL_DELTA);
                const double closest = app->sweepMode_ ? 40.0 : 110.0;
                if (app->surface_) app->surfaceDistance_ = std::clamp(app->surfaceDistance_ * factor, closest, 10500.0);
                else app->orbitDistance_ = std::clamp(app->orbitDistance_ * factor, 37000.0, 171000.0);
                return 0;
            }
            default: return DefWindowProcW(window, message, w, l);
        }
    }

    Options opts_;
    Planet planet_;
    int method_, pending_{};
    bool sweepMode_{};
    HWND window_{}, viewport_{};
    std::array<HWND, toolbarButtons.size()> buttons_{};
    HFONT font_{};
    HBRUSH background_{};
    std::unique_ptr<Dx12Renderer> renderer_;
    std::unique_ptr<World> world_;
    // Mesh reference for the sweep comparison: only alive while sweep mode is on
    // and the active backend is a field.
    std::unique_ptr<World> sweepMesh_;
    std::vector<SweepCast> sweepCasts_;
    SweepTally tally_;
    // The cast the sweep inspection view is showing, and the point it frames.
    int castIndex_{};
    Vec focus_{};
    bool focusValid_{};
    std::vector<Vertex> lines_;
    std::vector<Vec> invokers_;
    std::vector<VisibleBody> shapes_;
    std::vector<Vertex> objects_;
    std::size_t selected_{}, triangles_{};
    std::wstring status_;
    bool closed_{}, dragging_{}, surface_{}, lodTint_{}, showRanges_{}, wireframe_{}, paused_{};
    POINT mouse_{};
    double simulationTime_{}, streamMs_{}, fps_{};
    double yaw_{0.13}, pitch_{0.16}, orbitDistance_{64100}, bearing_{0.42}, elevation_{0.90}, surfaceDistance_{1950};
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
