#pragma once

#include "world.hpp"
#include "dx12_renderer.hpp"

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace voxel {

// Command-line / toolbar knobs for the sweep (mesh vs field) comparison view.
struct SweepOptions {
    bool enabled{false};
    // Invoker count used only while sweep mode is on; the interactive mode
    // keeps using the global Options::invokers count.
    int invokers{2};
    int shapesPerInvoker{21};
    int frames{100};
    int frame{-1};       // Negative opens on the worst cast; >= 0 starts on that frame.
    int warmup{1};
    int budget{48000};
    double tolerance{0.75};
};

// One cast of the validation loop: the shape, its path, and the answers the mesh
// reference and the active backend produced for it.
struct SweepCast {
    Shape shape;
    Vec from{}, to{};
    Hit hit{}, meshHit{};
    int frame{}, index{};
    bool hasMesh{};
    double toiError{0};  // |TOI mesh - TOI backend| * path length; 0 when not paired
};

// Aggregated verdict over a validation pass, widened with the space metrics the
// benchmark dumps: contacts/misses per backend, refusals vs conflicts, per-cast
// TOI error and contact-point distance, and where the worst cast lives.
struct SweepTally {
    int meshContacts{}, backendContacts{}, meshMiss{}, backendMiss{};
    int meshUnresolved{}, backendUnresolved{}, initial{}, conflicts{}, refusals{}, paired{};
    // How the two contact points sit against the mesh surface: EA stopped early
    // and floats above, PT penetrated through, OK agrees within slack.
    int early{}, penetrating{}, exact{};
    double meanError{}, maxError{}, medianError{}, p95Error{};
    double meanPath{}, meanPoint{}, maxPoint{};
    int worstError{-1}, worstPoint{-1};
    std::vector<double> errors;
    std::vector<int> mismatches;
};

// Owns the sweep comparison state: the mesh reference world, the validated cast
// set, the tally and the currently inspected cast. The viewer forwards lifecycle
// and navigation events into this inspector; it produces the overlay geometry
// and status text, but does not touch windows, the renderer or the main world.
class SweepInspector {
public:
    SweepInspector(const Planet& planet, const SweepOptions& options, std::uint64_t seed);

    // Bind the live backend and remember the current method. The mesh reference
    // is only built when the active backend is a field (Mesh has nothing to
    // compare against itself).
    void attachBackend(World& backend, Method method, bool morton, double density);

    // Build the mesh reference, run the warm-up and measured validation passes,
    // and open on the worst cast (or the frame named by the options).
    void begin(const std::vector<Vec>& invokers, Vec cameraEye);
    // Drop the mesh reference and the validated set when leaving sweep mode.
    void clear();

    bool active() const { return active_; }
    Method method() const { return method_; }

    // Re-sync the mesh reference to the terrain just published and re-run the
    // cast currently on screen; refocus and refresh the overlay accordingly.
    void onTerrainPublished(bool force, const WorldStats& stats,
                            const std::vector<Vec>& invokers, Vec cameraEye);

    // Navigation through the validated cast set.
    void nextCast();
    void prevCast();
    void nextMismatch();
    void nextWorstError();
    void nextWorstPoint();
    void jumpFrame();
    // Re-run the whole validation pass against the terrain as it is now.
    void revalidate();

    bool focusValid() const { return focusValid_; }
    Vec focus() const { return focus_; }
    double focusDistance() const { return focusDistance_; }

    // Overlay geometry: sweep path, contact markers and coordinate labels.
    void appendOverlay(std::vector<Vertex>& lines, const Camera& view) const;
    // Cast shape drawn at the start pose and at each backend's stopping pose.
    void appendShapes(std::vector<Vertex>& objects) const;

    // Status text rendered into the viewer footer.
    std::wstring statusLine(const std::array<const wchar_t*, 4>& methodLabels, std::size_t triangles) const;
    std::wstring castLine() const;
    std::wstring totalsLine() const;
    std::wstring controlsHint() const;

    const SweepTally& tally() const { return tally_; }
    const std::vector<SweepCast>& casts() const { return casts_; }
    std::size_t castIndex() const { return castIndex_; }
    std::size_t castCount() const { return casts_.size(); }

private:
    Shape castShape(Vec anchor, int index, std::mt19937_64& rng) const;
    std::pair<Vec, Vec> castPlacement(Vec anchor, const Shape& shape, std::mt19937_64& rng) const;
    void fillFrame(int frame, std::uint64_t salt, std::vector<SweepCast>& casts) const;
    void validateSweep(const std::vector<Vec>& invokers);
    void ensureMesh(const std::vector<Vec>& invokers, Vec eye);
    void resweepCurrent();
    void focusCurrent();
    void stepCast(int delta);
    // Locate the current cast inside the TOI-error ordering after a direct jump.
    void syncOrderPos();

    const Planet& planet_;
    SweepOptions opts_;
    std::uint64_t seed_{};
    World* backend_{};
    bool morton_{};
    double density_{1.0};
    Method method_{Method::Grid};
    std::unique_ptr<World> mesh_;
    std::vector<SweepCast> casts_;
    SweepTally tally_;
    std::size_t castIndex_{};
    std::size_t invokerCount_{};
    // Cast indices ordered by TOI error, largest first; nextCast walks this list
    // so each press shows a progressively smaller TOI error.
    std::vector<std::size_t> orderByError_;
    std::size_t orderPos_{};
    std::vector<Vec> invokers_;
    Vec focus_{};
    bool focusValid_{};
    double focusDistance_{};
    bool active_{};
};

}  // namespace voxel
