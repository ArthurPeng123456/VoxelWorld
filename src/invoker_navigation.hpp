#pragma once

#include "invoker_settings.hpp"
#include "collision.hpp"

namespace voxel {

// Stable, count-independent low-discrepancy directions, in antipodal pairs.
// Growing the population never relocates existing Invokers or repeats craters.
inline Vec initialInvokerDirection(std::size_t index, std::uint64_t seed) {
    constexpr double tau = 6.2831853071795864769, golden = 0.6180339887498948482;
    std::size_t bits = index / 2 + 1;
    double z = 0, weight = 0.5;
    while (bits) { z += (bits & 1) * weight; bits >>= 1; weight *= 0.5; }
    const double azimuth = tau * std::fmod((double(index / 2) + double(seed % 1000003)) * golden, 1.0);
    const double r = std::sqrt(std::max(0.0, 1 - z * z));
    return Vec{r * std::cos(azimuth), r * std::sin(azimuth), z} * (index % 2 ? -1 : 1);
}

constexpr double invokerGroundClearance = 0.05;
struct InvokerGround {
    Triangle support{};
    Key owner{};
    std::uint64_t revision{};
    bool ready{};
};

// Query published render geometry, not just the analytic height or a different
// zero-surface approximation. Reuse a supporting triangle until it is exited
// or its chunk is replaced. Never read the worker's unpublished chunks.
inline bool groundInvoker(Vec& position, const Chunks& chunks, TerrainElevationRange range, InvokerGround& ground) {
    const Vec direction = unit(position);
    const Vec from = direction * (planetRadius + range.upper + 1024);
    const Vec to = direction * (planetRadius + range.lower - 65536);
    Hit hit;
    const auto owner = ground.ready ? chunks.find(ground.owner) : chunks.end();
    if (owner != chunks.end() && owner->second->revision == ground.revision)
        hit = triangleRay(ground.support, from, to);
    if (hit.state != Hit::State::Contact) {
        hit = meshRay(chunks, from, to, &ground.support, &ground.owner);
        if (hit.state == Hit::State::Contact) ground.revision = chunks.at(ground.owner)->revision;
    }
    ground.ready = hit.state == Hit::State::Contact;
    if (ground.ready) position = hit.position + direction * invokerGroundClearance;
    return ground.ready;
}

class InvokerRoute {
public:
    void reset(Vec position, int index, Vec heading = {}) {
        anchor_ = unit(position);
        const Vec reference = std::abs(anchor_.z) < 0.9 ? Vec{0, 0, 1} : Vec{0, 1, 0};
        const Vec fallback = unit(cross(reference, anchor_)) * (index % 2 ? -1 : 1);
        tangent_ = unit(heading - anchor_ * dot(anchor_, heading), fallback);
        phase_ = 0;
    }

    void advance(double seconds, const InvokerSettings& settings) {
        constexpr double tau = 6.2831853071795864769;
        const double radius = settings.mode == InvokerCruiseMode::Area ? settings.patrolRadius : planetRadius;
        phase_ = std::remainder(phase_ + seconds * settings.cruiseSpeed / radius, tau);
    }

    Vec direction(const InvokerSettings& settings) const {
        const double angle = settings.mode == InvokerCruiseMode::Area
            ? settings.patrolRadius / planetRadius * std::sin(phase_) : phase_;
        return unit(anchor_ * std::cos(angle) + tangent_ * std::sin(angle));
    }

    Vec heading(const InvokerSettings& settings) const {
        const Vec normal = unit(cross(anchor_, tangent_));
        const double sign = settings.mode == InvokerCruiseMode::Area && std::cos(phase_) < 0 ? -1 : 1;
        return unit(cross(normal, direction(settings))) * sign;
    }

    double phase() const { return phase_; }

private:
    Vec anchor_{0, 0, 1}, tangent_{1, 0, 0};
    double phase_{};
};

struct InvokerAdvance { Vec position; bool arrived; };

inline InvokerAdvance advanceInvokerOnSurface(const Planet& planet, Vec position, Vec target, double meters) {
    const Vec up = unit(position), goal = unit(target);
    const double cosine = dot(up, goal);
    const Vec tangent = goal - up * cosine;
    const double angle = std::atan2(length(tangent), cosine);
    if (angle * length(position) <= meters) return {target, true};
    if (meters <= 0) return {position, false};
    const Vec reference = std::abs(up.z) < 0.9 ? Vec{0, 0, 1} : Vec{0, 1, 0};
    const Vec heading = unit(tangent, unit(cross(reference, up)));
    const double step = meters / length(position);
    return {planet.surface(unit(up * std::cos(step) + heading * std::sin(step))), false};
}

} // namespace voxel
