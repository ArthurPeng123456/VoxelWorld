#pragma once

#include "terrain.hpp"

namespace voxel {

// Independent double-precision flight camera. The entry view defines its frame,
// so switching from an orbital or surface camera never changes the initial view.
class FreeCamera {
public:
    void reset(Vec eye, Vec forward, Vec up) {
        position_ = eye;
        initialForward_ = unit(forward);
        initialRight_ = unit(cross(initialForward_, up));
        initialUp_ = unit(cross(initialRight_, initialForward_));
        yaw_ = pitch_ = 0;
    }

    Vec position() const { return position_; }
    Vec right() const { return initialRight_ * std::cos(yaw_) - initialForward_ * std::sin(yaw_); }
    Vec forward() const {
        return (initialForward_ * std::cos(yaw_) + initialRight_ * std::sin(yaw_)) * std::cos(pitch_)
             + initialUp_ * std::sin(pitch_);
    }
    Vec up() const { return unit(cross(right(), forward())); }

    void look(double rightRadians, double upRadians) {
        constexpr double pi = 3.14159265358979323846;
        yaw_ = std::remainder(yaw_ + rightRadians, 2 * pi);
        pitch_ = std::clamp(pitch_ + upRadians, -pi * 89 / 180, pi * 89 / 180);
    }

    void move(double ahead, double sideways, double vertical, double seconds, double speed) {
        Vec direction = forward() * ahead + right() * sideways + up() * vertical;
        const double magnitude = length(direction);
        if (magnitude > 1) direction = direction / magnitude;
        position_ = position_ + direction * (seconds * speed);
    }

private:
    Vec position_{}, initialForward_{0, 0, -1}, initialRight_{1, 0, 0}, initialUp_{0, 1, 0};
    double yaw_{}, pitch_{};
};

} // namespace voxel
