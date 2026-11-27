#pragma once

#include "terrain.hpp"

namespace voxel {

constexpr int maxInvokerCount = 10000;
enum class InvokerCruiseMode { Area, GreatCircle };

struct InvokerSettings {
    int count{2};
    InvokerCruiseMode mode{InvokerCruiseMode::Area};
    double patrolRadius{1792};
    double cruiseSpeed{1.5};
    double returnSpeed{3};
    double controlSpeed{4.5};
};

template<class Text> int parseInvokerCount(const Text& text) {
    int count = 0;
    if (!text.empty()) {
        for (auto digit : text) {
            if (digit < '0' || digit > '9' || count > (maxInvokerCount - int(digit - '0')) / 10)
                throw std::invalid_argument("Invoker count must be an integer from 1 to 10000");
            count = count * 10 + int(digit - '0');
        }
        if (count >= 1 && count <= maxInvokerCount) return count;
    }
    throw std::invalid_argument("Invoker count must be an integer from 1 to 10000");
}

inline bool validInvokerSettings(const InvokerSettings& settings) {
    const auto bounded = [](double value, double lo, double hi) {
        return std::isfinite(value) && value >= lo && value <= hi;
    };
    return settings.count >= 1 && settings.count <= maxInvokerCount &&
        (settings.mode == InvokerCruiseMode::Area || settings.mode == InvokerCruiseMode::GreatCircle) &&
        bounded(settings.patrolRadius, 1, 1000000) && bounded(settings.cruiseSpeed, 0, 1000) &&
        bounded(settings.returnSpeed, 0, 1000) && bounded(settings.controlSpeed, 0.1, 1000);
}

template<class Text> double parseInvokerNumber(const Text& text) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("Expected a finite number");
    return value;
}

inline double adjustedInvokerControlSpeed(double speed, bool increase) {
    return std::clamp(speed * (increase ? 1.25 : 0.8), 0.1, 1000.0);
}

} // namespace voxel
