// =============================================================================
// OrbitCamera.cpp
// =============================================================================
#include "viewer/core/OrbitCamera.h"

#include <algorithm>
#include <cmath>

namespace viewer {

namespace {
bool finite(float v) { return std::isfinite(v); }
} // namespace

void OrbitCamera::rotate(float dYaw, float dPitch) {
    if (!finite(dYaw) || !finite(dPitch)) return;
    yaw_ += dYaw;
    // Mantém o yaw em [-π, π] para não crescer sem limite depois de muitas voltas.
    const float twoPi = 6.28318530718f;
    yaw_ = std::fmod(yaw_, twoPi);
    if (yaw_ > 3.14159265359f) yaw_ -= twoPi;
    if (yaw_ < -3.14159265359f) yaw_ += twoPi;
    pitch_ = std::min(kMaxPitch, std::max(kMinPitch, pitch_ + dPitch));
}

void OrbitCamera::zoom(float wheel) {
    if (!finite(wheel)) return;
    distance_ *= std::pow(0.9f, wheel);
    distance_ = std::min(kMaxDistance, std::max(kMinDistance, distance_));
}

void OrbitCamera::setTarget(const Vec3f& t) {
    if (finite(t.x) && finite(t.y) && finite(t.z)) target = t;
}

void OrbitCamera::setDistance(float d) {
    if (finite(d)) distance_ = std::min(std::max(d, kMinDistance), kMaxDistance);
}

Vec3f OrbitCamera::position() const {
    const float cp = std::cos(pitch_);
    return Vec3f{target.x + distance_ * cp * std::sin(yaw_),
                 target.y + distance_ * std::sin(pitch_),
                 target.z + distance_ * cp * std::cos(yaw_)};
}

} // namespace viewer
