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

void OrbitCamera::reset(const Vec3f& newTarget, float newDistance) {
    yaw_ = kDefaultYaw;
    pitch_ = kDefaultPitch;
    setTarget(newTarget);
    setDistance(newDistance);
}

void OrbitCamera::followTarget(const Vec3f& goal, float dt, float rate) {
    if (!finite(dt) || !finite(rate) || !(dt > 0.0f) || !(rate > 0.0f)) return;
    if (!finite(goal.x) || !finite(goal.y) || !finite(goal.z)) return;
    const float f = 1.0f - std::exp(-rate * dt);
    target.x += (goal.x - target.x) * f;
    target.y += (goal.y - target.y) * f;
    target.z += (goal.z - target.z) * f;
}

Vec3f OrbitCamera::moveDelta(float forward, float right, float up, float dt, float boost) const {
    if (!std::isfinite(forward) || !std::isfinite(right) || !std::isfinite(up) || !std::isfinite(dt) || !std::isfinite(boost) || !(dt > 0.0f))
        return Vec3f{};
    const auto clamp1 = [](float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); };
    forward = clamp1(forward);
    right = clamp1(right);
    up = clamp1(up);
    if (dt > 0.1f) dt = 0.1f;                       // um quadro travado não "teletransporta" a câmera
    if (boost < 0.0f) boost = 0.0f;
    if (boost > 10.0f) boost = 10.0f;
    const float speed = (kMoveBase + kMovePerMeter * (distance_ - kMinDistance)) * boost;
    // Câmera em alvo + d·(cos p·sin y, sin p, cos p·cos y): olha para -(sin y, cos y) no chão; a direita da tela é (cos y, -sin y).
    const float sy = std::sin(yaw_), cy = std::cos(yaw_);
    const float k = speed * dt;
    return Vec3f{k * (-sy * forward + cy * right), k * up, k * (-cy * forward - sy * right)};
}

Vec3f OrbitCamera::position() const {
    const float cp = std::cos(pitch_);
    return Vec3f{target.x + distance_ * cp * std::sin(yaw_),
                 target.y + distance_ * std::sin(pitch_),
                 target.z + distance_ * cp * std::cos(yaw_)};
}

} // namespace viewer
