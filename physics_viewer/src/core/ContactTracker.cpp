// =============================================================================
// ContactTracker.cpp
// =============================================================================
#include "viewer/core/ContactTracker.h"

#include <algorithm>
#include <cmath>

namespace viewer {

namespace {
float clamp01(float v) { return std::isfinite(v) ? std::min(std::max(v, 0.0f), 1.0f) : 0.0f; }
unsigned char mix(unsigned char a, float b, float t) {
    const float v = static_cast<float>(a) * (1.0f - t) + b * t;
    return static_cast<unsigned char>(std::min(std::max(v + 0.5f, 0.0f), 255.0f));
}
} // namespace

Rgb stateTint(Rgb base, bool sleeping, float flash) {
    Rgb c = base;
    if (sleeping) {   // adormecido: apaga a cor e puxa para o azul
        c.r = mix(c.r, 60.0f, 0.55f);
        c.g = mix(c.g, 90.0f, 0.55f);
        c.b = mix(c.b, 175.0f, 0.55f);
    }
    const float f = clamp01(flash) * 0.9f;   // flash: branco-amarelado
    c.r = mix(c.r, 255.0f, f);
    c.g = mix(c.g, 250.0f, f);
    c.b = mix(c.b, 190.0f, f);
    return c;
}

float ContactTracker::intensityFromSpeed(float s) {
    if (!std::isfinite(s) || s < kMinSpeed) return 0.0f;
    return clamp01(s / kFullSpeed);
}

void ContactTracker::addFlash(physics::BodyId id, float intensity) {
    for (auto& f : flashes_)
        if (f.first == id) {
            f.second = std::max(f.second, intensity);
            return;
        }
    flashes_.emplace_back(id, intensity);
}

void ContactTracker::onContactBegin(const physics::ContactEvent& e) {
    ++begins_;
    const float speed = static_cast<float>(e.approachSpeed);
    const float k = intensityFromSpeed(speed);
    if (k <= 0.0f) return;
    lastImpact_ = speed;
    maxImpact_ = std::max(maxImpact_, speed);
    addFlash(e.a, k);
    addFlash(e.b, k);
    if (markers_.size() >= kMaxMarkers) markers_.erase(markers_.begin());   // descarta o mais antigo
    Marker m;
    m.point = e.point;
    m.intensity = k;
    markers_.push_back(m);
}

void ContactTracker::onContactEnd(const physics::ContactEvent&) { ++ends_; }

void ContactTracker::update(float dt) {
    if (!(dt > 0.0f) || !std::isfinite(dt)) return;
    const float dec = dt / kFlashSeconds;
    for (auto& f : flashes_) f.second -= dec;
    flashes_.erase(std::remove_if(flashes_.begin(), flashes_.end(), [](const auto& f) { return f.second <= 0.0f; }), flashes_.end());
    for (Marker& m : markers_) m.age += dt;
    markers_.erase(std::remove_if(markers_.begin(), markers_.end(), [](const Marker& m) { return m.age >= kFlashSeconds; }), markers_.end());
}

void ContactTracker::reset() {
    flashes_.clear();
    markers_.clear();
    begins_ = ends_ = 0;
    lastImpact_ = maxImpact_ = 0.0f;
}

float ContactTracker::flashOf(physics::BodyId id) const {
    for (const auto& f : flashes_)
        if (f.first == id) return clamp01(f.second);
    return 0.0f;
}

} // namespace viewer
