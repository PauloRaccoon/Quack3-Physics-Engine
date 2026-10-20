// =============================================================================
// TimeControl.cpp
// =============================================================================
#include "viewer/core/TimeControl.h"

#include <cmath>

namespace viewer {

namespace {
constexpr float kScales[TimeControl::kScaleCount] = {1.0f, 0.5f, 0.25f, 0.1f};
constexpr const char* kLabels[TimeControl::kScaleCount] = {"1x", "1/2", "1/4", "1/10"};
int clampIndex(int i) { return i < 0 ? 0 : (i >= TimeControl::kScaleCount ? TimeControl::kScaleCount - 1 : i); }
} // namespace

float TimeControl::scaleValue(int index) { return kScales[clampIndex(index)]; }
const char* TimeControl::scaleLabel(int index) { return kLabels[clampIndex(index)]; }

void TimeControl::setScaleIndex(int index) { scaleIndex_ = clampIndex(index); }

int TimeControl::advance(physics::PhysicsWorld& world, physics::Real frameTime) {
    if (!std::isfinite(frameTime) || frameTime < physics::Real(0)) return 0;
    if (paused_) {
        if (!stepRequested_) return 0;
        stepRequested_ = false;
        world.step();
        forceAlphaOne_ = true;
        return 1;
    }
    forceAlphaOne_ = false;
    return world.update(frameTime * static_cast<physics::Real>(timeScale()));
}

physics::Real TimeControl::renderAlpha(const physics::PhysicsWorld& world) const {
    return forceAlphaOne_ ? physics::Real(1) : world.getAlpha();
}

} // namespace viewer
