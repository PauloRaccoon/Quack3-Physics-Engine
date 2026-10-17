// =============================================================================
// MotionState.cpp
// =============================================================================
#include "physics/bridge/MotionState.h"

#include <cmath>
#include <stdexcept>

namespace physics {

FixedTimestepAccumulator::FixedTimestepAccumulator(Real fixedDt, int maxSubSteps)
    : fixedDt_(fixedDt), maxSubSteps_(maxSubSteps) {
    if (!std::isfinite(fixedDt) || !(fixedDt > Real(0))) {
        throw std::invalid_argument("FixedTimestepAccumulator: o passo fixo deve ser finito e positivo");
    }
    if (maxSubSteps < 1) {
        throw std::invalid_argument("FixedTimestepAccumulator: maxSubSteps deve ser >= 1");
    }
}

int FixedTimestepAccumulator::advance(Real frameTime) {
    if (!std::isfinite(frameTime) || !(frameTime > Real(0))) return 0;

    // Limita o quadro (anti "espiral da morte").
    const Real maxFrame = fixedDt_ * static_cast<Real>(maxSubSteps_);
    if (frameTime > maxFrame) {
        dropped_ += frameTime - maxFrame;
        frameTime = maxFrame;
    }

    accumulator_ += frameTime;
    int steps = 0;
    while (accumulator_ >= fixedDt_ && steps < maxSubSteps_) {
        accumulator_ -= fixedDt_;
        ++steps;
    }
    // Só sobra mais de um passo por arredondamento: descarta o excesso para manter α em [0,1).
    if (accumulator_ >= fixedDt_) {
        const Real rest = std::fmod(accumulator_, fixedDt_);
        dropped_ += accumulator_ - rest;
        accumulator_ = rest;
    }
    return steps;
}

} // namespace physics
