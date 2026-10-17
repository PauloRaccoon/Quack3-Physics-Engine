// =============================================================================
// IPhysicsTransformBridge.cpp
// =============================================================================
#include "physics/bridge/IPhysicsTransformBridge.h"

namespace physics {

void BufferedTransformBridge::write(BodyId id, const Transform& t) {
    if (id == kInvalidBodyId) return;
    if (id >= alive_.size()) {
        const std::size_t oldSize = alive_.size();
        alive_.resize(static_cast<std::size_t>(id) + 1, 0);
        data_.resize((static_cast<std::size_t>(id) + 1) * kFloatsPerBody, 0.0f);
        // Só os slots NOVOS começam com a orientação identidade (w = 1), não com zeros.
        for (std::size_t i = oldSize; i < alive_.size(); ++i) data_[i * kFloatsPerBody + 6] = 1.0f;
    }
    float* p = &data_[static_cast<std::size_t>(id) * kFloatsPerBody];
    p[0] = static_cast<float>(t.position.x);
    p[1] = static_cast<float>(t.position.y);
    p[2] = static_cast<float>(t.position.z);
    p[3] = static_cast<float>(t.rotation.x);
    p[4] = static_cast<float>(t.rotation.y);
    p[5] = static_cast<float>(t.rotation.z);
    p[6] = static_cast<float>(t.rotation.w);
    alive_[id] = 1;
}

void BufferedTransformBridge::onBodyCreated(BodyId id, const Transform& transform) { write(id, transform); }

void BufferedTransformBridge::onBodyDestroyed(BodyId id) {
    if (id < alive_.size()) alive_[id] = 0;
}

void BufferedTransformBridge::beginSync(Real alpha) {
    lastAlpha_ = alpha;
    ++syncCount_;
}

void BufferedTransformBridge::onTransformUpdated(BodyId id, const Transform& interpolated) {
    write(id, interpolated);
}

Transform BufferedTransformBridge::getTransform(BodyId id) const {
    Transform t;
    if (id >= alive_.size()) return t;
    const float* p = &data_[static_cast<std::size_t>(id) * kFloatsPerBody];
    t.position = Vector3(Real(p[0]), Real(p[1]), Real(p[2]));
    t.rotation = Quaternion(Real(p[6]), Real(p[3]), Real(p[4]), Real(p[5]));
    return t;
}

} // namespace physics
