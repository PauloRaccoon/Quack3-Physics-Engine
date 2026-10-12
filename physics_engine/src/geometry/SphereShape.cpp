// =============================================================================
// SphereShape.cpp
// =============================================================================
#include "physics/geometry/SphereShape.h"

#include <stdexcept>

namespace physics {

SphereShape::SphereShape(Real radius) : radius_(radius) {
    if (!std::isfinite(radius) || !(radius > Real(0))) {
        throw std::invalid_argument("SphereShape: o raio deve ser finito e positivo");
    }
}

Vector3 SphereShape::getSupport(const Vector3& direction) const {
    // Direção nula: qualquer ponto da superfície serve.
    return direction.normalizedOr(Vector3::unitX()) * radius_;
}

AABB SphereShape::computeAABB(const Transform& xf) const {
    // A esfera é invariante à rotação: AABB = centro ± raio.
    return AABB::fromCenterHalfExtents(xf.position, Vector3(radius_));
}

Real SphereShape::getVolume() const {
    return Real(4) / Real(3) * kPi * radius_ * radius_ * radius_;
}

Matrix3 SphereShape::computeInertia(Real mass) const {
    // Esfera maciça: I = 2/5 m r² (isotrópico).
    const Real i = Real(0.4) * mass * radius_ * radius_;
    return Matrix3::diagonal(i, i, i);
}

} // namespace physics
