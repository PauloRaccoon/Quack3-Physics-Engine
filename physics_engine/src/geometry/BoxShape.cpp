// =============================================================================
// BoxShape.cpp
// =============================================================================
#include "physics/geometry/BoxShape.h"

#include <stdexcept>

namespace physics {

BoxShape::BoxShape(const Vector3& halfExtents) : halfExtents_(halfExtents) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(halfExtents[i]) || !(halfExtents[i] > Real(0))) {
            throw std::invalid_argument("BoxShape: semi-extents devem ser finitos e positivos");
        }
    }
}

Vector3 BoxShape::getSupport(const Vector3& d) const {
    // O vértice da caixa mais distante em d tem, em cada eixo, o sinal de d.
    // (Empate/zero escolhe o lado positivo: resultado determinístico.)
    return Vector3(d.x >= Real(0) ? halfExtents_.x : -halfExtents_.x,
                   d.y >= Real(0) ? halfExtents_.y : -halfExtents_.y,
                   d.z >= Real(0) ? halfExtents_.z : -halfExtents_.z);
}

FeatureId BoxShape::getSupportFeature(const Vector3& d) const {
    return static_cast<FeatureId>((d.x >= Real(0) ? 1u : 0u) | (d.y >= Real(0) ? 2u : 0u) |
                                  (d.z >= Real(0) ? 4u : 0u));
}

AABB BoxShape::computeAABB(const Transform& xf) const {
    // Fórmula de Arvo: semi-extent no mundo = |R| * halfExtents.
    return AABB(-halfExtents_, halfExtents_).transformed(xf);
}

Real BoxShape::getVolume() const { return Real(8) * halfExtents_.x * halfExtents_.y * halfExtents_.z; }

Matrix3 BoxShape::computeInertia(Real mass) const {
    // Caixa maciça de lados (2hx, 2hy, 2hz):  Ixx = m/12 (ly² + lz²) = m/3 (hy² + hz²)
    const Vector3 h2(halfExtents_.x * halfExtents_.x, halfExtents_.y * halfExtents_.y,
                     halfExtents_.z * halfExtents_.z);
    const Real k = mass / Real(3);
    return Matrix3::diagonal(k * (h2.y + h2.z), k * (h2.x + h2.z), k * (h2.x + h2.y));
}

} // namespace physics
