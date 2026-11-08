// =============================================================================
// CapsuleShape.cpp  (V18)
// =============================================================================
#include "physics/geometry/CapsuleShape.h"

#include <cmath>
#include <stdexcept>

namespace physics {

CapsuleShape::CapsuleShape(Real radius, Real halfHeight) : radius_(radius), halfHeight_(halfHeight) {
    if (!std::isfinite(radius) || !(radius >= kMinShapeExtent) || !(radius <= kMaxShapeExtent) || !std::isfinite(halfHeight) ||
        !(halfHeight >= Real(0)) || !(halfHeight <= kMaxShapeExtent)) {
        throw std::invalid_argument("CapsuleShape: raio em [kMinShapeExtent, kMaxShapeExtent] e meia-altura em [0, kMaxShapeExtent]");
    }
}

Vector3 CapsuleShape::getCoreSupport(const Vector3& d) const {
    return Vector3(Real(0), d.y >= Real(0) ? halfHeight_ : -halfHeight_, Real(0));
}

Vector3 CapsuleShape::getSupport(const Vector3& d) const {
    // ponta do segmento + raio na direção (direção nula: o ponto mais alto)
    return getCoreSupport(d) + d.normalizedOr(Vector3::unitY()) * radius_;
}

bool CapsuleShape::getSupportSegment(const Vector3&, Vector3& p0, Vector3& p1) const {
    p0 = Vector3(Real(0), -halfHeight_, Real(0));
    p1 = Vector3(Real(0), halfHeight_, Real(0));
    return halfHeight_ > Real(0);   // h = 0 é uma esfera: sem segmento
}

AABB CapsuleShape::computeAABB(const Transform& xf) const {
    // Extensão em cada eixo do mundo: r + h·|componente do eixo Y local no mundo|.
    const Vector3 axis = xf.transformDirection(Vector3::unitY());
    const Vector3 half(radius_ + halfHeight_ * std::abs(axis.x), radius_ + halfHeight_ * std::abs(axis.y),
                       radius_ + halfHeight_ * std::abs(axis.z));
    return AABB::fromCenterHalfExtents(xf.position, half);
}

Real CapsuleShape::getVolume() const {
    const Real r2 = radius_ * radius_;
    return kPi * r2 * (Real(2) * halfHeight_) + Real(4) / Real(3) * kPi * r2 * radius_;
}

Matrix3 CapsuleShape::computeInertia(Real mass) const {
    // Cilindro (comprimento L = 2h) + duas meias-esferas, densidade uniforme.
    const Real r = radius_, L = Real(2) * halfHeight_;
    const Real vc = kPi * r * r * L, vs = Real(4) / Real(3) * kPi * r * r * r;
    const Real v = vc + vs;
    const Real mc = mass * vc / v, ms = mass * vs / v;
    const Real iAxis = Real(0.5) * mc * r * r + Real(0.4) * ms * r * r;
    // eixos perpendiculares: cilindro + hemisférios deslocados (Steiner: centro de massa de cada um a L/2 + 3r/8)
    const Real iPerp = mc * (r * r / Real(4) + L * L / Real(12)) + ms * (Real(0.4) * r * r + L * L / Real(4) + Real(0.375) * L * r);
    return Matrix3::diagonal(iPerp, iAxis, iPerp);
}

} // namespace physics
