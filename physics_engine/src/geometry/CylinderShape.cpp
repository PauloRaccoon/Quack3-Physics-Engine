// =============================================================================
// CylinderShape.cpp  (V18)
// =============================================================================
#include "physics/geometry/CylinderShape.h"

#include <cmath>
#include <stdexcept>

namespace physics {

namespace {
constexpr Real kCapAlign = Real(0.5);    // |d.y| >= isto: a tampa é a face mais alinhada
constexpr Real kSideAlign = Real(0.4);   // |d.y| <  isto: a geratriz (linha) é a feature mais alinhada
static_assert(CylinderShape::kCapSegments <= Shape::kMaxFaceVertices, "a tampa precisa caber em kMaxFaceVertices");
} // namespace

CylinderShape::CylinderShape(Real radius, Real halfHeight) : radius_(radius), halfHeight_(halfHeight) {
    if (!std::isfinite(radius) || !(radius >= kMinShapeExtent) || !(radius <= kMaxShapeExtent) || !std::isfinite(halfHeight) ||
        !(halfHeight >= kMinShapeExtent) || !(halfHeight <= kMaxShapeExtent)) {
        throw std::invalid_argument("CylinderShape: raio e meia-altura devem estar em [kMinShapeExtent, kMaxShapeExtent]");
    }
}

Vector3 CylinderShape::getSupport(const Vector3& d) const {
    const Real y = d.y >= Real(0) ? halfHeight_ : -halfHeight_;
    const Real lenXZ = std::sqrt(d.x * d.x + d.z * d.z);
    if (!(lenXZ > Real(1e-12))) return Vector3(radius_, y, Real(0));   // direção ao longo do eixo: qualquer ponto da borda da tampa
    return Vector3(radius_ * d.x / lenXZ, y, radius_ * d.z / lenXZ);
}

int CylinderShape::getSupportFace(const Vector3& d, Vector3* out, int maxVertices) const {
    const Real len = d.length();
    if (!(len > Real(1e-12)) || std::abs(d.y) < kCapAlign * len || maxVertices < kCapSegments) return 0;   // lado curvo: sem face plana
    const bool top = d.y > Real(0);
    const Real y = top ? halfHeight_ : -halfHeight_;
    // Anti-horário visto de fora (normal ±Y): em +Y o sentido é o de +X → −Z; em −Y, o oposto.
    for (int i = 0; i < kCapSegments; ++i) {
        const Real a = kTwoPi * Real(i) / Real(kCapSegments);
        const Real s = top ? -std::sin(a) : std::sin(a);
        out[i] = Vector3(radius_ * std::cos(a), y, radius_ * s);
    }
    return kCapSegments;
}

bool CylinderShape::getSupportSegment(const Vector3& d, Vector3& p0, Vector3& p1) const {
    const Real len = d.length();
    const Real lenXZ = std::sqrt(d.x * d.x + d.z * d.z);
    if (!(len > Real(1e-12)) || !(lenXZ > Real(1e-9)) || std::abs(d.y) >= kSideAlign * len) return false;
    const Real cx = radius_ * d.x / lenXZ, cz = radius_ * d.z / lenXZ;
    p0 = Vector3(cx, -halfHeight_, cz);
    p1 = Vector3(cx, halfHeight_, cz);
    return true;
}

AABB CylinderShape::computeAABB(const Transform& xf) const {
    // Extensão em cada eixo do mundo: h·|a| + r·sqrt(1 − a²), com a = componente do eixo Y local nesse eixo.
    const Vector3 axis = xf.transformDirection(Vector3::unitY());
    auto ext = [&](Real a) {
        const Real a2 = std::min(Real(1), a * a);
        return halfHeight_ * std::abs(a) + radius_ * std::sqrt(Real(1) - a2);
    };
    return AABB::fromCenterHalfExtents(xf.position, Vector3(ext(axis.x), ext(axis.y), ext(axis.z)));
}

Real CylinderShape::getVolume() const { return kPi * radius_ * radius_ * Real(2) * halfHeight_; }

Matrix3 CylinderShape::computeInertia(Real mass) const {
    const Real h = Real(2) * halfHeight_;
    const Real iAxis = Real(0.5) * mass * radius_ * radius_;
    const Real iPerp = mass * (Real(3) * radius_ * radius_ + h * h) / Real(12);
    return Matrix3::diagonal(iPerp, iAxis, iPerp);
}

} // namespace physics
