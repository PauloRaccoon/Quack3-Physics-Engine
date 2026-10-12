// =============================================================================
// Quaternion.cpp
// =============================================================================
#include "physics/math/Quaternion.h"

#include <ostream>

namespace physics {

Quaternion Quaternion::fromAxisAngle(const Vector3& axis, Real angle) {
    const Vector3 n = axis.normalized();
    if (n.isZero(Real(0))) return identity();  // eixo degenerado
    const Real half = angle * Real(0.5);
    const Real s = std::sin(half);
    return Quaternion(std::cos(half), n.x * s, n.y * s, n.z * s);
}

Quaternion Quaternion::fromRotationVector(const Vector3& rotation) {
    const Real angle = rotation.length();
    const Real half = angle * Real(0.5);
    // sin(θ/2)/θ com série de Taylor perto de zero (evita 0/0).
    const Real k = (angle < Real(1e-3)) ? Real(0.5) - angle * angle * Real(1.0 / 48.0)
                                        : std::sin(half) / angle;
    return Quaternion(std::cos(half), rotation.x * k, rotation.y * k, rotation.z * k).normalized();
}

Quaternion Quaternion::fromTwoVectors(const Vector3& from, const Vector3& to) {
    const Vector3 a = from.normalized();
    const Vector3 b = to.normalized();
    if (a.isZero(Real(0)) || b.isZero(Real(0))) return identity();

    const Real d = a.dot(b);
    if (d < Real(-1) + Real(1e-6)) {
        // Vetores opostos: gira π em torno de qualquer eixo perpendicular.
        Vector3 t1, t2;
        computeOrthonormalBasis(a, t1, t2);
        return Quaternion(0, t1.x, t1.y, t1.z);
    }
    // q = (1 + a·b, a × b) normalizado  (identidade do ângulo-metade).
    const Vector3 c = a.cross(b);
    return Quaternion(Real(1) + d, c.x, c.y, c.z).normalized();
}

Quaternion Quaternion::fromMatrix(const Matrix3& r) {
    // Método de Shepperd: escolhe o maior entre {traço, m00, m11, m22} como
    // pivô, o que evita raiz de valores próximos de zero (instabilidade).
    Quaternion q;
    const Real trace = r.trace();
    if (trace > Real(0)) {
        const Real s = std::sqrt(trace + Real(1)) * Real(2);  // s = 4w
        q.w = Real(0.25) * s;
        q.x = (r(2, 1) - r(1, 2)) / s;
        q.y = (r(0, 2) - r(2, 0)) / s;
        q.z = (r(1, 0) - r(0, 1)) / s;
    } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
        const Real s = std::sqrt(Real(1) + r(0, 0) - r(1, 1) - r(2, 2)) * Real(2);  // 4x
        q.w = (r(2, 1) - r(1, 2)) / s;
        q.x = Real(0.25) * s;
        q.y = (r(0, 1) + r(1, 0)) / s;
        q.z = (r(0, 2) + r(2, 0)) / s;
    } else if (r(1, 1) > r(2, 2)) {
        const Real s = std::sqrt(Real(1) + r(1, 1) - r(0, 0) - r(2, 2)) * Real(2);  // 4y
        q.w = (r(0, 2) - r(2, 0)) / s;
        q.x = (r(0, 1) + r(1, 0)) / s;
        q.y = Real(0.25) * s;
        q.z = (r(1, 2) + r(2, 1)) / s;
    } else {
        const Real s = std::sqrt(Real(1) + r(2, 2) - r(0, 0) - r(1, 1)) * Real(2);  // 4z
        q.w = (r(1, 0) - r(0, 1)) / s;
        q.x = (r(0, 2) + r(2, 0)) / s;
        q.y = (r(1, 2) + r(2, 1)) / s;
        q.z = Real(0.25) * s;
    }
    q.normalize();
    return q;
}

Matrix3 Quaternion::toMatrix() const {
    const Real xx = x * x, yy = y * y, zz = z * z;
    const Real xy = x * y, xz = x * z, yz = y * z;
    const Real wx = w * x, wy = w * y, wz = w * z;
    return Matrix3(Real(1) - Real(2) * (yy + zz), Real(2) * (xy - wz),        Real(2) * (xz + wy),
                   Real(2) * (xy + wz),        Real(1) - Real(2) * (xx + zz), Real(2) * (yz - wx),
                   Real(2) * (xz - wy),        Real(2) * (yz + wx),        Real(1) - Real(2) * (xx + yy));
}

void Quaternion::toAxisAngle(Vector3& axis, Real& angle) const {
    // Garante w >= 0 para obter o ângulo em [0, π] (caminho mais curto).
    const Quaternion q = (w < Real(0)) ? -(*this) : *this;
    const Vector3 v = q.vec();
    const Real s = v.length();
    if (s < Real(1e-8)) {
        axis = Vector3::unitX();
        angle = Real(0);
        return;
    }
    axis = v / s;
    angle = Real(2) * std::atan2(s, q.w);
}

Quaternion Quaternion::nlerp(const Quaternion& a, const Quaternion& b, Real t) {
    // Se o produto escalar é negativo, b está do "outro lado" da hiperesfera.
    const Quaternion bb = (a.dot(b) < Real(0)) ? -b : b;
    return (a * (Real(1) - t) + bb * t).normalized();
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, Real t) {
    Real d = a.dot(b);
    Quaternion bb = b;
    if (d < Real(0)) {  // caminho mais curto
        bb = -b;
        d = -d;
    }
    // Muito próximos: sin(θ) ≈ 0 tornaria a fórmula instável → usa nlerp.
    constexpr Real kLinearThreshold = Real(0.9995);
    if (d > kLinearThreshold) return nlerp(a, bb, t);

    const Real theta = safeAcos(d);
    const Real invSin = Real(1) / std::sin(theta);
    const Real wa = std::sin((Real(1) - t) * theta) * invSin;
    const Real wb = std::sin(t * theta) * invSin;
    return (a * wa + bb * wb).normalized();
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    return os << "(w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z << ')';
}

} // namespace physics
