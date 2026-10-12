// =============================================================================
// Transform.h
// Transformação rígida (rotação + translação):  p' = R p + t.
// =============================================================================
#pragma once

#include <iosfwd>

#include "physics/common/Settings.h"
#include "physics/math/Matrix3.h"
#include "physics/math/Quaternion.h"
#include "physics/math/Vector3.h"

namespace physics {

struct Transform {
    Vector3 position;     ///< translação
    Quaternion rotation;  ///< orientação (unitária)

    Transform() = default;
    Transform(const Vector3& p, const Quaternion& q) : position(p), rotation(q) {}
    explicit Transform(const Vector3& p) : position(p) {}

    static Transform identity() { return Transform(); }

    // --- Aplicação ------------------------------------------------------------------
    /// Ponto local → mundo.
    Vector3 transformPoint(const Vector3& local) const { return rotation.rotate(local) + position; }

    /// Direção local → mundo (ignora translação).
    Vector3 transformDirection(const Vector3& local) const { return rotation.rotate(local); }

    /// Ponto mundo → local.
    Vector3 inverseTransformPoint(const Vector3& world) const {
        return rotation.rotateInverse(world - position);
    }

    /// Direção mundo → local.
    Vector3 inverseTransformDirection(const Vector3& world) const {
        return rotation.rotateInverse(world);
    }

    /// Matriz de rotação 3x3 equivalente.
    Matrix3 rotationMatrix() const { return rotation.toMatrix(); }

    // --- Álgebra de transformações --------------------------------------------------------
    /// Transformação inversa: (R, t)⁻¹ = (R⁻¹, -R⁻¹ t).
    Transform inverse() const {
        const Quaternion qi = rotation.conjugate();
        return Transform(qi.rotate(-position), qi);
    }

    /// Composição: (a * b) aplica primeiro b e depois a.
    friend Transform operator*(const Transform& a, const Transform& b) {
        return Transform(a.rotation.rotate(b.position) + a.position,
                         (a.rotation * b.rotation).normalized());
    }

    /// Interpolação (lerp na posição, slerp na orientação). `alpha` em [0,1].
    /// Usada pela camada de renderização (MotionState).
    static Transform interpolate(const Transform& a, const Transform& b, Real alpha);

    bool isFinite() const { return position.isFinite() && rotation.isFinite(); }

    bool nearlyEqual(const Transform& o, Real eps = kCompareEpsilon) const {
        return position.nearlyEqual(o.position, eps) && rotation.nearlyEqual(o.rotation, eps);
    }
};

std::ostream& operator<<(std::ostream& os, const Transform& t);

} // namespace physics
