// =============================================================================
// MassData.h
// Propriedades de massa de um corpo rígido: massa, centro de massa e tensor
// de inércia (e seus inversos). Massa infinita (corpos estáticos/cinemáticos)
// é representada por invMass = 0 e invInertia = 0 — assim as fórmulas do
// solver funcionam sem casos especiais.
// =============================================================================
#pragma once

#include "physics/common/Settings.h"
#include "physics/geometry/Shape.h"
#include "physics/math/Matrix3.h"
#include "physics/math/Vector3.h"

namespace physics {

struct MassData {
    Real mass = Real(0);
    Real invMass = Real(0);
    Vector3 centerOfMass;                       ///< em coordenadas locais do corpo
    Matrix3 inertia = Matrix3::zero();          ///< em torno do centro de massa, eixos locais
    Matrix3 invInertia = Matrix3::zero();

    /// Massa infinita (corpo imóvel).
    static MassData infinite() { return MassData(); }

    /// Massa = densidade × volume da forma.
    /// @throws std::invalid_argument se densidade ou volume não forem positivos e finitos.
    static MassData fromShape(const Shape& shape, Real density);

    /// Massa explícita distribuída uniformemente pelo volume da forma.
    /// @throws std::invalid_argument se a massa não for positiva e finita.
    static MassData fromShapeMass(const Shape& shape, Real mass);

    /// Valores explícitos. `inertiaAtCenterOfMass` em eixos locais.
    /// Se o tensor for singular, invInertia = 0 (o corpo não rotaciona).
    /// @throws std::invalid_argument se a massa não for positiva e finita.
    static MassData fromMassAndInertia(Real mass, const Matrix3& inertiaAtCenterOfMass,
                                       const Vector3& centerOfMass = Vector3::zero());

    bool hasFiniteMass() const { return invMass > Real(0); }
};

} // namespace physics
