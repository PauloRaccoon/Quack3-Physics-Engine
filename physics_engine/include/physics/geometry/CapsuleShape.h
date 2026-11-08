// =============================================================================
// CapsuleShape.h  (V18)
// Cápsula: o segmento de (0, −h, 0) a (0, +h, 0) engrossado por um raio r (um cilindro com duas meias-esferas).
// Eixo = Y local. Comprimento total = 2h + 2r. Ideal para o corpo de um personagem.
//
// Colisão: GJK/EPA rodam no SEGMENTO (núcleo) e o raio é somado depois (ver Shape::getCoreRadius), então a
// superfície curva é exata. Com h = 0 a cápsula é uma esfera.
// =============================================================================
#pragma once

#include "physics/geometry/Shape.h"

namespace physics {

class CapsuleShape final : public Shape {
public:
    /// @param radius      raio [kMinShapeExtent, kMaxShapeExtent]
    /// @param halfHeight  meia-altura da parte cilíndrica (do centro ao centro da meia-esfera), [0, kMaxShapeExtent]
    /// @throws std::invalid_argument se algum valor não for finito ou estiver fora da faixa.
    CapsuleShape(Real radius, Real halfHeight);

    Real getRadius() const { return radius_; }
    Real getHalfHeight() const { return halfHeight_; }
    /// Altura total (ponta a ponta) = 2h + 2r.
    Real getTotalHeight() const { return Real(2) * (halfHeight_ + radius_); }

    ShapeType getType() const override { return ShapeType::Capsule; }
    Vector3 getSupport(const Vector3& direction) const override;
    Real getCoreRadius() const override { return radius_; }
    Vector3 getCoreSupport(const Vector3& direction) const override;
    bool getSupportSegment(const Vector3& direction, Vector3& p0, Vector3& p1) const override;
    AABB computeAABB(const Transform& xf) const override;
    Real getVolume() const override;
    Vector3 getCentroid() const override { return Vector3::zero(); }
    Matrix3 computeInertia(Real mass) const override;

private:
    Real radius_;
    Real halfHeight_;
};

} // namespace physics
