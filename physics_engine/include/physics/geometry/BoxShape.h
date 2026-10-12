// =============================================================================
// BoxShape.h
// Caixa (paralelepípedo retangular) centrada na origem local, definida pelos
// semi-extents (metade do comprimento em cada eixo local).
// =============================================================================
#pragma once

#include "physics/geometry/Shape.h"

namespace physics {

class BoxShape final : public Shape {
public:
    /// @throws std::invalid_argument se algum semi-extent não for finito e positivo.
    explicit BoxShape(const Vector3& halfExtents);

    const Vector3& getHalfExtents() const { return halfExtents_; }

    ShapeType getType() const override { return ShapeType::Box; }
    Vector3 getSupport(const Vector3& direction) const override;

    /// Índice (0..7) do vértice: bit0 = x>=0, bit1 = y>=0, bit2 = z>=0.
    FeatureId getSupportFeature(const Vector3& direction) const override;

    AABB computeAABB(const Transform& xf) const override;
    Real getVolume() const override;
    Vector3 getCentroid() const override { return Vector3::zero(); }
    Matrix3 computeInertia(Real mass) const override;

private:
    Vector3 halfExtents_;
};

} // namespace physics
