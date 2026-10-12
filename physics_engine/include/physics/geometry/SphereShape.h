// =============================================================================
// SphereShape.h
// Esfera centrada na origem local.
// =============================================================================
#pragma once

#include "physics/geometry/Shape.h"

namespace physics {

class SphereShape final : public Shape {
public:
    /// @throws std::invalid_argument se o raio não for finito e positivo.
    explicit SphereShape(Real radius);

    Real getRadius() const { return radius_; }

    ShapeType getType() const override { return ShapeType::Sphere; }
    Vector3 getSupport(const Vector3& direction) const override;
    AABB computeAABB(const Transform& xf) const override;
    Real getVolume() const override;
    Vector3 getCentroid() const override { return Vector3::zero(); }
    Matrix3 computeInertia(Real mass) const override;

private:
    Real radius_;
};

} // namespace physics
