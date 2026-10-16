// =============================================================================
// BoxShape.h
// Caixa (paralelepípedo retangular) centrada na origem local, definida pelos
// semi-extents (metade do comprimento em cada eixo local).
// =============================================================================
#pragma once

#include "physics/geometry/HalfEdgeStructure.h"
#include "physics/geometry/Shape.h"

namespace physics {

class BoxShape final : public Shape {
public:
    /// @throws std::invalid_argument se algum semi-extent não for finito e positivo.
    explicit BoxShape(const Vector3& halfExtents);

    const Vector3& getHalfExtents() const { return halfExtents_; }

    ShapeType getType() const override { return ShapeType::Box; }

    /// Topologia da caixa (8 vértices, 6 faces). O índice do vértice segue o
    /// mesmo padrão de getSupportFeature(): bit0 = x>=0, bit1 = y>=0, bit2 = z>=0.
    const HalfEdgeStructure* getHalfEdgeStructure() const override { return &topology_; }

    Vector3 getSupport(const Vector3& direction) const override;

    /// Índice (0..7) do vértice: bit0 = x>=0, bit1 = y>=0, bit2 = z>=0.
    FeatureId getSupportFeature(const Vector3& direction) const override;

    /// A face (4 vértices) cujo eixo é o de maior |componente| de `direction`.
    int getSupportFace(const Vector3& direction, Vector3* outVertices, int maxVertices) const override;

    AABB computeAABB(const Transform& xf) const override;
    Real getVolume() const override;
    Vector3 getCentroid() const override { return Vector3::zero(); }
    Matrix3 computeInertia(Real mass) const override;

private:
    Vector3 halfExtents_;
    HalfEdgeStructure topology_;
};

} // namespace physics
