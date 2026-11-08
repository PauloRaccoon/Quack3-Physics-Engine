// =============================================================================
// CylinderShape.h  (V18)
// Cilindro maciço de eixo Y local: raio r, meia-altura h (de −h a +h). Convexo, sem arredondamento (raio de núcleo 0).
//
// Contatos: a TAMPA (um polígono de kCapSegments lados) entra no clipping de faces, como a face de uma caixa; deitado,
// a geratriz (linha) dá dois pontos de contato (Shape::getSupportSegment); inclinado, o contato é a borda da tampa.
// =============================================================================
#pragma once

#include "physics/geometry/Shape.h"

namespace physics {

class CylinderShape final : public Shape {
public:
    static constexpr int kCapSegments = 16;   ///< lados do polígono que representa a tampa no clipping (≤ Shape::kMaxFaceVertices)

    /// @param radius      raio [kMinShapeExtent, kMaxShapeExtent]
    /// @param halfHeight  meia-altura [kMinShapeExtent, kMaxShapeExtent]
    /// @throws std::invalid_argument se algum valor não for finito ou estiver fora da faixa.
    CylinderShape(Real radius, Real halfHeight);

    Real getRadius() const { return radius_; }
    Real getHalfHeight() const { return halfHeight_; }

    ShapeType getType() const override { return ShapeType::Cylinder; }
    Vector3 getSupport(const Vector3& direction) const override;
    int getSupportFace(const Vector3& direction, Vector3* outVertices, int maxVertices) const override;
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
