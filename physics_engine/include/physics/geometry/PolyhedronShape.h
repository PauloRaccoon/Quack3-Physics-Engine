// =============================================================================
// PolyhedronShape.h
// Poliedro convexo arbitrário baseado em HalfEdgeStructure. As propriedades de
// massa (volume, centroide, inércia) são calculadas de forma EXATA na
// construção, por decomposição em tetraedros.
// =============================================================================
#pragma once

#include <vector>

#include "physics/geometry/HalfEdgeStructure.h"
#include "physics/geometry/Shape.h"

namespace physics {

class PolyhedronShape final : public Shape {
public:
    /// @param vertices posições locais
    /// @param faces    listas de índices em sentido anti-horário visto de fora
    /// @throws std::invalid_argument se a topologia for inválida, o sólido não
    ///         for convexo/planar, ou o volume for nulo/negativo (orientação invertida).
    PolyhedronShape(const std::vector<Vector3>& vertices,
                    const std::vector<std::vector<int>>& faces);

    /// Caixa como poliedro (8 vértices, 6 faces quadradas) — útil para testes
    /// e para obter topologia de face/aresta de caixas.
    static PolyhedronShape makeBox(const Vector3& halfExtents);

    const HalfEdgeStructure& getTopology() const { return topology_; }
    const HalfEdgeStructure* getHalfEdgeStructure() const override { return &topology_; }

    ShapeType getType() const override { return ShapeType::Polyhedron; }
    Vector3 getSupport(const Vector3& direction) const override;

    /// Índice do vértice de suporte. `hint` é um vértice inicial (ex.: o
    /// resultado do quadro anterior) para a busca por hill climbing.
    int getSupportVertexIndex(const Vector3& direction, int hint = 0) const;

    FeatureId getSupportFeature(const Vector3& direction) const override {
        return static_cast<FeatureId>(getSupportVertexIndex(direction));
    }

    /// A face cuja normal é a mais alinhada com `direction`. Retorna 0 se a
    /// face tiver mais de `maxVertices` vértices.
    int getSupportFace(const Vector3& direction, Vector3* outVertices, int maxVertices) const override;

    Real getVolume() const override { return volume_; }
    Vector3 getCentroid() const override { return centroid_; }
    Matrix3 computeInertia(Real mass) const override { return inertiaPerMass_ * mass; }

private:
    void computeMassProperties();

    HalfEdgeStructure topology_;
    Real volume_ = Real(0);
    Vector3 centroid_;
    Matrix3 inertiaPerMass_;  ///< tensor em torno do centroide para massa 1
};

} // namespace physics
