// =============================================================================
// PolyhedronShape.cpp
// =============================================================================
#include "physics/geometry/PolyhedronShape.h"

#include <stdexcept>
#include <string>

namespace physics {

namespace {
/// Abaixo deste número de vértices a busca exaustiva é mais rápida que
/// navegar pelo grafo (menos desvios de ramificação, tudo em cache).
constexpr int kBruteForceVertexLimit = 16;
} // namespace

PolyhedronShape::PolyhedronShape(const std::vector<Vector3>& vertices,
                                 const std::vector<std::vector<int>>& faces) {
    std::string error;
    if (!topology_.build(vertices, faces, &error)) {
        throw std::invalid_argument("PolyhedronShape: topologia invalida: " + error);
    }
    if (!topology_.validate(&error)) {
        throw std::invalid_argument("PolyhedronShape: half-edge inconsistente: " + error);
    }

    // Tolerância de planaridade/convexidade relativa ao tamanho do sólido.
    Real extent = Real(0);
    for (const HEVertex& v : topology_.vertices()) extent = std::max(extent, v.position.abs().maxComponent());
    const Real tolerance = Real(10) * kCompareEpsilon * std::max(extent, Real(1e-3));
    if (!topology_.isConvexAndPlanar(tolerance)) {
        throw std::invalid_argument(
            "PolyhedronShape: o poliedro deve ser convexo, com faces planares e normais para fora "
            "(vertices em sentido anti-horario visto de fora)");
    }

    computeMassProperties();
}

PolyhedronShape PolyhedronShape::makeBox(const Vector3& h) {
    // Índice do vértice: bit0 = x>0, bit1 = y>0, bit2 = z>0.
    std::vector<Vector3> v;
    v.reserve(8);
    for (int i = 0; i < 8; ++i) {
        v.emplace_back((i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z);
    }
    // Faces anti-horárias vistas de fora: +X, -X, +Y, -Y, +Z, -Z.
    const std::vector<std::vector<int>> f = {
        {1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};
    return PolyhedronShape(v, f);
}

void PolyhedronShape::computeMassProperties() {
    // Decompõe o sólido em tetraedros (origem, a, b, c), um por triângulo de
    // cada face (leque a partir do 1º vértice). Os volumes são ASSINADOS, então
    // a soma é correta mesmo com a origem fora do poliedro.
    //   V    = a · (b × c) / 6
    //   ∫x   = V (a + b + c) / 4
    //   ∫xxᵀ = V/20 (aaᵀ + bbᵀ + ccᵀ + S Sᵀ),  S = a + b + c
    Real volume = Real(0);
    Vector3 firstMoment;
    Matrix3 covariance = Matrix3::zero();

    for (int f = 0; f < topology_.getFaceCount(); ++f) {
        const int startEdge = topology_.getFace(f).edge;
        const Vector3 a = topology_.getVertex(topology_.getEdge(startEdge).origin).position;

        int e = topology_.getEdge(startEdge).next;
        // Triângulos (v0, v_i, v_{i+1}) para i = 1 .. k-2.
        while (topology_.getEdge(e).next != startEdge) {
            const Vector3 b = topology_.getVertex(topology_.getEdge(e).origin).position;
            const Vector3 c = topology_.getVertex(topology_.destination(e)).position;

            const Real v = tripleProduct(a, b, c) / Real(6);
            const Vector3 s = a + b + c;
            volume += v;
            firstMoment += s * (v / Real(4));
            covariance += (Matrix3::outer(a, a) + Matrix3::outer(b, b) + Matrix3::outer(c, c) +
                           Matrix3::outer(s, s)) * (v / Real(20));

            e = topology_.getEdge(e).next;
        }
    }

    if (!(volume > Real(0)) || !std::isfinite(volume)) {
        throw std::invalid_argument("PolyhedronShape: volume nulo ou negativo (faces invertidas?)");
    }

    volume_ = volume;
    centroid_ = firstMoment / volume;

    // Tensor de inércia a partir da covariância:  I = tr(C) · Id − C.
    // Dividindo por V obtém-se o tensor para massa 1 (densidade = 1/V),
    // ainda em torno da ORIGEM; o teorema dos eixos paralelos leva ao centroide:
    //   I_cm = I_origem − m (|c|² Id − c cᵀ)
    const Matrix3 aboutOrigin = (Matrix3::identity() * covariance.trace() - covariance) * (Real(1) / volume);
    inertiaPerMass_ = aboutOrigin - (Matrix3::identity() * centroid_.lengthSquared() - Matrix3::outer(centroid_, centroid_));
}

int PolyhedronShape::getSupportVertexIndex(const Vector3& direction, int hint) const {
    const int n = topology_.getVertexCount();
    if (n <= kBruteForceVertexLimit) {
        int best = 0;
        Real bestDot = direction.dot(topology_.getVertex(0).position);
        for (int i = 1; i < n; ++i) {
            const Real d = direction.dot(topology_.getVertex(i).position);
            if (d > bestDot) {
                bestDot = d;
                best = i;
            }
        }
        return best;
    }
    return topology_.findSupportVertex(direction, hint);
}

Vector3 PolyhedronShape::getSupport(const Vector3& direction) const {
    return topology_.getVertex(getSupportVertexIndex(direction)).position;
}

} // namespace physics
