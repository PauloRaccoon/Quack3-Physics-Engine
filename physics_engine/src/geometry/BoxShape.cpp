// =============================================================================
// BoxShape.cpp
// =============================================================================
#include "physics/geometry/BoxShape.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace physics {

BoxShape::BoxShape(const Vector3& halfExtents) : halfExtents_(halfExtents) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(halfExtents[i]) || !(halfExtents[i] > Real(0))) {
            throw std::invalid_argument("BoxShape: semi-extents devem ser finitos e positivos");
        }
    }

    // Topologia: vértice i tem bit0 = x>0, bit1 = y>0, bit2 = z>0 (igual a
    // getSupportFeature). Faces anti-horárias vistas de fora: +X, -X, +Y, -Y, +Z, -Z.
    std::vector<Vector3> v;
    v.reserve(8);
    for (int i = 0; i < 8; ++i) {
        v.emplace_back((i & 1) ? halfExtents.x : -halfExtents.x, (i & 2) ? halfExtents.y : -halfExtents.y,
                       (i & 4) ? halfExtents.z : -halfExtents.z);
    }
    const std::vector<std::vector<int>> faces = {
        {1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};
    std::string error;
    if (!topology_.build(v, faces, &error)) {
        throw std::logic_error("BoxShape: falha ao montar a topologia: " + error);
    }
}

Vector3 BoxShape::getSupport(const Vector3& d) const {
    // O vértice da caixa mais distante em d tem, em cada eixo, o sinal de d.
    // (Empate/zero escolhe o lado positivo: resultado determinístico.)
    return Vector3(d.x >= Real(0) ? halfExtents_.x : -halfExtents_.x,
                   d.y >= Real(0) ? halfExtents_.y : -halfExtents_.y,
                   d.z >= Real(0) ? halfExtents_.z : -halfExtents_.z);
}

FeatureId BoxShape::getSupportFeature(const Vector3& d) const {
    return static_cast<FeatureId>((d.x >= Real(0) ? 1u : 0u) | (d.y >= Real(0) ? 2u : 0u) |
                                  (d.z >= Real(0) ? 4u : 0u));
}

int BoxShape::getSupportFace(const Vector3& d, Vector3* out, int maxVertices) const {
    if (maxVertices < 4) return 0;
    const int i = d.maxAxis();                 // eixo normal da face
    const int j = (i + 1) % 3, k = (i + 2) % 3;
    const Real s = d[i] >= Real(0) ? Real(1) : Real(-1);

    // (i, j, k) é uma permutação cíclica => e_j × e_k = e_i, portanto percorrer
    // (-,-) → (+,-) → (+,+) → (-,+) em (j,k) é anti-horário visto de +e_i.
    static const Real kSigns[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    Vector3 corners[4];
    for (int n = 0; n < 4; ++n) {
        Vector3 p;
        p[i] = s * halfExtents_[i];
        p[j] = kSigns[n][0] * halfExtents_[j];
        p[k] = kSigns[n][1] * halfExtents_[k];
        corners[n] = p;
    }
    // Face em -e_i: inverte a ordem para a normal continuar apontando para fora.
    static const int kForward[4] = {0, 1, 2, 3};
    static const int kReverse[4] = {0, 3, 2, 1};
    const int* order = s > Real(0) ? kForward : kReverse;
    for (int n = 0; n < 4; ++n) out[n] = corners[order[n]];
    return 4;
}

AABB BoxShape::computeAABB(const Transform& xf) const {
    // Fórmula de Arvo: semi-extent no mundo = |R| * halfExtents.
    return AABB(-halfExtents_, halfExtents_).transformed(xf);
}

Real BoxShape::getVolume() const { return Real(8) * halfExtents_.x * halfExtents_.y * halfExtents_.z; }

Matrix3 BoxShape::computeInertia(Real mass) const {
    // Caixa maciça de lados (2hx, 2hy, 2hz):  Ixx = m/12 (ly² + lz²) = m/3 (hy² + hz²)
    const Vector3 h2(halfExtents_.x * halfExtents_.x, halfExtents_.y * halfExtents_.y,
                     halfExtents_.z * halfExtents_.z);
    const Real k = mass / Real(3);
    return Matrix3::diagonal(k * (h2.y + h2.z), k * (h2.x + h2.z), k * (h2.x + h2.y));
}

} // namespace physics
