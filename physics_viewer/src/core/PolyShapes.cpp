// =============================================================================
// PolyShapes.cpp
// =============================================================================
#include "viewer/core/PolyShapes.h"

#include <algorithm>

namespace viewer {

using namespace physics;

/// Poliedro convexo a partir de vértices e faces (cada face é um ciclo de índices, em qualquer
/// sentido): inverte as que ficarem voltadas para dentro, para que todas apontem para fora
/// (anti-horário visto de fora), como o motor exige.
std::shared_ptr<const PolyhedronShape> makePoly(const std::vector<Vector3>& verts,
                                                std::vector<std::vector<int>> faces) {
    Vector3 center;
    for (const Vector3& v : verts) center += v;
    center = center / Real(verts.size());

    for (std::vector<int>& f : faces) {
        // Normal de Newell (vale para faces planas com 3 ou mais vértices).
        Vector3 n;
        Vector3 fc;
        for (std::size_t i = 0; i < f.size(); ++i) {
            const Vector3& a = verts[static_cast<std::size_t>(f[i])];
            const Vector3& b = verts[static_cast<std::size_t>(f[(i + 1) % f.size()])];
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
            fc += a;
        }
        fc = fc / Real(f.size());
        if (n.dot(fc - center) < Real(0)) std::reverse(f.begin(), f.end());
    }
    return std::make_shared<PolyhedronShape>(verts, faces);
}

std::shared_ptr<const PolyhedronShape> makeTetrahedron(Real s) {
    return makePoly({Vector3(s, s, s), Vector3(s, -s, -s), Vector3(-s, s, -s), Vector3(-s, -s, s)},
                    {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}});
}

std::shared_ptr<const PolyhedronShape> makeOctahedron(Real s) {
    // Vértices: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z.
    return makePoly({Vector3(s, 0, 0), Vector3(-s, 0, 0), Vector3(0, s, 0), Vector3(0, -s, 0),
                     Vector3(0, 0, s), Vector3(0, 0, -s)},
                    {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}});
}

std::shared_ptr<const PolyhedronShape> makeTriPrism(Real r, Real halfHeight) {
    const Real c = r * Real(0.8660254);   // cos 30°
    const Real h = r * Real(0.5);
    // 0..2: triângulo de baixo; 3..5: triângulo de cima.
    return makePoly({Vector3(0, -halfHeight, r), Vector3(c, -halfHeight, -h), Vector3(-c, -halfHeight, -h),
                     Vector3(0, halfHeight, r), Vector3(c, halfHeight, -h), Vector3(-c, halfHeight, -h)},
                    {{0, 1, 2}, {3, 4, 5}, {0, 1, 4, 3}, {1, 2, 5, 4}, {2, 0, 3, 5}});
}

std::shared_ptr<const PolyhedronShape> makeSquarePyramid(Real halfBase, Real height) {
    const Real yb = -height * Real(0.25);   // base abaixo, ápice acima (o centro de massa fica perto da origem)
    return makePoly({Vector3(-halfBase, yb, -halfBase), Vector3(halfBase, yb, -halfBase),
                     Vector3(halfBase, yb, halfBase), Vector3(-halfBase, yb, halfBase),
                     Vector3(0, yb + height, 0)},
                    {{0, 1, 2, 3}, {0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}});
}

} // namespace viewer
