// =============================================================================
// ModelCollider.cpp  (V15a)
// =============================================================================
#include "viewer/core/ModelCollider.h"

#include <algorithm>
#include <cmath>
#include <exception>

namespace viewer {

using namespace physics;

float autoScale(const ObjModel& model, float targetSize) {
    Vec3f lo, hi;
    if (!objBounds(model, lo, hi) || !std::isfinite(targetSize) || !(targetSize > 0)) return 1.0f;
    const float size = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z});
    if (!(size > 0.0f)) return 1.0f;
    return std::min(modellimits::kMaxScale, std::max(modellimits::kMinScale, targetSize / size));
}

double meshVolume(const ObjModel& model, float scale) {
    double v = 0;
    const double s = static_cast<double>(scale);
    for (const ObjTriangle& t : model.triangles) {
        const Vec3f &a = model.positions[static_cast<std::size_t>(t.p[0])], &b = model.positions[static_cast<std::size_t>(t.p[1])],
                    &c = model.positions[static_cast<std::size_t>(t.p[2])];
        v += (static_cast<double>(a.x) * (static_cast<double>(b.y) * c.z - static_cast<double>(b.z) * c.y) -
              static_cast<double>(a.y) * (static_cast<double>(b.x) * c.z - static_cast<double>(b.z) * c.x) +
              static_cast<double>(a.z) * (static_cast<double>(b.x) * c.y - static_cast<double>(b.y) * c.x)) / 6.0;
    }
    return std::fabs(v) * s * s * s;
}

bool makeImportedShape(const ObjModel& model, float scale, ImportedShape& out, std::string& error) {
    out = ImportedShape{};
    if (!std::isfinite(scale) || scale < modellimits::kMinScale || scale > modellimits::kMaxScale) {
        error = "escala invalida (use de 0,001 a 1000)";
        return false;
    }
    Vec3f lo, hi;
    if (!objBounds(model, lo, hi)) {
        error = "o modelo nao tem faces";
        return false;
    }
    const float size = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z}) * scale;
    if (!(size >= modellimits::kMinSize) || size > modellimits::kMaxSize) {
        error = "tamanho final fora da faixa (maior dimensao " + std::to_string(size) + " m; use de 0,05 a 40 m)";
        return false;
    }
    // Vértices usados (sem repetição), centrados na caixa envolvente ANTES de escalar: a diferença é exata
    // para coordenadas grandes e mantém a tolerância do motor (relativa ao tamanho) válida.
    const Vec3f mid{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, (lo.z + hi.z) * 0.5f};
    std::vector<char> used(model.positions.size(), 0);
    for (const ObjTriangle& t : model.triangles) for (int k = 0; k < 3; ++k) used[static_cast<std::size_t>(t.p[k])] = 1;
    std::vector<Vector3> pts;
    for (std::size_t i = 0; i < used.size(); ++i)
        if (used[i]) {
            const Vec3f& v = model.positions[i];
            pts.emplace_back(static_cast<Real>((v.x - mid.x) * scale), static_cast<Real>((v.y - mid.y) * scale), static_cast<Real>((v.z - mid.z) * scale));
        }
    out.modelVertices = static_cast<int>(pts.size());
    ConvexHull hull;
    if (!buildConvexHull(pts, hull, &error)) {
        error = "fecho convexo: " + error;
        return false;
    }
    try {
        // 1ª forma só para achar o centroide; a final é reconstruída com o centroide na origem.
        const PolyhedronShape first(hull.vertices, hull.faces);
        const Vector3 c = first.getCentroid();
        for (Vector3& v : hull.vertices) v -= c;
        out.shape = std::make_shared<PolyhedronShape>(hull.vertices, hull.faces);
        out.offset = Vector3(static_cast<Real>(mid.x) * Real(scale), static_cast<Real>(mid.y) * Real(scale), static_cast<Real>(mid.z) * Real(scale)) + c;
        out.hullVolume = static_cast<double>(first.getVolume());
    } catch (const std::exception& e) {
        error = std::string("o motor recusou o fecho: ") + e.what();
        out = ImportedShape{};
        return false;
    }
    out.scale = scale;
    out.hullVertices = static_cast<int>(hull.vertices.size());
    out.hullFaces = static_cast<int>(hull.faces.size());
    out.hullTriangles = hull.triangleCount;
    out.modelVolume = meshVolume(model, scale);
    out.concave = out.modelVolume > 0 && out.modelVolume < 0.9 * out.hullVolume;
    return true;
}

} // namespace viewer
