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

const char* colliderKindName(ColliderKind kind) {
    switch (kind) {
        case ColliderKind::Hull: return "fecho convexo";
        case ColliderKind::Box: return "caixa";
        case ColliderKind::Sphere: return "esfera";
        case ColliderKind::Capsule: return "capsula";
        case ColliderKind::Cylinder: return "cilindro";
    }
    return "?";
}

bool makeImportedShape(const ObjModel& model, float scale, ImportedShape& out, std::string& error) {
    return makeImportedShape(model, scale, ColliderKind::Hull, out, error);
}

bool makeImportedShape(const ObjModel& model, float scale, ColliderKind kind, ImportedShape& out, std::string& error) {
    return makeImportedShape(model, scale, kind, ColliderTuning{}, out, error);
}

bool makeImportedShape(const ObjModel& model, float scale, ColliderKind kind, const ColliderTuning& tuning, ImportedShape& out,
                       std::string& error) {
    out = ImportedShape{};
    if (kind == ColliderKind::Box || kind == ColliderKind::Sphere || kind == ColliderKind::Capsule || kind == ColliderKind::Cylinder) {
        if (!std::isfinite(tuning.radiusScale) || !std::isfinite(tuning.heightScale)) {
            error = "ajuste de raio/altura invalido";
            return false;
        }
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
        const Vec3f mid{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, (lo.z + hi.z) * 0.5f};
        const float ext[3] = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
        out.longestAxis = ext[0] >= ext[1] && ext[0] >= ext[2] ? 0 : (ext[1] >= ext[2] ? 1 : 2);
        out.offset = Vector3(static_cast<Real>(mid.x) * Real(scale), static_cast<Real>(mid.y) * Real(scale), static_cast<Real>(mid.z) * Real(scale));
        out.scale = scale;
        out.collider = kind;
        std::vector<char> used(model.positions.size(), 0);
        for (const ObjTriangle& t : model.triangles) for (int k = 0; k < 3; ++k) used[static_cast<std::size_t>(t.p[k])] = 1;
        int nUsed = 0;
        for (char u : used) nUsed += u ? 1 : 0;
        out.modelVertices = nUsed;
        try {
            if (kind == ColliderKind::Box) {
                constexpr Real kMinHalf = Real(0.005);   // espessura mínima 1 cm (parede, folha)
                const Vector3 half(std::max(kMinHalf, static_cast<Real>((hi.x - lo.x) * 0.5f * scale)),
                                   std::max(kMinHalf, static_cast<Real>((hi.y - lo.y) * 0.5f * scale)),
                                   std::max(kMinHalf, static_cast<Real>((hi.z - lo.z) * 0.5f * scale)));
                out.shape = std::make_shared<BoxShape>(half);
                out.boxHalf = half;
                out.colliderVolume = 8.0 * static_cast<double>(half.x) * static_cast<double>(half.y) * static_cast<double>(half.z);
                out.hullVertices = 8;
                out.hullFaces = 6;
            } else if (kind == ColliderKind::Capsule || kind == ColliderKind::Cylinder) {
                // Ajuste automático à caixa envolvente (eixo Y): raio = a maior meia-aresta de X e Z; altura = a de Y.
                const float rs = std::min(kMaxTuning, std::max(kMinTuning, tuning.radiusScale));
                const float hs = std::min(kMaxTuning, std::max(kMinTuning, tuning.heightScale));
                const float hx = (hi.x - lo.x) * 0.5f * scale, hy = (hi.y - lo.y) * 0.5f * scale, hz = (hi.z - lo.z) * 0.5f * scale;
                const float r0 = std::max(0.005f, std::max(hx, hz));
                const float radius = std::max(0.005f, r0 * rs);
                if (kind == ColliderKind::Capsule) {
                    const float h0 = std::max(0.0f, hy - r0);   // meia-altura do SEGMENTO (as pontas arredondadas somam r)
                    const float hh = h0 * hs;
                    out.shape = std::make_shared<CapsuleShape>(static_cast<Real>(radius), static_cast<Real>(hh));
                    out.axisHalfHeight = hh;
                } else {
                    const float hh = std::max(0.005f, hy * hs);
                    out.shape = std::make_shared<CylinderShape>(static_cast<Real>(radius), static_cast<Real>(hh));
                    out.axisHalfHeight = hh;
                }
                out.axisRadius = radius;
                out.colliderVolume = static_cast<double>(out.shape->getVolume());
            } else {
                double r2 = 0;
                for (std::size_t i = 0; i < used.size(); ++i)
                    if (used[i]) {
                        const Vec3f& v = model.positions[i];
                        const double dx = static_cast<double>(v.x - mid.x), dy = static_cast<double>(v.y - mid.y), dz = static_cast<double>(v.z - mid.z);
                        r2 = std::max(r2, dx * dx + dy * dy + dz * dz);
                    }
                const float radius = std::max(0.005f, static_cast<float>(std::sqrt(r2)) * scale);
                out.shape = std::make_shared<SphereShape>(static_cast<Real>(radius));
                out.sphereRadius = radius;
                out.colliderVolume = 4.0 / 3.0 * 3.14159265358979323846 * static_cast<double>(radius) * radius * radius;
            }
        } catch (const std::exception& e) {
            error = std::string("o motor recusou a forma: ") + e.what();
            out = ImportedShape{};
            return false;
        }
        out.modelVolume = meshVolume(model, scale);
        out.fill = out.colliderVolume > 0 ? out.modelVolume / out.colliderVolume : 0.0;
        return true;
    }
    if (kind != ColliderKind::Hull) {
        error = "tipo de colisor desconhecido";
        return false;
    }
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
    if (static_cast<int>(hull.vertices.size()) > modellimits::kMaxHullVertices || static_cast<int>(hull.faces.size()) > modellimits::kMaxHullFaces) {
        error = "fecho grande demais (" + std::to_string(hull.vertices.size()) + " vertices, " + std::to_string(hull.faces.size()) +
                " faces; limite 4096 / 8192): use um modelo menos detalhado";
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
    out.collider = ColliderKind::Hull;
    out.colliderVolume = out.hullVolume;
    out.fill = out.hullVolume > 0 ? out.modelVolume / out.hullVolume : 0.0;
    return true;
}

} // namespace viewer
