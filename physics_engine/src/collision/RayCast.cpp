// =============================================================================
// RayCast.cpp
// =============================================================================
#include "physics/collision/RayCast.h"

#include <cmath>

#include "physics/geometry/HalfEdgeStructure.h"
#include "physics/geometry/SphereShape.h"

namespace physics {

namespace {

/// Resultado em espaço LOCAL da forma.
struct LocalHit {
    Real t = Real(0);
    Vector3 normal;
    bool inside = false;
};

/// Raio (o + t·d, |d| = 1) contra a esfera de raio r centrada na origem.
bool raySphere(Real radius, const Vector3& o, const Vector3& d, Real maxDistance, LocalHit& out) {
    const Real c = o.lengthSquared() - radius * radius;
    if (c <= Real(0)) {            // origem dentro (ou sobre a superfície)
        out.t = Real(0);
        out.inside = true;
        return true;
    }
    const Real b = o.dot(d);
    if (b > Real(0)) return false; // fora e se afastando
    const Real disc = b * b - c;
    if (disc < Real(0)) return false;
    // Forma estável para o menor |t|: evita cancelamento quando b² >> c.
    const Real q = -b + std::sqrt(disc);           // > 0 pois b <= 0
    const Real t = c / q;                          // raiz menor = c / (−b + √disc)
    if (!(t <= maxDistance)) return false;
    out.t = t;
    out.inside = false;
    out.normal = ((o + d * t) / radius).normalizedOr(-d);
    return true;
}

/// Raio contra um poliedro convexo (recorte pelos planos das faces).
bool rayConvex(const HalfEdgeStructure& he, const Vector3& o, const Vector3& d, Real maxDistance,
               LocalHit& out) {
    // Intervalo do raio dentro do sólido: [tEnter, tExit] (sem limitar por maxDistance ainda,
    // para distinguir "origem dentro" de "além do alcance").
    Real tEnter = -kRealMax;
    Real tExit = kRealMax;
    Vector3 enterNormal;
    bool entered = false;

    const int faceCount = he.getFaceCount();
    for (int i = 0; i < faceCount; ++i) {
        const HEFace& f = he.getFace(i);
        const Real denom = f.normal.dot(d);          // taxa de variação de n·p ao longo do raio
        const Real dist = f.offset - f.normal.dot(o);// > 0: origem atrás do plano (dentro do semi-espaço)
        if (std::abs(denom) < kDivideEpsilon) {
            if (dist < Real(0)) return false;        // paralelo e fora do semi-espaço
            continue;
        }
        const Real t = dist / denom;
        if (denom < Real(0)) {                       // entrando no semi-espaço
            if (t > tEnter) { tEnter = t; enterNormal = f.normal; entered = true; }
        } else {                                     // saindo
            if (t < tExit) tExit = t;
        }
        if (tEnter > tExit) return false;
    }
    if (tExit < Real(0)) return false;               // o sólido está atrás da origem
    if (tEnter < Real(0) || !entered) {              // origem dentro do sólido
        out.t = Real(0);
        out.inside = true;
        return true;
    }
    if (!(tEnter <= maxDistance)) return false;
    out.t = tEnter;
    out.inside = false;
    out.normal = enterNormal;
    return true;
}

} // namespace

bool raycastShape(const Shape& shape, const Transform& xf, const Vector3& origin,
                  const Vector3& direction, Real maxDistance, ShapeRayHit& hit) {
    if (!origin.isFinite() || !direction.isFinite() || !xf.isFinite()) return false;
    if (!(maxDistance >= Real(0))) return false;   // NaN ou negativo

    const Vector3 o = xf.inverseTransformPoint(origin);
    const Vector3 d = xf.inverseTransformDirection(direction);

    LocalHit lh;
    bool found = false;
    switch (shape.getType()) {
        case ShapeType::Sphere:
            found = raySphere(static_cast<const SphereShape&>(shape).getRadius(), o, d, maxDistance, lh);
            break;
        case ShapeType::Box:
        case ShapeType::Polyhedron: {
            const HalfEdgeStructure* he = shape.getHalfEdgeStructure();
            found = he != nullptr && rayConvex(*he, o, d, maxDistance, lh);
            break;
        }
    }
    if (!found) return false;

    hit.distance = lh.t;
    hit.point = origin + direction * lh.t;
    hit.startedInside = lh.inside;
    hit.normal = lh.inside ? -direction : xf.transformDirection(lh.normal);
    return true;
}

bool shapeContainsPoint(const Shape& shape, const Transform& xf, const Vector3& worldPoint) {
    if (!worldPoint.isFinite() || !xf.isFinite()) return false;
    const Vector3 p = xf.inverseTransformPoint(worldPoint);
    switch (shape.getType()) {
        case ShapeType::Sphere: {
            const Real r = static_cast<const SphereShape&>(shape).getRadius();
            return p.lengthSquared() <= r * r;
        }
        case ShapeType::Box:
        case ShapeType::Polyhedron: {
            const HalfEdgeStructure* he = shape.getHalfEdgeStructure();
            if (he == nullptr) return false;
            for (const HEFace& f : he->faces()) {
                if (f.normal.dot(p) > f.offset) return false;
            }
            return true;
        }
    }
    return false;
}

} // namespace physics
