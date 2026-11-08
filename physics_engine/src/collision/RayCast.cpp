// =============================================================================
// RayCast.cpp
// =============================================================================
#include "physics/collision/RayCast.h"

#include <cmath>

#include <algorithm>

#include "physics/geometry/CapsuleShape.h"
#include "physics/geometry/CylinderShape.h"
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

/// Raio contra o cilindro de eixo Y (raio r, meia-altura h): interseção do intervalo da laje |y| ≤ h com o do
/// cilindro infinito x² + z² ≤ r². O primeiro ponto de entrada dá o t e a normal.
bool rayCylinder(Real r, Real h, const Vector3& o, const Vector3& d, Real maxDistance, LocalHit& out) {
    Real tEnter = -kRealMax, tExit = kRealMax;
    Vector3 enterNormal;
    bool entered = false;

    // laje |y| <= h
    constexpr Real kParallel = Real(1e-9);
    if (std::abs(d.y) < kParallel) {
        if (std::abs(o.y) > h) return false;
    } else {
        Real t1 = (-h - o.y) / d.y, t2 = (h - o.y) / d.y;
        Vector3 n1(Real(0), Real(-1), Real(0));   // normal ao entrar pela tampa de baixo
        if (t1 > t2) { std::swap(t1, t2); n1 = Vector3(Real(0), Real(1), Real(0)); }
        tEnter = t1;
        enterNormal = n1;
        entered = true;
        tExit = t2;
    }
    // cilindro infinito x² + z² <= r²
    const Real a = d.x * d.x + d.z * d.z;
    const Real c = o.x * o.x + o.z * o.z - r * r;
    if (a < kParallel) {
        if (c > Real(0)) return false;   // paralelo ao eixo e fora do raio
    } else {
        const Real b = o.x * d.x + o.z * d.z;
        const Real disc = b * b - a * c;
        if (disc < Real(0)) return false;
        const Real sq = std::sqrt(disc);
        const Real t1 = (-b - sq) / a, t2 = (-b + sq) / a;
        if (t1 > tEnter) {
            tEnter = t1;
            enterNormal = Vector3(o.x + d.x * t1, Real(0), o.z + d.z * t1).normalizedOr(Vector3::unitX());
            entered = true;
        }
        tExit = std::min(tExit, t2);
    }
    if (tEnter > tExit || tExit < Real(0)) return false;
    if (tEnter < Real(0) || !entered) {   // origem dentro do sólido
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

/// Raio contra a cápsula de eixo Y (segmento de −h a +h, raio r): a parte lateral do cilindro infinito (onde
/// |y| ≤ h) e as duas esferas das pontas (onde a esfera passa de ±h); vale o menor t ≥ 0.
bool rayCapsule(Real r, Real h, const Vector3& o, const Vector3& d, Real maxDistance, LocalHit& out) {
    const Real cy = std::max(-h, std::min(h, o.y));
    if (Vector3(o.x, o.y - cy, o.z).lengthSquared() <= r * r) {   // origem dentro
        out.t = Real(0);
        out.inside = true;
        return true;
    }
    Real bestT = kRealMax;
    Vector3 bestN;
    // lado
    const Real a = d.x * d.x + d.z * d.z;
    if (a > Real(1e-9)) {
        const Real b = o.x * d.x + o.z * d.z;
        const Real c = o.x * o.x + o.z * o.z - r * r;
        const Real disc = b * b - a * c;
        if (disc >= Real(0)) {
            const Real t = (-b - std::sqrt(disc)) / a;
            const Real y = o.y + d.y * t;
            if (t >= Real(0) && std::abs(y) <= h) {
                bestT = t;
                bestN = Vector3(o.x + d.x * t, Real(0), o.z + d.z * t).normalizedOr(Vector3::unitX());
            }
        }
    }
    // pontas
    for (int k = 0; k < 2; ++k) {
        const Real sign = k == 0 ? Real(1) : Real(-1);
        const Vector3 center(Real(0), sign * h, Real(0));
        LocalHit sh;
        if (raySphere(r, o - center, d, kRealMax, sh) && !sh.inside && sh.t < bestT) {
            const Real y = o.y + d.y * sh.t;
            if (sign * y >= h - Real(1e-6)) {   // só a meia-esfera de fora
                bestT = sh.t;
                bestN = sh.normal;
            }
        }
    }
    if (!(bestT <= maxDistance)) return false;
    out.t = bestT;
    out.inside = false;
    out.normal = bestN;
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
        case ShapeType::Capsule: {
            const CapsuleShape& c = static_cast<const CapsuleShape&>(shape);
            found = rayCapsule(c.getRadius(), c.getHalfHeight(), o, d, maxDistance, lh);
            break;
        }
        case ShapeType::Cylinder: {
            const CylinderShape& c = static_cast<const CylinderShape&>(shape);
            found = rayCylinder(c.getRadius(), c.getHalfHeight(), o, d, maxDistance, lh);
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
        case ShapeType::Capsule: {
            const CapsuleShape& c = static_cast<const CapsuleShape&>(shape);
            const Real y = std::max(Real(-1) * c.getHalfHeight(), std::min(c.getHalfHeight(), p.y));   // ponto mais próximo do segmento
            return Vector3(p.x, p.y - y, p.z).lengthSquared() <= c.getRadius() * c.getRadius();
        }
        case ShapeType::Cylinder: {
            const CylinderShape& c = static_cast<const CylinderShape&>(shape);
            return std::abs(p.y) <= c.getHalfHeight() && p.x * p.x + p.z * p.z <= c.getRadius() * c.getRadius();
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
