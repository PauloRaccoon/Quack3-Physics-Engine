// =============================================================================
// ObjectSpawner.cpp
// =============================================================================
#include "viewer/core/ObjectSpawner.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/SphereShape.h"
#include "viewer/core/PolyShapes.h"

namespace viewer {

using namespace physics;

namespace {

Real clampTo(Real v, Real lo, Real hi) { return std::min(std::max(v, lo), hi); }

Vector3 clampVec(const Vector3& v, Real maxLen) { return v.clampedLength(maxLen); }

constexpr unsigned char kColors[][3] = {{240, 150, 60}, {90, 170, 240}, {120, 210, 120}, {230, 100, 120},
                                         {200, 160, 240}, {240, 220, 100}, {100, 220, 210}, {220, 130, 220}};
constexpr int kColorCount = static_cast<int>(sizeof(kColors) / sizeof(kColors[0]));

} // namespace

const char* spawnKindName(SpawnKind kind) {
    switch (kind) {
        case SpawnKind::Box: return "caixa";
        case SpawnKind::Sphere: return "esfera";
        case SpawnKind::Tetrahedron: return "tetraedro";
        case SpawnKind::Octahedron: return "octaedro";
        case SpawnKind::Prism: return "prisma";
        case SpawnKind::Pyramid: return "piramide";
    }
    return "?";
}

Quaternion orientationFromQuaternion(Real w, Real x, Real y, Real z, bool* ok) {
    const Quaternion q(w, x, y, z);
    const Real n = q.isFinite() ? q.norm() : Real(0);
    if (!(n > Real(1e-6))) {
        if (ok) *ok = false;
        return Quaternion::identity();
    }
    if (ok) *ok = true;
    return q.normalized();
}

Quaternion orientationFromAxisAngle(const Vector3& axis, Real angle, bool* ok) {
    if (!axis.isFinite() || !std::isfinite(angle) || axis.lengthSquared() < Real(1e-12)) {
        if (ok) *ok = false;
        return Quaternion::identity();
    }
    if (ok) *ok = true;
    return Quaternion::fromAxisAngle(axis.normalized(), angle);
}

BodyId spawnBody(Scene& scene, const SpawnParams& p) {
    if (!scene.world) return kInvalidBodyId;
    if (!std::isfinite(p.size) || !std::isfinite(p.mass) || !std::isfinite(p.friction) || !std::isfinite(p.restitution) ||
        !p.boxScale.isFinite() || !p.position.isFinite() || !p.orientation.isFinite() || !p.velocity.isFinite() || !p.angularVelocity.isFinite())
        return kInvalidBodyId;
    const Real n = p.orientation.norm();
    if (!(n > Real(1e-6))) return kInvalidBodyId;

    const Real size = clampTo(p.size, spawnlimits::kMinSize, spawnlimits::kMaxSize);
    SceneBody sb;
    RigidBodyDesc d;
    try {
        if (p.model && p.model->shape) {   // V15b: modelo importado
            sb.model = p.model;
            const Shape& ms = *p.model->shape;
            if (ms.getType() == ShapeType::Box) {   // V17: colisor em caixa; a malha continua sendo o modelo
                d.shape = p.model->shape;
                sb.kind = ShapeKind::Box;
                const Vector3 h = static_cast<const BoxShape&>(ms).getHalfExtents();
                sb.halfExtents = Vec3f{static_cast<float>(h.x), static_cast<float>(h.y), static_cast<float>(h.z)};
            } else if (ms.getType() == ShapeType::Sphere) {   // V17: colisor em esfera
                d.shape = p.model->shape;
                sb.kind = ShapeKind::Sphere;
                sb.radius = static_cast<float>(static_cast<const SphereShape&>(ms).getRadius());
            } else {
                sb.poly = std::dynamic_pointer_cast<const PolyhedronShape>(p.model->shape);
                if (!sb.poly) return kInvalidBodyId;
            }
        } else switch (p.kind) {
            case SpawnKind::Box:
            {
                // V14: aresta por eixo = size · escala, limitada (a V1 usava só a aresta única `size`).
                auto edge = [&](Real scale) {
                    return clampTo(size * clampTo(scale, spawnlimits::kMinBoxScale, spawnlimits::kMaxBoxScale),
                                   spawnlimits::kMinBoxExtent, spawnlimits::kMaxBoxExtent);
                };
                const Vector3 half(edge(p.boxScale.x) * Real(0.5), edge(p.boxScale.y) * Real(0.5), edge(p.boxScale.z) * Real(0.5));
                d.shape = std::make_shared<BoxShape>(half);
                sb.kind = ShapeKind::Box;
                sb.halfExtents = Vec3f{static_cast<float>(half.x), static_cast<float>(half.y), static_cast<float>(half.z)};
                break;
            }
            case SpawnKind::Sphere:
                d.shape = std::make_shared<SphereShape>(size * Real(0.5));
                sb.kind = ShapeKind::Sphere;
                sb.radius = static_cast<float>(size) * 0.5f;
                break;
            case SpawnKind::Tetrahedron:
                sb.poly = makeTetrahedron(Real(0.45) * size);
                break;
            case SpawnKind::Octahedron:
                sb.poly = makeOctahedron(Real(0.6) * size);
                break;
            case SpawnKind::Prism:
                sb.poly = makeTriPrism(Real(0.55) * size, Real(0.4) * size);
                break;
            case SpawnKind::Pyramid:
                sb.poly = makeSquarePyramid(Real(0.5) * size, Real(1.0) * size);
                break;
        }
        if (sb.poly) {
            d.shape = sb.poly;
            sb.kind = ShapeKind::Polyhedron;
        }
        d.mass = clampTo(p.mass, spawnlimits::kMinMass, spawnlimits::kMaxMass);
        Vector3 pos = p.position;
        pos.x = clampTo(pos.x, -spawnlimits::kMaxCoordinate, spawnlimits::kMaxCoordinate);
        pos.y = clampTo(pos.y, -spawnlimits::kMaxCoordinate, spawnlimits::kMaxCoordinate);
        pos.z = clampTo(pos.z, -spawnlimits::kMaxCoordinate, spawnlimits::kMaxCoordinate);
        d.transform = Transform(pos, p.orientation.normalized());
        if (p.isStatic) {
            d.type = BodyType::Static;   // V14: massa e velocidades não se aplicam
        } else {
            d.linearVelocity = clampVec(p.velocity, spawnlimits::kMaxLinearSpeed);
            d.angularVelocity = clampVec(p.angularVelocity, spawnlimits::kMaxAngularSpeed);
        }
        d.friction = clampTo(p.friction, Real(0), Real(2));
        d.restitution = clampTo(p.restitution, Real(0), Real(1));
        d.angularDamping = Real(0);
        sb.id = scene.world->createBody(d);   // pode lançar (mundo cheio, forma inválida): a cena não muda
    } catch (const std::exception&) {
        return kInvalidBodyId;
    }
    const int ci = static_cast<int>(scene.bodies.size()) % kColorCount;
    sb.r = kColors[ci][0];
    sb.g = kColors[ci][1];
    sb.b = kColors[ci][2];
    if (p.isStatic) {   // cenário: cinza-azulado, como o chão
        sb.isStatic = true;
        sb.r = 150; sb.g = 156; sb.b = 170;
    }
    scene.bodies.push_back(sb);
    return sb.id;
}

bool spawnPointFromRay(const Scene& scene, const Vector3& origin, const Vector3& direction, Real size, Vector3& point) {
    if (!scene.world || !origin.isFinite() || !direction.isFinite() || direction.lengthSquared() < Real(1e-12) ||
        !std::isfinite(size))
        return false;
    RaycastHit hit;
    if (!scene.world->raycast(origin, direction, Real(500), hit)) return false;
    // Folga: metade do tamanho com uma sobra (a diagonal de uma caixa passa de 0,86·tamanho).
    const Real clearance = clampTo(size, spawnlimits::kMinSize, spawnlimits::kMaxSize) * Real(0.9);
    point = hit.point + hit.normal * clearance;
    return point.isFinite();
}

BodyId launchFrom(Scene& scene, const SpawnParams& params, const Vector3& origin, const Vector3& direction, Real speed) {
    if (!origin.isFinite() || !direction.isFinite() || direction.lengthSquared() < Real(1e-12) || !std::isfinite(speed))
        return kInvalidBodyId;
    SpawnParams p = params;
    const Vector3 dir = direction.normalized();
    p.position = origin + dir * Real(2);
    p.velocity = dir * clampTo(speed, Real(0), spawnlimits::kMaxLinearSpeed);
    return spawnBody(scene, p);
}

} // namespace viewer
