// =============================================================================
// Interaction.cpp
// =============================================================================
#include "viewer/core/Interaction.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/SphereShape.h"

namespace viewer {

using namespace physics;

PickResult pickBody(const Scene& scene, const Vector3& origin, const Vector3& direction, Real maxDistance) {
    PickResult out;
    if (!scene.world || !origin.isFinite() || !direction.isFinite() || direction.lengthSquared() < Real(1e-12)) return out;
    QueryFilter filter;
    filter.includeStatic = false;   // clicar no chão não seleciona nada
    RaycastHit hit;
    if (!scene.world->raycast(origin, direction, maxDistance, hit, filter)) return out;
    if (!scene.world->isValidBody(hit.body)) return out;
    out.hit = true;
    out.body = hit.body;
    out.point = hit.point;
    out.distance = hit.distance;
    return out;
}

bool SpringDrag::begin(Scene& scene, const PickResult& pick) {
    end();
    if (!pick.hit || !scene.world || !scene.world->isValidBody(pick.body)) return false;
    const RigidBody& b = scene.world->getBody(pick.body);
    if (!b.isDynamic() || !pick.point.isFinite()) return false;
    body_ = pick.body;
    // Ponto agarrado guardado no referencial do corpo: acompanha o corpo quando ele gira.
    localAnchor_ = b.getOrientation().rotateInverse(pick.point - b.getWorldCenterOfMass());
    depth_ = std::max(pick.distance, Real(0.5));
    anchorWorld_ = pick.point;
    targetWorld_ = pick.point;
    scene.world->setAwake(body_, true);
    return true;
}

void SpringDrag::addDepth(Real delta) {
    if (!active() || !std::isfinite(delta)) return;
    depth_ = std::min(std::max(depth_ + delta, Real(0.5)), Real(100));
}

bool SpringDrag::apply(Scene& scene, const Vector3& rayOrigin, const Vector3& rayDirection, Real frameTime) {
    if (!active()) return false;
    if (!scene.world || !scene.world->isValidBody(body_)) {
        end();
        return false;
    }
    if (!rayOrigin.isFinite() || !rayDirection.isFinite() || rayDirection.lengthSquared() < Real(1e-12) ||
        !std::isfinite(frameTime) || frameTime <= Real(0))
        return false;

    RigidBody& b = scene.world->getBody(body_);
    const Real dt = scene.world->getFixedTimeStep();
    // ω0 limitado para que a mola explícita continue estável com passo físico largo (ω0·Δt ≤ 0,5).
    const Real omega = std::min(Real(kOmega), Real(0.5) / dt);
    const Real k = omega * omega;
    const Real c = Real(2) * omega;   // amortecimento crítico

    const Vector3 r = b.getOrientation().rotate(localAnchor_);          // braço (mundo)
    anchorWorld_ = b.getWorldCenterOfMass() + r;
    targetWorld_ = rayOrigin + rayDirection.normalized() * depth_;
    const Vector3 pointVel = b.getLinearVelocity() + b.getAngularVelocity().cross(r);

    // F = m·(k·(alvo − p) − c·v_p): por unidade de massa, a mesma mola serve para qualquer corpo.
    Vector3 accel = (targetWorld_ - anchorWorld_) * k - pointVel * c;
    accel = accel.clampedLength(Real(kMaxAccel));
    Vector3 force = accel * b.getMass();

    // A força é consumida só no passo FIXO. Com render mais rápido que a física, vários quadros somam
    // antes de um passo (somam frameTime/Δt vezes o impulso certo); com render mais lento, um passo
    // consome a força de um quadro só. Escalar por frameTime/Δt dá o impulso certo nos dois casos.
    force = force * (frameTime / dt);

    scene.world->setAwake(body_, true);
    b.applyForceAtPoint(force, anchorWorld_);
    return true;
}

namespace {
constexpr unsigned char kColors[][3] = {{240, 150, 60}, {90, 170, 240}, {120, 210, 120}, {230, 100, 120},
                                         {200, 160, 240}, {240, 220, 100}, {100, 220, 210}, {220, 130, 220}};
constexpr int kColorCount = static_cast<int>(sizeof(kColors) / sizeof(kColors[0]));
}

BodyId launchBody(Scene& scene, ShapeKind kind, const Vector3& origin, const Vector3& direction, Real speed) {
    if (!scene.world || !origin.isFinite() || !direction.isFinite() || !std::isfinite(speed) ||
        direction.lengthSquared() < Real(1e-12))
        return kInvalidBodyId;
    if (speed < Real(0)) speed = Real(0);
    if (speed > Real(60)) speed = Real(60);

    SceneBody sb;
    RigidBodyDesc d;
    if (kind == ShapeKind::Sphere) {
        d.shape = std::make_shared<SphereShape>(Real(0.3));
        sb.kind = ShapeKind::Sphere;
        sb.radius = 0.3f;
    } else {
        d.shape = std::make_shared<BoxShape>(Vector3(Real(0.3), Real(0.3), Real(0.3)));
        sb.kind = ShapeKind::Box;
        sb.halfExtents = Vec3f{0.3f, 0.3f, 0.3f};
    }
    const Vector3 dir = direction.normalized();
    d.mass = Real(1);
    d.transform = Transform(origin);
    d.linearVelocity = dir * speed;
    d.friction = Real(0.5);
    d.restitution = Real(0.2);
    d.angularDamping = Real(0);
    try {
        sb.id = scene.world->createBody(d);
    } catch (const std::exception&) {
        return kInvalidBodyId;   // mundo cheio (maxBodies) ou descrição inválida
    }
    const int ci = static_cast<int>(scene.bodies.size()) % kColorCount;
    sb.r = kColors[ci][0];
    sb.g = kColors[ci][1];
    sb.b = kColors[ci][2];
    scene.bodies.push_back(sb);
    return sb.id;
}

bool deleteBody(Scene& scene, BodyId id) {
    if (!scene.world || !scene.world->isValidBody(id) || !scene.world->getBody(id).isDynamic()) return false;
    scene.world->destroyBody(id);   // destrói também as juntas ligadas a ele
    scene.bodies.erase(std::remove_if(scene.bodies.begin(), scene.bodies.end(),
                                      [id](const SceneBody& b) { return b.id == id; }),
                       scene.bodies.end());
    scene.links.erase(std::remove_if(scene.links.begin(), scene.links.end(),
                                     [id](const SceneLink& l) { return l.a == id || l.b == id; }),
                      scene.links.end());
    if (scene.focus == id) scene.focus = kInvalidBodyId;
    return true;
}

} // namespace viewer
