// =============================================================================
// JointEditor.cpp  (V14)
// =============================================================================
#include "viewer/core/JointEditor.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include "physics/constraints/JointConstraint.h"
#include "viewer/core/Interaction.h"

namespace viewer {

using namespace physics;

namespace {

void setError(std::string* e, const char* msg) {
    if (e) *e = msg;
}

/// Pivô estático sem forma criado pelo editor / cenas (a bolinha cinza de raio 0,12).
BodyId makePivot(Scene& scene, const Vector3& pos) {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.transform = Transform(pos);
    SceneBody sb;
    sb.id = scene.world->createBody(d);
    sb.kind = ShapeKind::Sphere;
    sb.radius = 0.12f;
    sb.isStatic = true;
    sb.r = 170; sb.g = 170; sb.b = 170;
    scene.bodies.push_back(sb);
    return sb.id;
}

bool usedByJoint(const Scene& scene, BodyId id) {
    bool used = false;
    scene.world->forEachJoint([&](const JointConstraint&, BodyId a, BodyId b) {
        if (a == id || b == id) used = true;
    });
    return used;
}

/// Apaga o pivô se for estático, sem forma e sem junta.
void dropOrphanPivot(Scene& scene, BodyId id) {
    if (!scene.world->isValidBody(id)) return;
    const RigidBody& b = scene.world->getBody(id);
    if (b.isStatic() && b.getShape() == nullptr && !usedByJoint(scene, id)) deleteBody(scene, id, true);
}

} // namespace

const char* jointKindName(JointKind kind) {
    switch (kind) {
        case JointKind::Hinge: return "Dobradica";
        case JointKind::BallSocket: return "Bola-soquete";
        case JointKind::Distance: return "Distancia";
    }
    return "?";
}

int jointCount(const Scene& scene) {
    if (!scene.world) return 0;
    int n = 0;
    scene.world->forEachJoint([&](const JointConstraint&, BodyId, BodyId) { ++n; });
    return n;
}

bool createJoint(Scene& scene, const JointParams& p, std::string* error) {
    if (!scene.world) { setError(error, "sem mundo"); return false; }
    PhysicsWorld& w = *scene.world;
    if (!w.isValidBody(p.a)) { setError(error, "corpo A invalido"); return false; }
    const bool toWorld = (p.b == kInvalidBodyId);
    if (!toWorld && !w.isValidBody(p.b)) { setError(error, "corpo B invalido"); return false; }
    if (p.a == p.b) { setError(error, "A e B sao o mesmo corpo"); return false; }
    if (!w.getBody(p.a).isDynamic() && (toWorld || !w.getBody(p.b).isDynamic())) {
        setError(error, "pelo menos um dos corpos precisa ser dinamico");
        return false;
    }
    if (!p.anchor.isFinite() || std::fabs(p.anchor.x) > jointlimits::kMaxCoordinate ||
        std::fabs(p.anchor.y) > jointlimits::kMaxCoordinate || std::fabs(p.anchor.z) > jointlimits::kMaxCoordinate) {
        setError(error, "ancora invalida");
        return false;
    }
    Vector3 axis;
    if (p.kind == JointKind::Hinge) {
        if (!p.axis.isFinite() || p.axis.lengthSquared() < Real(1e-12)) { setError(error, "eixo nulo"); return false; }
        axis = p.axis.normalized();
    }
    Vector3 anchorA = w.getBody(p.a).getWorldCenterOfMass();
    Vector3 anchorB = toWorld ? p.anchor : w.getBody(p.b).getWorldCenterOfMass();
    Real length = 0;
    if (p.kind == JointKind::Distance) {
        length = (anchorB - anchorA).length();
        if (!(length >= jointlimits::kMinLength)) { setError(error, "os dois pontos estao muito proximos"); return false; }
    }
    if (jointCount(scene) >= jointlimits::kMaxJoints) { setError(error, "limite de juntas atingido"); return false; }

    // Daqui em diante pode lançar (mundo cheio, parâmetros do motor): desfaz o pivô se der errado.
    BodyId bodyB = p.b;
    bool madePivot = false;
    try {
        if (toWorld) {
            bodyB = makePivot(scene, p.anchor);
            madePivot = true;
        }
        switch (p.kind) {
            case JointKind::Hinge: w.createHingeJoint(p.a, bodyB, p.anchor, axis, p.collideConnected); break;
            case JointKind::BallSocket: w.createBallSocketJoint(p.a, bodyB, p.anchor, p.collideConnected); break;
            case JointKind::Distance:
                w.createDistanceJoint(p.a, bodyB, anchorA, anchorB, length, p.collideConnected);
                break;
        }
    } catch (const std::exception& ex) {
        if (madePivot) deleteBody(scene, bodyB, true);
        setError(error, "o motor recusou a junta");
        (void)ex;
        return false;
    }
    w.setAwake(p.a, true);
    if (!toWorld) w.setAwake(p.b, true);
    scene.links.push_back({p.a, bodyB});
    return true;
}

bool destroyLastJoint(Scene& scene) {
    if (!scene.world) return false;
    const JointConstraint* last = nullptr;
    BodyId a = kInvalidBodyId, b = kInvalidBodyId;
    scene.world->forEachJoint([&](const JointConstraint& j, BodyId ja, BodyId jb) {
        last = &j;
        a = ja;
        b = jb;
    });
    if (!last) return false;
    scene.world->destroyJoint(*last);
    // remove o último link que liga o mesmo par (em qualquer ordem)
    for (std::size_t i = scene.links.size(); i-- > 0;) {
        const SceneLink& l = scene.links[i];
        if ((l.a == a && l.b == b) || (l.a == b && l.b == a)) {
            scene.links.erase(scene.links.begin() + static_cast<std::ptrdiff_t>(i));
            break;
        }
    }
    dropOrphanPivot(scene, a);
    dropOrphanPivot(scene, b);
    return true;
}

bool pointOnRay(const Scene& scene, const Vector3& origin, const Vector3& direction, Vector3& point) {
    if (!scene.world || !origin.isFinite() || !direction.isFinite() || direction.lengthSquared() < Real(1e-12)) return false;
    RaycastHit hit;
    if (scene.world->raycast(origin, direction, Real(500), hit) && hit.point.isFinite()) {
        point = hit.point;
        return true;
    }
    const Vector3 d = direction.normalized();
    if (d.y < Real(-1e-4) && origin.y > 0) {   // plano y = 0
        const Real t = -origin.y / d.y;
        if (t < Real(500)) {
            point = origin + d * t;
            return point.isFinite();
        }
    }
    return false;
}

} // namespace viewer
