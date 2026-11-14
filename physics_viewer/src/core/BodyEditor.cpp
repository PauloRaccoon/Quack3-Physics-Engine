// =============================================================================
// BodyEditor.cpp
// =============================================================================
#include "viewer/core/BodyEditor.h"

#include <algorithm>
#include <cmath>

#include "physics/dynamics/MassData.h"

namespace viewer {

using namespace physics;

namespace {

bool editable(const PhysicsWorld& w, BodyId id) { return w.isValidBody(id) && w.getBody(id).isDynamic(); }

/// V14: pose e material também valem para corpos estáticos (para montar o cenário).
bool placeable(const PhysicsWorld& w, BodyId id) { return w.isValidBody(id); }

Real clampTo(Real v, Real lo, Real hi) { return std::min(std::max(v, lo), hi); }

} // namespace

bool readParams(const PhysicsWorld& world, BodyId id, BodyParams& out) {
    if (!world.isValidBody(id)) return false;
    const RigidBody& b = world.getBody(id);
    out.mass = b.getMass();
    out.friction = b.getFriction();
    out.restitution = b.getRestitution();
    out.linearDamping = b.getLinearDamping();
    out.angularDamping = b.getAngularDamping();
    out.gravityScale = b.getGravityScale();
    out.allowSleep = b.getAllowSleep();
    out.awake = b.isAwake();
    return true;
}

bool setMass(PhysicsWorld& world, BodyId id, Real mass) {
    if (!editable(world, id) || !std::isfinite(mass)) return false;
    RigidBody& b = world.getBody(id);
    if (b.getShape() == nullptr) return false;
    mass = clampTo(mass, limits::kMinMass, limits::kMaxMass);
    b.setMassData(MassData::fromShapeMass(*b.getShape(), mass));
    return true;
}

bool setFriction(PhysicsWorld& world, BodyId id, Real v) {
    if (!placeable(world, id) || !std::isfinite(v)) return false;
    world.getBody(id).setFriction(clampTo(v, Real(0), Real(2)));
    return true;
}

bool setRestitution(PhysicsWorld& world, BodyId id, Real v) {
    if (!placeable(world, id) || !std::isfinite(v)) return false;
    world.getBody(id).setRestitution(clampTo(v, Real(0), Real(1)));
    return true;
}

bool setLinearDamping(PhysicsWorld& world, BodyId id, Real v) {
    if (!editable(world, id) || !std::isfinite(v)) return false;
    world.getBody(id).setLinearDamping(clampTo(v, Real(0), limits::kMaxDamping));
    return true;
}

bool setAngularDamping(PhysicsWorld& world, BodyId id, Real v) {
    if (!editable(world, id) || !std::isfinite(v)) return false;
    world.getBody(id).setAngularDamping(clampTo(v, Real(0), limits::kMaxDamping));
    return true;
}

bool setGravityScale(PhysicsWorld& world, BodyId id, Real v) {
    if (!editable(world, id) || !std::isfinite(v)) return false;
    world.getBody(id).setGravityScale(clampTo(v, -limits::kMaxGravityScale, limits::kMaxGravityScale));
    return true;
}

bool setAllowSleep(PhysicsWorld& world, BodyId id, bool allow) {
    if (!editable(world, id)) return false;
    world.getBody(id).setAllowSleep(allow);
    return true;
}

bool setPosition(PhysicsWorld& world, BodyId id, const Vector3& position) {
    if (!placeable(world, id) || !position.isFinite()) return false;
    Transform xf = world.getBody(id).getTransform();
    xf.position = position;
    world.setBodyTransform(id, xf);
    return true;
}

bool setOrientationAxisAngle(PhysicsWorld& world, BodyId id, const Vector3& axis, Real angle) {
    if (!placeable(world, id) || !axis.isFinite() || !std::isfinite(angle) || axis.lengthSquared() < Real(1e-12)) return false;
    Transform xf = world.getBody(id).getTransform();
    xf.rotation = Quaternion::fromAxisAngle(axis.normalized(), angle);
    world.setBodyTransform(id, xf);
    return true;
}

Quaternion quaternionFromAnglesXYZ(Real wx, Real wy, Real wz) {
    const Quaternion qx = Quaternion::fromAxisAngle(Vector3(1, 0, 0), wx);
    const Quaternion qy = Quaternion::fromAxisAngle(Vector3(0, 1, 0), wy);
    const Quaternion qz = Quaternion::fromAxisAngle(Vector3(0, 0, 1), wz);
    return (qz * qy * qx).normalized();
}

Vector3 anglesXYZFromQuaternion(const Quaternion& qIn) {
    const Quaternion q = qIn.normalized();
    // R = Rz(c) Ry(b) Rx(a): R20 = −sen b, R21 = cos b sen a, R22 = cos b cos a, R10 = sen c cos b, R00 = cos c cos b
    const Vector3 c0 = q.rotate(Vector3(1, 0, 0)), c1 = q.rotate(Vector3(0, 1, 0)), c2 = q.rotate(Vector3(0, 0, 1));
    const double r20 = static_cast<double>(c0.z);
    const double b = std::asin(std::fmax(-1.0, std::fmin(1.0, -r20)));
    double a = 0.0, c = 0.0;
    if (std::fabs(r20) < 1.0 - 1e-9) {
        a = std::atan2(static_cast<double>(c1.z), static_cast<double>(c2.z));
        c = std::atan2(static_cast<double>(c0.y), static_cast<double>(c0.x));
    } else {   // gimbal lock: só a soma/diferença de ωx e ωz importa; fixa ωx = 0
        a = 0.0;
        c = std::atan2(-static_cast<double>(c1.x), static_cast<double>(c1.y));
    }
    return Vector3(static_cast<Real>(a), static_cast<Real>(b), static_cast<Real>(c));
}

bool setOrientationAnglesXYZ(PhysicsWorld& world, BodyId id, Real wx, Real wy, Real wz) {
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) return false;
    return setOrientationQuaternion(world, id, quaternionFromAnglesXYZ(wx, wy, wz));
}

bool setOrientationQuaternion(PhysicsWorld& world, BodyId id, const Quaternion& q) {
    if (!placeable(world, id) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) || q.normSquared() < Real(1e-12)) return false;
    Transform xf = world.getBody(id).getTransform();
    xf.rotation = q.normalized();
    world.setBodyTransform(id, xf);
    return true;
}

bool resetOrientation(PhysicsWorld& world, BodyId id) {
    if (!placeable(world, id)) return false;
    Transform xf = world.getBody(id).getTransform();
    xf.rotation = Quaternion::identity();
    world.setBodyTransform(id, xf);
    return true;
}

bool setLinearVelocity(PhysicsWorld& world, BodyId id, const Vector3& v) {
    if (!editable(world, id) || !v.isFinite()) return false;
    world.getBody(id).setLinearVelocity(v);
    return true;
}

bool setAngularVelocity(PhysicsWorld& world, BodyId id, const Vector3& w) {
    if (!editable(world, id) || !w.isFinite()) return false;
    world.getBody(id).setAngularVelocity(w);
    return true;
}

bool zeroVelocities(PhysicsWorld& world, BodyId id) {
    if (!editable(world, id)) return false;
    RigidBody& b = world.getBody(id);
    b.setLinearVelocity(Vector3(0, 0, 0));
    b.setAngularVelocity(Vector3(0, 0, 0));
    return true;
}

bool setAwake(PhysicsWorld& world, BodyId id, bool awake) {
    if (!editable(world, id)) return false;
    world.setAwake(id, awake);
    return true;
}

} // namespace viewer
