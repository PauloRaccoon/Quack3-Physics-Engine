// =============================================================================
// PhysicsWorld.cpp
// =============================================================================
#include "physics/pipeline/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace physics {

namespace {
/// Descrição de um "corpo vazio" usado para liberar o slot de um corpo destruído.
RigidBodyDesc blankDesc() {
    RigidBodyDesc d;
    d.type = BodyType::Static;   // sem forma, sem massa: inerte
    return d;
}
} // namespace

PhysicsWorld::PhysicsWorld(const PhysicsWorldSettings& s)
    : gravity_(s.gravity.isFinite() ? s.gravity : Vector3(0, -kGravity, 0)),
      integrator_(s.integrator),
      accumulator_(s.fixedTimeStep, s.maxSubSteps),   // valida Δt e maxSubSteps (lança invalid_argument)
      collision_(s.collision),
      solver_(s.solver),
      maxBodies_(s.maxBodies) {
    if (s.maxBodies == 0) throw std::invalid_argument("PhysicsWorld: maxBodies deve ser > 0");

    // Capacidade FIXA: garante endereços estáveis para RigidBody* (juntas e solver).
    bodies_.reserve(maxBodies_);
    motion_.reserve(maxBodies_);
    alive_.reserve(maxBodies_);
    dirty_.reserve(maxBodies_);

    collision_.setPairFilter([this](BodyId a, BodyId b) { return acceptPair(a, b); });
}

bool PhysicsWorld::acceptPair(BodyId a, BodyId b) const {
    if (!isValidBody(a) || !isValidBody(b)) return false;
    // Se nenhum dos dois é dinâmico não há o que resolver.
    if (!bodies_[a].isDynamic() && !bodies_[b].isDynamic()) return false;
    if (noCollidePairs_.count(pairKey(a, b)) != 0) return false;
    return !userFilter_ || userFilter_(a, b);
}

void PhysicsWorld::requireBody(BodyId id, const char* what) const {
    if (!isValidBody(id)) throw std::invalid_argument(std::string("PhysicsWorld: ") + what + " invalido");
}

// =============================================================================
// Corpos
// =============================================================================
BodyId PhysicsWorld::createBody(const RigidBodyDesc& desc) {
    // Constrói primeiro: se a descrição for inválida, lança sem consumir slot.
    RigidBody body(desc);

    BodyId id;
    const bool reuse = !freeIds_.empty();
    if (reuse) {
        id = freeIds_.back();
    } else {
        if (bodies_.size() >= maxBodies_) throw std::length_error("PhysicsWorld: capacidade de corpos esgotada");
        id = static_cast<BodyId>(bodies_.size());
    }
    body.setId(id);

    // Registra na colisão ANTES de mexer nos vetores: se o AABB for inválido, lança limpo.
    if (body.getShape()) {
        collision_.addObject(id, body.getShapePtr(), body.getTransform(), !body.isStatic());
    }

    if (reuse) {
        freeIds_.pop_back();
        bodies_[id] = std::move(body);
        motion_[id] = MotionState(bodies_[id].getTransform());
        alive_[id] = 1;
        dirty_[id] = 1;
    } else {
        bodies_.push_back(std::move(body));
        motion_.emplace_back(bodies_.back().getTransform());
        alive_.push_back(1);
        dirty_.push_back(1);
    }

    if (bridge_) bridge_->onBodyCreated(id, bodies_[id].getTransform());
    return id;
}

void PhysicsWorld::destroyBody(BodyId id) {
    if (!isValidBody(id)) return;

    // Juntas ligadas ao corpo (de trás para frente: a remoção troca com o último).
    for (std::size_t i = joints_.size(); i-- > 0;) {
        if (joints_[i].a == id || joints_[i].b == id) removeJointEntry(i);
    }
    collision_.removeObject(id);
    if (bridge_) bridge_->onBodyDestroyed(id);

    bodies_[id] = RigidBody(blankDesc());   // libera a forma; o endereço do slot é mantido
    alive_[id] = 0;
    dirty_[id] = 0;
    freeIds_.push_back(id);
}

RigidBody& PhysicsWorld::getBody(BodyId id) {
    if (!isValidBody(id)) throw std::out_of_range("PhysicsWorld::getBody: BodyId invalido");
    return bodies_[id];
}

const RigidBody& PhysicsWorld::getBody(BodyId id) const {
    if (!isValidBody(id)) throw std::out_of_range("PhysicsWorld::getBody: BodyId invalido");
    return bodies_[id];
}

void PhysicsWorld::setBodyTransform(BodyId id, const Transform& frame) {
    if (!isValidBody(id) || !frame.isFinite()) return;
    RigidBody& b = bodies_[id];
    b.setTransform(frame);
    motion_[id].reset(b.getTransform());   // sem rastro de interpolação
    dirty_[id] = 1;
    if (b.getShape()) collision_.updateObject(id, b.getTransform());
}

// =============================================================================
// Juntas
// =============================================================================
template <typename J, typename... Args>
J& PhysicsWorld::emplaceJoint(BodyId a, BodyId b, bool collideConnected, Args&&... args) {
    requireBody(a, "BodyId A");
    requireBody(b, "BodyId B");
    if (a == b) throw std::invalid_argument("PhysicsWorld: uma junta precisa de dois corpos diferentes");

    auto joint = std::make_unique<J>(&bodies_[a], &bodies_[b], std::forward<Args>(args)...);
    joint->setCollideConnected(collideConnected);
    J& ref = *joint;

    JointEntry entry;
    entry.joint = std::move(joint);
    entry.a = a;
    entry.b = b;
    entry.noCollide = !collideConnected;
    joints_.push_back(std::move(entry));

    if (!collideConnected) {
        ++noCollidePairs_[pairKey(a, b)];
        collision_.removePair(a, b);   // se já colidiam, para de colidir agora
    }
    return ref;
}

BallSocketJoint& PhysicsWorld::createBallSocketJoint(BodyId a, BodyId b, const Vector3& worldAnchor,
                                                     bool collideConnected) {
    if (!worldAnchor.isFinite()) throw std::invalid_argument("PhysicsWorld: ancora nao finita");
    return emplaceJoint<BallSocketJoint>(a, b, collideConnected, worldAnchor);
}

DistanceJoint& PhysicsWorld::createDistanceJoint(BodyId a, BodyId b, const Vector3& worldAnchorA,
                                                 const Vector3& worldAnchorB, Real length,
                                                 bool collideConnected) {
    if (!worldAnchorA.isFinite() || !worldAnchorB.isFinite()) {
        throw std::invalid_argument("PhysicsWorld: ancora nao finita");
    }
    return emplaceJoint<DistanceJoint>(a, b, collideConnected, worldAnchorA, worldAnchorB, length);
}

HingeJoint& PhysicsWorld::createHingeJoint(BodyId a, BodyId b, const Vector3& worldAnchor,
                                           const Vector3& worldAxis, bool collideConnected) {
    if (!worldAnchor.isFinite()) throw std::invalid_argument("PhysicsWorld: ancora nao finita");
    return emplaceJoint<HingeJoint>(a, b, collideConnected, worldAnchor, worldAxis);
}

void PhysicsWorld::removeJointEntry(std::size_t index) {
    const JointEntry& e = joints_[index];
    if (e.noCollide) {
        const auto it = noCollidePairs_.find(pairKey(e.a, e.b));
        if (it != noCollidePairs_.end() && --it->second <= 0) noCollidePairs_.erase(it);
    }
    // Remove preservando a ORDEM das juntas (a ordem de resolução é determinística).
    joints_.erase(joints_.begin() + static_cast<std::ptrdiff_t>(index));
}

void PhysicsWorld::destroyJoint(const JointConstraint& joint) {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i].joint.get() == &joint) {
            removeJointEntry(i);
            return;
        }
    }
}

// =============================================================================
// Simulação
// =============================================================================
void PhysicsWorld::stepImpl(Real dt) {
    // 1) Colisão com as poses atuais (fim do passo anterior).
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (!alive_[i]) continue;
        const RigidBody& b = bodies_[i];
        if (b.isStatic() || !b.getShape()) continue;
        collision_.updateObject(static_cast<BodyId>(i), b.getTransform(), b.getLinearVelocity() * dt);
    }
    collision_.detect();

    // 2) Forças, gravidade, amortecimento.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i]) Integrator::integrateVelocities(bodies_[i], gravity_, dt, integrator_);
    }

    // 3) Restrições.
    solver_.clear();
    for (ContactPair* cp : collision_.getTouching()) {
        solver_.addContact(cp->manifold, bodies_[cp->a], bodies_[cp->b]);
    }
    for (JointEntry& e : joints_) solver_.addJoint(*e.joint);
    solver_.solve(dt);

    // 4) Posições.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i]) Integrator::integratePositions(bodies_[i], dt, integrator_);
    }

    // 5) Poses para a interpolação de render.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i] && !bodies_[i].isStatic()) motion_[i].push(bodies_[i].getTransform());
    }
    ++stepCount_;
}

void PhysicsWorld::step() { stepImpl(accumulator_.getFixedDt()); }

int PhysicsWorld::update(Real frameTime) {
    const int steps = accumulator_.advance(frameTime);
    for (int i = 0; i < steps; ++i) stepImpl(accumulator_.getFixedDt());
    syncBridge();
    return steps;
}

// =============================================================================
// Bridge
// =============================================================================
void PhysicsWorld::setBridge(IPhysicsTransformBridge* bridge) {
    bridge_ = bridge;
    if (!bridge_) return;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (!alive_[i]) continue;
        bridge_->onBodyCreated(static_cast<BodyId>(i), motion_[i].interpolate(accumulator_.getAlpha()));
        dirty_[i] = 1;
    }
}

void PhysicsWorld::syncBridge() {
    if (!bridge_) return;
    const Real alpha = accumulator_.getAlpha();
    bridge_->beginSync(alpha);
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (!alive_[i]) continue;
        if (bodies_[i].isStatic() && !dirty_[i]) continue;
        bridge_->onTransformUpdated(static_cast<BodyId>(i), motion_[i].interpolate(alpha));
        dirty_[i] = 0;
    }
    bridge_->endSync();
}

Transform PhysicsWorld::getInterpolatedTransform(BodyId id) const {
    if (!isValidBody(id)) throw std::out_of_range("PhysicsWorld::getInterpolatedTransform: BodyId invalido");
    return motion_[id].interpolate(accumulator_.getAlpha());
}

const MotionState& PhysicsWorld::getMotionState(BodyId id) const {
    if (!isValidBody(id)) throw std::out_of_range("PhysicsWorld::getMotionState: BodyId invalido");
    return motion_[id];
}

WorldStats PhysicsWorld::getStats() const {
    WorldStats s;
    for (std::uint8_t a : alive_) s.bodies += a ? 1 : 0;
    s.joints = static_cast<int>(joints_.size());
    s.pairs = static_cast<int>(collision_.getPairCount());
    s.touchingPairs = static_cast<int>(collision_.getTouching().size());
    for (const ContactPair* cp : collision_.getTouching()) s.contactPoints += cp->manifold.count;
    s.stepCount = stepCount_;
    return s;
}

} // namespace physics
