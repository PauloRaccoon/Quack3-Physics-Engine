// =============================================================================
// PhysicsWorld.cpp
// =============================================================================
#include "physics/pipeline/PhysicsWorld.h"

#include "physics/geometry/SphereShape.h"

#include <algorithm>
#include <chrono>
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
      maxBodies_(s.maxBodies),
      sleep_(s.sleep),
      profiling_(s.enableProfiling) {
    if (s.maxBodies == 0) throw std::invalid_argument("PhysicsWorld: maxBodies deve ser > 0");

    // Capacidade FIXA: garante endereços estáveis para RigidBody* (juntas e solver).
    bodies_.reserve(maxBodies_);
    motion_.reserve(maxBodies_);
    alive_.reserve(maxBodies_);
    dirty_.reserve(maxBodies_);
    islandParent_.reserve(maxBodies_);
    islandMinSleep_.reserve(maxBodies_);
    islandAwake_.reserve(maxBodies_);
    // Eventos e rastreio de contatos: capacidade inicial para que o laço quase nunca aloque.
    events_.reserve(1024);
    prevContacts_.reserve(1024);
    curContacts_.reserve(1024);
    approachSpeed_.reserve(1024);

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

    // Raio de limite (em torno do centro de massa) para a folga das consultas espaciais.
    if (body.getShape()) {
        const AABB local = body.getShape()->computeAABB(Transform());
        const Vector3 far(std::max(std::abs(local.min.x), std::abs(local.max.x)),
                          std::max(std::abs(local.min.y), std::abs(local.max.y)),
                          std::max(std::abs(local.min.z), std::abs(local.max.z)));
        maxBodyRadius_ = std::max(maxBodyRadius_, far.length() + body.getLocalCenterOfMass().length());
    }

    // Registra na colisão ANTES de mexer nos vetores: se o AABB for inválido, lança limpo.
    if (body.getShape()) {
        collision_.addObject(id, body.getShapePtr(), body.getTransform(), !body.isStatic());
    }

    // Um corpo novo que nasce sobre corpos adormecidos os acorda (o par entre um corpo estático e um
    // adormecido nunca rodaria a narrowphase, e o corpo novo poderia ficar "dentro" deles).
    if (body.getShape()) {
        collision_.queryAABB(body.getShape()->computeAABB(body.getTransform()), [&](BodyId other) {
            if (isValidBody(other)) bodies_[other].setAwake(true);
            return true;
        });
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

    // Quem estava apoiado/encostado neste corpo precisa acordar (o apoio some).
    wakeTouching(id);

    // Eventos de FIM dos contatos que este corpo tinha (os dados dos eventos são cópias).
    for (std::size_t k = 0; k < prevContacts_.size();) {
        if (prevContacts_[k].last.a == id || prevContacts_[k].last.b == id) {
            ContactEvent e = prevContacts_[k].last;
            e.type = ContactEvent::Type::End;
            events_.push_back(e);
            prevContacts_.erase(prevContacts_.begin() + static_cast<std::ptrdiff_t>(k));   // preserva a ordem por chave
        } else {
            ++k;
        }
    }

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

    // Fora de um passo, entrega já os eventos de fim (dentro de um callback, o laço de entrega os pega).
    if (!stepping_) dispatchEvents();
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
    if (!isValidBody(id) || !frame.isFinite() || !frame.position.isWithin(kMaxCoordinate)) return;
    RigidBody& b = bodies_[id];
    wakeTouching(id);   // quem estava apoiado nele (ou encostado) acorda: o apoio saiu do lugar
    b.setTransform(frame);   // acorda o próprio corpo, se for dinâmico
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
    bodies_[a].setAwake(true);   // a junta liga os dois: acordados, a ilha deles nasce acordada
    bodies_[b].setAwake(true);
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
    if (!worldAnchor.isWithin(kMaxCoordinate)) throw std::invalid_argument("PhysicsWorld: ancora nao finita ou fora de kMaxCoordinate");
    return emplaceJoint<BallSocketJoint>(a, b, collideConnected, worldAnchor);
}

DistanceJoint& PhysicsWorld::createDistanceJoint(BodyId a, BodyId b, const Vector3& worldAnchorA,
                                                 const Vector3& worldAnchorB, Real length,
                                                 bool collideConnected) {
    if (!worldAnchorA.isWithin(kMaxCoordinate) || !worldAnchorB.isWithin(kMaxCoordinate)) {
        throw std::invalid_argument("PhysicsWorld: ancora nao finita ou fora de kMaxCoordinate");
    }
    return emplaceJoint<DistanceJoint>(a, b, collideConnected, worldAnchorA, worldAnchorB, length);
}

HingeJoint& PhysicsWorld::createHingeJoint(BodyId a, BodyId b, const Vector3& worldAnchor,
                                           const Vector3& worldAxis, bool collideConnected) {
    if (!worldAnchor.isWithin(kMaxCoordinate)) throw std::invalid_argument("PhysicsWorld: ancora nao finita ou fora de kMaxCoordinate");
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
            bodies_[joints_[i].a].setAwake(true);   // sem a junta, os corpos podem passar a cair
            bodies_[joints_[i].b].setAwake(true);
            removeJointEntry(i);
            return;
        }
    }
}

// =============================================================================
// Simulação
// =============================================================================
// -----------------------------------------------------------------------------
// Sleeping: ilhas por union-find
// -----------------------------------------------------------------------------
std::uint32_t PhysicsWorld::findRoot(std::uint32_t i) {
    while (islandParent_[i] != i) {
        islandParent_[i] = islandParent_[islandParent_[i]];   // path halving
        i = islandParent_[i];
    }
    return i;
}

void PhysicsWorld::wakeTouching(BodyId id) {
    for (const ContactPair* cp : collision_.getTouching()) {
        if (cp->a == id && isValidBody(cp->b)) bodies_[cp->b].setAwake(true);
        else if (cp->b == id && isValidBody(cp->a)) bodies_[cp->a].setAwake(true);
    }
}

void PhysicsWorld::updateIslands() {
    const std::size_t n = bodies_.size();
    islandParent_.resize(n);
    islandAwake_.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) islandParent_[i] = static_cast<std::uint32_t>(i);

    // Liga dois corpos DINÂMICOS (estáticos não propagam ilhas). Um dinâmico encostado/ligado a um
    // cinemático em movimento acorda: a plataforma o leva consigo.
    auto link = [&](BodyId a, BodyId b) {
        RigidBody& A = bodies_[a];
        RigidBody& B = bodies_[b];
        if (A.isDynamic() && B.isDynamic()) {
            const std::uint32_t ra = findRoot(a), rb = findRoot(b);
            if (ra != rb) islandParent_[std::max(ra, rb)] = std::min(ra, rb);   // raiz = menor índice (determinístico)
        } else if (A.isDynamic() && isMovingKinematic(B)) {
            A.setAwake(true);
        } else if (B.isDynamic() && isMovingKinematic(A)) {
            B.setAwake(true);
        }
    };
    for (const ContactPair* cp : collision_.getTouching()) link(cp->a, cp->b);
    for (const JointEntry& e : joints_) link(e.a, e.b);

    // Uma ilha com algum membro acordado acorda inteira.
    for (std::size_t i = 0; i < n; ++i) {
        if (alive_[i] && bodies_[i].isDynamic() && bodies_[i].isAwake()) islandAwake_[findRoot(static_cast<std::uint32_t>(i))] = 1;
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (alive_[i] && bodies_[i].isDynamic() && !bodies_[i].isAwake() &&
            islandAwake_[findRoot(static_cast<std::uint32_t>(i))]) {
            bodies_[i].setAwake(true);
        }
    }
}

void PhysicsWorld::updateSleep(Real dt) {
    const std::size_t n = bodies_.size();
    const Real linSq = sleep_.linearThreshold > Real(0) && isFinite(sleep_.linearThreshold)
                           ? sleep_.linearThreshold * sleep_.linearThreshold : Real(0);
    const Real angSq = sleep_.angularThreshold > Real(0) && isFinite(sleep_.angularThreshold)
                           ? sleep_.angularThreshold * sleep_.angularThreshold : Real(0);
    const Real timeToSleep = sleep_.timeToSleep > Real(0) && isFinite(sleep_.timeToSleep) ? sleep_.timeToSleep : Real(0);

    islandMinSleep_.assign(n, kRealMax);
    lastIslands_ = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!alive_[i]) continue;
        RigidBody& b = bodies_[i];
        if (!b.isDynamic() || !b.isAwake()) continue;
        const bool still = sleep_.enabled && b.getAllowSleep() &&
                           b.getLinearVelocity().lengthSquared() <= linSq &&
                           b.getAngularVelocity().lengthSquared() <= angSq;
        b.setSleepTime(still ? b.getSleepTime() + dt : Real(0));
        const std::uint32_t r = findRoot(static_cast<std::uint32_t>(i));
        if (islandMinSleep_[r] == kRealMax) ++lastIslands_;
        islandMinSleep_[r] = std::min(islandMinSleep_[r], b.getSleepTime());
    }
    if (!sleep_.enabled) return;

    // A ilha só adormece INTEIRA: todos os membros em repouso pelo tempo exigido.
    // (lastIslands_ passa a contar só as ilhas que continuam acordadas depois deste passo.)
    islandAwake_.assign(n, 0);   // reaproveitado como marcador "esta ilha acabou de adormecer"
    for (std::size_t i = 0; i < n; ++i) {
        if (!alive_[i]) continue;
        RigidBody& b = bodies_[i];
        if (!b.isDynamic() || !b.isAwake()) continue;
        const std::uint32_t root = findRoot(static_cast<std::uint32_t>(i));
        if (islandMinSleep_[root] < timeToSleep) continue;
        if (!islandAwake_[root]) {
            islandAwake_[root] = 1;
            --lastIslands_;
        }
        b.setAwake(false);                                  // zera v e ω
        motion_[i].reset(b.getTransform());                 // dormindo, a interpolação não pode "andar"
        dirty_[i] = 1;                                      // a bridge recebe a pose final uma vez
        if (b.getShape()) collision_.setObjectAsleep(static_cast<BodyId>(i), true);
    }
}

// -----------------------------------------------------------------------------
// Eventos de contato
// -----------------------------------------------------------------------------
void PhysicsWorld::collectContactEvents() {
    const std::vector<ContactPair*>& touching = collision_.getTouching();
    curContacts_.clear();
    for (std::size_t k = 0; k < touching.size(); ++k) {
        const ContactPair& cp = *touching[k];
        const ContactManifold& m = cp.manifold;
        const int count = std::max(m.count, 1);

        TrackedContact t;
        t.key = pairKey(cp.a, cp.b);
        ContactEvent& e = t.last;
        e.type = ContactEvent::Type::Begin;
        e.a = cp.a;
        e.b = cp.b;
        e.normal = m.normal;
        e.pointCount = m.count;
        Vector3 sum;
        Real maxDepth = -kRealMax, impulse = Real(0);
        for (int i = 0; i < m.count; ++i) {
            sum += m.points[i].getPosition();
            maxDepth = std::max(maxDepth, m.points[i].depth);
            impulse += m.points[i].normalImpulse;
        }
        e.point = sum / static_cast<Real>(count);
        e.depth = m.count > 0 ? maxDepth : Real(0);
        e.normalImpulse = impulse;
        e.approachSpeed = k < approachSpeed_.size() ? approachSpeed_[k] : Real(0);
        curContacts_.push_back(t);
    }
    // getTouching() sai da ordem do std::map (chave crescente); garante a ordem mesmo assim,
    // pois a comparação abaixo depende dela.
    const auto byKey = [](const TrackedContact& x, const TrackedContact& y) { return x.key < y.key; };
    if (!std::is_sorted(curContacts_.begin(), curContacts_.end(), byKey)) {
        std::sort(curContacts_.begin(), curContacts_.end(), byKey);
    }

    // Diferença entre o passo anterior e o atual (duas listas ordenadas): O(pares), sem alocação.
    std::size_t i = 0, j = 0;
    while (i < prevContacts_.size() || j < curContacts_.size()) {
        if (j >= curContacts_.size() || (i < prevContacts_.size() && prevContacts_[i].key < curContacts_[j].key)) {
            ContactEvent e = prevContacts_[i].last;   // o contato acabou: último dado conhecido
            e.type = ContactEvent::Type::End;
            events_.push_back(e);
            ++i;
        } else if (i >= prevContacts_.size() || curContacts_[j].key < prevContacts_[i].key) {
            events_.push_back(curContacts_[j].last);  // contato novo
            ++j;
        } else {
            ++i;
            ++j;
        }
    }
    prevContacts_.swap(curContacts_);
}

void PhysicsWorld::dispatchEvents() {
    if (dispatching_) return;   // o laço que já está entregando pega os eventos novos
    struct Guard {
        bool& f;
        explicit Guard(bool& x) : f(x) { f = true; }
        ~Guard() { f = false; }
    } guard(dispatching_);
    // Por índice: o callback pode criar/destruir corpos e, com isso, acrescentar eventos.
    while (dispatched_ < events_.size()) {
        const ContactEvent e = events_[dispatched_++];
        if (!listener_) continue;
        if (e.type == ContactEvent::Type::Begin) listener_->onContactBegin(e);
        else listener_->onContactEnd(e);
    }
}

// -----------------------------------------------------------------------------
// Passo
// -----------------------------------------------------------------------------
void PhysicsWorld::stepImpl(Real dt) {
    if (stepping_) throw std::logic_error("PhysicsWorld: step()/update() chamado de dentro de um callback de contato");
    struct Guard {
        bool& f;
        explicit Guard(bool& x) : f(x) { f = true; }
        ~Guard() { f = false; }
    } guard(stepping_);

    using Clock = std::chrono::steady_clock;
    Clock::time_point tp[6];
    const bool prof = profiling_;
    if (prof) tp[0] = Clock::now();

    // 1) Colisão com as poses atuais (fim do passo anterior). Corpos adormecidos não se movem:
    //    continuam na árvore como estão e seus pares entre si não rodam a narrowphase.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (!alive_[i]) continue;
        const RigidBody& b = bodies_[i];
        if (b.isStatic() || !b.getShape()) continue;
        const bool asleep = b.isDynamic() && !b.isAwake();
        collision_.setObjectAsleep(static_cast<BodyId>(i), asleep);
        if (!asleep) collision_.updateObject(static_cast<BodyId>(i), b.getTransform(), b.getLinearVelocity() * dt);
    }
    collision_.detect();
    if (prof) tp[1] = Clock::now();

    // 2) Ilhas: acorda quem deve acordar (antes de forças e solver).
    updateIslands();
    if (prof) tp[2] = Clock::now();

    // Velocidade de aproximação de cada contato ANTES de gravidade e solver: é a "velocidade de
    // impacto" dos eventos (com a gravidade somada, um corpo em repouso pareceria bater a g·Δt).
    {
        const std::vector<ContactPair*>& touching = collision_.getTouching();
        approachSpeed_.resize(touching.size());
        for (std::size_t k = 0; k < touching.size(); ++k) {
            const ContactManifold& m = touching[k]->manifold;
            Vector3 p;
            for (int i = 0; i < m.count; ++i) p += m.points[i].getPosition();
            p /= static_cast<Real>(std::max(m.count, 1));
            const Vector3 vrel = bodies_[touching[k]->b].getVelocityAtPoint(p) -
                                 bodies_[touching[k]->a].getVelocityAtPoint(p);
            approachSpeed_[k] = std::max(-vrel.dot(m.normal), Real(0));
        }
    }

    // 3) Forças, gravidade, amortecimento (corpos adormecidos ficam de fora).
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i] && (!bodies_[i].isDynamic() || bodies_[i].isAwake())) {
            Integrator::integrateVelocities(bodies_[i], gravity_, dt, integrator_);
        }
    }

    // 4) Restrições. Só entram as que têm pelo menos um corpo dinâmico ACORDADO (a ilha já foi
    //    acordada por inteiro; sobram contatos entre adormecidos e estáticos, que não têm o que resolver).
    solver_.clear();
    for (ContactPair* cp : collision_.getTouching()) {
        RigidBody& A = bodies_[cp->a];
        RigidBody& B = bodies_[cp->b];
        if (isActiveBody(A) || isActiveBody(B)) solver_.addContact(cp->manifold, A, B);
    }
    for (JointEntry& e : joints_) {
        if (isActiveBody(bodies_[e.a]) || isActiveBody(bodies_[e.b])) solver_.addJoint(*e.joint);
    }
    solver_.solve(dt);
    if (prof) tp[3] = Clock::now();

    // 5) Posições.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i] && (!bodies_[i].isDynamic() || bodies_[i].isAwake())) {
            Integrator::integratePositions(bodies_[i], dt, integrator_);
        }
    }
    solver_.solvePositions();   // NGS: corrige penetrações e juntas movendo as poses (sem criar velocidade)

    // 6) Poses para a interpolação de render.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i] && !bodies_[i].isStatic() && (!bodies_[i].isDynamic() || bodies_[i].isAwake())) {
            motion_[i].push(bodies_[i].getTransform());
        }
    }
    // 7) Consultas espaciais: a árvore da broadphase guarda as poses do INÍCIO do passo; as
    //    consultas passam a usar as poses de agora e inflam a árvore pelo deslocamento máximo
    //    que um ponto de qualquer corpo pode ter feito neste passo: Δt·(|v| + |ω|·R).
    //    (Corpos adormecidos não se moveram: a pose da árvore já é a atual.)
    Real maxLinear = Real(0), maxAngular = Real(0);
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (!alive_[i]) continue;
        const RigidBody& b = bodies_[i];
        if (b.isStatic() || !b.getShape() || (b.isDynamic() && !b.isAwake())) continue;
        collision_.setQueryPose(static_cast<BodyId>(i), b.getTransform());
        maxLinear = std::max(maxLinear, b.getLinearVelocity().length());
        maxAngular = std::max(maxAngular, b.getAngularVelocity().length());
    }
    collision_.setQuerySlack(dt * (maxLinear + maxAngular * maxBodyRadius_) * Real(1.01) + kLinearSlop);
    if (prof) tp[4] = Clock::now();

    // 8) Sleeping e eventos de contato.
    updateSleep(dt);
    collectContactEvents();
    ++stepCount_;
    if (prof) {
        tp[5] = Clock::now();
        const auto us = [&](int a, int b) { return std::chrono::duration<double, std::micro>(tp[b] - tp[a]).count(); };
        timings_.collisionUs = us(0, 1);
        timings_.islandsUs = us(1, 2);
        timings_.constraintsUs = us(2, 3);
        timings_.integrationUs = us(3, 4);
        timings_.sleepEventsUs = us(4, 5);
        timings_.totalUs = us(0, 5);   // sem o tempo gasto nos callbacks do usuário
    }

    // 9) Entrega dos eventos (código do usuário: fora de toda a medição).
    dispatchEvents();
}

bool PhysicsWorld::setFixedTimeStep(Real dt) {
    if (stepping_) return false;
    if (!std::isfinite(dt) || dt < kMinRuntimeTimeStep || dt > kMaxRuntimeTimeStep) return false;
    return accumulator_.setFixedDt(dt);
}

void PhysicsWorld::step() {
    if (stepping_) throw std::logic_error("PhysicsWorld: step() chamado de dentro de um callback de contato");
    clearContactEvents();
    stepImpl(accumulator_.getFixedDt());
}

int PhysicsWorld::update(Real frameTime) {
    if (stepping_) throw std::logic_error("PhysicsWorld: update() chamado de dentro de um callback de contato");
    clearContactEvents();
    const int steps = accumulator_.advance(frameTime);
    for (int i = 0; i < steps; ++i) stepImpl(accumulator_.getFixedDt());
    syncBridge();
    return steps;
}

void PhysicsWorld::setAwake(BodyId id, bool awake) {
    if (!isValidBody(id)) return;
    RigidBody& b = bodies_[id];
    if (!b.isDynamic()) return;
    b.setAwake(awake);
    if (!awake) {
        motion_[id].reset(b.getTransform());
        dirty_[id] = 1;
        if (b.getShape()) collision_.setObjectAsleep(id, true);
    }
}

void PhysicsWorld::wakeAll() {
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (alive_[i]) bodies_[i].setAwake(true);
    }
}

// =============================================================================
// Consultas espaciais
// =============================================================================
bool PhysicsWorld::acceptForQuery(BodyId id, const QueryFilter& f) const {
    if (!isValidBody(id) || id == f.ignoreBody) return false;
    const RigidBody& b = bodies_[id];
    if (b.isStatic() && !f.includeStatic) return false;
    if (b.isKinematic() && !f.includeKinematic) return false;
    if (b.isDynamic() && !f.includeDynamic) return false;
    return !f.predicate || f.predicate(id);
}

namespace {
/// Valida e normaliza a direção do raio. false se a consulta não faz sentido.
bool prepareRay(const Vector3& origin, const Vector3& direction, Real maxDistance, Vector3& unitDir) {
    if (!origin.isFinite() || !direction.isFinite() || !(maxDistance >= Real(0))) return false;
    unitDir = direction.normalized();
    return unitDir.lengthSquared() > Real(0);   // normalized() devolve (0,0,0) para direção nula/degenerada
}
} // namespace

bool PhysicsWorld::raycast(const Vector3& origin, const Vector3& direction, Real maxDistance,
                           RaycastHit& hit, const QueryFilter& filter) const {
    Vector3 dir;
    if (!prepareRay(origin, direction, maxDistance, dir)) return false;
    RaycastHit best;
    if (!collision_.raycastClosest(origin, dir, maxDistance, best,
                                   [&](BodyId id) { return acceptForQuery(id, filter); })) {
        return false;
    }
    hit = best;
    return true;
}

int PhysicsWorld::raycastAll(const Vector3& origin, const Vector3& direction, Real maxDistance,
                             std::vector<RaycastHit>& hits, const QueryFilter& filter) const {
    hits.clear();
    Vector3 dir;
    if (!prepareRay(origin, direction, maxDistance, dir)) return 0;
    collision_.raycastAll(origin, dir, maxDistance, [&](BodyId id) { return acceptForQuery(id, filter); },
                          [&](const RaycastHit& h) {
                              hits.push_back(h);
                              return true;
                          });
    std::sort(hits.begin(), hits.end(), [](const RaycastHit& a, const RaycastHit& b) {
        return a.distance != b.distance ? a.distance < b.distance : a.body < b.body;
    });
    return static_cast<int>(hits.size());
}

int PhysicsWorld::overlapShape(const Shape& shape, const Transform& xf, std::vector<BodyId>& bodies,
                               const QueryFilter& filter) const {
    bodies.clear();
    collision_.overlapShape(shape, xf, [&](BodyId id) { return acceptForQuery(id, filter); },
                            [&](BodyId id) {
                                bodies.push_back(id);
                                return true;
                            });
    std::sort(bodies.begin(), bodies.end());
    return static_cast<int>(bodies.size());
}

int PhysicsWorld::overlapPoint(const Vector3& point, std::vector<BodyId>& bodies,
                               const QueryFilter& filter) const {
    bodies.clear();
    collision_.overlapPoint(point, [&](BodyId id) { return acceptForQuery(id, filter); },
                            [&](BodyId id) {
                                bodies.push_back(id);
                                return true;
                            });
    std::sort(bodies.begin(), bodies.end());
    return static_cast<int>(bodies.size());
}

int PhysicsWorld::overlapSphere(const Vector3& center, Real radius, std::vector<BodyId>& bodies,
                                const QueryFilter& filter) const {
    bodies.clear();
    // Raio fora de [kMinShapeExtent, kMaxShapeExtent] (a faixa que SphereShape aceita) = inválido.
    if (!center.isFinite() || !isFinite(radius) || radius < kMinShapeExtent || radius > kMaxShapeExtent) return 0;
    const SphereShape sphere(radius);
    return overlapShape(sphere, Transform(center), bodies, filter);
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
        // Estáticos e adormecidos não mudam: só vão à bridge ao nascer, ser teletransportados ou adormecer.
        const bool still = bodies_[i].isStatic() || (bodies_[i].isDynamic() && !bodies_[i].isAwake());
        if (still && !dirty_[i]) continue;
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
    for (std::size_t i = 0; i < alive_.size(); ++i) {
        if (!alive_[i]) continue;
        ++s.bodies;
        if (bodies_[i].isDynamic()) (bodies_[i].isAwake() ? s.awakeBodies : s.sleepingBodies) += 1;
    }
    s.islands = lastIslands_;
    s.frozenPairs = static_cast<int>(collision_.getFrozenPairCount());
    s.timings = timings_;
    s.solverContacts = solver_.getStats().contactConstraints;
    s.solverJoints = solver_.getStats().jointConstraints;
    s.joints = static_cast<int>(joints_.size());
    s.pairs = static_cast<int>(collision_.getPairCount());
    s.touchingPairs = static_cast<int>(collision_.getTouching().size());
    for (const ContactPair* cp : collision_.getTouching()) s.contactPoints += cp->manifold.count;
    s.stepCount = stepCount_;
    return s;
}

} // namespace physics
