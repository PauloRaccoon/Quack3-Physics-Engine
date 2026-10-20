// =============================================================================
// robustness_tests.cpp — regressões dos defeitos achados pelo fuzzing (Fase 8).
//
// Cada teste reproduz, de forma DETERMINÍSTICA, uma entrada hostil que antes corrompia o
// estado do motor (NaN em corpos estáticos, exceção de tipo errado, pose infinita...).
// O fuzz (fuzz_tests.cpp) encontra a classe de erro; estes testes apontam a causa.
//
// Contrato verificado:
//   - entradas NÃO finitas em setters/forças/impulsos são ignoradas (o estado não muda);
//   - construtores recusam entradas inválidas SEMPRE com std::invalid_argument;
//   - valores finitos, mas fora do envelope suportado (kMin/kMaxShapeExtent, kMaxCoordinate,
//     kMaxFriction), são recusados (construtores) ou ignorados/limitados (setters);
//   - o estado do mundo continua finito e com |v| <= teto mesmo com razões de massa extremas.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <typeinfo>
#include <vector>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/pipeline/PhysicsWorld.h"

using namespace physics;

static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond)                                                                   \
    do {                                                                              \
        ++g_checks;                                                                   \
        if (!(cond)) {                                                                \
            ++g_failures;                                                             \
            std::printf("FALHOU %s:%d  %s\n", __FILE__, __LINE__, #cond);             \
        }                                                                             \
    } while (0)

static const Real kNaN = std::numeric_limits<Real>::quiet_NaN();
static const Real kInf = std::numeric_limits<Real>::infinity();
static const Real kBig = std::numeric_limits<Real>::max();

/// true se f() lança EXATAMENTE std::invalid_argument (nem logic_error, nem outra coisa).
template <typename F>
static bool throwsInvalidArgument(F&& f) {
    try {
        f();
    } catch (const std::invalid_argument&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

static RigidBodyDesc boxDesc(BodyType type, const Vector3& pos, Real mass = Real(1)) {
    RigidBodyDesc d;
    d.type = type;
    d.shape = std::make_shared<BoxShape>(Vector3(Real(0.5), Real(0.5), Real(0.5)));
    d.mass = mass;
    d.transform = Transform(pos);
    return d;
}

// ---------------------------------------------------------------------------------
// 1) Impulsos/forças não finitos: corpo estático NÃO pode virar NaN (0 · Inf = NaN)
// ---------------------------------------------------------------------------------
static void testNonFiniteImpulses() {
    RigidBody stat(boxDesc(BodyType::Static, Vector3::zero()));
    stat.applyAngularImpulse(Vector3(kInf, 0, 0));
    stat.applyLinearImpulse(Vector3(kNaN, kInf, -kInf));
    stat.applyImpulseAtPoint(Vector3(kInf, kInf, kInf), Vector3(1, 2, 3));
    CHECK(stat.getAngularVelocity().isFinite() && stat.getAngularVelocity().lengthSquared() == 0);
    CHECK(stat.getLinearVelocity().isFinite() && stat.getLinearVelocity().lengthSquared() == 0);

    // Corpo dinâmico: entradas não finitas não mudam nada.
    RigidBody dyn(boxDesc(BodyType::Dynamic, Vector3::zero()));
    dyn.setLinearVelocity(Vector3(1, 2, 3));
    dyn.setAngularVelocity(Vector3(0, 1, 0));
    dyn.applyLinearImpulse(Vector3(kNaN, 0, 0));
    dyn.applyAngularImpulse(Vector3(0, kInf, 0));
    dyn.applyImpulseAtPoint(Vector3(1, 1, 1), Vector3(kNaN, 0, 0));
    dyn.applyForce(Vector3(kInf, 0, 0));
    dyn.applyTorque(Vector3(0, 0, kNaN));
    dyn.applyForceAtPoint(Vector3(1, 0, 0), Vector3(0, kInf, 0));
    dyn.setLinearVelocity(Vector3(kNaN, 0, 0));
    dyn.setAngularVelocity(Vector3(0, kInf, 0));
    CHECK(dyn.getLinearVelocity().nearlyEqual(Vector3(1, 2, 3), Real(1e-6)));
    CHECK(dyn.getAngularVelocity().nearlyEqual(Vector3(0, 1, 0), Real(1e-6)));
    CHECK(dyn.getForce().lengthSquared() == 0 && dyn.getTorque().lengthSquared() == 0);

    // Impulso finito mas que estouraria a velocidade (1e30 · 1e30): ignorado, não vira Inf.
    RigidBody light(boxDesc(BodyType::Dynamic, Vector3::zero(), Real(1e-3)));
    light.applyLinearImpulse(Vector3(kBig, 0, 0));
    CHECK(light.getLinearVelocity().isFinite());

    // Impulso normal continua funcionando: Δv = J/m.
    RigidBody ok(boxDesc(BodyType::Dynamic, Vector3::zero(), Real(2)));
    ok.applyLinearImpulse(Vector3(4, 0, 0));
    CHECK(std::abs(ok.getLinearVelocity().x - Real(2)) < Real(1e-5));
}

// ---------------------------------------------------------------------------------
// 2) Parâmetros de material/dinâmica: NaN passava por `d < 0 ? 0 : d`
// ---------------------------------------------------------------------------------
static void testNonFiniteParameters() {
    RigidBody b(boxDesc(BodyType::Dynamic, Vector3::zero()));
    const Real f0 = b.getFriction(), r0 = b.getRestitution();
    const Real ld0 = b.getLinearDamping(), ad0 = b.getAngularDamping(), gs0 = b.getGravityScale();

    b.setFriction(kNaN);       b.setFriction(kInf);
    b.setRestitution(kNaN);    b.setRestitution(-kInf);
    b.setLinearDamping(kNaN);  b.setLinearDamping(kInf);
    b.setAngularDamping(kNaN); b.setAngularDamping(kInf);
    b.setGravityScale(kNaN);   b.setGravityScale(-kInf);
    CHECK(b.getFriction() == f0 && b.getRestitution() == r0);
    CHECK(b.getLinearDamping() == ld0 && b.getAngularDamping() == ad0 && b.getGravityScale() == gs0);

    // Finito mas astronômico: atrito é limitado (a mistura sqrt(μa μb) de FLT_MAX daria Inf).
    b.setFriction(kBig);
    CHECK(b.getFriction() == kMaxFriction);
    b.setFriction(-5);
    CHECK(b.getFriction() == 0);
    b.setRestitution(7);
    CHECK(b.getRestitution() == 1);

    // Construtor: parâmetros não finitos são recusados com invalid_argument.
    for (int field = 0; field < 5; ++field) {
        RigidBodyDesc d = boxDesc(BodyType::Dynamic, Vector3::zero());
        Real* p = field == 0 ? &d.friction : field == 1 ? &d.restitution : field == 2 ? &d.linearDamping
                : field == 3 ? &d.angularDamping : &d.gravityScale;
        *p = kNaN;
        CHECK(throwsInvalidArgument([&] { RigidBody rb(d); }));
        *p = kInf;
        CHECK(throwsInvalidArgument([&] { RigidBody rb(d); }));
    }

    // Efeito observável no mundo: nada disso contamina a simulação (antes: ω = NaN no estático).
    PhysicsWorld w;
    RigidBodyDesc g = boxDesc(BodyType::Static, Vector3(0, -1, 0));
    const BodyId gid = w.createBody(g);
    const BodyId bid = w.createBody(boxDesc(BodyType::Dynamic, Vector3(0, 2, 0)));
    w.getBody(gid).setAngularDamping(kNaN);
    w.getBody(bid).setLinearDamping(kNaN);
    w.getBody(bid).setGravityScale(kNaN);
    for (int i = 0; i < 30; ++i) w.step();
    CHECK(w.getBody(gid).getAngularVelocity().isFinite() && w.getBody(bid).getTransform().isFinite());
}

// ---------------------------------------------------------------------------------
// 3) Formas: tamanhos finitos mas fora do envelope lançavam logic_error (topologia degenerada)
// ---------------------------------------------------------------------------------
static void testShapeLimits() {
    const Real bad[] = {Real(0), Real(-1), Real(1e-30), kMinShapeExtent * Real(0.5), kMaxShapeExtent * Real(2),
                        Real(1e30), kBig, kNaN, kInf, -kInf};
    for (Real v : bad) {
        CHECK(throwsInvalidArgument([&] { BoxShape s(Vector3(v, 1, 1)); }));
        CHECK(throwsInvalidArgument([&] { BoxShape s(Vector3(1, v, 1)); }));
        CHECK(throwsInvalidArgument([&] { BoxShape s(Vector3(1, 1, v)); }));
        CHECK(throwsInvalidArgument([&] { SphereShape s(v); }));
    }
    // Os limites do envelope são aceitos (e a topologia monta).
    CHECK(!throwsInvalidArgument([&] { BoxShape s(Vector3(kMinShapeExtent, kMinShapeExtent, kMinShapeExtent)); }));
    CHECK(!throwsInvalidArgument([&] { BoxShape s(Vector3(kMaxShapeExtent, kMaxShapeExtent, kMaxShapeExtent)); }));
    CHECK(!throwsInvalidArgument([&] { SphereShape s(kMinShapeExtent); }));
    CHECK(!throwsInvalidArgument([&] { SphereShape s(kMaxShapeExtent); }));

    // overlapSphere com raio inválido: documentado como "retorna 0", sem lançar.
    PhysicsWorld w;
    w.createBody(boxDesc(BodyType::Dynamic, Vector3::zero()));
    std::vector<BodyId> out;
    bool threw = false;
    int n = -1;
    try {
        n = w.overlapSphere(Vector3::zero(), Real(1e-30), out);
        n += w.overlapSphere(Vector3::zero(), kBig, out);
        n += w.overlapSphere(Vector3::zero(), kNaN, out);
    } catch (...) {
        threw = true;
    }
    CHECK(!threw && n == 0 && out.empty());
}

// ---------------------------------------------------------------------------------
// 4) Massa e poses: centro de massa/posição astronômicos produziam pose = Inf
// ---------------------------------------------------------------------------------
static void testMassAndPoseLimits() {
    // fromMassAndInertia com centro de massa absurdo: invalid_argument.
    CHECK(throwsInvalidArgument([&] { MassData::fromMassAndInertia(1, Matrix3::identity(), Vector3(0, 0, kBig)); }));
    CHECK(throwsInvalidArgument([&] { MassData::fromMassAndInertia(1, Matrix3::identity(), Vector3(kNaN, 0, 0)); }));

    // setMassData com MassData montada à mão (sem passar pela fábrica): ignorada, pose intacta.
    RigidBody b(boxDesc(BodyType::Dynamic, Vector3(1, 2, 3)));
    const Transform before = b.getTransform();
    const Real invMassBefore = b.getInvMass();
    MassData evil;
    evil.mass = 1;
    evil.invMass = 1;
    evil.centerOfMass = Vector3(0, 0, kBig);
    evil.inertia = Matrix3::identity();
    evil.invInertia = Matrix3::identity();
    b.setMassData(evil);
    CHECK(b.getTransform().isFinite() && b.getTransform().position.nearlyEqual(before.position, Real(1e-6)));
    evil.centerOfMass = Vector3::zero();
    evil.invMass = kNaN;
    b.setMassData(evil);
    CHECK(b.getInvMass() == invMassBefore);   // MassData inválida (invMass NaN): o corpo fica como estava

    // Posição fora do mundo suportado: construtor recusa; teleporte é ignorado.
    RigidBodyDesc d = boxDesc(BodyType::Dynamic, Vector3(kMaxCoordinate * 2, 0, 0));
    CHECK(throwsInvalidArgument([&] { RigidBody rb(d); }));
    PhysicsWorld w;
    const BodyId id = w.createBody(boxDesc(BodyType::Dynamic, Vector3(1, 1, 1)));
    w.setBodyTransform(id, Transform(Vector3(0, kBig, 0)));
    w.setBodyTransform(id, Transform(Vector3(kMaxCoordinate * 2, 0, 0)));
    CHECK(w.getBody(id).getTransform().position.nearlyEqual(Vector3(1, 1, 1), Real(1e-4)));
    w.step();
    CHECK(w.getBody(id).getTransform().isFinite());
}

// ---------------------------------------------------------------------------------
// 5) Teto de velocidade: em float, 1e30² estourava e a velocidade virava ZERO (não 200)
// ---------------------------------------------------------------------------------
static void testSpeedCap() {
    // Vector3::clampedLength, à prova de overflow.
    CHECK(Vector3(kBig, 0, 0).clampedLength(200).nearlyEqual(Vector3(200, 0, 0), Real(1e-3)));
    CHECK(Vector3(Real(1e30), Real(1e30), 0).clampedLength(200).length() > Real(199.9));
    CHECK(Vector3(3, 4, 0).clampedLength(10).nearlyEqual(Vector3(3, 4, 0), Real(1e-6)));
    CHECK(Vector3(30, 40, 0).clampedLength(10).nearlyEqual(Vector3(6, 8, 0), Real(1e-4)));
    CHECK(Vector3(kNaN, 0, 0).clampedLength(10).lengthSquared() == 0);
    CHECK(Vector3(1, 0, 0).clampedLength(0).lengthSquared() == 0);

    // No mundo: a velocidade injetada é limitada ao teto, preservando a direção.
    for (Real speed : {Real(1e10), Real(1e30), kBig}) {
        PhysicsWorld w;
        w.setGravity(Vector3::zero());
        const BodyId id = w.createBody(boxDesc(BodyType::Dynamic, Vector3(0, 100, 0)));
        w.getBody(id).setLinearVelocity(Vector3(speed, 0, 0));
        w.step();
        const Real v = w.getBody(id).getLinearVelocity().x;
        CHECK(v > kMaxLinearSpeed * Real(0.99) && v <= kMaxLinearSpeed * Real(1.0001));
    }
}

// ---------------------------------------------------------------------------------
// 6) Juntas com âncoras/comprimentos fora do mundo suportado
// ---------------------------------------------------------------------------------
static void testJointLimits() {
    PhysicsWorld w;
    const BodyId a = w.createBody(boxDesc(BodyType::Dynamic, Vector3(0, 0, 0)));
    const BodyId b = w.createBody(boxDesc(BodyType::Dynamic, Vector3(2, 0, 0)));
    const Vector3 far(0, 0, kBig);
    CHECK(throwsInvalidArgument([&] { w.createBallSocketJoint(a, b, far); }));
    CHECK(throwsInvalidArgument([&] { w.createHingeJoint(a, b, far, Vector3::unitY()); }));
    CHECK(throwsInvalidArgument([&] { w.createDistanceJoint(a, b, far, Vector3::zero()); }));
    CHECK(throwsInvalidArgument([&] { w.createDistanceJoint(a, b, Vector3::zero(), Vector3(1, 0, 0), kBig); }));
    CHECK(throwsInvalidArgument([&] { w.createDistanceJoint(a, b, Vector3::zero(), Vector3(1, 0, 0), kNaN); }));
    CHECK(w.getStats().joints == 0);
    // Uma junta válida continua funcionando.
    w.createBallSocketJoint(a, b, Vector3(1, 0, 0));
    CHECK(w.getStats().joints == 1);
    for (int i = 0; i < 20; ++i) w.step();
    CHECK(w.getBody(a).getTransform().isFinite() && w.getBody(b).getTransform().isFinite());
}

// ---------------------------------------------------------------------------------
// 7) Razões de massa extremas: o estado fica finito e dentro do teto (float perde precisão,
//    mas nunca produz NaN/Inf nem |v| absurdo)
// ---------------------------------------------------------------------------------
static void testExtremeMassRatios() {
    for (double heavy : {1e3, 1e9, 1e12, 1e15}) {
        PhysicsWorld w;
        w.createBody(boxDesc(BodyType::Static, Vector3(0, -0.5, 0)));
        RigidBodyDesc d = boxDesc(BodyType::Dynamic, Vector3(0, Real(0.5), 0), Real(heavy));
        const BodyId id = w.createBody(d);
        for (int i = 0; i < 90; ++i) w.step();   // repouso: manifold com impulso ~ m g Δt
        // Troca a massa para 1 kg com o manifold persistido: o warm start vira um chute enorme.
        w.getBody(id).setMassData(MassData::fromMassAndInertia(1, Matrix3::identity(), Vector3::zero()));
        for (int i = 0; i < 10; ++i) w.step();
        const RigidBody& rb = w.getBody(id);
        CHECK(rb.getTransform().isFinite());
        CHECK(rb.getLinearVelocity().isFinite() && rb.getAngularVelocity().isFinite());
        CHECK(rb.getLinearVelocity().length() <= kMaxLinearSpeed * Real(1.001));
        CHECK(rb.getAngularVelocity().length() <= kMaxAngularSpeed * Real(1.001));
        for (const ContactPair* cp : w.getContacts()) {
            for (int i = 0; i < cp->manifold.count; ++i) {
                CHECK(std::isfinite(cp->manifold.points[i].normalImpulse) && cp->manifold.points[i].normalImpulse >= 0);
                CHECK(cp->manifold.points[i].tangentImpulse.isFinite());
            }
        }
    }
}

int main() {
    testNonFiniteImpulses();
    testNonFiniteParameters();
    testShapeLimits();
    testMassAndPoseLimits();
    testSpeedCap();
    testJointLimits();
    testExtremeMassRatios();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
