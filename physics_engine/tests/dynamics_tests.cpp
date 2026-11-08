// =============================================================================
// dynamics_tests.cpp — testes unitários da Fase 3 (sem frameworks externos).
// =============================================================================
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>

#include "physics/dynamics/Integrator.h"
#include "physics/dynamics/MassData.h"
#include "physics/dynamics/RigidBody.h"
#include "physics/geometry/BoxShape.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/SphereShape.h"

using namespace physics;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) {                                                           \
            ++g_failures;                                                        \
            std::printf("FALHOU %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                        \
    } while (0)

static const Real kTol = Real(1e-3);
static const Real kDt = Real(1) / Real(60);

template <typename F>
static bool throwsInvalidArgument(F&& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; } catch (...) { return false; }
    return false;
}

static void testMassData() {
    BoxShape box(Vector3(1, 2, 3));
    MassData m = MassData::fromShape(box, Real(2));
    CHECK(nearlyEqual(m.mass, Real(96), kTol));                 // 2 * 48
    CHECK(nearlyEqual(m.invMass * m.mass, Real(1), kTol));
    CHECK((m.inertia * m.invInertia).nearlyEqual(Matrix3::identity(), kTol));
    CHECK(m.inertia.nearlyEqual(box.computeInertia(m.mass), kTol));

    MassData fixed = MassData::fromShapeMass(box, Real(12));
    CHECK(nearlyEqual(fixed.mass, Real(12)));
    CHECK(fixed.inertia.nearlyEqual(Matrix3::diagonal(52, 40, 20), kTol));

    MassData inf = MassData::infinite();
    CHECK(inf.invMass == 0 && inf.invInertia.nearlyEqual(Matrix3::zero()) && !inf.hasFiniteMass());

    // Tensor singular → corpo que não rotaciona, sem NaN.
    MassData sing = MassData::fromMassAndInertia(1, Matrix3::zero());
    CHECK(sing.invInertia.isFinite() && sing.invInertia.nearlyEqual(Matrix3::zero()));

    CHECK(throwsInvalidArgument([&] { MassData::fromShape(box, 0); }));
    CHECK(throwsInvalidArgument([&] { MassData::fromShape(box, -1); }));
    CHECK(throwsInvalidArgument([&] { MassData::fromShapeMass(box, std::numeric_limits<Real>::infinity()); }));
    CHECK(throwsInvalidArgument([&] { MassData::fromMassAndInertia(0, Matrix3::identity()); }));
}

static RigidBodyDesc boxDesc(const Vector3& pos = Vector3::zero()) {
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(0.5f, 0.5f, 0.5f));
    d.density = 1;
    d.transform = Transform(pos);
    return d;
}

static void testRigidBodyConstruction() {
    RigidBody b(boxDesc(Vector3(1, 2, 3)));
    CHECK(b.isDynamic() && nearlyEqual(b.getMass(), Real(1), kTol));
    CHECK(b.getTransform().position.nearlyEqual(Vector3(1, 2, 3)));
    CHECK(b.getWorldCenterOfMass().nearlyEqual(Vector3(1, 2, 3)));

    // Forma com centroide fora da origem: COM ≠ origem do frame, mas o frame é preservado.
    const Vector3 h(1, 1, 1);
    std::vector<Vector3> v;
    for (int i = 0; i < 8; ++i) v.emplace_back(((i & 1) ? h.x : -h.x) + 3, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z);
    auto poly = std::make_shared<PolyhedronShape>(v, std::vector<std::vector<int>>{
        {1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}});
    RigidBodyDesc d;
    d.shape = poly;
    d.transform = Transform(Vector3(10, 0, 0), Quaternion::fromAxisAngle(Vector3::unitZ(), kHalfPi));
    RigidBody off(d);
    CHECK(off.getLocalCenterOfMass().nearlyEqual(Vector3(3, 0, 0), kTol));
    // COM em mundo = origem + R * (3,0,0) = (10,0,0) + (0,3,0)
    CHECK(off.getWorldCenterOfMass().nearlyEqual(Vector3(10, 3, 0), kTol));
    CHECK(off.getTransform().nearlyEqual(d.transform, kTol));
    // Rodar o frame em torno do COM não move o COM.
    off.setPose(off.getWorldCenterOfMass(), Quaternion::identity());
    CHECK(off.getWorldCenterOfMass().nearlyEqual(Vector3(10, 3, 0), kTol));
    CHECK(off.getTransform().position.nearlyEqual(Vector3(7, 3, 0), kTol));

    // setTransform round trip.
    Transform t(Vector3(-1, 5, 2), Quaternion::fromAxisAngle(Vector3(1, 1, 0), Real(0.4)));
    off.setTransform(t);
    CHECK(off.getTransform().nearlyEqual(t, kTol));

    // Entradas inválidas.
    RigidBodyDesc noShape;
    CHECK(throwsInvalidArgument([&] { RigidBody bad(noShape); }));
    RigidBodyDesc nan = boxDesc();
    nan.linearVelocity = Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0);
    CHECK(throwsInvalidArgument([&] { RigidBody bad(nan); }));
    RigidBodyDesc zeroDensity = boxDesc();
    zeroDensity.density = 0;
    CHECK(throwsInvalidArgument([&] { RigidBody bad(zeroDensity); }));

    // Inércia no mundo: I⁻¹ rotacionada, consistente com I (I_w * I_w⁻¹ = Id).
    RigidBodyDesc rd;
    rd.shape = std::make_shared<BoxShape>(Vector3(1, 2, 3));
    rd.transform = Transform(Vector3::zero(), Quaternion::fromAxisAngle(Vector3(1, 2, 3), Real(0.9)));
    RigidBody r(rd);
    const Matrix3 R = r.getOrientation().toMatrix();
    const Matrix3 Iw = Matrix3::rotateTensor(r.getMassData().inertia, R);
    CHECK((Iw * r.getInvInertiaWorld()).nearlyEqual(Matrix3::identity(), kTol));
}

static void testImpulsesAndForces() {
    RigidBodyDesc d = boxDesc();
    d.mass = 2;
    RigidBody b(d);

    b.applyLinearImpulse(Vector3(4, 0, 0));
    CHECK(b.getLinearVelocity().nearlyEqual(Vector3(2, 0, 0), kTol));

    // Impulso fora do centro gera rotação: Δω = I⁻¹ (r × J).
    RigidBody c(d);
    const Vector3 p(0, 1, 0), J(0, 0, 1);
    c.applyImpulseAtPoint(J, p);
    CHECK(c.getLinearVelocity().nearlyEqual(J * c.getInvMass(), kTol));
    CHECK(c.getAngularVelocity().nearlyEqual(c.getInvInertiaWorld() * p.cross(J), kTol));
    CHECK(c.getAngularVelocity().x > 0);  // (0,1,0)×(0,0,1) = (1,0,0)
    // v no ponto = v + ω × r
    CHECK(c.getVelocityAtPoint(p).nearlyEqual(c.getLinearVelocity() + c.getAngularVelocity().cross(p), kTol));

    // Estático/cinemático ignoram forças e impulsos.
    RigidBodyDesc sd = boxDesc();
    sd.type = BodyType::Static;
    RigidBody s(sd);
    s.applyLinearImpulse(Vector3(5, 5, 5));
    s.applyImpulseAtPoint(Vector3(0, 0, 5), Vector3(0, 1, 0));
    s.applyForce(Vector3(1, 1, 1));
    s.setLinearVelocity(Vector3(1, 0, 0));
    CHECK(s.getLinearVelocity().isZero(0) && s.getAngularVelocity().isZero(0) && s.getForce().isZero(0));
    CHECK(s.getInvMass() == 0 && s.getInvInertiaWorld().nearlyEqual(Matrix3::zero()));
}

static void testIntegrator() {
    const Vector3 g(0, -kGravity, 0);

    // Queda livre: solução discreta exata do Euler semi-implícito.
    //   v_n = -g n Δt ;  y_n = y0 - g Δt² n(n+1)/2
    RigidBodyDesc d = boxDesc(Vector3(0, 100, 0));
    d.angularDamping = 0;
    RigidBody b(d);
    const int n = 60;
    for (int i = 0; i < n; ++i) {
        Integrator::integrateVelocities(b, g, kDt);
        Integrator::integratePositions(b, kDt);
    }
    CHECK(b.getLinearVelocity().nearlyEqual(Vector3(0, -kGravity * n * kDt, 0), kTol));
    const Real yExpected = Real(100) - kGravity * kDt * kDt * Real(n) * Real(n + 1) / 2;
    CHECK(nearlyEqual(b.getWorldCenterOfMass().y, yExpected, kTol));
    CHECK(b.getTransform().position.nearlyEqual(b.getWorldCenterOfMass(), kTol));

    // Força constante F = m a.
    RigidBodyDesc fd = boxDesc();
    fd.mass = 4;
    RigidBody f(fd);
    f.applyForce(Vector3(8, 0, 0));
    Integrator::integrateVelocities(f, Vector3::zero(), Real(0.5));
    CHECK(f.getLinearVelocity().nearlyEqual(Vector3(1, 0, 0), kTol));
    CHECK(f.getForce().isZero(0));  // forças consumidas

    // Gravity scale zero → sem queda.
    RigidBodyDesc zg = boxDesc();
    zg.gravityScale = 0;
    RigidBody z(zg);
    Integrator::integrateVelocities(z, g, kDt);
    CHECK(z.getLinearVelocity().isZero(1e-6f));

    // Estático não se move; cinemático move-se com velocidade, sem gravidade.
    RigidBodyDesc sd = boxDesc(Vector3(1, 1, 1));
    sd.type = BodyType::Static;
    RigidBody st(sd);
    Integrator::integrateVelocities(st, g, kDt);
    Integrator::integratePositions(st, kDt);
    CHECK(st.getWorldCenterOfMass().nearlyEqual(Vector3(1, 1, 1)) && st.getLinearVelocity().isZero(0));

    RigidBodyDesc kd = boxDesc();
    kd.type = BodyType::Kinematic;
    kd.linearVelocity = Vector3(2, 0, 0);
    kd.angularVelocity = Vector3(0, 0, 1);
    RigidBody k(kd);
    for (int i = 0; i < 60; ++i) {
        Integrator::integrateVelocities(k, g, kDt);
        Integrator::integratePositions(k, kDt);
    }
    CHECK(k.getWorldCenterOfMass().nearlyEqual(Vector3(2, 0, 0), kTol));
    CHECK(k.getOrientation().nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1)), Real(2e-3)));

    // Amortecimento reduz velocidade monotonicamente.
    RigidBodyDesc dd = boxDesc();
    dd.linearDamping = 1;
    dd.linearVelocity = Vector3(10, 0, 0);
    RigidBody damp(dd);
    Real prev = damp.getLinearVelocity().length();
    bool monotone = true;
    for (int i = 0; i < 120; ++i) {
        Integrator::integrateVelocities(damp, Vector3::zero(), kDt);
        const Real now = damp.getLinearVelocity().length();
        monotone = monotone && now < prev;
        prev = now;
    }
    CHECK(monotone && prev < Real(2));

    // Limite de velocidade e proteção contra NaN.
    RigidBodyDesc fast = boxDesc();
    fast.linearVelocity = Vector3(1e6f, 0, 0);
    RigidBody fb(fast);
    Integrator::integrateVelocities(fb, Vector3::zero(), kDt);
    CHECK(nearlyEqual(fb.getLinearVelocity().length(), kMaxLinearSpeed, kTol));

    RigidBody nb(boxDesc());
    nb.setLinearVelocity(Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0));
    nb.setAngularVelocity(Vector3(0, std::numeric_limits<Real>::infinity(), 0));
    Integrator::integratePositions(nb, kDt);
    CHECK(nb.getWorldCenterOfMass().isFinite() && nb.getOrientation().isFinite());
    CHECK(nb.getLinearVelocity().isFinite() && nb.getAngularVelocity().isFinite());

    // Lote em std::vector.
    std::vector<RigidBody> bodies;
    bodies.emplace_back(boxDesc());
    bodies.emplace_back(boxDesc(Vector3(5, 0, 0)));
    Integrator::integrateVelocities(bodies, g, kDt);
    Integrator::integratePositions(bodies, kDt);
    CHECK(bodies[0].getWorldCenterOfMass().y < 0 && bodies[1].getWorldCenterOfMass().y < 0);
}

static void testRotationAndGyroscopic() {
    // Rotação sem torque de corpo simétrico (esfera): ω constante.
    RigidBodyDesc sd;
    sd.shape = std::make_shared<SphereShape>(Real(1));
    sd.angularVelocity = Vector3(0, 0, 2);
    sd.angularDamping = 0;
    RigidBody s(sd);
    for (int i = 0; i < 60; ++i) {
        Integrator::integrateVelocities(s, Vector3::zero(), kDt);
        Integrator::integratePositions(s, kDt);
    }
    CHECK(s.getAngularVelocity().nearlyEqual(Vector3(0, 0, 2), kTol));
    CHECK(s.getOrientation().nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitZ(), Real(2)), Real(2e-3)));
    CHECK(nearlyEqual(s.getOrientation().norm(), Real(1), kTol));

    // Caixa assimétrica girando livremente perto do eixo intermediário (rotação
    // instável → "tumbling"). Com o termo giroscópico o movimento é o de Euler
    // (precessão); o passo de Newton implícito é de 1ª ordem, então energia e |L|
    // têm um desvio O(Δt) que deve CAIR ao refinar o passo.
    RigidBodyDesc bd;
    bd.shape = std::make_shared<BoxShape>(Vector3(0.5f, 1.0f, 2.0f));
    bd.angularVelocity = Vector3(0.1f, 6.0f, 0.1f);
    bd.angularDamping = 0;

    auto run = [&](int hz, bool gyro, Real seconds, Real& maxEDev, Real& maxLDev, Real& bodyWChange) {
        RigidBody b(bd);
        IntegratorSettings cfg;
        cfg.gyroscopicTerm = gyro;
        const Real dt = Real(1) / Real(hz);
        const Real e0 = b.getKineticEnergy();
        const Real l0 = b.getAngularMomentum().length();
        const Vector3 wb0 = b.getOrientation().rotateInverse(b.getAngularVelocity());
        maxEDev = maxLDev = bodyWChange = 0;
        const int steps = static_cast<int>(seconds * Real(hz));
        for (int i = 0; i < steps; ++i) {
            Integrator::integrateVelocities(b, Vector3::zero(), dt, cfg);
            Integrator::integratePositions(b, dt, cfg);
            maxEDev = std::max(maxEDev, std::abs(b.getKineticEnergy() - e0) / e0);
            maxLDev = std::max(maxLDev, std::abs(b.getAngularMomentum().length() - l0) / l0);
            const Vector3 wb = b.getOrientation().rotateInverse(b.getAngularVelocity());
            bodyWChange = std::max(bodyWChange, (wb - wb0).length());
        }
        CHECK(nearlyEqual(b.getOrientation().norm(), Real(1), kTol));
    };

    Real e60, l60, c60, e240, l240, c240, eOff, lOff, cOff;
    run(60, true, 3, e60, l60, c60);
    run(240, true, 3, e240, l240, c240);
    run(60, false, 3, eOff, lOff, cOff);
    std::printf("  [info] giroscopico 3s: 60Hz dE=%.4f dL=%.4f | 240Hz dE=%.4f dL=%.4f\n",
                double(e60), double(l60), double(e240), double(l240));
    CHECK(e60 < Real(0.25) && l60 < Real(0.15));       // limitado mesmo a 60 Hz
    CHECK(e240 < e60 && l240 < l60);                   // converge ao refinar Δt
    CHECK(e240 < Real(0.06) && l240 < Real(0.03));
    // Precessão: com o termo, ω no referencial do corpo evolui (Euler);
    // sem ele, ω fica congelado — um estado estacionário fisicamente impossível.
    CHECK(c60 > Real(1) && c240 > Real(1));
    CHECK(cOff < Real(1e-3));
}

// V24: integração da orientação por exponencial de quatérnio x Euler.
static void testExponentialOrientation() {
    // ω constante: após N passos a exponencial dá EXATAMENTE a rotação de ângulo |ω| N Δt; Euler fica fora.
    const Vector3 axis = Vector3(1, 2, -0.5f).normalized();
    const Real wmag = Real(30);   // 30 rad/s a 60 Hz: 0,5 rad por passo
    const int n = 120;
    RigidBodyDesc d = boxDesc();
    d.angularDamping = 0;
    d.gravityScale = 0;
    auto run = [&](OrientationIntegration mode) {
        RigidBody b(d);
        b.setAngularVelocity(axis * wmag);
        IntegratorSettings is;
        is.orientation = mode;
        is.gyroscopicTerm = false;
        for (int i = 0; i < n; ++i) {
            Integrator::integrateVelocities(b, Vector3::zero(), kDt, is);
            Integrator::integratePositions(b, kDt, is);
        }
        return b.getOrientation();
    };
    const Quaternion exact = Quaternion::fromAxisAngle(axis, wmag * kDt * Real(n));
    const Quaternion qe = run(OrientationIntegration::Exponential);
    const Quaternion qu = run(OrientationIntegration::Euler);
    // distância angular (0 = iguais): 2 acos(|q1·q2|)
    auto dist = [](const Quaternion& a, const Quaternion& b) { return Real(2) * std::acos(std::fmin(Real(1), std::fabs(a.dot(b)))); };
    CHECK(dist(qe, exact) < Real(2e-3));       // float acumula arredondamento em 120 passos
    CHECK(dist(qu, exact) > Real(0.1));        // Euler com 0,5 rad/passo erra bastante
    CHECK(nearlyEqual(qe.norm(), Real(1), Real(1e-5)));
    CHECK(nearlyEqual(qu.norm(), Real(1), Real(1e-5)));

    // ω = 0 e ω minúsculo: sem NaN, orientação preservada.
    Quaternion q0 = Quaternion::fromAxisAngle(Vector3(0, 1, 0), Real(0.7));
    CHECK(q0.integratedExp(Vector3::zero(), kDt).nearlyEqual(q0, Real(1e-6)));
    const Quaternion tiny = q0.integratedExp(Vector3(1e-9f, 0, 0), kDt);
    CHECK(std::isfinite(tiny.w) && tiny.nearlyEqual(q0, Real(1e-5)));
    // para passo pequeno os dois modos concordam (primeira ordem)
    CHECK(dist(q0.integratedExp(Vector3(0, 0, 1), Real(1e-4)), q0.integrated(Vector3(0, 0, 1), Real(1e-4))) < Real(1e-6));
    // padrão é a exponencial
    CHECK(IntegratorSettings().orientation == OrientationIntegration::Exponential);
}

int main() {
    testMassData();
    testRigidBodyConstruction();
    testImpulsesAndForces();
    testIntegrator();
    testRotationAndGyroscopic();
    testExponentialOrientation();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
