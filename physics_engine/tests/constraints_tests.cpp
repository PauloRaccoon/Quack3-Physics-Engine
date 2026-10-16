// =============================================================================
// constraints_tests.cpp — testes unitários da Fase 6 (sem frameworks externos).
//
// Cada teste monta uma pequena "cena" com o pipeline que o PhysicsWorld da
// Fase 7 usará:   narrowphase → integrateVelocities → solver → integratePositions
// e compara o resultado com a física esperada (soluções analíticas quando existem).
// =============================================================================
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <vector>

#include "physics/collision/narrowphase/ContactManifold.h"
#include "physics/constraints/SequentialImpulseSolver.h"
#include "physics/dynamics/Integrator.h"
#include "physics/geometry/BoxShape.h"
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

static const Real kDt = Real(1) / Real(60);

// -----------------------------------------------------------------------------
// Cena de teste
// -----------------------------------------------------------------------------
struct Scene {
    std::vector<RigidBody> bodies;
    std::map<std::pair<int, int>, ContactManifold> manifolds;
    std::vector<std::unique_ptr<JointConstraint>> joints;
    SequentialImpulseSolver solver;
    Vector3 gravity = Vector3(0, -kGravity, 0);
    Real maxDepth = 0;   ///< maior penetração vista no último passo

    Scene() { bodies.reserve(512); }   // ponteiros estáveis para as juntas/solver

    int add(const RigidBodyDesc& d) {
        bodies.emplace_back(d);
        bodies.back().setId(static_cast<BodyId>(bodies.size() - 1));
        return static_cast<int>(bodies.size() - 1);
    }
    RigidBody& operator[](int i) { return bodies[static_cast<std::size_t>(i)]; }

    template <typename J, typename... Args>
    J& addJoint(Args&&... args) {
        joints.push_back(std::make_unique<J>(std::forward<Args>(args)...));
        return static_cast<J&>(*joints.back());
    }

    bool jointedNoCollide(const RigidBody* a, const RigidBody* b) const {
        for (const auto& j : joints) {
            if (j->getCollideConnected()) continue;
            if ((j->getBodyA() == a && j->getBodyB() == b) || (j->getBodyA() == b && j->getBodyB() == a)) return true;
        }
        return false;
    }

    void step(Real dt = kDt) {
        solver.clear();
        maxDepth = 0;
        const int n = static_cast<int>(bodies.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                RigidBody& a = (*this)[i];
                RigidBody& b = (*this)[j];
                if (!a.isDynamic() && !b.isDynamic()) continue;
                if (!a.getShape() || !b.getShape() || jointedNoCollide(&a, &b)) continue;
                // Pré-filtro com Fat AABB (margem como a da Broadphase): um par a poucos milímetros de
                // distância precisa chegar à narrowphase para virar contato especulativo.
                if (!a.getShape()->computeAABB(a.getTransform()).fattened(kAABBMargin).overlaps(b.getShape()->computeAABB(b.getTransform()))) {
                    auto it = manifolds.find({i, j});
                    if (it != manifolds.end()) it->second.clear();
                    continue;
                }
                auto it = manifolds.try_emplace({i, j}, ContactManifold(static_cast<BodyId>(i), static_cast<BodyId>(j))).first;
                ContactManifold& m = it->second;
                if (ContactGenerator::collide(*a.getShape(), a.getTransform(), *b.getShape(), b.getTransform(), m)) {
                    for (int k = 0; k < m.count; ++k) maxDepth = std::max(maxDepth, m.points[k].depth);
                    solver.addContact(m, a, b);
                }
            }
        }
        for (auto& j : joints) solver.addJoint(*j);

        Integrator::integrateVelocities(bodies, gravity, dt);
        solver.solve(dt);
        Integrator::integratePositions(bodies, dt);
    }

    void run(int steps) { for (int i = 0; i < steps; ++i) step(); }
};

static std::shared_ptr<const Shape> boxShape(Real hx, Real hy, Real hz) {
    return std::make_shared<BoxShape>(Vector3(hx, hy, hz));
}

static RigidBodyDesc staticBox(const Vector3& half, const Vector3& pos, const Quaternion& q = Quaternion::identity()) {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = boxShape(half.x, half.y, half.z);
    d.transform = Transform(pos, q);
    return d;
}

static RigidBodyDesc dynBox(const Vector3& half, const Vector3& pos, Real mass = 1) {
    RigidBodyDesc d;
    d.shape = boxShape(half.x, half.y, half.z);
    d.mass = mass;
    d.transform = Transform(pos);
    d.angularDamping = 0;
    return d;
}

static RigidBodyDesc dynSphere(Real r, const Vector3& pos, Real mass = 1) {
    RigidBodyDesc d;
    d.shape = std::make_shared<SphereShape>(r);
    d.mass = mass;
    d.transform = Transform(pos);
    d.angularDamping = 0;
    return d;
}

/// Chão estático com face superior em y = 0.
static RigidBodyDesc ground() { return staticBox(Vector3(50, 1, 50), Vector3(0, -1, 0)); }

// =============================================================================
// Restituição e conservação de momento
// =============================================================================
static void testRestitution() {
    // Esfera solta sobre o chão: o primeiro quique sai com  vy = e · |vn_impacto|  (exato:
    // o chão tem massa infinita e o bias de restituição domina a correção de posição).
    for (Real e : {Real(0.0), Real(0.3), Real(0.6), Real(0.9)}) {
        Scene s;
        s.add(ground());
        RigidBodyDesc d = dynSphere(Real(0.5), Vector3(0, 3, 0));
        d.restitution = e;
        const int b = s.add(d);

        bool hit = false;
        Real expected = 0, after = 0;
        for (int i = 0; i < 200 && !hit; ++i) {
            const Real vyBefore = s[b].getLinearVelocity().y;
            s.step();
            if (!s.solver.getContactCount()) continue;
            hit = true;
            const SolverSettings& cfg = s.solver.settings();
            const Real vImpact = std::abs(vyBefore - kGravity * kDt);
            // O alvo é o MAIOR entre restituição e correção de posição (Baumgarte c/ slop, limitado).
            const Real baumgarte = std::min(cfg.baumgarte / kDt * (s.maxDepth - cfg.linearSlop), cfg.maxBiasVelocity);
            expected = std::max(e * vImpact, baumgarte);
            after = s[b].getLinearVelocity().y;
        }
        CHECK(hit);
        CHECK(nearlyEqual(after, expected, Real(0.01) * expected + Real(1e-3)));
    }

    // Repouso: velocity slop impede "quiques" microscópicos de um corpo em repouso.
    Scene s;
    s.add(ground());
    RigidBodyDesc d = dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.5f, 0));
    d.restitution = Real(0.9);
    const int b = s.add(d);
    s.run(300);
    CHECK(std::abs(s[b].getLinearVelocity().y) < Real(0.02));
    CHECK(std::abs(s[b].getWorldCenterOfMass().y - Real(0.5)) < Real(0.02));
}

// Regressão: o quique não pode depender de o impacto cair dentro da margem especulativa.
// Esfera a 4 m/s (0,067 m por passo) sem gravidade; a altura inicial varre um passo inteiro
// de deslocamento, então alguns impactos começam com vão > 0 (contato especulativo) e outros
// já com penetração. Em todos a saída deve ser e·4.
static void testSpeculativeRestitution() {
    const Real e = Real(0.5);
    const Real speed = Real(4);
    int speculativeCases = 0, realCases = 0, bad = 0;
    const int kOffsets = 20;
    for (int k = 0; k < kOffsets; ++k) {
        Scene s;
        s.gravity = Vector3::zero();
        s.add(ground());
        RigidBodyDesc d = dynSphere(Real(0.5), Vector3(0, Real(1.5) + Real(k) * speed * kDt / Real(kOffsets), 0));
        d.restitution = e;
        d.linearVelocity = Vector3(0, -speed, 0);
        const int b = s.add(d);

        bool hit = false;
        for (int i = 0; i < 100 && !hit; ++i) {
            s.step();
            if (s.solver.getContactCount() == 0) continue;
            hit = true;
            // Profundidade real do ponto (maxDepth começa em 0 e esconderia valores negativos).
            const Real depth0 = s.manifolds.begin()->second.points[0].depth;
            (depth0 < Real(0) ? speculativeCases : realCases)++;
            if (std::abs(s[b].getLinearVelocity().y - e * speed) > Real(0.02)) ++bad;
        }
        if (!hit) ++bad;
    }
    std::printf("  [info] restituicao: %d impactos especulativos, %d com penetracao\n", speculativeCases, realCases);
    CHECK(bad == 0);
    CHECK(speculativeCases > 0 && realCases > 0);   // o teste realmente cobre os dois regimes
}

static void testMomentumConservation() {
    // Duas esferas, sem gravidade e sem atrito de contato (impacto central): o momento
    // linear se conserva exatamente e a velocidade relativa de afastamento é e·aproximação.
    for (Real e : {Real(0), Real(0.5), Real(1)}) {
        Scene s;
        s.gravity = Vector3::zero();
        RigidBodyDesc a = dynSphere(Real(0.5), Vector3(-1.0f, 0, 0), Real(1));
        RigidBodyDesc b = dynSphere(Real(0.5), Vector3(1.0f, 0, 0), Real(3));
        a.linearVelocity = Vector3(2, 0, 0);
        b.linearVelocity = Vector3(-2, 0, 0);
        a.restitution = b.restitution = e;
        const int ia = s.add(a), ib = s.add(b);
        const Vector3 p0 = s[ia].getLinearVelocity() * s[ia].getMass() + s[ib].getLinearVelocity() * s[ib].getMass();
        const Real approach = 4;
        s.run(120);
        const Vector3 p1 = s[ia].getLinearVelocity() * s[ia].getMass() + s[ib].getLinearVelocity() * s[ib].getMass();
        CHECK(p1.nearlyEqual(p0, Real(1e-3)));
        const Real sep = s[ib].getLinearVelocity().x - s[ia].getLinearVelocity().x;
        CHECK(std::abs(sep - e * approach) < Real(0.1) + Real(0.05) * e * approach);
        CHECK(s[ia].getWorldCenterOfMass().isFinite() && s[ib].getWorldCenterOfMass().isFinite());
    }
}

// =============================================================================
// Empilhamento, penetration slop, warm starting
// =============================================================================
static void testStacking() {
    Scene s;
    s.add(ground());
    const int N = 5;
    std::vector<int> ids;
    for (int i = 0; i < N; ++i) ids.push_back(s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, Real(0.5) + Real(i), 0))));

    Real worstDepth = 0;
    for (int i = 0; i < 600; ++i) {
        s.step();
        if (i > 120) worstDepth = std::max(worstDepth, s.maxDepth);
    }
    std::printf("  [info] pilha de %d caixas apos 10 s: penetracao max=%.4f m (slop=%.3f)\n", N, double(worstDepth), double(kLinearSlop));

    for (int i = 0; i < N; ++i) {
        const RigidBody& b = s[ids[static_cast<std::size_t>(i)]];
        CHECK(std::abs(b.getWorldCenterOfMass().y - (Real(0.5) + Real(i))) < Real(0.06));
        CHECK(std::abs(b.getWorldCenterOfMass().x) < Real(0.02) && std::abs(b.getWorldCenterOfMass().z) < Real(0.02));
        CHECK(b.getLinearVelocity().length() < Real(0.05) && b.getAngularVelocity().length() < Real(0.05));
        Vector3 axis; Real ang;
        b.getOrientation().toAxisAngle(axis, ang);
        CHECK(ang < Real(0.02));
    }
    CHECK(worstDepth < Real(0.03));   // penetração limitada a ~ alguns slops

    // Pilha desalinhada (offsets pequenos): continua em pé.
    Scene t;
    t.add(ground());
    std::vector<int> tid;
    const Real off[] = {Real(0.02), Real(-0.03), Real(0.04), Real(-0.02)};
    for (int i = 0; i < 4; ++i) tid.push_back(t.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(off[i], Real(0.5) + Real(i), -off[i]))));
    t.run(480);
    for (int i = 0; i < 4; ++i) {
        const RigidBody& b = t[tid[static_cast<std::size_t>(i)]];
        CHECK(b.getWorldCenterOfMass().y > Real(0.4) + Real(i) && b.getWorldCenterOfMass().y < Real(0.6) + Real(i));
        CHECK(b.getLinearVelocity().length() < Real(0.1));
    }
}

static Real stackSink(bool warm, int iterations, int steps) {
    Scene s;
    s.solver.settings().warmStarting = warm;
    s.solver.settings().velocityIterations = iterations;
    s.add(ground());
    const int N = 6;
    int top = 0;
    for (int i = 0; i < N; ++i) top = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, Real(0.5) + Real(i), 0)));
    s.run(steps);
    return (Real(0.5) + Real(N - 1)) - s[top].getWorldCenterOfMass().y;   // quanto a caixa do topo afundou
}

static void testWarmStarting() {
    const Real on = stackSink(true, 4, 90);
    const Real off = stackSink(false, 4, 90);
    std::printf("  [info] pilha de 6 caixas, 4 iteracoes, 1.5 s: afundamento do topo  warm=%.4f m, sem warm=%.4f m\n", double(on), double(off));
    CHECK(on < off);   // com warm starting o mesmo nº de iterações converge melhor

    // Os impulsos persistem no manifold entre os quadros.
    Scene s;
    s.add(ground());
    const int b = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.5f, 0), Real(2)));
    s.run(120);
    const ContactManifold& m = s.manifolds.begin()->second;
    CHECK(m.count == 4);
    // Em repouso a soma dos impulsos normais ≈ m g Δt (suporta o peso a cada passo).
    CHECK(std::abs(m.getTotalNormalImpulse() - s[b].getMass() * kGravity * kDt) < Real(0.05) * s[b].getMass() * kGravity * kDt);
    // Os ids dos pontos são estáveis entre quadros.
    std::uint32_t ids[4];
    for (int i = 0; i < 4; ++i) ids[i] = m.points[i].id;
    s.run(30);
    bool same = true;
    for (int i = 0; i < 4; ++i) same = same && (m.points[i].id == ids[i]);
    CHECK(same);
}

// =============================================================================
// Atrito
// =============================================================================
static Real incline(Real mu, Real angle, Real seconds, Real& slidSpeed) {
    // Plano inclinado: chão estático girado em torno de z. Caixa sobre a superfície, alinhada a ela.
    Scene s;
    const Quaternion q = Quaternion::fromAxisAngle(Vector3::unitZ(), angle);
    RigidBodyDesc g = staticBox(Vector3(50, 1, 50), q.rotate(Vector3(0, -1, 0)), q);
    g.friction = mu;
    s.add(g);
    RigidBodyDesc d = dynBox(Vector3(0.5f, 0.5f, 0.5f), q.rotate(Vector3(0, 0.5f, 0)));
    d.transform.rotation = q;
    d.friction = mu;
    const int b = s.add(d);
    s.run(static_cast<int>(seconds * Real(60)));
    const Vector3 downhill = q.rotate(Vector3(-1, 0, 0));
    slidSpeed = s[b].getLinearVelocity().dot(downhill);
    return s[b].getLinearVelocity().length();
}

static void testFriction() {
    const Real theta = Real(20) * kPi / Real(180);   // tan = 0.364
    Real v;
    // μ > tan θ: o atrito estático segura a caixa.
    const Real speedStick = incline(Real(0.8), theta, 2, v);
    CHECK(speedStick < Real(0.05));

    // μ < tan θ: desliza com  a = g (sin θ − μ cos θ).
    const Real mu = Real(0.2);
    incline(mu, theta, 1, v);
    const Real aExpected = kGravity * (std::sin(theta) - mu * std::cos(theta));
    std::printf("  [info] plano inclinado 20 graus, mu=0.2: v(1 s)=%.3f m/s, esperado %.3f\n", double(v), double(aExpected));
    CHECK(std::abs(v - aExpected) < Real(0.15) * aExpected);

    // Sem atrito: a = g sin θ.
    incline(Real(0), theta, 1, v);
    CHECK(std::abs(v - kGravity * std::sin(theta)) < Real(0.15) * kGravity * std::sin(theta));

    // Cone de Coulomb: o impulso tangencial acumulado nunca excede μ · λn.
    Scene s;
    const Real mu2 = Real(0.5);
    RigidBodyDesc g = ground();
    g.friction = mu2;
    s.add(g);
    RigidBodyDesc d = dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.5f, 0));
    d.friction = mu2;
    d.linearVelocity = Vector3(3, 0, 2);   // desliza em direção diagonal
    const int b = s.add(d);
    bool coneOk = true;
    Real worstCone = 0;
    Vector3 vAt03;
    for (int i = 0; i < 60; ++i) {
        s.step();
        if (i == 17) vAt03 = s[b].getLinearVelocity();   // t = 0.3 s: ainda deslizando
        const ContactManifold& m = s.manifolds.begin()->second;
        for (int k = 0; k < m.count; ++k) {
            const Real lim = mu2 * m.points[k].normalImpulse;
            if (lim > Real(1e-4)) worstCone = std::max(worstCone, m.points[k].tangentImpulse.length() / lim);
        }
    }
    // O cone é respeitado (com folga mínima: o impulso normal pode ceder um pouco depois do atrito
    // na última iteração de Gauss–Seidel).
    coneOk = worstCone < Real(1.02);
    CHECK(coneOk);
    // A direção do atrito é oposta ao deslizamento: a velocidade horizontal cai mantendo a direção (3:2)
    // e o módulo cai a μ g = 4.9 m/s².
    CHECK(vAt03.x > 0 && vAt03.z > 0 && nearlyEqual(vAt03.x / vAt03.z, Real(1.5), Real(0.03)));
    const Real speedExpected = std::sqrt(Real(13)) - mu2 * kGravity * Real(0.3);
    CHECK(std::abs(std::sqrt(vAt03.x * vAt03.x + vAt03.z * vAt03.z) - speedExpected) < Real(0.1));
    // Depois de parar, o atrito estático segura a caixa.
    CHECK(s[b].getLinearVelocity().length() < Real(0.02));
}

static void testKinematicPlatform() {
    // Plataforma cinemática movendo-se a 1 m/s: a caixa em cima é arrastada pelo atrito.
    Scene s;
    RigidBodyDesc plat = staticBox(Vector3(20, 0.25f, 4), Vector3(0, -0.25f, 0));
    plat.type = BodyType::Kinematic;
    plat.linearVelocity = Vector3(1, 0, 0);
    plat.friction = Real(0.8);
    const int p = s.add(plat);
    RigidBodyDesc d = dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.5f, 0));
    d.friction = Real(0.8);
    const int b = s.add(d);
    s.run(150);
    CHECK(std::abs(s[b].getLinearVelocity().x - Real(1)) < Real(0.05));
    CHECK(std::abs(s[b].getWorldCenterOfMass().y - Real(0.5)) < Real(0.03));
    CHECK(s[p].getLinearVelocity().nearlyEqual(Vector3(1, 0, 0), Real(1e-6)));   // o solver não altera o cinemático
}

// =============================================================================
// Juntas
// =============================================================================
static void testDistanceJointPendulum() {
    // Pêndulo simples: pivô estático, esfera presa por haste rígida de comprimento L.
    const Real L = 2, theta0 = Real(0.1);
    Scene s;
    RigidBodyDesc pivot = staticBox(Vector3(0.1f, 0.1f, 0.1f), Vector3(0, 5, 0));
    const int pv = s.add(pivot);
    const Vector3 bobPos(L * std::sin(theta0), 5 - L * std::cos(theta0), 0);
    RigidBodyDesc bob = dynSphere(Real(0.2), bobPos, Real(1));
    const int bb = s.add(bob);
    DistanceJoint& j = s.addJoint<DistanceJoint>(&s[pv], &s[bb], Vector3(0, 5, 0), bobPos);
    CHECK(nearlyEqual(j.getLength(), L, Real(1e-4)));

    // Pequenas oscilações: T = 2π √(L/g). Mede o 1º cruzamento por x = 0 (quarto de período).
    Real tCross = -1, maxLenErr = 0;
    for (int i = 0; i < 600; ++i) {
        const Real xPrev = s[bb].getWorldCenterOfMass().x;
        s.step();
        const Real x = s[bb].getWorldCenterOfMass().x;
        maxLenErr = std::max(maxLenErr, std::abs((s[bb].getWorldCenterOfMass() - Vector3(0, 5, 0)).length() - L));
        // Cruzamento de x por 0 neste passo: interpolação linear dentro do passo.
        if (tCross < 0 && xPrev > 0 && x <= 0) tCross = (Real(i) + xPrev / (xPrev - x)) * kDt;
    }
    const Real T = Real(2) * kPi * std::sqrt(L / kGravity);
    std::printf("  [info] pendulo L=%.1f: quarto de periodo=%.3f s (analitico %.3f), erro max de comprimento=%.4f m\n",
                double(L), double(tCross), double(T / 4), double(maxLenErr));
    CHECK(tCross > 0 && std::abs(tCross - T / 4) < Real(0.05) * (T / 4));
    CHECK(maxLenErr < Real(0.01));
}

static Real totalEnergy(const RigidBody& b) {
    return b.getKineticEnergy() + b.getMass() * kGravity * b.getWorldCenterOfMass().y;
}

static void testBallSocketPendulum() {
    // Caixa pendurada por uma ponta: o ponto de ancoragem fica no pivô; a energia se conserva.
    Scene s;
    const int pv = s.add(staticBox(Vector3(0.1f, 0.1f, 0.1f), Vector3(0, 5, 0)));
    const int bb = s.add(dynBox(Vector3(0.5f, 0.1f, 0.1f), Vector3(1.5f, 5, 0), Real(2)));   // ancoragem na ponta esquerda
    BallSocketJoint& j = s.addJoint<BallSocketJoint>(&s[pv], &s[bb], Vector3(0.5f, 5, 0));
    const Real e0 = totalEnergy(s[bb]);
    Real maxErr = 0, maxDrift = 0, minY = 99;
    for (int i = 0; i < 360; ++i) {
        s.step();
        maxErr = std::max(maxErr, j.getAnchorA().distance(j.getAnchorB()));
        maxDrift = std::max(maxDrift, std::abs(totalEnergy(s[bb]) - e0) / std::abs(e0));
        minY = std::min(minY, s[bb].getWorldCenterOfMass().y);
    }
    std::printf("  [info] caixa em esfera-soquete 6 s: erro de ancoragem max=%.4f m, deriva de energia max=%.3f\n", double(maxErr), double(maxDrift));
    CHECK(maxErr < Real(0.02));
    CHECK(maxDrift < Real(0.08));
    CHECK(minY < Real(4.2));   // realmente balançou para baixo
}

static void testHingeJoint() {
    // (a) Sem gravidade: velocidade angular fora do eixo é eliminada; só resta a rotação em torno do eixo.
    {
        Scene s;
        s.gravity = Vector3::zero();
        const int f = s.add(staticBox(Vector3(0.1f, 0.1f, 0.1f), Vector3(0, 0, 0)));
        RigidBodyDesc d = dynBox(Vector3(0.5f, 1, 0.05f), Vector3(0.5f, 0, 0), Real(1));
        d.angularVelocity = Vector3(1.5f, 2, 0.7f);   // eixo da dobradiça = y
        const int door = s.add(d);
        HingeJoint& h = s.addJoint<HingeJoint>(&s[f], &s[door], Vector3::zero(), Vector3::unitY());
        s.run(90);
        const Vector3 w = s[door].getAngularVelocity();
        CHECK(std::abs(w.x) < Real(0.05) && std::abs(w.z) < Real(0.05));
        // A rotação em torno do eixo é livre. Sem torque externo em y, o momento angular em torno do
        // PIVÔ se conserva:  Iyy·ω0 = (Iyy + m d²)·ω   (d = 0.5, distância do centro de massa ao eixo).
        const Real iyy = s[door].getMassData().inertia(1, 1);
        const Real expectedWy = iyy * Real(2) / (iyy + s[door].getMass() * Real(0.25));
        CHECK(std::abs(w.y - expectedWy) < Real(0.03) * expectedWy);
        CHECK(h.getAnchorA().distance(h.getAnchorB()) < Real(0.02));
        CHECK(h.getAxisA().distance(h.getAxisB()) < Real(0.02));
    }
    // (b) Com gravidade: porta horizontal em dobradiça horizontal balança como pêndulo, sem sair do plano.
    {
        Scene s;
        const int f = s.add(staticBox(Vector3(0.1f, 0.1f, 0.1f), Vector3(0, 5, 0)));
        RigidBodyDesc d = dynBox(Vector3(1, 0.05f, 0.5f), Vector3(1, 5, 0), Real(1));
        d.angularVelocity = Vector3(0.3f, 0.2f, 0);   // perturbação fora do eixo z
        const int door = s.add(d);
        HingeJoint& h = s.addJoint<HingeJoint>(&s[f], &s[door], Vector3(0, 5, 0), Vector3::unitZ());
        Real maxAxis = 0, maxAnchor = 0, maxZ = 0, minY = 99;
        for (int i = 0; i < 360; ++i) {
            s.step();
            maxAxis = std::max(maxAxis, h.getAxisA().distance(h.getAxisB()));
            maxAnchor = std::max(maxAnchor, h.getAnchorA().distance(h.getAnchorB()));
            maxZ = std::max(maxZ, std::abs(s[door].getWorldCenterOfMass().z));
            minY = std::min(minY, s[door].getWorldCenterOfMass().y);
        }
        std::printf("  [info] porta em dobradica 6 s: desalinhamento max=%.4f, erro de ancoragem max=%.4f, desvio em z=%.4f\n",
                    double(maxAxis), double(maxAnchor), double(maxZ));
        CHECK(maxAxis < Real(0.03) && maxAnchor < Real(0.02) && maxZ < Real(0.03));
        CHECK(minY < Real(4.2));
    }
}

static void testChain() {
    // Corrente de 6 elos (esfera-soquete) pendurada de um ponto fixo, com razão de massas 1:20.
    Scene s;
    int prev = s.add(staticBox(Vector3(0.1f, 0.1f, 0.1f), Vector3(0, 10, 0)));
    std::vector<BallSocketJoint*> js;
    for (int i = 0; i < 6; ++i) {
        const Real mass = (i == 5) ? Real(20) : Real(1);
        const int link = s.add(dynBox(Vector3(0.5f, 0.1f, 0.1f), Vector3(Real(0.5) + Real(i), 10, 0), mass));
        js.push_back(&s.addJoint<BallSocketJoint>(&s[prev], &s[link], Vector3(Real(i), 10, 0)));
        prev = link;
    }
    Real maxErr = 0;
    for (int i = 0; i < 300; ++i) {
        s.step();
        for (const auto* j : js) maxErr = std::max(maxErr, j->getAnchorA().distance(j->getAnchorB()));
    }
    std::printf("  [info] corrente de 6 elos (massas 1:20): erro de ancoragem max=%.4f m\n", double(maxErr));
    CHECK(maxErr < Real(0.15));
    for (const RigidBody& b : s.bodies) CHECK(b.getWorldCenterOfMass().isFinite() && b.getLinearVelocity().isFinite());
}

// =============================================================================
// Robustez
// =============================================================================
static void testRobustness() {
    // dt inválido, nenhuma restrição, manifold vazio, dois corpos estáticos: nada acontece / nada quebra.
    {
        Scene s;
        s.add(ground());
        const int b = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.5f, 0)));
        const Vector3 v0 = s[b].getLinearVelocity();
        SequentialImpulseSolver& sv = s.solver;
        sv.solve(0);
        sv.solve(-1);
        sv.solve(std::numeric_limits<Real>::quiet_NaN());
        sv.solve(kDt);   // sem restrições
        CHECK(s[b].getLinearVelocity() == v0);

        ContactManifold empty(0, 1);
        CHECK(!sv.addContact(empty, s[0], s[b]));
        ContactManifold m(0, 1);
        m.count = 1;
        m.points[0].depth = Real(0.01);
        RigidBodyDesc sd = staticBox(Vector3(1, 1, 1), Vector3(5, 0, 0));
        s.add(sd);
        CHECK(!sv.addContact(m, s[0], s[2]));   // estático × estático
        CHECK(!sv.addContact(m, s[b], s[b]));   // mesmo corpo
    }
    // Impulsos acumulados corrompidos (NaN/negativos) no manifold não contaminam as velocidades.
    {
        Scene s;
        s.add(ground());
        const int b = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.5f, 0)));
        s.run(5);
        ContactManifold& m = s.manifolds.begin()->second;
        for (int k = 0; k < m.count; ++k) {
            m.points[k].normalImpulse = std::numeric_limits<Real>::quiet_NaN();
            m.points[k].tangentImpulse = Vector3(std::numeric_limits<Real>::infinity(), 0, 0);
        }
        s.step();
        CHECK(s[b].getLinearVelocity().isFinite() && s[b].getAngularVelocity().isFinite());
        m.points[0].normalImpulse = -50;
        s.step();
        CHECK(s[b].getLinearVelocity().isFinite() && std::abs(s[b].getLinearVelocity().y) < Real(1));
    }
    // Construção inválida de juntas.
    {
        Scene s;
        const int a = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0, 0)));
        const int b = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(2, 0, 0)));
        bool t1 = false, t2 = false, t3 = false, t4 = false;
        try { BallSocketJoint j(nullptr, &s[b], Vector3::zero()); } catch (const std::invalid_argument&) { t1 = true; }
        try { BallSocketJoint j(&s[a], &s[a], Vector3::zero()); } catch (const std::invalid_argument&) { t2 = true; }
        try { HingeJoint j(&s[a], &s[b], Vector3::zero(), Vector3::zero()); } catch (const std::invalid_argument&) { t3 = true; }
        try { DistanceJoint j(&s[a], &s[b], Vector3::zero(), Vector3::unitX(), std::numeric_limits<Real>::infinity()); } catch (const std::invalid_argument&) { t4 = true; }
        CHECK(t1 && t2 && t3 && t4);
    }
    // Junta entre dois corpos de massa infinita (estático+cinemático): sem efeito, sem NaN.
    {
        Scene s;
        const int a = s.add(staticBox(Vector3(1, 1, 1), Vector3(0, 0, 0)));
        const int b = s.add(staticBox(Vector3(1, 1, 1), Vector3(3, 0, 0)));
        s.addJoint<BallSocketJoint>(&s[a], &s[b], Vector3(1.5f, 0, 0));
        s.run(10);
        CHECK(s[b].getWorldCenterOfMass().nearlyEqual(Vector3(3, 0, 0)));
    }
    // Determinismo: duas execuções idênticas dão resultados idênticos bit a bit.
    {
        auto run = [] {
            Scene s;
            s.add(ground());
            for (int i = 0; i < 4; ++i) s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(Real(0.03) * Real(i), Real(0.5) + Real(i), 0)));
            s.run(200);
            return s[4].getWorldCenterOfMass();
        };
        CHECK(run() == run());
    }
    // Penetração profunda inicial: a correção é limitada (maxBiasVelocity) e a caixa emerge sem explodir.
    {
        Scene s;
        s.add(ground());
        const int b = s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0.1f, 0)));   // 0.4 m enterrada
        Real maxSpeed = 0;
        for (int i = 0; i < 120; ++i) { s.step(); maxSpeed = std::max(maxSpeed, s[b].getLinearVelocity().length()); }
        CHECK(maxSpeed < Real(6));
        CHECK(std::abs(s[b].getWorldCenterOfMass().y - Real(0.5)) < Real(0.03));
    }
}

// =============================================================================
static void testPyramidInfo() {
    // Pirâmide de caixas: teste de estresse do solver (empilhamento com vários contatos laterais).
    Scene s;
    s.add(ground());
    const int layers = 6;
    std::vector<std::pair<int, Vector3>> boxes;
    for (int k = 0; k < layers; ++k) {
        const int n = layers - k;
        for (int j = 0; j < n; ++j) {
            const Vector3 p((Real(j) - Real(n - 1) / 2) * Real(1.05), Real(0.5) + Real(k), 0);
            boxes.push_back({s.add(dynBox(Vector3(0.5f, 0.5f, 0.5f), p)), p});
        }
    }
    auto t0 = std::chrono::steady_clock::now();
    const int steps = 300;
    s.run(steps);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    Real maxMove = 0, maxSpeed = 0;
    for (const auto& b : boxes) {
        maxMove = std::max(maxMove, s[b.first].getWorldCenterOfMass().distance(b.second));
        maxSpeed = std::max(maxSpeed, s[b.first].getLinearVelocity().length());
    }
    std::printf("  [info] piramide de %zu caixas, 5 s: deslocamento max=%.3f m, vel max=%.3f m/s, %.3f ms/passo (inclui O(N^2) de pares do teste)\n",
                boxes.size(), double(maxMove), double(maxSpeed), ms / steps);
    CHECK(maxMove < Real(0.15));
    CHECK(maxSpeed < Real(0.1));
}

int main() {
    testRestitution();
    testSpeculativeRestitution();
    testMomentumConservation();
    testStacking();
    testWarmStarting();
    testFriction();
    testKinematicPlatform();
    testDistanceJointPendulum();
    testBallSocketPendulum();
    testHingeJoint();
    testChain();
    testRobustness();
    testPyramidInfo();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
