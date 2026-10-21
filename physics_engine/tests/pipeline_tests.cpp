// =============================================================================
// pipeline_tests.cpp — testes unitários da Fase 7 (sem frameworks externos).
// Cobre: FixedTimestepAccumulator, MotionState, BufferedTransformBridge,
// CollisionWorld e PhysicsWorld (ciclo de vida, juntas, simulações completas,
// determinismo, independência da taxa de quadros e interpolação de render).
// =============================================================================
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "physics/bridge/IPhysicsTransformBridge.h"
#include "physics/bridge/MotionState.h"
#include "physics/collision/CollisionWorld.h"
#include "physics/geometry/BoxShape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/pipeline/PhysicsWorld.h"

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

static const Real kDt = kFixedTimeStep;
static const Real kTol = Real(1e-3);

template <typename E, typename F>
static bool throws(F&& f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

// Gerador pseudo-aleatório determinístico (LCG) em [0,1).
struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
};

// -----------------------------------------------------------------------------
// Fábricas de descrições
// -----------------------------------------------------------------------------
static RigidBodyDesc groundDesc() {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(Vector3(50, 1, 50));
    d.transform = Transform(Vector3(0, -1, 0));   // face superior em y = 0
    return d;
}

static RigidBodyDesc boxDesc(const Vector3& pos, Real half = Real(0.5), Real mass = Real(1)) {
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(half, half, half));
    d.mass = mass;
    d.transform = Transform(pos);
    d.angularDamping = 0;
    return d;
}

static RigidBodyDesc sphereDesc(const Vector3& pos, Real r = Real(0.5), Real mass = Real(1)) {
    RigidBodyDesc d;
    d.shape = std::make_shared<SphereShape>(r);
    d.mass = mass;
    d.transform = Transform(pos);
    d.angularDamping = 0;
    return d;
}

/// Compara o estado completo de todos os corpos de dois mundos (igualdade EXATA).
static bool sameState(PhysicsWorld& a, PhysicsWorld& b, int bodyCount) {
    for (int i = 0; i < bodyCount; ++i) {
        const BodyId id = static_cast<BodyId>(i);
        const RigidBody& x = a.getBody(id);
        const RigidBody& y = b.getBody(id);
        if (!(x.getWorldCenterOfMass() == y.getWorldCenterOfMass())) return false;
        if (!(x.getOrientation() == y.getOrientation())) return false;
        if (!(x.getLinearVelocity() == y.getLinearVelocity())) return false;
        if (!(x.getAngularVelocity() == y.getAngularVelocity())) return false;
    }
    return true;
}

/// Cena de referência para determinismo e independência de FPS: chão, pilha de
/// 5 caixas ligeiramente desalinhadas e uma esfera lançada contra ela. Retorna nº de corpos.
static int buildScene(PhysicsWorld& w) {
    w.createBody(groundDesc());
    for (int i = 0; i < 5; ++i) {
        w.createBody(boxDesc(Vector3(Real(0.03) * Real(i % 2), Real(0.5) + Real(i), 0)));
    }
    RigidBodyDesc ball = sphereDesc(Vector3(-6, 2.0f, 0), Real(0.5), Real(4));
    ball.linearVelocity = Vector3(10, 0, 0);
    w.createBody(ball);
    return 7;
}

// =============================================================================
// FixedTimestepAccumulator
// =============================================================================
static void testAccumulator() {
    // Render a 144 Hz durante 1 s: ~60 passos fixos no total.
    {
        FixedTimestepAccumulator acc(kDt, 8);
        int total = 0;
        bool alphaOk = true;
        for (int i = 0; i < 144; ++i) {
            total += acc.advance(Real(1) / Real(144));
            alphaOk = alphaOk && acc.getAlpha() >= Real(0) && acc.getAlpha() < Real(1);
        }
        CHECK(total >= 59 && total <= 60);
        CHECK(alphaOk);
    }
    // Render a exatamente 60 Hz: 1 passo por quadro; a 30 Hz: 2 passos por quadro.
    {
        FixedTimestepAccumulator acc(kDt, 8);
        bool one = true;
        for (int i = 0; i < 60; ++i) one = one && acc.advance(kDt) == 1;
        CHECK(one);
        FixedTimestepAccumulator slow(kDt, 8);
        bool two = true;
        for (int i = 0; i < 30; ++i) two = two && slow.advance(Real(2) * kDt) == 2;
        CHECK(two);
    }
    // α cresce dentro do passo.
    {
        FixedTimestepAccumulator acc(kDt, 8);
        CHECK(acc.advance(kDt * Real(0.25)) == 0 && nearlyEqual(acc.getAlpha(), Real(0.25), kTol));
        CHECK(acc.advance(kDt * Real(0.25)) == 0 && nearlyEqual(acc.getAlpha(), Real(0.5), kTol));
        CHECK(acc.advance(kDt * Real(0.75)) == 1 && nearlyEqual(acc.getAlpha(), Real(0.25), kTol));
    }
    // Espiral da morte: um quadro enorme é limitado a maxSubSteps e o resto é descartado.
    {
        FixedTimestepAccumulator acc(kDt, 8);
        CHECK(acc.advance(Real(5)) == 8);
        CHECK(acc.getDroppedTime() > Real(4.8));
        CHECK(acc.getAlpha() >= Real(0) && acc.getAlpha() < Real(1));
        CHECK(acc.advance(Real(1000)) == 8);   // continua limitado nos quadros seguintes
    }
    // Entradas inválidas são ignoradas e não alteram o estado.
    {
        FixedTimestepAccumulator acc(kDt, 8);
        acc.advance(kDt * Real(0.4));
        const Real before = acc.getAccumulator();
        CHECK(acc.advance(Real(-1)) == 0);
        CHECK(acc.advance(std::numeric_limits<Real>::quiet_NaN()) == 0);
        CHECK(acc.advance(std::numeric_limits<Real>::infinity()) == 0);
        CHECK(acc.advance(0) == 0);
        CHECK(acc.getAccumulator() == before);
    }
    CHECK(throws<std::invalid_argument>([] { FixedTimestepAccumulator a(0, 8); }));
    CHECK(throws<std::invalid_argument>([] { FixedTimestepAccumulator a(-1, 8); }));
    CHECK(throws<std::invalid_argument>([] { FixedTimestepAccumulator a(std::numeric_limits<Real>::quiet_NaN(), 8); }));
    CHECK(throws<std::invalid_argument>([] { FixedTimestepAccumulator a(kDt, 0); }));
}

// =============================================================================
// MotionState
// =============================================================================
static void testMotionState() {
    const Transform a(Vector3(0, 0, 0), Quaternion::identity());
    const Transform b(Vector3(2, 0, 4), Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1)));

    MotionState m(a);
    m.push(b);
    CHECK(m.getPrevious().nearlyEqual(a) && m.getCurrent().nearlyEqual(b));
    CHECK(m.interpolate(0).nearlyEqual(a, kTol));
    CHECK(m.interpolate(1).nearlyEqual(b, kTol));
    const Transform mid = m.interpolate(Real(0.5));
    CHECK(mid.position.nearlyEqual(Vector3(1, 0, 2), kTol));                               // lerp
    CHECK(mid.rotation.nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitZ(), Real(0.5)), kTol));   // slerp
    CHECK(m.interpolate(Real(-3)).nearlyEqual(a, kTol));   // α saturado em [0,1]
    CHECK(m.interpolate(Real(7)).nearlyEqual(b, kTol));

    // push desloca: a pose "atual" vira a "anterior".
    const Transform c(Vector3(5, 5, 5));
    m.push(c);
    CHECK(m.getPrevious().nearlyEqual(b) && m.getCurrent().nearlyEqual(c));

    // reset: sem rastro de interpolação.
    m.reset(a);
    CHECK(m.interpolate(Real(0.3)).nearlyEqual(a, kTol) && m.getPrevious().nearlyEqual(m.getCurrent()));

    // O slerp segue o caminho mais curto e mantém o quatérnio unitário.
    MotionState wrap(Transform(Vector3::zero(), Quaternion::fromAxisAngle(Vector3::unitY(), Real(3.0))));
    wrap.push(Transform(Vector3::zero(), Quaternion::fromAxisAngle(Vector3::unitY(), Real(-3.0))));
    const Transform w = wrap.interpolate(Real(0.5));
    CHECK(nearlyEqual(w.rotation.norm(), Real(1), kTol));
    // Ângulo de 3 rad → -3 rad: o caminho curto passa por π (6,28-6 = 0,28 rad), não por 0.
    CHECK(std::abs(w.rotation.w) < Real(0.2));
}

// =============================================================================
// BufferedTransformBridge
// =============================================================================
static void testBufferedBridge() {
    BufferedTransformBridge br;
    CHECK(br.getSlotCount() == 0 && br.getBodyData(0) == nullptr && !br.isAlive(0));

    const Transform t(Vector3(1, 2, 3), Quaternion::fromAxisAngle(Vector3(0, 1, 0), Real(0.7)));
    br.onBodyCreated(3, t);
    CHECK(br.getSlotCount() == 4 && br.isAlive(3) && !br.isAlive(1));
    CHECK(br.getTransform(3).nearlyEqual(t, Real(1e-5)));
    // Slots intermediários começam com orientação identidade (w = 1), não zeros.
    const float* s1 = br.getBodyData(1);
    CHECK(s1 != nullptr && s1[6] == 1.0f && s1[0] == 0.0f);
    // Layout plano: px py pz qx qy qz qw.
    const float* s3 = br.getBodyData(3);
    CHECK(s3 == br.data() + 3 * BufferedTransformBridge::kFloatsPerBody);
    CHECK(nearlyEqual(Real(s3[0]), Real(1), Real(1e-6)) && nearlyEqual(Real(s3[2]), Real(3), Real(1e-6)));
    CHECK(nearlyEqual(Real(s3[6]), t.rotation.w, Real(1e-6)) && nearlyEqual(Real(s3[4]), t.rotation.y, Real(1e-6)));

    br.beginSync(Real(0.25));
    br.onTransformUpdated(3, Transform(Vector3(9, 9, 9)));
    CHECK(nearlyEqual(br.getLastAlpha(), Real(0.25)) && br.getSyncCount() == 1);
    CHECK(br.getTransform(3).position.nearlyEqual(Vector3(9, 9, 9), Real(1e-5)));

    br.onBodyDestroyed(3);
    CHECK(!br.isAlive(3));
    br.onBodyDestroyed(999);   // id desconhecido: ignora
    CHECK(br.getBodyData(999) == nullptr);
}

// =============================================================================
// CollisionWorld
// =============================================================================
static void testCollisionWorld() {
    auto ground = std::make_shared<BoxShape>(Vector3(50, 1, 50));
    auto box = std::make_shared<BoxShape>(Vector3(0.5f, 0.5f, 0.5f));

    CollisionWorld cw;
    cw.addObject(0, ground, Transform(Vector3(0, -1, 0)), false);
    cw.addObject(1, box, Transform(Vector3(0, Real(0.49), 0)), true);
    cw.detect();

    CHECK(cw.getTouching().size() == 1);
    const ContactPair* p = cw.findPair(1, 0);   // qualquer ordem
    CHECK(p != nullptr && p == cw.findPair(0, 1) && p->a == 0 && p->b == 1);
    CHECK(p->manifold.normal.y > Real(0.99));    // de A (chão) para B (caixa): para cima
    CHECK(p->manifold.count == 4);                // base da caixa inteira (clipping de faces)
    CHECK(nearlyEqual(p->manifold.points[0].depth, Real(0.01), Real(2e-3)));

    // Persistência: impulsos acumulados sobrevivem a novos detect().
    ContactPair* mp = cw.getTouching()[0];
    mp->manifold.points[0].normalImpulse = Real(3);
    const std::uint32_t id0 = mp->manifold.points[0].id;
    cw.detect();
    CHECK(cw.getTouching().size() == 1);
    CHECK(cw.getTouching()[0]->manifold.points[0].id == id0);
    CHECK(nearlyEqual(cw.getTouching()[0]->manifold.getTotalNormalImpulse(), Real(3)));

    // Dois objetos imóveis nunca formam par.
    cw.addObject(2, box, Transform(Vector3(0, Real(0.2), 0)), false);
    cw.detect();
    CHECK(cw.findPair(0, 2) == nullptr);

    // Consulta de região (Fat AABB, conservadora).
    int found = 0;
    cw.queryAABB(AABB(Vector3(-0.1f, 0.3f, -0.1f), Vector3(0.1f, 0.4f, 0.1f)), [&](BodyId) { ++found; return true; });
    CHECK(found >= 2);

    // Afastar o objeto desfaz o par.
    cw.updateObject(1, Transform(Vector3(0, 10, 0)));
    cw.detect();
    CHECK(cw.findPair(0, 1) == nullptr && cw.getTouching().empty());

    // Voltar a encostar recria o par com manifold novo.
    cw.updateObject(1, Transform(Vector3(0, Real(0.49), 0)));
    cw.detect();
    // (o objeto estático 2 também encosta no 1: dois pares em contato)
    CHECK(cw.findPair(0, 1) != nullptr && cw.findPair(1, 2) != nullptr && cw.getTouching().size() == 2);

    // Remover objeto: pares somem e a lista de contatos não guarda ponteiros soltos.
    cw.removeObject(1);
    CHECK(!cw.hasObject(1) && cw.findPair(0, 1) == nullptr && cw.getTouching().empty());
    cw.removeObject(1);       // duplo remove é inofensivo
    cw.removeObject(12345);

    // Filtro de pares.
    CollisionWorld filtered;
    filtered.setPairFilter([](BodyId a, BodyId b) { return !(a == 0 && b == 1); });
    filtered.addObject(0, ground, Transform(Vector3(0, -1, 0)), false);
    filtered.addObject(1, box, Transform(Vector3(0, Real(0.49), 0)), true);
    filtered.detect();
    CHECK(filtered.getPairCount() == 0 && filtered.getTouching().empty());

    // removePair no meio do quadro limpa também a lista de contatos.
    CollisionWorld cw2;
    cw2.addObject(0, ground, Transform(Vector3(0, -1, 0)), false);
    cw2.addObject(1, box, Transform(Vector3(0, Real(0.49), 0)), true);
    cw2.detect();
    CHECK(cw2.getTouching().size() == 1);
    cw2.removePair(1, 0);
    CHECK(cw2.getTouching().empty() && cw2.getPairCount() == 0);

    // Entradas inválidas.
    CollisionWorld bad;
    CHECK(throws<std::invalid_argument>([&] { bad.addObject(0, nullptr, Transform(), true); }));
    bad.addObject(0, box, Transform(), true);
    CHECK(throws<std::invalid_argument>([&] { bad.addObject(0, box, Transform(), true); }));   // id repetido
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    CHECK(throws<std::invalid_argument>([&] { bad.addObject(1, box, Transform(Vector3(nan, 0, 0)), true); }));
    CHECK(!bad.hasObject(1));
    bad.updateObject(0, Transform(Vector3(nan, 0, 0)));   // pose não finita é ignorada
    bad.detect();
}

// =============================================================================
// PhysicsWorld: ciclo de vida
// =============================================================================
static void testWorldLifecycle() {
    PhysicsWorldSettings s;
    s.maxBodies = 4;
    PhysicsWorld w(s);

    const BodyId g = w.createBody(groundDesc());
    const BodyId b1 = w.createBody(boxDesc(Vector3(0, 0.5f, 0)));
    const BodyId b2 = w.createBody(boxDesc(Vector3(5, 0.5f, 0)));
    const BodyId b3 = w.createBody(sphereDesc(Vector3(-5, 0.5f, 0)));
    CHECK(g == 0 && b1 == 1 && b2 == 2 && b3 == 3);
    CHECK(w.getBody(b1).getId() == b1);

    // Capacidade esgotada: length_error; descrição inválida: invalid_argument (mesmo cheio) e nada é consumido.
    CHECK(throws<std::length_error>([&] { w.createBody(boxDesc(Vector3(9, 0.5f, 0))); }));
    RigidBodyDesc bad;   // dinâmico sem forma nem massData
    CHECK(throws<std::invalid_argument>([&] { w.createBody(bad); }));
    CHECK(w.getStats().bodies == 4);

    // Destruir libera o id, que é reaproveitado.
    w.destroyBody(b1);
    CHECK(!w.isValidBody(b1) && w.getStats().bodies == 3);
    CHECK(throws<std::out_of_range>([&] { w.getBody(b1); }));
    CHECK(throws<std::out_of_range>([&] { w.getInterpolatedTransform(b1); }));
    w.destroyBody(b1);        // duplo destroy
    w.destroyBody(999);       // id inexistente
    CHECK(w.createBody(boxDesc(Vector3(0, 0.5f, 0))) == b1);
    CHECK(w.getStats().bodies == 4);

    // Descrição inválida em mundo com espaço: não consome slot.
    PhysicsWorld w2;
    CHECK(throws<std::invalid_argument>([&] { w2.createBody(bad); }));
    CHECK(w2.createBody(groundDesc()) == 0);

    // Corpo estático sem forma (ponto de ancoragem) é permitido.
    RigidBodyDesc anchor;
    anchor.type = BodyType::Static;
    anchor.transform = Transform(Vector3(0, 5, 0));
    const BodyId a = w2.createBody(anchor);
    // Só o chão (com forma) tem proxy na broadphase: o ponto de ancoragem sem forma não registra nenhum.
    CHECK(w2.getBody(a).isStatic() && w2.getCollisionWorld().getBroadphase().getProxyCount() == 1);

    // Endereços estáveis: criar muitos corpos não invalida referências.
    PhysicsWorldSettings big;
    big.maxBodies = 200;
    PhysicsWorld w3(big);
    const BodyId first = w3.createBody(boxDesc(Vector3(0, 0.5f, 0)));
    const RigidBody* addr = &w3.getBody(first);
    for (int i = 0; i < 150; ++i) w3.createBody(boxDesc(Vector3(Real(i) * 3, 5, 0)));
    CHECK(&w3.getBody(first) == addr);

    // Configurações inválidas.
    CHECK(throws<std::invalid_argument>([] { PhysicsWorldSettings x; x.fixedTimeStep = 0; PhysicsWorld p(x); }));
    CHECK(throws<std::invalid_argument>([] { PhysicsWorldSettings x; x.maxSubSteps = 0; PhysicsWorld p(x); }));
    CHECK(throws<std::invalid_argument>([] { PhysicsWorldSettings x; x.maxBodies = 0; PhysicsWorld p(x); }));

    // forEachBody visita só os vivos, em ordem de id.
    int visited = 0;
    BodyId last = 0;
    bool ordered = true;
    w.destroyBody(b2);
    w.forEachBody([&](BodyId id, RigidBody&) { ordered = ordered && (visited == 0 || id > last); last = id; ++visited; });
    CHECK(visited == 3 && ordered);
}

// =============================================================================
// PhysicsWorld: simulação
// =============================================================================
static void testFreeFall() {
    PhysicsWorld w;
    const BodyId id = w.createBody(boxDesc(Vector3(0, 100, 0)));
    const int n = 60;
    for (int i = 0; i < n; ++i) w.step();
    // Solução discreta exata do Euler semi-implícito: y_n = y0 − g Δt² n(n+1)/2.
    const Real expected = Real(100) - kGravity * kDt * kDt * Real(n) * Real(n + 1) / 2;
    CHECK(nearlyEqual(w.getBody(id).getWorldCenterOfMass().y, expected, kTol));
    CHECK(w.getStats().stepCount == 60);

    // Gravidade configurável.
    PhysicsWorld moon;
    moon.setGravity(Vector3(0, Real(-1.62), 0));
    const BodyId m = moon.createBody(boxDesc(Vector3(0, 10, 0)));
    for (int i = 0; i < 60; ++i) moon.step();
    CHECK(std::abs(moon.getBody(m).getLinearVelocity().y + Real(1.62)) < Real(0.01));
}

/// Torre de 10 caixas perfeitamente alinhada (o caso mais difícil para um solver iterativo): roda
/// `seconds` s e devolve o pico de deriva do topo, a deriva no fim, o erro de altura e a
/// velocidade máxima na última janela de 5 s.
struct TowerResult { Real peakDrift, finalDrift, heightError, lateSpeed; bool sleeping; int touching; Real window[6]; };

static TowerResult runTower(int velocityIterations, int positionIterations, int seconds) {
    PhysicsWorldSettings st;
    st.solver.velocityIterations = velocityIterations;
    st.solver.positionIterations = positionIterations;
    PhysicsWorld w(st);
    w.createBody(groundDesc());
    const int N = 10;
    std::vector<BodyId> ids;
    for (int i = 0; i < N; ++i) ids.push_back(w.createBody(boxDesc(Vector3(0, Real(0.5) + Real(i), 0))));

    TowerResult r{0, 0, 0, 0, false, 0, {0, 0, 0, 0, 0, 0}};
    const int steps = seconds * 60;
    for (int i = 0; i < steps; ++i) {
        w.step();
        const Vector3 top = w.getBody(ids.back()).getWorldCenterOfMass();
        r.peakDrift = std::max(r.peakDrift, std::max(std::abs(top.x), std::abs(top.z)));
        for (BodyId id : ids) {   // pico de velocidade por janela de 10 s (tendência: decai, estável ou cresce?)
            const int wdw = std::min(5, i / 600);
            r.window[wdw] = std::max(r.window[wdw], w.getBody(id).getLinearVelocity().length());
        }
        if (i >= steps - 300) {
            for (BodyId id : ids) r.lateSpeed = std::max(r.lateSpeed, w.getBody(id).getLinearVelocity().length());
        }
    }
    const Vector3 top = w.getBody(ids.back()).getWorldCenterOfMass();
    r.finalDrift = std::max(std::abs(top.x), std::abs(top.z));
    r.heightError = std::abs(top.y - (Real(N) - Real(0.5)));
    r.sleeping = !w.getBody(ids.back()).isAwake();
    r.touching = w.getStats().touchingPairs;
    return r;
}

static void testStacking() {
    // 60 s com as configurações PADRÃO (10 iterações de velocidade, NGS). O critério NÃO é a
    // deriva num instante qualquer (a torre balança e o instante cai numa fase arbitrária) nem
    // "ficou parada": 10 iterações não bastam para um solver iterativo assentar 10 caixas
    // empilhadas. Medido: em double ela assenta e dorme em ~20 s; em float oscila de forma
    // sustentada (0,2 a 0,4 m/s por janela de 10 s, deriva <= 0,2 m), sem decair nem crescer.
    // O critério é o que o NGS garante: continua em pé, limitada e sem crescer. (Com o Baumgarte
    // legado, positionIterations = 0, esta torre tomba entre 20 e 40 s em float: deriva de ~19 m,
    // porque a correção de penetração vira velocidade e bombeia o balanço.)
    const TowerResult r = runTower(10, kPositionIterations, 60);
    std::printf("  [info] torre de 10 caixas, 60 s, 10 iteracoes + NGS: pico de deriva=%.4f m, final=%.4f m, "
                "erro de altura=%.4f m, %s\n", double(r.peakDrift), double(r.finalDrift), double(r.heightError),
                r.sleeping ? "dormindo" : "acordada");
    std::printf("  [info]   pico de velocidade por janela de 10 s: %.3f %.3f %.3f %.3f %.3f %.3f m/s\n",
                double(r.window[0]), double(r.window[1]), double(r.window[2]), double(r.window[3]), double(r.window[4]), double(r.window[5]));
    Real maxWindow = 0;
    for (Real v : r.window) maxWindow = std::max(maxWindow, v);
    CHECK(r.peakDrift < Real(0.35));                 // nunca sai de cima da base (meia caixa = 0,5 m)
    CHECK(r.heightError < Real(0.05));               // as 10 caixas continuam empilhadas
    CHECK(r.finalDrift < Real(0.2));
    CHECK(maxWindow < Real(0.6));                    // balanço limitado
    CHECK(r.window[5] <= Real(2) * std::max(r.window[1], Real(0.05)));   // e que não cresce
    CHECK(r.touching == 10);                         // cada caixa toca a de baixo (ou o chão)

    // Com mais iterações o solver CONVERGE: o pico de deriva cai e a torre assenta e adormece.
    // (Medido em float e double: 15 iterações já assentam; 20 dão folga.)
    const TowerResult r20 = runTower(20, kPositionIterations, 60);
    std::printf("  [info] mesma torre com 20 iteracoes: pico de deriva=%.4f m (contra %.4f com 10), %s\n",
                double(r20.peakDrift), double(r.peakDrift), r20.sleeping ? "dormindo" : "acordada");
    CHECK(r20.peakDrift < r.peakDrift);
    CHECK(r20.heightError < Real(0.05));
    CHECK(r20.sleeping);                              // assentou
    CHECK(r20.window[5] == Real(0));                  // dormindo: velocidade zero
}

static void testPyramid() {
    PhysicsWorld w;
    w.createBody(groundDesc());
    const int rows = 6;
    std::vector<BodyId> ids;
    std::vector<Vector3> start;
    for (int r = 0; r < rows; ++r) {
        const int count = rows - r;
        for (int c = 0; c < count; ++c) {
            const Vector3 p(Real(c) - Real(count - 1) * Real(0.5), Real(0.5) + Real(r), 0);
            ids.push_back(w.createBody(boxDesc(p)));
            start.push_back(p);
        }
    }
    for (int i = 0; i < 300; ++i) w.step();   // 5 s
    Real maxMove = 0, maxSpeed = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        maxMove = std::max(maxMove, (w.getBody(ids[i]).getWorldCenterOfMass() - start[i]).length());
        maxSpeed = std::max(maxSpeed, w.getBody(ids[i]).getLinearVelocity().length());
    }
    std::printf("  [info] piramide de %zu caixas, 5 s: deslocamento max=%.4f m, vel max=%.5f m/s\n", ids.size(), double(maxMove), double(maxSpeed));
    CHECK(maxMove < Real(0.05) && maxSpeed < Real(0.01));
}

static void testBounce() {
    PhysicsWorld w;
    w.createBody(groundDesc());
    RigidBodyDesc d = sphereDesc(Vector3(0, 3, 0));
    d.restitution = Real(0.8);
    const BodyId b = w.createBody(d);

    bool landed = false;
    Real peak = 0;
    for (int i = 0; i < 240; ++i) {
        w.step();
        const Real y = w.getBody(b).getWorldCenterOfMass().y;
        if (!landed && w.getStats().touchingPairs > 0) landed = true;
        else if (landed && w.getBody(b).getLinearVelocity().y > 0) peak = std::max(peak, y);
    }
    // Altura de queda 2,5 m (centro 3 → 0,5); depois do quique: e²·h acima de 0,5.
    const Real expectedPeak = Real(0.5) + Real(0.64) * Real(2.5);
    std::printf("  [info] quique e=0.8: pico=%.3f m (ideal %.3f)\n", double(peak), double(expectedPeak));
    CHECK(landed);
    CHECK(std::abs(peak - expectedPeak) < Real(0.15));
}

// Regressão: com restituição alta a bola tem de ASSENTAR. A restituição usava a velocidade já
// acelerada pela gravidade do passo, então cada batida ganhava g·Δt de "impacto" e havia um ponto
// fixo v* = e·g·Δt/(1-e) acima do velocity slop (e = 0,8: 0,65 m/s; e = 0,9: 1,5 m/s): a bola
// pulava PARA SEMPRE com amplitude constante. Falha no motor sem a correção (para e >= 0,8).
// O sleeping fica desligado: o que se testa é o assentamento físico, não o sono.
static void testRestitutionSettles() {
    for (Real e : {Real(0.5), Real(0.8), Real(0.9), Real(0.95)}) {
        PhysicsWorld w;
        w.sleepSettings().enabled = false;
        w.createBody(groundDesc());
        RigidBodyDesc d = sphereDesc(Vector3(0, 3, 0));
        d.restitution = e;
        const BodyId b = w.createBody(d);

        int steps = 0;
        Real maxSpeedLate = 0, minY = 100, maxY = -100;
        for (; steps < 3600; ++steps) {   // 60 s: cobre a série de quiques de e = 0,95
            w.step();
            if (steps >= 3000) {          // últimos 10 s
                const RigidBody& rb = w.getBody(b);
                maxSpeedLate = std::max(maxSpeedLate, rb.getLinearVelocity().length());
                minY = std::min(minY, rb.getWorldCenterOfMass().y);
                maxY = std::max(maxY, rb.getWorldCenterOfMass().y);
            }
        }
        std::printf("  [info] restituicao e=%.2f: nos ultimos 10 s |v| max = %.4f m/s, y em [%.4f, %.4f]\n", double(e),
                    double(maxSpeedLate), double(minY), double(maxY));
        CHECK(maxSpeedLate < Real(0.1));                          // parada, sem pulinhos
        CHECK(minY > Real(0.48) && maxY < Real(0.52));            // assentada sobre o chão (raio 0,5)
    }
}

static void testKinematicPlatform() {
    // Plataforma cinemática a 1 m/s carrega uma caixa só pelo atrito.
    PhysicsWorld w;
    RigidBodyDesc plat;
    plat.type = BodyType::Kinematic;
    plat.shape = std::make_shared<BoxShape>(Vector3(40, Real(0.5), 5));
    plat.transform = Transform(Vector3(0, Real(-0.5), 0));
    plat.linearVelocity = Vector3(1, 0, 0);
    const BodyId p = w.createBody(plat);
    const BodyId b = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
    for (int i = 0; i < 240; ++i) w.step();   // 4 s
    const RigidBody& box = w.getBody(b);
    CHECK(std::abs(box.getLinearVelocity().x - Real(1)) < Real(0.05));
    CHECK(std::abs(box.getWorldCenterOfMass().y - Real(0.5)) < Real(0.02));
    CHECK(std::abs(w.getBody(p).getWorldCenterOfMass().y + Real(0.5)) < Real(1e-4));   // cinemático ignora gravidade
    CHECK(w.getBody(p).getWorldCenterOfMass().x > Real(3.9));
}

static void testJointsInWorld() {
    // --- Pêndulo: DistanceJoint entre uma âncora estática e uma esfera --------------------------
    {
        PhysicsWorld w;
        RigidBodyDesc anchor;
        anchor.type = BodyType::Static;
        anchor.transform = Transform(Vector3(0, 5, 0));
        const BodyId a = w.createBody(anchor);
        const BodyId b = w.createBody(sphereDesc(Vector3(2, 5, 0), Real(0.25)));
        DistanceJoint& j = w.createDistanceJoint(a, b, Vector3(0, 5, 0), Vector3(2, 5, 0));
        CHECK(nearlyEqual(j.getLength(), Real(2), kTol));
        Real maxErr = 0, minY = 100;
        for (int i = 0; i < 120; ++i) {
            w.step();
            const Vector3 p = w.getBody(b).getWorldCenterOfMass();
            maxErr = std::max(maxErr, std::abs((p - Vector3(0, 5, 0)).length() - Real(2)));
            minY = std::min(minY, p.y);
        }
        // Com NGS (correção de posição sem velocidade) o comprimento não deriva: medido < 5e-6 m em
        // float e double. O Baumgarte legado dava 1,3 cm a ~6 m/s (a = v²/L ~ 17 m/s²).
        CHECK(maxErr < Real(0.002));
        CHECK(minY < Real(3.2) && minY > Real(2.9));   // desce até ~ 5 − 2 = 3
    }
    // --- Corpos ligados por junta não colidem (padrão), a menos que se peça ---------------------------
    auto overlapping = [](bool useJoint, bool collideConnected, int& touching, int& pairs) {
        PhysicsWorld w;
        w.setGravity(Vector3::zero());
        const BodyId a = w.createBody(boxDesc(Vector3(0, 0, 0)));
        const BodyId b = w.createBody(boxDesc(Vector3(Real(0.9), 0, 0)));   // 0,1 m de interpenetração
        if (useJoint) w.createBallSocketJoint(a, b, Vector3(Real(0.45), 0, 0), collideConnected);
        w.step();
        touching = w.getStats().touchingPairs;
        pairs = w.getStats().pairs;
    };
    int touching = 0, pairs = 0;
    overlapping(false, false, touching, pairs);
    CHECK(touching == 1 && pairs == 1);            // controle: sem junta, colidem
    overlapping(true, false, touching, pairs);
    CHECK(touching == 0 && pairs == 0);            // com junta e collideConnected=false: ignoram-se
    overlapping(true, true, touching, pairs);
    CHECK(touching == 1);                          // collideConnected=true: colidem

    // --- Remoção de corpo/junta limpa tudo, inclusive a regra de não colisão ----------------------------------
    {
        PhysicsWorld w;
        w.setGravity(Vector3::zero());
        const BodyId a = w.createBody(boxDesc(Vector3(0, 0, 0)));
        const BodyId b = w.createBody(boxDesc(Vector3(Real(0.9), 0, 0)));
        w.createBallSocketJoint(a, b, Vector3(Real(0.45), 0, 0));
        CHECK(w.getStats().joints == 1);
        w.destroyBody(b);
        CHECK(w.getStats().joints == 0 && w.getStats().bodies == 1);
        const BodyId b2 = w.createBody(boxDesc(Vector3(Real(0.9), 0, 0)));   // id reaproveitado
        CHECK(b2 == b);
        w.step();
        CHECK(w.getStats().touchingPairs == 1);   // a regra de "não colidir" não vazou para o id novo

        // destroyJoint com junta desconhecida é ignorado.
        BallSocketJoint& j = w.createBallSocketJoint(a, b2, Vector3(Real(0.45), 0, 0));
        w.destroyJoint(j);
        CHECK(w.getStats().joints == 0);
        PhysicsWorld other;
        const BodyId oa = other.createBody(boxDesc(Vector3()));
        const BodyId ob = other.createBody(boxDesc(Vector3(5, 0, 0)));
        BallSocketJoint& foreign = other.createBallSocketJoint(oa, ob, Vector3(2, 0, 0));
        w.destroyJoint(foreign);                  // não pertence a `w`
        CHECK(other.getStats().joints == 1);
    }
    // --- Parâmetros inválidos ------------------------------------------------------------------------------
    {
        PhysicsWorld w;
        const BodyId a = w.createBody(boxDesc(Vector3()));
        const BodyId b = w.createBody(boxDesc(Vector3(3, 0, 0)));
        const Real nan = std::numeric_limits<Real>::quiet_NaN();
        CHECK(throws<std::invalid_argument>([&] { w.createBallSocketJoint(a, a, Vector3()); }));
        CHECK(throws<std::invalid_argument>([&] { w.createBallSocketJoint(a, 77, Vector3()); }));
        CHECK(throws<std::invalid_argument>([&] { w.createBallSocketJoint(a, b, Vector3(nan, 0, 0)); }));
        CHECK(throws<std::invalid_argument>([&] { w.createHingeJoint(a, b, Vector3(), Vector3::zero()); }));
        CHECK(throws<std::invalid_argument>([&] { w.createDistanceJoint(a, b, Vector3(), Vector3(3, 0, 0), nan); }));
        CHECK(w.getStats().joints == 0);
    }
}

// =============================================================================
// Determinismo e independência da taxa de quadros
// =============================================================================
static void testDeterminism() {
    PhysicsWorld a, b;
    const int n = buildScene(a);
    buildScene(b);
    for (int i = 0; i < 300; ++i) { a.step(); b.step(); }
    CHECK(sameState(a, b, n));
    CHECK(a.getBody(1).getWorldCenterOfMass().isFinite());
}

static void testFrameRateIndependence() {
    const Real total = Real(3);
    struct Scheme { const char* name; int kind; };
    const Scheme schemes[] = {{"144 Hz", 0}, {"30 Hz", 1}, {"irregular", 2}};
    for (const Scheme& sc : schemes) {
        PhysicsWorld w;
        const int n = buildScene(w);
        Rng rng(4242);
        Real t = 0;
        int steps = 0;
        while (t < total) {
            Real ft = sc.kind == 0 ? Real(1) / Real(144) : (sc.kind == 1 ? Real(1) / Real(30)
                                                                          : Real(1) / Real(240) + rng.next() * (Real(1) / Real(20) - Real(1) / Real(240)));
            t += ft;
            steps += w.update(ft);
        }
        const std::uint64_t N = w.getStats().stepCount;
        CHECK(N == static_cast<std::uint64_t>(steps));
        CHECK(std::abs(static_cast<double>(N) - std::floor(static_cast<double>(t / kDt))) <= 1.0);
        CHECK(w.getAccumulator().getDroppedTime() == Real(0));

        // O estado depende só do NÚMERO de passos fixos, não de como o tempo de render foi fatiado.
        PhysicsWorld ref;
        buildScene(ref);
        for (std::uint64_t i = 0; i < N; ++i) ref.step();
        CHECK(sameState(w, ref, n));
    }
}

// =============================================================================
// Bridge e interpolação de render
// =============================================================================
struct CountingBridge final : IPhysicsTransformBridge {
    int created = 0, destroyed = 0, begins = 0, ends = 0;
    std::vector<int> updates;      // por BodyId
    Real lastAlpha = -1;
    bool balanced = true;          // begin/update/end bem ordenados
    bool inSync = false;

    void onBodyCreated(BodyId, const Transform&) override { ++created; }
    void onBodyDestroyed(BodyId) override { ++destroyed; }
    void beginSync(Real a) override { balanced = balanced && !inSync; inSync = true; ++begins; lastAlpha = a; }
    void onTransformUpdated(BodyId id, const Transform&) override {
        balanced = balanced && inSync;
        if (id >= updates.size()) updates.resize(id + 1, 0);
        ++updates[id];
    }
    void endSync() override { balanced = balanced && inSync; inSync = false; ++ends; }
};

static void testBridgeCallbacks() {
    PhysicsWorld w;
    CountingBridge cb;
    w.setBridge(&cb);
    const BodyId g = w.createBody(groundDesc());
    const BodyId b = w.createBody(boxDesc(Vector3(0, 10, 0)));
    CHECK(cb.created == 2);

    w.update(kDt);
    w.update(kDt);
    w.update(kDt);
    CHECK(cb.begins == 3 && cb.ends == 3 && cb.balanced);
    CHECK(cb.updates[b] == 3);        // dinâmico: a cada sincronização
    CHECK(cb.updates[g] == 1);        // estático: só na primeira (estava "sujo" desde a criação)

    // Teletransportar um estático o reenvia uma vez.
    w.setBodyTransform(g, Transform(Vector3(0, -2, 0)));
    w.update(kDt);
    w.update(kDt);
    CHECK(cb.updates[g] == 2);

    w.destroyBody(b);
    CHECK(cb.destroyed == 1);
    w.update(kDt);
    CHECK(cb.updates[b] == 5 - 0 - 2 + 0 || cb.updates[b] == 5);   // não recebe mais updates depois de destruído

    // Ligar a bridge depois: recebe onBodyCreated de todos os corpos vivos.
    PhysicsWorld late;
    late.createBody(groundDesc());
    late.createBody(boxDesc(Vector3(0, 5, 0)));
    CountingBridge lateBridge;
    late.setBridge(&lateBridge);
    CHECK(lateBridge.created == 2);
    late.setBridge(nullptr);
    late.update(kDt);    // sem bridge: nada quebra
    CHECK(lateBridge.begins == 0);
}

static void testInterpolation() {
    PhysicsWorld w;
    BufferedTransformBridge br;
    w.setBridge(&br);
    const BodyId id = w.createBody(boxDesc(Vector3(0, 10, 0)));

    // Um passo e meio quadro: α = 0,5 e a bridge recebe exatamente a pose interpolada.
    CHECK(w.update(kDt) == 1);
    CHECK(w.update(kDt * Real(0.5)) == 0);
    CHECK(nearlyEqual(w.getAlpha(), Real(0.5), kTol));
    const MotionState& ms = w.getMotionState(id);
    const Transform expected = Transform::interpolate(ms.getPrevious(), ms.getCurrent(), Real(0.5));
    CHECK(br.getTransform(id).nearlyEqual(expected, Real(1e-4)));
    CHECK(w.getInterpolatedTransform(id).nearlyEqual(expected, Real(1e-6)));
    CHECK(nearlyEqual(br.getLastAlpha(), Real(0.5), kTol));
    // A pose de render fica ENTRE as duas últimas poses físicas (um passo de atraso).
    const Real yPrev = ms.getPrevious().position.y, yCur = ms.getCurrent().position.y;
    CHECK(yCur < yPrev && br.getTransform(id).position.y < yPrev + Real(1e-4) && br.getTransform(id).position.y > yCur - Real(1e-4));

    // Render a 240 Hz sobre física a 60 Hz: a pose física repete ~4 quadros seguidos
    // (engasga), a interpolada muda em TODOS os quadros.
    PhysicsWorld w2;
    BufferedTransformBridge br2;
    w2.setBridge(&br2);
    const BodyId f = w2.createBody(boxDesc(Vector3(0, 50, 0)));
    const int frames = 240;
    int rawRepeats = 0, interpRepeats = 0;
    bool monotone = true;
    // Os quadros anteriores ao 1º passo físico (a 240 Hz, 3 quadros) não têm o que interpolar: não contam.
    while (w2.getStats().stepCount == 0) w2.update(Real(1) / Real(240));
    Real prevRaw = w2.getBody(f).getWorldCenterOfMass().y;
    Real prevInterp = br2.getTransform(f).position.y;
    for (int i = 0; i < frames; ++i) {
        w2.update(Real(1) / Real(240));
        const Real raw = w2.getBody(f).getWorldCenterOfMass().y;
        const Real interp = br2.getTransform(f).position.y;
        if (raw == prevRaw) ++rawRepeats;
        if (interp == prevInterp) ++interpRepeats;
        monotone = monotone && interp <= prevInterp;
        prevRaw = raw;
        prevInterp = interp;
    }
    std::printf("  [info] render 240 Hz / fisica 60 Hz: quadros sem movimento: fisica pura=%d/%d, interpolada=%d/%d\n",
                rawRepeats, frames, interpRepeats, frames);
    CHECK(rawRepeats > frames / 2);
    CHECK(interpRepeats == 0 && monotone);
}

static void testSpiralAndTeleport() {
    // Espiral da morte: quadro de 5 s executa no máximo maxSubSteps passos.
    PhysicsWorld w;
    w.createBody(boxDesc(Vector3(0, 10, 0)));
    CHECK(w.update(Real(5)) == w.getAccumulator().getMaxSubSteps());
    CHECK(w.getStats().stepCount == static_cast<std::uint64_t>(kMaxSubSteps));
    CHECK(w.getAccumulator().getDroppedTime() > Real(4.8));
    CHECK(w.getAlpha() >= Real(0) && w.getAlpha() < Real(1));
    CHECK(w.update(std::numeric_limits<Real>::quiet_NaN()) == 0);
    CHECK(w.update(Real(-2)) == 0);

    // Teletransporte: a interpolação não deixa rastro.
    PhysicsWorld t;
    BufferedTransformBridge br;
    t.setBridge(&br);
    const BodyId id = t.createBody(boxDesc(Vector3(0, 10, 0)));
    for (int i = 0; i < 5; ++i) t.update(kDt * Real(1.5));
    const Transform dest(Vector3(100, 20, -30), Quaternion::fromAxisAngle(Vector3::unitY(), Real(1)));
    t.setBodyTransform(id, dest);
    CHECK(t.getInterpolatedTransform(id).nearlyEqual(dest, Real(1e-4)));   // vale para qualquer α
    t.syncBridge();
    CHECK(br.getTransform(id).nearlyEqual(dest, Real(1e-3)));
    t.setBodyTransform(id, Transform(Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0)));   // inválido: ignorado
    CHECK(t.getBody(id).getWorldCenterOfMass().isFinite());
}

static void testCollisionFilterInWorld() {
    PhysicsWorld w;
    w.setGravity(Vector3::zero());
    w.setCollisionFilter([](BodyId, BodyId) { return false; });   // nada colide
    w.createBody(boxDesc(Vector3(0, 0, 0)));
    w.createBody(boxDesc(Vector3(Real(0.9), 0, 0)));
    w.step();
    CHECK(w.getStats().pairs == 0 && w.getStats().touchingPairs == 0);

    // Estático × estático nunca forma par, mesmo sem filtro.
    PhysicsWorld s;
    RigidBodyDesc a = groundDesc();
    RigidBodyDesc b = groundDesc();
    s.createBody(a);
    s.createBody(b);
    s.step();
    CHECK(s.getStats().pairs == 0);
}

// =============================================================================
// Desempenho (apenas informativo) e robustez
// =============================================================================
static void testScaleAndRobustness() {
    // Pirâmide grande: estabilidade + custo por passo.
    PhysicsWorldSettings s;
    s.maxBodies = 512;
    PhysicsWorld w(s);
    w.createBody(groundDesc());
    const int rows = 20;
    int boxes = 0;
    for (int r = 0; r < rows; ++r) {
        const int count = rows - r;
        for (int c = 0; c < count; ++c) {
            w.createBody(boxDesc(Vector3(Real(c) - Real(count - 1) * Real(0.5), Real(0.5) + Real(r), 0)));
            ++boxes;
        }
    }
    for (int i = 0; i < 60; ++i) w.step();   // acomoda
    const auto t0 = std::chrono::steady_clock::now();
    const int steps = 120;
    for (int i = 0; i < steps; ++i) w.step();
    const auto t1 = std::chrono::steady_clock::now();
    const double msPerStep = std::chrono::duration<double, std::milli>(t1 - t0).count() / steps;

    Real maxSpeed = 0;
    bool finite = true;
    w.forEachBody([&](BodyId, RigidBody& b) {
        maxSpeed = std::max(maxSpeed, b.getLinearVelocity().length());
        finite = finite && b.getWorldCenterOfMass().isFinite() && b.getOrientation().isFinite();
    });
    std::printf("  [info] piramide de %d caixas: %.3f ms/passo, vel max=%.4f m/s, pares=%d, contatos=%d\n", boxes,
                msPerStep, double(maxSpeed), w.getStats().pairs, w.getStats().contactPoints);
    CHECK(finite && maxSpeed < Real(0.5));

    // Estresse: muitas caixas caindo em cima umas das outras; nada pode virar NaN.
    PhysicsWorldSettings s2;
    s2.maxBodies = 300;
    PhysicsWorld rain(s2);
    rain.createBody(groundDesc());
    Rng rng(99);
    for (int i = 0; i < 200; ++i) {
        RigidBodyDesc d = boxDesc(Vector3(rng.next() * 6 - 3, Real(1) + rng.next() * 30, rng.next() * 6 - 3),
                                  Real(0.2) + rng.next() * Real(0.4), Real(1) + rng.next() * 4);
        d.transform.rotation = Quaternion::fromAxisAngle(Vector3(rng.next(), rng.next(), rng.next() + Real(0.1)), rng.next() * 6);
        d.angularVelocity = Vector3(rng.next() - Real(0.5), rng.next() - Real(0.5), rng.next() - Real(0.5)) * 4;
        rain.createBody(d);
    }
    for (int i = 0; i < 600; ++i) rain.step();
    bool ok = true;
    Real maxY = 0;
    rain.forEachBody([&](BodyId, RigidBody& b) {
        ok = ok && b.getWorldCenterOfMass().isFinite() && b.getOrientation().isFinite() &&
             nearlyEqual(b.getOrientation().norm(), Real(1), Real(1e-3));
        maxY = std::max(maxY, b.getWorldCenterOfMass().y);
    });
    std::printf("  [info] chuva de 200 caixas aleatorias, 10 s: pares=%d, contatos=%d, altura max=%.2f m\n",
                rain.getStats().pairs, rain.getStats().contactPoints, double(maxY));
    CHECK(ok);
}

// -----------------------------------------------------------------------------
// Ajustes em tempo de execução: Δt fixo e profiling
// -----------------------------------------------------------------------------
static void testRuntimeSettings() {
    // Acumulador: troca de passo preserva α.
    FixedTimestepAccumulator acc(Real(1) / Real(60), 8);
    CHECK(acc.advance(Real(0.4) / Real(60)) == 0);
    CHECK(nearlyEqual(acc.getAlpha(), Real(0.4), Real(1e-5)));
    CHECK(acc.setFixedDt(Real(1) / Real(30)));
    CHECK(nearlyEqual(acc.getFixedDt(), Real(1) / Real(30), Real(1e-7)));
    CHECK(nearlyEqual(acc.getAlpha(), Real(0.4), Real(1e-5)));
    CHECK(!acc.setFixedDt(Real(0)) && !acc.setFixedDt(Real(-1)) &&
          !acc.setFixedDt(std::numeric_limits<Real>::quiet_NaN()) &&
          !acc.setFixedDt(std::numeric_limits<Real>::infinity()));
    CHECK(nearlyEqual(acc.getFixedDt(), Real(1) / Real(30), Real(1e-7)));
    CHECK(acc.advance(Real(1) / Real(30)) == 1);   // o novo passo vale

    // Mundo: Δt em runtime. 1/120 s: um quadro de 1/60 s executa 2 passos.
    PhysicsWorld w;
    RigidBodyDesc d;
    d.shape = std::make_shared<SphereShape>(Real(0.5));
    d.mass = Real(1);
    d.transform = Transform(Vector3(0, 100, 0));
    d.angularDamping = Real(0);
    const BodyId id = w.createBody(d);
    CHECK(w.setFixedTimeStep(Real(1) / Real(120)));
    CHECK(nearlyEqual(w.getFixedTimeStep(), Real(1) / Real(120), Real(1e-7)));
    const std::uint64_t s0 = w.getStats().stepCount;
    CHECK(w.update(Real(1) / Real(60)) == 2);
    CHECK(w.getStats().stepCount == s0 + 2);
    CHECK(!w.setFixedTimeStep(Real(0)) && !w.setFixedTimeStep(Real(2)) && !w.setFixedTimeStep(Real(1e-6)) &&
          !w.setFixedTimeStep(std::numeric_limits<Real>::quiet_NaN()));
    CHECK(nearlyEqual(w.getFixedTimeStep(), Real(1) / Real(120), Real(1e-7)));
    CHECK(w.setFixedTimeStep(w.kMinRuntimeTimeStep) && w.setFixedTimeStep(w.kMaxRuntimeTimeStep));

    // Queda livre: 1 s simulado com Δt = 1/60 e com Δt = 1/240 chega quase ao mesmo lugar (Euler semi-implícito).
    PhysicsWorld a, b;
    const BodyId ia = a.createBody(d), ib = b.createBody(d);
    CHECK(b.setFixedTimeStep(Real(1) / Real(240)));
    for (int i = 0; i < 60; ++i) a.step();
    for (int i = 0; i < 240; ++i) b.step();
    const Real ya = a.getBody(ia).getWorldCenterOfMass().y, yb = b.getBody(ib).getWorldCenterOfMass().y;
    CHECK(std::fabs(ya - yb) < Real(0.1));
    CHECK(std::fabs(yb - (Real(100) - Real(0.5) * Real(9.81))) < Real(0.1));   // y = y0 - g t²/2
    (void)id;

    // Profiling liga/desliga em tempo de execução.
    PhysicsWorld p;
    p.createBody(d);
    CHECK(!p.isProfilingEnabled());
    p.step();
    CHECK(p.getStats().timings.totalUs == 0.0);
    p.setProfilingEnabled(true);
    CHECK(p.isProfilingEnabled());
    p.step();
    CHECK(p.getStats().timings.totalUs > 0.0);
    p.setProfilingEnabled(false);
    p.step();
    CHECK(p.getStats().timings.totalUs == 0.0);
}

int main() {
    testAccumulator();
    testRuntimeSettings();
    testMotionState();
    testBufferedBridge();
    testCollisionWorld();
    testWorldLifecycle();
    testFreeFall();
    testStacking();
    testPyramid();
    testBounce();
    testRestitutionSettles();
    testKinematicPlatform();
    testJointsInWorld();
    testDeterminism();
    testFrameRateIndependence();
    testBridgeCallbacks();
    testInterpolation();
    testSpiralAndTeleport();
    testCollisionFilterInWorld();
    testScaleAndRobustness();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
