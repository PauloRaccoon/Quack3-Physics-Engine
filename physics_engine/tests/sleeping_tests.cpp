// =============================================================================
// sleeping_tests.cpp — sleeping, ilhas de simulação e perfilamento (Fase 9).
//
// Contrato verificado:
//   - corpos em repouso adormecem (v = ω = 0), pose imutável, sem custo de solver/narrowphase;
//   - cada gatilho acorda (força, impulso, velocidade ≠ 0, teleporte, gravidade, contato com corpo
//     acordado, plataforma cinemática em movimento, destruição do apoio, corpo novo por cima...);
//   - ilhas dormem e acordam INTEIRAS; um membro que não pode dormir segura a ilha toda;
//   - dormir não muda o resultado físico (comparado com sleeping desligado) nem o determinismo;
//   - interpolação, bridge e consultas continuam coerentes com corpos dormindo.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

#include "physics/bridge/IPhysicsTransformBridge.h"
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

// -----------------------------------------------------------------------------
// Fábricas
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

static void run(PhysicsWorld& w, int steps) {
    for (int i = 0; i < steps; ++i) w.step();
}

/// Roda até `pred()` ser verdadeiro (no máximo `maxSteps`); devolve se foi.
template <typename P>
static bool runUntil(PhysicsWorld& w, int maxSteps, P&& pred) {
    for (int i = 0; i < maxSteps; ++i) {
        if (pred()) return true;
        w.step();
    }
    return pred();
}

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
};

/// Mundo com chão e uma caixa que já adormeceu sobre ele.
struct SleepyBox {
    PhysicsWorld w;
    BodyId ground = 0, box = 0;
    SleepyBox() {
        ground = w.createBody(groundDesc());
        box = w.createBody(boxDesc(Vector3(0, Real(0.52), 0)));
        runUntil(w, 600, [&] { return !w.isAwake(box); });
    }
};

static bool sameBits(const Vector3& a, const Vector3& b) { return a == b; }

// -----------------------------------------------------------------------------
// 1) Adormecer
// -----------------------------------------------------------------------------
static void testFallsAsleep() {
    PhysicsWorld w;
    const BodyId g = w.createBody(groundDesc());
    const BodyId b = w.createBody(boxDesc(Vector3(0, Real(0.6), 0)));
    CHECK(w.isAwake(b));
    CHECK(!w.isAwake(g));   // estático: nunca "acordado"

    // Não dorme antes de ficar em repouso: nos primeiros 10 passos a caixa ainda está caindo/pousando.
    run(w, 10);
    CHECK(w.isAwake(b));

    const bool slept = runUntil(w, 600, [&] { return !w.isAwake(b); });
    CHECK(slept);
    std::printf("  [info] caixa solta de 10 cm adormeceu em %llu passos (%.2f s)\n",
                static_cast<unsigned long long>(w.getStats().stepCount), double(w.getStats().stepCount) * double(kDt));

    const RigidBody& body = w.getBody(b);
    CHECK(body.getLinearVelocity() == Vector3::zero() && body.getAngularVelocity() == Vector3::zero());
    CHECK(std::abs(body.getWorldCenterOfMass().y - Real(0.5)) < Real(0.02));

    // Estatísticas: dormindo = sem custo de solver nem de narrowphase.
    run(w, 2);
    const WorldStats st = w.getStats();
    CHECK(st.sleepingBodies == 1 && st.awakeBodies == 0 && st.islands == 0);
    CHECK(st.solverContacts == 0 && st.solverJoints == 0);
    CHECK(st.frozenPairs == 1 && st.touchingPairs == 1);   // o par chão-caixa existe, mas congelado

    // Pose imutável (bit a bit) enquanto dorme.
    const Vector3 p0 = body.getWorldCenterOfMass();
    const Quaternion q0 = body.getOrientation();
    run(w, 180);
    CHECK(sameBits(body.getWorldCenterOfMass(), p0));
    CHECK(body.getOrientation().x == q0.x && body.getOrientation().y == q0.y &&
          body.getOrientation().z == q0.z && body.getOrientation().w == q0.w);
    CHECK(!w.isAwake(b));
}

// -----------------------------------------------------------------------------
// 2) Gatilhos que acordam
// -----------------------------------------------------------------------------
static void testWakeTriggers() {
    {   // Forças e impulsos acordam imediatamente (antes de qualquer passo).
        SleepyBox s;
        CHECK(!s.w.isAwake(s.box));
        s.w.getBody(s.box).applyForce(Vector3(0, 100, 0));
        CHECK(s.w.isAwake(s.box));
    }
    {
        SleepyBox s;
        s.w.getBody(s.box).applyTorque(Vector3(0, 1, 0));
        CHECK(s.w.isAwake(s.box));
    }
    {
        SleepyBox s;
        s.w.getBody(s.box).applyLinearImpulse(Vector3(0, 5, 0));
        CHECK(s.w.isAwake(s.box));
        const Real y0 = s.w.getBody(s.box).getWorldCenterOfMass().y;
        run(s.w, 20);
        CHECK(s.w.getBody(s.box).getWorldCenterOfMass().y > y0 + Real(0.2));   // saiu do lugar de verdade
    }
    {
        SleepyBox s;
        s.w.getBody(s.box).applyImpulseAtPoint(Vector3(0, 0, 1), Vector3(0, Real(0.9), 0));
        CHECK(s.w.isAwake(s.box));
    }
    {   // Velocidade não nula acorda; velocidade zero NÃO.
        SleepyBox s;
        s.w.getBody(s.box).setLinearVelocity(Vector3::zero());
        s.w.getBody(s.box).setAngularVelocity(Vector3::zero());
        CHECK(!s.w.isAwake(s.box));
        s.w.getBody(s.box).setLinearVelocity(Vector3(1, 0, 0));
        CHECK(s.w.isAwake(s.box));
    }
    {
        SleepyBox s;
        s.w.getBody(s.box).setAngularVelocity(Vector3(0, 1, 0));
        CHECK(s.w.isAwake(s.box));
    }
    {   // Teleporte, mudança de gravityScale, de massa e gravidade do mundo.
        SleepyBox s;
        s.w.setBodyTransform(s.box, Transform(Vector3(3, 2, 0)));
        CHECK(s.w.isAwake(s.box));
    }
    {
        SleepyBox s;
        s.w.getBody(s.box).setGravityScale(Real(0.5));
        CHECK(s.w.isAwake(s.box));
    }
    {
        SleepyBox s;
        s.w.setGravity(Vector3(0, Real(-1.62), 0));
        CHECK(s.w.isAwake(s.box));
    }
    {
        SleepyBox s;
        s.w.setAwake(s.box, true);
        CHECK(s.w.isAwake(s.box));
    }
    {   // Entradas inválidas NÃO acordam (são ignoradas).
        SleepyBox s;
        const Real nan = std::numeric_limits<Real>::quiet_NaN();
        s.w.getBody(s.box).applyForce(Vector3(nan, 0, 0));
        s.w.getBody(s.box).applyLinearImpulse(Vector3(0, nan, 0));
        s.w.getBody(s.box).setLinearVelocity(Vector3(nan, 0, 0));
        s.w.getBody(s.box).setGravityScale(nan);
        s.w.setGravity(Vector3(0, nan, 0));
        CHECK(!s.w.isAwake(s.box));
    }
    {   // Corpo estático e cinemático: setAwake é ignorado; cinemático é sempre "acordado".
        PhysicsWorld w;
        const BodyId g = w.createBody(groundDesc());
        RigidBodyDesc k;
        k.type = BodyType::Kinematic;
        k.shape = std::make_shared<BoxShape>(Vector3(1, 1, 1));
        k.transform = Transform(Vector3(0, 5, 0));
        const BodyId kin = w.createBody(k);
        w.setAwake(g, true);
        w.setAwake(kin, false);
        CHECK(!w.isAwake(g) && w.isAwake(kin));
    }
}

// -----------------------------------------------------------------------------
// 3) Ilhas
// -----------------------------------------------------------------------------
static void testIslands() {
    {   // Uma pilha dorme junta; tocar em qualquer corpo acorda a pilha toda.
        PhysicsWorld w;
        w.createBody(groundDesc());
        BodyId ids[3];
        for (int i = 0; i < 3; ++i) ids[i] = w.createBody(boxDesc(Vector3(0, Real(0.5) + Real(i) * Real(1.001), 0)));
        const bool slept = runUntil(w, 900, [&] {
            return !w.isAwake(ids[0]) && !w.isAwake(ids[1]) && !w.isAwake(ids[2]);
        });
        CHECK(slept);
        CHECK(w.getStats().islands == 0);

        w.getBody(ids[2]).applyLinearImpulse(Vector3(0, 1, 0));   // toca no TOPO
        CHECK(w.isAwake(ids[2]) && !w.isAwake(ids[0]));           // só o topo, por enquanto
        w.step();                                                 // a ilha acorda no passo seguinte
        CHECK(w.isAwake(ids[0]) && w.isAwake(ids[1]) && w.isAwake(ids[2]));
        CHECK(w.getStats().islands == 1 && w.getStats().solverContacts >= 3);

        // E de baixo para cima: dorme de novo e toca na BASE.
        const bool again = runUntil(w, 900, [&] {
            return !w.isAwake(ids[0]) && !w.isAwake(ids[1]) && !w.isAwake(ids[2]);
        });
        CHECK(again);
        w.getBody(ids[0]).applyForce(Vector3(1, 0, 0));
        w.step();
        CHECK(w.isAwake(ids[1]) && w.isAwake(ids[2]));
    }
    {   // Um membro que não pode dormir segura a ilha inteira; uma ilha separada dorme normalmente.
        PhysicsWorld w;
        w.createBody(groundDesc());
        const BodyId a = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
        RigidBodyDesc top = boxDesc(Vector3(0, Real(1.501), 0));
        top.allowSleep = false;
        const BodyId b = w.createBody(top);
        const BodyId c = w.createBody(boxDesc(Vector3(10, Real(0.5), 0)));   // longe: outra ilha
        run(w, 360);   // 6 s
        CHECK(w.isAwake(a) && w.isAwake(b));   // a tem repouso, mas a ilha tem b
        CHECK(!w.isAwake(c));
        CHECK(w.getStats().islands == 1);
        // Liberar b: a ilha passa a poder dormir.
        w.getBody(b).setAllowSleep(true);
        CHECK(runUntil(w, 900, [&] { return !w.isAwake(a) && !w.isAwake(b); }));
    }
    {   // Corpo sozinho no ar, sem gravidade, em repouso: dorme (ilha de um corpo só).
        PhysicsWorld w;
        w.setGravity(Vector3::zero());
        const BodyId s = w.createBody(sphereDesc(Vector3(0, 5, 0)));
        run(w, 40);
        CHECK(!w.isAwake(s));
        // Em movimento nunca dorme.
        PhysicsWorld w2;
        w2.setGravity(Vector3::zero());
        RigidBodyDesc d = sphereDesc(Vector3(0, 5, 0));
        d.linearVelocity = Vector3(Real(0.5), 0, 0);
        const BodyId m = w2.createBody(d);
        run(w2, 120);
        CHECK(w2.isAwake(m));
    }
}

// -----------------------------------------------------------------------------
// 4) O que acorda por causa dos OUTROS
// -----------------------------------------------------------------------------
static void testWakeByOthers() {
    {   // Uma esfera que cai sobre a caixa dormindo a acorda.
        SleepyBox s;
        const BodyId ball = s.w.createBody(sphereDesc(Vector3(0, 3, 0), Real(0.25)));
        CHECK(!s.w.isAwake(s.box));
        bool wokeBeforeImpact = false, woke = false;
        for (int i = 0; i < 150 && !woke; ++i) {
            s.w.step();
            const Real ballBottom = s.w.getBody(ball).getWorldCenterOfMass().y - Real(0.25);
            if (s.w.isAwake(s.box)) {
                woke = true;
                wokeBeforeImpact = ballBottom > Real(1.1);   // acordou com a bola ainda longe?
            }
        }
        CHECK(woke && !wokeBeforeImpact);
    }
    {   // Destruir o apoio acorda quem estava em cima, e ele cai.
        PhysicsWorld w;
        w.createBody(groundDesc());
        const BodyId lower = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
        const BodyId upper = w.createBody(boxDesc(Vector3(0, Real(1.501), 0)));
        CHECK(runUntil(w, 900, [&] { return !w.isAwake(lower) && !w.isAwake(upper); }));
        const Real y0 = w.getBody(upper).getWorldCenterOfMass().y;
        w.destroyBody(lower);
        CHECK(w.isAwake(upper));
        run(w, 40);
        CHECK(w.getBody(upper).getWorldCenterOfMass().y < y0 - Real(0.5));   // desceu até o chão
    }
    {   // Um corpo NOVO (estático) nascendo sobre um corpo adormecido o acorda.
        SleepyBox s;
        RigidBodyDesc wall;
        wall.type = BodyType::Static;
        wall.shape = std::make_shared<BoxShape>(Vector3(Real(0.2), 1, 1));
        wall.transform = Transform(Vector3(Real(0.6), 1, 0));
        s.w.createBody(wall);
        CHECK(s.w.isAwake(s.box));
    }
    {   // Plataforma cinemática: a caixa dorme sobre ela PARADA; ao começar a andar, leva a caixa.
        PhysicsWorld w;
        RigidBodyDesc k;
        k.type = BodyType::Kinematic;
        k.shape = std::make_shared<BoxShape>(Vector3(20, Real(0.5), 5));
        k.transform = Transform(Vector3(0, Real(-0.5), 0));
        const BodyId plat = w.createBody(k);
        const BodyId box = w.createBody(boxDesc(Vector3(0, Real(0.52), 0)));
        CHECK(runUntil(w, 600, [&] { return !w.isAwake(box); }));   // dorme em cima de um cinemático parado
        const Real x0 = w.getBody(box).getWorldCenterOfMass().x;

        w.getBody(plat).setLinearVelocity(Vector3(1, 0, 0));
        w.step();
        CHECK(w.isAwake(box));
        run(w, 90);
        CHECK(w.getBody(box).getWorldCenterOfMass().x > x0 + Real(0.5));
        std::printf("  [info] plataforma a 1 m/s: caixa andou %.2f m em 1,5 s\n",
                    double(w.getBody(box).getWorldCenterOfMass().x - x0));

        // Plataforma "movida" por teleporte (velocidade 0) também acorda quem está em cima.
        w.getBody(plat).setLinearVelocity(Vector3::zero());
        CHECK(runUntil(w, 900, [&] { return !w.isAwake(box); }));
        w.setBodyTransform(plat, Transform(Vector3(0, Real(-0.5), 0)));
        CHECK(w.isAwake(box));
    }
}

// -----------------------------------------------------------------------------
// 5) Opção desligada e sono manual
// -----------------------------------------------------------------------------
static void testDisabledAndManual() {
    {
        PhysicsWorld w;
        w.sleepSettings().enabled = false;
        w.createBody(groundDesc());
        const BodyId b = w.createBody(boxDesc(Vector3(0, Real(0.52), 0)));
        run(w, 480);
        CHECK(w.isAwake(b) && w.getStats().sleepingBodies == 0);
    }
    {   // Limiar de tempo configurável: com 0,1 s dorme bem mais cedo.
        PhysicsWorld fast, slow;
        fast.sleepSettings().timeToSleep = Real(0.1);
        for (PhysicsWorld* w : {&fast, &slow}) w->createBody(groundDesc());
        const BodyId f = fast.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
        const BodyId s = slow.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
        int stepsFast = 0, stepsSlow = 0;
        while (stepsFast < 600 && fast.isAwake(f)) { fast.step(); ++stepsFast; }
        while (stepsSlow < 600 && slow.isAwake(s)) { slow.step(); ++stepsSlow; }
        CHECK(stepsFast < 600 && stepsSlow < 600 && stepsFast + 20 < stepsSlow);
    }
    {   // Sono manual: zera a velocidade, congela o corpo no lugar, e acordar devolve a gravidade.
        PhysicsWorld w;
        RigidBodyDesc d = boxDesc(Vector3(0, 10, 0));
        d.linearVelocity = Vector3(1, 2, 3);
        const BodyId b = w.createBody(d);
        w.setAwake(b, false);
        CHECK(!w.isAwake(b) && w.getBody(b).getLinearVelocity() == Vector3::zero());
        const Vector3 p = w.getBody(b).getWorldCenterOfMass();
        run(w, 60);
        CHECK(sameBits(w.getBody(b).getWorldCenterOfMass(), p));
        w.setAwake(b, true);
        run(w, 30);
        CHECK(w.getBody(b).getWorldCenterOfMass().y < p.y - Real(0.5));
        w.setAwake(9999, false);   // id inválido: ignora
    }
    {   // wakeAll
        SleepyBox s;
        s.w.wakeAll();
        CHECK(s.w.isAwake(s.box));
    }
}

// -----------------------------------------------------------------------------
// 6) Juntas
// -----------------------------------------------------------------------------
static void testJoints() {
    PhysicsWorld w;
    RigidBodyDesc anchor;
    anchor.type = BodyType::Static;
    anchor.transform = Transform(Vector3(0, 5, 0));
    const BodyId a = w.createBody(anchor);
    const BodyId b = w.createBody(sphereDesc(Vector3(0, 3, 0), Real(0.25)));
    DistanceJoint& j = w.createDistanceJoint(a, b, Vector3(0, 5, 0), Vector3(0, 3, 0));

    CHECK(runUntil(w, 900, [&] { return !w.isAwake(b); }));   // pendurado em repouso: dorme
    run(w, 2);
    CHECK(w.getStats().solverJoints == 0 && w.getStats().sleepingBodies == 1);

    w.getBody(b).applyLinearImpulse(Vector3(3, 0, 0));        // empurrão: acorda e balança
    run(w, 5);
    CHECK(w.isAwake(b) && w.getStats().solverJoints == 1);
    Real maxX = 0;
    for (int i = 0; i < 60; ++i) {
        w.step();
        maxX = std::max(maxX, w.getBody(b).getWorldCenterOfMass().x);
    }
    CHECK(maxX > Real(0.5));
    CHECK(w.isAwake(b));   // balançando: não dorme

    // Duas esferas ligadas por junta formam UMA ilha: a que não pode dormir segura a outra.
    PhysicsWorld w2;
    RigidBodyDesc s1 = sphereDesc(Vector3(0, 5, 0), Real(0.25));
    RigidBodyDesc s2 = sphereDesc(Vector3(2, 5, 0), Real(0.25));
    s2.allowSleep = false;
    w2.setGravity(Vector3::zero());
    const BodyId p = w2.createBody(s1);
    const BodyId q = w2.createBody(s2);
    w2.createDistanceJoint(p, q, Vector3(0, 5, 0), Vector3(2, 5, 0));
    run(w2, 120);
    CHECK(w2.isAwake(p) && w2.isAwake(q));
    (void)j;

    // Criar/destruir junta acorda os dois corpos.
    SleepyBox s;
    const BodyId other = s.w.createBody(boxDesc(Vector3(5, Real(0.52), 0)));
    CHECK(runUntil(s.w, 900, [&] { return !s.w.isAwake(other); }));
    CHECK(!s.w.isAwake(s.box));
    s.w.createBallSocketJoint(s.box, other, Vector3(2.5, 0.5, 0));
    CHECK(s.w.isAwake(s.box) && s.w.isAwake(other));
}

// -----------------------------------------------------------------------------
// 7) Dormir não pode mudar a física nem o determinismo
// -----------------------------------------------------------------------------
struct RandomScene {
    PhysicsWorld w;
    std::vector<BodyId> ids;
    explicit RandomScene(bool sleeping, int boxes = 40) {
        w.sleepSettings().enabled = sleeping;
        w.createBody(groundDesc());
        Rng r(1234);
        for (int i = 0; i < boxes; ++i) {
            const Real x = (r.next() - Real(0.5)) * Real(8);
            const Real z = (r.next() - Real(0.5)) * Real(8);
            const Real y = Real(0.5) + Real(i) * Real(0.7);
            ids.push_back(w.createBody(i % 3 == 0 ? sphereDesc(Vector3(x, y, z), Real(0.4)) : boxDesc(Vector3(x, y, z), Real(0.4))));
        }
    }
};

static bool sameWorldState(RandomScene& a, RandomScene& b) {
    for (std::size_t i = 0; i < a.ids.size(); ++i) {
        const RigidBody& x = a.w.getBody(a.ids[i]);
        const RigidBody& y = b.w.getBody(b.ids[i]);
        if (!(x.getWorldCenterOfMass() == y.getWorldCenterOfMass())) return false;
        if (!(x.getLinearVelocity() == y.getLinearVelocity())) return false;
        if (!(x.getAngularVelocity() == y.getAngularVelocity())) return false;
        if (x.getOrientation().x != y.getOrientation().x || x.getOrientation().w != y.getOrientation().w) return false;
        if (x.isAwake() != y.isAwake() || x.getSleepTime() != y.getSleepTime()) return false;
    }
    return true;
}

static void testDeterminismAndEquivalence() {
    {   // Determinismo: dois mundos iguais, com sleeping, são idênticos bit a bit.
        RandomScene a(true), b(true);
        for (int i = 0; i < 900; ++i) {
            a.w.step();
            b.w.step();
        }
        CHECK(sameWorldState(a, b));
        const WorldStats st = a.w.getStats();
        std::printf("  [info] 40 corpos soltos, 15 s: %d adormecidos, %d acordados, %d pares congelados de %d\n",
                    st.sleepingBodies, st.awakeBodies, st.frozenPairs, st.pairs);
        CHECK(st.sleepingBodies > 0);
    }
    {   // Equivalência: com e sem sleeping as poses finais coincidem dentro de uma tolerância pequena.
        RandomScene on(true, 30), off(false, 30);
        for (int i = 0; i < 720; ++i) {
            on.w.step();
            off.w.step();
        }
        int asleep = 0;
        for (std::size_t i = 0; i < on.ids.size(); ++i) asleep += on.w.isAwake(on.ids[i]) ? 0 : 1;
        // Uma chuva de 30 corpos soltos é um sistema CAÓTICO: dormir congela cada corpo onde ele estava
        // (descartando velocidades abaixo do limiar), e esse desvio minúsculo basta para o caminho de
        // OUTROS corpos divergir. Medido: em float todos ficam a < 3 cm; em double um único corpo
        // terminou a 1,8 m (os dois mundos são válidos: ambos em repouso no chão, sem penetração
        // extra). Por isso a comparação corpo a corpo NÃO é um critério robusto aqui; o que se exige
        // é que a MAIORIA dos corpos coincida (mediana pequena, quase todos a < 15 cm). A prova de
        // correção do sleeping está nas invariantes (testIslandInvariant) e na equivalência de uma
        // cena não caótica (testEquivalenceNonChaotic).
        std::vector<Real> diffs;
        for (std::size_t i = 0; i < on.ids.size(); ++i) {
            diffs.push_back((on.w.getBody(on.ids[i]).getWorldCenterOfMass() -
                             off.w.getBody(off.ids[i]).getWorldCenterOfMass()).length());
        }
        std::sort(diffs.begin(), diffs.end());
        const Real median = diffs[diffs.size() / 2];
        int near = 0;
        for (Real d : diffs) near += d < Real(0.15) ? 1 : 0;
        std::printf("  [info] caos on/off: mediana = %.4f m, %d de %zu a < 15 cm\n", double(median), near, diffs.size());
        CHECK(median < Real(0.02));
        CHECK(near * 10 >= static_cast<int>(diffs.size()) * 9);
        CHECK(asleep > 0);
    }
}

// -----------------------------------------------------------------------------
// 7b) Invariante de ilhas e equivalência em cena NÃO caótica
// -----------------------------------------------------------------------------
/// Chuva aleatória com semente/tamanho variáveis (cada mundo é uma cena diferente).
struct InvariantScene {
    PhysicsWorld w;
    std::vector<BodyId> ids;
    InvariantScene(unsigned seed, int boxes) {
        w.createBody(groundDesc());
        Rng r(seed);
        for (int i = 0; i < boxes; ++i) {
            const Real x = (r.next() - Real(0.5)) * Real(6);
            const Real z = (r.next() - Real(0.5)) * Real(6);
            const Real y = Real(0.5) + Real(i) * Real(0.6);
            ids.push_back(w.createBody(i % 3 == 0 ? sphereDesc(Vector3(x, y, z), Real(0.4))
                                                  : boxDesc(Vector3(x, y, z), Real(0.4))));
        }
    }
};

static void testIslandInvariant() {
    // Invariante: ao fim de TODO passo, dois corpos dinâmicos em contato estão ambos acordados ou
    // ambos dormindo. Um par misto significaria um corpo adormecido sendo atingido sem acordar
    // (falha de propagação). Checada em 12 mundos × 600 passos, com um impulso no meio da
    // simulação (nos pares) para acordar ilhas já adormecidas.
    long pairs = 0, violations = 0, maxAsleep = 0;
    for (unsigned seed = 1; seed <= 12; ++seed) {
        InvariantScene sc(seed * 977u, 20 + static_cast<int>(seed % 4) * 10);
        for (int k = 0; k < 600; ++k) {
            if (k == 300 && seed % 2 == 0) {
                sc.w.getBody(sc.ids[seed % sc.ids.size()]).applyLinearImpulse(Vector3(0, 40, 0));
            }
            sc.w.step();
            long asleep = 0;
            for (BodyId id : sc.ids) asleep += sc.w.isAwake(id) ? 0 : 1;
            maxAsleep = std::max(maxAsleep, asleep);
            for (const ContactPair* cp : sc.w.getContacts()) {
                const RigidBody& a = sc.w.getBody(cp->a);
                const RigidBody& b = sc.w.getBody(cp->b);
                if (!a.isDynamic() || !b.isDynamic()) continue;
                ++pairs;
                if (a.isAwake() != b.isAwake()) ++violations;
            }
        }
    }
    std::printf("  [info] invariante de ilhas: %ld pares dinamicos checados, %ld violacoes, ate %ld corpos dormindo juntos\n",
                pairs, violations, maxAsleep);
    CHECK(pairs > 10000);
    CHECK(violations == 0);
    CHECK(maxAsleep > 0);
}

static void testEquivalenceNonChaotic() {
    // Cena NÃO caótica: pilhas de 3 caixas, bem separadas, soltas de pouca altura. Com e sem
    // sleeping, as poses finais devem coincidir quase exatamente (aqui sim vale comparar corpo a corpo).
    auto build = [](PhysicsWorld& w, std::vector<BodyId>& ids) {
        w.createBody(groundDesc());
        for (int s = 0; s < 4; ++s) {
            for (int k = 0; k < 3; ++k) {
                ids.push_back(w.createBody(boxDesc(Vector3(Real(s) * 4, Real(0.52) + Real(k) * Real(1.02), 0))));
            }
        }
    };
    PhysicsWorld on, off;
    off.sleepSettings().enabled = false;
    std::vector<BodyId> idsOn, idsOff;
    build(on, idsOn);
    build(off, idsOff);
    for (int i = 0; i < 600; ++i) {
        on.step();
        off.step();
    }
    Real maxDiff = 0;
    int asleep = 0;
    for (std::size_t i = 0; i < idsOn.size(); ++i) {
        maxDiff = std::max(maxDiff, (on.getBody(idsOn[i]).getWorldCenterOfMass() -
                                     off.getBody(idsOff[i]).getWorldCenterOfMass()).length());
        asleep += on.isAwake(idsOn[i]) ? 0 : 1;
    }
    std::printf("  [info] equivalencia (4 pilhas de 3): diferenca maxima = %.5f m, %d de %zu dormindo\n",
                double(maxDiff), asleep, idsOn.size());
    CHECK(maxDiff < Real(0.02));
    CHECK(asleep > 0);
}

// -----------------------------------------------------------------------------
// 8) Interpolação, bridge e consultas
// -----------------------------------------------------------------------------
struct CountingBridge final : IPhysicsTransformBridge {
    std::vector<int> updates;
    void onBodyCreated(BodyId id, const Transform&) override {
        if (id >= updates.size()) updates.resize(id + 1, 0);
    }
    void onTransformUpdated(BodyId id, const Transform&) override {
        if (id >= updates.size()) updates.resize(id + 1, 0);
        ++updates[id];
    }
};

static void testRenderQueriesAndBridge() {
    PhysicsWorld w;
    CountingBridge br;
    w.setBridge(&br);
    w.createBody(groundDesc());
    const BodyId b = w.createBody(boxDesc(Vector3(0, Real(0.52), 0)));
    for (int i = 0; i < 600 && w.isAwake(b); ++i) w.update(kDt);
    CHECK(!w.isAwake(b));

    // Dormindo, a bridge deixa de receber atualizações do corpo (e a pose de render não "anda").
    w.update(kDt);                                    // pode enviar a pose final uma vez
    const int sent = br.updates[b];
    for (int i = 0; i < 60; ++i) w.update(kDt * Real(0.37));
    CHECK(br.updates[b] == sent);
    const Transform cur = w.getBody(b).getTransform();
    for (Real alpha : {Real(0), Real(0.3), Real(0.99)}) {
        (void)alpha;
        const Transform t = w.getInterpolatedTransform(b);
        CHECK(t.position == cur.position);
    }

    // Acordar volta a enviar.
    w.getBody(b).applyLinearImpulse(Vector3(0, 3, 0));
    w.update(kDt);
    w.update(kDt);
    CHECK(br.updates[b] > sent);

    // Consultas enxergam corpos dormindo.
    SleepyBox s;
    RaycastHit hit;
    CHECK(s.w.raycast(Vector3(0, 5, 0), Vector3(0, -1, 0), 100, hit));
    CHECK(hit.body == s.box && std::abs(hit.distance - (Real(5) - Real(1.0))) < Real(0.03));
    std::vector<BodyId> bodies;
    CHECK(s.w.overlapPoint(Vector3(0, Real(0.5), 0), bodies) == 1 && bodies[0] == s.box);
    CHECK(s.w.overlapSphere(Vector3(0, Real(0.5), 0), Real(0.2), bodies) == 1);
    CHECK(!s.w.isAwake(s.box));   // consultar é const: não acorda ninguém
}

// -----------------------------------------------------------------------------
// 9) Escala e perfilamento
// -----------------------------------------------------------------------------
static void testScaleAndProfiling() {
    PhysicsWorldSettings cfg;
    cfg.enableProfiling = true;
    PhysicsWorld w(cfg);
    w.createBody(groundDesc());
    const int n = 14;
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < n; ++k) {
            w.createBody(boxDesc(Vector3(Real(i) * Real(1.5) - Real(10), Real(0.52), Real(k) * Real(1.5) - Real(10))));
        }
    }
    w.step();
    const WorldStats first = w.getStats();
    CHECK(first.timings.totalUs > 0);
    const double sum = first.timings.collisionUs + first.timings.islandsUs + first.timings.constraintsUs +
                       first.timings.integrationUs + first.timings.sleepEventsUs;
    CHECK(std::abs(sum - first.timings.totalUs) <= 1e-6 * first.timings.totalUs + 1e-3);

    double awakeUs = 0;
    for (int i = 0; i < 60; ++i) {
        w.step();
        awakeUs += w.getStats().timings.totalUs;
    }
    awakeUs /= 60.0;
    CHECK(runUntil(w, 1200, [&] { return w.getStats().awakeBodies == 0; }));
    double sleepUs = 0;
    for (int i = 0; i < 60; ++i) {
        w.step();
        sleepUs += w.getStats().timings.totalUs;
    }
    sleepUs /= 60.0;
    const WorldStats st = w.getStats();
    std::printf("  [info] %d caixas: %.0f us/passo acordadas -> %.0f us/passo dormindo (solver: %d contatos; pares congelados: %d/%d)\n",
                n * n, awakeUs, sleepUs, st.solverContacts, st.frozenPairs, st.pairs);
    CHECK(st.sleepingBodies == n * n && st.solverContacts == 0 && st.frozenPairs == st.pairs);
    CHECK(sleepUs < awakeUs);   // dormir tem que sair mais barato

    // Perfilamento desligado: tudo zero.
    PhysicsWorld off;
    off.createBody(groundDesc());
    off.createBody(boxDesc(Vector3(0, 1, 0)));
    run(off, 5);
    const StepTimings t = off.getStats().timings;
    CHECK(t.totalUs == 0 && t.collisionUs == 0 && t.constraintsUs == 0);
}

int main() {
    testFallsAsleep();
    testWakeTriggers();
    testIslands();
    testWakeByOthers();
    testDisabledAndManual();
    testJoints();
    testDeterminismAndEquivalence();
    testIslandInvariant();
    testEquivalenceNonChaotic();
    testRenderQueriesAndBridge();
    testScaleAndProfiling();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
