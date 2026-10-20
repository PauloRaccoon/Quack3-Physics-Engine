// =============================================================================
// events_tests.cpp — eventos de contato (Fase 9).
//
// Contrato verificado:
//   - Begin na chegada, com velocidade de impacto, ponto, normal, impulso e profundidade corretos;
//   - End ao separar; Begin/End se alternam num quique; nada é emitido para contatos que persistem
//     (pilha em repouso, corpos adormecidos);
//   - ouvinte e buffer entregam exatamente os mesmos eventos, na mesma ordem (determinística);
//   - destruir um corpo emite End de cada contato dele; o ouvinte pode criar/destruir corpos
//     dentro do callback; chamar step()/update() de dentro do callback lança std::logic_error
//     e deixa o mundo utilizável.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

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

static RigidBodyDesc groundDesc() {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(Vector3(50, 1, 50));
    d.transform = Transform(Vector3(0, -1, 0));
    return d;
}
static RigidBodyDesc boxDesc(const Vector3& pos, Real half = Real(0.5)) {
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(half, half, half));
    d.mass = 1;
    d.transform = Transform(pos);
    d.angularDamping = 0;
    return d;
}
static RigidBodyDesc sphereDesc(const Vector3& pos, Real r = Real(0.5), Real restitution = Real(0)) {
    RigidBodyDesc d;
    d.shape = std::make_shared<SphereShape>(r);
    d.mass = 1;
    d.transform = Transform(pos);
    d.restitution = restitution;
    d.angularDamping = 0;
    return d;
}

template <typename E, typename F>
static bool throws(F&& f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

/// Grava tudo o que recebe, com o passo em que chegou.
struct Recorder : IContactListener {
    std::vector<ContactEvent> events;
    void onContactBegin(const ContactEvent& e) override { events.push_back(e); }
    void onContactEnd(const ContactEvent& e) override { events.push_back(e); }

    int count(ContactEvent::Type t) const {
        int n = 0;
        for (const ContactEvent& e : events) n += e.type == t ? 1 : 0;
        return n;
    }
};

static bool sameEvent(const ContactEvent& x, const ContactEvent& y) {
    return std::memcmp(&x, &y, sizeof(ContactEvent)) == 0 ||
           (x.type == y.type && x.a == y.a && x.b == y.b && x.point == y.point && x.normal == y.normal &&
            x.depth == y.depth && x.pointCount == y.pointCount && x.normalImpulse == y.normalImpulse &&
            x.approachSpeed == y.approachSpeed);
}

// -----------------------------------------------------------------------------
// 1) Início do contato
// -----------------------------------------------------------------------------
static void testBegin() {
    PhysicsWorld w;
    Recorder rec;
    w.setContactListener(&rec);
    const BodyId ground = w.createBody(groundDesc());
    const BodyId ball = w.createBody(sphereDesc(Vector3(0, 3, 0)));   // base em y = 2,5: cai 2,5 m
    CHECK(ground < ball);

    int step = 0;
    while (step < 120 && rec.events.empty()) {
        w.step();
        ++step;
    }
    CHECK(rec.events.size() == 1);
    if (rec.events.empty()) return;
    const ContactEvent& e = rec.events[0];
    const Real expectedSpeed = std::sqrt(Real(2) * kGravity * Real(2.5));   // 7,0 m/s
    std::printf("  [info] impacto: v=%.2f m/s (analitico %.2f), impulso=%.2f N.s (m*v=%.2f), ponto=(%.3f,%.3f,%.3f), profundidade=%.4f\n",
                double(e.approachSpeed), double(expectedSpeed), double(e.normalImpulse), double(expectedSpeed),
                double(e.point.x), double(e.point.y), double(e.point.z), double(e.depth));

    CHECK(e.type == ContactEvent::Type::Begin);
    CHECK(e.a == ground && e.b == ball);                           // a < b
    CHECK(e.normal.y > Real(0.99));                                // de `a` (chão) para `b` (bola): para cima
    CHECK(std::abs(e.approachSpeed - expectedSpeed) < Real(0.35));
    CHECK(e.normalImpulse > Real(0.5) * expectedSpeed);            // parou a bola: impulso ~ m·v
    CHECK(e.normalImpulse < Real(1.5) * expectedSpeed);
    // Sem CCD o contato só é detectado no passo seguinte à chegada, então já nasce com penetração: a
    // 7 m/s a bola anda 7/60 = 11,7 cm por passo, e a profundidade do Begin fica entre 0 e v·Δt.
    // O ponto do evento é a MÉDIA dos pontos de contato (um em cada superfície): y = -profundidade/2.
    const Real maxDepth = expectedSpeed * kDt + kContactMargin;
    CHECK(e.depth > -kContactMargin - Real(1e-4) && e.depth < maxDepth);
    CHECK(std::abs(e.point.y + e.depth * Real(0.5)) < Real(0.005));
    CHECK(std::abs(e.point.x) < Real(0.01) && std::abs(e.point.z) < Real(0.01));
    CHECK(e.pointCount == 1);

    // Sem restituição a bola fica: nenhum End, nenhum Begin novo.
    for (int i = 0; i < 300; ++i) w.step();
    CHECK(rec.events.size() == 1);
}

// -----------------------------------------------------------------------------
// 2) Quique: Begin/End se alternam; em repouso, silêncio
// -----------------------------------------------------------------------------
static void testBounce() {
    PhysicsWorld w;
    w.sleepSettings().enabled = false;   // aqui o silêncio vem dos contatos persistentes, não do sono
    Recorder rec;
    w.setContactListener(&rec);
    w.createBody(groundDesc());
    w.createBody(sphereDesc(Vector3(0, 3, 0), Real(0.5), Real(0.8)));

    std::size_t lastCount = 0;
    int lastEventStep = 0;
    for (int i = 1; i <= 1200; ++i) {   // 20 s
        w.step();
        if (rec.events.size() != lastCount) {
            lastCount = rec.events.size();
            lastEventStep = i;
        }
    }
    CHECK(rec.count(ContactEvent::Type::Begin) >= 4);
    bool alternates = true;
    for (std::size_t i = 0; i < rec.events.size(); ++i) {
        const ContactEvent::Type expect = (i % 2 == 0) ? ContactEvent::Type::Begin : ContactEvent::Type::End;
        alternates = alternates && rec.events[i].type == expect;
    }
    CHECK(alternates);

    // A velocidade de impacto cai a cada quique (e = 0,8: ~0,8x por batida).
    bool decreasing = true;
    Real prev = std::numeric_limits<Real>::max();
    std::vector<Real> speeds;
    for (const ContactEvent& e : rec.events) {
        if (e.type != ContactEvent::Type::Begin) continue;
        speeds.push_back(e.approachSpeed);
        decreasing = decreasing && e.approachSpeed < prev;
        prev = e.approachSpeed;
    }
    CHECK(decreasing);
    CHECK(speeds.size() >= 2 && speeds[1] > Real(0.65) * speeds[0] && speeds[1] < Real(0.95) * speeds[0]);

    // Depois de assentar, silêncio total (sem tremida de Begin/End no repouso).
    std::printf("  [info] quique e=0.8: %d Begins; o ultimo evento foi no passo %d de 1200\n",
                rec.count(ContactEvent::Type::Begin), lastEventStep);
    CHECK(lastEventStep < 1000);
    // Termina em contato: a bola está em repouso sobre o chão.
    CHECK(rec.events.back().type == ContactEvent::Type::Begin);
}

static void testRestIsSilent() {
    for (int sleeping = 0; sleeping < 2; ++sleeping) {
        PhysicsWorld w;
        w.sleepSettings().enabled = sleeping != 0;
        Recorder rec;
        w.createBody(groundDesc());
        for (int i = 0; i < 4; ++i) w.createBody(boxDesc(Vector3(0, Real(0.5) + Real(i) * Real(1.001), 0)));
        w.setContactListener(&rec);
        for (int i = 0; i < 360; ++i) w.step();   // assenta (6 s)
        rec.events.clear();
        for (int i = 0; i < 600; ++i) w.step();   // 10 s de repouso
        std::printf("  [info] pilha em repouso (sleeping %s): %zu eventos em 10 s\n", sleeping ? "ligado" : "desligado",
                    rec.events.size());
        CHECK(rec.events.empty());
        CHECK(w.getStats().touchingPairs == 4);
    }
}

// -----------------------------------------------------------------------------
// 3) Ouvinte x buffer, ordem e determinismo
// -----------------------------------------------------------------------------
static void testBufferAndListener() {
    PhysicsWorld w;
    Recorder rec;
    w.setContactListener(&rec);
    w.createBody(groundDesc());
    for (int i = 0; i < 6; ++i) w.createBody(sphereDesc(Vector3(Real(i) * Real(1.3), Real(1) + Real(i) * Real(0.4), 0), Real(0.4), Real(0.5)));

    std::size_t total = 0;
    bool buffersMatch = true, clearedEachStep = true;
    for (int i = 0; i < 300; ++i) {
        const std::size_t before = rec.events.size();
        w.step();
        const std::vector<ContactEvent>& buf = w.getContactEvents();
        const std::size_t got = rec.events.size() - before;
        buffersMatch = buffersMatch && buf.size() == got;
        for (std::size_t k = 0; k < got && k < buf.size(); ++k) {
            buffersMatch = buffersMatch && sameEvent(buf[k], rec.events[before + k]);
        }
        total += buf.size();
    }
    CHECK(buffersMatch && clearedEachStep);
    CHECK(total == rec.events.size() && total > 0);

    // update() com vários passos: o buffer acumula todos os passos daquela chamada.
    PhysicsWorld w2;
    Recorder rec2;
    w2.setContactListener(&rec2);
    w2.createBody(groundDesc());
    for (int i = 0; i < 6; ++i) w2.createBody(sphereDesc(Vector3(Real(i) * Real(1.3), Real(0.4), 0), Real(0.4)));   // apoiadas no chão
    const int steps = w2.update(kDt * Real(3.2));
    CHECK(steps == 3);
    CHECK(w2.getContactEvents().size() == rec2.events.size() && !rec2.events.empty());

    // Sem ouvinte, só o buffer funciona.
    PhysicsWorld w3;
    w3.createBody(groundDesc());
    w3.createBody(sphereDesc(Vector3(0, Real(0.5), 0)));   // apoiada no chão (raio 0,5)
    w3.step();
    CHECK(w3.getContactEvents().size() == 1 && w3.getContactEvents()[0].type == ContactEvent::Type::Begin);
    w3.step();
    CHECK(w3.getContactEvents().empty());   // esvaziado no início do passo seguinte

    // Determinismo: dois mundos iguais, mesma sequência de eventos (bit a bit).
    PhysicsWorld a, b;
    Recorder ra, rb;
    a.setContactListener(&ra);
    b.setContactListener(&rb);
    for (PhysicsWorld* wp : {&a, &b}) {
        wp->createBody(groundDesc());
        unsigned seed = 77;
        auto rnd = [&] {
            seed = seed * 1664525u + 1013904223u;
            return Real((seed >> 8) & 0xFFFFFF) / Real(16777216);
        };
        for (int i = 0; i < 50; ++i) {
            const Vector3 p((rnd() - Real(0.5)) * 6, Real(0.5) + Real(i) * Real(0.5), (rnd() - Real(0.5)) * 6);
            wp->createBody(i % 2 ? boxDesc(p, Real(0.3)) : sphereDesc(p, Real(0.3), Real(0.4)));
        }
    }
    for (int i = 0; i < 600; ++i) {
        a.step();
        b.step();
    }
    bool same = ra.events.size() == rb.events.size() && !ra.events.empty();
    for (std::size_t i = 0; same && i < ra.events.size(); ++i) same = sameEvent(ra.events[i], rb.events[i]);
    CHECK(same);
    std::printf("  [info] 50 corpos caindo, 10 s: %zu eventos, identicos nos dois mundos\n", ra.events.size());
}

// -----------------------------------------------------------------------------
// 4) Destruição, callbacks que mexem no mundo, reentrância, juntas
// -----------------------------------------------------------------------------
static void testDestroyEmitsEnd() {
    PhysicsWorld w;
    Recorder rec;
    const BodyId g = w.createBody(groundDesc());
    const BodyId b1 = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
    const BodyId b2 = w.createBody(boxDesc(Vector3(0, Real(1.501), 0)));
    w.sleepSettings().enabled = false;
    w.setContactListener(&rec);
    for (int i = 0; i < 120; ++i) w.step();
    rec.events.clear();

    w.destroyBody(b1);   // fora de um passo: os Ends chegam já
    CHECK(rec.events.size() == 2);
    int endsWithB1 = 0;
    for (const ContactEvent& e : rec.events) {
        endsWithB1 += (e.type == ContactEvent::Type::End && (e.a == b1 || e.b == b1)) ? 1 : 0;
        CHECK(e.normal.isFinite() && e.point.isFinite());   // dados são cópias: valem com o corpo morto
    }
    CHECK(endsWithB1 == 2);
    rec.events.clear();

    // Os Ends não se repetem no passo seguinte; b2 cai e toca o chão (Begin novo).
    for (int i = 0; i < 60; ++i) w.step();
    CHECK(rec.count(ContactEvent::Type::End) == 0);
    CHECK(rec.count(ContactEvent::Type::Begin) == 1 && rec.events[0].a == g && rec.events[0].b == b2);
}

struct Mutator : IContactListener {
    PhysicsWorld* world = nullptr;
    BodyId toDestroy = kInvalidBodyId;
    int beginCount = 0, endCount = 0;
    BodyId created = kInvalidBodyId;
    void onContactBegin(const ContactEvent& e) override {
        ++beginCount;
        if (toDestroy != kInvalidBodyId) {
            world->destroyBody(toDestroy);   // destrói o corpo que acabou de bater, dentro do callback
            toDestroy = kInvalidBodyId;
        }
        if (created == kInvalidBodyId) created = world->createBody(boxDesc(e.point + Vector3(3, 2, 0)));
    }
    void onContactEnd(const ContactEvent&) override { ++endCount; }
};

static void testCallbackMutation() {
    PhysicsWorld w;
    Mutator m;
    m.world = &w;
    w.setContactListener(&m);
    w.createBody(groundDesc());
    const BodyId ball = w.createBody(sphereDesc(Vector3(0, 1, 0)));
    m.toDestroy = ball;
    for (int i = 0; i < 120; ++i) w.step();
    CHECK(m.beginCount >= 1);
    CHECK(m.endCount == 1);                      // o corpo destruído no callback gerou seu End
    // O BodyId da esfera destruída é REAPROVEITADO pelo corpo criado no mesmo callback (documentado):
    // o que prova a destruição é o id agora apontar para a CAIXA, não para a esfera.
    CHECK(m.created != kInvalidBodyId && w.isValidBody(m.created));
    CHECK(w.getBody(ball).getShape() && w.getBody(ball).getShape()->getType() == ShapeType::Box);
    for (int i = 0; i < 120; ++i) w.step();      // o mundo segue saudável (e com ASan, sem acessos inválidos)
    CHECK(w.getBody(m.created).getWorldCenterOfMass().isFinite());
}

struct Reentrant : IContactListener {
    PhysicsWorld* world = nullptr;
    int calls = 0;
    void onContactBegin(const ContactEvent&) override {
        ++calls;
        world->step();   // proibido: reentrância
    }
};

static void testReentrancy() {
    PhysicsWorld w;
    Reentrant r;
    r.world = &w;
    w.setContactListener(&r);
    w.createBody(groundDesc());
    w.createBody(sphereDesc(Vector3(0, Real(0.45), 0)));
    CHECK(throws<std::logic_error>([&] { w.step(); }));
    CHECK(r.calls == 1);

    // O mundo continua utilizável: sem ouvinte problemático, passos normais funcionam.
    w.setContactListener(nullptr);
    const std::uint64_t steps = w.getStats().stepCount;
    CHECK(!throws<std::logic_error>([&] { w.step(); w.update(kDt); }));
    CHECK(w.getStats().stepCount > steps);
    CHECK(w.getBody(1).getWorldCenterOfMass().isFinite());
}

static void testJointRemovesContact() {
    PhysicsWorld w;
    w.sleepSettings().enabled = false;
    Recorder rec;
    w.createBody(groundDesc());
    const BodyId a = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
    const BodyId b = w.createBody(boxDesc(Vector3(0, Real(1.501), 0)));
    w.setContactListener(&rec);
    for (int i = 0; i < 90; ++i) w.step();
    rec.events.clear();

    w.createBallSocketJoint(a, b, Vector3(0, 1, 0));   // collideConnected = false: o par (a,b) deixa de colidir
    w.step();
    CHECK(rec.events.size() == 1);
    CHECK(rec.events[0].type == ContactEvent::Type::End && rec.events[0].a == a && rec.events[0].b == b);
}

// -----------------------------------------------------------------------------
// 5) Dormir não emite eventos; acordar e pousar de novo emite o par End/Begin
// -----------------------------------------------------------------------------
static void testSleepAndEvents() {
    PhysicsWorld w;
    Recorder rec;
    w.setContactListener(&rec);
    w.createBody(groundDesc());
    const BodyId b = w.createBody(boxDesc(Vector3(0, Real(1.0), 0)));
    for (int i = 0; i < 600 && w.isAwake(b); ++i) w.step();
    CHECK(!w.isAwake(b));
    CHECK(rec.count(ContactEvent::Type::Begin) == 1 && rec.count(ContactEvent::Type::End) == 0);

    rec.events.clear();
    for (int i = 0; i < 120; ++i) w.step();
    CHECK(rec.events.empty());   // dormindo: contato persiste, sem eventos

    w.getBody(b).applyLinearImpulse(Vector3(0, 4, 0));   // acorda e salta
    for (int i = 0; i < 120; ++i) w.step();
    CHECK(rec.count(ContactEvent::Type::End) == 1 && rec.count(ContactEvent::Type::Begin) == 1);
    CHECK(rec.events.front().type == ContactEvent::Type::End && rec.events.back().type == ContactEvent::Type::Begin);
}

int main() {
    testBegin();
    testBounce();
    testRestIsSilent();
    testBufferAndListener();
    testDestroyEmitsEnd();
    testCallbackMutation();
    testReentrancy();
    testJointRemovesContact();
    testSleepAndEvents();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
