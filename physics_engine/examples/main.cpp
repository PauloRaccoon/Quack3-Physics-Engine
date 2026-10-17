// =============================================================================
// examples/main.cpp
// Demonstração (sem janela, sem dependências gráficas) das duas coisas que a
// Fase 7 entrega:
//
//   A) EMPILHAMENTO: uma torre de 10 caixas sobre o chão, simulada por 10 s
//      enquanto um "renderizador" simulado roda a ~144 FPS com tempo de quadro
//      IRREGULAR (jitter) e uma trava de 0,5 s no meio. A física continua em
//      passo fixo de 1/60 s; o acumulador decide quantos passos rodar por quadro.
//
//   B) INTERPOLAÇÃO: uma caixa em queda livre com render a 144 Hz sobre física a
//      60 Hz. Compara a pose "crua" (último passo físico) com a pose entregue à
//      bridge (lerp/slerp entre os dois últimos passos, com α = acumulador/Δt).
//
//   C) DETERMINISMO: a mesma torre simulada duas vezes, com o MESMO histórico
//      de tempos de quadro, termina bit a bit idêntica.
//
// O programa se auto-verifica e retorna 0 só se tudo passou (serve como teste).
// Em um jogo real o laço de render chamaria  world.update(dt)  e desenharia
// os corpos a partir da BufferedTransformBridge (vetor plano, ver README da bridge).
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "physics/bridge/IPhysicsTransformBridge.h"
#include "physics/geometry/BoxShape.h"
#include "physics/pipeline/PhysicsWorld.h"

using namespace physics;

namespace {

int g_failed = 0;

void expect(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "ok" : "FALHOU", what);
    if (!ok) ++g_failed;
}

/// Gerador pseudo-aleatório determinístico (LCG): o "jitter" do render é reprodutível.
struct Rng {
    std::uint32_t s;
    explicit Rng(std::uint32_t seed) : s(seed) {}
    double uniform() {   // [0,1)
        s = s * 1664525u + 1013904223u;
        return static_cast<double>((s >> 8) & 0xFFFFFF) / 16777216.0;
    }
};

// -----------------------------------------------------------------------------
// Cena
// -----------------------------------------------------------------------------
RigidBodyDesc makeGround() {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(Vector3(20, 0.5f, 20));
    d.transform = Transform(Vector3(0, -0.5f, 0));   // face superior em y = 0
    d.friction = Real(0.6);
    return d;
}

RigidBodyDesc makeBox(const Vector3& center, const Vector3& half) {
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(half);
    d.density = Real(1);
    d.transform = Transform(center);
    d.friction = Real(0.6);
    d.restitution = Real(0);
    return d;
}

constexpr int kTowerBoxes = 10;

/// Posição inicial da caixa i da torre (pequenos desalinhamentos laterais, como numa torre real).
Vector3 towerStart(int i) {
    return Vector3(Real(0.006) * Real(std::sin(1.7 * i)), Real(0.5) + Real(i), Real(0.006) * Real(std::cos(2.3 * i)));
}

std::vector<BodyId> buildTower(PhysicsWorld& world) {
    world.createBody(makeGround());
    std::vector<BodyId> ids;
    for (int i = 0; i < kTowerBoxes; ++i) {
        ids.push_back(world.createBody(makeBox(towerStart(i), Vector3(0.5f, 0.5f, 0.5f))));
    }
    return ids;
}

/// Sequência de tempos de quadro do renderizador: ~144 FPS com jitter ±30%, e uma
/// trava de 0,5 s (janela arrastada, breakpoint...) em t ≈ 4 s.
struct FrameClock {
    Rng rng{12345};
    double t = 0;
    bool hitchDone = false;

    double next() {
        double dt = (1.0 / 144.0) * (0.7 + 0.6 * rng.uniform());
        if (!hitchDone && t >= 4.0) {
            dt = 0.5;
            hitchDone = true;
        }
        t += dt;
        return dt;
    }
};

struct StackReport {
    Real maxDrift = 0;       ///< maior afastamento horizontal de uma caixa da posição inicial
    Real topSink = 0;        ///< quanto o topo desceu em relação ao início
    Real lateMaxSpeed = 0;   ///< maior velocidade linear de uma caixa nos ÚLTIMOS 3 s (balanço residual)
    Real maxPenetration = 0; ///< maior penetração vista nos contatos
    int steps = 0;
    int frames = 0;
    Real droppedTime = 0;
    bool finite = true;
};

StackReport runStack(PhysicsWorld& world, const std::vector<BodyId>& ids, double seconds, bool verbose) {
    StackReport rep;
    FrameClock clock;
    int nextPrint = 1;
    while (clock.t < seconds) {
        const double dt = clock.next();
        rep.steps += world.update(static_cast<Real>(dt));
        ++rep.frames;

        for (const ContactPair* cp : world.getContacts()) {
            for (int k = 0; k < cp->manifold.count; ++k) {
                rep.maxPenetration = std::max(rep.maxPenetration, cp->manifold.points[k].depth);
            }
        }
        if (clock.t > seconds - 3.0) {   // janela final: mede o balanço residual da torre
            for (BodyId id : ids) rep.lateMaxSpeed = std::max(rep.lateMaxSpeed, world.getBody(id).getLinearVelocity().length());
        }
        if (verbose && clock.t >= nextPrint) {
            const WorldStats st = world.getStats();
            const RigidBody& top = world.getBody(ids.back());
            std::printf("  t=%4.1f s  passos=%4llu  topo y=%.4f m  pares=%2d  contatos=%2d  pontos=%2d\n",
                        clock.t, static_cast<unsigned long long>(st.stepCount),
                        double(top.getWorldCenterOfMass().y), st.pairs, st.touchingPairs, st.contactPoints);
            ++nextPrint;
        }
    }

    for (std::size_t i = 0; i < ids.size(); ++i) {
        const RigidBody& b = world.getBody(ids[i]);
        const Vector3 p0 = towerStart(static_cast<int>(i));
        const Vector3 p = b.getWorldCenterOfMass();
        rep.maxDrift = std::max(rep.maxDrift, std::sqrt((p.x - p0.x) * (p.x - p0.x) + (p.z - p0.z) * (p.z - p0.z)));
        rep.finite = rep.finite && b.getTransform().isFinite() && b.getLinearVelocity().isFinite();
    }
    rep.topSink = towerStart(static_cast<int>(ids.size()) - 1).y - world.getBody(ids.back()).getWorldCenterOfMass().y;
    rep.droppedTime = world.getAccumulator().getDroppedTime();
    return rep;
}

bool identicalBits(const RigidBody& a, const RigidBody& b) {
    const Transform ta = a.getTransform(), tb = b.getTransform();
    const Real va[7] = {ta.position.x, ta.position.y, ta.position.z, ta.rotation.w, ta.rotation.x, ta.rotation.y, ta.rotation.z};
    const Real vb[7] = {tb.position.x, tb.position.y, tb.position.z, tb.rotation.w, tb.rotation.x, tb.rotation.y, tb.rotation.z};
    return std::memcmp(va, vb, sizeof(va)) == 0;
}

// -----------------------------------------------------------------------------
// A) Empilhamento
// -----------------------------------------------------------------------------
void demoStacking() {
    std::printf("\n=== A) Torre de %d caixas, 10 s, render ~144 FPS com jitter e uma trava de 0,5 s ===\n", kTowerBoxes);
    PhysicsWorld world;
    BufferedTransformBridge bridge;
    world.setBridge(&bridge);
    const std::vector<BodyId> ids = buildTower(world);

    const StackReport r = runStack(world, ids, 10.0, true);
    std::printf("\n  quadros de render: %d   passos fisicos: %d   tempo descartado pela trava: %.3f s\n",
                r.frames, r.steps, double(r.droppedTime));
    std::printf("  deriva lateral max: %.4f m   afundamento do topo: %.4f m   penetracao max: %.4f m (slop %.3f)\n",
                double(r.maxDrift), double(r.topSink), double(r.maxPenetration), double(kLinearSlop));
    std::printf("  balanco residual (|v| max nos ultimos 3 s): %.3f m/s\n", double(r.lateMaxSpeed));

    expect(r.finite, "nenhum NaN/Inf no estado dos corpos");
    expect(r.maxDrift < Real(0.05), "torre nao deriva lateralmente (< 5 cm)");
    expect(r.topSink < Real(0.08), "topo da torre nao afunda (< 8 cm, ~ soma dos slops)");
    // Uma torre de 10 m sobre um solver iterativo balança suavemente e amortece (ver comparacao abaixo);
    // um tombamento teria velocidades de m/s.
    expect(r.lateMaxSpeed < Real(0.2), "balanco residual pequeno e limitado (|v| < 0,2 m/s): torre de pe");
    expect(r.maxPenetration < Real(2) * kLinearSlop + Real(0.02), "penetracao permanece pequena");
    expect(r.droppedTime > Real(0.3), "a trava de 0,5 s foi limitada (anti 'espiral da morte')");

    // A bridge recebeu o mesmo estado que o mundo (a menos de float e do atraso de interpolação).
    bool bridgeOk = bridge.getSlotCount() == ids.size() + 1;
    for (BodyId id : ids) {
        const Vector3 phys = world.getBody(id).getWorldCenterOfMass();
        const Vector3 shown = bridge.getTransform(id).position;
        bridgeOk = bridgeOk && (phys - shown).length() < Real(0.01);
    }
    expect(bridgeOk, "BufferedTransformBridge reflete as poses do mundo");
}

/// A torre também balança pouco quando o solver tem mais iterações: tradeoff custo x rigidez.
void demoIterations() {
    std::printf("\n=== A2) Efeito do numero de iteracoes do solver na mesma torre (10 s) ===\n");
    std::printf("  iteracoes   balanco residual   afundamento do topo   deriva lateral\n");
    Real lateAt10 = 0, lateAt40 = 0;
    for (int iterations : {10, 20, 40}) {
        PhysicsWorld world;
        world.solverSettings().velocityIterations = iterations;
        const std::vector<BodyId> ids = buildTower(world);
        const StackReport r = runStack(world, ids, 10.0, false);
        std::printf("  %5d       %8.4f m/s        %8.4f m          %8.4f m\n", iterations, double(r.lateMaxSpeed),
                    double(r.topSink), double(r.maxDrift));
        if (iterations == 10) lateAt10 = r.lateMaxSpeed;
        if (iterations == 40) lateAt40 = r.lateMaxSpeed;
    }
    expect(lateAt40 < lateAt10, "mais iteracoes => torre mais rigida (menos balanco)");
}

// -----------------------------------------------------------------------------
// B) Interpolação
// -----------------------------------------------------------------------------
void demoInterpolation() {
    std::printf("\n=== B) Interpolacao: queda livre, fisica a 60 Hz, render a 144 Hz ===\n");
    PhysicsWorld world;
    BufferedTransformBridge bridge;
    world.setBridge(&bridge);
    const BodyId box = world.createBody(makeBox(Vector3(0, 50, 0), Vector3(0.5f, 0.5f, 0.5f)));

    const Real renderDt = Real(1) / Real(144);
    std::printf("  quadro  passos  alpha   y fisico (cru)   y interpolado   anterior <= interp <= atual\n");

    const int kFrames = 144;   // 1 s
    int rawStill = 0, interpStill = 0, outOfRange = 0;
    Real rawMaxJump = 0, interpMaxJump = 0;
    Real lastRaw = world.getBody(box).getWorldCenterOfMass().y;
    Real lastInterp = bridge.getTransform(box).position.y;
    bool counting = false;   // só conta depois do 1º passo físico: antes dele não há duas poses para interpolar

    for (int f = 0; f < kFrames; ++f) {
        const int steps = world.update(renderDt);
        if (steps > 0 && !counting) {
            counting = true;   // a 1ª pose "depois do passo" é o ponto de partida da medição
            lastRaw = world.getBody(box).getWorldCenterOfMass().y;
            lastInterp = bridge.getTransform(box).position.y;
            if (f < 12) {
                std::printf("  %4d    %d     %.3f   %14.5f   %13.5f        (1o passo fisico: inicio da medicao)\n", f, steps,
                            double(world.getAlpha()), double(lastRaw), double(lastInterp));
            }
            continue;
        }
        if (!counting) {
            if (f < 12) {
                std::printf("  %4d    %d     %.3f   %14.5f   %13.5f        (aquecimento: ainda sem passo fisico)\n", f, steps,
                            double(world.getAlpha()), double(lastRaw), double(lastInterp));
            }
            continue;
        }

        const Real raw = world.getBody(box).getWorldCenterOfMass().y;        // último passo físico
        const Real interp = bridge.getTransform(box).position.y;              // o que o render desenha
        const MotionState& ms = world.getMotionState(box);
        const Real lo = std::min(ms.getPrevious().position.y, ms.getCurrent().position.y);
        const Real hi = std::max(ms.getPrevious().position.y, ms.getCurrent().position.y);
        const bool inRange = interp >= lo - Real(1e-3) && interp <= hi + Real(1e-3);   // 1e-3: bridge guarda float

        if (f < 12) {
            std::printf("  %4d    %d     %.3f   %14.5f   %13.5f        %s\n", f, steps, double(world.getAlpha()),
                        double(raw), double(interp), inRange ? "sim" : "NAO");
        }
        if (raw == lastRaw) ++rawStill;
        if (interp == lastInterp) ++interpStill;
        if (!inRange) ++outOfRange;
        rawMaxJump = std::max(rawMaxJump, std::abs(raw - lastRaw));
        interpMaxJump = std::max(interpMaxJump, std::abs(interp - lastInterp));
        lastRaw = raw;
        lastInterp = interp;
    }
    std::printf("  ...\n");
    std::printf("  nos quadros medidos (apos o 1o passo): pose crua parada em %d quadros, maior salto %.3f m;\n", rawStill,
                double(rawMaxJump));
    std::printf("                                         pose interpolada parada em %d quadros, maior salto %.3f m\n",
                interpStill, double(interpMaxJump));

    expect(outOfRange == 0, "a pose interpolada sempre fica entre a pose anterior e a atual");
    expect(rawStill > 30, "sem interpolacao, o render repetiria o mesmo quadro (60 Hz mostrado a 144 Hz)");
    expect(interpStill == 0, "com interpolacao, o objeto se move em TODOS os quadros de render");
    expect(interpMaxJump < rawMaxJump * Real(0.6), "e o maior salto por quadro cai (movimento mais suave)");

    // Teletransporte: sem rastro de interpolação.
    world.setBodyTransform(box, Transform(Vector3(5, 2, 0)));
    world.update(renderDt);
    const Vector3 shown = bridge.getTransform(box).position;
    expect(std::abs(shown.x - Real(5)) < Real(0.2) && std::abs(shown.y - Real(2)) < Real(0.5),
           "setBodyTransform nao deixa 'rastro' de interpolacao entre a posicao antiga e a nova");
}

// -----------------------------------------------------------------------------
// C) Determinismo
// -----------------------------------------------------------------------------
void demoDeterminism() {
    std::printf("\n=== C) Determinismo: a mesma torre simulada duas vezes ===\n");
    PhysicsWorld w1, w2;
    const std::vector<BodyId> i1 = buildTower(w1);
    const std::vector<BodyId> i2 = buildTower(w2);
    runStack(w1, i1, 6.0, false);
    runStack(w2, i2, 6.0, false);

    bool same = i1.size() == i2.size();
    for (std::size_t i = 0; same && i < i1.size(); ++i) same = identicalBits(w1.getBody(i1[i]), w2.getBody(i2[i]));
    expect(same, "poses finais identicas bit a bit (mesma maquina e compilador)");
}

} // namespace

int main() {
#ifdef PHYSICS_USE_DOUBLE
    std::printf("physics_demo  (precisao: double)\n");
#else
    std::printf("physics_demo  (precisao: float)\n");
#endif
    demoStacking();
    demoIterations();
    demoInterpolation();
    demoDeterminism();

    std::printf("\n%s\n", g_failed == 0 ? "TODAS as verificacoes passaram." : "HOUVE FALHAS.");
    return g_failed == 0 ? 0 : 1;
}
