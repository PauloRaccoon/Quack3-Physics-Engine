// =============================================================================
// viewer_smoke.cpp
// Teste de FUMAÇA do viewer, sem janela e sem raylib (roda no ctest, inclusive sob ASan):
//   1) cada cena roda 20 s de tempo de render pelo mesmo caminho da janela (TimeControl + ContactTracker),
//      com poses (nos 3 modos de interpolação) finitas e quatérnios unitários o tempo todo;
//   2) "fuzz" determinístico: centenas de operações aleatórias (criar, lançar, apagar, editar parâmetros,
//      mudar o mundo, acordar/dormir, trocar de cena) intercaladas com passos, sem NaN e sem quebrar.
// Retorna 0 se tudo passou.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <limits>

#include "viewer/core/BodyEditor.h"
#include "viewer/core/ContactTracker.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/Scene.h"
#include "viewer/core/SceneFile.h"
#include "viewer/core/TimeControl.h"
#include "viewer/core/WorldEditor.h"

using namespace viewer;
using physics::Real;
using physics::Vector3;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                                \
    do {                                                                           \
        ++g_checks;                                                                \
        if (!(cond)) {                                                             \
            ++g_failures;                                                          \
            std::printf("FALHOU %s:%d  %s\n", __FILE__, __LINE__, #cond);          \
        }                                                                          \
    } while (0)

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * next(); }
    int pick(int n) { return static_cast<int>(next() * Real(n)) % n; }
};

/// Todos os corpos da cena: pose finita, quatérnio unitário, e as poses interpoladas (3 modos) finitas.
static bool sceneHealthy(const Scene& sc, Real alpha) {
    for (const SceneBody& b : sc.bodies) {
        if (!sc.world->isValidBody(b.id)) return false;
        const physics::RigidBody& rb = sc.world->getBody(b.id);
        if (!rb.getTransform().position.isFinite() || !rb.getOrientation().isFinite()) return false;
        if (std::fabs(rb.getOrientation().norm() - Real(1)) > Real(1e-2)) return false;
        if (!rb.getLinearVelocity().isFinite() || !rb.getAngularVelocity().isFinite()) return false;
        const physics::MotionState& ms = sc.world->getMotionState(b.id);
        for (int m = 0; m < kInterpModeCount; ++m) {
            const physics::Transform t = interpolatePose(ms.getPrevious(), ms.getCurrent(), alpha, static_cast<InterpMode>(m));
            if (!t.position.isFinite() || !t.rotation.isFinite()) return false;
        }
    }
    return true;
}

static void smokeAllScenes() {
    const std::size_t n = sceneList().size();
    CHECK(n >= 9);
    for (std::size_t i = 0; i < n; ++i) {
        Scene sc = makeScene(i);
        ContactTracker tracker;
        sc.world->setContactListener(&tracker);
        TimeControl tc;
        bool healthy = true;
        for (int f = 0; f < 20 * 60; ++f) {   // 20 s a 60 quadros/s
            tc.advance(*sc.world, Real(1) / Real(60));
            tracker.update(1.0f / 60.0f);
            if (f % 30 == 0) healthy = healthy && sceneHealthy(sc, tc.renderAlpha(*sc.world));
        }
        CHECK(healthy);
        CHECK(sceneHealthy(sc, tc.renderAlpha(*sc.world)));
        // Pausa e passo a passo: avança exatamente 1 passo.
        tc.setPaused(true);
        const auto before = sc.world->getStats().stepCount;
        tc.advance(*sc.world, Real(1) / Real(60));
        CHECK(sc.world->getStats().stepCount == before);
        tc.requestStep();
        tc.advance(*sc.world, Real(1) / Real(60));
        CHECK(sc.world->getStats().stepCount == before + 1);
        sc.world->setContactListener(nullptr);
    }
}

static void fuzz(unsigned seed) {
    Rng r(seed);
    Scene sc = makeScene(static_cast<std::size_t>(r.pick(static_cast<int>(sceneList().size()))));
    ContactTracker tracker;
    sc.world->setContactListener(&tracker);
    TimeControl tc;
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    bool healthy = true;
    for (int op = 0; op < 400; ++op) {
        const int k = r.pick(14);
        const std::size_t nb = sc.bodies.size();
        const physics::BodyId any = nb ? sc.bodies[static_cast<std::size_t>(r.pick(static_cast<int>(nb)))].id : physics::kInvalidBodyId;
        switch (k) {
            case 0: case 1: {   // criar, às vezes com valores absurdos
                SpawnParams p;
                p.kind = static_cast<SpawnKind>(r.pick(kSpawnKindCount));
                p.size = r.range(Real(-1), Real(10));
                p.mass = r.range(Real(-1), Real(500));
                p.position = Vector3(r.range(-15, 15), r.range(-3, 20), r.range(-15, 15));
                p.orientation = physics::Quaternion(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
                p.velocity = Vector3(r.range(-100, 100), r.range(-100, 100), r.range(-100, 100));
                p.angularVelocity = Vector3(r.range(-50, 50), r.range(-50, 50), r.range(-50, 50));
                if (r.pick(10) == 0) p.mass = nan;
                const physics::BodyId id = spawnBody(sc, p);
                CHECK(id == physics::kInvalidBodyId || sc.world->isValidBody(id));
                break;
            }
            case 2:
                launchFrom(sc, SpawnParams(), Vector3(r.range(-5, 5), 3, 12), Vector3(r.range(-1, 1), r.range(-0.3f, 0.3f), -1), r.range(0, 80));
                break;
            case 3: launchBody(sc, r.pick(2) ? ShapeKind::Sphere : ShapeKind::Box, Vector3(0, 4, 10), Vector3(0, 0, -1), Real(18)); break;
            case 4: if (r.pick(3) == 0) deleteBody(sc, any); break;
            case 5: setMass(*sc.world, any, r.range(Real(-1), Real(300))); break;
            case 6: setFriction(*sc.world, any, r.range(Real(-1), Real(3))); setRestitution(*sc.world, any, r.range(Real(-1), Real(2))); break;
            case 7: setPosition(*sc.world, any, Vector3(r.range(-12, 12), r.range(-2, 14), r.range(-12, 12))); break;
            case 8: setOrientationAxisAngle(*sc.world, any, Vector3(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)), r.range(0, 7)); break;
            case 9: setLinearVelocity(*sc.world, any, Vector3(r.range(-30, 30), r.range(-30, 30), r.range(-30, 30))); break;
            case 10: setAwake(*sc.world, any, r.pick(2) == 0); break;
            case 11: {   // mudar o mundo: gravidade, passo, iterações
                WorldParams wp = captureWorld(*sc.world);
                wp.gravity = Vector3(r.range(-30, 30), r.range(-30, 30), r.range(-30, 30));
                wp.fixedDt = Real(1) / r.range(Real(5), Real(300));
                wp.velocityIterations = 1 + r.pick(60);
                wp.positionIterations = r.pick(25);
                applyWorld(*sc.world, wp);
                break;
            }
            case 12:
                if (r.pick(3) == 0) {   // salvar e recarregar a cena no meio da bagunça (corpos criados, editados, lançados)
                    Scene loaded;
                    std::string err;
                    const bool ok = parseScene(serializeScene(sc), loaded, err);
                    CHECK(ok);
                    if (ok) {
                        tracker.reset();
                        sc = std::move(loaded);
                        sc.world->setContactListener(&tracker);
                        tc.onSceneReset();
                    }
                    break;
                }
                tc.setPaused(r.pick(2) == 0); tc.requestStep(); break;
            default: {   // trocar de cena (os ids antigos deixam de valer)
                tracker.reset();
                sc = makeScene(static_cast<std::size_t>(r.pick(static_cast<int>(sceneList().size()))));
                sc.world->setContactListener(&tracker);
                tc.onSceneReset();
                break;
            }
        }
        for (int f = 0; f < 6; ++f) tc.advance(*sc.world, Real(1) / Real(60));
        tracker.update(0.1f);
        healthy = healthy && sceneHealthy(sc, tc.renderAlpha(*sc.world));
    }
    CHECK(healthy);
    sc.world->setContactListener(nullptr);
}

int main() {
    smokeAllScenes();
    for (unsigned seed = 1; seed <= 6; ++seed) fuzz(seed * 7919u);
    std::printf("fumaca: %d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
