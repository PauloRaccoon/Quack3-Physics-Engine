// =============================================================================
// fuzz_tests.cpp — fuzzing determinístico da API pública (Fase 8).
//
// Sequências aleatórias (mas com semente fixa: reprodutíveis) de operações sobre o
// PhysicsWorld com valores hostis: NaN, ±Inf, 1e30, FLT_MAX, ids inválidos, formas
// degeneradas, juntas absurdas, passos de tempo malucos...
//
// Contrato verificado:
//   1. Nenhuma chamada trava, entra em laço infinito ou lança exceção fora da lista
//      documentada (invalid_argument, out_of_range, length_error).
//   2. Entradas NÃO finitas são ignoradas: o estado do mundo nunca fica não finito.
//   3. Depois de cada passo: poses e velocidades finitas, quatérnios unitários,
//      árvore da broadphase válida, contatos bem formados, estatísticas coerentes.
//   4. As consultas devolvem resultados bem formados (distância em [0, alcance],
//      normal unitária, ponto finito) ou "nada".
// =============================================================================
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "physics/bridge/IPhysicsTransformBridge.h"
#include "physics/debug/DebugDraw.h"
#include "physics/geometry/BoxShape.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/pipeline/PhysicsWorld.h"

using namespace physics;

static int g_failures = 0;
static int g_checks = 0;
static int g_rejected = 0;   // chamadas recusadas com exceção documentada (esperado)

#define CHECK_MSG(cond, ctx)                                                              \
    do {                                                                                  \
        ++g_checks;                                                                       \
        if (!(cond)) {                                                                    \
            ++g_failures;                                                                 \
            if (g_failures <= 40)                                                         \
                std::printf("FALHOU %s:%d  %s   [%s]\n", __FILE__, __LINE__, #cond, ctx); \
        }                                                                                 \
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
    bool chance(Real p) { return next() < p; }
};

static const Real kNaN = std::numeric_limits<Real>::quiet_NaN();
static const Real kInf = std::numeric_limits<Real>::infinity();

/// Valor hostil.
static Real weird(Rng& r) {
    switch (r.pick(14)) {
        case 0: return Real(0);
        case 1: return Real(1);
        case 2: return Real(-1);
        case 3: return Real(1e-30);
        case 4: return Real(-1e-30);
        case 5: return Real(1e10);
        case 6: return Real(-1e10);
        case 7: return Real(1e30);
        case 8: return Real(-1e30);
        case 9: return kNaN;
        case 10: return kInf;
        case 11: return -kInf;
        case 12: return std::numeric_limits<Real>::max();
        default: return r.range(-5, 5);
    }
}
/// Valor normal quase sempre; hostil de vez em quando.
static Real val(Rng& r, Real lo, Real hi, Real pWeird = Real(0.12)) {
    return r.chance(pWeird) ? weird(r) : r.range(lo, hi);
}
static Vector3 vec(Rng& r, Real lo, Real hi, Real pWeird = Real(0.12)) {
    return Vector3(val(r, lo, hi, pWeird), val(r, lo, hi, pWeird), val(r, lo, hi, pWeird));
}
static Quaternion randomRotation(Rng& r) {
    const Vector3 axis(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
    return Quaternion::fromAxisAngle(axis.normalizedOr(Vector3::unitY()), r.range(0, kTwoPi));
}

/// Executa f e classifica exceções. Retorna true se f terminou sem lançar.
template <typename F>
static bool guarded(const char* what, F&& f) {
    try {
        f();
        return true;
    } catch (const std::invalid_argument&) {
        ++g_rejected;
    } catch (const std::out_of_range&) {
        ++g_rejected;
    } catch (const std::length_error&) {
        ++g_rejected;
    } catch (const std::exception& e) {
        CHECK_MSG(false, (std::string("excecao inesperada: ") + e.what() + " em " + what).c_str());
    } catch (...) {
        CHECK_MSG(false, (std::string("excecao desconhecida em ") + what).c_str());
    }
    return false;
}

static std::shared_ptr<const Shape> randomShape(Rng& r) {
    switch (r.pick(4)) {
        case 0: return std::make_shared<SphereShape>(val(r, Real(0.2), Real(1.5)));
        case 1: return std::make_shared<BoxShape>(vec(r, Real(0.2), Real(1.5)));
        case 2: {
            // Octaedro escalado (convexo para escalas positivas; inválido para as demais → exceção).
            const Real a = val(r, Real(0.3), Real(1.5)), b = val(r, Real(0.3), Real(1.5)), c = val(r, Real(0.3), Real(1.5));
            const std::vector<Vector3> v = {Vector3(a, 0, 0), Vector3(-a, 0, 0), Vector3(0, b, 0),
                                            Vector3(0, -b, 0), Vector3(0, 0, c), Vector3(0, 0, -c)};
            std::vector<std::vector<int>> faces;
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    for (int sz = -1; sz <= 1; sz += 2) {
                        const int X = sx > 0 ? 0 : 1, Y = sy > 0 ? 2 : 3, Z = sz > 0 ? 4 : 5;
                        if (sx * sy * sz > 0) faces.push_back({X, Y, Z});
                        else                  faces.push_back({X, Z, Y});
                    }
            if (r.chance(Real(0.05))) faces.push_back({0, 1, 99});   // índice fora do limite
            return std::make_shared<PolyhedronShape>(v, faces);
        }
        default: return nullptr;   // sem forma
    }
}

static RigidBodyDesc randomDesc(Rng& r) {
    RigidBodyDesc d;
    d.type = r.pick(3) == 0 ? BodyType::Static : (r.pick(6) == 0 ? BodyType::Kinematic : BodyType::Dynamic);
    // A forma é construída por quem chama: pode lançar (reportado como "recusado").
    d.shape = randomShape(r);
    d.mass = r.chance(Real(0.5)) ? Real(0) : val(r, Real(0.1), Real(5));
    d.density = val(r, Real(0.1), Real(5));
    d.transform = Transform(vec(r, -6, 6, Real(0.08)), r.chance(Real(0.05)) ? Quaternion(kNaN, 0, 0, 0) : randomRotation(r));
    d.linearVelocity = vec(r, -10, 10, Real(0.05));
    d.angularVelocity = vec(r, -10, 10, Real(0.05));
    d.friction = val(r, 0, 1, Real(0.05));
    d.restitution = val(r, 0, 1, Real(0.05));
    d.linearDamping = val(r, 0, 1, Real(0.05));
    d.angularDamping = val(r, 0, 1, Real(0.05));
    d.gravityScale = val(r, 0, 2, Real(0.05));
    return d;
}

// ---------------------------------------------------------------------------------
// Invariantes
// ---------------------------------------------------------------------------------
/// Descreve o primeiro campo inválido do corpo ("" se está tudo certo).
static const char* badBodyField(const RigidBody& b) {
    if (!b.getTransform().isFinite()) return "transform";
    if (!b.getWorldCenterOfMass().isFinite()) return "centro de massa";
    if (!b.getLinearVelocity().isFinite()) return "velocidade linear";
    if (!b.getAngularVelocity().isFinite()) return "velocidade angular";
    if (!b.getOrientation().isFinite()) return "orientacao";
    if (!(std::abs(b.getOrientation().norm() - Real(1)) < Real(1e-3))) return "quaternio nao unitario";
    return "";
}

static void checkWorld(PhysicsWorld& w, const char* ctx) {
    int alive = 0;
    std::string bad;
    w.forEachBody([&](BodyId id, RigidBody& b) {
        ++alive;
        const char* field = badBodyField(b);
        if (*field && bad.empty()) bad = std::string("corpo ") + std::to_string(id) + ": " + field;
    });
    CHECK_MSG(bad.empty(), (std::string(ctx) + " " + bad).c_str());
    CHECK_MSG(w.getStats().bodies == alive, ctx);

    std::string err;
    CHECK_MSG(w.getCollisionWorld().getBroadphase().getTree().validate(&err), (std::string(ctx) + " arvore: " + err).c_str());

    std::string badContact;
    for (const ContactPair* cp : w.getContacts()) {
        const ContactManifold& m = cp->manifold;
        if (m.count < 1 || m.count > kMaxManifoldPoints) badContact = "contagem de pontos";
        if (!m.normal.isFinite() || std::abs(m.normal.length() - Real(1)) > Real(1e-3)) badContact = "normal";
        for (int i = 0; i < m.count; ++i) {
            const ContactPoint& p = m.points[i];
            if (!p.worldA.isFinite() || !p.worldB.isFinite()) badContact = "ponto nao finito";
            else if (!std::isfinite(p.depth)) badContact = "profundidade nao finita";
            else if (!std::isfinite(p.normalImpulse) || p.normalImpulse < Real(-1e-3)) badContact = "impulso normal";
            else if (!p.tangentImpulse.isFinite()) badContact = "impulso tangencial";
        }
    }
    CHECK_MSG(badContact.empty(), (std::string(ctx) + " contato: " + badContact).c_str());

    // A pose interpolada de cada corpo vivo é finita.
    bool interpOk = true;
    w.forEachBody([&](BodyId id, RigidBody&) {
        if (!w.getInterpolatedTransform(id).isFinite()) interpOk = false;
    });
    CHECK_MSG(interpOk, ctx);
}

static void checkQueryResults(const PhysicsWorld& w, const Vector3& o, const Vector3& d, Real maxDist, const char* ctx) {
    RaycastHit hit;
    if (w.raycast(o, d, maxDist, hit)) {
        CHECK_MSG(std::isfinite(hit.distance) && hit.distance >= 0, ctx);
        CHECK_MSG(hit.distance <= maxDist, ctx);
        CHECK_MSG(hit.point.isFinite() && hit.normal.isFinite(), ctx);
        CHECK_MSG(std::abs(hit.normal.length() - Real(1)) < Real(1e-3), ctx);
        CHECK_MSG(w.isValidBody(hit.body), ctx);
    }
    std::vector<RaycastHit> hits;
    const int n = w.raycastAll(o, d, maxDist, hits);
    CHECK_MSG(n == static_cast<int>(hits.size()), ctx);
    for (std::size_t i = 0; i < hits.size(); ++i) {
        CHECK_MSG(std::isfinite(hits[i].distance) && hits[i].distance >= 0 && hits[i].distance <= maxDist, ctx);
        if (i > 0) CHECK_MSG(hits[i - 1].distance <= hits[i].distance, ctx);   // ordenado
    }
}

// ---------------------------------------------------------------------------------
// Uma execução completa de fuzz
// ---------------------------------------------------------------------------------
static void runFuzz(unsigned seed, int operations) {
    Rng r(seed);
    PhysicsWorldSettings ws;
    ws.maxBodies = 48;            // pequeno: exercita length_error e reaproveitamento de ids
    ws.maxSubSteps = 4;
    PhysicsWorld w(ws);
    BufferedTransformBridge bridge;
    if (r.chance(Real(0.5))) w.setBridge(&bridge);
    char ctx[64];
    std::snprintf(ctx, sizeof ctx, "semente %u", seed);

    // Chão para as coisas poderem se apoiar.
    guarded("chao", [&] {
        RigidBodyDesc g;
        g.type = BodyType::Static;
        g.shape = std::make_shared<BoxShape>(Vector3(30, 1, 30));
        g.transform = Transform(Vector3(0, -1, 0));
        w.createBody(g);
    });

    DebugLineBuffer lines;
    for (int op = 0; op < operations; ++op) {
        const BodyId anyId = static_cast<BodyId>(r.pick(60));   // inclui ids inválidos (>= 48 ou mortos)
        switch (r.pick(22)) {
            case 0: case 1: case 2:   // criar corpo (a forma também pode lançar ao ser construída)
                guarded("createBody", [&] { w.createBody(randomDesc(r)); });
                break;
            case 3:
                w.destroyBody(anyId);
                break;
            case 4:   // forças e impulsos hostis
                guarded("forces", [&] {
                    RigidBody& b = w.getBody(anyId);
                    switch (r.pick(6)) {
                        case 0: b.applyForce(vec(r, -50, 50, Real(0.3))); break;
                        case 1: b.applyTorque(vec(r, -50, 50, Real(0.3))); break;
                        case 2: b.applyForceAtPoint(vec(r, -50, 50, Real(0.3)), vec(r, -5, 5, Real(0.3))); break;
                        case 3: b.applyLinearImpulse(vec(r, -20, 20, Real(0.3))); break;
                        case 4: b.applyAngularImpulse(vec(r, -20, 20, Real(0.3))); break;
                        default: b.applyImpulseAtPoint(vec(r, -20, 20, Real(0.3)), vec(r, -5, 5, Real(0.3))); break;
                    }
                });
                break;
            case 5:   // velocidades e parâmetros hostis
                guarded("setters", [&] {
                    RigidBody& b = w.getBody(anyId);
                    switch (r.pick(8)) {
                        case 0: b.setLinearVelocity(vec(r, -30, 30, Real(0.3))); break;
                        case 1: b.setAngularVelocity(vec(r, -30, 30, Real(0.3))); break;
                        case 2: b.setFriction(val(r, 0, 1, Real(0.3))); break;
                        case 3: b.setRestitution(val(r, 0, 1, Real(0.3))); break;
                        case 4: b.setLinearDamping(val(r, 0, 1, Real(0.3))); break;
                        case 5: b.setAngularDamping(val(r, 0, 1, Real(0.3))); break;
                        case 6: b.setGravityScale(val(r, 0, 2, Real(0.3))); break;
                        default: b.setMassData(MassData::fromMassAndInertia(val(r, Real(0.1), 5, Real(0.3)), Matrix3::identity(), vec(r, -1, 1, Real(0.3)))); break;
                    }
                });
                break;
            case 6:   // teletransporte
                guarded("setBodyTransform", [&] {
                    w.setBodyTransform(anyId, Transform(vec(r, -8, 8, Real(0.25)), r.chance(Real(0.2)) ? Quaternion(kNaN, kNaN, kNaN, kNaN) : randomRotation(r)));
                });
                break;
            case 7: case 8:   // juntas
                guarded("joint", [&] {
                    const BodyId a = static_cast<BodyId>(r.pick(60)), b = static_cast<BodyId>(r.pick(60));
                    const Vector3 anchor = vec(r, -6, 6, Real(0.2));
                    switch (r.pick(3)) {
                        case 0: w.createBallSocketJoint(a, b, anchor, r.chance(Real(0.3))); break;
                        case 1: w.createDistanceJoint(a, b, anchor, vec(r, -6, 6, Real(0.2)), r.chance(Real(0.5)) ? Real(-1) : val(r, 0, 5, Real(0.3))); break;
                        default: w.createHingeJoint(a, b, anchor, vec(r, -1, 1, Real(0.3))); break;
                    }
                });
                break;
            case 9: {  // destruir uma junta viva
                const JointConstraint* victim = nullptr;
                int idx = 0;
                const int want = r.pick(8);
                w.forEachJoint([&](const JointConstraint& j, BodyId, BodyId) { if (idx++ == want) victim = &j; });
                if (victim) w.destroyJoint(*victim);
                break;
            }
            case 10: case 11: case 12: case 13: case 14: {   // simular
                if (r.chance(Real(0.5))) {
                    w.step();
                } else {
                    const Real ft = r.chance(Real(0.5)) ? r.range(0, Real(0.05)) : weird(r);
                    guarded("update", [&] { w.update(ft); });
                }
                checkWorld(w, ctx);
                break;
            }
            case 15: {   // raycast hostil
                const Vector3 o = vec(r, -10, 10, Real(0.2)), d = vec(r, -1, 1, Real(0.2));
                const Real maxD = val(r, 0, 40, Real(0.25));
                checkQueryResults(w, o, d, maxD, ctx);
                break;
            }
            case 16: {   // overlaps hostis
                std::vector<BodyId> out;
                const int n1 = w.overlapSphere(vec(r, -8, 8, Real(0.2)), val(r, Real(0.1), 3, Real(0.25)), out);
                CHECK_MSG(n1 == static_cast<int>(out.size()), ctx);
                for (BodyId id : out) CHECK_MSG(w.isValidBody(id), ctx);
                w.overlapPoint(vec(r, -8, 8, Real(0.2)), out);
                guarded("overlapShape", [&] {
                    const BoxShape probe(vec(r, Real(0.1), 2, Real(0.1)));
                    w.overlapShape(probe, Transform(vec(r, -8, 8, Real(0.2)), randomRotation(r)), out);
                });
                break;
            }
            case 17:   // debug draw com qualquer combinação de flags
                lines.clear();
                {
                    DebugDrawSettings s;
                    s.flags = static_cast<std::uint32_t>(r.pick(0x200));
                    s.sphereSegments = r.pick(40) - 5;   // inclui negativos e zero
                    s.useInterpolatedPose = r.chance(Real(0.5));
                    debugDraw(w, lines, s);
                }
                break;
            case 18:
                w.setGravity(vec(r, -20, 20, Real(0.3)));
                break;
            case 19:   // acesso por id qualquer
                guarded("acessos", [&] {
                    (void)w.getInterpolatedTransform(anyId);
                    (void)w.getMotionState(anyId);
                    (void)w.getBody(anyId).getKineticEnergy();
                });
                break;
            case 20:
                if (r.chance(Real(0.5))) {
                    const BodyId ban = anyId;
                    w.setCollisionFilter([ban](BodyId a, BodyId b) { return a != ban && b != ban; });
                } else {
                    w.setCollisionFilter(nullptr);
                }
                break;
            default:
                (void)w.getStats();
                break;
        }
    }
    // Termina com alguns passos de assentamento e a verificação final.
    for (int i = 0; i < 20; ++i) w.step();
    checkWorld(w, ctx);
}

int main() {
    const unsigned seeds = 60;
    for (unsigned s = 1; s <= seeds; ++s) runFuzz(s * 7919u, 220);
    std::printf("  [info] fuzz: %u mundos, %d chamadas recusadas com excecao documentada\n", seeds, g_rejected);
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
