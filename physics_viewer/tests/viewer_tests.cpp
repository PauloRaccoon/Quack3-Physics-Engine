// =============================================================================
// viewer_tests.cpp — testes do núcleo do viewer (sem raylib, sem janela).
//
//   1. toMat4 aplicada a um ponto == Transform::transformPoint do motor (milhares de poses);
//   2. rotação conhecida (+90° em Y leva +X a −Z) e disposição em colunas (translação em m[12..14]);
//   3. as colunas de rotação formam uma base ortonormal destra;
//   4. OrbitCamera: posição, limites de pitch e zoom, entradas não finitas;
//   5. cena da V1: roda no PhysicsWorld, a caixa gira, pousa sobre o chão e o quatérnio segue unitário;
//   7. RenderPose (V3): Euler ida/volta, três modos de interpolação, gimbal lock, eixo-ângulo;
//   8. Interaction (V4): pick por raycast, arrastar com mola, lançar, apagar; OrbitCamera reset/follow;
//   9. TimeControl (V5): pausa, passo a passo, escala de tempo, α de render;
//   6. registro da V2: todas as cenas constroem, rodam 300 passos finitas/estáveis, nenhum corpo
//      atravessa o chão ou escapa; juntas mantêm os elos; setDistance da câmera.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/CapsuleShape.h"
#include "physics/geometry/CylinderShape.h"
#include "physics/geometry/SphereShape.h"
#include "viewer/core/BodyEditor.h"
#include "viewer/core/ConvexHull.h"
#include "viewer/core/ContactTracker.h"
#include "viewer/core/DockLayout.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/JointEditor.h"
#include "viewer/core/ModelAsset.h"
#include "viewer/core/ModelCollider.h"
#include "viewer/core/ModelImporter.h"
#include "viewer/core/PanelMetrics.h"
#include "viewer/core/ObjModel.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/OrbitCamera.h"
#include "viewer/core/WorldEditor.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/TimeControl.h"
#include "viewer/core/TrajectoryRecorder.h"
#include "viewer/core/Scene.h"
#include "viewer/core/SceneFile.h"
#include "viewer/core/ViewerMath.h"

using namespace viewer;

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

static bool near(float a, float b, float tol = 1e-4f) { return std::fabs(a - b) <= tol; }
static bool near(const Vec3f& a, const Vec3f& b, float tol = 1e-4f) {
    return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    physics::Real next() {   // [0, 1)
        s = s * 1664525u + 1013904223u;
        return physics::Real((s >> 8) & 0xFFFFFF) / physics::Real(16777216);
    }
    physics::Real range(physics::Real lo, physics::Real hi) { return lo + (hi - lo) * next(); }
};

static physics::Transform randomTransform(Rng& r) {
    using physics::Real;
    physics::Quaternion q(r.range(Real(-1), Real(1)), r.range(Real(-1), Real(1)),
                          r.range(Real(-1), Real(1)), r.range(Real(-1), Real(1)));
    if (q.normSquared() < Real(1e-6)) q = physics::Quaternion();
    q.normalize();
    return physics::Transform(physics::Vector3(r.range(Real(-10), Real(10)), r.range(Real(-10), Real(10)),
                                               r.range(Real(-10), Real(10))),
                              q);
}

// -----------------------------------------------------------------------------
static void testMatchesEngine() {
    Rng r(42);
    int bad = 0;
    for (int i = 0; i < 3000; ++i) {
        const physics::Transform xf = randomTransform(r);
        const physics::Vector3 p(r.range(-3, 3), r.range(-3, 3), r.range(-3, 3));
        const Vec3f viaMatrix = transformPoint(toMat4(xf), toVec3f(p));
        const Vec3f viaEngine = toVec3f(xf.transformPoint(p));
        if (!near(viaMatrix, viaEngine, 2e-4f)) ++bad;
    }
    CHECK(bad == 0);
}

static void testKnownRotation() {
    using physics::Real;
    // +90° em torno de Y (mão direita): +X → −Z, +Z → +X.
    const physics::Quaternion q = physics::Quaternion::fromAxisAngle(physics::Vector3(0, 1, 0), Real(1.57079632679));
    const Mat4f m = toMat4(physics::Transform(physics::Vector3(1, 2, 3), q));
    CHECK(near(transformPoint(m, Vec3f{1, 0, 0}), Vec3f{1, 2, 2}));   // (0,0,-1) + (1,2,3)
    CHECK(near(transformPoint(m, Vec3f{0, 0, 1}), Vec3f{2, 2, 3}));   // (1,0, 0) + (1,2,3)
    CHECK(near(transformPoint(m, Vec3f{0, 1, 0}), Vec3f{1, 3, 3}));   // Y não muda
    // Disposição em colunas: translação em m[12], m[13], m[14]; última linha (0,0,0,1).
    CHECK(m.m[12] == 1.0f && m.m[13] == 2.0f && m.m[14] == 3.0f);
    CHECK(m.m[3] == 0.0f && m.m[7] == 0.0f && m.m[11] == 0.0f && m.m[15] == 1.0f);
    // Identidade.
    const Mat4f id = toMat4(physics::Transform());
    CHECK(near(transformPoint(id, Vec3f{4, -5, 6}), Vec3f{4, -5, 6}));
}

static void testOrthonormalBasis() {
    Rng r(7);
    int bad = 0;
    for (int i = 0; i < 2000; ++i) {
        const Mat4f m = toMat4(randomTransform(r));
        const Vec3f x = axisOf(m, 0), y = axisOf(m, 1), z = axisOf(m, 2);
        auto dot = [](const Vec3f& a, const Vec3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
        const Vec3f xy{x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x};   // x × y
        const bool ok = near(dot(x, x), 1.0f, 1e-4f) && near(dot(y, y), 1.0f, 1e-4f) && near(dot(z, z), 1.0f, 1e-4f) &&
                        near(dot(x, y), 0.0f, 1e-4f) && near(dot(x, z), 0.0f, 1e-4f) && near(dot(y, z), 0.0f, 1e-4f) &&
                        near(xy, z, 1e-4f);   // destra: x × y = z (sem reflexão)
        if (!ok) ++bad;
    }
    CHECK(bad == 0);
}

// V19 — WASD: deslocamento da câmera
static void testCameraMove() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    OrbitCamera c;
    c.rotate(-c.getYaw(), -c.getPitch());   // yaw = 0, pitch = 0: a câmera está em +Z olhando para -Z
    Vec3f d = c.moveDelta(1, 0, 0, 1.0f);
    CHECK(d.z < 0 && std::fabs(d.x) < 1e-5f && std::fabs(d.y) < 1e-5f);        // W: para onde a câmera olha
    d = c.moveDelta(0, 1, 0, 1.0f);
    CHECK(d.x > 0 && std::fabs(d.z) < 1e-5f);                                    // D: direita da tela
    d = c.moveDelta(0, 0, 1, 1.0f);
    CHECK(d.y > 0 && std::fabs(d.x) < 1e-5f && std::fabs(d.z) < 1e-5f);        // E: sobe
    // com yaw = 90 graus a câmera fica em +X olhando para -X: frente = -X, direita = -Z... a direita da tela é +Z? conferir pela posição
    c.rotate(1.5707963f, 0.7f);
    const Vec3f p = c.position();
    const Vec3f f = c.moveDelta(1, 0, 0, 1.0f);
    // frente aponta do pé da câmera para o alvo no plano do chão
    const float tx = c.target.x - p.x, tz = c.target.z - p.z;
    const float tl = std::sqrt(tx * tx + tz * tz), fl = std::sqrt(f.x * f.x + f.z * f.z);
    CHECK(near((f.x * tx + f.z * tz) / (tl * fl), 1.0f, 1e-4f) && std::fabs(f.y) < 1e-6f);
    const Vec3f r = c.moveDelta(0, 1, 0, 1.0f);
    CHECK(std::fabs(f.x * r.x + f.z * r.z) < 1e-4f * fl * fl && (r.x * f.z - r.z * f.x) < 0);   // perpendicular, à direita (horário visto de cima)
    // a velocidade cresce com a distância, com dt e com o boost; é limitada
    OrbitCamera far; far.setDistance(100.0f);
    CHECK(far.moveDelta(1, 0, 0, 0.1f).z != 0.0f);
    const auto len = [](const Vec3f& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); };
    CHECK(len(far.moveDelta(1, 0, 0, 0.05f)) > len(c.moveDelta(1, 0, 0, 0.05f)) * 0 + 0);
    OrbitCamera nearC; nearC.setDistance(2.0f);
    CHECK(len(far.moveDelta(1, 0, 0, 0.05f)) > 5.0f * len(nearC.moveDelta(1, 0, 0, 0.05f)));
    CHECK(near(len(c.moveDelta(1, 0, 0, 0.05f, 3.0f)), 3.0f * len(c.moveDelta(1, 0, 0, 0.05f)), 1e-4f));
    CHECK(len(c.moveDelta(1, 0, 0, 100.0f)) <= len(c.moveDelta(1, 0, 0, 0.1f)) * 1.0001f);   // dt limitado
    // entradas inválidas: nada se move
    CHECK(len(c.moveDelta(nan, 0, 0, 1.0f)) == 0.0f && len(c.moveDelta(1, 0, 0, nan)) == 0.0f && len(c.moveDelta(1, 0, 0, -1.0f)) == 0.0f &&
          len(c.moveDelta(1, 0, 0, 1.0f, nan)) == 0.0f && len(c.moveDelta(0, 0, 0, 1.0f)) == 0.0f);
    // teclas fora de [-1,1] são limitadas
    CHECK(near(len(c.moveDelta(50, 0, 0, 0.05f)), len(c.moveDelta(1, 0, 0, 0.05f)), 1e-5f));
}

static void testOrbitCamera() {
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();

    OrbitCamera c;
    // Distância da câmera ao alvo == distance, em qualquer orientação.
    auto dist = [](const OrbitCamera& cam) {
        const Vec3f p = cam.position();
        const float dx = p.x - cam.target.x, dy = p.y - cam.target.y, dz = p.z - cam.target.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    CHECK(near(dist(c), c.getDistance(), 1e-4f));

    // yaw = 0, pitch = 0 → câmera no +Z do alvo.
    c.rotate(-c.getYaw(), -c.getPitch());
    CHECK(near(c.position(), Vec3f{c.target.x, c.target.y, c.target.z + c.getDistance()}, 1e-4f));

    // Pitch limitado ao intervalo (não passa do pólo); yaw dá a volta sem crescer.
    c.rotate(0.0f, 100.0f);
    CHECK(c.getPitch() == OrbitCamera::kMaxPitch);
    c.rotate(0.0f, -100.0f);
    CHECK(c.getPitch() == OrbitCamera::kMinPitch);
    for (int i = 0; i < 1000; ++i) c.rotate(0.7f, 0.0f);
    CHECK(std::fabs(c.getYaw()) <= 3.1416f);
    CHECK(near(dist(c), c.getDistance(), 1e-3f));

    // Zoom limitado nos dois sentidos; aproximar reduz a distância.
    OrbitCamera z;
    const float d0 = z.getDistance();
    z.zoom(1.0f);
    CHECK(z.getDistance() < d0);
    z.zoom(-1000.0f);
    CHECK(z.getDistance() == OrbitCamera::kMaxDistance);
    z.zoom(1000.0f);
    CHECK(z.getDistance() == OrbitCamera::kMinDistance);

    // Entradas não finitas são ignoradas (o estado não muda).
    OrbitCamera n;
    const float y0 = n.getYaw(), p0 = n.getPitch(), dd = n.getDistance();
    n.rotate(nan, 0.1f);
    n.rotate(0.1f, inf);
    n.zoom(nan);
    n.zoom(inf);
    n.setTarget(Vec3f{nan, 0, 0});
    CHECK(n.getYaw() == y0 && n.getPitch() == p0 && n.getDistance() == dd);
    CHECK(near(n.target, Vec3f{0.0f, 1.5f, 0.0f}));
}

static void testFallingBoxScene() {
    using namespace physics;
    Scene s = makeFallingBoxScene();
    CHECK(s.world != nullptr);
    CHECK(s.bodies.size() == 2);
    CHECK(s.bodies[0].isStatic && !s.bodies[1].isStatic);
    CHECK(s.focus == s.bodies[1].id);
    CHECK(s.world->isValidBody(s.focus));

    const Quaternion q0 = s.world->getBody(s.focus).getOrientation();

    // 0,5 s a 60 Hz: a caixa já girou bem (ω ≈ 2,4 rad/s), medido pelo quatérnio.
    for (int i = 0; i < 30; ++i) s.world->update(Real(1) / Real(60));
    const Quaternion q1 = s.world->getBody(s.focus).getOrientation();
    const Real dot = std::fabs(q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z);
    CHECK(dot < Real(0.97));                  // ângulo entre as orientações > ~28°

    // 12 s: a caixa pousa sobre o chão (centro a 0,5 m, ou tombada/deitada), tudo finito e unitário.
    bool finite = true, unit = true;
    for (int i = 0; i < 720; ++i) {
        s.world->update(Real(1) / Real(60));
        const RigidBody& b = s.world->getBody(s.focus);
        finite = finite && b.getWorldCenterOfMass().isFinite() && b.getLinearVelocity().isFinite();
        unit = unit && std::fabs(b.getOrientation().norm() - Real(1)) < Real(1e-4);
    }
    CHECK(finite);
    CHECK(unit);
    const Real y = s.world->getBody(s.focus).getWorldCenterOfMass().y;
    CHECK(y > Real(0.45) && y < Real(0.9));    // 0,5 deitada de face; até ~0,87 se apoiada numa aresta/vértice

    // A pose interpolada (a que a janela desenha) é uma rotação válida: base ortonormal.
    const Transform xf = s.world->getInterpolatedTransform(s.focus);
    CHECK(std::fabs(xf.rotation.norm() - Real(1)) < Real(1e-4));
    const Mat4f m = toMat4(xf);
    const Vec3f ax = axisOf(m, 0);
    CHECK(near(ax.x * ax.x + ax.y * ax.y + ax.z * ax.z, 1.0f, 1e-4f));
}

static void testSceneRegistry() {
    using namespace physics;
    const std::vector<SceneInfo>& list = sceneList();
    CHECK(list.size() == 14);   // 9 originais + 5 da V11 (anexadas: teclas 1-9 não mudam)
    CHECK(list[2].name == std::string("Piramide de caixas"));
    CHECK(makeScene(9999).name == list.back().make().name);   // índice fora do intervalo é limitado

    for (std::size_t i = 0; i < list.size(); ++i) {
        Scene s = makeScene(i);
        std::printf("  cena %zu: %s (%zu corpos)\n", i + 1, s.name.c_str(), s.bodies.size());
        CHECK(s.name == list[i].name);
        CHECK(s.world != nullptr);
        CHECK(!s.bodies.empty());
        CHECK(s.focus != kInvalidBodyId && s.world->isValidBody(s.focus));

        bool shapesOk = true, valid = true;
        for (const SceneBody& b : s.bodies) {
            valid = valid && s.world->isValidBody(b.id);
            if (b.kind == ShapeKind::Polyhedron) shapesOk = shapesOk && b.poly && b.poly->getTopology().getFaceCount() >= 4;
            if (b.kind == ShapeKind::Sphere) shapesOk = shapesOk && b.radius > 0.0f;
            if (b.kind == ShapeKind::Box)
                shapesOk = shapesOk && b.halfExtents.x > 0.0f && b.halfExtents.y > 0.0f && b.halfExtents.z > 0.0f;
        }
        CHECK(valid);
        CHECK(shapesOk);
        for (const SceneLink& l : s.links) CHECK(s.world->isValidBody(l.a) && s.world->isValidBody(l.b));

        // 300 passos (5 s): tudo finito, quatérnios unitários, nada atravessa o chão nem escapa.
        for (int k = 0; k < 300; ++k) s.world->update(Real(1) / Real(60));
        bool finite = true, unit = true, inside = true;
        for (const SceneBody& b : s.bodies) {
            const Transform xf = s.world->getInterpolatedTransform(b.id);
            finite = finite && xf.position.isFinite();
            unit = unit && std::fabs(xf.rotation.norm() - Real(1)) < Real(1e-3);
            if (!b.isStatic) inside = inside && xf.position.y > Real(-0.1) - (s.links.empty() ? Real(0) : Real(10)) &&
                                      std::fabs(xf.position.x) < Real(30) && std::fabs(xf.position.z) < Real(30);
        }
        CHECK(finite);
        CHECK(unit);
        CHECK(inside);

        // Juntas: os elos não se separam (distância entre centros quase constante / limitada).
        for (const SceneLink& l : s.links) {
            const Real d = (s.world->getBody(l.a).getWorldCenterOfMass() -
                            s.world->getBody(l.b).getWorldCenterOfMass()).length();
            CHECK(d < Real(3.2));
        }
    }

    // A pilha e a pirâmide continuam de pé: topo ainda alto e perto do eixo.
    for (std::size_t idx : {std::size_t(1), std::size_t(2)}) {
        Scene s = makeScene(idx);
        for (int k = 0; k < 300; ++k) s.world->update(Real(1) / Real(60));
        const Vector3 top = s.world->getBody(s.focus).getWorldCenterOfMass();
        CHECK(top.y > Real(1.4));
        CHECK(std::fabs(top.x) < Real(1.0) && std::fabs(top.z) < Real(1.0));
    }
}

/// Ângulo [rad] da rotação relativa entre dois quatérnios (estável perto de 0, ao contrário de acos).
static physics::Real angleBetween(const physics::Quaternion& a, const physics::Quaternion& b) {
    const physics::Quaternion r = a.conjugate() * b;
    const physics::Real vlen = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    return physics::Real(2) * std::atan2(vlen, std::fabs(r.w));
}

static void testRenderPose() {
    using namespace physics;
    const Real pi = Real(3.14159265358979323846);
    const Vector3 ax(1, 0, 0);

    // --- Euler: ida e volta em muitas orientações (fora do gimbal lock) -----------------------------
    unsigned seed = 12345;
    auto rnd = [&seed]() { seed = seed * 1664525u + 1013904223u; return Real((seed >> 8) & 0xFFFFFF) / Real(16777216); };
    bool roundTrip = true;
    for (int i = 0; i < 2000; ++i) {
        Vector3 axis(rnd() - Real(0.5), rnd() - Real(0.5), rnd() - Real(0.5));
        if (axis.lengthSquared() < Real(1e-4)) continue;
        const Quaternion q = Quaternion::fromAxisAngle(axis.normalized(), rnd() * Real(6.28));
        roundTrip = roundTrip && angleBetween(q, fromEuler(toEuler(q))) < Real(2e-3);
    }
    CHECK(roundTrip);

    // Conhecidos: rotação pura em X é pitch; em Y é yaw; em Z é roll.
    const EulerAngles ex = toEuler(Quaternion::fromAxisAngle(ax, Real(0.5)));
    CHECK(std::fabs(ex.pitch - Real(0.5)) < Real(1e-4) && std::fabs(ex.yaw) < Real(1e-4) && std::fabs(ex.roll) < Real(1e-4));
    const EulerAngles ey = toEuler(Quaternion::fromAxisAngle(Vector3(0, 1, 0), Real(0.7)));
    CHECK(std::fabs(ey.yaw - Real(0.7)) < Real(1e-4) && std::fabs(ey.pitch) < Real(1e-4));
    const EulerAngles ez = toEuler(Quaternion::fromAxisAngle(Vector3(0, 0, 1), Real(-0.4)));
    CHECK(std::fabs(ez.roll + Real(0.4)) < Real(1e-4) && std::fabs(ez.pitch) < Real(1e-4));

    // Gimbal lock: pitch = 90° ainda reconstrói a mesma ROTAÇÃO (mas roll é forçado a 0).
    const Quaternion lock = Quaternion::fromAxisAngle(ax, pi * Real(0.5));
    const EulerAngles el = toEuler(lock);
    CHECK(nearGimbalLock(el, Real(0.01)));
    CHECK(!nearGimbalLock(ex, Real(0.1)));
    CHECK(angleBetween(lock, fromEuler(el)) < Real(2e-3));
    CHECK(std::fabs(el.roll) < Real(1e-6));
    // Quatérnio não unitário não quebra a conversão.
    CHECK(toEuler(Quaternion(2, 0, 0, 0)).pitch == Real(0));

    // --- Modos de interpolação ----------------------------------------------------------------------------
    const Quaternion a = Quaternion::identity();
    const Quaternion b = Quaternion::fromAxisAngle(Vector3(0, 1, 0), pi * Real(0.5));   // 90° em Y

    CHECK(angleBetween(interpolateOrientation(a, b, 0, InterpMode::Slerp), a) < Real(1e-4));
    CHECK(angleBetween(interpolateOrientation(a, b, 1, InterpMode::Slerp), b) < Real(1e-4));
    const Quaternion sm = interpolateOrientation(a, b, Real(0.5), InterpMode::Slerp);
    CHECK(std::fabs(sm.norm() - Real(1)) < Real(1e-4));
    CHECK(std::fabs(angleBetween(a, sm) - pi * Real(0.25)) < Real(1e-3));   // metade do caminho: 45°

    // Lerp sem normalizar: extremos exatos, mas no meio |q| = cos(Δ/4) (Δ = ângulo da rotação) = 0,9239 para 90°.
    CHECK(angleBetween(interpolateOrientation(a, b, 0, InterpMode::LerpNoNormalize), a) < Real(1e-4));
    CHECK(angleBetween(interpolateOrientation(a, b, 1, InterpMode::LerpNoNormalize), b) < Real(1e-4));
    const Quaternion lm = interpolateOrientation(a, b, Real(0.5), InterpMode::LerpNoNormalize);
    CHECK(std::fabs(lm.norm() - Real(0.92387953)) < Real(1e-3));
    // Hemisfério oposto (-b = mesma rotação): o caminho continua o curto.
    const Quaternion nb(-b.w, -b.x, -b.y, -b.z);
    const Quaternion lm2 = interpolateOrientation(a, nb, Real(0.5), InterpMode::LerpNoNormalize);
    CHECK(std::fabs(lm2.norm() - Real(0.92387953)) < Real(1e-3));

    // Euler: extremos reproduzem a rotação; entre eles |q| = 1 (volta de fromEuler).
    CHECK(angleBetween(interpolateOrientation(a, b, 0, InterpMode::Euler), a) < Real(2e-3));
    CHECK(angleBetween(interpolateOrientation(a, b, 1, InterpMode::Euler), b) < Real(2e-3));
    CHECK(std::fabs(interpolateOrientation(a, b, Real(0.5), InterpMode::Euler).norm() - Real(1)) < Real(1e-4));

    // Através do gimbal lock o Euler erra o caminho; o slerp não.
    const Quaternion p0 = Quaternion::fromAxisAngle(ax, Real(60) * pi / Real(180));
    const Quaternion p1 = Quaternion::fromAxisAngle(ax, Real(120) * pi / Real(180));
    const Quaternion truth = Quaternion::fromAxisAngle(ax, Real(90) * pi / Real(180));
    const Quaternion es = interpolateOrientation(p0, p1, Real(0.5), InterpMode::Euler);
    const Quaternion ss = interpolateOrientation(p0, p1, Real(0.5), InterpMode::Slerp);
    CHECK(angleBetween(ss, truth) < Real(1e-3));
    CHECK(angleBetween(es, truth) > Real(0.4));   // ≈ 30° fora (desvio = 90° - pitch inicial)

    // Robustez: t fora de [0,1] é limitado; entradas não finitas devolvem b.
    CHECK(angleBetween(interpolateOrientation(a, b, Real(5), InterpMode::Slerp), b) < Real(1e-4));
    CHECK(angleBetween(interpolateOrientation(a, b, Real(-5), InterpMode::Euler), a) < Real(2e-3));
    const Quaternion bad(std::numeric_limits<Real>::quiet_NaN(), 0, 0, 0);
    CHECK(angleBetween(interpolateOrientation(bad, b, Real(0.5), InterpMode::Slerp), b) < Real(1e-6));
    CHECK(angleBetween(interpolateOrientation(a, b, std::numeric_limits<Real>::quiet_NaN(), InterpMode::Euler), b) < Real(1e-6));

    // Pose: a posição é sempre lerp.
    const Transform ta(Vector3(0, 0, 0), a), tb(Vector3(2, 4, 6), b);
    for (int m = 0; m < kInterpModeCount; ++m) {
        const Transform tm = interpolatePose(ta, tb, Real(0.25), static_cast<InterpMode>(m));
        CHECK(std::fabs(tm.position.x - Real(0.5)) < Real(1e-5) && std::fabs(tm.position.z - Real(1.5)) < Real(1e-5));
    }
    CHECK(std::string(interpModeName(InterpMode::Slerp)) == "slerp");

    // --- Eixo-ângulo ------------------------------------------------------------------------------------------
    const Vector3 u = Vector3(1, 2, 3).normalized();
    const Quaternion qa = Quaternion::fromAxisAngle(u, Real(0.9));
    const AxisAngleF aa = toAxisAngle(qa);
    CHECK(near(aa.angle, 0.9f, 1e-4f));
    CHECK(near(aa.axis.x, static_cast<float>(u.x), 1e-4f) && near(aa.axis.y, static_cast<float>(u.y), 1e-4f) &&
          near(aa.axis.z, static_cast<float>(u.z), 1e-4f));
    const AxisAngleF an = toAxisAngle(Quaternion(-qa.w, -qa.x, -qa.y, -qa.z));   // -q = mesma rotação
    CHECK(near(an.angle, 0.9f, 1e-4f) && near(an.axis.x, aa.axis.x, 1e-4f));
    CHECK(near(toAxisAngle(Quaternion::identity()).angle, 0.0f, 1e-6f));
}

static void testInteraction() {
    using namespace physics;
    const Real dt = Real(1) / Real(60);

    // --- pickBody: pilha de caixas (cena 2); o chão (estático) nunca é selecionado ----------------------
    Scene st = makeStackScene();
    for (int i = 0; i < 5; ++i) st.world->update(dt);
    const PickResult top = pickBody(st, Vector3(0, 10, 0), Vector3(0, -1, 0));
    CHECK(top.hit && top.body == st.focus);                          // de cima: a caixa do topo
    CHECK(std::fabs(top.point.y - st.world->getBody(st.focus).getWorldCenterOfMass().y - Real(0.5)) < Real(0.02));
    const PickResult side = pickBody(st, Vector3(10, Real(0.5), 0), Vector3(-1, 0, 0));
    CHECK(side.hit && side.body != st.focus && side.distance > Real(9.0));   // de lado: a de baixo
    CHECK(!pickBody(st, Vector3(8, 10, 8), Vector3(0, -1, 0)).hit);          // só o chão: nada
    CHECK(!pickBody(st, Vector3(0, 10, 0), Vector3(0, 0, 0)).hit);           // direção nula
    CHECK(!pickBody(st, Vector3(0, 10, 0), Vector3(std::numeric_limits<Real>::quiet_NaN(), -1, 0)).hit);
    CHECK(!pickBody(st, Vector3(0, 10, 0), Vector3(0, -1, 0), Real(3)).hit); // além do alcance

    // --- Mola: arrasta a caixa do topo para cima e para o lado; depois solta e ela cai -------------------
    {
        Scene s = makeStackScene();
        for (int i = 0; i < 30; ++i) s.world->update(dt);
        const Vector3 o(0, 14, 0);
        const PickResult p = pickBody(s, o, Vector3(0, -1, 0));
        CHECK(p.hit);
        SpringDrag drag;
        CHECK(drag.begin(s, p));
        CHECK(drag.active() && drag.body() == s.focus);
        // Mouse: o raio passa a apontar para um ponto 3 m acima e 2 m para o lado, à mesma profundidade.
        const Vector3 goal = p.point + Vector3(2, 3, 0);
        const Vector3 dir = (goal - o).normalized();
        bool finite = true;
        for (int i = 0; i < 180; ++i) {
            drag.apply(s, o, dir, dt);
            s.world->update(dt);
            finite = finite && s.world->getBody(s.focus).getWorldCenterOfMass().isFinite();
        }
        CHECK(finite);
        const Vector3 pos = s.world->getBody(s.focus).getWorldCenterOfMass();
        CHECK(pos.y > Real(5.5));                  // subiu bem além dos 5,5 m do topo da pilha
        CHECK(pos.x > Real(0.8));                  // foi para o lado
        CHECK((drag.anchorWorld() - drag.targetWorld()).length() < Real(1.2));   // a mola alcança o alvo
        // Ponto agarrado fora do centro: a caixa girou ao ser puxada (quatérnio ≠ identidade).
        const Quaternion q = s.world->getBody(s.focus).getOrientation();
        CHECK(std::fabs(q.w) < Real(0.9999));
        drag.end();
        CHECK(!drag.active());
        const Real y0 = s.world->getBody(s.focus).getWorldCenterOfMass().y;
        for (int i = 0; i < 30; ++i) s.world->update(dt);
        CHECK(s.world->getBody(s.focus).getWorldCenterOfMass().y < y0);   // solta: cai
    }
    // Corpo apagado durante o arrasto: apply termina o arrasto sem falhar.
    {
        Scene s = makeStackScene();
        s.world->update(dt);
        SpringDrag drag;
        const PickResult p = pickBody(s, Vector3(0, 14, 0), Vector3(0, -1, 0));
        CHECK(drag.begin(s, p));
        CHECK(deleteBody(s, p.body));
        CHECK(!drag.apply(s, Vector3(0, 14, 0), Vector3(0, -1, 0), dt));
        CHECK(!drag.active());
    }
    // Passo físico largo (0,5 s): a mola é limitada e não explode.
    {
        Scene s = makeGimbalScene();
        s.world->update(Real(0.5));
        const Vector3 o(0, 2.5, 9);
        const PickResult p = pickBody(s, o, Vector3(0, 0, -1));
        CHECK(p.hit);
        SpringDrag drag;
        CHECK(drag.begin(s, p));
        bool finite = true;
        for (int i = 0; i < 20; ++i) {
            drag.apply(s, o, Vector3(0, Real(0.2), -1), Real(1) / Real(60));
            s.world->update(Real(0.5));
            finite = finite && s.world->getBody(s.focus).getWorldCenterOfMass().isFinite() &&
                     s.world->getBody(s.focus).getWorldCenterOfMass().length() < Real(100);
        }
        CHECK(finite);
    }
    // begin com acerto inválido ou corpo estático não ativa o arrasto.
    {
        Scene s = makeStackScene();
        SpringDrag drag;
        CHECK(!drag.begin(s, PickResult()));
        PickResult fake;
        fake.hit = true;
        fake.body = s.bodies.front().id;   // o chão (estático)
        fake.point = Vector3(0, 0, 0);
        CHECK(!drag.begin(s, fake));
        CHECK(!drag.active());
        CHECK(!drag.apply(s, Vector3(0, 5, 0), Vector3(0, -1, 0), dt));
    }

    // --- launchBody -------------------------------------------------------------------------------------------------
    {
        Scene s = makeFallingBoxScene();
        const std::size_t n0 = s.bodies.size();
        const BodyId a = launchBody(s, ShapeKind::Box, Vector3(0, 3, 8), Vector3(0, 0, -2), Real(12));
        const BodyId b = launchBody(s, ShapeKind::Sphere, Vector3(1, 3, 8), Vector3(0, 0, -1), Real(500));   // limitada a 60
        CHECK(a != kInvalidBodyId && b != kInvalidBodyId && s.bodies.size() == n0 + 2);
        CHECK(s.bodies[n0].kind == ShapeKind::Box && s.bodies[n0 + 1].kind == ShapeKind::Sphere);
        const Vector3 va = s.world->getBody(a).getLinearVelocity();
        CHECK(std::fabs(va.z + Real(12)) < Real(1e-3) && std::fabs(va.x) < Real(1e-4));
        CHECK(std::fabs(s.world->getBody(b).getLinearVelocity().length() - Real(60)) < Real(1e-2));
        CHECK(launchBody(s, ShapeKind::Box, Vector3(0, 3, 0), Vector3(0, 0, 0), Real(5)) == kInvalidBodyId);
        CHECK(launchBody(s, ShapeKind::Box, Vector3(std::numeric_limits<Real>::infinity(), 0, 0), Vector3(0, 0, 1), Real(5)) == kInvalidBodyId);
        CHECK(s.bodies.size() == n0 + 2);
        bool finite = true;
        for (int i = 0; i < 120; ++i) {
            s.world->update(dt);
            finite = finite && s.world->getBody(a).getWorldCenterOfMass().isFinite();
        }
        CHECK(finite);
        // A caixa lançada é selecionável.
        CHECK(pickBody(s, Vector3(0, 40, -50), (s.world->getBody(a).getWorldCenterOfMass() - Vector3(0, 40, -50))).body == a);
    }

    // --- deleteBody -------------------------------------------------------------------------------------------------
    {
        Scene s = makeChainScene();
        const std::size_t nb = s.bodies.size(), nl = s.links.size();
        const BodyId mid = s.links[3].b;                      // um elo do meio
        CHECK(deleteBody(s, mid));
        CHECK(!s.world->isValidBody(mid));
        CHECK(s.bodies.size() == nb - 1);
        CHECK(s.links.size() == nl - 2);                      // as duas ligações do elo somem
        CHECK(s.focus != mid);
        bool finite = true;
        for (int i = 0; i < 120; ++i) s.world->update(dt);
        for (const SceneBody& b : s.bodies) finite = finite && s.world->getBody(b.id).getWorldCenterOfMass().isFinite();
        CHECK(finite);
        CHECK(!deleteBody(s, mid));                           // já apagado
        CHECK(!deleteBody(s, s.bodies.front().id));          // âncora (estática)
        CHECK(!deleteBody(s, kInvalidBodyId));
        // Apagar o corpo em foco limpa o foco.
        Scene f = makeFallingBoxScene();
        const BodyId fb = f.focus;
        CHECK(deleteBody(f, fb) && f.focus == kInvalidBodyId);
        CHECK(!deleteBody(f, f.bodies.front().id));           // o chão
    }

    // --- OrbitCamera: reset e follow ----------------------------------------------------------------------------------
    {
        OrbitCamera c;
        c.rotate(1.0f, 0.5f);
        c.zoom(5.0f);
        c.target = Vec3f{9.0f, 9.0f, 9.0f};
        c.reset(Vec3f{1.0f, 2.0f, 3.0f}, 14.0f);
        CHECK(near(c.getYaw(), 0.8f, 1e-6f) && near(c.getPitch(), 0.5f, 1e-6f));
        CHECK(near(c.getDistance(), 14.0f, 1e-6f) && near(c.target.x, 1.0f, 1e-6f) && near(c.target.z, 3.0f, 1e-6f));
        c.target = Vec3f{0, 0, 0};
        c.followTarget(Vec3f{10.0f, 0.0f, 0.0f}, 0.016f);
        CHECK(c.target.x > 1.0f && c.target.x < 3.0f);                      // andou uma fração
        for (int i = 0; i < 300; ++i) c.followTarget(Vec3f{10.0f, 0.0f, 0.0f}, 0.016f);
        CHECK(near(c.target.x, 10.0f, 1e-3f));                                // converge
        // Independente do FPS: 1 passo de 0,1 s ≈ 10 passos de 0,01 s.
        OrbitCamera p, q;
        p.target = q.target = Vec3f{0, 0, 0};
        p.followTarget(Vec3f{5.0f, 0, 0}, 0.1f);
        for (int i = 0; i < 10; ++i) q.followTarget(Vec3f{5.0f, 0, 0}, 0.01f);
        CHECK(near(p.target.x, q.target.x, 1e-3f));
        const Vec3f before = c.target;
        c.followTarget(Vec3f{std::numeric_limits<float>::quiet_NaN(), 0, 0}, 0.016f);
        c.followTarget(Vec3f{1, 1, 1}, -1.0f);
        CHECK(near(c.target.x, before.x, 1e-9f));                             // entradas inválidas ignoradas
    }
}

static void testTimeControl() {
    using namespace physics;
    const Real dt = Real(1) / Real(60);
    auto steps = [](const Scene& sc) { return sc.world->getStats().stepCount; };

    // Escalas.
    CHECK(TimeControl::scaleValue(0) == 1.0f && TimeControl::scaleValue(1) == 0.5f);
    CHECK(TimeControl::scaleValue(2) == 0.25f && TimeControl::scaleValue(3) == 0.1f);
    CHECK(TimeControl::scaleValue(-4) == 1.0f && TimeControl::scaleValue(99) == 0.1f);   // limitado
    CHECK(std::string(TimeControl::scaleLabel(0)) == "1x" && std::string(TimeControl::scaleLabel(3)) == "1/10");
    TimeControl tc;
    tc.setScaleIndex(77);
    CHECK(tc.scaleIndex() == TimeControl::kScaleCount - 1);
    tc.setScaleIndex(-1);
    CHECK(tc.scaleIndex() == 0 && tc.timeScale() == 1.0f && !tc.paused());

    // Rodando a 1x: 120 quadros de 1/60 s ≈ 120 passos (o acumulador pode atrasar 1).
    {
        Scene s = makeFallingBoxScene();
        TimeControl t;
        int n = 0;
        for (int i = 0; i < 120; ++i) n += t.advance(*s.world, dt);
        CHECK(n >= 119 && n <= 120);
        CHECK(steps(s) == static_cast<std::uint64_t>(n));
    }
    // Câmera lenta 1/4: 120 quadros ≈ 30 passos; 1/10 ≈ 12.
    {
        Scene s = makeFallingBoxScene();
        TimeControl t;
        t.setScaleIndex(2);
        int n = 0;
        for (int i = 0; i < 120; ++i) n += t.advance(*s.world, dt);
        CHECK(n >= 29 && n <= 31);
        Scene s2 = makeFallingBoxScene();
        TimeControl t2;
        t2.setScaleIndex(3);
        int n2 = 0;
        for (int i = 0; i < 120; ++i) n2 += t2.advance(*s2.world, dt);
        CHECK(n2 >= 11 && n2 <= 13);
    }
    // Pausado: nada avança, nem o α; a pose não muda.
    {
        Scene s = makeFallingBoxScene();
        TimeControl t;
        for (int i = 0; i < 10; ++i) t.advance(*s.world, dt);
        const Vector3 p0 = s.world->getBody(s.focus).getWorldCenterOfMass();
        const std::uint64_t n0 = steps(s);
        const Real a0 = t.renderAlpha(*s.world);
        t.setPaused(true);
        for (int i = 0; i < 60; ++i) CHECK(t.advance(*s.world, dt) == 0);
        CHECK(steps(s) == n0);
        CHECK((s.world->getBody(s.focus).getWorldCenterOfMass() - p0).length() == Real(0));
        CHECK(t.renderAlpha(*s.world) == a0);

        // Passo a passo: cada pedido = exatamente 1 passo físico; pedidos repetidos contam um só.
        t.requestStep();
        t.requestStep();
        CHECK(t.advance(*s.world, dt) == 1);
        CHECK(steps(s) == n0 + 1);
        CHECK(t.advance(*s.world, dt) == 0);                  // o pedido foi consumido
        CHECK(t.renderAlpha(*s.world) == Real(1));            // mostra o estado novo
        for (int i = 0; i < 5; ++i) { t.requestStep(); t.advance(*s.world, dt); }
        CHECK(steps(s) == n0 + 6);
        // A queda continua no passo a passo: y só diminui.
        const Real y1 = s.world->getBody(s.focus).getWorldCenterOfMass().y;
        t.requestStep();
        t.advance(*s.world, dt);
        CHECK(s.world->getBody(s.focus).getWorldCenterOfMass().y < y1);
        // Continuar: roda de novo e o α volta a ser o do acumulador.
        t.setPaused(false);
        CHECK(t.advance(*s.world, dt) >= 0);
        CHECK(t.renderAlpha(*s.world) == s.world->getAlpha());
    }
    // Pedido de passo com a simulação rodando é ignorado (não "sobra" para depois da pausa).
    {
        Scene s = makeFallingBoxScene();
        TimeControl t;
        t.requestStep();
        t.advance(*s.world, dt);
        t.setPaused(true);
        const std::uint64_t n0 = steps(s);
        CHECK(t.advance(*s.world, dt) == 0 && steps(s) == n0);
        // Pedido pendente some ao continuar e ao trocar/reiniciar a cena.
        t.requestStep();
        t.setPaused(false);
        t.setPaused(true);
        CHECK(t.advance(*s.world, dt) == 0);
        t.requestStep();
        t.onSceneReset();
        CHECK(t.advance(*s.world, dt) == 0);
        CHECK(t.paused());                                    // reiniciar não muda a pausa
    }
    // Passo manual == world.step() puro (determinismo).
    {
        Scene a = makeStackScene(), b = makeStackScene();
        TimeControl t;
        t.setPaused(true);
        for (int i = 0; i < 40; ++i) {
            t.requestStep();
            t.advance(*a.world, dt);
            b.world->step();
        }
        bool same = true;
        for (std::size_t i = 0; i < a.bodies.size(); ++i)
            same = same && (a.world->getBody(a.bodies[i].id).getWorldCenterOfMass() -
                            b.world->getBody(b.bodies[i].id).getWorldCenterOfMass()).length() == Real(0);
        CHECK(same);
    }
    // Entradas inválidas.
    {
        Scene s = makeFallingBoxScene();
        TimeControl t;
        const std::uint64_t n0 = steps(s);
        CHECK(t.advance(*s.world, std::numeric_limits<Real>::quiet_NaN()) == 0);
        CHECK(t.advance(*s.world, Real(-1)) == 0);
        CHECK(steps(s) == n0);
    }
}

static void testBodyEditor() {
    using namespace physics;
    const Real dt = Real(1) / Real(60);
    Scene s = makeFallingBoxScene();
    PhysicsWorld& w = *s.world;
    const BodyId id = s.focus;
    const BodyId ground = s.bodies.front().id;

    // --- leitura ----------------------------------------------------------------------------------------------
    BodyParams p;
    CHECK(readParams(w, id, p));
    CHECK(std::fabs(p.mass - Real(1)) < Real(1e-5) && std::fabs(p.friction - Real(0.5)) < Real(1e-6) &&
          std::fabs(p.restitution - Real(0.3)) < Real(1e-6) && p.allowSleep && p.awake && std::fabs(p.gravityScale - Real(1)) < Real(1e-6));
    BodyParams untouched;
    untouched.mass = Real(-7);
    CHECK(!readParams(w, kInvalidBodyId, untouched) && untouched.mass == Real(-7));

    // --- massa: a inércia acompanha; o impulso rende 1/m ---------------------------------------------------------
    const Real i0 = w.getBody(id).getMassData().inertia(0, 0);
    CHECK(setMass(w, id, Real(4)));
    CHECK(std::fabs(w.getBody(id).getMass() - Real(4)) < Real(1e-4));
    CHECK(std::fabs(w.getBody(id).getMassData().invMass - Real(0.25)) < Real(1e-4));
    CHECK(std::fabs(w.getBody(id).getMassData().inertia(0, 0) / i0 - Real(4)) < Real(1e-3));   // cubo homogêneo: I ∝ m
    w.getBody(id).setLinearVelocity(Vector3(0, 0, 0));
    w.getBody(id).applyLinearImpulse(Vector3(4, 0, 0));
    CHECK(std::fabs(w.getBody(id).getLinearVelocity().x - Real(1)) < Real(1e-4));
    CHECK(setMass(w, id, Real(1e6)) && std::fabs(w.getBody(id).getMass() - limits::kMaxMass) < Real(1e-3));
    CHECK(setMass(w, id, Real(-3)) && std::fabs(w.getBody(id).getMass() - limits::kMinMass) < Real(1e-5));
    CHECK(!setMass(w, id, std::numeric_limits<Real>::quiet_NaN()));
    CHECK(std::fabs(w.getBody(id).getMass() - limits::kMinMass) < Real(1e-5));   // NaN não mudou nada
    setMass(w, id, Real(1));

    // --- atrito, restituição, amortecimentos, gravidade (com limites) ----------------------------------------------
    CHECK(setFriction(w, id, Real(5)) && std::fabs(w.getBody(id).getFriction() - Real(2)) < Real(1e-6));
    CHECK(setFriction(w, id, Real(-1)) && w.getBody(id).getFriction() == Real(0));
    CHECK(setRestitution(w, id, Real(2)) && std::fabs(w.getBody(id).getRestitution() - Real(1)) < Real(1e-6));
    CHECK(setLinearDamping(w, id, Real(99)) && std::fabs(w.getBody(id).getLinearDamping() - limits::kMaxDamping) < Real(1e-6));
    CHECK(setAngularDamping(w, id, Real(-1)) && w.getBody(id).getAngularDamping() == Real(0));
    CHECK(setGravityScale(w, id, Real(99)) && std::fabs(w.getBody(id).getGravityScale() - limits::kMaxGravityScale) < Real(1e-6));
    CHECK(setGravityScale(w, id, Real(-99)) && std::fabs(w.getBody(id).getGravityScale() + limits::kMaxGravityScale) < Real(1e-6));
    CHECK(!setFriction(w, id, std::numeric_limits<Real>::infinity()));
    CHECK(setAllowSleep(w, id, false) && !w.getBody(id).getAllowSleep());
    setAllowSleep(w, id, true);
    setFriction(w, id, Real(0.5));
    setRestitution(w, id, Real(0.3));
    setLinearDamping(w, id, Real(0));

    // escala de gravidade 0: a caixa não cai; negativa: sobe.
    setGravityScale(w, id, Real(0));
    w.getBody(id).setLinearVelocity(Vector3(0, 0, 0));
    const Real y0 = w.getBody(id).getWorldCenterOfMass().y;
    for (int i = 0; i < 60; ++i) w.update(dt);
    CHECK(std::fabs(w.getBody(id).getWorldCenterOfMass().y - y0) < Real(0.01));
    setGravityScale(w, id, Real(-1));
    for (int i = 0; i < 60; ++i) w.update(dt);
    CHECK(w.getBody(id).getWorldCenterOfMass().y > y0 + Real(1));
    setGravityScale(w, id, Real(1));

    // --- posição: teletransporte sem rastro; orientação e velocidades ficam -----------------------------------------
    const Quaternion qBefore = w.getBody(id).getOrientation();
    const Vector3 vBefore = w.getBody(id).getLinearVelocity();
    CHECK(setPosition(w, id, Vector3(2, 7, -1)));
    CHECK(w.getBody(id).getTransform().position.nearlyEqual(Vector3(2, 7, -1), Real(1e-5)));
    CHECK(w.getInterpolatedTransform(id).position.nearlyEqual(Vector3(2, 7, -1), Real(1e-4)));   // render imediato
    CHECK(w.getBody(id).getOrientation().nearlyEqual(qBefore, Real(1e-6)));
    CHECK(w.getBody(id).getLinearVelocity().nearlyEqual(vBefore, Real(1e-6)));
    CHECK(!setPosition(w, id, Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0)));

    // --- orientação por eixo-ângulo e reset ---------------------------------------------------------------------------
    CHECK(setOrientationAxisAngle(w, id, Vector3(0, 2, 0), Real(1)));    // eixo não unitário é normalizado
    const Quaternion q = w.getBody(id).getOrientation();
    CHECK(std::fabs(q.w - std::cos(Real(0.5))) < Real(1e-5) && std::fabs(q.y - std::sin(Real(0.5))) < Real(1e-5) &&
          std::fabs(q.x) < Real(1e-6) && std::fabs(q.norm() - Real(1)) < Real(1e-5));
    CHECK(w.getBody(id).getTransform().position.nearlyEqual(Vector3(2, 7, -1), Real(1e-5)));   // posição intacta
    const Quaternion keep = w.getBody(id).getOrientation();
    CHECK(!setOrientationAxisAngle(w, id, Vector3(0, 0, 0), Real(1)));
    CHECK(!setOrientationAxisAngle(w, id, Vector3(1, 0, 0), std::numeric_limits<Real>::quiet_NaN()));
    CHECK(w.getBody(id).getOrientation().nearlyEqual(keep, Real(1e-7)));
    CHECK(resetOrientation(w, id));
    CHECK(w.getBody(id).getOrientation().nearlyEqual(Quaternion::identity(), Real(1e-6)));

    // --- velocidades, acordar/dormir -------------------------------------------------------------------------------------
    CHECK(setLinearVelocity(w, id, Vector3(1, 2, 3)) && setAngularVelocity(w, id, Vector3(-1, 0, 4)));
    CHECK(w.getBody(id).getLinearVelocity().nearlyEqual(Vector3(1, 2, 3), Real(1e-6)));
    CHECK(w.getBody(id).getAngularVelocity().nearlyEqual(Vector3(-1, 0, 4), Real(1e-6)));
    CHECK(!setLinearVelocity(w, id, Vector3(0, std::numeric_limits<Real>::infinity(), 0)));
    CHECK(zeroVelocities(w, id));
    CHECK(w.getBody(id).getLinearVelocity().isZero() && w.getBody(id).getAngularVelocity().isZero());
    CHECK(setAwake(w, id, false) && !w.getBody(id).isAwake());
    CHECK(setAwake(w, id, true) && w.getBody(id).isAwake());

    // --- estáticos e ids inválidos: nada muda ----------------------------------------------------------------------------------
    // V14: posição/orientação/material valem também para estáticos; massa, velocidades e sono continuam só para dinâmicos.
    CHECK(!setMass(w, ground, Real(2)) && !setAwake(w, ground, false) && !zeroVelocities(w, ground) &&
          !setGravityScale(w, ground, Real(1)));
    {
        const Real f0 = w.getBody(ground).getFriction();
        CHECK(setFriction(w, ground, Real(1)) && setFriction(w, ground, f0));
        CHECK(setPosition(w, ground, Vector3(0, 9, 0)) && setPosition(w, ground, Vector3(0, Real(-0.5), 0)));
        CHECK(setOrientationAxisAngle(w, ground, Vector3(0, 1, 0), Real(1)) && resetOrientation(w, ground));
    }
    CHECK(w.getBody(ground).getTransform().position.nearlyEqual(Vector3(0, Real(-0.5), 0), Real(1e-6)));
    CHECK(!setMass(w, kInvalidBodyId, Real(2)) && !setPosition(w, kInvalidBodyId, Vector3(0, 0, 0)) && !setAwake(w, 9999, true));

    // --- editar no meio da simulação não desestabiliza: pilha com o topo 50 vezes mais pesado --------------------------------
    {
        Scene st = makeStackScene();
        for (int i = 0; i < 30; ++i) st.world->update(dt);
        CHECK(setMass(*st.world, st.focus, Real(50)));
        CHECK(setFriction(*st.world, st.focus, Real(1.5)));
        bool finite = true;
        for (int i = 0; i < 240; ++i) {
            st.world->update(dt);
            finite = finite && st.world->getBody(st.focus).getWorldCenterOfMass().isFinite() &&
                     std::fabs(st.world->getBody(st.focus).getOrientation().norm() - Real(1)) < Real(1e-3);
        }
        CHECK(finite);
        CHECK(st.world->getBody(st.focus).getWorldCenterOfMass().y > Real(1.0));   // não afundou nas caixas de baixo
    }
}

static void testWorldEditor() {
    using namespace physics;
    const Real dt = Real(1) / Real(60);

    // --- captura dos padrões -----------------------------------------------------------------------------------
    Scene s = makeStackScene();
    PhysicsWorld& w = *s.world;
    const WorldParams def = captureWorld(w);
    CHECK(std::fabs(def.gravity.y + Real(9.81)) < Real(0.01) && def.gravity.x == Real(0));
    CHECK(std::fabs(def.fixedDt - dt) < Real(1e-6));
    CHECK(def.velocityIterations >= 1 && def.positionIterations >= 0 && def.warmStarting && def.sleepEnabled);
    CHECK(def.sleepTime > Real(0) && def.linearSlop >= Real(0));

    // --- setters: limites e entradas inválidas -------------------------------------------------------------------
    CHECK(setVelocityIterations(w, 9999) && w.solverSettings().velocityIterations == worldlimits::kMaxVelocityIterations);
    CHECK(setVelocityIterations(w, -3) && w.solverSettings().velocityIterations == 1);
    CHECK(setPositionIterations(w, 0) && w.solverSettings().positionIterations == 0);
    CHECK(setPositionIterations(w, 99) && w.solverSettings().positionIterations == worldlimits::kMaxPositionIterations);
    CHECK(setLinearSlop(w, Real(5)) && std::fabs(w.solverSettings().linearSlop - worldlimits::kMaxLinearSlop) < Real(1e-9));
    CHECK(setLinearSlop(w, Real(-1)) && w.solverSettings().linearSlop == Real(0));
    CHECK(!setLinearSlop(w, std::numeric_limits<Real>::quiet_NaN()));
    CHECK(setVelocitySlop(w, Real(50)) && std::fabs(w.solverSettings().velocitySlop - worldlimits::kMaxVelocitySlop) < Real(1e-9));
    CHECK(setSleepTime(w, Real(0)) && std::fabs(w.sleepSettings().timeToSleep - worldlimits::kMinSleepTime) < Real(1e-9));
    CHECK(!setSleepTime(w, std::numeric_limits<Real>::infinity()));
    setWarmStarting(w, false);
    CHECK(!w.solverSettings().warmStarting);
    CHECK(!setWorldGravity(w, Vector3(0, std::numeric_limits<Real>::quiet_NaN(), 0)));
    CHECK(setWorldGravity(w, Vector3(1000, -1000, 0)));
    CHECK(std::fabs(w.getGravity().x - worldlimits::kMaxGravity) < Real(1e-6) && std::fabs(w.getGravity().y + worldlimits::kMaxGravity) < Real(1e-6));
    CHECK(setWorldFixedStep(w, Real(1) / Real(120)) && std::fabs(w.getFixedTimeStep() - Real(1) / Real(120)) < Real(1e-7));
    CHECK(!setWorldFixedStep(w, Real(0)) && !setWorldFixedStep(w, Real(5)));

    // --- ida e volta: aplicar os padrões restaura tudo -------------------------------------------------------------
    applyWorld(w, def);
    const WorldParams back = captureWorld(w);
    CHECK(back.gravity.nearlyEqual(def.gravity, Real(1e-6)) && std::fabs(back.fixedDt - def.fixedDt) < Real(1e-7) &&
          back.velocityIterations == def.velocityIterations && back.positionIterations == def.positionIterations &&
          std::fabs(back.linearSlop - def.linearSlop) < Real(1e-9) && std::fabs(back.velocitySlop - def.velocitySlop) < Real(1e-9) &&
          back.warmStarting == def.warmStarting && back.sleepEnabled == def.sleepEnabled &&
          std::fabs(back.sleepTime - def.sleepTime) < Real(1e-9));

    // --- efeitos físicos ---------------------------------------------------------------------------------------------------
    // Gravidade zero: a caixa em queda livre não cai; gravidade lunar: cai mais devagar.
    {
        Scene a = makeFallingBoxScene();
        a.world->getBody(a.focus).setAngularVelocity(Vector3(0, 0, 0));
        const Real y0 = a.world->getBody(a.focus).getWorldCenterOfMass().y;
        setWorldGravity(*a.world, Vector3(0, 0, 0));
        for (int i = 0; i < 60; ++i) a.world->update(dt);
        CHECK(std::fabs(a.world->getBody(a.focus).getWorldCenterOfMass().y - y0) < Real(0.01));
        setWorldGravity(*a.world, Vector3(0, Real(-1.62), 0));
        for (int i = 0; i < 60; ++i) a.world->update(dt);
        const Real drop = y0 - a.world->getBody(a.focus).getWorldCenterOfMass().y;
        CHECK(std::fabs(drop - Real(0.5) * Real(1.62)) < Real(0.1));   // y = g t²/2 em 1 s
    }
    // Δt em runtime: 1 s de tempo de render continua sendo 1 s simulado.
    {
        Scene a = makeFallingBoxScene(), b = makeFallingBoxScene();
        a.world->getBody(a.focus).setAngularVelocity(Vector3(0, 0, 0));
        b.world->getBody(b.focus).setAngularVelocity(Vector3(0, 0, 0));
        CHECK(setWorldFixedStep(*b.world, Real(1) / Real(240)));
        for (int i = 0; i < 60; ++i) { a.world->update(dt); b.world->update(dt); }
        CHECK(std::fabs(a.world->getBody(a.focus).getWorldCenterOfMass().y - b.world->getBody(b.focus).getWorldCenterOfMass().y) < Real(0.15));
    }
    // Sleeping: com ele ligado a pilha dorme; desligado ninguém dorme; desligar acorda quem dormia.
    {
        Scene a = makeStackScene();
        for (int i = 0; i < 360; ++i) a.world->update(dt);
        CHECK(a.world->getStats().sleepingBodies > 0);
        setSleepEnabled(*a.world, false);
        CHECK(a.world->getStats().sleepingBodies == 0);                       // acordou todos
        for (int i = 0; i < 360; ++i) a.world->update(dt);
        CHECK(a.world->getStats().sleepingBodies == 0);                       // e ninguém dorme mais
        setSleepEnabled(*a.world, true);
        for (int i = 0; i < 360; ++i) a.world->update(dt);
        CHECK(a.world->getStats().sleepingBodies > 0);
    }
    // Menos iterações = mais penetração: pilha com 1 iteração afunda mais que com 20 (ambas finitas).
    {
        auto lowestGap = [&](int iters) {
            Scene a = makeStackScene();
            setVelocityIterations(*a.world, iters);
            setPositionIterations(*a.world, 0);
            Real minY = Real(100);
            for (int i = 0; i < 180; ++i) {
                a.world->update(dt);
                minY = std::min(minY, a.world->getBody(a.focus).getWorldCenterOfMass().y);
            }
            return a.world->getBody(a.focus).getWorldCenterOfMass().isFinite() ? minY : Real(-1);
        };
        const Real few = lowestGap(1), many = lowestGap(20);
        CHECK(few > Real(0) && many > Real(0));
        CHECK(many >= few - Real(1e-4));   // 20 iterações nunca afunda mais que 1
    }

    // --- profiling em runtime e estatísticas ------------------------------------------------------------------------
    {
        Scene a = makePyramidScene();
        for (int i = 0; i < 60; ++i) a.world->update(dt);
        a.world->setProfilingEnabled(true);
        for (int i = 0; i < 5; ++i) a.world->update(dt);
        const WorldStats st = a.world->getStats();
        CHECK(st.bodies == 16 && st.timings.totalUs > 0.0 && st.pairs > 0 && st.touchingPairs > 0 && st.contactPoints > 0);
        CHECK(st.timings.collisionUs >= 0.0 && st.timings.constraintsUs >= 0.0);
    }

    // --- debug draw: tabela e cada flag gerando linhas ------------------------------------------------------------------
    {
        const std::vector<DebugFlagInfo>& tab = debugFlagTable();
        CHECK(tab.size() == 9);
        std::uint32_t all = 0;
        bool distinct = true;
        for (const DebugFlagInfo& f : tab) {
            distinct = distinct && (all & f.flag) == 0 && f.label != nullptr && f.label[0] != '\0';
            all |= f.flag;
        }
        CHECK(distinct && all == kDebugAll);

        DebugLineBuffer buf;
        Scene stack = makeStackScene();
        for (int i = 0; i < 60; ++i) stack.world->update(dt);
        CHECK(buildDebugLines(*stack.world, 0, buf) == 0 && buf.size() == 0);
        for (std::uint32_t f : {kDebugShapes, kDebugAABBs, kDebugFatAABBs, kDebugContactPoints, kDebugContactNormals,
                                kDebugCenterOfMass, kDebugBodyAxes})
            CHECK(buildDebugLines(*stack.world, f, buf) > 0);
        Scene chain = makeChainScene();
        for (int i = 0; i < 30; ++i) chain.world->update(dt);
        CHECK(buildDebugLines(*chain.world, kDebugJoints, buf) > 0);
        Scene rain = makeRainScene();
        for (int i = 0; i < 40; ++i) rain.world->update(dt);
        CHECK(buildDebugLines(*rain.world, kDebugVelocities, buf) > 0);
        const std::size_t nAll = buildDebugLines(*rain.world, kDebugAll, buf);
        CHECK(nAll > buildDebugLines(*rain.world, kDebugShapes, buf));
        // Pose interpolada x pose atual: com α ≈ 0,5 as linhas dos corpos em queda diferem; com α = 0 não.
        {
            Scene fall = makeRainScene();
            for (int i = 0; i < 40; ++i) fall.world->update(dt);
            fall.world->update(dt * Real(0.5));   // sobra metade de um passo no acumulador (α ≈ 0,5)
            DebugLineBuffer a, b;
            const std::size_t na = buildDebugLines(*fall.world, kDebugShapes, a, true);
            const std::size_t nb = buildDebugLines(*fall.world, kDebugShapes, b, false);
            CHECK(na == nb && na > 0);
            float maxDiff = 0.0f;
            for (std::size_t i = 0; i < std::min(na, nb); ++i) maxDiff = std::max(maxDiff, std::fabs(a.lines()[i].ay - b.lines()[i].ay));
            CHECK(fall.world->getAlpha() > Real(0.3) && fall.world->getAlpha() < Real(0.7));
            CHECK(maxDiff > 0.005f);
        }
        // Reaproveita a capacidade: chamar de novo não acumula.
        CHECK(buildDebugLines(*rain.world, kDebugAll, buf) == nAll && buf.size() == nAll);
    }
}

static void testCameraDistance() {
    OrbitCamera c;
    c.setDistance(30.0f);
    CHECK(near(c.getDistance(), 30.0f, 1e-6f));
    c.setDistance(1e9f);
    CHECK(near(c.getDistance(), OrbitCamera::kMaxDistance, 1e-6f));
    c.setDistance(-5.0f);
    CHECK(near(c.getDistance(), OrbitCamera::kMinDistance, 1e-6f));
    c.setDistance(std::numeric_limits<float>::quiet_NaN());
    CHECK(near(c.getDistance(), OrbitCamera::kMinDistance, 1e-6f));   // NaN ignorado
}

static void testObjectSpawner() {
    using namespace physics;
    Scene sc = makeFallingBoxScene();
    const std::size_t n0 = sc.bodies.size();
    const BodyId focus0 = sc.focus;

    // Todos os tipos: cria, registra, tem massa e orientacao unitaria.
    for (int k = 0; k < kSpawnKindCount; ++k) {
        SpawnParams p;
        p.kind = static_cast<SpawnKind>(k);
        p.position = Vector3(Real(k), 5, 0);
        p.orientation = Quaternion(Real(2), 0, 0, 0);   // nao unitario
        const BodyId id = spawnBody(sc, p);
        CHECK(id != kInvalidBodyId);
        CHECK(sc.world->isValidBody(id));
        CHECK(sc.bodies.back().id == id);
        CHECK(std::fabs(sc.world->getBody(id).getOrientation().norm() - Real(1)) < Real(1e-5));
        CHECK(sc.world->getBody(id).getMass() > Real(0));
        CHECK(std::string(spawnKindName(p.kind)).size() > 2);
        const SceneBody& sb = sc.bodies.back();
        CHECK((p.kind == SpawnKind::Box) == (sb.kind == ShapeKind::Box));
        CHECK((p.kind == SpawnKind::Sphere) == (sb.kind == ShapeKind::Sphere));
        CHECK((k >= 2 && k <= 5) == (sb.poly != nullptr));
        CHECK((p.kind == SpawnKind::Capsule) == (sb.kind == ShapeKind::Capsule));
        CHECK((p.kind == SpawnKind::Cylinder) == (sb.kind == ShapeKind::Cylinder));
        CHECK(k < 6 || (sb.radius > 0.0f && sb.halfHeight > 0.0f));
    }
    CHECK(sc.bodies.size() == n0 + kSpawnKindCount);
    CHECK(sc.focus == focus0);

    // Limites: tamanho, massa, velocidades.
    SpawnParams big;
    big.size = Real(1000);
    big.mass = Real(1e9);
    big.velocity = Vector3(1000, 0, 0);
    big.angularVelocity = Vector3(0, 1000, 0);
    big.position = Vector3(1e9, 0, 0);
    const BodyId b = spawnBody(sc, big);
    CHECK(b != kInvalidBodyId);
    const RigidBody& rb = sc.world->getBody(b);
    CHECK(rb.getMass() <= spawnlimits::kMaxMass * Real(1.0001));
    CHECK(rb.getLinearVelocity().length() <= spawnlimits::kMaxLinearSpeed * Real(1.0001));
    CHECK(rb.getAngularVelocity().length() <= spawnlimits::kMaxAngularSpeed * Real(1.0001));
    CHECK(rb.getTransform().position.x <= spawnlimits::kMaxCoordinate * Real(1.0001));
    SpawnParams tiny;
    tiny.size = Real(0);
    tiny.mass = Real(0);
    CHECK(spawnBody(sc, tiny) != kInvalidBodyId);

    // Entradas invalidas: nada muda.
    const std::size_t n1 = sc.bodies.size();
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    SpawnParams bad;
    bad.mass = nan;
    CHECK(spawnBody(sc, bad) == kInvalidBodyId);
    bad = SpawnParams();
    bad.position = Vector3(0, nan, 0);
    CHECK(spawnBody(sc, bad) == kInvalidBodyId);
    bad = SpawnParams();
    bad.orientation = Quaternion(0, 0, 0, 0);
    CHECK(spawnBody(sc, bad) == kInvalidBodyId);
    bad = SpawnParams();
    bad.velocity = Vector3(nan, 0, 0);
    CHECK(spawnBody(sc, bad) == kInvalidBodyId);
    CHECK(sc.bodies.size() == n1);

    // Orientacao.
    bool ok = false;
    Quaternion q = orientationFromQuaternion(0, 0, 3, 4, &ok);
    CHECK(ok && std::fabs(q.norm() - Real(1)) < Real(1e-5));
    q = orientationFromQuaternion(0, 0, 0, 0, &ok);
    CHECK(!ok && std::fabs(q.w - Real(1)) < Real(1e-6));
    q = orientationFromQuaternion(nan, 0, 0, 0, &ok);
    CHECK(!ok);
    q = orientationFromAxisAngle(Vector3(0, 2, 0), Real(3.14159265358979), &ok);
    CHECK(ok && std::fabs(q.y - Real(1)) < Real(1e-4) && std::fabs(q.w) < Real(1e-4));
    q = orientationFromAxisAngle(Vector3(0, 0, 0), Real(1), &ok);
    CHECK(!ok && std::fabs(q.w - Real(1)) < Real(1e-6));

    // Clique: raio de cima para o chao -> ponto acima do chao.
    Vector3 pt;
    CHECK(spawnPointFromRay(sc, Vector3(5, 10, 5), Vector3(0, -1, 0), Real(1), pt));
    CHECK(pt.y > Real(0.5) && pt.y < Real(2) && std::fabs(pt.x - Real(5)) < Real(1e-3));
    CHECK(!spawnPointFromRay(sc, Vector3(0, 10, 0), Vector3(0, 1, 0), Real(1), pt));   // para o ceu
    CHECK(!spawnPointFromRay(sc, Vector3(0, 10, 0), Vector3(0, 0, 0), Real(1), pt));
    CHECK(!spawnPointFromRay(sc, Vector3(nan, 10, 0), Vector3(0, -1, 0), Real(1), pt));

    // Lancamento.
    SpawnParams lp;
    lp.velocity = Vector3(99, 99, 99);   // deve ser substituida
    const BodyId l = launchFrom(sc, lp, Vector3(0, 3, 10), Vector3(0, 0, -5), Real(12));
    CHECK(l != kInvalidBodyId);
    const RigidBody& lb = sc.world->getBody(l);
    CHECK(std::fabs(lb.getLinearVelocity().z + Real(12)) < Real(1e-4) && std::fabs(lb.getLinearVelocity().x) < Real(1e-4));
    CHECK(std::fabs(lb.getTransform().position.z - Real(8)) < Real(1e-4));
    CHECK(launchFrom(sc, lp, Vector3(0, 3, 10), Vector3(0, 0, 0), Real(12)) == kInvalidBodyId);
    CHECK(launchFrom(sc, lp, Vector3(0, 3, 10), Vector3(0, 0, -1), nan) == kInvalidBodyId);

    // A simulacao segue estavel com os novos corpos.
    for (int i = 0; i < 300; ++i) sc.world->step();
    bool finite = true;
    for (const SceneBody& s : sc.bodies)
        finite = finite && sc.world->getBody(s.id).getTransform().position.isFinite();
    CHECK(finite);
}

static void testContactTracker() {
    using namespace physics;
    // Intensidade e tinta.
    CHECK(ContactTracker::intensityFromSpeed(0.1f) == 0.0f);
    CHECK(near(ContactTracker::intensityFromSpeed(4.0f), 0.5f));
    CHECK(near(ContactTracker::intensityFromSpeed(100.0f), 1.0f));
    CHECK(ContactTracker::intensityFromSpeed(std::numeric_limits<float>::quiet_NaN()) == 0.0f);
    const Rgb base{200, 100, 50};
    const Rgb same = stateTint(base, false, 0.0f);
    CHECK(same.r == 200 && same.g == 100 && same.b == 50);
    const Rgb sl = stateTint(base, true, 0.0f);
    CHECK(sl.b > base.b && sl.r < base.r);                       // adormecido: mais azul, menos vermelho
    const Rgb fl = stateTint(base, false, 1.0f);
    CHECK(fl.r > base.r && fl.g > base.g && fl.b > base.b);      // flash: mais claro
    const Rgb nanT = stateTint(base, false, std::numeric_limits<float>::quiet_NaN());
    CHECK(nanT.r == 200);

    // Eventos sinteticos.
    ContactTracker t;
    ContactEvent e;
    e.a = 1; e.b = 2; e.point = Vector3(1, 0, 0);
    e.approachSpeed = Real(0.1);
    t.onContactBegin(e);
    CHECK(t.begins() == 1 && t.flashOf(1) == 0.0f && t.markers().empty());   // repouso: conta, sem flash
    e.approachSpeed = Real(8);
    t.onContactBegin(e);
    CHECK(t.begins() == 2 && near(t.flashOf(1), 1.0f) && near(t.flashOf(2), 1.0f) && t.markers().size() == 1);
    CHECK(near(t.maxImpact(), 8.0f) && near(t.lastImpact(), 8.0f) && t.flashOf(3) == 0.0f);
    t.onContactEnd(e);
    CHECK(t.ends() == 1);
    t.update(0.2f);
    CHECK(t.flashOf(1) > 0.4f && t.flashOf(1) < 0.6f);
    t.update(std::numeric_limits<float>::quiet_NaN());
    t.update(-1.0f);
    CHECK(t.flashOf(1) > 0.4f && t.flashOf(1) < 0.6f);          // dt invalido: nada muda
    t.update(1.0f);
    CHECK(t.flashOf(1) == 0.0f && t.markers().empty());
    for (int i = 0; i < 100; ++i) t.onContactBegin(e);
    CHECK(t.markers().size() == ContactTracker::kMaxMarkers);
    t.reset();
    CHECK(t.begins() == 0 && t.markers().empty() && t.maxImpact() == 0.0f);

    // Com o motor: a caixa que cai bate no chao -> inicio de contato com impacto > 0.
    Scene sc = makeFallingBoxScene();
    ContactTracker tr;
    sc.world->setContactListener(&tr);
    for (int i = 0; i < 300; ++i) sc.world->step();
    CHECK(tr.begins() >= 1);
    CHECK(tr.maxImpact() > 1.0f);
    sc.world->setContactListener(nullptr);

    // Sleeping: a pilha em repouso adormece (cor azulada no viewer).
    Scene st = makeStackScene();
    for (int i = 0; i < 900; ++i) st.world->step();
    int sleeping = 0;
    for (const SceneBody& b : st.bodies)
        if (!b.isStatic && !st.world->isAwake(b.id)) ++sleeping;
    CHECK(sleeping >= 1);
    st.world->setContactListener(nullptr);
}

static physics::Real angleAbout(const physics::Quaternion& q, const physics::Vector3& axisUnit) {
    // Ângulo [rad] da rotação q em torno de um eixo fixo (componente do vetor do quatérnio nesse eixo).
    const physics::Real s = q.x * axisUnit.x + q.y * axisUnit.y + q.z * axisUnit.z;
    return physics::Real(2) * std::atan2(s, q.w);
}

static void testSceneFile() {
    using namespace physics;
    // Ida e volta de TODAS as cenas.
    for (std::size_t i = 0; i < sceneList().size(); ++i) {
        // (1) Cena recém-criada: o arquivo recria o MESMO mundo, então as trajetórias coincidem.
        {
            Scene a = makeScene(i);
            Scene b;
            std::string err;
            const bool ok = parseScene(serializeScene(a), b, err);
            if (!ok) std::printf("  erro cena %zu: %s\n", i + 1, err.c_str());
            CHECK(ok);
            if (ok) {
                Real maxDiff = 0;
                for (int k = 0; k < 180; ++k) {
                    a.world->step();
                    b.world->step();
                }
                for (std::size_t k = 0; k < a.bodies.size(); ++k)
                    maxDiff = std::max(maxDiff, (a.world->getBody(a.bodies[k].id).getTransform().position -
                                                 b.world->getBody(b.bodies[k].id).getTransform().position).length());
                if (maxDiff >= Real(0.01)) std::printf("  cena %zu: diferenca %.4f\n", i + 1, double(maxDiff));
                CHECK(maxDiff < Real(0.01));
            }
        }
        // (2) Cena em andamento (velocidades, contatos): o texto sobrevive à ida e volta, e a cena carregada é válida.
        Scene a = makeScene(i);
        for (int k = 0; k < 20; ++k) a.world->step();
        const std::string text = serializeScene(a);
        Scene b;
        std::string err;
        const bool ok = parseScene(text, b, err);
        CHECK(ok);
        if (!ok) continue;
        CHECK(serializeScene(b) == text);
        CHECK(b.bodies.size() == a.bodies.size() && b.links.size() == a.links.size());
        CHECK(b.name == a.name);
        CHECK(near(b.cameraTarget, a.cameraTarget) && near(b.cameraDistance, a.cameraDistance));
        CHECK((b.focus == kInvalidBodyId) == (a.focus == kInvalidBodyId));
        int jointsA = 0, jointsB = 0;
        a.world->forEachJoint([&](const JointConstraint&, BodyId, BodyId) { ++jointsA; });
        b.world->forEachJoint([&](const JointConstraint&, BodyId, BodyId) { ++jointsB; });
        CHECK(jointsA == jointsB);
        for (int k = 0; k < 120; ++k) b.world->step();   // a cena carregada roda sem NaN
        bool finite = true;
        for (const SceneBody& sb : b.bodies) finite = finite && b.world->getBody(sb.id).getTransform().position.isFinite();
        CHECK(finite);
        for (std::size_t k = 0; k < a.bodies.size(); ++k)   // cores, tipo e forma preservados
            CHECK(a.bodies[k].r == b.bodies[k].r && a.bodies[k].isStatic == b.bodies[k].isStatic && a.bodies[k].kind == b.bodies[k].kind);
    }

    // Quaternio e velocidades exatos na volta (corpo qualquer).
    {
        Scene a = makeFallingBoxScene();
        for (int k = 0; k < 37; ++k) a.world->step();
        Scene b;
        std::string err;
        CHECK(parseScene(serializeScene(a), b, err));
        const RigidBody& ra = a.world->getBody(a.focus);
        const RigidBody& rb = b.world->getBody(b.focus);
        CHECK(ra.getOrientation().w == rb.getOrientation().w && ra.getOrientation().z == rb.getOrientation().z);
        CHECK(ra.getAngularVelocity().y == rb.getAngularVelocity().y);
        CHECK(ra.getTransform().position.y == rb.getTransform().position.y);
    }

    // Arquivo de verdade em disco.
    {
        Scene a = makeDoorScene();
        const std::string dir = "viewer_smoke_scenes_tmp";
        std::string err;
        CHECK(saveSceneFile(a, dir + "/porta_teste.scene", err));
        CHECK(saveSceneFile(a, dir + "/a_outra.scene", err));
        const std::vector<std::string> names = listSceneFiles(dir);
        CHECK(names.size() == 2 && names[0] == "a_outra" && names[1] == "porta_teste");
        Scene b;
        CHECK(loadSceneFile(dir + "/porta_teste.scene", b, err));
        CHECK(b.bodies.size() == a.bodies.size());
        CHECK(!loadSceneFile(dir + "/nao_existe.scene", b, err) && !err.empty());
        CHECK(listSceneFiles("pasta_que_nao_existe_xyz").empty());
        std::filesystem::remove_all(dir);
    }

    // Nome de arquivo seguro.
    CHECK(sanitizeSceneFileName("Minha cena 2") == "Minha_cena_2");
    CHECK(sanitizeSceneFileName("../../etc/passwd") == "etcpasswd");
    CHECK(sanitizeSceneFileName("") == "cena");
    CHECK(sanitizeSceneFileName("///") == "cena");
    CHECK(sanitizeSceneFileName(std::string(200, 'a')).size() == 48);

    // Entradas inválidas: erro com mensagem, destino intacto.
    const std::string good =
        "physics_scene 1\nname Teste\nbody\n type dynamic\n shape box 0.5 0.5 0.5\n mass 1\n position 0 2 0\nend\n";
    {
        Scene t;
        std::string err;
        CHECK(parseScene(good, t, err) && t.bodies.size() == 1 && t.name == "Teste");
    }
    struct Bad { const char* what; std::string text; const char* expect; };
    auto withLine = [&](const std::string& extra) {   // insere uma linha dentro do corpo
        std::string t = good;
        t.insert(t.find("end\n"), extra);
        return t;
    };
    const Bad bads[] = {
        {"vazio", "", "vazio"},
        {"sem cabecalho", "body\nend\n", "physics_scene"},
        {"versao", "physics_scene 7\n", "versao"},
        {"junta sem corpos", "physics_scene 1\nname x\njoint ballsocket 1 2 0 0 0 0\n", "nao existe"},
        {"comando", "physics_scene 1\nxyz 1\n", "desconhecido"},
        {"body sem end", "physics_scene 1\nbody\n type static\n", "sem 'end'"},
        {"numero", withLine(" velocity 1 abc 0\n"), "numero invalido"},
        {"nan", withLine(" velocity 1 nan 0\n"), "numero invalido"},
        {"inf", withLine(" velocity 1 inf 0\n"), "numero invalido"},
        {"quaternio zero", withLine(" orientation 0 0 0 0\n"), "norma zero"},
        {"massa negativa", withLine(" mass -3\n"), "massa"},
        {"valores a mais", withLine(" velocity 1 2 3 4\n"), "espera"},
        {"chave", withLine(" cor 1 2 3\n"), "chave desconhecida"},
        {"restituicao", withLine(" material 0.5 7\n"), "restituicao"},
        {"sem shape", "physics_scene 1\nbody\n type static\nend\n", "sem 'shape'"},
        {"dinamico sem massa", "physics_scene 1\nbody\n type dynamic\n shape sphere 1\nend\n", "mass"},
        {"junta fora", good + "joint ballsocket 1 5 0 0 0 0\n", "nao existe"},
        {"junta em si", good + "joint ballsocket 1 1 0 0 0 0\n", "ele mesmo"},
        {"eixo nulo", "physics_scene 1\nbody\n type static\n shape box 1 1 1\nend\nbody\n type dynamic\n shape box 1 1 1\n mass 1\nend\n"
                      "joint hinge 1 2 0 0 0 0 0 0 0\n", "junta invalida"},
        {"poliedro concavo",
         "physics_scene 1\nbody\n type dynamic\n shape poly\n mass 1\n vertex 0 0 0\n vertex 1 0 0\n vertex 0 1 0\n vertex 0 0 1\n"
         " vertex 0.1 0.1 0.1\n face 0 1 2\n face 0 1 3\n face 0 2 3\n face 1 2 4\nend\n", "linha"},
        {"face com indice", "physics_scene 1\nbody\n type dynamic\n shape poly\n mass 1\n vertex 0 0 0\n vertex 1 0 0\n vertex 0 1 0\n vertex 0 0 1\n"
                            " face 0 1 2\n face 0 1 3\n face 0 2 3\n face 1 2 9\nend\n", "nao existe"},
        {"dois focus", good + "body\n type static\n shape box 1 1 1\n focus\nend\n" , "mais de um"},
        {"timestep", "physics_scene 1\ntimestep 50\n", "timestep"},
    };
    for (const Bad& bd : bads) {
        Scene t = makeFallingBoxScene();
        const std::size_t before = t.bodies.size();
        std::string err;
        std::string text = bd.text;
        if (std::string(bd.what) == "dois focus") text.insert(text.find("end\n"), " focus\n");
        const bool ok = parseScene(text, t, err);
        const bool matches = err.find(bd.expect) != std::string::npos;
        if (ok || !matches) std::printf("  caso '%s': ok=%d erro='%s'\n", bd.what, ok ? 1 : 0, err.c_str());
        CHECK(!ok);
        CHECK(matches);
        CHECK(t.bodies.size() == before && t.world != nullptr);   // destino intacto
    }
    {   // limites: corpos demais e arquivo grande
        std::string many = "physics_scene 1\n";
        for (int i = 0; i < scenefile::kMaxBodies + 1; ++i) many += "body\n type static\n shape box 1 1 1\nend\n";
        Scene t;
        std::string err;
        CHECK(!parseScene(many, t, err) && err.find("corpos demais") != std::string::npos);
        const std::string big(scenefile::kMaxBytes + 1, ' ');
        CHECK(!parseScene(big, t, err) && err.find("grande demais") != std::string::npos);
    }
    {   // comentários, CRLF, linhas em branco e nome com '#'
        std::string t = "physics_scene 1  # versao\r\n\r\nname Cena #1 com hash\r\n# comentario\r\nbody\r\n type static # chao\r\n shape box 5 0.5 5\r\nend\r\n";
        Scene s;
        std::string err;
        CHECK(parseScene(t, s, err) && s.name == "Cena #1 com hash" && s.bodies.size() == 1 && s.bodies[0].isStatic);
    }
}

static void testDockLayout() {
    using namespace viewer;
    DockLayout d;
    d.setWindow(1280, 720);
    // Padrão: colunas de 400 e 380; as três áreas cobrem a janela sem sobrepor.
    const RectF L = d.dock(DockSide::Left), R = d.dock(DockSide::Right), V = d.viewport();
    CHECK(near(L.w, 400.0f) && near(R.w, 380.0f) && near(L.y, DockLayout::kTopBar) && near(L.h, 720.0f - DockLayout::kTopBar));
    CHECK(near(L.x, 0.0f) && near(R.right(), 1280.0f));
    CHECK(near(V.x, L.w + DockLayout::kSplitter) && near(V.right(), R.x - DockLayout::kSplitter));
    CHECK(V.w > 400.0f && near(V.h, L.h));
    CHECK(near(d.splitter(DockSide::Left).x, L.right()) && near(d.splitter(DockSide::Right).right(), R.x));

    // Limites da largura: mínimo, fração máxima e viewport mínimo.
    d.setWidth(DockSide::Left, 10.0f);
    CHECK(near(d.leftWidth(), DockLayout::kMinDock));
    d.setWidth(DockSide::Left, 5000.0f);
    CHECK(d.leftWidth() <= 0.45f * 1280.0f + 0.01f);
    CHECK(d.viewport().w >= DockLayout::kMinViewport - 0.01f);
    d.setWidth(DockSide::Right, 5000.0f);
    CHECK(d.viewport().w >= DockLayout::kMinViewport - 0.01f);
    d.setWidth(DockSide::Left, std::numeric_limits<float>::quiet_NaN());   // ignorado
    CHECK(std::isfinite(d.leftWidth()));
    d.setWidth(DockSide::Left, 400.0f);
    d.setWidth(DockSide::Right, 380.0f);

    // Recolher: faixa de 28 px, sem borda; o viewport cresce; reabrir volta à largura escolhida.
    d.toggleCollapsed(DockSide::Right);
    CHECK(near(d.dock(DockSide::Right).w, DockLayout::kCollapsed) && d.splitter(DockSide::Right).w == 0.0f);
    CHECK(near(d.viewport().right(), 1280.0f - DockLayout::kCollapsed));
    d.toggleCollapsed(DockSide::Right);
    CHECK(near(d.dock(DockSide::Right).w, 380.0f));

    // Arrastar a borda esquerda: a largura segue o mouse (mantendo o ponto agarrado).
    DockSide side;
    const RectF sp = d.splitter(DockSide::Left);
    CHECK(d.hitSplitter(sp.x + 1.0f, 300.0f, side) && side == DockSide::Left);
    CHECK(d.hitSplitter(sp.x - 2.0f, 300.0f, side));            // tolerância
    CHECK(!d.hitSplitter(sp.x + 40.0f, 300.0f, side));
    CHECK(!d.hitSplitter(sp.x + 1.0f, 10.0f, side));            // sobre a barra do topo não conta
    d.beginDrag(DockSide::Left, sp.x + 3.0f);
    CHECK(d.dragging());
    d.dragTo(sp.x + 3.0f + 100.0f);
    CHECK(near(d.leftWidth(), 500.0f, 0.6f));
    d.dragTo(-500.0f);                                           // além da janela: fica no mínimo
    CHECK(near(d.leftWidth(), DockLayout::kMinDock));
    d.endDrag();
    CHECK(!d.dragging());
    // Borda direita: arrastar para a esquerda alarga a coluna.
    d.setWidth(DockSide::Right, 380.0f);
    const RectF spr = d.splitter(DockSide::Right);
    d.beginDrag(DockSide::Right, spr.x + 3.0f);
    d.dragTo(spr.x + 3.0f - 60.0f);
    CHECK(near(d.rightWidth(), 440.0f, 0.6f));
    d.endDrag();

    // Janela pequena: as colunas cedem ao viewport mínimo; nada fica negativo nem se sobrepõe.
    DockLayout s;
    for (float w : {2000.0f, 1000.0f, 900.0f, 700.0f, 400.0f, 100.0f, 1.0f}) {
        s.setWindow(w, 500);
        const RectF a = s.dock(DockSide::Left), b = s.dock(DockSide::Right), v = s.viewport();
        CHECK(a.w >= 0 && b.w >= 0 && v.w >= 1.0f);
        CHECK(a.right() <= v.x + 0.01f && v.right() <= b.x + 0.01f || w < 700.0f);
        CHECK(v.x >= 0 && v.right() <= w + 0.01f || w < 700.0f);
    }
    s.setWindow(std::numeric_limits<float>::quiet_NaN(), 500);   // ignorado
    CHECK(s.viewport().w >= 1.0f);

    // Arquivo de layout: ida e volta e entradas inválidas.
    DockLayout a;
    a.setWindow(1600, 900);
    a.setWidth(DockSide::Left, 450);
    a.setWidth(DockSide::Right, 360);
    a.setCollapsed(DockSide::Right, true);
    DockLayout b;
    b.setWindow(1600, 900);
    CHECK(b.parse(a.serialize()));
    CHECK(near(b.leftWidth(), 450) && near(b.rightWidth(), 360) && b.collapsed(DockSide::Right) && !b.collapsed(DockSide::Left));
    CHECK(b.serialize() == a.serialize());
    const char* badInputs[] = {"", "left 400\n", "viewer_layout 2\n", "viewer_layout 1\nleft nan\n", "viewer_layout 1\nleft 5\n",
                               "viewer_layout 1\nleft_collapsed 2\n", "viewer_layout 1\nfoo 1\n", "viewer_layout 1\nleft 400 500\n",
                               "viewer_layout 1\nright inf\n"};
    for (const char* bad : badInputs) {
        DockLayout t;
        t.setWindow(1600, 900);
        const std::string before = t.serialize();
        CHECK(!t.parse(bad));
        CHECK(t.serialize() == before);   // nada mudou
    }

    // Rolagem.
    ScrollState sc;
    sc.setSizes(1000, 400);
    CHECK(sc.scrollable() && near(sc.maxOffset(), 600.0f) && sc.offset() == 0.0f);
    sc.wheel(-1.0f);                                             // roda para baixo: desce o conteúdo
    CHECK(near(sc.offset(), ScrollState::kWheelStep));
    sc.wheel(+100.0f);
    CHECK(sc.offset() == 0.0f);                                  // limitado em cima
    sc.wheel(-1000.0f);
    CHECK(near(sc.offset(), 600.0f));                            // e embaixo
    sc.setSizes(500, 400);                                       // conteúdo encolheu: o deslocamento acompanha
    CHECK(near(sc.offset(), 100.0f));
    sc.setSizes(300, 400);
    CHECK(!sc.scrollable() && sc.offset() == 0.0f && sc.maxOffset() == 0.0f);
    sc.setSizes(2000, 500);
    CHECK(near(sc.thumbHeight(500), 125.0f));                    // fração visível 1/4 do trilho
    sc.setOffset(0);
    CHECK(sc.thumbTop(500) == 0.0f);
    sc.setOffset(sc.maxOffset());
    CHECK(near(sc.thumbTop(500) + sc.thumbHeight(500), 500.0f, 0.01f));
    sc.setOffset(0);
    sc.dragThumb(375.0f, 500);                                   // arrastar o polegar até o fim
    CHECK(near(sc.offset(), sc.maxOffset(), 0.5f));
    sc.jumpTo(0.0f, 500);
    CHECK(sc.offset() == 0.0f);
    sc.setSizes(100000, 100);                                    // polegar nunca menor que 24 px
    CHECK(sc.thumbHeight(500) >= 24.0f);
    sc.setOffset(std::numeric_limits<float>::quiet_NaN());       // ignorado
    CHECK(std::isfinite(sc.offset()));
    sc.setSizes(std::numeric_limits<float>::infinity(), 100);
    CHECK(std::isfinite(sc.maxOffset()));
}

static void testCollisionScenes() {
    using namespace physics;
    auto run = [](Scene& s, double seconds) { for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i) s.world->step(); };
    auto displaced = [](const Scene& s, const std::vector<Vector3>& before, Real dist) {
        int n = 0;
        for (std::size_t i = 0; i < s.bodies.size(); ++i)
            if (!s.bodies[i].isStatic && (s.world->getBody(s.bodies[i].id).getTransform().position - before[i]).length() > dist) ++n;
        return n;
    };
    auto snapshot = [](const Scene& s) {
        std::vector<Vector3> v;
        for (const SceneBody& b : s.bodies) v.push_back(s.world->getBody(b.id).getTransform().position);
        return v;
    };

    {   // Impacto: a bola chega com v ~ 14 m/s e desmonta a pirâmide.
        Scene s = makePyramidImpactScene();
        CHECK(s.world->getBody(s.focus).getLinearVelocity().x > Real(13.9));
        const std::vector<Vector3> b0 = snapshot(s);
        run(s, 6);
        CHECK(displaced(s, b0, Real(0.7)) >= 8);   // a maioria dos 15 blocos saiu do lugar
    }
    {   // Porta: gira SÓ em torno de Y (a dobradiça), e a bola a faz girar.
        Scene s = makeDoorScene();
        const BodyId door = s.focus;
        Real maxAngle = 0, maxOff = 0;
        for (int i = 0; i < 360; ++i) {
            s.world->step();
            const Quaternion q = s.world->getBody(door).getOrientation();
            maxAngle = std::max(maxAngle, std::fabs(angleAbout(q, Vector3(0, 1, 0))));
            maxOff = std::max(maxOff, std::max(std::fabs(q.x), std::fabs(q.z)));
            // O centro da porta fica a 1 m da dobradiça (anel de raio 1 em torno do eixo Y).
            const Vector3 c = s.world->getBody(door).getTransform().position;
            CHECK(std::fabs(std::sqrt(c.x * c.x + c.z * c.z) - Real(1)) < Real(0.02) && std::fabs(c.y - Real(1.3)) < Real(0.02));
        }
        CHECK(maxAngle > Real(1));      // girou mais de 1 rad
        CHECK(maxOff < Real(0.02));     // nenhuma rotação fora do eixo Y
    }
    {   // Gangorra: a ponta atingida desce, a prancha gira só em torno de Z.
        Scene s = makeSeesawScene();
        const BodyId plank = s.focus;
        run(s, 1.5);   // (V19: com a resistência à rolagem a bola rola para a outra ponta mais tarde e a prancha volta: o sinal só vale logo após o impacto)
        {
            const Quaternion q1 = s.world->getBody(plank).getOrientation();
            CHECK(angleAbout(q1, Vector3(0, 0, 1)) > Real(0.1));
        }
        run(s, 2.5);
        const Quaternion q = s.world->getBody(plank).getOrientation();
        CHECK(std::fabs(angleAbout(q, Vector3(0, 0, 1))) > Real(0.1));
        CHECK(std::fabs(q.x) < Real(0.02) && std::fabs(q.y) < Real(0.02));
        CHECK(true);   // anti-horário em Z: a ponta direita sobe, a esquerda (onde a bola caiu) desce
    }
    {   // Martelo: oscila em Z e derruba a torre.
        Scene s = makeHammerScene();
        const std::vector<Vector3> b0 = snapshot(s);
        const BodyId hammer = s.focus;
        Real maxOff = 0;
        for (int i = 0; i < 600; ++i) {
            s.world->step();
            const Quaternion q = s.world->getBody(hammer).getOrientation();
            maxOff = std::max(maxOff, std::max(std::fabs(q.x), std::fabs(q.y)));
        }
        CHECK(maxOff < Real(0.02));
        int moved = 0;
        for (std::size_t i = 0; i < s.bodies.size(); ++i)
            if (!s.bodies[i].isStatic && s.bodies[i].id != hammer &&
                (s.world->getBody(s.bodies[i].id).getTransform().position - b0[i]).length() > Real(0.5)) ++moved;
        CHECK(moved >= 1);
    }
    {   // Bilhar: a bola branca espalha o triângulo; ninguém sai da mesa; energia nunca cresce.
        Scene s = makeBilliardsScene();
        const std::vector<Vector3> b0 = snapshot(s);
        auto energy = [&]() {
            Real e = 0;
            for (const SceneBody& b : s.bodies) if (!b.isStatic) e += s.world->getBody(b.id).getKineticEnergy();
            return e;
        };
        const Real e0 = energy();
        run(s, 3);
        CHECK(displaced(s, b0, Real(0.5)) >= 6);
        CHECK(energy() <= e0 * Real(1.001));
        for (const SceneBody& b : s.bodies)
            CHECK(s.world->getBody(b.id).getTransform().position.y > Real(-1));
    }
}

// =================================================================================================
// V14 — editor de cena: cena vazia, estáticos, dimensões, juntas
// =================================================================================================
static void testSceneEditor() {
    using namespace physics;
    // --- cena vazia / só chão -------------------------------------------------------------------------
    {
        Scene e = makeEmptyScene(false);
        CHECK(e.world && e.bodies.empty() && e.links.empty() && e.focus == kInvalidBodyId);
        Scene g = makeEmptyScene(true);
        CHECK(g.bodies.size() == 1 && g.bodies[0].isStatic && g.world->getBody(g.bodies[0].id).isStatic());
        // vazia: pode simular sem nenhum corpo
        for (int i = 0; i < 10; ++i) e.world->update(Real(1) / Real(60));
        // salvar/carregar cena vazia
        const std::string txt = serializeScene(e);
        Scene back;
        std::string err;
        CHECK(parseScene(txt, back, err) && back.bodies.empty());
        CHECK(serializeScene(back) == txt);
    }
    // --- estático + dimensões da caixa ------------------------------------------------------------------
    Scene s = makeEmptyScene(true);
    {
        SpawnParams p;
        p.isStatic = true;
        p.size = Real(1);
        p.boxScale = Vector3(6, 3, Real(0.4));   // parede 6 x 3 x 0,4
        p.position = Vector3(0, Real(1.5), -5);
        p.mass = 7; p.velocity = Vector3(5, 5, 5);
        const BodyId w = spawnBody(s, p);
        CHECK(w != kInvalidBodyId);
        const RigidBody& b = s.world->getBody(w);
        CHECK(b.isStatic() && !b.isDynamic());
        CHECK(near(b.getLinearVelocity().length(), 0));
        const SceneBody& sb = s.bodies.back();
        CHECK(sb.isStatic && sb.kind == ShapeKind::Box);
        CHECK(near(sb.halfExtents.x, 3.0f) && near(sb.halfExtents.y, 1.5f) && near(sb.halfExtents.z, 0.2f));
        // limites: escala e aresta
        SpawnParams q;
        q.size = Real(4);
        q.boxScale = Vector3(1000, Real(0.00001), 5);   // escala -> [0,1; 10]; aresta -> [0,05; 40]
        const BodyId r = spawnBody(s, q);
        CHECK(r != kInvalidBodyId);
        const SceneBody& sr = s.bodies.back();
        CHECK(near(sr.halfExtents.x, 20.0f) && near(sr.halfExtents.y, 0.2f) && near(sr.halfExtents.z, 10.0f));
        // não finito: recusa e não muda a cena
        const std::size_t n = s.bodies.size();
        SpawnParams bad;
        bad.boxScale = Vector3(std::numeric_limits<Real>::quiet_NaN(), 1, 1);
        CHECK(spawnBody(s, bad) == kInvalidBodyId && s.bodies.size() == n);
        // padrão (sem escala): caixa cúbica como na V8
        SpawnParams c;
        c.size = Real(2);
        spawnBody(s, c);
        CHECK(near(s.bodies.back().halfExtents.x, 1.0f) && near(s.bodies.back().halfExtents.z, 1.0f));
        // estático de outra forma
        SpawnParams sp;
        sp.kind = SpawnKind::Pyramid; sp.isStatic = true; sp.position = Vector3(3, 1, 3);
        const BodyId pyr = spawnBody(s, sp);
        CHECK(pyr != kInvalidBodyId && s.world->getBody(pyr).isStatic() && s.bodies.back().isStatic);
        // rampa: caixa estática inclinada 20 graus; uma esfera rola por ela
        SpawnParams ramp;
        ramp.isStatic = true;
        ramp.boxScale = Vector3(8, Real(0.3), 3);
        ramp.position = Vector3(-4, Real(1.5), 4);
        ramp.orientation = orientationFromAxisAngle(Vector3(0, 0, 1), Real(-0.35));
        CHECK(spawnBody(s, ramp) != kInvalidBodyId);
    }
    // --- BodyEditor em estáticos ------------------------------------------------------------------------
    {
        const BodyId wall = s.bodies[1].id;   // a parede
        CHECK(setPosition(*s.world, wall, Vector3(1, 2, 3)));
        CHECK(near(s.world->getBody(wall).getTransform().position.y, 2));
        CHECK(setOrientationAxisAngle(*s.world, wall, Vector3(0, 1, 0), Real(0.5)));
        const Quaternion& q = s.world->getBody(wall).getOrientation();
        CHECK(near(q.w, std::cos(Real(0.25))) && near(q.y, std::sin(Real(0.25))));
        CHECK(resetOrientation(*s.world, wall) && near(s.world->getBody(wall).getOrientation().w, 1));
        CHECK(setFriction(*s.world, wall, Real(0.9)) && near(s.world->getBody(wall).getFriction(), 0.9));
        CHECK(setRestitution(*s.world, wall, Real(0.5)));
        // o resto continua só para dinâmicos
        CHECK(!setMass(*s.world, wall, 3) && !setLinearVelocity(*s.world, wall, Vector3(1, 0, 0)) &&
              !zeroVelocities(*s.world, wall) && !setAwake(*s.world, wall, true) && !setGravityScale(*s.world, wall, 1));
        CHECK(!setPosition(*s.world, kInvalidBodyId, Vector3()));
        CHECK(!setPosition(*s.world, wall, Vector3(std::numeric_limits<Real>::infinity(), 0, 0)));
    }
    // --- deleteBody com allowStatic / pickBody com estáticos ---------------------------------------------
    {
        Scene t = makeEmptyScene(true);
        const BodyId ground = t.bodies[0].id;
        CHECK(!deleteBody(t, ground));
        CHECK(!pickBody(t, Vector3(0, 5, 0), Vector3(0, -1, 0)).hit);
        const PickResult pk = pickBody(t, Vector3(0, 5, 0), Vector3(0, -1, 0), Real(200), true);
        CHECK(pk.hit && pk.body == ground && near(pk.point.y, 0));
        CHECK(deleteBody(t, ground, true) && t.bodies.empty());
    }
    // --- JointEditor -------------------------------------------------------------------------------------
    {
        Scene t = makeEmptyScene(true);
        auto box = [&](Real x, Real y, bool st) {
            SpawnParams p;
            p.position = Vector3(x, y, 0);
            p.isStatic = st;
            return spawnBody(t, p);
        };
        const BodyId A = box(0, 3, false), B = box(2, 3, false), S = box(5, 3, true);
        CHECK(A != kInvalidBodyId && B != kInvalidBodyId && S != kInvalidBodyId);
        CHECK(jointCount(t) == 0);
        std::string err;
        JointParams jp;
        jp.kind = JointKind::Hinge; jp.a = A; jp.b = B; jp.anchor = Vector3(1, 3, 0); jp.axis = Vector3(0, 0, 2);
        CHECK(createJoint(t, jp, &err) && jointCount(t) == 1 && t.links.size() == 1);
        jp.kind = JointKind::BallSocket;
        CHECK(createJoint(t, jp) && jointCount(t) == 2);
        jp.kind = JointKind::Distance;
        CHECK(createJoint(t, jp) && jointCount(t) == 3);
        // para o mundo: cria pivô estático sem forma
        const std::size_t before = t.bodies.size();
        JointParams w;
        w.kind = JointKind::Hinge; w.a = A; w.b = kInvalidBodyId; w.anchor = Vector3(0, 6, 0); w.axis = Vector3(1, 0, 0);
        CHECK(createJoint(t, w) && t.bodies.size() == before + 1 && jointCount(t) == 4);
        const SceneBody& pivot = t.bodies.back();
        CHECK(pivot.isStatic && t.world->getBody(pivot.id).getShape() == nullptr);
        CHECK(near(t.world->getBody(pivot.id).getTransform().position.y, 6));
        // distância ao mundo: comprimento = distância atual
        JointParams dw;
        dw.kind = JointKind::Distance; dw.a = B; dw.anchor = Vector3(2, 8, 0);
        CHECK(createJoint(t, dw) && jointCount(t) == 5);
        {
            bool found = false;
            t.world->forEachJoint([&](const JointConstraint& j, BodyId, BodyId) {
                if (j.getType() == ConstraintType::Distance)
                    found = true;
            });
            CHECK(found);
        }
        // falhas não mudam a cena
        const std::size_t nb = t.bodies.size(), nl = t.links.size();
        const int nj = jointCount(t);
        JointParams bad = jp;
        bad.a = kInvalidBodyId; CHECK(!createJoint(t, bad, &err) && !err.empty());
        bad = jp; bad.b = 99999; CHECK(!createJoint(t, bad));
        bad = jp; bad.b = bad.a; CHECK(!createJoint(t, bad));
        bad = jp; bad.kind = JointKind::Hinge; bad.axis = Vector3(); CHECK(!createJoint(t, bad));
        bad = jp; bad.anchor = Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0); CHECK(!createJoint(t, bad));
        bad = jp; bad.anchor = Vector3(5000, 0, 0); CHECK(!createJoint(t, bad));
        bad = jp; bad.kind = JointKind::Distance; bad.b = kInvalidBodyId; bad.anchor = t.world->getBody(A).getWorldCenterOfMass();
        CHECK(!createJoint(t, bad, &err));   // pontos coincidentes
        JointParams ss; ss.a = S; ss.b = kInvalidBodyId; ss.anchor = Vector3(0, 0, 0); ss.kind = JointKind::BallSocket;
        CHECK(!createJoint(t, ss));          // estático ligado ao mundo: nada a simular
        CHECK(t.bodies.size() == nb && t.links.size() == nl && jointCount(t) == nj);
        // a simulação com as juntas segue estável
        for (int i = 0; i < 240; ++i) t.world->update(Real(1) / Real(60));
        CHECK(t.world->getBody(A).getTransform().position.isFinite());
        CHECK(t.world->getBody(A).getTransform().position.y > -1);   // preso: não caiu pelo chão
        // apagar: da última para a primeira; pivôs órfãos somem
        CHECK(destroyLastJoint(t) && jointCount(t) == 4);
        const std::size_t before2 = t.bodies.size();
        CHECK(destroyLastJoint(t) && jointCount(t) == 3 && t.bodies.size() == before2 - 1);   // junta do pivô
        while (jointCount(t) > 0) CHECK(destroyLastJoint(t));
        CHECK(t.links.empty() && !destroyLastJoint(t));
        // salvar com juntas e pivô: ida e volta
        CHECK(createJoint(t, w) && createJoint(t, jp));
        const std::string txt = serializeScene(t);
        Scene back;
        CHECK(parseScene(txt, back, err));
        CHECK(jointCount(back) == jointCount(t) && back.bodies.size() == t.bodies.size());
        // apagar um corpo remove as juntas dele
        CHECK(jointCount(t) == 2);
        CHECK(deleteBody(t, A) && jointCount(t) == 0);
    }
    // --- pointOnRay ---------------------------------------------------------------------------------------
    {
        Scene t = makeEmptyScene(true);
        Vector3 pt;
        CHECK(pointOnRay(t, Vector3(1, 5, 2), Vector3(0, -1, 0), pt) && near(pt.x, 1) && near(pt.y, 0) && near(pt.z, 2));
        Scene e = makeEmptyScene(false);   // sem chão: cai no plano y = 0
        CHECK(pointOnRay(e, Vector3(0, 4, 0), Vector3(1, -1, 0), pt) && near(pt.y, 0) && near(pt.x, 4));
        CHECK(!pointOnRay(e, Vector3(0, 4, 0), Vector3(0, 1, 0), pt));
        CHECK(!pointOnRay(e, Vector3(0, 4, 0), Vector3(), pt));
    }
}

// =================================================================================================
// V15a — fecho convexo, leitor .obj/.mtl, forma de colisão do modelo
// =================================================================================================
namespace {

struct Lcg {   // gerador determinístico simples
    unsigned s;
    explicit Lcg(unsigned seed) : s(seed) {}
    double next() { s = s * 1664525u + 1013904223u; return static_cast<double>((s >> 8) & 0xFFFFFF) / 16777216.0; }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

/// Cubo de aresta 1 com centro em (cx, cy, cz).
std::vector<physics::Vector3> cubeCorners(double cx = 0, double cy = 0, double cz = 0, double half = 0.5) {
    std::vector<physics::Vector3> v;
    for (int i = 0; i < 8; ++i)
        v.emplace_back(static_cast<physics::Real>(cx + ((i & 1) ? half : -half)), static_cast<physics::Real>(cy + ((i & 2) ? half : -half)),
                       static_cast<physics::Real>(cz + ((i & 4) ? half : -half)));
    return v;
}

int hullEdges(const viewer::ConvexHull& h) {
    int e = 0;
    for (const auto& f : h.faces) e += static_cast<int>(f.size());
    return e / 2;
}

/// Todo ponto está atrás (ou sobre) de todos os planos das faces, e a normal aponta para fora do centro.
bool hullContains(const viewer::ConvexHull& h, const std::vector<physics::Vector3>& pts, double tol) {
    physics::Vector3 c;
    for (const auto& v : h.vertices) c += v;
    c = c / static_cast<physics::Real>(h.vertices.size());
    for (const auto& f : h.faces) {
        physics::Vector3 n;   // normal de Newell
        for (std::size_t i = 0; i < f.size(); ++i) {
            const physics::Vector3& a = h.vertices[static_cast<std::size_t>(f[i])];
            const physics::Vector3& b = h.vertices[static_cast<std::size_t>(f[(i + 1) % f.size()])];
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
        if (n.lengthSquared() <= 0) return false;
        n = n.normalized();
        const double d = static_cast<double>(n.dot(h.vertices[static_cast<std::size_t>(f[0])]));
        if (static_cast<double>(n.dot(c)) > d) return false;   // normal para dentro
        for (const auto& p : pts)
            if (static_cast<double>(n.dot(p)) - d > tol) return false;
    }
    return true;
}

std::string cubeObj(const char* extra = "") {
    return std::string("# cubo\nv -1 -1 -1\nv 1 -1 -1\nv 1 1 -1\nv -1 1 -1\nv -1 -1 1\nv 1 -1 1\nv 1 1 1\nv -1 1 1\n"
                       "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n") + extra +
           "f 1/1/1 4/4/1 3/3/1 2/2/1\nf 5/1 6/2 7/3 8/4\nf 1/1 2/2 6/3 5/4\nf 4 8 7 3\nf 2/1 3/2 7/3 6/4\nf 1 5 8 4\n";
}

/// Toro (côncavo) com malha fechada: R = raio maior, r = raio do tubo.
std::string torusObj(double R, double r, int nu, int nv) {
    std::string t = "o toro\n";
    char buf[96];
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            const double u = 6.283185307179586 * i / nu, v = 6.283185307179586 * j / nv;
            std::snprintf(buf, sizeof buf, "v %.6f %.6f %.6f\n", (R + r * std::cos(v)) * std::cos(u), r * std::sin(v), (R + r * std::cos(v)) * std::sin(u));
            t += buf;
        }
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            const int a = i * nv + j + 1, b = ((i + 1) % nu) * nv + j + 1, c = ((i + 1) % nu) * nv + (j + 1) % nv + 1, d = i * nv + (j + 1) % nv + 1;
            std::snprintf(buf, sizeof buf, "f %d %d %d %d\n", a, d, c, b);
            t += buf;
        }
    return t;
}

} // namespace

static void testModelImport() {
    using namespace physics;
    using viewer::ConvexHull;
    std::string err;

    // --- fecho: cubo + muitos pontos internos -----------------------------------------------------------------------
    {
        Lcg rng(7);
        std::vector<Vector3> pts = cubeCorners();
        for (int i = 0; i < 1000; ++i)
            pts.emplace_back(static_cast<Real>(rng.range(-0.49, 0.49)), static_cast<Real>(rng.range(-0.49, 0.49)), static_cast<Real>(rng.range(-0.49, 0.49)));
        ConvexHull h;
        CHECK(viewer::buildConvexHull(pts, h, &err));
        CHECK(h.vertices.size() == 8 && h.faces.size() == 6 && h.triangleCount == 12);
        for (const auto& f : h.faces) CHECK(f.size() == 4);   // faces quadradas refeitas
        CHECK(static_cast<int>(h.vertices.size()) - hullEdges(h) + static_cast<int>(h.faces.size()) == 2);   // Euler
        for (std::size_t i = 0; i < h.vertices.size(); ++i) CHECK(h.vertices[i].nearlyEqual(pts[static_cast<std::size_t>(h.sourceIndex[i])], Real(0)));
        CHECK(hullContains(h, pts, 1e-6));
        const PolyhedronShape shape(h.vertices, h.faces);   // o motor aceita
        CHECK(near(static_cast<float>(shape.getVolume()), 1.0f, 1e-4f));
        ConvexHull tri;
        CHECK(viewer::buildConvexHull(pts, tri, &err, false) && tri.faces.size() == 12);
        for (const auto& f : tri.faces) CHECK(f.size() == 3);
        ConvexHull again;   // determinismo
        CHECK(viewer::buildConvexHull(pts, again, &err) && again.faces == h.faces && again.sourceIndex == h.sourceIndex);
    }
    // --- cubo girado (coplanaridade com arredondamento), grande, pequeno e longe da origem --------------------------
    {
        const double halfs[3] = {0.0005, 0.5, 300.0};
        for (double half : halfs) {
            const Quaternion q = Quaternion::fromAxisAngle(Vector3(1, 2, 3).normalized(), Real(0.7));
            std::vector<Vector3> pts;
            for (const Vector3& c : cubeCorners(0, 0, 0, half)) pts.push_back(q.rotate(c) + Vector3(Real(half * 3), Real(-half * 2), Real(half)));
            ConvexHull h;
            CHECK(viewer::buildConvexHull(pts, h, &err));
            CHECK(h.vertices.size() == 8 && h.faces.size() == 6);
            const PolyhedronShape shape(h.vertices, h.faces);
            CHECK(shape.getVolume() > 0);
        }
        std::vector<Vector3> far = cubeCorners(1000, -2000, 500);   // longe da origem
        ConvexHull h;
        CHECK(viewer::buildConvexHull(far, h, &err) && h.vertices.size() == 8);
    }
    // --- pontos sobre a esfera: todos são vértices, fecho convexo e aceito pelo motor ----------------------------------
    {
        Lcg rng(11);
        std::vector<Vector3> pts;
        for (int i = 0; i < 600; ++i) {
            Vector3 d(static_cast<Real>(rng.range(-1, 1)), static_cast<Real>(rng.range(-1, 1)), static_cast<Real>(rng.range(-1, 1)));
            if (d.lengthSquared() < Real(0.01)) continue;
            pts.push_back(d.normalized() * Real(2));
        }
        ConvexHull h;
        CHECK(viewer::buildConvexHull(pts, h, &err));
        CHECK(h.vertices.size() > 100 && static_cast<int>(h.vertices.size()) - hullEdges(h) + static_cast<int>(h.faces.size()) == 2);
        CHECK(hullContains(h, pts, 1e-4));
        const PolyhedronShape shape(h.vertices, h.faces);
        const double v = static_cast<double>(shape.getVolume());
        CHECK(v > 0.9 * 4.0 / 3.0 * 3.14159265 * 8.0 && v < 4.0 / 3.0 * 3.14159265 * 8.0 + 1e-3);
    }
    // --- nuvem aleatória (muitos pontos internos) e pontos duplicados --------------------------------------------------
    {
        Lcg rng(3);
        std::vector<Vector3> pts;
        for (int i = 0; i < 3000; ++i)
            pts.emplace_back(static_cast<Real>(rng.range(-3, 3)), static_cast<Real>(rng.range(-1, 1)), static_cast<Real>(rng.range(-2, 2)));
        for (int i = 0; i < 200; ++i) pts.push_back(pts[static_cast<std::size_t>(i)]);   // duplicados
        ConvexHull h;
        CHECK(viewer::buildConvexHull(pts, h, &err) && hullContains(h, pts, 1e-4));
        CHECK(static_cast<int>(h.vertices.size()) - hullEdges(h) + static_cast<int>(h.faces.size()) == 2);
        const PolyhedronShape shape(h.vertices, h.faces);
        CHECK(shape.getVolume() > 0);
    }
    // --- entradas degeneradas ----------------------------------------------------------------------------------------------
    {
        ConvexHull h;
        const Real nan = std::numeric_limits<Real>::quiet_NaN();
        CHECK(!viewer::buildConvexHull({}, h, &err) && err.find("4 pontos") != std::string::npos);
        CHECK(!viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0)}, h, &err));
        CHECK(!viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(1, 1, 0)}, h, &err) &&
              err.find("coplanares") != std::string::npos);                       // quadrado plano
        CHECK(!viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0), Vector3(3, 0, 0)}, h, &err) &&
              err.find("colineares") != std::string::npos);
        CHECK(!viewer::buildConvexHull({Vector3(1, 1, 1), Vector3(1, 1, 1), Vector3(1, 1, 1), Vector3(1, 1, 1)}, h, &err));
        CHECK(!viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0)}, h, &err));
        CHECK(!viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, nan)}, h, &err) &&
              err.find("finito") != std::string::npos);
        CHECK(h.vertices.empty() && h.faces.empty());   // falha deixa a saída limpa
        // quase coplanar (espessura 1e-9) também é recusado
        CHECK(!viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(1, 1, Real(1e-9))}, h, &err));
        // um tetraedro mínimo funciona
        CHECK(viewer::buildConvexHull({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)}, h, &err) && h.faces.size() == 4);
    }

    // --- leitor .obj ---------------------------------------------------------------------------------------------------------------
    {
        viewer::ObjModel m;
        CHECK(viewer::parseObj(cubeObj("usemtl vermelho\nmtllib cubo.mtl\n"), m, err));
        CHECK(m.positions.size() == 8 && m.uvs.size() == 4 && m.triangles.size() == 12);   // 6 quadrados em leque
        CHECK(m.materials.size() == 1 && m.materials[0].name == "vermelho" && m.mtlLibs.size() == 1 && m.mtlLibs[0] == "cubo.mtl");
        CHECK(m.triangles[0].material == 0 && m.triangles[0].t[0] == 0 && m.triangles[2].t[0] == 0);
        Vec3f lo, hi;
        CHECK(viewer::objBounds(m, lo, hi) && near(lo, Vec3f{-1, -1, -1}) && near(hi, Vec3f{1, 1, 1}));
        viewer::ObjModel neg;   // índices negativos, CRLF, comentário na linha, espaços extras
        CHECK(viewer::parseObj("v 0 0 0 # origem\r\nv 1 0 0\r\nv 0 1 0\r\nf -3 -2 -1\r\n", neg, err) && neg.triangles.size() == 1 && neg.triangles[0].p[0] == 0 &&
              neg.triangles[0].p[2] == 2);
        viewer::ObjModel poly;   // polígono de 5 lados -> 3 triângulos
        CHECK(viewer::parseObj("v 0 0 0\nv 1 0 0\nv 2 1 0\nv 1 2 0\nv 0 1 0\nf 1 2 3 4 5\n", poly, err) && poly.triangles.size() == 3);
        struct Bad { const char* what; std::string text; const char* expect; };
        const Bad bads[] = {
            {"vazio", "", "nao tem vertices"},
            {"so vertices", "v 0 0 0\nv 1 0 0\nv 0 1 0\n", "nao tem faces"},
            {"v curto", "v 1 2\n", "linha 1: 'v' precisa"},
            {"numero", "v 1 2 abc\n", "numero invalido"},
            {"nan", "v 1 2 nan\n", "numero invalido"},
            {"inf", "v 1 inf 3\n", "numero invalido"},
            {"enorme", "v 1 2 1e9\n", "fora do limite"},
            {"face curta", "v 0 0 0\nv 1 0 0\nf 1 2\n", "'f' precisa"},
            {"indice zero", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n", "fora do intervalo"},
            {"indice alto", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n", "fora do intervalo"},
            {"negativo alto", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -4 1 2\n", "fora do intervalo"},
            {"uv alto", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1 2/1 3/1\n", "textura"},
            {"indice lixo", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 x\n", "indice invalido"},
            {"comando", "v 0 0 0\nxyz 1\n", "comando desconhecido"},
            {"usemtl vazio", "v 0 0 0\nusemtl\n", "sem nome"},
        };
        for (const Bad& b : bads) {
            viewer::ObjModel x;
            x.positions.push_back({1, 2, 3});   // deve ser limpo na falha
            const bool ok = viewer::parseObj(b.text, x, err);
            CHECK(!ok && err.find(b.expect) != std::string::npos && x.positions.empty());
            if (ok || err.find(b.expect) == std::string::npos) std::printf("  caso '%s': '%s'\n", b.what, err.c_str());
        }
        viewer::ObjModel line3;   // número da linha na mensagem
        CHECK(!viewer::parseObj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n", line3, err) && err.find("linha 4") != std::string::npos);
    }
    // --- leitor .mtl e carregamento do disco ---------------------------------------------------------------------------
    {
        std::vector<viewer::ObjMaterial> mats;
        CHECK(viewer::parseMtl("# x\nnewmtl a\nKd 0.5 0.25 2\nmap_Kd tex.png\nnewmtl b b\nKa 1 1 1\nmap_Kd -s 1 1 1 sub/t2.png\n", mats, err));
        CHECK(mats.size() == 2 && mats[0].diffuseMap == "tex.png" && near(mats[0].kd[0], 0.5f) && near(mats[0].kd[2], 1.0f));   // Kd limitado a [0,1]
        CHECK(mats[1].name == "b b" && mats[1].diffuseMap == "sub/t2.png");
        std::vector<viewer::ObjMaterial> bad;
        CHECK(!viewer::parseMtl("Kd 1 1 1\n", bad, err) && err.find("antes de 'newmtl'") != std::string::npos);
        CHECK(!viewer::parseMtl("newmtl a\nKd 1 1\n", bad, err));

        namespace fs = std::filesystem;
        const fs::path dir = fs::temp_directory_path() / "viewer_v15a_test";
        fs::remove_all(dir);
        fs::create_directories(dir);
        auto put = [&](const char* name, const std::string& text) { std::FILE* f = std::fopen((dir / name).string().c_str(), "wb"); std::fputs(text.c_str(), f); std::fclose(f); };
        put("cubo.obj", cubeObj("mtllib cubo.mtl\nusemtl madeira\n"));
        put("cubo.mtl", "newmtl madeira\nKd 0.8 0.6 0.4\nmap_Kd madeira.png\nnewmtl nao_usado\nKd 1 0 0\n");
        viewer::ObjModel m;
        std::string warn;
        CHECK(viewer::loadObjFile((dir / "cubo.obj").string(), m, err, &warn));
        CHECK(m.name == "cubo" && m.materials.size() == 1 && m.materials[0].diffuseMap == "madeira.png" && near(m.materials[0].kd[1], 0.6f) && warn.empty());
        put("inseguro.obj", cubeObj("mtllib ../fora.mtl\nusemtl m\n"));
        CHECK(viewer::loadObjFile((dir / "inseguro.obj").string(), m, err, &warn) && warn.find("inseguro") != std::string::npos);
        put("semmtl.obj", cubeObj("mtllib nao_existe.mtl\nusemtl m\n"));
        CHECK(viewer::loadObjFile((dir / "semmtl.obj").string(), m, err, &warn) && warn.find("sem nao_existe.mtl") != std::string::npos && m.materials[0].diffuseMap.empty());
        put("texinsegura.obj", cubeObj("mtllib t.mtl\nusemtl m\n"));
        put("t.mtl", "newmtl m\nmap_Kd C:/x.png\n");
        CHECK(viewer::loadObjFile((dir / "texinsegura.obj").string(), m, err, &warn) && m.materials[0].diffuseMap.empty() && warn.find("textura recusada") != std::string::npos);
        put("ruim.obj", "v 0 0 0\nf 1 1 1\nxx\n");
        CHECK(!viewer::loadObjFile((dir / "ruim.obj").string(), m, err) && err.find("ruim.obj") != std::string::npos && err.find("linha 3") != std::string::npos);
        CHECK(!viewer::loadObjFile((dir / "nao_existe.obj").string(), m, err) && err.find("nao encontrado") != std::string::npos);
        CHECK(!viewer::loadObjFile(dir.string(), m, err));   // pasta
        fs::remove_all(dir);
    }

    // --- forma de colisão -------------------------------------------------------------------------------------------------------------
    {
        viewer::ObjModel cube;
        CHECK(viewer::parseObj(cubeObj(), cube, err));
        CHECK(near(viewer::autoScale(cube, 1.0f), 0.5f));
        CHECK(near(viewer::autoScale(cube, 0.0f), 1.0f) && near(viewer::autoScale(cube, std::numeric_limits<float>::quiet_NaN()), 1.0f));
        viewer::ImportedShape is;
        CHECK(viewer::makeImportedShape(cube, 0.5f, is, err));   // cubo de aresta 1 m
        CHECK(is.shape && is.hullVertices == 8 && is.hullFaces == 6 && is.hullTriangles == 12 && is.modelVertices == 8);
        CHECK(near(static_cast<float>(is.shape->getVolume()), 1.0f, 1e-4f) && !is.concave);
        CHECK(near(static_cast<float>(is.shape->getCentroid().length()), 0.0f, 1e-5f));
        CHECK(near(static_cast<float>(is.modelVolume), 1.0f, 1e-3f));
        // modelo deslocado: o deslocamento (offset) leva o centroide para a origem
        viewer::ObjModel moved = cube;
        for (Vec3f& v : moved.positions) { v.x += 10.0f; v.y -= 4.0f; }
        CHECK(viewer::makeImportedShape(moved, 0.5f, is, err));
        CHECK(near(static_cast<float>(is.offset.x), 5.0f, 1e-3f) && near(static_cast<float>(is.offset.y), -2.0f, 1e-3f) && near(static_cast<float>(is.shape->getCentroid().length()), 0.0f, 1e-5f));
        // limites
        CHECK(!viewer::makeImportedShape(cube, 0.0f, is, err) && !is.shape);
        CHECK(!viewer::makeImportedShape(cube, std::numeric_limits<float>::quiet_NaN(), is, err));
        CHECK(!viewer::makeImportedShape(cube, 1000.0f, is, err) && err.find("tamanho final") != std::string::npos);
        CHECK(!viewer::makeImportedShape(cube, 0.001f, is, err));
        viewer::ObjModel flat;   // placa sem volume
        CHECK(viewer::parseObj("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n", flat, err));
        CHECK(!viewer::makeImportedShape(flat, 1.0f, is, err) && err.find("fecho convexo") != std::string::npos);
        // toro: côncavo; o fecho é um disco grosso
        viewer::ObjModel torus;
        CHECK(viewer::parseObj(torusObj(1.0, 0.3, 24, 12), torus, err) && torus.triangles.size() == 24 * 12 * 2);
        CHECK(viewer::makeImportedShape(torus, 1.0f, is, err) && is.concave && is.modelVolume < 0.9 * is.hullVolume);
        // o buraco do toro é preenchido: um objeto no centro NÃO cai pelo buraco (colide com o fecho)
        PhysicsWorld w;
        RigidBodyDesc gd;
        gd.type = BodyType::Static;
        gd.shape = std::make_shared<BoxShape>(Vector3(10, Real(0.5), 10));
        gd.transform = Transform(Vector3(0, Real(-0.5), 0));
        w.createBody(gd);
        RigidBodyDesc td;
        td.shape = is.shape;
        td.mass = 2;
        td.transform = Transform(Vector3(0, 3, 0));
        const BodyId tid = w.createBody(td);
        RigidBodyDesc sd;
        sd.shape = std::make_shared<SphereShape>(Real(0.2));
        sd.mass = 1;
        sd.transform = Transform(Vector3(0, 6, 0));
        const BodyId sid = w.createBody(sd);
        for (int i = 0; i < 600; ++i) w.update(Real(1) / Real(60));
        CHECK(w.getBody(sid).getTransform().position.y > Real(0.5));   // apoiada no "toro cheio" (altura do tubo 0,3·2 = 0,6)
        CHECK(w.getBody(tid).getTransform().position.isFinite() && w.getBody(tid).getTransform().position.y > Real(0.1));
        // cubo importado e girado cai e repousa sobre uma face (fusão coplanar -> contato estável)
        viewer::ObjModel big;
        CHECK(viewer::parseObj(cubeObj(), big, err) && viewer::makeImportedShape(big, 1.0f, is, err));   // aresta 2 m
        PhysicsWorld w2;
        w2.createBody(gd);
        RigidBodyDesc cd;
        cd.shape = is.shape;
        cd.mass = 4;
        cd.transform = Transform(Vector3(0, 5, 0), Quaternion::fromAxisAngle(Vector3(1, 1, 0.3f).normalized(), Real(0.8)));
        const BodyId cid = w2.createBody(cd);
        for (int i = 0; i < 600; ++i) w2.update(Real(1) / Real(60));
        CHECK(near(static_cast<float>(w2.getBody(cid).getTransform().position.y), 1.0f, 0.03f));
        CHECK(w2.getBody(cid).getLinearVelocity().length() < Real(0.05));
    }
    // --- os modelos de exemplo da pasta models/ -------------------------------------------------------------------------
#ifdef VIEWER_MODELS_DIR
    {
        const std::string dir = VIEWER_MODELS_DIR;
        viewer::ObjModel m;
        std::string warn;
        viewer::ImportedShape is;
        CHECK(viewer::loadObjFile(dir + "/cubo.obj", m, err, &warn) && warn.empty() && m.triangles.size() == 12);
        CHECK(m.materials.size() == 1 && m.materials[0].diffuseMap == "xadrez.png" && std::filesystem::exists(dir + "/xadrez.png"));
        CHECK(viewer::makeImportedShape(m, 1.0f, is, err) && is.hullVertices == 8 && is.hullFaces == 6 && !is.concave);
        CHECK(viewer::loadObjFile(dir + "/esfera.obj", m, err, &warn) && warn.empty() && m.triangles.size() == 320 && m.uvs.size() == m.positions.size());
        CHECK(m.materials[0].diffuseMap == "listras.png" && std::filesystem::exists(dir + "/listras.png"));
        CHECK(viewer::makeImportedShape(m, 1.0f, is, err) && is.hullVertices == 162 && !is.concave);
        CHECK(is.shape->getVolume() > Real(0.5) && is.shape->getVolume() < Real(0.5236));   // 4/3·pi·0,5³ = 0,5236
        CHECK(viewer::loadObjFile(dir + "/toro.obj", m, err, &warn) && viewer::makeImportedShape(m, 1.0f, is, err) && is.concave);
    }
#endif
}

// =================================================================================================
// V15b — modelo importado na cena: asset, malha para desenhar, spawn, .scene
// =================================================================================================
static void testModelScene() {
    {   // fecho com vértices demais (modelo muito detalhado): recusa com mensagem, sem forma
        Lcg rng(5);
        std::string obj;
        char buf[96];
        for (int i = 0; i < 20000; ++i) {
            double x = rng.range(-1, 1), y = rng.range(-1, 1), z = rng.range(-1, 1);
            const double l = std::sqrt(x * x + y * y + z * z);
            if (l < 0.1) { x = 1; y = z = 0; } else { x /= l; y /= l; z /= l; }
            std::snprintf(buf, sizeof buf, "v %.6f %.6f %.6f\n", x, y, z);
            obj += buf;
        }
        for (int i = 1; i + 2 <= 20000; i += 3) { std::snprintf(buf, sizeof buf, "f %d %d %d\n", i, i + 1, i + 2); obj += buf; }
        viewer::ObjModel m;
        std::string e;
        viewer::ImportedShape is;
        const bool big = viewer::parseObj(obj, m, e) && !viewer::makeImportedShape(m, 1.0f, is, e);
        CHECK(big && (e.find("fecho grande demais") != std::string::npos || e.find("instavel") != std::string::npos) && !is.shape);   // em float o fecho de 20 mil pontos pode ser recusado por instabilidade
        if (!big || e.find("fecho grande") == std::string::npos) std::printf("  fecho de 20000 pontos: %s\n", e.c_str());
    }
#ifdef VIEWER_MODELS_DIR
    using namespace physics;
    const std::string dir = VIEWER_MODELS_DIR;
    std::string err, warn;
    // --- lista e carregamento -------------------------------------------------------------------------------------
    {
        const std::vector<std::string> files = viewer::listModelFiles(dir);
        CHECK(files.size() == 3 && files[0] == "cubo.obj" && files[1] == "esfera.obj" && files[2] == "toro.obj");   // ordem alfabética
        CHECK(viewer::listModelFiles(dir + "/nao_existe").empty());
        viewer::ModelAsset a;
        CHECK(viewer::loadModelAsset(dir, "cubo.obj", 2.0f, a, err, &warn));
        CHECK(near(a.scale, 2.0f) && a.shape && a.hullVertices == 8 && a.hullFaces == 6 && a.file == "cubo.obj" && !a.concave);
        CHECK(near(static_cast<float>(a.shape->getVolume()), 8.0f, 1e-3f));   // cubo de 1 m, maior dimensão 2 m
        CHECK(!viewer::loadModelAsset(dir, "../x.obj", 1.0f, a, err) && err.find("invalido") != std::string::npos);
        CHECK(!viewer::loadModelAsset(dir, "/etc/x.obj", 1.0f, a, err));
        CHECK(!viewer::loadModelAsset(dir, "nao_existe.obj", 1.0f, a, err) && !a.shape);
        CHECK(viewer::loadModelAsset(dir, "toro.obj", 1.0f, a, err) && a.concave);
        // malha para desenhar: todos os triângulos, UV invertido em v, normais unitárias, vértices dentro do fecho
        CHECK(viewer::loadModelAsset(dir, "esfera.obj", 1.0f, a, err));
        const std::vector<viewer::MeshGroup> groups = viewer::buildMeshGroups(a);
        CHECK(groups.size() == 1 && groups[0].triangleCount() == 320 && groups[0].positions.size() == 320 * 9 && groups[0].texcoords.size() == 320 * 6);
        CHECK(groups[0].material == 0);
        float maxR = 0;
        for (std::size_t i = 0; i < groups[0].positions.size(); i += 3)
            maxR = std::max(maxR, std::sqrt(groups[0].positions[i] * groups[0].positions[i] + groups[0].positions[i + 1] * groups[0].positions[i + 1] +
                                            groups[0].positions[i + 2] * groups[0].positions[i + 2]));
        CHECK(near(maxR, 0.5f * a.scale, 0.02f));   // esfera de raio 0,5 · escala, centrada na origem do corpo
        bool unit = true, uvOk = true;
        for (std::size_t i = 0; i < groups[0].normals.size(); i += 3) {
            const float l = std::sqrt(groups[0].normals[i] * groups[0].normals[i] + groups[0].normals[i + 1] * groups[0].normals[i + 1] + groups[0].normals[i + 2] * groups[0].normals[i + 2]);
            unit = unit && near(l, 1.0f, 1e-4f);
        }
        for (float u : groups[0].texcoords) uvOk = uvOk && u >= -1e-4f && u <= 1.0001f;
        CHECK(unit && uvOk);
        // as normais apontam para fora (esfera centrada): normal · posição do 1º vértice > 0
        bool outward = true;
        for (std::size_t tri = 0; tri < groups[0].triangleCount(); ++tri) {
            const float* p = &groups[0].positions[tri * 9];
            const float* n = &groups[0].normals[tri * 3];
            outward = outward && (n[0] * p[0] + n[1] * p[1] + n[2] * p[2]) > 0;
        }
        CHECK(outward);
        // v do .obj: o UV (0,0) do cubo vira (0,1) na raylib
        viewer::ModelAsset c;
        CHECK(viewer::loadModelAsset(dir, "cubo.obj", 1.0f, c, err));
        const auto cg = viewer::buildMeshGroups(c);
        CHECK(cg.size() == 1 && cg[0].triangleCount() == 12 && near(cg[0].texcoords[0], 0.0f) && near(cg[0].texcoords[1], 1.0f));
    }
    // --- spawn de um modelo ---------------------------------------------------------------------------------------
    viewer::Scene s = viewer::makeEmptyScene(true);
    auto asset = std::make_shared<viewer::ModelAsset>();
    CHECK(viewer::loadModelAsset(dir, "cubo.obj", 1.0f, *asset, err));
    viewer::SpawnParams sp;
    sp.model = asset;
    sp.mass = 3;
    sp.position = Vector3(1, 4, -2);
    sp.size = Real(0.2);   // ignorado para modelos
    sp.orientation = viewer::orientationFromAxisAngle(Vector3(1, 1, 0), Real(0.9));
    const BodyId id = viewer::spawnBody(s, sp);
    CHECK(id != kInvalidBodyId && s.bodies.back().model == asset && s.bodies.back().kind == viewer::ShapeKind::Polyhedron);
    CHECK(s.world->getBody(id).getShape() == static_cast<const Shape*>(asset->shape.get()) && near(static_cast<float>(s.world->getBody(id).getMass()), 3.0f));
    viewer::SpawnParams st = sp;
    st.isStatic = true;
    st.position = Vector3(5, 1, 5);
    const BodyId sid = viewer::spawnBody(s, st);
    CHECK(sid != kInvalidBodyId && s.world->getBody(sid).isStatic() && s.bodies.back().model);
    viewer::SpawnParams empty;   // modelo sem forma: cai no tipo comum
    empty.model = std::make_shared<viewer::ModelAsset>();
    CHECK(viewer::spawnBody(s, empty) != kInvalidBodyId && !s.bodies.back().model);
    // --- .scene: ida e volta com a linha `model` ---------------------------------------------------------------------------
    for (int i = 0; i < 60; ++i) s.world->update(Real(1) / Real(60));
    const std::string txt = viewer::serializeScene(s);
    CHECK(txt.find("  model ") != std::string::npos && txt.find("cubo.obj") != std::string::npos);
    viewer::Scene back;
    CHECK(viewer::parseScene(txt, back, err, dir, &warn) && warn.empty());
    CHECK(back.bodies.size() == s.bodies.size());
    int withModel = 0;
    for (const viewer::SceneBody& b : back.bodies)
        if (b.model) {
            ++withModel;
            CHECK(b.model->file == "cubo.obj" && near(b.model->scale, asset->scale, 1e-5f) && b.model->shape == b.poly && b.model->obj && b.model->obj->triangles.size() == 12);
            CHECK(near(static_cast<float>(b.model->offset.x), static_cast<float>(asset->offset.x), 1e-5f));
        }
    CHECK(withModel == 2);
    CHECK(viewer::serializeScene(back) == txt);   // texto idêntico
    // sem pasta de modelos ou arquivo ausente: poliedro comum + aviso, nunca erro
    viewer::Scene plain;
    CHECK(viewer::parseScene(txt, plain, err, "", &warn) && warn.find("sem pasta de modelos") != std::string::npos);
    for (const viewer::SceneBody& b : plain.bodies) CHECK(!b.model);
    CHECK(viewer::parseScene(txt, plain, err, dir + "/nao_existe", &warn) && warn.find("nao carregado") != std::string::npos && !plain.bodies.back().model);
    // entradas inválidas: erro com a linha, cena de destino intacta
    {
        auto withModelLine = [&](const std::string& line) {
            std::string t = txt;
            const std::size_t at = t.find("  model ");
            t.replace(at, t.find('\n', at) - at, line);
            return t;
        };
        viewer::Scene keep = viewer::makeEmptyScene(true);
        const std::size_t nb = keep.bodies.size();
        const char* bad[][2] = {
            {"  model 0 0 0 0 cubo.obj", "escala"}, {"  model 5000 0 0 0 cubo.obj", "escala"}, {"  model 1 0 0 cubo.obj", "espera"},
            {"  model 1 0 0 0 ../x.obj", "invalido"}, {"  model 1 0 0 0 /etc/passwd", "invalido"}, {"  model 1 0 0 0 C:/x.obj", "invalido"},
            {"  model 1 nan 0 0 cubo.obj", "numero invalido"}, {"  model 1 1e9 0 0 cubo.obj", "deslocamento"},
        };
        for (const auto& b : bad) {
            CHECK(!viewer::parseScene(withModelLine(b[0]), keep, err, dir) && err.find(b[1]) != std::string::npos && err.find("linha") != std::string::npos);
            CHECK(keep.bodies.size() == nb);
        }
        // `model` sem forma (pivô): recusado. (V17: com box ou sphere agora vale: o colisor é a caixa/esfera.)
        std::string t2 = "physics_scene 1\nbody\n type static\n shape none\n model 1 0 0 0 cubo.obj\nend\n";
        CHECK(!viewer::parseScene(t2, keep, err, dir) && err.find("so vale com") != std::string::npos);
        std::string t3 = "physics_scene 1\nbody\n type dynamic\n shape box 1 1 1\n model 1 0 0 0 cubo.obj\n mass 1\nend\n";
        viewer::Scene boxed;
        CHECK(viewer::parseScene(t3, boxed, err, dir) && boxed.bodies.size() == 1 && boxed.bodies[0].model && boxed.bodies[0].kind == viewer::ShapeKind::Box);
    }
    // arquivo no disco com a pasta de modelos
    {
        namespace fs = std::filesystem;
        const fs::path f = fs::temp_directory_path() / "viewer_v15b.scene";
        CHECK(viewer::saveSceneFile(s, f.string(), err));
        viewer::Scene loaded;
        CHECK(viewer::loadSceneFile(f.string(), loaded, err, dir, &warn) && warn.empty());
        int n = 0;
        for (const viewer::SceneBody& b : loaded.bodies) n += b.model ? 1 : 0;
        CHECK(n == 2);
        fs::remove(f);
    }
    // o corpo-modelo cai e repousa no chão (como o cubo importado do teste do núcleo), carregado do arquivo
    for (int i = 0; i < 600; ++i) back.world->update(Real(1) / Real(60));
    for (const viewer::SceneBody& b : back.bodies)
        if (b.model && !b.isStatic) CHECK(near(static_cast<float>(back.world->getBody(b.id).getTransform().position.y), 0.5f, 0.03f));
#endif
}

static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) { if (c == ',') { out.push_back(cur); cur.clear(); } else cur += c; }
    out.push_back(cur);
    return out;
}

// V16 — gravador de trajetória e CSV
static void testTrajectoryRecorder() {
    using namespace physics;
    const Real dt = Real(1) / Real(60);
    {   // sem corpo selecionado / cena sem dinamicos: recusa com mensagem
        viewer::Scene s = viewer::makeFallingBoxScene();
        viewer::TrajectoryRecorder r;
        std::string e;
        CHECK(!r.start(s, viewer::RecordTarget::Selected, kInvalidBodyId, &e) && !e.empty() && !r.recording());
        viewer::Scene empty = viewer::makeEmptyScene(true);
        e.clear();
        CHECK(!r.start(empty, viewer::RecordTarget::AllDynamic, kInvalidBodyId, &e) && !e.empty());
        CHECK(r.toCsv() == viewer::TrajectoryRecorder::header() + "\n");
    }
    {   // gravação de um corpo: linha inicial + uma por passo; CSV bem formado; quatérnio unitário
        viewer::Scene s = viewer::makeFallingBoxScene();
        viewer::TrajectoryRecorder r;
        CHECK(r.start(s, viewer::RecordTarget::Selected, s.focus));
        CHECK(r.recording() && r.rowCount() == 1 && r.rows()[0].t == 0.0);
        CHECK(r.capture(s) == 0);   // física não andou: nada novo
        for (int i = 0; i < 120; ++i) { s.world->update(dt); r.capture(s); }
        CHECK(r.rowCount() == 121 && r.recording());
        CHECK(near(static_cast<float>(r.duration()), 2.0f, 1e-3f));
        bool unit = true, mono = true;
        for (std::size_t i = 0; i < r.rows().size(); ++i) {
            unit = unit && std::fabs(r.rows()[i].qNorm - 1.0) < 1e-4;
            if (i) mono = mono && r.rows()[i].t > r.rows()[i - 1].t && r.rows()[i].step > r.rows()[i - 1].step;
        }
        CHECK(unit && mono);
        const std::string csv = r.toCsv();
        std::vector<std::string> lines;
        std::size_t a = 0;
        for (std::size_t b; (b = csv.find('\n', a)) != std::string::npos; a = b + 1) lines.push_back(csv.substr(a, b - a));
        CHECK(lines.size() == 122 && csv.back() == '\n');
        const std::size_t cols = splitCsv(lines[0]).size();
        CHECK(cols == 24);
        bool wellFormed = true, noNan = csv.find("nan") == std::string::npos && csv.find("inf") == std::string::npos;
        for (const std::string& l : lines) wellFormed = wellFormed && splitCsv(l).size() == cols;
        CHECK(wellFormed && noNan);
        // ida e volta: o quatérnio da última linha lido do texto == o do corpo
        const std::vector<std::string> f = splitCsv(lines.back());
        const Quaternion& q = s.world->getBody(s.focus).getOrientation();
        CHECK(near(static_cast<float>(std::stod(f[6])), static_cast<float>(q.w), 1e-6f) && near(static_cast<float>(std::stod(f[7])), static_cast<float>(q.x), 1e-6f)
              && near(static_cast<float>(std::stod(f[8])), static_cast<float>(q.y), 1e-6f) && near(static_cast<float>(std::stod(f[9])), static_cast<float>(q.z), 1e-6f));
    }
    {   // todos os dinâmicos; parada por reinício da cena; por corpo apagado
        viewer::Scene s = viewer::makeStackScene();
        std::size_t dyn = 0;
        for (const viewer::SceneBody& sb : s.bodies) if (s.world->getBody(sb.id).isDynamic()) ++dyn;
        viewer::TrajectoryRecorder r;
        CHECK(r.start(s, viewer::RecordTarget::AllDynamic, kInvalidBodyId) && r.rowCount() == dyn);
        for (int i = 0; i < 10; ++i) { s.world->update(dt); r.capture(s); }
        CHECK(r.rowCount() == dyn * 11);
        viewer::Scene s2 = viewer::makeStackScene();   // "reiniciada": contador de passos volta a zero
        CHECK(r.capture(s2) == 0 && !r.recording() && !r.stopReason().empty());
        const std::size_t keep = r.rowCount();
        s.world->update(dt);
        CHECK(r.capture(s) == 0 && r.rowCount() == keep);   // parada não grava mais
    }
    {
        viewer::Scene s = viewer::makeFallingBoxScene();
        viewer::TrajectoryRecorder r;
        CHECK(r.start(s, viewer::RecordTarget::Selected, s.focus));
        CHECK(viewer::deleteBody(s, s.focus));
        s.world->update(dt);
        CHECK(r.capture(s) == 0 && !r.recording());
    }
    {   // salvar em arquivo (cria pasta) e erro em caminho inválido
        viewer::Scene s = viewer::makeFallingBoxScene();
        viewer::TrajectoryRecorder r;
        r.start(s, viewer::RecordTarget::Selected, s.focus);
        for (int i = 0; i < 5; ++i) { s.world->update(dt); r.capture(s); }
        const std::string dir = (std::filesystem::temp_directory_path() / "viewer_traj_test").string();
        std::filesystem::remove_all(dir);
        std::string err;
        CHECK(r.save(dir + "/sub/t.csv", err));
        CHECK(std::filesystem::exists(dir + "/sub/t.csv") && std::filesystem::file_size(dir + "/sub/t.csv") == r.toCsv().size());
        std::filesystem::remove_all(dir);
        err.clear();
        CHECK(!r.save("/proc/nao/pode/t.csv", err) && !err.empty());
    }
}

// V17 — layout dos painéis, importador de modelos e tipo de colisor
static std::string slurp(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
static void spit(const std::filesystem::path& p, const std::string& t) {
    std::filesystem::create_directories(p.parent_path());
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f << t;
}

static void testV17() {
    using namespace physics;
    namespace fs = std::filesystem;
    {   // --- alturas dos painéis: o conteúdo nunca passa do rodapé, em nenhuma combinação ---
        namespace pm = viewer::panel;
        for (int box = 0; box < 2; ++box)
            for (int model = 0; model < 2; ++model)
                for (int axis = 0; axis < 2; ++axis)
                    for (int tun = 0; tun < 2; ++tun) {
                        if (box + model + axis > 1 || (tun && !model)) continue;   // formas exclusivas; ajuste só com modelo
                        const float end = pm::addFormContentEnd(box != 0, model != 0, axis != 0, tun != 0);
                        const float h = pm::objPanelHeight(1, 0, box != 0, model != 0, axis != 0, tun != 0);
                        CHECK(end + pm::kFooterGap + pm::kFooter <= h + 1e-3f);   // o rodapé começa abaixo do conteúdo
                        CHECK(h >= pm::kAddOtherTabsH);
                    }
        CHECK(pm::addFormHeight(false, true, false, true) > pm::addFormHeight(false, true, false, false));
        // o caso do print: caixa precisava de mais que os 440 px antigos
        CHECK(pm::addFormHeight(true, false) > 440.0f);
        CHECK(pm::objPanelHeight(1, 3, true, false) >= pm::addFormHeight(true, false));
        CHECK(pm::kQuatH >= pm::kQuatContentEnd + 8.0f);
        CHECK(pm::objPanelHeight(0, 0, false, false) == pm::kObjectTabH && pm::objPanelHeight(2, 0, false, false) == pm::kScenesTabH &&
              pm::objPanelHeight(3, 0, false, false) == pm::kJointsTabH);
    }
    {   // --- nomes seguros ---
        CHECK(viewer::sanitizeModelName("Minha Pessoa 3D") == "Minha_Pessoa_3D");
        CHECK(viewer::sanitizeModelName("../../etc/passwd") == "etcpasswd");
        CHECK(viewer::sanitizeModelName("") == "modelo" && viewer::sanitizeModelName("..//") == "modelo" && viewer::sanitizeModelName("___") == "modelo");
        CHECK(viewer::sanitizeModelName(std::string(200, 'a')).size() == 40);
        CHECK(viewer::sanitizeModelName("a\xc3\xa7\xc3\xa3o") == "ao");   // acentos somem
    }
    const fs::path root = fs::temp_directory_path() / "viewer_import_test";
    fs::remove_all(root);
    const std::string cube =
        "mtllib mat.mtl\nusemtl m1\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
        "f 1/1 4/4 3/3 2/2\nf 5/1 6/2 7/3 8/4\nf 1/1 2/2 6/3 5/4\nf 2/1 3/2 7/3 6/4\nf 3/1 4/2 8/3 7/4\nf 4/1 1/2 5/3 8/4\n";
    {   // --- importar: .obj + .mtl + textura, de uma pasta qualquer ---
        const fs::path src = root / "origem";
        spit(src / "Meu Cubo.obj", cube);
        spit(src / "mat.mtl", "newmtl m1\nKd 0.5 0.5 0.5\nmap_Kd tex/cor.png\n");
        spit(src / "tex" / "cor.png", "PNGFAKE");
        spit(src / "tex" / "nao_usada.png", "X");
        const std::string models = (root / "models").string();
        viewer::ImportResult r;
        std::string err;
        CHECK(viewer::importModel((src / "Meu Cubo.obj").string(), models, r, err));
        CHECK(r.file == "Meu_Cubo/Meu_Cubo.obj" && r.copied == 3);
        CHECK(fs::exists(root / "models/Meu_Cubo/Meu_Cubo.obj") && fs::exists(root / "models/Meu_Cubo/mat.mtl") && fs::exists(root / "models/Meu_Cubo/tex/cor.png"));
        CHECK(!fs::exists(root / "models/Meu_Cubo/tex/nao_usada.png"));   // só o que o modelo usa
        CHECK(slurp(root / "models/Meu_Cubo/Meu_Cubo.obj") == cube && slurp(root / "models/Meu_Cubo/tex/cor.png") == "PNGFAKE");
        // aparece na lista e carrega como asset, com a textura achada
        const std::vector<std::string> files = viewer::listModelFiles(models);
        CHECK(files.size() == 1 && files[0] == "Meu_Cubo/Meu_Cubo.obj");
        viewer::ModelAsset a;
        std::string warn;
        CHECK(viewer::loadModelAsset(models, r.file, 1.0f, a, err, &warn) && a.shape && a.obj->materials.size() == 1 &&
              a.obj->materials[0].diffuseMap == "tex/cor.png" && fs::exists(fs::path(a.baseDir) / a.obj->materials[0].diffuseMap));
        // de novo: nunca sobrescreve (nome _2, _3)
        viewer::ImportResult r2, r3;
        CHECK(viewer::importModel((src / "Meu Cubo.obj").string(), models, r2, err) && r2.file == "Meu_Cubo_2/Meu_Cubo_2.obj");
        CHECK(viewer::importModel((src / "Meu Cubo.obj").string(), models, r3, err) && r3.file == "Meu_Cubo_3/Meu_Cubo_3.obj");
        CHECK(slurp(root / "models/Meu_Cubo/Meu_Cubo.obj") == cube);
        // sem .mtl e sem textura: importa com avisos
        { std::string c2 = cube; c2.replace(c2.find("mat.mtl"), 7, "nao.mtl"); spit(src / "solto.obj", c2); }
        viewer::ImportResult r4;
        CHECK(viewer::importModel((src / "solto.obj").string(), models, r4, err) && r4.copied == 1 && !r4.notes.empty());
    }
    {   // --- recusas, sem deixar lixo em models/ ---
        const std::string models = (root / "models2").string();
        viewer::ImportResult r;
        std::string err;
        spit(root / "ruim" / "x.txt", "v 0 0 0");
        CHECK(!viewer::importModel((root / "ruim/x.txt").string(), models, r, err) && err.find(".obj") != std::string::npos);
        CHECK(!viewer::importModel((root / "ruim/nao_existe.obj").string(), models, r, err) && !err.empty());
        spit(root / "ruim" / "lixo.obj", "v 0 0 0\nf 1 2 9\n");
        CHECK(!viewer::importModel((root / "ruim/lixo.obj").string(), models, r, err) && err.find("linha") != std::string::npos);
        spit(root / "ruim" / "vazio.obj", "v 0 0 0\nv 1 0 0\n");
        CHECK(!viewer::importModel((root / "ruim/vazio.obj").string(), models, r, err) && err.find("faces") != std::string::npos);
        // caminhos inseguros dentro do .obj/.mtl: ignorados com aviso (nada copiado de fora)
        spit(root / "ruim" / "esperto.obj", "mtllib ../segredo.mtl\nmtllib /etc/passwd\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n");
        spit(root / "segredo.mtl", "newmtl a\n");
        CHECK(viewer::importModel((root / "ruim/esperto.obj").string(), models, r, err) && r.copied == 1 && !fs::exists(root / "models2/esperto/segredo.mtl"));
        // textura fora da pasta / formato estranho: ignoradas
        spit(root / "ruim" / "t.obj", "mtllib t.mtl\nusemtl a\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n");
        spit(root / "ruim" / "t.mtl", "newmtl a\nmap_Kd ../fora.png\n");
        CHECK(viewer::importModel((root / "ruim/t.obj").string(), models, r, err) && r.copied == 2);
        CHECK(!fs::exists(root / "models2/t/fora.png") && !fs::exists(root / "models2/fora.png"));
        // erro não deixa pasta pela metade
        std::size_t dirs = 0;
        for (const auto& e : fs::directory_iterator(models)) dirs += e.is_directory() ? 1 : 0;
        CHECK(dirs == 2);   // só "esperto" e "t"
    }
    {   // --- importPath: arquivo, pasta, material solto, inexistente ---
        const std::string models = (root / "models3").string();
        std::vector<viewer::ImportResult> got;
        std::vector<std::string> msg;
        spit(root / "pasta" / "a.obj", cube);
        spit(root / "pasta" / "mat.mtl", "newmtl m1\n");
        spit(root / "pasta" / "sub" / "b.obj", cube);
        CHECK(viewer::importPath((root / "pasta").string(), models, got, msg) && got.size() == 2);
        CHECK(!viewer::importPath((root / "pasta/mat.mtl").string(), models, got, msg) && got.empty() && !msg.empty());
        msg.clear();
        CHECK(!viewer::importPath((root / "nada").string(), models, got, msg) && !msg.empty() && msg[0].find("ERRO") == 0);
        fs::create_directories(root / "vazia");
        msg.clear();
        CHECK(!viewer::importPath((root / "vazia").string(), models, got, msg) && !msg.empty());
        CHECK(viewer::importPath((root / "pasta/a.obj").string(), models, got, msg) && got.size() == 1);
    }
    {   // --- tipos de colisor: caixa e esfera envolventes ---
        viewer::ObjModel m;
        std::string err;
        // paralelepípedo 2 x 4 x 1 centrado fora da origem (x de 10 a 12, y de 0 a 4, z de -0,5 a 0,5)
        const std::string txt = "v 10 0 -0.5\nv 12 0 -0.5\nv 12 4 -0.5\nv 10 4 -0.5\nv 10 0 0.5\nv 12 0 0.5\nv 12 4 0.5\nv 10 4 0.5\n"
                                "f 1 4 3 2\nf 5 6 7 8\nf 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
        CHECK(viewer::parseObj(txt, m, err));
        viewer::ImportedShape box, sph, hull;
        CHECK(viewer::makeImportedShape(m, 1.0f, viewer::ColliderKind::Box, box, err));
        CHECK(box.shape && box.shape->getType() == ShapeType::Box && box.collider == viewer::ColliderKind::Box);
        CHECK(near(static_cast<float>(box.boxHalf.x), 1.0f) && near(static_cast<float>(box.boxHalf.y), 2.0f) && near(static_cast<float>(box.boxHalf.z), 0.5f));
        CHECK(near(static_cast<float>(box.offset.x), 11.0f) && near(static_cast<float>(box.offset.y), 2.0f) && near(static_cast<float>(box.offset.z), 0.0f));   // origem = centro da caixa
        CHECK(near(static_cast<float>(box.colliderVolume), 8.0f, 1e-3f) && near(static_cast<float>(box.fill), 1.0f, 1e-3f));
        CHECK(viewer::makeImportedShape(m, 2.0f, viewer::ColliderKind::Sphere, sph, err));
        CHECK(sph.shape && sph.shape->getType() == ShapeType::Sphere);
        CHECK(near(sph.sphereRadius, std::sqrt(1.0f + 4.0f + 0.25f) * 2.0f, 1e-3f));   // distância do centro ao canto, na escala 2
        CHECK(sph.fill < 0.7 && sph.fill > 0.0);   // esfera envolvente folgada
        CHECK(viewer::makeImportedShape(m, 1.0f, viewer::ColliderKind::Hull, hull, err) && hull.shape && hull.shape->getType() == ShapeType::Polyhedron);
        CHECK(near(static_cast<float>(hull.fill), 1.0f, 1e-2f));
        // modelo plano: só Box/Sphere aceitam (espessura mínima), fecho recusa
        viewer::ObjModel flat;
        CHECK(viewer::parseObj("v 0 0 0\nv 2 0 0\nv 2 0 2\nv 0 0 2\nf 1 2 3 4\n", flat, err));
        viewer::ImportedShape fl;
        CHECK(!viewer::makeImportedShape(flat, 1.0f, viewer::ColliderKind::Hull, fl, err) && !fl.shape);
        CHECK(viewer::makeImportedShape(flat, 1.0f, viewer::ColliderKind::Box, fl, err) && near(static_cast<float>(static_cast<const BoxShape&>(*fl.shape).getHalfExtents().y), 0.005f));
        CHECK(viewer::makeImportedShape(flat, 1.0f, viewer::ColliderKind::Sphere, fl, err));
        // escala/tamanho inválidos
        CHECK(!viewer::makeImportedShape(m, 0.0f, viewer::ColliderKind::Box, fl, err) && !fl.shape);
        CHECK(!viewer::makeImportedShape(m, 100.0f, viewer::ColliderKind::Sphere, fl, err));   // 400 m
        CHECK(!viewer::makeImportedShape(m, 1.0f, static_cast<viewer::ColliderKind>(9), fl, err));
    }
#ifdef VIEWER_MODELS_DIR
    {   // --- corpos com cada colisor: criar, simular, salvar e carregar ---
        const std::string dir = VIEWER_MODELS_DIR;
        for (int k = 0; k < viewer::kColliderKindCount; ++k) {
            const viewer::ColliderKind ck = static_cast<viewer::ColliderKind>(k);
            viewer::Scene s = viewer::makeEmptyScene(true);
            viewer::ModelAsset a;
            std::string err, warn;
            CHECK(viewer::loadModelAsset(dir, "toro.obj", 1.5f, a, err, &warn, ck) && a.collider == ck && a.shape);
            viewer::SpawnParams sp;
            sp.model = std::make_shared<viewer::ModelAsset>(a);
            sp.position = Vector3(0, 3, 0);
            sp.orientation = Quaternion::fromAxisAngle(Vector3(1, 0, 1).normalized(), Real(0.7));
            const BodyId id = viewer::spawnBody(s, sp);
            CHECK(id != kInvalidBodyId && s.bodies.back().model);
            const viewer::ShapeKind kinds[] = {viewer::ShapeKind::Polyhedron, viewer::ShapeKind::Box, viewer::ShapeKind::Sphere,
                                               viewer::ShapeKind::Capsule, viewer::ShapeKind::Cylinder};
            const ShapeType types[] = {ShapeType::Polyhedron, ShapeType::Box, ShapeType::Sphere, ShapeType::Capsule, ShapeType::Cylinder};
            CHECK(s.bodies.back().kind == kinds[k]);
            CHECK(s.world->getBody(id).getShape()->getType() == types[k]);
            for (int i = 0; i < 240; ++i) s.world->update(Real(1) / Real(60));
            const RigidBody& b = s.world->getBody(id);
            CHECK(b.getTransform().position.isFinite() && b.getTransform().position.y > Real(0) && std::fabs(static_cast<double>(b.getOrientation().norm()) - 1.0) < 1e-4);
            // ida e volta pelo .scene (o `model` agora sai com box/sphere também)
            const std::string txt = viewer::serializeScene(s);
            CHECK(txt.find("\n  model ") != std::string::npos);
            viewer::Scene back;
            std::string e2, w2;
            CHECK(viewer::parseScene(txt, back, e2, dir, &w2) && w2.empty() && back.bodies.size() == s.bodies.size());
            CHECK(back.bodies.back().model && back.bodies.back().model->collider == ck && back.bodies.back().kind == s.bodies.back().kind);
            CHECK(viewer::serializeScene(back) == txt);
        }
    }
#endif
    fs::remove_all(root);
}

// V18 — capsula e cilindro: criar, colisor de modelo (ajuste automatico), .scene
static void testV18() {
    using namespace physics;
    for (int k = 6; k <= 7; ++k) {   // criar e simular em cima do chao
        viewer::Scene s = viewer::makeEmptyScene(true);
        viewer::SpawnParams p;
        p.kind = static_cast<viewer::SpawnKind>(k);
        p.size = Real(1);
        p.position = Vector3(0, 3, 0);
        p.orientation = Quaternion::fromAxisAngle(Vector3(1, 0, 1).normalized(), Real(1.2));
        const BodyId id = viewer::spawnBody(s, p);
        CHECK(id != kInvalidBodyId);
        const viewer::SceneBody sb = s.bodies.back();   // copia: spawns seguintes realocam o vetor
        CHECK(sb.kind == (k == 6 ? viewer::ShapeKind::Capsule : viewer::ShapeKind::Cylinder));
        CHECK(sb.radius > 0 && sb.halfHeight >= 0);
        for (int i = 0; i < 360; ++i) s.world->update(Real(1) / Real(60));
        const RigidBody& b = s.world->getBody(id);
        CHECK(b.getTransform().position.isFinite() && b.getTransform().position.y > Real(0.1) && b.getTransform().position.y < Real(3));
        // limites e entradas invalidas
        viewer::SpawnParams bad = p;
        bad.radiusScale = std::numeric_limits<Real>::quiet_NaN();
        CHECK(viewer::spawnBody(s, bad) == kInvalidBodyId);
        bad = p; bad.radiusScale = Real(1000); bad.heightScale = Real(0);
        CHECK(viewer::spawnBody(s, bad) != kInvalidBodyId);
        // salvar e carregar
        const std::string text = viewer::serializeScene(s);
        viewer::Scene back;
        std::string err;
        CHECK(viewer::parseScene(text, back, err));
        CHECK(back.bodies.size() == s.bodies.size());
        CHECK(back.bodies[1].kind == sb.kind && std::fabs(back.bodies[1].radius - sb.radius) < 1e-4f &&
              std::fabs(back.bodies[1].halfHeight - sb.halfHeight) < 1e-4f);
    }
    {   // erros de sintaxe
        viewer::Scene sc;
        std::string err;
        const std::string head = "physics_scene 1\nname t\nbody\n  type static\n  position 0 1 0\n";
        CHECK(viewer::parseScene(head + "  shape capsule 0.3 0\nend\n", sc, err));   // controle: valido (capsula = esfera)
        CHECK(viewer::parseScene(head + "  shape cylinder 0.3 0.5\nend\n", sc, err));
        CHECK(!viewer::parseScene(head + "  shape capsule 0 1\nend\n", sc, err));
        CHECK(!viewer::parseScene(head + "  shape cylinder 1\nend\n", sc, err));
        CHECK(!viewer::parseScene(head + "  shape cylinder 1 0\nend\n", sc, err));
    }
#ifdef VIEWER_MODELS_DIR
    {   // ajuste automatico: toro.obj, escala 1,5; ajuste fino muda raio e altura
        const std::string dir = VIEWER_MODELS_DIR;
        viewer::ColliderTuning one, wide;
        wide.radiusScale = 2.0f; wide.heightScale = 0.5f;
        viewer::ModelAsset a, b;
        std::string err, warn;
        for (int k = 3; k <= 4; ++k) {
            const viewer::ColliderKind ck = static_cast<viewer::ColliderKind>(k);
            CHECK(viewer::loadModelAsset(dir, "cubo.obj", 2.0f, a, err, &warn, ck, one));
            CHECK(a.collider == ck && a.shape && a.longestAxis >= 0);
            CHECK(viewer::loadModelAsset(dir, "cubo.obj", 2.0f, b, err, &warn, ck, wide));
            const auto get = [&](const viewer::ModelAsset& m, float& r, float& h) {
                if (k == 3) { const auto& c = static_cast<const CapsuleShape&>(*m.shape); r = float(c.getRadius()); h = float(c.getHalfHeight()); }
                else { const auto& c = static_cast<const CylinderShape&>(*m.shape); r = float(c.getRadius()); h = float(c.getHalfHeight()); }
            };
            float r1, h1, r2, h2;
            get(a, r1, h1); get(b, r2, h2);
            CHECK(r1 > 0.5f && r2 > r1 * 1.9f && r2 < r1 * 2.1f);   // cubo de 2 m: raio 1; dobrado
            if (k == 4) CHECK(std::fabs(h1 - 1.0f) < 1e-3f && std::fabs(h2 - 0.5f) < 1e-3f);
            else CHECK(h1 < 1e-3f && h2 < 1e-3f);                   // cubo: o segmento da capsula some (hy - r = 0)
            // tuning fora da faixa e nao finito
            viewer::ColliderTuning nan_t; nan_t.radiusScale = std::numeric_limits<float>::quiet_NaN();
            CHECK(!viewer::loadModelAsset(dir, "cubo.obj", 2.0f, a, err, &warn, ck, nan_t));
            viewer::ColliderTuning huge; huge.radiusScale = 1e9f; huge.heightScale = -5.0f;
            CHECK(viewer::loadModelAsset(dir, "cubo.obj", 2.0f, a, err, &warn, ck, huge));
            // corpo com modelo + salvar/carregar
            viewer::Scene s = viewer::makeEmptyScene(true);
            viewer::SpawnParams sp;
            sp.model = std::make_shared<viewer::ModelAsset>(b);
            sp.position = Vector3(0, 3, 0);
            const BodyId id = viewer::spawnBody(s, sp);
            CHECK(id != kInvalidBodyId && s.bodies.back().model);
            CHECK(s.bodies.back().kind == (k == 3 ? viewer::ShapeKind::Capsule : viewer::ShapeKind::Cylinder));
            for (int i = 0; i < 240; ++i) s.world->update(Real(1) / Real(60));
            CHECK(s.world->getBody(id).getTransform().position.isFinite());
            viewer::Scene back;
            std::string w2;
            CHECK(viewer::parseScene(viewer::serializeScene(s), back, err, dir, &w2));
            CHECK(back.bodies.back().kind == s.bodies.back().kind && back.bodies.back().model);
        }
    }
#endif
}

int main() {
    testMatchesEngine();
    testKnownRotation();
    testOrthonormalBasis();
    testOrbitCamera();
    testFallingBoxScene();
    testSceneRegistry();
    testCameraDistance();
    testRenderPose();
    testInteraction();
    testBodyEditor();
    testWorldEditor();
    testTimeControl();
    testObjectSpawner();
    testContactTracker();
    testCollisionScenes();
    testSceneFile();
    testDockLayout();
    testSceneEditor();
    testModelImport();
    testModelScene();
    testTrajectoryRecorder();
    testV17();
    testCameraMove();
    testV18();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
