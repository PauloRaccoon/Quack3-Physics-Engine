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
#include <limits>
#include <string>

#include "viewer/core/BodyEditor.h"
#include "viewer/core/ContactTracker.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/OrbitCamera.h"
#include "viewer/core/WorldEditor.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/TimeControl.h"
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
    CHECK(!setMass(w, ground, Real(2)) && !setFriction(w, ground, Real(1)) && !setPosition(w, ground, Vector3(0, 9, 0)) &&
          !setOrientationAxisAngle(w, ground, Vector3(0, 1, 0), Real(1)) && !setAwake(w, ground, false) && !zeroVelocities(w, ground));
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
        CHECK((k >= 2) == (sb.poly != nullptr));
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
        {"sem corpos", "physics_scene 1\nname x\n", "nenhum corpo"},
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
        run(s, 4);
        const Quaternion q = s.world->getBody(plank).getOrientation();
        CHECK(std::fabs(angleAbout(q, Vector3(0, 0, 1))) > Real(0.1));
        CHECK(std::fabs(q.x) < Real(0.02) && std::fabs(q.y) < Real(0.02));
        CHECK(angleAbout(q, Vector3(0, 0, 1)) > Real(0));   // anti-horário em Z: a ponta direita sobe, a esquerda (onde a bola caiu) desce
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
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
