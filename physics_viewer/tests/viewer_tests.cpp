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
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "viewer/core/BodyEditor.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/OrbitCamera.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/TimeControl.h"
#include "viewer/core/Scene.h"
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
    CHECK(list.size() == 9);
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
    testTimeControl();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
