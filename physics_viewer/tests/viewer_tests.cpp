// =============================================================================
// viewer_tests.cpp — testes do núcleo do viewer (sem raylib, sem janela).
//
//   1. toMat4 aplicada a um ponto == Transform::transformPoint do motor (milhares de poses);
//   2. rotação conhecida (+90° em Y leva +X a −Z) e disposição em colunas (translação em m[12..14]);
//   3. as colunas de rotação formam uma base ortonormal destra;
//   4. OrbitCamera: posição, limites de pitch e zoom, entradas não finitas;
//   5. cena da V1: roda no PhysicsWorld, a caixa gira, pousa sobre o chão e o quatérnio segue unitário;
//   6. registro da V2: todas as cenas constroem, rodam 300 passos finitas/estáveis, nenhum corpo
//      atravessa o chão ou escapa; juntas mantêm os elos; setDistance da câmera.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <limits>

#include "viewer/core/OrbitCamera.h"
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
    CHECK(list.size() == 7);
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
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
