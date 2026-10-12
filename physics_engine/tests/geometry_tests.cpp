// =============================================================================
// geometry_tests.cpp — testes unitários da Fase 2 (sem frameworks externos).
// =============================================================================
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "physics/geometry/AABB.h"
#include "physics/geometry/BoxShape.h"
#include "physics/geometry/HalfEdgeStructure.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/SphereShape.h"

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

static const Real kTol = Real(1e-4);

// Gerador pseudo-aleatório determinístico (LCG) para direções de teste.
static Real rnd(unsigned& s) {
    s = s * 1664525u + 1013904223u;
    return Real((s >> 8) & 0xFFFF) / Real(32768) - Real(1);
}

template <typename F>
static bool throwsInvalidArgument(F&& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; } catch (...) { return false; }
    return false;
}

static AABB bruteForceAABB(const std::vector<Vector3>& pts, const Transform& xf) {
    AABB b;
    for (const Vector3& p : pts) b.extend(xf.transformPoint(p));
    return b;
}

static bool aabbNear(const AABB& a, const AABB& b, Real eps = kTol) {
    return a.min.nearlyEqual(b.min, eps) && a.max.nearlyEqual(b.max, eps);
}

static void testAABB() {
    AABB empty;
    CHECK(!empty.isValid());
    CHECK(nearlyEqual(empty.surfaceArea(), Real(0)));

    AABB a(Vector3(0, 0, 0), Vector3(2, 2, 2));
    AABB b(Vector3(1, 1, 1), Vector3(3, 3, 3));
    AABB c(Vector3(5, 5, 5), Vector3(6, 6, 6));
    AABB touch(Vector3(2, 0, 0), Vector3(3, 1, 1));
    CHECK(a.overlaps(b) && b.overlaps(a));
    CHECK(!a.overlaps(c));
    CHECK(a.overlaps(touch));
    CHECK(a.contains(Vector3(1, 1, 1)) && !a.contains(Vector3(3, 1, 1)));
    CHECK(AABB::merge(a, c).contains(a) && AABB::merge(a, c).contains(c));
    CHECK(nearlyEqual(a.surfaceArea(), Real(24)) && nearlyEqual(a.volume(), Real(8)));
    CHECK(a.center().nearlyEqual(Vector3(1, 1, 1)) && a.halfExtents().nearlyEqual(Vector3(1, 1, 1)));

    AABB grow;
    grow.extend(Vector3(1, 2, 3));
    grow.extend(Vector3(-1, 0, 5));
    CHECK(grow.min.nearlyEqual(Vector3(-1, 0, 3)) && grow.max.nearlyEqual(Vector3(1, 2, 5)));

    // Fat AABB: margem + extensão na direção do movimento.
    AABB fat = a.fattenedForMotion(Vector3(1, -1, 0), Real(0.1), Real(2));
    CHECK(fat.contains(a));
    CHECK(fat.max.x > a.max.x + Real(0.1) + Real(1.9));   // estendeu em +x
    CHECK(fat.min.y < a.min.y - Real(0.1) - Real(1.9));   // estendeu em -y
    CHECK(nearlyEqual(fat.max.y, a.max.y + Real(0.1)));   // sem deslocamento em +y
    CHECK(fat.needsRefit(a.fattened(Real(5))));
    CHECK(!fat.needsRefit(a));

    // AABB local rotacionada == força bruta nos 8 cantos.
    const AABB local(Vector3(-1, -2, -0.5f), Vector3(1, 2, 0.5f));
    std::vector<Vector3> corners;
    for (int i = 0; i < 8; ++i)
        corners.emplace_back((i & 1) ? 1.0f : -1.0f, (i & 2) ? 2.0f : -2.0f, (i & 4) ? 0.5f : -0.5f);
    const Transform xf(Vector3(3, -1, 2), Quaternion::fromAxisAngle(Vector3(1, 1, 0), Real(0.8)));
    CHECK(aabbNear(local.transformed(xf), bruteForceAABB(corners, xf)));
}

static void testSphere() {
    SphereShape s(Real(2));
    CHECK(s.getSupport(Vector3(0, 3, 0)).nearlyEqual(Vector3(0, 2, 0)));
    CHECK(s.getSupport(Vector3(1, 1, 1)).nearlyEqual(Vector3(1, 1, 1).normalized() * 2, kTol));
    CHECK(s.getSupport(Vector3::zero()).isFinite());  // direção nula não gera NaN
    CHECK(nearlyEqual(s.getVolume(), Real(4) / 3 * kPi * 8, Real(1e-3)));
    CHECK(s.computeInertia(5).nearlyEqual(Matrix3::diagonal(8, 8, 8), Real(1e-3)));

    Transform xf(Vector3(1, 2, 3), Quaternion::fromAxisAngle(Vector3::unitZ(), 1));
    CHECK(aabbNear(s.computeAABB(xf), AABB(Vector3(-1, 0, 1), Vector3(3, 4, 5))));
    CHECK(throwsInvalidArgument([] { SphereShape bad(0); }));
    CHECK(throwsInvalidArgument([] { SphereShape bad(-1); }));
    CHECK(throwsInvalidArgument([] { SphereShape bad(std::numeric_limits<Real>::quiet_NaN()); }));
}

static void testBox() {
    const Vector3 h(1, 2, 3);
    BoxShape box(h);
    std::vector<Vector3> corners;
    for (int i = 0; i < 8; ++i) corners.emplace_back((i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z);

    CHECK(nearlyEqual(box.getVolume(), Real(48)));
    // Inércia: m/3 (hy²+hz²) etc. com m = 12 → (4*13, 4*10, 4*5)
    CHECK(box.computeInertia(12).nearlyEqual(Matrix3::diagonal(52, 40, 20), kTol));

    unsigned seed = 7;
    for (int i = 0; i < 200; ++i) {
        const Vector3 d(rnd(seed), rnd(seed), rnd(seed));
        // O suporte deve maximizar d·p entre os cantos.
        Real best = -kRealMax;
        for (const Vector3& c : corners) best = std::max(best, d.dot(c));
        CHECK(nearlyEqual(d.dot(box.getSupport(d)), best, kTol));
    }
    CHECK(box.getSupportFeature(Vector3(1, 1, 1)) == 7u);
    CHECK(box.getSupportFeature(Vector3(-1, -1, -1)) == 0u);

    const Transform xf(Vector3(-4, 1, 2), Quaternion::fromAxisAngle(Vector3(1, -2, 1), Real(1.1)));
    CHECK(aabbNear(box.computeAABB(xf), bruteForceAABB(corners, xf)));
    CHECK(throwsInvalidArgument([] { BoxShape bad(Vector3(1, 0, 1)); }));
}

static void testHalfEdge() {
    PolyhedronShape box = PolyhedronShape::makeBox(Vector3(1, 1, 1));
    const HalfEdgeStructure& t = box.getTopology();
    CHECK(t.getVertexCount() == 8 && t.getFaceCount() == 6 && t.getEdgeCount() == 24);
    CHECK(t.validate());

    // Cada vértice do cubo tem 3 vizinhos; cada face tem 4 vértices.
    for (int v = 0; v < t.getVertexCount(); ++v) {
        int n = 0;
        t.forEachVertexNeighbor(v, [&](int) { ++n; });
        CHECK(n == 3);
    }
    for (int f = 0; f < t.getFaceCount(); ++f) {
        int n = 0;
        t.forEachFaceVertex(f, [&](int) { ++n; });
        CHECK(n == 4);
        // normal para fora: o centro está atrás do plano
        CHECK(t.getFace(f).offset > 0);
    }
    CHECK(t.findBestFace(Vector3(0, 0, 5)) == 4);

    // Malhas inválidas devem falhar com mensagem, sem travar.
    HalfEdgeStructure he;
    std::string err;
    const std::vector<Vector3> tet = {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)};
    CHECK(he.build(tet, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}}, &err));
    CHECK(!he.build(tet, {{0, 1, 2}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}}, &err));  // orientação inconsistente
    CHECK(!err.empty());
    CHECK(!he.build(tet, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}}, &err));             // poucas faces
    CHECK(!he.build(tet, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 9}}, &err));  // índice inválido
    CHECK(!he.build(tet, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 3}}, &err));  // vértice repetido
    CHECK(he.isEmpty());
}

static void testPolyhedron() {
    const Vector3 h(1, 2, 3);
    BoxShape ref(h);
    PolyhedronShape poly = PolyhedronShape::makeBox(h);

    CHECK(nearlyEqual(poly.getVolume(), ref.getVolume(), Real(1e-3)));
    CHECK(poly.getCentroid().nearlyEqual(Vector3::zero(), kTol));
    CHECK(poly.computeInertia(12).nearlyEqual(ref.computeInertia(12), Real(1e-3)));

    const Transform xf(Vector3(1, 2, 3), Quaternion::fromAxisAngle(Vector3(0, 1, 1), Real(0.9)));
    CHECK(aabbNear(poly.computeAABB(xf), ref.computeAABB(xf)));

    // Caixa deslocada da origem: volume/centroide/inércia (em torno do centroide) iguais.
    std::vector<Vector3> v;
    for (int i = 0; i < 8; ++i)
        v.emplace_back(((i & 1) ? h.x : -h.x) + 3, ((i & 2) ? h.y : -h.y) - 1, ((i & 4) ? h.z : -h.z) + 5);
    PolyhedronShape moved(v, {{1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}});
    CHECK(nearlyEqual(moved.getVolume(), ref.getVolume(), Real(1e-3)));
    CHECK(moved.getCentroid().nearlyEqual(Vector3(3, -1, 5), kTol));
    CHECK(moved.computeInertia(12).nearlyEqual(ref.computeInertia(12), Real(5e-3)));

    // Tetraedro: V = 1/6, centroide = média dos vértices.
    const std::vector<Vector3> tet = {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)};
    PolyhedronShape t(tet, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}});
    CHECK(nearlyEqual(t.getVolume(), Real(1) / 6, kTol));
    CHECK(t.getCentroid().nearlyEqual(Vector3(0.25f, 0.25f, 0.25f), kTol));
    const Matrix3 It = t.computeInertia(1);
    CHECK(It.nearlyEqual(It.transposed(), kTol));
    CHECK(nearlyEqual(It(0, 0), It(1, 1), kTol) && nearlyEqual(It(1, 1), It(2, 2), kTol));  // simetria

    // Prisma regular de N lados (40 vértices → usa hill climbing).
    const int N = 20;
    const double R = 1.5, H = 0.75;
    std::vector<Vector3> pv;
    for (int layer = 0; layer < 2; ++layer)
        for (int i = 0; i < N; ++i) {
            const double a = 2.0 * 3.14159265358979323846 * i / N;
            pv.emplace_back(Real(R * std::cos(a)), Real(R * std::sin(a)), Real(layer ? H : -H));
        }
    std::vector<std::vector<int>> pf;
    std::vector<int> top, bottom;
    for (int i = 0; i < N; ++i) { top.push_back(N + i); bottom.push_back(N - 1 - i); }
    pf.push_back(top);
    pf.push_back(bottom);
    for (int i = 0; i < N; ++i) pf.push_back({i, (i + 1) % N, N + (i + 1) % N, N + i});
    PolyhedronShape prism(pv, pf);

    const double area = 0.5 * N * R * R * std::sin(2.0 * 3.14159265358979323846 / N);
    CHECK(nearlyEqual(prism.getVolume(), Real(area * 2 * H), Real(1e-3)));
    const double c2 = std::cos(3.14159265358979323846 / N);
    const double izzPerMass = (R * R / 6.0) * (1.0 + 2.0 * c2 * c2);   // N-gon regular
    CHECK(nearlyEqual(prism.computeInertia(1)(2, 2), Real(izzPerMass), Real(1e-3)));
    CHECK(nearlyEqual(prism.computeInertia(1)(0, 0), prism.computeInertia(1)(1, 1), Real(1e-3)));

    // Hill climbing == busca exaustiva (comparando o valor do suporte, pois há empates).
    unsigned seed = 99;
    int hint = 0;
    for (int i = 0; i < 500; ++i) {
        const Vector3 d(rnd(seed), rnd(seed), rnd(seed));
        Real best = -kRealMax;
        for (const Vector3& p : pv) best = std::max(best, d.dot(p));
        CHECK(nearlyEqual(d.dot(prism.getSupport(d)), best, kTol));
        hint = prism.getSupportVertexIndex(d, hint);  // com coerência temporal
        CHECK(nearlyEqual(d.dot(pv[static_cast<std::size_t>(hint)]), best, kTol));
    }
    CHECK(aabbNear(prism.computeAABB(Transform::identity()), bruteForceAABB(pv, Transform::identity())));

    // Entradas inválidas: faces invertidas, não-convexo e não-planar.
    CHECK(throwsInvalidArgument([&] { PolyhedronShape bad(tet, {{0, 1, 2}, {0, 3, 1}, {1, 3, 2}, {0, 2, 3}}); }));
    std::vector<Vector3> dent = tet;  // vértice 3 empurrado para dentro → côncavo/invertido
    dent.push_back(Vector3(0.2f, 0.2f, 0.2f));
    CHECK(throwsInvalidArgument([&] {
        PolyhedronShape bad(dent, {{0, 2, 1}, {0, 1, 4}, {1, 3, 4}, {0, 4, 3}, {1, 2, 3}, {0, 3, 2}, {2, 4, 3}});
    }));
    std::vector<Vector3> warped = {Vector3(-1, -1, 0), Vector3(1, -1, 0), Vector3(1, 1, 0.5f), Vector3(-1, 1, 0), Vector3(0, 0, 2)};
    CHECK(throwsInvalidArgument([&] {
        PolyhedronShape bad(warped, {{3, 2, 1, 0}, {0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}});
    }));
}

int main() {
    testAABB();
    testSphere();
    testBox();
    testHalfEdge();
    testPolyhedron();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
