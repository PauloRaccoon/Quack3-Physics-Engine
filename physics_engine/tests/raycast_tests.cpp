// =============================================================================
// raycast_tests.cpp — testes da Fase 8: raycast, overlap e debug draw.
//
// Estratégia: valores analíticos para os casos simples e um ORÁCULO INDEPENDENTE
// para os aleatórios — "marcha" ao longo do raio com passo fino testando
// shapeContainsPoint (que não usa o algoritmo do raycast) — além de comparar as
// consultas do mundo com força bruta sobre as poses atuais dos corpos.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "physics/collision/CollisionWorld.h"
#include "physics/collision/RayCast.h"
#include "physics/collision/narrowphase/GJK.h"
#include "physics/debug/DebugDraw.h"
#include "physics/geometry/BoxShape.h"
#include "physics/geometry/PolyhedronShape.h"
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

// Tolerâncias: float tem ~7 dígitos; as distâncias aqui são de ordem 1-1000.
static const Real kTol = detail::kIsSinglePrecision ? Real(2e-4) : Real(1e-9);
static const Real kInf = std::numeric_limits<Real>::infinity();
static const Real kNaN = std::numeric_limits<Real>::quiet_NaN();

static bool near(Real a, Real b, Real tol = kTol) { return std::abs(a - b) <= tol; }
static bool nearV(const Vector3& a, const Vector3& b, Real tol = kTol) {
    return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {   // [0,1)
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * next(); }
    Vector3 unit() {
        for (;;) {
            const Vector3 v(range(-1, 1), range(-1, 1), range(-1, 1));
            const Real l = v.lengthSquared();
            if (l > Real(0.01) && l <= Real(1)) return v / std::sqrt(l);
        }
    }
    Quaternion rotation() { return Quaternion::fromAxisAngle(unit(), range(0, kTwoPi)); }
};

// Octaedro |x|/a + |y|/b + |z|/c <= 1 (poliedro convexo com faces triangulares).
static PolyhedronShape makeOctahedron(Real a, Real b, Real c) {
    const std::vector<Vector3> v = {Vector3(a, 0, 0), Vector3(-a, 0, 0), Vector3(0, b, 0),
                                    Vector3(0, -b, 0), Vector3(0, 0, c), Vector3(0, 0, -c)};
    std::vector<std::vector<int>> faces;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                const int X = sx > 0 ? 0 : 1, Y = sy > 0 ? 2 : 3, Z = sz > 0 ? 4 : 5;
                if (sx * sy * sz > 0) faces.push_back({X, Y, Z});
                else                  faces.push_back({X, Z, Y});
            }
        }
    }
    return PolyhedronShape(v, faces);
}

// Tetraedro irregular (diedros agudos): estressa o caso de arestas.
static PolyhedronShape makeTetra() {
    const std::vector<Vector3> v = {Vector3(1, 0, 0), Vector3(-0.5f, 0.9f, 0), Vector3(-0.5f, -0.9f, 0),
                                    Vector3(0, 0, 1.6f)};
    return PolyhedronShape(v, {{0, 1, 3}, {1, 2, 3}, {2, 0, 3}, {0, 2, 1}});
}

// =============================================================================
// AABB::intersectsRay
// =============================================================================
static void testAABBRay() {
    const AABB box(Vector3(-1, -1, -1), Vector3(1, 1, 1));
    Real t = -1;

    CHECK(box.intersectsRay(Vector3(-5, 0, 0), Vector3(1, 0, 0), 100, t) && near(t, 4));
    CHECK(!box.intersectsRay(Vector3(-5, 0, 0), Vector3(1, 0, 0), Real(3.9), t));       // curto demais
    CHECK(box.intersectsRay(Vector3(-5, 0, 0), Vector3(1, 0, 0), Real(4.0), t));        // exatamente no alcance
    CHECK(box.intersectsRay(Vector3(0, 0, 0), Vector3(0, 1, 0), 1, t) && t == 0);       // origem dentro
    CHECK(!box.intersectsRay(Vector3(5, 0, 0), Vector3(1, 0, 0), 100, t));              // caixa atrás
    CHECK(!box.intersectsRay(Vector3(-5, 2, 0), Vector3(1, 0, 0), 100, t));             // paralelo, fora do slab
    CHECK(box.intersectsRay(Vector3(-5, Real(0.5), 0), Vector3(1, 0, 0), 100, t));      // paralelo, dentro
    CHECK(!box.intersectsRay(Vector3(-5, 0, 0), Vector3(0, 1, 0), 100, t));             // perpendicular, passa ao lado
    const Vector3 diag = Vector3(1, 1, 1).normalized();
    CHECK(box.intersectsRay(Vector3(-3, -3, -3), diag, 100, t) && near(t, 2 * std::sqrt(Real(3))));
    CHECK(!AABB().intersectsRay(Vector3(0, 0, 0), Vector3(1, 0, 0), 100, t));           // AABB vazio

    // Oráculo por marcha: se alguma amostra do raio está dentro, intersectsRay tem de ser true
    // e a distância de entrada tem de estar a no máximo um passo da primeira amostra interna.
    Rng rng(11);
    int hits = 0;
    const Real step = Real(0.01);
    for (int i = 0; i < 2000; ++i) {
        const Vector3 c(rng.range(-3, 3), rng.range(-3, 3), rng.range(-3, 3));
        const AABB b = AABB::fromCenterHalfExtents(c, Vector3(rng.range(Real(0.2), 2), rng.range(Real(0.2), 2),
                                                              rng.range(Real(0.2), 2)));
        const Vector3 o(rng.range(-8, 8), rng.range(-8, 8), rng.range(-8, 8));
        Vector3 d = rng.unit();
        if (i % 2 == 0) d = (c - o).normalizedOr(d);   // metade dos raios mira a caixa: acertos garantidos
        const Real maxT = rng.range(1, 20);
        Real firstInside = -1;
        for (Real s = 0; s <= maxT; s += step) {
            if (b.contains(o + d * s)) { firstInside = s; break; }
        }
        Real te = 0;
        const bool h = b.intersectsRay(o, d, maxT, te);
        if (firstInside >= 0) {
            ++hits;
            CHECK(h);
            CHECK(h && te <= firstInside + Real(1e-4) && te >= firstInside - step - Real(1e-4));
        }
        if (h) CHECK(b.fattened(Real(1e-3)).contains(o + d * te));   // o ponto de entrada está no AABB
    }
    CHECK(hits > 200);   // o teste exercita acertos de verdade
}

// =============================================================================
// Raio contra formas
// =============================================================================
static void testSphereRay() {
    const SphereShape s(1);
    ShapeRayHit h;

    CHECK(raycastShape(s, Transform(), Vector3(0, 0, -5), Vector3(0, 0, 1), 100, h));
    CHECK(near(h.distance, 4) && nearV(h.point, Vector3(0, 0, -1)) && nearV(h.normal, Vector3(0, 0, -1)));
    CHECK(!h.startedInside);

    // Esfera deslocada e girada (a rotação não pode mudar nada).
    Transform xf(Vector3(3, 2, 1), Quaternion::fromAxisAngle(Vector3(1, 1, 0).normalized(), Real(0.7)));
    CHECK(raycastShape(s, xf, Vector3(3, 2, -9), Vector3(0, 0, 1), 100, h));
    CHECK(near(h.distance, 9) && nearV(h.point, Vector3(3, 2, 0)));

    // Tangente: 0,999 acerta (normal quase +x), 1,001 erra.
    CHECK(raycastShape(s, Transform(), Vector3(Real(0.999), 0, -5), Vector3(0, 0, 1), 100, h));
    CHECK(near(h.normal.x, Real(0.999), Real(1e-3)) && near(h.normal.lengthSquared(), 1, Real(1e-4)));
    CHECK(!raycastShape(s, Transform(), Vector3(Real(1.001), 0, -5), Vector3(0, 0, 1), 100, h));

    CHECK(!raycastShape(s, Transform(), Vector3(0, 0, 5), Vector3(0, 0, 1), 100, h));      // se afastando
    CHECK(!raycastShape(s, Transform(), Vector3(0, 0, -5), Vector3(0, 0, 1), Real(3.99), h)); // curto demais
    CHECK(raycastShape(s, Transform(), Vector3(0, 0, -5), Vector3(0, 0, 1), Real(4.01), h));
    CHECK(raycastShape(s, Transform(), Vector3(0, 0, -5), Vector3(0, 0, 1), kInf, h));     // alcance infinito

    // Origem dentro: acerto em 0 com normal contrária ao raio.
    const Vector3 dir = Vector3(1, 2, 3).normalized();
    CHECK(raycastShape(s, Transform(), Vector3(Real(0.1), 0, 0), dir, 100, h));
    CHECK(h.startedInside && h.distance == 0 && nearV(h.normal, -dir) && nearV(h.point, Vector3(Real(0.1), 0, 0)));

    // Longe da origem do mundo (precisão): esfera a 1000 m.
    const Transform far(Vector3(1000, 0, 0));
    CHECK(raycastShape(s, far, Vector3(0, 0, 0), Vector3(1, 0, 0), 2000, h));
    CHECK(near(h.distance, 999, Real(1e-2)));
}

static void testBoxRay() {
    const BoxShape b(Vector3(1, 2, 3));
    const Transform xf(Vector3(1, -1, 2));
    ShapeRayHit h;

    // Cada uma das 6 faces, por raios axiais passando pelo centro.
    struct Case { Vector3 origin, dir, normal; Real dist; };
    const Case cases[] = {
        {Vector3(-9, -1, 2), Vector3(1, 0, 0), Vector3(-1, 0, 0), 9},   // face -X em x = 0
        {Vector3(11, -1, 2), Vector3(-1, 0, 0), Vector3(1, 0, 0), 9},   // face +X em x = 2
        {Vector3(1, -9, 2), Vector3(0, 1, 0), Vector3(0, -1, 0), 6},    // face -Y em y = -3
        {Vector3(1, 9, 2), Vector3(0, -1, 0), Vector3(0, 1, 0), 8},     // face +Y em y = 1
        {Vector3(1, -1, -9), Vector3(0, 0, 1), Vector3(0, 0, -1), 8},   // face -Z em z = -1
        {Vector3(1, -1, 9), Vector3(0, 0, -1), Vector3(0, 0, 1), 4},    // face +Z em z = 5
    };
    for (const Case& c : cases) {
        CHECK(raycastShape(b, xf, c.origin, c.dir, 100, h));
        CHECK(near(h.distance, c.dist) && nearV(h.normal, c.normal) && !h.startedInside);
        CHECK(nearV(h.point, c.origin + c.dir * c.dist));
    }

    // Rotação de 90° em z: os semi-extents x e y trocam (agora x vai a ±2, y a ±1).
    const Transform rot(Vector3(0, 0, 0), Quaternion::fromAxisAngle(Vector3(0, 0, 1), kHalfPi));
    CHECK(raycastShape(b, rot, Vector3(-9, 0, 0), Vector3(1, 0, 0), 100, h));
    CHECK(near(h.distance, 7) && nearV(h.normal, Vector3(-1, 0, 0), Real(1e-4)));
    CHECK(raycastShape(b, rot, Vector3(0, 9, 0), Vector3(0, -1, 0), 100, h));
    CHECK(near(h.distance, 8) && nearV(h.normal, Vector3(0, 1, 0), Real(1e-4)));

    // Paralelo a uma face: por dentro do slab acerta, por fora erra.
    CHECK(raycastShape(b, Transform(), Vector3(-9, Real(1.9), 0), Vector3(1, 0, 0), 100, h));
    CHECK(!raycastShape(b, Transform(), Vector3(-9, Real(2.1), 0), Vector3(1, 0, 0), 100, h));
    CHECK(!raycastShape(b, Transform(), Vector3(9, 0, 0), Vector3(1, 0, 0), 100, h));      // atrás
    CHECK(!raycastShape(b, Transform(), Vector3(-9, 0, 0), Vector3(1, 0, 0), Real(7.9), h)); // curto (face em x = -1 → t=8)
    CHECK(raycastShape(b, Transform(), Vector3(-9, 0, 0), Vector3(1, 0, 0), Real(8.1), h));

    // Dentro da caixa.
    const Vector3 dir = Vector3(-1, 3, 2).normalized();
    CHECK(raycastShape(b, Transform(), Vector3(Real(0.2), Real(0.3), Real(0.1)), dir, 100, h));
    CHECK(h.startedInside && h.distance == 0 && nearV(h.normal, -dir));

    // Entradas inválidas: nada de exceção, nada de acerto, `hit` intocado.
    ShapeRayHit sentinel;
    sentinel.distance = Real(42);
    ShapeRayHit hh = sentinel;
    CHECK(!raycastShape(b, Transform(), Vector3(kNaN, 0, 0), Vector3(1, 0, 0), 10, hh));
    CHECK(!raycastShape(b, Transform(), Vector3(0, 0, 0), Vector3(kNaN, 0, 0), 10, hh));
    CHECK(!raycastShape(b, Transform(), Vector3(-9, 0, 0), Vector3(1, 0, 0), kNaN, hh));
    CHECK(!raycastShape(b, Transform(), Vector3(-9, 0, 0), Vector3(1, 0, 0), Real(-1), hh));
    Transform bad;
    bad.position = Vector3(kInf, 0, 0);
    CHECK(!raycastShape(b, bad, Vector3(-9, 0, 0), Vector3(1, 0, 0), 10, hh));
    CHECK(hh.distance == Real(42));
}

static void testPolyhedronRay() {
    const PolyhedronShape oct = makeOctahedron(1, 1, 1);
    ShapeRayHit h;
    // |x|+|y|+|z| <= 1: em y = z = 0,2 o raio ao longo de +x entra em x = −0,6, na face (−,+,+).
    CHECK(raycastShape(oct, Transform(), Vector3(-5, Real(0.2), Real(0.2)), Vector3(1, 0, 0), 100, h));
    CHECK(near(h.distance, Real(4.4), Real(1e-4)) && nearV(h.normal, Vector3(-1, 1, 1).normalized(), Real(1e-4)));
    // Passa por cima da quina (|y| + |z| > 1): erra.
    CHECK(!raycastShape(oct, Transform(), Vector3(-5, Real(0.6), Real(0.6)), Vector3(1, 0, 0), 100, h));
    // Faces opostas, vindo de baixo.
    CHECK(raycastShape(oct, Transform(), Vector3(Real(0.1), -9, Real(0.1)), Vector3(0, 1, 0), 100, h));
    CHECK(near(h.distance, Real(9) - Real(0.8), Real(1e-4)) && nearV(h.normal, Vector3(1, -1, 1).normalized(), Real(1e-4)));
    // Caixa construída como poliedro dá o mesmo resultado que a BoxShape.
    const PolyhedronShape pb = PolyhedronShape::makeBox(Vector3(1, 2, 3));
    const BoxShape bb(Vector3(1, 2, 3));
    ShapeRayHit h2;
    const Transform xf(Vector3(1, 2, 3), Quaternion::fromAxisAngle(Vector3(1, 2, 3).normalized(), Real(1.1)));
    Rng rng(5);
    for (int i = 0; i < 500; ++i) {
        const Vector3 o(rng.range(-9, 9), rng.range(-9, 9), rng.range(-9, 9)), d = rng.unit();
        const bool a = raycastShape(pb, xf, o, d, 30, h), b = raycastShape(bb, xf, o, d, 30, h2);
        CHECK(a == b);
        if (a && b) CHECK(near(h.distance, h2.distance, Real(1e-3)) && nearV(h.normal, h2.normal, Real(1e-3)));
    }
}

// Raios aleatórios contra esfera, caixa, octaedro e tetraedro, confrontados com o oráculo por marcha.
static void testRandomRaysAgainstOracle() {
    const SphereShape sphere(Real(1.3));
    const BoxShape box(Vector3(Real(0.7), Real(1.1), Real(0.4)));
    const PolyhedronShape oct = makeOctahedron(Real(1.4), Real(0.8), Real(1.1));
    const PolyhedronShape tet = makeTetra();
    const Shape* shapes[] = {&sphere, &box, &oct, &tet};
    const char* names[] = {"esfera", "caixa", "octaedro", "tetraedro"};

    Rng rng(2024);
    const Real step = Real(0.002);
    const Real maxT = Real(14);
    for (int si = 0; si < 4; ++si) {
        const Shape& s = *shapes[si];
        int analyticHits = 0, marchHits = 0, inside = 0, missCheck = 0;
        int mismatches = 0, surfaceErrors = 0;
        for (int i = 0; i < 1500; ++i) {
            const Transform xf(Vector3(rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)), rng.rotation());
            // Metade dos raios aponta para perto do centro (acertos garantidos), metade é livre.
            const Vector3 o(rng.range(-6, 6), rng.range(-6, 6), rng.range(-6, 6));
            Vector3 d = rng.unit();
            if (i % 2 == 0) {
                const Vector3 aim = xf.position + Vector3(rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1));
                d = (aim - o).normalizedOr(d);
            }
            ShapeRayHit h;
            const bool hit = raycastShape(s, xf, o, d, maxT, h);

            // Oráculo: primeira amostra do raio dentro da forma.
            Real firstInside = -1;
            for (Real t = 0; t <= maxT; t += step) {
                if (shapeContainsPoint(s, xf, o + d * t)) { firstInside = t; break; }
            }
            if (firstInside >= 0) {
                ++marchHits;
                if (!hit) { ++mismatches; continue; }               // a marcha achou o que o raio não achou
                if (firstInside == 0) {
                    if (!h.startedInside || h.distance != 0) ++mismatches;
                    ++inside;
                } else if (h.distance > firstInside + Real(1e-3) || h.distance < firstInside - step - Real(1e-3)) {
                    ++mismatches;
                }
            }
            if (hit) {
                ++analyticHits;
                if (h.startedInside) continue;
                // Propriedades da superfície atingida.
                const Real delta = Real(0.003);
                const bool onSurface = shapeContainsPoint(s, xf, h.point - h.normal * delta) &&
                                       !shapeContainsPoint(s, xf, h.point + h.normal * delta);
                const bool entering = h.normal.dot(d) < Real(1e-3);
                const bool normalUnit = near(h.normal.lengthSquared(), 1, Real(1e-3));
                const bool pointOk = nearV(h.point, o + d * h.distance, Real(1e-4));
                if (!(onSurface && entering && normalUnit && pointOk)) ++surfaceErrors;
                // Antes do acerto o raio não pode ter tocado a forma.
                for (Real t = 0; t < h.distance - Real(0.01); t += step * 5) {
                    if (shapeContainsPoint(s, xf, o + d * t)) { ++mismatches; break; }
                }
            } else {
                ++missCheck;
            }
        }
        std::printf("  [info] %-9s raios: acertos=%d (marcha=%d, origem dentro=%d), erros=%d, superficie=%d\n",
                    names[si], analyticHits, marchHits, inside, mismatches, surfaceErrors);
        CHECK(mismatches == 0);
        CHECK(surfaceErrors == 0);
        CHECK(analyticHits > 200 && missCheck > 200);   // o teste exercita acertos E erros
    }
}

// =============================================================================
// DynamicAABBTree::raycast contra força bruta
// =============================================================================
static void testTreeRaycast() {
    Rng rng(77);
    DynamicAABBTree tree;
    std::vector<ProxyId> ids;
    std::vector<AABB> tight;   // por índice de `ids` (AABB justo)

    for (int i = 0; i < 300; ++i) {
        const Vector3 c(rng.range(-20, 20), rng.range(-20, 20), rng.range(-20, 20));
        const AABB b = AABB::fromCenterHalfExtents(c, Vector3(rng.range(Real(0.2), 2), rng.range(Real(0.2), 2),
                                                              rng.range(Real(0.2), 2)));
        tight.push_back(b);
        ids.push_back(tree.createProxy(b, static_cast<std::uint32_t>(i)));
    }
    // Remove um terço e move outro terço (a árvore é rebalanceada).
    for (int i = 0; i < 300; i += 3) { tree.destroyProxy(ids[static_cast<std::size_t>(i)]); ids[static_cast<std::size_t>(i)] = kNullProxy; }
    for (int i = 1; i < 300; i += 3) {
        const Vector3 c(rng.range(-20, 20), rng.range(-20, 20), rng.range(-20, 20));
        tight[static_cast<std::size_t>(i)] = AABB::fromCenterHalfExtents(c, Vector3(1, 1, 1));
        tree.moveProxy(ids[static_cast<std::size_t>(i)], tight[static_cast<std::size_t>(i)], Vector3::zero());
    }
    CHECK(tree.validate());

    for (const Real slack : {Real(0), Real(0.7)}) {
        int rays = 0, nonEmpty = 0;
        for (int r = 0; r < 600; ++r) {
            const Vector3 o(rng.range(-25, 25), rng.range(-25, 25), rng.range(-25, 25));
            const Vector3 d = rng.unit();
            const Real maxT = rng.range(2, 60);
            ++rays;

            // Força bruta: folhas cujo Fat AABB (+ folga) o raio toca.
            std::set<std::uint32_t> expected;
            Real bestExpected = kInf;
            for (std::size_t i = 0; i < ids.size(); ++i) {
                if (ids[i] == kNullProxy) continue;
                Real te;
                if (tree.getFatAABB(ids[i]).fattened(slack).intersectsRay(o, d, maxT, te)) {
                    expected.insert(static_cast<std::uint32_t>(i));
                    bestExpected = std::min(bestExpected, te);
                }
            }

            // (a) Sem podar: reporta exatamente o conjunto esperado, sem repetição.
            std::set<std::uint32_t> got;
            int calls = 0;
            tree.raycast(o, d, maxT, slack, [&](ProxyId id, Real m) {
                got.insert(tree.getUserData(id));
                ++calls;
                return m;
            });
            CHECK(got == expected);
            CHECK(calls == static_cast<int>(got.size()));
            if (!expected.empty()) ++nonEmpty;

            // (b) Podando pelo acerto mais próximo: o mais próximo é preservado e ninguém além dele é visitado
            //     depois que o alcance encolhe (todo visitado tem entrada <= alcance vigente).
            Real best = kInf;
            tree.raycast(o, d, maxT, slack, [&](ProxyId id, Real m) {
                Real te = 0;
                const bool h = tree.getFatAABB(id).fattened(slack).intersectsRay(o, d, m, te);
                CHECK(h);
                best = std::min(best, te);
                return te;
            });
            CHECK(best == bestExpected);   // inclusive "ninguém" (kInf == kInf)

            // (c) Retornar valor negativo interrompe.
            int calls2 = 0;
            tree.raycast(o, d, maxT, slack, [&](ProxyId, Real) { ++calls2; return Real(-1); });
            CHECK(calls2 == (expected.empty() ? 0 : 1));
        }
        CHECK(nonEmpty > 100);
        (void)rays;
    }

    // Raio inválido não visita nada.
    int calls = 0;
    tree.raycast(Vector3(0, 0, 0), Vector3(1, 0, 0), kNaN, Real(0), [&](ProxyId, Real m) { ++calls; return m; });
    tree.raycast(Vector3(0, 0, 0), Vector3(1, 0, 0), Real(-1), Real(0), [&](ProxyId, Real m) { ++calls; return m; });
    CHECK(calls == 0);
    DynamicAABBTree empty;
    empty.raycast(Vector3(0, 0, 0), Vector3(1, 0, 0), 10, Real(0), [&](ProxyId, Real m) { ++calls; return m; });
    CHECK(calls == 0);
}

// =============================================================================
// CollisionWorld isolado
// =============================================================================
static void testCollisionWorldQueries() {
    CollisionWorld cw;
    auto box = std::make_shared<BoxShape>(Vector3(Real(0.5), Real(0.5), Real(0.5)));
    auto sphere = std::make_shared<SphereShape>(Real(0.5));
    cw.addObject(0, box, Transform(Vector3(0, 0, 0)), true);
    cw.addObject(1, box, Transform(Vector3(0, 3, 0)), true);
    cw.addObject(2, sphere, Transform(Vector3(0, 6, 0)), true);
    const auto all = [](BodyId) { return true; };

    // Raio vindo de cima descendo: acerta o mais próximo (esfera em y = 6, topo em 6,5).
    RaycastHit hit;
    CHECK(cw.raycastClosest(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, all));
    CHECK(hit.body == 2 && near(hit.distance, Real(3.5)) && nearV(hit.normal, Vector3(0, 1, 0)));

    // Filtro: ignorando o 2, acerta o 1; ignorando 1 e 2, acerta o 0.
    CHECK(cw.raycastClosest(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, [](BodyId id) { return id != 2; }));
    CHECK(hit.body == 1 && near(hit.distance, Real(6.5)));
    CHECK(cw.raycastClosest(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, [](BodyId id) { return id == 0; }));
    CHECK(hit.body == 0 && near(hit.distance, Real(9.5)));
    CHECK(!cw.raycastClosest(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, [](BodyId) { return false; }));
    CHECK(!cw.raycastClosest(Vector3(0, 10, 0), Vector3(0, -1, 0), Real(3.4), hit, all));       // curto
    CHECK(!cw.raycastClosest(Vector3(5, 10, 0), Vector3(0, -1, 0), 100, hit, all));             // ao lado

    // raycastAll: todos, e o callback pode interromper.
    std::vector<BodyId> order;
    cw.raycastAll(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, all, [&](const RaycastHit& h) { order.push_back(h.body); return true; });
    std::sort(order.begin(), order.end());
    CHECK((order == std::vector<BodyId>{0, 1, 2}));
    int count = 0;
    cw.raycastAll(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, all, [&](const RaycastHit&) { ++count; return false; });
    CHECK(count == 1);

    // Empate exato de distância: vence o menor id (duas caixas lado a lado, mesma face).
    CollisionWorld tie;
    tie.addObject(5, box, Transform(Vector3(0, 0, 0)), true);
    tie.addObject(3, box, Transform(Vector3(0, 0, 0)), true);   // mesma pose: mesmo acerto
    CHECK(tie.raycastClosest(Vector3(0, 5, 0), Vector3(0, -1, 0), 100, hit, all));
    CHECK(hit.body == 3 && near(hit.distance, Real(4.5)));

    // Origem dentro de um objeto.
    CHECK(cw.raycastClosest(Vector3(0, 3, 0), Vector3(1, 0, 0), 100, hit, all));
    CHECK(hit.body == 1 && hit.startedInside && hit.distance == 0);

    // Overlap de forma (GJK exato): esfera de teste raio 0,3.
    const SphereShape probe(Real(0.3));
    std::vector<BodyId> found;
    auto collect = [&](BodyId id) { found.push_back(id); return true; };
    cw.overlapShape(probe, Transform(Vector3(0, Real(0.7), 0)), all, collect);   // 0,2 de distância da face do 0 (topo em 0,5)
    CHECK((found == std::vector<BodyId>{0}));
    found.clear();
    cw.overlapShape(probe, Transform(Vector3(0, Real(1.0), 0)), all, collect);   // 0,5 acima: não toca (0,3 < 0,5)
    CHECK(found.empty());
    found.clear();
    cw.overlapShape(probe, Transform(Vector3(0, Real(1.0), 0)), all, collect);
    // Esfera contra esfera: raio 0,5 + 0,3 = 0,8 → toca a partir de 0,8 de distância.
    cw.overlapShape(probe, Transform(Vector3(Real(0.79), 6, 0)), all, collect);
    CHECK((found == std::vector<BodyId>{2}));
    found.clear();
    cw.overlapShape(probe, Transform(Vector3(Real(0.81), 6, 0)), all, collect);
    CHECK(found.empty());
    // Caixa de teste cobrindo vários objetos.
    const BoxShape big(Vector3(2, Real(3.5), 2));
    found.clear();
    cw.overlapShape(big, Transform(Vector3(0, Real(1.5), 0)), all, collect);
    std::sort(found.begin(), found.end());
    CHECK((found == std::vector<BodyId>{0, 1}));   // y ∈ [-2, 5]: alcança as caixas, não a esfera (base em 5,5)
    // E encostando exatamente na base da esfera (y = 5,5) ela entra: toque conta como overlap.
    const BoxShape touching(Vector3(2, 4, 2));
    found.clear();
    cw.overlapShape(touching, Transform(Vector3(0, Real(1.5), 0)), all, collect);
    std::sort(found.begin(), found.end());
    CHECK((found == std::vector<BodyId>{0, 1, 2}));

    // Ponto dentro / fora.
    found.clear();
    cw.overlapPoint(Vector3(Real(0.2), Real(3.1), 0), all, collect);
    CHECK((found == std::vector<BodyId>{1}));
    found.clear();
    cw.overlapPoint(Vector3(5, 5, 5), all, collect);
    CHECK(found.empty());
    cw.overlapPoint(Vector3(kNaN, 0, 0), all, collect);
    CHECK(found.empty());

    // setQueryPose + setQuerySlack: a pose de consulta pode andar na frente da árvore.
    CollisionWorld moved;
    moved.addObject(0, box, Transform(Vector3(0, 0, 0)), true);   // folha na árvore: x ∈ [-0,6; 0,6]
    moved.setQueryPose(0, Transform(Vector3(2, 0, 0)));           // pose atual: x ∈ [1,5; 2,5]
    CHECK(!moved.raycastClosest(Vector3(2, 5, 0), Vector3(0, -1, 0), 100, hit, all));   // folga 0: árvore poda
    moved.setQuerySlack(Real(2));
    CHECK(moved.raycastClosest(Vector3(2, 5, 0), Vector3(0, -1, 0), 100, hit, all));
    CHECK(near(hit.distance, Real(4.5)));
    moved.setQuerySlack(kNaN);                                    // inválido → volta a 0
    CHECK(moved.getQuerySlack() == 0);
    moved.setQueryPose(0, Transform(Vector3(kNaN, 0, 0)));        // pose inválida é ignorada
    moved.setQuerySlack(Real(2));
    CHECK(moved.raycastClosest(Vector3(2, 5, 0), Vector3(0, -1, 0), 100, hit, all) && near(hit.distance, Real(4.5)));
}

// =============================================================================
// PhysicsWorld
// =============================================================================
static RigidBodyDesc groundDesc() {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(Vector3(50, 1, 50));
    d.transform = Transform(Vector3(0, -1, 0));   // topo em y = 0
    return d;
}
static RigidBodyDesc boxDesc(const Vector3& pos, Real half = Real(0.5)) {
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(half, half, half));
    d.mass = 1;
    d.transform = Transform(pos);
    return d;
}
static RigidBodyDesc sphereDesc(const Vector3& pos, Real r) {
    RigidBodyDesc d;
    d.shape = std::make_shared<SphereShape>(r);
    d.mass = 1;
    d.transform = Transform(pos);
    return d;
}

static void testWorldQueries() {
    PhysicsWorld w;
    const BodyId ground = w.createBody(groundDesc());
    const BodyId a = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));            // topo em y = 1
    const BodyId b = w.createBody(sphereDesc(Vector3(3, 1, 0), 1));              // topo em y = 2
    RaycastHit hit;

    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit));
    CHECK(hit.body == a && near(hit.distance, 9) && nearV(hit.normal, Vector3(0, 1, 0)) && nearV(hit.point, Vector3(0, 1, 0)));
    CHECK(w.raycast(Vector3(3, 10, 0), Vector3(0, -1, 0), 100, hit));
    CHECK(hit.body == b && near(hit.distance, 8));
    CHECK(w.raycast(Vector3(10, 10, 10), Vector3(0, -1, 0), 100, hit));
    CHECK(hit.body == ground && near(hit.distance, 10));

    // A direção não precisa ser unitária; a distância continua em metros.
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -7, 0), 100, hit) && hit.body == a && near(hit.distance, 9));
    // Alcance: curto erra, infinito acerta.
    CHECK(!w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), Real(8.9), hit));
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), kInf, hit));

    // Filtros.
    QueryFilter f;
    f.ignoreBody = a;
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, f) && hit.body == ground && near(hit.distance, 10));
    f = QueryFilter();
    f.includeStatic = false;
    CHECK(!w.raycast(Vector3(10, 10, 10), Vector3(0, -1, 0), 100, hit, f));
    f = QueryFilter();
    f.includeDynamic = false;
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, f) && hit.body == ground);
    f = QueryFilter();
    f.predicate = [&](BodyId id) { return id == b; };
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(1, -0.1f, 0), 100, hit, f) == false || hit.body == b);
    CHECK(w.raycast(Vector3(3, 10, 0), Vector3(0, -1, 0), 100, hit, f) && hit.body == b);
    f = QueryFilter();
    f.includeKinematic = false;   // não há corpos cinemáticos: nada muda
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit, f) && hit.body == a);

    // raycastAll: ordenado por distância (caixa a 9, chão a 10).
    std::vector<RaycastHit> hits;
    CHECK(w.raycastAll(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hits) == 2);
    CHECK(hits[0].body == a && near(hits[0].distance, 9) && hits[1].body == ground && near(hits[1].distance, 10));

    // Origem dentro do corpo.
    CHECK(w.raycast(Vector3(0, Real(0.5), 0), Vector3(1, 0, 0), 100, hit));
    CHECK(hit.body == a && hit.startedInside && hit.distance == 0);

    // Entradas inválidas: sem exceção, sem acerto, hit intocado.
    RaycastHit sentinel;
    sentinel.distance = Real(42);
    RaycastHit hh = sentinel;
    CHECK(!w.raycast(Vector3(kNaN, 10, 0), Vector3(0, -1, 0), 100, hh));
    CHECK(!w.raycast(Vector3(0, 10, 0), Vector3(0, 0, 0), 100, hh));          // direção nula
    CHECK(!w.raycast(Vector3(0, 10, 0), Vector3(kNaN, -1, 0), 100, hh));
    CHECK(!w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), kNaN, hh));
    CHECK(!w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), Real(-5), hh));
    CHECK(!w.raycast(Vector3(kInf, 10, 0), Vector3(0, -1, 0), 100, hh));
    CHECK(hh.distance == Real(42));
    CHECK(w.raycastAll(Vector3(0, 10, 0), Vector3(0, 0, 0), 100, hits) == 0 && hits.empty());

    // Overlaps.
    std::vector<BodyId> bodies;
    CHECK(w.overlapPoint(Vector3(0, Real(0.5), 0), bodies) == 1 && bodies[0] == a);
    CHECK(w.overlapPoint(Vector3(0, Real(-0.5), 0), bodies) == 1 && bodies[0] == ground);
    CHECK(w.overlapPoint(Vector3(0, 5, 0), bodies) == 0);
    CHECK(w.overlapPoint(Vector3(kNaN, 5, 0), bodies) == 0);
    CHECK(w.overlapSphere(Vector3(0, Real(0.5), 0), Real(0.1), bodies) == 1 && bodies[0] == a);
    // Esfera de raio 1 em (1,4; 0,5; 0): toca o chão (0,5 < 1), a caixa (0,9 < 1) e a esfera B (1,68 < 2).
    CHECK(w.overlapSphere(Vector3(Real(1.4), Real(0.5), 0), 1, bodies) == 3);
    CHECK(bodies[0] == ground && bodies[1] == a && bodies[2] == b);       // ordem crescente de id
    f = QueryFilter();
    f.includeStatic = false;
    CHECK(w.overlapSphere(Vector3(Real(1.4), Real(0.5), 0), 1, bodies, f) == 2 && bodies[0] == a && bodies[1] == b);
    CHECK(w.overlapSphere(Vector3(0, 0, 0), Real(0), bodies) == 0);       // raio inválido
    CHECK(w.overlapSphere(Vector3(0, 0, 0), kNaN, bodies) == 0);
    CHECK(w.overlapSphere(Vector3(kNaN, 0, 0), 1, bodies) == 0);
    const BoxShape probe(Vector3(Real(0.2), Real(0.2), Real(0.2)));
    CHECK(w.overlapShape(probe, Transform(Vector3(Real(0.6), Real(1.15), 0)), bodies) == 1 && bodies[0] == a);   // canto da caixa
    CHECK(w.overlapShape(probe, Transform(Vector3(Real(0.8), Real(1.3), 0)), bodies) == 0);

    // Corpo teletransportado: as consultas veem a nova pose imediatamente.
    w.setBodyTransform(a, Transform(Vector3(20, Real(0.5), 0)));
    CHECK(w.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0), 100, hit) && hit.body == ground);
    CHECK(w.raycast(Vector3(20, 10, 0), Vector3(0, -1, 0), 100, hit) && hit.body == a && near(hit.distance, 9));

    // Corpo destruído some das consultas.
    w.destroyBody(b);
    CHECK(w.raycast(Vector3(3, 10, 0), Vector3(0, -1, 0), 100, hit) && hit.body == ground);
    CHECK(w.overlapPoint(Vector3(3, 1, 0), bodies) == 0);
}

// Cena dinâmica: depois de CADA passo as consultas têm de concordar com a força bruta sobre as poses atuais.
static void testQueriesTrackMovingBodies() {
    PhysicsWorld w;
    w.createBody(groundDesc());
    Rng rng(321);
    std::vector<BodyId> ids;
    for (int i = 0; i < 40; ++i) {
        RigidBodyDesc d = (i % 3 == 0) ? sphereDesc(Vector3(rng.range(-6, 6), rng.range(1, 14), rng.range(-6, 6)), rng.range(Real(0.3), Real(0.7)))
                                       : boxDesc(Vector3(rng.range(-6, 6), rng.range(1, 14), rng.range(-6, 6)), rng.range(Real(0.3), Real(0.6)));
        d.transform.rotation = rng.rotation();
        d.linearVelocity = Vector3(rng.range(-30, 30), rng.range(-30, 30), rng.range(-30, 30));   // rápidos: 0,5 m por passo
        d.angularVelocity = Vector3(rng.range(-20, 20), rng.range(-20, 20), rng.range(-20, 20));
        d.restitution = Real(0.6);
        ids.push_back(w.createBody(d));
    }

    int rayChecks = 0, rayHits = 0, overlapChecks = 0, overlapHits = 0, mismatches = 0;
    for (int step = 0; step < 150; ++step) {
        w.step();
        for (int q = 0; q < 12; ++q) {
            // --- raycast: melhor acerto por força bruta (poses atuais) ---
            const Vector3 o(rng.range(-8, 8), rng.range(0, 16), rng.range(-8, 8));
            const Vector3 d = rng.unit();
            const Real maxT = rng.range(2, 30);
            RaycastHit expected;
            bool found = false;
            w.forEachBody([&](BodyId id, const RigidBody& b) {
                ShapeRayHit h;
                if (!b.getShape() || !raycastShape(*b.getShape(), b.getTransform(), o, d, maxT, h)) return;
                if (!found || h.distance < expected.distance || (h.distance == expected.distance && id < expected.body)) {
                    expected.body = id;
                    expected.distance = h.distance;
                    found = true;
                }
            });
            RaycastHit got;
            const bool hit = w.raycast(o, d, maxT, got);
            ++rayChecks;
            if (found) ++rayHits;
            if (hit != found || (hit && (got.body != expected.body || !near(got.distance, expected.distance, Real(1e-3))))) ++mismatches;

            // --- overlapSphere: força bruta com GJK sobre as poses atuais ---
            const Vector3 c(rng.range(-8, 8), rng.range(0, 8), rng.range(-8, 8));
            const Real rad = rng.range(Real(0.3), Real(2));
            const SphereShape probe(rad);
            std::vector<BodyId> exp;
            w.forEachBody([&](BodyId id, const RigidBody& b) {
                if (!b.getShape()) return;
                const MinkowskiSupport sup(probe, Transform(c), *b.getShape(), b.getTransform());
                const GJKResult r = GJK::compute(sup);
                if (r.intersecting || !r.converged || r.distance <= sup.getRadiusA() + sup.getRadiusB()) exp.push_back(id);
            });
            std::vector<BodyId> gotIds;
            w.overlapSphere(c, rad, gotIds);
            ++overlapChecks;
            if (!exp.empty()) ++overlapHits;
            if (exp != gotIds) ++mismatches;
        }
    }
    std::printf("  [info] cena dinamica: raios=%d (acertos %d), overlaps=%d (nao vazios %d), divergencias=%d\n",
                rayChecks, rayHits, overlapChecks, overlapHits, mismatches);
    CHECK(mismatches == 0);
    CHECK(rayHits > 300 && overlapHits > 200);
    // A folga de consulta é dinâmica: cresce com a velocidade, não é um número mágico.
    CHECK(w.getCollisionWorld().getQuerySlack() > 0);
    PhysicsWorld still;
    still.createBody(groundDesc());
    still.step();
    CHECK(still.getCollisionWorld().getQuerySlack() <= 2 * kLinearSlop);   // nada se move: folga mínima
}

// =============================================================================
// Debug draw
// =============================================================================
static void testDebugDraw() {
    PhysicsWorld w;
    w.createBody(groundDesc());
    const BodyId bx = w.createBody(boxDesc(Vector3(0, Real(0.5), 0)));
    w.createBody(sphereDesc(Vector3(4, 1, 0), 1));

    DebugDrawSettings s;
    s.flags = kDebugShapes;
    DebugLineBuffer buf;
    debugDraw(w, buf, s);
    // chão (12 arestas) + caixa (12) + esfera (3 círculos × 24 segmentos).
    CHECK(buf.size() == 12 + 12 + 3 * 24);

    // Cada aresta da caixa aparece uma vez e liga dois cantos da caixa.
    {
        const BoxShape ref(Vector3(Real(0.5), Real(0.5), Real(0.5)));
        std::set<std::pair<std::vector<long>, std::vector<long>>> edges;
        auto key = [](float x, float y, float z) {
            return std::vector<long>{std::lround(x * 1000), std::lround(y * 1000), std::lround(z * 1000)};
        };
        DebugLineBuffer one;
        debugDrawShape(ref, w.getBody(bx).getTransform(), one, DebugColor{});
        CHECK(one.size() == 12);
        for (const DebugLine& l : one.lines()) {
            auto k1 = key(l.ax, l.ay, l.az), k2 = key(l.bx, l.by, l.bz);
            if (k2 < k1) std::swap(k1, k2);
            edges.insert({k1, k2});
            // Cada ponta é um canto: |x|, |y|, |z| = 0,5 em torno do centro (0, 0,5, 0).
            CHECK(std::abs(std::abs(l.ax) - 0.5f) < 1e-4f && std::abs(std::abs(l.ay - 0.5f) - 0.5f) < 1e-4f &&
                  std::abs(std::abs(l.az) - 0.5f) < 1e-4f);
        }
        CHECK(edges.size() == 12);
    }

    s.flags = 0;
    buf.clear();
    debugDraw(w, buf, s);
    CHECK(buf.size() == 0);

    s.flags = kDebugAABBs;
    debugDraw(w, buf, s);
    CHECK(buf.size() == 3 * 12);
    buf.clear();
    s.flags = kDebugFatAABBs;
    debugDraw(w, buf, s);
    CHECK(buf.size() == 3 * 12);
    buf.clear();
    s.flags = kDebugCenterOfMass | kDebugVelocities | kDebugBodyAxes;
    debugDraw(w, buf, s);
    CHECK(buf.size() == 3 * 3 /*COM*/ + 2 /*velocidades dos 2 dinâmicos*/ + 3 * 3 /*eixos*/);
    buf.clear();
    s.flags = kDebugShapes;
    s.sphereSegments = 1;   // clamp em 3
    debugDraw(w, buf, s);
    CHECK(buf.size() == 12 + 12 + 3 * 3);

    // Contatos: depois de assentar há pontos e normais.
    for (int i = 0; i < 60; ++i) w.step();
    buf.clear();
    s = DebugDrawSettings();
    s.flags = kDebugContactPoints | kDebugContactNormals;
    debugDraw(w, buf, s);
    CHECK(buf.size() > 0);
    int normals = 0;
    for (const DebugLine& l : buf.lines()) {
        CHECK(std::isfinite(l.ax) && std::isfinite(l.ay) && std::isfinite(l.az) && std::isfinite(l.bx) &&
              std::isfinite(l.by) && std::isfinite(l.bz));
        if (l.r == s.normalColor.r && l.g == s.normalColor.g && l.b == s.normalColor.b) ++normals;
    }
    CHECK(normals == w.getStats().contactPoints);   // uma normal por ponto de contato

    // Juntas: ancoragens (cruz de 3 linhas cada) + linha entre elas (+ eixo, na dobradiça).
    PhysicsWorld jw;
    const BodyId j0 = jw.createBody(boxDesc(Vector3(0, 5, 0)));
    const BodyId j1 = jw.createBody(boxDesc(Vector3(2, 5, 0)));
    jw.createBallSocketJoint(j0, j1, Vector3(1, 5, 0));
    jw.createHingeJoint(j0, j1, Vector3(1, 5, 0), Vector3(0, 0, 1));
    s = DebugDrawSettings();
    s.flags = kDebugJoints;
    buf.clear();
    debugDraw(jw, buf, s);
    CHECK(buf.size() == (3 + 3 + 1) + (3 + 3 + 1 + 1));

    // Pose interpolada: com α = 0,5 as formas ficam entre as duas poses físicas.
    PhysicsWorld iw;
    const BodyId f = iw.createBody(boxDesc(Vector3(0, 50, 0)));
    iw.update(kFixedTimeStep);
    iw.update(kFixedTimeStep * Real(0.5));
    DebugDrawSettings si;
    si.flags = kDebugCenterOfMass;
    si.useInterpolatedPose = false;
    DebugLineBuffer phys, interp;
    debugDraw(iw, phys, si);
    si.useInterpolatedPose = true;
    debugDraw(iw, interp, si);
    const Real yPhys = iw.getBody(f).getTransform().position.y;
    const Real yInterp = iw.getInterpolatedTransform(f).position.y;
    CHECK(yInterp > yPhys);   // o render está um passo "atrás" da física
    // O centro da cruz é o ponto médio da primeira linha.
    CHECK(near(Real(0.5f * (phys.lines()[0].ay + phys.lines()[0].by)), yPhys, Real(1e-3)));
    CHECK(near(Real(0.5f * (interp.lines()[0].ay + interp.lines()[0].by)), yInterp, Real(1e-3)));

    // Renderizador que sobrescreve drawPoint recebe pontos (e não as 3 linhas).
    struct Counting : IDebugRenderer {
        int lines = 0, points = 0;
        void drawLine(const Vector3&, const Vector3&, const DebugColor&) override { ++lines; }
        void drawPoint(const Vector3&, Real, const DebugColor&) override { ++points; }
    } counting;
    s.flags = kDebugCenterOfMass;
    debugDraw(w, counting, s);
    CHECK(counting.points == 3 && counting.lines == 0);
}

int main() {
    testAABBRay();
    testSphereRay();
    testBoxRay();
    testPolyhedronRay();
    testRandomRaysAgainstOracle();
    testTreeRaycast();
    testCollisionWorldQueries();
    testWorldQueries();
    testQueriesTrackMovingBodies();
    testDebugDraw();

    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
