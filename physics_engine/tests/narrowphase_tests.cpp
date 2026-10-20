// =============================================================================
// narrowphase_tests.cpp — testes unitários da Fase 5 (sem frameworks externos).
//
// Estratégia: GJK/EPA são comparados com o Teorema do Eixo Separador (SAT),
// um método independente e exato para poliedros convexos:
//   - GJK.intersecting  == (SAT: sobreposição em todos os eixos)
//   - EPA.depth         == menor sobreposição entre todos os eixos (SAT)
//   - GJK.distance      : certificado de otimalidade (A e B em lados opostos do
//                         plano que passa pelos pontos mais próximos).
// =============================================================================
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

#include "physics/collision/narrowphase/ContactManifold.h"
#include "physics/collision/narrowphase/EPA.h"
#include "physics/collision/narrowphase/GJK.h"
#include "physics/geometry/BoxShape.h"
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

static const Real kTol = Real(2e-3);   // tolerância geométrica (float)

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * next(); }
    Vector3 vec(Real lo, Real hi) { return Vector3(range(lo, hi), range(lo, hi), range(lo, hi)); }
};

static Transform randomTransform(Rng& r, Real extent) {
    const Vector3 axis = r.vec(-1, 1);
    return Transform(r.vec(-extent, extent), Quaternion::fromAxisAngle(axis, r.range(0, kTwoPi)));
}

// ---- Formas de teste ----------------------------------------------------------------------------------
static PolyhedronShape makeTetra(Real s) {
    const std::vector<Vector3> v = {Vector3(0, 0, 0), Vector3(s, 0, 0), Vector3(0, s, 0), Vector3(0, 0, s)};
    return PolyhedronShape(v, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}});
}

static PolyhedronShape makePrism(int N, Real R, Real H) {
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
    return PolyhedronShape(pv, pf);
}

static std::vector<Vector3> worldVertices(const PolyhedronShape& p, const Transform& xf) {
    std::vector<Vector3> r;
    for (const HEVertex& v : p.getTopology().vertices()) r.push_back(xf.transformPoint(v.position));
    return r;
}

// ---- SAT (referência independente) -----------------------------------------------------------------------
struct SatResult {
    Real minOverlap = kRealMax;   // < 0 => separados (módulo = maior folga entre eixos)
    Real secondOverlap = kRealMax;  // menor sobreposição entre eixos NÃO paralelos ao melhor
    Vector3 axis;                 // do melhor eixo, orientado de A para B
};

static SatResult satTest(const PolyhedronShape& A, const Transform& xa, const PolyhedronShape& B, const Transform& xb) {
    const std::vector<Vector3> va = worldVertices(A, xa), vb = worldVertices(B, xb);
    Vector3 ca, cb;
    for (const Vector3& p : va) ca += p;
    for (const Vector3& p : vb) cb += p;
    ca /= Real(va.size());
    cb /= Real(vb.size());

    std::vector<Vector3> axes, edgesA, edgesB;
    for (const HEFace& f : A.getTopology().faces()) axes.push_back(xa.rotation.rotate(f.normal));
    for (const HEFace& f : B.getTopology().faces()) axes.push_back(xb.rotation.rotate(f.normal));
    for (int e = 0; e < A.getTopology().getEdgeCount(); ++e)
        if (e < A.getTopology().getEdge(e).twin) edgesA.push_back(xa.rotation.rotate(A.getTopology().edgeVector(e)));
    for (int e = 0; e < B.getTopology().getEdgeCount(); ++e)
        if (e < B.getTopology().getEdge(e).twin) edgesB.push_back(xb.rotation.rotate(B.getTopology().edgeVector(e)));
    for (const Vector3& ea : edgesA)
        for (const Vector3& eb : edgesB) {
            Vector3 c = ea.cross(eb);
            if (c.lengthSquared() > Real(1e-10) * ea.lengthSquared() * eb.lengthSquared()) axes.push_back(c);
        }

    struct AxisOverlap { Vector3 axis; Real overlap; };
    std::vector<AxisOverlap> all;
    SatResult r;
    for (Vector3 ax : axes) {
        if (ax.normalize() <= Real(0)) continue;
        Real minA = kRealMax, maxA = -kRealMax, minB = kRealMax, maxB = -kRealMax;
        for (const Vector3& p : va) { const Real d = p.dot(ax); minA = std::min(minA, d); maxA = std::max(maxA, d); }
        for (const Vector3& p : vb) { const Real d = p.dot(ax); minB = std::min(minB, d); maxB = std::max(maxB, d); }
        const Real overlap = std::min(maxA - minB, maxB - minA);
        if ((cb - ca).dot(ax) < Real(0)) ax = -ax;    // orienta de A para B
        all.push_back({ax, overlap});
        if (overlap < r.minOverlap) { r.minOverlap = overlap; r.axis = ax; }
    }
    for (const AxisOverlap& o : all)
        if (std::abs(o.axis.dot(r.axis)) < Real(0.999)) r.secondOverlap = std::min(r.secondOverlap, o.overlap);
    return r;
}

// ---- Diretórios / utilidades ----------------------------------------------------------------------------------
static bool gjkIntersects(const Shape& a, const Transform& xa, const Shape& b, const Transform& xb) {
    return GJK::compute(MinkowskiSupport(a, xa, b, xb)).intersecting;
}

// =============================================================================
static void testSimplexClosestPoint() {
    // Segmento: origem projeta no interior.
    {
        Simplex s;
        SupportPoint p0, p1;
        p0.point = Vector3(-1, 1, 0); p1.point = Vector3(1, 1, 0);
        s.add(p0); s.add(p1);
        Real lam[4];
        const Vector3 q = GJK::closestPointOnSimplex(s, lam);
        CHECK(q.nearlyEqual(Vector3(0, 1, 0)) && s.size == 2);
        CHECK(nearlyEqual(lam[0], Real(0.5)) && nearlyEqual(lam[1], Real(0.5)));
    }
    // Segmento: mais perto de uma ponta → reduz para 1 ponto.
    {
        Simplex s;
        SupportPoint p0, p1;
        p0.point = Vector3(2, 1, 0); p1.point = Vector3(4, 1, 0);
        s.add(p0); s.add(p1);
        Real lam[4];
        const Vector3 q = GJK::closestPointOnSimplex(s, lam);
        CHECK(q.nearlyEqual(Vector3(2, 1, 0)) && s.size == 1 && nearlyEqual(lam[0], Real(1)));
    }
    // Triângulo: pé da origem no interior; distância = altura.
    {
        Simplex s;
        SupportPoint a, b, c;
        a.point = Vector3(-1, -1, 2); b.point = Vector3(2, -1, 2); c.point = Vector3(-1, 2, 2);
        s.add(a); s.add(b); s.add(c);
        Real lam[4];
        const Vector3 q = GJK::closestPointOnSimplex(s, lam);
        CHECK(q.nearlyEqual(Vector3(0, 0, 2)) && s.size == 3);
        CHECK(nearlyEqual(lam[0] + lam[1] + lam[2], Real(1)));
    }
    // Tetraedro que contém a origem.
    {
        Simplex s;
        const Vector3 pts[4] = {Vector3(1, 1, 1), Vector3(-1, -1, 1), Vector3(-1, 1, -1), Vector3(1, -1, -1)};
        for (const Vector3& p : pts) { SupportPoint sp; sp.point = p; s.add(sp); }
        Real lam[4];
        const Vector3 q = GJK::closestPointOnSimplex(s, lam);
        CHECK(q.isZero(0) && s.size == 4);
    }
    // Tetraedro com a origem fora: reduz para a face/aresta/vértice mais próximo.
    {
        Simplex s;
        const Vector3 pts[4] = {Vector3(1, 1, 3), Vector3(-1, -1, 3), Vector3(-1, 1, 5), Vector3(1, -1, 5)};
        for (const Vector3& p : pts) { SupportPoint sp; sp.point = p; s.add(sp); }
        Real lam[4];
        const Vector3 q = GJK::closestPointOnSimplex(s, lam);
        CHECK(s.size < 4 && nearlyEqual(q.z, Real(3), kTol));
    }
    // Triângulo degenerado (colinear): sem NaN.
    {
        Simplex s;
        SupportPoint a, b, c;
        a.point = Vector3(1, 1, 0); b.point = Vector3(2, 1, 0); c.point = Vector3(3, 1, 0);
        s.add(a); s.add(b); s.add(c);
        Real lam[4];
        const Vector3 q = GJK::closestPointOnSimplex(s, lam);
        CHECK(q.isFinite() && q.nearlyEqual(Vector3(1, 1, 0)));
    }
}

static void testGJKBoxesAnalytic() {
    BoxShape box(Vector3(1, 1, 1));
    auto at = [](Real x, Real y, Real z) { return Transform(Vector3(x, y, z)); };

    // Separadas ao longo de x: folga de 0.5.
    GJKResult r = GJK::compute(MinkowskiSupport(box, at(0, 0, 0), box, at(2.5f, 0, 0)));
    CHECK(!r.intersecting && nearlyEqual(r.distance, Real(0.5), kTol));
    CHECK(r.closestA.nearlyEqual(Vector3(1, r.closestA.y, r.closestA.z), kTol));
    CHECK(r.closestB.nearlyEqual(Vector3(1.5f, r.closestB.y, r.closestB.z), kTol));
    CHECK(nearlyEqual((r.closestB - r.closestA).length(), r.distance, kTol));

    // Folga diagonal (aresta contra aresta): sqrt(0.3² + 0.4²) = 0.5.
    r = GJK::compute(MinkowskiSupport(box, at(0, 0, 0), box, at(2.3f, 2.4f, 0)));
    CHECK(!r.intersecting && nearlyEqual(r.distance, Real(0.5), kTol));

    // Folga em vértice: sqrt(3·0.2²) .
    r = GJK::compute(MinkowskiSupport(box, at(0, 0, 0), box, at(2.2f, 2.2f, 2.2f)));
    CHECK(!r.intersecting && nearlyEqual(r.distance, std::sqrt(Real(3)) * Real(0.2), kTol));

    // Sobrepostas e coincidentes.
    CHECK(gjkIntersects(box, at(0, 0, 0), box, at(1.5f, 0, 0)));
    CHECK(gjkIntersects(box, at(0, 0, 0), box, at(0, 0, 0)));
    // Tocando por uma face (contato exato) conta como interseção.
    CHECK(gjkIntersects(box, at(0, 0, 0), box, at(2, 0, 0)));
    // Longe.
    CHECK(!gjkIntersects(box, at(0, 0, 0), box, at(100, 50, -30)));
}

static void testGJKvsSAT() {
    Rng rng(4242);
    std::vector<PolyhedronShape> shapes;
    shapes.push_back(PolyhedronShape::makeBox(Vector3(1, 1, 1)));
    shapes.push_back(PolyhedronShape::makeBox(Vector3(0.4f, 1.2f, 0.7f)));
    shapes.push_back(makeTetra(Real(1.5)));
    shapes.push_back(makePrism(6, Real(0.9), Real(0.6)));
    shapes.push_back(makePrism(20, Real(0.7), Real(1.1)));   // > 16 vértices: usa hill climbing

    int overlaps = 0, separated = 0, skipped = 0;
    int witnessFailures = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        const PolyhedronShape& A = shapes[static_cast<std::size_t>(rng.next() * 5) % 5];
        const PolyhedronShape& B = shapes[static_cast<std::size_t>(rng.next() * 5) % 5];
        const Transform xa = randomTransform(rng, Real(0.5));
        const Transform xb = randomTransform(rng, Real(2.2));

        const SatResult sat = satTest(A, xa, B, xb);
        if (std::abs(sat.minOverlap) < Real(2e-3)) { ++skipped; continue; }   // quase tangentes: ambíguo

        const MinkowskiSupport S(A, xa, B, xb);
        const GJKResult g = GJK::compute(S);
        CHECK(g.intersecting == (sat.minOverlap > 0));

        if (sat.minOverlap > 0) {
            ++overlaps;
            if (!g.intersecting) continue;
            const EPAResult e = EPA::compute(S, g.simplex);
            CHECK(e.valid && e.converged);
            if (!e.valid) continue;
            CHECK(nearlyEqual(e.depth, sat.minOverlap, kTol));
            CHECK(nearlyEqual(e.normal.length(), Real(1), kTol));
            // pointA − pointB = n·d
            CHECK((e.pointA - e.pointB).nearlyEqual(e.normal * e.depth, Real(5e-3)));
            // Normal igual ao eixo SAT (só quando o mínimo é inequívoco).
            if (sat.secondOverlap - sat.minOverlap > Real(0.02)) CHECK(e.normal.dot(sat.axis) > Real(0.99));
        } else {
            ++separated;
            CHECK(!g.intersecting);
            // Distância >= melhor folga entre eixos SAT.
            CHECK(g.distance >= -sat.minOverlap - kTol);
            CHECK(nearlyEqual((g.closestB - g.closestA).length(), g.distance, kTol));
            // Certificado de otimalidade: A inteiro de um lado, B do outro do plano dos pontos mais próximos.
            const Vector3 n = (g.closestB - g.closestA) / g.distance;
            bool ok = true;
            for (const Vector3& p : worldVertices(A, xa)) ok = ok && (p - g.closestA).dot(n) <= kTol;
            for (const Vector3& p : worldVertices(B, xb)) ok = ok && (p - g.closestB).dot(n) >= -kTol;
            if (!ok) ++witnessFailures;
        }
    }
    CHECK(witnessFailures == 0);
    CHECK(overlaps > 300 && separated > 300);   // a amostra cobre os dois regimes
    std::printf("  [info] GJK/EPA vs SAT: %d sobrepostos, %d separados, %d quase-tangentes ignorados\n",
                overlaps, separated, skipped);
}

static void testSpheres() {
    SphereShape s1(Real(1)), s2(Real(0.5));
    BoxShape box(Vector3(1, 1, 1));
    ContactPatch p;

    // Esfera × esfera.
    CHECK(ContactGenerator::generate(s1, Transform(Vector3(0, 0, 0)), s2, Transform(Vector3(1.2f, 0, 0)), p));
    CHECK(p.count == 1 && p.normal.nearlyEqual(Vector3(1, 0, 0), kTol) && nearlyEqual(p.points[0].depth, Real(0.3), kTol));
    CHECK(p.points[0].pointA.nearlyEqual(Vector3(1, 0, 0), kTol));
    CHECK(p.points[0].pointB.nearlyEqual(Vector3(0.7f, 0, 0), kTol));
    CHECK(!ContactGenerator::generate(s1, Transform(Vector3(0, 0, 0)), s2, Transform(Vector3(1.6f, 0, 0)), p));
    // Centros coincidentes: normal arbitrária mas válida.
    CHECK(ContactGenerator::generate(s1, Transform(), s2, Transform(), p));
    CHECK(nearlyEqual(p.normal.length(), Real(1)) && nearlyEqual(p.points[0].depth, Real(1.5), kTol));

    // Esfera (A) apoiada em caixa-chão (B): normal A→B aponta para baixo.
    BoxShape ground(Vector3(5, 0.5f, 5));
    SphereShape ball(Real(0.5));
    CHECK(ContactGenerator::generate(ball, Transform(Vector3(0, 0.95f, 0)), ground, Transform(), p));
    CHECK(p.count == 1 && p.normal.nearlyEqual(Vector3(0, -1, 0), kTol));
    CHECK(nearlyEqual(p.points[0].depth, Real(0.05), kTol));
    CHECK(p.points[0].pointA.nearlyEqual(Vector3(0, 0.45f, 0), kTol));   // base da esfera
    CHECK(p.points[0].pointB.nearlyEqual(Vector3(0, 0.5f, 0), kTol));    // topo do chão
    // Mesma coisa com os papéis trocados (A = chão, B = esfera): normal para cima.
    CHECK(ContactGenerator::generate(ground, Transform(), ball, Transform(Vector3(0, 0.95f, 0)), p));
    CHECK(p.normal.nearlyEqual(Vector3(0, 1, 0), kTol) && nearlyEqual(p.points[0].depth, Real(0.05), kTol));
    // Sem contato (folga).
    CHECK(!ContactGenerator::generate(ball, Transform(Vector3(0, 1.1f, 0)), ground, Transform(), p));

    // Centro da esfera DENTRO da caixa (EPA no núcleo): depth = raio + distância à face mais próxima.
    SphereShape small(Real(0.25));
    CHECK(ContactGenerator::generate(small, Transform(Vector3(0, 0.4f, 0)), ground, Transform(), p));
    CHECK(p.normal.nearlyEqual(Vector3(0, -1, 0), kTol) && nearlyEqual(p.points[0].depth, Real(0.35), kTol));

    // Esfera × quina da caixa (contato com vértice): normal na direção vértice→centro.
    const Vector3 corner(1, 1, 1);
    const Vector3 dirOut = Vector3(1, 1, 1).normalized();
    const Vector3 c = corner + dirOut * Real(0.4);    // centro a 0.4 do vértice, raio 0.5 → depth 0.1
    CHECK(ContactGenerator::generate(s2, Transform(c), box, Transform(), p));
    CHECK(p.normal.nearlyEqual(-dirOut, kTol) && nearlyEqual(p.points[0].depth, Real(0.1), kTol));
}

static void testFaceClipping() {
    BoxShape unit(Vector3(0.5f, 0.5f, 0.5f));
    BoxShape ground(Vector3(10, 0.5f, 10));
    ContactPatch p;

    // Caixa assentada sobre o chão, penetração 0.01: 4 pontos nos cantos da base.
    const Transform xGround;
    Transform xBox(Vector3(0, 0.99f, 0));
    CHECK(ContactGenerator::generate(unit, xBox, ground, xGround, p));
    CHECK(p.count == 4);
    CHECK(p.normal.nearlyEqual(Vector3(0, -1, 0), kTol));
    for (int i = 0; i < p.count; ++i) {
        CHECK(nearlyEqual(p.points[i].depth, Real(0.01), kTol));
        CHECK(std::abs(std::abs(p.points[i].pointA.x) - Real(0.5)) < kTol && std::abs(std::abs(p.points[i].pointA.z) - Real(0.5)) < kTol);
        // pointA − pointB = n·depth
        CHECK((p.points[i].pointA - p.points[i].pointB).nearlyEqual(p.normal * p.points[i].depth, kTol));
    }
    // Mesmos 4 pontos com os papéis trocados (A = chão).
    CHECK(ContactGenerator::generate(ground, xGround, unit, xBox, p));
    CHECK(p.count == 4 && p.normal.nearlyEqual(Vector3(0, 1, 0), kTol));

    // Girada 45° em torno de y: continua 4 pontos (os cantos da base).
    xBox.rotation = Quaternion::fromAxisAngle(Vector3::unitY(), kPi / 4);
    CHECK(ContactGenerator::generate(unit, xBox, ground, xGround, p));
    CHECK(p.count == 4);

    // Duas caixas do mesmo tamanho, uma girada 45°: o recorte é um octógono (8 pontos) → reduz a 4,
    // maximizando a área do contato.
    BoxShape big(Vector3(0.5f, 0.5f, 0.5f));
    Transform xTop(Vector3(0, 0.99f, 0), Quaternion::fromAxisAngle(Vector3::unitY(), kPi / 4));
    CHECK(ContactGenerator::generate(big, xTop, big, Transform(), p));
    CHECK(p.count == 4);
    Real area = 0;
    {
        const Vector3 a = p.points[0].pointA, b = p.points[1].pointA, c = p.points[2].pointA, d = p.points[3].pointA;
        const Real a1 = (b - a).cross(c - a).length() * Real(0.5) + (c - a).cross(d - a).length() * Real(0.5);
        const Real a2 = (b - a).cross(d - a).length() * Real(0.5) + (c - b).cross(d - b).length() * Real(0.5);
        area = std::max(a1, a2);
    }
    CHECK(area > Real(0.3));   // octógono inscrito tem 0.828·0.25... os 4 pontos escolhidos cobrem boa parte

    // Caixa inclinada 15° sobre o chão: só a aresta mais baixa penetra → 2 pontos.
    Transform xTilt(Vector3(0, 0.5f * std::cos(degToRad(15)) + 0.5f * std::sin(degToRad(15)) + 0.5f - 0.01f, 0),
                    Quaternion::fromAxisAngle(Vector3::unitZ(), degToRad(15)));
    CHECK(ContactGenerator::generate(unit, xTilt, ground, xGround, p));
    CHECK(p.count == 2);
    CHECK(nearlyEqual(p.points[0].depth, Real(0.01), Real(4e-3)) && nearlyEqual(p.points[1].depth, Real(0.01), Real(4e-3)));
    CHECK(std::abs(p.points[0].pointA.z - p.points[1].pointA.z) > Real(0.9));   // as pontas da aresta

    // Aresta contra aresta (duas caixas em "X"): sem face alinhada → 1 ponto do EPA.
    Transform xe1(Vector3(0, 0, 0), Quaternion::fromAxisAngle(Vector3(1, 0, 0), kPi / 4) * Quaternion::fromAxisAngle(Vector3(0, 0, 1), kPi / 4));
    Transform xe2(Vector3(0, 0.95f, 0), Quaternion::fromAxisAngle(Vector3(0, 1, 0), kPi / 2) * Quaternion::fromAxisAngle(Vector3(1, 0, 0), kPi / 4) * Quaternion::fromAxisAngle(Vector3(0, 0, 1), kPi / 4));
    const bool hit = ContactGenerator::generate(unit, xe1, unit, xe2, p);
    CHECK(!hit || p.count >= 1);

    // Sem clipping: sempre 1 ponto.
    ContactGeneratorSettings noClip;
    noClip.useFaceClipping = false;
    CHECK(ContactGenerator::generate(unit, Transform(Vector3(0, 0.99f, 0)), ground, xGround, p, noClip) && p.count == 1);

    // Poliedro (prisma de 8 lados) sobre o chão: face inferior com 8 vértices, recorte → 4 pontos.
    PolyhedronShape prism = makePrism(8, Real(0.5), Real(0.5));
    // prisma tem eixo z; gira para o eixo y e apoia a base no chão
    Transform xPr(Vector3(0, 0.99f, 0), Quaternion::fromAxisAngle(Vector3::unitX(), kHalfPi));
    CHECK(ContactGenerator::generate(prism, xPr, ground, xGround, p));
    CHECK(p.count == 4 && p.normal.nearlyEqual(Vector3(0, -1, 0), kTol));
    for (int i = 0; i < p.count; ++i) CHECK(nearlyEqual(p.points[i].depth, Real(0.01), kTol));

    // Entradas inválidas / sem colisão.
    CHECK(!ContactGenerator::generate(unit, Transform(Vector3(0, 3, 0)), ground, xGround, p) && p.count == 0);
    Transform bad(Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0));
    CHECK(!ContactGenerator::generate(unit, bad, ground, xGround, p) && p.count == 0);
}

static void testDegenerateCases() {
    BoxShape box(Vector3(1, 1, 1));
    ContactPatch p;

    // Idênticas e coincidentes: profundidade = 2 (qualquer eixo).
    const Transform id;
    CHECK(ContactGenerator::generate(box, id, box, id, p));
    CHECK(p.count >= 1 && nearlyEqual(p.points[0].depth, Real(2), kTol) && p.normal.isFinite());

    // Penetração profunda: centro de A dentro de B.
    BoxShape small(Vector3(0.1f, 0.1f, 0.1f));
    CHECK(ContactGenerator::generate(small, Transform(Vector3(0.9f, 0, 0)), box, id, p));
    // A ocupa x ∈ [0.8, 1.0] e B ocupa [-1, 1]: a menor translação é 0.2 empurrando B para -x.
    CHECK(nearlyEqual(p.points[0].depth, Real(0.2), kTol));
    CHECK(p.normal.nearlyEqual(Vector3(-1, 0, 0), kTol));

    // Coordenadas grandes.
    CHECK(ContactGenerator::generate(box, Transform(Vector3(1000, 2000, -500)), box, Transform(Vector3(1001.5f, 2000, -500)), p));
    CHECK(nearlyEqual(p.points[0].depth, Real(0.5), Real(0.02)));

    // Formas minúsculas e enormes.
    BoxShape tiny(Vector3(1e-3f, 1e-3f, 1e-3f)), huge(Vector3(1e3f, 1e3f, 1e3f));
    CHECK(ContactGenerator::generate(tiny, Transform(Vector3(0, 1000.0005f, 0)), huge, id, p));
    CHECK(p.points[0].depth > 0 && p.normal.isFinite());

    // Aleatório massivo: nunca NaN, nunca trava.
    Rng rng(99);
    PolyhedronShape tet = makeTetra(Real(1));
    PolyhedronShape pr = makePrism(7, Real(0.8), Real(0.4));
    SphereShape sph(Real(0.6));
    const Shape* all[4] = {&box, &tet, &pr, &sph};
    int bad = 0, nonConverged = 0;
    for (int i = 0; i < 20000; ++i) {
        const Shape& a = *all[static_cast<std::size_t>(rng.next() * 4) % 4];
        const Shape& b = *all[static_cast<std::size_t>(rng.next() * 4) % 4];
        ContactPatch q;
        const bool hit = ContactGenerator::generate(a, randomTransform(rng, Real(0.3)), b, randomTransform(rng, Real(1.8)), q);
        if (hit) {
            bool ok = q.count >= 1 && q.count <= 4 && q.normal.isFinite() && std::abs(q.normal.length() - Real(1)) < Real(1e-3);
            for (int k = 0; k < q.count; ++k) ok = ok && q.points[k].pointA.isFinite() && q.points[k].pointB.isFinite() && std::isfinite(q.points[k].depth);
            if (!ok) ++bad;
        } else if (q.count != 0) {
            ++bad;
        }
    }
    CHECK(bad == 0);
    (void)nonConverged;

    // NaN numa forma já em GJK não trava.
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    GJKResult g = GJK::compute(MinkowskiSupport(box, Transform(Vector3(nan, 0, 0)), box, id));
    CHECK(!g.intersecting);
}

static void testManifold() {
    // Corpos "A" e "B" com transformações identidade/transladadas.
    Transform xa, xb;
    ContactManifold m(1, 2);
    m.normal = Vector3(0, -1, 0);

    auto pt = [](Real x, Real z, Real depth) {
        PatchPoint p;
        p.pointA = Vector3(x, 0.5f - depth * Real(0.5), z);
        p.pointB = Vector3(x, 0.5f + depth * Real(0.5), z);
        // pointA − pointB = (0,−depth,0) = normal·depth com normal (0,−1,0)
        p.depth = depth;
        return p;
    };

    // Adiciona 4 cantos de um quadrado.
    const int i0 = m.addPoint(xa, xb, pt(-1, -1, 0.01f));
    const int i1 = m.addPoint(xa, xb, pt(1, -1, 0.01f));
    const int i2 = m.addPoint(xa, xb, pt(1, 1, 0.01f));
    const int i3 = m.addPoint(xa, xb, pt(-1, 1, 0.01f));
    CHECK(m.count == 4 && i0 == 0 && i1 == 1 && i2 == 2 && i3 == 3);
    const std::uint32_t id1 = m.points[1].id;
    CHECK(m.points[0].id != m.points[1].id && m.points[2].id != m.points[3].id);

    // Warm start: impulsos acumulados são herdados por um contato que casa.
    m.points[1].normalImpulse = 7;
    m.points[1].tangentImpulse = Vector3(0.5f, 0, 0.25f);
    const int j = m.addPoint(xa, xb, pt(1.005f, -1.003f, 0.012f));
    CHECK(j == 1 && m.count == 4);
    CHECK(m.points[1].id == id1 && nearlyEqual(m.points[1].normalImpulse, Real(7)));
    CHECK(m.points[1].tangentImpulse.nearlyEqual(Vector3(0.5f, 0, 0.25f)));
    CHECK(nearlyEqual(m.points[1].depth, Real(0.012), Real(1e-6)));   // literal 0.012f: tolerância > erro de float
    CHECK(nearlyEqual(m.getTotalNormalImpulse(), Real(7)));

    // Quinto ponto no CENTRO não melhora a área: é descartado.
    CHECK(m.addPoint(xa, xb, pt(0, 0, 0.01f)) == -1 && m.count == 4);
    // Quinto ponto longe, MAIS profundo que os demais: entra (o mais profundo nunca é descartado).
    const int k = m.addPoint(xa, xb, pt(3, 3, 0.05f));
    CHECK(k >= 0 && m.count == 4);
    bool hasDeepest = false;
    for (int i = 0; i < m.count; ++i) if (nearlyEqual(m.points[i].depth, Real(0.05), Real(1e-6))) hasDeepest = true;
    CHECK(hasDeepest);

    // refresh: mover B ao longo da normal (-y) aprofunda; afastar além do limiar remove.
    ContactManifold r(1, 2);
    r.normal = Vector3(0, -1, 0);
    r.addPoint(xa, xb, pt(0, 0, 0.01f));
    Transform xbUp = xb;
    // normal A→B = (0,−1,0): B está abaixo de A. Descer B (−y) afasta, reduz a penetração.
    xbUp.position = Vector3(0, -0.005f, 0);
    r.refresh(xa, xbUp);
    CHECK(r.count == 1 && nearlyEqual(r.points[0].depth, Real(0.01) - Real(0.005), Real(1e-4)));
    Transform xbFar = xb;
    xbFar.position = Vector3(0, -0.1f, 0);   // B desce 10 cm: depth ≈ −0.09 < −limiar (0.02) → separou
    r.refresh(xa, xbFar);
    CHECK(r.count == 0);

    // refresh: deslize tangencial remove.
    ContactManifold s(1, 2);
    s.normal = Vector3(0, -1, 0);
    s.addPoint(xa, xb, pt(0, 0, 0.01f));
    Transform xbSlide = xb;
    xbSlide.position = Vector3(0.05f, 0, 0);
    s.refresh(xa, xbSlide);
    CHECK(s.count == 0);
    // deslize pequeno permanece.
    s.addPoint(xa, xb, pt(0, 0, 0.01f));
    xbSlide.position = Vector3(0.01f, 0, 0);
    s.refresh(xa, xbSlide);
    CHECK(s.count == 1);

    // update(): patch completo substitui, mas herda impulsos por proximidade e preserva ids.
    ContactManifold u(1, 2);
    ContactPatch patch;
    patch.normal = Vector3(0, -1, 0);
    const Real xs[4] = {-1, 1, 1, -1}, zs[4] = {-1, -1, 1, 1};
    for (int i = 0; i < 4; ++i) patch.points[i] = pt(xs[i], zs[i], 0.01f);
    patch.count = 4;
    u.update(xa, xb, patch);
    CHECK(u.count == 4);
    std::uint32_t ids[4];
    for (int i = 0; i < 4; ++i) { ids[i] = u.points[i].id; u.points[i].normalImpulse = Real(i + 1); }
    // Quadro seguinte: cantos ligeiramente deslocados, em OUTRA ordem.
    ContactPatch patch2 = patch;
    patch2.points[0] = pt(1.004f, 0.998f, 0.011f);   // ↔ antigo 2
    patch2.points[1] = pt(-0.998f, -1.004f, 0.011f); // ↔ antigo 0
    patch2.points[2] = pt(-1.002f, 1.001f, 0.011f);  // ↔ antigo 3
    patch2.points[3] = pt(1.001f, -1.002f, 0.011f);  // ↔ antigo 1
    u.update(xa, xb, patch2);
    CHECK(u.count == 4);
    CHECK(u.points[0].id == ids[2] && nearlyEqual(u.points[0].normalImpulse, Real(3)));
    CHECK(u.points[1].id == ids[0] && nearlyEqual(u.points[1].normalImpulse, Real(1)));
    CHECK(u.points[2].id == ids[3] && nearlyEqual(u.points[2].normalImpulse, Real(4)));
    CHECK(u.points[3].id == ids[1] && nearlyEqual(u.points[3].normalImpulse, Real(2)));

    // Patch vazio limpa.
    ContactPatch empty;
    u.update(xa, xb, empty);
    CHECK(u.count == 0);

    // Persistência com patches de 1 ponto (esfera rolando): os pontos antigos permanecem e acumulam até 4.
    ContactManifold acc(1, 2);
    ContactPatch single;
    single.normal = Vector3(0, -1, 0);
    single.count = 1;
    for (int i = 0; i < 6; ++i) {
        single.points[0] = pt(Real(i) * Real(0.004), 0, 0.01f);   // dentro do raio de casamento: sempre o mesmo ponto
        acc.update(xa, xb, single);
    }
    CHECK(acc.count == 1);
    for (int i = 0; i < 6; ++i) {
        single.points[0] = pt(Real(i) * Real(0.5), Real(i) * Real(0.3), 0.01f);   // pontos distintos
        acc.update(xa, xb, single);
    }
    CHECK(acc.count <= 4 && acc.count >= 3);
}

static void testCollideWarmStart() {
    // Fluxo completo: caixa em repouso sobre o chão por vários quadros; impulsos persistem.
    BoxShape box(Vector3(0.5f, 0.5f, 0.5f)), ground(Vector3(10, 0.5f, 10));
    ContactManifold m(1, 2);
    Transform xg, xb(Vector3(0, 0.99f, 0));

    CHECK(ContactGenerator::collide(box, xb, ground, xg, m));
    CHECK(m.count == 4);
    std::uint32_t ids[4];
    for (int i = 0; i < 4; ++i) { ids[i] = m.points[i].id; m.points[i].normalImpulse = Real(10 + i); }

    for (int frame = 0; frame < 30; ++frame) {
        xb.position.y = Real(0.99) + Real(0.0005) * Real(frame % 3);   // oscila ~mm, como num solver
        xb.position.x = Real(0.0001) * Real(frame);
        CHECK(ContactGenerator::collide(box, xb, ground, xg, m));
    }
    CHECK(m.count == 4);
    int preserved = 0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (m.points[i].id == ids[j] && m.points[i].normalImpulse > Real(9)) ++preserved;
    CHECK(preserved == 4);

    // Ao separar, o manifold é limpo.
    xb.position.y = Real(5);
    CHECK(!ContactGenerator::collide(box, xb, ground, xg, m));
    CHECK(m.count == 0);
}

// =============================================================================
// Formas GRANDES com micro-penetração (regressão).
//
// O GJK em ponto flutuante perde precisão quando uma das formas é enorme: um chão de ±100 m dá
// diferença de Minkowski com coordenadas de dezenas de metros, e 1 ulp (~1e-5 m em float) já não
// separa uma folga de décimos de milímetro de uma penetração. Medido no motor da Fase 8 (float):
// 0,17 % das caixas sobre um chão de ±100 m e 0,7 % sobre um de ±500 m saíam com normal arbitrária
// e profundidade de 160 a 875 m — um contato assim derruba uma pilha inteira num único passo.
// Duas defesas: GJK::kGJKNoiseFactor (ruído relativo) e refineContactAxis (valida o eixo).
//
// Este teste também protege contra a falha INVERSA, que apareceu na primeira versão do refino:
// ele "consertava" contatos corretos (a inclinação de ~1e-5 rad da normal, vezes 50 m de extensão,
// passava da tolerância) e trocava o ponto de contato por um canto do chão — 27 % dos contatos
// esfera × chão em float, 84 % com chão de ±500 m. Por isso os pontos também são verificados.
// =============================================================================
static void testLargeGroundRobustness() {
    struct Cfg { Real half, penMax, tiltMax; };
    const Cfg cfgs[] = {{Real(50), Real(0.02), Real(0.01)},
                        {Real(50), Real(0.0005), Real(0.0002)},
                        {Real(100), Real(0.002), Real(0.0005)},
                        {Real(500), Real(0.002), Real(0.0005)}};
    int badBox = 0, badSphere = 0, noContact = 0, total = 0;
    Real worstDepth = 0;
    for (const Cfg& c : cfgs) {
        BoxShape ground(Vector3(c.half, 1, c.half));
        BoxShape box(Vector3(0.5f, 0.5f, 0.5f));
        SphereShape sphere(Real(0.5));
        const Transform xfG(Vector3(0, -1, 0));   // face superior em y = 0
        Rng rng(777);
        for (int i = 0; i < 4000; ++i, ++total) {
            const Real cx = rng.range(-c.half * Real(0.8), c.half * Real(0.8));
            const Real cz = rng.range(-c.half * Real(0.8), c.half * Real(0.8));
            const Real pen = rng.range(Real(1e-5), c.penMax);

            // Caixa girada em torno de Y e levemente inclinada.
            Quaternion q = Quaternion::fromAxisAngle(Vector3::unitY(), rng.range(0, Real(6.28)));
            q = q * Quaternion::fromAxisAngle(Vector3::unitX(), rng.range(0, c.tiltMax));
            ContactPatch p;
            if (!ContactGenerator::generate(ground, xfG, box, Transform(Vector3(cx, Real(0.5) - pen, cz), q), p)) {
                ++noContact;
            } else {
                bool ok = p.normal.y > Real(0.99);
                for (int k = 0; k < p.count; ++k) {
                    ok = ok && p.points[k].depth > Real(-0.01) && p.points[k].depth < Real(0.05);
                    // O ponto tem que estar sob a caixa, não num canto do chão.
                    ok = ok && std::abs(p.points[k].pointB.x - cx) < Real(1) && std::abs(p.points[k].pointB.z - cz) < Real(1);
                    worstDepth = std::max(worstDepth, p.points[k].depth);
                }
                badBox += ok ? 0 : 1;
            }

            const Real sp = rng.range(Real(1e-5), Real(0.05));
            if (!ContactGenerator::generate(ground, xfG, sphere, Transform(Vector3(cx, Real(0.5) - sp, cz)), p)) {
                ++noContact;
            } else {
                const bool ok = p.normal.y > Real(0.99) && p.count >= 1 && std::abs(p.points[0].depth - sp) < Real(0.002) &&
                                std::abs(p.points[0].pointB.x - cx) < Real(0.6) && std::abs(p.points[0].pointB.z - cz) < Real(0.6);
                badSphere += ok ? 0 : 1;
            }
        }
    }
    std::printf("  [info] formas grandes: %d pares x2, caixas ruins=%d, esferas ruins=%d, sem contato=%d, pior profundidade=%.3f m\n",
                total, badBox, badSphere, noContact, double(worstDepth));
    CHECK(badBox == 0);
    CHECK(badSphere == 0);
    CHECK(noContact == 0);
    CHECK(worstDepth < Real(0.05));
}

static void testPerformanceInfo() {
    BoxShape a(Vector3(0.5f, 0.5f, 0.5f)), b(Vector3(0.5f, 0.5f, 0.5f));
    Rng rng(5);
    std::vector<Transform> xa, xb;
    for (int i = 0; i < 1024; ++i) { xa.push_back(randomTransform(rng, Real(0.2))); xb.push_back(randomTransform(rng, Real(1.2))); }
    const int n = 200000;
    int hits = 0;
    const auto t0 = std::chrono::steady_clock::now();
    ContactPatch p;
    for (int i = 0; i < n; ++i) hits += ContactGenerator::generate(a, xa[static_cast<std::size_t>(i) & 1023], b, xb[static_cast<std::size_t>(i * 7) & 1023], p) ? 1 : 0;
    const auto t1 = std::chrono::steady_clock::now();
    std::printf("  [info] caixa×caixa: %.2f us por par (%d%% colidindo)\n",
                std::chrono::duration<double, std::micro>(t1 - t0).count() / n, hits * 100 / n);
    CHECK(hits > 0);
}

int main() {
    testSimplexClosestPoint();
    testGJKBoxesAnalytic();
    testGJKvsSAT();
    testSpheres();
    testFaceClipping();
    testDegenerateCases();
    testManifold();
    testCollideWarmStart();
    testLargeGroundRobustness();
    testPerformanceInfo();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
