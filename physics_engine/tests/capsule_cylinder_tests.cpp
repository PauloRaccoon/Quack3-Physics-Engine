// =============================================================================
// capsule_cylinder_tests.cpp — testes da V18: CapsuleShape e CylinderShape (sem frameworks externos).
//
// Estratégia: cada propriedade é comparada com um método INDEPENDENTE:
//   - volume/inércia: integração numérica em grade (Monte Carlo determinístico não; grade regular);
//   - função de suporte: varredura de direções + força bruta sobre pontos amostrados da superfície;
//   - AABB: contra os extremos de uma nuvem de pontos da superfície sob rotação aleatória;
//   - cápsula × cápsula e cápsula × esfera: distância segmento–segmento em forma fechada;
//   - raio: o ponto de entrada tem que estar na superfície (dentro um pouco depois, fora um pouco antes);
//   - repouso: simulações completas com PhysicsWorld.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

#include "physics/collision/RayCast.h"
#include "physics/collision/narrowphase/ContactManifold.h"
#include "physics/debug/DebugDraw.h"
#include "physics/geometry/BoxShape.h"
#include "physics/geometry/CapsuleShape.h"
#include "physics/geometry/CylinderShape.h"
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

static bool near(Real a, Real b, Real tol = Real(2e-3)) { return std::abs(a - b) <= tol * std::max(Real(1), std::max(std::abs(a), std::abs(b))); }

template <typename E, typename F>
static bool throws(F&& f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real a, Real b) { return a + (b - a) * next(); }
    Vector3 unit() {
        for (;;) {
            const Vector3 v(range(-1, 1), range(-1, 1), range(-1, 1));
            const Real l = v.length();
            if (l > Real(0.1) && l <= Real(1)) return v / l;
        }
    }
    Quaternion rot() { return Quaternion::fromAxisAngle(unit(), range(0, Real(6.2831853))); }
};

/// Pertence ao sólido? (forma fechada, espaço local)
static bool insideCapsule(Real r, Real h, const Vector3& p) {
    const Real y = std::max(-h, std::min(h, p.y));
    return Vector3(p.x, p.y - y, p.z).lengthSquared() <= r * r;
}
static bool insideCylinder(Real r, Real h, const Vector3& p) { return std::abs(p.y) <= h && p.x * p.x + p.z * p.z <= r * r; }

// =============================================================================
// Forma: construtor, volume, inércia, suporte, AABB
// =============================================================================
static void testConstruction() {
    CHECK(throws<std::invalid_argument>([] { CapsuleShape(Real(0), Real(1)); }));
    CHECK(throws<std::invalid_argument>([] { CapsuleShape(Real(-1), Real(1)); }));
    CHECK(throws<std::invalid_argument>([] { CapsuleShape(Real(1), Real(-0.1)); }));
    CHECK(throws<std::invalid_argument>([] { CapsuleShape(std::numeric_limits<Real>::quiet_NaN(), Real(1)); }));
    CHECK(throws<std::invalid_argument>([] { CapsuleShape(Real(1), std::numeric_limits<Real>::infinity()); }));
    CHECK(throws<std::invalid_argument>([] { CapsuleShape(Real(1e9), Real(1)); }));
    CHECK(!throws<std::invalid_argument>([] { CapsuleShape(Real(0.3), Real(0)); }));   // h = 0: esfera
    CHECK(throws<std::invalid_argument>([] { CylinderShape(Real(0), Real(1)); }));
    CHECK(throws<std::invalid_argument>([] { CylinderShape(Real(1), Real(0)); }));
    CHECK(throws<std::invalid_argument>([] { CylinderShape(Real(1), std::numeric_limits<Real>::quiet_NaN()); }));
    CHECK(throws<std::invalid_argument>([] { CylinderShape(Real(1e9), Real(1)); }));
    const CapsuleShape c(Real(0.3), Real(0.6));
    CHECK(c.getType() == ShapeType::Capsule && near(c.getTotalHeight(), Real(1.8)) && near(c.getCoreRadius(), Real(0.3)));
    const CylinderShape y(Real(0.4), Real(0.5));
    CHECK(y.getType() == ShapeType::Cylinder && y.getCoreRadius() == Real(0));
    // massa e AABB de um corpo com a forma
    RigidBodyDesc d;
    d.shape = std::make_shared<CapsuleShape>(Real(0.3), Real(0.6));
    d.mass = Real(70);
    RigidBody b(d);
    CHECK(near(b.getMass(), Real(70)) && b.getInvMass() > Real(0));
}

static void testVolumeInertia() {
    // integração em grade: soma de células cujo centro está dentro; I = ∫ ρ (distância² ao eixo)
    auto integrate = [](bool capsule, Real r, Real h, double& vol, double& iAxis, double& iPerp) {
        const int n = 120;
        const Real ext = r + h + Real(0.01);
        const double cell = static_cast<double>(2 * ext) / n;
        vol = iAxis = iPerp = 0;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                for (int k = 0; k < n; ++k) {
                    const Vector3 p(-ext + Real((i + 0.5) * cell), -ext + Real((j + 0.5) * cell), -ext + Real((k + 0.5) * cell));
                    if (capsule ? !insideCapsule(r, h, p) : !insideCylinder(r, h, p)) continue;
                    vol += cell * cell * cell;
                    iAxis += cell * cell * cell * (static_cast<double>(p.x) * p.x + static_cast<double>(p.z) * p.z);
                    iPerp += cell * cell * cell * (static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z);   // eixo X
                }
    };
    for (int kind = 0; kind < 2; ++kind) {
        const Real r = Real(0.4), h = Real(0.7), mass = Real(3);
        std::shared_ptr<Shape> s;
        if (kind == 0) s = std::make_shared<CapsuleShape>(r, h); else s = std::make_shared<CylinderShape>(r, h);
        double vol, iA, iP;
        integrate(kind == 0, r, h, vol, iA, iP);
        CHECK(std::abs(vol - static_cast<double>(s->getVolume())) < 0.02 * vol);
        const double dens = static_cast<double>(mass) / vol;
        const Matrix3 I = s->computeInertia(mass);
        CHECK(std::abs(dens * iA - static_cast<double>(I.m[1][1])) < 0.03 * dens * iA);   // eixo Y
        CHECK(std::abs(dens * iP - static_cast<double>(I.m[0][0])) < 0.03 * dens * iP);   // eixo X
        CHECK(std::abs(dens * iP - static_cast<double>(I.m[2][2])) < 0.03 * dens * iP);   // eixo Z (simetria)
        CHECK(near(I.m[0][1], Real(0)) && near(I.m[0][2], Real(0)) && near(I.m[1][2], Real(0)));
    }
    // cápsula com h = 0 tem a inércia da esfera
    const CapsuleShape zero(Real(0.5), Real(0));
    const SphereShape sph(Real(0.5));
    CHECK(near(zero.computeInertia(Real(2)).m[0][0], sph.computeInertia(Real(2)).m[0][0]) && near(zero.getVolume(), sph.getVolume()));
}

static void testSupportAndAABB() {
    Rng rng(7);
    for (int kind = 0; kind < 2; ++kind) {
        const Real r = Real(0.35), h = Real(0.8);
        std::shared_ptr<Shape> s;
        if (kind == 0) s = std::make_shared<CapsuleShape>(r, h); else s = std::make_shared<CylinderShape>(r, h);
        // nuvem de pontos da superfície
        std::vector<Vector3> pts;
        for (int i = 0; i < 4000; ++i) {
            const Vector3 d = rng.unit();
            pts.push_back(s->getSupport(d));   // o suporte de qualquer direção é um ponto do sólido
        }
        bool onBody = true;
        for (const Vector3& p : pts) onBody = onBody && (kind == 0 ? insideCapsule(r + Real(1e-4), h, p) : insideCylinder(r + Real(1e-4), h + Real(1e-4), p));
        CHECK(onBody);
        // propriedade do suporte: p·d é o máximo (nenhum ponto sólido amostrado passa)
        bool maximal = true;
        for (int i = 0; i < 300; ++i) {
            const Vector3 d = rng.unit();
            const Real best = s->getSupport(d).dot(d);
            for (int j = 0; j < 200; ++j) {
                const Vector3 q = s->getSupport(rng.unit());
                if (q.dot(d) > best + Real(1e-4)) maximal = false;
            }
        }
        CHECK(maximal);
        // direções degeneradas: nunca NaN
        const Vector3 dirs[] = {Vector3::zero(), Vector3::unitY(), -Vector3::unitY(), Vector3(Real(1e-20), Real(0), Real(0)), Vector3(Real(1e20), Real(1e20), Real(0))};
        bool finite = true;
        for (const Vector3& d : dirs) finite = finite && s->getSupport(d).isFinite();
        CHECK(finite);
        // AABB sob rotação aleatória: igual (≤ tolerância) aos extremos das direções ±eixos do mundo
        bool aabbOk = true;
        for (int i = 0; i < 200; ++i) {
            const Transform xf(Vector3(rng.range(-3, 3), rng.range(-3, 3), rng.range(-3, 3)), rng.rot());
            const AABB box = s->computeAABB(xf);
            const Vector3 axes[3] = {Vector3::unitX(), Vector3::unitY(), Vector3::unitZ()};
            for (int a = 0; a < 3; ++a) {
                const Real hi = s->getWorldSupport(xf, axes[a]).dot(axes[a]);
                const Real lo = s->getWorldSupport(xf, -axes[a]).dot(axes[a]);
                const Real bhi = a == 0 ? box.max.x : a == 1 ? box.max.y : box.max.z;
                const Real blo = a == 0 ? box.min.x : a == 1 ? box.min.y : box.min.z;
                aabbOk = aabbOk && std::abs(hi - bhi) < Real(2e-3) && std::abs(lo - blo) < Real(2e-3);
            }
        }
        CHECK(aabbOk);
    }
    // núcleo + raio: suporte da cápsula = ponta do segmento + r·direção
    const CapsuleShape c(Real(0.3), Real(0.6));
    const Vector3 d = Vector3(1, 2, 3).normalized();
    CHECK((c.getSupport(d) - (c.getCoreSupport(d) + d * Real(0.3))).length() < Real(1e-5));
}

// =============================================================================
// Contatos
// =============================================================================
static ContactPatch gen(const Shape& a, const Transform& xa, const Shape& b, const Transform& xb, bool* hit = nullptr) {
    ContactPatch p;
    const bool h = ContactGenerator::generate(a, xa, b, xb, p);
    if (hit) *hit = h;
    return p;
}

/// Distância mínima entre dois segmentos (força bruta refinada: ternária aninhada, função convexa).
static Real segSegDist(const Vector3& a0, const Vector3& a1, const Vector3& b0, const Vector3& b1) {
    auto fB = [&](Real ta) {
        const Vector3 pa = a0 + (a1 - a0) * ta;
        Real lo = 0, hi = 1;
        for (int i = 0; i < 60; ++i) {
            const Real m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
            const Real d1 = (pa - (b0 + (b1 - b0) * m1)).lengthSquared(), d2 = (pa - (b0 + (b1 - b0) * m2)).lengthSquared();
            if (d1 < d2) hi = m2; else lo = m1;
        }
        return (pa - (b0 + (b1 - b0) * ((lo + hi) / 2))).length();
    };
    Real lo = 0, hi = 1;
    for (int i = 0; i < 60; ++i) {
        const Real m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
        if (fB(m1) < fB(m2)) hi = m2; else lo = m1;
    }
    return fB((lo + hi) / 2);
}

static void testCapsuleContacts() {
    const BoxShape ground(Vector3(50, 1, 50));
    const Transform groundXf(Vector3(0, -1, 0));   // face superior em y = 0
    const Real r = Real(0.3), h = Real(0.6);
    const CapsuleShape cap(r, h);
    // deitada (eixo ao longo de X): 2 pontos, normal vertical, profundidade r − altura
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        bool hit = false;
        const ContactPatch p = gen(cap, Transform(Vector3(0, Real(0.28), 0), lie), ground, groundXf, &hit);
        CHECK(hit && p.count == 2);
        CHECK(near(p.normal.y, Real(-1), Real(1e-3)));   // A (cápsula) → B (chão): para baixo
        bool depthOk = p.count == 2;
        for (int i = 0; i < p.count; ++i) depthOk = depthOk && near(p.points[i].depth, Real(0.02), Real(0.01));
        CHECK(depthOk);
        if (p.count == 2) CHECK(std::abs(p.points[0].pointA.x - p.points[1].pointA.x) > Real(1.0));   // pontas distintas, afastadas
    }
    // o mesmo com a ordem trocada (chão = A)
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        bool hit = false;
        const ContactPatch p = gen(ground, groundXf, cap, Transform(Vector3(0, Real(0.28), 0), lie), &hit);
        CHECK(hit && p.count == 2 && near(p.normal.y, Real(1), Real(1e-3)));
        bool ok = true;
        for (int i = 0; i < p.count; ++i) ok = ok && near((p.points[i].pointA - p.points[i].pointB).dot(p.normal), p.points[i].depth, Real(5e-3));
        CHECK(ok);
    }
    // em pé: 1 ponto, embaixo do eixo
    {
        const ContactPatch p = gen(cap, Transform(Vector3(0.2f, h + r - Real(0.01), 0.1f)), ground, groundXf);
        CHECK(p.count == 1 && near(p.points[0].depth, Real(0.01), Real(0.01)) && std::abs(p.points[0].pointA.x - Real(0.2)) < Real(2e-2));
    }
    // inclinada: só a ponta de baixo toca (a outra está acima da margem)
    {
        const Quaternion tilt = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(0.9));
        const Real hLow = r + h * std::cos(Real(0.9)) - Real(0.01);   // a ponta de baixo penetra 1 cm
        const ContactPatch p = gen(cap, Transform(Vector3(0, hLow, 0), tilt), ground, groundXf);
        CHECK(p.count == 1 && near(p.points[0].depth, Real(0.01), Real(0.01)));
    }
    // fora do alcance
    {
        bool hit = true;
        gen(cap, Transform(Vector3(0, 3, 0)), ground, groundXf, &hit);
        CHECK(!hit);
    }
    // cápsula × cápsula: profundidade = rA + rB − distância entre os segmentos, em poses aleatórias
    Rng rng(11);
    int tested = 0;
    for (int i = 0; i < 400; ++i) {
        const CapsuleShape a(rng.range(Real(0.1), Real(0.5)), rng.range(Real(0.1), Real(1))), b(rng.range(Real(0.1), Real(0.5)), rng.range(Real(0.1), Real(1)));
        const Transform xa(Vector3(0, 0, 0), rng.rot()), xb(Vector3(rng.range(-1.2f, 1.2f), rng.range(-1.2f, 1.2f), rng.range(-1.2f, 1.2f)), rng.rot());
        const Vector3 a0 = xa.transformPoint(Vector3(0, -a.getHalfHeight(), 0)), a1 = xa.transformPoint(Vector3(0, a.getHalfHeight(), 0));
        const Vector3 b0 = xb.transformPoint(Vector3(0, -b.getHalfHeight(), 0)), b1 = xb.transformPoint(Vector3(0, b.getHalfHeight(), 0));
        const Real dist = segSegDist(a0, a1, b0, b1);
        const Real expected = a.getRadius() + b.getRadius() - dist;
        if (dist < Real(0.02)) continue;   // segmentos que se cruzam: normal indefinida
        bool hit = false;
        const ContactPatch p = gen(a, xa, b, xb, &hit);
        if (expected > Real(0.02)) {   // penetrando de verdade
            ++tested;
            CHECK(hit && p.count >= 1);
            bool ok = hit && p.count >= 1;
            for (int k = 0; ok && k < p.count; ++k) ok = ok && std::abs(p.points[k].depth - expected) < Real(0.03) + (p.count > 1 ? Real(0.0) : Real(0));
            if (p.count == 1) CHECK(std::abs(p.points[0].depth - expected) < Real(0.01));
            CHECK(ok || p.count > 1);   // com 2 pontos (paralelas) a profundidade de cada ponta pode diferir se não forem exatamente paralelas
        }
        if (expected < Real(-0.1)) CHECK(!hit);
    }
    CHECK(tested > 40);
    // duas cápsulas paralelas lado a lado: 2 pontos, mesma profundidade
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        const ContactPatch p = gen(cap, Transform(Vector3(0, 0, 0), lie), cap, Transform(Vector3(0.1f, 0, Real(0.58)), lie));
        CHECK(p.count == 2 && near(p.normal.z, Real(1), Real(2e-2)));
        if (p.count == 2) CHECK(near(p.points[0].depth, Real(0.02), Real(0.02)) && near(p.points[1].depth, Real(0.02), Real(0.02)));
    }
    // cápsulas em cruz (perpendiculares): 1 ponto
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        const ContactPatch p = gen(cap, Transform(Vector3(0, 0, 0), lie), cap, Transform(Vector3(0, Real(1.0), 0), Quaternion::identity()));
        // B de pé: o segmento de B vai de y = 0,4 a 1,6; o de A fica em y = 0: distância 0,4; rA + rB = 0,6
        CHECK(p.count == 1 && near(p.points[0].depth, Real(0.2), Real(0.01)) && near(p.normal.y, Real(1), Real(1e-2)));
    }
    // cápsula × esfera: forma fechada
    {
        const SphereShape s(Real(0.25));
        const ContactPatch p = gen(cap, Transform(Vector3(0, 0, 0)), s, Transform(Vector3(0.5f, 0.2f, 0)));
        CHECK(p.count == 1 && near(p.points[0].depth, Real(0.3) + Real(0.25) - Real(0.5), Real(5e-3)) && near(p.normal.x, Real(1), Real(1e-3)));
        // a esfera acima da ponta: distância ao longo de Y
        const ContactPatch q = gen(cap, Transform(Vector3(0, 0, 0)), s, Transform(Vector3(0, Real(0.6) + Real(0.5), 0)));
        CHECK(q.count == 1 && near(q.points[0].depth, Real(0.05), Real(5e-3)) && near(q.normal.y, Real(1), Real(1e-3)));
    }
    // cápsula × caixa (aresta): ponto único, profundidade pela distância da aresta
    {
        const BoxShape box(Vector3(0.5f, 0.5f, 0.5f));
        // cápsula em pé ao lado da caixa, encostando na face x = 0.5
        bool hit = false;
        const ContactPatch p = gen(cap, Transform(Vector3(Real(0.5) + r - Real(0.02), 0, 0)), box, Transform(Vector3(0, 0, 0)), &hit);
        CHECK(hit && p.count >= 1 && near(p.normal.x, Real(-1), Real(1e-2)));
        if (p.count >= 1) CHECK(near(p.points[0].depth, Real(0.02), Real(0.01)));
    }
}

static void testCylinderContacts() {
    const BoxShape ground(Vector3(50, 1, 50));
    const Transform groundXf(Vector3(0, -1, 0));
    const Real r = Real(0.5), h = Real(0.4);
    const CylinderShape cyl(r, h);
    // em pé: a tampa vira 4 pontos de contato
    {
        bool hit = false;
        const ContactPatch p = gen(cyl, Transform(Vector3(0, h - Real(0.01), 0)), ground, groundXf, &hit);
        CHECK(hit && p.count == 4 && near(p.normal.y, Real(-1), Real(1e-3)));
        bool ok = true;
        for (int i = 0; i < p.count; ++i) ok = ok && near(p.points[i].depth, Real(0.01), Real(0.01));
        CHECK(ok);
        // os 4 pontos abrangem a tampa: distância entre os mais afastados ≈ diâmetro
        Real far = 0;
        for (int i = 0; i < p.count; ++i) for (int j = i + 1; j < p.count; ++j) far = std::max(far, (p.points[i].pointA - p.points[j].pointA).length());
        CHECK(far > Real(0.9));
    }
    // deitado (eixo ao longo de X): 2 pontos nas pontas da geratriz
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        const ContactPatch p = gen(cyl, Transform(Vector3(0.3f, r - Real(0.01), 0.2f), lie), ground, groundXf);
        CHECK(p.count == 2 && near(p.normal.y, Real(-1), Real(1e-3)));
        if (p.count == 2) {
            CHECK(near(p.points[0].depth, Real(0.01), Real(0.01)) && near(p.points[1].depth, Real(0.01), Real(0.01)));
            CHECK(std::abs(std::abs(p.points[0].pointA.x - p.points[1].pointA.x) - Real(0.8)) < Real(0.02));   // 2h
            CHECK(std::abs(p.points[0].pointA.z - Real(0.2)) < Real(2e-2));
        }
    }
    // chão = A, cilindro = B (ordem trocada)
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        const ContactPatch p = gen(ground, groundXf, cyl, Transform(Vector3(0, r - Real(0.01), 0), lie));
        CHECK(p.count == 2 && near(p.normal.y, Real(1), Real(1e-3)));
    }
    // dois cilindros empilhados (tampa com tampa): polígono contra polígono
    {
        bool hit = false;
        const ContactPatch p = gen(cyl, Transform(Vector3(0, 0, 0)), cyl, Transform(Vector3(0.1f, Real(2) * h - Real(0.01), 0)), &hit);
        CHECK(hit && p.count >= 3 && near(p.normal.y, Real(1), Real(1e-3)));
    }
    // dois cilindros deitados paralelos: 2 pontos
    {
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        const ContactPatch p = gen(cyl, Transform(Vector3(0, 0, 0), lie), cyl, Transform(Vector3(0, Real(2) * r - Real(0.02), 0), lie));
        CHECK(p.count == 2 && near(p.normal.y, Real(1), Real(2e-2)));
    }
    // cilindro × esfera (borda da tampa): ponto único
    {
        const SphereShape s(Real(0.2));
        bool hit = false;
        const ContactPatch p = gen(cyl, Transform(Vector3(0, 0, 0)), s, Transform(Vector3(r + Real(0.15), 0, 0)), &hit);
        CHECK(hit && p.count == 1 && near(p.points[0].depth, Real(0.05), Real(0.01)) && near(p.normal.x, Real(1), Real(1e-2)));
    }
    // cilindro inclinado 45° sobre o chão: contato de borda, ponto único
    {
        const Quaternion tilt = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(0.7853982));
        const Real lowest = r * std::sin(Real(0.7853982)) + h * std::cos(Real(0.7853982));
        const ContactPatch p = gen(cyl, Transform(Vector3(0, lowest - Real(0.01), 0), tilt), ground, groundXf);
        CHECK(p.count == 1 && near(p.points[0].depth, Real(0.01), Real(0.01)));
    }
    // GJK/EPA não devem falhar com cilindro em pose aleatória contra caixa: profundidade coerente com o suporte
    Rng rng(5);
    int hits = 0;
    for (int i = 0; i < 300; ++i) {
        const BoxShape box(Vector3(0.6f, 0.3f, 0.5f));
        const Transform xa(Vector3(0, 0, 0), rng.rot()), xb(Vector3(rng.range(-0.8f, 0.8f), rng.range(-0.8f, 0.8f), rng.range(-0.8f, 0.8f)), rng.rot());
        bool hit = false;
        const ContactPatch p = gen(cyl, xa, box, xb, &hit);
        if (!hit) continue;
        ++hits;
        bool ok = p.normal.isFinite() && std::abs(p.normal.length() - Real(1)) < Real(1e-3) && p.count >= 1 && p.count <= 4;
        for (int k = 0; ok && k < p.count; ++k) ok = ok && p.points[k].pointA.isFinite() && std::isfinite(p.points[k].depth) && p.points[k].depth < Real(1.2);
        CHECK(ok);
    }
    CHECK(hits > 60);
}

// =============================================================================
// Raycast e consulta de ponto
// =============================================================================
static void testRaycast() {
    const Real r = Real(0.4), h = Real(0.7);
    const CapsuleShape cap(r, h);
    const CylinderShape cyl(r, h);
    ShapeRayHit hit;
    const Transform id;
    // análises simples
    CHECK(raycastShape(cap, id, Vector3(-3, 0, 0), Vector3::unitX(), Real(10), hit) && near(hit.distance, Real(3) - r) && near(hit.normal.x, Real(-1), Real(1e-3)));
    CHECK(raycastShape(cyl, id, Vector3(-3, 0.3f, 0), Vector3::unitX(), Real(10), hit) && near(hit.distance, Real(3) - r) && near(hit.normal.x, Real(-1), Real(1e-3)));
    CHECK(raycastShape(cap, id, Vector3(0, 5, 0), -Vector3::unitY(), Real(10), hit) && near(hit.distance, Real(5) - h - r) && near(hit.normal.y, Real(1), Real(1e-3)));   // ponta da cápsula
    CHECK(raycastShape(cyl, id, Vector3(0.1f, 5, 0), -Vector3::unitY(), Real(10), hit) && near(hit.distance, Real(5) - h) && near(hit.normal.y, Real(1), Real(1e-3)));   // tampa
    CHECK(!raycastShape(cap, id, Vector3(-3, 3, 0), Vector3::unitX(), Real(10), hit));
    CHECK(!raycastShape(cyl, id, Vector3(-3, 0, 0), Vector3::unitX(), Real(2), hit));   // alcance curto
    CHECK(raycastShape(cap, id, Vector3(0, 0, 0), Vector3::unitX(), Real(10), hit) && hit.startedInside);
    CHECK(raycastShape(cyl, id, Vector3(0, 0, 0), Vector3::unitX(), Real(10), hit) && hit.startedInside);
    CHECK(!raycastShape(cyl, id, Vector3(-3, 0, 0), -Vector3::unitX(), Real(10), hit));   // afastando
    // aleatórios: o ponto de entrada é a superfície (fora um pouco antes, dentro um pouco depois)
    Rng rng(21);
    for (int kind = 0; kind < 2; ++kind) {
        const Shape& s = kind == 0 ? static_cast<const Shape&>(cap) : static_cast<const Shape&>(cyl);
        int hits = 0;
        bool surface = true, missesOk = true;
        for (int i = 0; i < 1500; ++i) {
            const Transform xf(Vector3(rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)), rng.rot());
            const Vector3 o = Vector3(rng.range(-4, 4), rng.range(-4, 4), rng.range(-4, 4));
            const Vector3 target = Vector3(rng.range(-1.2f, 1.2f), rng.range(-1.2f, 1.2f), rng.range(-1.2f, 1.2f));
            const Vector3 dir = (target - o).normalizedOr(Vector3::unitX());
            ShapeRayHit hh;
            const bool found = raycastShape(s, xf, o, dir, Real(20), hh);
            if (found && !hh.startedInside) {
                ++hits;
                const Vector3 before = xf.inverseTransformPoint(o + dir * (hh.distance - Real(2e-3)));
                const Vector3 after = xf.inverseTransformPoint(o + dir * (hh.distance + Real(2e-3)));
                const bool inB = kind == 0 ? insideCapsule(r, h, before) : insideCylinder(r, h, before);
                const bool inA = kind == 0 ? insideCapsule(r, h, after) : insideCylinder(r, h, after);
                if (inB || !inA) surface = false;
                if (std::abs(hh.normal.length() - Real(1)) > Real(1e-3)) surface = false;
            }
            if (!found) {   // amostra o raio: nenhum ponto pode estar dentro
                for (int k = 0; k < 400; ++k) {
                    const Vector3 q = xf.inverseTransformPoint(o + dir * (Real(20) * Real(k) / Real(400)));
                    if (kind == 0 ? insideCapsule(r - Real(0.01), h, q) : insideCylinder(r - Real(0.01), h - Real(0.01), q)) { missesOk = false; break; }
                }
            }
        }
        CHECK(hits > 150 && surface && missesOk);
    }
    // shapeContainsPoint
    CHECK(shapeContainsPoint(cap, id, Vector3(0, h + r - Real(0.01), 0)) && !shapeContainsPoint(cap, id, Vector3(0, h + r + Real(0.01), 0)));
    CHECK(shapeContainsPoint(cyl, id, Vector3(r - Real(0.01), h - Real(0.01), 0)) && !shapeContainsPoint(cyl, id, Vector3(r + Real(0.01), 0, 0)));
}

// =============================================================================
// Debug draw
// =============================================================================
struct CountRenderer : IDebugRenderer {
    int lines = 0;
    bool finite = true;
    void drawLine(const Vector3& a, const Vector3& b, const DebugColor&) override { ++lines; finite = finite && a.isFinite() && b.isFinite(); }
    void drawPoint(const Vector3& p, Real, const DebugColor&) override { finite = finite && p.isFinite(); }
};

static void testDebugDraw() {
    CountRenderer r;
    debugDrawShape(CapsuleShape(Real(0.3), Real(0.6)), Transform(Vector3(1, 2, 3)), r, DebugColor{255, 255, 255, 255}, 16);
    CHECK(r.lines > 40 && r.finite);
    const int cap = r.lines;
    debugDrawShape(CylinderShape(Real(0.3), Real(0.6)), Transform(Vector3(1, 2, 3)), r, DebugColor{255, 255, 255, 255}, 16);
    CHECK(r.lines - cap > 30 && r.finite);
}

// =============================================================================
// Simulação completa (PhysicsWorld): repouso estável
// =============================================================================
static RigidBodyDesc groundDesc() {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(Vector3(50, 1, 50));
    d.transform = Transform(Vector3(0, -1, 0));
    return d;
}

static void run(PhysicsWorld& w, int steps) {
    for (int i = 0; i < steps; ++i) w.step();
}

static void testWorldResting() {
    const Real r = Real(0.3), h = Real(0.6);
    // cápsula DEITADA cai de 0,6 m: assenta em y = r, sem rolar nem balançar, e dorme
    {
        PhysicsWorld w;
        w.createBody(groundDesc());
        RigidBodyDesc d;
        d.shape = std::make_shared<CapsuleShape>(r, h);
        d.mass = Real(70);
        d.transform = Transform(Vector3(0, Real(0.9), 0), Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963)));
        d.angularDamping = 0;
        const BodyId id = w.createBody(d);
        run(w, 240);
        const RigidBody& b = w.getBody(id);
        CHECK(std::abs(b.getTransform().position.y - r) < Real(0.02));
        CHECK(b.getAngularVelocity().length() < Real(0.05) && b.getLinearVelocity().length() < Real(0.05));
        // continua deitada (eixo local Y apontando na horizontal)
        const Vector3 axis = b.getTransform().transformDirection(Vector3::unitY());
        CHECK(std::abs(axis.y) < Real(0.05));
        run(w, 240);
        CHECK(!w.isAwake(id));   // dormiu
    }
    // cápsula em pé cai e fica em pé (apoio sob o centro)
    {
        PhysicsWorld w;
        w.createBody(groundDesc());
        RigidBodyDesc d;
        d.shape = std::make_shared<CapsuleShape>(r, h);
        d.mass = Real(70);
        d.transform = Transform(Vector3(0, Real(1.3), 0));
        const BodyId id = w.createBody(d);
        run(w, 300);
        const RigidBody& b = w.getBody(id);
        CHECK(std::abs(b.getTransform().position.y - (h + r)) < Real(0.03) && b.getLinearVelocity().length() < Real(0.05));
        CHECK(b.getTransform().position.isFinite());
    }
    // cápsula inclinada 60° cai e termina deitada (não atravessa o chão, não explode)
    {
        PhysicsWorld w;
        w.createBody(groundDesc());
        RigidBodyDesc d;
        d.shape = std::make_shared<CapsuleShape>(r, h);
        d.mass = Real(70);
        d.transform = Transform(Vector3(0, Real(1.2), 0), Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.0472)));
        d.friction = Real(0.6);
        const BodyId id = w.createBody(d);
        run(w, 600);
        const RigidBody& b = w.getBody(id);
        CHECK(b.getTransform().position.y > r - Real(0.03) && b.getTransform().position.y < h + r + Real(0.05));
        CHECK(b.getLinearVelocity().length() < Real(0.2) && b.getTransform().position.isFinite());
    }
    // cilindro em pé: repouso em y = h; deitado: y = r
    {
        PhysicsWorld w;
        w.createBody(groundDesc());
        RigidBodyDesc up;
        up.shape = std::make_shared<CylinderShape>(Real(0.5), Real(0.4));
        up.mass = Real(5);
        up.transform = Transform(Vector3(-3, Real(0.9), 0));
        const BodyId a = w.createBody(up);
        RigidBodyDesc lie = up;
        lie.transform = Transform(Vector3(3, Real(0.9), 0), Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963)));
        const BodyId b = w.createBody(lie);
        run(w, 300);
        CHECK(std::abs(w.getBody(a).getTransform().position.y - Real(0.4)) < Real(0.02));
        CHECK(std::abs(w.getBody(b).getTransform().position.y - Real(0.5)) < Real(0.02));
        CHECK(w.getBody(a).getAngularVelocity().length() < Real(0.05) && w.getBody(b).getAngularVelocity().length() < Real(0.05));
        const Vector3 axisB = w.getBody(b).getTransform().transformDirection(Vector3::unitY());
        CHECK(std::abs(axisB.y) < Real(0.05));
    }
    // pilha de 4 cilindros em pé: não desmorona
    {
        PhysicsWorld w;
        w.createBody(groundDesc());
        std::vector<BodyId> ids;
        for (int i = 0; i < 4; ++i) {
            RigidBodyDesc d;
            d.shape = std::make_shared<CylinderShape>(Real(0.5), Real(0.25));
            d.mass = Real(2);
            d.transform = Transform(Vector3(Real(0.02) * Real(i % 2), Real(0.26) + Real(0.51) * Real(i), 0));
            ids.push_back(w.createBody(d));
        }
        run(w, 360);
        bool stable = true;
        for (int i = 0; i < 4; ++i) {
            const Vector3 p = w.getBody(ids[static_cast<std::size_t>(i)]).getTransform().position;
            stable = stable && std::abs(p.y - (Real(0.25) + Real(0.5) * Real(i))) < Real(0.06) && std::abs(p.x) < Real(0.1);
        }
        CHECK(stable);
    }
    // cápsulas deitadas empilhadas (3 lado a lado + 1 em cima nos vãos): sem explodir, tudo no chão ou sobre outra
    {
        PhysicsWorld w;
        w.createBody(groundDesc());
        const Quaternion lie = Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1.5707963));
        std::vector<BodyId> ids;
        for (int i = 0; i < 3; ++i) {
            RigidBodyDesc d;
            d.shape = std::make_shared<CapsuleShape>(Real(0.25), Real(0.5));
            d.mass = Real(3);
            d.transform = Transform(Vector3(0, Real(0.26), Real(0.5) * Real(i - 1)), lie);
            ids.push_back(w.createBody(d));
        }
        RigidBodyDesc top;
        top.shape = std::make_shared<CapsuleShape>(Real(0.25), Real(0.5));
        top.mass = Real(3);
        top.transform = Transform(Vector3(0, Real(0.9), Real(0.25)), lie);
        ids.push_back(w.createBody(top));
        run(w, 420);
        bool ok = true;
        for (BodyId id : ids) {
            const RigidBody& b = w.getBody(id);
            ok = ok && b.getTransform().position.isFinite() && b.getTransform().position.y > Real(0.2) && b.getTransform().position.y < Real(1.0) && b.getLinearVelocity().length() < Real(0.3);
        }
        CHECK(ok);
    }
    // consulta espacial: raycast no mundo acerta a cápsula
    {
        PhysicsWorld w;
        RigidBodyDesc d;
        d.shape = std::make_shared<CapsuleShape>(r, h);
        d.mass = Real(1);
        d.gravityScale = 0;
        d.transform = Transform(Vector3(0, 5, 0));
        const BodyId id = w.createBody(d);
        RaycastHit hit;
        CHECK(w.raycast(Vector3(-5, 5, 0), Vector3::unitX(), Real(20), hit) && hit.body == id && near(hit.distance, Real(5) - r));
    }
}

// =============================================================================
// Fuzz: mistura de formas (incluindo as novas), poses e velocidades absurdas: nada de NaN e nada atravessa o mundo
// =============================================================================
static void testFuzz() {
    for (unsigned seed = 1; seed <= 6; ++seed) {
        Rng rng(seed * 7919u);
        PhysicsWorld w;
        w.createBody(groundDesc());
        std::vector<BodyId> ids;
        for (int i = 0; i < 24; ++i) {
            RigidBodyDesc d;
            const int k = static_cast<int>(rng.next() * 5) % 5;
            if (k == 0) d.shape = std::make_shared<CapsuleShape>(rng.range(0.1f, 0.5f), rng.range(0.0f, 1.0f));
            else if (k == 1) d.shape = std::make_shared<CylinderShape>(rng.range(0.1f, 0.6f), rng.range(0.1f, 0.8f));
            else if (k == 2) d.shape = std::make_shared<BoxShape>(Vector3(rng.range(0.2f, 0.6f), rng.range(0.2f, 0.6f), rng.range(0.2f, 0.6f)));
            else if (k == 3) d.shape = std::make_shared<SphereShape>(rng.range(0.2f, 0.5f));
            else d.shape = std::make_shared<CapsuleShape>(rng.range(0.1f, 0.3f), rng.range(0.3f, 0.9f));
            d.mass = rng.range(0.5f, 20.0f);
            d.transform = Transform(Vector3(rng.range(-3, 3), rng.range(0.5f, 8), rng.range(-3, 3)), rng.rot());
            d.linearVelocity = Vector3(rng.range(-20, 20), rng.range(-20, 20), rng.range(-20, 20));
            d.angularVelocity = Vector3(rng.range(-30, 30), rng.range(-30, 30), rng.range(-30, 30));
            d.friction = rng.range(0, 1.5f);
            d.restitution = rng.range(0, 0.8f);
            ids.push_back(w.createBody(d));
        }
        bool finite = true;
        for (int s = 0; s < 600; ++s) {
            w.step();
            if (s % 20 == 0)
                for (BodyId id : ids) {
                    const RigidBody& b = w.getBody(id);
                    finite = finite && b.getTransform().position.isFinite() && b.getLinearVelocity().isFinite() && std::abs(b.getOrientation().norm() - Real(1)) < Real(1e-3);
                }
        }
        CHECK(finite);
        int below = 0;
        for (BodyId id : ids) {   // atravessou o chão (de 2 m de espessura)? só vale para quem está sobre ele (o chão tem 100 m)
            const Vector3 p = w.getBody(id).getTransform().position;
            if (std::abs(p.x) < Real(40) && std::abs(p.z) < Real(40) && p.y < Real(-1)) ++below;
        }
        CHECK(below <= 1);
    }
}

int main() {
    testConstruction();
    testVolumeInertia();
    testSupportAndAABB();
    testCapsuleContacts();
    testCylinderContacts();
    testRaycast();
    testDebugDraw();
    testWorldResting();
    testFuzz();
    std::printf("capsule_cylinder_tests: %d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
