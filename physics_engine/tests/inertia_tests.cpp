// =============================================================================
// inertia_tests.cpp (V25) — revisão do momento de inércia para efeitos de pião e giroscópio.
//   1. Tensor de inércia de cada forma x integração numérica independente (grade de pontos dentro do sólido).
//   2. Pião simétrico livre (sem torque): ω no referencial do corpo precessiona a (Ia − Ip)/Ip · ω_axial.
//   3. Giroscópio num pivô (esfera-soquete) com gravidade: precessão ≈ m g l / (I_axial ω_spin).
//   4. Corpo com centro de massa deslocado do referencial: o centro de massa segue reta, L conservado.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <vector>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/CapsuleShape.h"
#include "physics/geometry/CylinderShape.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/pipeline/PhysicsWorld.h"

using namespace physics;

static int g_failures = 0, g_checks = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        ++g_checks;                                                       \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            std::printf("FALHOU %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

// ----------------------------------------------------------------------------- 1. grade numérica
struct GridResult {
    double volume = 0;
    double c[3] = {0, 0, 0};
    double I[3][3] = {{0}};   // por unidade de massa, em torno do centroide
};

static GridResult gridIntegrate(const std::function<bool(double, double, double)>& inside, const double lo[3], const double hi[3], int n) {
    GridResult r;
    double m1[3] = {0, 0, 0}, m2[3][3] = {{0}};
    long count = 0;
    double d[3] = {(hi[0] - lo[0]) / n, (hi[1] - lo[1]) / n, (hi[2] - lo[2]) / n};
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < n; ++k) {
                const double p[3] = {lo[0] + (i + 0.5) * d[0], lo[1] + (j + 0.5) * d[1], lo[2] + (k + 0.5) * d[2]};
                if (!inside(p[0], p[1], p[2])) continue;
                ++count;
                for (int a = 0; a < 3; ++a) {
                    m1[a] += p[a];
                    for (int b = 0; b < 3; ++b) m2[a][b] += p[a] * p[b];
                }
            }
    r.volume = static_cast<double>(count) * d[0] * d[1] * d[2];
    const double cnt = static_cast<double>(count);
    for (int a = 0; a < 3; ++a) r.c[a] = m1[a] / cnt;
    double s[3][3];   // segundos momentos centrais por unidade de massa
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) s[a][b] = m2[a][b] / cnt - r.c[a] * r.c[b];
    const double tr = s[0][0] + s[1][1] + s[2][2];
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) r.I[a][b] = (a == b ? tr : 0.0) - s[a][b];
    return r;
}

static void compareShape(const char* name, const Shape& shape, const std::function<bool(double, double, double)>& inside, const double lo[3], const double hi[3]) {
    const GridResult g = gridIntegrate(inside, lo, hi, 110);
    const Matrix3 I = shape.computeInertia(Real(1));
    double scale = 0;
    for (int a = 0; a < 3; ++a) scale = std::max(scale, g.I[a][a]);
    double maxErr = 0;
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) maxErr = std::max(maxErr, std::abs(static_cast<double>(I(a, b)) - g.I[a][b]) / scale);
    const Vector3 cc = shape.getCentroid();
    const double cErr = std::max({std::abs(cc.x - g.c[0]), std::abs(cc.y - g.c[1]), std::abs(cc.z - g.c[2])});
    const double vErr = std::abs(static_cast<double>(shape.getVolume()) - g.volume) / g.volume;
    std::printf("  [info] %-10s inercia: erro max %.3f%% do maior I | centroide %.4f | volume %.3f%%\n", name, maxErr * 100, cErr, vErr * 100);
    CHECK(maxErr < 0.02);
    CHECK(cErr < 0.01);
    CHECK(vErr < 0.02);
}

static void testShapeInertia() {
    {   // caixa
        BoxShape s(Vector3(0.5f, 1.0f, 1.5f));
        const double lo[3] = {-0.5, -1, -1.5}, hi[3] = {0.5, 1, 1.5};
        compareShape("caixa", s, [](double, double, double) { return true; }, lo, hi);
    }
    {   // esfera
        SphereShape s(Real(0.8));
        const double lo[3] = {-0.8, -0.8, -0.8}, hi[3] = {0.8, 0.8, 0.8};
        compareShape("esfera", s, [](double x, double y, double z) { return x * x + y * y + z * z <= 0.64; }, lo, hi);
    }
    {   // cápsula (eixo Y)
        const double r = 0.4, h = 0.7;
        CapsuleShape s{Real(r), Real(h)};
        const double lo[3] = {-r, -h - r, -r}, hi[3] = {r, h + r, r};
        compareShape("capsula", s, [=](double x, double y, double z) {
            const double cy = std::max(-h, std::min(h, y));
            return x * x + (y - cy) * (y - cy) + z * z <= r * r;
        }, lo, hi);
    }
    {   // cilindro (eixo Y)
        const double r = 0.5, h = 0.9;
        CylinderShape s{Real(r), Real(h)};
        const double lo[3] = {-r, -h, -r}, hi[3] = {r, h, r};
        compareShape("cilindro", s, [=](double x, double, double z) { return x * x + z * z <= r * r; }, lo, hi);
    }
    {   // tetraedro irregular afastado da origem: centroide != 0 e produtos de inércia != 0
        const Vector3 off(2, -1, 3);
        const std::vector<Vector3> v = {Vector3(0, 0, 0) + off, Vector3(2, 0, 0) + off, Vector3(0, 1, 0) + off, Vector3(0.3f, 0.2f, 1.5f) + off};
        PolyhedronShape t(v, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}});
        const HalfEdgeStructure& topo = t.getTopology();
        const double lo[3] = {2, -1, 3}, hi[3] = {4, 0.2, 4.5};
        // dentro = atrás de todos os planos das faces; o integral usa coordenadas do sólido, o centroide vem em coordenadas do sólido.
        compareShape("tetraedro", t, [&](double x, double y, double z) {
            for (const HEFace& f : topo.faces())
                if (f.normal.x * x + f.normal.y * y + f.normal.z * z > f.offset) return false;
            return true;
        }, lo, hi);
        const Matrix3 I = t.computeInertia(Real(1));
        CHECK(std::abs(I(0, 1)) > Real(1e-3));   // os produtos de inércia existem de fato (tensor cheio, não só a diagonal)
        CHECK(I.nearlyEqual(I.transposed(), Real(1e-5)));
    }
}

// ----------------------------------------------------------------------------- utilitários dinâmicos
static PhysicsWorldSettings noGravity() {
    PhysicsWorldSettings s;
    s.gravity = Vector3::zero();
    return s;
}

// ----------------------------------------------------------------------------- 2. pião livre
static void testFreeSymmetricTop() {
    PhysicsWorld w(noGravity());
    w.sleepSettings().enabled = false;
    RigidBodyDesc d;
    d.shape = std::make_shared<CylinderShape>(Real(1), Real(0.2));   // disco achatado: Ia > Ip
    d.mass = Real(2);
    d.angularDamping = 0;
    const double spin = 20.0, tilt = 2.0;
    d.angularVelocity = Vector3(static_cast<Real>(tilt), static_cast<Real>(spin), 0);
    const BodyId id = w.createBody(d);
    const RigidBody& b = w.getBody(id);
    const Matrix3 I = b.getMassData().inertia;
    const double Ia = I(1, 1), Ip = I(0, 0);
    const double omegaBody = (Ia - Ip) / Ip * spin;   // taxa de giro de ω_perp no referencial do corpo
    const double e0 = static_cast<double>(b.getKineticEnergy());
    const double l0 = static_cast<double>(b.getAngularMomentum().length());
    const int steps = 120;   // 2 s a 60 Hz
    double phase = 0, prev = std::atan2(static_cast<double>(tilt), 0.0), maxE = 0, maxL = 0, maxAx = 0;
    prev = std::atan2(static_cast<double>(b.getOrientation().rotateInverse(b.getAngularVelocity()).z), static_cast<double>(b.getOrientation().rotateInverse(b.getAngularVelocity()).x));
    for (int i = 0; i < steps; ++i) {
        w.step();
        const Vector3 wb = b.getOrientation().rotateInverse(b.getAngularVelocity());
        double ph = std::atan2(static_cast<double>(wb.z), static_cast<double>(wb.x));
        double dph = ph - prev;
        while (dph > 3.14159265358979) dph -= 6.28318530717959;
        while (dph < -3.14159265358979) dph += 6.28318530717959;
        phase += dph;
        prev = ph;
        maxE = std::max(maxE, std::abs(static_cast<double>(b.getKineticEnergy()) - e0) / e0);
        maxL = std::max(maxL, std::abs(static_cast<double>(b.getAngularMomentum().length()) - l0) / l0);
        maxAx = std::max(maxAx, std::abs(static_cast<double>(wb.y) - spin) / spin);
    }
    const double measured = phase / (steps / 60.0);
    std::printf("  [info] piao livre: taxa de precessao no corpo medida %.3f rad/s, teorica %.3f | dE=%.2e dL=%.2e dw_axial=%.2e\n", measured, omegaBody, maxE, maxL, maxAx);
    CHECK(std::abs(std::abs(measured) - std::abs(omegaBody)) < 0.02 * std::abs(omegaBody));
    CHECK(maxE < 2e-3 && maxL < 2e-3);
    CHECK(maxAx < 2e-3);   // a componente axial de ω é constante num pião simétrico livre
}

// ----------------------------------------------------------------------------- 3. giroscópio no pivô
static void testPivotedGyroscope() {
    PhysicsWorldSettings gs;
    PhysicsWorld w(gs);
    w.sleepSettings().enabled = false;
    RigidBodyDesc pivot;
    pivot.type = BodyType::Static;
    pivot.shape = std::make_shared<BoxShape>(Vector3(0.05f, 0.05f, 0.05f));
    const BodyId pid = w.createBody(pivot);
    const Real m = Real(1), r = Real(0.5), hh = Real(0.05), l = Real(0.6), spin = Real(60);
    RigidBodyDesc d;
    d.shape = std::make_shared<CylinderShape>(r, hh);
    d.mass = m;
    d.angularDamping = 0;
    d.transform = Transform(Vector3(l, 0, 0), Quaternion::fromAxisAngle(Vector3(0, 0, 1), Real(-1.5707963267948966)));   // eixo local Y -> mundo +X
    d.angularVelocity = Vector3(spin, 0, 0);
    const BodyId gid = w.createBody(d);
    w.createBallSocketJoint(pid, gid, Vector3(0, 0, 0));
    const RigidBody& g = w.getBody(gid);
    const double Ia = g.getMassData().inertia(1, 1);
    const double expected = static_cast<double>(m) * 9.81 * static_cast<double>(l) / (Ia * static_cast<double>(spin));   // rad/s em torno de +Y
    auto heading = [&]() {
        const Vector3 ax = g.getOrientation().rotate(Vector3(0, 1, 0));
        return std::atan2(-static_cast<double>(ax.z), static_cast<double>(ax.x));
    };
    const int skip = 30, total = 210;   // mede entre 0.5 s e 3.5 s
    double h0 = 0, h1 = 0, minY = 1e9, maxY = -1e9;
    for (int i = 0; i <= total; ++i) {
        w.step();
        if (i == skip) h0 = heading();
        if (i == total) h1 = heading();
        const double y = static_cast<double>(g.getWorldCenterOfMass().y);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
    }
    const double measured = (h1 - h0) / ((total - skip) / 60.0);
    const double spinNow = static_cast<double>(g.getOrientation().rotateInverse(g.getAngularVelocity()).y);
    std::printf("  [info] giroscopio no pivo: precessao medida %.3f rad/s, teorica m g l/(Ia w) = %.3f | spin restante %.1f/%.1f | y do centro em [%.3f, %.3f]\n",
                measured, expected, spinNow, double(spin), minY, maxY);
    CHECK(measured > 0);                                       // sentido certo (+Y)
    CHECK(std::abs(measured - expected) < 0.10 * expected);    // taxa de precessão
    CHECK(spinNow > 0.95 * static_cast<double>(spin));         // o spin não se perde
    CHECK(minY > -0.25);                                       // não cai: o giroscópio "desafia" a gravidade
}

// ----------------------------------------------------------------------------- 4. centro de massa deslocado
static void testOffsetCenterOfMass() {
    PhysicsWorld w(noGravity());
    w.sleepSettings().enabled = false;
    const Vector3 off(2, -1, 3);
    const std::vector<Vector3> v = {Vector3(0, 0, 0) + off, Vector3(2, 0, 0) + off, Vector3(0, 1, 0) + off, Vector3(0.3f, 0.2f, 1.5f) + off};
    RigidBodyDesc d;
    d.shape = std::make_shared<PolyhedronShape>(v, std::vector<std::vector<int>>{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}});
    d.mass = Real(3);
    d.angularDamping = 0;
    d.linearVelocity = Vector3(1, 0, 0.5f);
    d.angularVelocity = Vector3(3, 4, 5);
    const BodyId id = w.createBody(d);
    const RigidBody& b = w.getBody(id);
    const Vector3 c0 = b.getWorldCenterOfMass();
    const double e0 = static_cast<double>(b.getKineticEnergy());
    const Vector3 l0 = b.getAngularMomentum();
    for (int i = 0; i < 180; ++i) w.step();
    const Vector3 expected = c0 + Vector3(1, 0, 0.5f) * Real(3);
    std::printf("  [info] CM deslocado 3 s: desvio do centro de massa %.2e m | dE=%.2e | dL=%.2e\n", double((b.getWorldCenterOfMass() - expected).length()),
                std::abs(static_cast<double>(b.getKineticEnergy()) - e0) / e0, double((b.getAngularMomentum() - l0).length() / l0.length()));
    CHECK((b.getWorldCenterOfMass() - expected).length() < Real(1e-3));          // sem força: o CM segue reta
    CHECK(std::abs(static_cast<double>(b.getKineticEnergy()) - e0) / e0 < 5e-3);
    CHECK((b.getAngularMomentum() - l0).length() / l0.length() < Real(5e-3));    // L conservado (gira em torno do CM, não do referencial)
}

int main() {
    testShapeInertia();
    testFreeSymmetricTop();
    testPivotedGyroscope();
    testOffsetCenterOfMass();
    std::printf("inertia_tests: %d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
