// =============================================================================
// friction_tests.cpp  (V19)
// Atrito de torção (giro em torno da normal) e resistência à rolagem em formas redondas.
// Bug corrigido: uma esfera girando em Y (ou rolando) no chão nunca parava, porque o cone de Coulomb só
// atua nos pontos de contato e uma esfera toca em um ponto só.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

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

namespace {

BodyId addGround(PhysicsWorld& w, Real mu) {
    RigidBodyDesc g;
    g.type = BodyType::Static;
    g.shape = std::make_shared<BoxShape>(Vector3(200, 0.5, 200));
    g.transform = Transform(Vector3(0, Real(-0.5), 0));
    g.friction = mu;
    return w.createBody(g);
}

BodyId addBody(PhysicsWorld& w, std::shared_ptr<const Shape> s, Real y, Real mu, const Vector3& v, const Vector3& om) {
    RigidBodyDesc d;
    d.shape = std::move(s);
    d.mass = 1;
    d.friction = mu;
    d.restitution = 0;
    d.angularDamping = 0;
    d.allowSleep = false;   // medimos a velocidade de verdade, sem o sleeping zerá-la
    d.transform = Transform(Vector3(0, y, 0));
    d.linearVelocity = v;
    d.angularVelocity = om;
    return w.createBody(d);
}

void run(PhysicsWorld& w, int steps) {
    for (int i = 0; i < steps; ++i) w.step();
}

Real lin(PhysicsWorld& w, BodyId id) { return w.getBody(id).getLinearVelocity().length(); }
Real ang(PhysicsWorld& w, BodyId id) { return w.getBody(id).getAngularVelocity().length(); }

} // namespace

int main() {
    const std::shared_ptr<const Shape> sphere = std::make_shared<SphereShape>(Real(0.5));
    const std::shared_ptr<const Shape> capsule = std::make_shared<CapsuleShape>(Real(0.3), Real(0.5));
    const std::shared_ptr<const Shape> cylinder = std::make_shared<CylinderShape>(Real(0.4), Real(0.5));

    // 1) giro em Y de uma esfera, caixa de atrito 0,5: pára em poucos segundos e não sai do lugar
    {
        PhysicsWorld w;
        addGround(w, Real(0.5));
        const BodyId s = addBody(w, sphere, Real(0.5), Real(0.5), Vector3(), Vector3(0, 10, 0));
        run(w, 60 * 12);
        CHECK(ang(w, s) < Real(0.05));
        CHECK(lin(w, s) < Real(0.02));
        CHECK(std::abs(w.getBody(s).getTransform().position.x) < Real(0.05));
    }
    // 2) atrito máximo para um giro em Y pára mais depressa que atrito baixo; atrito 0: não freia (gelo)
    {
        Real t90[3];
        const Real mus[3] = {Real(0.1), Real(0.5), Real(2)};
        for (int k = 0; k < 3; ++k) {
            PhysicsWorld w;
            addGround(w, mus[k]);
            const BodyId s = addBody(w, sphere, Real(0.5), mus[k], Vector3(), Vector3(0, 10, 0));
            int n = 0;
            while (n < 60 * 60 && ang(w, s) > Real(1)) { w.step(); ++n; }
            t90[k] = Real(n) / Real(60);
        }
        CHECK(t90[0] > t90[1] && t90[1] > t90[2]);
        PhysicsWorld w;
        addGround(w, 0);
        const BodyId s = addBody(w, sphere, Real(0.5), 0, Vector3(), Vector3(0, 10, 0));
        run(w, 60 * 10);
        CHECK(std::abs(ang(w, s) - Real(10)) < Real(1e-2));   // μ = 0: nada freia
    }
    // 2b) controle negativo: com spin/rolling desligados (o comportamento ANTIGO) o giro em Y não pára
    {
        PhysicsWorld w;
        w.solverSettings().spinFriction = 0;
        w.solverSettings().rollingFriction = 0;
        addGround(w, Real(2));
        const BodyId s = addBody(w, sphere, Real(0.5), Real(2), Vector3(), Vector3(0, 10, 0));
        run(w, 60 * 12);
        CHECK(std::abs(ang(w, s) - Real(10)) < Real(0.05));
    }
    // 3) deslizando: vira rolagem pura (v = ω·r) e depois a resistência à rolagem pára a bola
    {
        PhysicsWorld w;
        addGround(w, Real(0.5));
        const BodyId s = addBody(w, sphere, Real(0.5), Real(0.5), Vector3(5, 0, 0), Vector3());
        run(w, 30);
        const Real v = w.getBody(s).getLinearVelocity().x;
        CHECK(v < Real(5) && v > Real(2.5));   // o atrito de deslizamento já agia
        run(w, 60 * 40);
        CHECK(lin(w, s) < Real(0.05) && ang(w, s) < Real(0.1));
    }
    // 4) rolagem em X e em Z (ω horizontal) pára
    for (int axis = 0; axis < 2; ++axis) {
        PhysicsWorld w;
        addGround(w, Real(0.8));
        const Vector3 om = axis == 0 ? Vector3(8, 0, 0) : Vector3(0, 0, 8);
        const BodyId s = addBody(w, sphere, Real(0.5), Real(0.8), Vector3(), om);
        run(w, 60 * 30);
        CHECK(ang(w, s) < Real(0.1) && lin(w, s) < Real(0.1));
    }
    // 5) cápsula e cilindro: giro em Y em pé (apoio plano) também pára; cápsula deitada rolando pára
    {
        PhysicsWorld w;
        addGround(w, Real(0.6));
        const BodyId c = addBody(w, capsule, Real(0.8), Real(0.6), Vector3(), Vector3(0, 8, 0));
        const BodyId y = addBody(w, cylinder, Real(0.5), Real(0.6), Vector3(5, 0, 0), Vector3(0, 8, 0));
        w.getBody(y).setPose(Vector3(5, Real(0.5), 0), Quaternion::identity());
        run(w, 60 * 25);
        CHECK(ang(w, c) < Real(0.1));
        CHECK(ang(w, y) < Real(0.2));
    }
    // 6) não perturba o que já funcionava: caixa girando em Y sobre o chão continua sendo freada pelo atrito normal,
    //    e uma esfera em repouso fica em repouso (sem deriva)
    {
        PhysicsWorld w;
        addGround(w, Real(0.6));
        const BodyId b = addBody(w, std::make_shared<BoxShape>(Vector3(0.5, 0.5, 0.5)), Real(0.5), Real(0.6), Vector3(), Vector3(0, 6, 0));
        const BodyId s = addBody(w, sphere, Real(0.5), Real(0.6), Vector3(), Vector3());
        w.getBody(s).setPose(Vector3(5, Real(0.5), 0), Quaternion::identity());
        run(w, 60 * 10);
        CHECK(ang(w, b) < Real(0.1));
        CHECK(lin(w, s) < Real(1e-3) && ang(w, s) < Real(1e-3));
        CHECK(std::abs(w.getBody(s).getTransform().position.x - Real(5)) < Real(1e-3));
    }
    // 7) bola numa rampa (30 graus) rola para baixo e acelera menos que 5/7·g·sen(θ)·... mas continua descendo: a resistência
    //    à rolagem não a prende (torque de resistência << torque da gravidade)
    {
        PhysicsWorld w;
        RigidBodyDesc ramp;
        ramp.type = BodyType::Static;
        ramp.shape = std::make_shared<BoxShape>(Vector3(50, 0.5, 5));
        ramp.transform = Transform(Vector3(0, 0, 0), Quaternion::fromAxisAngle(Vector3::unitZ(), Real(-0.5235988)));
        ramp.friction = Real(0.8);
        w.createBody(ramp);
        const BodyId s = addBody(w, sphere, Real(1.2), Real(0.8), Vector3(), Vector3());
        w.getBody(s).setPose(Vector3(-5, Real(3.5), 0), Quaternion::identity());
        run(w, 90);
        CHECK(w.getBody(s).getLinearVelocity().length() > Real(1));
    }
    // 8) determinismo e estabilidade com tudo ligado: 40 esferas girando, nada vira NaN
    {
        PhysicsWorld w;
        addGround(w, Real(1));
        for (int i = 0; i < 40; ++i) {
            const BodyId s = addBody(w, sphere, Real(0.5) + Real(1.2) * Real(i % 5), Real(1), Vector3(), Vector3(Real(i % 3), Real(10), Real(i % 4)));
            w.getBody(s).setPose(Vector3(Real(i % 8) * Real(1.4) - 5, Real(0.5) + Real(1.2) * Real(i / 8), Real(i % 3) * Real(1.4)), Quaternion::identity());
        }
        run(w, 60 * 20);
        bool finite = true;
        for (BodyId id = 1; id <= 41; ++id)
            if (w.isValidBody(id) && !(w.getBody(id).getLinearVelocity().isFinite() && w.getBody(id).getAngularVelocity().isFinite())) finite = false;
        CHECK(finite);
    }
    std::printf("friction_tests: %d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
