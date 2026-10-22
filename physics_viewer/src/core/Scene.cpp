// =============================================================================
// Scene.cpp — as cenas do viewer e o registro
// =============================================================================
#include "viewer/core/Scene.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/math/Quaternion.h"
#include "viewer/core/PolyShapes.h"

namespace viewer {

using namespace physics;

namespace {

// --- Utilidades -------------------------------------------------------------------------------------

/// Gerador pseudoaleatório determinístico (as cenas são reproduzíveis).
struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {   // [0, 1)
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * next(); }
};

struct Rgb { unsigned char r, g, b; };
constexpr Rgb kPalette[] = {{240, 150, 60}, {90, 170, 240}, {120, 210, 120}, {230, 100, 120},
                            {200, 160, 240}, {240, 220, 100}, {100, 220, 210}, {220, 130, 220}};
constexpr int kPaletteSize = static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));

Rgb paletteColor(int i) { return kPalette[((i % kPaletteSize) + kPaletteSize) % kPaletteSize]; }

/// Orientação aleatória: eixo aleatório + ângulo aleatório (quatérnio unitário).
Quaternion randomOrientation(Rng& r) {
    Vector3 axis(r.range(Real(-1), Real(1)), r.range(Real(-1), Real(1)), r.range(Real(-1), Real(1)));
    if (axis.lengthSquared() < Real(1e-4)) axis = Vector3(0, 1, 0);
    return Quaternion::fromAxisAngle(axis.normalized(), r.range(Real(0), Real(6.2831853)));
}

/// Chão: caixa estática 20 x 1 x 20 m com a face superior em y = 0.
void addGround(Scene& s) {
    const Vector3 half(10, Real(0.5), 10);
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(half);
    d.transform = Transform(Vector3(0, Real(-0.5), 0));
    d.friction = Real(0.6);

    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Box;
    b.halfExtents = toVec3f(half);
    b.isStatic = true;
    b.r = 120; b.g = 130; b.b = 140;
    s.bodies.push_back(b);
}

SceneBody& addBox(Scene& s, const Vector3& pos, const Vector3& half, Real mass, Rgb c,
                  const Quaternion& q = Quaternion()) {
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(half);
    d.mass = mass;
    d.transform = Transform(pos, q);
    d.friction = Real(0.5);
    d.angularDamping = Real(0);

    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Box;
    b.halfExtents = toVec3f(half);
    b.r = c.r; b.g = c.g; b.b = c.b;
    s.bodies.push_back(b);
    return s.bodies.back();
}

SceneBody& addSphere(Scene& s, const Vector3& pos, Real radius, Real mass, Rgb c) {
    RigidBodyDesc d;
    d.shape = std::make_shared<SphereShape>(radius);
    d.mass = mass;
    d.transform = Transform(pos);
    d.friction = Real(0.5);
    d.angularDamping = Real(0);

    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Sphere;
    b.radius = static_cast<float>(radius);
    b.r = c.r; b.g = c.g; b.b = c.b;
    s.bodies.push_back(b);
    return s.bodies.back();
}

SceneBody& addPoly(Scene& s, const std::shared_ptr<const PolyhedronShape>& shape, const Vector3& pos,
                   Real mass, Rgb c, const Quaternion& q) {
    RigidBodyDesc d;
    d.shape = shape;
    d.mass = mass;
    d.transform = Transform(pos, q);
    d.friction = Real(0.5);
    d.angularDamping = Real(0);

    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Polyhedron;
    b.poly = shape;
    b.r = c.r; b.g = c.g; b.b = c.b;
    s.bodies.push_back(b);
    return s.bodies.back();
}

/// Quatro paredes estáticas (1 m de altura útil) ao redor do chão, para que esferas não caiam da borda.
void addWalls(Scene& s) {
    const Vector3 halves[4] = {Vector3(10, 1, Real(0.25)), Vector3(10, 1, Real(0.25)),
                               Vector3(Real(0.25), 1, 10), Vector3(Real(0.25), 1, 10)};
    const Vector3 centers[4] = {Vector3(0, 1, Real(9.75)), Vector3(0, 1, Real(-9.75)),
                                Vector3(Real(9.75), 1, 0), Vector3(Real(-9.75), 1, 0)};
    for (int i = 0; i < 4; ++i) {
        RigidBodyDesc d;
        d.type = BodyType::Static;
        d.shape = std::make_shared<BoxShape>(halves[i]);
        d.transform = Transform(centers[i]);
        d.friction = Real(0.6);
        SceneBody b;
        b.id = s.world->createBody(d);
        b.kind = ShapeKind::Box;
        b.halfExtents = toVec3f(halves[i]);
        b.isStatic = true;
        b.r = 90; b.g = 100; b.b = 112;
        s.bodies.push_back(b);
    }
}

/// Ponto de ancoragem imóvel e SEM forma (só um pivô): desenhado como uma pequena esfera cinza.
BodyId addAnchor(Scene& s, const Vector3& pos) {
    RigidBodyDesc d;
    d.type = BodyType::Static;
    d.transform = Transform(pos);

    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Sphere;
    b.radius = 0.12f;
    b.isStatic = true;
    b.r = 170; b.g = 170; b.b = 170;
    s.bodies.push_back(b);
    return b.id;
}

} // namespace

// =============================================================================
// Cenas
// =============================================================================
Scene makeFallingBoxScene() {
    Scene s;
    s.name = "Caixa em queda";
    s.world = std::make_unique<PhysicsWorld>();   // gravidade padrão, Δt = 1/60 s
    addGround(s);

    // Caixa de 1 kg: começa inclinada (quatérnio de eixo-ângulo) e já girando, para que a
    // rotação, guardada como quatérnio, seja bem visível.
    const Quaternion q0 = Quaternion::fromAxisAngle(Vector3(1, 1, Real(0.3)).normalized(), Real(0.6));
    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(Real(0.5), Real(0.5), Real(0.5)));
    d.mass = Real(1);
    d.transform = Transform(Vector3(0, 5, 0), q0);
    d.angularVelocity = Vector3(Real(1.2), Real(2.0), Real(0.7));   // rad/s
    d.friction = Real(0.5);
    d.restitution = Real(0.3);
    d.angularDamping = Real(0);

    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Box;
    b.halfExtents = Vec3f{0.5f, 0.5f, 0.5f};
    b.r = 240; b.g = 150; b.b = 60;
    s.focus = b.id;
    s.bodies.push_back(b);
    return s;
}

Scene makeStackScene() {
    Scene s;
    s.name = "Pilha de caixas";
    s.world = std::make_unique<PhysicsWorld>();
    addGround(s);
    const Vector3 half(Real(0.5), Real(0.5), Real(0.5));
    for (int i = 0; i < 6; ++i) {
        SceneBody& b = addBox(s, Vector3(0, Real(0.5) + Real(i) * Real(1.001), 0), half, Real(1), paletteColor(i));
        s.focus = b.id;   // o foco termina na caixa do topo
    }
    s.cameraTarget = Vec3f{0.0f, 3.0f, 0.0f};
    s.cameraDistance = 14.0f;
    return s;
}

Scene makePyramidScene() {
    Scene s;
    s.name = "Piramide de caixas";
    s.world = std::make_unique<PhysicsWorld>();
    addGround(s);
    const Vector3 half(Real(0.5), Real(0.5), Real(0.5));
    const int rows = 5;
    int color = 0;
    for (int row = 0; row < rows; ++row) {
        const int n = rows - row;
        for (int i = 0; i < n; ++i) {
            const Real x = (Real(i) - Real(n - 1) * Real(0.5)) * Real(1.02);
            SceneBody& b = addBox(s, Vector3(x, Real(0.5) + Real(row) * Real(1.002), 0), half, Real(1),
                                  paletteColor(color++));
            s.focus = b.id;   // termina na caixa do topo
        }
    }
    s.cameraTarget = Vec3f{0.0f, 2.0f, 0.0f};
    s.cameraDistance = 13.0f;
    return s;
}

Scene makeRainScene() {
    Scene s;
    s.name = "Chuva de corpos";
    s.world = std::make_unique<PhysicsWorld>();
    addGround(s);
    addWalls(s);
    Rng r(2026);
    for (int i = 0; i < 36; ++i) {
        const Vector3 pos(r.range(Real(-3), Real(3)), Real(2) + Real(i) * Real(0.6), r.range(Real(-3), Real(3)));
        SceneBody& b = (i % 3 == 0)
            ? addSphere(s, pos, Real(0.35), Real(1), paletteColor(i))
            : addBox(s, pos, Vector3(Real(0.3), Real(0.3), Real(0.3)), Real(1), paletteColor(i), randomOrientation(r));
        if (i == 1) s.focus = b.id;   // uma caixa (as esferas giram sem que se veja)
    }
    s.cameraTarget = Vec3f{0.0f, 3.0f, 0.0f};
    s.cameraDistance = 18.0f;
    return s;
}

Scene makePendulumScene() {
    Scene s;
    s.name = "Pendulo duplo";
    s.world = std::make_unique<PhysicsWorld>();
    const BodyId anchor = addAnchor(s, Vector3(0, 6, 0));
    const BodyId p1 = addSphere(s, Vector3(Real(2.5), 6, 0), Real(0.35), Real(1), paletteColor(0)).id;
    const BodyId p2 = addSphere(s, Vector3(5, 6, 0), Real(0.35), Real(1), paletteColor(1)).id;
    s.world->createDistanceJoint(anchor, p1, Vector3(0, 6, 0), Vector3(Real(2.5), 6, 0));
    s.world->createDistanceJoint(p1, p2, Vector3(Real(2.5), 6, 0), Vector3(5, 6, 0));
    s.links.push_back({anchor, p1});
    s.links.push_back({p1, p2});
    s.focus = p2;
    s.cameraTarget = Vec3f{2.5f, 3.5f, 0.0f};
    s.cameraDistance = 15.0f;
    return s;
}

Scene makeChainScene() {
    Scene s;
    s.name = "Corrente (juntas)";
    s.world = std::make_unique<PhysicsWorld>();
    const BodyId anchor = addAnchor(s, Vector3(0, 8, 0));
    BodyId prev = anchor;
    const Vector3 half(Real(0.25), Real(0.08), Real(0.08));
    const int links = 8;
    for (int i = 0; i < links; ++i) {
        // Elo i ocupa [0,5·i, 0,5·i + 0,5] em x; a junta bola-soquete fica na ponta de cima do elo.
        const BodyId id = addBox(s, Vector3(Real(0.25) + Real(i) * Real(0.5), 8, 0), half, Real(0.5),
                                 paletteColor(i)).id;
        s.world->createBallSocketJoint(prev, id, Vector3(Real(i) * Real(0.5), 8, 0));
        s.links.push_back({prev, id});
        prev = id;
    }
    s.focus = prev;
    s.cameraTarget = Vec3f{2.0f, 5.0f, 0.0f};
    s.cameraDistance = 12.0f;
    return s;
}

Scene makeShapesScene() {
    Scene s;
    s.name = "Formas (poliedros)";
    s.world = std::make_unique<PhysicsWorld>();
    addGround(s);
    Rng r(7);
    const std::shared_ptr<const PolyhedronShape> tetra = makeTetrahedron(Real(0.45));
    const std::shared_ptr<const PolyhedronShape> octa = makeOctahedron(Real(0.6));
    const std::shared_ptr<const PolyhedronShape> prism = makeTriPrism(Real(0.55), Real(0.4));
    const std::shared_ptr<const PolyhedronShape> pyramid = makeSquarePyramid(Real(0.5), Real(1.0));
    for (int i = 0; i < 12; ++i) {
        const Vector3 pos((Real(i % 4) - Real(1.5)) * Real(1.7), Real(2) + Real(i % 5) * Real(1.0),
                          (Real(i / 4) - Real(1)) * Real(1.7));
        const Quaternion q = randomOrientation(r);
        SceneBody* b = nullptr;
        switch (i % 6) {
            case 0: b = &addPoly(s, tetra, pos, Real(1), paletteColor(i), q); break;
            case 1: b = &addPoly(s, octa, pos, Real(1), paletteColor(i), q); break;
            case 2: b = &addPoly(s, prism, pos, Real(1), paletteColor(i), q); break;
            case 3: b = &addPoly(s, pyramid, pos, Real(1), paletteColor(i), q); break;
            case 4: b = &addBox(s, pos, Vector3(Real(0.4), Real(0.4), Real(0.4)), Real(1), paletteColor(i), q); break;
            default: b = &addSphere(s, pos, Real(0.4), Real(1), paletteColor(i)); break;
        }
        if (i == 0) s.focus = b->id;   // o tetraedro: sua rotação é bem visível
    }
    s.cameraTarget = Vec3f{0.0f, 1.5f, 0.0f};
    s.cameraDistance = 13.0f;
    return s;
}

Scene makeGimbalScene() {
    // Gravidade zero e passo largo (0,5 s): a caixa gira em torno do eixo X (o eixo do "pitch" da
    // convenção YXZ) e atravessa pitch = 90°, onde yaw e roll viram o mesmo eixo (gimbal lock).
    // Com o modo "Euler" a pose pula perto dali; com slerp, nada acontece.
    Scene s;
    s.name = "Gimbal lock (passo 0,5 s)";
    PhysicsWorldSettings cfg;
    cfg.gravity = Vector3(0, 0, 0);
    cfg.fixedTimeStep = Real(0.5);
    s.world = std::make_unique<PhysicsWorld>(cfg);

    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(Real(1.4), Real(0.5), Real(0.25)));
    d.mass = Real(1);
    d.transform = Transform(Vector3(0, Real(2.5), 0));
    d.angularVelocity = Vector3(Real(1.8), 0, 0);   // rad/s, eixo X do mundo (~49° por passo)
    d.angularDamping = Real(0);
    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Box;
    b.halfExtents = Vec3f{1.4f, 0.5f, 0.25f};
    b.r = 230; b.g = 120; b.b = 90;
    s.focus = b.id;
    s.bodies.push_back(b);
    s.cameraTarget = Vec3f{0.0f, 2.5f, 0.0f};
    s.cameraDistance = 9.0f;
    return s;
}

Scene makeWideStepScene() {
    // Gravidade zero, passo de 0,35 s e |ω| ≈ 8,2 rad/s: ~110° de giro entre dois passos. A
    // diferença entre slerp, lerp sem normalizar e Euler fica enorme (e visível). Um CUBO (inércia
    // isotrópica) evita efeitos giroscópicos do passo grande: só a interpolação de render difere.
    Scene s;
    s.name = "Interpolacao (passo 0,35 s)";
    PhysicsWorldSettings cfg;
    cfg.gravity = Vector3(0, 0, 0);
    cfg.fixedTimeStep = Real(0.35);
    s.world = std::make_unique<PhysicsWorld>(cfg);

    RigidBodyDesc d;
    d.shape = std::make_shared<BoxShape>(Vector3(Real(0.9), Real(0.9), Real(0.9)));
    d.mass = Real(1);
    d.transform = Transform(Vector3(0, Real(2.5), 0));
    d.angularVelocity = Vector3(Real(4.6), Real(5.9), Real(3.3));
    d.angularDamping = Real(0);
    SceneBody b;
    b.id = s.world->createBody(d);
    b.kind = ShapeKind::Box;
    b.halfExtents = Vec3f{0.9f, 0.9f, 0.9f};
    b.r = 90; b.g = 170; b.b = 240;
    s.focus = b.id;
    s.bodies.push_back(b);
    s.cameraTarget = Vec3f{0.0f, 2.5f, 0.0f};
    s.cameraDistance = 9.0f;
    return s;
}

// =============================================================================
// Registro
// =============================================================================
const std::vector<SceneInfo>& sceneList() {
    static const std::vector<SceneInfo> list = {
        {"Caixa em queda", &makeFallingBoxScene},
        {"Pilha de caixas", &makeStackScene},
        {"Piramide de caixas", &makePyramidScene},
        {"Chuva de corpos", &makeRainScene},
        {"Pendulo duplo", &makePendulumScene},
        {"Corrente (juntas)", &makeChainScene},
        {"Formas (poliedros)", &makeShapesScene},
        {"Gimbal lock (passo 0,5 s)", &makeGimbalScene},
        {"Interpolacao (passo 0,35 s)", &makeWideStepScene},
    };
    return list;
}

Scene makeScene(std::size_t index) {
    const std::vector<SceneInfo>& list = sceneList();
    return list[std::min(index, list.size() - 1)].make();
}

} // namespace viewer
