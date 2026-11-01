// =============================================================================
// viewer_smoke.cpp
// Teste de FUMAÇA do viewer, sem janela e sem raylib (roda no ctest, inclusive sob ASan):
//   1) cada cena roda 20 s de tempo de render pelo mesmo caminho da janela (TimeControl + ContactTracker),
//      com poses (nos 3 modos de interpolação) finitas e quatérnios unitários o tempo todo;
//   2) "fuzz" determinístico: centenas de operações aleatórias (criar, lançar, apagar, editar parâmetros,
//      mudar o mundo, acordar/dormir, trocar de cena) intercaladas com passos, sem NaN e sem quebrar.
// Retorna 0 se tudo passou.
// =============================================================================
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "physics/geometry/PolyhedronShape.h"

#include "viewer/core/BodyEditor.h"
#include "viewer/core/ContactTracker.h"
#include "viewer/core/ConvexHull.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/JointEditor.h"
#include "viewer/core/ModelAsset.h"
#include "viewer/core/ModelCollider.h"
#include "viewer/core/ModelImporter.h"
#include "viewer/core/ObjModel.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/Scene.h"
#include "viewer/core/SceneFile.h"
#include "viewer/core/TimeControl.h"
#include "viewer/core/TrajectoryRecorder.h"
#include "viewer/core/WorldEditor.h"

using namespace viewer;
using physics::Real;
using physics::Vector3;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                                \
    do {                                                                           \
        ++g_checks;                                                                \
        if (!(cond)) {                                                             \
            ++g_failures;                                                          \
            std::printf("FALHOU %s:%d  %s\n", __FILE__, __LINE__, #cond);          \
        }                                                                          \
    } while (0)

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * next(); }
    int pick(int n) { return static_cast<int>(next() * Real(n)) % n; }
};

/// Todos os corpos da cena: pose finita, quatérnio unitário, e as poses interpoladas (3 modos) finitas.
static bool sceneHealthy(const Scene& sc, Real alpha) {
    for (const SceneBody& b : sc.bodies) {
        if (!sc.world->isValidBody(b.id)) return false;
        const physics::RigidBody& rb = sc.world->getBody(b.id);
        if (!rb.getTransform().position.isFinite() || !rb.getOrientation().isFinite()) return false;
        if (std::fabs(rb.getOrientation().norm() - Real(1)) > Real(1e-2)) return false;
        if (!rb.getLinearVelocity().isFinite() || !rb.getAngularVelocity().isFinite()) return false;
        const physics::MotionState& ms = sc.world->getMotionState(b.id);
        for (int m = 0; m < kInterpModeCount; ++m) {
            const physics::Transform t = interpolatePose(ms.getPrevious(), ms.getCurrent(), alpha, static_cast<InterpMode>(m));
            if (!t.position.isFinite() || !t.rotation.isFinite()) return false;
        }
    }
    return true;
}

static void smokeAllScenes() {
    const std::size_t n = sceneList().size();
    CHECK(n >= 9);
    for (std::size_t i = 0; i < n; ++i) {
        Scene sc = makeScene(i);
        ContactTracker tracker;
        sc.world->setContactListener(&tracker);
        TimeControl tc;
        bool healthy = true;
        for (int f = 0; f < 20 * 60; ++f) {   // 20 s a 60 quadros/s
            tc.advance(*sc.world, Real(1) / Real(60));
            tracker.update(1.0f / 60.0f);
            if (f % 30 == 0) healthy = healthy && sceneHealthy(sc, tc.renderAlpha(*sc.world));
        }
        CHECK(healthy);
        CHECK(sceneHealthy(sc, tc.renderAlpha(*sc.world)));
        // Pausa e passo a passo: avança exatamente 1 passo.
        tc.setPaused(true);
        const auto before = sc.world->getStats().stepCount;
        tc.advance(*sc.world, Real(1) / Real(60));
        CHECK(sc.world->getStats().stepCount == before);
        tc.requestStep();
        tc.advance(*sc.world, Real(1) / Real(60));
        CHECK(sc.world->getStats().stepCount == before + 1);
        sc.world->setContactListener(nullptr);
    }
}

#ifdef VIEWER_MODELS_DIR
static const char* const kModelDir = VIEWER_MODELS_DIR;
#else
static const char* const kModelDir = "";
#endif

/// V15b: alguns assets (cubo, esfera, toro em tamanhos diferentes) para o fuzz espalhar pela cena.
static const std::vector<std::shared_ptr<const ModelAsset>>& modelPool() {
    static std::vector<std::shared_ptr<const ModelAsset>> pool;
    if (pool.empty() && kModelDir[0] != '\0') {
        const char* files[] = {"cubo.obj", "esfera.obj", "toro.obj"};
        const float sizes[] = {0.4f, 1.0f, 2.5f};
        for (const char* f : files)
            for (float sz : sizes)
                for (int ck = 0; ck < kColliderKindCount; ++ck) {   // V17: cada tipo de colisor (fecho, caixa, esfera)
                    auto a = std::make_shared<ModelAsset>();
                    std::string err;
                    if (loadModelAsset(kModelDir, f, sz, *a, err, nullptr, static_cast<ColliderKind>(ck))) pool.push_back(a);
                }
    }
    return pool;
}

/// Primeiro corpo dinâmico da cena (ou inválido) — alvo do gravador no fuzz.
static physics::BodyId nb0(const Scene& sc) {
    for (const SceneBody& b : sc.bodies)
        if (sc.world->isValidBody(b.id) && sc.world->getBody(b.id).isDynamic()) return b.id;
    return physics::kInvalidBodyId;
}

static void fuzz(unsigned seed) {
    Rng r(seed);
    Scene sc = makeScene(static_cast<std::size_t>(r.pick(static_cast<int>(sceneList().size()))));
    ContactTracker tracker;
    sc.world->setContactListener(&tracker);
    TimeControl tc;
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    bool healthy = true;
    TrajectoryRecorder rec;   // V16: gravador junto do fuzz (corpos criados/apagados, cena reiniciada, valores absurdos)
    for (int op = 0; op < 400; ++op) {
        if (op % 40 == 0) rec.start(sc, r.pick(2) ? RecordTarget::AllDynamic : RecordTarget::Selected, nb0(sc));
        const int k = r.pick(18);
        const std::size_t nb = sc.bodies.size();
        const physics::BodyId any = nb ? sc.bodies[static_cast<std::size_t>(r.pick(static_cast<int>(nb)))].id : physics::kInvalidBodyId;
        switch (k) {
            case 0: case 1: {   // criar, às vezes com valores absurdos
                SpawnParams p;
                p.kind = static_cast<SpawnKind>(r.pick(kSpawnKindCount));
                p.size = r.range(Real(-1), Real(10));
                p.mass = r.range(Real(-1), Real(500));
                p.position = Vector3(r.range(-15, 15), r.range(-3, 20), r.range(-15, 15));
                p.orientation = physics::Quaternion(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
                p.velocity = Vector3(r.range(-100, 100), r.range(-100, 100), r.range(-100, 100));
                p.angularVelocity = Vector3(r.range(-50, 50), r.range(-50, 50), r.range(-50, 50));
                if (r.pick(10) == 0) p.mass = nan;
                if (r.pick(5) == 0 && !modelPool().empty()) {   // V15b: modelo importado (às vezes estático)
                    p.model = modelPool()[static_cast<std::size_t>(r.pick(static_cast<int>(modelPool().size())))];
                    p.isStatic = r.pick(4) == 0;
                }
                const physics::BodyId id = spawnBody(sc, p);
                CHECK(id == physics::kInvalidBodyId || sc.world->isValidBody(id));
                break;
            }
            case 2:
                launchFrom(sc, SpawnParams(), Vector3(r.range(-5, 5), 3, 12), Vector3(r.range(-1, 1), r.range(-0.3f, 0.3f), -1), r.range(0, 80));
                break;
            case 3: launchBody(sc, r.pick(2) ? ShapeKind::Sphere : ShapeKind::Box, Vector3(0, 4, 10), Vector3(0, 0, -1), Real(18)); break;
            case 4: if (r.pick(3) == 0) deleteBody(sc, any); break;
            case 5: setMass(*sc.world, any, r.range(Real(-1), Real(300))); break;
            case 6: setFriction(*sc.world, any, r.range(Real(-1), Real(3))); setRestitution(*sc.world, any, r.range(Real(-1), Real(2))); break;
            case 7: setPosition(*sc.world, any, Vector3(r.range(-12, 12), r.range(-2, 14), r.range(-12, 12))); break;
            case 8: setOrientationAxisAngle(*sc.world, any, Vector3(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)), r.range(0, 7)); break;
            case 9: setLinearVelocity(*sc.world, any, Vector3(r.range(-30, 30), r.range(-30, 30), r.range(-30, 30))); break;
            case 10: setAwake(*sc.world, any, r.pick(2) == 0); break;
            case 11: {   // mudar o mundo: gravidade, passo, iterações
                WorldParams wp = captureWorld(*sc.world);
                wp.gravity = Vector3(r.range(-30, 30), r.range(-30, 30), r.range(-30, 30));
                wp.fixedDt = Real(1) / r.range(Real(5), Real(300));
                wp.velocityIterations = 1 + r.pick(60);
                wp.positionIterations = r.pick(25);
                applyWorld(*sc.world, wp);
                break;
            }
            case 12:
                if (r.pick(3) == 0) {   // salvar e recarregar a cena no meio da bagunça (corpos criados, editados, lançados)
                    Scene loaded;
                    std::string err;
                    const bool ok = parseScene(serializeScene(sc), loaded, err, kModelDir);
                    CHECK(ok);
                    if (!ok) std::printf("  recarga falhou: %s\n", err.c_str());
                    if (ok) {
                        tracker.reset();
                        sc = std::move(loaded);
                        sc.world->setContactListener(&tracker);
                        tc.onSceneReset();
                    }
                    break;
                }
                tc.setPaused(r.pick(2) == 0); tc.requestStep(); break;
            case 14: {   // V14: cenário estático (caixas de qualquer tamanho, também valores absurdos)
                SpawnParams p;
                p.kind = r.pick(3) == 0 ? static_cast<SpawnKind>(r.pick(kSpawnKindCount)) : SpawnKind::Box;
                p.isStatic = true;
                p.size = r.range(Real(-1), Real(6));
                p.boxScale = Vector3(r.range(Real(-1), Real(30)), r.range(Real(-1), Real(30)), r.range(Real(-1), Real(30)));
                p.position = Vector3(r.range(-15, 15), r.range(-1, 8), r.range(-15, 15));
                p.orientation = physics::Quaternion(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
                if (r.pick(10) == 0) p.boxScale.y = nan;
                const physics::BodyId id = spawnBody(sc, p);
                CHECK(id == physics::kInvalidBodyId || (sc.world->isValidBody(id) && sc.world->getBody(id).isStatic()));
                break;
            }
            case 15: {   // V14: junta aleatória (corpo-corpo ou corpo-mundo), às vezes inválida
                JointParams jp;
                jp.kind = static_cast<JointKind>(r.pick(kJointKindCount));
                jp.a = any;
                jp.b = r.pick(3) == 0 ? physics::kInvalidBodyId
                                      : (nb ? sc.bodies[static_cast<std::size_t>(r.pick(static_cast<int>(nb)))].id : physics::kInvalidBodyId);
                jp.anchor = Vector3(r.range(-10, 10), r.range(0, 10), r.range(-10, 10));
                jp.axis = Vector3(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
                jp.collideConnected = r.pick(2) == 0;
                if (r.pick(12) == 0) jp.anchor.x = nan;
                const int before = jointCount(sc);
                const bool ok = createJoint(sc, jp);
                CHECK(ok ? jointCount(sc) == before + 1 : jointCount(sc) == before);
                break;
            }
            case 16: if (r.pick(2) == 0) destroyLastJoint(sc); break;
            case 17: if (r.pick(4) == 0) deleteBody(sc, any, true); break;   // também apaga estáticos (chão, pivôs)
            default: {   // trocar de cena (os ids antigos deixam de valer)
                tracker.reset();
                sc = r.pick(4) == 0 ? makeEmptyScene(r.pick(2) == 0)
                                    : makeScene(static_cast<std::size_t>(r.pick(static_cast<int>(sceneList().size()))));
                sc.world->setContactListener(&tracker);
                tc.onSceneReset();
                break;
            }
        }
        for (int f = 0; f < 6; ++f) tc.advance(*sc.world, Real(1) / Real(60));
        tracker.update(0.1f);
        rec.capture(sc);
        healthy = healthy && sceneHealthy(sc, tc.renderAlpha(*sc.world));
    }
    CHECK(healthy);
    {   // o CSV de tudo que foi gravado é sempre bem formado e sem nan/inf
        const std::string csv = rec.toCsv();
        bool wf = csv.find("nan") == std::string::npos && csv.find("inf") == std::string::npos && !csv.empty() && csv.back() == '\n';
        std::size_t lines = 0;
        for (char c : csv) lines += c == '\n';
        CHECK(wf && lines == rec.rowCount() + 1);
    }
    sc.world->setContactListener(nullptr);
}

/// V17: fuzz do importador: arquivos .obj de lixo (linhas cortadas, índices ruins, nomes estranhos, mtl/textura
/// inseguros). Regra: falhou -> NADA fica em models/; deu certo -> o arquivo existe, está na lista e carrega.
static void fuzzImporter(unsigned seed) {
    namespace fs = std::filesystem;
    Rng r(seed);
    const fs::path root = fs::temp_directory_path() / ("viewer_import_fuzz_" + std::to_string(seed));
    fs::remove_all(root);
    fs::create_directories(root / "src");
    const std::string models = (root / "models").string();
    int ok = 0, bad = 0;
    for (int i = 0; i < 60; ++i) {
        std::string txt;
        const int kind = r.pick(6);
        if (r.pick(2)) txt += "mtllib m" + std::to_string(r.pick(3)) + ".mtl\n";
        if (r.pick(3) == 0) txt += "mtllib ../x.mtl\nmtllib /etc/hosts\n";
        if (r.pick(2)) txt += "usemtl a\n";
        const int nv = kind == 0 ? 0 : 4 + r.pick(8);
        for (int v = 0; v < nv; ++v)
            txt += "v " + std::to_string(r.range(-3.0f, 3.0f)) + " " + std::to_string(r.range(-3.0f, 3.0f)) + " " + std::to_string(r.range(-3.0f, 3.0f)) + "\n";
        const int nf = kind == 1 ? 0 : 2 + r.pick(8);
        for (int f = 0; f < nf; ++f) {
            int a = 1 + r.pick(nv + 2), b = 1 + r.pick(nv + 2), c = 1 + r.pick(nv + 2);
            if (kind != 2 && nv > 0) { a = 1 + r.pick(nv); b = 1 + r.pick(nv); c = 1 + r.pick(nv); }
            txt += "f " + std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c) + "\n";
        }
        if (kind == 3 && !txt.empty()) txt.resize(txt.size() / 2);      // cortado no meio
        if (kind == 4) txt += "v nan 0 0\nf 1 2 3\n";
        const char* names[] = {"a.obj", "Meu Modelo.obj", "..obj", "x y z.OBJ", "\xc3\xa9.obj"};
        const fs::path f = root / "src" / names[r.pick(5)];
        { std::ofstream o(f, std::ios::binary | std::ios::trunc); o << txt; }
        if (r.pick(2)) { std::ofstream o(root / "src" / ("m" + std::to_string(r.pick(3)) + ".mtl")); o << "newmtl a\nmap_Kd t.png\n"; }
        std::size_t before = 0;
        std::error_code ec;
        if (fs::exists(models, ec)) for (const auto& e : fs::directory_iterator(models)) { (void)e; ++before; }
        ImportResult res;
        std::string err;
        const bool good = importModel(f.string(), models, res, err);
        std::size_t after = 0;
        if (fs::exists(models, ec)) for (const auto& e : fs::directory_iterator(models)) { (void)e; ++after; }
        if (good) {
            ++ok;
            CHECK(after == before + 1 && fs::exists(fs::path(models) / res.file));
            const std::vector<std::string> list = listModelFiles(models);
            CHECK(std::find(list.begin(), list.end(), res.file) != list.end());
            ModelAsset a;
            std::string e2;
            for (int ck = 0; ck < kColliderKindCount; ++ck) {   // carregar com qualquer colisor: ou funciona, ou dá mensagem, nunca quebra
                const bool l = loadModelAsset(models, res.file, 1.0f, a, e2, nullptr, static_cast<ColliderKind>(ck));
                CHECK(l ? static_cast<bool>(a.shape) : !e2.empty());
            }
        } else {
            ++bad;
            CHECK(after == before && !err.empty());   // recusado: nenhum lixo
        }
    }
    CHECK(ok > 3 && bad > 3);
    fs::remove_all(root);
}

/// V15a: fuzz do fecho convexo e do leitor .obj. Nuvens de todos os tipos (bolas, planas, em linha, com
/// duplicados, em escalas de 1e-3 a 1e3, quase degeneradas): ou falha com mensagem, ou o motor aceita o fecho.
static void fuzzHullAndObj(unsigned seed) {
    Rng r(seed);
    int okCount = 0, failCount = 0;
    for (int it = 0; it < 300; ++it) {
        const int kind = r.pick(6);
        const Real scale = std::pow(Real(10), r.range(Real(-3), Real(3)));
        const int n = 4 + r.pick(120);
        std::vector<Vector3> pts;
        for (int i = 0; i < n; ++i) {
            Vector3 v(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
            switch (kind) {
                case 0: break;                                           // bola/cubo
                case 1: v.y = 0; break;                                  // plano
                case 2: v.y = v.z = 0; break;                            // reta
                case 3: v = v.lengthSquared() > Real(1e-4) ? v.normalized() : v; break;   // esfera
                case 4: v.y *= Real(1e-6); break;                        // quase plano
                default: break;                                          // duplicados (abaixo)
            }
            Vector3 q = v * scale + Vector3(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)) * ((r.pick(4) == 0 && kind != 1 && kind != 2) ? scale * 5 : Real(0));
            if (kind == 5 && i > 0 && r.pick(3) == 0) q = pts[static_cast<std::size_t>(r.pick(i))];   // duplicado exato
            pts.push_back(q);
        }
        ConvexHull h;
        std::string err;
        if (buildConvexHull(pts, h, &err)) {
            ++okCount;
            CHECK(h.faces.size() >= 4 && h.vertices.size() >= 4);
            bool engineOk = true;
            try {
                const physics::PolyhedronShape shape(h.vertices, h.faces);
                engineOk = shape.getVolume() > 0;
            } catch (const std::exception& ex) {
                engineOk = false;
                std::printf("  nuvem kind=%d n=%d escala=%g recusada pelo motor: %s\n", kind, n, static_cast<double>(scale), ex.what());
            }
            CHECK(engineOk);
        } else {
            ++failCount;
            CHECK(!err.empty() && h.vertices.empty());
        }
    }
    CHECK(okCount > 50 && failCount > 10);
    std::printf("  fecho: %d ok, %d recusados (seed %u)\n", okCount, failCount, seed);
    // .obj com lixo: nunca trava nem devolve modelo parcial
    for (int it = 0; it < 300; ++it) {
        std::string text;
        const char* words[] = {"v", "vt", "vn", "f", "usemtl", "mtllib", "o", "1", "2", "3", "-1", "0", "1/1", "1//1", "1/2/3", "x", "nan", "1e9", "#", "\n", "\n", " ", "\r\n", "/", "-"};
        const int len = r.pick(80);
        for (int i = 0; i < len; ++i) { text += words[r.pick(static_cast<int>(sizeof(words) / sizeof(words[0])))]; text += ' '; }
        ObjModel m;
        std::string err;
        if (parseObj(text, m, err)) {
            CHECK(!m.positions.empty() && !m.triangles.empty());
            for (const ObjTriangle& t : m.triangles)
                for (int k = 0; k < 3; ++k) CHECK(t.p[k] >= 0 && static_cast<std::size_t>(t.p[k]) < m.positions.size());
        } else {
            CHECK(!err.empty() && m.positions.empty() && m.triangles.empty());
        }
    }
}

int main() {
    smokeAllScenes();
    for (unsigned seed = 1; seed <= 6; ++seed) fuzz(seed * 7919u);
    for (unsigned seed = 1; seed <= 4; ++seed) fuzzHullAndObj(seed * 104729u);
    for (unsigned seed = 1; seed <= 3; ++seed) fuzzImporter(seed * 7727u);
    std::printf("fumaca: %d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
