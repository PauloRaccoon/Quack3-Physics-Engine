// =============================================================================
// broadphase_tests.cpp — testes unitários da Fase 4 (sem frameworks externos).
// Estratégia: comparar a árvore e a Broadphase com força bruta O(N²).
// =============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <set>
#include <stdexcept>
#include <vector>

#include "physics/collision/broadphase/Broadphase.h"
#include "physics/collision/broadphase/DynamicAABBTree.h"

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

// Gerador pseudo-aleatório determinístico (LCG) em [0,1).
struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    Real next() {
        s = s * 1664525u + 1013904223u;
        return Real((s >> 8) & 0xFFFFFF) / Real(16777216);
    }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * next(); }
};

static AABB randomBox(Rng& r, Real world, Real maxSize) {
    const Vector3 c(r.range(0, world), r.range(0, world), r.range(0, world));
    const Vector3 h(r.range(0.1f, maxSize), r.range(0.1f, maxSize), r.range(0.1f, maxSize));
    return AABB::fromCenterHalfExtents(c, h);
}

template <typename F>
static bool throwsInvalidArgument(F&& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; } catch (...) { return false; }
    return false;
}

// -----------------------------------------------------------------------------
static void testTreeBasics() {
    DynamicAABBTree tree(Real(0.1));
    CHECK(tree.validate() && tree.getProxyCount() == 0 && tree.getHeight() == 0);

    const AABB a(Vector3(0, 0, 0), Vector3(1, 1, 1));
    const ProxyId ia = tree.createProxy(a, 42);
    CHECK(tree.isProxy(ia) && tree.getUserData(ia) == 42);
    CHECK(tree.getFatAABB(ia).contains(a));                    // engordado
    CHECK(nearlyEqual(tree.getFatAABB(ia).min.x, Real(-0.1)));

    int hits = 0;
    tree.query(AABB(Vector3(0.5f, 0.5f, 0.5f), Vector3(2, 2, 2)), [&](ProxyId) { ++hits; return true; });
    CHECK(hits == 1);
    hits = 0;
    tree.query(AABB(Vector3(5, 5, 5), Vector3(6, 6, 6)), [&](ProxyId) { ++hits; return true; });
    CHECK(hits == 0);

    tree.destroyProxy(ia);
    CHECK(!tree.isProxy(ia) && tree.getProxyCount() == 0 && tree.validate());
    tree.destroyProxy(ia);          // duplo destroy é inofensivo
    tree.destroyProxy(-1);
    tree.destroyProxy(100000);
    CHECK(tree.validate());

    // Entradas inválidas.
    CHECK(throwsInvalidArgument([&] { tree.createProxy(AABB(), 0); }));   // vazio
    CHECK(throwsInvalidArgument([&] {
        tree.createProxy(AABB(Vector3(std::numeric_limits<Real>::quiet_NaN(), 0, 0), Vector3(1, 1, 1)), 0);
    }));
    CHECK(tree.getProxyCount() == 0 && tree.validate());

    // Parada antecipada do callback.
    for (int i = 0; i < 10; ++i) tree.createProxy(AABB(Vector3(0, 0, 0), Vector3(1, 1, 1)), static_cast<std::uint32_t>(i));
    int visited = 0;
    tree.query(AABB(Vector3(0, 0, 0), Vector3(1, 1, 1)), [&](ProxyId) { ++visited; return visited < 3; });
    CHECK(visited == 3);
}

static std::set<ProxyId> bruteQuery(const std::vector<ProxyId>& ids, const DynamicAABBTree& tree, const AABB& q) {
    std::set<ProxyId> r;
    for (ProxyId id : ids)
        if (tree.getFatAABB(id).overlaps(q)) r.insert(id);
    return r;
}

static std::set<ProxyId> treeQuery(const DynamicAABBTree& tree, const AABB& q) {
    std::set<ProxyId> r;
    tree.query(q, [&](ProxyId id) { r.insert(id); return true; });
    return r;
}

static void testTreeVsBruteForce() {
    Rng rng(12345);
    DynamicAABBTree tree(Real(0.1), Real(2), 8);   // capacidade pequena: força crescimento
    std::vector<ProxyId> ids;

    for (int i = 0; i < 500; ++i) ids.push_back(tree.createProxy(randomBox(rng, 50, 1.5f), static_cast<std::uint32_t>(i)));
    std::string err;
    CHECK(tree.validate(&err));
    if (!err.empty()) std::printf("  validate: %s\n", err.c_str());
    CHECK(tree.getProxyCount() == 500);

    auto compareQueries = [&](int n) {
        bool ok = true;
        for (int i = 0; i < n; ++i) {
            const AABB q = randomBox(rng, 50, 6);
            ok = ok && (treeQuery(tree, q) == bruteQuery(ids, tree, q));
        }
        return ok;
    };
    CHECK(compareQueries(200));

    // Remove ~metade, em ordem aleatória.
    for (int i = 0; i < 250; ++i) {
        const std::size_t k = static_cast<std::size_t>(rng.next() * Real(ids.size())) % ids.size();
        tree.destroyProxy(ids[k]);
        ids.erase(ids.begin() + static_cast<std::ptrdiff_t>(k));
    }
    CHECK(tree.validate(&err));
    CHECK(tree.getProxyCount() == 250);
    CHECK(compareQueries(200));

    // Reinsere: nós liberados devem ser reutilizados (sem vazamento).
    for (int i = 0; i < 250; ++i) ids.push_back(tree.createProxy(randomBox(rng, 50, 1.5f), 1000u + static_cast<unsigned>(i)));
    CHECK(tree.validate(&err));
    CHECK(compareQueries(200));

    // Movimentos aleatórios (pequenos e grandes).
    int changed = 0, kept = 0;
    for (int step = 0; step < 2000; ++step) {
        const ProxyId id = ids[static_cast<std::size_t>(rng.next() * Real(ids.size())) % ids.size()];
        const Real big = (step % 10 == 0) ? Real(20) : Real(0.05);
        const Vector3 d(rng.range(-big, big), rng.range(-big, big), rng.range(-big, big));
        AABB tight = tree.getFatAABB(id).fattened(Real(-0.1));   // aproxima o AABB justo atual
        tight = AABB(tight.min + d, tight.max + d);
        if (tree.moveProxy(id, tight, d)) ++changed; else ++kept;
        CHECK(tree.getFatAABB(id).contains(tight));              // invariante: o Fat AABB sempre contém o objeto
    }
    CHECK(tree.validate(&err));
    CHECK(changed > 0 && kept > 0);   // ambos os caminhos (O(1) e reinserção) exercitados
    CHECK(compareQueries(200));
    std::printf("  [info] moveProxy: %d reinsercoes, %d O(1)\n", changed, kept);
}

static void testTreeBalance() {
    // Pior caso clássico: objetos em linha, inseridos em ordem.
    DynamicAABBTree tree(Real(0.1));
    const int N = 2000;
    for (int i = 0; i < N; ++i) {
        tree.createProxy(AABB(Vector3(Real(i) * 2, 0, 0), Vector3(Real(i) * 2 + 1, 1, 1)), static_cast<std::uint32_t>(i));
    }
    std::string err;
    CHECK(tree.validate(&err));
    const int height = tree.getHeight();
    std::printf("  [info] %d proxies em linha: altura=%d (log2=%.1f), max balance=%d, razao de area=%.2f\n",
                N, height, std::log2(double(N)), tree.getMaxBalance(), double(tree.getAreaRatio()));
    CHECK(height <= 2 * static_cast<int>(std::ceil(std::log2(double(N)))));   // O(log N)
    CHECK(tree.getMaxBalance() <= 2);
}

// -----------------------------------------------------------------------------
static std::set<std::pair<ProxyId, ProxyId>> bruteForcePairs(const Broadphase& bp, const std::vector<ProxyId>& ids,
                                                             const std::function<bool(std::uint32_t, std::uint32_t)>& filter = nullptr) {
    std::set<std::pair<ProxyId, ProxyId>> r;
    for (std::size_t i = 0; i < ids.size(); ++i)
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const ProxyId a = std::min(ids[i], ids[j]), b = std::max(ids[i], ids[j]);
            if (!bp.testOverlap(a, b)) continue;
            if (filter && !filter(bp.getUserData(a), bp.getUserData(b))) continue;
            r.insert({a, b});
        }
    return r;
}

static std::set<std::pair<ProxyId, ProxyId>> pairSet(const Broadphase& bp) {
    std::set<std::pair<ProxyId, ProxyId>> r;
    for (const ProxyPair& p : bp.getPairs()) {
        CHECK(p.proxyA < p.proxyB);
        CHECK(bp.getUserData(p.proxyA) == p.userDataA && bp.getUserData(p.proxyB) == p.userDataB);
        r.insert({p.proxyA, p.proxyB});
    }
    CHECK(r.size() == bp.getPairs().size());   // sem duplicatas
    return r;
}

static void testBroadphasePairs() {
    Rng rng(777);
    Broadphase bp(Real(0.1), Real(2), 16);
    std::vector<ProxyId> ids;
    std::vector<AABB> boxes;
    for (int i = 0; i < 300; ++i) {
        boxes.push_back(randomBox(rng, 30, 1.2f));
        ids.push_back(bp.createProxy(boxes.back(), static_cast<std::uint32_t>(i)));
    }
    CHECK(bp.getMoveBufferSize() == 300);
    bp.updatePairs();
    CHECK(bp.getMoveBufferSize() == 0);
    CHECK(pairSet(bp) == bruteForcePairs(bp, ids));
    CHECK(!bp.getPairs().empty());

    // Sem movimento: nenhum trabalho, nenhum par novo.
    bp.updatePairs();
    CHECK(bp.getPairs().empty());
    for (std::size_t i = 0; i < ids.size(); ++i) CHECK(!bp.moveProxy(ids[i], boxes[i], Vector3::zero()));
    CHECK(bp.getMoveBufferSize() == 0);

    // Move um proxy para dentro de outro: só pares contendo esse proxy aparecem.
    const ProxyId mover = ids[0];
    const AABB target = bp.getFatAABB(ids[1]);
    boxes[0] = AABB::fromCenterHalfExtents(target.center(), Vector3(0.2f, 0.2f, 0.2f));
    CHECK(bp.moveProxy(mover, boxes[0], Vector3(5, 5, 5)));
    bp.updatePairs();
    const auto got = pairSet(bp);
    for (const auto& p : got) CHECK(p.first == mover || p.second == mover);
    CHECK(got.count({std::min(mover, ids[1]), std::max(mover, ids[1])}) == 1);
    // E são exatamente os pares de força bruta que envolvem `mover`.
    std::set<std::pair<ProxyId, ProxyId>> expected;
    for (const auto& p : bruteForcePairs(bp, ids)) if (p.first == mover || p.second == mover) expected.insert(p);
    CHECK(got == expected);

    // Todos se movem ao mesmo tempo: sem pares duplicados e iguais à força bruta.
    for (std::size_t i = 0; i < ids.size(); ++i) {
        const Vector3 d(rng.range(-3, 3), rng.range(-3, 3), rng.range(-3, 3));
        boxes[i] = AABB(boxes[i].min + d, boxes[i].max + d);
        bp.moveProxy(ids[i], boxes[i], d);
    }
    bp.updatePairs();
    CHECK(pairSet(bp) == bruteForcePairs(bp, ids));

    // Destruir proxies: nunca aparecem em pares, mesmo estando no move buffer.
    for (std::size_t i = 0; i < ids.size(); ++i) bp.touchProxy(ids[i]);
    std::set<ProxyId> dead;
    for (std::size_t i = 0; i < ids.size(); i += 3) { bp.destroyProxy(ids[i]); dead.insert(ids[i]); }
    std::vector<ProxyId> alive;
    for (ProxyId id : ids) if (!dead.count(id)) alive.push_back(id);
    bp.updatePairs();
    for (const ProxyPair& p : bp.getPairs()) CHECK(!dead.count(p.proxyA) && !dead.count(p.proxyB));
    CHECK(pairSet(bp) == bruteForcePairs(bp, alive));
    CHECK(bp.getTree().validate());

    // Criar após destruir (IDs reutilizados no mesmo quadro) continua correto.
    for (int i = 0; i < 40; ++i) alive.push_back(bp.createProxy(randomBox(rng, 30, 1.2f), 5000u + static_cast<unsigned>(i)));
    for (ProxyId id : alive) bp.touchProxy(id);
    bp.updatePairs();
    CHECK(pairSet(bp) == bruteForcePairs(bp, alive));
}

static void testBroadphaseFilter() {
    Broadphase bp(Real(0.1));
    // Filtro: userData par = "estático"; dois estáticos nunca formam par.
    bp.setPairFilter([](std::uint32_t a, std::uint32_t b) { return !((a % 2 == 0) && (b % 2 == 0)); });
    std::vector<ProxyId> ids;
    for (int i = 0; i < 8; ++i) ids.push_back(bp.createProxy(AABB(Vector3(0, 0, 0), Vector3(1, 1, 1)), static_cast<std::uint32_t>(i)));
    bp.updatePairs();
    auto filter = [](std::uint32_t a, std::uint32_t b) { return !((a % 2 == 0) && (b % 2 == 0)); };
    CHECK(pairSet(bp) == bruteForcePairs(bp, ids, filter));
    CHECK(bp.getPairs().size() == 28 - 6);   // C(8,2) - C(4,2) pares estático-estático

    // Consulta de região via Broadphase.
    int n = 0;
    bp.query(AABB(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.6f, 0.6f, 0.6f)), [&](ProxyId, std::uint32_t) { ++n; return true; });
    CHECK(n == 8);
}

static void testPerformanceInfo() {
    // Apenas informativo (não afirma tempo): 10.000 corpos, ~10% se movendo por quadro.
    Rng rng(2024);
    Broadphase bp(Real(0.1), Real(2), 1 << 14);
    const int N = 10000;
    std::vector<ProxyId> ids;
    std::vector<AABB> boxes;
    for (int i = 0; i < N; ++i) {
        boxes.push_back(randomBox(rng, 200, 0.6f));
        ids.push_back(bp.createProxy(boxes.back(), static_cast<std::uint32_t>(i)));
    }
    auto t0 = std::chrono::steady_clock::now();
    bp.updatePairs();
    auto t1 = std::chrono::steady_clock::now();
    const std::size_t initialPairs = bp.getPairs().size();

    double frameMs = 0;
    const int frames = 30;
    for (int f = 0; f < frames; ++f) {
        for (int i = 0; i < N; i += 10) {
            const Vector3 d(rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1));
            boxes[static_cast<std::size_t>(i)] = AABB(boxes[static_cast<std::size_t>(i)].min + d, boxes[static_cast<std::size_t>(i)].max + d);
            bp.moveProxy(ids[static_cast<std::size_t>(i)], boxes[static_cast<std::size_t>(i)], d);
        }
        auto a = std::chrono::steady_clock::now();
        bp.updatePairs();
        auto b = std::chrono::steady_clock::now();
        frameMs += std::chrono::duration<double, std::milli>(b - a).count();
    }
    std::printf("  [info] %d proxies: updatePairs inicial=%.2f ms (%zu pares); quadro com ~10%% movendo=%.3f ms\n",
                N, std::chrono::duration<double, std::milli>(t1 - t0).count(), initialPairs, frameMs / frames);
    CHECK(bp.getTree().validate());
}

int main() {
    testTreeBasics();
    testTreeVsBruteForce();
    testTreeBalance();
    testBroadphasePairs();
    testBroadphaseFilter();
    testPerformanceInfo();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
