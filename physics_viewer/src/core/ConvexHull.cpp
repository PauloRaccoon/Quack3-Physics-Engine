// =============================================================================
// ConvexHull.cpp  (V15a) — quickhull próprio; ver ConvexHull.h
// =============================================================================
#include "viewer/core/ConvexHull.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

#include "physics/common/Settings.h"

namespace viewer {

namespace {

struct D3 {
    double x = 0, y = 0, z = 0;
};
D3 operator-(const D3& a, const D3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
D3 cross(const D3& a, const D3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double dot(const D3& a, const D3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double len(const D3& a) { return std::sqrt(dot(a, a)); }

struct Face {
    std::array<int, 3> v{};      ///< vértices, anti-horário visto de fora
    std::array<int, 3> nb{};     ///< nb[i] = face vizinha pela aresta v[i] -> v[(i+1)%3]
    D3 n;                        ///< normal unitária para fora
    double d = 0;                ///< n · p = d no plano
    bool alive = true;
    std::vector<int> outside;    ///< pontos fora desta face ainda não processados
};

struct Builder {
    const std::vector<D3>& p;
    double eps;
    std::vector<Face> f;
    explicit Builder(const std::vector<D3>& pts, double e) : p(pts), eps(e) {}

    double dist(const Face& fc, int pi) const { return dot(fc.n, p[static_cast<std::size_t>(pi)]) - fc.d; }

    int addFace(int a, int b, int c) {
        Face fc;
        fc.v = {a, b, c};
        fc.nb = {-1, -1, -1};
        const D3 &pa = p[static_cast<std::size_t>(a)], &pb = p[static_cast<std::size_t>(b)], &pc = p[static_cast<std::size_t>(c)];
        D3 n = cross(pb - pa, pc - pa);
        const double l = len(n);
        if (l > 0) n = {n.x / l, n.y / l, n.z / l};
        fc.n = n;
        fc.d = dot(n, pa);
        f.push_back(std::move(fc));
        return static_cast<int>(f.size()) - 1;
    }

    /// Liga a aresta (a->b) de `fi` à aresta oposta (b->a) de `gi`.
    void link(int fi, int edge, int gi) { f[static_cast<std::size_t>(fi)].nb[static_cast<std::size_t>(edge)] = gi; }
    int edgeIndex(int fi, int a, int b) const {
        const Face& fc = f[static_cast<std::size_t>(fi)];
        for (int i = 0; i < 3; ++i)
            if (fc.v[static_cast<std::size_t>(i)] == a && fc.v[static_cast<std::size_t>((i + 1) % 3)] == b) return i;
        return -1;
    }
};

bool fail(std::string* error, const std::string& msg) {
    if (error) *error = msg;
    return false;
}

} // namespace

bool buildConvexHull(const std::vector<physics::Vector3>& points, ConvexHull& out, std::string* error, bool mergeCoplanar) {
    out = ConvexHull{};
    const std::size_t n = points.size();
    if (n < 4) return fail(error, "o fecho convexo precisa de ao menos 4 pontos");
    if (n > hulllimits::kMaxPoints) return fail(error, "pontos demais para o fecho convexo (limite " + std::to_string(hulllimits::kMaxPoints) + ")");
    std::vector<D3> p(n);
    double extent = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!points[i].isFinite()) return fail(error, "o ponto " + std::to_string(i) + " nao e finito");
        p[i] = {static_cast<double>(points[i].x), static_cast<double>(points[i].y), static_cast<double>(points[i].z)};
        extent = std::max({extent, std::fabs(p[i].x), std::fabs(p[i].y), std::fabs(p[i].z)});
    }
    if (!(extent > 0)) return fail(error, "todos os pontos estao na origem");
    // Tolerância: abaixo da do motor (10·kCompareEpsilon·extent), com folga para a fusão coplanar e o arredondamento.
    const double ce = static_cast<double>(physics::kCompareEpsilon);
    const double eps = ce * extent;
    const double mergeEps = 2.0 * ce * extent;

    // ---- 1) tetraedro inicial a partir de pontos extremos ----
    auto pickExtreme = [&](int axis, bool maxSide) {
        std::size_t best = 0;
        for (std::size_t i = 1; i < n; ++i) {
            const double a = axis == 0 ? p[i].x : axis == 1 ? p[i].y : p[i].z;
            const double b = axis == 0 ? p[best].x : axis == 1 ? p[best].y : p[best].z;
            if (maxSide ? a > b : a < b) best = i;
        }
        return static_cast<int>(best);
    };
    int ext[6] = {pickExtreme(0, false), pickExtreme(0, true), pickExtreme(1, false),
                  pickExtreme(1, true),  pickExtreme(2, false), pickExtreme(2, true)};
    int i0 = ext[0], i1 = ext[1];
    double bestD = -1;
    for (int a : ext)
        for (int b : ext) {
            const double d = len(p[static_cast<std::size_t>(a)] - p[static_cast<std::size_t>(b)]);
            if (d > bestD) { bestD = d; i0 = a; i1 = b; }
        }
    if (!(bestD > 10 * eps)) return fail(error, "os pontos sao (quase) coincidentes");
    const D3 e01 = p[static_cast<std::size_t>(i1)] - p[static_cast<std::size_t>(i0)];
    int i2 = -1;
    bestD = -1;
    for (std::size_t i = 0; i < n; ++i) {
        const double d = len(cross(e01, p[i] - p[static_cast<std::size_t>(i0)])) / len(e01);   // distância à reta
        if (d > bestD) { bestD = d; i2 = static_cast<int>(i); }
    }
    if (!(bestD > 10 * eps)) return fail(error, "os pontos sao colineares (sem fecho em 3D)");
    D3 pn = cross(e01, p[static_cast<std::size_t>(i2)] - p[static_cast<std::size_t>(i0)]);
    pn = {pn.x / len(pn), pn.y / len(pn), pn.z / len(pn)};
    int i3 = -1;
    bestD = -1;
    for (std::size_t i = 0; i < n; ++i) {
        const double d = std::fabs(dot(pn, p[i] - p[static_cast<std::size_t>(i0)]));
        if (d > bestD) { bestD = d; i3 = static_cast<int>(i); }
    }
    if (!(bestD > 10 * eps)) return fail(error, "os pontos sao coplanares (volume nulo): o fecho nao tem volume");

    Builder b(p, eps);
    // Orientação: o 4º ponto deve ficar ATRÁS da face (i0, i1, i2).
    if (dot(pn, p[static_cast<std::size_t>(i3)] - p[static_cast<std::size_t>(i0)]) > 0) std::swap(i1, i2);
    // faces (CCW visto de fora): (0,1,2) (0,3,1) (1,3,2) (2,3,0)
    const int t0 = b.addFace(i0, i1, i2), t1 = b.addFace(i0, i3, i1), t2 = b.addFace(i1, i3, i2), t3 = b.addFace(i2, i3, i0);
    const int tf[4] = {t0, t1, t2, t3};
    for (int a = 0; a < 4; ++a)
        for (int e = 0; e < 3; ++e) {
            const Face& fa = b.f[static_cast<std::size_t>(tf[a])];
            const int va = fa.v[static_cast<std::size_t>(e)], vb = fa.v[static_cast<std::size_t>((e + 1) % 3)];
            for (int c = 0; c < 4; ++c)
                if (c != a && b.edgeIndex(tf[c], vb, va) >= 0) b.link(tf[a], e, tf[c]);
        }
    // pontos -> conjunto externo da primeira face que os enxerga
    for (std::size_t i = 0; i < n; ++i) {
        const int pi = static_cast<int>(i);
        if (pi == i0 || pi == i1 || pi == i2 || pi == i3) continue;
        for (int a = 0; a < 4; ++a)
            if (b.dist(b.f[static_cast<std::size_t>(tf[a])], pi) > eps) { b.f[static_cast<std::size_t>(tf[a])].outside.push_back(pi); break; }
    }

    // ---- 2) laço principal ----
    const std::size_t maxFaces = 8 * n + 64;
    std::vector<int> stack;
    for (int a = 0; a < 4; ++a) if (!b.f[static_cast<std::size_t>(tf[a])].outside.empty()) stack.push_back(tf[a]);
    std::vector<int> visible, newFaces;
    std::vector<char> isVisible;
    while (!stack.empty()) {
        const int fi = stack.back();
        stack.pop_back();
        if (!b.f[static_cast<std::size_t>(fi)].alive || b.f[static_cast<std::size_t>(fi)].outside.empty()) continue;
        if (b.f.size() > maxFaces) return fail(error, "o fecho nao convergiu (falha numerica)");
        // olho = ponto mais distante desta face
        int eye = -1;
        double far = -1;
        for (int pi : b.f[static_cast<std::size_t>(fi)].outside) {
            const double d = b.dist(b.f[static_cast<std::size_t>(fi)], pi);
            if (d > far) { far = d; eye = pi; }
        }
        // região visível (busca em profundidade pelos vizinhos)
        isVisible.assign(b.f.size(), 0);
        visible.clear();
        std::vector<int> dfs{fi};
        isVisible[static_cast<std::size_t>(fi)] = 1;
        while (!dfs.empty()) {
            const int c = dfs.back();
            dfs.pop_back();
            visible.push_back(c);
            for (int e = 0; e < 3; ++e) {
                const int g = b.f[static_cast<std::size_t>(c)].nb[static_cast<std::size_t>(e)];
                if (g < 0 || isVisible[static_cast<std::size_t>(g)] || !b.f[static_cast<std::size_t>(g)].alive) continue;
                if (b.dist(b.f[static_cast<std::size_t>(g)], eye) > eps) {
                    isVisible[static_cast<std::size_t>(g)] = 1;
                    dfs.push_back(g);
                }
            }
        }
        // horizonte: arestas de faces visíveis cujo vizinho não é visível
        struct Horizon { int a, b, outer; };
        std::vector<Horizon> horizon;
        for (int c : visible)
            for (int e = 0; e < 3; ++e) {
                const int g = b.f[static_cast<std::size_t>(c)].nb[static_cast<std::size_t>(e)];
                if (g >= 0 && isVisible[static_cast<std::size_t>(g)]) continue;
                horizon.push_back({b.f[static_cast<std::size_t>(c)].v[static_cast<std::size_t>(e)],
                                   b.f[static_cast<std::size_t>(c)].v[static_cast<std::size_t>((e + 1) % 3)], g});
            }
        if (horizon.size() < 3) return fail(error, "horizonte invalido no fecho (falha numerica)");
        // faces novas (a, b, eye)
        newFaces.clear();
        std::map<int, int> byStart, byEnd;
        for (const Horizon& h : horizon) {
            const int nf = b.addFace(h.a, h.b, eye);
            newFaces.push_back(nf);
            if (!byStart.emplace(h.a, nf).second || !byEnd.emplace(h.b, nf).second)
                return fail(error, "horizonte nao e um laco simples (falha numerica)");
            if (h.outer >= 0) {   // liga à face de fora, nos dois sentidos
                b.link(nf, 0, h.outer);
                const int back = b.edgeIndex(h.outer, h.b, h.a);
                if (back < 0) return fail(error, "adjacencia inconsistente (falha numerica)");
                b.link(h.outer, back, nf);
            }
        }
        for (const Horizon& h : horizon) {
            const int nf = byStart[h.a];
            const auto nextIt = byStart.find(h.b);   // face nova cuja aresta começa onde esta termina
            const auto prevIt = byEnd.find(h.a);
            if (nextIt == byStart.end() || prevIt == byEnd.end()) return fail(error, "horizonte aberto (falha numerica)");
            b.link(nf, 1, nextIt->second);   // aresta b -> eye
            b.link(nf, 2, prevIt->second);   // aresta eye -> a
        }
        // redistribui os pontos das faces visíveis
        for (int c : visible) {
            for (int pi : b.f[static_cast<std::size_t>(c)].outside) {
                if (pi == eye) continue;
                for (int nf : newFaces)
                    if (b.dist(b.f[static_cast<std::size_t>(nf)], pi) > eps) { b.f[static_cast<std::size_t>(nf)].outside.push_back(pi); break; }
            }
            b.f[static_cast<std::size_t>(c)].outside.clear();
            b.f[static_cast<std::size_t>(c)].outside.shrink_to_fit();
            b.f[static_cast<std::size_t>(c)].alive = false;
        }
        for (int nf : newFaces) if (!b.f[static_cast<std::size_t>(nf)].outside.empty()) stack.push_back(nf);
    }

    // ---- 3) triângulos vivos -> vértices do fecho ----
    std::vector<int> alive;
    for (std::size_t i = 0; i < b.f.size(); ++i) if (b.f[i].alive) alive.push_back(static_cast<int>(i));
    if (alive.size() < 4) return fail(error, "fecho degenerado");
    std::vector<int> remap(n, -1);
    for (int fi : alive)
        for (int v : b.f[static_cast<std::size_t>(fi)].v)
            if (remap[static_cast<std::size_t>(v)] < 0) {
                remap[static_cast<std::size_t>(v)] = static_cast<int>(out.vertices.size());
                out.vertices.push_back(points[static_cast<std::size_t>(v)]);
                out.sourceIndex.push_back(v);
            }
    out.triangleCount = static_cast<int>(alive.size());

    auto triangles = [&]() {
        out.faces.clear();
        for (int fi : alive) {
            const Face& fc = b.f[static_cast<std::size_t>(fi)];
            out.faces.push_back({remap[static_cast<std::size_t>(fc.v[0])], remap[static_cast<std::size_t>(fc.v[1])], remap[static_cast<std::size_t>(fc.v[2])]});
        }
    };

    // Confere o resultado como o motor vai conferir (com os valores já convertidos para Real): faces com normal
    // válida, todos os vértices atrás de todos os planos e cada face planar, com folga em relação à tolerância
    // do motor (10·kCompareEpsilon·extent).
    auto verify = [&]() {
        const double tol = 5.0 * ce * extent;
        for (const std::vector<int>& face : out.faces) {
            D3 nrm, c;
            physics::Vector3 nr;   // a mesma conta do motor, em Real: precisa dar comprimento > 0
            for (std::size_t i = 0; i < face.size(); ++i) {
                const physics::Vector3& a = out.vertices[static_cast<std::size_t>(face[i])];
                const physics::Vector3& q = out.vertices[static_cast<std::size_t>(face[(i + 1) % face.size()])];
                nr.x += (a.y - q.y) * (a.z + q.z);
                nr.y += (a.z - q.z) * (a.x + q.x);
                nr.z += (a.x - q.x) * (a.y + q.y);
                nrm.x += (static_cast<double>(a.y) - q.y) * (static_cast<double>(a.z) + q.z);
                nrm.y += (static_cast<double>(a.z) - q.z) * (static_cast<double>(a.x) + q.x);
                nrm.z += (static_cast<double>(a.x) - q.x) * (static_cast<double>(a.y) + q.y);
                c.x += a.x; c.y += a.y; c.z += a.z;
            }
            const double l = len(nrm);
            if (!(l > 0) || !(nr.length() > physics::Real(0)) || !nr.isFinite()) return false;
            nrm = {nrm.x / l, nrm.y / l, nrm.z / l};
            c = {c.x / static_cast<double>(face.size()), c.y / static_cast<double>(face.size()), c.z / static_cast<double>(face.size())};
            const double off = dot(nrm, c);
            for (const physics::Vector3& v : out.vertices)
                if (dot(nrm, {static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)}) - off > tol) return false;
            for (int vi : face) {
                const physics::Vector3& v = out.vertices[static_cast<std::size_t>(vi)];
                if (std::fabs(dot(nrm, {static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)}) - off) > tol) return false;
            }
        }
        return true;
    };
    // Remove vértices que nenhuma face usa mais (um ponto no meio de uma face fundida) e reindexa.
    auto dropUnused = [&]() {
        std::vector<int> map(out.vertices.size(), -1);
        std::vector<physics::Vector3> nv;
        std::vector<int> ns;
        for (const std::vector<int>& face : out.faces)
            for (int vi : face)
                if (map[static_cast<std::size_t>(vi)] < 0) {
                    map[static_cast<std::size_t>(vi)] = static_cast<int>(nv.size());
                    nv.push_back(out.vertices[static_cast<std::size_t>(vi)]);
                    ns.push_back(out.sourceIndex[static_cast<std::size_t>(vi)]);
                }
        for (std::vector<int>& face : out.faces)
            for (int& vi : face) vi = map[static_cast<std::size_t>(vi)];
        out.vertices = std::move(nv);
        out.sourceIndex = std::move(ns);
    };
    auto giveUp = [&]() {
        out = ConvexHull{};
        return fail(error, "o fecho ficou numericamente instavel (modelo muito fino ou de proporcoes extremas)");
    };
    if (!mergeCoplanar) {
        triangles();
        return verify() ? true : giveUp();
    }

    // ---- 4) fusão de triângulos coplanares vizinhos em polígonos ----
    std::vector<int> group(b.f.size(), -1);
    int groupCount = 0;
    std::vector<std::vector<int>> members;
    for (int seed : alive) {
        if (group[static_cast<std::size_t>(seed)] >= 0) continue;
        const Face& sf = b.f[static_cast<std::size_t>(seed)];
        std::vector<int> mem{seed}, queue{seed};
        group[static_cast<std::size_t>(seed)] = groupCount;
        while (!queue.empty()) {
            const int c = queue.back();
            queue.pop_back();
            for (int e = 0; e < 3; ++e) {
                const int g = b.f[static_cast<std::size_t>(c)].nb[static_cast<std::size_t>(e)];
                if (g < 0 || group[static_cast<std::size_t>(g)] >= 0) continue;
                const Face& gf = b.f[static_cast<std::size_t>(g)];
                bool coplanar = dot(gf.n, sf.n) > 0.999;
                for (int k = 0; k < 3 && coplanar; ++k)
                    if (std::fabs(dot(sf.n, p[static_cast<std::size_t>(gf.v[static_cast<std::size_t>(k)])]) - sf.d) > mergeEps) coplanar = false;
                if (!coplanar) continue;
                group[static_cast<std::size_t>(g)] = groupCount;
                mem.push_back(g);
                queue.push_back(g);
            }
        }
        members.push_back(std::move(mem));
        ++groupCount;
    }
    out.faces.clear();
    for (int gi = 0; gi < groupCount; ++gi) {
        const std::vector<int>& mem = members[static_cast<std::size_t>(gi)];
        auto emitTriangles = [&]() {
            for (int fi : mem) {
                const Face& fc = b.f[static_cast<std::size_t>(fi)];
                out.faces.push_back({remap[static_cast<std::size_t>(fc.v[0])], remap[static_cast<std::size_t>(fc.v[1])], remap[static_cast<std::size_t>(fc.v[2])]});
            }
        };
        if (mem.size() == 1) { emitTriangles(); continue; }
        // arestas de fronteira do grupo (orientadas como nos triângulos)
        std::map<int, int> next;
        bool ok = true;
        for (int fi : mem)
            for (int e = 0; e < 3; ++e) {
                const Face& fc = b.f[static_cast<std::size_t>(fi)];
                const int g = fc.nb[static_cast<std::size_t>(e)];
                if (g >= 0 && group[static_cast<std::size_t>(g)] == gi) continue;
                if (!next.emplace(fc.v[static_cast<std::size_t>(e)], fc.v[static_cast<std::size_t>((e + 1) % 3)]).second) ok = false;
            }
        std::vector<int> loop;
        if (ok && !next.empty()) {
            const int start = next.begin()->first;
            int cur = start;
            do {
                loop.push_back(remap[static_cast<std::size_t>(cur)]);
                const auto it = next.find(cur);
                if (it == next.end() || loop.size() > next.size()) { ok = false; break; }
                cur = it->second;
            } while (cur != start);
            if (ok && loop.size() != next.size()) ok = false;   // mais de um laço (buraco): não funde
        } else {
            ok = false;
        }
        if (ok && loop.size() >= 3 && loop.size() <= static_cast<std::size_t>(hulllimits::kMaxFaceVertices)) out.faces.push_back(std::move(loop));
        else emitTriangles();
    }
    dropUnused();
    if (verify()) return true;
    // A fusão deixou alguma face fora da tolerância: usa os triângulos (sempre exatamente planares).
    out.vertices.clear();
    out.sourceIndex.clear();
    std::fill(remap.begin(), remap.end(), -1);
    for (int fi : alive)
        for (int v : b.f[static_cast<std::size_t>(fi)].v)
            if (remap[static_cast<std::size_t>(v)] < 0) {
                remap[static_cast<std::size_t>(v)] = static_cast<int>(out.vertices.size());
                out.vertices.push_back(points[static_cast<std::size_t>(v)]);
                out.sourceIndex.push_back(v);
            }
    triangles();
    return verify() ? true : giveUp();
}

} // namespace viewer
