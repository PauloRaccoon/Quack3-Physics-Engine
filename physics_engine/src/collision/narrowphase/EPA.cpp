// =============================================================================
// EPA.cpp
// =============================================================================
#include "physics/collision/narrowphase/EPA.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace physics {

namespace {

constexpr int kMaxVerts = kEPAMaxIterations + 8;   // 4 iniciais + 1 por iteração + folga
constexpr int kMaxFaces = 3 * kMaxVerts;           // um politopo fechado tem 2V-4 faces

constexpr Real kPointEpsSq = detail::kIsSinglePrecision ? Real(1e-12) : Real(1e-24);
/// sen²(ângulo) abaixo do qual uma face/tetraedro é considerado degenerado.
constexpr Real kDegenerateSinSq = detail::kIsSinglePrecision ? Real(1e-10) : Real(1e-20);

struct Face {
    int i[3];     ///< índices dos vértices (anti-horário visto de fora)
    Vector3 n;    ///< normal unitária para fora
    Real d;       ///< distância da origem ao plano (>= 0)
};

struct Edge {
    int a, b;
};

// ---- Testes de degeneração (relativos à escala) --------------------------------------------------
bool segmentDegenerate(const Vector3& a, const Vector3& b) {
    return (b - a).lengthSquared() <= kPointEpsSq;
}

bool triangleDegenerate(const Vector3& a, const Vector3& b, const Vector3& c) {
    const Vector3 e1 = b - a, e2 = c - a;
    return e1.cross(e2).lengthSquared() <= kDegenerateSinSq * e1.lengthSquared() * e2.lengthSquared() ||
           e1.lengthSquared() <= kPointEpsSq || e2.lengthSquared() <= kPointEpsSq;
}

bool tetraDegenerate(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d) {
    const Vector3 e1 = b - a, e2 = c - a, e3 = d - a;
    const Real vol = e1.dot(e2.cross(e3));
    return vol * vol <= kDegenerateSinSq * e1.lengthSquared() * e2.lengthSquared() * e3.lengthSquared();
}

// ---- Politopo inicial (tetraedro) -------------------------------------------------------------------
/// Constrói 4 vértices não coplanares de A ⊖ B a partir do simplex do GJK
/// (que pode ter 1..4 pontos, possivelmente degenerados).
bool buildInitialTetra(const MinkowskiSupport& S, const Simplex& in, SupportPoint* v) {
    int n = in.size;
    if (n < 1 || n > 4) return false;
    for (int i = 0; i < n; ++i) v[i] = in.v[i];

    // 1) descarta degeneração (do mais alto para o mais baixo)
    if (n == 4 && tetraDegenerate(v[0].point, v[1].point, v[2].point, v[3].point)) n = 3;
    if (n == 3 && triangleDegenerate(v[0].point, v[1].point, v[2].point)) n = 2;
    if (n == 2 && segmentDegenerate(v[0].point, v[1].point)) n = 1;

    // 2) 1 → 2 pontos: procura um suporte distinto ao longo dos eixos
    if (n == 1) {
        static const Vector3 axes[6] = {Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0),
                                        Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(0, 0, -1)};
        for (const Vector3& d : axes) {
            const SupportPoint w = S.support(d);
            if (!segmentDegenerate(v[0].point, w.point)) {
                v[1] = w;
                n = 2;
                break;
            }
        }
        if (n == 1) return false;   // A ⊖ B é um único ponto: sem volume
    }

    // 3) 2 → 3 pontos: suporte em direções perpendiculares ao segmento
    if (n == 2) {
        const Vector3 e = (v[1].point - v[0].point).normalized();
        // Eixo menos alinhado com e → perpendicular bem condicionada.
        Vector3 helper = Vector3::unitX();
        if (std::abs(e.y) < std::abs(e.x) && std::abs(e.y) <= std::abs(e.z)) helper = Vector3::unitY();
        else if (std::abs(e.z) < std::abs(e.x) && std::abs(e.z) < std::abs(e.y)) helper = Vector3::unitZ();
        const Vector3 u = e.cross(helper).normalized();
        const Vector3 t = e.cross(u).normalized();
        const Vector3 dirs[4] = {u, -u, t, -t};
        for (const Vector3& d : dirs) {
            const SupportPoint w = S.support(d);
            if (!triangleDegenerate(v[0].point, v[1].point, w.point)) {
                v[2] = w;
                n = 3;
                break;
            }
        }
        if (n == 2) return false;   // A ⊖ B é um segmento: sem área
    }

    // 3 → 4 pontos: apex do lado em que A ⊖ B se estende mais.
    if (n == 3) {
        const Vector3 nrm = (v[1].point - v[0].point).cross(v[2].point - v[0].point).normalized();
        const SupportPoint wp = S.support(nrm);
        const SupportPoint wm = S.support(-nrm);
        const Real hp = (wp.point - v[0].point).dot(nrm);
        const Real hm = -(wm.point - v[0].point).dot(nrm);
        const SupportPoint& w = hp >= hm ? wp : wm;
        if (!tetraDegenerate(v[0].point, v[1].point, v[2].point, w.point)) {
            v[3] = w;
            n = 4;
        }
        if (n == 3) return false;   // A ⊖ B é plano: sem volume
    }
    return n == 4;
}

/// Cria a face (a,b,c) com normal calculada pelo enrolamento. Retorna false se degenerada.
bool makeFace(const SupportPoint* verts, int a, int b, int c, Face& f) {
    const Vector3& pa = verts[a].point;
    const Vector3 e1 = verts[b].point - pa;
    const Vector3 e2 = verts[c].point - pa;
    Vector3 n = e1.cross(e2);
    const Real lsq = n.lengthSquared();
    if (!(lsq > kDegenerateSinSq * e1.lengthSquared() * e2.lengthSquared()) || !(lsq > Real(0))) {
        return false;
    }
    n /= std::sqrt(lsq);
    f.i[0] = a; f.i[1] = b; f.i[2] = c;
    f.n = n;
    // O enrolamento já garante orientação para fora; d < 0 só ocorre por ruído
    // numérico (origem sobre o plano) e é saturado em 0.
    f.d = std::max(Real(0), n.dot(pa));
    return true;
}

/// Adiciona a aresta (a,b); se a oposta (b,a) já está na lista, as duas se
/// cancelam (aresta interna entre duas faces visíveis).
void addEdge(Edge* edges, int& count, int a, int b) {
    for (int e = 0; e < count; ++e) {
        if (edges[e].a == b && edges[e].b == a) {
            edges[e] = edges[--count];
            return;
        }
    }
    edges[count].a = a;
    edges[count].b = b;
    ++count;
}

int closestFace(const Face* faces, int count) {
    int best = 0;
    for (int i = 1; i < count; ++i) {
        if (faces[i].d < faces[best].d) best = i;
    }
    return best;
}

} // namespace

EPAResult EPA::compute(const MinkowskiSupport& S, const Simplex& simplex) {
    EPAResult res;

    SupportPoint verts[kMaxVerts];
    int nv = 4;
    if (!buildInitialTetra(S, simplex, verts)) return res;

    // Orienta o tetraedro: com volume assinado negativo, as faces abaixo
    // ficam todas com a normal para fora.
    const Real vol = (verts[1].point - verts[0].point).cross(verts[2].point - verts[0].point)
                         .dot(verts[3].point - verts[0].point);
    if (vol > Real(0)) std::swap(verts[1], verts[2]);

    Face faces[kMaxFaces];
    int nf = 0;
    static const int kInitial[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    for (const auto& t : kInitial) {
        if (!makeFace(verts, t[0], t[1], t[2], faces[nf])) return res;
        ++nf;
    }

    Face best = faces[0];
    bool intact = true;   // o politopo corrente é um sólido fechado e válido?
    int it = 0;
    for (; it < kEPAMaxIterations; ++it) {
        best = faces[closestFace(faces, nf)];

        // Suporte na direção da normal da face mais próxima.
        const SupportPoint w = S.support(best.n);
        const Real dw = w.point.dot(best.n);
        if (!(dw - best.d > kEPATolerance)) {   // sem avanço: a face está na borda de A ⊖ B
            res.converged = true;
            break;
        }

        if (nv >= kMaxVerts) { intact = false; break; }
        const int wi = nv;
        verts[nv++] = w;

        // Remove as faces visíveis de w e coleta as arestas do "horizonte".
        Edge edges[kMaxFaces];
        int ne = 0;
        for (int f = 0; f < nf;) {
            const Face& F = faces[f];
            if (F.n.dot(w.point) - F.d > kEPAVisibleEpsilon) {
                addEdge(edges, ne, F.i[0], F.i[1]);
                addEdge(edges, ne, F.i[1], F.i[2]);
                addEdge(edges, ne, F.i[2], F.i[0]);
                faces[f] = faces[--nf];
            } else {
                ++f;
            }
        }
        if (ne == 0 || nf + ne > kMaxFaces) { intact = false; break; }

        // Fecha o buraco ligando cada aresta do horizonte a w.
        bool ok = true;
        for (int e = 0; e < ne; ++e) {
            if (!makeFace(verts, edges[e].a, edges[e].b, wi, faces[nf])) { ok = false; break; }
            ++nf;
        }
        if (!ok) { intact = false; break; }
    }
    if (!res.converged && intact) best = faces[closestFace(faces, nf)];   // limite de iterações
    res.iterations = it;

    // --- Resultado a partir da face mais próxima ---------------------------------------------------
    // O pé da origem no plano da face é  p = n·d. Uma face grande de A ⊖ B costuma
    // ser triangulada em VÁRIOS triângulos coplanares com o mesmo `d`: o pé só
    // está dentro de um deles. Entre os coplanares, escolhe o que contém o pé
    // (maior coordenada baricêntrica mínima).
    const Vector3 p = best.n * best.d;
    auto barycentric = [&](const Face& f, Real& bu, Real& bv, Real& bw) {
        const Vector3& a = verts[f.i[0]].point;
        const Vector3 e0 = verts[f.i[1]].point - a, e1 = verts[f.i[2]].point - a, e2 = p - a;
        const Real d00 = e0.dot(e0), d01 = e0.dot(e1), d11 = e1.dot(e1);
        const Real d20 = e2.dot(e0), d21 = e2.dot(e1);
        const Real den = d00 * d11 - d01 * d01;
        bu = Real(1); bv = Real(0); bw = Real(0);
        if (std::abs(den) > Real(0)) {
            bv = (d11 * d20 - d01 * d21) / den;
            bw = (d00 * d21 - d01 * d20) / den;
            bu = Real(1) - bv - bw;
        }
    };

    Face chosen = best;
    Real u, v, wgt;
    barycentric(chosen, u, v, wgt);
    if (intact && std::min(u, std::min(v, wgt)) < Real(0)) {   // o pé caiu fora deste triângulo
        Real bestScore = std::min(u, std::min(v, wgt));
        const Real dTol = std::max(kEPATolerance, Real(1e-4) * best.d);
        for (int f = 0; f < nf; ++f) {
            const Face& F = faces[f];
            if (std::abs(F.d - best.d) > dTol || F.n.dot(best.n) < Real(0.9999)) continue;   // outro plano
            Real fu, fv, fw;
            barycentric(F, fu, fv, fw);
            const Real score = std::min(fu, std::min(fv, fw));
            if (score > bestScore) {
                bestScore = score; chosen = F; u = fu; v = fv; wgt = fw;
            }
        }
    }
    const SupportPoint& pa = verts[chosen.i[0]];
    const SupportPoint& pb = verts[chosen.i[1]];
    const SupportPoint& pc = verts[chosen.i[2]];
    // O pé fica dentro da face (face mais próxima de um politopo convexo); ruído
    // numérico pode deixá-lo ligeiramente fora: satura e renormaliza.
    u = std::max(u, Real(0)); v = std::max(v, Real(0)); wgt = std::max(wgt, Real(0));
    const Real sum = u + v + wgt;
    if (sum > Real(0)) { u /= sum; v /= sum; wgt /= sum; } else { u = Real(1); v = wgt = Real(0); }

    res.normal = best.n;
    res.depth = best.d;
    res.pointA = pa.a * u + pb.a * v + pc.a * wgt;
    res.pointB = pa.b * u + pb.b * v + pc.b * wgt;
    res.valid = res.normal.isFinite() && std::isfinite(res.depth) && res.pointA.isFinite() &&
                res.pointB.isFinite();
    return res;
}

} // namespace physics
