// =============================================================================
// ContactManifold.cpp
// Manifold persistente + gerador de contatos (GJK → EPA → clipping de faces).
// =============================================================================
#include "physics/collision/narrowphase/ContactManifold.h"

#include <algorithm>
#include <cmath>

#include "physics/collision/narrowphase/EPA.h"
#include "physics/collision/narrowphase/GJK.h"
#include "physics/collision/narrowphase/SupportFunction.h"
#include "physics/geometry/HalfEdgeStructure.h"
#include "physics/geometry/SphereShape.h"

namespace physics {

// =============================================================================
// ContactManifold
// =============================================================================
namespace {

constexpr Real kThresholdSq = kContactBreakingThreshold * kContactBreakingThreshold;

/// Área do quadrilátero formado por 4 pontos (em qualquer ordem): a maior,
/// entre os 3 pareamentos possíveis, de ½|d1 × d2| com d1,d2 as "diagonais".
Real quadArea(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3) {
    const Real a = (p0 - p1).cross(p2 - p3).lengthSquared();
    const Real b = (p0 - p2).cross(p1 - p3).lengthSquared();
    const Real c = (p0 - p3).cross(p1 - p2).lengthSquared();
    return Real(0.5) * std::sqrt(std::max(a, std::max(b, c)));
}

} // namespace

ContactPoint ContactManifold::makePoint(const Transform& xfA, const Transform& xfB,
                                        const PatchPoint& c) const {
    ContactPoint p;
    p.localA = xfA.inverseTransformPoint(c.pointA);
    p.localB = xfB.inverseTransformPoint(c.pointB);
    p.worldA = c.pointA;
    p.worldB = c.pointB;
    p.depth = c.depth;
    return p;
}

void ContactManifold::refresh(const Transform& xfA, const Transform& xfB) {
    int kept = 0;
    for (int i = 0; i < count; ++i) {
        ContactPoint p = points[i];
        p.worldA = xfA.transformPoint(p.localA);
        p.worldB = xfB.transformPoint(p.localB);
        const Vector3 d = p.worldA - p.worldB;
        p.depth = d.dot(normal);
        const Vector3 drift = d - normal * p.depth;   // componente tangencial (deslize)

        if (p.depth < -kContactBreakingThreshold) continue;       // se separou
        if (drift.lengthSquared() > kThresholdSq) continue;       // deslizou
        points[kept++] = p;
    }
    count = kept;
}

int ContactManifold::findMatch(const Vector3& localA, const bool* used) const {
    int best = -1;
    Real bestSq = kThresholdSq;
    for (int i = 0; i < count; ++i) {
        if (used && used[i]) continue;
        const Real d = (points[i].localA - localA).lengthSquared();
        if (d < bestSq) {
            bestSq = d;
            best = i;
        }
    }
    return best;
}

int ContactManifold::chooseDiscard(const ContactPoint& incoming) const {
    // Candidatos: os 4 pontos atuais (0..3) e o novo (4).
    const Vector3* pos[5];
    Real depth[5];
    for (int i = 0; i < 4; ++i) { pos[i] = &points[i].localA; depth[i] = points[i].depth; }
    pos[4] = &incoming.localA;
    depth[4] = incoming.depth;

    int deepest = 0;
    for (int i = 1; i < 5; ++i) if (depth[i] > depth[deepest]) deepest = i;

    // Descarta o candidato (≠ mais profundo) cuja ausência deixa a maior área.
    // Começa por um candidato VÁLIDO (o primeiro que não é o mais profundo): com coordenadas
    // absurdas o produto vetorial pode estourar para Inf e a área virar NaN, e NaN nunca vence
    // uma comparação — sem este valor inicial o retorno seria -1 e o chamador escreveria em
    // points[-1] (corrupção de memória, achada por fuzzing).
    int discard = (deepest == 0) ? 1 : 0;
    Real bestArea = -kRealMax;
    for (int r = 0; r < 5; ++r) {
        if (r == deepest) continue;
        const Vector3* q[4];
        int n = 0;
        for (int i = 0; i < 5; ++i) if (i != r) q[n++] = pos[i];
        Real area = quadArea(*q[0], *q[1], *q[2], *q[3]);
        if (std::isnan(area)) area = Real(0);   // área indefinida: pior que qualquer área real
        if (area > bestArea) {
            bestArea = area;
            discard = r;
        }
    }
    return discard;
}

int ContactManifold::addPoint(const Transform& xfA, const Transform& xfB, const PatchPoint& contact) {
    ContactPoint np = makePoint(xfA, xfB, contact);

    // 1) Casa com um ponto existente: atualiza posição, herda id e impulsos.
    const int m = findMatch(np.localA, nullptr);
    if (m >= 0) {
        np.id = points[m].id;
        np.normalImpulse = points[m].normalImpulse;
        np.tangentImpulse = points[m].tangentImpulse;
        points[m] = np;
        return m;
    }

    // 2) Ponto novo.
    np.id = nextId_++;
    if (count < kMaxManifoldPoints) {
        points[count] = np;
        return count++;
    }

    // 3) Cheio: descarta o que menos contribui para a área.
    const int r = chooseDiscard(np);
    if (r < 0 || r >= kMaxManifoldPoints) return -1;   // o próprio contato novo é o menos útil
    points[r] = np;
    return r;
}

void ContactManifold::update(const Transform& xfA, const Transform& xfB, const ContactPatch& patch) {
    if (patch.count <= 0) {
        clear();
        return;
    }
    normal = patch.normal;
    refresh(xfA, xfB);

    if (patch.count == 1) {
        addPoint(xfA, xfB, patch.points[0]);
        return;
    }

    // Patch completo (várias pontas de uma face): substitui os pontos, mas cada
    // novo ponto herda id/impulsos do ponto antigo mais próximo (warm starting).
    ContactPoint old[kMaxManifoldPoints];
    const int oldCount = count;
    for (int i = 0; i < oldCount; ++i) old[i] = points[i];
    bool used[kMaxManifoldPoints] = {false, false, false, false};

    count = 0;
    const int n = std::min(patch.count, kMaxManifoldPoints);
    for (int i = 0; i < n; ++i) {
        ContactPoint np = makePoint(xfA, xfB, patch.points[i]);

        int best = -1;
        Real bestSq = kThresholdSq;
        for (int j = 0; j < oldCount; ++j) {
            if (used[j]) continue;
            const Real d = (old[j].localA - np.localA).lengthSquared();
            if (d < bestSq) {
                bestSq = d;
                best = j;
            }
        }
        if (best >= 0) {
            used[best] = true;
            np.id = old[best].id;
            np.normalImpulse = old[best].normalImpulse;
            np.tangentImpulse = old[best].tangentImpulse;
        } else {
            np.id = nextId_++;
        }
        points[count++] = np;
    }
}

Real ContactManifold::getTotalNormalImpulse() const {
    Real sum = Real(0);
    for (int i = 0; i < count; ++i) sum += points[i].normalImpulse;
    return sum;
}

// =============================================================================
// ContactGenerator
// =============================================================================
namespace {

constexpr int kMaxFace = Shape::kMaxFaceVertices;
constexpr int kMaxClip = 2 * kMaxFace + 16;   // clipar um polígono convexo adiciona ≤ 1 vértice por plano

struct Candidate {
    Vector3 pointA;
    Vector3 pointB;
    Real depth;
};

/// Normal unitária de um polígono (método de Newell). false se degenerado.
bool newellNormal(const Vector3* p, int n, Vector3& out) {
    Vector3 nrm;
    for (int i = 0; i < n; ++i) {
        const Vector3& a = p[i];
        const Vector3& b = p[(i + 1) % n];
        nrm.x += (a.y - b.y) * (a.z + b.z);
        nrm.y += (a.z - b.z) * (a.x + b.x);
        nrm.z += (a.x - b.x) * (a.y + b.y);
    }
    if (nrm.normalize() <= Real(0)) return false;
    out = nrm;
    return true;
}

/// 2× a área assinada do triângulo (a,b,c) projetada em N.
Real triArea2(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& N) {
    return (b - a).cross(c - a).dot(N);
}

/// Reduz `n` candidatos a 4: o mais profundo, o mais distante dele, e os dois
/// que maximizam a área do polígono. Retorna a nova contagem (<= 4).
int reduceToFour(Candidate* c, int n, const Vector3& N) {
    if (n <= kMaxManifoldPoints) return n;
    auto pos = [&](int i) { return (c[i].pointA + c[i].pointB) * Real(0.5); };

    int i0 = 0;
    for (int i = 1; i < n; ++i) if (c[i].depth > c[i0].depth) i0 = i;

    int i1 = -1;
    Real best = Real(-1);
    for (int i = 0; i < n; ++i) {
        if (i == i0) continue;
        const Real d = (pos(i) - pos(i0)).lengthSquared();
        if (d > best) { best = d; i1 = i; }
    }

    int i2 = -1;
    best = Real(-1);
    for (int i = 0; i < n; ++i) {
        if (i == i0 || i == i1) continue;
        const Real a = std::abs(triArea2(pos(i0), pos(i1), pos(i), N));
        if (a > best) { best = a; i2 = i; }
    }

    const Vector3 p0 = pos(i0), p1 = pos(i1), p2 = pos(i2);
    const Real orient = triArea2(p0, p1, p2, N) >= Real(0) ? Real(1) : Real(-1);
    int i3 = -1;
    best = Real(-1e30);
    for (int i = 0; i < n; ++i) {
        if (i == i0 || i == i1 || i == i2) continue;
        const Vector3 x = pos(i);
        // quanto x está "fora" de cada aresta do triângulo (ganho de área)
        const Real g = std::max(-orient * triArea2(p0, p1, x, N),
                                std::max(-orient * triArea2(p1, p2, x, N),
                                         -orient * triArea2(p2, p0, x, N)));
        if (g > best) { best = g; i3 = i; }
    }

    const Candidate sel[4] = {c[i0], c[i1], c[i2], c[i3]};
    for (int i = 0; i < 4; ++i) c[i] = sel[i];
    return 4;
}

/// Polígono da face de suporte de `s` na direção de mundo `worldDir`, em coordenadas de mundo.
int worldFace(const Shape& s, const Transform& xf, const Vector3& worldDir, Vector3* out) {
    const int n = s.getSupportFace(xf.inverseTransformDirection(worldDir), out, kMaxFace);
    for (int i = 0; i < n; ++i) out[i] = xf.transformPoint(out[i]);
    return n;
}

/// Gera até 4 contatos por clipping da face incidente contra a face de
/// referência (Sutherland–Hodgman). `n` é a normal do contato (A→B).
/// Retorna 0 se o clipping não se aplica (o chamador usa o ponto do EPA).
int clipFaces(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB,
              const Vector3& n, const ContactGeneratorSettings& st, ContactPatch& patch) {
    Vector3 fa[kMaxFace], fb[kMaxFace];
    int na = worldFace(a, xfA, n, fa);
    int nb = worldFace(b, xfB, -n, fb);
    if (na < 3 || nb < 3) return 0;

    Vector3 nA, nB;
    if (!newellNormal(fa, na, nA) || !newellNormal(fb, nb, nB)) return 0;

    // Face de referência: a mais alinhada com a normal do contato.
    const Real alignA = nA.dot(n);
    const Real alignB = -nB.dot(n);
    const bool refIsA = alignA >= alignB;
    if (std::max(alignA, alignB) < st.minFaceAlignment) return 0;   // aresta×aresta, vértice, etc.

    const Vector3 N = refIsA ? nA : nB;   // normal (para fora) da face de referência
    const Vector3* ref;
    int nr;
    const Vector3* inc;
    int ni;
    if (refIsA) {   // face incidente de B: a mais anti-paralela a N
        nb = worldFace(b, xfB, -N, fb);
        if (nb < 3) return 0;
        ref = fa; nr = na; inc = fb; ni = nb;
    } else {        // face incidente de A
        na = worldFace(a, xfA, -N, fa);
        if (na < 3) return 0;
        ref = fb; nr = nb; inc = fa; ni = na;
    }

    // --- Sutherland–Hodgman: recorta a face incidente pelos planos laterais da referência ---
    Vector3 bufA[kMaxClip], bufB[kMaxClip];
    Vector3* in = bufA;
    Vector3* out = bufB;
    int m = ni;
    for (int i = 0; i < ni; ++i) in[i] = inc[i];

    for (int e = 0; e < nr && m > 0; ++e) {
        const Vector3& p = ref[e];
        const Vector3& q = ref[(e + 1) % nr];
        Vector3 s = (q - p).cross(N);               // normal lateral, para fora do polígono
        if (s.normalize() <= Real(0)) continue;     // aresta degenerada
        const Real offset = s.dot(p);

        int outCount = 0;
        for (int i = 0; i < m; ++i) {
            const Vector3& cur = in[i];
            const Vector3& nxt = in[(i + 1) % m];
            const Real dc = s.dot(cur) - offset;
            const Real dn = s.dot(nxt) - offset;
            if (dc <= Real(0)) {
                if (outCount < kMaxClip) out[outCount++] = cur;
                if (dc < Real(0) && dn > Real(0) && outCount < kMaxClip) {
                    out[outCount++] = cur + (nxt - cur) * (dc / (dc - dn));
                }
            } else if (dn < Real(0) && outCount < kMaxClip) {
                out[outCount++] = cur + (nxt - cur) * (dc / (dc - dn));
            }
        }
        std::swap(in, out);
        m = outCount;
    }
    if (m == 0) return 0;

    // --- Mantém os pontos abaixo (ou muito perto) do plano de referência ---------------------------------
    Candidate cand[kMaxClip];
    int nc = 0;
    for (int i = 0; i < m; ++i) {
        const Real sep = N.dot(in[i] - ref[0]);   // < 0: penetrando a face de referência
        if (sep > st.clipMargin) continue;
        const Vector3 onRef = in[i] - N * sep;
        Candidate c;
        c.pointA = refIsA ? onRef : in[i];
        c.pointB = refIsA ? in[i] : onRef;
        c.depth = -sep;
        cand[nc++] = c;
    }
    if (nc == 0) return 0;

    nc = reduceToFour(cand, nc, N);
    patch.normal = refIsA ? N : -N;
    for (int i = 0; i < nc; ++i) {
        patch.points[i].pointA = cand[i].pointA;
        patch.points[i].pointB = cand[i].pointB;
        patch.points[i].depth = cand[i].depth;
    }
    patch.count = nc;
    return nc;
}

/// Segmento extremo de `s` na direção de mundo `worldDir`, em coordenadas de mundo (false se a forma não tem).
bool worldSegment(const Shape& s, const Transform& xf, const Vector3& worldDir, Vector3& p0, Vector3& p1) {
    if (!s.getSupportSegment(xf.inverseTransformDirection(worldDir), p0, p1)) return false;
    p0 = xf.transformPoint(p0);
    p1 = xf.transformPoint(p1);
    return p0.isFinite() && p1.isFinite();
}

/// V18 — SEGMENTO × FACE: o núcleo de uma cápsula (ou a geratriz de um cilindro deitado) apoiado numa face plana.
/// Os dois extremos do segmento, recortados pela face (Cyrus–Beck contra os planos laterais), dão até 2 contatos:
/// uma cápsula deitada no chão não balança como balançaria com um único ponto. `segIsA` diz de que lado está o
/// segmento; `rS` é o raio do núcleo (cápsula: r; cilindro: 0). A normal do contato é a da face.
/// Retorna 0 (sem alterar o patch) se não se aplica: a outra forma não tem face, a face não está alinhada com a
/// normal (contato de aresta/vértice), o segmento caiu fora da face ou os dois extremos estão acima da margem.
int clipSegmentFace(const Vector3& P0, const Vector3& P1, Real rS, bool segIsA, const Shape& other, const Transform& xfO,
                    const Vector3& n, const ContactGeneratorSettings& st, ContactPatch& patch) {
    Vector3 face[kMaxFace];
    const Vector3 toward = segIsA ? -n : n;   // da outra forma para o segmento
    const int nf = worldFace(other, xfO, toward, face);
    if (nf < 3) return 0;
    Vector3 N;
    if (!newellNormal(face, nf, N)) return 0;
    if (N.dot(toward) < st.minFaceAlignment) return 0;

    // recorta o segmento pelos planos laterais da face
    Real t0 = Real(0), t1 = Real(1);
    for (int e = 0; e < nf; ++e) {
        const Vector3& p = face[e];
        const Vector3& q = face[(e + 1) % nf];
        Vector3 sd = (q - p).cross(N);
        if (sd.normalize() <= Real(0)) continue;
        const Real offset = sd.dot(p);
        const Real d0 = sd.dot(P0) - offset;
        const Real d1 = sd.dot(P1) - offset;
        if (d0 > Real(0) && d1 > Real(0)) return 0;   // fora desta aresta
        if (d0 > Real(0)) t0 = std::max(t0, d0 / (d0 - d1));
        else if (d1 > Real(0)) t1 = std::min(t1, d0 / (d0 - d1));
        if (t0 > t1) return 0;
    }
    const Vector3 Q[2] = {P0 + (P1 - P0) * t0, P0 + (P1 - P0) * t1};
    const bool single = (Q[1] - Q[0]).lengthSquared() < Real(1e-10);

    int count = 0;
    ContactPatch tmp;
    for (int i = 0; i < (single ? 1 : 2); ++i) {
        const Real height = N.dot(Q[i] - face[0]);   // altura do núcleo sobre o plano da face
        const Real depth = rS - height;
        if (depth < -st.clipMargin) continue;
        const Vector3 onPlane = Q[i] - N * height;
        const Vector3 onSeg = Q[i] - N * rS;   // ponto da superfície do segmento que olha para a face
        tmp.points[count].pointA = segIsA ? onSeg : onPlane;
        tmp.points[count].pointB = segIsA ? onPlane : onSeg;
        tmp.points[count].depth = depth;
        ++count;
    }
    if (count == 0) return 0;
    patch.normal = segIsA ? -N : N;
    for (int i = 0; i < count; ++i) patch.points[i] = tmp.points[i];
    patch.count = count;
    return count;
}

/// V18 — SEGMENTO × SEGMENTO paralelos (duas cápsulas deitadas lado a lado, cilindros deitados): dois contatos nas
/// pontas do trecho em que os segmentos se sobrepõem. Cruzados ou inclinados: retorna 0 (vale o ponto único do EPA).
int clipSegmentSegment(const Vector3& a0, const Vector3& a1, Real rA, const Vector3& b0, const Vector3& b1, Real rB,
                       const Vector3& n, const ContactGeneratorSettings& st, ContactPatch& patch) {
    Vector3 uA = a1 - a0, uB = b1 - b0;
    const Real lA = uA.normalize(), lB = uB.normalize();
    if (!(lA > Real(1e-6)) || !(lB > Real(1e-6))) return 0;
    if (std::abs(uA.dot(uB)) < Real(0.998) || std::abs(uA.dot(n)) > Real(0.2)) return 0;   // não paralelos / normal não perpendicular
    const Real tb0 = (b0 - a0).dot(uA), tb1 = (b1 - a0).dot(uA);
    const Real lo = std::max(Real(0), std::min(tb0, tb1));
    const Real hi = std::min(lA, std::max(tb0, tb1));
    if (!(hi - lo > Real(1e-4))) return 0;   // quase sem sobreposição: contato de ponta, o EPA resolve

    int count = 0;
    ContactPatch tmp;
    for (int i = 0; i < 2; ++i) {
        const Vector3 qA = a0 + uA * (i == 0 ? lo : hi);
        const Real tb = std::min(lB, std::max(Real(0), (qA - b0).dot(uB)));
        const Vector3 qB = b0 + uB * tb;
        const Real sep = (qB - qA).dot(n);
        const Real depth = rA + rB - sep;
        if (depth < -st.clipMargin) continue;
        tmp.points[count].pointA = qA + n * rA;
        tmp.points[count].pointB = qB - n * rB;
        tmp.points[count].depth = depth;
        ++count;
    }
    if (count == 0) return 0;
    patch.normal = n;
    for (int i = 0; i < count; ++i) patch.points[i] = tmp.points[i];
    patch.count = count;
    return count;
}

/// V18 — contatos de LINHA: cápsulas e cilindros deitados. Tenta segmento×segmento e segmento×face.
int clipFeatureSegments(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB, Real rA, Real rB,
                        const Vector3& n, const ContactGeneratorSettings& st, ContactPatch& patch) {
    Vector3 a0, a1, b0, b1;
    const bool sa = worldSegment(a, xfA, n, a0, a1);
    const bool sb = worldSegment(b, xfB, -n, b0, b1);
    if (sa && sb) return clipSegmentSegment(a0, a1, rA, b0, b1, rB, n, st, patch);
    if (sa) return clipSegmentFace(a0, a1, rA, true, b, xfB, n, st, patch);
    if (sb) return clipSegmentFace(b0, b1, rB, false, a, xfA, n, st, patch);
    return 0;
}

/// VALIDAÇÃO E REPARO do eixo de contato (rede de segurança contra GJK/EPA imprecisos).
///
/// O GJK em ponto flutuante perde precisão com formas grandes: um chão de 100 m tem diferença de
/// Minkowski com coordenadas de dezenas de metros, e a 1 ulp (~1e-5 m em float) já não distingue
/// uma folga ou penetração de décimos de milímetro. Medido: ~0,3 % das micro-penetrações caixa ×
/// chão de 101 m davam normal arbitrária ou profundidade de dezenas de metros (até 500 m).
/// Um contato assim empurra um corpo em repouso para o lado e basta UM quadro para derrubar uma pilha.
///
/// Para a diferença de Minkowski A ⊖ B, h(d) = max_{p ∈ A⊖B} p·d é a sobreposição das formas ao longo
/// do eixo d (A→B): > 0 penetra, < 0 há folga. A profundidade mínima de penetração é min_d h(d). Logo:
///   1) CONSISTÊNCIA: h(n) tem que coincidir com a profundidade reportada;
///   2) OTIMALIDADE: nenhum eixo de face de A ou de B pode ter h MENOR que a profundidade reportada
///      (se tiver, o reportado não é o mínimo — é o caso "consistente mas errado": normal lateral e
///      profundidade de 50 m, que é um eixo de separação válido, só que péssimo).
/// Se uma das duas falha, troca pelo melhor eixo de face, que é válido por construção. Resultados
/// corretos não são alterados (a troca só ocorre se existe um eixo estritamente melhor/consistente,
/// acima da tolerância numérica). Custo: ~1 + 6 chamadas de suporte para duas caixas.
void refineContactAxis(const MinkowskiSupport& S, const Shape& a, const Transform& xfA, const Shape& b,
                       const Transform& xfB, Vector3& n, Real& depth, Vector3& pA, Vector3& pB) {
    constexpr int kMaxFacesChecked = 64;   // poliedros enormes: só os primeiros (é uma rede de segurança)

    Vector3 bestAxis;
    SupportPoint bestW;
    Real bestH = kRealMax;
    bool haveBest = false;
    Real scale = Real(0);   // maior coordenada vista: define o ruído numérico

    auto consider = [&](const Vector3& axis) {
        const SupportPoint w = S.support(axis);
        const Real h = w.point.dot(axis);
        scale = std::max(scale, w.point.abs().maxComponent());
        if (std::isfinite(h) && (!haveBest || h < bestH)) {
            haveBest = true;
            bestH = h;
            bestAxis = axis;
            bestW = w;
        }
    };

    // Eixos candidatos: normais de face de A e de B, com o sinal que aponta de A para B. Com os
    // centros (quase) coincidentes o sinal é ambíguo e valem os dois.
    const Vector3 dirAB = xfB.position - xfA.position;
    const Real dirLen = dirAB.length();
    auto addFaces = [&](const Shape& sh, const Transform& xf) {
        const HalfEdgeStructure* he = sh.getHalfEdgeStructure();
        if (he == nullptr) return;
        const int nf = std::min(he->getFaceCount(), kMaxFacesChecked);
        for (int i = 0; i < nf; ++i) {
            const Vector3 w = xf.transformDirection(he->getFace(i).normal);
            const Real d = w.dot(dirAB);
            if (!(dirLen > Real(1e-6)) || std::abs(d) <= Real(1e-3) * dirLen) {
                consider(w);
                consider(-w);
            } else {
                consider(d > Real(0) ? w : -w);
            }
        }
    };
    addFaces(a, xfA);
    addFaces(b, xfB);
    if (!haveBest) return;

    const SupportPoint wN = S.support(n);
    const Real hN = wN.point.dot(n);
    scale = std::max(scale, wN.point.abs().maxComponent());
    const Real tol = std::max(Real(0.1) * kLinearSlop, Real(64) * kEpsilon * scale);

    // O resultado JÁ concorda com o melhor eixo de face (mesma direção até ~1°, mesma profundidade)?
    // Então está certo e não há o que reparar. Esta checagem vem ANTES do teste de consistência
    // porque h(n) é medida no ponto de suporte mais distante: numa forma grande (chão de 50 m) a
    // inclinação de ~1e-5 rad que o GJK deixa em n é multiplicada pela extensão e já passa da
    // tolerância (1,8 mm contra 0,5 mm), e o refino "consertava" contatos perfeitos — trocando o
    // ponto de contato por um canto do chão (medido: 27 % dos contatos esfera × chão em float).
    if (n.dot(bestAxis) >= Real(0.9998) && std::abs(depth - bestH) <= tol) return;

    const bool inconsistent = !(std::abs(hN - depth) <= tol);   // também pega NaN
    const bool notMinimal = bestH < depth - tol;
    if (!inconsistent && !notMinimal) return;

    n = bestAxis;
    depth = bestH;
    // Posição do contato: o GJK/EPA erra bem mais o EIXO que a posição lateral, então mantém o ponto
    // original se ele está sobre A (no AABB de A com folga). Só se não estiver cai no ponto de
    // suporte — que numa forma grande é um canto a dezenas de metros do contato de verdade.
    const AABB boxA = a.computeAABB(xfA).fattened(Real(0.05));
    if (!(pA.isFinite() && boxA.contains(pA))) pA = bestW.a;
    pB = pA - n * depth;   // mantém pA − pB = n · depth
}

} // namespace

bool ContactGenerator::generate(const Shape& a, const Transform& xfA, const Shape& b,
                                const Transform& xfB, ContactPatch& patch, const Settings& st) {
    patch.count = 0;
    if (!xfA.isFinite() || !xfB.isFinite()) return false;

    // --- Esfera × esfera: forma fechada --------------------------------------------------------------
    if (a.getType() == ShapeType::Sphere && b.getType() == ShapeType::Sphere) {
        const Real rA = static_cast<const SphereShape&>(a).getRadius();
        const Real rB = static_cast<const SphereShape&>(b).getRadius();
        const Vector3 d = xfB.position - xfA.position;
        const Real dist = d.length();
        const Real rsum = rA + rB;
        if (!(dist <= rsum + st.contactMargin)) return false;
        const Vector3 n = d.normalizedOr(Vector3::unitY());   // centros coincidentes: normal arbitrária
        patch.normal = n;
        patch.points[0].pointA = xfA.position + n * rA;
        patch.points[0].pointB = xfB.position - n * rB;
        patch.points[0].depth = rsum - dist;
        patch.count = 1;
        return true;
    }

    // --- Caso geral: GJK nos NÚCLEOS (esfera = ponto) + raios --------------------------------------------------
    const MinkowskiSupport support(a, xfA, b, xfB);
    const Real rA = support.getRadiusA();
    const Real rB = support.getRadiusB();

    const GJKResult g = GJK::compute(support);

    // Normal (A→B), penetração (> 0) ou −folga (< 0) e pontos de contato dos NÚCLEOS (sem os raios).
    Vector3 n, pA, pB;
    Real depthCore;
    if (!g.intersecting) {
        if (!(g.distance < kRealMax) || !(g.distance > Real(0))) return false;   // inválido
        // v = closestA − closestB ⇒ A→B é o oposto.
        n = (g.closestB - g.closestA) / g.distance;
        pA = g.closestA;
        pB = g.closestB;
        depthCore = -g.distance;
    } else {
        const EPAResult e = EPA::compute(support, g.simplex);
        if (!e.valid) return false;
        n = e.normal;
        pA = e.pointA;
        pB = e.pointB;
        depthCore = e.depth;
    }
    if (!n.isFinite() || !pA.isFinite() || !pB.isFinite() || !std::isfinite(depthCore)) return false;

    refineContactAxis(support, a, xfA, b, xfB, n, depthCore, pA, pB);   // rede de segurança numérica

    const Real depth = depthCore + rA + rB;
    if (!(depth >= -st.contactMargin)) return false;   // separados por mais que a margem
    pA = pA + n * rA;
    pB = pB - n * rB;
    if (!n.isFinite() || !pA.isFinite() || !pB.isFinite() || !std::isfinite(depth)) return false;

    // Ponto único (EPA / GJK) — o fallback sempre disponível.
    patch.normal = n;
    patch.points[0].pointA = pA;
    patch.points[0].pointB = pB;
    patch.points[0].depth = depth;
    patch.count = 1;

    // Várias pontas por clipping, se ambas as formas têm faces planas.
    if (st.useFaceClipping) {
        // Faces planas (caixa, poliedro, tampa de cilindro); se não se aplica, contatos de linha (V18).
        const bool done = rA == Real(0) && rB == Real(0) && clipFaces(a, xfA, b, xfB, n, st, patch) > 0;
        if (!done) clipFeatureSegments(a, xfA, b, xfB, rA, rB, n, st, patch);   // não altera o patch se retornou 0
    }
    return true;
}

bool ContactGenerator::collide(const Shape& a, const Transform& xfA, const Shape& b,
                               const Transform& xfB, ContactManifold& manifold,
                               const Settings& settings) {
    ContactPatch patch;
    if (!generate(a, xfA, b, xfB, patch, settings)) {
        manifold.clear();
        return false;
    }
    manifold.update(xfA, xfB, patch);
    return true;
}

} // namespace physics
