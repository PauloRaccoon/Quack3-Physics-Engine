// =============================================================================
// GJK.cpp
// =============================================================================
#include "physics/collision/narrowphase/GJK.h"

#include <algorithm>
#include <cmath>

namespace physics {

namespace {

constexpr Real kDuplicateEpsSq = detail::kIsSinglePrecision ? Real(1e-12) : Real(1e-24);

/// Fator de RUÍDO numérico relativo: distâncias abaixo de  kGJKNoiseFactor · (maior coordenada do
/// simplex)  não se distinguem de zero em ponto flutuante. Com formas grandes (um chão de 100 m
/// tem diferença de Minkowski com coordenadas ~70 m) 1 ulp já passa de 1e-5 m, mais que o limiar
/// ABSOLUTO kGJKOriginEpsilonSq: a origem sobre o simplex aparecia como "a 1e-5 m", o suporte
/// seguinte repetia um vértice (a direção −v é só ruído) e o laço terminava declarando SEPARAÇÃO.
/// O contato gerado a partir daí tinha normal arbitrária e profundidade de dezenas de metros.
constexpr Real kGJKNoiseFactor = Real(16) * kEpsilon;

/// Reduz o simplex ao vértice `i`.
inline void keepOne(Simplex& s, Real* lam, int i) {
    s.v[0] = s.v[i];
    s.size = 1;
    lam[0] = Real(1);
}

// ---- Segmento (2 pontos) ---------------------------------------------------------
Vector3 closestOnSegment(Simplex& s, Real* lam) {
    const Vector3 a = s.v[0].point;
    const Vector3 b = s.v[1].point;
    const Vector3 ab = b - a;
    const Real abSq = ab.lengthSquared();
    if (!(abSq > kNormalizeEpsilonSq)) {  // segmento degenerado (pontos coincidentes)
        keepOne(s, lam, 0);
        return a;
    }
    // Projeção da origem em ab:  t = -a·ab / |ab|²
    const Real t = -a.dot(ab) / abSq;
    if (t <= Real(0)) { keepOne(s, lam, 0); return a; }
    if (t >= Real(1)) { keepOne(s, lam, 1); return b; }
    lam[0] = Real(1) - t;
    lam[1] = t;
    return a + ab * t;
}

// ---- Triângulo (3 pontos): regiões de Voronoi de Ericson --------------------------------
Vector3 closestOnTriangle(Simplex& s, Real* lam) {
    const SupportPoint A = s.v[0], B = s.v[1], C = s.v[2];
    const Vector3 a = A.point, b = B.point, c = C.point;
    const Vector3 ab = b - a, ac = c - a;

    // p = origem  ⇒  ap = -a, bp = -b, cp = -c
    const Real d1 = ab.dot(-a), d2 = ac.dot(-a);
    if (d1 <= Real(0) && d2 <= Real(0)) {                       // vértice A
        s.v[0] = A; s.size = 1; lam[0] = Real(1);
        return a;
    }
    const Real d3 = ab.dot(-b), d4 = ac.dot(-b);
    if (d3 >= Real(0) && d4 <= d3) {                            // vértice B
        s.v[0] = B; s.size = 1; lam[0] = Real(1);
        return b;
    }
    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= Real(0) && d1 >= Real(0) && d3 <= Real(0)) {     // aresta AB
        const Real t = d1 / (d1 - d3);
        s.v[0] = A; s.v[1] = B; s.size = 2;
        lam[0] = Real(1) - t; lam[1] = t;
        return a + ab * t;
    }
    const Real d5 = ab.dot(-c), d6 = ac.dot(-c);
    if (d6 >= Real(0) && d5 <= d6) {                            // vértice C
        s.v[0] = C; s.size = 1; lam[0] = Real(1);
        return c;
    }
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= Real(0) && d2 >= Real(0) && d6 <= Real(0)) {      // aresta AC
        const Real t = d2 / (d2 - d6);
        s.v[0] = A; s.v[1] = C; s.size = 2;
        lam[0] = Real(1) - t; lam[1] = t;
        return a + ac * t;
    }
    const Real va = d3 * d6 - d5 * d4;
    if (va <= Real(0) && (d4 - d3) >= Real(0) && (d5 - d6) >= Real(0)) {  // aresta BC
        const Real t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        s.v[0] = B; s.v[1] = C; s.size = 2;
        lam[0] = Real(1) - t; lam[1] = t;
        return b + (c - b) * t;
    }

    // Interior da face. va+vb+vc = |ab × ac|²; ~0 => triângulo degenerado (colinear).
    const Real denom = va + vb + vc;
    if (!(denom > kEpsilon * ab.lengthSquared() * ac.lengthSquared())) {
        // Escolhe a melhor das 3 arestas.
        const SupportPoint pts[3] = {A, B, C};
        static const int pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
        Real bestSq = kRealMax;
        Simplex best;
        Real bestLam[2] = {Real(1), Real(0)};
        Vector3 bestPt = a;
        for (const auto& pr : pairs) {
            Simplex seg;
            seg.v[0] = pts[pr[0]];
            seg.v[1] = pts[pr[1]];
            seg.size = 2;
            Real sl[2] = {Real(1), Real(0)};
            const Vector3 q = closestOnSegment(seg, sl);
            const Real sq = q.lengthSquared();
            if (sq < bestSq) {
                bestSq = sq; best = seg; bestPt = q;
                bestLam[0] = sl[0]; bestLam[1] = sl[1];
            }
        }
        s = best;
        lam[0] = bestLam[0]; lam[1] = bestLam[1];
        return bestPt;
    }
    const Real v = vb / denom;
    const Real w = vc / denom;
    lam[0] = Real(1) - v - w; lam[1] = v; lam[2] = w;
    return a + ab * v + ac * w;
}

// ---- Tetraedro (4 pontos) ----------------------------------------------------------------------
Vector3 closestOnTetra(Simplex& s, Real* lam) {
    // Cada linha: (i, j, k) = face; l = vértice oposto.
    static const int faces[4][4] = {{0, 1, 2, 3}, {0, 3, 1, 2}, {0, 2, 3, 1}, {1, 3, 2, 0}};

    Real bestSq = kRealMax;
    Simplex best;
    Real bestLam[3] = {Real(1), Real(0), Real(0)};
    Vector3 bestPt;
    bool outsideSomewhere = false;

    for (const auto& f : faces) {
        const Vector3 a = s.v[f[0]].point, b = s.v[f[1]].point, c = s.v[f[2]].point;
        const Vector3 d = s.v[f[3]].point;
        const Vector3 n = (b - a).cross(c - a);
        const Real signP = (-a).dot(n);   // lado da origem
        const Real signD = (d - a).dot(n);  // lado do vértice oposto
        // Origem do mesmo lado (estrito) que o vértice oposto: dentro deste semi-espaço.
        // Em caso de tetraedro achatado (signD≈0) a face é tratada como "fora" (seguro).
        if (signP * signD > Real(0)) continue;
        outsideSomewhere = true;

        Simplex tri;
        tri.v[0] = s.v[f[0]]; tri.v[1] = s.v[f[1]]; tri.v[2] = s.v[f[2]];
        tri.size = 3;
        Real tl[3];
        const Vector3 q = closestOnTriangle(tri, tl);
        const Real sq = q.lengthSquared();
        if (sq < bestSq) {
            bestSq = sq; best = tri; bestPt = q;
            bestLam[0] = tl[0]; bestLam[1] = tl[1]; bestLam[2] = tl[2];
        }
    }

    if (!outsideSomewhere) return Vector3::zero();   // origem dentro do tetraedro

    s = best;
    for (int i = 0; i < s.size; ++i) lam[i] = bestLam[i];
    return bestPt;
}

} // namespace

Vector3 GJK::closestPointOnSimplex(Simplex& s, Real* lam) {
    switch (s.size) {
        case 1:  lam[0] = Real(1); return s.v[0].point;
        case 2:  return closestOnSegment(s, lam);
        case 3:  return closestOnTriangle(s, lam);
        default: return closestOnTetra(s, lam);
    }
}

GJKResult GJK::compute(const MinkowskiSupport& support) {
    GJKResult r;
    Simplex& sx = r.simplex;
    Real lam[4] = {Real(1), Real(0), Real(0), Real(0)};

    sx.add(support.support(support.getInitialDirection()));

    Vector3 v;
    for (int it = 0;; ++it) {
        r.iterations = it;

        // 1) ponto do simplex mais próximo da origem (+ redução do simplex)
        v = closestPointOnSimplex(sx, lam);
        const Real lsq = v.lengthSquared();

        if (!std::isfinite(lsq)) {          // entrada com NaN/Inf: aborta sem travar
            r.converged = false;
            r.intersecting = false;
            r.distance = kRealMax;
            return r;
        }
        if (lsq <= kGJKOriginEpsilonSq || sx.size == 4) {   // origem no simplex
            r.intersecting = true;
            return r;
        }
        if (it >= kGJKMaxIterations) { r.converged = false; break; }

        // 2) suporte na direção da origem
        const SupportPoint w = support.support(-v);

        // 3) convergência: w não está significativamente mais perto da origem que v.
        //    gap = |v|² − v·w  (≥ 0); tolerância relativa à escala dos números.
        const Real gap = lsq - v.dot(w.point);
        const Real scale = std::max(lsq, std::sqrt(lsq * w.point.lengthSquared()));
        if (gap <= kGJKTolerance * scale) break;

        // 4) ponto repetido => sem progresso (estagnação numérica)
        bool duplicate = false;
        for (int i = 0; i < sx.size; ++i) {
            if ((w.point - sx.v[i].point).lengthSquared() <=
                kDuplicateEpsSq * (Real(1) + w.point.lengthSquared())) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) break;

        sx.add(w);
    }

    // O laço parou por estagnação (convergência, ponto repetido ou limite de iterações). Se a origem
    // está no nível do RUÍDO numérico do simplex, as formas se tocam/penetram: é interseção, e o EPA
    // (que aceita simplex de 1 a 3 pontos) dá a normal e a profundidade. Declarar separação aqui
    // usaria uma direção (v) que é só ruído.
    {
        Real mag2 = Real(0);
        for (int i = 0; i < sx.size; ++i) mag2 = std::max(mag2, sx.v[i].point.lengthSquared());
        if (v.lengthSquared() <= kGJKNoiseFactor * kGJKNoiseFactor * mag2) {
            r.intersecting = true;
            return r;
        }
    }

    // Separados: pontos mais próximos em A e B pelas baricêntricas do simplex reduzido.
    Vector3 cA, cB;
    for (int i = 0; i < sx.size; ++i) {
        cA += sx.v[i].a * lam[i];
        cB += sx.v[i].b * lam[i];
    }
    r.closestA = cA;
    r.closestB = cB;
    r.distance = v.length();
    return r;
}

} // namespace physics
