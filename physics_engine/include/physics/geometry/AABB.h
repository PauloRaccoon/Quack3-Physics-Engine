// =============================================================================
// AABB.h
// Caixa delimitadora alinhada aos eixos (Axis-Aligned Bounding Box).
// Inclui as operações necessárias para os "Fat AABBs" da broadphase:
//  - fattened():          expande por uma margem fixa;
//  - fattenedForMotion(): expande também na direção do deslocamento do corpo;
//  - transformed():       AABB de uma AABB local rotacionada (cobre rotação).
// Um AABB recém-construído é "vazio" (min = +max, max = -max), de modo que
// extend()/merge() funcionam sem caso especial.
// =============================================================================
#pragma once

#include <algorithm>
#include <utility>

#include "physics/common/Settings.h"
#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

namespace physics {

struct AABB {
    Vector3 min = Vector3(kRealMax);
    Vector3 max = Vector3(-kRealMax);

    AABB() = default;
    AABB(const Vector3& lo, const Vector3& hi) : min(lo), max(hi) {}

    static AABB fromCenterHalfExtents(const Vector3& center, const Vector3& half) {
        return AABB(center - half, center + half);
    }

    // --- Consultas -----------------------------------------------------------
    /// true se min <= max em todos os eixos (AABB vazio é inválido).
    bool isValid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    bool isFinite() const { return min.isFinite() && max.isFinite(); }

    Vector3 center() const { return (min + max) * Real(0.5); }
    Vector3 halfExtents() const { return (max - min) * Real(0.5); }
    Vector3 size() const { return max - min; }

    Real volume() const {
        if (!isValid()) return Real(0);
        const Vector3 d = size();
        return d.x * d.y * d.z;
    }

    /// Área da superfície; é a métrica do custo (SAH) usada pela DynamicAABBTree.
    Real surfaceArea() const {
        if (!isValid()) return Real(0);
        const Vector3 d = size();
        return Real(2) * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    bool contains(const Vector3& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    /// true se `o` está inteiramente dentro deste AABB.
    bool contains(const AABB& o) const {
        return o.min.x >= min.x && o.max.x <= max.x && o.min.y >= min.y && o.max.y <= max.y &&
               o.min.z >= min.z && o.max.z <= max.z;
    }

    /// Interseção inclusiva (AABBs que apenas se tocam contam como sobrepostos).
    bool overlaps(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x && min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }

    /// Raio (origem + t·direção, t em [0, maxDistance]) contra o AABB, pelo método
    /// dos "slabs". `direction` deve ser UNITÁRIA e `origin` finita (o chamador
    /// valida: NaN aqui produziria um falso positivo). Componentes de direção
    /// ~0 (raio paralelo a um par de faces) só passam se a origem está entre as
    /// faces. Em caso de interseção, `tEnter` recebe a distância de entrada
    /// (0 se a origem está dentro do AABB). AABB inválido (vazio) nunca intersecta.
    bool intersectsRay(const Vector3& origin, const Vector3& direction, Real maxDistance,
                       Real& tEnter) const {
        if (!isValid()) return false;
        Real tmin = Real(0);
        Real tmax = maxDistance;
        for (int i = 0; i < 3; ++i) {
            const Real d = direction[i];
            if (std::abs(d) < kDivideEpsilon) {
                if (origin[i] < min[i] || origin[i] > max[i]) return false;
            } else {
                const Real inv = Real(1) / d;
                Real t1 = (min[i] - origin[i]) * inv;
                Real t2 = (max[i] - origin[i]) * inv;
                if (t1 > t2) std::swap(t1, t2);
                if (t1 > tmin) tmin = t1;
                if (t2 < tmax) tmax = t2;
                if (tmin > tmax) return false;
            }
        }
        tEnter = tmin;
        return true;
    }

    // --- Modificação ------------------------------------------------------------
    void extend(const Vector3& p) { min = minVec(min, p); max = maxVec(max, p); }
    void extend(const AABB& o) { min = minVec(min, o.min); max = maxVec(max, o.max); }

    /// União de dois AABBs.
    static AABB merge(const AABB& a, const AABB& b) {
        return AABB(minVec(a.min, b.min), maxVec(a.max, b.max));
    }

    // --- Fat AABB ------------------------------------------------------------------
    /// Expande uniformemente em `margin` em cada direção.
    AABB fattened(Real margin) const {
        const Vector3 m(margin);
        return AABB(min - m, max + m);
    }

    /// Fat AABB para um corpo em movimento: margem fixa + extensão na direção
    /// do deslocamento previsto (`displacement * multiplier`). Assim o corpo
    /// pode continuar a se mover na mesma direção por vários quadros sem forçar
    /// a reinserção na árvore (técnica do Box2D).
    AABB fattenedForMotion(const Vector3& displacement, Real margin,
                           Real multiplier = kAABBDisplacementMultiplier) const {
        AABB r = fattened(margin);
        const Vector3 d = displacement * multiplier;
        for (int i = 0; i < 3; ++i) {
            if (d[i] < Real(0)) r.min[i] += d[i];
            else                r.max[i] += d[i];
        }
        return r;
    }

    /// true se o AABB justo `tight` escapou deste Fat AABB (precisa reinserir).
    bool needsRefit(const AABB& tight) const { return !contains(tight); }

    // --- Transformação -----------------------------------------------------------------
    /// AABB (no mundo) desta AABB local sob a transformação `xf`.
    /// Método de Arvo: novo semi-extent = |R| * semi-extent (exato e O(1)).
    AABB transformed(const Transform& xf) const {
        if (!isValid()) return *this;
        const Matrix3 R = xf.rotation.toMatrix();
        const Vector3 e = halfExtents();
        const Vector3 newHalf(
            std::abs(R(0, 0)) * e.x + std::abs(R(0, 1)) * e.y + std::abs(R(0, 2)) * e.z,
            std::abs(R(1, 0)) * e.x + std::abs(R(1, 1)) * e.y + std::abs(R(1, 2)) * e.z,
            std::abs(R(2, 0)) * e.x + std::abs(R(2, 1)) * e.y + std::abs(R(2, 2)) * e.z);
        return fromCenterHalfExtents(xf.transformPoint(center()), newHalf);
    }
};

} // namespace physics
