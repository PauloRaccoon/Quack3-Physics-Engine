// =============================================================================
// MathUtils.h
// Funções escalares utilitárias e "seguras" (proteção contra divisão por zero,
// raiz de negativos, NaN/Inf).
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>

#include "physics/common/Settings.h"

namespace physics {

/// x^2
constexpr Real square(Real x) { return x * x; }

/// Restringe v ao intervalo [lo, hi].
constexpr Real clamp(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }

/// Interpolação linear: a + (b - a) * t.
constexpr Real lerp(Real a, Real b, Real t) { return a + (b - a) * t; }

/// Sinal: -1, 0 ou +1.
constexpr Real sign(Real x) { return (x > Real(0)) - (x < Real(0)); }

constexpr Real degToRad(Real deg) { return deg * (kPi / Real(180)); }
constexpr Real radToDeg(Real rad) { return rad * (Real(180) / kPi); }

/// true se v não é NaN nem infinito.
inline bool isFinite(Real v) { return std::isfinite(v); }

/// |v| <= eps
inline bool nearlyZero(Real v, Real eps = kCompareEpsilon) { return std::abs(v) <= eps; }

/// Comparação com tolerância mista (absoluta + relativa).
inline bool nearlyEqual(Real a, Real b, Real eps = kCompareEpsilon) {
    const Real diff = std::abs(a - b);
    if (diff <= eps) return true;
    return diff <= eps * std::max(std::abs(a), std::abs(b));
}

/// 1/v, ou 0 quando |v| é pequeno demais (evita divisão por zero / Inf).
/// Retornar 0 equivale a "massa infinita" nas fórmulas do solver.
inline Real safeInverse(Real v) {
    return std::abs(v) > kDivideEpsilon ? Real(1) / v : Real(0);
}

/// sqrt que não retorna NaN para pequenas entradas negativas (erro numérico).
inline Real safeSqrt(Real v) { return v > Real(0) ? std::sqrt(v) : Real(0); }

/// acos com argumento saturado em [-1, 1] (evita NaN por arredondamento).
inline Real safeAcos(Real v) { return std::acos(clamp(v, Real(-1), Real(1))); }

/// asin com argumento saturado em [-1, 1].
inline Real safeAsin(Real v) { return std::asin(clamp(v, Real(-1), Real(1))); }

} // namespace physics
