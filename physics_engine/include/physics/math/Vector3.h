// =============================================================================
// Vector3.h
// Vetor 3D com sobrecarga completa de operadores. Operações triviais são
// inline (caminho quente do solver); utilidades maiores ficam em Vector3.cpp.
// =============================================================================
#pragma once

#include <cmath>
#include <iosfwd>

#include "physics/common/Settings.h"
#include "physics/math/MathUtils.h"

namespace physics {

struct Vector3 {
    Real x = Real(0);
    Real y = Real(0);
    Real z = Real(0);

    // --- Construção ----------------------------------------------------------
    constexpr Vector3() = default;
    constexpr Vector3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}
    explicit constexpr Vector3(Real s) : x(s), y(s), z(s) {}

    static constexpr Vector3 zero()  { return Vector3(0, 0, 0); }
    static constexpr Vector3 one()   { return Vector3(1, 1, 1); }
    static constexpr Vector3 unitX() { return Vector3(1, 0, 0); }
    static constexpr Vector3 unitY() { return Vector3(0, 1, 0); }
    static constexpr Vector3 unitZ() { return Vector3(0, 0, 1); }

    // --- Acesso por índice (0=x, 1=y, 2=z) ------------------------------------
    Real& operator[](int i)             { return i == 0 ? x : (i == 1 ? y : z); }
    const Real& operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    // --- Operadores compostos --------------------------------------------------
    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vector3& operator*=(Real s)           { x *= s; y *= s; z *= s; return *this; }
    /// Divisão segura: divisor ~0 produz vetor nulo (ver safeInverse).
    Vector3& operator/=(Real s)           { return *this *= safeInverse(s); }

    constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }

    // --- Álgebra vetorial ------------------------------------------------------
    constexpr Real dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }

    constexpr Vector3 cross(const Vector3& o) const {
        return Vector3(y * o.z - z * o.y,
                       z * o.x - x * o.z,
                       x * o.y - y * o.x);
    }

    constexpr Real lengthSquared() const { return x * x + y * y + z * z; }
    Real length() const { return std::sqrt(lengthSquared()); }

    Real distance(const Vector3& o) const { return (*this - o).length(); }
    constexpr Real distanceSquared(const Vector3& o) const {
        return (*this - o).lengthSquared();
    }

    /// Normaliza in-place e devolve o comprimento original. Se o vetor for
    /// degenerado (comprimento ~0, NaN ou Inf) ele vira (0,0,0) e retorna 0.
    Real normalize() {
        const Real lsq = lengthSquared();
        if (!(lsq > kNormalizeEpsilonSq) || !std::isfinite(lsq)) {
            x = y = z = Real(0);
            return Real(0);
        }
        const Real len = std::sqrt(lsq);
        const Real inv = Real(1) / len;
        x *= inv; y *= inv; z *= inv;
        return len;
    }

    /// Cópia normalizada; retorna (0,0,0) para vetores degenerados.
    Vector3 normalized() const { Vector3 r = *this; r.normalize(); return r; }

    /// Cópia normalizada; retorna `fallback` para vetores degenerados.
    Vector3 normalizedOr(const Vector3& fallback) const {
        Vector3 r = *this;
        return r.normalize() > Real(0) ? r : fallback;
    }

    // --- Consultas ---------------------------------------------------------------
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
    bool isZero(Real eps = kCompareEpsilon) const { return lengthSquared() <= eps * eps; }

    bool nearlyEqual(const Vector3& o, Real eps = kCompareEpsilon) const {
        return physics::nearlyEqual(x, o.x, eps) && physics::nearlyEqual(y, o.y, eps) &&
               physics::nearlyEqual(z, o.z, eps);
    }

    Vector3 abs() const { return Vector3(std::abs(x), std::abs(y), std::abs(z)); }
    Real minComponent() const { return std::min(x, std::min(y, z)); }
    Real maxComponent() const { return std::max(x, std::max(y, z)); }

    /// Índice (0..2) do eixo de maior |componente|.
    int maxAxis() const {
        const Real ax = std::abs(x), ay = std::abs(y), az = std::abs(z);
        return ax >= ay ? (ax >= az ? 0 : 2) : (ay >= az ? 1 : 2);
    }

    // --- Operações geométricas ---------------------------------------------------
    /// Projeção deste vetor sobre `onto` (0 se `onto` for ~0).
    Vector3 projectOnto(const Vector3& onto) const {
        return onto * (dot(onto) * safeInverse(onto.lengthSquared()));
    }

    /// Reflexão em torno de uma normal unitária.
    Vector3 reflect(const Vector3& unitNormal) const {
        return *this - unitNormal * (Real(2) * dot(unitNormal));
    }

    static Vector3 lerp(const Vector3& a, const Vector3& b, Real t) { return a + (b - a) * t; }

    // --- Operadores binários (definidos como amigos para ficarem inline) ---------
    friend constexpr Vector3 operator+(const Vector3& a, const Vector3& b) {
        return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    friend constexpr Vector3 operator-(const Vector3& a, const Vector3& b) {
        return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
    }
    friend constexpr Vector3 operator*(const Vector3& v, Real s) {
        return Vector3(v.x * s, v.y * s, v.z * s);
    }
    friend constexpr Vector3 operator*(Real s, const Vector3& v) { return v * s; }
    /// Produto componente a componente (Hadamard).
    friend constexpr Vector3 operator*(const Vector3& a, const Vector3& b) {
        return Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
    }
    /// Divisão segura por escalar: divisor ~0 produz vetor nulo.
    friend Vector3 operator/(const Vector3& v, Real s) { return v * safeInverse(s); }

    friend constexpr bool operator==(const Vector3& a, const Vector3& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
    friend constexpr bool operator!=(const Vector3& a, const Vector3& b) { return !(a == b); }
};

// --- Funções livres -------------------------------------------------------------
constexpr Real dot(const Vector3& a, const Vector3& b) { return a.dot(b); }
constexpr Vector3 cross(const Vector3& a, const Vector3& b) { return a.cross(b); }

/// Produto misto a · (b × c)  (volume orientado do paralelepípedo).
constexpr Real tripleProduct(const Vector3& a, const Vector3& b, const Vector3& c) {
    return a.dot(b.cross(c));
}

inline Vector3 minVec(const Vector3& a, const Vector3& b) {
    return Vector3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}
inline Vector3 maxVec(const Vector3& a, const Vector3& b) {
    return Vector3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

/// Dada uma normal unitária `n`, calcula t1 e t2 tais que {n, t1, t2} forma uma
/// base ortonormal. Usada para os eixos tangenciais do atrito de Coulomb.
void computeOrthonormalBasis(const Vector3& n, Vector3& t1, Vector3& t2);

std::ostream& operator<<(std::ostream& os, const Vector3& v);

} // namespace physics
