// =============================================================================
// Quaternion.h
// Quatérnio (w, x, y, z) usado para representar orientações de corpos rígidos.
// Convenção de Hamilton; rotação ativa  v' = q * v * q⁻¹.
// A maioria das operações assume quatérnio UNITÁRIO; use normalize() após
// acumular erro (por exemplo, depois de integrar a velocidade angular).
// =============================================================================
#pragma once

#include <iosfwd>

#include "physics/common/Settings.h"
#include "physics/math/MathUtils.h"
#include "physics/math/Matrix3.h"
#include "physics/math/Vector3.h"

namespace physics {

struct Quaternion {
    Real w = Real(1);
    Real x = Real(0);
    Real y = Real(0);
    Real z = Real(0);

    // --- Construção ----------------------------------------------------------
    constexpr Quaternion() = default;
    constexpr Quaternion(Real w_, Real x_, Real y_, Real z_) : w(w_), x(x_), y(y_), z(z_) {}
    /// Quatérnio puro (w = 0) a partir de um vetor.
    explicit constexpr Quaternion(const Vector3& v) : w(0), x(v.x), y(v.y), z(v.z) {}

    static constexpr Quaternion identity() { return Quaternion(1, 0, 0, 0); }

    /// Rotação de `angle` radianos em torno de `axis` (não precisa ser unitário;
    /// eixo nulo resulta na identidade).
    static Quaternion fromAxisAngle(const Vector3& axis, Real angle);

    /// Mapa exponencial: vetor cuja direção é o eixo e cujo módulo é o ângulo.
    static Quaternion fromRotationVector(const Vector3& rotation);

    /// Menor rotação que leva a direção `from` em `to`.
    static Quaternion fromTwoVectors(const Vector3& from, const Vector3& to);

    /// Converte uma matriz de rotação (método de Shepperd, estável).
    static Quaternion fromMatrix(const Matrix3& rotation);

    // --- Parte vetorial -----------------------------------------------------------
    constexpr Vector3 vec() const { return Vector3(x, y, z); }

    // --- Álgebra --------------------------------------------------------------------
    constexpr Real dot(const Quaternion& o) const { return w * o.w + x * o.x + y * o.y + z * o.z; }
    constexpr Real normSquared() const { return dot(*this); }
    Real norm() const { return std::sqrt(normSquared()); }

    constexpr Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    /// Inversa geral (conj / |q|²). Retorna identidade se q for degenerado.
    Quaternion inverse() const {
        const Real nsq = normSquared();
        if (!(nsq > kNormalizeEpsilonSq)) return identity();
        const Real inv = Real(1) / nsq;
        return Quaternion(w * inv, -x * inv, -y * inv, -z * inv);
    }

    /// Normaliza in-place. Quatérnios degenerados (norma ~0, NaN, Inf) são
    /// substituídos pela identidade. Retorna a norma original.
    Real normalize() {
        const Real nsq = normSquared();
        if (!(nsq > kNormalizeEpsilonSq) || !std::isfinite(nsq)) {
            *this = identity();
            return Real(0);
        }
        const Real n = std::sqrt(nsq);
        const Real inv = Real(1) / n;
        w *= inv; x *= inv; y *= inv; z *= inv;
        return n;
    }

    Quaternion normalized() const { Quaternion r = *this; r.normalize(); return r; }

    bool isFinite() const {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    // --- Rotação de vetores (assume q unitário) ----------------------------------------
    /// v' = q v q⁻¹, calculado sem montar a matriz:
    ///   t = 2 (u × v);  v' = v + w t + u × t     (u = parte vetorial)
    Vector3 rotate(const Vector3& v) const {
        const Vector3 u(x, y, z);
        const Vector3 t = u.cross(v) * Real(2);
        return v + t * w + u.cross(t);
    }

    /// Rotação inversa: q⁻¹ v q (equivale a conjugate().rotate(v)).
    Vector3 rotateInverse(const Vector3& v) const {
        const Vector3 u(-x, -y, -z);
        const Vector3 t = u.cross(v) * Real(2);
        return v + t * w + u.cross(t);
    }

    // --- Conversões --------------------------------------------------------------------
    /// Matriz de rotação 3x3 (assume q unitário).
    Matrix3 toMatrix() const;

    /// Decompõe em eixo unitário + ângulo em [0, 2π]. Para rotação nula,
    /// devolve eixo (1,0,0) e ângulo 0.
    void toAxisAngle(Vector3& axis, Real& angle) const;

    // --- Integração ---------------------------------------------------------------------
    /// Integra uma velocidade angular `omega` (referencial do MUNDO) por `dt`:
    ///     q ← normalize( q + ½ dt · (0, ω) ⊗ q )
    /// Euler semi-implícito para a orientação (primeira ordem + renormalização).
    void integrate(const Vector3& omega, Real dt) {
        const Quaternion dq = Quaternion(omega) * (*this);
        w += dq.w * Real(0.5) * dt;
        x += dq.x * Real(0.5) * dt;
        y += dq.y * Real(0.5) * dt;
        z += dq.z * Real(0.5) * dt;
        normalize();
    }

    /// Integração EXATA para ω constante em [0, dt] (mapa exponencial, V24):
    ///     q ← exp(½ dt ω) ⊗ q ,   exp(½ dt ω) = (cos(θ/2), sin(θ/2) ω/|ω|),  θ = |ω| dt
    /// A rotação aplicada tem exatamente o ângulo |ω|·dt (Euler gira um pouco a menos/mais: erro O(θ³));
    /// o resultado já tem norma 1 (a renormalização só absorve o erro de arredondamento).
    void integrateExp(const Vector3& omega, Real dt) {
        *this = (fromRotationVector(omega * dt) * (*this)).normalized();
    }

    Quaternion integratedExp(const Vector3& omega, Real dt) const {
        Quaternion r = *this;
        r.integrateExp(omega, dt);
        return r;
    }

    Quaternion integrated(const Vector3& omega, Real dt) const {
        Quaternion r = *this;
        r.integrate(omega, dt);
        return r;
    }

    // --- Interpolação ---------------------------------------------------------------------
    /// Interpolação linear normalizada (caminho mais curto).
    static Quaternion nlerp(const Quaternion& a, const Quaternion& b, Real t);

    /// Interpolação esférica (caminho mais curto; cai para nlerp quando a≈b).
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, Real t);

    // --- Operadores ----------------------------------------------------------------------------
    /// Produto de Hamilton: (a * b) aplica primeiro b e depois a.
    friend constexpr Quaternion operator*(const Quaternion& a, const Quaternion& b) {
        return Quaternion(a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
    }
    Quaternion& operator*=(const Quaternion& o) { return *this = *this * o; }

    friend constexpr Quaternion operator+(const Quaternion& a, const Quaternion& b) {
        return Quaternion(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z);
    }
    friend constexpr Quaternion operator-(const Quaternion& a, const Quaternion& b) {
        return Quaternion(a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z);
    }
    friend constexpr Quaternion operator*(const Quaternion& q, Real s) {
        return Quaternion(q.w * s, q.x * s, q.y * s, q.z * s);
    }
    friend constexpr Quaternion operator*(Real s, const Quaternion& q) { return q * s; }
    constexpr Quaternion operator-() const { return Quaternion(-w, -x, -y, -z); }

    friend constexpr bool operator==(const Quaternion& a, const Quaternion& b) {
        return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z;
    }
    friend constexpr bool operator!=(const Quaternion& a, const Quaternion& b) { return !(a == b); }

    /// Igualdade de rotação (q e -q representam a mesma orientação).
    bool nearlyEqual(const Quaternion& o, Real eps = kCompareEpsilon) const {
        const Real d = std::abs(dot(o));
        return physics::nearlyEqual(d, Real(1), eps);
    }
};

std::ostream& operator<<(std::ostream& os, const Quaternion& q);

} // namespace physics
