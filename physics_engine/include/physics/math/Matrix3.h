// =============================================================================
// Matrix3.h
// Matriz 3x3 armazenada em ordem de linhas (m[linha][coluna]).
// Convenção: vetores-coluna, logo  v' = M * v.
// Usada para tensores de inércia, rotações e matrizes antissimétricas.
// =============================================================================
#pragma once

#include <iosfwd>

#include "physics/common/Settings.h"
#include "physics/math/MathUtils.h"
#include "physics/math/Vector3.h"

namespace physics {

struct Matrix3 {
    Real m[3][3];

    // --- Construção ----------------------------------------------------------
    /// Construtor padrão: matriz identidade.
    Matrix3() : m{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {}

    /// Elementos em ordem de linhas.
    Matrix3(Real m00, Real m01, Real m02,
            Real m10, Real m11, Real m12,
            Real m20, Real m21, Real m22)
        : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

    static Matrix3 identity() { return Matrix3(); }
    static Matrix3 zero() { return Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0); }

    static Matrix3 diagonal(Real a, Real b, Real c) { return Matrix3(a, 0, 0, 0, b, 0, 0, 0, c); }
    static Matrix3 diagonal(const Vector3& d) { return diagonal(d.x, d.y, d.z); }

    static Matrix3 fromRows(const Vector3& r0, const Vector3& r1, const Vector3& r2) {
        return Matrix3(r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z);
    }
    static Matrix3 fromColumns(const Vector3& c0, const Vector3& c1, const Vector3& c2) {
        return Matrix3(c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z);
    }

    /// Matriz antissimétrica [v]x tal que [v]x * w = v × w.
    static Matrix3 skew(const Vector3& v) {
        return Matrix3(0, -v.z, v.y,
                       v.z, 0, -v.x,
                       -v.y, v.x, 0);
    }

    /// Produto externo a ⊗ b = a * bᵀ.
    static Matrix3 outer(const Vector3& a, const Vector3& b) {
        return fromColumns(a * b.x, a * b.y, a * b.z);
    }

    // --- Acesso ----------------------------------------------------------------
    Real& operator()(int r, int c)             { return m[r][c]; }
    const Real& operator()(int r, int c) const { return m[r][c]; }

    Vector3 row(int r) const { return Vector3(m[r][0], m[r][1], m[r][2]); }
    Vector3 column(int c) const { return Vector3(m[0][c], m[1][c], m[2][c]); }

    void setRow(int r, const Vector3& v) { m[r][0] = v.x; m[r][1] = v.y; m[r][2] = v.z; }
    void setColumn(int c, const Vector3& v) { m[0][c] = v.x; m[1][c] = v.y; m[2][c] = v.z; }

    // --- Operações matriciais -----------------------------------------------------
    Matrix3 transposed() const {
        return Matrix3(m[0][0], m[1][0], m[2][0],
                       m[0][1], m[1][1], m[2][1],
                       m[0][2], m[1][2], m[2][2]);
    }

    Real trace() const { return m[0][0] + m[1][1] + m[2][2]; }

    Real determinant() const { return row(0).dot(row(1).cross(row(2))); }

    /// Tenta inverter. Retorna false (e deixa `out` = zero) se a matriz for
    /// singular ou mal condicionada (determinante pequeno em relação à escala).
    bool tryInverse(Matrix3& out) const;

    /// Inversa segura: retorna a matriz nula se singular. Para tensores de
    /// inércia isso equivale a "inércia infinita" (corpo que não rotaciona).
    Matrix3 inverse() const;

    /// Mᵀ * v (evita construir a transposta).
    Vector3 transformTranspose(const Vector3& v) const {
        return Vector3(m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
                       m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
                       m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z);
    }

    /// Transforma um tensor de inércia de corpo para mundo:  R * I * Rᵀ.
    static Matrix3 rotateTensor(const Matrix3& tensor, const Matrix3& rotation) {
        return rotation * tensor * rotation.transposed();
    }

    bool isFinite() const {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (!std::isfinite(m[r][c])) return false;
        return true;
    }

    bool nearlyEqual(const Matrix3& o, Real eps = kCompareEpsilon) const {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (!physics::nearlyEqual(m[r][c], o.m[r][c], eps)) return false;
        return true;
    }

    // --- Operadores compostos --------------------------------------------------------
    Matrix3& operator+=(const Matrix3& o) {
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m[r][c] += o.m[r][c];
        return *this;
    }
    Matrix3& operator-=(const Matrix3& o) {
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m[r][c] -= o.m[r][c];
        return *this;
    }
    Matrix3& operator*=(Real s) {
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m[r][c] *= s;
        return *this;
    }
    /// Divisão segura por escalar (divisor ~0 → matriz nula).
    Matrix3& operator/=(Real s) { return *this *= safeInverse(s); }

    // --- Operadores binários -----------------------------------------------------------
    friend Matrix3 operator+(Matrix3 a, const Matrix3& b) { return a += b; }
    friend Matrix3 operator-(Matrix3 a, const Matrix3& b) { return a -= b; }
    friend Matrix3 operator*(Matrix3 a, Real s) { return a *= s; }
    friend Matrix3 operator*(Real s, Matrix3 a) { return a *= s; }
    friend Matrix3 operator/(Matrix3 a, Real s) { return a /= s; }
    Matrix3 operator-() const { return *this * Real(-1); }

    /// Matriz * vetor.
    friend Vector3 operator*(const Matrix3& a, const Vector3& v) {
        return Vector3(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                       a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                       a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
    }

    /// Matriz * matriz.
    friend Matrix3 operator*(const Matrix3& a, const Matrix3& b) {
        Matrix3 r = Matrix3::zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        return r;
    }
};

/// Funções livres equivalentes aos métodos (estilo pedido na especificação).
inline Matrix3 transpose(const Matrix3& a) { return a.transposed(); }
inline Matrix3 inverse(const Matrix3& a) { return a.inverse(); }

std::ostream& operator<<(std::ostream& os, const Matrix3& a);

} // namespace physics
