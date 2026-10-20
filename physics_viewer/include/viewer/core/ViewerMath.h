// =============================================================================
// ViewerMath.h
// Matemática do viewer SEM dependência de raylib (testável sem janela).
//
// A peça central é toMat4(): transforma a pose do motor (posição + QUATÉRNIO) na
// matriz 4x4 que a placa de vídeo consome. O quatérnio continua sendo o estado da
// física; a matriz existe só no momento de desenhar. A conversão usa o
// Quaternion::toMatrix() do PRÓPRIO motor, não o da raylib.
//
// Convenções (iguais às do raylib::Matrix, campos m0..m15):
//   - ordem de COLUNAS: o elemento (linha r, coluna c) está em m[c*4 + r];
//   - vetor coluna: p' = M * (x, y, z, 1); a translação fica em m[12], m[13], m[14];
//   - mão direita, Y para cima (a mesma do motor).
// =============================================================================
#pragma once

#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

namespace viewer {

/// Vetor em float (o que a raylib usa), independente da precisão do motor (float ou double).
struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/// Matriz 4x4 em ordem de colunas (mesma disposição de memória de raylib::Matrix).
struct Mat4f {
    float m[16] = {1, 0, 0, 0,   0, 1, 0, 0,   0, 0, 1, 0,   0, 0, 0, 1};
};

inline Vec3f toVec3f(const physics::Vector3& v) {
    return Vec3f{static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

/// Pose do motor → matriz de modelo. Supõe quatérnio UNITÁRIO (o do motor sempre é).
Mat4f toMat4(const physics::Transform& xf);

/// Aplica a matriz a um ponto (w = 1). Usado nos testes e para desenhar os eixos locais.
Vec3f transformPoint(const Mat4f& mat, const Vec3f& p);

/// Coluna `c` (0 = eixo X local, 1 = Y, 2 = Z) da parte de rotação: o eixo do corpo no mundo.
Vec3f axisOf(const Mat4f& mat, int c);

} // namespace viewer
