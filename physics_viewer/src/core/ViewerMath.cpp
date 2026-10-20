// =============================================================================
// ViewerMath.cpp
// =============================================================================
#include "viewer/core/ViewerMath.h"

#include "physics/math/Matrix3.h"

namespace viewer {

Mat4f toMat4(const physics::Transform& xf) {
    // Rotação: a matriz 3x3 vem do quatérnio, pelo código do motor (R tal que p' = R·p).
    const physics::Matrix3 r = xf.rotation.toMatrix();

    Mat4f out;
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            out.m[col * 4 + row] = static_cast<float>(r(row, col));   // ordem de colunas
        }
        out.m[col * 4 + 3] = 0.0f;
    }
    // Translação na quarta coluna.
    out.m[12] = static_cast<float>(xf.position.x);
    out.m[13] = static_cast<float>(xf.position.y);
    out.m[14] = static_cast<float>(xf.position.z);
    out.m[15] = 1.0f;
    return out;
}

Vec3f transformPoint(const Mat4f& a, const Vec3f& p) {
    return Vec3f{a.m[0] * p.x + a.m[4] * p.y + a.m[8] * p.z + a.m[12],
                 a.m[1] * p.x + a.m[5] * p.y + a.m[9] * p.z + a.m[13],
                 a.m[2] * p.x + a.m[6] * p.y + a.m[10] * p.z + a.m[14]};
}

Vec3f axisOf(const Mat4f& mat, int c) {
    if (c < 0) c = 0;
    if (c > 2) c = 2;
    return Vec3f{mat.m[c * 4 + 0], mat.m[c * 4 + 1], mat.m[c * 4 + 2]};
}

} // namespace viewer
