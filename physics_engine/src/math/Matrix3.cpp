// =============================================================================
// Matrix3.cpp
// =============================================================================
#include "physics/math/Matrix3.h"

#include <ostream>

namespace physics {

bool Matrix3::tryInverse(Matrix3& out) const {
    // Inversa por cofatores usando produtos vetoriais das linhas:
    //   A⁻¹ = [ r1×r2 | r2×r0 | r0×r1 ] / det(A)   (vetores como COLUNAS)
    const Vector3 r0 = row(0), r1 = row(1), r2 = row(2);
    const Vector3 c0 = r1.cross(r2);
    const Vector3 c1 = r2.cross(r0);
    const Vector3 c2 = r0.cross(r1);
    const Real det = r0.dot(c0);

    // Singularidade RELATIVA: compara |det| ao volume do paralelepípedo
    // formado por linhas ortogonais de mesmo comprimento. Funciona tanto para
    // tensores de inércia minúsculos quanto enormes.
    const Real scale = r0.length() * r1.length() * r2.length();
    if (!std::isfinite(det) || !(scale > Real(0)) || std::abs(det) <= kSingularEpsilon * scale) {
        out = Matrix3::zero();
        return false;
    }

    out = Matrix3::fromColumns(c0, c1, c2) * (Real(1) / det);
    return true;
}

Matrix3 Matrix3::inverse() const {
    Matrix3 r;
    tryInverse(r);  // em caso de falha, r já é a matriz nula
    return r;
}

std::ostream& operator<<(std::ostream& os, const Matrix3& a) {
    for (int r = 0; r < 3; ++r) {
        os << (r == 0 ? "[[" : " [") << a.m[r][0] << ", " << a.m[r][1] << ", " << a.m[r][2]
           << (r == 2 ? "]]" : "]\n");
    }
    return os;
}

} // namespace physics
