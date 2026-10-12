// =============================================================================
// Vector3.cpp
// =============================================================================
#include "physics/math/Vector3.h"

#include <ostream>

namespace physics {

void computeOrthonormalBasis(const Vector3& n, Vector3& t1, Vector3& t2) {
    // Escolhe o eixo de menor componente de n para evitar degeneração:
    // 0.57735 = 1/sqrt(3). Se |n.x| é grande, cruzamos com Z; senão com X.
    constexpr Real kInvSqrt3 = Real(0.57735026918962576451);
    if (std::abs(n.x) >= kInvSqrt3) {
        t1 = Vector3(n.y, -n.x, Real(0));
    } else {
        t1 = Vector3(Real(0), n.z, -n.y);
    }
    // Se n não era unitário/válido, t1 pode sair nulo: cai para um eixo fixo.
    t1 = t1.normalizedOr(Vector3::unitX());
    t2 = n.cross(t1).normalizedOr(Vector3::unitY());
}

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
}

} // namespace physics
