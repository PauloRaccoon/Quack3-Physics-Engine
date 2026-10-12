// =============================================================================
// Shape.cpp
// =============================================================================
#include "physics/geometry/Shape.h"

namespace physics {

AABB Shape::computeAABB(const Transform& xf) const {
    // Para cada eixo do mundo, o ponto de suporte em +e e -e define o limite
    // superior/inferior do AABB. Exato para formas convexas.
    AABB box;
    for (int axis = 0; axis < 3; ++axis) {
        Vector3 e;
        e[axis] = Real(1);
        box.max[axis] = getWorldSupport(xf, e)[axis];
        box.min[axis] = getWorldSupport(xf, -e)[axis];
    }
    return box;
}

} // namespace physics
