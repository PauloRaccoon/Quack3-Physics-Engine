// =============================================================================
// Transform.cpp
// =============================================================================
#include "physics/math/Transform.h"

#include <ostream>

namespace physics {

Transform Transform::interpolate(const Transform& a, const Transform& b, Real alpha) {
    const Real t = clamp(alpha, Real(0), Real(1));
    return Transform(Vector3::lerp(a.position, b.position, t),
                     Quaternion::slerp(a.rotation, b.rotation, t));
}

std::ostream& operator<<(std::ostream& os, const Transform& t) {
    return os << "Transform{pos=" << t.position << ", rot=" << t.rotation << '}';
}

} // namespace physics
