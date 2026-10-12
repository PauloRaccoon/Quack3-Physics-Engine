// =============================================================================
// MassData.cpp
// =============================================================================
#include "physics/dynamics/MassData.h"

#include <stdexcept>

namespace physics {

MassData MassData::fromMassAndInertia(Real mass, const Matrix3& inertia, const Vector3& com) {
    if (!std::isfinite(mass) || !(mass > Real(0))) {
        throw std::invalid_argument("MassData: a massa deve ser positiva e finita");
    }
    if (!inertia.isFinite() || !com.isFinite()) {
        throw std::invalid_argument("MassData: inercia/centro de massa nao finitos");
    }
    MassData d;
    d.mass = mass;
    d.invMass = Real(1) / mass;
    d.centerOfMass = com;
    d.inertia = inertia;
    d.invInertia = inertia.inverse();  // singular → matriz nula (sem rotação)
    return d;
}

MassData MassData::fromShapeMass(const Shape& shape, Real mass) {
    const Real volume = shape.getVolume();
    if (!std::isfinite(volume) || !(volume > Real(0))) {
        throw std::invalid_argument("MassData: volume da forma nulo ou invalido");
    }
    return fromMassAndInertia(mass, shape.computeInertia(mass), shape.getCentroid());
}

MassData MassData::fromShape(const Shape& shape, Real density) {
    if (!std::isfinite(density) || !(density > Real(0))) {
        throw std::invalid_argument("MassData: a densidade deve ser positiva e finita");
    }
    return fromShapeMass(shape, density * shape.getVolume());
}

} // namespace physics
