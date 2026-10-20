// =============================================================================
// RigidBody.cpp
// =============================================================================
#include "physics/dynamics/RigidBody.h"

#include <stdexcept>

namespace physics {

RigidBody::RigidBody(const RigidBodyDesc& desc)
    : shape_(desc.shape),
      friction_(clamp(desc.friction, Real(0), kMaxFriction)),
      restitution_(clamp(desc.restitution, Real(0), Real(1))),
      linearDamping_(desc.linearDamping < Real(0) ? Real(0) : desc.linearDamping),
      angularDamping_(desc.angularDamping < Real(0) ? Real(0) : desc.angularDamping),
      gravityScale_(desc.gravityScale),
      type_(desc.type),
      userData_(desc.userData),
      awake_(desc.type != BodyType::Static),
      allowSleep_(desc.allowSleep) {
    if (!desc.transform.isFinite() || !desc.linearVelocity.isFinite() ||
        !desc.angularVelocity.isFinite()) {
        throw std::invalid_argument("RigidBody: transformacao/velocidades nao finitas");
    }
    if (!desc.transform.position.isWithin(kMaxCoordinate)) {
        throw std::invalid_argument("RigidBody: posicao fora de [-kMaxCoordinate, kMaxCoordinate]");
    }
    // NaN/Inf passariam pelos clamps acima e contaminariam a simulação: recusa a descrição.
    if (!isFinite(desc.friction) || !isFinite(desc.restitution) || !isFinite(desc.linearDamping) ||
        !isFinite(desc.angularDamping) || !isFinite(desc.gravityScale)) {
        throw std::invalid_argument("RigidBody: atrito/restituicao/amortecimento/gravityScale nao finitos");
    }

    // --- Massa --------------------------------------------------------------------
    if (type_ == BodyType::Dynamic) {
        if (desc.massData) {
            massData_ = *desc.massData;
        } else if (shape_) {
            massData_ = desc.mass > Real(0) ? MassData::fromShapeMass(*shape_, desc.mass)
                                            : MassData::fromShape(*shape_, desc.density);
        } else {
            throw std::invalid_argument("RigidBody: corpo dinamico precisa de forma ou massData");
        }
    } else if (shape_) {
        // Estático/cinemático: massa infinita, mas o centro de massa continua
        // sendo o centroide da forma (mantém a origem do frame consistente).
        massData_ = MassData::infinite();
        massData_.centerOfMass = shape_->getCentroid();
    }
    invMass_ = massData_.invMass;

    // --- Pose e velocidades --------------------------------------------------------------
    orientation_ = desc.transform.rotation.normalized();
    position_ = desc.transform.position + orientation_.rotate(massData_.centerOfMass);
    if (type_ != BodyType::Static) {
        linearVelocity_ = desc.linearVelocity;
        angularVelocity_ = desc.angularVelocity;
    }
    updateDerived();
}

void RigidBody::updateDerived() {
    const Matrix3 R = orientation_.toMatrix();
    invInertiaWorld_ = isDynamic() ? Matrix3::rotateTensor(massData_.invInertia, R) : Matrix3::zero();
    transform_.rotation = orientation_;
    transform_.position = position_ - orientation_.rotate(massData_.centerOfMass);
}

void RigidBody::setTransform(const Transform& frame) {
    if (!frame.isFinite() || !frame.position.isWithin(kMaxCoordinate)) return;
    const Quaternion q = frame.rotation.normalized();
    setPose(frame.position + q.rotate(massData_.centerOfMass), q);
}

void RigidBody::setPose(const Vector3& worldCenterOfMass, const Quaternion& orientation) {
    if (!worldCenterOfMass.isWithin(kMaxCoordinate) || !orientation.isFinite()) return;
    wake();
    position_ = worldCenterOfMass;
    orientation_ = orientation.normalized();
    updateDerived();
}

void RigidBody::setMassData(const MassData& data) {
    if (!isDynamic()) return;
    // MassData montada à mão pode trazer NaN/Inf ou valores absurdos: ignora (estado intacto).
    if (!isFinite(data.mass) || !isFinite(data.invMass) || data.mass < Real(0) || data.invMass < Real(0) ||
        !data.centerOfMass.isWithin(kMaxCoordinate) || !data.inertia.isFinite() || !data.invInertia.isFinite()) {
        return;
    }
    // Mantém o frame do corpo fixo: o centro de massa pode mudar de lugar.
    const Transform frame = transform_;
    massData_ = data;
    invMass_ = data.invMass;
    setTransform(frame);   // (acorda o corpo)
}

void RigidBody::setAwake(bool awake) {
    if (type_ != BodyType::Dynamic) return;
    if (awake) {
        wake();
        return;
    }
    awake_ = false;
    sleepTime_ = Real(0);
    linearVelocity_ = Vector3::zero();
    angularVelocity_ = Vector3::zero();
    clearForces();
}

Real RigidBody::getKineticEnergy() const {
    const Vector3 wb = orientation_.rotateInverse(angularVelocity_);
    return Real(0.5) * getMass() * linearVelocity_.lengthSquared() +
           Real(0.5) * wb.dot(massData_.inertia * wb);
}

Vector3 RigidBody::getAngularMomentum() const {
    const Vector3 wb = orientation_.rotateInverse(angularVelocity_);
    return orientation_.rotate(massData_.inertia * wb);
}

} // namespace physics
