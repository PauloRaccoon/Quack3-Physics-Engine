// =============================================================================
// JointConstraint.cpp
// =============================================================================
#include "physics/constraints/JointConstraint.h"

#include <cmath>

namespace physics {

namespace {

/// Limita o comprimento de `v` a `maxLen`.
Vector3 clampLength(const Vector3& v, Real maxLen) {
    const Real lsq = v.lengthSquared();
    return lsq > maxLen * maxLen ? v * (maxLen / std::sqrt(lsq)) : v;
}

/// Matriz de massa efetiva 3×3 de uma junta de ponto:
///   K = (1/mA + 1/mB) Id − [rA]× IA⁻¹ [rA]× − [rB]× IB⁻¹ [rB]×
/// e sua inversa (nula se singular: ex. dois corpos sem massa).
Matrix3 pointMassInverse(const detail::BodyState& A, const detail::BodyState& B, const Vector3& rA,
                         const Vector3& rB) {
    const Matrix3 sA = Matrix3::skew(rA), sB = Matrix3::skew(rB);
    const Matrix3 K = Matrix3::identity() * (A.invMass + B.invMass) - sA * A.invI * sA - sB * B.invI * sB;
    Matrix3 inv;
    K.tryInverse(inv);   // falha → matriz nula (sem impulso)
    return inv;
}

} // namespace

// =============================================================================
// BallSocketJoint
// =============================================================================
BallSocketJoint::BallSocketJoint(RigidBody* a, RigidBody* b, const Vector3& worldAnchor)
    : JointConstraint(a, b) {
    localAnchorA_ = a->worldToLocal(worldAnchor);
    localAnchorB_ = b->worldToLocal(worldAnchor);
}

void BallSocketJoint::prepare(const StepContext& ctx) {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    const Vector3 pA = getAnchorA(), pB = getAnchorB();
    rA_ = pA - A.com;
    rB_ = pB - B.com;
    invK_ = pointMassInverse(A, B, rA_, rB_);

    // Erro de posição C = pB − pA → velocidade de correção β/Δt · C.
    bias_ = clampLength((pB - pA) * (ctx.settings.baumgarte * ctx.invDt), ctx.settings.maxBiasVelocity);

    if (ctx.settings.warmStarting && impulse_.isFinite()) {
        A.applyImpulse(-impulse_, rA_);
        B.applyImpulse(impulse_, rB_);
        A.store(*bodyA_);
        B.store(*bodyB_);
    } else {
        impulse_ = Vector3::zero();
    }
}

void BallSocketJoint::solveVelocity() {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    const Vector3 dv = B.velocityAt(rB_) - A.velocityAt(rA_);
    const Vector3 lambda = invK_ * (-(dv + bias_));
    impulse_ += lambda;

    A.applyImpulse(-lambda, rA_);
    B.applyImpulse(lambda, rB_);
    A.store(*bodyA_);
    B.store(*bodyB_);
}

// =============================================================================
// DistanceJoint
// =============================================================================
DistanceJoint::DistanceJoint(RigidBody* a, RigidBody* b, const Vector3& worldAnchorA,
                             const Vector3& worldAnchorB, Real length)
    : JointConstraint(a, b), length_(length) {
    if (!std::isfinite(length)) throw std::invalid_argument("DistanceJoint: comprimento nao finito");
    localAnchorA_ = a->worldToLocal(worldAnchorA);
    localAnchorB_ = b->worldToLocal(worldAnchorB);
    if (length_ < Real(0)) length_ = worldAnchorA.distance(worldAnchorB);
}

void DistanceJoint::prepare(const StepContext& ctx) {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    const Vector3 pA = getAnchorA(), pB = getAnchorB();
    rA_ = pA - A.com;
    rB_ = pB - B.com;

    const Vector3 d = pB - pA;
    const Real len = d.length();
    if (len > kNormalizeEpsilonSq) {
        n_ = d / len;
        mass_ = detail::linearEffectiveMass(A, B, rA_, rB_, n_);
    } else {
        // Pontos coincidentes: direção indefinida → linha desativada neste passo.
        n_ = Vector3::zero();
        mass_ = Real(0);
    }

    const Real C = len - length_;
    bias_ = clamp(ctx.settings.baumgarte * ctx.invDt * C, -ctx.settings.maxBiasVelocity,
                  ctx.settings.maxBiasVelocity);

    if (ctx.settings.warmStarting && std::isfinite(impulse_)) {
        const Vector3 P = n_ * impulse_;
        A.applyImpulse(-P, rA_);
        B.applyImpulse(P, rB_);
        A.store(*bodyA_);
        B.store(*bodyB_);
    } else {
        impulse_ = Real(0);
    }
}

void DistanceJoint::solveVelocity() {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    const Real vn = (B.velocityAt(rB_) - A.velocityAt(rA_)).dot(n_);
    const Real lambda = -mass_ * (vn + bias_);
    impulse_ += lambda;

    const Vector3 P = n_ * lambda;
    A.applyImpulse(-P, rA_);
    B.applyImpulse(P, rB_);
    A.store(*bodyA_);
    B.store(*bodyB_);
}

// =============================================================================
// HingeJoint
// =============================================================================
HingeJoint::HingeJoint(RigidBody* a, RigidBody* b, const Vector3& worldAnchor, const Vector3& worldAxis)
    : JointConstraint(a, b) {
    Vector3 axis = worldAxis;
    if (!axis.isFinite() || axis.normalize() <= Real(0)) {
        throw std::invalid_argument("HingeJoint: eixo nulo ou nao finito");
    }
    localAnchorA_ = a->worldToLocal(worldAnchor);
    localAnchorB_ = b->worldToLocal(worldAnchor);
    localAxisA_ = a->getTransform().inverseTransformDirection(axis);
    localAxisB_ = b->getTransform().inverseTransformDirection(axis);
}

void HingeJoint::prepare(const StepContext& ctx) {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    // --- Parte linear (ponto coincidente) -----------------------------------------------------
    const Vector3 pA = getAnchorA(), pB = getAnchorB();
    rA_ = pA - A.com;
    rB_ = pB - B.com;
    invK_ = pointMassInverse(A, B, rA_, rB_);
    linearBias_ = clampLength((pB - pA) * (ctx.settings.baumgarte * ctx.invDt), ctx.settings.maxBiasVelocity);

    // --- Parte angular (eixos paralelos) --------------------------------------------------------------
    // p1,p2 ⟂ eixoA. Com C_i = eixoB · p_i (≈ ângulo de desalinhamento), a derivada é
    //   dC_i/dt = (ωB − ωA) · q_i,   q_i = eixoA × p_i.
    const Vector3 axisA = getAxisA();
    const Vector3 axisB = getAxisB();
    Vector3 p[2];
    computeOrthonormalBasis(axisA, p[0], p[1]);
    const Matrix3 invISum = A.invI + B.invI;
    for (int i = 0; i < 2; ++i) {
        q_[i] = axisA.cross(p[i]);
        angularMass_[i] = safeInverse(q_[i].dot(invISum * q_[i]));
        const Real C = axisB.dot(p[i]);
        angularBias_[i] = clamp(ctx.settings.baumgarte * ctx.invDt * C, -ctx.settings.maxBiasVelocity,
                                ctx.settings.maxBiasVelocity);
    }

    // --- Warm start ---------------------------------------------------------------------------------------
    if (ctx.settings.warmStarting && linearImpulse_.isFinite() && angularImpulse_.isFinite()) {
        // O torque acumulado é reprojetado nas direções q_i atuais.
        const Real a0 = angularImpulse_.dot(q_[0]);
        const Real a1 = angularImpulse_.dot(q_[1]);
        angularImpulse_ = q_[0] * a0 + q_[1] * a1;

        A.applyImpulse(-linearImpulse_, rA_);
        B.applyImpulse(linearImpulse_, rB_);
        A.applyAngularImpulse(-angularImpulse_);
        B.applyAngularImpulse(angularImpulse_);
        A.store(*bodyA_);
        B.store(*bodyB_);
    } else {
        linearImpulse_ = Vector3::zero();
        angularImpulse_ = Vector3::zero();
    }
}

void HingeJoint::solveVelocity() {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    // --- Angular: 2 linhas escalares -------------------------------------------------------------------------
    for (int i = 0; i < 2; ++i) {
        const Real jv = q_[i].dot(B.w - A.w);
        const Real lambda = -angularMass_[i] * (jv + angularBias_[i]);
        const Vector3 L = q_[i] * lambda;
        angularImpulse_ += L;
        A.applyAngularImpulse(-L);
        B.applyAngularImpulse(L);
    }

    // --- Linear: 3 linhas acopladas --------------------------------------------------------------------------------
    const Vector3 dv = B.velocityAt(rB_) - A.velocityAt(rA_);
    const Vector3 lambda = invK_ * (-(dv + linearBias_));
    linearImpulse_ += lambda;
    A.applyImpulse(-lambda, rA_);
    B.applyImpulse(lambda, rB_);

    A.store(*bodyA_);
    B.store(*bodyB_);
}

} // namespace physics
