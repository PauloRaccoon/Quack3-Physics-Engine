// =============================================================================
// JointConstraint.cpp
// =============================================================================
#include "physics/constraints/JointConstraint.h"

#include <cmath>

namespace physics {

namespace {

/// Limita o comprimento de `v` a `maxLen`.
Vector3 clampLength(const Vector3& v, Real maxLen) { return v.clampedLength(maxLen); }

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
    // (Com NGS, positionIterations > 0, o erro é tratado em solvePosition: sem bias de velocidade.)
    bias_ = ctx.settings.positionIterations > 0
                ? Vector3::zero()
                : clampLength((pB - pA) * (ctx.settings.baumgarte * ctx.invDt), ctx.settings.maxBiasVelocity);

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
    if (!std::isfinite(length) || length > kMaxCoordinate) {
        throw std::invalid_argument("DistanceJoint: comprimento nao finito ou maior que kMaxCoordinate");
    }
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
    bias_ = ctx.settings.positionIterations > 0
                ? Real(0)
                : clamp(ctx.settings.baumgarte * ctx.invDt * C, -ctx.settings.maxBiasVelocity,
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
    linearBias_ = ctx.settings.positionIterations > 0
                      ? Vector3::zero()
                      : clampLength((pB - pA) * (ctx.settings.baumgarte * ctx.invDt), ctx.settings.maxBiasVelocity);

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
        angularBias_[i] = ctx.settings.positionIterations > 0
                              ? Real(0)
                              : clamp(ctx.settings.baumgarte * ctx.invDt * C, -ctx.settings.maxBiasVelocity,
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

// =============================================================================
// NGS: correção de posição das juntas (depois da integração das posições)
// =============================================================================
namespace {

/// Correção de ponto: move os corpos para que as ancoragens pA (em A) e pB (em B) coincidam.
/// Usa o erro inteiro (limitado), pois o passe converge sem precisar do fator β dos contatos.
/// Retorna |pB − pA| medido antes da correção.
Real correctPointError(RigidBody& bodyA, RigidBody& bodyB, const Vector3& localAnchorA,
                       const Vector3& localAnchorB, const SolverSettings& cfg) {
    const detail::BodyState A(bodyA), B(bodyB);
    const Vector3 pA = bodyA.localToWorld(localAnchorA), pB = bodyB.localToWorld(localAnchorB);
    const Vector3 C = pB - pA;
    const Real err = C.length();
    if (!std::isfinite(err) || !(err > Real(0))) return std::isfinite(err) ? err : Real(0);

    const Vector3 rA = pA - A.com, rB = pB - B.com;
    const Matrix3 invK = pointMassInverse(A, B, rA, rB);
    const Vector3 P = invK * (-clampLength(C, cfg.maxLinearCorrection));
    if (!P.isFinite()) return err;

    detail::movePose(bodyA, -P * A.invMass, A.invI * rA.cross(-P));
    detail::movePose(bodyB, P * B.invMass, B.invI * rB.cross(P));
    return err;
}

} // namespace

Real BallSocketJoint::solvePosition(const SolverSettings& cfg) {
    return correctPointError(*bodyA_, *bodyB_, localAnchorA_, localAnchorB_, cfg);
}

Real DistanceJoint::solvePosition(const SolverSettings& cfg) {
    const detail::BodyState A(*bodyA_), B(*bodyB_);
    const Vector3 pA = getAnchorA(), pB = getAnchorB();
    const Vector3 d = pB - pA;
    const Real len = d.length();
    if (!std::isfinite(len) || len <= kNormalizeEpsilonSq) return Real(0);   // direção indefinida

    const Real C = len - length_;
    const Vector3 n = d / len;
    const Vector3 rA = pA - A.com, rB = pB - B.com;
    const Real mass = detail::linearEffectiveMass(A, B, rA, rB, n);
    const Real lambda = -mass * clamp(C, -cfg.maxLinearCorrection, cfg.maxLinearCorrection);
    if (!std::isfinite(lambda)) return std::abs(C);

    const Vector3 P = n * lambda;
    detail::movePose(*bodyA_, -P * A.invMass, A.invI * rA.cross(-P));
    detail::movePose(*bodyB_, P * B.invMass, B.invI * rB.cross(P));
    return std::abs(C);
}

Real HingeJoint::solvePosition(const SolverSettings& cfg) {
    // --- Angular: eixos paralelos (as 2 linhas somadas e aplicadas de uma vez) ----------------------
    Real angularErr = Real(0);
    {
        const detail::BodyState A(*bodyA_), B(*bodyB_);
        const Vector3 axisA = getAxisA(), axisB = getAxisB();
        Vector3 p[2];
        computeOrthonormalBasis(axisA, p[0], p[1]);
        const Matrix3 invISum = A.invI + B.invI;
        Vector3 L;   // "impulso" angular de posição total (mundo), A→B
        for (int i = 0; i < 2; ++i) {
            const Vector3 q = axisA.cross(p[i]);
            const Real mass = safeInverse(q.dot(invISum * q));
            const Real C = axisB.dot(p[i]);                   // ≈ ângulo de desalinhamento
            angularErr = std::max(angularErr, std::abs(C));
            L += q * (-mass * clamp(C, -cfg.maxAngularCorrection, cfg.maxAngularCorrection));
        }
        if (L.isFinite() && L.lengthSquared() > Real(0)) {
            detail::movePose(*bodyA_, Vector3::zero(), A.invI * (-L));
            detail::movePose(*bodyB_, Vector3::zero(), B.invI * L);
        }
    }
    // --- Linear: ancoragens coincidentes (com as poses já giradas) ------------------------------------
    const Real linearErr = correctPointError(*bodyA_, *bodyB_, localAnchorA_, localAnchorB_, cfg);
    return std::max(linearErr, angularErr);
}

} // namespace physics
