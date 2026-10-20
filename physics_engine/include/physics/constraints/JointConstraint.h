// =============================================================================
// JointConstraint.h
// Juntas bilaterais (igualdade, sem limite de impulso) entre dois corpos.
// Os pontos de ancoragem ficam em coordenadas LOCAIS ao frame de cada corpo,
// então acompanham o corpo automaticamente. O erro de posição é corrigido por NGS
// (solvePosition, depois da integração, movendo as poses sem criar velocidade) ou, com
// SolverSettings::positionIterations == 0, por Baumgarte (β/Δt · C) como velocidade de bias; os
// impulsos acumulados são reinjetados (warm start) a cada quadro.
//
//  - BallSocketJoint : 3 linhas — os dois pontos de ancoragem coincidem
//                      (articulação esférica, 3 graus de liberdade angulares).
//  - DistanceJoint   : 1 linha  — a distância entre os pontos é constante
//                      (haste rígida / pêndulo).
//  - HingeJoint      : 5 linhas — esfera-soquete + alinhamento dos eixos
//                      (dobradiça: só gira em torno do eixo comum).
// =============================================================================
#pragma once

#include "physics/constraints/Constraint.h"

namespace physics {

class JointConstraint : public Constraint {
public:
    const Vector3& getLocalAnchorA() const { return localAnchorA_; }
    const Vector3& getLocalAnchorB() const { return localAnchorB_; }
    Vector3 getAnchorA() const { return bodyA_->localToWorld(localAnchorA_); }
    Vector3 getAnchorB() const { return bodyB_->localToWorld(localAnchorB_); }

    /// Se false, o mundo deve ignorar colisões entre os dois corpos ligados.
    bool getCollideConnected() const { return collideConnected_; }
    void setCollideConnected(bool c) { collideConnected_ = c; }

protected:
    JointConstraint(RigidBody* a, RigidBody* b) : Constraint(a, b) {}

    Vector3 localAnchorA_;
    Vector3 localAnchorB_;
    bool collideConnected_ = false;
};

// -----------------------------------------------------------------------------
class BallSocketJoint final : public JointConstraint {
public:
    /// Ancora os dois corpos no ponto `worldAnchor` (mundo), na pose atual.
    BallSocketJoint(RigidBody* a, RigidBody* b, const Vector3& worldAnchor);

    ConstraintType getType() const override { return ConstraintType::BallSocket; }
    void prepare(const StepContext& ctx) override;
    void solveVelocity() override;
    /// NGS: aproxima os dois pontos de ancoragem (erro = |pB − pA|, em metros).
    Real solvePosition(const SolverSettings& settings) override;

    const Vector3& getAccumulatedImpulse() const { return impulse_; }

private:
    Vector3 rA_, rB_;      ///< ancoragens relativas ao centro de massa
    Matrix3 invK_;         ///< inversa da matriz de massa efetiva 3×3
    Vector3 bias_;         ///< velocidade-alvo de correção de posição
    Vector3 impulse_;      ///< impulso acumulado (mundo)
};

// -----------------------------------------------------------------------------
class DistanceJoint final : public JointConstraint {
public:
    /// Mantém a distância entre `worldAnchorA` (em A) e `worldAnchorB` (em B).
    /// `length < 0` usa a distância atual entre os pontos.
    /// @throws std::invalid_argument se length for NaN/Inf.
    DistanceJoint(RigidBody* a, RigidBody* b, const Vector3& worldAnchorA,
                  const Vector3& worldAnchorB, Real length = Real(-1));

    ConstraintType getType() const override { return ConstraintType::Distance; }
    void prepare(const StepContext& ctx) override;
    void solveVelocity() override;
    /// NGS: corrige o comprimento (erro = |distância − comprimento|, em metros).
    Real solvePosition(const SolverSettings& settings) override;

    Real getLength() const { return length_; }
    Real getAccumulatedImpulse() const { return impulse_; }

private:
    Real length_;
    Vector3 n_;            ///< direção A→B atual
    Vector3 rA_, rB_;
    Real mass_ = 0;
    Real bias_ = 0;
    Real impulse_ = 0;
};

// -----------------------------------------------------------------------------
class HingeJoint final : public JointConstraint {
public:
    /// Dobradiça no ponto `worldAnchor`, com eixo `worldAxis` (não precisa ser unitário).
    /// @throws std::invalid_argument se o eixo for nulo/não finito.
    HingeJoint(RigidBody* a, RigidBody* b, const Vector3& worldAnchor, const Vector3& worldAxis);

    ConstraintType getType() const override { return ConstraintType::Hinge; }
    void prepare(const StepContext& ctx) override;
    void solveVelocity() override;
    /// NGS: ancoragens coincidentes + eixos paralelos. Erro = o maior entre o linear [m] e o
    /// angular [rad].
    Real solvePosition(const SolverSettings& settings) override;

    /// Eixo da dobradiça no mundo, segundo cada corpo (coincidem se a junta está satisfeita).
    Vector3 getAxisA() const { return bodyA_->getTransform().transformDirection(localAxisA_); }
    Vector3 getAxisB() const { return bodyB_->getTransform().transformDirection(localAxisB_); }

private:
    Vector3 localAxisA_, localAxisB_;

    // Parte linear (ponto): idêntica à BallSocketJoint.
    Vector3 rA_, rB_;
    Matrix3 invK_;
    Vector3 linearBias_;
    Vector3 linearImpulse_;

    // Parte angular: 2 linhas q_i·(ωB − ωA) = −β/Δt · C_i.
    Vector3 q_[2];
    Real angularMass_[2] = {0, 0};
    Real angularBias_[2] = {0, 0};
    Vector3 angularImpulse_;   ///< torque×tempo acumulado (mundo), ⟂ ao eixo
};

} // namespace physics
