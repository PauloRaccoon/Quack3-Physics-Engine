// =============================================================================
// Constraint.h
// Base de todas as restrições do solver de impulsos sequenciais.
//
// Cada restrição é uma ou mais linhas  J v = bias  sobre as velocidades de dois
// corpos, onde a Jacobiana J = [ J_vA  J_wA  J_vB  J_wB ] mapeia as velocidades
// (linear/angular de A e B) numa velocidade de restrição. Resolver uma linha é
//
//        λ = −m_eff · (J v − bias)        m_eff = 1 / (J M⁻¹ Jᵀ)
//
// e o impulso  P = Jᵀ λ  é aplicado aos corpos (Δv = M⁻¹ P). Os impulsos
// ACUMULADOS (soma dos λ ao longo das iterações) é que são limitados (≥ 0 para
// contatos, ±μ·λn para atrito); isso é o que torna a convergência de
// Gauss–Seidel correta ("clamping do impulso acumulado", Catto 2005).
//
// Ciclo de vida por passo (chamado pelo SequentialImpulseSolver):
//   prepare()       → Jacobianas, massas efetivas, bias, warm start
//   solveVelocity() → uma iteração de Gauss–Seidel (repetida N vezes)
//   finish()        → devolve os impulsos acumulados para a persistência
// =============================================================================
#pragma once

#include <stdexcept>

#include "physics/common/Settings.h"
#include "physics/dynamics/RigidBody.h"
#include "physics/math/Matrix3.h"
#include "physics/math/Vector3.h"

namespace physics {

/// Parâmetros do solver (ajustáveis em tempo de execução).
struct SolverSettings {
    int velocityIterations = kVelocityIterations;
    Real baumgarte = kBaumgarteFactor;     ///< fração do erro de posição corrigida por passo (β)
    /// Correção de posição. > 0: NGS (Gauss–Seidel não linear): depois da integração, um passe
    /// separado (SequentialImpulseSolver::solvePositions) move as POSES direto, sem criar
    /// velocidade, então NÃO injeta energia. 0: Baumgarte legado (a correção entra no bias de
    /// velocidade e vira momento: tira energia de uma pilha parada e uma torre de 10 caixas com
    /// 10 iterações acaba tombando).
    int positionIterations = kPositionIterations;
    Real maxLinearCorrection = kMaxLinearCorrection;     ///< teto da correção linear por iteração [m]
    Real maxAngularCorrection = kMaxAngularCorrection;   ///< teto da correção angular por iteração [rad]
    Real linearSlop = kLinearSlop;         ///< penetration slop: penetração tolerada sem correção [m]
    Real velocitySlop = kVelocitySlop;     ///< velocity slop: abaixo disso não há restituição [m/s]
    Real maxBiasVelocity = Real(4);        ///< teto da velocidade de correção de posição [m/s]
    bool warmStarting = true;              ///< reinjeta os impulsos do quadro anterior
};

/// Dados do passo atual entregues a prepare().
struct StepContext {
    Real dt = kFixedTimeStep;
    Real invDt = Real(1) / kFixedTimeStep;
    SolverSettings settings;
};

enum class ConstraintType : std::uint8_t { Contact, BallSocket, Distance, Hinge };

namespace detail {

/// Cópia local das velocidades e propriedades de massa de um corpo. As
/// restrições trabalham sobre esta cópia (registradores/cache) e gravam o
/// resultado no corpo uma única vez, no fim da iteração.
struct BodyState {
    Vector3 v;
    Vector3 w;
    Matrix3 invI;
    Vector3 com;
    Real invMass;
    Vector3 dv;   ///< variação de velocidade linear deste passo (forças/gravidade; ver RigidBody)
    Vector3 dw;   ///< idem angular

    explicit BodyState(const RigidBody& b)
        : v(b.getLinearVelocity()), w(b.getAngularVelocity()), invI(b.getInvInertiaWorld()),
          com(b.getWorldCenterOfMass()), invMass(b.getInvMass()),
          dv(b.getStepLinearDelta()), dw(b.getStepAngularDelta()) {}

    /// Velocidade do ponto à distância `r` do centro de massa.
    Vector3 velocityAt(const Vector3& r) const { return v + w.cross(r); }

    /// Parte da velocidade do ponto que veio das forças/gravidade deste passo.
    Vector3 stepDeltaAt(const Vector3& r) const { return dv + dw.cross(r); }

    /// Impulso `P` aplicado no ponto à distância `r` do centro de massa.
    void applyImpulse(const Vector3& P, const Vector3& r) {
        v += P * invMass;
        w += invI * r.cross(P);
    }
    /// Impulso angular puro (torque × tempo).
    void applyAngularImpulse(const Vector3& L) { w += invI * L; }

    /// Grava no corpo (corpos estáticos ignoram; cinemáticos têm invMass = 0 e não mudam).
    void store(RigidBody& b) const {
        b.setLinearVelocity(v);
        b.setAngularVelocity(w);
    }
};

/// Move a POSE do corpo (não a velocidade): translada o centro de massa por `dx` e gira pelo
/// vetor de rotação `dtheta` (ângulos pequenos). Corpos não dinâmicos não se movem.
inline void movePose(RigidBody& b, const Vector3& dx, const Vector3& dtheta) {
    if (!b.isDynamic()) return;
    b.setPose(b.getWorldCenterOfMass() + dx, b.getOrientation().integrated(dtheta, Real(1)));
}

/// Massa efetiva de uma linha com direção linear `n` aplicada nos braços rA, rB.
inline Real linearEffectiveMass(const BodyState& A, const BodyState& B, const Vector3& rA,
                                const Vector3& rB, const Vector3& n) {
    const Vector3 raN = rA.cross(n), rbN = rB.cross(n);
    const Real k = A.invMass + B.invMass + raN.dot(A.invI * raN) + rbN.dot(B.invI * rbN);
    return safeInverse(k);
}

} // namespace detail

class Constraint {
public:
    virtual ~Constraint() = default;

    virtual ConstraintType getType() const = 0;

    /// Calcula Jacobianas/massas efetivas/bias e reinjeta os impulsos acumulados.
    virtual void prepare(const StepContext& ctx) = 0;

    /// Uma iteração de Gauss–Seidel sobre as linhas da restrição.
    virtual void solveVelocity() = 0;

    /// Chamado após a última iteração (persistência dos impulsos).
    virtual void finish() {}

    /// Uma iteração de correção de posição (NGS), DEPOIS da integração das posições: recalcula o
    /// erro com as poses atuais e move os corpos para reduzi-lo, sem alterar velocidades.
    /// Retorna o erro de posição (≥ 0, em metros; para juntas angulares, radianos) medido ANTES
    /// da correção desta iteração. Padrão: nada a corrigir.
    virtual Real solvePosition(const SolverSettings& settings) {
        (void)settings;
        return Real(0);
    }

    RigidBody* getBodyA() const { return bodyA_; }
    RigidBody* getBodyB() const { return bodyB_; }

    bool isEnabled() const { return enabled_; }
    void setEnabled(bool e) { enabled_ = e; }

    /// Só vale resolver se habilitada e ao menos um dos corpos for dinâmico.
    bool isActive() const { return enabled_ && (bodyA_->isDynamic() || bodyB_->isDynamic()); }

protected:
    /// @throws std::invalid_argument se algum corpo for nulo ou se forem o mesmo corpo.
    Constraint(RigidBody* a, RigidBody* b) : bodyA_(a), bodyB_(b) {
        if (!a || !b) throw std::invalid_argument("Constraint: corpo nulo");
        if (a == b) throw std::invalid_argument("Constraint: os dois corpos sao o mesmo");
    }
    Constraint(const Constraint&) = default;
    Constraint& operator=(const Constraint&) = default;

    RigidBody* bodyA_;
    RigidBody* bodyB_;
    bool enabled_ = true;
};

} // namespace physics
