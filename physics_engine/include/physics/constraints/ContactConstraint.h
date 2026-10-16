// =============================================================================
// ContactConstraint.h
// Restrições de contato (não-penetração + atrito) para um ContactManifold.
// Cada ponto do manifold gera 3 linhas:
//
//   normal    : J_n = [ −n, −(rA×n),  n,  rB×n ]      λn ≥ 0
//   tangente 1: J_t1 (idem com t1)                    |λt1| ≤ μ λn
//   tangente 2: J_t2 (idem com t2)                    |λt2| ≤ μ λn
//
// Convenção: n aponta de A para B; velocidade relativa no ponto
//   v_rel = (vB + ωB × rB) − (vA + ωA × rA),   vn = v_rel · n.
// O contato exige vn ≥ bias, onde o bias combina:
//   - correção de penetração (Baumgarte com PENETRATION SLOP):
//         β/Δt · max(profundidade − slop, 0)           (limitado a maxBiasVelocity)
//   - restituição, só se a velocidade de impacto passa do VELOCITY SLOP e a
//     aproximação alcança o contato neste passo (vale também para contato
//     especulativo, senão o quique se perderia quando o vão é de poucos mm):
//         −e · vn⁻     (vn⁻ = vn antes do solver)
//   - contato "especulativo" (separação > 0): permite aproximar apenas o vão/Δt.
// O bias é o MAIOR entre correção de posição e restituição (somá-los injetaria
// energia).
//
// Atrito: cone de Coulomb LINEARIZADO nos dois eixos tangenciais. O 1º eixo é
// ALINHADO com a velocidade de deslizamento (como no Bullet), de modo que o
// atrito dominante cai num único eixo e a força se opõe ao deslizamento real;
// o 2º eixo cuida do desvio lateral (rotação). Cada linha é resolvida com limite
// de caixa ±μ·λn e o vetor acumulado é reprojetado no disco de raio μ·λn.
// (Resolver as duas linhas com a matriz 2×2 acoplada foi testado e descartado:
// o PGS então empurra cada impulso na direção de −K⁻¹v, e não de −v, e as
// direções de atrito dos 4 cantos de uma caixa divergem.)
//
// WARM STARTING: os impulsos acumulados ficam nos ContactPoints do manifold
// (que persiste entre quadros). O de atrito é guardado como VETOR de mundo e
// reprojetado na base tangente de cada quadro.
// =============================================================================
#pragma once

#include "physics/collision/narrowphase/ContactManifold.h"
#include "physics/constraints/Constraint.h"

namespace physics {

class ContactConstraint final : public Constraint {
public:
    /// O manifold deve permanecer válido até finish(); a normal aponta de `a` para `b`.
    ContactConstraint(ContactManifold* manifold, RigidBody* a, RigidBody* b);

    ConstraintType getType() const override { return ConstraintType::Contact; }
    void prepare(const StepContext& ctx) override;
    void solveVelocity() override;
    void finish() override;

    const ContactManifold* getManifold() const { return manifold_; }
    int getPointCount() const { return count_; }
    Real getFriction() const { return friction_; }
    Real getRestitution() const { return restitution_; }
    /// Impulso normal acumulado do ponto `i` (diagnóstico/testes).
    Real getNormalImpulse(int i) const { return points_[i].normalImpulse; }
    Real getTangentImpulse(int i, int axis) const { return points_[i].tangentImpulse[axis]; }

private:
    struct Point {
        Vector3 rA;               ///< ponto de contato − centro de massa de A
        Vector3 rB;
        Real normalMass = 0;      ///< massa efetiva ao longo de n
        Real tangentMass[2] = {0, 0};   ///< massa efetiva ao longo de t1 e t2
        Real bias = 0;           ///< velocidade-alvo mínima ao longo de n
        Real normalImpulse = 0;   ///< acumulado, ≥ 0
        Real tangentImpulse[2] = {0, 0};  ///< acumulados nos eixos t1, t2
    };

    ContactManifold* manifold_;
    Vector3 normal_;
    Vector3 tangent_[2];
    Real friction_ = 0;
    Real restitution_ = 0;
    Point points_[kMaxManifoldPoints];
    int count_ = 0;
};

} // namespace physics
