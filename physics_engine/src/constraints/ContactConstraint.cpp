// =============================================================================
// ContactConstraint.cpp
// =============================================================================
#include "physics/constraints/ContactConstraint.h"

#include <algorithm>
#include <cmath>

namespace physics {

ContactConstraint::ContactConstraint(ContactManifold* manifold, RigidBody* a, RigidBody* b)
    : Constraint(a, b), manifold_(manifold) {
    if (!manifold) throw std::invalid_argument("ContactConstraint: manifold nulo");
}

void ContactConstraint::prepare(const StepContext& ctx) {
    const SolverSettings& cfg = ctx.settings;

    count_ = std::min(manifold_->count, kMaxManifoldPoints);

    // Normal A→B (renormalizada: protege contra erro acumulado).
    normal_ = manifold_->normal.normalizedOr(Vector3::unitY());

    // Combinação de materiais: média geométrica no atrito, máximo na restituição.
    friction_ = std::sqrt(std::max(bodyA_->getFriction(), Real(0)) * std::max(bodyB_->getFriction(), Real(0)));
    restitution_ = std::max(bodyA_->getRestitution(), bodyB_->getRestitution());

    detail::BodyState A(*bodyA_), B(*bodyB_);

    // Base tangente. Se há deslizamento, o 1º eixo é alinhado com a velocidade tangencial média
    // dos pontos (como no Bullet): o atrito dominante cai num único eixo e a direção da força
    // acompanha a do deslizamento. Em repouso, usa uma base arbitrária estável.
    {
        Vector3 slide;
        for (int i = 0; i < count_; ++i) {
            const Vector3 pos = manifold_->points[i].getPosition();
            const Vector3 vrel = B.velocityAt(pos - B.com) - A.velocityAt(pos - A.com);
            slide += vrel - normal_ * vrel.dot(normal_);
        }
        slide /= static_cast<Real>(std::max(count_, 1));
        constexpr Real kSlideSpeedSq = Real(1e-6);   // (1 mm/s)²
        if (slide.lengthSquared() > kSlideSpeedSq) {
            tangent_[0] = slide.normalized();
            tangent_[1] = normal_.cross(tangent_[0]).normalized();
        } else {
            computeOrthonormalBasis(normal_, tangent_[0], tangent_[1]);
        }
    }

    for (int i = 0; i < count_; ++i) {
        const ContactPoint& cp = manifold_->points[i];
        Point& p = points_[i];

        const Vector3 pos = cp.getPosition();
        p.rA = pos - A.com;
        p.rB = pos - B.com;

        p.normalMass = detail::linearEffectiveMass(A, B, p.rA, p.rB, normal_);
        p.tangentMass[0] = detail::linearEffectiveMass(A, B, p.rA, p.rB, tangent_[0]);
        p.tangentMass[1] = detail::linearEffectiveMass(A, B, p.rA, p.rB, tangent_[1]);

        // --- Bias de velocidade ----------------------------------------------------------------
        const Real vn = (B.velocityAt(p.rB) - A.velocityAt(p.rA)).dot(normal_);
        const Real depth = cp.depth;

        Real bias = Real(0);
        if (depth > cfg.linearSlop) {
            // Baumgarte com penetration slop.
            bias = std::min(cfg.baumgarte * ctx.invDt * (depth - cfg.linearSlop), cfg.maxBiasVelocity);
        } else if (depth < Real(0)) {
            // Contato especulativo: pode se aproximar no máximo o vão por passo.
            bias = depth * ctx.invDt;
        }
        // Restituição: só com impacto acima do velocity slop e se a aproximação ALCANÇA o contato
        // neste passo (|vn|·Δt ≥ vão). Isso vale para contato real (depth ≥ 0) e também para o
        // especulativo: sem isso, um corpo que chega com um vão de poucos mm (dentro da margem)
        // teria a velocidade zerada pelo bias de vão e perderia o quique.
        if (vn < -cfg.velocitySlop && depth >= vn * ctx.dt) {
            bias = std::max(bias, -restitution_ * vn);
        }
        p.bias = bias;

        // --- Warm starting -----------------------------------------------------------------------
        if (cfg.warmStarting) {
            Real an = cp.normalImpulse;
            if (!std::isfinite(an) || an < Real(0)) an = Real(0);

            Real a1 = cp.tangentImpulse.dot(tangent_[0]);
            Real a2 = cp.tangentImpulse.dot(tangent_[1]);
            if (!std::isfinite(a1) || !std::isfinite(a2)) a1 = a2 = Real(0);

            p.normalImpulse = an;
            p.tangentImpulse[0] = a1;
            p.tangentImpulse[1] = a2;

            const Vector3 P = normal_ * an + tangent_[0] * a1 + tangent_[1] * a2;
            A.applyImpulse(-P, p.rA);
            B.applyImpulse(P, p.rB);
        } else {
            p.normalImpulse = Real(0);
            p.tangentImpulse[0] = p.tangentImpulse[1] = Real(0);
        }
    }

    A.store(*bodyA_);
    B.store(*bodyB_);
}

void ContactConstraint::solveVelocity() {
    detail::BodyState A(*bodyA_), B(*bodyB_);

    // --- 1) Atrito primeiro (a restrição normal, resolvida depois, "vence") ------------------------
    for (int i = 0; i < count_; ++i) {
        Point& p = points_[i];
        const Real maxF = friction_ * p.normalImpulse;

        // Duas linhas desacopladas, cada uma com limite de caixa |acumulado| ≤ μ λn.
        for (int t = 0; t < 2; ++t) {
            const Vector3 dv = B.velocityAt(p.rB) - A.velocityAt(p.rA);
            const Real lambda = -p.tangentMass[t] * dv.dot(tangent_[t]);
            const Real newImpulse = clamp(p.tangentImpulse[t] + lambda, -maxF, maxF);
            const Real applied = newImpulse - p.tangentImpulse[t];
            p.tangentImpulse[t] = newImpulse;

            const Vector3 P = tangent_[t] * applied;
            A.applyImpulse(-P, p.rA);
            B.applyImpulse(P, p.rB);
        }

        // Cone circular: reprojeta o vetor tangencial acumulado no disco de raio μ λn.
        const Real a1 = p.tangentImpulse[0], a2 = p.tangentImpulse[1];
        const Real lenSq = a1 * a1 + a2 * a2;
        if (lenSq > maxF * maxF && lenSq > Real(0)) {
            const Real s = maxF / std::sqrt(lenSq);
            const Vector3 P = tangent_[0] * (a1 * s - a1) + tangent_[1] * (a2 * s - a2);
            A.applyImpulse(-P, p.rA);
            B.applyImpulse(P, p.rB);
            p.tangentImpulse[0] = a1 * s;
            p.tangentImpulse[1] = a2 * s;
        }
    }

    // --- 2) Não-penetração ------------------------------------------------------------------------------
    for (int i = 0; i < count_; ++i) {
        Point& p = points_[i];
        const Vector3 dv = B.velocityAt(p.rB) - A.velocityAt(p.rA);
        const Real vn = dv.dot(normal_);
        const Real lambda = -p.normalMass * (vn - p.bias);

        // Impulso ACUMULADO ≥ 0 (contato só empurra, nunca puxa).
        const Real newImpulse = std::max(p.normalImpulse + lambda, Real(0));
        const Real applied = newImpulse - p.normalImpulse;
        p.normalImpulse = newImpulse;

        const Vector3 P = normal_ * applied;
        A.applyImpulse(-P, p.rA);
        B.applyImpulse(P, p.rB);
    }

    A.store(*bodyA_);
    B.store(*bodyB_);
}

void ContactConstraint::finish() {
    for (int i = 0; i < count_; ++i) {
        ContactPoint& cp = manifold_->points[i];
        cp.normalImpulse = points_[i].normalImpulse;
        cp.tangentImpulse = tangent_[0] * points_[i].tangentImpulse[0] +
                            tangent_[1] * points_[i].tangentImpulse[1];
    }
}

} // namespace physics
