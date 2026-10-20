// =============================================================================
// SequentialImpulseSolver.cpp
// =============================================================================
#include "physics/constraints/SequentialImpulseSolver.h"

#include <algorithm>
#include <cmath>

namespace physics {

void SequentialImpulseSolver::reserve(std::size_t contacts, std::size_t joints) {
    contacts_.reserve(contacts);
    joints_.reserve(joints);
}

void SequentialImpulseSolver::clear() {
    contacts_.clear();
    joints_.clear();
    stats_ = SolverStats();
}

bool SequentialImpulseSolver::addContact(ContactManifold& manifold, RigidBody& a, RigidBody& b) {
    if (manifold.count <= 0 || &a == &b) return false;
    if (!a.isDynamic() && !b.isDynamic()) return false;   // nada a resolver
    if (!manifold.normal.isFinite()) return false;
    contacts_.emplace_back(&manifold, &a, &b);
    return true;
}

void SequentialImpulseSolver::addJoint(JointConstraint& joint) {
    joints_.push_back(&joint);
}

void SequentialImpulseSolver::solve(Real dt) {
    if (!(dt > Real(0)) || !std::isfinite(dt)) return;

    StepContext ctx;
    ctx.dt = dt;
    ctx.invDt = Real(1) / dt;
    ctx.settings = settings_;

    // --- 1) Preparação: Jacobianas, massas efetivas, bias e warm start ------------------------------------
    for (JointConstraint* j : joints_) {
        if (j->isActive()) j->prepare(ctx);
    }
    for (ContactConstraint& c : contacts_) c.prepare(ctx);

    // --- 2) Iterações de Gauss–Seidel ------------------------------------------------------------------------
    const int iterations = settings_.velocityIterations > 0 ? settings_.velocityIterations : 0;
    for (int it = 0; it < iterations; ++it) {
        for (JointConstraint* j : joints_) {
            if (j->isActive()) j->solveVelocity();
        }
        for (ContactConstraint& c : contacts_) c.solveVelocity();
    }

    // --- 3) Persistência dos impulsos (warm start do próximo quadro) ---------------------------------------------
    stats_ = SolverStats();
    stats_.iterations = iterations;
    stats_.jointConstraints = static_cast<int>(joints_.size());
    stats_.contactConstraints = static_cast<int>(contacts_.size());
    for (ContactConstraint& c : contacts_) {
        c.finish();
        stats_.contactPoints += c.getPointCount();
    }
    for (JointConstraint* j : joints_) j->finish();
}

void SequentialImpulseSolver::solvePositions() {
    stats_.positionIterations = 0;
    stats_.maxPenetration = Real(0);
    stats_.maxJointError = Real(0);
    const int iterations = settings_.positionIterations;
    if (iterations <= 0) return;

    // Tolerâncias de parada: contatos aceitam até 3·slop (como o Box2D); juntas, 1·slop.
    const Real contactTolerance = settings_.linearSlop * Real(3);
    const Real jointTolerance = settings_.linearSlop;

    for (int it = 0; it < iterations; ++it) {
        Real jointError = Real(0), contactError = Real(0);
        for (JointConstraint* j : joints_) {
            if (j->isActive()) jointError = std::max(jointError, j->solvePosition(settings_));
        }
        for (ContactConstraint& c : contacts_) contactError = std::max(contactError, c.solvePosition(settings_));

        ++stats_.positionIterations;
        stats_.maxJointError = jointError;
        stats_.maxPenetration = contactError;
        if (contactError <= contactTolerance && jointError <= jointTolerance) break;
    }
}

} // namespace physics
