// =============================================================================
// Integrator.cpp
// =============================================================================
#include "physics/dynamics/Integrator.h"

namespace physics {

namespace {
/// Limita |v| a `maxSpeed` preservando a direção.
inline Vector3 clampLength(const Vector3& v, Real maxSpeed) {
    const Real lsq = v.lengthSquared();
    if (lsq > maxSpeed * maxSpeed) return v * (maxSpeed / std::sqrt(lsq));
    return v;
}
} // namespace

Vector3 Integrator::applyGyroscopicTerm(const RigidBody& body, const Vector3& w, Real dt) {
    // Resolve, no referencial do CORPO (onde I é constante):
    //     f(ω') = Δt ω' × (I ω') + I (ω' − ω) = 0
    // com um passo de Newton a partir de ω' = ω:
    //     J = I + Δt ( [ω]ₓ I − [Iω]ₓ ),   ω' = ω − J⁻¹ f(ω)
    const Matrix3& I = body.getMassData().inertia;
    const Vector3 wb = body.getOrientation().rotateInverse(w);
    const Vector3 Iw = I * wb;

    const Vector3 f = wb.cross(Iw) * dt;
    const Matrix3 J = I + (Matrix3::skew(wb) * I - Matrix3::skew(Iw)) * dt;

    // J singular → inverse() = 0 → ω' = ω (sem correção): falha segura.
    const Vector3 newWb = wb - J.inverse() * f;
    return body.getOrientation().rotate(newWb);
}

void Integrator::integrateVelocities(RigidBody& b, const Vector3& gravity, Real dt,
                                     const IntegratorSettings& s) {
    if (!b.isDynamic()) {
        b.clearForces();
        return;
    }

    // v ← v + (F/m + g·escala) Δt
    Vector3 v = b.getLinearVelocity();
    v += (b.getForce() * b.getInvMass() + gravity * b.getGravityScale()) * dt;

    // ω ← ω + I⁻¹ τ Δt  (+ termo giroscópico)
    Vector3 w = b.getAngularVelocity();
    w += b.getInvInertiaWorld() * b.getTorque() * dt;
    if (s.gyroscopicTerm) w = applyGyroscopicTerm(b, w, dt);

    // Amortecimento implícito: v ← v / (1 + c Δt) — estável para qualquer c Δt.
    v *= Real(1) / (Real(1) + dt * b.getLinearDamping());
    w *= Real(1) / (Real(1) + dt * b.getAngularDamping());

    b.setLinearVelocity(clampLength(v, s.maxLinearSpeed));
    b.setAngularVelocity(clampLength(w, s.maxAngularSpeed));
    b.clearForces();
}

void Integrator::integratePositions(RigidBody& b, Real dt, const IntegratorSettings& s) {
    if (b.isStatic()) return;

    // Sanitiza antes de integrar: um NaN/Inf aqui contaminaria a pose e, via
    // contatos, o mundo inteiro.
    if (!b.getLinearVelocity().isFinite()) b.setLinearVelocity(Vector3::zero());
    if (!b.getAngularVelocity().isFinite()) b.setAngularVelocity(Vector3::zero());

    const Vector3 v = clampLength(b.getLinearVelocity(), s.maxLinearSpeed);
    const Vector3 w = clampLength(b.getAngularVelocity(), s.maxAngularSpeed);

    // x ← x + v Δt ;  q ← normalize(q + ½ Δt (0,ω) q)
    b.setPose(b.getWorldCenterOfMass() + v * dt, b.getOrientation().integrated(w, dt));
}

void Integrator::integrateVelocities(std::vector<RigidBody>& bodies, const Vector3& gravity, Real dt,
                                     const IntegratorSettings& s) {
    for (RigidBody& b : bodies) integrateVelocities(b, gravity, dt, s);
}

void Integrator::integratePositions(std::vector<RigidBody>& bodies, Real dt,
                                    const IntegratorSettings& s) {
    for (RigidBody& b : bodies) integratePositions(b, dt, s);
}

} // namespace physics
