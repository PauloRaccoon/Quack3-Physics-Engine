// =============================================================================
// Integrator.cpp
// =============================================================================
#include "physics/dynamics/Integrator.h"

namespace physics {

namespace {
/// Limita |v| a `maxSpeed` preservando a direção (à prova de overflow, ver Vector3::clampedLength).
inline Vector3 clampLength(const Vector3& v, Real maxSpeed) { return v.clampedLength(maxSpeed); }
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
    // O estado entra no solver logo depois: um NaN/Inf aqui contaminaria os contatos. Se a
    // atualização estourar (forças/massas extremas), a velocidade anterior é mantida.
    const Vector3 v0 = b.getLinearVelocity().isFinite() ? b.getLinearVelocity() : Vector3::zero();
    const Vector3 w0 = b.getAngularVelocity().isFinite() ? b.getAngularVelocity() : Vector3::zero();
    Vector3 v = v0 + (b.getForce() * b.getInvMass() + gravity * b.getGravityScale()) * dt;

    // ω ← ω + I⁻¹ τ Δt  (+ termo giroscópico)
    Vector3 w = w0 + b.getInvInertiaWorld() * b.getTorque() * dt;
    if (s.gyroscopicTerm && w.isFinite()) w = applyGyroscopicTerm(b, w, dt);

    // Amortecimento implícito: v ← v / (1 + c Δt) — estável para qualquer c Δt.
    v *= Real(1) / (Real(1) + dt * b.getLinearDamping());
    w *= Real(1) / (Real(1) + dt * b.getAngularDamping());
    if (!v.isFinite()) v = v0;
    if (!w.isFinite()) w = w0;

    const Vector3 vNew = clampLength(v, s.maxLinearSpeed);
    const Vector3 wNew = clampLength(w, s.maxAngularSpeed);
    b.setLinearVelocity(vNew);
    b.setAngularVelocity(wNew);
    b.setStepVelocityDelta(vNew - v0, wNew - w0);   // a restituição do contato precisa disto (ver RigidBody.h)
    b.clearForces();
}

void Integrator::integratePositions(RigidBody& b, Real dt, const IntegratorSettings& s) {
    if (b.isStatic()) return;

    // Sanitiza antes de integrar: um NaN/Inf aqui contaminaria a pose e, via
    // contatos, o mundo inteiro.
    if (!b.getLinearVelocity().isFinite()) b.setLinearVelocity(Vector3::zero());
    if (!b.getAngularVelocity().isFinite()) b.setAngularVelocity(Vector3::zero());

    // A velocidade LIMITADA é gravada de volta: o solver pode ter ultrapassado o teto (razões de
    // massa extremas, penetrações profundas) e o estado nunca deve guardar |v| absurdo.
    const Vector3 v = clampLength(b.getLinearVelocity(), s.maxLinearSpeed);
    const Vector3 w = clampLength(b.getAngularVelocity(), s.maxAngularSpeed);
    b.setLinearVelocity(v);
    b.setAngularVelocity(w);

    // x ← x + v Δt ;  q ← exp(½ Δt ω) q   (ou, no modo Euler, normalize(q + ½ Δt (0,ω) q))
    const Quaternion q = (s.orientation == OrientationIntegration::Exponential)
                             ? b.getOrientation().integratedExp(w, dt)
                             : b.getOrientation().integrated(w, dt);
    b.setPose(b.getWorldCenterOfMass() + v * dt, q);
    b.clearStepVelocityDelta();   // o passo acabou: a variação de velocidade deixa de valer
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
