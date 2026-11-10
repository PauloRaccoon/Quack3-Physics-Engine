// =============================================================================
// Integrator.cpp
// =============================================================================
#include "physics/dynamics/Integrator.h"

#include <algorithm>
#include <cmath>

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

bool Integrator::integrateGyroscopicAccurate(const RigidBody& body, const Vector3& wWorld, Real dt, Vector3& wEnd,
                                             Vector3& correction) {
    return integrateGyroscopicAccurate(body.getOrientation(), body.getMassData().inertia, body.getMassData().invInertia, wWorld, dt, wEnd,
                                       correction);
}

bool Integrator::integrateGyroscopicAccurate(const Quaternion& q0, const Matrix3& I, const Matrix3& Iinv, const Vector3& wWorld, Real dt,
                                             Vector3& wEnd, Vector3& correction) {
    if (!wWorld.isFinite() || !(dt > Real(0))) return false;
    Vector3 wb = q0.rotateInverse(wWorld);
    const Real mag = wb.length();
    if (!(mag * dt <= Real(3))) return mag * dt <= Real(0) ? (wEnd = wWorld, correction = Vector3::zero(), true) : false;

    // f(ω) = I⁻¹ ((Iω) × ω)   (equações de Euler, sem torque)
    auto f = [&](const Vector3& w) { return Iinv * ((I * w).cross(w)); };
    const int m = std::min(32, std::max(1, static_cast<int>(std::ceil(static_cast<double>(mag * dt) / 0.1))));
    const Real h = dt / static_cast<Real>(m);
    Quaternion dq = Quaternion::identity();   // rotação acumulada no referencial do corpo: q_fim = q0 · dq
    for (int i = 0; i < m; ++i) {
        const Vector3 k1 = f(wb);
        const Vector3 k2 = f(wb + k1 * (h * Real(0.5)));
        const Vector3 k3 = f(wb + k2 * (h * Real(0.5)));
        const Vector3 k4 = f(wb + k3 * h);
        const Vector3 wMid = wb;   // ω no meio do subpasso: RK4 de meio passo
        const Vector3 h1 = f(wMid);
        const Vector3 h2 = f(wMid + h1 * (h * Real(0.25)));
        const Vector3 h3 = f(wMid + h2 * (h * Real(0.25)));
        const Vector3 h4 = f(wMid + h3 * (h * Real(0.5)));
        const Vector3 wHalf = wMid + (h1 + h2 * Real(2) + h3 * Real(2) + h4) * (h * Real(0.5) / Real(6));
        const Vector3 wNext = wb + (k1 + k2 * Real(2) + k3 * Real(2) + k4) * (h / Real(6));
        const Vector3 wAvg = (wb + wHalf * Real(4) + wNext) * (Real(1) / Real(6));   // Simpson
        dq = (dq * Quaternion::fromRotationVector(wAvg * h)).normalized();
        wb = wNext;
    }
    if (!wb.isFinite()) return false;

    // Rotação do passo no corpo (dq) -> vetor de rotação -> ω média no mundo (q0 · exp(a) · q0⁻¹ = exp(q0 a)).
    Quaternion d = dq;
    if (d.w < Real(0)) d = Quaternion(-d.w, -d.x, -d.y, -d.z);
    const Vector3 vec(d.x, d.y, d.z);
    const Real s = vec.length();
    const Real angle = Real(2) * std::atan2(s, d.w);
    const Vector3 rotVecBody = s > Real(1e-9) ? vec * (angle / s) : vec * Real(2);
    const Vector3 wAvgWorld = q0.rotate(rotVecBody / dt);
    const Quaternion qEnd = (q0 * dq).normalized();
    const Vector3 wEndWorld = qEnd.rotate(wb);
    if (!wAvgWorld.isFinite() || !wEndWorld.isFinite()) return false;
    wEnd = wEndWorld;
    correction = wAvgWorld - wEndWorld;
    return true;
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
    Vector3 w = w0;
    Vector3 gyroCorr = Vector3::zero();
    Vector3 halfAvg = Vector3::zero();   // ω média da 1ª metade (divisão de Strang); só vale se halfValid
    bool halfValid = false;
    const bool strang = s.gyroscopicTerm && s.gyroMethod == GyroscopicMethod::Accurate && s.gyroStrangSplitting;
    if (strang && w.isFinite()) {
        // Strang: ½Δt de rotação livre ANTES do solver (+ os chutes de torque/gravidade); a outra ½ vem em integratePositions.
        Vector3 wEnd, corr;
        if (integrateGyroscopicAccurate(b, w, dt * Real(0.5), wEnd, corr)) {
            w = wEnd;
            halfAvg = wEnd + corr;
            halfValid = true;
        } else {
            w = applyGyroscopicTerm(b, w, dt);   // |ω|Δt grande demais: método implícito inteiro, sem 2ª metade
        }
    }
    w += b.getInvInertiaWorld() * b.getTorque() * dt;
    if (s.gyroscopicTerm && !strang && w.isFinite()) {
        Vector3 wEnd, corr;
        if (s.gyroMethod == GyroscopicMethod::Accurate && integrateGyroscopicAccurate(b, w, dt, wEnd, corr)) {
            w = wEnd;
            gyroCorr = corr;
        } else {
            w = applyGyroscopicTerm(b, w, dt);
        }
    }

    // Amortecimento implícito: v ← v / (1 + c Δt) — estável para qualquer c Δt.
    v *= Real(1) / (Real(1) + dt * b.getLinearDamping());
    const Real angDamp = Real(1) / (Real(1) + dt * b.getAngularDamping());
    w *= angDamp;
    gyroCorr *= angDamp;
    halfAvg *= angDamp;
    if (!v.isFinite()) v = v0;
    if (!w.isFinite()) { w = w0; gyroCorr = Vector3::zero(); halfValid = false; }

    const Vector3 vNew = clampLength(v, s.maxLinearSpeed);
    const Vector3 wNew = clampLength(w, s.maxAngularSpeed);
    b.setLinearVelocity(vNew);
    b.setAngularVelocity(wNew);
    // A correção só vale se o limite de velocidade não cortou ω (senão ela deixa de ser consistente com ele).
    const bool clamped = (wNew - w).lengthSquared() > Real(0);
    b.setGyroCorrection(clamped ? Vector3::zero() : gyroCorr);
    if (halfValid && !clamped) b.setGyroHalf(halfAvg); else b.clearGyroHalf();
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
    // V25: com o giroscópio preciso, a rotação do passo usa ω + correção (ω varia dentro do passo); a velocidade guardada
    // continua sendo ω (o solver e o resto do motor não veem a correção).
    auto rotate = [&](const Quaternion& q0, const Vector3& omega, Real h) {
        return s.orientation == OrientationIntegration::Exponential ? q0.integratedExp(omega, h) : q0.integrated(omega, h);
    };
    Quaternion q;
    if (b.hasGyroHalf() && s.gyroscopicTerm && s.gyroMethod == GyroscopicMethod::Accurate && s.gyroStrangSplitting) {
        // Strang: 1ª metade já foi (ω média guardada); a 2ª usa a ω que o solver deixou, evoluindo livremente de qMid.
        const Real half = dt * Real(0.5);
        const Quaternion qMid = rotate(b.getOrientation(), b.getGyroHalfOmega(), half);
        Vector3 wEnd, corr;
        if (integrateGyroscopicAccurate(qMid, b.getMassData().inertia, b.getMassData().invInertia, w, half, wEnd, corr)) {
            q = rotate(qMid, wEnd + corr, half);
            b.setAngularVelocity(clampLength(wEnd, s.maxAngularSpeed));
        } else {
            q = rotate(qMid, w, half);
        }
    } else {
        const Vector3 wRot = w + b.getGyroCorrection();
        q = rotate(b.getOrientation(), wRot, dt);
    }
    b.setGyroCorrection(Vector3::zero());
    b.clearGyroHalf();
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
