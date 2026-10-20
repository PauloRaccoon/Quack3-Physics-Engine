// =============================================================================
// RenderPose.cpp
// =============================================================================
#include "viewer/core/RenderPose.h"

#include <cmath>

#include "physics/math/Matrix3.h"

namespace viewer {

using physics::Quaternion;
using physics::Real;
using physics::Transform;
using physics::Vector3;

namespace {

constexpr Real kPi = Real(3.14159265358979323846);

Real clamp01(Real t) { return t < Real(0) ? Real(0) : (t > Real(1) ? Real(1) : t); }

/// Diferença angular b - a levada ao intervalo (-π, π] (menor giro).
Real shortestDelta(Real a, Real b) {
    Real d = std::fmod(b - a, Real(2) * kPi);
    if (d > kPi) d -= Real(2) * kPi;
    if (d <= -kPi) d += Real(2) * kPi;
    return d;
}

} // namespace

const char* interpModeName(InterpMode mode) {
    switch (mode) {
        case InterpMode::Slerp: return "slerp";
        case InterpMode::LerpNoNormalize: return "lerp sem normalizar";
        case InterpMode::Euler: return "Euler (yaw/pitch/roll)";
    }
    return "?";
}

EulerAngles toEuler(const Quaternion& q) {
    // Matriz do quatérnio normalizado (a conversão supõe |q| = 1).
    const physics::Matrix3 r = q.normalized().toMatrix();
    // Para R = Ry(ψ)·Rx(θ)·Rz(φ):  R(1,2) = -sen θ;  R(0,2) = sen ψ cos θ;  R(2,2) = cos ψ cos θ;
    //                              R(1,0) = cos θ sen φ;  R(1,1) = cos θ cos φ.
    EulerAngles e;
    Real s = -r(1, 2);
    if (s > Real(1)) s = Real(1);
    if (s < Real(-1)) s = Real(-1);
    e.pitch = std::asin(s);
    if (std::fabs(s) < Real(1) - Real(1e-6)) {
        e.yaw = std::atan2(r(0, 2), r(2, 2));
        e.roll = std::atan2(r(1, 0), r(1, 1));
    } else {
        // Gimbal lock: yaw e roll giram em torno do mesmo eixo; só a soma/diferença é definida.
        e.roll = Real(0);
        e.yaw = std::atan2(-r(2, 0), r(0, 0));
    }
    return e;
}

Quaternion fromEuler(const EulerAngles& e) {
    const Quaternion qy = Quaternion::fromAxisAngle(Vector3(0, 1, 0), e.yaw);
    const Quaternion qx = Quaternion::fromAxisAngle(Vector3(1, 0, 0), e.pitch);
    const Quaternion qz = Quaternion::fromAxisAngle(Vector3(0, 0, 1), e.roll);
    return (qy * qx * qz).normalized();
}

bool nearGimbalLock(const EulerAngles& e, Real margin) {
    return std::fabs(e.pitch) > kPi * Real(0.5) - margin;
}

Quaternion interpolateOrientation(const Quaternion& a, const Quaternion& b, Real t, InterpMode mode) {
    if (!a.isFinite() || !b.isFinite() || !std::isfinite(t)) return b;
    t = clamp01(t);
    switch (mode) {
        case InterpMode::Slerp:
            return Quaternion::slerp(a, b, t);
        case InterpMode::LerpNoNormalize: {
            // Caminho curto: se os quatérnios estão em hemisférios opostos, inverte b (mesma rotação).
            const Real dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
            const Quaternion bb = dot < Real(0) ? Quaternion(-b.w, -b.x, -b.y, -b.z) : b;
            return a * (Real(1) - t) + bb * t;   // propositalmente SEM normalizar
        }
        case InterpMode::Euler: {
            const EulerAngles ea = toEuler(a), eb = toEuler(b);
            EulerAngles e;
            e.yaw = ea.yaw + shortestDelta(ea.yaw, eb.yaw) * t;
            e.pitch = ea.pitch + (eb.pitch - ea.pitch) * t;
            e.roll = ea.roll + shortestDelta(ea.roll, eb.roll) * t;
            return fromEuler(e);
        }
    }
    return b;
}

Transform interpolatePose(const Transform& a, const Transform& b, Real t, InterpMode mode) {
    if (mode == InterpMode::Slerp) return Transform::interpolate(a, b, clamp01(t));
    Transform out;
    out.position = Transform::interpolate(a, b, clamp01(t)).position;   // posição: sempre lerp
    out.rotation = interpolateOrientation(a.rotation, b.rotation, t, mode);
    return out;
}

AxisAngleF toAxisAngle(const Quaternion& q) {
    Vector3 axis;
    Real angle = 0;
    q.normalized().toAxisAngle(axis, angle);
    AxisAngleF out;
    out.axis = toVec3f(axis);
    out.angle = static_cast<float>(angle);
    return out;
}

} // namespace viewer
