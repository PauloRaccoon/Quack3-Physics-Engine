// =============================================================================
// RenderPose.h
// Orientação de RENDER: como desenhar um corpo entre dois passos físicos, e como mostrar o
// quatérnio na tela (eixo-ângulo, ângulos de Euler). Sem raylib.
//
// Os três modos de interpolar a orientação entre o quatérnio anterior `a` e o atual `b`:
//   Slerp           - esférica: velocidade angular constante, |q| = 1 (o padrão do motor);
//   LerpNoNormalize - (1-t)·a + t·b SEM renormalizar: |q| < 1 no meio do caminho, e a matriz
//                     de rotação calculada a partir dele sai encolhida/deformada (visível);
//   Euler           - converte `a` e `b` em (yaw, pitch, roll), interpola os ângulos e volta
//                     a quatérnio: o caminho é torto e, perto de pitch = ±90° (gimbal lock),
//                     yaw e roll ficam ambíguos e a pose "pula".
// =============================================================================
#pragma once

#include "physics/common/Types.h"
#include "physics/math/Quaternion.h"
#include "physics/math/Transform.h"
#include "viewer/core/ViewerMath.h"

namespace viewer {

enum class InterpMode { Slerp = 0, LerpNoNormalize = 1, Euler = 2 };
constexpr int kInterpModeCount = 3;

/// Nome curto do modo (para o HUD).
const char* interpModeName(InterpMode mode);

/// Ângulos de Euler [rad], convenção YXZ: R = Ry(yaw) · Rx(pitch) · Rz(roll).
/// pitch ∈ [-π/2, π/2]; gimbal lock em pitch = ±π/2.
struct EulerAngles {
    physics::Real yaw = 0;
    physics::Real pitch = 0;
    physics::Real roll = 0;
};

/// Quatérnio (unitário) → Euler. No gimbal lock (|sen pitch| ≈ 1) devolve roll = 0 e põe a rotação
/// restante em yaw: é aqui que a perda de um grau de liberdade aparece.
EulerAngles toEuler(const physics::Quaternion& q);

/// Euler → quatérnio unitário (q = qY(yaw) · qX(pitch) · qZ(roll)).
physics::Quaternion fromEuler(const EulerAngles& e);

/// true se |pitch| está a menos de `margin` rad de ±π/2.
bool nearGimbalLock(const EulerAngles& e, physics::Real margin);

/// Orientação entre `a` (t = 0) e `b` (t = 1) pelo modo escolhido. t é limitado a [0, 1].
/// Entradas não finitas devolvem `b`.
physics::Quaternion interpolateOrientation(const physics::Quaternion& a, const physics::Quaternion& b,
                                           physics::Real t, InterpMode mode);

/// Pose completa: posição por lerp, orientação pelo modo.
physics::Transform interpolatePose(const physics::Transform& a, const physics::Transform& b,
                                   physics::Real t, InterpMode mode);

/// Eixo (unitário) e ângulo [rad, 0..π] de um quatérnio unitário.
struct AxisAngleF {
    Vec3f axis{1.0f, 0.0f, 0.0f};
    float angle = 0.0f;
};
AxisAngleF toAxisAngle(const physics::Quaternion& q);

} // namespace viewer
