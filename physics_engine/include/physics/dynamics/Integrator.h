// =============================================================================
// Integrator.h
// Integrador de Euler Semi-Implícito (simplético), dividido em duas etapas
// para que o solver de restrições atue entre elas:
//
//   1) integrateVelocities:  v ← v + (F/m + g) Δt ,  ω ← ω + I⁻¹ τ Δt
//   2) (solver de impulsos modifica v e ω)
//   3) integratePositions:   x ← x + v Δt ,          q ← q + ½ Δt (0,ω) q
//
// A velocidade é atualizada ANTES da posição, usando a velocidade nova — é
// isso que torna o esquema semi-implícito (estável para osciladores).
// =============================================================================
#pragma once

#include <vector>

#include "physics/common/Settings.h"
#include "physics/dynamics/RigidBody.h"
#include "physics/math/Vector3.h"

namespace physics {

struct IntegratorSettings {
    Real maxLinearSpeed = kMaxLinearSpeed;    ///< limite de |v| [m/s]
    Real maxAngularSpeed = kMaxAngularSpeed;  ///< limite de |ω| [rad/s]

    /// Inclui o torque giroscópico  ω × Iω  (corpo assimétrico em rotação livre)
    /// por um passo de Newton implícito (Catto, GDC 2015). Sem ele, o momento
    /// angular não é conservado e caixas "alongadas" giram de modo irreal.
    bool gyroscopicTerm = true;
};

struct Integrator {
    /// Etapa 1: forças, gravidade, amortecimento e limites de velocidade.
    /// Consome (zera) as forças acumuladas. Corpos não-dinâmicos só têm as
    /// forças descartadas.
    static void integrateVelocities(RigidBody& body, const Vector3& gravity, Real dt,
                                    const IntegratorSettings& settings = IntegratorSettings());

    /// Etapa 3: avança posição e orientação com as velocidades atuais.
    /// Velocidades não finitas são zeradas (proteção contra NaN/Inf).
    static void integratePositions(RigidBody& body, Real dt,
                                   const IntegratorSettings& settings = IntegratorSettings());

    // Versões em lote sobre o vetor contíguo de corpos.
    static void integrateVelocities(std::vector<RigidBody>& bodies, const Vector3& gravity, Real dt,
                                    const IntegratorSettings& settings = IntegratorSettings());
    static void integratePositions(std::vector<RigidBody>& bodies, Real dt,
                                   const IntegratorSettings& settings = IntegratorSettings());

    /// Nova velocidade angular (mundo) após o passo giroscópico implícito.
    /// Exposta para testes.
    static Vector3 applyGyroscopicTerm(const RigidBody& body, const Vector3& angularVelocity, Real dt);
};

} // namespace physics
