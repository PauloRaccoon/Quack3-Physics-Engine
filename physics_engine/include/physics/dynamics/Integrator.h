// =============================================================================
// Integrator.h
// Integrador de Euler Semi-Implícito (simplético), dividido em duas etapas
// para que o solver de restrições atue entre elas:
//
//   1) integrateVelocities:  v ← v + (F/m + g) Δt ,  ω ← ω + I⁻¹ τ Δt
//   2) (solver de impulsos modifica v e ω)
//   3) integratePositions:   x ← x + v Δt ,          q ← exp(½ Δt ω) q   (ou Euler: q + ½ Δt (0,ω) q)
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

/// Como a orientação avança com ω constante no passo (V24).
enum class OrientationIntegration {
    Euler,        ///< q ← normalize(q + ½ Δt (0,ω) q): primeira ordem
    Exponential   ///< q ← exp(½ Δt ω) q: exata para ω constante (padrão)
};

/// Como o termo giroscópico ω × Iω é integrado (V25).
enum class GyroscopicMethod {
    Implicit,   ///< um passo de Newton implícito (Catto): estável, mas amortece a nutação e erra a fase da precessão (1ª ordem)
    Accurate    ///< RK4 com subpassos das equações de Euler no referencial do corpo + rotação real do passo (padrão): quase conserva E e L
};

struct IntegratorSettings {
    GyroscopicMethod gyroMethod = GyroscopicMethod::Accurate;
    /// V25: divisão de Strang (½ rotação livre antes do solver, ½ depois): erro O(Δt²) no acoplamento rotor-junta/contato
    /// em vez de O(Δt). Só vale com gyroMethod = Accurate. false = rotação livre inteira antes do solver (modo anterior).
    bool gyroStrangSplitting = true;
    OrientationIntegration orientation = OrientationIntegration::Exponential;
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

    /// V25: integra as equações de Euler  I ω̇ = (Iω) × ω  no referencial do corpo (RK4, subpassos de ≤ ~0,1 rad) a partir
    /// de `angularVelocity` (mundo, antes do giroscópico) e devolve a ω do mundo no FIM do passo (na orientação prevista
    /// do fim). `correction` = ω média equivalente do passo − ω final (ambas no mundo): somada à ω final, gira o corpo
    /// exatamente como a dinâmica livre. Falha segura (devolve false e não usa os valores) se algo não for finito ou
    /// |ω|Δt passar de ~3 rad (o chamador usa o método implícito).
    static bool integrateGyroscopicAccurate(const RigidBody& body, const Vector3& angularVelocity, Real dt, Vector3& wEnd,
                                            Vector3& correction);

    /// Mesma coisa para uma orientação inicial `q0` e um tensor (no corpo) dados: usada pelas duas metades do passo.
    static bool integrateGyroscopicAccurate(const Quaternion& q0, const Matrix3& inertia, const Matrix3& invInertia,
                                            const Vector3& angularVelocity, Real dt, Vector3& wEnd, Vector3& correction);
};

} // namespace physics
