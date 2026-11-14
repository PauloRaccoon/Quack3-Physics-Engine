// =============================================================================
// BodyEditor.h
// Edição de um corpo em tempo de execução (sem raylib, testável): parâmetros materiais, pose,
// velocidades, acordar/dormir. O painel do objeto selecionado só chama estas funções.
// Todas validam o id (corpo vivo e dinâmico, exceto onde dito) e ignoram valores não finitos.
// V14: posição, orientação, atrito e restituição também valem para corpos ESTÁTICOS (montar cenário).
// =============================================================================
#pragma once

#include "physics/common/Types.h"
#include "physics/math/Quaternion.h"
#include "physics/math/Vector3.h"
#include "physics/pipeline/PhysicsWorld.h"

namespace viewer {

/// Parâmetros editáveis (fotografia do corpo).
struct BodyParams {
    physics::Real mass = 0;
    physics::Real friction = 0;
    physics::Real restitution = 0;
    physics::Real linearDamping = 0;
    physics::Real angularDamping = 0;
    physics::Real gravityScale = 1;
    bool allowSleep = true;
    bool awake = true;
};

namespace limits {
constexpr physics::Real kMinMass = physics::Real(0.05);
constexpr physics::Real kMaxMass = physics::Real(100);
constexpr physics::Real kMaxDamping = physics::Real(5);
constexpr physics::Real kMaxGravityScale = physics::Real(5);
} // namespace limits

/// Lê os parâmetros. Corpo inválido: devolve false e não toca em `out`.
bool readParams(const physics::PhysicsWorld& world, physics::BodyId id, BodyParams& out);

/// Muda a massa mantendo a forma (e portanto a densidade muda): massa e tensor de inércia são
/// recalculados pela forma (MassData::fromShapeMass). Limitada a [kMinMass, kMaxMass]. Acorda o corpo.
bool setMass(physics::PhysicsWorld& world, physics::BodyId id, physics::Real mass);

/// Atrito [0, 2], restituição [0, 1], amortecimentos [0, kMaxDamping], escala de gravidade
/// [-kMaxGravityScale, kMaxGravityScale] (negativa = o corpo "sobe"). Valores fora são limitados.
bool setFriction(physics::PhysicsWorld& world, physics::BodyId id, physics::Real v);
bool setRestitution(physics::PhysicsWorld& world, physics::BodyId id, physics::Real v);
bool setLinearDamping(physics::PhysicsWorld& world, physics::BodyId id, physics::Real v);
bool setAngularDamping(physics::PhysicsWorld& world, physics::BodyId id, physics::Real v);
bool setGravityScale(physics::PhysicsWorld& world, physics::BodyId id, physics::Real v);
bool setAllowSleep(physics::PhysicsWorld& world, physics::BodyId id, bool allow);

/// Move o frame do corpo para `position` (a orientação e as velocidades não mudam; sem rastro na interpolação).
bool setPosition(physics::PhysicsWorld& world, physics::BodyId id, const physics::Vector3& position);

/// Define a orientação por eixo-ângulo [rad]: q = (cos θ/2, sen θ/2 · eixo normalizado). Eixo nulo ou não
/// finito: ignora (devolve false). A posição não muda.
bool setOrientationAxisAngle(physics::PhysicsWorld& world, physics::BodyId id, const physics::Vector3& axis,
                             physics::Real angle);

/// V24: define a orientação por um quatérnio qualquer (normalizado aqui). Não finito ou norma ~0: ignora (false).
/// A posição não muda; usado pelo arcball.
bool setOrientationQuaternion(physics::PhysicsWorld& world, physics::BodyId id, const physics::Quaternion& q);

/// V25: ângulos de rotação em torno dos eixos FIXOS do mundo, aplicados nesta ordem: ωx em torno de X, depois ωy em torno
/// de Y, depois ωz em torno de Z (q = qz(ωz) · qy(ωy) · qx(ωx)); em radianos. Mais intuitivo que eixo-ângulo/quatérnio.
physics::Quaternion quaternionFromAnglesXYZ(physics::Real wx, physics::Real wy, physics::Real wz);

/// Inversa: ângulos (ωx, ωy, ωz) [rad] com ωx, ωz em (−π, π] e ωy em [−π/2, π/2]. Em ωy = ±π/2 (gimbal lock) ωx = 0 e o
/// restante vai em ωz. quaternionFromAnglesXYZ(anglesXYZFromQuaternion(q)) == q (a menos do sinal).
physics::Vector3 anglesXYZFromQuaternion(const physics::Quaternion& q);

/// Aplica a orientação dada por três ângulos [rad] (ver acima). false se algum não for finito ou o corpo não for editável.
bool setOrientationAnglesXYZ(physics::PhysicsWorld& world, physics::BodyId id, physics::Real wx, physics::Real wy, physics::Real wz);

/// Orientação = identidade (quatérnio 1, 0, 0, 0).
bool resetOrientation(physics::PhysicsWorld& world, physics::BodyId id);

bool setLinearVelocity(physics::PhysicsWorld& world, physics::BodyId id, const physics::Vector3& v);
bool setAngularVelocity(physics::PhysicsWorld& world, physics::BodyId id, const physics::Vector3& w);
bool zeroVelocities(physics::PhysicsWorld& world, physics::BodyId id);

/// Acorda / adormece o corpo (adormecer zera as velocidades e pode não "pegar" se algo o acordar de novo).
bool setAwake(physics::PhysicsWorld& world, physics::BodyId id, bool awake);

} // namespace viewer
