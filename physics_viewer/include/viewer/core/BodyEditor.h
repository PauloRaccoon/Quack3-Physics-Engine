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

/// Orientação = identidade (quatérnio 1, 0, 0, 0).
bool resetOrientation(physics::PhysicsWorld& world, physics::BodyId id);

bool setLinearVelocity(physics::PhysicsWorld& world, physics::BodyId id, const physics::Vector3& v);
bool setAngularVelocity(physics::PhysicsWorld& world, physics::BodyId id, const physics::Vector3& w);
bool zeroVelocities(physics::PhysicsWorld& world, physics::BodyId id);

/// Acorda / adormece o corpo (adormecer zera as velocidades e pode não "pegar" se algo o acordar de novo).
bool setAwake(physics::PhysicsWorld& world, physics::BodyId id, bool awake);

} // namespace viewer
