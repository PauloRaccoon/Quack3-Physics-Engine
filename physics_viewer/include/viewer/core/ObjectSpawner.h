// =============================================================================
// ObjectSpawner.h
// Criar objetos na cena em execução (sem raylib, testável): o painel "Adicionar objeto" monta um
// SpawnParams e chama spawnBody(); a janela só lê sliders e cliques.
// =============================================================================
#pragma once

#include "physics/common/Types.h"
#include "physics/math/Quaternion.h"
#include "physics/math/Vector3.h"
#include "viewer/core/Scene.h"

namespace viewer {

enum class SpawnKind { Box = 0, Sphere = 1, Tetrahedron = 2, Octahedron = 3, Prism = 4, Pyramid = 5 };
constexpr int kSpawnKindCount = 6;

/// Nome curto (sem acentos: a fonte padrão da raylib é ASCII).
const char* spawnKindName(SpawnKind kind);

namespace spawnlimits {
constexpr physics::Real kMinSize = physics::Real(0.2);
constexpr physics::Real kMaxSize = physics::Real(4);
constexpr physics::Real kMinMass = physics::Real(0.05);
constexpr physics::Real kMaxMass = physics::Real(100);
constexpr physics::Real kMaxLinearSpeed = physics::Real(60);    ///< [m/s]
constexpr physics::Real kMaxAngularSpeed = physics::Real(30);   ///< [rad/s]
constexpr physics::Real kMaxCoordinate = physics::Real(1000);   ///< |posição| por eixo [m]
} // namespace spawnlimits

/// Descrição do objeto a criar. "Tamanho 1" = o mesmo tamanho dos objetos da cena Formas: caixa de aresta
/// 1 m; esfera de diâmetro 1 m; tetraedro s = 0,45; octaedro s = 0,6; prisma r = 0,55 e meia-altura 0,4;
/// pirâmide de base 1 m e altura 1 m (tudo multiplicado por `size`).
struct SpawnParams {
    SpawnKind kind = SpawnKind::Box;
    physics::Real size = physics::Real(1);
    physics::Real mass = physics::Real(1);
    physics::Real friction = physics::Real(0.5);
    physics::Real restitution = physics::Real(0.2);
    physics::Vector3 position = physics::Vector3(0, 6, 0);
    physics::Quaternion orientation = physics::Quaternion::identity();   ///< normalizada ao criar
    physics::Vector3 velocity;
    physics::Vector3 angularVelocity;
};

/// Quatérnio (w, x, y, z) -> unitário. Norma ~0 ou não finito: identidade e `ok = false`.
physics::Quaternion orientationFromQuaternion(physics::Real w, physics::Real x, physics::Real y, physics::Real z,
                                              bool* ok = nullptr);

/// Eixo-ângulo [rad] -> quatérnio unitário. Eixo nulo ou não finito: identidade e `ok = false`.
physics::Quaternion orientationFromAxisAngle(const physics::Vector3& axis, physics::Real angle, bool* ok = nullptr);

/// Cria o objeto, registra em `scene.bodies` e devolve o BodyId. Valores fora das faixas são limitados
/// (tamanho, massa, velocidades, posição). Entradas não finitas, mundo cheio ou forma inválida: devolve
/// kInvalidBodyId e NÃO muda a cena. `scene.focus` não muda.
physics::BodyId spawnBody(Scene& scene, const SpawnParams& params);

/// Onde um raio (origem, direção) encontra a cena (chão ou corpos): ponto de impacto + normal · folga,
/// com folga proporcional ao tamanho para o objeto não nascer enterrado. Devolve false se o raio não
/// atinge nada ou a entrada é inválida.
bool spawnPointFromRay(const Scene& scene, const physics::Vector3& origin, const physics::Vector3& direction,
                       physics::Real size, physics::Vector3& point);

/// Lança de `origin` na direção `direction` com velocidade `speed` (a velocidade linear de `params` é
/// substituída; a angular e a orientação ficam). O objeto nasce 2 m à frente da origem.
physics::BodyId launchFrom(Scene& scene, const SpawnParams& params, const physics::Vector3& origin,
                           const physics::Vector3& direction, physics::Real speed);

} // namespace viewer
