// =============================================================================
// WorldEditor.h
// Parâmetros do MUNDO editáveis em tempo de execução (sem raylib, testável) e a tabela de
// opções do desenho de depuração do motor. O painel "Mundo" só chama estas funções.
// Todas ignoram valores não finitos e limitam o que excede a faixa.
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

#include "physics/common/Types.h"
#include "physics/debug/DebugDraw.h"
#include "physics/math/Vector3.h"
#include "physics/pipeline/PhysicsWorld.h"

namespace viewer {

/// Fotografia dos parâmetros editáveis do mundo (também serve para "restaurar os padrões da cena").
struct WorldParams {
    physics::Vector3 gravity;
    physics::Real fixedDt = physics::Real(1) / physics::Real(60);
    int velocityIterations = 10;
    int positionIterations = 3;
    physics::Real linearSlop = 0;
    physics::Real velocitySlop = 0;
    bool warmStarting = true;
    bool sleepEnabled = true;
    physics::Real sleepTime = physics::Real(0.5);
    bool orientationExponential = true;   ///< V24: orientação por exp(½ Δt ω) q (true) ou Euler q + ½ Δt (0,ω) q (false)
};

namespace worldlimits {
constexpr int kMaxVelocityIterations = 50;
constexpr int kMaxPositionIterations = 20;
constexpr physics::Real kMaxLinearSlop = physics::Real(0.1);     ///< [m]
constexpr physics::Real kMaxVelocitySlop = physics::Real(5);     ///< [m/s]
constexpr physics::Real kMinSleepTime = physics::Real(0.05);     ///< [s]
constexpr physics::Real kMaxSleepTime = physics::Real(10);       ///< [s]
constexpr physics::Real kMaxGravity = physics::Real(100);        ///< por componente [m/s²]
} // namespace worldlimits

WorldParams captureWorld(const physics::PhysicsWorld& world);

/// Aplica todos os campos (com limites). A gravidade só é regravada se mudou (regravar acorda todos os corpos).
void applyWorld(physics::PhysicsWorld& world, const WorldParams& params);

bool setWorldGravity(physics::PhysicsWorld& world, const physics::Vector3& g);   ///< componentes limitados a ±kMaxGravity
/// Δt fixo [s]; aceita 1e-4 a 1 (limites do motor). Devolve false se recusado.
bool setWorldFixedStep(physics::PhysicsWorld& world, physics::Real dt);
bool setVelocityIterations(physics::PhysicsWorld& world, int n);   ///< 1 .. kMaxVelocityIterations
bool setPositionIterations(physics::PhysicsWorld& world, int n);   ///< 0 (Baumgarte legado) .. kMaxPositionIterations
bool setLinearSlop(physics::PhysicsWorld& world, physics::Real v);
bool setVelocitySlop(physics::PhysicsWorld& world, physics::Real v);
void setWarmStarting(physics::PhysicsWorld& world, bool on);
/// Liga/desliga o sleeping. Desligar acorda todos os corpos.
void setSleepEnabled(physics::PhysicsWorld& world, bool on);
bool setSleepTime(physics::PhysicsWorld& world, physics::Real seconds);
/// V24: integração da orientação: exponencial de quatérnio (true, exata para ω constante) ou Euler (false).
void setOrientationExponential(physics::PhysicsWorld& world, bool exponential);

// --- Desenho de depuração ---------------------------------------------------------------------------
struct DebugFlagInfo {
    const char* label;     ///< texto da caixa de seleção (sem acentos: a fonte padrão da raylib é ASCII)
    std::uint32_t flag;    ///< bit de physics::DebugDrawFlags
};

/// As 9 opções do motor, na ordem em que aparecem no painel.
const std::vector<DebugFlagInfo>& debugFlagTable();

/// Gera as linhas de depuração do mundo para `flags` (pose interpolada nas formas, para casar com o que
/// é desenhado; com `interpolatedPose = false`, a pose ATUAL, p.ex. logo após um passo manual, quando o desenho usa α = 1). `out` é limpo antes (a capacidade é reaproveitada). Devolve o nº de linhas.
std::size_t buildDebugLines(const physics::PhysicsWorld& world, std::uint32_t flags, physics::DebugLineBuffer& out,
                            bool interpolatedPose = true);

} // namespace viewer
