// =============================================================================
// JointEditor.h  (V14 — editor de cena)
// Criar e apagar juntas em tempo de execução (sem raylib, testável). A aba "Juntas" só monta um
// JointParams e chama createJoint(). Uma junta liga o corpo A a um corpo B ou ao MUNDO (B = kInvalidBodyId):
// neste caso cria-se um corpo estático sem forma (pivô, bolinha cinza) no ponto de ancoragem.
// =============================================================================
#pragma once

#include <string>

#include "physics/common/Types.h"
#include "physics/math/Vector3.h"
#include "viewer/core/Scene.h"

namespace viewer {

enum class JointKind { Hinge = 0, BallSocket = 1, Distance = 2 };
constexpr int kJointKindCount = 3;

/// Nome curto sem acentos (fonte padrão da raylib é ASCII).
const char* jointKindName(JointKind kind);

namespace jointlimits {
constexpr int kMaxJoints = 2000;                              ///< igual ao limite do arquivo .scene
constexpr physics::Real kMinLength = physics::Real(0.05);     ///< distância mínima entre os pontos de uma junta de distância [m]
constexpr physics::Real kMaxCoordinate = physics::Real(1000); ///< |âncora| por eixo [m]
} // namespace jointlimits

struct JointParams {
    JointKind kind = JointKind::Hinge;
    physics::BodyId a = physics::kInvalidBodyId;   ///< corpo A (obrigatório, existente)
    physics::BodyId b = physics::kInvalidBodyId;   ///< corpo B; kInvalidBodyId = mundo
    /// Ponto de ancoragem no MUNDO. Hinge/BallSocket: o pivô. Distance: o ponto de B (ou do mundo);
    /// a ponta em A é sempre o centro de massa de A.
    physics::Vector3 anchor;
    physics::Vector3 axis = physics::Vector3(0, 1, 0);   ///< eixo da dobradiça (mundo), só Hinge; não precisa ser unitário
    bool collideConnected = false;
};

/// Cria a junta e registra o link de desenho. Falhas (devolve false, `error` explica, a cena NÃO muda):
/// A inválido, B inválido (e diferente de "mundo"), A == B, âncora não finita ou |coord| > 1000, eixo nulo
/// (Hinge), pontos coincidentes (Distance), limite de juntas, exceção do motor.
/// Distance: comprimento = distância atual entre os dois pontos (a junta nasce "sem tensão").
bool createJoint(Scene& scene, const JointParams& params, std::string* error = nullptr);

/// Quantas juntas o mundo tem.
int jointCount(const Scene& scene);

/// Apaga a última junta criada (ordem de criação do mundo), acorda os corpos, remove o último link
/// correspondente e apaga pivôs estáticos sem forma que ficaram sem nenhuma junta. false = não há juntas.
bool destroyLastJoint(Scene& scene);

/// Ponto onde o raio acerta a cena (corpos dinâmicos E estáticos); se não acerta nada e o raio desce,
/// o plano y = 0. false = sem ponto (entrada inválida ou raio para cima sem acerto).
bool pointOnRay(const Scene& scene, const physics::Vector3& origin, const physics::Vector3& direction,
                physics::Vector3& point);

} // namespace viewer
