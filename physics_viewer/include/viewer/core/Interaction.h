// =============================================================================
// Interaction.h
// Interação com a cena (sem raylib, testável): selecionar por raycast, arrastar com mola,
// lançar corpos e apagar. A janela só converte o mouse em um raio (origem + direção no mundo)
// e chama estas funções.
// =============================================================================
#pragma once

#include "physics/common/Types.h"
#include "physics/math/Vector3.h"
#include "viewer/core/Scene.h"

namespace viewer {

/// Resultado de um clique: qual corpo o raio atingiu (só corpos NÃO estáticos são selecionáveis).
struct PickResult {
    bool hit = false;
    physics::BodyId body = physics::kInvalidBodyId;
    physics::Vector3 point;        ///< ponto de impacto (mundo)
    physics::Real distance = 0;    ///< distância da origem do raio ao ponto
};

/// Raycast do motor contra os corpos dinâmicos/cinemáticos da cena (V14: `includeStatic` inclui o cenário estático). Direção não finita ou nula: sem acerto.
PickResult pickBody(const Scene& scene, const physics::Vector3& origin, const physics::Vector3& direction,
                    physics::Real maxDistance = physics::Real(200), bool includeStatic = false);

/// Arrastar com mola: uma mola-amortecedor liga o ponto clicado do corpo ao ponto do raio do mouse
/// na MESMA profundidade em que o clique aconteceu. A força é aplicada no ponto (gera torque: o
/// corpo gira ao ser puxado pela quina). Aplique UMA vez por quadro de render, antes de update().
class SpringDrag {
public:
    /// Começa a arrastar o corpo de `pick` (ignorado se o acerto for inválido). `origin` = origem do raio.
    bool begin(Scene& scene, const PickResult& pick);
    void end() { body_ = physics::kInvalidBodyId; }
    bool active() const { return body_ != physics::kInvalidBodyId; }
    physics::BodyId body() const { return body_; }

    /// Muda a profundidade do ponto-alvo (roda do mouse durante o arrasto); limitada a [0,5; 100] m.
    void addDepth(physics::Real delta);

    /// Aplica a força da mola neste quadro. `frameTime` = tempo do quadro de render. Se o corpo não
    /// existe mais, o arrasto termina. Devolve false quando não havia o que aplicar.
    bool apply(Scene& scene, const physics::Vector3& rayOrigin, const physics::Vector3& rayDirection,
               physics::Real frameTime);

    /// Ponto do corpo (mundo) e ponto-alvo do mouse (mundo), para desenhar a mola. Válidos após apply().
    const physics::Vector3& anchorWorld() const { return anchorWorld_; }
    const physics::Vector3& targetWorld() const { return targetWorld_; }

    /// Rigidez: ω0 [rad/s] da mola por unidade de massa (k/m = ω0², amortecimento crítico).
    static constexpr float kOmega = 8.0f;
    /// Aceleração máxima que a mola pode causar [m/s²] (evita "chicote" ao puxar de longe).
    static constexpr float kMaxAccel = 150.0f;

private:
    physics::BodyId body_ = physics::kInvalidBodyId;
    physics::Vector3 localAnchor_;   ///< ponto agarrado, no referencial do centro de massa
    physics::Real depth_ = 0;
    physics::Vector3 anchorWorld_, targetWorld_;
};

/// Lança um corpo (caixa de 0,6 m ou esfera de 0,3 m de raio, 1 kg) a partir de `origin`, na direção
/// `direction`, com velocidade `speed` [m/s]. Devolve o BodyId, ou kInvalidBodyId se a entrada for inválida
/// ou o mundo estiver cheio. Registra o corpo em `scene.bodies`.
physics::BodyId launchBody(Scene& scene, ShapeKind kind, const physics::Vector3& origin,
                           const physics::Vector3& direction, physics::Real speed);

/// Apaga um corpo NÃO estático: destrói no mundo (e suas juntas) e remove de `bodies`, de `links` e
/// do foco. Estáticos (chão, âncoras) e ids inválidos: devolve false e não muda nada — a menos que
/// `allowStatic` (V14, editor de cena) permita apagar também o cenário estático.
bool deleteBody(Scene& scene, physics::BodyId id, bool allowStatic = false);

} // namespace viewer
