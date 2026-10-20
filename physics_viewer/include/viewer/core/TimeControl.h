// =============================================================================
// TimeControl.h
// Controle do tempo da simulação (sem raylib, testável): pausa/play, passo a passo e câmera
// lenta (escala de tempo). A janela chama advance() uma vez por quadro no lugar de
// world.update().
// =============================================================================
#pragma once

#include "physics/common/Types.h"
#include "physics/pipeline/PhysicsWorld.h"

namespace viewer {

class TimeControl {
public:
    /// Escalas de tempo disponíveis: 1x, 1/2, 1/4 e 1/10 (câmera lenta).
    static constexpr int kScaleCount = 4;
    static float scaleValue(int index);          ///< índice fora do intervalo é limitado
    static const char* scaleLabel(int index);    ///< "1x", "1/2", "1/4", "1/10"

    bool paused() const { return paused_; }
    void setPaused(bool p) { paused_ = p; if (!p) stepRequested_ = false; }
    void togglePause() { setPaused(!paused_); }

    /// Pede UM passo físico fixo; só vale com a simulação pausada (consumido no próximo advance()).
    /// Várias chamadas antes do advance() contam uma só.
    void requestStep() { if (paused_) stepRequested_ = true; }

    int scaleIndex() const { return scaleIndex_; }
    void setScaleIndex(int index);
    float timeScale() const { return scaleValue(scaleIndex_); }

    /// Avança o mundo por um quadro de render de `frameTime` s. Rodando: world.update(frameTime·escala).
    /// Pausado: nada acontece, exceto se um passo foi pedido (então executa world.step()). Tempo não
    /// finito ou negativo: nada. Devolve o número de passos físicos executados.
    int advance(physics::PhysicsWorld& world, physics::Real frameTime);

    /// α a usar na interpolação de render: o do acumulador, ou 1 logo após um passo manual (mostra o
    /// estado novo em vez de uma mistura com o anterior).
    physics::Real renderAlpha(const physics::PhysicsWorld& world) const;

    /// Zera pedidos pendentes e o α forçado (chamar ao trocar ou reiniciar a cena). Não muda pausa nem escala.
    void onSceneReset() { stepRequested_ = false; forceAlphaOne_ = false; }

private:
    bool paused_ = false;
    bool stepRequested_ = false;
    bool forceAlphaOne_ = false;
    int scaleIndex_ = 0;
};

} // namespace viewer
