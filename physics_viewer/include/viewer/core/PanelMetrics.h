// =============================================================================
// PanelMetrics.h  (V17)
// Alturas e passos de linha dos painéis das colunas, num lugar só e SEM raylib. A janela usa estas constantes para
// desenhar e para dimensionar os painéis; o teste confere que o conteúdo de cada formulário cabe acima do rodapé.
// (A janela não pode ser aberta nos testes; por isso o layout é uma conta pura que dá para verificar.)
// =============================================================================
#pragma once

namespace viewer::panel {

constexpr float kPad = 8.0f;                 ///< espaço entre painéis de uma coluna

// Coluna esquerda
constexpr float kQuatH = 244.0f;             ///< painel do quatérnio (cabe a linha do gimbal lock)
constexpr float kQuatContentEnd = 220.0f;    ///< onde termina a última linha (gimbal lock): 34+24+24+28+22+26+46 + 16 de fonte
constexpr float kWorldH = 270.0f;            ///< painel do mundo e debug
constexpr float kRecH = 176.0f;              ///< painel de gravação CSV (V16)

// Coluna direita
constexpr float kInterpH = 214.0f;           ///< painel da interpolação

// Painel de objetos: passos verticais do formulário "Adicionar > Forma"
constexpr float kObjTitle = 32.0f;           ///< do topo do painel até a linha das abas Objeto/Adicionar/Cenas/Juntas
constexpr float kObjTabsStep = 36.0f;
constexpr float kAddTabsStep = 34.0f;        ///< Forma/Posicao/Orient/Veloc
constexpr float kKindRowStep = 28.0f;        ///< cada linha de botões de forma (V18: são 3 linhas de 3 botões)
constexpr int kKindRows = 3;
constexpr float kModelRowStep = 32.0f;       ///< V18: linha "Atualizar lista" (só com modelo 3D escolhido)
constexpr float kListStep = 84.0f;           ///< lista de modelos (80 + 4)
constexpr float kImportStep = 62.0f;         ///< V17: dica de importar + campo de caminho + botão
constexpr float kModelInfoStep = 54.0f;      ///< três linhas de resumo do modelo
constexpr float kColliderStep = 58.0f;       ///< V17: escolha do colisor (botões + uma linha de explicação)
constexpr float kSliderStep = 26.0f;
constexpr float kCheckStep = 24.0f;
constexpr float kFooter = 62.0f;             ///< "posicionar com clique" + botões Adicionar/Lançar
constexpr float kFooterGap = 8.0f;           ///< folga entre o fim do conteúdo e o rodapé

/// Y (a partir do topo do painel de objetos) onde termina o conteúdo da aba Adicionar > Forma.
/// `axisShape`: cápsula/cilindro (sliders de raio e altura). `tuning`: modelo com colisor cápsula/cilindro
/// (sliders de ajuste fino do raio e da altura). V18.
inline float addFormContentEnd(bool box, bool model, bool axisShape = false, bool tuning = false) {
    float y = kObjTitle + kObjTabsStep + kAddTabsStep + static_cast<float>(kKindRows) * kKindRowStep;
    if (model) y += kModelRowStep + kListStep + kImportStep + kModelInfoStep + kColliderStep;
    if (model && tuning) y += 2.0f * kSliderStep;
    y += 4.0f * kSliderStep;            // tamanho, massa, atrito, restituição
    if (box) y += 3.0f * kSliderStep;   // arestas x, y, z
    if (axisShape) y += 2.0f * kSliderStep;   // raio, altura
    y += kCheckStep;                    // estático
    if (model) y += kCheckStep;         // mostrar o colisor (fio)
    return y;
}

/// Altura do painel de objetos na aba Adicionar > Forma (conteúdo + folga + rodapé).
inline float addFormHeight(bool box, bool model, bool axisShape = false, bool tuning = false) {
    return addFormContentEnd(box, model, axisShape, tuning) + kFooterGap + kFooter;
}

/// Altura mínima das outras abas do painel de objetos (Posicao, Orient, Veloc: cabem em bem menos).
constexpr float kAddOtherTabsH = 440.0f;
constexpr float kObjectTabH = 336.0f;
constexpr float kScenesTabH = 392.0f;
constexpr float kJointsTabH = 520.0f;

/// Altura do painel de objetos para a aba principal e o estado do formulário.
inline float objPanelHeight(int mainTab, int addTab, bool box, bool model, bool axisShape = false, bool tuning = false) {
    if (mainTab == 0) return kObjectTabH;
    if (mainTab == 1) {
        const float form = addFormHeight(box, model, axisShape, tuning);
        // as abas Posicao/Orient/Veloc usam a altura do formulário se ele for maior: o painel não muda de tamanho
        // ao trocar de aba dentro de "Adicionar"
        (void)addTab;
        return form > kAddOtherTabsH ? form : kAddOtherTabsH;
    }
    if (mainTab == 2) return kScenesTabH;
    return kJointsTabH;
}

} // namespace viewer::panel
