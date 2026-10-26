// =============================================================================
// DockLayout.h
// Layout da janela do viewer (sem raylib, testável): barra de controle no topo, duas colunas FIXAS nas
// laterais (esquerda e direita) com largura ajustável e recolhíveis, e a cena 3D ocupando o espaço que sobra
// (como no Blender). Também a rolagem vertical de cada coluna e o arquivo de layout.
//
//        +----------------------- barra de controle (topKH) ---------------------+
//        | coluna esq. |s|            viewport 3D (o que sobra)            |s| coluna dir. |
//        |  (rolagem)  |p|                                                  |p|  (rolagem)  |
//        +-------------+-+--------------------------------------------------+-+-------------+
//        s/p = borda arrastável (splitter)
// =============================================================================
#pragma once

#include <string>

namespace viewer {

struct RectF {
    float x = 0, y = 0, w = 0, h = 0;
    bool contains(float px, float py) const { return px >= x && px < x + w && py >= y && py < y + h; }
    float right() const { return x + w; }
    float bottom() const { return y + h; }
};

enum class DockSide { Left, Right };

class DockLayout {
public:
    static constexpr float kTopBar = 52.0f;          ///< altura reservada à barra de controle [px]
    static constexpr float kMinDock = 340.0f;        ///< largura mínima de uma coluna aberta
    static constexpr float kMaxDockFraction = 0.45f; ///< máximo: fração da largura da janela
    static constexpr float kMinViewport = 320.0f;    ///< a cena 3D nunca fica mais estreita que isto
    static constexpr float kSplitter = 6.0f;         ///< espessura da borda arrastável
    static constexpr float kCollapsed = 28.0f;       ///< largura da faixa de uma coluna recolhida
    static constexpr float kDefaultLeft = 400.0f;
    static constexpr float kDefaultRight = 380.0f;

    /// Tamanho da janela (valores não finitos ou < 1 são ignorados). Reaplica os limites.
    void setWindow(float w, float h);

    float leftWidth() const { return leftW_; }      ///< largura escolhida (mesmo recolhida)
    float rightWidth() const { return rightW_; }
    bool collapsed(DockSide s) const { return s == DockSide::Left ? leftCollapsed_ : rightCollapsed_; }
    void setCollapsed(DockSide s, bool c);
    void toggleCollapsed(DockSide s) { setCollapsed(s, !collapsed(s)); }

    /// Ajusta a largura da coluna, limitada a [kMinDock, min(kMaxDockFraction·janela, janela − outra − kMinViewport)].
    void setWidth(DockSide s, float w);

    /// Largura realmente ocupada (kCollapsed se recolhida; 0 se a janela for pequena demais para a coluna).
    float effectiveWidth(DockSide s) const;

    RectF dock(DockSide s) const;          ///< retângulo da coluna (sem a borda arrastável)
    RectF splitter(DockSide s) const;      ///< a borda arrastável (não existe se recolhida: w = 0)
    RectF viewport() const;                ///< a cena 3D
    RectF topBar() const;

    // Arrastar a borda: beginDrag quando o botão desce sobre o splitter; dragTo a cada quadro; endDrag ao soltar.
    bool hitSplitter(float mx, float my, DockSide& side) const;   ///< (com tolerância de 3 px de cada lado)
    void beginDrag(DockSide s, float mouseX);
    void dragTo(float mouseX);
    void endDrag() { dragging_ = false; }
    bool dragging() const { return dragging_; }
    DockSide dragSide() const { return dragSide_; }

    /// Largura mínima da janela para a configuração atual (as duas colunas abertas no mínimo + viewport mínimo).
    static float minWindowWidth() { return 2.0f * kMinDock + kMinViewport + 2.0f * kSplitter; }

    // Arquivo de layout ("viewer_layout 1\nleft 400\nright 380\nleft_collapsed 0\nright_collapsed 0\n").
    std::string serialize() const;
    /// Lê; valores ausentes ficam como estão; inválidos (NaN, fora da faixa) -> false e nada muda.
    bool parse(const std::string& text);

private:
    float maxWidthFor(DockSide s) const;
    void reclamp();

    float winW_ = 1280.0f, winH_ = 720.0f;
    float leftW_ = kDefaultLeft, rightW_ = kDefaultRight;
    bool leftCollapsed_ = false, rightCollapsed_ = false;
    bool dragging_ = false;
    DockSide dragSide_ = DockSide::Left;
    float dragGrabOffset_ = 0.0f;   ///< distância entre o mouse e a borda no início do arrasto
};

/// Rolagem vertical de uma coluna: conteúdo mais alto que a área visível.
class ScrollState {
public:
    static constexpr float kBarWidth = 12.0f;
    static constexpr float kWheelStep = 48.0f;   ///< px por "clique" da roda

    /// Define as alturas (conteúdo e área visível) e mantém o deslocamento válido.
    void setSizes(float contentH, float viewH);
    float offset() const { return offset_; }
    float maxOffset() const { return contentH_ > viewH_ ? contentH_ - viewH_ : 0.0f; }
    bool scrollable() const { return contentH_ > viewH_ + 0.5f; }
    void setOffset(float o);
    void wheel(float notches) { setOffset(offset_ - notches * kWheelStep); }   ///< roda para cima (positiva) sobe

    // Barra: trilho de altura `trackH`; o polegar é proporcional à fração visível (mínimo 24 px).
    float thumbHeight(float trackH) const;
    float thumbTop(float trackH) const;          ///< posição do topo do polegar dentro do trilho
    /// Arrastou o polegar `dy` pixels: converte para deslocamento do conteúdo.
    void dragThumb(float dy, float trackH);
    /// Clique no trilho (fora do polegar): salta para o ponto clicado (y relativo ao topo do trilho).
    void jumpTo(float yInTrack, float trackH);

private:
    float contentH_ = 0, viewH_ = 0, offset_ = 0;
};

} // namespace viewer
