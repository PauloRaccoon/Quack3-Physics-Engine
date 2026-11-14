// =============================================================================
// main.cpp — janela do physics_viewer (V5)
//
// O que mostra: cenas selecionáveis (caixa, pilha, pirâmide, chuva, pêndulo, corrente,
// poliedros). A física roda em passo fixo
// (PhysicsWorld::update acumula o tempo de render); o desenho usa a pose INTERPOLADA
// (lerp na posição + slerp no quatérnio), então o movimento é suave com qualquer FPS.
//
// Onde o quatérnio entra: a orientação do corpo é um quatérnio do início ao fim. A matriz
// que vai para a placa de vídeo é calculada por viewer::toMat4(), que usa o
// Quaternion::toMatrix() do PRÓPRIO motor (não o da raylib) e é entregue ao OpenGL por
// rlMultMatrixf().
//
// V3 — o quatérnio na tela: painel com x y z w, |q|, eixo-ângulo e ângulos de Euler do corpo
// em foco; chave do modo de interpolação de render (slerp / lerp sem normalizar / Euler);
// slider do FPS de render. As cenas "Gimbal lock" e "Interpolacao" usam passo físico largo
// para que a diferença entre os modos seja visível.
//
// V4 — interação: botão esquerdo seleciona um corpo (raycast do motor) e o arrasta com uma mola
// (força aplicada no ponto clicado: puxar pela quina também gira o corpo); B / O lançam uma
// caixa / esfera na direção do mouse; Delete ou X apagam o selecionado. A câmera gira ao redor
// do ALVO com o botão direito ou com a roda do mouse pressionada; T alterna o alvo entre o
// centro do mundo e o corpo selecionado; Home reseta a câmera.
//
// V5 — barra de controle (topo): Pausar/Continuar, Passo (1 passo físico fixo), Reiniciar, escala de
// tempo (1x, 1/2, 1/4, 1/10 = câmera lenta), cena anterior/próxima, Resetar câmera e Alvo da
// câmera (mundo / corpo selecionado). O tempo é controlado por viewer::TimeControl.
//
// V6 — painel do objeto selecionado (à direita, abaixo da interpolação): abas Material (massa, atrito,
// restituição, amortecimentos, escala de gravidade, pode dormir), Posicao, Veloc. (linear e angular) e
// Orient. (eixo-ângulo → quatérnio, botão Aplicar). A edição passa por viewer/core/BodyEditor.h.
//
// V7 — painel do mundo (esquerda, embaixo; tecla M mostra/esconde): abas Mundo (gravidade, Δt fixo,
// sleeping), Solver (iterações de velocidade e de posição, slops, warm start), Debug (as 9 opções do
// debug draw do motor desenhadas com DrawLine3D, mais "esconder modelos") e Stats (contagens e tempos
// por fase). A edição passa por viewer/core/WorldEditor.h.
//
// V8 — painel "Adicionar objeto" (direita, abas Objeto | Adicionar): tipo (caixa, esfera, tetraedro,
// octaedro, prisma, pirâmide), tamanho, massa, atrito, restituição, posição, orientação inicial (eixo-ângulo
// OU quatérnio w x y z, que é normalizado) e velocidades linear/angular. "Adicionar" cria no ponto dado;
// "posicionar com clique" cria onde o mouse aponta (com prévia em fio); "Lancar da camera" dispara o objeto
// na direção para onde a câmera olha. A criação passa por viewer/core/ObjectSpawner.h.
//
// V9 — eventos e sleeping: o viewer escuta os eventos de contato do motor (IContactListener). Corpos
// adormecidos ficam azulados; no INÍCIO de um contato o corpo pisca em branco-amarelado, mais forte quanto
// maior a velocidade de impacto (8 m/s = máximo), com um anel no ponto do impacto. A aba "Evento" do painel
// do mundo liga/desliga cada efeito e mostra contadores. Lógica em viewer/core/ContactTracker.h.
//
// V12 — salvar e carregar cenas: aba "Cenas" do painel da direita. Digite um nome, "Salvar" grava o ESTADO ATUAL
// (poses, velocidades, objetos adicionados, parâmetros do mundo) em scenes/<nome>.scene; a lista mostra os
// arquivos da pasta e "Carregar" substitui a cena (começa pausada; R reinicia a partir do arquivo). Enquanto o
// campo do nome está em edição os atalhos de teclado ficam desligados. Formato e leitura em viewer/core/SceneFile.h.
//
// V13 — painéis acoplados: duas colunas FIXAS nas laterais (esquerda: quatérnio e mundo; direita: interpolação,
// objeto, adicionar e cenas), cada uma com barra de rolagem (roda do mouse sobre a coluna, ou arrastar a barra);
// arrastar a borda de uma coluna muda sua largura e a cena 3D ocupa o espaço que sobra (renderizada numa textura
// do tamanho exato do viewport); o botão "<" / ">" recolhe a coluna. O layout é salvo em scenes/viewer_layout.ini.
// A geometria e a rolagem estão em viewer/core/DockLayout.h (testadas sem janela).
//
// Controles: Espaço ou P = pausar/continuar; . (ponto) = um passo (pausa se estiver rodando);
//            esquerdo = selecionar/arrastar (roda = profundidade durante o arrasto); direito ou
//            roda pressionada + mover = girar a câmera; roda = zoom; T = alvo da câmera;
//            Home = resetar câmera; B/O = lançar caixa/esfera; Delete/X = apagar;
//            ←/→ ou 1–9 = trocar de cena; R = reiniciar a cena; I = modo de interpolação;
//            M = painel do mundo; F11 = tela cheia sem borda; ESC = sair.
//
// V10 — fechamento: janela redimensionável (mínimo 960 x 700; os painéis se reorganizam abaixo de 1320 px de
// largura), F11 alterna tela cheia sem borda, e o ctest roda os testes do núcleo e a fumaça sem janela.
//
// Verificação automática (sem interação):
//     physics_viewer --frames 240 --screenshot saida.png
// roda 240 quadros, salva uma captura da tela e sai com código 0.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>

#include "raylib.h"
#include "rlgl.h"

// raygui: biblioteca de um só cabeçalho; a implementação é compilada neste arquivo.
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "viewer/core/BodyEditor.h"
#include "viewer/core/ContactTracker.h"
#include "viewer/core/DockLayout.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/JointEditor.h"
#include "viewer/core/ModelAsset.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/OrbitCamera.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/TimeControl.h"
#include "viewer/core/TrajectoryRecorder.h"
#include "viewer/core/ModelImporter.h"
#include "viewer/core/PanelMetrics.h"
#include "viewer/core/AppInfo.h"
#include "viewer/core/Arcball.h"
#include "viewer/core/SceneLibrary.h"
#include "viewer/core/SliderText.h"
#include "viewer/core/WorldEditor.h"
#include "viewer/core/Scene.h"
#include "viewer/core/SceneFile.h"
#include "viewer/core/ViewerMath.h"

namespace {

constexpr int kScreenW = 1280;
constexpr int kScreenH = 720;

Vector3 toRay(const viewer::Vec3f& v) { return Vector3{v.x, v.y, v.z}; }

// Cores de texto. Os painéis do raygui são CLAROS (cinza), então o texto dentro deles é ESCURO. O texto que fica
// direto sobre a cena 3D (HUD) é claro e ganha uma faixa escura translúcida (hudText), senão some no chão cinza.
// V23: tema cinza-grafite (antes os painéis eram brancos). Texto claro sobre superfícies cinza escuras.
constexpr Color kInk{226, 229, 236, 255};      // texto principal nos painéis
constexpr Color kInkDim{150, 157, 172, 255};   // texto secundário nos painéis
constexpr Color kAccent{255, 176, 80, 255};    // valores do quatérnio
constexpr Color kWarn{255, 112, 112, 255};     // avisos (|q| != 1, gimbal lock)
constexpr Color kBlueInk{130, 185, 255, 255};
constexpr Color kDockBg{32, 34, 41, 255};      // fundo das colunas laterais
constexpr Color kSurface{68, 73, 86, 255};     // fundo dos painéis

/// V23: estilo do raygui (todos os controles) — cinza-grafite com destaque azul. Chamado uma vez após InitWindow.
void applyTheme() {
    const auto set = [](int prop, Color c) { GuiSetStyle(DEFAULT, prop, ColorToInt(c)); };
    set(BACKGROUND_COLOR, kSurface);
    set(LINE_COLOR, Color{78, 83, 97, 255});
    set(BASE_COLOR_NORMAL, Color{47, 50, 59, 255});   // botões, sliders e cabeçalhos
    set(BORDER_COLOR_NORMAL, Color{96, 102, 118, 255});
    set(TEXT_COLOR_NORMAL, kInk);
    set(BASE_COLOR_FOCUSED, Color{60, 65, 78, 255});
    set(BORDER_COLOR_FOCUSED, Color{130, 170, 255, 255});
    set(TEXT_COLOR_FOCUSED, Color{245, 247, 250, 255});
    set(BASE_COLOR_PRESSED, Color{54, 118, 228, 255});   // destaque: aba/botão ativo, miolo do slider
    set(BORDER_COLOR_PRESSED, Color{150, 190, 255, 255});
    set(TEXT_COLOR_PRESSED, Color{255, 255, 255, 255});
    set(BASE_COLOR_DISABLED, Color{52, 56, 66, 255});
    set(BORDER_COLOR_DISABLED, Color{70, 75, 88, 255});
    set(TEXT_COLOR_DISABLED, Color{122, 128, 142, 255});
    // Campo de texto em edição: fundo escuro com borda azul (senão ficaria todo azul).
    GuiSetStyle(TEXTBOX, BASE_COLOR_PRESSED, ColorToInt(Color{26, 28, 34, 255}));
    GuiSetStyle(TEXTBOX, BORDER_COLOR_PRESSED, ColorToInt(Color{130, 170, 255, 255}));
    GuiSetStyle(TEXTBOX, TEXT_COLOR_PRESSED, ColorToInt(Color{240, 242, 246, 255}));
    GuiSetStyle(TEXTBOX, BASE_COLOR_NORMAL, ColorToInt(Color{36, 39, 47, 255}));
    GuiSetStyle(LISTVIEW, BASE_COLOR_FOCUSED, ColorToInt(Color{62, 67, 80, 255}));
    GuiSetStyle(LISTVIEW, BORDER_COLOR_FOCUSED, ColorToInt(Color{62, 67, 80, 255}));
    GuiSetStyle(LISTVIEW, BASE_COLOR_PRESSED, ColorToInt(Color{54, 118, 228, 255}));
    GuiSetStyle(LISTVIEW, BORDER_COLOR_PRESSED, ColorToInt(Color{54, 118, 228, 255}));
}

/// V17: borda direita (px) onde o texto dos painéis das colunas deve terminar; cada coluna ajusta ao abrir.
float gTextRight = 100000.0f;

/// Texto de painel que NUNCA passa da coluna: encolhe a fonte (até 11 px) e, se ainda assim não couber, corta com "..".
/// Mesma assinatura de DrawText. (O layout é checado nos testes; isto cobre colunas estreitas e valores longos.)
void panelText(const char* text, int x, int y, int size, Color color) {
    const int maxW = static_cast<int>(gTextRight) - x;
    if (maxW <= 8) return;
    int sz = size;
    while (sz > 11 && MeasureText(text, sz) > maxW) --sz;
    if (MeasureText(text, sz) <= maxW) {
        DrawText(text, x, y + (size - sz) / 2, sz, color);
        return;
    }
    std::string t = text;
    while (t.size() > 3 && MeasureText((t + "..").c_str(), sz) > maxW) t.pop_back();
    t += "..";
    DrawText(t.c_str(), x, y + (size - sz) / 2, sz, color);
}

/// V23: botão cujo texto sempre cabe: se não couber com a fonte atual, usa uma fonte menor só neste botão (mín. 10 px).
int fitButton(Rectangle r, const char* text) {
    const int base = GuiGetStyle(DEFAULT, TEXT_SIZE);
    const Font font = GuiGetFont();
    const float spacing = static_cast<float>(GuiGetStyle(DEFAULT, TEXT_SPACING));
    int sz = base;
    while (sz > 10 && MeasureTextEx(font, text, static_cast<float>(sz), spacing).x > r.width - 12.0f) --sz;
    if (sz != base) GuiSetStyle(DEFAULT, TEXT_SIZE, sz);
    const int res = GuiButton(r, text);
    if (sz != base) GuiSetStyle(DEFAULT, TEXT_SIZE, base);
    return res;
}

/// V23: grupo de abas (rótulos separados por ';') com a fonte reduzida até o MAIOR rótulo caber em um botão.
int fitToggleGroup(Rectangle r, const char* labels, int* active) {
    const int base = GuiGetStyle(DEFAULT, TEXT_SIZE);
    const Font font = GuiGetFont();
    const float spacing = static_cast<float>(GuiGetStyle(DEFAULT, TEXT_SPACING));
    int sz = base;
    const std::string all = labels;
    std::size_t pos = 0;
    while (sz > 10) {
        bool fits = true;
        pos = 0;
        while (pos <= all.size()) {
            std::size_t e = all.find(';', pos);
            if (e == std::string::npos) e = all.size();
            if (MeasureTextEx(font, all.substr(pos, e - pos).c_str(), static_cast<float>(sz), spacing).x > r.width - 10.0f) { fits = false; break; }
            pos = e + 1;
        }
        if (fits) break;
        --sz;
    }
    if (sz != base) GuiSetStyle(DEFAULT, TEXT_SIZE, sz);
    const int res = GuiToggleGroup(r, labels, active);
    if (sz != base) GuiSetStyle(DEFAULT, TEXT_SIZE, base);
    return res;
}

/// V21: rótulo de linha de slider: nunca passa de `right` (onde começa a barra), senão o texto ficava por baixo dela.
void labelText(const char* text, float x, float y, float right) {
    const float saved = gTextRight;
    gTextRight = std::min(gTextRight, right);
    panelText(text, static_cast<int>(x), static_cast<int>(y), 16, kInk);
    gTextRight = saved;
}

/// V20: texto de painel quebrado em linhas (até `maxLines`; a última leva ".." se ainda sobrar). Prefere quebrar depois
/// de '/', '\\', '_' ou espaço, para caminhos de arquivo ficarem legíveis. Devolve a altura usada (px).
float wrapPanelText(const std::string& text, int x, int y, int size, Color color, int maxLines) {
    const int maxW = static_cast<int>(gTextRight) - x;
    const std::vector<std::string> lines = viewer::wrapText(text, maxW, maxLines, [&](const std::string& t) { return MeasureText(t.c_str(), size); });
    for (std::size_t i = 0; i < lines.size(); ++i) DrawText(lines[i].c_str(), x, y + static_cast<int>(i) * (size + 2), size, color);
    return static_cast<float>(lines.size() * static_cast<std::size_t>(size + 2));
}

/// Texto sobre a cena: faixa escura translúcida atrás (mesma assinatura de DrawText).
void hudText(const char* text, int x, int y, int size, Color color) {
    const int w = MeasureText(text, size);
    DrawRectangle(x - 6, y - 3, w + 12, size + 6, Color{10, 12, 18, 190});
    DrawText(text, x, y, size, color);
}

/// V20: edição de um slider por clique duplo. Só existe UM campo de texto ativo por vez; ele é identificado
/// pelo retângulo da barra (se o layout mudar — troca de aba, rolagem —, a edição é cancelada).
struct SliderEditState {
    bool active = false;
    bool seen = false;          ///< o slider em edição foi desenhado neste quadro?
    Rectangle rect{0, 0, 0, 0};
    char buf[64] = {0};
    // detecção do clique duplo
    Rectangle lastRect{0, 0, 0, 0};
    double lastTime = -10.0;
    float pressValue = 0.0f;    ///< valor ANTES do primeiro clique (o primeiro clique move a barra; o duplo o desfaz)
};
SliderEditState gSE;

bool sameRect(const Rectangle& a, const Rectangle& b) {
    return std::fabs(a.x - b.x) < 0.5f && std::fabs(a.y - b.y) < 0.5f && std::fabs(a.width - b.width) < 0.5f;
}

/// Início de cada quadro: edição cujo slider não foi mais desenhado (aba trocada, painel rolado) é cancelada.
void sliderEditBeginFrame() {
    if (gSE.active && !gSE.seen) gSE.active = false;
    gSE.seen = false;
}

/// Slider com a regra do painel: só conta como edição do usuário se o valor mudou COM o botão apertado
/// e o mouse sobre a barra (o raygui limita valores fora da faixa, e isso não pode alterar o mundo).
/// V20: clique duplo rápido abre um campo de texto no lugar da barra: Enter ou clique fora aplica (limitado a [lo, hi]),
/// Esc cancela. V24: o campo já vem com o valor atual (editável) e aceita contas ("4/2", "1-4", "2*(3+1)"). Devolve true quando o usuário mudou o valor (arrastando ou digitando).
bool sliderBox(const Rectangle& bar, float& v, float lo, float hi, viewer::SliderUnit unit = viewer::SliderUnit{}) {
    const float before = v;
    const Vector2 mp = GetMousePosition();
    if (gSE.active && sameRect(gSE.rect, bar)) {   // --- este slider está sendo editado: campo de texto
        gSE.seen = true;
        if (GuiTextBox(bar, gSE.buf, static_cast<int>(sizeof(gSE.buf)) - 1, true)) {   // Enter ou clique fora
            gSE.active = false;
            float nv = 0.0f;
            if (!viewer::sliderValueFromText(gSE.buf, unit, lo, hi, nv)) return false;   // texto inválido: mantém o valor
            v = nv;
            return v != before;
        }
        return false;
    }
    const bool hover = !GuiIsLocked() && CheckCollisionPointRec(mp, bar);
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        const double now = GetTime();
        if (now - gSE.lastTime < 0.4 && sameRect(gSE.lastRect, bar)) {   // clique duplo: edição por texto
            v = gSE.pressValue;   // desfaz o que o primeiro clique fez na barra
            std::snprintf(gSE.buf, sizeof(gSE.buf), "%s", viewer::sliderTextFromValue(v, unit).c_str());
            gSE.active = true;
            textBoxCursorIndex = static_cast<int>(std::strlen(gSE.buf));   // V24: o valor atual continua no campo; cursor no fim
            gSE.seen = true;
            gSE.rect = bar;
            gSE.lastTime = -10.0;
            GuiTextBox(bar, gSE.buf, static_cast<int>(sizeof(gSE.buf)) - 1, true);   // desenha já neste quadro
            return v != before;
        }
        gSE.lastRect = bar;
        gSE.lastTime = now;
        gSE.pressValue = v;
    }
    GuiSlider(bar, nullptr, nullptr, &v, lo, hi);
    return v != before && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && hover;
}

/// V20: caixa de seleção cujo rótulo NUNCA passa da coluna (encolhe a fonte ou corta, como panelText) e que também
/// alterna ao clicar no rótulo. `right` = borda direita do texto (por padrão a da coluna).
void checkLabel(float x, float y, const char* label, bool* on, float right = -1.0f) {
    GuiCheckBox(Rectangle{x, y, 20.0f, 20.0f}, "", on);
    while (*label == ' ') ++label;
    const float savedRight = gTextRight;
    if (right > 0.0f) gTextRight = std::min(gTextRight, right);
    panelText(label, static_cast<int>(x) + 26, static_cast<int>(y) + 2, 16, kInk);
    const float labelRight = std::max(x + 30.0f, gTextRight);
    gTextRight = savedRight;
    const Rectangle hit{x + 22.0f, y, labelRight - (x + 22.0f), 20.0f};
    if (!GuiIsLocked() && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(GetMousePosition(), hit)) *on = !*on;
}

// V15b — modelos 3D importados: texturas e grupos de malha em cache (a janela é a única parte que toca a GPU).
struct ModelDrawCache {
    std::map<std::string, Texture2D> textures;   // chave: caminho do arquivo; id 0 = não carregou (não tenta de novo)
    struct Entry {
        std::shared_ptr<const viewer::ModelAsset> asset;   // mantém o asset vivo enquanto a chave (ponteiro) valer
        std::vector<viewer::MeshGroup> groups;
    };
    std::map<const viewer::ModelAsset*, Entry> groups;
};
ModelDrawCache gModelCache;
bool gShowHullWire = false;   // fio do fecho convexo por cima do modelo (mostra com o que ele REALMENTE colide)

void clearModelMeshes() { gModelCache.groups.clear(); }

void unloadModelCache() {
    for (auto& kv : gModelCache.textures)
        if (kv.second.id != 0) UnloadTexture(kv.second);
    gModelCache.textures.clear();
    gModelCache.groups.clear();
}

/// Textura do caminho (carrega uma vez; mipmaps e repetição). id 0 se faltar o arquivo ou for enorme.
Texture2D modelTexture(const std::string& path) {
    const auto it = gModelCache.textures.find(path);
    if (it != gModelCache.textures.end()) return it->second;
    Texture2D tex{};
    if (std::filesystem::exists(path)) {
        Image img = LoadImage(path.c_str());
        if (img.data != nullptr && img.width > 0 && img.height > 0 && img.width <= 8192 && img.height <= 8192) {
            tex = LoadTextureFromImage(img);
            if (tex.id != 0) {
                GenTextureMipmaps(&tex);
                SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);
                SetTextureWrap(tex, TEXTURE_WRAP_REPEAT);
            }
        }
        if (img.data != nullptr) UnloadImage(img);
    }
    gModelCache.textures[path] = tex;
    return tex;
}

/// Malha do modelo (já no referencial do corpo: a matriz do corpo foi aplicada por quem chama). Cada triângulo
/// leva o tom da luz fixa conforme a normal NO MUNDO, como nos poliedros; a textura multiplica a cor.
void drawModelMesh(const viewer::SceneBody& body, const viewer::Mat4f& m) {
    const viewer::ModelAsset& asset = *body.model;
    auto it = gModelCache.groups.find(&asset);
    if (it == gModelCache.groups.end()) {
        if (gModelCache.groups.size() > 64) gModelCache.groups.clear();
        it = gModelCache.groups.emplace(&asset, ModelDrawCache::Entry{body.model, viewer::buildMeshGroups(asset)}).first;
    }
    const viewer::Vec3f light{0.4f, 0.8f, 0.45f};
    rlDisableBackfaceCulling();   // modelos de terceiros nem sempre têm o sentido das faces certo
    for (const viewer::MeshGroup& g : it->second.groups) {
        Color base{body.r, body.g, body.b, 255};
        unsigned int texId = 0;
        if (g.material >= 0 && asset.obj && static_cast<std::size_t>(g.material) < asset.obj->materials.size()) {
            const viewer::ObjMaterial& mat = asset.obj->materials[static_cast<std::size_t>(g.material)];
            if (!mat.diffuseMap.empty()) texId = modelTexture((std::filesystem::path(asset.baseDir) / mat.diffuseMap).string()).id;
            if (texId != 0) base = Color{255, 255, 255, 255};
            else if (mat.hasKd) base = Color{static_cast<unsigned char>(mat.kd[0] * 255.0f), static_cast<unsigned char>(mat.kd[1] * 255.0f),
                                             static_cast<unsigned char>(mat.kd[2] * 255.0f), 255};
        }
        for (std::size_t t = 0; t < g.triangleCount(); ++t) {
            const float* n = &g.normals[t * 3];
            const float nx = m.m[0] * n[0] + m.m[4] * n[1] + m.m[8] * n[2];
            const float ny = m.m[1] * n[0] + m.m[5] * n[1] + m.m[9] * n[2];
            const float nz = m.m[2] * n[0] + m.m[6] * n[1] + m.m[10] * n[2];
            // sem culling o triângulo pode estar "de costas": usa o valor absoluto para as duas faces ficarem iluminadas
            float k = 0.55f + 0.45f * std::fabs(nx * light.x + ny * light.y + nz * light.z);
            if (k < 0.35f) k = 0.35f;
            if (k > 1.0f) k = 1.0f;
            const Color c{static_cast<unsigned char>(base.r * k), static_cast<unsigned char>(base.g * k), static_cast<unsigned char>(base.b * k), 255};
            rlCheckRenderBatchLimit(3);
            rlSetTexture(texId);
            rlBegin(RL_TRIANGLES);
            rlColor4ub(c.r, c.g, c.b, 255);
            for (int v = 0; v < 3; ++v) {
                const float* p = &g.positions[t * 9 + static_cast<std::size_t>(v) * 3];
                const float* uv = &g.texcoords[t * 6 + static_cast<std::size_t>(v) * 2];
                rlTexCoord2f(uv[0], uv[1]);
                rlVertex3f(p[0], p[1], p[2]);
            }
            rlEnd();
        }
        rlSetTexture(0);
    }
    rlEnableBackfaceCulling();
}

/// V18: cápsula/cilindro de eixo Y local, na origem do corpo. `grow` aumenta um pouco (contorno de seleção).
void drawAxisWire(viewer::ShapeKind kind, float r, float hh, Color c, float grow = 1.0f) {
    r *= grow;
    if (kind == viewer::ShapeKind::Capsule) {
        const float seg = hh * grow;
        if (seg < 1e-4f) DrawSphereWires(Vector3{0, 0, 0}, r, 10, 14, c);   // segmento nulo: raylib dividiria por zero
        else DrawCapsuleWires(Vector3{0, -seg, 0}, Vector3{0, seg, 0}, r, 14, 8, c);
    } else {
        const float h = hh * grow;
        DrawCylinderWires(Vector3{0, -h, 0}, r, r, 2.0f * h, 18, c);
    }
}

void drawAxisSolid(const viewer::SceneBody& body, Color fill, Color wire) {
    if (body.kind == viewer::ShapeKind::Capsule) {
        if (body.halfHeight < 1e-4f) {
            DrawSphereEx(Vector3{0, 0, 0}, body.radius, 16, 16, fill);
            DrawSphereWires(Vector3{0, 0, 0}, body.radius * 1.002f, 16, 16, wire);
        } else {
            DrawCapsule(Vector3{0, -body.halfHeight, 0}, Vector3{0, body.halfHeight, 0}, body.radius, 18, 10, fill);
            DrawCapsuleWires(Vector3{0, -body.halfHeight, 0}, Vector3{0, body.halfHeight, 0}, body.radius * 1.002f, 18, 10, wire);
        }
    } else {
        DrawCylinder(Vector3{0, -body.halfHeight, 0}, body.radius, body.radius, 2.0f * body.halfHeight, 24, fill);
        DrawCylinderWires(Vector3{0, -body.halfHeight, 0}, body.radius * 1.002f, body.radius * 1.002f, 2.0f * body.halfHeight, 24, wire);
    }
}

/// Desenha um corpo na pose `m` (matriz de modelo vinda do quatérnio do motor).
void drawBody(const viewer::SceneBody& body, const viewer::Mat4f& m) {
    const Color fill{body.r, body.g, body.b, 255};
    const Color wire{static_cast<unsigned char>(body.r / 3), static_cast<unsigned char>(body.g / 3),
                     static_cast<unsigned char>(body.b / 3), 255};

    rlPushMatrix();
    rlMultMatrixf(m.m);   // 16 floats em ordem de colunas: a pose inteira (rotação + translação)

    if (body.model && body.kind != viewer::ShapeKind::Polyhedron) {   // V17: malha do modelo com colisor em caixa/esfera
        drawModelMesh(body, m);
        if (gShowHullWire) {   // o colisor em fio, para ver a folga entre a forma física e a malha
            const Color cw{255, 200, 60, 255};
            if (body.kind == viewer::ShapeKind::Box)
                DrawCubeWires(Vector3{0, 0, 0}, 2.0f * body.halfExtents.x, 2.0f * body.halfExtents.y, 2.0f * body.halfExtents.z, cw);
            else if (body.kind == viewer::ShapeKind::Capsule || body.kind == viewer::ShapeKind::Cylinder)
                drawAxisWire(body.kind, body.radius, body.halfHeight, cw);
            else
                DrawSphereWires(Vector3{0, 0, 0}, body.radius, 12, 16, cw);
        }
    } else if (body.kind == viewer::ShapeKind::Capsule || body.kind == viewer::ShapeKind::Cylinder) {   // V18
        drawAxisSolid(body, fill, wire);
    } else if (body.kind == viewer::ShapeKind::Box) {
        const float w = 2.0f * body.halfExtents.x;
        const float h = 2.0f * body.halfExtents.y;
        const float l = 2.0f * body.halfExtents.z;
        DrawCube(Vector3{0, 0, 0}, w, h, l, fill);
        DrawCubeWires(Vector3{0, 0, 0}, w, h, l, wire);
    } else if (body.kind == viewer::ShapeKind::Polyhedron && body.poly) {
        const physics::HalfEdgeStructure& topo = body.poly->getTopology();
        const bool asModel = body.model != nullptr;   // V15b: a malha do .obj no lugar do fecho
        if (asModel) drawModelMesh(body, m);
        // Cada face vira um leque de triângulos (anti-horário visto de fora). O tom varia com a
        // normal da face NO MUNDO (rotação da matriz m), simulando uma luz fixa.
        const viewer::Vec3f light{0.4f, 0.8f, 0.45f};
        for (const physics::HEFace& f : topo.faces()) {
            if (asModel) break;
            const viewer::Vec3f nl{static_cast<float>(f.normal.x), static_cast<float>(f.normal.y),
                                   static_cast<float>(f.normal.z)};
            const viewer::Vec3f nw{m.m[0] * nl.x + m.m[4] * nl.y + m.m[8] * nl.z,
                                   m.m[1] * nl.x + m.m[5] * nl.y + m.m[9] * nl.z,
                                   m.m[2] * nl.x + m.m[6] * nl.y + m.m[10] * nl.z};
            float k = 0.55f + 0.45f * (nw.x * light.x + nw.y * light.y + nw.z * light.z);
            if (k < 0.35f) k = 0.35f;
            if (k > 1.0f) k = 1.0f;
            const Color c{static_cast<unsigned char>(body.r * k), static_cast<unsigned char>(body.g * k),
                          static_cast<unsigned char>(body.b * k), 255};
            const int e0 = f.edge;
            const physics::Vector3& p0 = topo.getVertex(topo.getEdge(e0).origin).position;
            int e = topo.getEdge(e0).next;
            while (topo.getEdge(e).next != e0) {
                const physics::Vector3& p1 = topo.getVertex(topo.getEdge(e).origin).position;
                const physics::Vector3& p2 = topo.getVertex(topo.destination(e)).position;
                DrawTriangle3D(Vector3{float(p0.x), float(p0.y), float(p0.z)},
                               Vector3{float(p1.x), float(p1.y), float(p1.z)},
                               Vector3{float(p2.x), float(p2.y), float(p2.z)}, c);
                e = topo.getEdge(e).next;
            }
        }
        for (int i = 0; i < topo.getEdgeCount() && (!asModel || gShowHullWire); ++i) {   // cada aresta uma vez (half-edge < twin)
            if (i > topo.getEdge(i).twin) continue;
            const physics::Vector3& a = topo.getVertex(topo.getEdge(i).origin).position;
            const physics::Vector3& b = topo.getVertex(topo.destination(i)).position;
            DrawLine3D(Vector3{float(a.x), float(a.y), float(a.z)}, Vector3{float(b.x), float(b.y), float(b.z)},
                       wire);
        }
    } else {
        // Os meridianos/paralelos (wires) deixam a rotação da esfera visível.
        DrawSphereEx(Vector3{0, 0, 0}, body.radius, 16, 16, fill);
        DrawSphereWires(Vector3{0, 0, 0}, body.radius * 1.002f, 16, 16, wire);
    }
    rlPopMatrix();
}

/// Contorno amarelo do corpo selecionado (um pouco maior que o corpo).
void drawHighlight(const viewer::SceneBody& body, const viewer::Mat4f& m) {
    const Color c{255, 235, 60, 255};
    rlPushMatrix();
    rlMultMatrixf(m.m);
    if (body.kind == viewer::ShapeKind::Box) {
        DrawCubeWires(Vector3{0, 0, 0}, 2.0f * body.halfExtents.x * 1.06f, 2.0f * body.halfExtents.y * 1.06f,
                      2.0f * body.halfExtents.z * 1.06f, c);
    } else if (body.kind == viewer::ShapeKind::Polyhedron && body.poly) {
        const physics::HalfEdgeStructure& topo = body.poly->getTopology();
        for (int i = 0; i < topo.getEdgeCount(); ++i) {
            if (i > topo.getEdge(i).twin) continue;
            const physics::Vector3& a = topo.getVertex(topo.getEdge(i).origin).position;
            const physics::Vector3& b = topo.getVertex(topo.destination(i)).position;
            DrawLine3D(Vector3{float(a.x) * 1.05f, float(a.y) * 1.05f, float(a.z) * 1.05f},
                       Vector3{float(b.x) * 1.05f, float(b.y) * 1.05f, float(b.z) * 1.05f}, c);
        }
    } else if (body.kind == viewer::ShapeKind::Capsule || body.kind == viewer::ShapeKind::Cylinder) {   // V18
        drawAxisWire(body.kind, body.radius, body.halfHeight, c, 1.06f);
    } else {
        DrawSphereWires(Vector3{0, 0, 0}, body.radius * 1.08f, 12, 12, c);
    }
    rlPopMatrix();
}

/// Eixos locais do corpo (X vermelho, Y verde, Z azul), tirados da matriz de rotação.
void drawLocalAxes(const viewer::Mat4f& m, float length) {
    const viewer::Vec3f origin{m.m[12], m.m[13], m.m[14]};
    const Color colors[3] = {RED, GREEN, BLUE};
    for (int c = 0; c < 3; ++c) {
        const viewer::Vec3f a = viewer::axisOf(m, c);
        const viewer::Vec3f tip{origin.x + a.x * length, origin.y + a.y * length, origin.z + a.z * length};
        DrawLine3D(toRay(origin), toRay(tip), colors[c]);
    }
}

} // namespace

int main(int argc, char** argv) {
    // --- Argumentos (só para a verificação automática) --------------------------------------------------
    int autoFrames = 0;                 // > 0: roda N quadros e sai
    const char* screenshotPath = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) autoFrames = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) screenshotPath = argv[++i];
    }

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(kScreenW, kScreenH, viewer::appinfo::windowTitle().c_str());   // "Quack3 v1.0.0"
    SetWindowMinSize(1100, 700);   // duas colunas de 340 + cena de 320 + bordas; abaixo disso as colunas encolhem
    int fpsApplied = 120;
    float fpsSlider = 120.0f;
    SetTargetFPS(fpsApplied);   // o render NÃO precisa casar com a taxa da física: há interpolação
    GuiSetStyle(DEFAULT, TEXT_SIZE, 16);
    applyTheme();   // V23
    viewer::InterpMode mode = viewer::InterpMode::Slerp;

    viewer::OrbitCamera orbit;
#ifdef VIEWER_SCENES_DIR
    const std::string sceneDir = VIEWER_SCENES_DIR;
#else
    const std::string sceneDir = "scenes";
#endif
#ifdef VIEWER_MODELS_DIR
    const std::string modelDir = VIEWER_MODELS_DIR;
#else
    const std::string modelDir = "models";
#endif
    // V22: lista única de cenas = originais + arquivos .scene salvos; ordem e remoções ficam em scenes/biblioteca.txt.
    // (sceneDir/modelDir são declarados acima, antes do bloco da janela.)
    viewer::SceneLibrary lib;
    const std::string libPath = sceneDir + "/" + viewer::scenelibrary::kFileName;
    {
        std::string saved;
        std::ifstream f(libPath);
        if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            saved = ss.str();
        }
        lib.rebuild(viewer::listSceneFiles(sceneDir), saved);
    }
    bool libDirty = false;
    std::size_t sceneIndex = 0;
    std::string startupMsg;
    viewer::ContactTracker tracker;   // V9: efeitos dos eventos de contato (vive mais que os mundos)
    bool tintOn = true, flashOn = true, ringsOn = true;
    viewer::Scene scene;
    for (std::size_t i = 0; i < lib.size(); ++i) {   // 1ª cena que carrega (um arquivo ruim é marcado e pulado)
        std::string err;
        if (viewer::buildLibraryScene(lib.at(i), sceneDir, modelDir, scene, err)) { sceneIndex = i; break; }
        lib.at(i).broken = true;
        startupMsg = "ERRO ao carregar " + lib.at(i).name + ": " + err;
        if (i + 1 == lib.size()) scene = viewer::makeScene(0);
    }
    scene.world->setContactListener(&tracker);
    orbit.target = scene.cameraTarget;
    orbit.setDistance(scene.cameraDistance);
    viewer::TimeControl timeCtl;
    const bool startPaused = autoFrames <= 0;   // cenas começam PAUSADAS (a verificação automática --frames roda direto)
    timeCtl.setPaused(startPaused);
    // Pedidos da barra de botões: o raygui devolve o clique durante o DESENHO, então viram flags
    // tratadas no início do quadro seguinte.
    bool uiRestart = false, uiStep = false, uiTogglePause = false, uiResetCam = false, uiPivot = false;
    int uiSceneDelta = 0;
    int frame = 0;
    physics::BodyId selected = physics::kInvalidBodyId;   // corpo selecionado (clique)
    viewer::SpringDrag drag;
    bool pivotSelected = false;                             // alvo da câmera: false = centro do mundo
    // Painel do objeto (V6): aba atual e o rascunho da orientação (eixo + ângulo), que só vai ao corpo em "Aplicar".
    // Painel do mundo (V7).
    bool showWorld = true;
    int worldTab = 0;
    bool profilingOn = false;
    bool hideModels = false;
    std::uint32_t dbgFlags = 0;                 // opções do debug draw ligadas (0 = desligado)
    physics::DebugLineBuffer dbgBuf;
    viewer::WorldParams sceneDefaults = viewer::captureWorld(*scene.world);   // "Restaurar padroes da cena"
    int objTab = 0;
    viewer::Vec3f camPan{};   // V19: deslocamento da câmera por WASD (soma ao alvo)
    // Painel "Adicionar objeto" (V8): aba principal (0 = Objeto, 1 = Adicionar) e o rascunho do que será criado.
    int mainTab = 0, addTab = 0, kindIdx = 0, qMode = 0, addFailFrames = 0;
    // Cenas salvas/carregadas (V12).
    std::vector<std::string> sceneFiles = viewer::listSceneFiles(sceneDir);
    // V15b: modelos 3D (aba Adicionar > Forma > "modelo 3D"). kindIdx == 8 (kSpawnKindCount) é o modelo.
    std::vector<std::string> modelFiles = viewer::listModelFiles(modelDir);
    int modelScroll = 0, modelActive = -1, modelLoaded = -2;
    std::shared_ptr<const viewer::ObjModel> modelObj;
    std::string modelObjFile, modelInfo1, modelInfo2, modelInfo3;
    bool modelInfoWarn = false, modelToggle = false;
    char fileName[49] = "";
    bool fileNameEdit = false;
    // V17: importar um arquivo 3D próprio (arrastar e soltar na janela ou digitar o caminho) e o tipo de colisor.
    char importPathBuf[300] = "";
    bool importEdit = false;
    std::string importMsg;
    bool importMsgErr = false;
    int importMsgFrames = 0;
    int colliderIdx = 0;   // 0 = fecho, 1 = caixa, 2 = esfera, 3 = capsula, 4 = cilindro (viewer::ColliderKind)
    float tuneR = 1.0f, tuneH = 1.0f;   // V18: ajuste fino do raio/altura do colisor capsula/cilindro
    int fileMsgFrames = 0;
    int libActive = -1, libScroll = 0, gotoScene = -1;   // V22: seleção na lista do painel Cenas, rolagem e pedido de troca
    bool delConfirm = false;                              // V22: "Apagar" pede um segundo clique
    viewer::RectF sceneListRect;                          // V22: a roda rola a lista de cenas, não a coluna
    bool fileMsgErr = false;
    std::string fileMsg;
    if (!startupMsg.empty()) { fileMsg = startupMsg; fileMsgErr = true; fileMsgFrames = 900; }
    // V13: layout em colunas, rolagem e o viewport 3D (textura do tamanho exato da área central).
    viewer::DockLayout dock;
    viewer::ScrollState scrollL, scrollR;
    int scrollDrag = -1;           // coluna cuja barra está sendo arrastada (0 = esquerda, 1 = direita)
    bool layoutDirty = false, orbiting = false;
    viewer::RectF fileListRect;    // última posição da lista de arquivos (a roda rola a lista, não a coluna)
    RenderTexture2D vpTex{};
    const std::string layoutPath = sceneDir + "/viewer_layout.ini";
    // V16: gravação da trajetória (CSV) e modo apresentação.
    const std::string exportDir = sceneDir + "/exports";
    viewer::TrajectoryRecorder recorder;
    int recTarget = 0;                       // 0 = corpo selecionado, 1 = todos os dinâmicos
    std::string recMsg;
    std::string recLastPath;   // V20: caminho completo do último CSV salvo (fica na tela até o próximo)
    int recMsgFrames = 0;
    int recFileCounter = 0;
    bool uiRecToggle = false, uiRecSave = false;
    bool presentation = false;               // F5: só a cena 3D em tela cheia + quatérnio grande
    bool autoOrbit = false;                  // F6: a câmera gira sozinha ao redor do alvo
    {
        std::ifstream f(layoutPath);
        if (f) {
            std::ostringstream ss;
            ss << f.rdbuf();
            dock.parse(ss.str());   // inválido: mantém o padrão
        }
    }
    bool customActive = false;   // a cena atual veio de um arquivo: R recria a partir de customText
    std::string customText;
    bool clickPlace = false;
    // V14 — editor de cena: estático/dimensões da caixa, seleção de estáticos, aba Juntas, cena nova.
    bool spStatic = false, pickStaticsOpt = false;
    float spAxisR = 0.5f, spAxisH = 2.0f;   // V18: capsula/cilindro: raio = tamanho x spAxisR; altura total = tamanho x spAxisH
    float spBox[3] = {1.0f, 1.0f, 1.0f};   // escala por eixo da caixa (aresta = tamanho x escala)
    int jKind = 0, jBMode = 0, jAnchorMode = 0, jMsgFrames = 0, newSceneRequest = 0;   // jBMode: 0 = mundo, 1 = corpo B
    physics::BodyId jB = physics::kInvalidBodyId;
    bool jPickB = false, jCollide = false, jHasClick = false, jMsgErr = false;
    float jAnchor[3] = {0.0f, 4.0f, 0.0f}, jAxis[3] = {0.0f, 0.0f, 1.0f};
    physics::Vector3 jClick;
    std::string jMsg;
    float sizeF = 1.0f, massS = 1.0f /* raiz da massa */, fricF = 0.5f, restF = 0.2f, launchSpd = 18.0f;
    float spPos[3] = {0.0f, 6.0f, 0.0f}, spVel[3] = {0.0f, 0.0f, 0.0f}, spAngV[3] = {0.0f, 0.0f, 0.0f};
    float spAxis[3] = {0.0f, 1.0f, 0.0f}, spAngDeg = 0.0f, spQ[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    physics::BodyId lastSelected = physics::kInvalidBodyId;
    float oAxisX = 0.0f, oAxisY = 1.0f, oAxisZ = 0.0f, oAngleDeg = 0.0f;
    int oMode = 0;                                   // V25: 0 = eixo-angulo, 1 = angulos omega x/y/z (graus)
    float oWx = 0.0f, oWy = 0.0f, oWz = 0.0f;       // V25: ωx ωy ωz [graus] em torno dos eixos fixos do mundo (X, depois Y, depois Z)

    // Pose de RENDER de um corpo: interpola os dois últimos passos físicos com o α atual, no modo escolhido.
    auto renderPose = [&](physics::BodyId id) {
        const physics::MotionState& ms = scene.world->getMotionState(id);
        return viewer::interpolatePose(ms.getPrevious(), ms.getCurrent(), timeCtl.renderAlpha(*scene.world), mode);
    };

    // V17: importa o que foi solto na janela ou digitado, atualiza a lista e já seleciona o 1º modelo importado.
    auto doImport = [&](const std::string& path) {
        std::vector<viewer::ImportResult> got;
        std::vector<std::string> msgs;
        const bool ok = viewer::importPath(path, modelDir, got, msgs);
        modelFiles = viewer::listModelFiles(modelDir);
        if (ok) {
            for (std::size_t i = 0; i < modelFiles.size(); ++i)
                if (modelFiles[i] == got[0].file) modelActive = static_cast<int>(i);
            modelLoaded = -2;
            mainTab = 1;
            addTab = 0;
            kindIdx = viewer::kSpawnKindCount;
            modelToggle = true;
            importMsg = "importado: " + got[0].file + (got.size() > 1 ? " (+" + std::to_string(got.size() - 1) + ")" : "") +
                        "  [.mtl: " + std::to_string(got[0].mtlCount) + ", texturas: " + std::to_string(got[0].textureCount) + "]";
            for (const std::string& m : msgs) importMsg += "  | " + m;   // V25: todos os avisos (antes só o 1o, e cortado)
            importMsgErr = false;
        } else {
            importMsg = msgs.empty() ? "ERRO: nada importado" : msgs[0];
            importMsgErr = true;
        }
        importMsgFrames = 1800;
    };

    // Troca a cena atual por `next` (cena carregada de arquivo ou nova): mesmo reinício de uma troca de cena.
    auto adoptScene = [&](viewer::Scene&& next) {
        scene = std::move(next);
        customText = viewer::serializeScene(scene);
        customActive = true;
        tracker.reset();
        scene.world->setContactListener(&tracker);
        sceneDefaults = viewer::captureWorld(*scene.world);
        scene.world->setProfilingEnabled(profilingOn);
        selected = physics::kInvalidBodyId;
        jB = physics::kInvalidBodyId;
        jBMode = 0;
        jPickB = false;
        jHasClick = false;
        clearModelMeshes();
        recorder.stop("cena trocada");
        drag.end();
        timeCtl.onSceneReset();
        timeCtl.setPaused(startPaused);
        orbit.target = scene.cameraTarget;
        camPan = viewer::Vec3f{};
        orbit.setDistance(scene.cameraDistance);
    };

    // V12: com o campo de nome em edição os atalhos de teclado ficam desligados (senão "r", "t", espaço... agiriam).
    SetExitKey(KEY_NULL);   // ESC não fecha a janela sozinho: fecha só fora da edição (ver abaixo)
    bool quitRequested = false;
    auto keyPressed = [&](int k) { return !fileNameEdit && !importEdit && !gSE.active && IsKeyPressed(k); };
    while (!WindowShouldClose() && !quitRequested) {
        sliderEditBeginFrame();   // V20
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (gSE.active) gSE.active = false;   // V20: Esc cancela a edição do slider (não fecha o programa nem aplica)
            else if (fileNameEdit || importEdit) fileNameEdit = importEdit = false;   // ESC sai da edição do texto
            else if (presentation) presentation = false;   // V16: ESC sai da apresentação antes de fechar o programa
            else quitRequested = true;
        }
        if (IsFileDropped()) {   // V17: arrastar um .obj (ou uma pasta com .obj) para a janela
            FilePathList dropped = LoadDroppedFiles();
            std::string all;
            for (unsigned i = 0; i < dropped.count && i < 8; ++i) {
                doImport(dropped.paths[i]);
                if (i == 0) all = importMsg;
            }
            if (dropped.count > 1) importMsg = all + (dropped.count > 8 ? "  (so os 8 primeiros)" : "");
            UnloadDroppedFiles(dropped);
        }
        // --- Layout (V13): colunas laterais, viewport 3D no meio; recalculado a cada quadro (janela redimensionável) ---
        const float screenW = static_cast<float>(GetScreenWidth()), screenH = static_cast<float>(GetScreenHeight());
        const Vector2 mouse = GetMousePosition();
        dock.setWindow(screenW, screenH);
        {   // bordas arrastáveis
            viewer::DockSide sd = viewer::DockSide::Left;
            const bool hoverSplit = !presentation && dock.hitSplitter(mouse.x, mouse.y, sd);
            if (!dock.dragging() && scrollDrag < 0 && hoverSplit && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) dock.beginDrag(sd, mouse.x);
            if (dock.dragging()) {
                if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    dock.dragTo(mouse.x);
                } else {
                    dock.endDrag();
                    layoutDirty = true;
                }
            }
            SetMouseCursor(dock.dragging() || hoverSplit ? MOUSE_CURSOR_RESIZE_EW : MOUSE_CURSOR_DEFAULT);
        }
        const viewer::RectF vp = presentation ? viewer::RectF{0.0f, 0.0f, screenW, screenH} : dock.viewport();   // V16: apresentação = janela inteira
        const viewer::RectF dockL = dock.dock(viewer::DockSide::Left), dockR = dock.dock(viewer::DockSide::Right);
        constexpr float kHdr = 26.0f, kPad = 8.0f, kBar = viewer::ScrollState::kBarWidth;
        const viewer::RectF bodyL{dockL.x, dockL.y + kHdr, dockL.w, std::max(0.0f, dockL.h - kHdr)};
        const viewer::RectF bodyR{dockR.x, dockR.y + kHdr, dockR.w, std::max(0.0f, dockR.h - kHdr)};
        const float innerWL = std::max(240.0f, dockL.w - 2.0f * kPad - kBar), innerWR = std::max(240.0f, dockR.w - 2.0f * kPad - kBar);
        const float objH = viewer::panel::objPanelHeight(mainTab, addTab, kindIdx == 0, kindIdx == viewer::kSpawnKindCount,
                                                                kindIdx == 6 || kindIdx == 7, kindIdx == viewer::kSpawnKindCount && colliderIdx >= 3);   // V17: altura calculada do conteúdo
        scrollL.setSizes(kPad + viewer::panel::kQuatH + kPad + viewer::panel::kArcballH + kPad + (showWorld ? viewer::panel::kWorldH + kPad : 0.0f) + viewer::panel::kRecH + kPad, bodyL.h);
        scrollR.setSizes(kPad + viewer::panel::kScenesH + kPad + viewer::panel::kInterpH + kPad + objH + kPad, bodyR.h);
        const float cxL = dockL.x + kPad, cyL = bodyL.y + kPad - scrollL.offset();
        const float cxR = dockR.x + kPad, cyR = bodyR.y + kPad - scrollR.offset();
        const Rectangle controlBar{16.0f, 8.0f, screenW - 32.0f, 36.0f};
        const Rectangle quatPanel{cxL, cyL, innerWL, viewer::panel::kQuatH};
        const Rectangle arcPanel{cxL, cyL + viewer::panel::kQuatH + kPad, innerWL, viewer::panel::kArcballH};
        const Rectangle worldPanel{cxL, arcPanel.y + viewer::panel::kArcballH + kPad, innerWL, viewer::panel::kWorldH};
        const Rectangle recPanel{cxL, arcPanel.y + viewer::panel::kArcballH + kPad + (showWorld ? viewer::panel::kWorldH + kPad : 0.0f), innerWL, viewer::panel::kRecH};
        // V23: coluna direita = Interpolação (topo), Cenas, Objetos (V22 tinha posto Cenas no topo).
        const Rectangle interpPanel{cxR, cyR, innerWR, viewer::panel::kInterpH};
        const Rectangle scenesPanel{cxR, cyR + viewer::panel::kInterpH + kPad, innerWR, viewer::panel::kScenesH};
        const Rectangle objPanel{cxR, cyR + viewer::panel::kScenesH + kPad + viewer::panel::kInterpH + kPad, innerWR, objH};
        // Só a cena 3D recebe cliques de cena: sobre colunas, borda, barra de rolagem ou barra de controle não.
        const bool overGui = !vp.contains(mouse.x, mouse.y) || dock.dragging() || scrollDrag >= 0;

        // --- Entrada: câmera ------------------------------------------------------------------------------
        // Gira ao redor do alvo com o botão direito OU com a roda do mouse pressionada.
        const bool orbitDown = IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE);
        if (!orbitDown) orbiting = false;
        else if (!orbiting && (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) || IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) && !overGui) orbiting = true;
        if (orbiting) {   // só gira se o botão desceu sobre a cena (não sobre uma coluna)
            const Vector2 d = GetMouseDelta();
            orbit.rotate(-d.x * 0.005f, d.y * 0.005f);
        }
        const float wheel = GetMouseWheelMove();
        {   // rolagem das colunas: roda sobre a coluna (menos sobre a lista de arquivos, que rola sozinha) e barra arrastável
            auto scrollInput = [&](int side, viewer::ScrollState& sc, const viewer::RectF& body, bool open) {
                if (!open) {
                    if (scrollDrag == side) scrollDrag = -1;
                    return;
                }
                const viewer::RectF track{body.right() - kBar, body.y, kBar, body.h};
                if (sc.scrollable()) {
                    if (scrollDrag < 0 && !dock.dragging() && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && track.contains(mouse.x, mouse.y)) {
                        const float ty = mouse.y - track.y;
                        if (ty < sc.thumbTop(track.h) || ty > sc.thumbTop(track.h) + sc.thumbHeight(track.h)) sc.jumpTo(ty, track.h);
                        scrollDrag = side;
                    }
                    if (scrollDrag == side) {
                        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) sc.dragThumb(GetMouseDelta().y, track.h);
                        else scrollDrag = -1;
                    }
                } else if (scrollDrag == side) {
                    scrollDrag = -1;
                }
                if (wheel != 0.0f && body.contains(mouse.x, mouse.y) && !fileListRect.contains(mouse.x, mouse.y) && !sceneListRect.contains(mouse.x, mouse.y) && !drag.active()) sc.wheel(wheel);
            };
            scrollInput(0, scrollL, bodyL, !dock.collapsed(viewer::DockSide::Left) && bodyL.w > 120.0f);
            scrollInput(1, scrollR, bodyR, !dock.collapsed(viewer::DockSide::Right) && bodyR.w > 120.0f);
        }
        if (drag.active()) drag.addDepth(static_cast<physics::Real>(wheel) * physics::Real(0.5));   // arrastando: roda = profundidade
        else if (!overGui) orbit.zoom(wheel);
        if (keyPressed(KEY_I))
            mode = static_cast<viewer::InterpMode>((static_cast<int>(mode) + 1) % viewer::kInterpModeCount);
        if (keyPressed(KEY_T) || uiPivot) pivotSelected = !pivotSelected;
        if (keyPressed(KEY_M)) showWorld = !showWorld;
        if (keyPressed(KEY_F5)) presentation = !presentation;
        if (keyPressed(KEY_F6)) autoOrbit = !autoOrbit;
        if (keyPressed(KEY_F7)) uiRecToggle = true;
        if (keyPressed(KEY_F11)) ToggleBorderlessWindowed();   // tela cheia sem borda (e volta)

        // Troca de cena: ←/→ (com volta) ou 1–9 percorrem a biblioteca (V22: originais + salvas, na ordem dela);
        // R recria a atual (reinicia); o painel Cenas pede um índice em gotoScene.
        const std::size_t sceneCount = lib.size();
        std::size_t wanted = sceneIndex;
        int stepDir = 1;
        if (sceneCount > 0) {
            if (keyPressed(KEY_RIGHT) || uiSceneDelta > 0) { wanted = (sceneIndex + 1) % sceneCount; stepDir = 1; }
            if (keyPressed(KEY_LEFT) || uiSceneDelta < 0) { wanted = (sceneIndex + sceneCount - 1) % sceneCount; stepDir = -1; }
            for (int k = 0; k < 9; ++k)
                if (keyPressed(KEY_ONE + k) && static_cast<std::size_t>(k) < sceneCount) wanted = static_cast<std::size_t>(k);
        }
        const bool forceLib = gotoScene >= 0 && static_cast<std::size_t>(gotoScene) < sceneCount;   // escolha explícita no painel
        if (forceLib) wanted = static_cast<std::size_t>(gotoScene);
        gotoScene = -1;
        if (newSceneRequest != 0) {   // aba Cenas: cena nova (V14)
            adoptScene(viewer::makeEmptyScene(newSceneRequest == 2));
            fileMsg = newSceneRequest == 2 ? "cena nova: so o chao (pausada)" : "cena nova: vazia (pausada)";
            fileMsgErr = false;
            fileMsgFrames = 600;
            newSceneRequest = 0;
        }
        const bool changed = wanted != sceneIndex;
        if (sceneCount > 0 && (changed || forceLib || keyPressed(KEY_R) || uiRestart)) {
            viewer::Scene next;
            std::size_t target = wanted;
            bool built = false;
            const bool useCustom = !changed && !forceLib && customActive;   // R numa cena nova/adotada: recria do texto
            if (useCustom) {
                std::string err;
                built = viewer::parseScene(customText, next, err, modelDir);
            }
            const bool fromCustom = built;
            for (std::size_t attempt = 0; !built && attempt < sceneCount; ++attempt) {
                std::string err, warn;
                if (viewer::buildLibraryScene(lib.at(target), sceneDir, modelDir, next, err, &warn)) {
                    built = true;
                    lib.at(target).broken = false;
                    if (!warn.empty()) { fileMsg = "AVISO: " + warn; fileMsgErr = true; fileMsgFrames = 600; }
                } else {
                    lib.at(target).broken = true;
                    fileMsg = "ERRO ao carregar " + lib.at(target).name + ": " + err;
                    fileMsgErr = true;
                    fileMsgFrames = 900;
                    if (!changed || forceLib) break;   // reinício ou escolha explícita: não pula para outra
                    target = stepDir > 0 ? (target + 1) % sceneCount : (target + sceneCount - 1) % sceneCount;   // seta: pula a cena ruim
                }
            }
            if (built) {
                scene = std::move(next);
                if (!fromCustom) {
                    sceneIndex = target;
                    customActive = false;
                }
            tracker.reset();
            scene.world->setContactListener(&tracker);
            sceneDefaults = viewer::captureWorld(*scene.world);
            scene.world->setProfilingEnabled(profilingOn);
            selected = physics::kInvalidBodyId;   // os ids são de outro mundo
            clearModelMeshes();
            recorder.stop("cena trocada ou reiniciada");
            jB = physics::kInvalidBodyId;
            jBMode = 0;
            jPickB = false;
            jHasClick = false;
            drag.end();
            timeCtl.onSceneReset();
            timeCtl.setPaused(startPaused);   // trocar ou reiniciar a cena: volta pausada
            if (!fromCustom && (changed || forceLib)) {   // só ao trocar: R mantém a câmera que o usuário ajustou
                orbit.target = scene.cameraTarget;
                camPan = viewer::Vec3f{};
                orbit.setDistance(scene.cameraDistance);
            }
            }   // if (built)
        }
        if (keyPressed(KEY_HOME) || uiResetCam) {   // câmera: enquadramento inicial, alvo no centro do mundo
            orbit.reset(scene.cameraTarget, scene.cameraDistance);
            camPan = viewer::Vec3f{};
            pivotSelected = false;
        }
        if (keyPressed(KEY_P) || keyPressed(KEY_SPACE) || uiTogglePause) timeCtl.togglePause();
        if (keyPressed(KEY_PERIOD) || uiStep) {   // um passo: se estava rodando, pausa primeiro
            timeCtl.setPaused(true);
            timeCtl.requestStep();
        }
        if (uiRecToggle) {   // V16: iniciar/parar a gravação
            if (recorder.recording()) {
                recorder.stop();
                recMsg = "gravacao parada: " + std::to_string(recorder.rowCount()) + " linhas";
            } else {
                std::string e;
                if (recorder.start(scene, recTarget == 0 ? viewer::RecordTarget::Selected : viewer::RecordTarget::AllDynamic, selected, &e))
                    recMsg = "gravando... (rode a simulacao)";
                else
                    recMsg = "ERRO: " + e;
            }
            recMsgFrames = 600;
        }
        if (uiRecSave) {
            std::string e;
            std::string base = viewer::sanitizeSceneFileName(scene.name);
            if (base.empty()) base = "cena";
            const std::string path = exportDir + "/trajetoria_" + base + "_" + std::to_string(++recFileCounter) + ".csv";
            if (recorder.rowCount() == 0) recMsg = "ERRO: nada gravado ainda";
            else if (recorder.save(path, e)) { recMsg = "salvo!"; recLastPath = path; }
            else recMsg = "ERRO: " + e;
            recMsgFrames = 900;
        }
        uiRecToggle = uiRecSave = false;
        if (recMsgFrames > 0) --recMsgFrames;
        uiRestart = uiStep = uiTogglePause = uiResetCam = uiPivot = false;
        uiSceneDelta = 0;
        if (selected != physics::kInvalidBodyId && !scene.world->isValidBody(selected)) {
            selected = physics::kInvalidBodyId;
            drag.end();
        }
        if (jBMode == 1 && !scene.world->isValidBody(jB)) { jBMode = 0; jB = physics::kInvalidBodyId; jPickB = false; }
        if (mainTab != 2) jPickB = false;   // V22: Juntas é a aba 2 (Cenas saiu das abas)

        // Alvo da câmera: centro do mundo (da cena) ou o corpo selecionado; a transição é suave.
        {
            viewer::Vec3f goal = scene.cameraTarget;
            if (pivotSelected && selected != physics::kInvalidBodyId) goal = viewer::toVec3f(renderPose(selected).position);
            // V19: WASD move a câmera pelo cenário (E sobe, Q desce, Shift acelera). O deslocamento acumulado
            // `camPan` soma ao alvo, senão o seguimento suave o devolveria ao centro a cada quadro.
            if (!fileNameEdit && !importEdit && !gSE.active) {
                const float f = (IsKeyDown(KEY_W) ? 1.0f : 0.0f) - (IsKeyDown(KEY_S) ? 1.0f : 0.0f);
                const float r = (IsKeyDown(KEY_D) ? 1.0f : 0.0f) - (IsKeyDown(KEY_A) ? 1.0f : 0.0f);
                const float u = (IsKeyDown(KEY_E) ? 1.0f : 0.0f) - (IsKeyDown(KEY_Q) ? 1.0f : 0.0f);
                if (f != 0.0f || r != 0.0f || u != 0.0f) {
                    const viewer::Vec3f d = orbit.moveDelta(f, r, u, GetFrameTime(), IsKeyDown(KEY_LEFT_SHIFT) ? 3.0f : 1.0f);
                    camPan.x = std::max(-500.0f, std::min(500.0f, camPan.x + d.x));
                    camPan.y = std::max(-500.0f, std::min(500.0f, camPan.y + d.y));
                    camPan.z = std::max(-500.0f, std::min(500.0f, camPan.z + d.z));
                }
            }
            goal = viewer::Vec3f{goal.x + camPan.x, goal.y + camPan.y, goal.z + camPan.z};
            orbit.followTarget(goal, GetFrameTime(), (camPan.x != 0.0f || camPan.y != 0.0f || camPan.z != 0.0f) ? 25.0f : 10.0f);
        }

        Camera3D cam{};
        cam.position = toRay(orbit.position());
        cam.target = toRay(orbit.target);
        cam.up = Vector3{0, 1, 0};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;

        // --- Entrada: mouse na cena (a câmera deste quadro já existe, então o raio é o que se vê) --------------------
        const Ray mouseRay = GetScreenToWorldRayEx(Vector2{mouse.x - vp.x, mouse.y - vp.y}, cam, static_cast<int>(vp.w), static_cast<int>(vp.h));
        const physics::Vector3 rayO(mouseRay.position.x, mouseRay.position.y, mouseRay.position.z);
        const physics::Vector3 rayD(mouseRay.direction.x, mouseRay.direction.y, mouseRay.direction.z);
        // Orientação do rascunho (quatérnio unitário; entrada nula -> identidade) e ponto de colocação por clique.
        bool spOk = true;
        const physics::Quaternion spOrient =
            qMode == 0 ? viewer::orientationFromAxisAngle(physics::Vector3(spAxis[0], spAxis[1], spAxis[2]),
                                                          static_cast<physics::Real>(spAngDeg * 0.017453292519943295f), &spOk)
                       : viewer::orientationFromQuaternion(spQ[0], spQ[1], spQ[2], spQ[3], &spOk);
        auto makeSpawn = [&]() {
            viewer::SpawnParams sp;
            sp.kind = kindIdx >= viewer::kSpawnKindCount ? viewer::SpawnKind::Box : static_cast<viewer::SpawnKind>(kindIdx);
            sp.size = sizeF;
            sp.mass = massS * massS;
            sp.friction = fricF;
            sp.restitution = restF;
            sp.position = physics::Vector3(spPos[0], spPos[1], spPos[2]);
            sp.orientation = spOrient;
            sp.velocity = physics::Vector3(spVel[0], spVel[1], spVel[2]);
            sp.angularVelocity = physics::Vector3(spAngV[0], spAngV[1], spAngV[2]);
            sp.isStatic = spStatic;
            sp.boxScale = physics::Vector3(spBox[0], spBox[1], spBox[2]);
            sp.radiusScale = spAxisR;
            sp.heightScale = spAxisH;
            if (kindIdx == viewer::kSpawnKindCount) {   // V15b: modelo importado (o fecho é recalculado para o tamanho escolhido)
                viewer::ModelAsset asset;
                std::string merr;
                if (modelObj && viewer::makeModelAsset(modelObj, modelObjFile, viewer::modelBaseDir(modelDir, modelObjFile),
                                                       viewer::autoScale(*modelObj, sizeF), asset, merr,
                                                       static_cast<viewer::ColliderKind>(colliderIdx), viewer::ColliderTuning{tuneR, tuneH})) {
                    sp.model = std::make_shared<viewer::ModelAsset>(asset);
                } else {
                    modelInfo1 = modelObj ? "ERRO: " + merr : "escolha um modelo na lista";
                    modelInfo2.clear();
                    modelInfo3.clear();
                    modelInfoWarn = true;
                    sp.size = std::numeric_limits<physics::Real>::quiet_NaN();   // spawnBody recusa: "NAO foi possivel criar"
                }
            }
            return sp;
        };
        auto afterSpawn = [&](physics::BodyId id) {
            if (id == physics::kInvalidBodyId) { addFailFrames = 120; return; }
            selected = id;   // o corpo novo fica selecionado (aba Objeto mostra os parâmetros dele)
            drag.end();
        };
        physics::Vector3 placePt;
        const bool hasPlace = clickPlace && mainTab == 1 && !overGui && !drag.active() &&
                              viewer::spawnPointFromRay(scene, rayO, rayD, sizeF, placePt);
        if (hasPlace && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            viewer::SpawnParams sp = makeSpawn();
            sp.position = placePt;
            for (int i = 0; i < 3; ++i) spPos[i] = static_cast<float>(placePt[i]);
            afterSpawn(viewer::spawnBody(scene, sp));
        } else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !overGui && !(clickPlace && mainTab == 1)) {
            const viewer::PickResult pick = viewer::pickBody(scene, rayO, rayD, physics::Real(200), pickStaticsOpt || mainTab == 2);
            if (mainTab == 2) {   // aba Juntas (V14): o clique escolhe A / B e guarda o ponto clicado; sem arrastar
                physics::Vector3 pt;
                if (viewer::pointOnRay(scene, rayO, rayD, pt)) { jClick = pt; jHasClick = true; }
                if (jPickB) {
                    if (pick.hit && pick.body != selected) { jB = pick.body; jBMode = 1; jPickB = false; }
                } else {
                    selected = pick.hit ? pick.body : physics::kInvalidBodyId;
                }
            } else {
                selected = pick.hit ? pick.body : physics::kInvalidBodyId;   // clicar no vazio ou no chão desmarca
                if (pick.hit) drag.begin(scene, pick);
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) || !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) drag.end();
        if (keyPressed(KEY_B) || keyPressed(KEY_O)) {   // lança uma caixa (B) ou esfera (O) na direção do mouse
            viewer::launchBody(scene, keyPressed(KEY_O) ? viewer::ShapeKind::Sphere : viewer::ShapeKind::Box,
                               rayO + rayD * physics::Real(1.5), rayD, physics::Real(18));
        }
        if ((keyPressed(KEY_DELETE) || keyPressed(KEY_X)) && selected != physics::kInvalidBodyId) {
            drag.end();
            viewer::deleteBody(scene, selected, true);   // V14: o editor também apaga o cenário estático
            selected = physics::kInvalidBodyId;
        }

        // --- Física: N passos fixos conforme o tempo de render ------------------------------------------------
        // A mola só atua com a física rodando (a força é consumida no passo, e pausado não há passo).
        if (!timeCtl.paused() && drag.active()) drag.apply(scene, rayO, rayD,
                                                      static_cast<physics::Real>(GetFrameTime()) * static_cast<physics::Real>(timeCtl.timeScale()));   // tempo FÍSICO do quadro (câmera lenta incluída)
        if (autoOrbit && !orbiting) orbit.rotate(GetFrameTime() * 0.4f, 0.0f);   // V16: câmera giratória (apresentação)
        timeCtl.advance(*scene.world, static_cast<physics::Real>(GetFrameTime()));
        if (recorder.recording()) {
            const bool was = true;
            recorder.capture(scene);
            if (was && !recorder.recording()) { recMsg = "gravacao encerrada: " + recorder.stopReason(); recMsgFrames = 600; }
        }
        if (!timeCtl.paused()) tracker.update(GetFrameTime() * timeCtl.timeScale());   // flashes decaem em tempo físico

        // --- Desenho ----------------------------------------------------------------------------------------------
        const int vpW = std::max(1, static_cast<int>(vp.w)), vpH = std::max(1, static_cast<int>(vp.h));
        if (vpTex.id == 0 || vpTex.texture.width != vpW || vpTex.texture.height != vpH) {   // viewport mudou de tamanho
            if (vpTex.id != 0) UnloadRenderTexture(vpTex);
            vpTex = LoadRenderTexture(vpW, vpH);
        }
        BeginTextureMode(vpTex);
        ClearBackground(Color{28, 30, 36, 255});

        BeginMode3D(cam);
        DrawGrid(20, 1.0f);
        const physics::BodyId infoBody = selected != physics::kInvalidBodyId ? selected : scene.focus;   // quem o painel mostra
        viewer::Mat4f focusMatrix;
        bool hasFocus = false;
        physics::Transform focusPose;
        for (const viewer::SceneBody& body : scene.bodies) {
            const physics::Transform pose = renderPose(body.id);
            const viewer::Mat4f m = viewer::toMat4(pose);
            if (!hideModels) {
                if (!body.isStatic && (tintOn || flashOn)) {   // V9: cor por estado (dormindo) e flash de impacto
                    viewer::SceneBody tb = body;
                    const viewer::Rgb c = viewer::stateTint(viewer::Rgb{body.r, body.g, body.b},
                                                            tintOn && !scene.world->isAwake(body.id),
                                                            flashOn ? tracker.flashOf(body.id) : 0.0f);
                    tb.r = c.r; tb.g = c.g; tb.b = c.b;
                    drawBody(tb, m);
                } else {
                    drawBody(body, m);
                }
            }
            if (body.id == selected) drawHighlight(body, m);
            if (body.id == infoBody) {
                focusMatrix = m;
                focusPose = pose;
                hasFocus = true;
            }
        }
        if (dbgFlags != 0) {   // debug draw do motor: as linhas vêm do DebugLineBuffer e vão para o DrawLine3D
            const bool interp = timeCtl.renderAlpha(*scene.world) == scene.world->getAlpha();   // após passo manual: pose atual
            viewer::buildDebugLines(*scene.world, dbgFlags, dbgBuf, interp);
            for (const physics::DebugLine& l : dbgBuf.lines())
                DrawLine3D(Vector3{l.ax, l.ay, l.az}, Vector3{l.bx, l.by, l.bz}, Color{l.r, l.g, l.b, l.a});
        }
        for (const viewer::SceneLink& link : scene.links) {   // juntas: linha entre os centros
            const physics::Vector3 pa = renderPose(link.a).position;
            const physics::Vector3 pb = renderPose(link.b).position;
            DrawLine3D(Vector3{float(pa.x), float(pa.y), float(pa.z)}, Vector3{float(pb.x), float(pb.y), float(pb.z)},
                       Color{235, 235, 235, 255});
        }
        if (ringsOn) {   // anéis nos pontos de impacto: crescem e somem em 0,4 s
            for (const viewer::ContactTracker::Marker& mk : tracker.markers()) {
                const float k = mk.age / viewer::ContactTracker::kFlashSeconds;
                const Vector3 c{static_cast<float>(mk.point.x), static_cast<float>(mk.point.y), static_cast<float>(mk.point.z)};
                DrawSphereWires(c, 0.1f + 0.5f * mk.intensity * k, 6, 10,
                                Color{255, 235, 120, static_cast<unsigned char>(255.0f * (1.0f - k))});
            }
        }
        if (hasPlace) {   // prévia do "posicionar com clique": fio amarelo no ponto de criação
            const Vector3 c{static_cast<float>(placePt.x), static_cast<float>(placePt.y), static_cast<float>(placePt.z)};
            if (kindIdx == 1) DrawSphereWires(c, sizeF * 0.5f, 8, 12, YELLOW);
            else if (kindIdx == 6 || kindIdx == 7) {   // V18
                rlPushMatrix();
                rlTranslatef(c.x, c.y, c.z);
                const float pr = sizeF * spAxisR, ph = sizeF * spAxisH * 0.5f;
                drawAxisWire(kindIdx == 6 ? viewer::ShapeKind::Capsule : viewer::ShapeKind::Cylinder, pr,
                             kindIdx == 6 ? std::max(0.0f, ph - pr) : ph, YELLOW);
                rlPopMatrix();
            } else DrawCubeWires(c, sizeF, sizeF, sizeF, YELLOW);
        }
        if (drag.active()) {   // a mola: do ponto agarrado até onde o mouse aponta
            DrawLine3D(Vector3{float(drag.anchorWorld().x), float(drag.anchorWorld().y), float(drag.anchorWorld().z)},
                       Vector3{float(drag.targetWorld().x), float(drag.targetWorld().y), float(drag.targetWorld().z)}, ORANGE);
            DrawSphere(Vector3{float(drag.targetWorld().x), float(drag.targetWorld().y), float(drag.targetWorld().z)}, 0.08f, ORANGE);
        }
        if (mainTab == 2) {   // V14: prévia da junta que será criada
            auto v3 = [](const physics::Vector3& v) { return Vector3{float(v.x), float(v.y), float(v.z)}; };
            physics::Vector3 anchor;
            bool okAnchor = true;
            if (jAnchorMode == 0) anchor = physics::Vector3(jAnchor[0], jAnchor[1], jAnchor[2]);
            else if (jAnchorMode == 1 && jHasClick) anchor = jClick;
            else if (jAnchorMode == 2 && scene.world->isValidBody(selected)) anchor = renderPose(selected).position;
            else okAnchor = false;
            if (jHasClick) DrawSphereWires(v3(jClick), 0.06f, 4, 6, Color{120, 220, 255, 255});
            if (okAnchor) {
                DrawSphere(v3(anchor), 0.1f, YELLOW);
                if (scene.world->isValidBody(selected)) DrawLine3D(v3(renderPose(selected).position), v3(anchor), YELLOW);
                if (jBMode == 1 && scene.world->isValidBody(jB)) DrawLine3D(v3(renderPose(jB).position), v3(anchor), Color{120, 220, 255, 255});
                if (jKind == 0) {   // eixo da dobradiça
                    const physics::Vector3 ax(jAxis[0], jAxis[1], jAxis[2]);
                    if (ax.lengthSquared() > physics::Real(1e-8)) {
                        const physics::Vector3 u = ax.normalized() * physics::Real(1.3);
                        DrawLine3D(v3(anchor - u), v3(anchor + u), Color{255, 90, 90, 255});
                    }
                }
            }
        }
        if (hasFocus) drawLocalAxes(focusMatrix, 1.2f);
        EndMode3D();
        EndTextureMode();

        BeginDrawing();
        ClearBackground(Color{24, 26, 32, 255});
        DrawTextureRec(vpTex.texture, Rectangle{0.0f, 0.0f, static_cast<float>(vpW), -static_cast<float>(vpH)}, Vector2{vp.x, vp.y}, WHITE);

        // --- HUD dentro do viewport (recortado para não passar sobre as colunas) -----------------------------------
        BeginScissorMode(static_cast<int>(vp.x), static_cast<int>(vp.y), vpW, vpH);
        if (presentation) {   // V16: modo apresentação — nome da cena + quatérnio grande, nada de painéis
            const int hx = 24, hy = 20;
            hudText(TextFormat("%s%s", scene.name.c_str(), timeCtl.paused() ? "   [PAUSADO]" : ""), hx, hy, 34, RAYWHITE);
            if (hasFocus) {
                const physics::Quaternion& q = focusPose.rotation;
                const viewer::AxisAngleF aa = viewer::toAxisAngle(q);
                const Color qc = Color{120, 220, 255, 255};
                hudText("quaternio q = w + xi + yj + zk", hx, hy + 56, 22, Color{200, 205, 215, 255});
                hudText(TextFormat("w %+.4f", static_cast<double>(q.w)), hx, hy + 88, 40, qc);
                hudText(TextFormat("x %+.4f", static_cast<double>(q.x)), hx, hy + 136, 40, qc);
                hudText(TextFormat("y %+.4f", static_cast<double>(q.y)), hx, hy + 184, 40, qc);
                hudText(TextFormat("z %+.4f", static_cast<double>(q.z)), hx, hy + 232, 40, qc);
                hudText(TextFormat("|q| = %.5f", static_cast<double>(q.norm())), hx, hy + 284, 24, Color{200, 205, 215, 255});
                hudText(TextFormat("eixo (%+.2f, %+.2f, %+.2f)   angulo %.1f graus", static_cast<double>(aa.axis.x), static_cast<double>(aa.axis.y),
                                   static_cast<double>(aa.axis.z), static_cast<double>(aa.angle) * 57.29577951308232),
                        hx, hy + 318, 24, Color{255, 235, 60, 255});
            }
            if (recorder.recording())
                hudText(TextFormat("* REC  %d linhas  %.1f s", static_cast<int>(recorder.rowCount()), recorder.duration()), hx, static_cast<int>(vp.bottom()) - 78, 22, Color{255, 90, 90, 255});
            hudText("F5/ESC: sair   Espaco: pausa   F6: camera giratoria   F7: gravar   setas: cena   R: reiniciar", hx, static_cast<int>(vp.bottom()) - 36, 18, Color{200, 205, 215, 255});
        } else {
            const int hx = static_cast<int>(vp.x) + 12, hb = static_cast<int>(vp.bottom());
            if (recorder.recording())
                hudText(TextFormat("* REC  %d linhas  %.1f s", static_cast<int>(recorder.rowCount()), recorder.duration()), hx + 460, static_cast<int>(vp.y) + 8, 20, Color{255, 90, 90, 255});
            hudText(customActive ? TextFormat("[arquivo] %s   (%d corpos)", scene.name.c_str(), static_cast<int>(scene.bodies.size()))
                                 : TextFormat("[%d/%d] %s   (%d corpos)", static_cast<int>(sceneIndex) + 1, static_cast<int>(lib.size()),
                                              scene.name.c_str(), static_cast<int>(scene.bodies.size())),
                     hx, static_cast<int>(vp.y) + 8, 22, RAYWHITE);
            if (selected != physics::kInvalidBodyId && scene.world->isValidBody(selected)) {
                const physics::RigidBody& sb = scene.world->getBody(selected);
                hudText(TextFormat("selecionado: corpo %u   massa %.2f kg   |v| = %.2f m/s   %s", static_cast<unsigned>(selected),
                                    static_cast<double>(sb.getMass()), static_cast<double>(sb.getLinearVelocity().length()),
                                    sb.isAwake() ? "acordado" : "dormindo"),
                         hx, static_cast<int>(vp.y) + 38, 16, Color{255, 235, 60, 255});
            }
            hudText(TextFormat("%d FPS   fisica %.0f Hz   passo #%llu   alpha = %.2f   tempo %s%s", GetFPS(),
                                1.0 / static_cast<double>(scene.world->getFixedTimeStep()),
                                static_cast<unsigned long long>(scene.world->getStats().stepCount),
                                static_cast<double>(timeCtl.renderAlpha(*scene.world)),
                                viewer::TimeControl::scaleLabel(timeCtl.scaleIndex()), timeCtl.paused() ? "   [PAUSADO]" : ""),
                     hx, hb - 100, 16, Color{235, 238, 245, 255});
            hudText("esq: selecionar/arrastar  dir/roda: girar  roda: zoom  WASD+Q/E: mover (Shift: rapido)", hx, hb - 78, 16, Color{200, 205, 215, 255});
            hudText("B/O: lancar   Del/X: apagar   Espaco/P: pausa   . : passo", hx, hb - 58, 16, Color{200, 205, 215, 255});
            hudText("setas/1-9: cena   R: reiniciar   I: interpolacao   T: alvo", hx, hb - 38, 16, Color{200, 205, 215, 255});
            hudText("Home: camera   M: mundo   F5: apresentacao   F6: girar   F7: gravar   F11: tela cheia   ESC: sair", hx, hb - 18, 16, Color{200, 205, 215, 255});
        }
        EndScissorMode();

        if (!presentation) {   // V16: na apresentação não há colunas nem barra de controle
        // --- Colunas laterais: fundo, cabeçalho com o botão de recolher, e as bordas arrastáveis -----------------------
        for (viewer::DockSide side : {viewer::DockSide::Left, viewer::DockSide::Right}) {
            const viewer::RectF d = dock.dock(side);
            DrawRectangle(static_cast<int>(d.x), static_cast<int>(d.y), static_cast<int>(d.w), static_cast<int>(d.h), kDockBg);
            const bool left = side == viewer::DockSide::Left;
            const bool open = !dock.collapsed(side) && d.w > 120.0f;
            if (open) {
                DrawText(left ? "Quaternio, mundo e gravacao" : "Interpolacao, cenas e objetos", static_cast<int>(d.x) + 10, static_cast<int>(d.y) + 5, 16,
                         Color{200, 205, 215, 255});
                if (fitButton(Rectangle{d.right() - 30.0f, d.y + 2.0f, 26.0f, 22.0f}, left ? "<" : ">")) {
                    dock.toggleCollapsed(side);
                    layoutDirty = true;
                }
            } else if (fitButton(Rectangle{d.x + 1.0f, d.y + 2.0f, d.w - 2.0f, 22.0f}, left ? ">" : "<")) {
                dock.toggleCollapsed(side);
                layoutDirty = true;
            }
            const viewer::RectF sp = dock.splitter(side);
            if (sp.w > 0.0f) {
                const bool hot = dock.dragging() && dock.dragSide() == side;
                DrawRectangle(static_cast<int>(sp.x), static_cast<int>(sp.y), static_cast<int>(sp.w), static_cast<int>(sp.h),
                              hot ? Color{120, 170, 255, 255} : Color{62, 66, 78, 255});
            }
        }
        // Painéis de uma coluna: recortados na área do corpo e com os cliques bloqueados fora dela (um botão
        // escondido pela rolagem não pode disparar por baixo de outra coisa).
        auto beginDock = [&](viewer::DockSide side) {
            const viewer::RectF& body = side == viewer::DockSide::Left ? bodyL : bodyR;
            const bool open = !dock.collapsed(side) && body.w > 120.0f;
            if (open) BeginScissorMode(static_cast<int>(body.x), static_cast<int>(body.y), static_cast<int>(body.w), static_cast<int>(body.h));
            else BeginScissorMode(0, 0, 0, 0);
            gTextRight = body.right() - kBar - 6.0f;   // V17: o texto dos painéis termina aqui
            if (!fileNameEdit && !importEdit && !gSE.active && (!open || !body.contains(mouse.x, mouse.y) || dock.dragging() || scrollDrag >= 0)) GuiLock();
        };
        auto endDock = [&]() {
            GuiUnlock();
            EndScissorMode();
        };
        // Barra de controle (topo).
        GuiPanel(controlBar, nullptr);
        {
            float bx = controlBar.x + 6.0f;
            const float by = controlBar.y + 4.0f, bh = 28.0f;
            auto place = [&](float w) {
                const Rectangle r{bx, by, w, bh};
                bx += w + 6.0f;
                return r;
            };
            if (fitButton(place(96.0f), timeCtl.paused() ? "Continuar" : "Pausar")) uiTogglePause = true;
            if (fitButton(place(60.0f), "Passo")) uiStep = true;
            if (fitButton(place(84.0f), "Reiniciar")) uiRestart = true;
            GuiLabel(place(34.0f), "Vel.");
            int sc = timeCtl.scaleIndex();
            fitToggleGroup(Rectangle{bx, by, 42.0f, bh}, "1x;1/2;1/4;1/10", &sc);
            bx += 4.0f * 42.0f + 12.0f;
            timeCtl.setScaleIndex(sc);
            if (fitButton(place(26.0f), "<")) uiSceneDelta = -1;
            GuiLabel(place(84.0f), customActive ? "arquivo" : TextFormat("cena %d/%d", static_cast<int>(sceneIndex) + 1, static_cast<int>(lib.size())));
            if (fitButton(place(26.0f), ">")) uiSceneDelta = 1;
            if (fitButton(place(92.0f), "Reset cam")) uiResetCam = true;
            if (fitButton(place(150.0f), pivotSelected ? "Alvo: selecionado" : "Alvo: mundo")) uiPivot = true;
        }

        beginDock(viewer::DockSide::Left);
        // Painel do quatérnio do corpo em foco (a pose que está sendo DESENHADA, no modo escolhido).
        GuiPanel(quatPanel, selected != physics::kInvalidBodyId ? "Quaternio do corpo selecionado" : "Quaternio do corpo em foco");
        if (hasFocus) {
            const physics::Quaternion& q = focusPose.rotation;
            const physics::Real qn = q.norm();
            const Color txt = kAccent;
            const float x0 = quatPanel.x + 12.0f;
            float y0 = quatPanel.y + 34.0f;
            panelText(TextFormat("x %+.4f   y %+.4f", static_cast<double>(q.x), static_cast<double>(q.y)), static_cast<int>(x0),
                     static_cast<int>(y0), 20, txt);
            y0 += 24.0f;
            panelText(TextFormat("z %+.4f   w %+.4f", static_cast<double>(q.z), static_cast<double>(q.w)), static_cast<int>(x0),
                     static_cast<int>(y0), 20, txt);
            y0 += 24.0f;
            // |q| = 1 sempre, exceto no modo "lerp sem normalizar" entre dois passos.
            const bool unit = std::fabs(static_cast<double>(qn) - 1.0) < 1e-4;
            panelText(TextFormat("|q| = %.6f%s", static_cast<double>(qn), unit ? "" : "   (NAO unitario)"),
                     static_cast<int>(x0), static_cast<int>(y0), 20, unit ? txt : kWarn);
            y0 += 28.0f;
            const viewer::AxisAngleF aa = viewer::toAxisAngle(q);
            panelText(TextFormat("eixo (%+.3f %+.3f %+.3f)", static_cast<double>(aa.axis.x), static_cast<double>(aa.axis.y),
                                static_cast<double>(aa.axis.z)),
                     static_cast<int>(x0), static_cast<int>(y0), 18, kInkDim);
            y0 += 22.0f;
            panelText(TextFormat("angulo %.1f graus (%.3f rad)", static_cast<double>(aa.angle) * 57.29577951308232,
                                static_cast<double>(aa.angle)),
                     static_cast<int>(x0), static_cast<int>(y0), 18, kInkDim);
            y0 += 26.0f;
            const viewer::EulerAngles eu = viewer::toEuler(q);
            const double rad2deg = 57.29577951308232;
            const bool lock = viewer::nearGimbalLock(eu, physics::Real(0.087));   // ~5 graus
            panelText(TextFormat("Euler  yaw %+.1f  pitch %+.1f  roll %+.1f", static_cast<double>(eu.yaw) * rad2deg,
                                static_cast<double>(eu.pitch) * rad2deg, static_cast<double>(eu.roll) * rad2deg),
                     static_cast<int>(x0), static_cast<int>(y0), 18, lock ? kWarn : kInkDim);
            // Velocidade angular do corpo (a que o motor integra no quatérnio: dq/dt = 1/2 w q).
            if (scene.world->isValidBody(infoBody)) {
                const physics::Vector3 w = scene.world->getBody(infoBody).getAngularVelocity();
                panelText(TextFormat("w (%+.2f %+.2f %+.2f)  |w| %.2f rad/s", static_cast<double>(w.x), static_cast<double>(w.y),
                                    static_cast<double>(w.z), static_cast<double>(w.length())),
                         static_cast<int>(x0), static_cast<int>(y0) + 24, 16, kBlueInk);
            }
            if (lock) panelText("GIMBAL LOCK: yaw e roll giram o mesmo eixo", static_cast<int>(x0), static_cast<int>(y0) + 46, 16, kWarn);
        } else {
            panelText("(esta cena nao tem corpo em foco)", static_cast<int>(quatPanel.x) + 12, static_cast<int>(quatPanel.y) + 40, 18, kInkDim);
        }

        // --- V24: esfera de arcball -----------------------------------------------------------------------------------
        // Mostra a orientação do corpo em foco como 3 círculos máximos (um por eixo local) numa esfera, mais o eixo de
        // rotação do quatérnio. Arrastar a esfera (com o corpo selecionado) gira o corpo: q = Δ · q0 (Shoemake).
        GuiPanel(arcPanel, "Arcball (arraste para girar)");
        {
            static viewer::ArcballDrag arcDrag;
            static physics::BodyId arcBody = physics::kInvalidBodyId;
            const float arcR = std::max(40.0f, std::min(95.0f, (arcPanel.width - 24.0f) / 2.0f));
            const float acx = arcPanel.x + arcPanel.width / 2.0f, acy = arcPanel.y + 34.0f + arcR;
            const bool canDrag = hasFocus && selected != physics::kInvalidBodyId;
            const viewer::ArcballView av = viewer::arcballViewFromCamera(orbit);
            const Vector2 amp = GetMousePosition();
            const double anx = static_cast<double>(amp.x - acx) / static_cast<double>(arcR);
            const double any = -static_cast<double>(amp.y - acy) / static_cast<double>(arcR);
            const bool overSphere = !GuiIsLocked() && anx * anx + any * any <= 1.0;
            if (arcDrag.active() && (!canDrag || arcBody != selected || !IsMouseButtonDown(MOUSE_BUTTON_LEFT))) arcDrag.end();
            if (canDrag && overSphere && !arcDrag.active() && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                arcDrag.begin(anx, any, focusPose.rotation, av);
                arcBody = selected;
            }
            if (arcDrag.active()) {   // gira o corpo; zera ω para o motor não brigar com o gesto
                viewer::setOrientationQuaternion(*scene.world, selected, arcDrag.update(anx, any, av));
                viewer::setAngularVelocity(*scene.world, selected, physics::Vector3::zero());
            }
            const physics::Quaternion qd = arcDrag.active() ? scene.world->getBody(selected).getOrientation() : focusPose.rotation;
            DrawCircle(static_cast<int>(acx), static_cast<int>(acy), arcR, Color{24, 26, 32, 255});
            DrawCircleLines(static_cast<int>(acx), static_cast<int>(acy), arcR, (overSphere && canDrag) || arcDrag.active() ? Color{130, 170, 255, 255} : Color{96, 102, 118, 255});
            if (hasFocus) {
                const Color axisCol[3] = {Color{255, 95, 95, 255}, Color{110, 225, 120, 255}, Color{110, 160, 255, 255}};
                constexpr int kSeg = 48;
                viewer::ArcballPoint pts[kSeg];
                for (int pass = 0; pass < 2; ++pass) {   // pass 0 = metade de trás (apagada), 1 = metade da frente
                    for (int a = 0; a < 3; ++a) {
                        viewer::arcballCircle(qd, a, av, pts, kSeg);
                        for (int i = 0; i < kSeg; ++i) {
                            const viewer::ArcballPoint& p0 = pts[i];
                            const viewer::ArcballPoint& p1 = pts[(i + 1) % kSeg];
                            const bool front = (p0.depth + p1.depth) * 0.5f >= 0.0f;
                            if (front != (pass == 1)) continue;
                            Color c = axisCol[a];
                            if (!front) c.a = 70;
                            DrawLineEx(Vector2{acx + p0.x * arcR, acy - p0.y * arcR}, Vector2{acx + p1.x * arcR, acy - p1.y * arcR}, front ? 2.0f : 1.0f, c);
                        }
                    }
                }
                // pontas dos eixos locais (x, y, z do corpo)
                static const char* const kAxisName[3] = {"x", "y", "z"};
                for (int a = 0; a < 3; ++a) {
                    physics::Vector3 e(0, 0, 0);
                    e[a] = physics::Real(1);
                    const viewer::ArcballPoint tp = viewer::arcballProject(qd.rotate(e), av);
                    const float tx = acx + tp.x * arcR, ty = acy - tp.y * arcR;
                    DrawLineEx(Vector2{acx, acy}, Vector2{tx, ty}, 2.0f, tp.depth >= 0 ? axisCol[a] : Color{axisCol[a].r, axisCol[a].g, axisCol[a].b, 90});
                    DrawCircle(static_cast<int>(tx), static_cast<int>(ty), 4.0f, axisCol[a]);
                    DrawText(kAxisName[a], static_cast<int>(tx) + 6, static_cast<int>(ty) - 7, 14, axisCol[a]);
                }
                // eixo de rotação do quatérnio (eixo-ângulo): destaque do quatérnio
                physics::Vector3 qaxis;
                physics::Real qang = 0;
                qd.toAxisAngle(qaxis, qang);
                if (std::fabs(static_cast<double>(qang)) > 1e-4) {
                    const viewer::ArcballPoint ap = viewer::arcballProject(qaxis, av);
                    const Vector2 a0{acx - ap.x * arcR, acy + ap.y * arcR}, a1{acx + ap.x * arcR, acy - ap.y * arcR};
                    DrawLineEx(a0, a1, 2.0f, Color{kAccent.r, kAccent.g, kAccent.b, 150});
                    DrawCircle(static_cast<int>(a1.x), static_cast<int>(a1.y), 5.0f, kAccent);
                }
                const float ty0 = acy + arcR + 8.0f;
                panelText(TextFormat("giro de %.1f graus em torno de (%+.2f %+.2f %+.2f)", static_cast<double>(qang) * 57.29577951308232,
                                    static_cast<double>(qaxis.x), static_cast<double>(qaxis.y), static_cast<double>(qaxis.z)),
                         static_cast<int>(arcPanel.x) + 12, static_cast<int>(ty0), 14, kAccent);
                panelText(canDrag ? "x vermelho  y verde  z azul (eixos do corpo)" : "selecione o corpo (clique) para girar com o mouse",
                         static_cast<int>(arcPanel.x) + 12, static_cast<int>(ty0) + 18, 14, canDrag ? kInkDim : kWarn);
            } else {
                panelText("(esta cena nao tem corpo em foco)", static_cast<int>(arcPanel.x) + 12, static_cast<int>(arcPanel.y) + 40, 18, kInkDim);
            }
        }

        endDock();
        beginDock(viewer::DockSide::Right);
        // --- Painel Cenas (V22): fora do painel de objetos, no topo da coluna direita ---------------------------------
        {
            sceneListRect = viewer::RectF{};
            GuiPanel(scenesPanel, "Cenas");
            const float sx = scenesPanel.x + 12.0f, sw = scenesPanel.width - 24.0f;
            float sy = scenesPanel.y + 32.0f;
            panelText("nome do arquivo (sem extensao):", static_cast<int>(sx), static_cast<int>(sy), 14, kInkDim);
            sy += 18.0f;
            if (GuiTextBox(Rectangle{sx, sy, sw, 26.0f}, fileName, 48, fileNameEdit)) fileNameEdit = !fileNameEdit;
            sy += 32.0f;
            if (fitButton(Rectangle{sx, sy, sw / 2.0f - 4.0f, 28.0f}, "Salvar cena atual")) {
                const std::string typed = fileName;
                const std::string base = viewer::sanitizeSceneFileName(typed);
                const std::string path = sceneDir + "/" + base + viewer::scenefile::kExtension;
                const bool existed = lib.findFile(base) >= 0;
                if (!typed.empty()) scene.name = typed;   // o nome digitado vira o nome da cena
                std::string err;
                if (viewer::saveSceneFile(scene, path, err)) {
                    fileMsg = "salvo: " + base + viewer::scenefile::kExtension + (existed ? " (sobrescreveu)" : " (entrou na lista)");
                    fileMsgErr = false;
                    const std::size_t idx = lib.addFile(base);   // V22: a cena salva entra na lista (fim) e vira a atual
                    sceneIndex = idx;
                    customActive = false;
                    libActive = static_cast<int>(idx);
                    libDirty = true;
                } else {
                    fileMsg = "ERRO: " + err;
                    fileMsgErr = true;
                }
                fileMsgFrames = 600;
                fileNameEdit = false;
                delConfirm = false;
            }
            if (fitButton(Rectangle{sx + sw / 2.0f + 4.0f, sy, sw / 2.0f - 4.0f, 28.0f}, "Atualizar lista")) {
                // relê a pasta: arquivos novos entram no fim, os que sumiram saem; a ordem atual fica
                const viewer::LibraryEntry cur = sceneIndex < lib.size() ? lib.at(sceneIndex) : viewer::LibraryEntry{};
                lib.rebuild(viewer::listSceneFiles(sceneDir), lib.serialize());
                const int found = cur.kind == viewer::LibraryEntry::Kind::Builtin ? lib.findBuiltin(cur.builtinIndex) : lib.findFile(cur.file);
                if (found >= 0) {
                    sceneIndex = static_cast<std::size_t>(found);
                } else if (!lib.empty()) {   // a cena atual era um arquivo que sumiu: continua rodando, sem lugar na lista
                    customActive = true;
                    customText = viewer::serializeScene(scene);
                    sceneIndex = std::min(sceneIndex, lib.size() - 1);
                }
                libActive = -1;
                libDirty = true;
                delConfirm = false;
                fileMsg = "lista atualizada";
                fileMsgErr = false;
                fileMsgFrames = 300;
            }
            sy += 36.0f;
            if (fitButton(Rectangle{sx, sy, sw / 2.0f - 4.0f, 28.0f}, "Nova (vazia)")) newSceneRequest = 1;
            if (fitButton(Rectangle{sx + sw / 2.0f + 4.0f, sy, sw / 2.0f - 4.0f, 28.0f}, "Nova (so o chao)")) newSceneRequest = 2;
            sy += 36.0f;
            panelText(TextFormat("cenas (%d): setas e 1-9 seguem esta ordem", static_cast<int>(lib.size())), static_cast<int>(sx), static_cast<int>(sy), 14, kInkDim);
            sy += 18.0f;
            if (libActive >= static_cast<int>(lib.size())) libActive = -1;
            std::string joined;
            for (std::size_t i = 0; i < lib.size(); ++i) {
                const viewer::LibraryEntry& e = lib.at(i);
                std::string n = e.name;
                std::replace(n.begin(), n.end(), ';', '_');   // ';' separa os itens da lista do raygui
                const bool current = i == sceneIndex && !customActive;
                joined += (i ? ";" : "") + std::string(current ? "> " : "  ") + std::to_string(i + 1) + ". " + n +
                          (e.kind == viewer::LibraryEntry::Kind::File ? " (salva)" : "") + (e.broken ? " [ERRO]" : "");
            }
            sceneListRect = viewer::RectF{sx, sy, sw, viewer::panel::kSceneListH};
            const int activeBefore = libActive;
            GuiListView(Rectangle{sx, sy, sw, viewer::panel::kSceneListH}, joined.empty() ? "(nenhuma cena)" : joined.c_str(), &libScroll, &libActive);
            if (libActive != activeBefore) delConfirm = false;
            sy += viewer::panel::kSceneListH + 6.0f;
            const bool hasSel = libActive >= 0 && libActive < static_cast<int>(lib.size());
            const std::size_t li = hasSel ? static_cast<std::size_t>(libActive) : 0;
            if (fitButton(Rectangle{sx, sy, sw / 2.0f - 4.0f, 28.0f}, "Subir") && hasSel) {
                std::size_t ni = 0;
                if (lib.moveUp(li, &ni)) {
                    if (sceneIndex == li) sceneIndex = ni; else if (sceneIndex == ni) sceneIndex = li;
                    libActive = static_cast<int>(ni);
                    libDirty = true;
                }
                delConfirm = false;
            }
            if (fitButton(Rectangle{sx + sw / 2.0f + 4.0f, sy, sw / 2.0f - 4.0f, 28.0f}, "Descer") && hasSel) {
                std::size_t ni = 0;
                if (lib.moveDown(li, &ni)) {
                    if (sceneIndex == li) sceneIndex = ni; else if (sceneIndex == ni) sceneIndex = li;
                    libActive = static_cast<int>(ni);
                    libDirty = true;
                }
                delConfirm = false;
            }
            sy += 34.0f;
            if (fitButton(Rectangle{sx, sy, sw / 2.0f - 4.0f, 28.0f}, delConfirm ? "Confirmar apagar" : "Apagar") && hasSel) {
                if (lib.size() <= 1) {
                    fileMsg = "mantenha ao menos uma cena na lista";
                    fileMsgErr = true;
                    fileMsgFrames = 400;
                } else if (!delConfirm) {
                    delConfirm = true;
                    const viewer::LibraryEntry& e = lib.at(li);
                    fileMsg = e.kind == viewer::LibraryEntry::Kind::File ? "clique de novo: APAGA o arquivo " + e.file + ".scene do disco"
                                                                         : "clique de novo: tira a cena original da lista (da para restaurar)";
                    fileMsgErr = true;
                    fileMsgFrames = 600;
                } else {
                    delConfirm = false;
                    bool ok = true;
                    const viewer::LibraryEntry e = lib.at(li);
                    if (e.kind == viewer::LibraryEntry::Kind::File) {
                        std::error_code ec;
                        std::filesystem::remove(sceneDir + "/" + e.file + viewer::scenefile::kExtension, ec);
                        ok = !ec;
                        if (!ok) { fileMsg = "ERRO ao apagar o arquivo: " + ec.message(); fileMsgErr = true; fileMsgFrames = 600; }
                    }
                    if (ok && lib.remove(li)) {
                        if (li < sceneIndex) {
                            --sceneIndex;
                        } else if (li == sceneIndex) {   // a cena que roda foi apagada: continua rodando, sem lugar na lista
                            customActive = true;
                            customText = viewer::serializeScene(scene);
                            sceneIndex = (li + lib.size() - 1) % lib.size();
                        }
                        libActive = lib.empty() ? -1 : static_cast<int>(std::min(li, lib.size() - 1));
                        libDirty = true;
                        fileMsg = e.kind == viewer::LibraryEntry::Kind::File ? "apagado: " + e.file + ".scene" : "original tirada da lista: " + e.name;
                        fileMsgErr = false;
                        fileMsgFrames = 600;
                    }
                }
            }
            const std::size_t hidden = lib.hiddenBuiltinCount();
            if (fitButton(Rectangle{sx + sw / 2.0f + 4.0f, sy, sw / 2.0f - 4.0f, 28.0f}, TextFormat("Restaurar originais (%d)", static_cast<int>(hidden))) && hidden > 0) {
                const std::size_t back = lib.restoreBuiltins();
                libDirty = true;
                delConfirm = false;
                fileMsg = "voltaram " + std::to_string(back) + " cena(s) original(is), no fim da lista";
                fileMsgErr = false;
                fileMsgFrames = 600;
            }
            sy += 34.0f;
            if (fitButton(Rectangle{sx, sy, sw, 28.0f}, "Carregar a escolhida") && hasSel) {
                gotoScene = libActive;
                delConfirm = false;
            }
            sy += 34.0f;
            if (fileMsgFrames > 0) {
                --fileMsgFrames;
                wrapPanelText(fileMsg, static_cast<int>(sx), static_cast<int>(sy), 14, fileMsgErr ? kWarn : kBlueInk, 2);
            }
        }
        // Painel da interpolação de render e do FPS.
        GuiPanel(interpPanel, "Interpolacao de render");
        {
            const float x0 = interpPanel.x + 12.0f, w = interpPanel.width - 24.0f;
            int active = static_cast<int>(mode);
            fitToggleGroup(Rectangle{x0, interpPanel.y + 34.0f, w / 3.0f, 30.0f}, "slerp;lerp;Euler", &active);
            mode = static_cast<viewer::InterpMode>(active);
            panelText(viewer::interpModeName(mode), static_cast<int>(x0), static_cast<int>(interpPanel.y) + 72, 18, kInk);
            const char* help = "";
            if (mode == viewer::InterpMode::Slerp) help = "esferica: giro constante, |q| = 1";
            if (mode == viewer::InterpMode::LerpNoNormalize) help = "reta entre q0 e q1: |q| < 1, o\ncubo encolhe e muda de ritmo";
            if (mode == viewer::InterpMode::Euler) help = "interpola yaw/pitch/roll: caminho\ntorto; pula no gimbal lock";
            panelText(help, static_cast<int>(x0), static_cast<int>(interpPanel.y) + 96, 16, kInk);
            panelText(TextFormat("FPS de render: %d", fpsApplied), static_cast<int>(x0), static_cast<int>(interpPanel.y) + 144, 18, kInk);
            panelText("10", static_cast<int>(x0), static_cast<int>(interpPanel.y) + 171, 16, kInk);
            panelText("240", static_cast<int>(x0 + w - 34.0f), static_cast<int>(interpPanel.y) + 171, 16, kInk);
            sliderBox(Rectangle{x0 + 28.0f, interpPanel.y + 168.0f, w - 68.0f, 22.0f}, fpsSlider, 10.0f, 240.0f);
            const int wantedFps = static_cast<int>(std::lround(static_cast<double>(fpsSlider)));
            if (wantedFps != fpsApplied) {
                fpsApplied = wantedFps;
                SetTargetFPS(fpsApplied);
            }
        }
        // --- Painel do objeto selecionado (V6) --------------------------------------------------------------
        {
            physics::PhysicsWorld& world = *scene.world;
            fileListRect = viewer::RectF{};   // só existe na aba Cenas
            const bool selValid = world.isValidBody(selected);
            if (selValid && selected != lastSelected) {   // rascunho da orientação parte da orientação atual do corpo
                lastSelected = selected;
                const viewer::AxisAngleF aa0 = viewer::toAxisAngle(scene.world->getBody(selected).getOrientation());
                oAxisX = aa0.axis.x; oAxisY = aa0.axis.y; oAxisZ = aa0.axis.z;
                oAngleDeg = aa0.angle * 57.29578f;
                const physics::Vector3 w0 = viewer::anglesXYZFromQuaternion(scene.world->getBody(selected).getOrientation());
                oWx = static_cast<float>(w0.x) * 57.29578f; oWy = static_cast<float>(w0.y) * 57.29578f; oWz = static_cast<float>(w0.z) * 57.29578f;
            }
            if (addFailFrames > 0) --addFailFrames;
            GuiPanel(objPanel, addFailFrames > 0 ? "Objetos - NAO foi possivel criar"
                               : (mainTab == 0 && selValid ? TextFormat("Objeto selecionado (corpo %u)", static_cast<unsigned>(selected))
                                                           : "Objetos"));
            const float ox = objPanel.x + 12.0f, ow = objPanel.width - 24.0f;
            float oy = objPanel.y + 32.0f;
            fitToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "Objeto;Adicionar;Juntas", &mainTab);
            oy += 36.0f;
            // V20: rótulo e valor proporcionais à largura da coluna (na coluna mais estreita a barra ficava com 70 px):
            // a barra usa o que sobra; os textos encolhem a fonte (panelText) se não couberem.
            const float labelW = std::min(124.0f, std::max(92.0f, ow * 0.34f)), valueW = std::min(88.0f, std::max(56.0f, ow * 0.20f));
            const float barW = ow - labelW - valueW - 8.0f;
            auto sliderRow = [&](const char* name, float& v, float lo, float hi, const char* valueText, viewer::SliderUnit unit = viewer::SliderUnit{}) {
                const Rectangle bar{ox + labelW, oy, barW, 20.0f};
                labelText(name, ox, oy + 2.0f, ox + labelW - 6.0f);   // V21: o rótulo termina antes da barra
                const bool edited = sliderBox(bar, v, lo, hi, unit);
                panelText(valueText, static_cast<int>(ox + labelW + barW) + 6, static_cast<int>(oy) + 2, 16, kInk);
                oy += viewer::panel::kSliderStep;
                return edited;
            };
            using physics::Real;
            if (mainTab == 1) {   // ---- Adicionar objeto (V8) ----
                fitToggleGroup(Rectangle{ox, oy, ow / 4.0f, 26.0f}, "Forma;Posicao;Orient;Veloc", &addTab);
                oy += viewer::panel::kAddTabsStep;
                if (addTab == 0) {   // forma, tamanho, massa, material
                    {   // V18: 3 linhas de 3 botões: 0-2, 3-5, 6-8 (8 = modelo 3D)
                        static const char* const kRows[3] = {"caixa;esfera;tetraedro", "octaedro;prisma;piramide", "capsula;cilindro;modelo 3D"};
                        const int orig = kindIdx;
                        int pick = -1;
                        for (int r = 0; r < viewer::panel::kKindRows; ++r) {
                            int sel = orig / 3 == r ? orig % 3 : -1;
                            const int was = sel;
                            fitToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, kRows[r], &sel);
                            if (sel >= 0 && sel != was) pick = r * 3 + sel;
                            oy += viewer::panel::kKindRowStep;
                        }
                        if (pick >= 0) kindIdx = pick;
                    }
                    const bool isModel = kindIdx == viewer::kSpawnKindCount;
                    modelToggle = isModel;
                    if (isModel) {
                        if (fitButton(Rectangle{ox, oy, ow / 2.0f, 26.0f}, "Atualizar lista")) {
                            modelFiles = viewer::listModelFiles(modelDir);
                            modelLoaded = -2;
                            modelActive = -1;
                        }
                        oy += viewer::panel::kModelRowStep;
                    }
                    if (kindIdx == viewer::kSpawnKindCount) {   // ---- V15b: escolher o modelo ----
                        std::string joined;
                        for (std::size_t i = 0; i < modelFiles.size(); ++i) {
                            std::string n = modelFiles[i];
                            std::replace(n.begin(), n.end(), ';', '_');
                            joined += (i ? ";" : "") + n;
                        }
                        fileListRect = viewer::RectF{ox, oy, ow, 80.0f};   // a roda rola a lista, não a coluna
                        GuiListView(Rectangle{ox, oy, ow, 80.0f}, joined.empty() ? "(nenhum .obj: importe um abaixo)" : joined.c_str(), &modelScroll, &modelActive);
                        oy += viewer::panel::kListStep;
                        // ---- V17: importar o SEU arquivo 3D (arrastar para a janela ou digitar o caminho) ----
                        panelText("arraste um .obj para a janela, ou cole o caminho:", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                        {
                            const float btnW = 84.0f;
                            if (GuiTextBox(Rectangle{ox, oy + 17.0f, ow - btnW - 4.0f, 26.0f}, importPathBuf, 299, importEdit)) importEdit = !importEdit;
                            if (fitButton(Rectangle{ox + ow - btnW, oy + 17.0f, btnW, 26.0f}, "Importar")) {
                                std::string pth = importPathBuf;
                                while (!pth.empty() && (pth.back() == ' ' || pth.back() == '\n' || pth.back() == '\r')) pth.pop_back();
                                while (!pth.empty() && (pth.front() == ' ' || pth.front() == '"')) pth.erase(pth.begin());
                                while (!pth.empty() && pth.back() == '"') pth.pop_back();
                                if (pth.empty()) { importMsg = "ERRO: digite ou cole o caminho do .obj"; importMsgErr = true; importMsgFrames = 600; }
                                else { doImport(pth); if (!importMsgErr) importPathBuf[0] = '\0'; importEdit = false; }
                            }
                            if (importMsgFrames > 0) {
                                --importMsgFrames;
                                wrapPanelText(importMsg, static_cast<int>(ox), static_cast<int>(oy) + 45, 14, importMsgErr ? kWarn : kBlueInk, 3);
                            } else {
                                panelText("copiado para models/ (com .mtl e texturas)", static_cast<int>(ox), static_cast<int>(oy) + 45, 14, kInkDim);
                            }
                        }
                        oy += viewer::panel::kImportStep;
                        if (modelActive != modelLoaded) {   // escolha nova: lê o .obj e mostra o resumo
                            modelLoaded = modelActive;
                            modelObj.reset();
                            modelInfo1.clear(); modelInfo2.clear(); modelInfo3.clear();
                            modelInfoWarn = false;
                            if (modelActive >= 0 && modelActive < static_cast<int>(modelFiles.size())) {
                                auto obj = std::make_shared<viewer::ObjModel>();
                                std::string lerr, lwarn;
                                const std::string file = modelFiles[static_cast<std::size_t>(modelActive)];
                                viewer::ImportedShape is;
                                if (!viewer::loadObjFile(modelDir + "/" + file, *obj, lerr, &lwarn)) {
                                    modelInfo1 = "ERRO: " + lerr;
                                    modelInfoWarn = true;
                                } else if (!viewer::makeImportedShape(*obj, viewer::autoScale(*obj, sizeF), static_cast<viewer::ColliderKind>(colliderIdx), viewer::ColliderTuning{tuneR, tuneH}, is, lerr)) {
                                    modelInfo1 = "ERRO: " + lerr;
                                    modelInfoWarn = true;
                                } else {
                                    modelObj = obj;
                                    modelObjFile = file;
                                    modelInfo1 = std::to_string(obj->triangles.size()) + " triangulos | ";
                                    if (is.collider == viewer::ColliderKind::Hull)
                                        modelInfo1 += "fecho " + std::to_string(is.hullVertices) + " vert, " + std::to_string(is.hullFaces) + " faces";
                                    else if (is.collider == viewer::ColliderKind::Box)
                                        modelInfo1 += TextFormat("caixa %.2f x %.2f x %.2f m", 2.0 * static_cast<double>(is.boxHalf.x), 2.0 * static_cast<double>(is.boxHalf.y),
                                                                 2.0 * static_cast<double>(is.boxHalf.z));
                                    else if (is.collider == viewer::ColliderKind::Sphere)
                                        modelInfo1 += TextFormat("esfera raio %.2f m", static_cast<double>(is.sphereRadius));
                                    else
                                        modelInfo1 += TextFormat("%s raio %.2f, altura %.2f m", is.collider == viewer::ColliderKind::Capsule ? "capsula" : "cilindro",
                                                                 static_cast<double>(is.axisRadius),
                                                                 2.0 * static_cast<double>(is.axisHalfHeight) + (is.collider == viewer::ColliderKind::Capsule ? 2.0 * static_cast<double>(is.axisRadius) : 0.0));
                                    std::string tex = "sem textura (usa a cor)";
                                    for (const viewer::ObjMaterial& mat : obj->materials)
                                        if (!mat.diffuseMap.empty()) {
                                            const bool ok = std::filesystem::exists(std::filesystem::path(viewer::modelBaseDir(modelDir, file)) / mat.diffuseMap);
                                            tex = "textura: " + mat.diffuseMap + (ok ? "" : " (NAO ACHADA)");
                                            if (!ok) modelInfoWarn = true;
                                            break;
                                        }
                                    modelInfo2 = tex;
                                    if (is.collider == viewer::ColliderKind::Hull)
                                        modelInfo3 = is.concave ? "CONCAVO: colide como o fecho (preenche o buraco)" : "convexo: colisao exata";
                                    else
                                        modelInfo3 = is.fill > 0.0 ? std::string(TextFormat("colisor %.0f%% maior que o modelo", 100.0 * (1.0 / is.fill - 1.0))) : "colisor envolvente";
                                    if ((is.collider == viewer::ColliderKind::Capsule || is.collider == viewer::ColliderKind::Cylinder) && is.longestAxis != 1)
                                        modelInfo3 = "AVISO: o eixo mais longo nao e Y: gire o modelo no .obj";
                                    if (obj->triangles.size() > 60000) modelInfo3 = "modelo grande: pode ficar lento";
                                    if (!lwarn.empty()) modelInfo3 = lwarn;
                                }
                            }
                        }
                        if (modelInfo1.empty() && !modelObj) modelInfo1 = "escolha um modelo na lista";
                        panelText(modelInfo1.c_str(), static_cast<int>(ox), static_cast<int>(oy), 14, modelInfoWarn ? kWarn : kInk);
                        oy += 17.0f;
                        if (!modelInfo2.empty()) panelText(modelInfo2.c_str(), static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                        oy += 17.0f;
                        if (!modelInfo3.empty()) panelText(modelInfo3.c_str(), static_cast<int>(ox), static_cast<int>(oy), 14, modelInfo3[0] == 'C' ? kAccent : kInkDim);
                        oy += 20.0f;
                        // ---- V17: com que forma FISICA o motor enxerga o modelo ----
                        panelText("colisor (forma fisica do corpo):", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                        {
                            const int before = colliderIdx;
                            fitToggleGroup(Rectangle{ox, oy + 16.0f, ow / 5.0f, 26.0f}, "Fecho;Caixa;Esfera;Capsula;Cilindro", &colliderIdx);
                            if (colliderIdx != before) modelLoaded = -2;   // refaz o resumo com o novo colisor
                            static const char* const kWhy[5] = {"convexo fiel ao modelo (colisao mais exata)", "caixa envolvente: estavel, sobra folga nos cantos",
                                                                 "esfera envolvente: rola, sobra bastante folga",
                                                                 "capsula (eixo Y): ideal p/ pessoa em pe, rola ao tombar",
                                                                 "cilindro (eixo Y): colunas, latas; tem tampas planas"};
                            panelText(kWhy[colliderIdx >= 0 && colliderIdx < 5 ? colliderIdx : 0], static_cast<int>(ox), static_cast<int>(oy) + 44, 14, kInkDim);
                        }
                        oy += viewer::panel::kColliderStep;
                        if (colliderIdx >= 3) {   // V18: ajuste fino sobre o ajuste automatico
                            const float r0 = tuneR, h0 = tuneH;
                            sliderRow("raio do colisor", tuneR, viewer::kMinTuning, viewer::kMaxTuning, TextFormat("%.0f%%", static_cast<double>(tuneR * 100.0f)), viewer::SliderUnit{100.0f, false});
                            sliderRow("altura do colisor", tuneH, viewer::kMinTuning, viewer::kMaxTuning, TextFormat("%.0f%%", static_cast<double>(tuneH * 100.0f)), viewer::SliderUnit{100.0f, false});
                            if (tuneR != r0 || tuneH != h0) modelLoaded = -2;
                        }
                    }
                    sliderRow(isModel ? "maior dimensao" : "tamanho", sizeF, 0.2f, 4.0f, TextFormat("%.2f m", static_cast<double>(sizeF)));
                    sliderRow("massa", massS, 0.2236f, 10.0f, TextFormat("%.2f kg", static_cast<double>(massS * massS)), viewer::SliderUnit{1.0f, true});
                    sliderRow("atrito", fricF, 0.0f, 2.0f, TextFormat("%.2f", static_cast<double>(fricF)));
                    sliderRow("restituicao", restF, 0.0f, 1.0f, TextFormat("%.2f", static_cast<double>(restF)));
                    if (kindIdx == 0) {   // V14: dimensões por eixo (paredes, plataformas, rampas)
                        const char* en[3] = {"aresta x", "aresta y", "aresta z"};
                        for (int i = 0; i < 3; ++i)
                            sliderRow(en[i], spBox[i], 0.1f, 10.0f,
                                      TextFormat("%.2f m", static_cast<double>(std::min(40.0f, std::max(0.05f, sizeF * spBox[i])))));
                    }
                    if (kindIdx == 6 || kindIdx == 7) {   // V18: capsula/cilindro
                        sliderRow("raio", spAxisR, 0.1f, 2.0f, TextFormat("%.2f m", static_cast<double>(std::min(20.0f, std::max(0.02f, sizeF * spAxisR)))), viewer::SliderUnit{sizeF, false});
                        sliderRow("altura", spAxisH, 0.2f, 6.0f, TextFormat("%.2f m", static_cast<double>(std::min(40.0f, std::max(0.04f, sizeF * spAxisH)))), viewer::SliderUnit{sizeF, false});
                    }
                    checkLabel(ox, oy, " estatico (nao se move)", &spStatic);
                    if (kindIdx == viewer::kSpawnKindCount) {
                        oy += viewer::panel::kCheckStep;
                        checkLabel(ox, oy, " mostrar o colisor (fio)", &gShowHullWire);
                    }
                } else if (addTab == 1) {   // posição e velocidade de lançamento
                    sliderRow("x", spPos[0], -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(spPos[0])));
                    sliderRow("y", spPos[1], -2.0f, 20.0f, TextFormat("%+.2f m", static_cast<double>(spPos[1])));
                    sliderRow("z", spPos[2], -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(spPos[2])));
                    sliderRow("veloc. lanc.", launchSpd, 1.0f, 40.0f, TextFormat("%.1f m/s", static_cast<double>(launchSpd)));
                    panelText("lancar: sai 2 m a frente da camera", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                } else if (addTab == 2) {   // orientação inicial: eixo-ângulo OU quatérnio
                    fitToggleGroup(Rectangle{ox, oy, ow / 2.0f, 26.0f}, "Eixo-angulo;Quaternio", &qMode);
                    oy += 32.0f;
                    if (qMode == 0) {
                        sliderRow("eixo x", spAxis[0], -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(spAxis[0])));
                        sliderRow("eixo y", spAxis[1], -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(spAxis[1])));
                        sliderRow("eixo z", spAxis[2], -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(spAxis[2])));
                        sliderRow("angulo", spAngDeg, 0.0f, 360.0f, TextFormat("%.0f gr", static_cast<double>(spAngDeg)));
                    } else {
                        sliderRow("w", spQ[0], -1.0f, 1.0f, TextFormat("%+.3f", static_cast<double>(spQ[0])));
                        sliderRow("x", spQ[1], -1.0f, 1.0f, TextFormat("%+.3f", static_cast<double>(spQ[1])));
                        sliderRow("y", spQ[2], -1.0f, 1.0f, TextFormat("%+.3f", static_cast<double>(spQ[2])));
                        sliderRow("z", spQ[3], -1.0f, 1.0f, TextFormat("%+.3f", static_cast<double>(spQ[3])));
                    }
                    if (spOk)
                        panelText(TextFormat("q = (%+.3f, %+.3f, %+.3f, %+.3f)", static_cast<double>(spOrient.w), static_cast<double>(spOrient.x),
                                            static_cast<double>(spOrient.y), static_cast<double>(spOrient.z)),
                                 static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                    else
                        panelText("entrada nula: usa a identidade", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kWarn);
                } else {   // velocidades iniciais
                    const char* vn[3] = {"v x", "v y", "v z"};
                    const char* wn[3] = {"w x", "w y", "w z"};
                    for (int i = 0; i < 3; ++i) sliderRow(vn[i], spVel[i], -20.0f, 20.0f, TextFormat("%+.2f m/s", static_cast<double>(spVel[i])));
                    for (int i = 0; i < 3; ++i) sliderRow(wn[i], spAngV[i], -20.0f, 20.0f, TextFormat("%+.2f r/s", static_cast<double>(spAngV[i])));
                }
                // Rodapé fixo: posicionar com clique + botões Adicionar / Lançar.
                const float by = objPanel.y + objPanel.height - viewer::panel::kFooter;
                checkLabel(ox, by, " posicionar com clique", &clickPlace);
                if (fitButton(Rectangle{ox, by + 26.0f, ow / 2.0f - 4.0f, 28.0f}, "Adicionar")) afterSpawn(viewer::spawnBody(scene, makeSpawn()));
                if (fitButton(Rectangle{ox + ow / 2.0f + 4.0f, by + 26.0f, ow / 2.0f - 4.0f, 28.0f}, "Lancar da camera")) {
                    const physics::Vector3 camPos(cam.position.x, cam.position.y, cam.position.z);
                    const physics::Vector3 camDir(cam.target.x - cam.position.x, cam.target.y - cam.position.y, cam.target.z - cam.position.z);
                    afterSpawn(viewer::launchFrom(scene, makeSpawn(), camPos, camDir, static_cast<physics::Real>(launchSpd)));
                }
            } else if (mainTab == 2) {   // ---- Juntas (V14): dobradiça, bola-soquete e distância ----
                fitToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "Dobradica;Bola-soquete;Distancia", &jKind);
                oy += 32.0f;
                panelText(selValid ? TextFormat("A: corpo %u (o selecionado)", static_cast<unsigned>(selected))
                                  : "A: clique num corpo da cena", static_cast<int>(ox), static_cast<int>(oy) + 1, 14, selValid ? kInk : kWarn);
                oy += 18.0f;
                int bSel = jPickB ? 2 : (jBMode == 1 ? 1 : 0);
                const int bOrig = bSel;
                fitToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "B: mundo;B: corpo;B: clicar", &bSel);
                if (bSel != bOrig) {
                    jPickB = (bSel == 2) || (bSel == 1 && !world.isValidBody(jB));
                    jBMode = (bSel == 0) ? 0 : 1;
                    if (bSel == 0) { jB = physics::kInvalidBodyId; }
                }
                oy += 32.0f;
                panelText(jPickB ? "B: clique no corpo B na cena..." : (jBMode == 1 ? TextFormat("B: corpo %u", static_cast<unsigned>(jB)) : "B: o mundo (cria um pivo fixo)"),
                         static_cast<int>(ox), static_cast<int>(oy) + 1, 14, jPickB ? kAccent : kInk);
                oy += 18.0f;
                fitToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "Sliders;Ponto clicado;Centro de A", &jAnchorMode);
                oy += 32.0f;
                if (jAnchorMode == 0) {
                    sliderRow("ancora x", jAnchor[0], -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(jAnchor[0])));
                    sliderRow("ancora y", jAnchor[1], -2.0f, 14.0f, TextFormat("%+.2f m", static_cast<double>(jAnchor[1])));
                    sliderRow("ancora z", jAnchor[2], -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(jAnchor[2])));
                } else if (jAnchorMode == 1) {
                    panelText(jHasClick ? TextFormat("ponto clicado: (%.2f, %.2f, %.2f)", static_cast<double>(jClick.x), static_cast<double>(jClick.y),
                                                    static_cast<double>(jClick.z))
                                       : "clique na cena para marcar o ponto",
                             static_cast<int>(ox), static_cast<int>(oy) + 2, 14, jHasClick ? kInk : kWarn);
                    oy += 26.0f;
                } else {
                    panelText("a ancora fica no centro do corpo A", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                    oy += 26.0f;
                }
                if (jKind == 0) {
                    sliderRow("eixo x", jAxis[0], -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(jAxis[0])));
                    sliderRow("eixo y", jAxis[1], -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(jAxis[1])));
                    sliderRow("eixo z", jAxis[2], -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(jAxis[2])));
                } else if (jKind == 2) {
                    panelText("distancia: pontas = centro de A e B/ancora", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                    oy += 26.0f;
                }
                checkLabel(ox, oy, " os dois corpos colidem entre si", &jCollide);
                oy += 28.0f;
                if (fitButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Criar junta")) {
                    viewer::JointParams jp;
                    jp.kind = static_cast<viewer::JointKind>(jKind);
                    jp.a = selected;
                    jp.b = jBMode == 1 ? jB : physics::kInvalidBodyId;
                    jp.collideConnected = jCollide;
                    jp.axis = physics::Vector3(jAxis[0], jAxis[1], jAxis[2]);
                    bool haveAnchor = true;
                    if (jAnchorMode == 0) jp.anchor = physics::Vector3(jAnchor[0], jAnchor[1], jAnchor[2]);
                    else if (jAnchorMode == 1 && jHasClick) jp.anchor = jClick;
                    else if (jAnchorMode == 2 && selValid) jp.anchor = world.getBody(selected).getWorldCenterOfMass();
                    else haveAnchor = false;
                    std::string err;
                    if (!haveAnchor) {
                        jMsg = jAnchorMode == 1 ? "ERRO: clique na cena para marcar o ponto" : "ERRO: selecione o corpo A";
                        jMsgErr = true;
                    } else if (viewer::createJoint(scene, jp, &err)) {
                        jMsg = std::string("criada: ") + viewer::jointKindName(jp.kind);
                        jMsgErr = false;
                    } else {
                        jMsg = "ERRO: " + err;
                        jMsgErr = true;
                    }
                    jMsgFrames = 420;
                }
                if (fitButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Apagar a ultima")) {
                    jMsg = viewer::destroyLastJoint(scene) ? "ultima junta apagada" : "nao ha juntas";
                    jMsgErr = false;
                    jMsgFrames = 420;
                }
                oy += 34.0f;
                panelText(TextFormat("juntas na cena: %d", viewer::jointCount(scene)), static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                oy += 18.0f;
                if (jMsgFrames > 0) {
                    --jMsgFrames;
                    panelText(jMsg.size() > 42 ? (jMsg.substr(0, 40) + "..").c_str() : jMsg.c_str(), static_cast<int>(ox), static_cast<int>(oy), 14,
                             jMsgErr ? kWarn : kBlueInk);
                }
            } else if (!selValid) {
                panelText("Nenhum objeto selecionado.", static_cast<int>(ox), static_cast<int>(oy) + 4, 16, kInk);
                panelText("Clique num corpo na cena, ou use a aba", static_cast<int>(ox), static_cast<int>(oy) + 32, 14, kInkDim);
                panelText("Adicionar para criar um novo.", static_cast<int>(ox), static_cast<int>(oy) + 50, 14, kInkDim);
            } else {
            fitToggleGroup(Rectangle{ox, oy, ow / 4.0f, 26.0f}, "Material;Posicao;Veloc;Orient", &objTab);
            oy += 36.0f;
            viewer::BodyParams bp;
            viewer::readParams(world, selected, bp);
            const physics::RigidBody& body = world.getBody(selected);
            if (objTab == 0 && body.isStatic()) {   // V14: estático — só o material (massa infinita)
                float f = static_cast<float>(bp.friction);
                if (sliderRow("atrito", f, 0.0f, 2.0f, TextFormat("%.2f", static_cast<double>(f)))) viewer::setFriction(world, selected, static_cast<Real>(f));
                float r = static_cast<float>(bp.restitution);
                if (sliderRow("restituicao", r, 0.0f, 1.0f, TextFormat("%.2f", static_cast<double>(r)))) viewer::setRestitution(world, selected, static_cast<Real>(r));
                panelText("corpo estatico: massa infinita", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                oy += 26.0f;
                if (fitButton(Rectangle{ox, oy, ow, 28.0f}, "Apagar este objeto (Del)")) {
                    drag.end();
                    viewer::deleteBody(scene, selected, true);
                    selected = physics::kInvalidBodyId;
                }
            } else if (objTab == 0) {   // Material
                float m = std::sqrt(static_cast<float>(bp.mass));   // escala √: mais precisão nas massas pequenas
                if (sliderRow("massa", m, 0.2236f, 10.0f, TextFormat("%.2f kg", static_cast<double>(m * m)), viewer::SliderUnit{1.0f, true}))
                    viewer::setMass(world, selected, static_cast<Real>(m * m));
                float f = static_cast<float>(bp.friction);
                if (sliderRow("atrito", f, 0.0f, 2.0f, TextFormat("%.2f", static_cast<double>(f)))) viewer::setFriction(world, selected, static_cast<Real>(f));
                float r = static_cast<float>(bp.restitution);
                if (sliderRow("restituicao", r, 0.0f, 1.0f, TextFormat("%.2f", static_cast<double>(r)))) viewer::setRestitution(world, selected, static_cast<Real>(r));
                float ld = static_cast<float>(bp.linearDamping);
                if (sliderRow("amort. linear", ld, 0.0f, 5.0f, TextFormat("%.2f", static_cast<double>(ld)))) viewer::setLinearDamping(world, selected, static_cast<Real>(ld));
                float ad = static_cast<float>(bp.angularDamping);
                if (sliderRow("amort. angular", ad, 0.0f, 5.0f, TextFormat("%.2f", static_cast<double>(ad)))) viewer::setAngularDamping(world, selected, static_cast<Real>(ad));
                float gs = static_cast<float>(bp.gravityScale);
                if (sliderRow("escala grav.", gs, -2.0f, 5.0f, TextFormat("%+.2f", static_cast<double>(gs)))) viewer::setGravityScale(world, selected, static_cast<Real>(gs));
                bool allow = bp.allowSleep;
                checkLabel(ox, oy, " pode dormir (sleeping)", &allow);
                if (allow != bp.allowSleep) viewer::setAllowSleep(world, selected, allow);
            } else if (objTab == 1) {   // Posicao + acordar/dormir
                const physics::Vector3 pos = body.getTransform().position;
                float px = static_cast<float>(pos.x), py = static_cast<float>(pos.y), pz = static_cast<float>(pos.z);
                bool moved = false;
                moved |= sliderRow("x", px, -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(px)));
                moved |= sliderRow("y", py, -2.0f, 14.0f, TextFormat("%+.2f m", static_cast<double>(py)));
                moved |= sliderRow("z", pz, -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(pz)));
                if (moved) viewer::setPosition(world, selected, physics::Vector3(px, py, pz));
                oy += 6.0f;
                if (body.isStatic()) {
                    panelText("estatico: mova pelos sliders", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                } else {
                panelText(bp.awake ? "estado: acordado" : "estado: dormindo", static_cast<int>(ox), static_cast<int>(oy), 16, bp.awake ? kInkDim : kBlueInk);
                oy += 24.0f;
                if (fitButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Acordar")) viewer::setAwake(world, selected, true);
                if (fitButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Dormir")) viewer::setAwake(world, selected, false);
                }
            } else if (objTab == 2 && body.isStatic()) {
                panelText("estatico: nao tem velocidade", static_cast<int>(ox), static_cast<int>(oy) + 4, 14, kInkDim);
            } else if (objTab == 2) {   // Velocidades
                const physics::Vector3 v = body.getLinearVelocity(), w = body.getAngularVelocity();
                float vv[3] = {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
                float ww[3] = {static_cast<float>(w.x), static_cast<float>(w.y), static_cast<float>(w.z)};
                bool chV = false, chW = false;
                const char* vn[3] = {"v x", "v y", "v z"};
                const char* wn[3] = {"w x", "w y", "w z"};
                for (int i = 0; i < 3; ++i) chV |= sliderRow(vn[i], vv[i], -20.0f, 20.0f, TextFormat("%+.2f m/s", static_cast<double>(vv[i])));
                for (int i = 0; i < 3; ++i) chW |= sliderRow(wn[i], ww[i], -20.0f, 20.0f, TextFormat("%+.2f r/s", static_cast<double>(ww[i])));
                if (chV) viewer::setLinearVelocity(world, selected, physics::Vector3(vv[0], vv[1], vv[2]));
                if (chW) viewer::setAngularVelocity(world, selected, physics::Vector3(ww[0], ww[1], ww[2]));
                oy += 2.0f;
                if (fitButton(Rectangle{ox, oy, ow, 28.0f}, "Zerar velocidades")) viewer::zeroVelocities(world, selected);
            } else {   // Orientação: eixo + ângulo -> quatérnio, ou três ângulos ωx ωy ωz (V25)
                const int oModePrev = oMode;
                fitToggleGroup(Rectangle{ox, oy, ow / 2.0f, 26.0f}, "Eixo-angulo;Omega x y z", &oMode);
                oy += 32.0f;
                const auto readAngles = [&]() {   // ωx ωy ωz a partir da orientação atual do corpo
                    const physics::Vector3 w1 = viewer::anglesXYZFromQuaternion(body.getOrientation());
                    oWx = static_cast<float>(w1.x) * 57.29578f; oWy = static_cast<float>(w1.y) * 57.29578f; oWz = static_cast<float>(w1.z) * 57.29578f;
                };
                if (oMode == 1 && oModePrev == 0) readAngles();
                if (oMode == 0) {
                sliderRow("eixo x", oAxisX, -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(oAxisX)));
                sliderRow("eixo y", oAxisY, -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(oAxisY)));
                sliderRow("eixo z", oAxisZ, -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(oAxisZ)));
                sliderRow("angulo", oAngleDeg, 0.0f, 360.0f, TextFormat("%.0f gr", static_cast<double>(oAngleDeg)));
                oy += 2.0f;
                if (fitButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Aplicar q"))
                    viewer::setOrientationAxisAngle(world, selected, physics::Vector3(oAxisX, oAxisY, oAxisZ),
                                                    static_cast<Real>(oAngleDeg * 0.017453292519943295f));
                if (fitButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Identidade")) viewer::resetOrientation(world, selected);
                oy += 34.0f;
                panelText("q = (cos(a/2), sen(a/2) * eixo unitario)", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                } else {
                    // ωx ωy ωz: aplica na hora (arrastar o slider gira o corpo) — gira ωx em torno de X, depois ωy em Y, depois ωz em Z
                    bool chA = false;
                    chA |= sliderRow("omega x", oWx, -180.0f, 180.0f, TextFormat("%+.0f gr", static_cast<double>(oWx)));
                    chA |= sliderRow("omega y", oWy, -90.0f, 90.0f, TextFormat("%+.0f gr", static_cast<double>(oWy)));
                    chA |= sliderRow("omega z", oWz, -180.0f, 180.0f, TextFormat("%+.0f gr", static_cast<double>(oWz)));
                    if (chA)
                        viewer::setOrientationAnglesXYZ(world, selected, static_cast<Real>(oWx * 0.017453292519943295f),
                                                        static_cast<Real>(oWy * 0.017453292519943295f), static_cast<Real>(oWz * 0.017453292519943295f));
                    oy += 2.0f;
                    if (fitButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Ler do corpo")) readAngles();
                    if (fitButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Identidade")) {
                        viewer::resetOrientation(world, selected);
                        oWx = oWy = oWz = 0.0f;
                    }
                    oy += 34.0f;
                    panelText("gira omega x (eixo X), depois y, depois z", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                    oy += 16.0f;
                    panelText("eixos fixos do mundo; q = qz * qy * qx", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                }
            }
            }
            if (mainTab == 0)   // V14: permite clicar no cenário estático (chão, paredes) para editá-lo
                checkLabel(ox, objPanel.y + objPanel.height - 30.0f, " clicar tambem em estaticos", &pickStaticsOpt);
        }

        endDock();

        // --- Painel do mundo (V7) -----------------------------------------------------------------------------
        beginDock(viewer::DockSide::Left);
        if (showWorld) {
            physics::PhysicsWorld& world = *scene.world;
            GuiPanel(worldPanel, "Mundo e debug (M esconde)");
            const float wx = worldPanel.x + 12.0f, ww = worldPanel.width - 24.0f;
            float wy = worldPanel.y + 30.0f;
            fitToggleGroup(Rectangle{wx, wy, ww / 5.0f, 26.0f}, "Mundo;Solver;Debug;Stats;Evento", &worldTab);
            wy += 34.0f;
            const float labelW = std::min(128.0f, std::max(96.0f, ww * 0.36f)), valueW = std::min(96.0f, std::max(60.0f, ww * 0.22f));
            const float barW = ww - labelW - valueW - 8.0f;
            // Uma linha de slider: devolve true se o usuário mexeu; o valor aparece à direita.
            auto row = [&](const char* name, float& v, float lo, float hi, const char* valueText, viewer::SliderUnit unit = viewer::SliderUnit{}) {
                labelText(name, wx, wy + 2.0f, wx + labelW - 6.0f);   // V21
                const bool edited = sliderBox(Rectangle{wx + labelW, wy, barW, 20.0f}, v, lo, hi, unit);
                panelText(valueText, static_cast<int>(wx + labelW + barW) + 6, static_cast<int>(wy) + 2, 16, kInk);
                wy += 26.0f;
                return edited;
            };
            using physics::Real;
            viewer::WorldParams wp = viewer::captureWorld(world);
            bool changed = false;
            if (worldTab == 0) {   // Mundo
                float gx = static_cast<float>(wp.gravity.x), gy = static_cast<float>(wp.gravity.y), gz = static_cast<float>(wp.gravity.z);
                if (row("gravidade x", gx, -30.0f, 30.0f, TextFormat("%+.2f m/s2", static_cast<double>(gx)))) { wp.gravity.x = gx; changed = true; }
                if (row("gravidade y", gy, -30.0f, 30.0f, TextFormat("%+.2f m/s2", static_cast<double>(gy)))) { wp.gravity.y = gy; changed = true; }
                if (row("gravidade z", gz, -30.0f, 30.0f, TextFormat("%+.2f m/s2", static_cast<double>(gz)))) { wp.gravity.z = gz; changed = true; }
                float hz = static_cast<float>(1.0 / static_cast<double>(wp.fixedDt));
                if (row("fisica (passo)", hz, 10.0f, 240.0f, TextFormat("%.0f Hz", static_cast<double>(hz)))) { wp.fixedDt = Real(1) / static_cast<Real>(hz); changed = true; }
                bool sl = wp.sleepEnabled;
                checkLabel(wx, wy, " sleeping ligado", &sl);
                if (sl != wp.sleepEnabled) { wp.sleepEnabled = sl; changed = true; }
                wy += 26.0f;
                float st = static_cast<float>(wp.sleepTime);
                if (row("dormir apos", st, 0.05f, 10.0f, TextFormat("%.2f s", static_cast<double>(st)))) { wp.sleepTime = st; changed = true; }
            } else if (worldTab == 1) {   // Solver
                float vi = static_cast<float>(wp.velocityIterations), pi = static_cast<float>(wp.positionIterations);
                if (row("iter. veloc.", vi, 1.0f, 50.0f, TextFormat("%d", static_cast<int>(std::lround(static_cast<double>(vi)))))) { wp.velocityIterations = static_cast<int>(std::lround(static_cast<double>(vi))); changed = true; }
                if (row("iter. posicao", pi, 0.0f, 20.0f, TextFormat("%d%s", static_cast<int>(std::lround(static_cast<double>(pi))), pi < 0.5f ? " (Baumg.)" : ""))) { wp.positionIterations = static_cast<int>(std::lround(static_cast<double>(pi))); changed = true; }
                float ls = static_cast<float>(wp.linearSlop), vs = static_cast<float>(wp.velocitySlop);
                if (row("slop linear", ls, 0.0f, 0.05f, TextFormat("%.1f mm", static_cast<double>(ls) * 1000.0), viewer::SliderUnit{1000.0f, false})) { wp.linearSlop = ls; changed = true; }
                if (row("slop veloc.", vs, 0.0f, 2.0f, TextFormat("%.2f m/s", static_cast<double>(vs)))) { wp.velocitySlop = vs; changed = true; }
                bool ws = wp.warmStarting;
                checkLabel(wx, wy, " warm starting", &ws);
                if (ws != wp.warmStarting) { wp.warmStarting = ws; changed = true; }
                wy += 26.0f;
                bool oe = wp.orientationExponential;   // V24
                checkLabel(wx, wy, " orientacao: exponencial de quaternio", &oe);
                if (oe != wp.orientationExponential) { wp.orientationExponential = oe; changed = true; }
                wy += 26.0f;
                panelText("desligado = Euler: q + 1/2 dt (0,w) q", static_cast<int>(wx), static_cast<int>(wy), 14, kInkDim);
                wy += 18.0f;
                panelText("menos iteracoes: mais barato, mais mole", static_cast<int>(wx), static_cast<int>(wy), 14, kInkDim);
            } else if (worldTab == 2) {   // Debug
                const std::vector<viewer::DebugFlagInfo>& table = viewer::debugFlagTable();
                const float colW = ww / 2.0f;
                for (std::size_t i = 0; i < table.size(); ++i) {
                    const float cx = wx + (i < 5 ? 0.0f : colW), cy = wy + static_cast<float>(i < 5 ? i : i - 5) * 24.0f;
                    bool on = (dbgFlags & table[i].flag) != 0;
                    checkLabel(cx, cy, TextFormat(" %s", table[i].label), &on, cx + colW - 6.0f);   // V20: o rótulo não invade a outra coluna
                    if (on) dbgFlags |= table[i].flag; else dbgFlags &= ~table[i].flag;
                }
                wy += 5.0f * 24.0f + 6.0f;
                checkLabel(wx, wy, " esconder modelos (so o debug)", &hideModels);
                wy += 30.0f;
                if (fitButton(Rectangle{wx, wy, ww / 2.0f - 4.0f, 26.0f}, "Padrao")) dbgFlags = static_cast<std::uint32_t>(physics::kDebugDefault);
                if (fitButton(Rectangle{wx + ww / 2.0f + 4.0f, wy, ww / 2.0f - 4.0f, 26.0f}, "Desligar")) dbgFlags = 0;
            } else if (worldTab == 4) {   // Evento (V9)
                checkLabel(wx, wy, " cor por estado (dormindo = azul)", &tintOn);
                wy += 24.0f;
                checkLabel(wx, wy, " flash no inicio do contato", &flashOn);
                wy += 24.0f;
                checkLabel(wx, wy, " aneis nos pontos de impacto", &ringsOn);
                wy += 30.0f;
                const physics::WorldStats st = world.getStats();
                auto line = [&](const char* text) { panelText(text, static_cast<int>(wx), static_cast<int>(wy), 16, kInk); wy += 19.0f; };
                line(TextFormat("contatos agora %d   dormindo %d de %d", static_cast<int>(world.getContacts().size()), st.sleepingBodies, st.awakeBodies + st.sleepingBodies));
                line(TextFormat("inicios %llu   fins %llu", static_cast<unsigned long long>(tracker.begins()), static_cast<unsigned long long>(tracker.ends())));
                line(TextFormat("impacto: ultimo %.1f   maximo %.1f m/s", static_cast<double>(tracker.lastImpact()), static_cast<double>(tracker.maxImpact())));
                if (fitButton(Rectangle{wx, wy + 4.0f, ww, 24.0f}, "Zerar contadores")) tracker.reset();
            } else {   // Stats
                bool prof = profilingOn;
                checkLabel(wx, wy, " medir tempos (profiling)", &prof);
                if (prof != profilingOn) { profilingOn = prof; world.setProfilingEnabled(profilingOn); }
                wy += 28.0f;
                const physics::WorldStats st = world.getStats();
                auto line = [&](const char* text) { panelText(text, static_cast<int>(wx), static_cast<int>(wy), 16, kInk); wy += 19.0f; };
                line(TextFormat("corpos %d   acordados %d   dormindo %d", st.bodies, st.awakeBodies, st.sleepingBodies));
                line(TextFormat("ilhas acordadas %d   juntas %d (solver %d)", st.islands, st.joints, st.solverJoints));
                line(TextFormat("pares %d   tocando %d   congelados %d", st.pairs, st.touchingPairs, st.frozenPairs));
                line(TextFormat("pontos de contato %d   no solver %d", st.contactPoints, st.solverContacts));
                line(TextFormat("passo #%llu   linhas de debug %d", static_cast<unsigned long long>(st.stepCount), static_cast<int>(dbgBuf.size())));
                if (profilingOn) {
                    line(TextFormat("tempos do ultimo passo (us): total %.0f", st.timings.totalUs));
                    line(TextFormat("  colisao %.0f   ilhas %.0f   restricoes %.0f", st.timings.collisionUs, st.timings.islandsUs, st.timings.constraintsUs));
                    line(TextFormat("  integracao %.0f   sono/eventos %.0f", st.timings.integrationUs, st.timings.sleepEventsUs));
                } else {
                    line("ligue \"medir tempos\" para ver os custos");
                }
            }
            // Rodapé das abas Mundo e Solver: restaurar os valores com que a cena foi criada.
            if (worldTab <= 1) {
                wy = worldPanel.y + worldPanel.height - 36.0f;
                if (fitButton(Rectangle{wx, wy, ww, 26.0f}, "Restaurar padroes da cena")) { wp = sceneDefaults; changed = true; }
            }
            if (changed) viewer::applyWorld(world, wp);
        }

        // --- Gravação da trajetória em CSV (V16) ---
        {
            GuiPanel(recPanel, "Gravar trajetoria (CSV)   F7");
            const float rx = recPanel.x + 12.0f, rw = recPanel.width - 24.0f;
            float ry = recPanel.y + 30.0f;
            fitToggleGroup(Rectangle{rx, ry, rw / 2.0f, 26.0f}, "Selecionado;Todos", &recTarget);
            ry += 32.0f;
            if (fitButton(Rectangle{rx, ry, rw / 2.0f - 3.0f, 28.0f}, recorder.recording() ? "Parar" : "Gravar")) uiRecToggle = true;
            if (fitButton(Rectangle{rx + rw / 2.0f + 3.0f, ry, rw / 2.0f - 3.0f, 28.0f}, "Salvar CSV")) uiRecSave = true;
            ry += 34.0f;
            panelText(recorder.recording() ? TextFormat("* gravando: %d linhas, %.2f s", static_cast<int>(recorder.rowCount()), recorder.duration())
                                         : TextFormat("parado: %d linhas, %.2f s", static_cast<int>(recorder.rowCount()), recorder.duration()),
                     static_cast<int>(rx), static_cast<int>(ry), 16, recorder.recording() ? Color{255, 90, 90, 255} : kInk);
            ry += 20.0f;
            panelText("t, passo, corpo, x y z, qw qx qy qz, eixo-angulo,", static_cast<int>(rx), static_cast<int>(ry), 14, kInk);
            ry += 16.0f;
            panelText("Euler, v, w   (so grava com a fisica andando)", static_cast<int>(rx), static_cast<int>(ry), 14, kInk);
            ry += 18.0f;
            if (recMsgFrames > 0 && !recMsg.empty()) panelText(recMsg.c_str(), static_cast<int>(rx), static_cast<int>(ry), 14, kAccent);
            ry += 18.0f;
            if (!recLastPath.empty()) {   // V20: o caminho inteiro, em várias linhas (antes era cortado com "..")
                panelText("ultimo CSV salvo em:", static_cast<int>(rx), static_cast<int>(ry), 14, kInkDim);
                ry += 16.0f;
                ry += wrapPanelText(recLastPath, static_cast<int>(rx), static_cast<int>(ry), 14, kAccent, viewer::panel::kRecPathLines) + 4.0f;
                if (fitButton(Rectangle{rx, ry, rw, 24.0f}, "Copiar caminho")) SetClipboardText(recLastPath.c_str());
            }
        }

        endDock();
        }   // fim de !presentation

        // Barras de rolagem (por cima de tudo na coluna) e persistência do layout.
        for (int side = 0; !presentation && side < 2; ++side) {
            viewer::ScrollState& sc = side == 0 ? scrollL : scrollR;
            const viewer::RectF& body = side == 0 ? bodyL : bodyR;
            const viewer::DockSide ds = side == 0 ? viewer::DockSide::Left : viewer::DockSide::Right;
            if (dock.collapsed(ds) || body.w <= 120.0f || !sc.scrollable()) continue;
            const viewer::RectF track{body.right() - kBar, body.y, kBar, body.h};
            DrawRectangle(static_cast<int>(track.x), static_cast<int>(track.y), static_cast<int>(track.w), static_cast<int>(track.h), Color{28, 30, 38, 255});
            DrawRectangle(static_cast<int>(track.x) + 2, static_cast<int>(track.y + sc.thumbTop(track.h)), static_cast<int>(track.w) - 4,
                          static_cast<int>(sc.thumbHeight(track.h)), scrollDrag == side ? Color{150, 190, 255, 255} : Color{110, 118, 138, 255});
        }
        if (libDirty) {   // V22: ordem e remoções das cenas (scenes/biblioteca.txt)
            std::error_code ec;
            std::filesystem::create_directories(sceneDir, ec);
            std::ofstream f(libPath, std::ios::trunc);
            if (f) f << lib.serialize();
            libDirty = false;
        }
        if (layoutDirty && !dock.dragging()) {
            std::error_code ec;
            std::filesystem::create_directories(sceneDir, ec);
            std::ofstream f(layoutPath, std::ios::trunc);
            if (f) f << dock.serialize();
            layoutDirty = false;
        }
        EndDrawing();

        // --- Modo automático --------------------------------------------------------------------------------------------
        ++frame;
        if (autoFrames > 0 && frame >= autoFrames) {
            if (screenshotPath != nullptr) {
                Image img = LoadImageFromScreen();
                const bool ok = ExportImage(img, screenshotPath);
                UnloadImage(img);
                std::printf("captura %s: %s\n", screenshotPath, ok ? "salva" : "FALHOU");
                if (!ok) {
                    unloadModelCache();
                    CloseWindow();
                    return 1;
                }
            }
            break;
        }
    }

    unloadModelCache();
    if (vpTex.id != 0) UnloadRenderTexture(vpTex);
    CloseWindow();
    return 0;
}
