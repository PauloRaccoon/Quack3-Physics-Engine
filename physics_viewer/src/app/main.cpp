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

#include "raylib.h"
#include "rlgl.h"

// raygui: biblioteca de um só cabeçalho; a implementação é compilada neste arquivo.
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "viewer/core/BodyEditor.h"
#include "viewer/core/ContactTracker.h"
#include "viewer/core/Interaction.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/OrbitCamera.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/TimeControl.h"
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
constexpr Color kInk{30, 32, 40, 255};       // texto principal nos painéis
constexpr Color kInkDim{70, 74, 86, 255};    // texto secundário nos painéis
constexpr Color kAccent{150, 70, 0, 255};    // valores do quatérnio
constexpr Color kWarn{190, 25, 25, 255};     // avisos (|q| != 1, gimbal lock)
constexpr Color kBlueInk{20, 80, 170, 255};

/// Texto sobre a cena: faixa escura translúcida atrás (mesma assinatura de DrawText).
void hudText(const char* text, int x, int y, int size, Color color) {
    const int w = MeasureText(text, size);
    DrawRectangle(x - 6, y - 3, w + 12, size + 6, Color{10, 12, 18, 190});
    DrawText(text, x, y, size, color);
}

/// Slider com a regra do painel: só conta como edição do usuário se o valor mudou COM o botão apertado
/// e o mouse sobre a barra (o raygui limita valores fora da faixa, e isso não pode alterar o mundo).
bool sliderEdited(const Rectangle& bar, float& v, float lo, float hi) {
    const float before = v;
    GuiSlider(bar, nullptr, nullptr, &v, lo, hi);
    return v != before && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(GetMousePosition(), bar);
}

/// Desenha um corpo na pose `m` (matriz de modelo vinda do quatérnio do motor).
void drawBody(const viewer::SceneBody& body, const viewer::Mat4f& m) {
    const Color fill{body.r, body.g, body.b, 255};
    const Color wire{static_cast<unsigned char>(body.r / 3), static_cast<unsigned char>(body.g / 3),
                     static_cast<unsigned char>(body.b / 3), 255};

    rlPushMatrix();
    rlMultMatrixf(m.m);   // 16 floats em ordem de colunas: a pose inteira (rotação + translação)

    if (body.kind == viewer::ShapeKind::Box) {
        const float w = 2.0f * body.halfExtents.x;
        const float h = 2.0f * body.halfExtents.y;
        const float l = 2.0f * body.halfExtents.z;
        DrawCube(Vector3{0, 0, 0}, w, h, l, fill);
        DrawCubeWires(Vector3{0, 0, 0}, w, h, l, wire);
    } else if (body.kind == viewer::ShapeKind::Polyhedron && body.poly) {
        const physics::HalfEdgeStructure& topo = body.poly->getTopology();
        // Cada face vira um leque de triângulos (anti-horário visto de fora). O tom varia com a
        // normal da face NO MUNDO (rotação da matriz m), simulando uma luz fixa.
        const viewer::Vec3f light{0.4f, 0.8f, 0.45f};
        for (const physics::HEFace& f : topo.faces()) {
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
        for (int i = 0; i < topo.getEdgeCount(); ++i) {   // cada aresta uma vez (half-edge < twin)
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
    InitWindow(kScreenW, kScreenH, "physics_viewer — V10");
    SetWindowMinSize(960, 700);   // abaixo disso os painéis da direita passam por cima do texto de baixo
    int fpsApplied = 120;
    float fpsSlider = 120.0f;
    SetTargetFPS(fpsApplied);   // o render NÃO precisa casar com a taxa da física: há interpolação
    GuiSetStyle(DEFAULT, TEXT_SIZE, 16);
    viewer::InterpMode mode = viewer::InterpMode::Slerp;

    viewer::OrbitCamera orbit;
    const std::size_t sceneCount = viewer::sceneList().size();
    std::size_t sceneIndex = 0;
    viewer::ContactTracker tracker;   // V9: efeitos dos eventos de contato (vive mais que os mundos)
    bool tintOn = true, flashOn = true, ringsOn = true;
    viewer::Scene scene = viewer::makeScene(sceneIndex);
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
    // Painel "Adicionar objeto" (V8): aba principal (0 = Objeto, 1 = Adicionar) e o rascunho do que será criado.
    int mainTab = 0, addTab = 0, kindIdx = 0, qMode = 0, addFailFrames = 0;
    // Cenas salvas/carregadas (V12).
#ifdef VIEWER_SCENES_DIR
    const std::string sceneDir = VIEWER_SCENES_DIR;
#else
    const std::string sceneDir = "scenes";
#endif
    std::vector<std::string> sceneFiles = viewer::listSceneFiles(sceneDir);
    char fileName[49] = "";
    bool fileNameEdit = false;
    int fileScroll = 0, fileActive = -1, fileMsgFrames = 0;
    bool fileMsgErr = false;
    std::string fileMsg, loadRequest;
    bool customActive = false;   // a cena atual veio de um arquivo: R recria a partir de customText
    std::string customText;
    bool clickPlace = false;
    float sizeF = 1.0f, massS = 1.0f /* raiz da massa */, fricF = 0.5f, restF = 0.2f, launchSpd = 18.0f;
    float spPos[3] = {0.0f, 6.0f, 0.0f}, spVel[3] = {0.0f, 0.0f, 0.0f}, spAngV[3] = {0.0f, 0.0f, 0.0f};
    float spAxis[3] = {0.0f, 1.0f, 0.0f}, spAngDeg = 0.0f, spQ[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    physics::BodyId lastSelected = physics::kInvalidBodyId;
    float oAxisX = 0.0f, oAxisY = 1.0f, oAxisZ = 0.0f, oAngleDeg = 0.0f;

    // Pose de RENDER de um corpo: interpola os dois últimos passos físicos com o α atual, no modo escolhido.
    auto renderPose = [&](physics::BodyId id) {
        const physics::MotionState& ms = scene.world->getMotionState(id);
        return viewer::interpolatePose(ms.getPrevious(), ms.getCurrent(), timeCtl.renderAlpha(*scene.world), mode);
    };

    // V12: com o campo de nome em edição os atalhos de teclado ficam desligados (senão "r", "t", espaço... agiriam).
    SetExitKey(KEY_NULL);   // ESC não fecha a janela sozinho: fecha só fora da edição (ver abaixo)
    bool quitRequested = false;
    auto keyPressed = [&](int k) { return !fileNameEdit && IsKeyPressed(k); };
    while (!WindowShouldClose() && !quitRequested) {
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (fileNameEdit) fileNameEdit = false;   // ESC sai da edição do nome
            else quitRequested = true;
        }
        // Painéis (calculados a cada quadro: a janela é redimensionável).
        const float screenW = static_cast<float>(GetScreenWidth());
        const bool narrow = screenW < 1320.0f;   // barra e painel da direita não cabem lado a lado
        const Rectangle controlBar{16.0f, 8.0f, narrow ? screenW - 32.0f : screenW - 408.0f, 36.0f};
        const Rectangle quatPanel{16.0f, 84.0f, 400.0f, 224.0f};
        const Rectangle interpPanel{screenW - 376.0f, narrow ? 52.0f : 12.0f, 360.0f, 214.0f};
        const Vector2 mouse = GetMousePosition();
        const float worldY = quatPanel.y + quatPanel.height + 62.0f;
        // Altura adaptativa: termina antes da faixa de texto de baixo (96 px), com no mínimo 180 px.
        const float worldH = std::min(276.0f, std::max(180.0f, static_cast<float>(GetScreenHeight()) - 96.0f - worldY));
        const Rectangle worldPanel{quatPanel.x, worldY, quatPanel.width, worldH};
        const Rectangle objPanel{screenW - 376.0f, interpPanel.y + interpPanel.height + 8.0f, 360.0f, mainTab >= 1 ? 344.0f : 306.0f};
        const bool overGui = CheckCollisionPointRec(mouse, quatPanel) || CheckCollisionPointRec(mouse, interpPanel) ||
                             CheckCollisionPointRec(mouse, controlBar) || CheckCollisionPointRec(mouse, objPanel) ||
                             (showWorld && CheckCollisionPointRec(mouse, worldPanel));

        // --- Entrada: câmera ------------------------------------------------------------------------------
        // Gira ao redor do alvo com o botão direito OU com a roda do mouse pressionada.
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            const Vector2 d = GetMouseDelta();
            orbit.rotate(-d.x * 0.005f, d.y * 0.005f);
        }
        const float wheel = GetMouseWheelMove();
        if (drag.active()) drag.addDepth(static_cast<physics::Real>(wheel) * physics::Real(0.5));   // arrastando: roda = profundidade
        else if (!overGui) orbit.zoom(wheel);
        if (keyPressed(KEY_I))
            mode = static_cast<viewer::InterpMode>((static_cast<int>(mode) + 1) % viewer::kInterpModeCount);
        if (keyPressed(KEY_T) || uiPivot) pivotSelected = !pivotSelected;
        if (keyPressed(KEY_M)) showWorld = !showWorld;
        if (keyPressed(KEY_F11)) ToggleBorderlessWindowed();   // tela cheia sem borda (e volta)

        // Troca de cena: ←/→ (com volta) ou 1–9; R recria a atual (reinicia).
        std::size_t wanted = sceneIndex;
        if (keyPressed(KEY_RIGHT) || uiSceneDelta > 0) wanted = (sceneIndex + 1) % sceneCount;
        if (keyPressed(KEY_LEFT) || uiSceneDelta < 0) wanted = (sceneIndex + sceneCount - 1) % sceneCount;
        for (int k = 0; k < 9; ++k)
            if (keyPressed(KEY_ONE + k) && static_cast<std::size_t>(k) < sceneCount) wanted = static_cast<std::size_t>(k);
        if (!loadRequest.empty()) {   // aba Cenas: carregar um arquivo
            viewer::Scene loaded;
            std::string err;
            const std::string path = sceneDir + "/" + loadRequest + viewer::scenefile::kExtension;
            if (viewer::loadSceneFile(path, loaded, err)) {
                scene = std::move(loaded);
                customText = viewer::serializeScene(scene);
                customActive = true;
                tracker.reset();
                scene.world->setContactListener(&tracker);
                sceneDefaults = viewer::captureWorld(*scene.world);
                scene.world->setProfilingEnabled(profilingOn);
                selected = physics::kInvalidBodyId;
                drag.end();
                timeCtl.onSceneReset();
                timeCtl.setPaused(startPaused);
                orbit.target = scene.cameraTarget;
                orbit.setDistance(scene.cameraDistance);
                fileMsg = "carregado: " + loadRequest + " (pausada)";
                fileMsgErr = false;
            } else {
                fileMsg = "ERRO: " + err;
                fileMsgErr = true;
            }
            fileMsgFrames = 600;
            loadRequest.clear();
        }
        const bool changed = wanted != sceneIndex;
        if (changed || keyPressed(KEY_R) || uiRestart) {
            sceneIndex = wanted;
            bool fromFile = false;
            if (!changed && customActive) {   // R / Reiniciar numa cena carregada: recria a partir do arquivo
                std::string err;
                fromFile = viewer::parseScene(customText, scene, err);
            }
            if (!fromFile) {
                customActive = false;
                scene = viewer::makeScene(sceneIndex);
            }
            tracker.reset();
            scene.world->setContactListener(&tracker);
            sceneDefaults = viewer::captureWorld(*scene.world);
            scene.world->setProfilingEnabled(profilingOn);
            selected = physics::kInvalidBodyId;   // os ids são de outro mundo
            drag.end();
            timeCtl.onSceneReset();
            timeCtl.setPaused(startPaused);   // trocar ou reiniciar a cena: volta pausada
            if (changed) {   // só ao trocar: R mantém a câmera que o usuário ajustou
                orbit.target = scene.cameraTarget;
                orbit.setDistance(scene.cameraDistance);
            }
        }
        if (keyPressed(KEY_HOME) || uiResetCam) {   // câmera: enquadramento inicial, alvo no centro do mundo
            orbit.reset(scene.cameraTarget, scene.cameraDistance);
            pivotSelected = false;
        }
        if (keyPressed(KEY_P) || keyPressed(KEY_SPACE) || uiTogglePause) timeCtl.togglePause();
        if (keyPressed(KEY_PERIOD) || uiStep) {   // um passo: se estava rodando, pausa primeiro
            timeCtl.setPaused(true);
            timeCtl.requestStep();
        }
        uiRestart = uiStep = uiTogglePause = uiResetCam = uiPivot = false;
        uiSceneDelta = 0;
        if (selected != physics::kInvalidBodyId && !scene.world->isValidBody(selected)) {
            selected = physics::kInvalidBodyId;
            drag.end();
        }

        // Alvo da câmera: centro do mundo (da cena) ou o corpo selecionado; a transição é suave.
        {
            viewer::Vec3f goal = scene.cameraTarget;
            if (pivotSelected && selected != physics::kInvalidBodyId) goal = viewer::toVec3f(renderPose(selected).position);
            orbit.followTarget(goal, GetFrameTime());
        }

        Camera3D cam{};
        cam.position = toRay(orbit.position());
        cam.target = toRay(orbit.target);
        cam.up = Vector3{0, 1, 0};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;

        // --- Entrada: mouse na cena (a câmera deste quadro já existe, então o raio é o que se vê) --------------------
        const Ray mouseRay = GetScreenToWorldRay(mouse, cam);
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
            sp.kind = static_cast<viewer::SpawnKind>(kindIdx);
            sp.size = sizeF;
            sp.mass = massS * massS;
            sp.friction = fricF;
            sp.restitution = restF;
            sp.position = physics::Vector3(spPos[0], spPos[1], spPos[2]);
            sp.orientation = spOrient;
            sp.velocity = physics::Vector3(spVel[0], spVel[1], spVel[2]);
            sp.angularVelocity = physics::Vector3(spAngV[0], spAngV[1], spAngV[2]);
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
            const viewer::PickResult pick = viewer::pickBody(scene, rayO, rayD);
            selected = pick.hit ? pick.body : physics::kInvalidBodyId;   // clicar no vazio ou no chão desmarca
            if (pick.hit) drag.begin(scene, pick);
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) || !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) drag.end();
        if (keyPressed(KEY_B) || keyPressed(KEY_O)) {   // lança uma caixa (B) ou esfera (O) na direção do mouse
            viewer::launchBody(scene, keyPressed(KEY_O) ? viewer::ShapeKind::Sphere : viewer::ShapeKind::Box,
                               rayO + rayD * physics::Real(1.5), rayD, physics::Real(18));
        }
        if ((keyPressed(KEY_DELETE) || keyPressed(KEY_X)) && selected != physics::kInvalidBodyId) {
            drag.end();
            viewer::deleteBody(scene, selected);
            selected = physics::kInvalidBodyId;
        }

        // --- Física: N passos fixos conforme o tempo de render ------------------------------------------------
        // A mola só atua com a física rodando (a força é consumida no passo, e pausado não há passo).
        if (!timeCtl.paused() && drag.active()) drag.apply(scene, rayO, rayD,
                                                      static_cast<physics::Real>(GetFrameTime()) * static_cast<physics::Real>(timeCtl.timeScale()));   // tempo FÍSICO do quadro (câmera lenta incluída)
        timeCtl.advance(*scene.world, static_cast<physics::Real>(GetFrameTime()));
        if (!timeCtl.paused()) tracker.update(GetFrameTime() * timeCtl.timeScale());   // flashes decaem em tempo físico

        // --- Desenho ----------------------------------------------------------------------------------------------
        BeginDrawing();
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
            else DrawCubeWires(c, sizeF, sizeF, sizeF, YELLOW);
        }
        if (drag.active()) {   // a mola: do ponto agarrado até onde o mouse aponta
            DrawLine3D(Vector3{float(drag.anchorWorld().x), float(drag.anchorWorld().y), float(drag.anchorWorld().z)},
                       Vector3{float(drag.targetWorld().x), float(drag.targetWorld().y), float(drag.targetWorld().z)}, ORANGE);
            DrawSphere(Vector3{float(drag.targetWorld().x), float(drag.targetWorld().y), float(drag.targetWorld().z)}, 0.08f, ORANGE);
        }
        if (hasFocus) drawLocalAxes(focusMatrix, 1.2f);
        EndMode3D();

        // --- HUD ------------------------------------------------------------------------------------------------------
        hudText(customActive ? TextFormat("[arquivo] %s   (%d corpos)", scene.name.c_str(), static_cast<int>(scene.bodies.size()))
                             : TextFormat("[%d/%d] %s   (%d corpos)", static_cast<int>(sceneIndex) + 1, static_cast<int>(sceneCount),
                                          scene.name.c_str(), static_cast<int>(scene.bodies.size())),
                 16, 52, 22, RAYWHITE);
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
            if (GuiButton(place(96.0f), timeCtl.paused() ? "Continuar" : "Pausar")) uiTogglePause = true;
            if (GuiButton(place(60.0f), "Passo")) uiStep = true;
            if (GuiButton(place(84.0f), "Reiniciar")) uiRestart = true;
            GuiLabel(place(34.0f), "Vel.");
            int sc = timeCtl.scaleIndex();
            GuiToggleGroup(Rectangle{bx, by, 42.0f, bh}, "1x;1/2;1/4;1/10", &sc);
            bx += 4.0f * 42.0f + 12.0f;
            timeCtl.setScaleIndex(sc);
            if (GuiButton(place(26.0f), "<")) uiSceneDelta = -1;
            GuiLabel(place(84.0f), customActive ? "arquivo" : TextFormat("cena %d/%d", static_cast<int>(sceneIndex) + 1, static_cast<int>(sceneCount)));
            if (GuiButton(place(26.0f), ">")) uiSceneDelta = 1;
            if (GuiButton(place(92.0f), "Reset cam")) uiResetCam = true;
            if (GuiButton(place(150.0f), pivotSelected ? "Alvo: selecionado" : "Alvo: mundo")) uiPivot = true;
        }

        // Painel do quatérnio do corpo em foco (a pose que está sendo DESENHADA, no modo escolhido).
        GuiPanel(quatPanel, selected != physics::kInvalidBodyId ? "Quaternio do corpo selecionado" : "Quaternio do corpo em foco");
        if (hasFocus) {
            const physics::Quaternion& q = focusPose.rotation;
            const physics::Real qn = q.norm();
            const Color txt = kAccent;
            const float x0 = quatPanel.x + 12.0f;
            float y0 = quatPanel.y + 34.0f;
            DrawText(TextFormat("x %+.4f   y %+.4f", static_cast<double>(q.x), static_cast<double>(q.y)), static_cast<int>(x0),
                     static_cast<int>(y0), 20, txt);
            y0 += 24.0f;
            DrawText(TextFormat("z %+.4f   w %+.4f", static_cast<double>(q.z), static_cast<double>(q.w)), static_cast<int>(x0),
                     static_cast<int>(y0), 20, txt);
            y0 += 24.0f;
            // |q| = 1 sempre, exceto no modo "lerp sem normalizar" entre dois passos.
            const bool unit = std::fabs(static_cast<double>(qn) - 1.0) < 1e-4;
            DrawText(TextFormat("|q| = %.6f%s", static_cast<double>(qn), unit ? "" : "   (NAO unitario)"),
                     static_cast<int>(x0), static_cast<int>(y0), 20, unit ? txt : kWarn);
            y0 += 28.0f;
            const viewer::AxisAngleF aa = viewer::toAxisAngle(q);
            DrawText(TextFormat("eixo (%+.3f %+.3f %+.3f)", static_cast<double>(aa.axis.x), static_cast<double>(aa.axis.y),
                                static_cast<double>(aa.axis.z)),
                     static_cast<int>(x0), static_cast<int>(y0), 18, kInkDim);
            y0 += 22.0f;
            DrawText(TextFormat("angulo %.1f graus (%.3f rad)", static_cast<double>(aa.angle) * 57.29577951308232,
                                static_cast<double>(aa.angle)),
                     static_cast<int>(x0), static_cast<int>(y0), 18, kInkDim);
            y0 += 26.0f;
            const viewer::EulerAngles eu = viewer::toEuler(q);
            const double rad2deg = 57.29577951308232;
            const bool lock = viewer::nearGimbalLock(eu, physics::Real(0.087));   // ~5 graus
            DrawText(TextFormat("Euler  yaw %+.1f  pitch %+.1f  roll %+.1f", static_cast<double>(eu.yaw) * rad2deg,
                                static_cast<double>(eu.pitch) * rad2deg, static_cast<double>(eu.roll) * rad2deg),
                     static_cast<int>(x0), static_cast<int>(y0), 18, lock ? kWarn : kInkDim);
            // Velocidade angular do corpo (a que o motor integra no quatérnio: dq/dt = 1/2 w q).
            if (scene.world->isValidBody(infoBody)) {
                const physics::Vector3 w = scene.world->getBody(infoBody).getAngularVelocity();
                DrawText(TextFormat("w (%+.2f %+.2f %+.2f)  |w| %.2f rad/s", static_cast<double>(w.x), static_cast<double>(w.y),
                                    static_cast<double>(w.z), static_cast<double>(w.length())),
                         static_cast<int>(x0), static_cast<int>(y0) + 24, 16, kBlueInk);
            }
            if (lock) DrawText("GIMBAL LOCK: yaw e roll giram o mesmo eixo", static_cast<int>(x0), static_cast<int>(y0) + 46, 16, kWarn);
        } else {
            DrawText("(esta cena nao tem corpo em foco)", static_cast<int>(quatPanel.x) + 12, static_cast<int>(quatPanel.y) + 40, 18, kInkDim);
        }

        // Painel da interpolação de render e do FPS.
        GuiPanel(interpPanel, "Interpolacao de render");
        {
            const float x0 = interpPanel.x + 12.0f, w = interpPanel.width - 24.0f;
            int active = static_cast<int>(mode);
            GuiToggleGroup(Rectangle{x0, interpPanel.y + 34.0f, w / 3.0f, 30.0f}, "slerp;lerp;Euler", &active);
            mode = static_cast<viewer::InterpMode>(active);
            DrawText(viewer::interpModeName(mode), static_cast<int>(x0), static_cast<int>(interpPanel.y) + 72, 18, kInk);
            const char* help = "";
            if (mode == viewer::InterpMode::Slerp) help = "esferica: giro constante, |q| = 1";
            if (mode == viewer::InterpMode::LerpNoNormalize) help = "reta entre q0 e q1: |q| < 1, o\ncubo encolhe e muda de ritmo";
            if (mode == viewer::InterpMode::Euler) help = "interpola yaw/pitch/roll: caminho\ntorto; pula no gimbal lock";
            DrawText(help, static_cast<int>(x0), static_cast<int>(interpPanel.y) + 96, 16, kInk);
            DrawText(TextFormat("FPS de render: %d", fpsApplied), static_cast<int>(x0), static_cast<int>(interpPanel.y) + 144, 18, kInk);
            GuiSlider(Rectangle{x0 + 24.0f, interpPanel.y + 168.0f, w - 64.0f, 22.0f}, "10", "240", &fpsSlider, 10.0f, 240.0f);
            const int wantedFps = static_cast<int>(std::lround(static_cast<double>(fpsSlider)));
            if (wantedFps != fpsApplied) {
                fpsApplied = wantedFps;
                SetTargetFPS(fpsApplied);
            }
        }
        // --- Painel do objeto selecionado (V6) --------------------------------------------------------------
        {
            physics::PhysicsWorld& world = *scene.world;
            const bool selValid = world.isValidBody(selected);
            if (selValid && selected != lastSelected) {   // rascunho da orientação parte da orientação atual do corpo
                lastSelected = selected;
                const viewer::AxisAngleF aa0 = viewer::toAxisAngle(world.getBody(selected).getOrientation());
                oAxisX = aa0.axis.x; oAxisY = aa0.axis.y; oAxisZ = aa0.axis.z;
                oAngleDeg = aa0.angle * 57.29578f;
            }
            if (addFailFrames > 0) --addFailFrames;
            GuiPanel(objPanel, addFailFrames > 0 ? "Objetos - NAO foi possivel criar"
                               : (mainTab == 0 && selValid ? TextFormat("Objeto selecionado (corpo %u)", static_cast<unsigned>(selected))
                                                           : "Objetos"));
            const float ox = objPanel.x + 12.0f, ow = objPanel.width - 24.0f;
            float oy = objPanel.y + 32.0f;
            GuiToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "Objeto;Adicionar;Cenas", &mainTab);
            oy += 36.0f;
            const float labelW = 124.0f, valueW = 88.0f;
            const float barW = ow - labelW - valueW - 8.0f;
            auto sliderRow = [&](const char* name, float& v, float lo, float hi, const char* valueText) {
                const Rectangle bar{ox + labelW, oy, barW, 20.0f};
                const float before = v;
                DrawText(name, static_cast<int>(ox), static_cast<int>(oy) + 2, 16, kInk);
                GuiSlider(bar, nullptr, nullptr, &v, lo, hi);
                DrawText(valueText, static_cast<int>(ox + labelW + barW) + 6, static_cast<int>(oy) + 2, 16, kInk);
                oy += 26.0f;
                return v != before && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, bar);
            };
            using physics::Real;
            if (mainTab == 2) {   // ---- Cenas: salvar e carregar (V12) ----
                DrawText("nome do arquivo (sem extensao):", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                oy += 18.0f;
                if (GuiTextBox(Rectangle{ox, oy, ow, 26.0f}, fileName, 48, fileNameEdit)) fileNameEdit = !fileNameEdit;
                oy += 32.0f;
                if (GuiButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Salvar cena atual")) {
                    const std::string typed = fileName;
                    const std::string base = viewer::sanitizeSceneFileName(typed);
                    const std::string path = sceneDir + "/" + base + viewer::scenefile::kExtension;
                    const bool existed = std::find(sceneFiles.begin(), sceneFiles.end(), base) != sceneFiles.end();
                    if (!typed.empty()) scene.name = typed;   // o nome digitado vira o nome da cena
                    std::string err;
                    if (viewer::saveSceneFile(scene, path, err)) {
                        fileMsg = "salvo: " + base + viewer::scenefile::kExtension + (existed ? " (sobrescreveu)" : "");
                        fileMsgErr = false;
                        sceneFiles = viewer::listSceneFiles(sceneDir);
                    } else {
                        fileMsg = "ERRO: " + err;
                        fileMsgErr = true;
                    }
                    fileMsgFrames = 600;
                    fileNameEdit = false;
                }
                if (GuiButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Atualizar lista")) sceneFiles = viewer::listSceneFiles(sceneDir);
                oy += 36.0f;
                DrawText(TextFormat("arquivos em scenes/ (%d):", static_cast<int>(sceneFiles.size())), static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
                oy += 18.0f;
                std::string joined;
                for (std::size_t i = 0; i < sceneFiles.size(); ++i) {
                    std::string n = sceneFiles[i];
                    std::replace(n.begin(), n.end(), ';', '_');   // ';' separa os itens da lista do raygui
                    joined += (i ? ";" : "") + n;
                }
                GuiListView(Rectangle{ox, oy, ow, 104.0f}, joined.empty() ? "(nenhum arquivo)" : joined.c_str(), &fileScroll, &fileActive);
                oy += 110.0f;
                if (GuiButton(Rectangle{ox, oy, ow, 28.0f}, "Carregar o arquivo escolhido") &&
                    fileActive >= 0 && fileActive < static_cast<int>(sceneFiles.size()))
                    loadRequest = sceneFiles[static_cast<std::size_t>(fileActive)];
                oy += 32.0f;
                if (fileMsgFrames > 0) {
                    --fileMsgFrames;
                    DrawText(fileMsg.size() > 40 ? (fileMsg.substr(0, 38) + "..").c_str() : fileMsg.c_str(), static_cast<int>(ox),
                             static_cast<int>(oy), 14, fileMsgErr ? kWarn : kBlueInk);
                }
            } else if (mainTab == 1) {   // ---- Adicionar objeto (V8) ----
                GuiToggleGroup(Rectangle{ox, oy, ow / 4.0f, 26.0f}, "Forma;Posicao;Orient;Veloc", &addTab);
                oy += 34.0f;
                if (addTab == 0) {   // forma, tamanho, massa, material
                    const int orig = kindIdx;
                    int ka = orig < 3 ? orig : -1, kb = orig >= 3 ? orig - 3 : -1;
                    GuiToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "caixa;esfera;tetraedro", &ka);
                    oy += 28.0f;
                    GuiToggleGroup(Rectangle{ox, oy, ow / 3.0f, 26.0f}, "octaedro;prisma;piramide", &kb);
                    oy += 32.0f;
                    if (ka >= 0 && ka != (orig < 3 ? orig : -99)) kindIdx = ka;
                    else if (kb >= 0 && kb != (orig >= 3 ? orig - 3 : -99)) kindIdx = kb + 3;
                    sliderRow("tamanho", sizeF, 0.2f, 4.0f, TextFormat("%.2f", static_cast<double>(sizeF)));
                    sliderRow("massa", massS, 0.2236f, 10.0f, TextFormat("%.2f kg", static_cast<double>(massS * massS)));
                    sliderRow("atrito", fricF, 0.0f, 2.0f, TextFormat("%.2f", static_cast<double>(fricF)));
                    sliderRow("restituicao", restF, 0.0f, 1.0f, TextFormat("%.2f", static_cast<double>(restF)));
                } else if (addTab == 1) {   // posição e velocidade de lançamento
                    sliderRow("x", spPos[0], -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(spPos[0])));
                    sliderRow("y", spPos[1], -2.0f, 20.0f, TextFormat("%+.2f m", static_cast<double>(spPos[1])));
                    sliderRow("z", spPos[2], -12.0f, 12.0f, TextFormat("%+.2f m", static_cast<double>(spPos[2])));
                    sliderRow("veloc. lanc.", launchSpd, 1.0f, 40.0f, TextFormat("%.1f m/s", static_cast<double>(launchSpd)));
                    DrawText("lancar: sai 2 m a frente da camera", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                } else if (addTab == 2) {   // orientação inicial: eixo-ângulo OU quatérnio
                    GuiToggleGroup(Rectangle{ox, oy, ow / 2.0f, 26.0f}, "Eixo-angulo;Quaternio", &qMode);
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
                        DrawText(TextFormat("q = (%+.3f, %+.3f, %+.3f, %+.3f)", static_cast<double>(spOrient.w), static_cast<double>(spOrient.x),
                                            static_cast<double>(spOrient.y), static_cast<double>(spOrient.z)),
                                 static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kInkDim);
                    else
                        DrawText("entrada nula: usa a identidade", static_cast<int>(ox), static_cast<int>(oy) + 2, 14, kWarn);
                } else {   // velocidades iniciais
                    const char* vn[3] = {"v x", "v y", "v z"};
                    const char* wn[3] = {"w x", "w y", "w z"};
                    for (int i = 0; i < 3; ++i) sliderRow(vn[i], spVel[i], -20.0f, 20.0f, TextFormat("%+.2f m/s", static_cast<double>(spVel[i])));
                    for (int i = 0; i < 3; ++i) sliderRow(wn[i], spAngV[i], -20.0f, 20.0f, TextFormat("%+.2f r/s", static_cast<double>(spAngV[i])));
                }
                // Rodapé fixo: posicionar com clique + botões Adicionar / Lançar.
                const float by = objPanel.y + objPanel.height - 62.0f;
                GuiCheckBox(Rectangle{ox, by, 20.0f, 20.0f}, " posicionar com clique (esquerdo)", &clickPlace);
                if (GuiButton(Rectangle{ox, by + 26.0f, ow / 2.0f - 4.0f, 28.0f}, "Adicionar")) afterSpawn(viewer::spawnBody(scene, makeSpawn()));
                if (GuiButton(Rectangle{ox + ow / 2.0f + 4.0f, by + 26.0f, ow / 2.0f - 4.0f, 28.0f}, "Lancar da camera")) {
                    const physics::Vector3 camPos(cam.position.x, cam.position.y, cam.position.z);
                    const physics::Vector3 camDir(cam.target.x - cam.position.x, cam.target.y - cam.position.y, cam.target.z - cam.position.z);
                    afterSpawn(viewer::launchFrom(scene, makeSpawn(), camPos, camDir, static_cast<physics::Real>(launchSpd)));
                }
            } else if (!selValid) {
                DrawText("Nenhum objeto selecionado.", static_cast<int>(ox), static_cast<int>(oy) + 4, 16, kInk);
                DrawText("Clique num corpo na cena, ou use a aba", static_cast<int>(ox), static_cast<int>(oy) + 32, 14, kInkDim);
                DrawText("Adicionar para criar um novo.", static_cast<int>(ox), static_cast<int>(oy) + 50, 14, kInkDim);
            } else {
            GuiToggleGroup(Rectangle{ox, oy, ow / 4.0f, 26.0f}, "Material;Posicao;Veloc;Orient", &objTab);
            oy += 36.0f;
            viewer::BodyParams bp;
            viewer::readParams(world, selected, bp);
            const physics::RigidBody& body = world.getBody(selected);
            if (objTab == 0) {   // Material
                float m = std::sqrt(static_cast<float>(bp.mass));   // escala √: mais precisão nas massas pequenas
                if (sliderRow("massa", m, 0.2236f, 10.0f, TextFormat("%.2f kg", static_cast<double>(m * m))))
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
                GuiCheckBox(Rectangle{ox, oy, 20.0f, 20.0f}, " pode dormir (sleeping)", &allow);
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
                DrawText(bp.awake ? "estado: acordado" : "estado: dormindo", static_cast<int>(ox), static_cast<int>(oy), 16, bp.awake ? kInkDim : kBlueInk);
                oy += 24.0f;
                if (GuiButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Acordar")) viewer::setAwake(world, selected, true);
                if (GuiButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Dormir")) viewer::setAwake(world, selected, false);
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
                if (GuiButton(Rectangle{ox, oy, ow, 28.0f}, "Zerar velocidades")) viewer::zeroVelocities(world, selected);
            } else {   // Orientação: eixo + ângulo -> quatérnio
                sliderRow("eixo x", oAxisX, -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(oAxisX)));
                sliderRow("eixo y", oAxisY, -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(oAxisY)));
                sliderRow("eixo z", oAxisZ, -1.0f, 1.0f, TextFormat("%+.2f", static_cast<double>(oAxisZ)));
                sliderRow("angulo", oAngleDeg, 0.0f, 360.0f, TextFormat("%.0f gr", static_cast<double>(oAngleDeg)));
                oy += 2.0f;
                if (GuiButton(Rectangle{ox, oy, ow / 2.0f - 4.0f, 28.0f}, "Aplicar q"))
                    viewer::setOrientationAxisAngle(world, selected, physics::Vector3(oAxisX, oAxisY, oAxisZ),
                                                    static_cast<Real>(oAngleDeg * 0.017453292519943295f));
                if (GuiButton(Rectangle{ox + ow / 2.0f + 4.0f, oy, ow / 2.0f - 4.0f, 28.0f}, "Identidade")) viewer::resetOrientation(world, selected);
                oy += 34.0f;
                DrawText("q = (cos(a/2), sen(a/2) * eixo unitario)", static_cast<int>(ox), static_cast<int>(oy), 14, kInkDim);
            }
            }
        }

        // --- Painel do mundo (V7) -----------------------------------------------------------------------------
        if (showWorld) {
            physics::PhysicsWorld& world = *scene.world;
            GuiPanel(worldPanel, "Mundo e debug (M esconde)");
            const float wx = worldPanel.x + 12.0f, ww = worldPanel.width - 24.0f;
            float wy = worldPanel.y + 30.0f;
            GuiToggleGroup(Rectangle{wx, wy, ww / 5.0f, 26.0f}, "Mundo;Solver;Debug;Stats;Evento", &worldTab);
            wy += 34.0f;
            const float labelW = 128.0f, valueW = 96.0f;
            const float barW = ww - labelW - valueW - 8.0f;
            // Uma linha de slider: devolve true se o usuário mexeu; o valor aparece à direita.
            auto row = [&](const char* name, float& v, float lo, float hi, const char* valueText) {
                DrawText(name, static_cast<int>(wx), static_cast<int>(wy) + 2, 16, kInk);
                const bool edited = sliderEdited(Rectangle{wx + labelW, wy, barW, 20.0f}, v, lo, hi);
                DrawText(valueText, static_cast<int>(wx + labelW + barW) + 6, static_cast<int>(wy) + 2, 16, kInk);
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
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " sleeping ligado", &sl);
                if (sl != wp.sleepEnabled) { wp.sleepEnabled = sl; changed = true; }
                wy += 26.0f;
                float st = static_cast<float>(wp.sleepTime);
                if (row("dormir apos", st, 0.05f, 10.0f, TextFormat("%.2f s", static_cast<double>(st)))) { wp.sleepTime = st; changed = true; }
            } else if (worldTab == 1) {   // Solver
                float vi = static_cast<float>(wp.velocityIterations), pi = static_cast<float>(wp.positionIterations);
                if (row("iter. veloc.", vi, 1.0f, 50.0f, TextFormat("%d", static_cast<int>(std::lround(static_cast<double>(vi)))))) { wp.velocityIterations = static_cast<int>(std::lround(static_cast<double>(vi))); changed = true; }
                if (row("iter. posicao", pi, 0.0f, 20.0f, TextFormat("%d%s", static_cast<int>(std::lround(static_cast<double>(pi))), pi < 0.5f ? " (Baumg.)" : ""))) { wp.positionIterations = static_cast<int>(std::lround(static_cast<double>(pi))); changed = true; }
                float ls = static_cast<float>(wp.linearSlop), vs = static_cast<float>(wp.velocitySlop);
                if (row("slop linear", ls, 0.0f, 0.05f, TextFormat("%.1f mm", static_cast<double>(ls) * 1000.0))) { wp.linearSlop = ls; changed = true; }
                if (row("slop veloc.", vs, 0.0f, 2.0f, TextFormat("%.2f m/s", static_cast<double>(vs)))) { wp.velocitySlop = vs; changed = true; }
                bool ws = wp.warmStarting;
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " warm starting", &ws);
                if (ws != wp.warmStarting) { wp.warmStarting = ws; changed = true; }
                wy += 26.0f;
                DrawText("menos iteracoes: mais barato, mais mole", static_cast<int>(wx), static_cast<int>(wy), 14, kInkDim);
            } else if (worldTab == 2) {   // Debug
                const std::vector<viewer::DebugFlagInfo>& table = viewer::debugFlagTable();
                const float colW = ww / 2.0f;
                for (std::size_t i = 0; i < table.size(); ++i) {
                    const float cx = wx + (i < 5 ? 0.0f : colW), cy = wy + static_cast<float>(i < 5 ? i : i - 5) * 24.0f;
                    bool on = (dbgFlags & table[i].flag) != 0;
                    GuiCheckBox(Rectangle{cx, cy, 20.0f, 20.0f}, TextFormat(" %s", table[i].label), &on);
                    if (on) dbgFlags |= table[i].flag; else dbgFlags &= ~table[i].flag;
                }
                wy += 5.0f * 24.0f + 6.0f;
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " esconder modelos (so o debug)", &hideModels);
                wy += 30.0f;
                if (GuiButton(Rectangle{wx, wy, ww / 2.0f - 4.0f, 26.0f}, "Padrao")) dbgFlags = static_cast<std::uint32_t>(physics::kDebugDefault);
                if (GuiButton(Rectangle{wx + ww / 2.0f + 4.0f, wy, ww / 2.0f - 4.0f, 26.0f}, "Desligar")) dbgFlags = 0;
            } else if (worldTab == 4) {   // Evento (V9)
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " cor por estado (dormindo = azul)", &tintOn);
                wy += 24.0f;
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " flash no inicio do contato", &flashOn);
                wy += 24.0f;
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " aneis nos pontos de impacto", &ringsOn);
                wy += 30.0f;
                const physics::WorldStats st = world.getStats();
                auto line = [&](const char* text) { DrawText(text, static_cast<int>(wx), static_cast<int>(wy), 16, kInk); wy += 19.0f; };
                line(TextFormat("contatos agora %d   dormindo %d de %d", static_cast<int>(world.getContacts().size()), st.sleepingBodies, st.awakeBodies + st.sleepingBodies));
                line(TextFormat("inicios %llu   fins %llu", static_cast<unsigned long long>(tracker.begins()), static_cast<unsigned long long>(tracker.ends())));
                line(TextFormat("impacto: ultimo %.1f   maximo %.1f m/s", static_cast<double>(tracker.lastImpact()), static_cast<double>(tracker.maxImpact())));
                if (GuiButton(Rectangle{wx, wy + 4.0f, ww, 24.0f}, "Zerar contadores")) tracker.reset();
            } else {   // Stats
                bool prof = profilingOn;
                GuiCheckBox(Rectangle{wx, wy, 20.0f, 20.0f}, " medir tempos (profiling)", &prof);
                if (prof != profilingOn) { profilingOn = prof; world.setProfilingEnabled(profilingOn); }
                wy += 28.0f;
                const physics::WorldStats st = world.getStats();
                auto line = [&](const char* text) { DrawText(text, static_cast<int>(wx), static_cast<int>(wy), 16, kInk); wy += 19.0f; };
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
                if (GuiButton(Rectangle{wx, wy, ww, 26.0f}, "Restaurar padroes da cena")) { wp = sceneDefaults; changed = true; }
            }
            if (changed) viewer::applyWorld(world, wp);
        }

        if (selected != physics::kInvalidBodyId) {
            const physics::RigidBody& sb = scene.world->getBody(selected);
            hudText(TextFormat("selecionado: corpo %u   massa %.2f kg   |v| = %.2f m/s   %s", static_cast<unsigned>(selected),
                                static_cast<double>(sb.getMass()), static_cast<double>(sb.getLinearVelocity().length()),
                                sb.isAwake() ? "acordado" : "dormindo"),
                     static_cast<int>(quatPanel.x), static_cast<int>(quatPanel.y + quatPanel.height) + 8, 18, Color{255, 235, 60, 255});
        }
        hudText(TextFormat("%d FPS   fisica %.0f Hz   passo #%llu   alpha = %.2f   tempo %s%s", GetFPS(),
                            1.0 / static_cast<double>(scene.world->getFixedTimeStep()),
                            static_cast<unsigned long long>(scene.world->getStats().stepCount),
                            static_cast<double>(timeCtl.renderAlpha(*scene.world)),
                            viewer::TimeControl::scaleLabel(timeCtl.scaleIndex()), timeCtl.paused() ? "   [PAUSADO]" : ""),
                 16, GetScreenHeight() - 80, 18, Color{235, 238, 245, 255});
        hudText("esq: selecionar/arrastar   dir ou roda apertada: girar   roda: zoom   B/O: lancar   Del/X: apagar", 16,
                 GetScreenHeight() - 54, 18, Color{200, 205, 215, 255});
        hudText("Espaco/P: pausa   . : passo   setas/1-9: cena   R: reiniciar   I: interpolacao   T: alvo   Home: camera   M: mundo   ESC: sair", 16,
                 GetScreenHeight() - 30, 18, Color{200, 205, 215, 255});
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
                    CloseWindow();
                    return 1;
                }
            }
            break;
        }
    }

    CloseWindow();
    return 0;
}
