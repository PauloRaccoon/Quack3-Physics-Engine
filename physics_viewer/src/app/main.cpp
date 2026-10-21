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
// Controles: Espaço ou P = pausar/continuar; . (ponto) = um passo (pausa se estiver rodando);
//            esquerdo = selecionar/arrastar (roda = profundidade durante o arrasto); direito ou
//            roda pressionada + mover = girar a câmera; roda = zoom; T = alvo da câmera;
//            Home = resetar câmera; B/O = lançar caixa/esfera; Delete/X = apagar;
//            ←/→ ou 1–9 = trocar de cena; R = reiniciar a cena; I = modo de interpolação;
//            ESC = sair.
//
// Verificação automática (sem interação):
//     physics_viewer --frames 240 --screenshot saida.png
// roda 240 quadros, salva uma captura da tela e sai com código 0.
// =============================================================================
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
#include "viewer/core/Interaction.h"
#include "viewer/core/OrbitCamera.h"
#include "viewer/core/RenderPose.h"
#include "viewer/core/TimeControl.h"
#include "viewer/core/Scene.h"
#include "viewer/core/ViewerMath.h"

namespace {

constexpr int kScreenW = 1280;
constexpr int kScreenH = 720;

Vector3 toRay(const viewer::Vec3f& v) { return Vector3{v.x, v.y, v.z}; }

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
    InitWindow(kScreenW, kScreenH, "physics_viewer — V6");
    int fpsApplied = 120;
    float fpsSlider = 120.0f;
    SetTargetFPS(fpsApplied);   // o render NÃO precisa casar com a taxa da física: há interpolação
    GuiSetStyle(DEFAULT, TEXT_SIZE, 16);
    viewer::InterpMode mode = viewer::InterpMode::Slerp;

    viewer::OrbitCamera orbit;
    const std::size_t sceneCount = viewer::sceneList().size();
    std::size_t sceneIndex = 0;
    viewer::Scene scene = viewer::makeScene(sceneIndex);
    orbit.target = scene.cameraTarget;
    orbit.setDistance(scene.cameraDistance);
    viewer::TimeControl timeCtl;
    // Pedidos da barra de botões: o raygui devolve o clique durante o DESENHO, então viram flags
    // tratadas no início do quadro seguinte.
    bool uiRestart = false, uiStep = false, uiTogglePause = false, uiResetCam = false, uiPivot = false;
    int uiSceneDelta = 0;
    int frame = 0;
    physics::BodyId selected = physics::kInvalidBodyId;   // corpo selecionado (clique)
    viewer::SpringDrag drag;
    bool pivotSelected = false;                             // alvo da câmera: false = centro do mundo
    // Painel do objeto (V6): aba atual e o rascunho da orientação (eixo + ângulo), que só vai ao corpo em "Aplicar".
    int objTab = 0;
    physics::BodyId lastSelected = physics::kInvalidBodyId;
    float oAxisX = 0.0f, oAxisY = 1.0f, oAxisZ = 0.0f, oAngleDeg = 0.0f;

    // Pose de RENDER de um corpo: interpola os dois últimos passos físicos com o α atual, no modo escolhido.
    auto renderPose = [&](physics::BodyId id) {
        const physics::MotionState& ms = scene.world->getMotionState(id);
        return viewer::interpolatePose(ms.getPrevious(), ms.getCurrent(), timeCtl.renderAlpha(*scene.world), mode);
    };

    while (!WindowShouldClose()) {
        // Painéis (calculados a cada quadro: a janela é redimensionável).
        const float screenW = static_cast<float>(GetScreenWidth());
        const bool narrow = screenW < 1220.0f;   // barra e painel da direita não cabem lado a lado
        const Rectangle controlBar{16.0f, 8.0f, narrow ? screenW - 32.0f : screenW - 368.0f, 36.0f};
        const Rectangle quatPanel{16.0f, 84.0f, 400.0f, 196.0f};
        const Rectangle interpPanel{screenW - 336.0f, narrow ? 52.0f : 12.0f, 320.0f, 214.0f};
        const Vector2 mouse = GetMousePosition();
        const Rectangle objPanel{screenW - 336.0f, interpPanel.y + interpPanel.height + 8.0f, 320.0f, 270.0f};
        const bool objPanelShown = selected != physics::kInvalidBodyId;
        const bool overGui = CheckCollisionPointRec(mouse, quatPanel) || CheckCollisionPointRec(mouse, interpPanel) ||
                             CheckCollisionPointRec(mouse, controlBar) || (objPanelShown && CheckCollisionPointRec(mouse, objPanel));

        // --- Entrada: câmera ------------------------------------------------------------------------------
        // Gira ao redor do alvo com o botão direito OU com a roda do mouse pressionada.
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            const Vector2 d = GetMouseDelta();
            orbit.rotate(-d.x * 0.005f, d.y * 0.005f);
        }
        const float wheel = GetMouseWheelMove();
        if (drag.active()) drag.addDepth(static_cast<physics::Real>(wheel) * physics::Real(0.5));   // arrastando: roda = profundidade
        else if (!overGui) orbit.zoom(wheel);
        if (IsKeyPressed(KEY_I))
            mode = static_cast<viewer::InterpMode>((static_cast<int>(mode) + 1) % viewer::kInterpModeCount);
        if (IsKeyPressed(KEY_T) || uiPivot) pivotSelected = !pivotSelected;

        // Troca de cena: ←/→ (com volta) ou 1–9; R recria a atual (reinicia).
        std::size_t wanted = sceneIndex;
        if (IsKeyPressed(KEY_RIGHT) || uiSceneDelta > 0) wanted = (sceneIndex + 1) % sceneCount;
        if (IsKeyPressed(KEY_LEFT) || uiSceneDelta < 0) wanted = (sceneIndex + sceneCount - 1) % sceneCount;
        for (int k = 0; k < 9; ++k)
            if (IsKeyPressed(KEY_ONE + k) && static_cast<std::size_t>(k) < sceneCount) wanted = static_cast<std::size_t>(k);
        const bool changed = wanted != sceneIndex;
        if (changed || IsKeyPressed(KEY_R) || uiRestart) {
            sceneIndex = wanted;
            scene = viewer::makeScene(sceneIndex);
            selected = physics::kInvalidBodyId;   // os ids são de outro mundo
            drag.end();
            timeCtl.onSceneReset();
            if (changed) {   // só ao trocar: R mantém a câmera que o usuário ajustou
                orbit.target = scene.cameraTarget;
                orbit.setDistance(scene.cameraDistance);
            }
        }
        if (IsKeyPressed(KEY_HOME) || uiResetCam) {   // câmera: enquadramento inicial, alvo no centro do mundo
            orbit.reset(scene.cameraTarget, scene.cameraDistance);
            pivotSelected = false;
        }
        if (IsKeyPressed(KEY_P) || IsKeyPressed(KEY_SPACE) || uiTogglePause) timeCtl.togglePause();
        if (IsKeyPressed(KEY_PERIOD) || uiStep) {   // um passo: se estava rodando, pausa primeiro
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
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !overGui) {
            const viewer::PickResult pick = viewer::pickBody(scene, rayO, rayD);
            selected = pick.hit ? pick.body : physics::kInvalidBodyId;   // clicar no vazio ou no chão desmarca
            if (pick.hit) drag.begin(scene, pick);
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) || !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) drag.end();
        if (IsKeyPressed(KEY_B) || IsKeyPressed(KEY_O)) {   // lança uma caixa (B) ou esfera (O) na direção do mouse
            viewer::launchBody(scene, IsKeyPressed(KEY_O) ? viewer::ShapeKind::Sphere : viewer::ShapeKind::Box,
                               rayO + rayD * physics::Real(1.5), rayD, physics::Real(18));
        }
        if ((IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_X)) && selected != physics::kInvalidBodyId) {
            drag.end();
            viewer::deleteBody(scene, selected);
            selected = physics::kInvalidBodyId;
        }

        // --- Física: N passos fixos conforme o tempo de render ------------------------------------------------
        // A mola só atua com a física rodando (a força é consumida no passo, e pausado não há passo).
        if (!timeCtl.paused() && drag.active()) drag.apply(scene, rayO, rayD,
                                                      static_cast<physics::Real>(GetFrameTime()) * static_cast<physics::Real>(timeCtl.timeScale()));   // tempo FÍSICO do quadro (câmera lenta incluída)
        timeCtl.advance(*scene.world, static_cast<physics::Real>(GetFrameTime()));

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
            drawBody(body, m);
            if (body.id == selected) drawHighlight(body, m);
            if (body.id == infoBody) {
                focusMatrix = m;
                focusPose = pose;
                hasFocus = true;
            }
        }
        for (const viewer::SceneLink& link : scene.links) {   // juntas: linha entre os centros
            const physics::Vector3 pa = renderPose(link.a).position;
            const physics::Vector3 pb = renderPose(link.b).position;
            DrawLine3D(Vector3{float(pa.x), float(pa.y), float(pa.z)}, Vector3{float(pb.x), float(pb.y), float(pb.z)},
                       Color{235, 235, 235, 255});
        }
        if (drag.active()) {   // a mola: do ponto agarrado até onde o mouse aponta
            DrawLine3D(Vector3{float(drag.anchorWorld().x), float(drag.anchorWorld().y), float(drag.anchorWorld().z)},
                       Vector3{float(drag.targetWorld().x), float(drag.targetWorld().y), float(drag.targetWorld().z)}, ORANGE);
            DrawSphere(Vector3{float(drag.targetWorld().x), float(drag.targetWorld().y), float(drag.targetWorld().z)}, 0.08f, ORANGE);
        }
        if (hasFocus) drawLocalAxes(focusMatrix, 1.2f);
        EndMode3D();

        // --- HUD ------------------------------------------------------------------------------------------------------
        DrawText(TextFormat("[%d/%d] %s   (%d corpos)", static_cast<int>(sceneIndex) + 1,
                            static_cast<int>(sceneCount), scene.name.c_str(),
                            static_cast<int>(scene.bodies.size())),
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
            GuiLabel(place(64.0f), TextFormat("cena %d/%d", static_cast<int>(sceneIndex) + 1, static_cast<int>(sceneCount)));
            if (GuiButton(place(26.0f), ">")) uiSceneDelta = 1;
            if (GuiButton(place(92.0f), "Reset cam")) uiResetCam = true;
            if (GuiButton(place(130.0f), pivotSelected ? "Alvo: selecionado" : "Alvo: mundo")) uiPivot = true;
        }

        // Painel do quatérnio do corpo em foco (a pose que está sendo DESENHADA, no modo escolhido).
        GuiPanel(quatPanel, selected != physics::kInvalidBodyId ? "Quaternio do corpo selecionado" : "Quaternio do corpo em foco");
        if (hasFocus) {
            const physics::Quaternion& q = focusPose.rotation;
            const physics::Real qn = q.norm();
            const Color txt{240, 190, 110, 255};
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
                     static_cast<int>(x0), static_cast<int>(y0), 20, unit ? txt : Color{255, 110, 110, 255});
            y0 += 28.0f;
            const viewer::AxisAngleF aa = viewer::toAxisAngle(q);
            DrawText(TextFormat("eixo (%+.3f %+.3f %+.3f)", static_cast<double>(aa.axis.x), static_cast<double>(aa.axis.y),
                                static_cast<double>(aa.axis.z)),
                     static_cast<int>(x0), static_cast<int>(y0), 18, LIGHTGRAY);
            y0 += 22.0f;
            DrawText(TextFormat("angulo %.1f graus (%.3f rad)", static_cast<double>(aa.angle) * 57.29577951308232,
                                static_cast<double>(aa.angle)),
                     static_cast<int>(x0), static_cast<int>(y0), 18, LIGHTGRAY);
            y0 += 26.0f;
            const viewer::EulerAngles eu = viewer::toEuler(q);
            const double rad2deg = 57.29577951308232;
            const bool lock = viewer::nearGimbalLock(eu, physics::Real(0.087));   // ~5 graus
            DrawText(TextFormat("Euler  yaw %+.1f  pitch %+.1f  roll %+.1f", static_cast<double>(eu.yaw) * rad2deg,
                                static_cast<double>(eu.pitch) * rad2deg, static_cast<double>(eu.roll) * rad2deg),
                     static_cast<int>(x0), static_cast<int>(y0), 18, lock ? Color{255, 110, 110, 255} : LIGHTGRAY);
            if (lock) DrawText("GIMBAL LOCK: yaw e roll giram o mesmo eixo", static_cast<int>(x0), static_cast<int>(y0) + 22, 16, Color{255, 110, 110, 255});
        } else {
            DrawText("(esta cena nao tem corpo em foco)", static_cast<int>(quatPanel.x) + 12, static_cast<int>(quatPanel.y) + 40, 18, GRAY);
        }

        // Painel da interpolação de render e do FPS.
        GuiPanel(interpPanel, "Interpolacao de render");
        {
            const float x0 = interpPanel.x + 12.0f, w = interpPanel.width - 24.0f;
            int active = static_cast<int>(mode);
            GuiToggleGroup(Rectangle{x0, interpPanel.y + 34.0f, w / 3.0f, 30.0f}, "slerp;lerp;Euler", &active);
            mode = static_cast<viewer::InterpMode>(active);
            DrawText(viewer::interpModeName(mode), static_cast<int>(x0), static_cast<int>(interpPanel.y) + 72, 18, RAYWHITE);
            const char* help = "";
            if (mode == viewer::InterpMode::Slerp) help = "esferica: giro constante, |q| = 1";
            if (mode == viewer::InterpMode::LerpNoNormalize) help = "reta entre q0 e q1: |q| < 1, o\ncubo encolhe e muda de ritmo";
            if (mode == viewer::InterpMode::Euler) help = "interpola yaw/pitch/roll: caminho\ntorto; pula no gimbal lock";
            DrawText(help, static_cast<int>(x0), static_cast<int>(interpPanel.y) + 96, 16, LIGHTGRAY);
            DrawText(TextFormat("FPS de render: %d", fpsApplied), static_cast<int>(x0), static_cast<int>(interpPanel.y) + 144, 18, RAYWHITE);
            GuiSlider(Rectangle{x0 + 24.0f, interpPanel.y + 168.0f, w - 64.0f, 22.0f}, "10", "240", &fpsSlider, 10.0f, 240.0f);
            const int wantedFps = static_cast<int>(std::lround(static_cast<double>(fpsSlider)));
            if (wantedFps != fpsApplied) {
                fpsApplied = wantedFps;
                SetTargetFPS(fpsApplied);
            }
        }
        // --- Painel do objeto selecionado (V6) --------------------------------------------------------------
        if (objPanelShown && scene.world->isValidBody(selected)) {
            physics::PhysicsWorld& world = *scene.world;
            if (selected != lastSelected) {   // rascunho da orientação parte da orientação atual do corpo
                lastSelected = selected;
                const viewer::AxisAngleF aa0 = viewer::toAxisAngle(world.getBody(selected).getOrientation());
                oAxisX = aa0.axis.x; oAxisY = aa0.axis.y; oAxisZ = aa0.axis.z;
                oAngleDeg = aa0.angle * 57.29578f;
            }
            GuiPanel(objPanel, TextFormat("Objeto selecionado (corpo %u)", static_cast<unsigned>(selected)));
            const float ox = objPanel.x + 12.0f, ow = objPanel.width - 24.0f;
            float oy = objPanel.y + 32.0f;
            GuiToggleGroup(Rectangle{ox, oy, ow / 4.0f, 26.0f}, "Material;Posicao;Veloc;Orient", &objTab);
            oy += 36.0f;
            const float labelW = 96.0f, valueW = 58.0f;
            const float barW = ow - labelW - valueW - 8.0f;
            // Uma linha: nome, slider e valor. Devolve true se o USUÁRIO mexeu no slider (mouse sobre a barra com o
            // botão apertado); o valor lido do corpo fora da faixa não conta como edição (o raygui o limita).
            auto sliderRow = [&](const char* name, float& v, float lo, float hi, const char* valueText) {
                const Rectangle bar{ox + labelW, oy, barW, 20.0f};
                const float before = v;
                DrawText(name, static_cast<int>(ox), static_cast<int>(oy) + 2, 16, LIGHTGRAY);
                GuiSlider(bar, nullptr, nullptr, &v, lo, hi);
                DrawText(valueText, static_cast<int>(ox + labelW + barW) + 6, static_cast<int>(oy) + 2, 16, RAYWHITE);
                oy += 26.0f;
                return v != before && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, bar);
            };
            using physics::Real;
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
                DrawText(bp.awake ? "estado: acordado" : "estado: dormindo", static_cast<int>(ox), static_cast<int>(oy), 16, bp.awake ? LIGHTGRAY : Color{110, 170, 255, 255});
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
                DrawText("q = (cos(a/2), sen(a/2) * eixo unitario)", static_cast<int>(ox), static_cast<int>(oy), 14, GRAY);
            }
        }

        if (selected != physics::kInvalidBodyId) {
            const physics::RigidBody& sb = scene.world->getBody(selected);
            DrawText(TextFormat("selecionado: corpo %u   massa %.2f kg   |v| = %.2f m/s   %s", static_cast<unsigned>(selected),
                                static_cast<double>(sb.getMass()), static_cast<double>(sb.getLinearVelocity().length()),
                                sb.isAwake() ? "acordado" : "dormindo"),
                     static_cast<int>(quatPanel.x), static_cast<int>(quatPanel.y + quatPanel.height) + 8, 18, Color{255, 235, 60, 255});
        }
        DrawText(TextFormat("%d FPS   fisica %.0f Hz   passo #%llu   alpha = %.2f   tempo %s%s", GetFPS(),
                            1.0 / static_cast<double>(scene.world->getFixedTimeStep()),
                            static_cast<unsigned long long>(scene.world->getStats().stepCount),
                            static_cast<double>(timeCtl.renderAlpha(*scene.world)),
                            viewer::TimeControl::scaleLabel(timeCtl.scaleIndex()), timeCtl.paused() ? "   [PAUSADO]" : ""),
                 16, GetScreenHeight() - 80, 18, LIGHTGRAY);
        DrawText("esq: selecionar/arrastar   dir ou roda apertada: girar   roda: zoom   B/O: lancar   Del/X: apagar", 16,
                 GetScreenHeight() - 54, 18, GRAY);
        DrawText("Espaco/P: pausa   . : passo   setas/1-9: cena   R: reiniciar   I: interpolacao   T: alvo   Home: camera   ESC: sair", 16,
                 GetScreenHeight() - 30, 18, GRAY);
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
