// =============================================================================
// main.cpp — janela do physics_viewer (V2)
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
// Controles: botão direito + arrastar = girar a câmera; roda = zoom; ←/→ ou 1–7 = trocar de
//            cena; R = reiniciar a cena atual; P = pausar; ESC = sair.
//
// Verificação automática (sem interação):
//     physics_viewer --frames 240 --screenshot saida.png
// roda 240 quadros, salva uma captura da tela e sai com código 0.
// =============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "raylib.h"
#include "rlgl.h"

#include "viewer/core/OrbitCamera.h"
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
    InitWindow(kScreenW, kScreenH, "physics_viewer — V2");
    SetTargetFPS(120);   // o render NÃO precisa casar com os 60 Hz da física: há interpolação

    viewer::OrbitCamera orbit;
    const std::size_t sceneCount = viewer::sceneList().size();
    std::size_t sceneIndex = 0;
    viewer::Scene scene = viewer::makeScene(sceneIndex);
    orbit.target = scene.cameraTarget;
    orbit.setDistance(scene.cameraDistance);
    bool paused = false;
    int frame = 0;

    while (!WindowShouldClose()) {
        // --- Entrada -------------------------------------------------------------------------------------
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const Vector2 d = GetMouseDelta();
            orbit.rotate(-d.x * 0.005f, d.y * 0.005f);
        }
        orbit.zoom(GetMouseWheelMove());
        // Troca de cena: ←/→ (com volta) ou 1–9; R recria a atual (reinicia).
        std::size_t wanted = sceneIndex;
        if (IsKeyPressed(KEY_RIGHT)) wanted = (sceneIndex + 1) % sceneCount;
        if (IsKeyPressed(KEY_LEFT)) wanted = (sceneIndex + sceneCount - 1) % sceneCount;
        for (int k = 0; k < 9; ++k)
            if (IsKeyPressed(KEY_ONE + k) && static_cast<std::size_t>(k) < sceneCount) wanted = static_cast<std::size_t>(k);
        const bool changed = wanted != sceneIndex;
        if (changed || IsKeyPressed(KEY_R)) {
            sceneIndex = wanted;
            scene = viewer::makeScene(sceneIndex);
            if (changed) {   // só ao trocar: R mantém a câmera que o usuário ajustou
                orbit.target = scene.cameraTarget;
                orbit.setDistance(scene.cameraDistance);
            }
        }
        if (IsKeyPressed(KEY_P)) paused = !paused;

        // --- Física: N passos fixos conforme o tempo de render ------------------------------------------------
        if (!paused) scene.world->update(static_cast<physics::Real>(GetFrameTime()));

        // --- Câmera ----------------------------------------------------------------------------------------------
        Camera3D cam{};
        cam.position = toRay(orbit.position());
        cam.target = toRay(orbit.target);
        cam.up = Vector3{0, 1, 0};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;

        // --- Desenho ----------------------------------------------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{28, 30, 36, 255});

        BeginMode3D(cam);
        DrawGrid(20, 1.0f);
        viewer::Mat4f focusMatrix;
        bool hasFocus = false;
        physics::Transform focusPose;
        for (const viewer::SceneBody& body : scene.bodies) {
            // Pose de RENDER: interpolação entre os dois últimos passos físicos, com o α atual.
            const physics::Transform pose = scene.world->getInterpolatedTransform(body.id);
            const viewer::Mat4f m = viewer::toMat4(pose);
            drawBody(body, m);
            if (body.id == scene.focus) {
                focusMatrix = m;
                focusPose = pose;
                hasFocus = true;
            }
        }
        for (const viewer::SceneLink& link : scene.links) {   // juntas: linha entre os centros
            const physics::Vector3 pa = scene.world->getInterpolatedTransform(link.a).position;
            const physics::Vector3 pb = scene.world->getInterpolatedTransform(link.b).position;
            DrawLine3D(Vector3{float(pa.x), float(pa.y), float(pa.z)}, Vector3{float(pb.x), float(pb.y), float(pb.z)},
                       Color{235, 235, 235, 255});
        }
        if (hasFocus) drawLocalAxes(focusMatrix, 1.2f);
        EndMode3D();

        // --- HUD ------------------------------------------------------------------------------------------------------
        DrawText(TextFormat("[%d/%d] %s   (%d corpos)", static_cast<int>(sceneIndex) + 1,
                            static_cast<int>(sceneCount), scene.name.c_str(),
                            static_cast<int>(scene.bodies.size())),
                 16, 12, 22, RAYWHITE);
        if (hasFocus) {
            const physics::Quaternion& q = focusPose.rotation;
            DrawText(TextFormat("q = (w %+.3f   x %+.3f   y %+.3f   z %+.3f)", static_cast<double>(q.w),
                                static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z)),
                     16, 44, 20, Color{240, 190, 110, 255});
            DrawText(TextFormat("|q| = %.6f", static_cast<double>(q.norm())), 16, 68, 20,
                     Color{240, 190, 110, 255});
        }
        DrawText(TextFormat("%d FPS   fisica 60 Hz   alpha = %.2f%s", GetFPS(),
                            static_cast<double>(scene.world->getAlpha()), paused ? "   [PAUSADO]" : ""),
                 16, GetScreenHeight() - 56, 18, LIGHTGRAY);
        DrawText("botao direito: girar   roda: zoom   setas/1-7: cena   R: reiniciar   P: pausar   ESC: sair", 16,
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
