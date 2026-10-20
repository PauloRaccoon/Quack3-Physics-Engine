// =============================================================================
// Scene.h
// Uma cena = um PhysicsWorld + a descrição do que desenhar (forma, tamanho, cor, ligações
// de junta). Fica fora da janela para poder ser testada sem raylib. A janela só percorre
// `bodies`, pede a pose interpolada ao mundo e desenha.
//
// REGISTRO DE CENAS: sceneList() lista as cenas disponíveis (nome + fábrica) e
// makeScene(i) cria a i-ésima do zero. Reiniciar uma cena é recriá-la.
// =============================================================================
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "physics/common/Types.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/pipeline/PhysicsWorld.h"
#include "viewer/core/ViewerMath.h"

namespace viewer {

enum class ShapeKind { Box, Sphere, Polyhedron };

/// O que a janela precisa saber para desenhar um corpo do mundo.
struct SceneBody {
    physics::BodyId id = physics::kInvalidBodyId;
    ShapeKind kind = ShapeKind::Box;
    Vec3f halfExtents;        ///< Box: meias-arestas [m]
    float radius = 0.0f;      ///< Sphere: raio [m]
    /// Polyhedron: a forma (vértices e faces vêm de poly->getTopology()). Mantém a forma viva.
    std::shared_ptr<const physics::PolyhedronShape> poly;
    bool isStatic = false;
    unsigned char r = 200, g = 200, b = 200;
};

/// Uma junta, só para desenhar: linha entre os centros dos dois corpos.
struct SceneLink {
    physics::BodyId a = physics::kInvalidBodyId;
    physics::BodyId b = physics::kInvalidBodyId;
};

struct Scene {
    std::string name;
    std::unique_ptr<physics::PhysicsWorld> world;
    std::vector<SceneBody> bodies;
    std::vector<SceneLink> links;
    /// Corpo cujo quatérnio o HUD mostra (e cujos eixos locais são desenhados).
    physics::BodyId focus = physics::kInvalidBodyId;
    /// Enquadramento inicial da câmera para esta cena.
    Vec3f cameraTarget{0.0f, 1.5f, 0.0f};
    float cameraDistance = 12.0f;
};

/// Entrada do registro: nome e fábrica (cria a cena do zero).
struct SceneInfo {
    const char* name;
    Scene (*make)();
};

/// Todas as cenas, na ordem em que a janela as percorre.
const std::vector<SceneInfo>& sceneList();

/// Cria a cena `index` (fora do intervalo: usa o índice limitado ao intervalo válido).
Scene makeScene(std::size_t index);

// --- Cenas individuais (também usadas diretamente pelos testes) ----------------------------------
Scene makeFallingBoxScene();   ///< chão + uma caixa que cai já girando
Scene makeStackScene();        ///< torre de 6 caixas
Scene makePyramidScene();      ///< pirâmide de caixas (5 fileiras)
Scene makeRainScene();         ///< caixas e esferas caindo em orientações aleatórias
Scene makePendulumScene();     ///< pêndulo duplo (juntas de distância)
Scene makeChainScene();        ///< corrente de caixas (juntas bola-soquete)
Scene makeGimbalScene();       ///< gravidade zero, passo largo; gira através de pitch = 90° (gimbal lock)
Scene makeWideStepScene();     ///< gravidade zero, passo de 0,35 s: interpolação de render bem visível
Scene makeShapesScene();       ///< caixa, esfera e poliedros (tetraedro, octaedro, prisma, pirâmide)

} // namespace viewer
