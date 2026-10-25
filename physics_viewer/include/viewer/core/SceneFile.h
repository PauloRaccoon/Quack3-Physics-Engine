// =============================================================================
// SceneFile.h
// Salvar e carregar cenas em arquivo de texto (`.scene`), legível e editável à mão (sem raylib, testável).
//
// FORMATO (versão 1; `#` começa um comentário; números em notação decimal com ponto):
//
//   physics_scene 1
//   name Minha cena
//   gravity 0 -9.81 0
//   timestep 0.0166666667
//   iterations 10 3              # velocidade, posição
//   slop 0.005 0.5               # linear [m], velocidade [m/s]
//   warm_start 1
//   sleep 1 0.5                  # ligado, tempo [s]
//   camera 0 1.5 0 12            # alvo x y z e distância
//   body                         # corpos: índice 1, 2, 3... na ordem do arquivo
//     type dynamic               # static | dynamic | kinematic
//     shape box 0.5 0.5 0.5      # box hx hy hz (meias-arestas) | sphere r | none | poly
//     vertex 1 0 0               #   (só em poly) vértices locais
//     face 0 1 2                 #   (só em poly) índices a partir de 0, anti-horário visto de fora
//     mass 1                     # [kg], obrigatório em dynamic
//     position 0 5 0
//     orientation 1 0 0 0        # QUATÉRNIO  w x y z  (normalizado ao carregar; nulo = erro)
//     velocity 0 0 0
//     angular_velocity 1.2 2 0.7
//     material 0.5 0.3           # atrito, restituição
//     damping 0 0                # linear, angular
//     gravity_scale 1
//     allow_sleep 1
//     color 240 150 60
//     focus                      # o corpo do painel do quatérnio (no máximo um)
//   end
//   joint hinge 1 2  ax ay az  dx dy dz  collide      # âncora (mundo), eixo, colide 0|1
//   joint ballsocket 1 2  ax ay az  collide
//   joint distance 1 2  ax ay az  bx by bz  length collide
//   link 1 2                     # linha desenhada entre os centros
//
// `shape none` é um pivô sem forma (estático), desenhado como uma pequena esfera.
// NÃO é salvo: estado de sono (carrega acordado), contatos e impulsos acumulados (warm start).
// Entrada hostil (arquivo grande, número inválido, índice fora da faixa, poliedro côncavo...) gera
// uma mensagem de erro com o número da linha e NÃO altera a cena de destino.
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include "viewer/core/Scene.h"

namespace viewer {

namespace scenefile {
constexpr int kVersion = 1;
constexpr std::size_t kMaxBytes = 8u * 1024u * 1024u;
constexpr int kMaxBodies = 2000;
constexpr int kMaxJoints = 2000;
constexpr int kMaxPolyVertices = 256;
constexpr int kMaxPolyFaces = 512;
constexpr int kMaxFaceVertices = 32;
constexpr std::size_t kMaxNameLength = 64;
constexpr const char* kExtension = ".scene";
} // namespace scenefile

/// Texto do arquivo para a cena (estado ATUAL dos corpos: poses, velocidades, parâmetros e o que foi
/// adicionado). Os números saem no menor número de dígitos que volta exatamente ao mesmo valor.
std::string serializeScene(const Scene& scene);

/// Texto -> cena. `out` só é tocado se devolver true; senão `error` diz o que houve ("linha N: ...").
bool parseScene(const std::string& text, Scene& out, std::string& error);

/// Arquivos. `path` é o caminho completo; as pastas que faltam são criadas ao salvar.
bool saveSceneFile(const Scene& scene, const std::string& path, std::string& error);
bool loadSceneFile(const std::string& path, Scene& out, std::string& error);

/// Nomes (sem a extensão) dos `.scene` de uma pasta, em ordem alfabética. Pasta inexistente: lista vazia.
std::vector<std::string> listSceneFiles(const std::string& dir);

/// Nome de arquivo seguro a partir do que o usuário digitou: letras, dígitos, `-` e `_` (espaço vira `_`),
/// sem acento, sem barra, no máximo 48 caracteres. Vazio ou só símbolos: "cena".
std::string sanitizeSceneFileName(const std::string& typed);

} // namespace viewer
