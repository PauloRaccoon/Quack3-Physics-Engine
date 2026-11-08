// =============================================================================
// ModelCollider.h  (V15a)
// Do modelo .obj à forma de colisão: fecho convexo dos vértices (ConvexHull) -> PolyhedronShape do motor,
// centrado no centroide do sólido (assim o corpo gira em torno do centro de massa correto).
// Um modelo CÔNCAVO (uma xícara, um "C") colide como o seu fecho: o buraco é preenchido. Isto é uma
// limitação assumida da V15; a malha desenhada continua sendo o modelo original.
// =============================================================================
#pragma once

#include <memory>
#include <string>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/CapsuleShape.h"
#include "physics/geometry/CylinderShape.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/Shape.h"
#include "physics/geometry/SphereShape.h"
#include "viewer/core/ConvexHull.h"
#include "viewer/core/ObjModel.h"

namespace viewer {

namespace modellimits {
constexpr float kMinScale = 1.0e-3f;
constexpr float kMaxScale = 1.0e3f;
constexpr float kMaxSize = 40.0f;   ///< maior dimensão do modelo já escalado [m]
constexpr float kMinSize = 0.05f;   ///< menor dimensão (a maior das três) aceita [m]
constexpr int kMaxHullVertices = 4096;   ///< o fecho de um modelo muito detalhado é grande demais para o motor/arquivo
constexpr int kMaxHullFaces = 8192;
} // namespace modellimits

/// V17: com que forma o MOTOR enxerga o modelo (a malha desenhada é sempre a original).
///   Hull   — fecho convexo dos vértices (colisão mais fiel; modelo côncavo tem o buraco preenchido);
///   Box    — caixa envolvente alinhada aos eixos do modelo (barata e estável; sobra folga nos cantos);
///   Sphere — esfera envolvente centrada na caixa envolvente (rola; sobra folga).
///   Capsule  — (V18) cápsula de eixo Y local, ajustada à caixa envolvente: raio = a maior meia-aresta de X e Z, e o
///              segmento cobre o resto da altura. Ideal para pessoas e personagens em pé (modelo com Y para cima).
///   Cylinder — (V18) cilindro de eixo Y local, mesmo ajuste (colunas, latas, rodas em pé).
enum class ColliderKind { Hull = 0, Box = 1, Sphere = 2, Capsule = 3, Cylinder = 4 };
constexpr int kColliderKindCount = 5;

/// V18: ajuste fino de Capsule/Cylinder sobre o ajuste automático (1 = o ajuste automático). Limitado a [0,25; 2].
struct ColliderTuning {
    float radiusScale = 1.0f;
    float heightScale = 1.0f;
};
constexpr float kMinTuning = 0.25f;
constexpr float kMaxTuning = 2.0f;

/// Nome curto sem acentos (a fonte da raylib é ASCII), para a janela e as mensagens.
const char* colliderKindName(ColliderKind kind);

struct ImportedShape {
    std::shared_ptr<const physics::Shape> shape;   ///< Hull: PolyhedronShape centrada no centroide; Box/Sphere: centrada na caixa envolvente
    ColliderKind collider = ColliderKind::Hull;
    physics::Vector3 boxHalf;         ///< Box: meias-arestas [m] (já escaladas)
    float sphereRadius = 0.0f;        ///< Sphere: raio [m]
    float axisRadius = 0.0f;          ///< Capsule/Cylinder: raio [m] (V18)
    float axisHalfHeight = 0.0f;      ///< Capsule: meia-altura do segmento; Cylinder: meia-altura [m] (V18)
    int longestAxis = 1;              ///< eixo mais longo do modelo (0 = X, 1 = Y, 2 = Z): a cápsula/cilindro é de eixo Y
    double colliderVolume = 0;        ///< volume da forma de colisão [m³]
    double fill = 0;                  ///< volume do modelo / volume do colisor (0 se desconhecido): perto de 1 = colisor justo
    /// Ponto do modelo (já escalado) que é a origem do corpo: ponto_local = v·escala − offset.
    physics::Vector3 offset;
    float scale = 1.0f;
    int hullVertices = 0;
    int hullFaces = 0;        ///< faces após a fusão coplanar
    int hullTriangles = 0;    ///< triângulos do fecho antes da fusão
    int modelVertices = 0;    ///< vértices usados pelo modelo
    double hullVolume = 0;    ///< volume do fecho [m³]
    double modelVolume = 0;   ///< volume da malha (só confiável se fechada); 0 se não calculado
    bool concave = false;     ///< o fecho é bem maior que o modelo (volume do modelo < 90% do fecho): colisão preenche buracos
};

/// Escala para que a MAIOR dimensão da caixa envolvente do modelo seja `targetSize` [m]. Modelo sem
/// extensão: 1.
float autoScale(const ObjModel& model, float targetSize);

/// Constrói a forma de colisão. `scale` em [kMinScale, kMaxScale]; maior dimensão resultante em
/// [kMinSize, kMaxSize]. Falha (false + `error`) para modelo sem volume (plano, linha, ponto), escala
/// inválida, tamanho fora da faixa ou recusa do motor.
bool makeImportedShape(const ObjModel& model, float scale, ImportedShape& out, std::string& error);

/// Igual, escolhendo o tipo de colisor (V17). Box e Sphere aceitam também modelos planos/finos (a espessura mínima de
/// uma caixa é 1 cm); Hull continua exigindo volume.
bool makeImportedShape(const ObjModel& model, float scale, ColliderKind kind, ImportedShape& out, std::string& error);

/// Igual, com o ajuste fino do raio e da altura (só faz diferença para Capsule e Cylinder). V18.
bool makeImportedShape(const ObjModel& model, float scale, ColliderKind kind, const ColliderTuning& tuning,
                       ImportedShape& out, std::string& error);

/// Volume da malha fechada (soma de tetraedros com sinal, em unidades do modelo ao cubo × escala³);
/// valor absoluto. Malha aberta dá um valor sem significado.
double meshVolume(const ObjModel& model, float scale);

} // namespace viewer
