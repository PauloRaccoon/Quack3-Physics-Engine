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

#include "physics/geometry/PolyhedronShape.h"
#include "viewer/core/ConvexHull.h"
#include "viewer/core/ObjModel.h"

namespace viewer {

namespace modellimits {
constexpr float kMinScale = 1.0e-3f;
constexpr float kMaxScale = 1.0e3f;
constexpr float kMaxSize = 40.0f;   ///< maior dimensão do modelo já escalado [m]
constexpr float kMinSize = 0.05f;   ///< menor dimensão (a maior das três) aceita [m]
} // namespace modellimits

struct ImportedShape {
    std::shared_ptr<const physics::PolyhedronShape> shape;   ///< convexa, centrada no centroide
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

/// Volume da malha fechada (soma de tetraedros com sinal, em unidades do modelo ao cubo × escala³);
/// valor absoluto. Malha aberta dá um valor sem significado.
double meshVolume(const ObjModel& model, float scale);

} // namespace viewer
