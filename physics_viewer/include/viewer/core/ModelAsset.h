// =============================================================================
// ModelAsset.h  (V15b)
// Um modelo 3D importado já posto na cena: o ObjModel (malha, UV, materiais; compartilhado entre as cópias),
// a forma de colisão (fecho convexo centrado no centroide), a escala e o deslocamento que alinham a malha
// à forma, e a pasta de onde vêm as texturas. Sem raylib: a janela só pede os grupos de malha prontos para
// desenhar (buildMeshGroups) e carrega as texturas.
//
// Alinhamento: ponto_local_do_corpo = vértice_do_modelo · escala − offset.
// =============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "physics/geometry/PolyhedronShape.h"
#include "viewer/core/ModelCollider.h"
#include "viewer/core/ObjModel.h"

namespace viewer {

struct ModelAsset {
    std::shared_ptr<const ObjModel> obj;
    std::string file;        ///< caminho do .obj relativo à pasta de modelos (é o que o .scene guarda)
    std::string baseDir;     ///< pasta do .obj: as texturas `map_Kd` são relativas a ela
    float scale = 1.0f;
    physics::Vector3 offset;
    std::shared_ptr<const physics::Shape> shape;   ///< a forma de colisão (a mesma do corpo): poliedro, caixa ou esfera
    ColliderKind collider = ColliderKind::Hull;     ///< V17: tipo do colisor
    // Dados do colisor, só para exibir:
    int hullVertices = 0, hullFaces = 0, modelVertices = 0;
    bool concave = false;
    double fill = 0;                                ///< volume do modelo / volume do colisor
};

/// Triângulos de um material, prontos para desenhar: posições já no referencial do corpo.
struct MeshGroup {
    int material = -1;                   ///< índice em ObjModel::materials (-1 = sem material)
    std::vector<float> positions;        ///< 9 floats por triângulo
    std::vector<float> texcoords;        ///< 6 floats por triângulo (v já invertido para a raylib)
    std::vector<float> normals;          ///< 3 floats por triângulo (normal plana unitária no referencial do corpo)
    std::size_t triangleCount() const { return normals.size() / 3; }
};

/// Pasta de modelos: lista (ordenada) os `.obj` dela, um nível de subpasta incluído ("sub/arq.obj").
std::vector<std::string> listModelFiles(const std::string& dir);

/// Lê `modelDir/file`, calcula o fecho e monta o asset. `targetSize` > 0 escolhe a escala para que a maior
/// dimensão do modelo seja `targetSize` metros (autoScale). Falhas (arquivo, sintaxe, fecho, limites) dão
/// false e `error`. `warning` recebe avisos (mtl ausente, textura recusada).
bool loadModelAsset(const std::string& modelDir, const std::string& file, float targetSize, ModelAsset& out,
                    std::string& error, std::string* warning = nullptr, ColliderKind collider = ColliderKind::Hull);

/// Novo asset a partir de um modelo já lido (várias cópias com tamanhos diferentes reaproveitam a malha).
bool makeModelAsset(const std::shared_ptr<const ObjModel>& obj, const std::string& file, const std::string& baseDir,
                    float scale, ModelAsset& out, std::string& error, ColliderKind collider = ColliderKind::Hull);

/// Reconstrói o asset de um corpo carregado de arquivo: a forma e o alinhamento vêm do `.scene` (nada de recalcular).
ModelAsset attachModel(const std::shared_ptr<const ObjModel>& obj, const std::string& file, const std::string& baseDir,
                       float scale, const physics::Vector3& offset, const std::shared_ptr<const physics::Shape>& shape);

/// Triângulos agrupados por material (um grupo por material usado, na ordem de aparição).
std::vector<MeshGroup> buildMeshGroups(const ModelAsset& asset);

/// Chave do `.scene` -> pasta do arquivo: "sub/arq.obj" em "base" dá "base/sub".
std::string modelBaseDir(const std::string& modelDir, const std::string& file);

} // namespace viewer
