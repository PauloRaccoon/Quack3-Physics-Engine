// =============================================================================
// ObjModel.h  (V15a)
// Leitor de modelos Wavefront .obj (+ .mtl), sem raylib. Lê só o que o viewer usa:
//   .obj: v x y z | vt u v | f (v, v/vt, v//vn, v/vt/vn; índices 1-based ou negativos; polígonos de
//         qualquer tamanho viram triângulos em leque) | usemtl nome | mtllib arquivo | o/g/s/vn (ignorados)
//   .mtl: newmtl nome | Kd r g b | map_Kd arquivo (a textura difusa)
// O arquivo é validado antes de virar modelo: erro de sintaxe, índice fora do intervalo, número não
// finito ou limite estourado dão uma mensagem com o número da linha e NÃO produzem modelo parcial.
// O desenho (textura na GPU) é da janela (V15b); a colisão usa o fecho convexo (ConvexHull.h).
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include "viewer/core/ViewerMath.h"

namespace viewer {

namespace objlimits {
constexpr std::size_t kMaxBytes = 32u * 1024u * 1024u;   ///< tamanho máximo do arquivo
constexpr std::size_t kMaxVertices = 400000;
constexpr std::size_t kMaxTriangles = 800000;
constexpr float kMaxCoordinate = 1.0e6f;                  ///< |coordenada| máxima de um vértice
} // namespace objlimits

struct ObjUV {
    float u = 0, v = 0;
};

struct ObjMaterial {
    std::string name;
    std::string diffuseMap;                   ///< map_Kd (caminho relativo ao .mtl); vazio = sem textura
    float kd[3] = {0.8f, 0.8f, 0.8f};         ///< cor difusa (0..1)
};

struct ObjTriangle {
    int p[3] = {0, 0, 0};      ///< índices em positions
    int t[3] = {-1, -1, -1};   ///< índices em uvs (-1 = sem UV)
    int material = -1;         ///< índice em materials (-1 = sem material)
};

struct ObjModel {
    std::string name;                         ///< nome do arquivo sem extensão (ou o `o` do arquivo)
    std::vector<Vec3f> positions;
    std::vector<ObjUV> uvs;
    std::vector<ObjTriangle> triangles;
    std::vector<ObjMaterial> materials;
    std::vector<std::string> mtlLibs;         ///< arquivos `mtllib` citados pelo .obj
};

/// Lê o texto de um .obj. Falha: devolve false, `error` = "linha N: ..." (ou mensagem geral) e `out` limpo.
bool parseObj(const std::string& text, ObjModel& out, std::string& error);

/// Lê o texto de um .mtl e ACRESCENTA os materiais a `out`.
bool parseMtl(const std::string& text, std::vector<ObjMaterial>& out, std::string& error);

/// Lê o .obj do disco e os .mtl que ele cita (na mesma pasta; nomes com ".." ou absolutos são recusados).
/// .mtl ausente NÃO é erro (o modelo fica com materiais só de nome, sem textura); o resultado diz isso em `warning`.
bool loadObjFile(const std::string& path, ObjModel& out, std::string& error, std::string* warning = nullptr);

/// Caixa envolvente (min, max) dos vértices usados pelos triângulos. false se o modelo não tem triângulos.
bool objBounds(const ObjModel& model, Vec3f& lo, Vec3f& hi);

} // namespace viewer
