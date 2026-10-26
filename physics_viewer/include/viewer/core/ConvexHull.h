// =============================================================================
// ConvexHull.h  (V15a)
// Fecho convexo 3D por QUICKHULL, algoritmo próprio (sem bibliotecas), sem raylib. O motor só aceita
// poliedros convexos de faces planas (PolyhedronShape); este módulo transforma qualquer nuvem de pontos
// (por exemplo os vértices de um modelo .obj) no poliedro convexo mínimo que a contém.
//
// Como funciona:
//  1) tetraedro inicial com pontos extremos; cada ponto restante entra no "conjunto externo" da face
//     que ele enxerga;
//  2) repetidamente: o ponto mais distante de uma face é o "olho"; as faces que o enxergam formam a região
//     visível, cuja fronteira (o horizonte) é ligada ao olho por faces novas;
//  3) triângulos vizinhos e coplanares (tolerância relativa ao tamanho) são fundidos em polígonos: o contato
//     de uma caixa apoiada usa a face inteira (4 vértices), não um triângulo.
// Toda a conta é em double; a tolerância é relativa à maior coordenada e fica abaixo da tolerância de
// planaridade/convexidade do motor, então o resultado é aceito por PolyhedronShape.
// =============================================================================
#pragma once

#include <string>
#include <vector>

#include "physics/common/Types.h"
#include "physics/math/Vector3.h"

namespace viewer {

namespace hulllimits {
constexpr std::size_t kMaxPoints = 500000;   ///< pontos de entrada
constexpr int kMaxFaceVertices = 32;         ///< faces fundidas maiores que isto voltam a triângulos (limite do motor para o contato)
} // namespace hulllimits

struct ConvexHull {
    std::vector<physics::Vector3> vertices;        ///< subconjunto dos pontos de entrada (só os que são vértices do fecho)
    std::vector<std::vector<int>> faces;           ///< ciclos de índices, anti-horário visto de fora
    std::vector<int> sourceIndex;                  ///< sourceIndex[i] = índice do vértice i na lista de entrada
    int triangleCount = 0;                         ///< triângulos antes da fusão coplanar
};

/// Calcula o fecho. Falha (false, `error` explica, `out` limpo) se: menos de 4 pontos, algum ponto não
/// finito, mais de kMaxPoints, ou os pontos são coplanares/colineares/coincidentes (volume nulo), ou a
/// conta numérica não fecha o poliedro. `mergeCoplanar = false` devolve só triângulos.
bool buildConvexHull(const std::vector<physics::Vector3>& points, ConvexHull& out, std::string* error = nullptr,
                     bool mergeCoplanar = true);

} // namespace viewer
