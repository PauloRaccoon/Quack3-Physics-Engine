// =============================================================================
// GJK.h
// Algoritmo de Gilbert-Johnson-Keerthi (versão de distância, estilo Ericson /
// van den Bergen) sobre a diferença de Minkowski A ⊖ B.
//
// Mantém um simplex de até 4 pontos (tetraedro). A cada iteração:
//   1) acha o ponto do simplex mais próximo da origem e descarta os vértices
//      que não contribuem (redução do simplex);
//   2) pede o ponto de suporte na direção da origem;
//   3) para quando o suporte não avança (distância convergiu) ou quando a
//      origem fica dentro do simplex (formas se interpenetram).
//
// Resultado:
//   - interpenetração: o simplex que contém a origem, a ser usado pelo EPA;
//   - separação: a distância e os pontos mais próximos em A e em B.
// =============================================================================
#pragma once

#include "physics/collision/narrowphase/SupportFunction.h"

namespace physics {

/// Até 4 pontos de suporte (ponto, tetraedro).
struct Simplex {
    SupportPoint v[4];
    int size = 0;

    void add(const SupportPoint& p) { v[size++] = p; }
};

struct GJKResult {
    bool intersecting = false;   ///< a origem está dentro de A ⊖ B (ou sobre a borda)
    bool converged = true;       ///< false se atingiu o limite de iterações / entrada inválida
    Simplex simplex;             ///< intersecting: contém a origem (1..4 pontos)
    Vector3 closestA;            ///< separado: ponto mais próximo em A (núcleo)
    Vector3 closestB;            ///< separado: ponto mais próximo em B (núcleo)
    Real distance = Real(0);     ///< separado: distância entre os núcleos
    int iterations = 0;
};

class GJK {
public:
    static GJKResult compute(const MinkowskiSupport& support);

    /// Ponto do simplex mais próximo da origem. Reduz `simplex` in-place aos
    /// vértices que contribuem e preenche `lambdas` (coordenadas baricêntricas,
    /// alinhadas com o simplex reduzido). Para um tetraedro que contém a
    /// origem retorna (0,0,0) e mantém os 4 vértices. Exposta para testes.
    static Vector3 closestPointOnSimplex(Simplex& simplex, Real* lambdas);
};

} // namespace physics
