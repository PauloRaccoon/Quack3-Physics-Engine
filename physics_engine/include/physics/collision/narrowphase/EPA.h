// =============================================================================
// EPA.h
// Expanding Polytope Algorithm: dado o simplex do GJK que contém a origem
// (formas interpenetrando), expande um politopo dentro de A ⊖ B até encontrar
// a face da borda mais próxima da origem. Essa face dá:
//
//   normal  n : direção de A para B   (mover B por +n·depth separa as formas)
//   depth   d : profundidade de penetração (>= 0)
//   pointA / pointB : pontos de contato nas superfícies de A e de B,
//                     com  pointA − pointB = n · d
//
// Todo o armazenamento é de capacidade fixa na pilha (sem alocação dinâmica).
// =============================================================================
#pragma once

#include "physics/collision/narrowphase/GJK.h"

namespace physics {

struct EPAResult {
    bool valid = false;       ///< false => não foi possível calcular (degenerado)
    bool converged = false;   ///< false => parou por limite de iterações/capacidade (resultado aproximado)
    Vector3 normal;           ///< unitária, de A para B
    Real depth = Real(0);
    Vector3 pointA;           ///< ponto de contato em A (mundo, núcleo)
    Vector3 pointB;           ///< ponto de contato em B (mundo, núcleo)
    int iterations = 0;
};

class EPA {
public:
    /// @param simplex simplex retornado pelo GJK com `intersecting == true` (1..4 pontos)
    static EPAResult compute(const MinkowskiSupport& support, const Simplex& simplex);
};

} // namespace physics
