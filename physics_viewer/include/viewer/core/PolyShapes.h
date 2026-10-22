// =============================================================================
// PolyShapes.h
// Fábricas de poliedros convexos (tetraedro, octaedro, prisma triangular, pirâmide de base quadrada)
// para o motor, a partir de vértices e faces. Usadas pelas cenas e pelo painel "Adicionar objeto".
// =============================================================================
#pragma once

#include <memory>
#include <vector>

#include "physics/common/Types.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/math/Vector3.h"

namespace viewer {

/// Poliedro convexo a partir de vértices e faces (cada face é um ciclo de índices, em qualquer
/// sentido): inverte as que ficarem voltadas para dentro, para que todas apontem para fora
/// (anti-horário visto de fora), como o motor exige. Lança std::exception se a malha for inválida.
std::shared_ptr<const physics::PolyhedronShape> makePoly(const std::vector<physics::Vector3>& verts,
                                                         std::vector<std::vector<int>> faces);

/// Tetraedro regular com vértices em (±s, ±s, ±s) alternados (aresta 2·√2·s).
std::shared_ptr<const physics::PolyhedronShape> makeTetrahedron(physics::Real s);
/// Octaedro com vértices em ±s sobre cada eixo.
std::shared_ptr<const physics::PolyhedronShape> makeOctahedron(physics::Real s);
/// Prisma de base triangular equilátera (circunraio r) e altura 2·halfHeight, eixo Y.
std::shared_ptr<const physics::PolyhedronShape> makeTriPrism(physics::Real r, physics::Real halfHeight);
/// Pirâmide de base quadrada (meia-aresta halfBase) e altura `height`, eixo Y; base abaixo, ápice acima.
std::shared_ptr<const physics::PolyhedronShape> makeSquarePyramid(physics::Real halfBase, physics::Real height);

} // namespace viewer
