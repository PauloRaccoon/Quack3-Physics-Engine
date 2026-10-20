// =============================================================================
// RayCast.h
// Consultas geométricas contra UMA forma convexa: raio e ponto.
//
//   raycastShape()       raio (origem + t·direção) contra a superfície da forma
//   shapeContainsPoint() ponto dentro (ou sobre a superfície) da forma
//
// Algoritmos:
//   - Esfera: equação quadrática exata |o + t·d|² = r².
//   - Caixa e Poliedro: recorte do raio pelos planos das faces (Cyrus-Beck) usando
//     a HalfEdgeStructure — um único algoritmo para qualquer poliedro convexo.
//     Para cada face (n·p ≤ offset) o raio entra (n·d < 0) ou sai (n·d > 0) do
//     semi-espaço; o raio atravessa o sólido em [max(entradas), min(saídas)].
//
// Convenção quando a origem está DENTRO da forma: o resultado é um acerto com
// distância 0, `startedInside = true` e normal = −direção (nenhuma superfície foi
// "atravessada"). Quem não quer esse acerto filtra por `startedInside`.
// =============================================================================
#pragma once

#include "physics/geometry/Shape.h"

namespace physics {

/// Resultado de um raio contra uma forma (coordenadas de MUNDO).
struct ShapeRayHit {
    Real distance = Real(0);   ///< t ao longo da direção UNITÁRIA, em [0, maxDistance]
    Vector3 point;             ///< origem + t·direção
    Vector3 normal;            ///< normal unitária da superfície atingida, para fora da forma
    bool startedInside = false;///< a origem já estava dentro da forma (t = 0, normal = −direção)
};

/// Raio contra a superfície de `shape` (posicionada por `xf`).
/// @param direction  deve ser UNITÁRIA (o chamador normaliza)
/// @param maxDistance alcance máximo do raio (>= 0; +infinito é aceito)
/// Retorna false (sem tocar em `hit`) se não acerta, ou se qualquer entrada for
/// inválida (origem/direção/transformação não finitas, alcance NaN ou negativo).
bool raycastShape(const Shape& shape, const Transform& xf, const Vector3& origin,
                  const Vector3& direction, Real maxDistance, ShapeRayHit& hit);

/// true se `worldPoint` está dentro de `shape` ou sobre a sua superfície.
/// Retorna false para ponto/transformação não finitos.
bool shapeContainsPoint(const Shape& shape, const Transform& xf, const Vector3& worldPoint);

} // namespace physics
