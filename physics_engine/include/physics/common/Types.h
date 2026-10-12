// =============================================================================
// Types.h
// Identificadores e aliases de tipo compartilhados entre os módulos.
// Usamos índices/handles inteiros (em vez de ponteiros) para manter os dados
// em std::vector contíguos e evitar invalidação de ponteiros.
// =============================================================================
#pragma once

#include <cstdint>
#include <limits>

#include "physics/common/Settings.h"

namespace physics {

/// Índice de um corpo rígido dentro do PhysicsWorld.
using BodyId = std::uint32_t;
constexpr BodyId kInvalidBodyId = std::numeric_limits<BodyId>::max();

/// Índice de um nó da DynamicAABBTree (-1 = nulo).
using ProxyId = std::int32_t;
constexpr ProxyId kNullProxy = -1;

/// Identificador de "feature" (vértice/aresta/face) que gerou um ponto de
/// contato; usado para casar contatos entre quadros (warm starting).
using FeatureId = std::uint32_t;
constexpr FeatureId kInvalidFeature = std::numeric_limits<FeatureId>::max();

/// Máscara de categorias/filtros de colisão.
using CollisionMask = std::uint32_t;

} // namespace physics
