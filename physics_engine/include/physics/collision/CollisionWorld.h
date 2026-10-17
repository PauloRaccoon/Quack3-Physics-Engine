// =============================================================================
// CollisionWorld.h
// Detecção de colisão completa, independente de dinâmica: junta a Broadphase
// (Fat AABBs em DynamicAABBTree) com a narrowphase (GJK/EPA + manifolds
// persistentes) e mantém o conjunto de PARES ATIVOS entre quadros.
//
// Fluxo por quadro:
//   1) updateObject(...) para cada objeto que se moveu;
//   2) detect():
//        a) a broadphase gera os pares NOVOS (só para proxies que escaparam do
//           Fat AABB);
//        b) pares cujos Fat AABBs deixaram de se sobrepor são descartados;
//        c) a narrowphase atualiza o ContactManifold de cada par restante
//           (preservando ids e impulsos para o warm starting);
//   3) getTouching() lista os pares que de fato têm pontos de contato.
//
// Os objetos são identificados por BodyId (índice do chamador). Os pares ficam
// num std::map ordenado pelo par (a,b): a iteração é DETERMINÍSTICA, requisito
// para simulações reprodutíveis. As únicas alocações ocorrem quando um par
// nasce ou morre — não há alocação para pares que persistem.
// =============================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "physics/collision/broadphase/Broadphase.h"
#include "physics/collision/narrowphase/ContactManifold.h"
#include "physics/geometry/Shape.h"

namespace physics {

struct CollisionWorldSettings {
    Real aabbMargin = kAABBMargin;
    Real aabbDisplacementMultiplier = kAABBDisplacementMultiplier;
    ContactGeneratorSettings contact;
    int initialCapacity = 256;   ///< proxies pré-alocados na árvore
};

/// Par de objetos cujos Fat AABBs se sobrepõem, com o manifold persistente.
/// Convenção: a < b; a normal do manifold aponta de `a` para `b`.
struct ContactPair {
    BodyId a = kInvalidBodyId;
    BodyId b = kInvalidBodyId;
    ProxyId proxyA = kNullProxy;
    ProxyId proxyB = kNullProxy;
    ContactManifold manifold;
};

class CollisionWorld {
public:
    /// Filtro adicional de pares (camadas, grupos...). Chamado só quando um par
    /// NASCE; para efeito imediato sobre um par existente use removePair().
    using PairFilter = std::function<bool(BodyId, BodyId)>;

    explicit CollisionWorld(const CollisionWorldSettings& settings = CollisionWorldSettings());
    CollisionWorld(const CollisionWorld&) = delete;
    CollisionWorld& operator=(const CollisionWorld&) = delete;

    void setPairFilter(PairFilter filter) { filter_ = std::move(filter); }

    // --- Objetos ------------------------------------------------------------------------
    /// Registra um objeto de colisão. `moving = false` marca um objeto imóvel:
    /// dois imóveis nunca formam par.
    /// @throws std::invalid_argument se a forma for nula, o id já estiver em uso
    ///         ou a transformação/AABB não forem finitos.
    void addObject(BodyId id, std::shared_ptr<const Shape> shape, const Transform& transform, bool moving);

    /// Remove o objeto e todos os seus pares. Id desconhecido: ignora.
    void removeObject(BodyId id);

    bool hasObject(BodyId id) const { return id < objects_.size() && objects_[id].active; }

    /// Informa a nova pose. `displacement` (deslocamento previsto neste passo)
    /// estende o Fat AABB na direção do movimento. Pose não finita é ignorada.
    void updateObject(BodyId id, const Transform& transform, const Vector3& displacement = Vector3::zero());

    // --- Detecção ---------------------------------------------------------------------------
    /// Veja o fluxo no cabeçalho. Invalida os ponteiros devolvidos por getTouching().
    void detect();

    /// Pares que têm ao menos um ponto de contato após o último detect().
    const std::vector<ContactPair*>& getTouching() const { return touching_; }

    std::size_t getPairCount() const { return pairs_.size(); }

    /// Par (a,b) em qualquer ordem, ou nullptr.
    const ContactPair* findPair(BodyId a, BodyId b) const;

    /// Descarta o par (a,b), se existir (ex.: depois de ligar os corpos por uma junta).
    void removePair(BodyId a, BodyId b);

    /// Objetos cujo Fat AABB intersecta `box` (conservador: pode incluir falsos
    /// positivos). `callback(BodyId)` retorna true para continuar.
    template <typename Callback>
    void queryAABB(const AABB& box, Callback&& callback) const {
        broadphase_.query(box, [&](ProxyId, std::uint32_t id) { return callback(static_cast<BodyId>(id)); });
    }

    const Broadphase& getBroadphase() const { return broadphase_; }
    const CollisionWorldSettings& getSettings() const { return settings_; }

    void clear();

private:
    struct Object {
        std::shared_ptr<const Shape> shape;
        Transform transform;
        ProxyId proxy = kNullProxy;
        bool moving = false;
        bool active = false;
    };

    static std::uint64_t pairKey(BodyId a, BodyId b) {
        return (static_cast<std::uint64_t>(a) << 32) | static_cast<std::uint64_t>(b);
    }
    bool acceptPair(BodyId a, BodyId b) const;

    CollisionWorldSettings settings_;
    Broadphase broadphase_;
    std::vector<Object> objects_;                       ///< indexado por BodyId
    std::map<std::uint64_t, ContactPair> pairs_;        ///< ordenado: iteração determinística
    std::vector<ContactPair*> touching_;
    PairFilter filter_;
};

} // namespace physics
