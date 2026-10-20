// =============================================================================
// Broadphase.h
// Fase ampla da detecção de colisão: mantém a DynamicAABBTree e gera os pares
// de objetos cujos Fat AABBs se sobrepõem, em O(M log N) onde M é o número de
// proxies que se moveram (todos, no pior caso: O(N log N)).
//
// Uso típico por quadro:
//   1) para cada corpo ativo:   broadphase.moveProxy(id, aabb, deslocamento);
//   2) broadphase.updatePairs();
//   3) for (pair : broadphase.getPairs()) → narrowphase.
//
// Só proxies "movidos" (que escaparam do Fat AABB, ou recém-criados) são
// consultados; um corpo em repouso não gera trabalho. Os pares entre dois
// objetos que não se moveram já foram reportados em quadros anteriores e
// persistem no ContactManifold (fase 5), por isso não são regenerados aqui.
// =============================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "physics/collision/broadphase/DynamicAABBTree.h"

namespace physics {

/// Par de proxies potencialmente colidindo (proxyA < proxyB).
struct ProxyPair {
    ProxyId proxyA = kNullProxy;
    ProxyId proxyB = kNullProxy;
    std::uint32_t userDataA = 0;
    std::uint32_t userDataB = 0;

    friend bool operator<(const ProxyPair& a, const ProxyPair& b) {
        return a.proxyA != b.proxyA ? a.proxyA < b.proxyA : a.proxyB < b.proxyB;
    }
    friend bool operator==(const ProxyPair& a, const ProxyPair& b) {
        return a.proxyA == b.proxyA && a.proxyB == b.proxyB;
    }
};

class Broadphase {
public:
    /// Filtro opcional: recebe os userData dos dois objetos e retorna false
    /// para descartar o par (ex.: dois corpos estáticos, camadas de colisão,
    /// corpos ligados por junta).
    using PairFilter = std::function<bool(std::uint32_t userDataA, std::uint32_t userDataB)>;

    explicit Broadphase(Real margin = kAABBMargin,
                        Real displacementMultiplier = kAABBDisplacementMultiplier,
                        int initialCapacity = 256);

    void setPairFilter(PairFilter filter) { filter_ = std::move(filter); }

    // --- Proxies --------------------------------------------------------------------
    /// Cria um proxy; ele entra no move buffer (seus pares serão gerados no
    /// próximo updatePairs()).
    ProxyId createProxy(const AABB& tightAABB, std::uint32_t userData);

    void destroyProxy(ProxyId id);

    /// Atualiza o AABB justo. Só entra no move buffer se a árvore mudou.
    /// Retorna true se mudou.
    bool moveProxy(ProxyId id, const AABB& tightAABB, const Vector3& displacement);

    /// Força o proxy a ter seus pares regenerados no próximo updatePairs()
    /// (ex.: mudou o filtro de colisão).
    void touchProxy(ProxyId id);

    // --- Pares -------------------------------------------------------------------------------
    /// Consulta a árvore para cada proxy movido e reconstrói a lista de pares
    /// (sem duplicatas, ordenada). Esvazia o move buffer.
    void updatePairs();

    /// Pares gerados pelo último updatePairs().
    const std::vector<ProxyPair>& getPairs() const { return pairs_; }

    // --- Consultas -------------------------------------------------------------------------------
    bool testOverlap(ProxyId a, ProxyId b) const {
        return tree_.getFatAABB(a).overlaps(tree_.getFatAABB(b));
    }

    /// Proxies cujo Fat AABB intersecta `box`. `callback(ProxyId, userData)` → continuar?
    template <typename Callback>
    void query(const AABB& box, Callback&& callback) const {
        tree_.query(box, [&](ProxyId id) { return callback(id, tree_.getUserData(id)); });
    }

    /// Raio contra os Fat AABBs (ver DynamicAABBTree::raycast).
    /// `callback(ProxyId, userData, Real maxDistance)` → novo alcance máximo (< 0 para parar).
    template <typename Callback>
    void raycast(const Vector3& origin, const Vector3& direction, Real maxDistance, Real slack,
                 Callback&& callback) const {
        tree_.raycast(origin, direction, maxDistance, slack, [&](ProxyId id, Real maxDist) {
            return callback(id, tree_.getUserData(id), maxDist);
        });
    }

    const AABB& getFatAABB(ProxyId id) const { return tree_.getFatAABB(id); }
    std::uint32_t getUserData(ProxyId id) const { return tree_.getUserData(id); }
    int getProxyCount() const { return tree_.getProxyCount(); }
    std::size_t getMoveBufferSize() const { return moveBuffer_.size(); }
    const DynamicAABBTree& getTree() const { return tree_; }

    void clear();

private:
    void bufferMove(ProxyId id);

    DynamicAABBTree tree_;
    std::vector<ProxyId> moveBuffer_;   ///< pode conter IDs obsoletos (destruídos): filtrados em updatePairs
    std::vector<ProxyPair> pairs_;
    PairFilter filter_;
};

} // namespace physics
