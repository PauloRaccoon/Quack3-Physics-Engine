// =============================================================================
// Broadphase.cpp
// =============================================================================
#include "physics/collision/broadphase/Broadphase.h"

#include <algorithm>

namespace physics {

Broadphase::Broadphase(Real margin, Real displacementMultiplier, int initialCapacity)
    : tree_(margin, displacementMultiplier, initialCapacity) {
    moveBuffer_.reserve(static_cast<std::size_t>(std::max(initialCapacity, 16)));
    pairs_.reserve(static_cast<std::size_t>(std::max(initialCapacity, 16)) * 2);
}

void Broadphase::bufferMove(ProxyId id) {
    if (tree_.isMoved(id)) return;   // já está no buffer
    tree_.setMoved(id, true);
    moveBuffer_.push_back(id);
}

ProxyId Broadphase::createProxy(const AABB& tightAABB, std::uint32_t userData) {
    const ProxyId id = tree_.createProxy(tightAABB, userData);
    bufferMove(id);
    return id;
}

void Broadphase::destroyProxy(ProxyId id) {
    if (!tree_.isProxy(id)) return;
    // O ID permanece no moveBuffer_ (remoção O(1)); updatePairs() ignora IDs
    // que não são mais proxies ou que já foram processados.
    tree_.destroyProxy(id);
}

bool Broadphase::moveProxy(ProxyId id, const AABB& tightAABB, const Vector3& displacement) {
    const bool changed = tree_.moveProxy(id, tightAABB, displacement);
    if (changed) bufferMove(id);
    return changed;
}

void Broadphase::touchProxy(ProxyId id) {
    if (tree_.isProxy(id)) bufferMove(id);
}

void Broadphase::updatePairs() {
    pairs_.clear();

    // Consulta a árvore com o Fat AABB de cada proxy movido.
    for (const ProxyId queryId : moveBuffer_) {
        if (!tree_.isProxy(queryId)) continue;   // destruído depois de entrar no buffer

        const AABB& fat = tree_.getFatAABB(queryId);
        tree_.query(fat, [&](ProxyId otherId) {
            if (otherId == queryId) return true;                 // não pareia consigo mesmo
            // Se ambos se moveram, o par seria gerado duas vezes (A→B e B→A):
            // só o proxy de MENOR id o emite.
            if (tree_.isMoved(otherId) && otherId < queryId) return true;

            const std::uint32_t dataQ = tree_.getUserData(queryId);
            const std::uint32_t dataO = tree_.getUserData(otherId);
            if (filter_ && !filter_(dataQ, dataO)) return true;

            ProxyPair p;
            if (queryId < otherId) {
                p.proxyA = queryId; p.proxyB = otherId; p.userDataA = dataQ; p.userDataB = dataO;
            } else {
                p.proxyA = otherId; p.proxyB = queryId; p.userDataA = dataO; p.userDataB = dataQ;
            }
            pairs_.push_back(p);
            return true;
        });
    }

    // Limpa as marcas e esvazia o buffer.
    for (const ProxyId id : moveBuffer_) {
        if (tree_.isProxy(id)) tree_.setMoved(id, false);
    }
    moveBuffer_.clear();

    // Ordena e remove duplicatas (IDs repetidos no buffer ou reutilizados).
    std::sort(pairs_.begin(), pairs_.end());
    pairs_.erase(std::unique(pairs_.begin(), pairs_.end()), pairs_.end());
}

void Broadphase::clear() {
    tree_.clear();
    moveBuffer_.clear();
    pairs_.clear();
}

} // namespace physics
