// =============================================================================
// DynamicAABBTree.h
// Árvore BVH dinâmica de AABBs (estilo Box2D): cada folha guarda um "Fat AABB"
// de um objeto; nós internos guardam a união dos filhos.
//
//  - Inserção: descida guiada pelo custo SAH (área de superfície), O(log N).
//  - Balanceamento: rotações do tipo AVL após cada inserção/remoção.
//  - Atualização: moveProxy() só mexe na árvore se o AABB justo escapou do
//    Fat AABB; caso contrário é O(1) e não custa nada.
//  - Consulta: percorre com pilha local (sem alocação em árvores normais).
//
// Os nós ficam em um std::vector contíguo e são referenciados por índice
// (ProxyId); a lista livre reaproveita nós destruídos, então o laço de
// simulação não aloca memória (só há realocação quando a capacidade dobra).
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "physics/common/Settings.h"
#include "physics/common/Types.h"
#include "physics/geometry/AABB.h"
#include "physics/math/Vector3.h"

namespace physics {

struct TreeNode {
    AABB aabb;                           ///< Fat AABB (folha) ou união dos filhos
    ProxyId parent = kNullProxy;         ///< pai; em nós livres, é o "próximo" da lista livre
    ProxyId child1 = kNullProxy;
    ProxyId child2 = kNullProxy;
    std::int32_t height = -1;            ///< -1 = livre, 0 = folha
    std::uint32_t userData = 0;          ///< dado do cliente (ex.: BodyId) — só em folhas
    bool moved = false;                  ///< "está no move buffer" (usado pela Broadphase)

    bool isLeaf() const { return child1 == kNullProxy; }
};

namespace detail {
/// Pilha LIFO com 128 posições inline; só aloca no heap se a árvore for
/// anormalmente profunda.
class NodeStack {
public:
    void push(ProxyId id) {
        if (count_ < kInline) inline_[count_++] = id;
        else heap_.push_back(id);
    }
    ProxyId pop() {
        if (!heap_.empty()) {
            const ProxyId id = heap_.back();
            heap_.pop_back();
            return id;
        }
        return inline_[--count_];
    }
    bool empty() const { return count_ == 0 && heap_.empty(); }

private:
    static constexpr int kInline = 128;
    ProxyId inline_[kInline];
    int count_ = 0;
    std::vector<ProxyId> heap_;
};

/// Pilha LIFO de (nó, distância de entrada do raio), usada por DynamicAABBTree::raycast.
/// Mesmo esquema da NodeStack: 128 posições inline, heap só para árvores anormalmente profundas.
class RayStack {
public:
    struct Item {
        ProxyId id;
        Real t;
    };
    void push(ProxyId id, Real t) {
        if (count_ < kInline) inline_[count_++] = Item{id, t};
        else heap_.push_back(Item{id, t});
    }
    Item pop() {
        if (!heap_.empty()) {
            const Item it = heap_.back();
            heap_.pop_back();
            return it;
        }
        return inline_[--count_];
    }
    bool empty() const { return count_ == 0 && heap_.empty(); }

private:
    static constexpr int kInline = 128;
    Item inline_[kInline];
    int count_ = 0;
    std::vector<Item> heap_;
};
} // namespace detail

class DynamicAABBTree {
public:
    /// @param margin                 margem fixa dos Fat AABBs
    /// @param displacementMultiplier antecipação do movimento (ver AABB::fattenedForMotion)
    /// @param initialCapacity        nós pré-alocados (cresce ×2 se necessário)
    explicit DynamicAABBTree(Real margin = kAABBMargin,
                             Real displacementMultiplier = kAABBDisplacementMultiplier,
                             int initialCapacity = 64);

    // --- Proxies ---------------------------------------------------------------------
    /// Insere um objeto com AABB justo `tight` (será engordado pela margem).
    /// @throws std::invalid_argument se o AABB for inválido ou não finito.
    ProxyId createProxy(const AABB& tight, std::uint32_t userData);

    /// Remove um proxy. IDs inválidos são ignorados.
    void destroyProxy(ProxyId id);

    /// Atualiza o AABB justo de um proxy. Retorna true se a árvore foi
    /// modificada (o objeto escapou do Fat AABB, ou este ficou grande demais)
    /// e false se o Fat AABB atual ainda serve (custo O(1)).
    bool moveProxy(ProxyId id, const AABB& tight, const Vector3& displacement);

    void clear();

    // --- Acesso -------------------------------------------------------------------------
    bool isProxy(ProxyId id) const {
        return id >= 0 && id < static_cast<ProxyId>(nodes_.size()) &&
               nodes_[static_cast<std::size_t>(id)].height == 0;
    }
    const AABB& getFatAABB(ProxyId id) const { return nodes_[static_cast<std::size_t>(id)].aabb; }
    std::uint32_t getUserData(ProxyId id) const { return nodes_[static_cast<std::size_t>(id)].userData; }
    bool isMoved(ProxyId id) const { return nodes_[static_cast<std::size_t>(id)].moved; }
    void setMoved(ProxyId id, bool moved) { nodes_[static_cast<std::size_t>(id)].moved = moved; }

    int getProxyCount() const { return proxyCount_; }
    Real getMargin() const { return margin_; }

    // --- Consulta -----------------------------------------------------------------------------
    /// Chama `callback(ProxyId)` para cada folha cujo Fat AABB intersecta `box`.
    /// O callback retorna true para continuar ou false para parar. NÃO pode
    /// modificar a árvore durante a consulta.
    template <typename Callback>
    void query(const AABB& box, Callback&& callback) const {
        if (root_ == kNullProxy) return;
        detail::NodeStack stack;
        stack.push(root_);
        while (!stack.empty()) {
            const ProxyId id = stack.pop();
            const TreeNode& node = nodes_[static_cast<std::size_t>(id)];
            if (!node.aabb.overlaps(box)) continue;
            if (node.isLeaf()) {
                if (!callback(id)) return;
            } else {
                stack.push(node.child1);
                stack.push(node.child2);
            }
        }
    }

    /// Raio (origem + t·direção, t ∈ [0, maxDistance]) contra os Fat AABBs, do mais
    /// próximo para o mais distante (o filho com menor distância de entrada é
    /// visitado primeiro). `direction` deve ser UNITÁRIA e `origin` finita.
    /// `slack` (>= 0) infla cada AABB antes do teste: serve a quem consulta com
    /// poses mais novas que as da árvore (ver CollisionWorld::setQuerySlack).
    ///
    /// `callback(ProxyId, Real maxDistance)` é chamado para cada folha cujo AABB
    /// o raio toca dentro do alcance atual e devolve o NOVO alcance máximo:
    ///   - o mesmo `maxDistance` para continuar sem alterar;
    ///   - um valor menor para encurtar o raio (típico de "acertou a distância t":
    ///     poda tudo que está além);
    ///   - um valor NEGATIVO para parar a travessia.
    /// NÃO pode modificar a árvore durante a consulta.
    template <typename Callback>
    void raycast(const Vector3& origin, const Vector3& direction, Real maxDistance, Real slack,
                 Callback&& callback) const {
        if (root_ == kNullProxy || !(maxDistance >= Real(0))) return;
        detail::RayStack stack;
        Real t;
        if (!nodes_[static_cast<std::size_t>(root_)].aabb.fattened(slack).intersectsRay(origin, direction, maxDistance, t))
            return;
        stack.push(root_, t);
        while (!stack.empty()) {
            const detail::RayStack::Item item = stack.pop();
            if (item.t > maxDistance) continue;   // o alcance encolheu depois de empilhar
            const TreeNode& node = nodes_[static_cast<std::size_t>(item.id)];
            if (node.isLeaf()) {
                const Real next = callback(item.id, maxDistance);
                if (next < Real(0)) return;
                if (next < maxDistance) maxDistance = next;
            } else {
                Real t1 = Real(0), t2 = Real(0);
                const bool h1 = nodes_[static_cast<std::size_t>(node.child1)].aabb.fattened(slack)
                                    .intersectsRay(origin, direction, maxDistance, t1);
                const bool h2 = nodes_[static_cast<std::size_t>(node.child2)].aabb.fattened(slack)
                                    .intersectsRay(origin, direction, maxDistance, t2);
                // O mais próximo é empilhado por último (sai primeiro).
                if (h1 && h2) {
                    if (t1 < t2) { stack.push(node.child2, t2); stack.push(node.child1, t1); }
                    else         { stack.push(node.child1, t1); stack.push(node.child2, t2); }
                } else if (h1) {
                    stack.push(node.child1, t1);
                } else if (h2) {
                    stack.push(node.child2, t2);
                }
            }
        }
    }

    // --- Diagnóstico ---------------------------------------------------------------------------------
    /// Altura da árvore (0 se vazia ou só uma folha).
    int getHeight() const { return root_ == kNullProxy ? 0 : nodes_[static_cast<std::size_t>(root_)].height; }
    /// Maior diferença de altura entre irmãos (≤ 1 numa árvore perfeitamente balanceada).
    int getMaxBalance() const;
    /// Soma das áreas dos nós internos / área da raiz: métrica de qualidade (menor = melhor).
    Real getAreaRatio() const;
    /// Verifica todos os invariantes (ponteiros, alturas, AABBs, lista livre).
    bool validate(std::string* error = nullptr) const;

private:
    ProxyId allocateNode();
    void freeNode(ProxyId id);
    void insertLeaf(ProxyId leaf);
    void removeLeaf(ProxyId leaf);
    ProxyId balance(ProxyId index);

    std::vector<TreeNode> nodes_;
    ProxyId root_ = kNullProxy;
    ProxyId freeList_ = kNullProxy;
    int nodeCount_ = 0;    ///< nós em uso (folhas + internos)
    int proxyCount_ = 0;
    Real margin_;
    Real displacementMultiplier_;
};

} // namespace physics
