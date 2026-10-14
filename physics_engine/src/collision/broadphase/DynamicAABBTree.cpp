// =============================================================================
// DynamicAABBTree.cpp
// =============================================================================
#include "physics/collision/broadphase/DynamicAABBTree.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace physics {

namespace {
inline std::size_t idx(ProxyId id) { return static_cast<std::size_t>(id); }
} // namespace

DynamicAABBTree::DynamicAABBTree(Real margin, Real displacementMultiplier, int initialCapacity)
    : margin_(margin < Real(0) ? Real(0) : margin),
      displacementMultiplier_(displacementMultiplier < Real(0) ? Real(0) : displacementMultiplier) {
    const int capacity = std::max(initialCapacity, 2);
    nodes_.resize(static_cast<std::size_t>(capacity));
    // Encadeia todos os nós na lista livre.
    for (int i = 0; i < capacity - 1; ++i) nodes_[idx(i)].parent = i + 1;
    nodes_[idx(capacity - 1)].parent = kNullProxy;
    freeList_ = 0;
}

void DynamicAABBTree::clear() {
    const int capacity = static_cast<int>(nodes_.size());
    for (int i = 0; i < capacity; ++i) {
        TreeNode& n = nodes_[idx(i)];
        n = TreeNode();
        n.parent = (i + 1 < capacity) ? i + 1 : kNullProxy;
    }
    root_ = kNullProxy;
    freeList_ = 0;
    nodeCount_ = 0;
    proxyCount_ = 0;
}

ProxyId DynamicAABBTree::allocateNode() {
    if (freeList_ == kNullProxy) {
        // Capacidade esgotada: dobra. (Única alocação possível; índices continuam válidos.)
        const int oldCapacity = static_cast<int>(nodes_.size());
        const int newCapacity = oldCapacity * 2;
        nodes_.resize(static_cast<std::size_t>(newCapacity));
        for (int i = oldCapacity; i < newCapacity - 1; ++i) nodes_[idx(i)].parent = i + 1;
        nodes_[idx(newCapacity - 1)].parent = kNullProxy;
        freeList_ = oldCapacity;
    }
    const ProxyId id = freeList_;
    TreeNode& n = nodes_[idx(id)];
    freeList_ = n.parent;
    n = TreeNode();
    n.height = 0;
    ++nodeCount_;
    return id;
}

void DynamicAABBTree::freeNode(ProxyId id) {
    TreeNode& n = nodes_[idx(id)];
    n = TreeNode();
    n.parent = freeList_;   // height = -1 marca o nó como livre
    freeList_ = id;
    --nodeCount_;
}

ProxyId DynamicAABBTree::createProxy(const AABB& tight, std::uint32_t userData) {
    if (!tight.isValid() || !tight.isFinite()) {
        throw std::invalid_argument("DynamicAABBTree::createProxy: AABB invalido ou nao finito");
    }
    const ProxyId id = allocateNode();
    TreeNode& n = nodes_[idx(id)];
    n.aabb = tight.fattened(margin_);
    n.userData = userData;
    n.height = 0;
    insertLeaf(id);
    ++proxyCount_;
    return id;
}

void DynamicAABBTree::destroyProxy(ProxyId id) {
    if (!isProxy(id)) return;
    removeLeaf(id);
    freeNode(id);
    --proxyCount_;
}

bool DynamicAABBTree::moveProxy(ProxyId id, const AABB& tight, const Vector3& displacement) {
    if (!isProxy(id) || !tight.isValid() || !tight.isFinite()) return false;

    // Novo Fat AABB: margem + antecipação na direção do movimento.
    const AABB fat = tight.fattenedForMotion(displacement.isFinite() ? displacement : Vector3::zero(),
                                             margin_, displacementMultiplier_);

    const AABB& treeAABB = nodes_[idx(id)].aabb;
    if (treeAABB.contains(tight)) {
        // O Fat AABB ainda contém o objeto, mas pode estar grande demais (ex.:
        // objeto rápido que desacelerou). Se não for exageradamente maior que o
        // novo Fat AABB, mantém — a atualização custa O(1).
        const AABB huge = fat.fattened(Real(4) * margin_);
        if (huge.contains(treeAABB)) return false;
    }

    removeLeaf(id);
    nodes_[idx(id)].aabb = fat;
    insertLeaf(id);
    return true;
}

void DynamicAABBTree::insertLeaf(ProxyId leaf) {
    if (root_ == kNullProxy) {
        root_ = leaf;
        nodes_[idx(root_)].parent = kNullProxy;
        return;
    }

    // --- 1) Descida: escolhe o irmão que minimiza o aumento de área (SAH) ----------------------
    const AABB leafAABB = nodes_[idx(leaf)].aabb;
    ProxyId index = root_;
    while (!nodes_[idx(index)].isLeaf()) {
        const TreeNode& node = nodes_[idx(index)];
        const ProxyId child1 = node.child1;
        const ProxyId child2 = node.child2;

        const Real area = node.aabb.surfaceArea();
        const Real combinedArea = AABB::merge(node.aabb, leafAABB).surfaceArea();

        // Custo de criar um novo pai aqui (par {nó atual, folha nova}).
        const Real cost = Real(2) * combinedArea;
        // Custo mínimo herdado por descer: cada ancestral cresce.
        const Real inheritanceCost = Real(2) * (combinedArea - area);

        auto descendCost = [&](ProxyId child) {
            const TreeNode& c = nodes_[idx(child)];
            const Real newArea = AABB::merge(leafAABB, c.aabb).surfaceArea();
            return c.isLeaf() ? newArea + inheritanceCost
                              : (newArea - c.aabb.surfaceArea()) + inheritanceCost;
        };
        const Real cost1 = descendCost(child1);
        const Real cost2 = descendCost(child2);

        if (cost < cost1 && cost < cost2) break;
        index = (cost1 < cost2) ? child1 : child2;
    }
    const ProxyId sibling = index;

    // --- 2) Cria o novo pai -----------------------------------------------------------------------
    const ProxyId oldParent = nodes_[idx(sibling)].parent;
    const ProxyId newParent = allocateNode();   // pode realocar nodes_: não guardar referências antes
    {
        TreeNode& np = nodes_[idx(newParent)];
        np.parent = oldParent;
        np.aabb = AABB::merge(leafAABB, nodes_[idx(sibling)].aabb);
        np.height = nodes_[idx(sibling)].height + 1;
        np.child1 = sibling;
        np.child2 = leaf;
    }
    nodes_[idx(sibling)].parent = newParent;
    nodes_[idx(leaf)].parent = newParent;

    if (oldParent != kNullProxy) {
        TreeNode& op = nodes_[idx(oldParent)];
        if (op.child1 == sibling) op.child1 = newParent;
        else op.child2 = newParent;
    } else {
        root_ = newParent;
    }

    // --- 3) Sobe até a raiz atualizando AABBs/alturas e balanceando -------------------------------------
    index = nodes_[idx(leaf)].parent;
    while (index != kNullProxy) {
        index = balance(index);
        TreeNode& n = nodes_[idx(index)];
        const TreeNode& c1 = nodes_[idx(n.child1)];
        const TreeNode& c2 = nodes_[idx(n.child2)];
        n.height = 1 + std::max(c1.height, c2.height);
        n.aabb = AABB::merge(c1.aabb, c2.aabb);
        index = n.parent;
    }
}

void DynamicAABBTree::removeLeaf(ProxyId leaf) {
    if (leaf == root_) {
        root_ = kNullProxy;
        return;
    }

    const ProxyId parent = nodes_[idx(leaf)].parent;
    const ProxyId grandParent = nodes_[idx(parent)].parent;
    const ProxyId sibling = (nodes_[idx(parent)].child1 == leaf) ? nodes_[idx(parent)].child2
                                                                 : nodes_[idx(parent)].child1;

    if (grandParent != kNullProxy) {
        // O irmão assume o lugar do pai.
        TreeNode& gp = nodes_[idx(grandParent)];
        if (gp.child1 == parent) gp.child1 = sibling;
        else gp.child2 = sibling;
        nodes_[idx(sibling)].parent = grandParent;
        freeNode(parent);

        ProxyId index = grandParent;
        while (index != kNullProxy) {
            index = balance(index);
            TreeNode& n = nodes_[idx(index)];
            const TreeNode& c1 = nodes_[idx(n.child1)];
            const TreeNode& c2 = nodes_[idx(n.child2)];
            n.aabb = AABB::merge(c1.aabb, c2.aabb);
            n.height = 1 + std::max(c1.height, c2.height);
            index = n.parent;
        }
    } else {
        root_ = sibling;
        nodes_[idx(sibling)].parent = kNullProxy;
        freeNode(parent);
    }
    nodes_[idx(leaf)].parent = kNullProxy;
}

ProxyId DynamicAABBTree::balance(ProxyId iA) {
    // Rotações AVL (A é o nó possivelmente desbalanceado; B e C seus filhos).
    TreeNode& A = nodes_[idx(iA)];
    if (A.isLeaf() || A.height < 2) return iA;

    const ProxyId iB = A.child1;
    const ProxyId iC = A.child2;
    TreeNode& B = nodes_[idx(iB)];
    TreeNode& C = nodes_[idx(iC)];
    const int balanceFactor = C.height - B.height;

    // --- C é muito mais alto: sobe C ---------------------------------------------------------
    if (balanceFactor > 1) {
        const ProxyId iF = C.child1;
        const ProxyId iG = C.child2;
        TreeNode& F = nodes_[idx(iF)];
        TreeNode& G = nodes_[idx(iG)];

        // A vira filho esquerdo de C; C assume o lugar de A.
        C.child1 = iA;
        C.parent = A.parent;
        A.parent = iC;
        if (C.parent != kNullProxy) {
            TreeNode& p = nodes_[idx(C.parent)];
            if (p.child1 == iA) p.child1 = iC;
            else p.child2 = iC;
        } else {
            root_ = iC;
        }

        if (F.height > G.height) {
            C.child2 = iF;
            A.child2 = iG;
            G.parent = iA;
            A.aabb = AABB::merge(B.aabb, G.aabb);
            C.aabb = AABB::merge(A.aabb, F.aabb);
            A.height = 1 + std::max(B.height, G.height);
            C.height = 1 + std::max(A.height, F.height);
        } else {
            C.child2 = iG;
            A.child2 = iF;
            F.parent = iA;
            A.aabb = AABB::merge(B.aabb, F.aabb);
            C.aabb = AABB::merge(A.aabb, G.aabb);
            A.height = 1 + std::max(B.height, F.height);
            C.height = 1 + std::max(A.height, G.height);
        }
        return iC;
    }

    // --- B é muito mais alto: sobe B ---------------------------------------------------------
    if (balanceFactor < -1) {
        const ProxyId iD = B.child1;
        const ProxyId iE = B.child2;
        TreeNode& D = nodes_[idx(iD)];
        TreeNode& E = nodes_[idx(iE)];

        B.child1 = iA;
        B.parent = A.parent;
        A.parent = iB;
        if (B.parent != kNullProxy) {
            TreeNode& p = nodes_[idx(B.parent)];
            if (p.child1 == iA) p.child1 = iB;
            else p.child2 = iB;
        } else {
            root_ = iB;
        }

        if (D.height > E.height) {
            B.child2 = iD;
            A.child1 = iE;
            E.parent = iA;
            A.aabb = AABB::merge(C.aabb, E.aabb);
            B.aabb = AABB::merge(A.aabb, D.aabb);
            A.height = 1 + std::max(C.height, E.height);
            B.height = 1 + std::max(A.height, D.height);
        } else {
            B.child2 = iE;
            A.child1 = iD;
            D.parent = iA;
            A.aabb = AABB::merge(C.aabb, D.aabb);
            B.aabb = AABB::merge(A.aabb, E.aabb);
            A.height = 1 + std::max(C.height, D.height);
            B.height = 1 + std::max(A.height, E.height);
        }
        return iB;
    }
    return iA;
}

int DynamicAABBTree::getMaxBalance() const {
    int maxBalance = 0;
    for (const TreeNode& n : nodes_) {
        if (n.height <= 1) continue;  // livre, folha ou nó com duas folhas
        const int b = std::abs(nodes_[idx(n.child2)].height - nodes_[idx(n.child1)].height);
        maxBalance = std::max(maxBalance, b);
    }
    return maxBalance;
}

Real DynamicAABBTree::getAreaRatio() const {
    if (root_ == kNullProxy) return Real(0);
    const Real rootArea = nodes_[idx(root_)].aabb.surfaceArea();
    if (!(rootArea > Real(0))) return Real(0);
    Real total = Real(0);
    for (const TreeNode& n : nodes_) {
        if (n.height < 0 || n.isLeaf()) continue;  // só nós internos em uso
        total += n.aabb.surfaceArea();
    }
    return total / rootArea;
}

bool DynamicAABBTree::validate(std::string* error) const {
    auto fail = [&](const std::string& m) {
        if (error) *error = m;
        return false;
    };

    int reachable = 0;
    int leaves = 0;
    if (root_ != kNullProxy) {
        if (nodes_[idx(root_)].parent != kNullProxy) return fail("raiz com pai");
        // Percurso iterativo pré-ordem verificando cada nó.
        std::vector<ProxyId> stack{root_};
        while (!stack.empty()) {
            const ProxyId id = stack.back();
            stack.pop_back();
            const TreeNode& n = nodes_[idx(id)];
            ++reachable;
            if (n.height < 0) return fail("no livre alcancavel a partir da raiz");
            if (n.isLeaf()) {
                ++leaves;
                if (n.child2 != kNullProxy || n.height != 0) return fail("folha malformada");
                continue;
            }
            if (n.child1 < 0 || n.child2 < 0 ||
                n.child1 >= static_cast<ProxyId>(nodes_.size()) || n.child2 >= static_cast<ProxyId>(nodes_.size())) {
                return fail("indice de filho invalido");
            }
            const TreeNode& c1 = nodes_[idx(n.child1)];
            const TreeNode& c2 = nodes_[idx(n.child2)];
            if (c1.parent != id || c2.parent != id) return fail("ponteiro de pai inconsistente");
            if (n.height != 1 + std::max(c1.height, c2.height)) return fail("altura incorreta");
            const AABB u = AABB::merge(c1.aabb, c2.aabb);
            if (!(u.min == n.aabb.min) || !(u.max == n.aabb.max)) return fail("AABB do no interno != uniao dos filhos");
            stack.push_back(n.child1);
            stack.push_back(n.child2);
        }
    }
    if (reachable != nodeCount_) return fail("contagem de nos em uso != nos alcancaveis");
    if (leaves != proxyCount_) return fail("contagem de proxies incorreta");

    // Lista livre: nós livres + em uso = capacidade.
    int freeCount = 0;
    for (ProxyId f = freeList_; f != kNullProxy; f = nodes_[idx(f)].parent) {
        if (nodes_[idx(f)].height != -1) return fail("no na lista livre nao esta marcado como livre");
        if (++freeCount > static_cast<int>(nodes_.size())) return fail("ciclo na lista livre");
    }
    if (freeCount + nodeCount_ != static_cast<int>(nodes_.size())) return fail("vazamento de nos (livres + usados != capacidade)");
    return true;
}

} // namespace physics
