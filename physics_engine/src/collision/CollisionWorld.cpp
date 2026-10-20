// =============================================================================
// CollisionWorld.cpp
// =============================================================================
#include "physics/collision/CollisionWorld.h"

#include "physics/collision/narrowphase/GJK.h"

#include <stdexcept>
#include <utility>

namespace physics {

CollisionWorld::CollisionWorld(const CollisionWorldSettings& settings)
    : settings_(settings),
      broadphase_(settings.aabbMargin, settings.aabbDisplacementMultiplier, settings.initialCapacity) {
    broadphase_.setPairFilter(
        [this](std::uint32_t a, std::uint32_t b) { return acceptPair(static_cast<BodyId>(a), static_cast<BodyId>(b)); });
}

bool CollisionWorld::acceptPair(BodyId a, BodyId b) const {
    if (a == b || !hasObject(a) || !hasObject(b)) return false;
    // Dois objetos imóveis nunca colidem entre si.
    if (!objects_[a].moving && !objects_[b].moving) return false;
    return !filter_ || filter_(a, b);
}

void CollisionWorld::addObject(BodyId id, std::shared_ptr<const Shape> shape, const Transform& transform,
                               bool moving) {
    if (!shape) throw std::invalid_argument("CollisionWorld::addObject: forma nula");
    if (id == kInvalidBodyId) throw std::invalid_argument("CollisionWorld::addObject: id invalido");
    if (hasObject(id)) throw std::invalid_argument("CollisionWorld::addObject: id ja em uso");
    if (!transform.isFinite()) throw std::invalid_argument("CollisionWorld::addObject: transformacao nao finita");

    // Cria o proxy antes de tocar em objects_: createProxy lança se o AABB for inválido.
    const ProxyId proxy = broadphase_.createProxy(shape->computeAABB(transform), static_cast<std::uint32_t>(id));

    if (id >= objects_.size()) objects_.resize(static_cast<std::size_t>(id) + 1);
    Object& o = objects_[id];
    o.shape = std::move(shape);
    o.transform = transform;
    o.proxy = proxy;
    o.moving = moving;
    o.active = true;
}

void CollisionWorld::removeObject(BodyId id) {
    if (!hasObject(id)) return;
    // Tira o objeto da lista de contatos (antes de destruir os nós apontados)...
    for (std::size_t i = 0; i < touching_.size();) {
        if (touching_[i]->a == id || touching_[i]->b == id) {
            touching_[i] = touching_.back();
            touching_.pop_back();
        } else {
            ++i;
        }
    }
    // ...remove todos os pares do objeto ANTES de destruir o proxy (ids de proxy são reaproveitados).
    for (auto it = pairs_.begin(); it != pairs_.end();) {
        if (it->second.a == id || it->second.b == id) it = pairs_.erase(it);
        else ++it;
    }
    broadphase_.destroyProxy(objects_[id].proxy);
    objects_[id] = Object();
}

void CollisionWorld::updateObject(BodyId id, const Transform& transform, const Vector3& displacement) {
    if (!hasObject(id) || !transform.isFinite()) return;
    Object& o = objects_[id];
    o.transform = transform;
    broadphase_.moveProxy(o.proxy, o.shape->computeAABB(transform), displacement);
}

void CollisionWorld::detect() {
    // 1) Pares novos vindos da broadphase.
    broadphase_.updatePairs();
    for (const ProxyPair& p : broadphase_.getPairs()) {
        BodyId a = static_cast<BodyId>(p.userDataA);
        BodyId b = static_cast<BodyId>(p.userDataB);
        ProxyId pa = p.proxyA, pb = p.proxyB;
        if (a > b) {
            std::swap(a, b);
            std::swap(pa, pb);
        }
        const auto result = pairs_.try_emplace(pairKey(a, b));
        if (result.second) {
            ContactPair& cp = result.first->second;
            cp.a = a;
            cp.b = b;
            cp.proxyA = pa;
            cp.proxyB = pb;
            cp.manifold = ContactManifold(a, b);
        }
    }

    // 2) Descarta pares separados e roda a narrowphase nos demais.
    touching_.clear();
    frozenPairs_ = 0;
    for (auto it = pairs_.begin(); it != pairs_.end();) {
        ContactPair& cp = it->second;
        if (!broadphase_.testOverlap(cp.proxyA, cp.proxyB)) {
            it = pairs_.erase(it);
            continue;
        }
        const Object& A = objects_[cp.a];
        const Object& B = objects_[cp.b];
        if (isFrozen(A) && isFrozen(B)) {
            // Nada se moveu desde a última vez: o manifold continua válido (e com os impulsos).
            ++frozenPairs_;
            if (cp.manifold.count > 0) touching_.push_back(&cp);
            ++it;
            continue;
        }
        if (ContactGenerator::collide(*A.shape, A.transform, *B.shape, B.transform, cp.manifold,
                                      settings_.contact)) {
            touching_.push_back(&cp);
        }
        ++it;
    }
}

bool CollisionWorld::overlapObject(BodyId id, const Shape& shape, const Transform& xf) const {
    if (!hasObject(id) || !xf.isFinite()) return false;
    const Object& o = objects_[id];
    const MinkowskiSupport support(shape, xf, *o.shape, o.transform);
    const GJKResult r = GJK::compute(support);
    // Núcleos que se interpenetram: sobrepõem. Senão, esferas (núcleo + raio) ainda
    // se tocam se a distância entre os núcleos não passa da soma dos raios.
    if (r.intersecting) return true;
    // Sem convergência (caso degenerado): responde de forma conservadora.
    if (!r.converged) return true;
    return r.distance <= support.getRadiusA() + support.getRadiusB();
}

const ContactPair* CollisionWorld::findPair(BodyId a, BodyId b) const {
    if (a > b) std::swap(a, b);
    const auto it = pairs_.find(pairKey(a, b));
    return it == pairs_.end() ? nullptr : &it->second;
}

void CollisionWorld::removePair(BodyId a, BodyId b) {
    if (a > b) std::swap(a, b);
    // Primeiro tira da lista de contatos deste quadro (os ponteiros ainda são válidos)...
    for (std::size_t i = 0; i < touching_.size();) {
        if (touching_[i]->a == a && touching_[i]->b == b) {
            touching_[i] = touching_.back();
            touching_.pop_back();
        } else {
            ++i;
        }
    }
    // ...e só então destrói o nó do mapa.
    pairs_.erase(pairKey(a, b));
}

void CollisionWorld::clear() {
    pairs_.clear();
    touching_.clear();
    objects_.clear();
    broadphase_.clear();
}

} // namespace physics
