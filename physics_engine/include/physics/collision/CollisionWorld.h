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
// CONSULTAS ESPACIAIS (raycast, overlap): usam a broadphase para podar e testam
// a geometria exata de cada candidato com a pose guardada do objeto. São const,
// não alteram nenhum estado da simulação (não afetam o determinismo) e não
// alocam. Ver setQueryPose()/setQuerySlack() para consultar poses mais novas
// que as da árvore (é o que o PhysicsWorld faz depois de cada passo).
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

#include "physics/collision/RayCast.h"
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

/// Acerto de um raio contra o mundo (coordenadas de mundo).
struct RaycastHit {
    BodyId body = kInvalidBodyId;   ///< objeto atingido
    Real distance = Real(0);        ///< distância ao longo da direção unitária
    Vector3 point;                  ///< ponto de impacto
    Vector3 normal;                 ///< normal da superfície, para fora do objeto
    bool startedInside = false;     ///< a origem já estava dentro do objeto (distance = 0, normal = −direção)
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

    /// Marca o objeto como ADORMECIDO (ou o acorda). Um par em que os DOIS objetos estão
    /// "congelados" (adormecidos ou imóveis) não roda a narrowphase: o manifold guardado
    /// permanece como está (com seus impulsos) e o par continua em getTouching(), para que a
    /// ilha de simulação possa ser acordada por ele. Id desconhecido: ignora.
    void setObjectAsleep(BodyId id, bool asleep) {
        if (hasObject(id)) objects_[id].asleep = asleep;
    }
    bool isObjectAsleep(BodyId id) const { return hasObject(id) && objects_[id].asleep; }

    /// Pares cuja narrowphase foi pulada no último detect() por estarem congelados.
    std::size_t getFrozenPairCount() const { return frozenPairs_; }

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

    // --- Consultas espaciais ----------------------------------------------------------------
    /// Pose que as consultas usam para o objeto, SEM mexer na broadphase nem nos
    /// pares. Serve a quem move os objetos depois de updateObject() (o PhysicsWorld
    /// a chama ao fim de cada passo, quando a integração já avançou os corpos).
    /// Combine com setQuerySlack(). Id desconhecido ou pose não finita: ignora.
    void setQueryPose(BodyId id, const Transform& transform) {
        if (hasObject(id) && transform.isFinite()) objects_[id].transform = transform;
    }

    /// Folga (>= 0) somada aos AABBs da árvore nas consultas: deve ser ao menos a
    /// distância máxima que qualquer ponto de um objeto andou desde a última
    /// atualização da sua folha (setQueryPose depois de updateObject). Padrão 0:
    /// correto para quem só usa updateObject.
    void setQuerySlack(Real slack) { querySlack_ = (slack >= Real(0) && isFinite(slack)) ? slack : Real(0); }
    Real getQuerySlack() const { return querySlack_; }

    /// Raio contra UM objeto (pose de consulta). `direction` unitária.
    bool raycastObject(BodyId id, const Vector3& origin, const Vector3& direction, Real maxDistance,
                       ShapeRayHit& hit) const {
        return hasObject(id) &&
               raycastShape(*objects_[id].shape, objects_[id].transform, origin, direction, maxDistance, hit);
    }

    /// A forma `shape` (em `xf`) intersecta ou toca o objeto `id`? (GJK exato.)
    bool overlapObject(BodyId id, const Shape& shape, const Transform& xf) const;

    /// O ponto está dentro do objeto (ou sobre a superfície)?
    bool containsPoint(BodyId id, const Vector3& point) const {
        return hasObject(id) && shapeContainsPoint(*objects_[id].shape, objects_[id].transform, point);
    }

    /// Acerto MAIS PRÓXIMO do raio (origem + t·direção, t ∈ [0, maxDistance]) entre os
    /// objetos que `accept(BodyId)` aceita. Empates de distância: menor BodyId.
    /// `direction` deve ser UNITÁRIA e `origin` finita (quem chama valida; o
    /// PhysicsWorld::raycast cuida disso). Retorna false se nada foi atingido.
    template <typename Filter>
    bool raycastClosest(const Vector3& origin, const Vector3& direction, Real maxDistance,
                        RaycastHit& hit, Filter&& accept) const {
        bool found = false;
        broadphase_.raycast(origin, direction, maxDistance, querySlack_,
                            [&](ProxyId, std::uint32_t userData, Real currentMax) -> Real {
            const BodyId id = static_cast<BodyId>(userData);
            if (!accept(id)) return currentMax;
            ShapeRayHit h;
            if (!raycastObject(id, origin, direction, currentMax, h)) return currentMax;
            if (!found || h.distance < hit.distance || (h.distance == hit.distance && id < hit.body)) {
                hit.body = id;
                hit.distance = h.distance;
                hit.point = h.point;
                hit.normal = h.normal;
                hit.startedInside = h.startedInside;
                found = true;
            }
            return h.distance;   // poda tudo além deste acerto (os empates ainda são visitados)
        });
        return found;
    }

    /// Todos os acertos do raio, em ORDEM DA ÁRVORE (determinística, não ordenada
    /// por distância). `callback(const RaycastHit&)` retorna true para continuar.
    template <typename Filter, typename Callback>
    void raycastAll(const Vector3& origin, const Vector3& direction, Real maxDistance, Filter&& accept,
                    Callback&& callback) const {
        broadphase_.raycast(origin, direction, maxDistance, querySlack_,
                            [&](ProxyId, std::uint32_t userData, Real currentMax) -> Real {
            const BodyId id = static_cast<BodyId>(userData);
            if (!accept(id)) return currentMax;
            ShapeRayHit h;
            if (!raycastObject(id, origin, direction, currentMax, h)) return currentMax;
            RaycastHit r;
            r.body = id;
            r.distance = h.distance;
            r.point = h.point;
            r.normal = h.normal;
            r.startedInside = h.startedInside;
            return callback(r) ? currentMax : Real(-1);
        });
    }

    /// Chama `callback(BodyId)` para cada objeto aceito que intersecta `shape` (em `xf`).
    /// Ordem da árvore (determinística). `callback` retorna true para continuar.
    template <typename Filter, typename Callback>
    void overlapShape(const Shape& shape, const Transform& xf, Filter&& accept, Callback&& callback) const {
        if (!xf.isFinite()) return;
        const AABB box = shape.computeAABB(xf);
        if (!box.isValid() || !box.isFinite()) return;
        broadphase_.query(box.fattened(querySlack_), [&](ProxyId, std::uint32_t userData) {
            const BodyId id = static_cast<BodyId>(userData);
            if (!accept(id) || !overlapObject(id, shape, xf)) return true;
            return static_cast<bool>(callback(id));
        });
    }

    /// Chama `callback(BodyId)` para cada objeto aceito que contém o ponto.
    template <typename Filter, typename Callback>
    void overlapPoint(const Vector3& point, Filter&& accept, Callback&& callback) const {
        if (!point.isFinite()) return;
        broadphase_.query(AABB(point, point).fattened(querySlack_), [&](ProxyId, std::uint32_t userData) {
            const BodyId id = static_cast<BodyId>(userData);
            if (!accept(id) || !containsPoint(id, point)) return true;
            return static_cast<bool>(callback(id));
        });
    }

    /// Fat AABB atual do objeto na broadphase (para debug). false se o id não existe.
    bool getFatAABB(BodyId id, AABB& out) const {
        if (!hasObject(id)) return false;
        out = broadphase_.getFatAABB(objects_[id].proxy);
        return true;
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
        bool asleep = false;
    };

    /// Imóvel (estático) ou adormecido: nada neste objeto muda até alguém o acordar.
    bool isFrozen(const Object& o) const { return !o.moving || o.asleep; }

    static std::uint64_t pairKey(BodyId a, BodyId b) {
        return (static_cast<std::uint64_t>(a) << 32) | static_cast<std::uint64_t>(b);
    }
    bool acceptPair(BodyId a, BodyId b) const;

    CollisionWorldSettings settings_;
    Broadphase broadphase_;
    std::vector<Object> objects_;                       ///< indexado por BodyId
    std::map<std::uint64_t, ContactPair> pairs_;        ///< ordenado: iteração determinística
    std::vector<ContactPair*> touching_;
    std::size_t frozenPairs_ = 0;
    PairFilter filter_;
    Real querySlack_ = Real(0);
};

} // namespace physics
