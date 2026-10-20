// =============================================================================
// PhysicsWorld.h
// Ponto de entrada do motor: possui os corpos, as juntas, o CollisionWorld e o
// solver, e executa o pipeline por passo FIXO.
//
// Uso típico (laço do renderizador, FPS variável):
//
//     PhysicsWorld world;                       // gravidade, Δt = 1/60 s...
//     world.setBridge(&bridge);                 // opcional: recebe poses interpoladas
//     BodyId chao = world.createBody(descChao);
//     BodyId caixa = world.createBody(descCaixa);
//     while (janelaAberta) {
//         world.update(frameTimeDoRender);      // N passos fixos + sincroniza a bridge
//         desenha(bridge);                      // poses interpoladas
//     }
//
// Pipeline de CADA passo fixo (Δt):
//   1. colisão   : atualiza os Fat AABBs → broadphase → narrowphase (manifolds
//                  persistentes, com warm starting; pares de corpos adormecidos são pulados)
//   2. ilhas     : agrupa corpos ligados por contatos/juntas; acorda a ilha inteira se algum
//                  membro está acordado (ou toca um corpo cinemático em movimento)
//   3. forças    : Integrator::integrateVelocities (gravidade, forças, damping)
//   4. restrições: SequentialImpulseSolver (contatos + juntas)
//   5. posições  : Integrator::integratePositions (Euler semi-implícito)
//   6. render    : MotionState::push (pose anterior/atual para interpolar)
//   7. sleeping  : ilhas que ficaram em repouso por settings.sleep.timeToSleep adormecem
//   8. eventos   : início/fim de contato (ver ContactEvents.h), entregues ao ouvinte
//
// SLEEPING: um corpo dinâmico dorme quando |v| e |ω| ficam abaixo dos limiares POR TODO o
// tempo timeToSleep, e SÓ junto com toda a sua ilha (corpos ligados por contato ou junta;
// estáticos e cinemáticos não ligam ilhas). Dormindo ele não é integrado, não entra no solver e
// seu par com outro corpo adormecido/estático não roda a narrowphase. Acordam: força, impulso,
// velocidade não nula, setBodyTransform, mudança de massa ou gravityScale, setGravity, destruição
// de um corpo que o apoiava, criação/remoção de junta, e contato com um corpo acordado ou
// cinemático em movimento. Pose, velocidades (= 0) e resultados das consultas não mudam ao dormir.
//
// Consultas espaciais (raycast, overlap): enxergam a pose ATUAL dos corpos e não
// alteram nenhum estado do mundo (são const; não afetam o determinismo). Para
// mover um corpo "à mão" use setBodyTransform(): mexer na pose por
// getBody().setTransform() só chega às consultas depois do próximo passo.
//
// Ordenação e iteração são DETERMINÍSTICAS: dois mundos construídos com as
// mesmas chamadas produzem resultados idênticos bit a bit (mesma máquina e
// compilador).
//
// Corpos ficam num std::vector pré-alocado (capacidade maxBodies, fixa): os
// ponteiros RigidBody* usados pelas juntas e pelo solver nunca são invalidados.
// Criar corpos além da capacidade lança std::length_error. Um BodyId destruído
// é REAPROVEITADO por um corpo criado depois.
//
// Não é thread-safe.
// =============================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "physics/bridge/IPhysicsTransformBridge.h"
#include "physics/bridge/MotionState.h"
#include "physics/collision/CollisionWorld.h"
#include "physics/pipeline/ContactEvents.h"
#include "physics/constraints/SequentialImpulseSolver.h"
#include "physics/dynamics/Integrator.h"
#include "physics/dynamics/RigidBody.h"

namespace physics {

/// Limiares para um corpo dinâmico adormecer (ver o cabeçalho).
struct SleepSettings {
    bool enabled = true;
    Real linearThreshold = Real(0.03);    ///< |v| abaixo disto conta como repouso [m/s]
    Real angularThreshold = Real(0.05);   ///< |ω| abaixo disto conta como repouso [rad/s]
    Real timeToSleep = Real(0.5);         ///< repouso contínuo necessário (da ilha toda) [s]
};

struct PhysicsWorldSettings {
    Vector3 gravity = Vector3(0, -kGravity, 0);
    Real fixedTimeStep = kFixedTimeStep;    ///< Δt da simulação [s]
    int maxSubSteps = kMaxSubSteps;         ///< máximo de passos por update() (anti "espiral da morte")
    std::size_t maxBodies = 4096;           ///< capacidade fixa de corpos
    SolverSettings solver;
    IntegratorSettings integrator;
    CollisionWorldSettings collision;
    SleepSettings sleep;
    /// Mede o tempo de cada fase do passo (std::chrono::steady_clock) em WorldStats::timings.
    /// Desligado por padrão: não custa nada.
    bool enableProfiling = false;
};

/// Duração das fases do ÚLTIMO passo fixo, em microssegundos (0 se o perfilamento está desligado).
struct StepTimings {
    double collisionUs = 0;     ///< atualização dos AABBs + broadphase + narrowphase
    double islandsUs = 0;       ///< ilhas e propagação de "acordar"
    double constraintsUs = 0;   ///< forças + preparação e iterações do solver
    double integrationUs = 0;   ///< posições, poses de render, poses de consulta
    double sleepEventsUs = 0;   ///< decisão de adormecer + eventos de contato
    double totalUs = 0;
};

struct WorldStats {
    int bodies = 0;
    int awakeBodies = 0;      ///< dinâmicos acordados
    int sleepingBodies = 0;   ///< dinâmicos adormecidos
    int islands = 0;          ///< ilhas que continuam ACORDADAS ao fim do último passo
    int joints = 0;
    int pairs = 0;            ///< pares com Fat AABBs sobrepostos
    int frozenPairs = 0;      ///< pares cuja narrowphase foi pulada no último passo (ambos adormecidos/imóveis)
    int touchingPairs = 0;    ///< pares com pontos de contato
    int contactPoints = 0;
    int solverContacts = 0;   ///< contatos que o solver resolveu no último passo (corpos adormecidos não entram)
    int solverJoints = 0;     ///< juntas que o solver resolveu no último passo
    std::uint64_t stepCount = 0;
    StepTimings timings;
};

/// Filtro das consultas espaciais (raycast, overlap). Tudo é aceito por padrão.
struct QueryFilter {
    bool includeStatic = true;
    bool includeKinematic = true;
    bool includeDynamic = true;
    BodyId ignoreBody = kInvalidBodyId;          ///< corpo a ignorar (ex.: o que lança o raio)
    std::function<bool(BodyId)> predicate;       ///< opcional: false descarta o corpo (camadas, grupos...)
};

class PhysicsWorld {
public:
    /// @throws std::invalid_argument para Δt inválido, maxSubSteps < 1 ou maxBodies == 0.
    explicit PhysicsWorld(const PhysicsWorldSettings& settings = PhysicsWorldSettings());
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // --- Corpos -----------------------------------------------------------------------------
    /// @throws std::invalid_argument se a descrição for inválida (nenhum slot é consumido);
    /// @throws std::length_error se maxBodies já foi atingido.
    BodyId createBody(const RigidBodyDesc& desc);

    /// Destrói o corpo e as juntas ligadas a ele. BodyId inválido: ignora.
    void destroyBody(BodyId id);

    bool isValidBody(BodyId id) const { return id < alive_.size() && alive_[id] != 0; }

    /// @throws std::out_of_range se o id não for de um corpo vivo.
    RigidBody& getBody(BodyId id);
    const RigidBody& getBody(BodyId id) const;

    /// "Teletransporte" do corpo pelo frame: a interpolação não deixa rastro
    /// (o render vê a nova pose imediatamente). As velocidades não mudam.
    void setBodyTransform(BodyId id, const Transform& frame);

    // --- Juntas (o mundo é o dono) ---------------------------------------------------------------
    // A decisão "colisão entre os corpos ligados" (getCollideConnected, padrão
    // false) é tomada NO MOMENTO da criação da junta.
    // @throws std::invalid_argument para ids inválidos, corpos iguais ou parâmetros inválidos.
    BallSocketJoint& createBallSocketJoint(BodyId a, BodyId b, const Vector3& worldAnchor,
                                           bool collideConnected = false);
    DistanceJoint& createDistanceJoint(BodyId a, BodyId b, const Vector3& worldAnchorA,
                                       const Vector3& worldAnchorB, Real length = Real(-1),
                                       bool collideConnected = false);
    HingeJoint& createHingeJoint(BodyId a, BodyId b, const Vector3& worldAnchor, const Vector3& worldAxis,
                                 bool collideConnected = false);

    /// Destrói uma junta criada por este mundo (referência desconhecida: ignora).
    void destroyJoint(const JointConstraint& joint);

    // --- Simulação ------------------------------------------------------------------------------------
    /// Avança o mundo `frameTime` segundos de tempo de RENDER: executa 0..maxSubSteps
    /// passos fixos, atualiza α e sincroniza a bridge (se houver). Retorna o nº de passos.
    int update(Real frameTime);

    /// Um passo fixo, sem acumulador nem interpolação (uso determinístico/testes).
    /// Não altera α nem sincroniza a bridge.
    /// @throws std::logic_error se chamado de dentro de um callback de contato (reentrância).
    void step();

    // --- Sleeping ---------------------------------------------------------------------------------
    /// Corpo dinâmico acordado? (cinemático: sempre true; estático: sempre false.)
    /// @throws std::out_of_range se o id não for de um corpo vivo.
    bool isAwake(BodyId id) const { return getBody(id).isAwake(); }

    /// true: acorda o corpo (a ilha dele acorda junto no próximo passo).
    /// false: adormece o corpo AGORA (zera velocidades). Atenção: se a ilha dele tem algum corpo
    /// acordado (ex.: ele está apoiado em um corpo em movimento), o passo seguinte o acorda de novo.
    /// Só corpos dinâmicos. Id inválido: ignora.
    void setAwake(BodyId id, bool awake);

    /// Acorda todos os corpos dinâmicos.
    void wakeAll();

    SleepSettings& sleepSettings() { return sleep_; }
    const SleepSettings& sleepSettings() const { return sleep_; }

    // --- Eventos de contato --------------------------------------------------------------------------
    /// Ouvinte (não é de propriedade do mundo; deve sobreviver a ele ou ser desligado com nullptr).
    void setContactListener(IContactListener* listener) { listener_ = listener; }

    /// Eventos do último update()/step() (para update(): todos os passos executados), em ordem.
    /// Esvaziado no início do próximo update()/step().
    const std::vector<ContactEvent>& getContactEvents() const { return events_; }
    void clearContactEvents() {
        if (dispatching_) return;
        events_.clear();
        dispatched_ = 0;
    }

    // --- Consultas espaciais ---------------------------------------------------------------------------
    // Entradas inválidas (origem/direção não finitas, direção nula, alcance NaN ou
    // negativo, forma/pose não finita) resultam em "nada encontrado", nunca em exceção.

    /// Raio de `origin` na `direction` (qualquer módulo; é normalizada): acerto MAIS
    /// PRÓXIMO até `maxDistance` (metros; +infinito é aceito). Origem dentro de um
    /// corpo: acerto com distance = 0 e startedInside = true. Empate de distância:
    /// menor BodyId. Retorna false se nada foi atingido.
    bool raycast(const Vector3& origin, const Vector3& direction, Real maxDistance, RaycastHit& hit,
                 const QueryFilter& filter = QueryFilter()) const;

    /// Todos os acertos do raio, ordenados por distância (empate: BodyId). `hits` é
    /// esvaziado antes; passe um vetor com capacidade reservada para não alocar.
    /// Retorna o nº de acertos.
    int raycastAll(const Vector3& origin, const Vector3& direction, Real maxDistance,
                   std::vector<RaycastHit>& hits, const QueryFilter& filter = QueryFilter()) const;

    /// Corpos que intersectam (ou tocam) `shape` posicionada em `xf` — teste exato por
    /// GJK, não só de AABB. `bodies` é esvaziado e preenchido em ordem crescente de id.
    int overlapShape(const Shape& shape, const Transform& xf, std::vector<BodyId>& bodies,
                     const QueryFilter& filter = QueryFilter()) const;

    /// Corpos que contêm o ponto (ou o têm na superfície), em ordem crescente de id.
    int overlapPoint(const Vector3& point, std::vector<BodyId>& bodies,
                     const QueryFilter& filter = QueryFilter()) const;

    /// Atalho de overlapShape para uma esfera (raio fora de [kMinShapeExtent, kMaxShapeExtent]: retorna 0).
    int overlapSphere(const Vector3& center, Real radius, std::vector<BodyId>& bodies,
                      const QueryFilter& filter = QueryFilter()) const;

    // --- Ponte com o renderizador ----------------------------------------------------------------------------
    /// Liga (ou desliga, com nullptr) a bridge. Ao ligar, ela recebe onBodyCreated
    /// de todos os corpos vivos. A bridge NÃO é de propriedade do mundo e deve
    /// sobreviver a ele (ou ser desligada antes).
    void setBridge(IPhysicsTransformBridge* bridge);

    /// Empurra as poses interpoladas (com o α atual) para a bridge: todos os
    /// corpos não estáticos, e os estáticos só ao criar/teletransportar.
    void syncBridge();

    /// Pose de render do corpo: interpolação com o α atual.
    Transform getInterpolatedTransform(BodyId id) const;
    const MotionState& getMotionState(BodyId id) const;
    Real getAlpha() const { return accumulator_.getAlpha(); }
    const FixedTimestepAccumulator& getAccumulator() const { return accumulator_; }

    // --- Configuração e consultas -------------------------------------------------------------------------------------
    const Vector3& getGravity() const { return gravity_; }
    /// Mudar a gravidade acorda todos os corpos.
    void setGravity(const Vector3& g) {
        if (!g.isFinite()) return;
        gravity_ = g;
        wakeAll();
    }
    Real getFixedTimeStep() const { return accumulator_.getFixedDt(); }
    SolverSettings& solverSettings() { return solver_.settings(); }
    IntegratorSettings& integratorSettings() { return integrator_; }

    /// Filtro adicional de colisão entre corpos (camadas, grupos...). Vale para
    /// pares que nascem DEPOIS; não afeta pares já existentes.
    void setCollisionFilter(CollisionWorld::PairFilter filter) { userFilter_ = std::move(filter); }

    /// Pares em contato após o último passo (para eventos, som, debug).
    const std::vector<ContactPair*>& getContacts() const { return collision_.getTouching(); }
    const CollisionWorld& getCollisionWorld() const { return collision_; }

    WorldStats getStats() const;

    /// Percorre os corpos vivos em ordem de id: f(BodyId, RigidBody&).
    template <typename F>
    void forEachBody(F&& f) {
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            if (alive_[i]) f(static_cast<BodyId>(i), bodies_[i]);
        }
    }

    /// Versão somente-leitura: f(BodyId, const RigidBody&).
    template <typename F>
    void forEachBody(F&& f) const {
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            if (alive_[i]) f(static_cast<BodyId>(i), bodies_[i]);
        }
    }

    /// Percorre as juntas na ordem de criação: f(const JointConstraint&, BodyId a, BodyId b).
    template <typename F>
    void forEachJoint(F&& f) const {
        for (const JointEntry& e : joints_) f(*e.joint, e.a, e.b);
    }

private:
    struct JointEntry {
        std::unique_ptr<JointConstraint> joint;
        BodyId a = kInvalidBodyId;
        BodyId b = kInvalidBodyId;
        bool noCollide = false;
    };

    static std::uint64_t pairKey(BodyId a, BodyId b) {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(a) << 32) | b;
    }
    bool acceptPair(BodyId a, BodyId b) const;
    template <typename J, typename... Args>
    J& emplaceJoint(BodyId a, BodyId b, bool collideConnected, Args&&... args);
    void removeJointEntry(std::size_t index);
    void requireBody(BodyId id, const char* what) const;
    void stepImpl(Real dt);
    bool acceptForQuery(BodyId id, const QueryFilter& filter) const;

    // Sleeping
    std::uint32_t findRoot(std::uint32_t i);
    void updateIslands();
    void updateSleep(Real dt);
    void wakeTouching(BodyId id);
    bool isActiveBody(const RigidBody& b) const { return b.isDynamic() && b.isAwake(); }
    static bool isMovingKinematic(const RigidBody& b) {
        return b.isKinematic() && (b.getLinearVelocity().lengthSquared() > Real(0) ||
                                   b.getAngularVelocity().lengthSquared() > Real(0));
    }

    // Eventos de contato
    struct TrackedContact {
        std::uint64_t key = 0;
        ContactEvent last;
    };
    void collectContactEvents();
    void dispatchEvents();

    Vector3 gravity_;
    IntegratorSettings integrator_;
    FixedTimestepAccumulator accumulator_;
    CollisionWorld collision_;
    SequentialImpulseSolver solver_;

    std::size_t maxBodies_;
    std::vector<RigidBody> bodies_;          ///< capacidade reservada: endereços estáveis
    std::vector<MotionState> motion_;
    std::vector<std::uint8_t> alive_;
    std::vector<std::uint8_t> dirty_;        ///< estático criado/teletransportado: ainda não enviado à bridge
    std::vector<BodyId> freeIds_;

    std::vector<JointEntry> joints_;
    std::unordered_map<std::uint64_t, int> noCollidePairs_;   ///< pares ligados por junta sem colisão

    CollisionWorld::PairFilter userFilter_;
    IPhysicsTransformBridge* bridge_ = nullptr;
    std::uint64_t stepCount_ = 0;

    SleepSettings sleep_;
    bool profiling_ = false;
    StepTimings timings_;
    int lastIslands_ = 0;
    std::vector<std::uint32_t> islandParent_;    ///< union-find por índice de corpo
    std::vector<Real> islandMinSleep_;           ///< menor sleepTime de cada ilha (pela raiz)
    std::vector<std::uint8_t> islandAwake_;      ///< a ilha (pela raiz) tem algum membro acordado?

    IContactListener* listener_ = nullptr;
    std::vector<ContactEvent> events_;
    std::size_t dispatched_ = 0;
    bool dispatching_ = false;
    bool stepping_ = false;
    std::vector<TrackedContact> prevContacts_;   ///< contatos do passo anterior, ordenados por chave
    std::vector<TrackedContact> curContacts_;
    std::vector<Real> approachSpeed_;            ///< por par de getTouching(), medida ANTES do solver

    /// Maior distância de um ponto de qualquer forma ao centro de massa do seu corpo
    /// (cresce ao criar corpos; nunca diminui). Entra no cálculo da folga das consultas.
    Real maxBodyRadius_ = Real(0);
};

} // namespace physics
