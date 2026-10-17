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
//                  persistentes, com warm starting)
//   2. forças    : Integrator::integrateVelocities (gravidade, forças, damping)
//   3. restrições: SequentialImpulseSolver (contatos + juntas)
//   4. posições  : Integrator::integratePositions (Euler semi-implícito)
//   5. render    : MotionState::push (pose anterior/atual para interpolar)
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
#include "physics/constraints/SequentialImpulseSolver.h"
#include "physics/dynamics/Integrator.h"
#include "physics/dynamics/RigidBody.h"

namespace physics {

struct PhysicsWorldSettings {
    Vector3 gravity = Vector3(0, -kGravity, 0);
    Real fixedTimeStep = kFixedTimeStep;    ///< Δt da simulação [s]
    int maxSubSteps = kMaxSubSteps;         ///< máximo de passos por update() (anti "espiral da morte")
    std::size_t maxBodies = 4096;           ///< capacidade fixa de corpos
    SolverSettings solver;
    IntegratorSettings integrator;
    CollisionWorldSettings collision;
};

struct WorldStats {
    int bodies = 0;
    int joints = 0;
    int pairs = 0;            ///< pares com Fat AABBs sobrepostos
    int touchingPairs = 0;    ///< pares com pontos de contato
    int contactPoints = 0;
    std::uint64_t stepCount = 0;
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
    void step();

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
    void setGravity(const Vector3& g) { if (g.isFinite()) gravity_ = g; }
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
};

} // namespace physics
