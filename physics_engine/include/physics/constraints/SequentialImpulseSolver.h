// =============================================================================
// SequentialImpulseSolver.h
// Solver de restrições de velocidade por impulsos sequenciais (Gauss–Seidel
// projetado), na linha de Catto (GDC 2006/2009) e Allen Chou.
//
// Uso por passo (o PhysicsWorld faz exatamente isto):
//     Integrator::integrateVelocities(...)     // forças + gravidade
//     solver.clear();
//     solver.addContact(manifold, a, b);  ...  // um por par colidindo
//     solver.addJoint(joint);             ...
//     solver.solve(dt);                        // prepare → warm start → N iterações → finish
//     Integrator::integratePositions(...)
//     solver.solvePositions();                 // NGS: corrige penetrações/juntas movendo as poses
//
// Ordem dentro de cada iteração: juntas primeiro, contatos depois (contatos
// são os mais numerosos e os que mais se beneficiam de ter a última palavra).
// Nenhuma alocação no laço: os vetores internos só crescem até o maior número
// de restrições visto e depois reutilizam a capacidade.
// =============================================================================
#pragma once

#include <vector>

#include "physics/constraints/ContactConstraint.h"
#include "physics/constraints/JointConstraint.h"

namespace physics {

struct SolverStats {
    int contactConstraints = 0;
    int contactPoints = 0;
    int jointConstraints = 0;
    int iterations = 0;
    int positionIterations = 0;     ///< iterações de correção de posição (NGS) realmente executadas
    Real maxPenetration = Real(0);  ///< maior penetração de contato medida na ÚLTIMA iteração de posição [m]
    Real maxJointError = Real(0);   ///< maior erro de junta medido na última iteração de posição [m ou rad]
};

class SequentialImpulseSolver {
public:
    explicit SequentialImpulseSolver(const SolverSettings& settings = SolverSettings())
        : settings_(settings) {}

    SolverSettings& settings() { return settings_; }
    const SolverSettings& settings() const { return settings_; }

    /// Pré-aloca espaço para evitar crescimento durante a simulação.
    void reserve(std::size_t contacts, std::size_t joints);

    /// Remove todas as restrições do passo (mantém a capacidade).
    void clear();

    /// Registra um contato. Retorna false (e ignora) se o manifold está vazio,
    /// os corpos são nulos/iguais, nenhum é dinâmico ou a normal não é finita.
    /// O manifold deve permanecer válido até o fim de solve().
    bool addContact(ContactManifold& manifold, RigidBody& a, RigidBody& b);

    /// Registra uma junta (propriedade do chamador; deve viver até o fim de solve()).
    void addJoint(JointConstraint& joint);

    /// Resolve todas as restrições de VELOCIDADE. dt ≤ 0 ou não finito: não faz nada.
    void solve(Real dt);

    /// Correção de posição (NGS): chame DEPOIS de Integrator::integratePositions, com as mesmas
    /// restrições do solve(). Move as poses para reduzir penetrações e erros de junta, sem criar
    /// velocidade (não injeta energia). Faz até settings().positionIterations iterações e para
    /// cedo quando o erro já é pequeno. Com positionIterations == 0 não faz nada (Baumgarte legado).
    void solvePositions();

    const SolverStats& getStats() const { return stats_; }
    std::size_t getContactCount() const { return contacts_.size(); }
    std::size_t getJointCount() const { return joints_.size(); }

private:
    SolverSettings settings_;
    std::vector<ContactConstraint> contacts_;
    std::vector<JointConstraint*> joints_;
    SolverStats stats_;
};

} // namespace physics
