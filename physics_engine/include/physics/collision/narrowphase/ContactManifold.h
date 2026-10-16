// =============================================================================
// ContactManifold.h
// Pontos de contato entre dois corpos, persistentes entre quadros para permitir
// WARM STARTING no solver de impulsos.
//
//  - Cada ContactPoint guarda os pontos em coordenadas LOCAIS de cada corpo.
//    Como são rigidamente ligados aos corpos, a cada quadro basta
//    retransformá-los (refresh) para obter profundidade e deslize atuais;
//    pontos que se afastaram ou deslizaram demais são descartados.
//  - Cada ponto tem um `id` ESTÁVEL: um novo contato que casa (por proximidade)
//    com um ponto existente herda o id e os impulsos acumulados dele.
//  - O manifold tem no máximo 4 pontos; ao exceder, mantém o mais profundo e
//    a combinação que maximiza a área do contato (melhor estabilidade).
//
// Convenção: `normal` aponta de A para B; depth > 0 = penetração;
//            worldA − worldB = normal · depth  (para cada ponto).
//
// ContactGenerator liga tudo: GJK → EPA → (clipping de faces) → ContactPatch,
// e então atualiza o manifold persistente.
// =============================================================================
#pragma once

#include <cstdint>

#include "physics/common/Settings.h"
#include "physics/common/Types.h"
#include "physics/geometry/Shape.h"
#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

namespace physics {

constexpr int kMaxManifoldPoints = 4;

struct ContactPoint {
    Vector3 localA;   ///< ponto no frame local do corpo A
    Vector3 localB;   ///< ponto no frame local do corpo B
    Vector3 worldA;   ///< ponto em A no mundo (recalculado a cada quadro)
    Vector3 worldB;   ///< ponto em B no mundo
    Real depth = Real(0);          ///< penetração ao longo de `normal` (< 0 = folga)

    // --- Estado do solver (warm starting) -----------------------------------
    Real normalImpulse = Real(0);  ///< impulso acumulado ao longo da normal (>= 0)
    /// Impulso de atrito acumulado, em coordenadas do MUNDO e perpendicular à
    /// normal. Guardar o vetor (e não 2 escalares) permite reprojetá-lo na base
    /// tangente do quadro seguinte mesmo que a normal tenha girado um pouco.
    Vector3 tangentImpulse;

    std::uint32_t id = 0;          ///< identificador estável do ponto no manifold

    /// Ponto de contato "médio" no mundo.
    Vector3 getPosition() const { return (worldA + worldB) * Real(0.5); }
};

/// Pontos de contato recém-gerados pela narrowphase (coordenadas de mundo).
struct PatchPoint {
    Vector3 pointA;
    Vector3 pointB;
    Real depth = Real(0);
};

struct ContactPatch {
    Vector3 normal;                      ///< de A para B, unitária
    PatchPoint points[kMaxManifoldPoints];
    int count = 0;
};

class ContactManifold {
public:
    ContactManifold() = default;
    ContactManifold(BodyId a, BodyId b) : bodyA(a), bodyB(b) {}

    BodyId bodyA = kInvalidBodyId;
    BodyId bodyB = kInvalidBodyId;
    Vector3 normal = Vector3::unitY();     ///< de A para B
    ContactPoint points[kMaxManifoldPoints];
    int count = 0;

    bool isEmpty() const { return count == 0; }
    void clear() { count = 0; }

    /// Retransforma os pontos para o mundo, recalcula a profundidade e descarta
    /// os que se separaram (depth < −limiar) ou deslizaram (> limiar) demais.
    void refresh(const Transform& xfA, const Transform& xfB);

    /// Insere um contato (mundo). Se casar com um ponto existente (dentro de
    /// kContactBreakingThreshold) atualiza-o preservando id e impulsos; senão
    /// adiciona, reduzindo a 4 pontos por área se necessário.
    /// Retorna o índice do ponto resultante, ou −1 se o contato foi descartado
    /// por não contribuir para a área do manifold.
    int addPoint(const Transform& xfA, const Transform& xfB, const PatchPoint& contact);

    /// Atualização completa por quadro:
    ///   1) adota a normal do patch e faz refresh dos pontos persistidos;
    ///   2) patch de 1 ponto  → acumula (estilo Bullet: pontos persistem);
    ///      patch de 2+ pontos → substitui, herdando impulsos por proximidade.
    void update(const Transform& xfA, const Transform& xfB, const ContactPatch& patch);

    /// Soma dos impulsos normais acumulados (útil para diagnóstico/testes).
    Real getTotalNormalImpulse() const;

private:
    ContactPoint makePoint(const Transform& xfA, const Transform& xfB, const PatchPoint& c) const;
    int findMatch(const Vector3& localA, const bool* used) const;
    int chooseDiscard(const ContactPoint& incoming) const;

    std::uint32_t nextId_ = 1;
};

// =============================================================================
// ContactGenerator
// =============================================================================
struct ContactGeneratorSettings {
    /// Formas separadas por até esta distância ainda geram contato (folga/"speculative").
    Real contactMargin = Real(0);
    /// No clipping, mantém pontos com separação até este valor (evita perder
    /// pontos de uma face apoiada com leve inclinação).
    Real clipMargin = kLinearSlop;
    /// cos(ângulo) mínimo entre a normal do contato e a normal da face de
    /// referência para usar clipping; abaixo disso (aresta contra aresta,
    /// vértice...) usa o único ponto do EPA.
    Real minFaceAlignment = Real(0.95);
    bool useFaceClipping = true;
};

class ContactGenerator {
public:
    using Settings = ContactGeneratorSettings;

    /// Detecta a colisão e gera até 4 pontos de contato (coordenadas de mundo).
    /// Retorna false (e patch.count = 0) se as formas não colidem ou a entrada
    /// é inválida (transformação não finita).
    static bool generate(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB,
                         ContactPatch& patch, const Settings& settings = Settings());

    /// generate() + atualização do manifold persistente (ou clear() se não colidem).
    static bool collide(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB,
                        ContactManifold& manifold, const Settings& settings = Settings());
};

} // namespace physics
