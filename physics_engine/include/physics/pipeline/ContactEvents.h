// =============================================================================
// ContactEvents.h
// Eventos de contato do PhysicsWorld: início e fim do toque entre dois corpos.
//
// Dois jeitos de consumir (podem ser combinados):
//   1) OUVINTE (push):   world.setContactListener(&meuOuvinte);
//   2) BUFFER  (pull):   depois de world.update(dt) / world.step(), percorrer
//                        world.getContactEvents()  (esvaziado no início do próximo update/step).
//
// O QUE É "CONTATO": o par tem ao menos um ponto de contato, isto é, as formas estão a
// no máximo kContactMargin (5 mm) uma da outra (contatos "especulativos" contam).
// Um par que persiste entre passos NÃO gera evento; um par que adormece junto com a ilha
// também não (o contato continua existindo; ver PhysicsWorld::isAwake).
//
// QUANDO DISPARAM: no fim de cada passo fixo, DEPOIS do solver e da integração, e nunca no
// meio do laço de restrições. Por isso o ouvinte pode, dentro do callback, criar ou destruir
// corpos, aplicar impulsos, consultar o mundo, etc. NÃO pode chamar step()/update() (lança
// std::logic_error se tentar: reentrância).
//
// Um corpo destruído gera um evento de FIM para cada contato em que estava.
// Todos os dados são CÓPIAS: valem mesmo que os corpos já tenham sido destruídos.
// =============================================================================
#pragma once

#include "physics/common/Types.h"
#include "physics/math/Vector3.h"

namespace physics {

struct ContactEvent {
    enum class Type : std::uint8_t { Begin, End };

    Type type = Type::Begin;
    BodyId a = kInvalidBodyId;      ///< corpo de menor id do par
    BodyId b = kInvalidBodyId;      ///< corpo de maior id do par
    Vector3 point;                  ///< média dos pontos de contato (mundo)
    Vector3 normal;                 ///< de `a` para `b`, unitária
    Real depth = Real(0);           ///< maior penetração entre os pontos (< 0: folga/especulativo) [m]
    int pointCount = 0;             ///< pontos de contato do manifold (1..4)

    /// Soma dos impulsos normais aplicados pelo solver NESTE passo [N·s]
    /// (Begin: o impacto de chegada; End: o do último passo em contato).
    /// Dividir por Δt dá a força média de contato.
    Real normalImpulse = Real(0);

    /// Velocidade de aproximação ao longo da normal ANTES do solver [m/s] (>= 0). Em um Begin é a
    /// "velocidade de impacto": é o que se usa para volume de som, partículas, dano...
    Real approachSpeed = Real(0);
};

/// Ouvinte de eventos de contato. Implemente só o que precisar.
class IContactListener {
public:
    virtual ~IContactListener() = default;
    virtual void onContactBegin(const ContactEvent& event) { (void)event; }
    virtual void onContactEnd(const ContactEvent& event) { (void)event; }
};

} // namespace physics
