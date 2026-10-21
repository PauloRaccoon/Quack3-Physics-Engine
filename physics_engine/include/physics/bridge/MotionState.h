// =============================================================================
// MotionState.h
// Camada de DESACOPLAMENTO entre a taxa de quadros do renderizador (variável)
// e a taxa fixa da simulação física (Δt_fixo).
//
//   FixedTimestepAccumulator  decide QUANTOS passos fixos rodar a cada quadro
//                             de render e calcula  α = acumulador / Δt_fixo.
//   MotionState               guarda a pose do passo anterior e a do atual, e
//                             interpola entre elas com α:
//                                 posição     → lerp
//                                 orientação  → slerp
//
// Por que interpolar: se o render desenhasse sempre a última pose física, um
// motor a 60 Hz mostrado a 144 Hz repetiria o mesmo quadro ~2 vezes e a
// animação "engasgaria". Interpolando entre as duas últimas poses (um passo
// de atraso) o movimento fica contínuo a qualquer taxa de render.
//
//      tempo físico:   |---- passo n-1 ----|---- passo n ----|
//      poses:               previous             current
//      render (α):                     ●  ← desenha lerp(previous, current, α)
// =============================================================================
#pragma once

#include "physics/common/Settings.h"
#include "physics/math/Transform.h"

namespace physics {

class MotionState {
public:
    MotionState() = default;
    explicit MotionState(const Transform& t) : previous_(t), current_(t) {}

    /// "Teletransporte": as duas poses passam a ser `t`, sem rastro de interpolação.
    void reset(const Transform& t) { previous_ = current_ = t; }

    /// Chamado após CADA passo fixo com a nova pose física.
    void push(const Transform& newCurrent) {
        previous_ = current_;
        current_ = newCurrent;
    }

    const Transform& getPrevious() const { return previous_; }
    const Transform& getCurrent() const { return current_; }

    /// Pose para desenhar: lerp na posição, slerp na orientação.
    /// `alpha` é saturado em [0,1].
    Transform interpolate(Real alpha) const { return Transform::interpolate(previous_, current_, alpha); }

private:
    Transform previous_;
    Transform current_;
};

// -----------------------------------------------------------------------------
class FixedTimestepAccumulator {
public:
    /// @throws std::invalid_argument se fixedDt não for finito e positivo ou maxSubSteps < 1.
    explicit FixedTimestepAccumulator(Real fixedDt = kFixedTimeStep, int maxSubSteps = kMaxSubSteps);

    /// Soma o tempo do quadro de render e devolve quantos passos fixos executar
    /// agora (0..maxSubSteps).
    ///  - tempo negativo, NaN ou infinito é ignorado (0 passos);
    ///  - um quadro muito longo (ex.: janela arrastada, breakpoint) é limitado a
    ///    maxSubSteps·Δt: sem isso a simulação ficaria devendo tempo e cada
    ///    quadro exigiria ainda mais passos ("espiral da morte"). O tempo
    ///    excedente é descartado e contabilizado em getDroppedTime().
    int advance(Real frameTime);

    /// Fração do passo fixo já acumulada, em [0,1): o α da interpolação.
    Real getAlpha() const { return accumulator_ / fixedDt_; }

    Real getFixedDt() const { return fixedDt_; }

    /// Troca o passo fixo em tempo de execução. `dt` deve ser finito e > 0 (senão não muda nada e
    /// devolve false). O α é preservado: o tempo acumulado é reescalado para a mesma fração do novo passo,
    /// então a interpolação de render não dá salto.
    bool setFixedDt(Real dt);
    int getMaxSubSteps() const { return maxSubSteps_; }
    Real getAccumulator() const { return accumulator_; }
    Real getDroppedTime() const { return dropped_; }

    void reset() { accumulator_ = Real(0); dropped_ = Real(0); }

private:
    Real fixedDt_;
    int maxSubSteps_;
    Real accumulator_ = Real(0);
    Real dropped_ = Real(0);
};

} // namespace physics
