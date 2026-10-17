// =============================================================================
// IPhysicsTransformBridge.h
// Contrato entre o motor de física e QUALQUER renderizador (raylib, OpenGL,
// engine de jogo, binding Python...). O motor não conhece o renderizador: ele
// só chama esta interface, uma vez por quadro de render, com a pose JÁ
// INTERPOLADA (ver MotionState) de cada corpo.
//
//   mundo.update(frameTime)  →  beginSync(α)
//                               onTransformUpdated(id, pose)   (um por corpo)
//                               endSync()
//
// BufferedTransformBridge é uma implementação pronta que grava as poses num
// vetor PLANO de floats (7 por corpo: px py pz qx qy qz qw), indexado pelo
// BodyId — formato fácil de ler de fora do C++ (ponteiro + tamanho), por
// exemplo via ctypes/numpy em Python, sem copiar objeto a objeto.
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

#include "physics/common/Settings.h"
#include "physics/common/Types.h"
#include "physics/math/Transform.h"

namespace physics {

class IPhysicsTransformBridge {
public:
    virtual ~IPhysicsTransformBridge() = default;

    /// Um corpo foi criado (ou o mundo acabou de ser ligado à bridge).
    virtual void onBodyCreated(BodyId id, const Transform& transform) {
        (void)id;
        (void)transform;
    }

    /// O corpo foi destruído: o renderizador deve parar de desenhá-lo.
    virtual void onBodyDestroyed(BodyId id) { (void)id; }

    /// Início de uma sincronização; `alpha` é o fator de interpolação usado.
    virtual void beginSync(Real alpha) { (void)alpha; }

    /// Pose interpolada de um corpo (mundo → render).
    virtual void onTransformUpdated(BodyId id, const Transform& interpolated) = 0;

    virtual void endSync() {}

protected:
    IPhysicsTransformBridge() = default;
    IPhysicsTransformBridge(const IPhysicsTransformBridge&) = default;
    IPhysicsTransformBridge& operator=(const IPhysicsTransformBridge&) = default;
};

// -----------------------------------------------------------------------------
class BufferedTransformBridge final : public IPhysicsTransformBridge {
public:
    /// px, py, pz, qx, qy, qz, qw  (quatérnio na ordem x,y,z,w — a mesma do raylib).
    static constexpr int kFloatsPerBody = 7;

    void onBodyCreated(BodyId id, const Transform& transform) override;
    void onBodyDestroyed(BodyId id) override;
    void beginSync(Real alpha) override;
    void onTransformUpdated(BodyId id, const Transform& interpolated) override;

    /// Número de "slots" (maior BodyId visto + 1).
    std::size_t getSlotCount() const { return alive_.size(); }
    bool isAlive(BodyId id) const { return id < alive_.size() && alive_[id] != 0; }

    /// Vetor plano com kFloatsPerBody floats por slot (válido até a próxima chamada que cresça o buffer).
    const float* data() const { return data_.data(); }
    /// Ponteiro para os 7 floats do corpo, ou nullptr se o id nunca foi visto.
    const float* getBodyData(BodyId id) const { return id < alive_.size() ? &data_[id * kFloatsPerBody] : nullptr; }

    /// Reconstrói a pose gravada (identidade se o id nunca foi visto).
    Transform getTransform(BodyId id) const;

    Real getLastAlpha() const { return lastAlpha_; }
    std::uint64_t getSyncCount() const { return syncCount_; }

private:
    void write(BodyId id, const Transform& t);

    std::vector<float> data_;
    std::vector<std::uint8_t> alive_;
    Real lastAlpha_ = Real(0);
    std::uint64_t syncCount_ = 0;
};

} // namespace physics
