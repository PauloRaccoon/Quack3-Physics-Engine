// =============================================================================
// RigidBody.h
// Corpo rígido 3D com orientação por quatérnio.
//
// Convenções:
//  - O estado dinâmico (posição, velocidades) refere-se ao CENTRO DE MASSA
//    em coordenadas de mundo. O "frame do corpo" (origem das formas locais,
//    usado para renderização) é derivado:  origem = com − R · centroLocal.
//  - Velocidade angular e tensores "world" estão no referencial do MUNDO.
//  - Corpos Static/Kinematic têm invMass = 0 e invInertia = 0; só Kinematic
//    se move (por sua velocidade), sem sofrer forças nem contatos.
// =============================================================================
#pragma once

#include <memory>
#include <optional>

#include "physics/common/Settings.h"
#include "physics/common/Types.h"
#include "physics/dynamics/MassData.h"
#include "physics/geometry/Shape.h"
#include "physics/math/Matrix3.h"
#include "physics/math/Quaternion.h"
#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

namespace physics {

enum class BodyType : std::uint8_t {
    Static,     ///< nunca se move
    Kinematic,  ///< move-se por velocidade imposta; massa infinita
    Dynamic     ///< simulado: forças, gravidade, contatos
};

/// Parâmetros de criação de um corpo.
struct RigidBodyDesc {
    BodyType type = BodyType::Dynamic;
    std::shared_ptr<const Shape> shape;      ///< obrigatório para Dynamic (salvo massData)
    Real density = Real(1);                  ///< usada se mass == 0
    Real mass = Real(0);                     ///< > 0 sobrescreve a densidade
    std::optional<MassData> massData;        ///< sobrescreve tudo (inércia customizada)

    Transform transform;                     ///< posição/orientação do FRAME do corpo
    Vector3 linearVelocity;
    Vector3 angularVelocity;

    Real friction = Real(0.5);
    Real restitution = Real(0);
    Real linearDamping = Real(0);
    Real angularDamping = Real(0.01);
    Real gravityScale = Real(1);
    /// Permite que o corpo (e a ilha de corpos ligados a ele) adormeça quando fica em repouso.
    /// false mantém a ilha inteira sempre acordada (ex.: corpo controlado por script).
    bool allowSleep = true;
    void* userData = nullptr;
};

class RigidBody {
public:
    /// @throws std::invalid_argument para descrição inválida (ex.: corpo
    ///         dinâmico sem forma nem massData, massa/densidade inválidas,
    ///         estado ou parâmetros (atrito, restituição, amortecimentos,
    ///         gravityScale) não finitos).
    explicit RigidBody(const RigidBodyDesc& desc);

    // --- Identidade ------------------------------------------------------------
    BodyId getId() const { return id_; }
    void setId(BodyId id) { id_ = id; }
    BodyType getType() const { return type_; }
    bool isStatic() const { return type_ == BodyType::Static; }
    bool isKinematic() const { return type_ == BodyType::Kinematic; }
    bool isDynamic() const { return type_ == BodyType::Dynamic; }

    const Shape* getShape() const { return shape_.get(); }
    const std::shared_ptr<const Shape>& getShapePtr() const { return shape_; }

    void* getUserData() const { return userData_; }
    void setUserData(void* p) { userData_ = p; }

    // --- Pose ----------------------------------------------------------------------
    /// Transformação do FRAME do corpo (posição da origem local + orientação).
    /// É a que deve ser aplicada à forma e ao modelo gráfico.
    const Transform& getTransform() const { return transform_; }

    const Vector3& getWorldCenterOfMass() const { return position_; }
    const Vector3& getLocalCenterOfMass() const { return massData_.centerOfMass; }
    const Quaternion& getOrientation() const { return orientation_; }

    /// Define a pose pelo frame do corpo.
    void setTransform(const Transform& frame);

    /// Define a pose pelo centro de massa (usado pelo integrador). O
    /// quatérnio é renormalizado; entradas não finitas são ignoradas.
    void setPose(const Vector3& worldCenterOfMass, const Quaternion& orientation);

    // --- Velocidades -------------------------------------------------------------------
    const Vector3& getLinearVelocity() const { return linearVelocity_; }
    const Vector3& getAngularVelocity() const { return angularVelocity_; }
    /// Corpos estáticos ignoram a atribuição (permanecem em repouso); valores não finitos
    /// (NaN/±Inf) também são ignorados: o estado do corpo nunca fica não finito pela API.
    /// Uma velocidade não nula ACORDA um corpo dinâmico adormecido.
    void setLinearVelocity(const Vector3& v) {
        if (isStatic() || !v.isFinite()) return;
        linearVelocity_ = v;
        if (v.lengthSquared() > Real(0)) wake();
    }
    void setAngularVelocity(const Vector3& w) {
        if (isStatic() || !w.isFinite()) return;
        angularVelocity_ = w;
        if (w.lengthSquared() > Real(0)) wake();
    }

    // --- Sleeping ------------------------------------------------------------------------------
    // Um corpo dinâmico em repouso por tempo suficiente adormece (o PhysicsWorld decide, por
    // ilhas de corpos ligados por contatos/juntas): deixa de ser integrado e de gastar solver e
    // narrowphase. Dormindo, v = ω = 0. Corpos cinemáticos estão sempre "acordados"; estáticos nunca.
    // Forças, impulsos, velocidade não nula, mudança de pose, de massa ou de gravityScale
    // acordam o corpo automaticamente (e, no passo seguinte, a ilha inteira dele).
    bool isAwake() const { return awake_; }
    /// true acorda (só corpos dinâmicos); false adormece: zera velocidades e forças. Outros tipos: ignora.
    void setAwake(bool awake);
    bool getAllowSleep() const { return allowSleep_; }
    void setAllowSleep(bool allow) { allowSleep_ = allow; if (!allow) wake(); }
    /// Tempo [s] consecutivo abaixo dos limiares de repouso (gerido pelo PhysicsWorld).
    Real getSleepTime() const { return sleepTime_; }
    void setSleepTime(Real t) { sleepTime_ = isFinite(t) && t > Real(0) ? t : Real(0); }

    /// Velocidade do ponto material em `worldPoint`:  v + ω × (p − com).
    Vector3 getVelocityAtPoint(const Vector3& worldPoint) const {
        return linearVelocity_ + angularVelocity_.cross(worldPoint - position_);
    }

    // --- Massa ------------------------------------------------------------------------------
    const MassData& getMassData() const { return massData_; }
    Real getMass() const { return massData_.mass; }
    Real getInvMass() const { return invMass_; }
    /// Inverso do tensor de inércia no referencial do mundo: R · I⁻¹ · Rᵀ.
    const Matrix3& getInvInertiaWorld() const { return invInertiaWorld_; }

    /// Substitui as propriedades de massa (apenas corpos Dynamic).
    void setMassData(const MassData& data);

    // --- Material / parâmetros ------------------------------------------------------------------
    Real getFriction() const { return friction_; }
    Real getRestitution() const { return restitution_; }
    Real getLinearDamping() const { return linearDamping_; }
    Real getAngularDamping() const { return angularDamping_; }
    Real getGravityScale() const { return gravityScale_; }
    // Valores NÃO finitos (NaN/±Inf) são ignorados (o parâmetro mantém o valor atual): NaN passa
    // por comparações como `d < 0` e contaminaria velocidades e impulsos de contato.
    // Atrito é limitado a [0, kMaxFriction], restituição a [0, 1], amortecimentos a >= 0.
    void setFriction(Real f) { if (isFinite(f)) friction_ = clamp(f, Real(0), kMaxFriction); }
    void setRestitution(Real r) { if (isFinite(r)) restitution_ = clamp(r, Real(0), Real(1)); }
    void setLinearDamping(Real d) { if (isFinite(d)) linearDamping_ = d < Real(0) ? Real(0) : d; }
    void setAngularDamping(Real d) { if (isFinite(d)) angularDamping_ = d < Real(0) ? Real(0) : d; }
    void setGravityScale(Real s) { if (isFinite(s)) { gravityScale_ = s; wake(); } }

    // --- Forças e impulsos --------------------------------------------------------------------------
    /// Forças são acumuladas e consumidas (zeradas) pelo integrador a cada passo.
    /// Só corpos Dynamic aceitam forças; argumentos não finitos (ou cuja soma estoure) são ignorados.
    void applyForce(const Vector3& force) { accumulate(force_, force); }
    void applyTorque(const Vector3& torque) { accumulate(torque_, torque); }
    void applyForceAtPoint(const Vector3& force, const Vector3& worldPoint) {
        if (!isDynamic() || !force.isFinite() || !worldPoint.isFinite()) return;
        const Vector3 torque = (worldPoint - position_).cross(force);
        if (!torque.isFinite()) return;   // força e braço enormes: o produto estourou, nada é aplicado
        accumulate(force_, force);
        accumulate(torque_, torque);
    }
    // --- Variação de velocidade do passo (para a restituição) -----------------------------------
    /// Quanto Integrator::integrateVelocities MUDOU a velocidade neste passo (gravidade, forças,
    /// torques, termo giroscópico e amortecimento). O contato a subtrai para achar a velocidade de
    /// IMPACTO "de verdade" (antes dessa variação): sem isso a restituição enxerga a aceleração de
    /// um passo inteiro de gravidade como parte do impacto e um quique pequeno vira um ponto fixo
    /// que nunca decai (v* = e·g·Δt/(1-e): para e = 0,8 a bola pula para sempre a 0,65 m/s).
    /// Zerada por Integrator::integratePositions (fim do passo); sem o integrador fica 0 e a
    /// restituição usa a velocidade atual, como antes.
    const Vector3& getStepLinearDelta() const { return stepDeltaV_; }
    const Vector3& getStepAngularDelta() const { return stepDeltaW_; }
    void setStepVelocityDelta(const Vector3& dv, const Vector3& dw) {
        if (dv.isFinite() && dw.isFinite()) { stepDeltaV_ = dv; stepDeltaW_ = dw; }
        else clearStepVelocityDelta();
    }
    void clearStepVelocityDelta() { stepDeltaV_ = Vector3::zero(); stepDeltaW_ = Vector3::zero(); }

    const Vector3& getForce() const { return force_; }
    const Vector3& getTorque() const { return torque_; }
    void clearForces() { force_ = Vector3::zero(); torque_ = Vector3::zero(); }

    /// Impulso aplicado imediatamente: Δv = J/m,  Δω = I⁻¹ (r × J).
    /// Só corpos Dynamic respondem. Impulso não finito — ou que faria a velocidade estourar — é
    /// ignorado (sem isso, 0·Inf = NaN contaminaria até corpos de massa infinita e, via contatos,
    /// o mundo todo).
    void applyLinearImpulse(const Vector3& impulse) {
        if (!isDynamic() || !impulse.isFinite()) return;
        const Vector3 v = linearVelocity_ + impulse * invMass_;
        if (v.isFinite()) {
            linearVelocity_ = v;
            wake();
        }
    }
    void applyAngularImpulse(const Vector3& angularImpulse) {
        if (!isDynamic() || !angularImpulse.isFinite()) return;
        const Vector3 w = angularVelocity_ + invInertiaWorld_ * angularImpulse;
        if (w.isFinite()) {
            angularVelocity_ = w;
            wake();
        }
    }
    void applyImpulseAtPoint(const Vector3& impulse, const Vector3& worldPoint) {
        if (!isDynamic() || !impulse.isFinite() || !worldPoint.isFinite()) return;
        const Vector3 angular = (worldPoint - position_).cross(impulse);
        if (!angular.isFinite()) return;
        applyLinearImpulse(impulse);
        applyAngularImpulse(angular);
    }

    // --- Conversões de coordenadas (frame do corpo) -----------------------------------------------------
    Vector3 localToWorld(const Vector3& localPoint) const { return transform_.transformPoint(localPoint); }
    Vector3 worldToLocal(const Vector3& worldPoint) const { return transform_.inverseTransformPoint(worldPoint); }

    // --- Grandezas físicas (diagnóstico/testes) -------------------------------------------------------------
    Real getKineticEnergy() const;
    /// Momento angular no referencial do mundo: R I Rᵀ ω.
    Vector3 getAngularMomentum() const;

private:
    /// acc += x, só para corpos Dynamic e só se x e o resultado forem finitos.
    void accumulate(Vector3& acc, const Vector3& x) {
        if (!isDynamic() || !x.isFinite()) return;
        const Vector3 sum = acc + x;
        if (sum.isFinite()) {
            acc = sum;
            wake();
        }
    }

    /// Acorda um corpo dinâmico adormecido (não faz nada se já está acordado).
    void wake() {
        if (!awake_ && type_ == BodyType::Dynamic) {
            awake_ = true;
            sleepTime_ = Real(0);
        }
    }

    /// Recalcula transform_ e invInertiaWorld_ a partir do estado primário.
    void updateDerived();

    // --- Dados quentes (acessados pelo solver a cada iteração) ---
    Vector3 position_;           ///< centro de massa em mundo
    Vector3 linearVelocity_;
    Vector3 angularVelocity_;
    Quaternion orientation_;
    Matrix3 invInertiaWorld_ = Matrix3::zero();
    Real invMass_ = Real(0);

    // --- Dados menos quentes ---
    Vector3 force_;
    Vector3 torque_;
    Vector3 stepDeltaV_;         ///< variação de velocidade linear do último integrateVelocities
    Vector3 stepDeltaW_;         ///< idem angular
    Transform transform_;        ///< frame do corpo (derivado)
    MassData massData_;
    std::shared_ptr<const Shape> shape_;

    Real friction_;
    Real restitution_;
    Real linearDamping_;
    Real angularDamping_;
    Real gravityScale_;
    Real sleepTime_ = Real(0);
    BodyType type_;
    BodyId id_ = kInvalidBodyId;
    void* userData_ = nullptr;
    bool awake_ = true;          ///< dinâmico/cinemático: acordado; estático: false
    bool allowSleep_ = true;
};

} // namespace physics
