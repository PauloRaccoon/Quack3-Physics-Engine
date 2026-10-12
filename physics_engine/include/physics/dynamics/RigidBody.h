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
    void* userData = nullptr;
};

class RigidBody {
public:
    /// @throws std::invalid_argument para descrição inválida (ex.: corpo
    ///         dinâmico sem forma nem massData, massa/densidade inválidas,
    ///         estado não finito).
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
    /// Corpos estáticos ignoram a atribuição (permanecem em repouso).
    void setLinearVelocity(const Vector3& v) { if (!isStatic()) linearVelocity_ = v; }
    void setAngularVelocity(const Vector3& w) { if (!isStatic()) angularVelocity_ = w; }

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
    void setFriction(Real f) { friction_ = f < Real(0) ? Real(0) : f; }
    void setRestitution(Real r) { restitution_ = clamp(r, Real(0), Real(1)); }
    void setLinearDamping(Real d) { linearDamping_ = d < Real(0) ? Real(0) : d; }
    void setAngularDamping(Real d) { angularDamping_ = d < Real(0) ? Real(0) : d; }
    void setGravityScale(Real s) { gravityScale_ = s; }

    // --- Forças e impulsos --------------------------------------------------------------------------
    /// Forças são acumuladas e consumidas (zeradas) pelo integrador a cada passo.
    void applyForce(const Vector3& force) { if (isDynamic()) force_ += force; }
    void applyTorque(const Vector3& torque) { if (isDynamic()) torque_ += torque; }
    void applyForceAtPoint(const Vector3& force, const Vector3& worldPoint) {
        if (!isDynamic()) return;
        force_ += force;
        torque_ += (worldPoint - position_).cross(force);
    }
    const Vector3& getForce() const { return force_; }
    const Vector3& getTorque() const { return torque_; }
    void clearForces() { force_ = Vector3::zero(); torque_ = Vector3::zero(); }

    /// Impulso aplicado imediatamente: Δv = J/m,  Δω = I⁻¹ (r × J).
    void applyLinearImpulse(const Vector3& impulse) { linearVelocity_ += impulse * invMass_; }
    void applyAngularImpulse(const Vector3& angularImpulse) {
        angularVelocity_ += invInertiaWorld_ * angularImpulse;
    }
    void applyImpulseAtPoint(const Vector3& impulse, const Vector3& worldPoint) {
        applyLinearImpulse(impulse);
        applyAngularImpulse((worldPoint - position_).cross(impulse));
    }

    // --- Conversões de coordenadas (frame do corpo) -----------------------------------------------------
    Vector3 localToWorld(const Vector3& localPoint) const { return transform_.transformPoint(localPoint); }
    Vector3 worldToLocal(const Vector3& worldPoint) const { return transform_.inverseTransformPoint(worldPoint); }

    // --- Grandezas físicas (diagnóstico/testes) -------------------------------------------------------------
    Real getKineticEnergy() const;
    /// Momento angular no referencial do mundo: R I Rᵀ ω.
    Vector3 getAngularMomentum() const;

private:
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
    Transform transform_;        ///< frame do corpo (derivado)
    MassData massData_;
    std::shared_ptr<const Shape> shape_;

    Real friction_;
    Real restitution_;
    Real linearDamping_;
    Real angularDamping_;
    Real gravityScale_;
    BodyType type_;
    BodyId id_ = kInvalidBodyId;
    void* userData_ = nullptr;
};

} // namespace physics
