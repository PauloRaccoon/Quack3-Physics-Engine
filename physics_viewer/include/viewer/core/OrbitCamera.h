// =============================================================================
// OrbitCamera.h
// Câmera orbital (gira em torno de um alvo) SEM raylib: só guarda ângulos e
// distância e calcula a posição. A janela alimenta rotate()/zoom() com o mouse.
//
// Coordenadas esféricas em torno do alvo, Y para cima:
//     posição = alvo + d · (cos(pitch)·sin(yaw),  sin(pitch),  cos(pitch)·cos(yaw))
// yaw = 0 e pitch = 0 → a câmera fica no +Z olhando para o alvo.
// =============================================================================
#pragma once

#include "viewer/core/ViewerMath.h"

namespace viewer {

class OrbitCamera {
public:
    static constexpr float kMinPitch = -1.50f;      ///< ~ -86°: nunca passa do pólo (evita o "flip")
    static constexpr float kMaxPitch = 1.50f;       ///< ~ +86°
    static constexpr float kMinDistance = 1.0f;
    static constexpr float kMaxDistance = 200.0f;

    Vec3f target{0.0f, 1.5f, 0.0f};

    /// Soma aos ângulos [rad]. Entradas não finitas são ignoradas; o pitch é limitado.
    void rotate(float dYaw, float dPitch);

    /// Roda do mouse: cada "clique" aproxima/afasta ~10 %. Distância limitada.
    void zoom(float wheel);

    /// Move o alvo (para seguir um corpo, por exemplo). Valores não finitos são ignorados.
    void setTarget(const Vec3f& t);

    /// Define a distância ao alvo (limitada a [kMinDistance, kMaxDistance]; não finito é ignorado).
    void setDistance(float d);

    /// Volta ao enquadramento inicial: ângulos padrão, alvo e distância dados (distância limitada).
    void reset(const Vec3f& newTarget, float newDistance);

    /// Aproxima o alvo de `goal` de forma suave (exponencial): a fração (1 - e^(-rate·dt)) da distância
    /// restante por chamada, independente do FPS. dt <= 0 ou valores não finitos: não muda nada.
    void followTarget(const Vec3f& goal, float dt, float rate = 10.0f);

    Vec3f position() const;

    float getYaw() const { return yaw_; }
    float getPitch() const { return pitch_; }
    float getDistance() const { return distance_; }

private:
    static constexpr float kDefaultYaw = 0.8f;
    static constexpr float kDefaultPitch = 0.5f;
    float yaw_ = kDefaultYaw;
    float pitch_ = kDefaultPitch;
    float distance_ = 12.0f;
};

} // namespace viewer
