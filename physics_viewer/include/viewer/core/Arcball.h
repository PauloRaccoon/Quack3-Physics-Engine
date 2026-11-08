// =============================================================================
// Arcball.h  (V24)
// Esfera de arcball (Shoemake): girar um corpo arrastando o mouse sobre uma esfera. Parte SEM raylib:
//   - pontos da esfera a partir de coordenadas do mouse normalizadas (raio 1, y para cima);
//   - o quatérnio que leva um ponto da esfera a outro (a "agarrada" acompanha o cursor);
//   - a base da câmera (direita, cima, "para trás") para converter o giro do espaço da tela ao do mundo;
//   - projeção de pontos e de círculos máximos do corpo para desenhar a esfera em 2D.
// O estado do arrasto guarda a orientação INICIAL e sempre recalcula  q = Δ · q0  (sem acumular erro).
// =============================================================================
#pragma once

#include "physics/math/Quaternion.h"
#include "physics/math/Vector3.h"
#include "viewer/core/OrbitCamera.h"

namespace viewer {

/// Base da câmera no mundo: x_tela = p·right, y_tela = p·up, profundidade = p·back (positivo = mais perto da câmera).
struct ArcballView {
    physics::Vector3 right{1, 0, 0};
    physics::Vector3 up{0, 1, 0};
    physics::Vector3 back{0, 0, 1};
};

/// Base da câmera orbital (olha de `position()` para `target`, Y do mundo para cima).
ArcballView arcballViewFromCamera(const OrbitCamera& cam);

/// Ponto da esfera unitária sob o mouse em (nx, ny) (raio da esfera = 1, y para cima, z para a câmera).
/// Dentro do disco: z = sqrt(1 − r²); fora dele: o ponto da borda (z = 0) na mesma direção. Sempre norma 1.
/// Entradas não finitas viram o polo da câmera (0,0,1).
physics::Vector3 arcballSpherePoint(double nx, double ny);

/// Rotação de menor ângulo que leva `a` em `b` (ambos pontos da esfera). Assim o ponto agarrado fica sob o cursor.
/// (A versão clássica de Shoemake, q = (a×b, a·b), gira o DOBRO do ângulo; aqui o giro é o do arco.) a ≈ b → identidade.
physics::Quaternion arcballDelta(const physics::Vector3& a, const physics::Vector3& b);

/// Giro `delta` dado no espaço da câmera (x direita, y cima, z para trás) levado ao mundo: mesmo ângulo, eixo reescrito na base `view`.
physics::Quaternion arcballToWorld(const physics::Quaternion& delta, const ArcballView& view);

/// Arrasto: begin() fixa o ponto agarrado e a orientação inicial; update() devolve a orientação nova (norma 1).
class ArcballDrag {
public:
    void begin(double nx, double ny, const physics::Quaternion& startOrientation, const ArcballView& view);
    physics::Quaternion update(double nx, double ny, const ArcballView& view) const;
    void end() { active_ = false; }
    bool active() const { return active_; }

private:
    bool active_ = false;
    physics::Vector3 start_{0, 0, 1};
    physics::Quaternion q0_ = physics::Quaternion::identity();
};

/// Ponto 2D (espaço da tela, raio 1, y para cima) e profundidade (>0 = metade de frente).
struct ArcballPoint {
    float x = 0.0f, y = 0.0f, depth = 0.0f;
};

/// Projeta um vetor do mundo.
ArcballPoint arcballProject(const physics::Vector3& worldVector, const ArcballView& view);

/// Círculo máximo do corpo perpendicular ao eixo local `axis` (0 = X, 1 = Y, 2 = Z) na orientação `q`, com `n` pontos
/// (n >= 3) escritos em `out`. Devolve quantos escreveu (0 se n < 3 ou out nulo).
int arcballCircle(const physics::Quaternion& q, int axis, const ArcballView& view, ArcballPoint* out, int n);

} // namespace viewer
