// =============================================================================
// DebugDraw.h
// Desenho de depuração do mundo físico, independente de qualquer biblioteca
// gráfica. O renderizador (raylib, OpenGL, ImGui...) só precisa implementar UMA
// função, IDebugRenderer::drawLine(); o motor gera as linhas:
//
//     struct MeuRenderer : physics::IDebugRenderer {
//         void drawLine(const Vector3& a, const Vector3& b, const DebugColor& c) override {
//             DrawLine3D(toRaylib(a), toRaylib(b), Color{c.r, c.g, c.b, c.a});
//         }
//     };
//     physics::debugDraw(world, meuRenderer);          // a cada quadro
//
// Para renderizadores sem callbacks (ex.: Python via ctypes) use o
// DebugLineBuffer, que acumula as linhas num vetor contíguo de structs simples.
//
// O desenho só LÊ o mundo (const): não altera a simulação.
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

#include "physics/math/Vector3.h"
#include "physics/pipeline/PhysicsWorld.h"

namespace physics {

struct DebugColor {
    std::uint8_t r = 255, g = 255, b = 255, a = 255;
};

/// O que desenhar (combine com |).
enum DebugDrawFlags : std::uint32_t {
    kDebugShapes        = 1u << 0,   ///< arestas das formas (caixa/poliedro) e 3 círculos por esfera
    kDebugAABBs         = 1u << 1,   ///< AABB justo de cada corpo
    kDebugFatAABBs      = 1u << 2,   ///< Fat AABB da broadphase
    kDebugContactPoints = 1u << 3,   ///< pontos de contato (vermelho: penetração; amarelo: folga/especulativo)
    kDebugContactNormals= 1u << 4,   ///< normal de cada contato
    kDebugJoints        = 1u << 5,   ///< ancoragens e eixo das juntas
    kDebugCenterOfMass  = 1u << 6,   ///< cruz no centro de massa
    kDebugVelocities    = 1u << 7,   ///< vetor velocidade linear (escalado)
    kDebugBodyAxes      = 1u << 8,   ///< eixos locais (RGB = XYZ) do frame do corpo
    kDebugDefault       = kDebugShapes | kDebugContactPoints | kDebugContactNormals | kDebugJoints,
    kDebugAll           = 0x1FFu
};

struct DebugDrawSettings {
    std::uint32_t flags = kDebugDefault;
    int sphereSegments = 24;          ///< segmentos de cada círculo da esfera (>= 3)
    Real normalLength = Real(0.3);    ///< comprimento do traço da normal de contato [m]
    Real velocityScale = Real(0.1);   ///< metros desenhados por (m/s)
    Real axisLength = Real(0.5);      ///< comprimento dos eixos locais [m]
    Real pointSize = Real(0.04);      ///< meia-largura da cruz de um ponto [m]
    /// Desenha formas/eixos/centros na pose INTERPOLADA (a mesma que o render usa para o
    /// modelo), em vez da pose física do último passo. Contatos e AABBs ficam sempre na pose física.
    bool useInterpolatedPose = false;

    DebugColor staticColor{130, 130, 130, 255};
    DebugColor kinematicColor{90, 150, 255, 255};
    DebugColor dynamicColor{90, 220, 110, 255};
    DebugColor aabbColor{255, 220, 60, 255};
    DebugColor fatAabbColor{120, 100, 40, 255};
    DebugColor contactColor{255, 60, 60, 255};
    DebugColor speculativeColor{255, 220, 60, 255};
    DebugColor normalColor{255, 150, 40, 255};
    DebugColor jointColor{230, 80, 230, 255};
    DebugColor velocityColor{70, 220, 255, 255};
};

/// Interface do renderizador de depuração. Só drawLine() é obrigatória.
class IDebugRenderer {
public:
    virtual ~IDebugRenderer() = default;

    virtual void drawLine(const Vector3& from, const Vector3& to, const DebugColor& color) = 0;

    /// Ponto (padrão: pequena cruz de 3 linhas, de meia-largura `size`).
    virtual void drawPoint(const Vector3& p, Real size, const DebugColor& color);
};

/// Linha acumulada pelo DebugLineBuffer (sempre em float, layout simples para ctypes/numpy).
struct DebugLine {
    float ax, ay, az;
    float bx, by, bz;
    std::uint8_t r, g, b, a;
};

/// Renderizador que só acumula as linhas num vetor contíguo.
class DebugLineBuffer final : public IDebugRenderer {
public:
    void drawLine(const Vector3& from, const Vector3& to, const DebugColor& color) override;

    void clear() { lines_.clear(); }
    const std::vector<DebugLine>& lines() const { return lines_; }
    const DebugLine* data() const { return lines_.data(); }
    std::size_t size() const { return lines_.size(); }

private:
    std::vector<DebugLine> lines_;
};

/// Gera o desenho de depuração do mundo no renderizador.
void debugDraw(const PhysicsWorld& world, IDebugRenderer& renderer,
               const DebugDrawSettings& settings = DebugDrawSettings());

/// Linhas de uma forma na pose `xf` (útil para desenhar a forma de um objeto fora do mundo,
/// ex.: o "fantasma" de uma caixa que o usuário vai lançar).
void debugDrawShape(const Shape& shape, const Transform& xf, IDebugRenderer& renderer,
                    const DebugColor& color, int sphereSegments = 24);

/// 12 arestas de um AABB.
void debugDrawAABB(const AABB& box, IDebugRenderer& renderer, const DebugColor& color);

} // namespace physics
