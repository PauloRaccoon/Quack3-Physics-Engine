// =============================================================================
// Shape.h
// Interface abstrata das formas CONVEXAS do motor. Todas as consultas são em
// espaço LOCAL da forma; a transformação para o mundo é feita por Transform.
//
// A função de suporte é a única primitiva geométrica que GJK/EPA precisam:
//     support(d) = argmax_{p ∈ Shape} (p · d)
// =============================================================================
#pragma once

#include <cstdint>

#include "physics/common/Types.h"
#include "physics/geometry/AABB.h"
#include "physics/math/Matrix3.h"
#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

namespace physics {

enum class ShapeType : std::uint8_t { Sphere, Box, Polyhedron, Capsule, Cylinder };

class HalfEdgeStructure;  // declaração antecipada (ver HalfEdgeStructure.h)

class Shape {
public:
    virtual ~Shape() = default;

    virtual ShapeType getType() const = 0;

    /// Topologia (vértices/arestas/faces) para formas poliédricas; nullptr para
    /// formas curvas (esfera). Usada pela narrowphase para gerar contatos de
    /// face por clipping (Sutherland-Hodgman). Os índices de vértice coincidem
    /// com os FeatureId devolvidos por getSupportFeature().
    virtual const HalfEdgeStructure* getHalfEdgeStructure() const { return nullptr; }

    /// Ponto da forma mais distante na direção `direction` (espaço local).
    /// `direction` não precisa ser unitário; direção nula devolve um ponto
    /// válido qualquer da forma.
    virtual Vector3 getSupport(const Vector3& direction) const = 0;

    /// Identificador da feature (vértice) escolhida por getSupport(). Usado
    /// pelo ContactManifold para casar contatos entre quadros.
    virtual FeatureId getSupportFeature(const Vector3& direction) const {
        (void)direction;
        return 0;
    }

    /// Maior número de vértices que getSupportFace() devolve.
    static constexpr int kMaxFaceVertices = 32;

    /// Polígono (espaço LOCAL) da face mais alinhada com `localDirection`, com
    /// os vértices em sentido anti-horário visto de fora (normal = regra da mão
    /// direita). Usado pelo clipping de faces para gerar vários pontos de
    /// contato de uma vez. Retorna o nº de vértices escritos em `outVertices`,
    /// ou 0 se a forma não tem faces planas (esfera) ou a face não cabe em
    /// `maxVertices` — o chamador então usa um único ponto de contato.
    virtual int getSupportFace(const Vector3& localDirection, Vector3* outVertices,
                               int maxVertices) const {
        (void)localDirection;
        (void)outVertices;
        (void)maxVertices;
        return 0;
    }

    /// NÚCLEO + RAIO (V18). Uma forma "arredondada" é a soma de Minkowski de um núcleo (ponto ou segmento) com uma
    /// esfera de raio `getCoreRadius()`: esfera = ponto + r; cápsula = segmento + r. GJK/EPA rodam só no núcleo e o
    /// gerador de contatos soma o raio depois, o que os torna EXATOS para superfícies curvas. Formas comuns: o núcleo
    /// é a própria forma e o raio é 0 (as definições padrão abaixo).
    virtual Real getCoreRadius() const { return Real(0); }
    /// Ponto do núcleo mais distante na direção local `direction` (espaço local).
    virtual Vector3 getCoreSupport(const Vector3& direction) const { return getSupport(direction); }

    /// Aresta (segmento) extrema na direção local `direction`, em espaço local, quando a feature mais alinhada da forma
    /// é uma LINHA e não um ponto nem uma face: o núcleo da cápsula; a geratriz do cilindro deitado. Usada pelo gerador de
    /// contatos para dar dois pontos de contato (uma cápsula/cilindro deitados não balançam). false = não há segmento.
    virtual bool getSupportSegment(const Vector3& direction, Vector3& p0, Vector3& p1) const {
        (void)direction;
        (void)p0;
        (void)p1;
        return false;
    }

    /// AABB justo no mundo. A implementação padrão consulta a função de
    /// suporte nos 6 eixos do mundo (exata para qualquer forma convexa);
    /// formas simples sobrescrevem com fórmula fechada.
    virtual AABB computeAABB(const Transform& xf) const;

    /// Volume (densidade 1 → massa = volume).
    virtual Real getVolume() const = 0;

    /// Centro de massa em espaço local (assumindo densidade uniforme).
    virtual Vector3 getCentroid() const = 0;

    /// Tensor de inércia em torno do CENTRO DE MASSA, em eixos locais, para a
    /// massa dada. (A Fase 3 combina isso com o deslocamento do centroide.)
    virtual Matrix3 computeInertia(Real mass) const = 0;

    /// Suporte no mundo: direção em mundo → ponto em mundo.
    Vector3 getWorldSupport(const Transform& xf, const Vector3& worldDirection) const {
        return xf.transformPoint(getSupport(xf.inverseTransformDirection(worldDirection)));
    }

protected:
    // Construtores protegidos: impedem "slicing" acidental ao copiar via base.
    Shape() = default;
    Shape(const Shape&) = default;
    Shape& operator=(const Shape&) = default;
};

} // namespace physics
