// =============================================================================
// SupportFunction.h
// Função de suporte da DIFERENÇA DE MINKOWSKI  A ⊖ B = { a − b }  no mundo:
//
//     support(d) = supportA(d) − supportB(−d)
//
// É a única primitiva geométrica que GJK e EPA consomem. Cada ponto de suporte
// guarda também os pontos originais `a` e `b`, para recuperar os pontos de
// contato em cada forma ("witness points") a partir de coordenadas
// baricêntricas do simplex/politopo.
//
// Formas arredondadas são tratadas como NÚCLEO + RAIO (Shape::getCoreRadius): o núcleo da esfera é um ponto (o
// centro), o da cápsula é um segmento, e o raio é somado pelo gerador de contatos. Isso torna GJK/EPA
// exatos para esferas (sem aproximar uma superfície curva por um politopo).
// =============================================================================
#pragma once

#include "physics/common/Settings.h"
#include "physics/geometry/Shape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

namespace physics {

/// Ponto do espaço de configuração (A ⊖ B) com a sua "procedência".
struct SupportPoint {
    Vector3 point;  ///< a − b
    Vector3 a;      ///< ponto do núcleo de A (mundo)
    Vector3 b;      ///< ponto do núcleo de B (mundo)
};

class MinkowskiSupport {
public:
    MinkowskiSupport(const Shape& shapeA, const Transform& xfA,
                     const Shape& shapeB, const Transform& xfB)
        : shapeA_(&shapeA), shapeB_(&shapeB), xfA_(xfA), xfB_(xfB),
          radiusA_(coreRadius(shapeA)), radiusB_(coreRadius(shapeB)) {}

    /// Ponto extremo de A ⊖ B na direção `worldDirection` (não precisa ser unitária).
    SupportPoint support(const Vector3& worldDirection) const {
        SupportPoint s;
        s.a = coreSupport(*shapeA_, xfA_, radiusA_, worldDirection);
        s.b = coreSupport(*shapeB_, xfB_, radiusB_, -worldDirection);
        s.point = s.a - s.b;
        return s;
    }

    /// Raio somado ao núcleo de A (0 se não for esfera). Idem para B.
    Real getRadiusA() const { return radiusA_; }
    Real getRadiusB() const { return radiusB_; }

    /// Direção inicial do GJK: dos centros (heurística que costuma convergir mais rápido).
    Vector3 getInitialDirection() const {
        const Vector3 d = xfA_.position - xfB_.position;
        return d.lengthSquared() > kNormalizeEpsilonSq ? d : Vector3::unitX();
    }

    const Shape& getShapeA() const { return *shapeA_; }
    const Shape& getShapeB() const { return *shapeB_; }

private:
    static Real coreRadius(const Shape& s) { return s.getCoreRadius(); }
    static Vector3 coreSupport(const Shape& s, const Transform& xf, Real /*radius*/, const Vector3& d) {
        // Núcleo da forma: esfera = centro; cápsula = segmento; as demais = a própria forma.
        return xf.transformPoint(s.getCoreSupport(xf.inverseTransformDirection(d)));
    }

    const Shape* shapeA_;
    const Shape* shapeB_;
    Transform xfA_;
    Transform xfB_;
    Real radiusA_;
    Real radiusB_;
};

} // namespace physics
