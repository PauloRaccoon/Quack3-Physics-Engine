// =============================================================================
// DebugDraw.cpp
// =============================================================================
#include "physics/debug/DebugDraw.h"
#include "physics/geometry/CylinderShape.h"
#include "physics/geometry/CapsuleShape.h"

#include <algorithm>
#include <cmath>

#include "physics/constraints/JointConstraint.h"
#include "physics/geometry/HalfEdgeStructure.h"
#include "physics/geometry/SphereShape.h"

namespace physics {

void IDebugRenderer::drawPoint(const Vector3& p, Real size, const DebugColor& color) {
    drawLine(p - Vector3(size, 0, 0), p + Vector3(size, 0, 0), color);
    drawLine(p - Vector3(0, size, 0), p + Vector3(0, size, 0), color);
    drawLine(p - Vector3(0, 0, size), p + Vector3(0, 0, size), color);
}

void DebugLineBuffer::drawLine(const Vector3& from, const Vector3& to, const DebugColor& color) {
    DebugLine l;
    l.ax = static_cast<float>(from.x); l.ay = static_cast<float>(from.y); l.az = static_cast<float>(from.z);
    l.bx = static_cast<float>(to.x);   l.by = static_cast<float>(to.y);   l.bz = static_cast<float>(to.z);
    l.r = color.r; l.g = color.g; l.b = color.b; l.a = color.a;
    lines_.push_back(l);
}

void debugDrawAABB(const AABB& box, IDebugRenderer& renderer, const DebugColor& color) {
    if (!box.isValid() || !box.isFinite()) return;
    Vector3 c[8];
    for (int i = 0; i < 8; ++i) {
        c[i] = Vector3((i & 1) ? box.max.x : box.min.x, (i & 2) ? box.max.y : box.min.y,
                       (i & 4) ? box.max.z : box.min.z);
    }
    // Duas arestas por vértice na direção "positiva": cada aresta do cubo aparece uma única vez.
    for (int i = 0; i < 8; ++i) {
        for (int bit = 1; bit <= 4; bit <<= 1) {
            if ((i & bit) == 0) renderer.drawLine(c[i], c[i | bit], color);
        }
    }
}

namespace {

/// Três círculos máximos (planos XY, YZ, ZX locais) de uma esfera.
void drawSphereWire(Real radius, const Transform& xf, IDebugRenderer& r, const DebugColor& color,
                    int segments) {
    const int n = std::max(segments, 3);
    for (int plane = 0; plane < 3; ++plane) {
        Vector3 prev;
        for (int i = 0; i <= n; ++i) {
            const Real a = kTwoPi * Real(i) / Real(n);
            const Real c = std::cos(a) * radius, s = std::sin(a) * radius;
            const Vector3 local = plane == 0 ? Vector3(c, s, 0) : (plane == 1 ? Vector3(0, c, s) : Vector3(s, 0, c));
            const Vector3 p = xf.transformPoint(local);
            if (i > 0) r.drawLine(prev, p, color);
            prev = p;
        }
    }
}

/// Cada aresta do poliedro uma vez (a half-edge de menor índice de cada par).
void drawPolyhedronWire(const HalfEdgeStructure& he, const Transform& xf, IDebugRenderer& r,
                        const DebugColor& color) {
    const int edges = he.getEdgeCount();
    for (int e = 0; e < edges; ++e) {
        const int twin = he.getEdge(e).twin;
        if (twin >= 0 && twin < e) continue;
        const Vector3 a = xf.transformPoint(he.getVertex(he.getEdge(e).origin).position);
        const Vector3 b = xf.transformPoint(he.getVertex(he.destination(e)).position);
        r.drawLine(a, b, color);
    }
}

/// Anel de `seg` lados no plano ⊥ Y à altura y (espaço local), mais as 4 geratrizes entre dois anéis em y0 e y1.
void drawRing(Real r, Real y, const Transform& xf, IDebugRenderer& rd, const DebugColor& color, int seg) {
    for (int i = 0; i < seg; ++i) {
        const Real a0 = kTwoPi * Real(i) / Real(seg), a1 = kTwoPi * Real(i + 1) / Real(seg);
        rd.drawLine(xf.transformPoint(Vector3(r * std::cos(a0), y, r * std::sin(a0))), xf.transformPoint(Vector3(r * std::cos(a1), y, r * std::sin(a1))), color);
    }
}

void drawCylinderWire(Real r, Real h, const Transform& xf, IDebugRenderer& rd, const DebugColor& color, int segments) {
    const int seg = std::max(8, segments);
    drawRing(r, -h, xf, rd, color, seg);
    drawRing(r, h, xf, rd, color, seg);
    for (int k = 0; k < 4; ++k) {
        const Real a = kHalfPi * Real(k);
        rd.drawLine(xf.transformPoint(Vector3(r * std::cos(a), -h, r * std::sin(a))), xf.transformPoint(Vector3(r * std::cos(a), h, r * std::sin(a))), color);
    }
}

/// Cápsula: dois anéis nas pontas do segmento, 4 geratrizes e, em dois planos (XY e ZY), as duas meias-circunferências de cada ponta.
void drawCapsuleWire(Real r, Real h, const Transform& xf, IDebugRenderer& rd, const DebugColor& color, int segments) {
    const int seg = std::max(8, segments);
    drawRing(r, -h, xf, rd, color, seg);
    drawRing(r, h, xf, rd, color, seg);
    for (int k = 0; k < 4; ++k) {
        const Real a = kHalfPi * Real(k);
        rd.drawLine(xf.transformPoint(Vector3(r * std::cos(a), -h, r * std::sin(a))), xf.transformPoint(Vector3(r * std::cos(a), h, r * std::sin(a))), color);
    }
    const int half = std::max(4, seg / 2);
    for (int plane = 0; plane < 2; ++plane)
        for (int cap = 0; cap < 2; ++cap) {
            const Real sign = cap == 0 ? Real(1) : Real(-1);
            for (int i = 0; i < half; ++i) {
                const Real a0 = kPi * Real(i) / Real(half), a1 = kPi * Real(i + 1) / Real(half);
                auto pt = [&](Real a) {
                    const Real u = r * std::cos(a), y = sign * (h + r * std::sin(a));
                    return xf.transformPoint(plane == 0 ? Vector3(u, y, Real(0)) : Vector3(Real(0), y, u));
                };
                rd.drawLine(pt(a0), pt(a1), color);
            }
        }
}

const DebugColor& bodyColor(const RigidBody& b, const DebugDrawSettings& s) {
    return b.isStatic() ? s.staticColor : (b.isKinematic() ? s.kinematicColor : s.dynamicColor);
}

} // namespace

void debugDrawShape(const Shape& shape, const Transform& xf, IDebugRenderer& renderer,
                    const DebugColor& color, int sphereSegments) {
    if (!xf.isFinite()) return;
    switch (shape.getType()) {
        case ShapeType::Sphere:
            drawSphereWire(static_cast<const SphereShape&>(shape).getRadius(), xf, renderer, color, sphereSegments);
            break;
        case ShapeType::Box:
        case ShapeType::Polyhedron:
            if (const HalfEdgeStructure* he = shape.getHalfEdgeStructure()) drawPolyhedronWire(*he, xf, renderer, color);
            break;
        case ShapeType::Capsule: {
            const CapsuleShape& c = static_cast<const CapsuleShape&>(shape);
            drawCapsuleWire(c.getRadius(), c.getHalfHeight(), xf, renderer, color, sphereSegments);
            break;
        }
        case ShapeType::Cylinder: {
            const CylinderShape& c = static_cast<const CylinderShape&>(shape);
            drawCylinderWire(c.getRadius(), c.getHalfHeight(), xf, renderer, color, sphereSegments);
            break;
        }
    }
}

void debugDraw(const PhysicsWorld& world, IDebugRenderer& renderer, const DebugDrawSettings& s) {
    const std::uint32_t f = s.flags;

    // --- Corpos ---------------------------------------------------------------------------------
    world.forEachBody([&](BodyId id, const RigidBody& b) {
        const Transform xf = s.useInterpolatedPose ? world.getInterpolatedTransform(id) : b.getTransform();

        if ((f & kDebugShapes) && b.getShape()) {
            debugDrawShape(*b.getShape(), xf, renderer, bodyColor(b, s), s.sphereSegments);
        }
        if ((f & kDebugAABBs) && b.getShape()) {
            debugDrawAABB(b.getShape()->computeAABB(b.getTransform()), renderer, s.aabbColor);
        }
        if (f & kDebugFatAABBs) {
            AABB fat;
            if (world.getCollisionWorld().getFatAABB(id, fat)) debugDrawAABB(fat, renderer, s.fatAabbColor);
        }
        if (f & kDebugCenterOfMass) {
            renderer.drawPoint(xf.transformPoint(b.getLocalCenterOfMass()), s.pointSize, bodyColor(b, s));
        }
        if (f & kDebugBodyAxes) {
            const Vector3 o = xf.position;
            renderer.drawLine(o, o + xf.transformDirection(Vector3::unitX()) * s.axisLength, DebugColor{255, 70, 70, 255});
            renderer.drawLine(o, o + xf.transformDirection(Vector3::unitY()) * s.axisLength, DebugColor{70, 255, 70, 255});
            renderer.drawLine(o, o + xf.transformDirection(Vector3::unitZ()) * s.axisLength, DebugColor{90, 130, 255, 255});
        }
        if ((f & kDebugVelocities) && !b.isStatic()) {
            const Vector3 com = b.getWorldCenterOfMass();
            renderer.drawLine(com, com + b.getLinearVelocity() * s.velocityScale, s.velocityColor);
        }
    });

    // --- Contatos ------------------------------------------------------------------------------------
    if (f & (kDebugContactPoints | kDebugContactNormals)) {
        for (const ContactPair* cp : world.getContacts()) {
            const ContactManifold& m = cp->manifold;
            for (int i = 0; i < m.count; ++i) {
                const ContactPoint& p = m.points[i];
                const Vector3 pos = p.getPosition();
                if (f & kDebugContactPoints) {
                    renderer.drawPoint(pos, s.pointSize, p.depth >= Real(0) ? s.contactColor : s.speculativeColor);
                }
                if (f & kDebugContactNormals) {
                    renderer.drawLine(pos, pos + m.normal * s.normalLength, s.normalColor);
                }
            }
        }
    }

    // --- Juntas --------------------------------------------------------------------------------------
    if (f & kDebugJoints) {
        world.forEachJoint([&](const JointConstraint& j, BodyId, BodyId) {
            const Vector3 a = j.getAnchorA(), b = j.getAnchorB();
            renderer.drawPoint(a, s.pointSize, s.jointColor);
            renderer.drawPoint(b, s.pointSize, s.jointColor);
            renderer.drawLine(a, b, s.jointColor);
            if (j.getType() == ConstraintType::Hinge) {
                const Vector3 axis = static_cast<const HingeJoint&>(j).getAxisA();
                renderer.drawLine(a - axis * s.axisLength, a + axis * s.axisLength, s.jointColor);
            }
        });
    }
}

} // namespace physics
