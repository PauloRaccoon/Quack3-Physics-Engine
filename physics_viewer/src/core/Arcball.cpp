// =============================================================================
// Arcball.cpp
// =============================================================================
#include "viewer/core/Arcball.h"

#include <cmath>

namespace viewer {

using physics::Quaternion;
using physics::Real;
using physics::Vector3;

ArcballView arcballViewFromCamera(const OrbitCamera& cam) {
    const Vec3f p = cam.position();
    Vector3 fwd(static_cast<Real>(cam.target.x - p.x), static_cast<Real>(cam.target.y - p.y), static_cast<Real>(cam.target.z - p.z));
    fwd = fwd.normalized();
    if (fwd.lengthSquared() < Real(0.5)) fwd = Vector3(0, 0, -1);
    Vector3 right = fwd.cross(Vector3(0, 1, 0));
    if (right.lengthSquared() < Real(1e-8)) right = Vector3(1, 0, 0);   // olhando reto para cima/baixo (a câmera não deixa, mas por segurança)
    right = right.normalized();
    ArcballView v;
    v.right = right;
    v.up = right.cross(fwd).normalized();
    v.back = fwd * Real(-1);
    return v;
}

Vector3 arcballSpherePoint(double nx, double ny) {
    if (!std::isfinite(nx) || !std::isfinite(ny)) return Vector3(0, 0, 1);
    const double r2 = nx * nx + ny * ny;
    if (r2 <= 1.0) return Vector3(static_cast<Real>(nx), static_cast<Real>(ny), static_cast<Real>(std::sqrt(1.0 - r2)));
    const double r = std::sqrt(r2);
    return Vector3(static_cast<Real>(nx / r), static_cast<Real>(ny / r), Real(0));
}

Quaternion arcballDelta(const Vector3& a, const Vector3& b) {
    if (!a.isFinite() || !b.isFinite()) return Quaternion::identity();
    return Quaternion::fromTwoVectors(a.normalized(), b.normalized());
}

Quaternion arcballToWorld(const Quaternion& delta, const ArcballView& view) {
    Vector3 axis;
    Real angle = 0;
    delta.toAxisAngle(axis, angle);
    if (!(std::fabs(angle) > Real(1e-9)) || !axis.isFinite()) return Quaternion::identity();
    const Vector3 w = view.right * axis.x + view.up * axis.y + view.back * axis.z;
    if (w.lengthSquared() < Real(1e-12)) return Quaternion::identity();
    return Quaternion::fromAxisAngle(w.normalized(), angle);
}

void ArcballDrag::begin(double nx, double ny, const Quaternion& startOrientation, const ArcballView&) {
    active_ = true;
    start_ = arcballSpherePoint(nx, ny);
    q0_ = startOrientation.normalized();
}

Quaternion ArcballDrag::update(double nx, double ny, const ArcballView& view) const {
    if (!active_) return q0_;
    const Quaternion d = arcballToWorld(arcballDelta(start_, arcballSpherePoint(nx, ny)), view);
    return (d * q0_).normalized();
}

ArcballPoint arcballProject(const Vector3& v, const ArcballView& view) {
    return ArcballPoint{static_cast<float>(v.dot(view.right)), static_cast<float>(v.dot(view.up)), static_cast<float>(v.dot(view.back))};
}

int arcballCircle(const Quaternion& q, int axis, const ArcballView& view, ArcballPoint* out, int n) {
    if (!out || n < 3 || axis < 0 || axis > 2) return 0;
    const int a = (axis + 1) % 3, b = (axis + 2) % 3;
    const double two_pi = 6.283185307179586;
    for (int i = 0; i < n; ++i) {
        const double t = two_pi * static_cast<double>(i) / static_cast<double>(n);
        Vector3 local(0, 0, 0);
        local[a] = static_cast<Real>(std::cos(t));
        local[b] = static_cast<Real>(std::sin(t));
        out[i] = arcballProject(q.rotate(local), view);
    }
    return n;
}

} // namespace viewer
