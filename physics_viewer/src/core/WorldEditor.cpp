// =============================================================================
// WorldEditor.cpp
// =============================================================================
#include "viewer/core/WorldEditor.h"

#include <algorithm>
#include <cmath>

namespace viewer {

using namespace physics;

namespace {
Real clampTo(Real v, Real lo, Real hi) { return std::min(std::max(v, lo), hi); }
int clampInt(int v, int lo, int hi) { return std::min(std::max(v, lo), hi); }
} // namespace

WorldParams captureWorld(const PhysicsWorld& world) {
    WorldParams p;
    p.gravity = world.getGravity();
    p.fixedDt = world.getFixedTimeStep();
    const SolverSettings& s = world.solverSettings();
    p.velocityIterations = s.velocityIterations;
    p.positionIterations = s.positionIterations;
    p.linearSlop = s.linearSlop;
    p.velocitySlop = s.velocitySlop;
    p.warmStarting = s.warmStarting;
    p.sleepEnabled = world.sleepSettings().enabled;
    p.sleepTime = world.sleepSettings().timeToSleep;
    return p;
}

bool setWorldGravity(PhysicsWorld& world, const Vector3& g) {
    if (!g.isFinite()) return false;
    const Real m = worldlimits::kMaxGravity;
    world.setGravity(Vector3(clampTo(g.x, -m, m), clampTo(g.y, -m, m), clampTo(g.z, -m, m)));
    return true;
}

bool setWorldFixedStep(PhysicsWorld& world, Real dt) { return world.setFixedTimeStep(dt); }

bool setVelocityIterations(PhysicsWorld& world, int n) {
    world.solverSettings().velocityIterations = clampInt(n, 1, worldlimits::kMaxVelocityIterations);
    return true;
}

bool setPositionIterations(PhysicsWorld& world, int n) {
    world.solverSettings().positionIterations = clampInt(n, 0, worldlimits::kMaxPositionIterations);
    return true;
}

bool setLinearSlop(PhysicsWorld& world, Real v) {
    if (!std::isfinite(v)) return false;
    world.solverSettings().linearSlop = clampTo(v, Real(0), worldlimits::kMaxLinearSlop);
    return true;
}

bool setVelocitySlop(PhysicsWorld& world, Real v) {
    if (!std::isfinite(v)) return false;
    world.solverSettings().velocitySlop = clampTo(v, Real(0), worldlimits::kMaxVelocitySlop);
    return true;
}

void setWarmStarting(PhysicsWorld& world, bool on) { world.solverSettings().warmStarting = on; }

void setSleepEnabled(PhysicsWorld& world, bool on) {
    const bool was = world.sleepSettings().enabled;
    world.sleepSettings().enabled = on;
    if (was && !on) world.wakeAll();
}

bool setSleepTime(PhysicsWorld& world, Real seconds) {
    if (!std::isfinite(seconds)) return false;
    world.sleepSettings().timeToSleep = clampTo(seconds, worldlimits::kMinSleepTime, worldlimits::kMaxSleepTime);
    return true;
}

void applyWorld(PhysicsWorld& world, const WorldParams& p) {
    if (!world.getGravity().nearlyEqual(p.gravity, Real(1e-9))) setWorldGravity(world, p.gravity);
    setWorldFixedStep(world, p.fixedDt);
    setVelocityIterations(world, p.velocityIterations);
    setPositionIterations(world, p.positionIterations);
    setLinearSlop(world, p.linearSlop);
    setVelocitySlop(world, p.velocitySlop);
    setWarmStarting(world, p.warmStarting);
    setSleepEnabled(world, p.sleepEnabled);
    setSleepTime(world, p.sleepTime);
}

const std::vector<DebugFlagInfo>& debugFlagTable() {
    static const std::vector<DebugFlagInfo> table = {
        {"formas", kDebugShapes},
        {"AABB", kDebugAABBs},
        {"Fat AABB", kDebugFatAABBs},
        {"pontos de contato", kDebugContactPoints},
        {"normais de contato", kDebugContactNormals},
        {"juntas", kDebugJoints},
        {"centro de massa", kDebugCenterOfMass},
        {"velocidades", kDebugVelocities},
        {"eixos locais", kDebugBodyAxes},
    };
    return table;
}

std::size_t buildDebugLines(const PhysicsWorld& world, std::uint32_t flags, DebugLineBuffer& out, bool interpolatedPose) {
    out.clear();
    if (flags == 0) return 0;
    DebugDrawSettings s;
    s.flags = flags;
    s.useInterpolatedPose = interpolatedPose;
    s.sphereSegments = 16;
    debugDraw(world, out, s);
    return out.size();
}

} // namespace viewer
