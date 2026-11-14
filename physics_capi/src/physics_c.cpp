// =============================================================================
// physics_c.cpp  (V26)
// Implementação da API C (include/physics_c/physics_c.h) sobre o motor C++ de physics_engine/.
//
// Princípios:
//  - NENHUMA exceção atravessa a fronteira C: cada função exportada roda dentro de guarded(...), que converte
//    exceções do motor em códigos PC_ERR_* e guarda o texto em um buffer thread_local (pc_last_error).
//  - Validação nas bordas: ponteiros nulos, valores não finitos e ids inválidos viram PC_ERR_*; o motor nunca
//    recebe lixo vindo de outra linguagem.
//  - Tudo é double na API; o motor usa physics::Real (float ou double): a conversão é feita aqui.
//  - O mundo é dono dos corpos/juntas; as juntas ganham ids próprios (a referência do motor some quando um corpo
//    ligado é destruído, então o id é revalidado a cada uso).
// =============================================================================
#include "physics_c/physics_c.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/CapsuleShape.h"
#include "physics/geometry/CylinderShape.h"
#include "physics/geometry/PolyhedronShape.h"
#include "physics/geometry/SphereShape.h"
#include "physics/pipeline/PhysicsWorld.h"

using physics::Quaternion;
using physics::Real;
using physics::Vector3;

struct pc_world {
    std::unique_ptr<physics::PhysicsWorld> w;
    std::unordered_map<pc_joint_id, const physics::JointConstraint*> joints;   // id -> junta (revalidada a cada uso)
    pc_joint_id nextJoint = 1;
    std::vector<physics::RaycastHit> hitBuf;                                    // buffers reutilizados (sem alocar a cada consulta)
    std::vector<physics::BodyId> idBuf;
};

struct pc_shape {
    std::shared_ptr<const physics::Shape> s;
};

namespace {

thread_local std::string g_lastError;

int fail(int code, const char* msg) {
    g_lastError = msg;
    return code;
}

/// Executa `f` (devolve int) convertendo exceções em códigos de erro.
template <typename F>
int guarded(F&& f) noexcept {
    try {
        return f();
    } catch (const std::invalid_argument& e) {
        return fail(PC_ERR_INVALID_ARGUMENT, e.what());
    } catch (const std::out_of_range& e) {
        return fail(PC_ERR_INVALID_HANDLE, e.what());
    } catch (const std::length_error& e) {
        return fail(PC_ERR_CAPACITY, e.what());
    } catch (const std::bad_alloc&) {
        return fail(PC_ERR_INTERNAL, "memoria insuficiente");
    } catch (const std::exception& e) {
        return fail(PC_ERR_INTERNAL, e.what());
    } catch (...) {
        return fail(PC_ERR_INTERNAL, "erro desconhecido");
    }
}

bool isFin(double v) { return std::isfinite(v); }
bool isFin(const pc_vec3& v) { return isFin(v.x) && isFin(v.y) && isFin(v.z); }
bool isFin(const pc_quat& q) { return isFin(q.w) && isFin(q.x) && isFin(q.y) && isFin(q.z); }

Vector3 toV(const pc_vec3& v) { return Vector3(static_cast<Real>(v.x), static_cast<Real>(v.y), static_cast<Real>(v.z)); }
pc_vec3 fromV(const Vector3& v) { return pc_vec3{static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)}; }
pc_quat fromQ(const Quaternion& q) { return pc_quat{static_cast<double>(q.w), static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z)}; }

/// Quaternio da API -> motor, normalizado (norma ~0 ou não finito: nulo).
bool toQ(const pc_quat& q, Quaternion& out) {
    if (!isFin(q)) return false;
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 1e-12)) return false;
    out = Quaternion(static_cast<Real>(q.w / n), static_cast<Real>(q.x / n), static_cast<Real>(q.y / n), static_cast<Real>(q.z / n));
    return true;
}

#define PC_NEED(cond, msg) do { if (!(cond)) return fail(PC_ERR_INVALID_ARGUMENT, msg); } while (0)
#define PC_WORLD(wp) do { if (!(wp) || !(wp)->w) return fail(PC_ERR_INVALID_HANDLE, "mundo nulo"); } while (0)
#define PC_BODY(wp, id) do { PC_WORLD(wp); if (!(wp)->w->isValidBody(id)) return fail(PC_ERR_INVALID_HANDLE, "corpo inexistente"); } while (0)

physics::QueryFilter makeFilter(uint32_t flags, pc_body_id ignore) {
    physics::QueryFilter f;
    f.includeStatic = (flags & PC_QUERY_STATIC) != 0;
    f.includeKinematic = (flags & PC_QUERY_KINEMATIC) != 0;
    f.includeDynamic = (flags & PC_QUERY_DYNAMIC) != 0;
    f.ignoreBody = ignore;
    return f;
}

pc_raycast_hit fromHit(const physics::RaycastHit& h) {
    pc_raycast_hit o;
    o.body = h.body;
    o.distance = static_cast<double>(h.distance);
    o.point = fromV(h.point);
    o.normal = fromV(h.normal);
    o.started_inside = h.startedInside ? 1 : 0;
    return o;
}

pc_shape* makeShape(std::shared_ptr<const physics::Shape> s) {
    auto* h = new pc_shape;
    h->s = std::move(s);
    return h;
}

/// Cria uma forma; em erro devolve nullptr e guarda a mensagem.
template <typename F>
pc_shape* shapeGuard(F&& f) noexcept {
    try {
        return f();
    } catch (const std::exception& e) {
        g_lastError = e.what();
    } catch (...) {
        g_lastError = "erro desconhecido";
    }
    return nullptr;
}

} // namespace

// ===== versão / erro ================================================================================================
extern "C" {

PC_API int pc_abi_version(void) { return PC_ABI_VERSION; }
PC_API const char* pc_version_string(void) { return "1.0.0"; }
PC_API int pc_real_is_double(void) {
#ifdef PHYSICS_USE_DOUBLE
    return 1;
#else
    return 0;
#endif
}
PC_API const char* pc_last_error(void) { return g_lastError.c_str(); }

// ===== mundo ========================================================================================================
PC_API int pc_world_settings_default(pc_world_settings* out) {
    PC_NEED(out, "out nulo");
    const physics::PhysicsWorldSettings d;
    out->gravity = fromV(d.gravity);
    out->fixed_time_step = static_cast<double>(d.fixedTimeStep);
    out->max_sub_steps = d.maxSubSteps;
    out->max_bodies = static_cast<uint32_t>(d.maxBodies);
    out->velocity_iterations = d.solver.velocityIterations;
    out->position_iterations = d.solver.positionIterations;
    out->warm_starting = d.solver.warmStarting ? 1 : 0;
    out->orientation_exponential = d.integrator.orientation == physics::OrientationIntegration::Exponential ? 1 : 0;
    out->sleep_enabled = d.sleep.enabled ? 1 : 0;
    out->sleep_time = static_cast<double>(d.sleep.timeToSleep);
    return PC_OK;
}

PC_API pc_world* pc_world_create(const pc_world_settings* settings) {
    try {
        physics::PhysicsWorldSettings s;
        if (settings) {
            if (!isFin(settings->gravity) || !isFin(settings->fixed_time_step) || !isFin(settings->sleep_time) ||
                settings->max_sub_steps < 1 || settings->max_bodies < 1 || settings->velocity_iterations < 1 ||
                settings->position_iterations < 0) {
                g_lastError = "pc_world_settings invalido";
                return nullptr;
            }
            s.gravity = toV(settings->gravity);
            s.fixedTimeStep = static_cast<Real>(settings->fixed_time_step);
            s.maxSubSteps = settings->max_sub_steps;
            s.maxBodies = settings->max_bodies;
            s.solver.velocityIterations = settings->velocity_iterations;
            s.solver.positionIterations = settings->position_iterations;
            s.solver.warmStarting = settings->warm_starting != 0;
            s.integrator.orientation = settings->orientation_exponential ? physics::OrientationIntegration::Exponential
                                                                         : physics::OrientationIntegration::Euler;
            s.sleep.enabled = settings->sleep_enabled != 0;
            s.sleep.timeToSleep = static_cast<Real>(settings->sleep_time);
        }
        auto* h = new pc_world;
        try {
            h->w = std::make_unique<physics::PhysicsWorld>(s);
        } catch (...) {
            delete h;
            throw;
        }
        return h;
    } catch (const std::exception& e) {
        g_lastError = e.what();
    } catch (...) {
        g_lastError = "erro desconhecido";
    }
    return nullptr;
}

PC_API void pc_world_destroy(pc_world* world) { delete world; }

PC_API int pc_world_update(pc_world* world, double frame_time) {
    PC_WORLD(world);
    PC_NEED(isFin(frame_time) && frame_time >= 0.0, "frame_time deve ser finito e >= 0");
    return guarded([&] { return world->w->update(static_cast<Real>(frame_time)); });
}

PC_API int pc_world_step(pc_world* world) {
    PC_WORLD(world);
    return guarded([&] { world->w->step(); return PC_OK; });
}

PC_API int pc_world_get_alpha(const pc_world* world, double* out_alpha) {
    PC_WORLD(world);
    PC_NEED(out_alpha, "out nulo");
    *out_alpha = static_cast<double>(world->w->getAlpha());
    return PC_OK;
}

PC_API int pc_world_get_gravity(const pc_world* world, pc_vec3* out) {
    PC_WORLD(world);
    PC_NEED(out, "out nulo");
    *out = fromV(world->w->getGravity());
    return PC_OK;
}

PC_API int pc_world_set_gravity(pc_world* world, const pc_vec3* gravity) {
    PC_WORLD(world);
    PC_NEED(gravity && isFin(*gravity), "gravidade invalida");
    world->w->setGravity(toV(*gravity));
    return PC_OK;
}

PC_API int pc_world_get_fixed_time_step(const pc_world* world, double* out) {
    PC_WORLD(world);
    PC_NEED(out, "out nulo");
    *out = static_cast<double>(world->w->getFixedTimeStep());
    return PC_OK;
}

PC_API int pc_world_set_fixed_time_step(pc_world* world, double dt) {
    PC_WORLD(world);
    PC_NEED(isFin(dt), "dt nao finito");
    return world->w->setFixedTimeStep(static_cast<Real>(dt)) ? PC_OK : fail(PC_ERR_INVALID_ARGUMENT, "dt fora de [1e-4, 1]");
}

PC_API int pc_world_set_solver_iterations(pc_world* world, int32_t velocity_iterations, int32_t position_iterations) {
    PC_WORLD(world);
    PC_NEED(velocity_iterations >= 1 && velocity_iterations <= 1000 && position_iterations >= 0 && position_iterations <= 1000,
            "iteracoes fora de faixa (velocidade 1..1000, posicao 0..1000)");
    world->w->solverSettings().velocityIterations = velocity_iterations;
    world->w->solverSettings().positionIterations = position_iterations;
    return PC_OK;
}

PC_API int pc_world_set_orientation_exponential(pc_world* world, int32_t on) {
    PC_WORLD(world);
    world->w->integratorSettings().orientation = on ? physics::OrientationIntegration::Exponential : physics::OrientationIntegration::Euler;
    return PC_OK;
}

PC_API int pc_world_set_sleep(pc_world* world, int32_t enabled, double sleep_time) {
    PC_WORLD(world);
    PC_NEED(isFin(sleep_time) && sleep_time >= 0.0, "sleep_time invalido");
    world->w->sleepSettings().enabled = enabled != 0;
    world->w->sleepSettings().timeToSleep = static_cast<Real>(sleep_time);
    if (!enabled) world->w->wakeAll();
    return PC_OK;
}

PC_API int pc_world_wake_all(pc_world* world) {
    PC_WORLD(world);
    world->w->wakeAll();
    return PC_OK;
}

PC_API int pc_world_get_stats(const pc_world* world, pc_world_stats* out) {
    PC_WORLD(world);
    PC_NEED(out, "out nulo");
    const physics::WorldStats st = world->w->getStats();
    out->bodies = st.bodies;
    out->awake_bodies = st.awakeBodies;
    out->sleeping_bodies = st.sleepingBodies;
    out->islands = st.islands;
    out->joints = st.joints;
    out->pairs = st.pairs;
    out->touching_pairs = st.touchingPairs;
    out->contact_points = st.contactPoints;
    out->step_count = st.stepCount;
    return PC_OK;
}

// ===== formas =======================================================================================================
PC_API pc_shape* pc_shape_create_sphere(double radius) {
    return shapeGuard([&]() -> pc_shape* {
        if (!isFin(radius)) throw std::invalid_argument("raio nao finito");
        return makeShape(std::make_shared<physics::SphereShape>(static_cast<Real>(radius)));
    });
}

PC_API pc_shape* pc_shape_create_box(const pc_vec3* half_extents) {
    return shapeGuard([&]() -> pc_shape* {
        if (!half_extents || !isFin(*half_extents)) throw std::invalid_argument("half_extents invalido");
        return makeShape(std::make_shared<physics::BoxShape>(toV(*half_extents)));
    });
}

PC_API pc_shape* pc_shape_create_capsule(double radius, double half_height) {
    return shapeGuard([&]() -> pc_shape* {
        if (!isFin(radius) || !isFin(half_height)) throw std::invalid_argument("medida nao finita");
        return makeShape(std::make_shared<physics::CapsuleShape>(static_cast<Real>(radius), static_cast<Real>(half_height)));
    });
}

PC_API pc_shape* pc_shape_create_cylinder(double radius, double half_height) {
    return shapeGuard([&]() -> pc_shape* {
        if (!isFin(radius) || !isFin(half_height)) throw std::invalid_argument("medida nao finita");
        return makeShape(std::make_shared<physics::CylinderShape>(static_cast<Real>(radius), static_cast<Real>(half_height)));
    });
}

PC_API pc_shape* pc_shape_create_polyhedron(const pc_vec3* vertices, int32_t n_vertices, const int32_t* face_indices,
                                            const int32_t* face_sizes, int32_t n_faces) {
    return shapeGuard([&]() -> pc_shape* {
        if (!vertices || !face_indices || !face_sizes || n_vertices < 4 || n_faces < 4 || n_vertices > 100000 || n_faces > 100000)
            throw std::invalid_argument("poliedro: ponteiros nulos ou contagens invalidas (minimo 4 vertices e 4 faces)");
        std::vector<Vector3> v;
        v.reserve(static_cast<std::size_t>(n_vertices));
        for (int32_t i = 0; i < n_vertices; ++i) {
            if (!isFin(vertices[i])) throw std::invalid_argument("vertice nao finito");
            v.push_back(toV(vertices[i]));
        }
        std::vector<std::vector<int>> faces;
        faces.reserve(static_cast<std::size_t>(n_faces));
        std::size_t k = 0;
        for (int32_t f = 0; f < n_faces; ++f) {
            const int32_t sz = face_sizes[f];
            if (sz < 3 || sz > 1000) throw std::invalid_argument("face com tamanho invalido (3..1000 vertices)");
            std::vector<int> face(static_cast<std::size_t>(sz));
            for (int32_t j = 0; j < sz; ++j, ++k) {
                const int32_t idx = face_indices[k];
                if (idx < 0 || idx >= n_vertices) throw std::invalid_argument("indice de vertice fora da faixa");
                face[static_cast<std::size_t>(j)] = idx;
            }
            faces.push_back(std::move(face));
        }
        return makeShape(std::make_shared<physics::PolyhedronShape>(v, faces));
    });
}

PC_API void pc_shape_destroy(pc_shape* shape) { delete shape; }

// ===== corpos =======================================================================================================
PC_API int pc_body_desc_default(pc_body_desc* out) {
    PC_NEED(out, "out nulo");
    std::memset(out, 0, sizeof(*out));
    out->type = PC_BODY_DYNAMIC;
    out->density = 1.0;
    out->rotation = pc_quat{1, 0, 0, 0};
    out->friction = 0.5;
    out->restitution = 0.0;
    out->linear_damping = 0.0;
    out->angular_damping = 0.01;
    out->gravity_scale = 1.0;
    out->allow_sleep = 1;
    return PC_OK;
}

PC_API int pc_body_create(pc_world* world, const pc_body_desc* desc, pc_body_id* out_id) {
    PC_WORLD(world);
    PC_NEED(desc && out_id, "desc ou out_id nulo");
    PC_NEED(desc->shape && desc->shape->s, "desc.shape obrigatorio");
    PC_NEED(desc->type >= PC_BODY_STATIC && desc->type <= PC_BODY_DYNAMIC, "tipo de corpo invalido");
    PC_NEED(isFin(desc->density) && isFin(desc->mass) && isFin(desc->friction) && isFin(desc->restitution) &&
                isFin(desc->linear_damping) && isFin(desc->angular_damping) && isFin(desc->gravity_scale) &&
                isFin(desc->position) && isFin(desc->linear_velocity) && isFin(desc->angular_velocity),
            "desc contem valor nao finito");
    Quaternion q;
    PC_NEED(toQ(desc->rotation, q), "rotation invalida (nao finita ou norma 0)");
    return guarded([&] {
        physics::RigidBodyDesc d;
        d.type = desc->type == PC_BODY_STATIC ? physics::BodyType::Static
                 : desc->type == PC_BODY_KINEMATIC ? physics::BodyType::Kinematic : physics::BodyType::Dynamic;
        d.shape = desc->shape->s;
        d.density = static_cast<Real>(desc->density);
        d.mass = static_cast<Real>(desc->mass);
        d.transform = physics::Transform(toV(desc->position), q);
        d.linearVelocity = toV(desc->linear_velocity);
        d.angularVelocity = toV(desc->angular_velocity);
        d.friction = static_cast<Real>(desc->friction);
        d.restitution = static_cast<Real>(desc->restitution);
        d.linearDamping = static_cast<Real>(desc->linear_damping);
        d.angularDamping = static_cast<Real>(desc->angular_damping);
        d.gravityScale = static_cast<Real>(desc->gravity_scale);
        d.allowSleep = desc->allow_sleep != 0;
        *out_id = world->w->createBody(d);
        return PC_OK;
    });
}

PC_API int pc_body_destroy(pc_world* world, pc_body_id id) {
    PC_BODY(world, id);
    return guarded([&] {
        world->w->destroyBody(id);
        return PC_OK;
    });
}

PC_API int pc_body_is_valid(const pc_world* world, pc_body_id id) { return world && world->w && world->w->isValidBody(id) ? 1 : 0; }

PC_API int pc_body_get_state(const pc_world* world, pc_body_id id, pc_body_state* out) {
    PC_BODY(world, id);
    PC_NEED(out, "out nulo");
    const physics::RigidBody& b = world->w->getBody(id);
    out->type = b.isStatic() ? PC_BODY_STATIC : b.isKinematic() ? PC_BODY_KINEMATIC : PC_BODY_DYNAMIC;
    out->awake = b.isAwake() ? 1 : 0;
    out->mass = static_cast<double>(b.getMass());
    out->position = fromV(b.getTransform().position);
    out->center_of_mass = fromV(b.getWorldCenterOfMass());
    out->rotation = fromQ(b.getOrientation());
    out->linear_velocity = fromV(b.getLinearVelocity());
    out->angular_velocity = fromV(b.getAngularVelocity());
    return PC_OK;
}

PC_API int pc_body_get_interpolated(const pc_world* world, pc_body_id id, pc_vec3* out_position, pc_quat* out_rotation) {
    PC_BODY(world, id);
    PC_NEED(out_position && out_rotation, "out nulo");
    const physics::Transform t = world->w->getInterpolatedTransform(id);
    *out_position = fromV(t.position);
    *out_rotation = fromQ(t.rotation);
    return PC_OK;
}

PC_API int pc_body_set_transform(pc_world* world, pc_body_id id, const pc_vec3* position, const pc_quat* rotation) {
    PC_BODY(world, id);
    PC_NEED(position && rotation && isFin(*position), "posicao/rotacao invalida");
    Quaternion q;
    PC_NEED(toQ(*rotation, q), "rotation invalida (nao finita ou norma 0)");
    return guarded([&] {
        world->w->setBodyTransform(id, physics::Transform(toV(*position), q));
        return PC_OK;
    });
}

PC_API int pc_body_set_linear_velocity(pc_world* world, pc_body_id id, const pc_vec3* v) {
    PC_BODY(world, id);
    PC_NEED(v && isFin(*v), "velocidade invalida");
    world->w->getBody(id).setLinearVelocity(toV(*v));
    return PC_OK;
}

PC_API int pc_body_set_angular_velocity(pc_world* world, pc_body_id id, const pc_vec3* w) {
    PC_BODY(world, id);
    PC_NEED(w && isFin(*w), "velocidade angular invalida");
    world->w->getBody(id).setAngularVelocity(toV(*w));
    return PC_OK;
}

PC_API int pc_body_apply_force(pc_world* world, pc_body_id id, const pc_vec3* force) {
    PC_BODY(world, id);
    PC_NEED(force && isFin(*force), "forca invalida");
    world->w->getBody(id).applyForce(toV(*force));
    return PC_OK;
}

PC_API int pc_body_apply_force_at_point(pc_world* world, pc_body_id id, const pc_vec3* force, const pc_vec3* world_point) {
    PC_BODY(world, id);
    PC_NEED(force && world_point && isFin(*force) && isFin(*world_point), "forca/ponto invalido");
    world->w->getBody(id).applyForceAtPoint(toV(*force), toV(*world_point));
    return PC_OK;
}

PC_API int pc_body_apply_torque(pc_world* world, pc_body_id id, const pc_vec3* torque) {
    PC_BODY(world, id);
    PC_NEED(torque && isFin(*torque), "torque invalido");
    world->w->getBody(id).applyTorque(toV(*torque));
    return PC_OK;
}

PC_API int pc_body_apply_impulse_at_point(pc_world* world, pc_body_id id, const pc_vec3* impulse, const pc_vec3* world_point) {
    PC_BODY(world, id);
    PC_NEED(impulse && world_point && isFin(*impulse) && isFin(*world_point), "impulso/ponto invalido");
    world->w->getBody(id).applyImpulseAtPoint(toV(*impulse), toV(*world_point));
    return PC_OK;
}

PC_API int pc_body_set_material(pc_world* world, pc_body_id id, double friction, double restitution) {
    PC_BODY(world, id);
    PC_NEED(isFin(friction) && isFin(restitution), "valor nao finito");
    world->w->getBody(id).setFriction(static_cast<Real>(friction));
    world->w->getBody(id).setRestitution(static_cast<Real>(restitution));
    return PC_OK;
}

PC_API int pc_body_set_damping(pc_world* world, pc_body_id id, double linear, double angular) {
    PC_BODY(world, id);
    PC_NEED(isFin(linear) && isFin(angular), "valor nao finito");
    world->w->getBody(id).setLinearDamping(static_cast<Real>(linear));
    world->w->getBody(id).setAngularDamping(static_cast<Real>(angular));
    return PC_OK;
}

PC_API int pc_body_set_gravity_scale(pc_world* world, pc_body_id id, double scale) {
    PC_BODY(world, id);
    PC_NEED(isFin(scale), "valor nao finito");
    world->w->getBody(id).setGravityScale(static_cast<Real>(scale));
    return PC_OK;
}

PC_API int pc_body_set_awake(pc_world* world, pc_body_id id, int32_t awake) {
    PC_BODY(world, id);
    world->w->setAwake(id, awake != 0);
    return PC_OK;
}

PC_API int pc_world_read_transforms(const pc_world* world, int32_t interpolated, uint32_t capacity, pc_body_id* ids, double* data) {
    PC_WORLD(world);
    return guarded([&] {
        uint32_t n = 0;
        world->w->forEachBody([&](physics::BodyId id, const physics::RigidBody& b) {
            if (n < capacity && ids && data) {
                ids[n] = id;
                double* d = data + 7 * static_cast<std::size_t>(n);
                Vector3 p = b.getTransform().position;
                Quaternion q = b.getOrientation();
                if (interpolated) {
                    const physics::Transform t = world->w->getInterpolatedTransform(id);
                    p = t.position;
                    q = t.rotation;
                }
                d[0] = static_cast<double>(p.x); d[1] = static_cast<double>(p.y); d[2] = static_cast<double>(p.z);
                d[3] = static_cast<double>(q.w); d[4] = static_cast<double>(q.x); d[5] = static_cast<double>(q.y); d[6] = static_cast<double>(q.z);
            }
            ++n;
        });
        return static_cast<int>(n);
    });
}

// ===== juntas =======================================================================================================
namespace {

/// Registra a junta recém-criada e devolve o id.
pc_joint_id registerJoint(pc_world* world, const physics::JointConstraint& j) {
    const pc_joint_id id = world->nextJoint++;
    world->joints[id] = &j;
    return id;
}

/// Junta do id, se ainda viva no mundo (ponteiro revalidado contra a lista do motor); senão nullptr.
const physics::JointConstraint* findJoint(const pc_world* world, pc_joint_id id) {
    const auto it = world->joints.find(id);
    if (it == world->joints.end()) return nullptr;
    bool alive = false;
    world->w->forEachJoint([&](const physics::JointConstraint& j, physics::BodyId, physics::BodyId) {
        if (&j == it->second) alive = true;
    });
    return alive ? it->second : nullptr;
}

} // namespace

PC_API int pc_joint_create_ball_socket(pc_world* world, pc_body_id a, pc_body_id b, const pc_vec3* anchor, int32_t collide_connected,
                                       pc_joint_id* out_id) {
    PC_BODY(world, a);
    PC_BODY(world, b);
    PC_NEED(anchor && out_id && isFin(*anchor), "ancora invalida");
    return guarded([&] {
        *out_id = registerJoint(world, world->w->createBallSocketJoint(a, b, toV(*anchor), collide_connected != 0));
        return PC_OK;
    });
}

PC_API int pc_joint_create_distance(pc_world* world, pc_body_id a, pc_body_id b, const pc_vec3* anchor_a, const pc_vec3* anchor_b,
                                    double length, int32_t collide_connected, pc_joint_id* out_id) {
    PC_BODY(world, a);
    PC_BODY(world, b);
    PC_NEED(anchor_a && anchor_b && out_id && isFin(*anchor_a) && isFin(*anchor_b) && isFin(length), "ancoras/comprimento invalidos");
    return guarded([&] {
        *out_id = registerJoint(world, world->w->createDistanceJoint(a, b, toV(*anchor_a), toV(*anchor_b), static_cast<Real>(length),
                                                                      collide_connected != 0));
        return PC_OK;
    });
}

PC_API int pc_joint_create_hinge(pc_world* world, pc_body_id a, pc_body_id b, const pc_vec3* anchor, const pc_vec3* axis,
                                 int32_t collide_connected, pc_joint_id* out_id) {
    PC_BODY(world, a);
    PC_BODY(world, b);
    PC_NEED(anchor && axis && out_id && isFin(*anchor) && isFin(*axis), "ancora/eixo invalidos");
    return guarded([&] {
        *out_id = registerJoint(world, world->w->createHingeJoint(a, b, toV(*anchor), toV(*axis), collide_connected != 0));
        return PC_OK;
    });
}

PC_API int pc_joint_destroy(pc_world* world, pc_joint_id id) {
    PC_WORLD(world);
    const physics::JointConstraint* j = findJoint(world, id);
    if (!j) {
        world->joints.erase(id);
        return fail(PC_ERR_INVALID_HANDLE, "junta inexistente");
    }
    return guarded([&] {
        world->w->destroyJoint(*j);
        world->joints.erase(id);
        return PC_OK;
    });
}

PC_API int pc_joint_is_valid(const pc_world* world, pc_joint_id id) { return world && world->w && findJoint(world, id) ? 1 : 0; }

// ===== eventos ======================================================================================================
PC_API int pc_world_read_contact_events(const pc_world* world, uint32_t capacity, pc_contact_event* out) {
    PC_WORLD(world);
    const std::vector<physics::ContactEvent>& ev = world->w->getContactEvents();
    for (uint32_t i = 0; out && i < capacity && i < ev.size(); ++i) {
        const physics::ContactEvent& e = ev[i];
        pc_contact_event& o = out[i];
        o.type = e.type == physics::ContactEvent::Type::Begin ? PC_CONTACT_BEGIN : PC_CONTACT_END;
        o.body_a = e.a;
        o.body_b = e.b;
        o.point = fromV(e.point);
        o.normal = fromV(e.normal);
        o.depth = static_cast<double>(e.depth);
        o.point_count = e.pointCount;
        o.normal_impulse = static_cast<double>(e.normalImpulse);
        o.approach_speed = static_cast<double>(e.approachSpeed);
    }
    return static_cast<int>(ev.size());
}

PC_API int pc_world_clear_contact_events(pc_world* world) {
    PC_WORLD(world);
    world->w->clearContactEvents();
    return PC_OK;
}

// ===== consultas ====================================================================================================
PC_API int pc_world_raycast(const pc_world* world, const pc_vec3* origin, const pc_vec3* direction, double max_distance,
                            uint32_t query_flags, pc_body_id ignore_body, pc_raycast_hit* out) {
    PC_WORLD(world);
    PC_NEED(origin && direction && out && isFin(*origin) && isFin(*direction) && !std::isnan(max_distance), "raio invalido");
    return guarded([&] {
        physics::RaycastHit h;
        if (!world->w->raycast(toV(*origin), toV(*direction), static_cast<Real>(max_distance), h, makeFilter(query_flags, ignore_body))) return 0;
        *out = fromHit(h);
        return 1;
    });
}

PC_API int pc_world_raycast_all(const pc_world* world, const pc_vec3* origin, const pc_vec3* direction, double max_distance,
                                uint32_t query_flags, pc_body_id ignore_body, uint32_t capacity, pc_raycast_hit* out) {
    PC_WORLD(world);
    PC_NEED(origin && direction && isFin(*origin) && isFin(*direction) && !std::isnan(max_distance), "raio invalido");
    return guarded([&] {
        std::vector<physics::RaycastHit> hits;   // pequeno e fora do laço de simulação
        const int n = world->w->raycastAll(toV(*origin), toV(*direction), static_cast<Real>(max_distance), hits, makeFilter(query_flags, ignore_body));
        for (uint32_t i = 0; out && i < capacity && i < hits.size(); ++i) out[i] = fromHit(hits[i]);
        return n;
    });
}

PC_API int pc_world_overlap_sphere(const pc_world* world, const pc_vec3* center, double radius, uint32_t query_flags,
                                   pc_body_id ignore_body, uint32_t capacity, pc_body_id* out_ids) {
    PC_WORLD(world);
    PC_NEED(center && isFin(*center) && isFin(radius), "esfera invalida");
    return guarded([&] {
        std::vector<physics::BodyId> ids;
        const int n = world->w->overlapSphere(toV(*center), static_cast<Real>(radius), ids, makeFilter(query_flags, ignore_body));
        for (uint32_t i = 0; out_ids && i < capacity && i < ids.size(); ++i) out_ids[i] = ids[i];
        return n;
    });
}

PC_API int pc_world_overlap_point(const pc_world* world, const pc_vec3* point, uint32_t query_flags, pc_body_id ignore_body,
                                  uint32_t capacity, pc_body_id* out_ids) {
    PC_WORLD(world);
    PC_NEED(point && isFin(*point), "ponto invalido");
    return guarded([&] {
        std::vector<physics::BodyId> ids;
        const int n = world->w->overlapPoint(toV(*point), ids, makeFilter(query_flags, ignore_body));
        for (uint32_t i = 0; out_ids && i < capacity && i < ids.size(); ++i) out_ids[i] = ids[i];
        return n;
    });
}

} // extern "C"
