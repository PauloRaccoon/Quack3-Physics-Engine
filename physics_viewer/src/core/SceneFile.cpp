// =============================================================================
// SceneFile.cpp — ver SceneFile.h para o formato
// =============================================================================
#include "viewer/core/SceneFile.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

#include "physics/geometry/BoxShape.h"
#include "physics/geometry/SphereShape.h"
#include "viewer/core/PolyShapes.h"
#include "viewer/core/WorldEditor.h"

namespace viewer {

using namespace physics;

namespace {

// ----------------------------------------------------------------------------------------------------
// Escrita
// ----------------------------------------------------------------------------------------------------
/// Menor representação decimal que lê de volta exatamente o mesmo Real.
std::string num(Real v) {
    char buf[40];
    for (int digits = 6; digits <= 17; ++digits) {
        std::snprintf(buf, sizeof buf, "%.*g", digits, static_cast<double>(v));
        if (static_cast<Real>(std::strtod(buf, nullptr)) == v) break;
    }
    return buf;
}

std::string vec(const Vector3& v) { return num(v.x) + " " + num(v.y) + " " + num(v.z); }

std::string cleanName(const std::string& n) {
    std::string out;
    for (char c : n) {
        if (c == '\n' || c == '\r' || c == '\t') c = ' ';
        out.push_back(c);
        if (out.size() >= scenefile::kMaxNameLength) break;
    }
    return out;
}

// ----------------------------------------------------------------------------------------------------
// Leitura
// ----------------------------------------------------------------------------------------------------
struct ParseError {
    int line;
    std::string message;
};

[[noreturn]] void fail(int line, const std::string& msg) { throw ParseError{line, msg}; }

struct Tokens {
    std::vector<std::string> t;
    int line = 0;

    std::size_t size() const { return t.size(); }
    void need(std::size_t n) const {
        if (t.size() != n) fail(line, "'" + t[0] + "' espera " + std::to_string(n - 1) + " valor(es), recebeu " + std::to_string(t.size() - 1));
    }
    Real real(std::size_t i) const {
        const std::string& s = t.at(i);
        char* end = nullptr;
        errno = 0;
        const double d = std::strtod(s.c_str(), &end);
        if (s.empty() || end != s.c_str() + s.size() || !std::isfinite(d) || errno == ERANGE)
            fail(line, "numero invalido '" + s + "'");
        return static_cast<Real>(d);
    }
    int integer(std::size_t i, int lo, int hi) const {
        const std::string& s = t.at(i);
        char* end = nullptr;
        const long v = std::strtol(s.c_str(), &end, 10);
        if (s.empty() || end != s.c_str() + s.size() || v < lo || v > hi)
            fail(line, "inteiro invalido '" + s + "' (esperado de " + std::to_string(lo) + " a " + std::to_string(hi) + ")");
        return static_cast<int>(v);
    }
    bool flag(std::size_t i) const { return integer(i, 0, 1) != 0; }
    Vector3 vector(std::size_t i) const { return Vector3(real(i), real(i + 1), real(i + 2)); }
};

struct BodyDraft {
    int line = 0;
    BodyType type = BodyType::Dynamic;
    bool hasType = false;
    enum class Shape { Unset, Box, Sphere, None, Poly } shape = Shape::Unset;
    Vector3 half;
    Real radius = 0;
    std::vector<Vector3> verts;
    std::vector<std::vector<int>> faces;
    Real mass = 0;
    bool hasMass = false;
    Vector3 position, velocity, angularVelocity;
    Quaternion orientation = Quaternion::identity();
    Real friction = Real(0.5), restitution = 0, linearDamping = 0, angularDamping = Real(0.01), gravityScale = 1;
    bool allowSleep = true;
    unsigned char r = 200, g = 200, b = 200;
    bool focus = false;
};

struct JointDraft {
    int line = 0;
    enum class Kind { Hinge, BallSocket, Distance } kind = Kind::Hinge;
    int a = 0, b = 0;   // 1-based
    Vector3 anchorA, anchorB, axis;
    Real length = 0;
    bool collide = false;
};

Real checkRange(int line, const char* what, Real v, Real lo, Real hi) {
    if (!(v >= lo && v <= hi)) fail(line, std::string(what) + " fora da faixa [" + num(lo) + ", " + num(hi) + "]");
    return v;
}

std::vector<std::string> splitTokens(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream is(line);
    std::string w;
    while (is >> w) out.push_back(w);
    return out;
}

void parseBodyLine(const Tokens& k, BodyDraft& d) {
    const std::string& key = k.t[0];
    const int ln = k.line;
    if (key == "type") {
        k.need(2);
        if (k.t[1] == "static") d.type = BodyType::Static;
        else if (k.t[1] == "dynamic") d.type = BodyType::Dynamic;
        else if (k.t[1] == "kinematic") d.type = BodyType::Kinematic;
        else fail(ln, "type deve ser static, dynamic ou kinematic");
        d.hasType = true;
    } else if (key == "shape") {
        if (k.size() < 2) fail(ln, "shape sem tipo");
        if (k.t[1] == "box") {
            k.need(5);
            d.shape = BodyDraft::Shape::Box;
            d.half = k.vector(2);
            for (Real h : {d.half.x, d.half.y, d.half.z}) checkRange(ln, "meia-aresta", h, Real(0.001), Real(1000));
        } else if (k.t[1] == "sphere") {
            k.need(3);
            d.shape = BodyDraft::Shape::Sphere;
            d.radius = checkRange(ln, "raio", k.real(2), Real(0.001), Real(1000));
        } else if (k.t[1] == "none") {
            k.need(2);
            d.shape = BodyDraft::Shape::None;
        } else if (k.t[1] == "poly") {
            k.need(2);
            d.shape = BodyDraft::Shape::Poly;
        } else fail(ln, "shape deve ser box, sphere, none ou poly");
    } else if (key == "vertex") {
        k.need(4);
        if (d.shape != BodyDraft::Shape::Poly) fail(ln, "vertex so vale depois de 'shape poly'");
        if (static_cast<int>(d.verts.size()) >= scenefile::kMaxPolyVertices) fail(ln, "vertices demais no poliedro");
        const Vector3 v = k.vector(1);
        checkRange(ln, "vertice", std::max({std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)}), Real(0), Real(1000));
        d.verts.push_back(v);
    } else if (key == "face") {
        if (d.shape != BodyDraft::Shape::Poly) fail(ln, "face so vale depois de 'shape poly'");
        if (k.size() < 4 || static_cast<int>(k.size()) - 1 > scenefile::kMaxFaceVertices) fail(ln, "face deve ter de 3 a 32 vertices");
        if (static_cast<int>(d.faces.size()) >= scenefile::kMaxPolyFaces) fail(ln, "faces demais no poliedro");
        std::vector<int> f;
        for (std::size_t i = 1; i < k.size(); ++i) f.push_back(k.integer(i, 0, scenefile::kMaxPolyVertices - 1));
        d.faces.push_back(f);
    } else if (key == "mass") {
        k.need(2);
        d.mass = checkRange(ln, "massa", k.real(1), Real(1e-4), Real(1e6));
        d.hasMass = true;
    } else if (key == "position") {
        k.need(4);
        d.position = k.vector(1);
        for (Real c : {d.position.x, d.position.y, d.position.z}) checkRange(ln, "posicao", c, Real(-1e5), Real(1e5));
    } else if (key == "orientation") {
        k.need(5);
        const Quaternion q(k.real(1), k.real(2), k.real(3), k.real(4));
        const Real n = q.norm();
        if (!(n > Real(1e-6))) fail(ln, "quaternio de norma zero");
        d.orientation = q.normalized();
    } else if (key == "velocity") {
        k.need(4);
        d.velocity = k.vector(1);
        checkRange(ln, "|velocidade|", d.velocity.length(), Real(0), Real(1e4));
    } else if (key == "angular_velocity") {
        k.need(4);
        d.angularVelocity = k.vector(1);
        checkRange(ln, "|velocidade angular|", d.angularVelocity.length(), Real(0), Real(1e4));
    } else if (key == "material") {
        k.need(3);
        d.friction = checkRange(ln, "atrito", k.real(1), Real(0), Real(10));
        d.restitution = checkRange(ln, "restituicao", k.real(2), Real(0), Real(1));
    } else if (key == "damping") {
        k.need(3);
        d.linearDamping = checkRange(ln, "amortecimento linear", k.real(1), Real(0), Real(100));
        d.angularDamping = checkRange(ln, "amortecimento angular", k.real(2), Real(0), Real(100));
    } else if (key == "gravity_scale") {
        k.need(2);
        d.gravityScale = checkRange(ln, "escala de gravidade", k.real(1), Real(-100), Real(100));
    } else if (key == "allow_sleep") {
        k.need(2);
        d.allowSleep = k.flag(1);
    } else if (key == "color") {
        k.need(4);
        d.r = static_cast<unsigned char>(k.integer(1, 0, 255));
        d.g = static_cast<unsigned char>(k.integer(2, 0, 255));
        d.b = static_cast<unsigned char>(k.integer(3, 0, 255));
    } else if (key == "focus") {
        k.need(1);
        d.focus = true;
    } else {
        fail(ln, "chave desconhecida dentro de 'body': '" + key + "'");
    }
}

} // namespace

// ----------------------------------------------------------------------------------------------------
// serializeScene
// ----------------------------------------------------------------------------------------------------
std::string serializeScene(const Scene& scene) {
    std::ostringstream o;
    o << "physics_scene " << scenefile::kVersion << "\n";
    o << "# quaternios em 'orientation' sao w x y z\n";
    o << "name " << cleanName(scene.name) << "\n";
    if (scene.world) {
        const WorldParams w = captureWorld(*scene.world);
        o << "gravity " << vec(w.gravity) << "\n";
        o << "timestep " << num(w.fixedDt) << "\n";
        o << "iterations " << w.velocityIterations << " " << w.positionIterations << "\n";
        o << "slop " << num(w.linearSlop) << " " << num(w.velocitySlop) << "\n";
        o << "warm_start " << (w.warmStarting ? 1 : 0) << "\n";
        o << "sleep " << (w.sleepEnabled ? 1 : 0) << " " << num(w.sleepTime) << "\n";
    }
    o << "camera " << num(scene.cameraTarget.x) << " " << num(scene.cameraTarget.y) << " " << num(scene.cameraTarget.z) << " "
      << num(scene.cameraDistance) << "\n";
    if (!scene.world) return o.str();

    // índice (1-based) de cada corpo no arquivo
    std::vector<std::pair<BodyId, int>> index;
    int n = 0;
    for (const SceneBody& sb : scene.bodies)
        if (scene.world->isValidBody(sb.id)) index.emplace_back(sb.id, ++n);
    auto indexOf = [&](BodyId id) {
        for (const auto& p : index)
            if (p.first == id) return p.second;
        return 0;
    };

    for (const SceneBody& sb : scene.bodies) {
        if (!scene.world->isValidBody(sb.id)) continue;
        const RigidBody& b = scene.world->getBody(sb.id);
        o << "body\n";
        o << "  type " << (b.isStatic() ? "static" : b.isKinematic() ? "kinematic" : "dynamic") << "\n";
        const Shape* shape = b.getShape();
        if (!shape) {
            o << "  shape none\n";
        } else if (shape->getType() == ShapeType::Box) {
            o << "  shape box " << vec(static_cast<const BoxShape*>(shape)->getHalfExtents()) << "\n";
        } else if (shape->getType() == ShapeType::Sphere) {
            o << "  shape sphere " << num(static_cast<const SphereShape*>(shape)->getRadius()) << "\n";
        } else {
            const HalfEdgeStructure& topo = static_cast<const PolyhedronShape*>(shape)->getTopology();
            o << "  shape poly\n";
            for (const HEVertex& v : topo.vertices()) o << "  vertex " << vec(v.position) << "\n";
            for (int f = 0; f < topo.getFaceCount(); ++f) {
                o << "  face";
                topo.forEachFaceVertex(f, [&](int vi) { o << " " << vi; });
                o << "\n";
            }
        }
        if (b.isDynamic()) o << "  mass " << num(b.getMass()) << "\n";
        o << "  position " << vec(b.getTransform().position) << "\n";
        const Quaternion& q = b.getOrientation();
        o << "  orientation " << num(q.w) << " " << num(q.x) << " " << num(q.y) << " " << num(q.z) << "\n";
        o << "  velocity " << vec(b.getLinearVelocity()) << "\n";
        o << "  angular_velocity " << vec(b.getAngularVelocity()) << "\n";
        o << "  material " << num(b.getFriction()) << " " << num(b.getRestitution()) << "\n";
        o << "  damping " << num(b.getLinearDamping()) << " " << num(b.getAngularDamping()) << "\n";
        o << "  gravity_scale " << num(b.getGravityScale()) << "\n";
        o << "  allow_sleep " << (b.getAllowSleep() ? 1 : 0) << "\n";
        o << "  color " << static_cast<int>(sb.r) << " " << static_cast<int>(sb.g) << " " << static_cast<int>(sb.b) << "\n";
        if (sb.id == scene.focus) o << "  focus\n";
        o << "end\n";
    }

    scene.world->forEachJoint([&](const JointConstraint& j, BodyId a, BodyId b) {
        const int ia = indexOf(a), ib = indexOf(b);
        if (!ia || !ib) return;
        const int collide = j.getCollideConnected() ? 1 : 0;
        switch (j.getType()) {
            case ConstraintType::Hinge: {
                const auto& h = static_cast<const HingeJoint&>(j);
                o << "joint hinge " << ia << " " << ib << " " << vec(h.getAnchorA()) << " " << vec(h.getAxisA()) << " " << collide << "\n";
                break;
            }
            case ConstraintType::BallSocket:
                o << "joint ballsocket " << ia << " " << ib << " " << vec(j.getAnchorA()) << " " << collide << "\n";
                break;
            case ConstraintType::Distance: {
                const auto& d = static_cast<const DistanceJoint&>(j);
                o << "joint distance " << ia << " " << ib << " " << vec(d.getAnchorA()) << " " << vec(d.getAnchorB()) << " "
                  << num(d.getLength()) << " " << collide << "\n";
                break;
            }
            default: break;
        }
    });
    for (const SceneLink& l : scene.links) {
        const int ia = indexOf(l.a), ib = indexOf(l.b);
        if (ia && ib) o << "link " << ia << " " << ib << "\n";
    }
    return o.str();
}

// ----------------------------------------------------------------------------------------------------
// parseScene
// ----------------------------------------------------------------------------------------------------
bool parseScene(const std::string& text, Scene& out, std::string& error) {
    try {
        if (text.size() > scenefile::kMaxBytes) fail(0, "arquivo grande demais (limite 8 MB)");

        // ---- 1) texto -> rascunhos (nada é criado ainda) ----
        std::string name = "Cena carregada";
        WorldParams wp;
        wp.gravity = Vector3(0, Real(-9.81), 0);
        Vec3f camTarget{0.0f, 1.5f, 0.0f};
        float camDistance = 12.0f;
        std::vector<BodyDraft> bodies;
        std::vector<JointDraft> joints;
        std::vector<std::pair<int, std::pair<int, int>>> links;   // (linha, (a, b))
        bool sawHeader = false, inBody = false;
        BodyDraft cur;

        std::istringstream in(text);
        std::string raw;
        int lineNo = 0;
        while (std::getline(in, raw)) {
            ++lineNo;
            if (lineNo > 200000) fail(lineNo, "linhas demais");
            if (!raw.empty() && raw.back() == '\r') raw.pop_back();
            // O nome pode conter '#'; todo o resto aceita comentário.
            std::string line = raw;
            std::size_t first = line.find_first_not_of(" \t");
            if (first == std::string::npos) continue;
            const bool isName = line.compare(first, 5, "name ") == 0;
            if (!isName) {
                const std::size_t hash = line.find('#');
                if (hash != std::string::npos) line.erase(hash);
            }
            Tokens k;
            k.line = lineNo;
            if (isName) {
                std::string n = raw.substr(first + 5);
                const std::size_t a = n.find_first_not_of(" \t");
                n = a == std::string::npos ? std::string() : n.substr(a);
                while (!n.empty() && (n.back() == ' ' || n.back() == '\t')) n.pop_back();
                if (n.size() > scenefile::kMaxNameLength) n.resize(scenefile::kMaxNameLength);
                if (!n.empty()) name = n;
                continue;
            }
            k.t = splitTokens(line);
            if (k.t.empty()) continue;
            const std::string& key = k.t[0];

            if (!sawHeader) {
                if (key != "physics_scene" || k.size() != 2) fail(lineNo, "o arquivo deve comecar com 'physics_scene 1'");
                if (k.integer(1, 0, 1000000) != scenefile::kVersion) fail(lineNo, "versao do formato nao suportada (esperada " + std::to_string(scenefile::kVersion) + ")");
                sawHeader = true;
                continue;
            }
            if (inBody) {
                if (key == "end") {
                    k.need(1);
                    bodies.push_back(cur);
                    inBody = false;
                } else {
                    parseBodyLine(k, cur);
                }
                continue;
            }
            if (key == "body") {
                k.need(1);
                if (static_cast<int>(bodies.size()) >= scenefile::kMaxBodies) fail(lineNo, "corpos demais (limite " + std::to_string(scenefile::kMaxBodies) + ")");
                cur = BodyDraft();
                cur.line = lineNo;
                inBody = true;
            } else if (key == "gravity") {
                k.need(4);
                wp.gravity = k.vector(1);
            } else if (key == "timestep") {
                k.need(2);
                wp.fixedDt = checkRange(lineNo, "timestep", k.real(1), Real(1) / Real(1000), Real(1));
            } else if (key == "iterations") {
                k.need(3);
                wp.velocityIterations = k.integer(1, 1, worldlimits::kMaxVelocityIterations);
                wp.positionIterations = k.integer(2, 0, worldlimits::kMaxPositionIterations);
            } else if (key == "slop") {
                k.need(3);
                wp.linearSlop = checkRange(lineNo, "slop linear", k.real(1), Real(0), worldlimits::kMaxLinearSlop);
                wp.velocitySlop = checkRange(lineNo, "slop de velocidade", k.real(2), Real(0), worldlimits::kMaxVelocitySlop);
            } else if (key == "warm_start") {
                k.need(2);
                wp.warmStarting = k.flag(1);
            } else if (key == "sleep") {
                k.need(3);
                wp.sleepEnabled = k.flag(1);
                wp.sleepTime = checkRange(lineNo, "tempo de sono", k.real(2), worldlimits::kMinSleepTime, worldlimits::kMaxSleepTime);
            } else if (key == "camera") {
                k.need(5);
                camTarget = Vec3f{static_cast<float>(checkRange(lineNo, "camera", k.real(1), Real(-1e5), Real(1e5))),
                                  static_cast<float>(checkRange(lineNo, "camera", k.real(2), Real(-1e5), Real(1e5))),
                                  static_cast<float>(checkRange(lineNo, "camera", k.real(3), Real(-1e5), Real(1e5)))};
                camDistance = static_cast<float>(checkRange(lineNo, "distancia da camera", k.real(4), Real(0.5), Real(1000)));
            } else if (key == "joint") {
                if (static_cast<int>(joints.size()) >= scenefile::kMaxJoints) fail(lineNo, "juntas demais");
                if (k.size() < 4) fail(lineNo, "joint incompleta");
                JointDraft j;
                j.line = lineNo;
                if (k.t[1] == "hinge") {
                    k.need(11);
                    j.kind = JointDraft::Kind::Hinge;
                    j.a = k.integer(2, 1, scenefile::kMaxBodies);
                    j.b = k.integer(3, 1, scenefile::kMaxBodies);
                    j.anchorA = k.vector(4);
                    j.axis = k.vector(7);
                    j.collide = k.flag(10);
                } else if (k.t[1] == "ballsocket") {
                    k.need(8);
                    j.kind = JointDraft::Kind::BallSocket;
                    j.a = k.integer(2, 1, scenefile::kMaxBodies);
                    j.b = k.integer(3, 1, scenefile::kMaxBodies);
                    j.anchorA = k.vector(4);
                    j.collide = k.flag(7);
                } else if (k.t[1] == "distance") {
                    k.need(12);
                    j.kind = JointDraft::Kind::Distance;
                    j.a = k.integer(2, 1, scenefile::kMaxBodies);
                    j.b = k.integer(3, 1, scenefile::kMaxBodies);
                    j.anchorA = k.vector(4);
                    j.anchorB = k.vector(7);
                    j.length = checkRange(lineNo, "comprimento", k.real(10), Real(0), Real(1e5));
                    j.collide = k.flag(11);
                } else fail(lineNo, "joint deve ser hinge, ballsocket ou distance");
                if (j.a == j.b) fail(lineNo, "junta ligando o corpo a ele mesmo");
                joints.push_back(j);
            } else if (key == "link") {
                k.need(3);
                links.push_back({lineNo, {k.integer(1, 1, scenefile::kMaxBodies), k.integer(2, 1, scenefile::kMaxBodies)}});
            } else {
                fail(lineNo, "comando desconhecido '" + key + "'");
            }
        }
        if (!sawHeader) fail(0, "arquivo vazio (falta 'physics_scene 1')");
        if (inBody) fail(lineNo, "'body' sem 'end'");
        // V14: cena sem corpos é válida (cena nova vazia).
        const int nBodies = static_cast<int>(bodies.size());
        for (const JointDraft& j : joints)
            if (j.a > nBodies || j.b > nBodies) fail(j.line, "a junta usa um corpo que nao existe (corpos: 1 a " + std::to_string(nBodies) + ")");
        for (const auto& l : links)
            if (l.second.first > nBodies || l.second.second > nBodies) fail(l.first, "link para um corpo que nao existe");
        int focusCount = 0;
        for (const BodyDraft& b : bodies) focusCount += b.focus ? 1 : 0;
        if (focusCount > 1) fail(0, "mais de um corpo com 'focus'");

        // ---- 2) rascunhos -> mundo (erros do motor viram mensagens) ----
        Scene s;
        s.name = name;
        s.world = std::make_unique<PhysicsWorld>();
        s.cameraTarget = camTarget;
        s.cameraDistance = camDistance;
        std::vector<BodyId> ids;
        int bodyNo = 0;
        for (const BodyDraft& d : bodies) {
            ++bodyNo;
            const std::string where = "linha " + std::to_string(d.line) + " (corpo " + std::to_string(bodyNo) + "): ";
            if (!d.hasType) fail(d.line, "corpo sem 'type'");
            if (d.shape == BodyDraft::Shape::Unset) fail(d.line, "corpo sem 'shape'");
            if (d.type == BodyType::Dynamic && d.shape == BodyDraft::Shape::None) fail(d.line, "corpo dinamico precisa de forma");
            if (d.type == BodyType::Dynamic && !d.hasMass) fail(d.line, "corpo dinamico precisa de 'mass'");
            RigidBodyDesc rd;
            SceneBody sb;
            try {
                switch (d.shape) {
                    case BodyDraft::Shape::Box:
                        rd.shape = std::make_shared<BoxShape>(d.half);
                        sb.kind = ShapeKind::Box;
                        sb.halfExtents = Vec3f{static_cast<float>(d.half.x), static_cast<float>(d.half.y), static_cast<float>(d.half.z)};
                        break;
                    case BodyDraft::Shape::Sphere:
                        rd.shape = std::make_shared<SphereShape>(d.radius);
                        sb.kind = ShapeKind::Sphere;
                        sb.radius = static_cast<float>(d.radius);
                        break;
                    case BodyDraft::Shape::None:
                        sb.kind = ShapeKind::Sphere;   // pivô: só uma bolinha cinza
                        sb.radius = 0.12f;
                        break;
                    case BodyDraft::Shape::Poly:
                        if (d.verts.size() < 4 || d.faces.size() < 4) fail(d.line, "poliedro precisa de ao menos 4 vertices e 4 faces");
                        for (const auto& f : d.faces)
                            for (int vi : f)
                                if (vi >= static_cast<int>(d.verts.size())) fail(d.line, "face usa um vertice que nao existe");
                        sb.poly = makePoly(d.verts, d.faces);
                        sb.kind = ShapeKind::Polyhedron;
                        rd.shape = sb.poly;
                        break;
                    case BodyDraft::Shape::Unset: break;
                }
                rd.type = d.type;
                if (d.type == BodyType::Dynamic) rd.mass = d.mass;
                rd.transform = Transform(d.position, d.orientation);
                rd.linearVelocity = d.velocity;
                rd.angularVelocity = d.angularVelocity;
                rd.friction = d.friction;
                rd.restitution = d.restitution;
                rd.linearDamping = d.linearDamping;
                rd.angularDamping = d.angularDamping;
                rd.gravityScale = d.gravityScale;
                rd.allowSleep = d.allowSleep;
                sb.id = s.world->createBody(rd);
            } catch (const ParseError&) {
                throw;
            } catch (const std::exception& e) {
                fail(d.line, where + e.what());
            }
            sb.isStatic = d.type == BodyType::Static;
            sb.r = d.r;
            sb.g = d.g;
            sb.b = d.b;
            s.bodies.push_back(sb);
            ids.push_back(sb.id);
            if (d.focus) s.focus = sb.id;
        }
        for (const JointDraft& j : joints) {
            try {
                const BodyId a = ids[static_cast<std::size_t>(j.a - 1)], b = ids[static_cast<std::size_t>(j.b - 1)];
                switch (j.kind) {
                    case JointDraft::Kind::Hinge: s.world->createHingeJoint(a, b, j.anchorA, j.axis, j.collide); break;
                    case JointDraft::Kind::BallSocket: s.world->createBallSocketJoint(a, b, j.anchorA, j.collide); break;
                    case JointDraft::Kind::Distance: s.world->createDistanceJoint(a, b, j.anchorA, j.anchorB, j.length, j.collide); break;
                }
            } catch (const std::exception& e) {
                fail(j.line, std::string("junta invalida: ") + e.what());
            }
        }
        for (const auto& l : links) s.links.push_back({ids[static_cast<std::size_t>(l.second.first - 1)], ids[static_cast<std::size_t>(l.second.second - 1)]});
        applyWorld(*s.world, wp);
        out = std::move(s);
        error.clear();
        return true;
    } catch (const ParseError& e) {
        error = e.line > 0 ? "linha " + std::to_string(e.line) + ": " + e.message : e.message;
        return false;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

// ----------------------------------------------------------------------------------------------------
// Arquivos
// ----------------------------------------------------------------------------------------------------
bool saveSceneFile(const Scene& scene, const std::string& path, std::string& error) {
    namespace fs = std::filesystem;
    try {
        const fs::path p(path);
        if (p.has_parent_path()) fs::create_directories(p.parent_path());
        std::ofstream f(p, std::ios::binary | std::ios::trunc);
        if (!f) {
            error = "nao foi possivel abrir '" + path + "' para escrita";
            return false;
        }
        const std::string text = serializeScene(scene);
        f.write(text.data(), static_cast<std::streamsize>(text.size()));
        f.flush();
        if (!f) {
            error = "falha ao escrever '" + path + "'";
            return false;
        }
        error.clear();
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

bool loadSceneFile(const std::string& path, Scene& out, std::string& error) {
    namespace fs = std::filesystem;
    try {
        std::error_code ec;
        const auto size = fs::file_size(fs::path(path), ec);
        if (ec) {
            error = "nao foi possivel abrir '" + path + "'";
            return false;
        }
        if (size > scenefile::kMaxBytes) {
            error = "arquivo grande demais (limite 8 MB)";
            return false;
        }
        std::ifstream f(fs::path(path), std::ios::binary);
        if (!f) {
            error = "nao foi possivel abrir '" + path + "'";
            return false;
        }
        std::ostringstream ss;
        ss << f.rdbuf();
        return parseScene(ss.str(), out, error);
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

std::vector<std::string> listSceneFiles(const std::string& dir) {
    namespace fs = std::filesystem;
    std::vector<std::string> names;
    std::error_code ec;
    for (fs::directory_iterator it(fs::path(dir), ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        const fs::path& p = it->path();
        if (p.extension().string() == scenefile::kExtension) names.push_back(p.stem().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}

std::string sanitizeSceneFileName(const std::string& typed) {
    std::string out;
    for (unsigned char c : typed) {
        if (std::isalnum(c) && c < 128) out.push_back(static_cast<char>(c));
        else if (c == '-' || c == '_') out.push_back(static_cast<char>(c));
        else if (c == ' ') out.push_back('_');
        if (out.size() >= 48) break;
    }
    const bool hasAlnum = std::any_of(out.begin(), out.end(), [](char c) { return std::isalnum(static_cast<unsigned char>(c)) != 0; });
    return hasAlnum ? out : std::string("cena");
}

} // namespace viewer
