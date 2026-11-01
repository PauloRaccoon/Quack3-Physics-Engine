// =============================================================================
// TrajectoryRecorder.cpp  (V16)
// =============================================================================
#include "viewer/core/TrajectoryRecorder.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "viewer/core/RenderPose.h"

namespace viewer {

using namespace physics;

namespace {
constexpr double kRad2Deg = 57.29577951308232;

/// Dígitos para ida e volta exata do tipo Real (float 9, double 17).
constexpr int kDigits = sizeof(Real) == sizeof(float) ? 9 : 17;

void put(std::string& out, double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.*g", kDigits, std::isfinite(v) ? v : 0.0);   // nunca "nan"/"inf" no CSV
    out += buf;
}
} // namespace

std::string TrajectoryRecorder::header() {
    return "t,passo,corpo,x,y,z,qw,qx,qy,qz,norma_q,eixo_x,eixo_y,eixo_z,angulo_graus,yaw_graus,pitch_graus,roll_graus,vx,vy,vz,wx,wy,wz";
}

void TrajectoryRecorder::appendRow(const Scene& scene, BodyId id) {
    const RigidBody& b = scene.world->getBody(id);
    TrajectoryRow r;
    r.t = time_;
    r.step = scene.world->getStats().stepCount;
    r.body = id;
    const Vector3& pos = b.getTransform().position;
    r.p[0] = static_cast<double>(pos.x); r.p[1] = static_cast<double>(pos.y); r.p[2] = static_cast<double>(pos.z);
    const Quaternion& q = b.getOrientation();
    r.q[0] = static_cast<double>(q.w); r.q[1] = static_cast<double>(q.x); r.q[2] = static_cast<double>(q.y); r.q[3] = static_cast<double>(q.z);
    r.qNorm = static_cast<double>(q.norm());
    const AxisAngleF aa = toAxisAngle(q);
    r.axis[0] = static_cast<double>(aa.axis.x); r.axis[1] = static_cast<double>(aa.axis.y); r.axis[2] = static_cast<double>(aa.axis.z);
    r.angleDeg = static_cast<double>(aa.angle) * kRad2Deg;
    const EulerAngles eu = toEuler(q);
    r.eulerDeg[0] = static_cast<double>(eu.yaw) * kRad2Deg;
    r.eulerDeg[1] = static_cast<double>(eu.pitch) * kRad2Deg;
    r.eulerDeg[2] = static_cast<double>(eu.roll) * kRad2Deg;
    const Vector3& v = b.getLinearVelocity();
    const Vector3& w = b.getAngularVelocity();
    r.v[0] = static_cast<double>(v.x); r.v[1] = static_cast<double>(v.y); r.v[2] = static_cast<double>(v.z);
    r.w[0] = static_cast<double>(w.x); r.w[1] = static_cast<double>(w.y); r.w[2] = static_cast<double>(w.z);
    rows_.push_back(r);
}

bool TrajectoryRecorder::start(const Scene& scene, RecordTarget target, BodyId selected, std::string* error) {
    auto fail = [&](const char* msg) {
        if (error) *error = msg;
        return false;
    };
    if (!scene.world) return fail("sem cena");
    clear();
    if (target == RecordTarget::Selected) {
        if (!scene.world->isValidBody(selected) || !scene.world->getBody(selected).isDynamic())
            return fail("selecione um corpo dinamico (clique nele) para gravar");
        body_ = selected;
    } else {
        bool any = false;
        for (const SceneBody& sb : scene.bodies)
            if (scene.world->isValidBody(sb.id) && scene.world->getBody(sb.id).isDynamic()) { any = true; break; }
        if (!any) return fail("a cena nao tem corpos dinamicos");
        body_ = kInvalidBodyId;
    }
    target_ = target;
    recording_ = true;
    stopReason_.clear();
    time_ = 0;
    lastStep_ = scene.world->getStats().stepCount;
    // linha do instante inicial
    if (target_ == RecordTarget::Selected) {
        appendRow(scene, body_);
    } else {
        std::size_t n = 0;
        for (const SceneBody& sb : scene.bodies)
            if (n < kMaxBodiesPerFrame && scene.world->isValidBody(sb.id) && scene.world->getBody(sb.id).isDynamic()) { appendRow(scene, sb.id); ++n; }
    }
    return true;
}

std::size_t TrajectoryRecorder::capture(const Scene& scene) {
    if (!recording_) return 0;
    if (!scene.world) { stop("cena trocada"); return 0; }
    const std::uint64_t step = scene.world->getStats().stepCount;
    if (step < lastStep_) { stop("a cena foi reiniciada"); return 0; }
    if (step == lastStep_) return 0;
    if (target_ == RecordTarget::Selected && !scene.world->isValidBody(body_)) { stop("o corpo gravado foi apagado"); return 0; }
    time_ += static_cast<double>(step - lastStep_) * static_cast<double>(scene.world->getFixedTimeStep());
    lastStep_ = step;
    const std::size_t before = rows_.size();
    if (target_ == RecordTarget::Selected) {
        if (rows_.size() >= kMaxRows) { stop("limite de linhas atingido"); return 0; }
        appendRow(scene, body_);
    } else {
        std::size_t n = 0;
        for (const SceneBody& sb : scene.bodies) {
            if (n >= kMaxBodiesPerFrame) break;
            if (!scene.world->isValidBody(sb.id) || !scene.world->getBody(sb.id).isDynamic()) continue;
            if (rows_.size() >= kMaxRows) { stop("limite de linhas atingido"); break; }
            appendRow(scene, sb.id);
            ++n;
        }
    }
    return rows_.size() - before;
}

std::string TrajectoryRecorder::toCsv() const {
    std::string out = header();
    out += '\n';
    out.reserve(out.size() + rows_.size() * 220);
    for (const TrajectoryRow& r : rows_) {
        put(out, r.t); out += ',';
        out += std::to_string(r.step); out += ',';
        out += std::to_string(r.body); out += ',';
        for (double v : r.p) { put(out, v); out += ','; }
        for (double v : r.q) { put(out, v); out += ','; }
        put(out, r.qNorm); out += ',';
        for (double v : r.axis) { put(out, v); out += ','; }
        put(out, r.angleDeg); out += ',';
        for (double v : r.eulerDeg) { put(out, v); out += ','; }
        for (double v : r.v) { put(out, v); out += ','; }
        for (int i = 0; i < 3; ++i) { put(out, r.w[i]); if (i < 2) out += ','; }
        out += '\n';
    }
    return out;
}

bool TrajectoryRecorder::save(const std::string& path, std::string& error) const {
    namespace fs = std::filesystem;
    try {
        std::error_code ec;
        const fs::path p(path);
        if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);
        std::ofstream f(p, std::ios::binary | std::ios::trunc);
        if (!f) {
            error = "nao foi possivel criar '" + path + "'";
            return false;
        }
        const std::string csv = toCsv();
        f.write(csv.data(), static_cast<std::streamsize>(csv.size()));
        f.flush();
        if (!f) {
            error = "falha ao gravar '" + path + "'";
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

} // namespace viewer
