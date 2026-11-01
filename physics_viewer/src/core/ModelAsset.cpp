// =============================================================================
// ModelAsset.cpp  (V15b)
// =============================================================================
#include "viewer/core/ModelAsset.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>

namespace viewer {

namespace fs = std::filesystem;

std::string modelBaseDir(const std::string& modelDir, const std::string& file) {
    return (fs::path(modelDir) / fs::path(file).parent_path()).string();
}

std::vector<std::string> listModelFiles(const std::string& dir) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return out;
    auto isObj = [](const fs::path& p) {
        std::string e = p.extension().string();
        std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return e == ".obj";
    };
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code e2;
        if (it->is_regular_file(e2) && isObj(it->path())) {
            out.push_back(it->path().filename().string());
        } else if (it->is_directory(e2)) {
            for (fs::directory_iterator sub(it->path(), e2), send; !e2 && sub != send; sub.increment(e2)) {
                std::error_code e3;
                if (sub->is_regular_file(e3) && isObj(sub->path())) out.push_back(it->path().filename().string() + "/" + sub->path().filename().string());
            }
        }
        if (out.size() > 500) break;
    }
    std::sort(out.begin(), out.end());
    return out;
}

bool makeModelAsset(const std::shared_ptr<const ObjModel>& obj, const std::string& file, const std::string& baseDir,
                    float scale, ModelAsset& out, std::string& error, ColliderKind collider) {
    out = ModelAsset{};
    if (!obj) {
        error = "modelo vazio";
        return false;
    }
    ImportedShape is;
    if (!makeImportedShape(*obj, scale, collider, is, error)) return false;
    out.obj = obj;
    out.file = file;
    out.baseDir = baseDir;
    out.scale = scale;
    out.offset = is.offset;
    out.shape = is.shape;
    out.hullVertices = is.hullVertices;
    out.hullFaces = is.hullFaces;
    out.modelVertices = is.modelVertices;
    out.concave = is.concave;
    out.collider = is.collider;
    out.fill = is.fill;
    return true;
}

bool loadModelAsset(const std::string& modelDir, const std::string& file, float targetSize, ModelAsset& out,
                    std::string& error, std::string* warning, ColliderKind collider) {
    out = ModelAsset{};
    std::string rel = file;
    std::replace(rel.begin(), rel.end(), '\\', '/');
    if (rel.empty() || rel[0] == '/' || rel.find(':') != std::string::npos || rel.find("..") != std::string::npos) {
        error = "nome de modelo invalido: " + file;
        return false;
    }
    auto obj = std::make_shared<ObjModel>();
    if (!loadObjFile((fs::path(modelDir) / rel).string(), *obj, error, warning)) return false;
    const float scale = targetSize > 0.0f ? autoScale(*obj, targetSize) : 1.0f;
    return makeModelAsset(obj, rel, modelBaseDir(modelDir, rel), scale, out, error, collider);
}

ModelAsset attachModel(const std::shared_ptr<const ObjModel>& obj, const std::string& file, const std::string& baseDir,
                       float scale, const physics::Vector3& offset, const std::shared_ptr<const physics::Shape>& shape) {
    ModelAsset a;
    a.obj = obj;
    a.file = file;
    a.baseDir = baseDir;
    a.scale = scale;
    a.offset = offset;
    a.shape = shape;
    if (shape && shape->getType() == physics::ShapeType::Polyhedron) {
        const physics::HalfEdgeStructure& topo = static_cast<const physics::PolyhedronShape&>(*shape).getTopology();
        a.hullVertices = topo.getVertexCount();
        a.hullFaces = topo.getFaceCount();
        a.collider = ColliderKind::Hull;
    } else if (shape && shape->getType() == physics::ShapeType::Box) {
        a.hullVertices = 8;
        a.hullFaces = 6;
        a.collider = ColliderKind::Box;
    } else if (shape && shape->getType() == physics::ShapeType::Sphere) {
        a.collider = ColliderKind::Sphere;
    }
    return a;
}

std::vector<MeshGroup> buildMeshGroups(const ModelAsset& asset) {
    std::vector<MeshGroup> groups;
    if (!asset.obj) return groups;
    const ObjModel& m = *asset.obj;
    std::map<int, std::size_t> byMaterial;
    const float ox = static_cast<float>(asset.offset.x), oy = static_cast<float>(asset.offset.y), oz = static_cast<float>(asset.offset.z);
    for (const ObjTriangle& t : m.triangles) {
        auto it = byMaterial.find(t.material);
        if (it == byMaterial.end()) {
            it = byMaterial.emplace(t.material, groups.size()).first;
            groups.emplace_back();
            groups.back().material = t.material;
        }
        MeshGroup& g = groups[it->second];
        Vec3f p[3];
        for (int k = 0; k < 3; ++k) {
            const Vec3f& v = m.positions[static_cast<std::size_t>(t.p[k])];
            p[k] = {v.x * asset.scale - ox, v.y * asset.scale - oy, v.z * asset.scale - oz};
            g.positions.insert(g.positions.end(), {p[k].x, p[k].y, p[k].z});
            if (t.t[k] >= 0) {
                const ObjUV& uv = m.uvs[static_cast<std::size_t>(t.t[k])];
                g.texcoords.insert(g.texcoords.end(), {uv.u, 1.0f - uv.v});   // a raylib tem a origem da textura no topo
            } else {
                g.texcoords.insert(g.texcoords.end(), {0.0f, 0.0f});
            }
        }
        const Vec3f e1{p[1].x - p[0].x, p[1].y - p[0].y, p[1].z - p[0].z}, e2{p[2].x - p[0].x, p[2].y - p[0].y, p[2].z - p[0].z};
        Vec3f n{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x};
        const float l = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (l > 1e-20f) n = {n.x / l, n.y / l, n.z / l};
        else n = {0.0f, 1.0f, 0.0f};
        g.normals.insert(g.normals.end(), {n.x, n.y, n.z});
    }
    return groups;
}

} // namespace viewer
