// =============================================================================
// ModelImporter.cpp  (V17)
// =============================================================================
#include "viewer/core/ModelImporter.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <set>

#include "viewer/core/ObjModel.h"

namespace viewer {

namespace fs = std::filesystem;

namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool hasExt(const fs::path& p, const char* ext) { return lower(p.extension().string()) == ext; }

bool imageExt(const fs::path& p) {
    const std::string e = lower(p.extension().string());
    return e == ".png" || e == ".jpg" || e == ".jpeg" || e == ".bmp" || e == ".tga";
}

/// Relativo seguro (como o leitor de .obj): sem "..", sem absoluto, sem unidade.
bool safeRel(std::string& name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    if (name.empty() || name[0] == '/' || name.find(':') != std::string::npos) return false;
    std::size_t i = 0;
    while (i <= name.size()) {
        const std::size_t j = name.find('/', i);
        const std::string part = name.substr(i, j == std::string::npos ? std::string::npos : j - i);
        if (part == "..") return false;
        if (j == std::string::npos) break;
        i = j + 1;
    }
    return true;
}

bool regularSmall(const fs::path& p, std::string& why) {
    std::error_code ec;
    if (!fs::is_regular_file(p, ec)) { why = "nao encontrado"; return false; }
    const auto size = fs::file_size(p, ec);
    if (ec) { why = "nao foi possivel ler"; return false; }
    if (size > importlimits::kMaxFileBytes) { why = "maior que 32 MB"; return false; }
    return true;
}

} // namespace

std::string sanitizeModelName(const std::string& typed) {
    std::string out;
    for (unsigned char c : typed) {
        if (std::isalnum(c) && c < 128) out += static_cast<char>(c);
        else if (c == '-' || c == '_') out += static_cast<char>(c);
        else if (c == ' ') out += '_';
        if (out.size() >= 40) break;
    }
    bool useful = false;   // só "_" / "-" não serve de nome
    for (char c : out) useful = useful || std::isalnum(static_cast<unsigned char>(c));
    return useful ? out : "modelo";
}

bool importModel(const std::string& srcObjPath, const std::string& modelDir, ImportResult& out, std::string& error) {
    out = ImportResult{};
    const fs::path src(srcObjPath);
    std::string why;
    if (!hasExt(src, ".obj")) {
        error = "'" + src.filename().string() + "' nao e um .obj (por enquanto so o formato Wavefront .obj)";
        return false;
    }
    if (!regularSmall(src, why)) {
        error = "'" + src.filename().string() + "': " + why;
        return false;
    }
    // 1) valida lendo (sintaxe, limites, triângulos); descobre os .mtl e as texturas usadas
    ObjModel model;
    std::string warn;
    if (!loadObjFile(src.string(), model, error, &warn)) return false;
    if (model.triangles.empty()) {
        error = "o modelo nao tem faces";
        return false;
    }
    const fs::path srcDir = src.parent_path();

    // 2) lista de cópias (origem -> caminho relativo no destino); nada é escrito ainda
    struct Item { fs::path from; std::string rel; };
    std::vector<Item> items;
    std::set<std::string> seen;
    const std::string base = sanitizeModelName(src.stem().string());
    items.push_back({src, base + ".obj"});
    seen.insert(lower(base + ".obj"));
    if (!warn.empty()) out.notes.push_back(warn);
    for (std::string lib : model.mtlLibs) {
        if (!safeRel(lib)) { out.notes.push_back("mtllib ignorado (caminho inseguro)"); continue; }
        if (lib.find('/') != std::string::npos) { out.notes.push_back("mtllib em subpasta ignorado: " + lib); continue; }
        if (!hasExt(fs::path(lib), ".mtl")) { out.notes.push_back("mtllib ignorado (nao e .mtl): " + lib); continue; }
        std::string w;
        if (!regularSmall(srcDir / lib, w)) { out.notes.push_back("sem o .mtl '" + lib + "' (" + w + "): o modelo fica sem materiais"); continue; }
        if (seen.insert(lower(lib)).second) items.push_back({srcDir / lib, lib});
    }
    for (const ObjMaterial& mat : model.materials) {
        if (mat.diffuseMap.empty()) continue;
        std::string rel = mat.diffuseMap;
        if (!safeRel(rel)) { out.notes.push_back("textura ignorada (caminho inseguro)"); continue; }
        if (!imageExt(fs::path(rel))) { out.notes.push_back("textura em formato nao suportado ignorada: " + rel + " (use png, jpg, bmp ou tga)"); continue; }
        std::string w;
        if (!regularSmall(srcDir / rel, w)) { out.notes.push_back("textura '" + rel + "' " + w + ": o modelo usa a cor do material"); continue; }
        if (seen.insert(lower(rel)).second) items.push_back({srcDir / rel, rel});
    }
    if (static_cast<int>(items.size()) > importlimits::kMaxFiles) {
        error = "arquivos demais para um modelo (limite " + std::to_string(importlimits::kMaxFiles) + ")";
        return false;
    }

    // 3) pasta de destino nova (nunca reaproveita uma existente)
    std::error_code ec;
    fs::create_directories(modelDir, ec);
    if (ec) {
        error = "nao foi possivel criar a pasta de modelos '" + modelDir + "'";
        return false;
    }
    std::string name = base;
    fs::path dstDir = fs::path(modelDir) / name;
    for (int n = 2; fs::exists(dstDir, ec) && n < 1000; ++n) {
        name = base + "_" + std::to_string(n);
        dstDir = fs::path(modelDir) / name;
    }
    if (fs::exists(dstDir, ec)) {
        error = "nao foi possivel achar um nome livre em models/";
        return false;
    }
    if (!fs::create_directories(dstDir, ec) || ec) {
        error = "nao foi possivel criar '" + dstDir.string() + "'";
        return false;
    }
    // 4) copia; se algo falhar, remove a pasta nova inteira (sem lixo)
    for (const Item& it : items) {
        const fs::path dst = dstDir / (it.rel == base + ".obj" ? name + ".obj" : it.rel);
        fs::create_directories(dst.parent_path(), ec);
        fs::copy_file(it.from, dst, fs::copy_options::none, ec);
        if (ec) {
            error = "falha ao copiar '" + it.from.filename().string() + "'";
            fs::remove_all(dstDir, ec);
            return false;
        }
        ++out.copied;
    }
    out.file = name + "/" + name + ".obj";
    return true;
}

bool importPath(const std::string& path, const std::string& modelDir, std::vector<ImportResult>& imported,
                std::vector<std::string>& messages) {
    imported.clear();
    std::error_code ec;
    const fs::path p(path);
    auto one = [&](const fs::path& obj) {
        ImportResult r;
        std::string err;
        if (importModel(obj.string(), modelDir, r, err)) {
            for (const std::string& n : r.notes) messages.push_back(r.file + ": " + n);
            imported.push_back(std::move(r));
        } else {
            messages.push_back("ERRO: " + err);
        }
    };
    if (fs::is_directory(p, ec)) {
        std::vector<fs::path> objs;
        for (fs::directory_iterator it(p, ec), end; !ec && it != end; it.increment(ec)) {
            std::error_code e2;
            if (it->is_regular_file(e2) && hasExt(it->path(), ".obj")) objs.push_back(it->path());
            else if (it->is_directory(e2))
                for (fs::directory_iterator sub(it->path(), e2), send; !e2 && sub != send; sub.increment(e2)) {
                    std::error_code e3;
                    if (sub->is_regular_file(e3) && hasExt(sub->path(), ".obj")) objs.push_back(sub->path());
                }
            if (objs.size() > 200) break;
        }
        std::sort(objs.begin(), objs.end());
        if (objs.empty()) messages.push_back("ERRO: a pasta '" + p.filename().string() + "' nao tem nenhum .obj");
        if (static_cast<int>(objs.size()) > importlimits::kMaxFolderObjs) {
            messages.push_back("a pasta tem " + std::to_string(objs.size()) + " arquivos .obj: importei so os " + std::to_string(importlimits::kMaxFolderObjs) + " primeiros");
            objs.resize(static_cast<std::size_t>(importlimits::kMaxFolderObjs));
        }
        for (const fs::path& o : objs) one(o);
    } else if (fs::exists(p, ec)) {
        if (hasExt(p, ".mtl") || imageExt(p))
            messages.push_back("'" + p.filename().string() + "' e material/textura: arraste o .obj (o .mtl e as texturas vao junto)");
        else
            one(p);
    } else {
        messages.push_back("ERRO: nao achei '" + path + "'");
    }
    return !imported.empty();
}

} // namespace viewer
