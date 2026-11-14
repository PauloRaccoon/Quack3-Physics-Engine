// =============================================================================
// ModelImporter.cpp  (V17)
// =============================================================================
#include "viewer/core/ModelImporter.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

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


/// V25: último componente de um caminho (aceita / e \\: o .obj pode vir de outro sistema).
std::string baseName(std::string s) {
    std::replace(s.begin(), s.end(), '\\', '/');
    const std::size_t k = s.find_last_of('/');
    return k == std::string::npos ? s : s.substr(k + 1);
}

std::string trimWs(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

/// V25: acha `name` (sem pasta) em `dir`: exato, ou ignorando maiúsculas/minúsculas (Windows e Linux divergem nisso).
bool findInDir(const fs::path& dir, const std::string& name, fs::path& found) {
    std::error_code ec;
    if (name.empty()) return false;
    if (fs::is_regular_file(dir / name, ec)) { found = dir / name; return true; }
    const std::string want = lower(name);
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code e2;
        if (it->is_regular_file(e2) && lower(it->path().filename().string()) == want) { found = it->path(); return true; }
    }
    return false;
}

bool readTextFile(const fs::path& p, std::string& out) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
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

    // 2) lista de cópias (origem -> caminho relativo no destino); nada é escrito ainda.
    //    V25: o .mtl e as texturas são PROCURADOS com tolerância (outro nome, caminho do outro computador, maiúsculas, falta de
    //    'mtllib'); quando o nome citado não bate com o arquivo achado, o .obj/.mtl copiados são reescritos para apontar certo.
    struct Item {
        fs::path from;
        std::string rel;
        bool hasText = false;   // true: grava `text` em vez de copiar o arquivo
        std::string text;
    };
    std::vector<Item> items;
    std::set<std::string> seen;
    const std::string base = sanitizeModelName(src.stem().string());
    items.push_back({src, base + ".obj", false, {}});
    seen.insert(lower(base + ".obj"));
    if (!warn.empty()) out.notes.push_back(warn);

    struct MtlPick { fs::path from; std::string destName; bool renamed = false; };
    std::vector<MtlPick> mtls;
    std::set<std::string> usedMtl;
    auto addMtl = [&](const fs::path& f, const std::string& cited, const char* how) {
        const std::string dest = f.filename().string();
        if (!usedMtl.insert(lower(dest)).second) return;
        std::string w;
        if (!regularSmall(f, w)) { out.notes.push_back("sem o .mtl '" + dest + "' (" + w + "): o modelo fica sem materiais"); return; }
        MtlPick m;
        m.from = f;
        m.destName = dest;
        m.renamed = cited != dest;
        mtls.push_back(m);
        if (how && how[0]) out.notes.push_back(std::string(".mtl '") + dest + "' " + how + (cited.empty() ? "" : " (o .obj cita '" + cited + "')"));
    };
    auto fallbackMtl = [&](const std::string& cited) {
        fs::path f;
        if (findInDir(srcDir, src.stem().string() + ".mtl", f) && !usedMtl.count(lower(f.filename().string()))) { addMtl(f, cited, "usado (mesmo nome do .obj)"); return true; }
        std::vector<fs::path> all;
        std::error_code ec;
        for (fs::directory_iterator it(srcDir, ec), end; !ec && it != end; it.increment(ec)) {
            std::error_code e2;
            if (it->is_regular_file(e2) && hasExt(it->path(), ".mtl") && !usedMtl.count(lower(it->path().filename().string()))) all.push_back(it->path());
        }
        if (all.size() == 1) { addMtl(all[0], cited, "usado (e o unico .mtl da pasta)"); return true; }
        return false;
    };
    bool needObjRewrite = false;
    for (const std::string& rawLib : model.mtlLibs) {
        const std::string cited = trimWs(rawLib);
        fs::path f;
        // 1) o nome inteiro (com espaços), 2) só o último componente (caminho de outro computador), 3) palavras soltas
        std::vector<std::string> tries = {baseName(cited)};
        {
            std::istringstream ws(cited);
            std::string word;
            while (ws >> word) tries.push_back(baseName(word));
        }
        bool ok = false;
        for (const std::string& t : tries)
            if (hasExt(fs::path(t), ".mtl") && findInDir(srcDir, t, f)) { addMtl(f, cited, f.filename().string() == cited ? "" : "achado"); ok = true; break; }
        if (!ok) {
            out.notes.push_back("o .obj cita o material '" + cited + "' mas esse arquivo nao esta ao lado do .obj");
            if (!fallbackMtl(cited)) out.notes.push_back("nenhum .mtl encontrado: o modelo fica sem materiais");
        }
    }
    if (model.mtlLibs.empty()) {
        // o .obj nem cita material: se existe um .mtl com o nome do .obj (ou o único da pasta), usa e escreve o 'mtllib'
        if (fallbackMtl("")) out.notes.push_back("o .obj nao tinha 'mtllib': acrescentei");
    }
    for (const MtlPick& m : mtls) needObjRewrite = needObjRewrite || m.renamed;
    needObjRewrite = needObjRewrite || (model.mtlLibs.empty() && !mtls.empty());
    if (model.mtlLibs.size() != mtls.size() && !mtls.empty()) needObjRewrite = true;

    // texturas: de TODOS os materiais dos .mtl escolhidos (não só os usados), com a mesma tolerância
    std::vector<std::vector<std::pair<std::string, std::string>>> mtlTexRewrites(mtls.size());
    int textureCount = 0;
    for (std::size_t mi = 0; mi < mtls.size(); ++mi) {
        std::string mtlText, e2;
        std::vector<ObjMaterial> defs;
        if (!readTextFile(mtls[mi].from, mtlText) || !parseMtl(mtlText, defs, e2)) {
            out.notes.push_back("nao consegui ler o .mtl '" + mtls[mi].destName + "'" + (e2.empty() ? "" : ": " + e2));
            continue;
        }
        const fs::path mtlDir = mtls[mi].from.parent_path();
        for (const ObjMaterial& mat : defs) {
            if (mat.diffuseMap.empty()) continue;
            const std::string typed = mat.diffuseMap;
            std::string rel = typed;
            fs::path found;
            std::string destRel;
            bool ok = false;
            if (safeRel(rel) && imageExt(fs::path(rel))) {
                std::error_code ec;
                if (fs::is_regular_file(mtlDir / rel, ec)) { found = mtlDir / rel; destRel = rel; ok = true; }
            }
            if (!ok) {
                const std::string bn = baseName(typed);
                if (!imageExt(fs::path(bn))) { out.notes.push_back("textura em formato nao suportado ignorada: " + typed + " (use png, jpg, bmp ou tga)"); continue; }
                if (findInDir(mtlDir, bn, found) || findInDir(srcDir, bn, found) || findInDir(srcDir / "textures", bn, found) || findInDir(srcDir / "Textures", bn, found)) {
                    destRel = found.filename().string();
                    ok = true;
                    if (destRel != typed) mtlTexRewrites[mi].push_back({typed, destRel});
                }
            }
            if (!ok) { out.notes.push_back("textura '" + typed + "' nao encontrada: o modelo usa a cor do material"); continue; }
            std::string w;
            if (!regularSmall(found, w)) { out.notes.push_back("textura '" + typed + "' " + w + ": o modelo usa a cor do material"); continue; }
            if (seen.insert(lower(destRel)).second) { items.push_back({found, destRel, false, {}}); ++textureCount; }
        }
    }

    // monta os .mtl (copiados ou reescritos) e o .obj (reescrito se o 'mtllib' mudou)
    for (std::size_t mi = 0; mi < mtls.size(); ++mi) {
        if (!seen.insert(lower(mtls[mi].destName)).second) continue;
        Item it{mtls[mi].from, mtls[mi].destName, false, {}};
        if (!mtlTexRewrites[mi].empty()) {
            std::string text;
            if (readTextFile(mtls[mi].from, text)) {
                std::istringstream in(text);
                std::string line, outText;
                while (std::getline(in, line)) {
                    const std::string t = trimWs(line);
                    if (t.rfind("map_Kd", 0) == 0) {
                        const std::string rest = trimWs(t.substr(6));
                        for (const auto& rw : mtlTexRewrites[mi])
                            if (rest == rw.first || (!rest.empty() && rest.size() >= rw.first.size() && rest.compare(rest.size() - rw.first.size(), rw.first.size(), rw.first) == 0)) {
                                line = "map_Kd " + rw.second;
                                break;
                            }
                    }
                    outText += line + "\n";
                }
                it.hasText = true;
                it.text = outText;
            }
        }
        items.push_back(it);
    }
    if (needObjRewrite) {
        std::string text;
        if (readTextFile(src, text)) {
            std::istringstream in(text);
            std::string line, outText, libs;
            for (const MtlPick& m : mtls) libs += "mtllib " + m.destName + "\n";
            bool placed = false;
            std::string body;
            while (std::getline(in, line)) {
                const std::string t = trimWs(line);
                if (t.rfind("mtllib", 0) == 0 && (t.size() == 6 || std::isspace(static_cast<unsigned char>(t[6])))) {
                    if (!placed) { body += libs; placed = true; }
                    continue;   // as citações antigas saem
                }
                body += line + "\n";
            }
            items[0].hasText = true;
            items[0].text = placed ? body : libs + body;
        }
    }
    out.mtlCount = static_cast<int>(mtls.size());
    out.textureCount = textureCount;
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
        if (it.hasText) {
            std::ofstream o(dst, std::ios::binary);
            o << it.text;
            if (!o) ec = std::make_error_code(std::errc::io_error);
        } else {
            fs::copy_file(it.from, dst, fs::copy_options::none, ec);
        }
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
