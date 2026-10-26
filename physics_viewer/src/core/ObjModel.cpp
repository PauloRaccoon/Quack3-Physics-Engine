// =============================================================================
// ObjModel.cpp  (V15a)
// =============================================================================
#include "viewer/core/ObjModel.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace viewer {

namespace {

struct ParseError {
    int line;
    std::string message;
};

[[noreturn]] void fail(int line, const std::string& msg) { throw ParseError{line, msg}; }

std::string formatError(const ParseError& e) {
    return e.line > 0 ? "linha " + std::to_string(e.line) + ": " + e.message : e.message;
}

std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> tokens(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream is(s);
    std::string t;
    while (is >> t) out.push_back(t);
    return out;
}

float parseFloat(const std::string& t, int line, const char* what) {
    char* end = nullptr;
    errno = 0;
    const float v = std::strtof(t.c_str(), &end);
    if (end == t.c_str() || *end != '\0' || !std::isfinite(v)) fail(line, std::string("numero invalido em ") + what + ": '" + t + "'");
    return v;
}

/// Índice de uma face: "" vira 0 (ausente); senão inteiro com sinal.
long parseIndex(const std::string& t, int line) {
    if (t.empty()) return 0;
    char* end = nullptr;
    errno = 0;
    const long v = std::strtol(t.c_str(), &end, 10);
    if (end == t.c_str() || *end != '\0' || errno == ERANGE) fail(line, "indice invalido: '" + t + "'");
    return v;
}

int resolve(long idx, std::size_t count, int line, const char* what) {
    if (idx > 0 && static_cast<std::size_t>(idx) <= count) return static_cast<int>(idx - 1);
    if (idx < 0 && static_cast<std::size_t>(-idx) <= count) return static_cast<int>(static_cast<long>(count) + idx);
    fail(line, std::string("indice de ") + what + " fora do intervalo (" + std::to_string(idx) + ", existem " + std::to_string(count) + ")");
}

/// Nome de arquivo relativo seguro: sem "..", sem caminho absoluto nem unidade.
bool safeRelative(std::string& name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    if (name.empty() || name[0] == '/' || name.find(':') != std::string::npos) return false;
    std::istringstream is(name);
    std::string part;
    while (std::getline(is, part, '/'))
        if (part == "..") return false;
    return true;
}

std::string stem(const std::string& path) { return std::filesystem::path(path).stem().string(); }

} // namespace

bool parseObj(const std::string& text, ObjModel& out, std::string& error) {
    out = ObjModel{};
    try {
        if (text.size() > objlimits::kMaxBytes) fail(0, "arquivo grande demais (limite 32 MB)");
        ObjModel m;
        int currentMaterial = -1;
        std::istringstream in(text);
        std::string raw;
        int lineNo = 0;
        while (std::getline(in, raw)) {
            ++lineNo;
            const std::size_t hash = raw.find('#');
            if (hash != std::string::npos) raw.erase(hash);
            const std::string line = trim(raw);
            if (line.empty()) continue;
            const std::vector<std::string> tk = tokens(line);
            const std::string& key = tk[0];
            if (key == "v") {
                if (tk.size() < 4) fail(lineNo, "'v' precisa de x y z");
                if (m.positions.size() >= objlimits::kMaxVertices) fail(lineNo, "vertices demais (limite 400000)");
                Vec3f v{parseFloat(tk[1], lineNo, "v"), parseFloat(tk[2], lineNo, "v"), parseFloat(tk[3], lineNo, "v")};
                if (std::fabs(v.x) > objlimits::kMaxCoordinate || std::fabs(v.y) > objlimits::kMaxCoordinate || std::fabs(v.z) > objlimits::kMaxCoordinate)
                    fail(lineNo, "coordenada fora do limite (1e6)");
                m.positions.push_back(v);
            } else if (key == "vt") {
                if (tk.size() < 2) fail(lineNo, "'vt' precisa de u [v]");
                if (m.uvs.size() >= objlimits::kMaxVertices) fail(lineNo, "coordenadas de textura demais");
                m.uvs.push_back({parseFloat(tk[1], lineNo, "vt"), tk.size() > 2 ? parseFloat(tk[2], lineNo, "vt") : 0.0f});
            } else if (key == "f") {
                if (tk.size() < 4) fail(lineNo, "'f' precisa de ao menos 3 vertices");
                std::vector<int> pi, ti;
                for (std::size_t k = 1; k < tk.size(); ++k) {
                    std::string parts[3];
                    std::size_t np = 0, start = 0;
                    const std::string& t = tk[k];
                    for (std::size_t c = 0; c <= t.size() && np < 3; ++c)
                        if (c == t.size() || t[c] == '/') { parts[np++] = t.substr(start, c - start); start = c + 1; }
                    if (parts[0].empty()) fail(lineNo, "face sem indice de vertice: '" + t + "'");
                    pi.push_back(resolve(parseIndex(parts[0], lineNo), m.positions.size(), lineNo, "vertice"));
                    const long u = parseIndex(parts[1], lineNo);
                    ti.push_back(u == 0 ? -1 : resolve(u, m.uvs.size(), lineNo, "textura"));
                }
                for (std::size_t k = 1; k + 1 < pi.size(); ++k) {   // leque a partir do primeiro vértice
                    if (m.triangles.size() >= objlimits::kMaxTriangles) fail(lineNo, "triangulos demais (limite 800000)");
                    ObjTriangle tr;
                    tr.p[0] = pi[0]; tr.p[1] = pi[k]; tr.p[2] = pi[k + 1];
                    tr.t[0] = ti[0]; tr.t[1] = ti[k]; tr.t[2] = ti[k + 1];
                    tr.material = currentMaterial;
                    m.triangles.push_back(tr);
                }
            } else if (key == "usemtl") {
                const std::string name = trim(line.substr(6));
                if (name.empty()) fail(lineNo, "'usemtl' sem nome");
                int found = -1;
                for (std::size_t i = 0; i < m.materials.size(); ++i) if (m.materials[i].name == name) found = static_cast<int>(i);
                if (found < 0) {
                    ObjMaterial mat;
                    mat.name = name;
                    m.materials.push_back(mat);
                    found = static_cast<int>(m.materials.size()) - 1;
                }
                currentMaterial = found;
            } else if (key == "mtllib") {
                const std::string name = trim(line.substr(6));
                if (name.empty()) fail(lineNo, "'mtllib' sem arquivo");
                m.mtlLibs.push_back(name);
            } else if (key == "o" || key == "g" || key == "s" || key == "vn" || key == "vp" || key == "l" || key == "p" ||
                       key == "usemap" || key == "maplib" || key == "cstype" || key == "deg" || key == "curv" || key == "surf") {
                if (key == "o" && m.name.empty()) m.name = trim(line.substr(1));
            } else {
                fail(lineNo, "comando desconhecido '" + key + "'");
            }
        }
        if (m.positions.empty()) fail(0, "o modelo nao tem vertices");
        if (m.triangles.empty()) fail(0, "o modelo nao tem faces");
        out = std::move(m);
        return true;
    } catch (const ParseError& e) {
        error = formatError(e);
        out = ObjModel{};
        return false;
    }
}

bool parseMtl(const std::string& text, std::vector<ObjMaterial>& out, std::string& error) {
    try {
        if (text.size() > objlimits::kMaxBytes) fail(0, "arquivo grande demais (limite 32 MB)");
        std::vector<ObjMaterial> mats;
        std::istringstream in(text);
        std::string raw;
        int lineNo = 0;
        while (std::getline(in, raw)) {
            ++lineNo;
            const std::size_t hash = raw.find('#');
            if (hash != std::string::npos) raw.erase(hash);
            const std::string line = trim(raw);
            if (line.empty()) continue;
            const std::vector<std::string> tk = tokens(line);
            const std::string& key = tk[0];
            if (key == "newmtl") {
                const std::string name = trim(line.substr(6));
                if (name.empty()) fail(lineNo, "'newmtl' sem nome");
                if (mats.size() >= 4096) fail(lineNo, "materiais demais");
                ObjMaterial m;
                m.name = name;
                mats.push_back(m);
            } else if (key == "Kd") {
                if (mats.empty()) fail(lineNo, "'Kd' antes de 'newmtl'");
                if (tk.size() < 4) fail(lineNo, "'Kd' precisa de r g b");
                for (int i = 0; i < 3; ++i) {
                    const float v = parseFloat(tk[static_cast<std::size_t>(i) + 1], lineNo, "Kd");
                    mats.back().kd[i] = std::min(1.0f, std::max(0.0f, v));
                }
            } else if (key == "map_Kd") {
                if (mats.empty()) fail(lineNo, "'map_Kd' antes de 'newmtl'");
                if (tk.size() < 2) fail(lineNo, "'map_Kd' sem arquivo");
                std::string file = trim(line.substr(6));
                if (file[0] == '-') file = tk.back();   // com opções (-s, -o...): usa a última palavra
                mats.back().diffuseMap = file;
            }   // demais comandos (Ka, Ks, Ns, d, illum, map_Bump...) são ignorados
        }
        out.insert(out.end(), mats.begin(), mats.end());
        return true;
    } catch (const ParseError& e) {
        error = formatError(e);
        return false;
    }
}

namespace {
bool readFile(const std::string& path, std::string& text, std::string& error) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        error = "arquivo nao encontrado: " + path;
        return false;
    }
    if (std::filesystem::file_size(path, ec) > objlimits::kMaxBytes) {
        error = "arquivo grande demais (limite 32 MB): " + path;
        return false;
    }
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "nao foi possivel abrir: " + path;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    text = ss.str();
    return true;
}
} // namespace

bool loadObjFile(const std::string& path, ObjModel& out, std::string& error, std::string* warning) {
    out = ObjModel{};
    std::string text;
    if (!readFile(path, text, error)) return false;
    ObjModel m;
    if (!parseObj(text, m, error)) {
        error = path + ": " + error;
        return false;
    }
    if (m.name.empty()) m.name = stem(path);
    const std::filesystem::path dir = std::filesystem::path(path).parent_path();
    std::string warn;
    for (std::string lib : m.mtlLibs) {
        if (!safeRelative(lib)) { warn += "mtllib recusado (caminho inseguro); "; continue; }
        std::string mtlText, mtlErr;
        if (!readFile((dir / lib).string(), mtlText, mtlErr)) { warn += "sem " + lib + "; "; continue; }
        std::vector<ObjMaterial> defs;
        if (!parseMtl(mtlText, defs, mtlErr)) {
            error = lib + ": " + mtlErr;
            return false;
        }
        for (const ObjMaterial& d : defs)
            for (ObjMaterial& used : m.materials)
                if (used.name == d.name) {
                    used.diffuseMap = d.diffuseMap;
                    for (int i = 0; i < 3; ++i) used.kd[i] = d.kd[i];
                }
    }
    for (ObjMaterial& mat : m.materials) {
        if (mat.diffuseMap.empty()) continue;
        if (!safeRelative(mat.diffuseMap)) { warn += "textura recusada (caminho inseguro); "; mat.diffuseMap.clear(); }
    }
    if (warning) *warning = warn;
    out = std::move(m);
    return true;
}

bool objBounds(const ObjModel& model, Vec3f& lo, Vec3f& hi) {
    if (model.triangles.empty()) return false;
    bool first = true;
    for (const ObjTriangle& t : model.triangles)
        for (int k = 0; k < 3; ++k) {
            const Vec3f& v = model.positions[static_cast<std::size_t>(t.p[k])];
            if (first) { lo = hi = v; first = false; continue; }
            lo.x = std::min(lo.x, v.x); lo.y = std::min(lo.y, v.y); lo.z = std::min(lo.z, v.z);
            hi.x = std::max(hi.x, v.x); hi.y = std::max(hi.y, v.y); hi.z = std::max(hi.z, v.z);
        }
    return true;
}

} // namespace viewer
