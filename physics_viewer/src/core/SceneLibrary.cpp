// =============================================================================
// SceneLibrary.cpp
// =============================================================================
#include "viewer/core/SceneLibrary.h"

#include <algorithm>
#include <cctype>
#include <sstream>

#include "viewer/core/SceneFile.h"

namespace viewer {

namespace {

LibraryEntry makeBuiltin(int i) {
    LibraryEntry e;
    e.kind = LibraryEntry::Kind::Builtin;
    e.builtinIndex = i;
    e.name = sceneList()[static_cast<std::size_t>(i)].name;
    return e;
}

LibraryEntry makeFile(const std::string& file) {
    LibraryEntry e;
    e.kind = LibraryEntry::Kind::File;
    e.file = file;
    e.name = file;
    return e;
}

/// Tira "NN_" (dígitos + sublinhado) do começo.
std::string stripNumberPrefix(const std::string& f) {
    std::size_t i = 0;
    while (i < f.size() && std::isdigit(static_cast<unsigned char>(f[i]))) ++i;
    if (i > 0 && i < f.size() && f[i] == '_') return f.substr(i + 1);
    return f;
}

bool isSafeFileToken(const std::string& s) {
    if (s.empty() || s.size() > 128) return false;
    for (unsigned char c : s)
        if (!(std::isalnum(c) || c == '_' || c == '-' || c == '.' || c == ' ')) return false;   // sem barras: só nomes
    return s.find("..") == std::string::npos;
}

} // namespace

bool isBuiltinExportName(const std::string& file) {
    const std::string tail = stripNumberPrefix(file);
    if (tail == file) return false;   // sem "NN_": é um arquivo do usuário
    for (const SceneInfo& info : sceneList())
        if (sanitizeSceneFileName(info.name) == tail) return true;
    return false;
}

void SceneLibrary::rebuild(const std::vector<std::string>& files, const std::string& savedText) {
    entries_.clear();
    hidden_.clear();
    const int builtinCount = static_cast<int>(sceneList().size());
    std::set<std::string> fileSet(files.begin(), files.end());
    std::set<int> haveBuiltin;
    std::set<std::string> haveFile;

    if (savedText.size() <= scenelibrary::kMaxBytes) {
        std::istringstream in(savedText);
        std::string line;
        bool first = true;
        while (std::getline(in, line)) {
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
            if (line.empty()) continue;
            if (first) {   // cabeçalho inválido: ignora o arquivo todo (cai na ordem padrão)
                first = false;
                if (line != scenelibrary::kHeader) break;
                continue;
            }
            std::istringstream ls(line);
            std::string kind;
            ls >> kind;
            if (kind == "builtin" || kind == "hidden") {
                int idx = -1;
                if (!(ls >> idx) || idx < 0 || idx >= builtinCount) continue;
                if (kind == "hidden") {
                    if (!haveBuiltin.count(idx)) hidden_.insert(idx);
                } else if (!haveBuiltin.count(idx) && !hidden_.count(idx) && entries_.size() < scenelibrary::kMaxEntries) {
                    haveBuiltin.insert(idx);
                    entries_.push_back(makeBuiltin(idx));
                }
            } else if (kind == "file") {
                std::string name;
                std::getline(ls >> std::ws, name);
                while (!name.empty() && name.back() == ' ') name.pop_back();
                if (isSafeFileToken(name) && fileSet.count(name) && !haveFile.count(name) && entries_.size() < scenelibrary::kMaxEntries) {
                    haveFile.insert(name);
                    entries_.push_back(makeFile(name));
                }
            }
        }
    }
    // O que não estava no texto salvo entra no fim: originais (novas) e depois arquivos novos.
    for (int i = 0; i < builtinCount && entries_.size() < scenelibrary::kMaxEntries; ++i)
        if (!haveBuiltin.count(i) && !hidden_.count(i)) entries_.push_back(makeBuiltin(i));
    for (const std::string& f : files) {
        if (entries_.size() >= scenelibrary::kMaxEntries) break;
        if (haveFile.count(f) || isBuiltinExportName(f) || !isSafeFileToken(f)) continue;
        haveFile.insert(f);
        entries_.push_back(makeFile(f));
    }
}

int SceneLibrary::findFile(const std::string& file) const {
    for (std::size_t i = 0; i < entries_.size(); ++i)
        if (entries_[i].kind == LibraryEntry::Kind::File && entries_[i].file == file) return static_cast<int>(i);
    return -1;
}

int SceneLibrary::findBuiltin(int builtinIndex) const {
    for (std::size_t i = 0; i < entries_.size(); ++i)
        if (entries_[i].kind == LibraryEntry::Kind::Builtin && entries_[i].builtinIndex == builtinIndex) return static_cast<int>(i);
    return -1;
}

bool SceneLibrary::moveUp(std::size_t i, std::size_t* newIndex) {
    if (i == 0 || i >= entries_.size()) return false;
    std::swap(entries_[i], entries_[i - 1]);
    if (newIndex) *newIndex = i - 1;
    return true;
}

bool SceneLibrary::moveDown(std::size_t i, std::size_t* newIndex) {
    if (i + 1 >= entries_.size()) return false;
    std::swap(entries_[i], entries_[i + 1]);
    if (newIndex) *newIndex = i + 1;
    return true;
}

bool SceneLibrary::remove(std::size_t i, LibraryEntry* removed) {
    if (i >= entries_.size() || entries_.size() <= 1) return false;   // sempre sobra pelo menos uma cena
    if (removed) *removed = entries_[i];
    if (entries_[i].kind == LibraryEntry::Kind::Builtin) hidden_.insert(entries_[i].builtinIndex);
    entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(i));
    return true;
}

std::size_t SceneLibrary::addFile(const std::string& file) {
    const int existing = findFile(file);
    if (existing >= 0) {
        entries_[static_cast<std::size_t>(existing)].broken = false;
        return static_cast<std::size_t>(existing);
    }
    entries_.push_back(makeFile(file));
    return entries_.size() - 1;
}

std::size_t SceneLibrary::restoreBuiltins() {
    std::size_t n = 0;
    for (int idx : hidden_) {
        if (entries_.size() >= scenelibrary::kMaxEntries) break;
        entries_.push_back(makeBuiltin(idx));
        ++n;
    }
    hidden_.clear();
    return n;
}

std::string SceneLibrary::serialize() const {
    std::ostringstream o;
    o << scenelibrary::kHeader << "\n";
    for (const LibraryEntry& e : entries_) {
        if (e.kind == LibraryEntry::Kind::Builtin) o << "builtin " << e.builtinIndex << "\n";
        else o << "file " << e.file << "\n";
    }
    for (int idx : hidden_) o << "hidden " << idx << "\n";
    return o.str();
}

bool buildLibraryScene(const LibraryEntry& entry, const std::string& sceneDir, const std::string& modelDir, Scene& out,
                       std::string& error, std::string* warning) {
    if (entry.kind == LibraryEntry::Kind::Builtin) {
        if (entry.builtinIndex < 0 || static_cast<std::size_t>(entry.builtinIndex) >= sceneList().size()) {
            error = "cena original inexistente";
            return false;
        }
        out = makeScene(static_cast<std::size_t>(entry.builtinIndex));
        return true;
    }
    return loadSceneFile(sceneDir + "/" + entry.file + scenefile::kExtension, out, error, modelDir, warning);
}

} // namespace viewer
