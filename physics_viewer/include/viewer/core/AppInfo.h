// =============================================================================
// AppInfo.h
// Nome e versão do programa (Quack3). Sem raylib, para o teste conferir o formato.
// =============================================================================
#pragma once

#include <string>

namespace viewer::appinfo {

constexpr const char* kName = "Quack3";
constexpr int kVersionMajor = 1;
constexpr int kVersionMinor = 0;
constexpr int kVersionPatch = 0;

/// "v1.0.0"
inline std::string versionString() {
    return "v" + std::to_string(kVersionMajor) + "." + std::to_string(kVersionMinor) + "." + std::to_string(kVersionPatch);
}

/// "Quack3 v1.0.0": título da janela.
inline std::string windowTitle() { return std::string(kName) + " " + versionString(); }

} // namespace viewer::appinfo
