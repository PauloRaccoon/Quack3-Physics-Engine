// =============================================================================
// AppInfo.h  (V23)
// Nome e versão do programa. O programa se chama Quack3; a versão é v0.<número da fase>
// (V23 do roteiro = v0.23). Sem raylib, para o teste conferir o formato.
// =============================================================================
#pragma once

#include <string>

namespace viewer::appinfo {

constexpr const char* kName = "Quack3";
constexpr int kVersionMajor = 0;
constexpr int kVersionMinor = 24;   ///< = número da fase (V24)
constexpr int kVersionPatch = 0;    ///< correções entre fases (v0.23.1); 0 = sem sufixo

/// "v0.23.1"
inline std::string versionString() {
    std::string v = "v" + std::to_string(kVersionMajor) + "." + std::to_string(kVersionMinor);
    if (kVersionPatch > 0) v += "." + std::to_string(kVersionPatch);
    return v;
}

/// "Quack3 v0.23.1": título da janela.
inline std::string windowTitle() { return std::string(kName) + " " + versionString(); }

} // namespace viewer::appinfo
