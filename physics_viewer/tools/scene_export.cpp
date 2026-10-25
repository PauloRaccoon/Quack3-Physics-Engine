// =============================================================================
// scene_export — grava as cenas embutidas do viewer como arquivos .scene (exemplos).
//     scene_export <pasta>        (sem raylib; usa só o núcleo do viewer)
// =============================================================================
#include <cstdio>
#include <string>

#include "viewer/core/SceneFile.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("uso: scene_export <pasta>\n");
        return 2;
    }
    const std::string dir = argv[1];
    int failures = 0;
    for (std::size_t i = 0; i < viewer::sceneList().size(); ++i) {
        const viewer::Scene s = viewer::makeScene(i);
        char prefix[16];
        std::snprintf(prefix, sizeof prefix, "%02d_", static_cast<int>(i) + 1);
        const std::string path = dir + "/" + prefix + viewer::sanitizeSceneFileName(s.name) + viewer::scenefile::kExtension;
        std::string err;
        if (viewer::saveSceneFile(s, path, err)) {
            std::printf("gravou %s\n", path.c_str());
        } else {
            std::printf("FALHOU %s: %s\n", path.c_str(), err.c_str());
            ++failures;
        }
    }
    return failures == 0 ? 0 : 1;
}
