// =============================================================================
// scene_export — grava as cenas embutidas do viewer como arquivos .scene (exemplos).
//     scene_export <pasta>        (sem raylib; usa só o núcleo do viewer)
// =============================================================================
#include <cstdio>
#include <string>

#include "viewer/core/JointEditor.h"
#include "viewer/core/ObjectSpawner.h"
#include "viewer/core/SceneFile.h"

// Exemplo de cena montada só com as funções do editor (V14): chão, rampa e muro estáticos, esfera que desce a
// rampa e uma pá presa ao mundo por uma dobradiça de eixo Z que a bola derruba.
static viewer::Scene makeEditorExample() {
    using namespace viewer;
    using physics::Real;
    using physics::Vector3;
    Scene s = makeEmptyScene(true);
    s.name = "Meu cenario (editor)";
    SpawnParams ramp;
    ramp.isStatic = true;
    ramp.boxScale = Vector3(6, Real(0.3), 3);
    ramp.position = Vector3(-4, Real(2.2), 0);
    ramp.orientation = orientationFromAxisAngle(Vector3(0, 0, 1), Real(-0.4));
    spawnBody(s, ramp);
    SpawnParams wall;
    wall.isStatic = true;
    wall.boxScale = Vector3(Real(0.4), 3, 6);
    wall.position = Vector3(8, Real(1.5), 0);
    spawnBody(s, wall);
    SpawnParams ball;
    ball.kind = SpawnKind::Sphere;
    ball.mass = 2;
    ball.position = Vector3(-6, 5, 0);
    ball.restitution = Real(0.3);
    const physics::BodyId ballId = spawnBody(s, ball);
    SpawnParams paddle;
    paddle.boxScale = Vector3(Real(0.3), 4, 2);
    paddle.mass = 3;
    paddle.position = Vector3(3, Real(2.5), 0);
    const physics::BodyId paddleId = spawnBody(s, paddle);
    JointParams hinge;
    hinge.kind = JointKind::Hinge;
    hinge.a = paddleId;
    hinge.anchor = Vector3(3, Real(0.4), 0);
    hinge.axis = Vector3(0, 0, 1);
    createJoint(s, hinge);
    s.focus = ballId;
    return s;
}

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
    {   // exemplo do editor (V14)
        const viewer::Scene s = makeEditorExample();
        const std::string path = dir + "/15_Meu_cenario_editor" + viewer::scenefile::kExtension;
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
