cmake_minimum_required(VERSION 3.14)
project(physics_viewer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VIEWER_BUILD_APP "Compilar a janela (usa a raylib; baixa e compila se não estiver instalada)" ON)
option(VIEWER_BUILD_TESTS "Compilar os testes do núcleo (não precisam da raylib)" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# -----------------------------------------------------------------------------
# O motor: a pasta AO LADO. Como subprojeto, os testes e o demo dele ficam desligados.
# A precisão (float/double) é a do motor: passe -DPHYSICS_USE_DOUBLE=ON para usar double.
# -----------------------------------------------------------------------------
set(PHYSICS_ENGINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../physics_engine" CACHE PATH "Pasta do physics_engine")
if(NOT EXISTS "${PHYSICS_ENGINE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "physics_engine não encontrado em ${PHYSICS_ENGINE_DIR} "
                      "(use -DPHYSICS_ENGINE_DIR=<caminho>)")
endif()
add_subdirectory(${PHYSICS_ENGINE_DIR} physics_engine_build)

# -----------------------------------------------------------------------------
# Núcleo do viewer: matemática, câmera orbital e cenas. SEM raylib.
# -----------------------------------------------------------------------------
add_library(viewer_core STATIC
  src/core/ViewerMath.cpp
  src/core/OrbitCamera.cpp
  src/core/RenderPose.cpp
  src/core/Interaction.cpp
  src/core/JointEditor.cpp
  src/core/ConvexHull.cpp
  src/core/ObjModel.cpp
  src/core/ModelCollider.cpp
  src/core/ModelAsset.cpp
  src/core/TrajectoryRecorder.cpp
  src/core/ModelImporter.cpp
  src/core/TimeControl.cpp
  src/core/BodyEditor.cpp
  src/core/PolyShapes.cpp
  src/core/ObjectSpawner.cpp
  src/core/ContactTracker.cpp
  src/core/SceneFile.cpp
  src/core/DockLayout.cpp
  src/core/WorldEditor.cpp
  src/core/Scene.cpp)
target_include_directories(viewer_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_core PUBLIC physics::physics)
if(MSVC)
  target_compile_options(viewer_core PRIVATE /W4)
else()
  target_compile_options(viewer_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Ferramenta (sem raylib): grava as cenas embutidas como arquivos .scene de exemplo.
add_executable(scene_export tools/scene_export.cpp)
target_link_libraries(scene_export PRIVATE viewer_core)

if(VIEWER_BUILD_TESTS)
  enable_testing()
  add_executable(viewer_tests tests/viewer_tests.cpp)
  target_link_libraries(viewer_tests PRIVATE viewer_core)
  target_compile_definitions(viewer_tests PRIVATE VIEWER_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
  add_test(NAME viewer_tests COMMAND viewer_tests)

  # Fumaça sem janela: todas as cenas por 20 s + fuzz de criar/editar/apagar/trocar de cena.
  add_executable(viewer_smoke tests/viewer_smoke.cpp)
  target_link_libraries(viewer_smoke PRIVATE viewer_core)
  target_compile_definitions(viewer_smoke PRIVATE VIEWER_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
  add_test(NAME viewer_smoke COMMAND viewer_smoke)
endif()

# -----------------------------------------------------------------------------
# A janela (raylib). Usa a raylib instalada no sistema, se houver; senão baixa a 5.5.
# -----------------------------------------------------------------------------
if(VIEWER_BUILD_APP)
  find_package(raylib 5.0 QUIET)
  if(NOT raylib_FOUND)
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
      GIT_REPOSITORY https://github.com/raysan5/raylib.git
      GIT_TAG 5.5
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(raylib)
  endif()

  # raygui: um único cabeçalho (src/raygui.h), baixado do GitHub (a raylib não o inclui).
  include(FetchContent)
  FetchContent_Declare(raygui
    GIT_REPOSITORY https://github.com/raysan5/raygui.git
    GIT_TAG 4.0
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(raygui)

  add_executable(physics_viewer src/app/main.cpp)
  # Opcional: roda a janela por alguns quadros e sai (precisa de tela/OpenGL; por isso desligado por padrão).
  option(VIEWER_TEST_APP "Registrar no ctest um teste que abre a janela por 5 quadros" OFF)
  if(VIEWER_TEST_APP AND VIEWER_BUILD_TESTS)
    add_test(NAME viewer_app_frames COMMAND physics_viewer --frames 5)
  endif()
  target_link_libraries(physics_viewer PRIVATE viewer_core raylib)
  # Pasta das cenas salvas/carregadas pela aba "Cenas" (a pasta scenes/ ao lado do código).
  target_compile_definitions(physics_viewer PRIVATE VIEWER_SCENES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scenes"
                                                  VIEWER_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
  # SYSTEM: o código do raygui não precisa passar pelos nossos avisos de compilação.
  target_include_directories(physics_viewer SYSTEM PRIVATE ${raygui_SOURCE_DIR}/src)
  if(MSVC)
    target_compile_options(physics_viewer PRIVATE /W4)
  else()
    target_compile_options(physics_viewer PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()
