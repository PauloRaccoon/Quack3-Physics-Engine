cmake_minimum_required(VERSION 3.14)
project(physics_engine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Projeto raiz (cmake -S physics_engine) ou subprojeto (add_subdirectory(../physics_engine) de uma
# pasta ao lado)? Como subprojeto, testes/exemplos/install ficam DESLIGADOS por padrão: quem usa o
# motor só quer a biblioteca `physics` (alias physics::physics).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(PHYSICS_IS_TOP_LEVEL ON)
else()
  set(PHYSICS_IS_TOP_LEVEL OFF)
endif()

option(PHYSICS_USE_DOUBLE "Usar double em vez de float" OFF)
option(PHYSICS_BUILD_TESTS "Compilar testes unitários" ${PHYSICS_IS_TOP_LEVEL})
option(PHYSICS_BUILD_EXAMPLES "Compilar o exemplo (examples/main.cpp)" ${PHYSICS_IS_TOP_LEVEL})
option(PHYSICS_INSTALL "Gerar as regras de instalação (install / find_package)" ${PHYSICS_IS_TOP_LEVEL})

if(PHYSICS_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Todos os .cpp de src/ (recursivo: src/math, src/geometry, ...). Novos módulos entram sozinhos.
file(GLOB_RECURSE PHYSICS_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(physics STATIC ${PHYSICS_SOURCES})
add_library(physics::physics ALIAS physics)
target_compile_features(physics PUBLIC cxx_std_17)
target_include_directories(physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(PHYSICS_USE_DOUBLE)
  target_compile_definitions(physics PUBLIC PHYSICS_USE_DOUBLE)
endif()
if(MSVC)
  target_compile_options(physics PRIVATE /W4)
else()
  target_compile_options(physics PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PHYSICS_BUILD_TESTS)
  enable_testing()
  foreach(test_name math_tests geometry_tests dynamics_tests broadphase_tests narrowphase_tests constraints_tests pipeline_tests raycast_tests fuzz_tests robustness_tests sleeping_tests events_tests capsule_cylinder_tests friction_tests inertia_tests)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE physics)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

if(PHYSICS_BUILD_EXAMPLES)
  add_executable(physics_demo examples/main.cpp)
  target_link_libraries(physics_demo PRIVATE physics)
  if(PHYSICS_BUILD_TESTS)
    add_test(NAME physics_demo COMMAND physics_demo)   # o demo se auto-verifica (retorna 0 se tudo passou)
  endif()
endif()

# -----------------------------------------------------------------------------
# Instalação:  cmake --install <build> --prefix <destino>
# Uso depois:  find_package(physics REQUIRED)  +  target_link_libraries(app PRIVATE physics::physics)
# ATENÇÃO: float e double NÃO são intercambiáveis (ABI diferente). A configuração instalada
# registra qual foi usada em PHYSICS_USE_DOUBLE (ver physicsConfig.cmake).
# -----------------------------------------------------------------------------
if(PHYSICS_INSTALL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  install(TARGETS physics EXPORT physicsTargets
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
          INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
  install(EXPORT physicsTargets FILE physicsTargets.cmake NAMESPACE physics::
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/physics)

  configure_package_config_file(cmake/physicsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/physicsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/physics)
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/physicsConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/physicsConfig.cmake
                ${CMAKE_CURRENT_BINARY_DIR}/physicsConfigVersion.cmake
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/physics)
endif()
