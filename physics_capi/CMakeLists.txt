cmake_minimum_required(VERSION 3.14)
project(physics_capi VERSION 1.0.0 LANGUAGES C CXX)

# API C do motor de física (V26): biblioteca compartilhada `physics_c` (physics_c.dll / libphysics_c.so /
# libphysics_c.dylib) que expõe o physics_engine para Python, C#, Rust, Lua, Java, Go...
# O motor é a pasta AO LADO (../physics_engine); esta pasta só o consome.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(PC_BUILD_TESTS "Compilar os testes em C puro" ON)
option(PC_USE_DOUBLE "Motor em double (PHYSICS_USE_DOUBLE) em vez de float" OFF)
option(PC_SANITIZE "ASan + UBSan (só GCC/Clang; para os testes)" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# O motor entra como biblioteca estática DENTRO da .so/.dll: precisa ser código independente de posição.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Símbolos escondidos por padrão (vale também para o motor): só o que leva PC_API sai da biblioteca.
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
if(PC_USE_DOUBLE)
  set(PHYSICS_USE_DOUBLE ON CACHE BOOL "" FORCE)
endif()
set(PHYSICS_ENGINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../physics_engine" CACHE PATH "Pasta do physics_engine")
if(NOT EXISTS "${PHYSICS_ENGINE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "physics_engine não encontrado em ${PHYSICS_ENGINE_DIR} (use -DPHYSICS_ENGINE_DIR=<caminho>)")
endif()
if(PC_SANITIZE AND NOT MSVC)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()
add_subdirectory(${PHYSICS_ENGINE_DIR} physics_engine_build)

add_library(physics_c SHARED src/physics_c.cpp)
target_include_directories(physics_c PUBLIC include)
target_compile_definitions(physics_c PRIVATE PC_BUILDING_DLL)
target_link_libraries(physics_c PRIVATE physics::physics)
set_target_properties(physics_c PROPERTIES
  CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)   # só o que tem PC_API é exportado
if(MSVC)
  target_compile_options(physics_c PRIVATE /W4)
else()
  target_compile_options(physics_c PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(physics_c PRIVATE -Wl,--exclude-libs,ALL)
  endif()
  if(MINGW AND NOT PC_SANITIZE)
    # A .dll não pode depender das DLLs do MinGW (libstdc++-6, libgcc_s...): o Python não as acha no PATH.
    target_link_options(physics_c PRIVATE -static-libgcc -static-libstdc++ -static)
  endif()
endif()

if(PC_BUILD_TESTS)
  enable_testing()
  add_executable(capi_tests tests/capi_tests.c)
  target_link_libraries(capi_tests PRIVATE physics_c)
  if(NOT MSVC)
    target_compile_options(capi_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(capi_tests PRIVATE m)
  endif()
  add_test(NAME capi_tests COMMAND capi_tests)

  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND AND NOT PC_SANITIZE)
    add_test(NAME python_tests
      COMMAND ${Python3_EXECUTABLE} -I -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/python/tests -t ${CMAKE_CURRENT_SOURCE_DIR}/python -v
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python)
    set_tests_properties(python_tests PROPERTIES ENVIRONMENT "QUACK_PHYSICS_LIB=$<TARGET_FILE:physics_c>;PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python")
  endif()
endif()
