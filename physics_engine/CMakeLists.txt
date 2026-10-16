cmake_minimum_required(VERSION 3.14)
project(physics_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PHYSICS_USE_DOUBLE "Usar double em vez de float" OFF)
option(PHYSICS_BUILD_TESTS "Compilar testes unitários" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Todos os .cpp de src/ (recursivo: src/math, src/geometry, ...). Novos módulos entram sozinhos.
file(GLOB_RECURSE PHYSICS_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(physics STATIC ${PHYSICS_SOURCES})
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
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
  foreach(test_name math_tests geometry_tests dynamics_tests broadphase_tests narrowphase_tests)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE physics)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
