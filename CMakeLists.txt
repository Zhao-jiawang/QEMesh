cmake_minimum_required(VERSION 3.16)
project(QEM_Simplifier LANGUAGES CXX)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mesh_core
    src/core/Mesh.cpp
    src/core/Simplify.cpp
    src/core/Tsv.cpp
    src/core/HalfEdgeMesh.cpp
)

target_include_directories(mesh_core PUBLIC src/core)

add_executable(mesh_simp
    src/cli/main.cpp
)

target_link_libraries(mesh_simp PRIVATE mesh_core)

include(FetchContent)

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
)

FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG v0.1.36
)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.4
)

set(GLAD_PROFILE "core" CACHE STRING "" FORCE)
set(GLAD_API "gl=3.3" CACHE STRING "" FORCE)

FetchContent_MakeAvailable(glfw glad imgui)

set(IMGUI_SOURCES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
)

add_library(imgui ${IMGUI_SOURCES})

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${imgui_SOURCE_DIR}/misc/cpp
)

target_link_libraries(imgui PUBLIC glfw glad)

target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)

find_package(OpenGL REQUIRED)

add_executable(mesh_viewer
    src/viewer/main.cpp
    src/viewer/Shader.cpp
)

target_include_directories(mesh_viewer PRIVATE src/core)

target_link_libraries(mesh_viewer PRIVATE mesh_core imgui glad glfw OpenGL::GL)
