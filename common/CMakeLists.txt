# ----------------------------------------------------------------------------
#  common/ — the reusable framework library shared by every chapter.
#  Header-only parts (math/, camera, color) need no compilation; the .cpp files
#  below carry the renderer, meshes, debug draw, and the app loop.
# ----------------------------------------------------------------------------
add_library(p3d_common
    render/mesh.cpp
    render/renderer3d.cpp
    render/debug_draw.cpp
    app.cpp)

# Expose the project root as an include root so consumers write
# `#include "common/app.hpp"`. SDL3 is PUBLIC because app.hpp includes SDL.
target_include_directories(p3d_common PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(p3d_common
    PUBLIC  SDL3::SDL3
    PRIVATE project_warnings)
