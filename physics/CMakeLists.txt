# ----------------------------------------------------------------------------
#  physics/ — the from-scratch engine, introduced in Part II and grown per part.
#  It depends only on the header-only math in common/ (no SDL, no rendering), so
#  the physics can be unit-tested in isolation and reused anywhere.
#  Header-only pieces (particle, constraint) need no compilation; forces.cpp is
#  the only translation unit so far.
# ----------------------------------------------------------------------------
add_library(p3d_physics
    forces.cpp
    broadphase.cpp
    narrowphase.cpp
    sat.cpp
    gjk.cpp
    solver.cpp
    world.cpp)

target_include_directories(p3d_physics PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(p3d_physics PRIVATE project_warnings)
