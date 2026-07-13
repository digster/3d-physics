# ----------------------------------------------------------------------------
#  tests/ — doctest unit tests, registered with CTest.
#  The math is header-only, so the tests do NOT link the SDL-dependent common
#  library; they just include the headers. This keeps `ctest` fast and free of
#  any windowing requirement.
# ----------------------------------------------------------------------------
add_executable(p3d_tests
    test_main.cpp
    test_math.cpp
    test_integration.cpp
    test_physics.cpp
    test_rigidbody.cpp
    test_collision.cpp
    test_solver.cpp)

target_include_directories(p3d_tests PRIVATE ${CMAKE_SOURCE_DIR})
# test_physics.cpp exercises the force generators (forces.cpp), so it links the
# physics engine. p3d_physics has no SDL dependency, keeping the tests headless.
target_link_libraries(p3d_tests PRIVATE doctest::doctest p3d_physics project_warnings)

add_test(NAME unit_tests COMMAND p3d_tests)
