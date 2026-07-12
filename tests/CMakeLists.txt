# ----------------------------------------------------------------------------
#  tests/ — doctest unit tests, registered with CTest.
#  The math is header-only, so the tests do NOT link the SDL-dependent common
#  library; they just include the headers. This keeps `ctest` fast and free of
#  any windowing requirement.
# ----------------------------------------------------------------------------
add_executable(p3d_tests
    test_main.cpp
    test_math.cpp
    test_integration.cpp)

target_include_directories(p3d_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(p3d_tests PRIVATE doctest::doctest project_warnings)

add_test(NAME unit_tests COMMAND p3d_tests)
