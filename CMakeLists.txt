# ============================================================================
#  3D Physics From Scratch — root build script
# ----------------------------------------------------------------------------
#  This one file builds:
#    * common/   the shared framework library (math, software renderer, app loop)
#    * physics/  the from-scratch physics engine (grows part by part)
#    * chapters/ one small executable per tutorial chapter
#    * tests/    doctest unit tests, registered with CTest
#
#  Design goals for the build itself (it is part of the tutorial too):
#    * Zero mandatory third-party downloads: we prefer a system SDL3, and only
#      fall back to fetching it if none is installed.
#    * Loud compiler warnings, because a physics engine is exactly the kind of
#      code where a silent narrowing conversion becomes a mysterious NaN.
# ============================================================================
cmake_minimum_required(VERSION 3.16)

project(physics3d
    VERSION 0.1.0
    DESCRIPTION "A from-scratch 3D physics engine tutorial in SDL3 + C++"
    LANGUAGES CXX)

# --- Language level ---------------------------------------------------------
# C++20 gives us designated initialisers, <numbers> (std::numbers::pi), and
# concise constexpr — all of which keep the math code readable.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Put every executable in build/bin so the docs can give one run command.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Default to a Release-ish build if the user did not pick one. Physics with
# -O0 can be deceptively slow, which muddies the "is my algorithm slow or is
# it just a debug build?" question for a learner.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# --- Warnings ---------------------------------------------------------------
# Applied via a tiny INTERFACE target that every one of our own targets links.
add_library(project_warnings INTERFACE)
if(MSVC)
    target_compile_options(project_warnings INTERFACE /W4)
else()
    target_compile_options(project_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow)
endif()

# ----------------------------------------------------------------------------
#  SDL3: prefer a system install, fall back to fetching the pinned release.
# ----------------------------------------------------------------------------
find_package(SDL3 CONFIG QUIET)
if(NOT SDL3_FOUND)
    message(STATUS "System SDL3 not found — fetching release-3.4.8 with FetchContent.")
    include(FetchContent)
    FetchContent_Declare(
        SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG        release-3.4.8
        GIT_SHALLOW    TRUE)
    # We only need the library, not SDL's own tests/examples.
    set(SDL_TEST      OFF CACHE BOOL "" FORCE)
    set(SDL_EXAMPLES  OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(SDL3)
else()
    message(STATUS "Using system SDL3 ${SDL3_VERSION}")
endif()

# ----------------------------------------------------------------------------
#  doctest: a single-header unit-test framework, fetched at configure time.
# ----------------------------------------------------------------------------
option(PHYS3D_BUILD_TESTS "Build the unit tests" ON)
if(PHYS3D_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG        v2.4.11
        GIT_SHALLOW    TRUE)
    # doctest's own CMakeLists sets a very old cmake_minimum_required that newer
    # CMake refuses; this policy knob lets it configure under CMake 4.x.
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
    FetchContent_MakeAvailable(doctest)
    enable_testing()
endif()

# --- Our own code -----------------------------------------------------------
add_subdirectory(common)     # the reusable framework (math + render + app)
# physics/ is added here once Part II introduces it:
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/physics/CMakeLists.txt")
    add_subdirectory(physics)
endif()
add_subdirectory(chapters)   # the per-chapter demo executables
if(PHYS3D_BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Friendly summary so a first-time builder knows where things went.
message(STATUS "----------------------------------------------------------")
message(STATUS " physics3d configured.")
message(STATUS "   Build:      cmake --build ${CMAKE_BINARY_DIR} -j")
message(STATUS "   Run a demo: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ch01_hello_sdl3")
message(STATUS "   Run tests:  ctest --test-dir ${CMAKE_BINARY_DIR}")
message(STATUS "----------------------------------------------------------")
