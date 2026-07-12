# ----------------------------------------------------------------------------
#  chapters/ — one small executable per tutorial chapter.
#  add_chapter(<target-name> <subdir>) compiles <subdir>/main.cpp and links the
#  shared framework. Binaries land in build/bin (set at the top level).
# ----------------------------------------------------------------------------
function(add_chapter target dir)
    add_executable(${target} ${dir}/main.cpp)
    target_link_libraries(${target} PRIVATE p3d_common project_warnings)
endfunction()

# Part I — Foundations
add_chapter(ch01_hello_sdl3  01-hello-sdl3)
add_chapter(ch02_vectors     02-vectors)
add_chapter(ch03_matrices    03-matrices)
add_chapter(ch04_camera      04-camera)
add_chapter(ch05_integration 05-integration)
