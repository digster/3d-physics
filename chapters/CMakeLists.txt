# ----------------------------------------------------------------------------
#  chapters/ — one small executable per tutorial chapter.
#  add_chapter(<target-name> <subdir>) compiles <subdir>/main.cpp and links the
#  shared framework. Binaries land in build/bin (set at the top level).
# ----------------------------------------------------------------------------
# add_chapter(<target> <subdir> [physics])
# Pass a third argument to also link the from-scratch physics engine (Part II+).
function(add_chapter target dir)
    add_executable(${target} ${dir}/main.cpp)
    target_link_libraries(${target} PRIVATE p3d_common project_warnings)
    if(ARGC GREATER 2)
        target_link_libraries(${target} PRIVATE p3d_physics)
    endif()
endfunction()

# Part I — Foundations
add_chapter(ch01_hello_sdl3  01-hello-sdl3)
add_chapter(ch02_vectors     02-vectors)
add_chapter(ch03_matrices    03-matrices)
add_chapter(ch04_camera      04-camera)
add_chapter(ch05_integration 05-integration)

# Part II — Particles & Forces (link the physics engine)
add_chapter(ch06_particles   06-particles   physics)
add_chapter(ch07_springs     07-springs     physics)
add_chapter(ch08_ropes       08-ropes       physics)
add_chapter(ch09_cloth       09-cloth       physics)

# Part III — Rigid Bodies
add_chapter(ch10_quaternions 10-quaternions physics)
add_chapter(ch11_rigidbody   11-rigidbody   physics)

# Part IV — Collision Detection
add_chapter(ch12_broadphase  12-broadphase  physics)
add_chapter(ch13_narrowphase 13-narrowphase physics)
add_chapter(ch14_sat         14-sat         physics)
add_chapter(ch15_gjk         15-gjk         physics)

# Part V — Collision Response
add_chapter(ch16_impulses    16-impulses    physics)
add_chapter(ch17_friction    17-friction    physics)
add_chapter(ch18_resting     18-resting     physics)

# Part VI — Constraints & Joints
add_chapter(ch19_joints      19-joints      physics)

# Part VII — Beyond Rigid Bodies
add_chapter(ch20_xpbd        20-xpbd        physics)
