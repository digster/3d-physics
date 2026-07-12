# 3D Physics From Scratch

A hands-on tutorial for learning **3D physics** by building a small physics
engine from nothing but vectors and Newton's laws — in **C++20** and **SDL3**.

Every concept ships as a pair:

- a **runnable demo** (`chapters/NN-topic/main.cpp`) — small, self-contained,
  and heavily commented, and
- an **in-depth documentation page** (`docs/chapters/NN-topic.html`) that
  explains the intuition, the math (with diagrams), and the implementation.

There is **no engine dependency** — no Bullet, no Jolt, no PhysX. We write the
integrators, the collision detection, and the constraint solver ourselves,
because understanding *why* they work is the entire point. There is also **no
graphics dependency** beyond SDL3: the demos use a tiny software 3D renderer
that we build in Part I, so the 3D math is tutorial content rather than a
black box.

> **New here? Start with [`docs/index.html`](docs/index.html)** — open it in
> any browser (no server or build step required) for the full curriculum,
> reading order, and diagrams.

---

## What you will build

The curriculum runs from "what is a vector" to a small but real rigid-body
engine with joints and soft bodies:

| Part | Chapters | You learn to… |
|------|----------|---------------|
| I · Foundations        | 1–5   | run an SDL3 loop, and build the vector/matrix/quaternion + software-render + numerical-integration toolkit everything else stands on |
| II · Particles & Forces| 6–9   | simulate points under forces: fountains, springs, ropes, cloth |
| III · Rigid Bodies     | 10–11 | rotate bodies correctly with quaternions and inertia tensors |
| IV · Collision Detection| 12–15| find contacts: broadphase, SAT, GJK/EPA |
| V · Collision Response | 16–18 | make things bounce, rest, stack, and sleep with an impulse solver |
| VI · Constraints & Joints| 19  | connect bodies: chains, hinges, ragdolls |
| VII · Beyond Rigid Bodies| 20  | XPBD soft bodies (jello) |

*(Parts I–III are implemented; later parts land part by part.)*

---

## Quick start

### Prerequisites

- A C++20 compiler (AppleClang 15+, GCC 11+, or MSVC 2022)
- [CMake](https://cmake.org/) 3.16+
- [SDL3](https://libsdl.org/) — optional. If it is not installed, the build
  fetches and compiles it automatically.

On macOS: `brew install sdl3 cmake`

### Build

```sh
cmake -B build
cmake --build build -j
```

All demo binaries land in `build/bin/`.

### Run a demo

```sh
./build/bin/ch01_hello_sdl3      # the first chapter
./build/bin/ch05_integration     # numerical integration playground
./build/bin/ch09_cloth           # a flag waving in the wind (Part II finale)
./build/bin/ch11_rigidbody       # the self-flipping Dzhanibekov T-handle (Part III)
```

Every 3D demo shares the same controls:

| Input | Action |
|-------|--------|
| Left-drag | Orbit the camera |
| Mouse wheel | Zoom |
| `Space` | Pause / resume the simulation |
| `.` | Single-step one physics tick (while paused) |
| `R` | Reset the scene |
| `1`–`9` | Switch scene variant |
| `W` | Toggle wireframe |
| `H` | Toggle the on-screen help/HUD |
| `Esc` | Quit |

### Run the tests

```sh
ctest --test-dir build --output-on-failure
```

Each demo also supports a headless smoke test used by CI and for verification:

```sh
./build/bin/ch03_matrices --selftest   # renders a few frames offscreen, saves a BMP, exits 0
```

---

## Repository layout

```
common/     Reusable framework introduced in Part I:
  math/       vec3, vec4, mat4, quat  (header-only, exhaustively commented)
  render/     orbit camera, software renderer, mesh primitives, debug draw
  app.*       window + fixed-timestep loop + input + HUD + --selftest harness
physics/    The from-scratch engine (added starting in Part II)
chapters/   One tiny executable per chapter (NN-topic/main.cpp)
tests/      doctest unit tests
docs/        Static HTML tutorial — open docs/index.html, no build step
```

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the "why" behind the structure and
the data flow through the software renderer and (later) the physics step.

## License

MIT — see [`LICENSE`](LICENSE).
