# Architecture

This document explains the *big picture* — the pieces, how they fit, and the
reasoning behind the structural choices. If you only want to run things, see
[`README.md`](README.md). If you want to learn the physics, read the tutorial
in [`docs/`](docs/index.html). This file is for someone modifying the code.

## The shape of the project

There are three layers, from most reusable to most specific:

```
        ┌─────────────────────────────────────────────┐
        │  chapters/NN-topic/main.cpp                  │  ← one thin demo per chapter
        │  (scene setup + narration in comments)       │
        └───────────────┬───────────────┬─────────────┘
                        │               │
             uses       │               │  uses (Part II onward)
                        ▼               ▼
        ┌───────────────────────┐   ┌──────────────────────────┐
        │  common/              │   │  physics/                │
        │   math   render  app  │   │  particles → rigid bodies │
        │  (the framework)      │   │  → collision → solver     │
        └───────────┬───────────┘   └───────────┬──────────────┘
                    │                            │
                    └──────────────┬─────────────┘
                                   ▼
                            ┌────────────┐
                            │   SDL3     │  window, input, 2D triangle raster
                            └────────────┘
```

**Why this split?** The tutorial grows by *addition*, not rewriting. Chapter 14
should be able to reuse the exact camera and renderer from chapter 4 without
copy-paste, so the reusable parts live in `common/` and `physics/` and each
chapter's `main.cpp` stays small enough to read in one sitting. Each file in
`common/` and `physics/` carries a header comment noting the chapter that
introduces it, so you can read the codebase in tutorial order.

## `common/` — the framework (Part I)

| Module | Responsibility | Introduced in |
|--------|----------------|---------------|
| `math/vec3.hpp`   | 3D vectors + operations (dot, cross, length, normalize) | Ch 2 |
| `math/vec4.hpp`   | 4D vectors for homogeneous coordinates / projection | Ch 4 |
| `math/mat4.hpp`   | column-major 4×4 matrices; transforms, lookAt, perspective | Ch 3–4 |
| `math/quat.hpp`   | quaternions: rotation without gimbal lock | Ch 10 |
| `render/camera.*` | orbit camera → view & projection matrices | Ch 4 |
| `render/renderer3d.*` | the software 3D pipeline (see below) | Ch 4 |
| `render/mesh.*`   | primitive mesh generators (box, sphere, plane, …) | Ch 4 |
| `render/debug_draw.*` | immediate-mode lines/arrows/AABBs/gizmos for visualizing physics | Ch 4 |
| `app.*`           | window, the fixed-timestep loop, input, HUD, `--selftest` | Ch 3 |

### The software 3D pipeline (`render/renderer3d.*`)

We do **not** use a GPU shader pipeline. Instead we transform geometry on the
CPU and hand SDL flat 2D triangles. This is deliberate: the transform → clip →
project → shade sequence *is* chapters 3–4, so it must be code the reader owns.

Data flow for one frame:

```
  Mesh (model-space vertices + triangle indices)
    │  × model matrix            (place the object in the world)
    ▼
  world space
    │  × view matrix             (move the world in front of the camera)
    ▼
  view/camera space  ──► backface cull (skip triangles facing away)
    │                └─► flat shading (one lambert term from a face normal)
    │  near-plane clip           (split triangles straddling the camera plane)
    ▼
  clip space  ── ÷ w ──►  NDC  ──► viewport transform ──► screen pixels
    │
    ▼
  painter's algorithm: sort all triangles back-to-front by view depth,
  then emit them in one SDL_RenderGeometry call.
```

**Key design decision — one global triangle buffer per frame.** `Renderer3D`
accumulates triangles from *every* mesh into a single list, then sorts once in
`end()`. If each mesh sorted and drew itself, objects could not correctly
occlude one another. The trade-off is the painter's algorithm's known
limitation: mutually interpenetrating triangles can sort wrong for a frame.
For a physics tutorial that is acceptable — interpenetration is exactly the
state the collision response is about to fix, and the `W` wireframe toggle makes
overlaps legible.

### The fixed-timestep loop (`app.*`)

Physics is integrated at a **fixed** `dt` (default 1/120 s) regardless of frame
rate, using the accumulator pattern (Glenn Fiedler's "Fix Your Timestep"):

```
  accumulator += frameTime (clamped, to survive a debugger pause)
  while accumulator >= dt:
      previousState = currentState        # keep last two states…
      fixedUpdate(dt)                      # …integrate one deterministic step
      accumulator -= dt
  alpha = accumulator / dt                 # leftover fraction
  render(alpha)                            # …and interpolate between them
```

Fixing `dt` matters here more than in most games: integrator stability and
restitution/energy behavior depend on a constant step, and a constant step is
what makes the demos *deterministic* — a prerequisite for the `--selftest`
harness and for reproducible bug reports. `App` is a small base class; each
chapter overrides `fixedUpdate(dt)` and `render(alpha)`.

## `physics/` — the engine (Part II onward)

Grown one part at a time; each chapter adds files rather than editing old ones
where possible. It depends only on the header-only math in `common/` — no SDL,
no rendering — so it links into `p3d_physics`, is unit-tested in isolation
(`tests/test_physics.cpp`), and could be reused headless.

### Part II — Particles & Forces (implemented)

| File | Responsibility | Introduced in |
|------|----------------|---------------|
| `physics/particle.hpp` | a point mass: position, velocity, **inverse mass** (0 = immovable), a force accumulator, and *both* integration paths | Ch 6 |
| `physics/forces.hpp/.cpp` | force generators that only *add* to a particle's accumulator: gravity, drag, spring/anchored spring (with damping), bungee | Ch 6–7 |
| `physics/constraint.hpp` | `DistanceConstraint` + `satisfyConstraints` — Jakobsen position projection | Ch 8–9 |

**Two simulation styles, deliberately side by side.** `Particle` supports both,
so the tutorial can contrast them directly:

- **Force-based** (Ch 6–7): callers sum forces into `forceAccum` via the
  generators, then `integrateForces()` runs semi-implicit Euler
  (`a = forceAccum · invMass`). This is the classic force→accumulate→integrate
  loop; forces superpose, so their order is irrelevant.
- **Position-based / Verlet** (Ch 8–9): `integrateVerlet()` advances position
  from its own history (no explicit velocity), then `satisfyConstraints()`
  *projects* particle positions to satisfy distance constraints, iterating a few
  passes to converge (relaxation). Inextensible by construction, so it cannot
  explode the way a stiff spring can. Pinning is just `invMass = 0`.

Cloth (Ch 9) is the payoff: a grid of Verlet particles laced with structural,
shear, and bend distance constraints, rendered by rebuilding a `Mesh` from the
particle positions each frame and drawing it **double-sided** (a flag has no
"inside") — the one rendering feature Part II added to `Renderer3D::drawMesh`.

### Part III — Rigid Bodies (implemented)

| File | Responsibility | Introduced in |
|------|----------------|---------------|
| `common/math/mat3.hpp` | 3×3 matrix (multiply, transpose, inverse) — the natural size for inertia tensors and rotation; `toMat3(quat)` lives in `quat.hpp` | Ch 11 |
| `physics/inertia.hpp` | inertia tensors for box/sphere/cylinder, plus `shift` (parallel-axis theorem) for composite bodies | Ch 11 |
| `physics/rigidbody.hpp` | linear + angular state, `applyForceAtPoint` (torque = r×F), and the integration step | Ch 11 |

**Key decision — store angular momentum `L`, not angular velocity `ω`.** Under
zero torque `L` is exactly conserved, so keeping it as the primary variable
makes free rotation robust; `ω = I⁻¹_world · L` is derived each step (with
`I⁻¹_world = R · I⁻¹_body · Rᵀ`).

**Key decision — exponential-map midpoint orientation update.** A naive explicit
step (`q += ½ ω⊗q · dt`) conserves `L` but *injects energy* — a tumbling body
spins ever faster instead of flipping (the Chapter 5 energy-leak, in rotational
form). `RigidBody::integrate` instead samples `ω` at the step midpoint and
rotates the orientation by the exact angle `|ω|·dt` (`RigidBody::spin`). Energy
then holds steady, which is exactly what lets the **Dzhanibekov / intermediate-
axis** effect appear (Ch 11 scene 1) — verified in `tests/test_rigidbody.cpp` by
checking that the intermediate-axis spin axis reverses while the extreme axes stay
put, all with bounded kinetic energy. Chapter 10 introduces the orientation math
itself (gimbal lock vs. quaternions, slerp) using only the existing `quat.hpp`.

### Part IV — Collision Detection (implemented)

Detection only — it finds contacts; making them push back is Part V. It is a
funnel from cheap-and-approximate to exact, all producing the same `Contact`.

| File | Responsibility | Introduced in |
|------|----------------|---------------|
| `physics/aabb.hpp` | axis-aligned bounding boxes + overlap; world AABB of a sphere / oriented box | Ch 12 |
| `physics/contact.hpp` | `Contact` = point + normal + penetration. **Normal points B → A** (the push-A-out direction) — one convention, fixed everywhere | Ch 13 |
| `physics/broadphase.hpp/.cpp` | candidate-pair finding: brute O(N²), sweep-and-prune, uniform grid (all return the same set — the tests check it) | Ch 12 |
| `physics/narrowphase.hpp/.cpp` | exact contacts for sphere-sphere / sphere-plane / sphere-box / box-plane | Ch 13 |
| `physics/sat.hpp/.cpp` | oriented box-box via the Separating-Axis Theorem (15 axes), with a face-clipping **manifold** (up to 4 points) | Ch 14 |
| `physics/gjk.hpp/.cpp` | GJK intersection + EPA penetration for **any** convex vertex set, via support functions on the Minkowski difference | Ch 15 |

**Design notes.**
- The pipeline is broadphase → narrowphase. A chapter builds AABBs, runs a
  broadphase to get candidate pairs, then runs the exact test on each.
- SAT gives a multi-point manifold (needed for stable stacking in Part V); GJK/EPA
  gives one contact but works for arbitrary convex shapes. They agree on the
  penetration depth for boxes (cross-checked in `tests/test_collision.cpp`).
- **EPA is winding-sensitive**: the starting tetrahedron's faces must be
  *consistently wound* and expansion must flip only the normal *vector*, never the
  vertex order, or the polytope diverges. See `LEARNINGS.md`.
- The Part IV demos do only crude positional separation to stay watchable; they do
  **not** implement restitution, friction, or rotation from contacts (that is Part V).

### Part V — Collision Response (implemented)

The climax: contacts (IV) acting on rigid bodies (III) via an impulse solver.

| File | Responsibility | Introduced in |
|------|----------------|---------------|
| `physics/collider.hpp` | a shape (sphere/box) + material (restitution, friction) bound to a body; builds its AABB and inertia tensor | Ch 16 |
| `physics/solver.hpp/.cpp` | the sequential-impulse solver: `ContactConstraint` (effective masses, tangent basis, bias), warm start, and one velocity pass (normal impulse clamped ≥0, friction clamped to the Coulomb cone) | Ch 16–17 |
| `physics/world.hpp/.cpp` | **the whole engine assembled** — bodies + colliders + the full `step()`: gravity → broadphase → narrowphase dispatch → warm-started solve → integrate positions → sleeping, with a body-pair contact cache for warm starting | Ch 16–18 |

`RigidBody` gained velocity-level operations for the solver: `applyImpulse`
(linear + `r×J` angular), `velocityAtPoint`, and `integratePosition` (advance
using solved velocities; the orientation update was factored into
`advanceOrientation`, shared with the Ch 11 force-driven `integrate`).

**Design notes.**
- `World::step` is the entire course in six stages. Bodies are parallel arrays
  (`rbs` / `colliders` / `awake` / `sleepTimer`) so the solver takes the rigid
  bodies directly.
- **Restitution vs. Baumgarte**: the normal bias is `max(restitution, baumgarte)`,
  not their sum — summing lets position correction inject energy and a ball
  rebounds higher than it fell (a bug we hit; see `LEARNINGS.md`).
- **Warm starting** (re-applying last frame's per-contact impulses, matched by
  body pair + point proximity) is what makes stacks stable at ~10 iterations.
- **Sleeping**: a body still for `timeToSleep` seconds is frozen and skipped;
  bodies in contact keep each other awake, so an "island" sleeps/wakes together.
  Basic sequential impulse drifts slowly on tall stacks without sleeping — which
  is exactly why sleeping (Ch 18) matters.

### Planned (later parts)

Joints (VI) → XPBD soft bodies (VII). Filled in as those parts land.

## `docs/` — the tutorial (no build step)

Plain static HTML/CSS/JS that runs from `file://` — open `docs/index.html`
directly. Conventions:

- **Math** is written in **MathML** (native to modern browsers) so there is no
  MathJax/KaTeX download and equations render offline.
- **Diagrams** are hand-authored inline **SVG**, themed with CSS variables so
  they adapt to light/dark mode. No binary image assets are committed.
- A few chapters embed small **vanilla-JS `<canvas>` widgets** (e.g. the
  integrator comparison in Ch 5) so the reader can *feel* a parameter, not just
  read about it.
- `docs/js/docs.js` provides the table of contents, prev/next navigation, the
  light/dark toggle, and a tiny C++ syntax highlighter (again, no dependency).

## Conventions worth knowing

- **Units**: SI — meters, kilograms, seconds. Gravity is `-9.81 m/s²` on `y`.
- **Handedness**: right-handed world space; the camera looks down its local
  `-z`, matching the perspective matrix in `mat4.hpp`.
- **Matrices** are **column-major** and stored so that `A * B` means "apply B,
  then A" (standard OpenGL-style convention). See the long comment in
  `mat4.hpp`.
- **Quaternions** are stored `(w, x, y, z)` with `w` the scalar part, to match
  the notation `q = w + xi + yj + zk` used in the docs.

## Testing & verification

- **Unit tests** (`tests/`, doctest + CTest) cover the math identities,
  integrator energy behavior, and later the geometry/solver routines.
- **`--selftest`** on every demo renders a few frames to an offscreen surface,
  writes a BMP, and exits `0` — a fast smoke test that the whole pipeline links
  and runs headlessly (used in CI and during development).
