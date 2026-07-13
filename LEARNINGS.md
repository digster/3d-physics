# Learnings

Hard-won, project-specific gotchas. Read before you trip over the same wire.

## Build / toolchain

- **Clang + default `= {}` argument with an NSDMI type.** A constructor like
  `explicit App(Config cfg = {})` fails to compile when `Config` has a member
  with a default initializer that is a non-trivial type (e.g. `std::string title
  = "..."`) *and* the default arg is used inside the enclosing class. Fix: give a
  separate default constructor that delegates — `App() : App(Config{}) {}` — as in
  `common/app.hpp`.

- **doctest under CMake 4.x.** doctest's bundled `CMakeLists.txt` declares a very
  old `cmake_minimum_required`, which CMake ≥ 4 refuses. We set
  `CMAKE_POLICY_VERSION_MINIMUM=3.5` before `FetchContent_MakeAvailable(doctest)`
  in the root `CMakeLists.txt`. Revisit if we bump the doctest pin.

- **SDL3 version vs. brew revision.** Homebrew’s `sdl3` "3.4.8" is the true
  upstream version (the `SDL_MICRO_VERSION` macro agrees), so the FetchContent
  fallback pins the real git tag `release-3.4.8`. Don’t assume the brew string is
  a formula revision.

## Physics

- **Free rotation leaks energy with a naive quaternion step.** Integrating
  orientation as `q += ½ ω⊗q · dt; normalize` conserves angular momentum `L`
  (we store it directly) but steadily *adds kinetic energy* — a torque-free spin
  accelerates without bound instead of tumbling. It's the Chapter-5 explicit-Euler
  energy leak wearing a rotational costume. Fix: the exponential-map **midpoint**
  step in `RigidBody::integrate` — sample `ω` at the step midpoint and rotate by
  the exact angle `|ω|·dt`. After that, energy is bounded and free precession
  (the Dzhanibekov effect) is correct.

- **Test the right signal for the Dzhanibekov flip.** The intermediate-axis flip
  does *not* show up as the world-frame `ω` component along the spin axis changing
  sign — that stays roughly constant. The flip is the **body-fixed spin axis
  reversing in world space** (equivalently, the body-frame `ω` reversing). Track a
  body-fixed axis via `rotate(orientation, axis)` and watch its dot with the
  initial direction go negative. An early test asserted on world-frame `ω_y` and
  wrongly "failed" a correct simulation.

- **`inertia::box` takes half-extents** (to match `mesh::box`), so the moment
  formula collapses to `m/3·(h_i² + h_j²)`, not the textbook `m/12·(s_i² + s_j²)`
  with full side lengths `s = 2h`. Easy to be off by 4× if you mix them.

- **EPA diverges if the polytope winding is inconsistent.** Two bugs, one symptom
  (hundreds of faces, never converging): (1) the *initial tetrahedron* face set
  must be consistently wound — every shared edge must appear in opposite
  directions on its two faces. The set `{0,1,2},{0,3,1},{0,2,3},{1,3,2}` works;
  `{0,1,2},{0,1,3},{0,2,3},{1,2,3}` does NOT (edge 0-1 is `0→1` on two faces). (2)
  When orienting a face's normal outward, flip only the **normal vector**, never
  `std::swap` the vertex indices — swapping breaks the winding the horizon-edge
  cancellation relies on. See `physics/gjk.cpp::epa`.

- **GJK's terminating simplex can be degenerate for boxes.** Flat faces make the
  support function return the same vertex for several directions, so the 4 GJK
  points may be coincident/coplanar — not a valid EPA start. Fix: "blow up" the
  simplex to a real tetrahedron with fresh support points before EPA
  (`buildTetrahedron`), and reject if the volume is still ~0.

- **EPA's outward normal points A→B, not B→A.** With the Minkowski difference
  built as `support_A(d) − support_B(−d)` (i.e. A⊖B), EPA's outward face normal is
  the direction to push *B*; negate it to match the engine's B→A `Contact`
  convention. Verified against SAT on the same box pose.

- **The Dzhanibekov flip is a body-frame signal** (repeat of the Part III lesson,
  relevant when testing collision demos too): don't assert on world-frame ω.

## Rendering (software pipeline)

- **Winding drives everything.** Mesh triangles are wound counter-clockwise as
  seen from outside (`common/render/mesh.cpp`). Both the flat-shading normal
  (`cross(w1-w0, w2-w0)` points outward) and the view-space backface test depend
  on it. If a new primitive renders "inside-out" or unlit, check its winding first.

- **Backface cull test is view-space, not screen-space.** We cull when
  `dot(viewNormal, centroid) >= 0`. Doing it in view space (before the y-flip of
  the viewport transform) avoids sign confusion from the screen mapping.

- **Painter’s algorithm limits.** Interpenetrating triangles can sort wrong for a
  frame — accepted on purpose (penetration is what the solver fixes). If you see
  z-fighting flicker, it’s the sort, not a bug; `W` wireframe helps read it.

## Docs / browser verification

- **The in-app automated browser reports `document.hidden = true`,** which pauses
  `requestAnimationFrame`. Canvas widgets (e.g. the Ch5 integrator) therefore
  won’t animate in the pane. They’re fine for real users; to verify in-pane,
  monkey-patch `requestAnimationFrame` to `setTimeout(cb, 16)` before init.

- **Screenshot compositor can blank after a *programmatic* scroll** in that pane.
  Scrolling back to `(0,0)` (or using the tool’s own scroll) repaints. Verify page
  structure with JS/DOM queries when a screenshot looks empty but the DOM is fine.

- **A single stray quote breaks the whole `docs.js`.** It’s one IIFE, so one
  syntax error blanks every page (no chrome, no highlighting). `node --check
  docs/js/docs.js` catches it instantly — run it after editing.
