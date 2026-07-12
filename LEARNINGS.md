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
