# Prompt log

A running record of the prompts that shaped this project.

---

## 2026-07-11 — Project kickoff

> - i want to create a tutorial based project for learning 3d physics
> - the project should be in sdl3 and c++
> - the code should be well commented
> - the accompanying documentation should be in HTML without requiring build steps (as a 'docs' folder).
> - the tutorial documentation should go from beginner to advanced, should be in-depth and should have useful diagrams where necessary
> - also explain the math intuitively in the documentation

Clarifications gathered before planning:
- **Rendering:** software 3D pipeline on `SDL_RenderGeometry` (no extra deps;
  the 3D math becomes tutorial content).
- **Curriculum depth:** "everything + extras" — through joints/constraints,
  plus GJK/EPA and XPBD soft bodies (20 chapters, 7 parts).
- **Delivery:** checkpoint per part — build the scaffold + Part I first for a
  style/tone review, then continue part by part.

> implement the plan.

Began implementation with Milestone 1: project scaffold + `common/` framework
(math, software renderer, app loop) + Part I chapters (1–5) with code, unit
tests, and the HTML docs (index + 5 chapter pages).
