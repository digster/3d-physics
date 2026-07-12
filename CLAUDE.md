# Project brief & working guidelines

Instructions for any Claude session working on this repository. Read this first,
then `ARCHITECTURE.md` for structure and `README.md` for how to build/run.

## What this project is (from the original brief)

- A **tutorial-based project for learning 3D physics**.
- Built in **SDL3 and C++**.
- The **code must be well commented**.
- The accompanying **documentation is HTML that requires no build step** (it
  lives in the `docs/` folder and opens directly in a browser).
- The **tutorial documentation goes from beginner to advanced**, is **in-depth**,
  and has **useful diagrams where necessary**.
- The documentation **explains the math intuitively**.

## Audience & documentation depth (important)

**Assume a genuine beginner** — someone who can program, but is new to 3D math and
physics. When in doubt, **err toward more hand-holding**, not less. Specifically:

- **Always be in-depth.** Prefer a longer, clearer explanation over a terse one.
  It is fine — encouraged — for a page to be long if the extra length is teaching.
- **Never assume prior math.** The reader may not remember trig, linear algebra,
  or calculus. Reintroduce what you use: say what a term means the first time it
  appears (radians, dot/cross product, matrix, derivative, etc.).
- **Gloss every piece of jargon inline.** The first time a word like
  *homogeneous coordinates*, *symplectic*, *Lambert term*, *normalize*, *NDC*, or
  *stiff* appears, add a short plain-language "in other words…" right there.
- **Build intuition before formalism.** Lead with a concrete picture or analogy,
  then show the math, then the code. The recurring page shape is:
  *Goal → Intuition (with a diagram) → The Math → Implementation → Run it →
  Experiments → Takeaways.*
- **Show the steps.** Don't collapse a derivation to its result; walk through it.
  Worked numeric examples ("a point twice as far draws at half the size") land
  better than symbols alone for this audience.
- **Diagrams are first-class.** Reach for an inline SVG or a small interactive
  widget whenever a picture would explain faster than a paragraph.
- **Tone:** warm, encouraging, plainspoken. Second person ("you"). Short
  sentences. It's okay to acknowledge that something is tricky and slow down.

If you add or revise a chapter, hold it to this bar. If an existing page is terser
than this, improve it.

## Math rendering & assets

- Equations use **native MathML** (no MathJax/KaTeX) so they render offline with
  no dependency. Keep it that way.
- Diagrams are **hand-authored inline SVG**, themed via CSS variables so they work
  in light and dark mode. **No binary image assets** are committed.
- Small interactive explainers are **vanilla-JS `<canvas>` widgets** (see the Ch5
  integrator widget and `docs/js/docs.js`). No frameworks.

## Code conventions

- Units are SI (meters, kilograms, seconds); gravity is `-9.81` on `y`.
- World space is right-handed; the camera looks down its local `-z`.
- Matrices are column-major; quaternions are stored `(w, x, y, z)`.
- Every `common/` and `physics/` file carries a header comment noting the chapter
  that introduces it, so the codebase can be read in tutorial order.
- Comment **generously** — these files double as teaching material.

## Delivery

- The curriculum is delivered **part by part** (see the roadmap in
  `docs/index.html`). After each part: update `README`, `ARCHITECTURE`, the
  `memory/YYYY-MM-DD.md` summary, `PROMPT.md`, and `LEARNINGS.md` as needed, and
  provide a commit message (do not commit automatically; never add Claude as an
  author or co-author).
