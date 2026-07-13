/* ===========================================================================
   docs.js — the shared chrome for every tutorial page. No dependencies.
   Responsibilities:
     * build the sidebar navigation + prev/next pager from one manifest,
     * the light/dark theme toggle (remembered in localStorage),
     * a tiny C++ syntax highlighter (so we ship no highlighting library),
     * boot any interactive <canvas> widgets on the page.
   =========================================================================== */
(function () {
  "use strict";

  // --- The single source of truth for chapter order/titles ------------------
  const CHAPTERS = [
    { part: "Part I · Foundations", num: "01", slug: "01-hello-sdl3",   title: "Hello, SDL3" },
    { part: "Part I · Foundations", num: "02", slug: "02-vectors",      title: "Vectors" },
    { part: "Part I · Foundations", num: "03", slug: "03-matrices",     title: "Matrices & Transforms" },
    { part: "Part I · Foundations", num: "04", slug: "04-camera",       title: "Camera & Projection" },
    { part: "Part I · Foundations", num: "05", slug: "05-integration",  title: "Numerical Integration" },
    { part: "Part II · Particles & Forces", num: "06", slug: "06-particles", title: "Particles & Forces" },
    { part: "Part II · Particles & Forces", num: "07", slug: "07-springs",   title: "Springs & Dampers" },
    { part: "Part II · Particles & Forces", num: "08", slug: "08-ropes",     title: "Ropes & Constraints" },
    { part: "Part II · Particles & Forces", num: "09", slug: "09-cloth",     title: "Cloth" },
    { part: "Part III · Rigid Bodies", num: "10", slug: "10-quaternions", title: "Rotation & Quaternions" },
    { part: "Part III · Rigid Bodies", num: "11", slug: "11-rigidbody",   title: "Rigid-Body Dynamics" },
    { part: "Part IV · Collision Detection", num: "12", slug: "12-broadphase",  title: "Broadphase" },
    { part: "Part IV · Collision Detection", num: "13", slug: "13-narrowphase", title: "Narrowphase: Spheres & Planes" },
    { part: "Part IV · Collision Detection", num: "14", slug: "14-sat",         title: "SAT & Manifolds" },
    { part: "Part IV · Collision Detection", num: "15", slug: "15-gjk",         title: "GJK & EPA" },
  ];

  // Later parts, shown greyed-out on the roadmap so the arc is visible.
  const UPCOMING = [
    "Part V · Collision Response",
    "Part VI · Constraints & Joints", "Part VII · Beyond Rigid Bodies",
  ];

  const inChapters = location.pathname.indexOf("/chapters/") !== -1;
  const toChapter = (slug) => (inChapters ? slug + ".html" : "chapters/" + slug + ".html");
  const toHome = inChapters ? "../index.html" : "index.html";
  const asset = (p) => (inChapters ? "../" + p : p);

  // --- Theme -----------------------------------------------------------------
  function applyStoredTheme() {
    const t = localStorage.getItem("theme");
    if (t === "light" || t === "dark") document.documentElement.dataset.theme = t;
  }
  function toggleTheme() {
    const cur = document.documentElement.dataset.theme;
    // If unset, infer what we're currently showing from the OS preference.
    const showingDark = cur ? cur === "dark"
      : window.matchMedia("(prefers-color-scheme: dark)").matches;
    const next = showingDark ? "light" : "dark";
    document.documentElement.dataset.theme = next;
    localStorage.setItem("theme", next);
    document.dispatchEvent(new CustomEvent("themechange"));
  }
  applyStoredTheme();

  // --- Build the top bar -----------------------------------------------------
  function buildTopbar() {
    const bar = document.createElement("header");
    bar.className = "topbar";
    bar.innerHTML =
      '<button class="menu-btn" aria-label="Menu">☰</button>' +
      '<a class="brand" href="' + toHome + '"><span class="dot"></span>3D Physics From Scratch</a>' +
      '<span class="spacer"></span>' +
      '<a class="plain" href="' + toHome + '">Contents</a>' +
      '<button class="theme-toggle" aria-label="Toggle theme">◐</button>';
    document.body.prepend(bar);
    bar.querySelector(".theme-toggle").addEventListener("click", toggleTheme);
    bar.querySelector(".menu-btn").addEventListener("click", () => {
      const sb = document.querySelector(".sidebar");
      if (sb) sb.classList.toggle("open");
    });
  }

  // --- Build the sidebar -----------------------------------------------------
  function buildSidebar(currentNum) {
    const nav = document.createElement("nav");
    nav.className = "sidebar";
    let html = "";
    let lastPart = "";
    CHAPTERS.forEach((c) => {
      if (c.part !== lastPart) { html += "<h4>" + c.part + "</h4>"; lastPart = c.part; }
      const cls = c.num === currentNum ? "current" : "";
      html += '<a class="' + cls + '" href="' + toChapter(c.slug) + '">' +
              '<span class="ch-num">' + c.num + "</span>" + c.title + "</a>";
    });
    nav.innerHTML = html;
    return nav;
  }

  // --- Build prev/next pager -------------------------------------------------
  function buildPager(currentNum) {
    const idx = CHAPTERS.findIndex((c) => c.num === currentNum);
    if (idx < 0) return null;
    const prev = CHAPTERS[idx - 1], next = CHAPTERS[idx + 1];
    const pager = document.createElement("nav");
    pager.className = "pager";
    const prevHtml = prev
      ? '<a class="prev" href="' + toChapter(prev.slug) + '"><div class="dir">← Previous</div><div class="ttl">' + prev.num + " · " + prev.title + "</div></a>"
      : '<a class="prev disabled"><div class="dir">← Previous</div><div class="ttl">Start of course</div></a>';
    const nextHtml = next
      ? '<a class="next" href="' + toChapter(next.slug) + '"><div class="dir">Next →</div><div class="ttl">' + next.num + " · " + next.title + "</div></a>"
      : '<a class="next disabled"><div class="dir">Next →</div><div class="ttl">More coming soon</div></a>';
    pager.innerHTML = prevHtml + nextHtml;
    return pager;
  }

  // --- C++ syntax highlighter ------------------------------------------------
  const KW = new Set(("alignas alignof and auto bool break case catch char class const constexpr " +
    "continue default delete do double else enum explicit extern false float for friend goto if " +
    "inline int long mutable namespace new noexcept not nullptr operator or private protected public " +
    "register return short signed sizeof static static_cast struct switch template this throw true try " +
    "typedef typename union unsigned using virtual void volatile while override final").split(" "));
  const TYPES = new Set(("Real Vec3 Vec4 Mat4 Quat Color Mesh State App Renderer3D OrbitCamera AccelFn " +
    "SDL_Renderer SDL_Window SDL_Event size_t Uint8 Uint64 std string vector deque array function").split(" "));

  function esc(s) { return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;"); }

  function highlightCpp(src) {
    // Sticky-regex scanner: try each rule at the current position, in order.
    const rules = [
      ["cmt", /\/\/[^\n]*|\/\*[\s\S]*?\*\//y],
      ["pre", /^[ \t]*#\w+/my],
      ["str", /"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'/y],
      ["num", /\b(?:0x[0-9a-fA-F]+|\d[\d.']*)[fFuUlL]*\b/y],
      ["id",  /[A-Za-z_]\w*/y],
      ["any", /\s+|[^\sA-Za-z_0-9]/y],
    ];
    let out = "", i = 0;
    while (i < src.length) {
      let matched = false;
      for (const [kind, re] of rules) {
        re.lastIndex = i;
        const m = re.exec(src);
        if (!m || m.index !== i) continue;
        const text = m[0];
        if (kind === "id") {
          if (KW.has(text)) out += '<span class="tok-kw">' + text + "</span>";
          else if (TYPES.has(text)) out += '<span class="tok-typ">' + text + "</span>";
          else out += esc(text);
        } else if (kind === "any") {
          out += esc(text);
        } else {
          out += '<span class="tok-' + kind + '">' + esc(text) + "</span>";
        }
        i += text.length;
        matched = true;
        break;
      }
      if (!matched) { out += esc(src[i]); i++; }
    }
    return out;
  }

  function highlightAll() {
    document.querySelectorAll("pre > code.cpp").forEach((code) => {
      code.innerHTML = highlightCpp(code.textContent);
    });
  }

  // --- Interactive widget: integrator comparison -----------------------------
  // Mirrors Chapter 5's orbit demo, in the browser: a 2D harmonic oscillator
  // a = -k x integrated by three methods, so you can watch how the timestep
  // controls stability and energy drift.
  function initIntegratorWidget(root) {
    const canvas = root.querySelector("canvas");
    const ctx = canvas.getContext("2d");
    const dtSlider = root.querySelector(".dt");
    const dtOut = root.querySelector(".dt-val");
    const energyOut = root.querySelector(".energy");
    const resetBtn = root.querySelector(".reset");
    const K = 6.0;

    const methods = [
      { name: "Explicit Euler", color: "#f0645f", step: (s, dt) => { const a = -K, ax = a * s.x, ay = a * s.y;
          return { x: s.x + s.vx * dt, y: s.y + s.vy * dt, vx: s.vx + ax * dt, vy: s.vy + ay * dt }; } },
      { name: "Semi-implicit",  color: "#29c8be", step: (s, dt) => { const vx = s.vx - K * s.x * dt, vy = s.vy - K * s.y * dt;
          return { x: s.x + vx * dt, y: s.y + vy * dt, vx, vy }; } },
      { name: "RK4",            color: "#96d25a", step: (s, dt) => rk4(s, dt) },
    ];
    function accel(x) { return -K * x; }
    function rk4(s, dt) {
      const k1 = { x: s.vx, y: s.vy, vx: accel(s.x), vy: accel(s.y) };
      const s2 = { x: s.x + k1.x * dt / 2, y: s.y + k1.y * dt / 2, vx: s.vx + k1.vx * dt / 2, vy: s.vy + k1.vy * dt / 2 };
      const k2 = { x: s2.vx, y: s2.vy, vx: accel(s2.x), vy: accel(s2.y) };
      const s3 = { x: s.x + k2.x * dt / 2, y: s.y + k2.y * dt / 2, vx: s.vx + k2.vx * dt / 2, vy: s.vy + k2.vy * dt / 2 };
      const k3 = { x: s3.vx, y: s3.vy, vx: accel(s3.x), vy: accel(s3.y) };
      const s4 = { x: s.x + k3.x * dt, y: s.y + k3.y * dt, vx: s.vx + k3.vx * dt, vy: s.vy + k3.vy * dt };
      const k4 = { x: s4.vx, y: s4.vy, vx: accel(s4.x), vy: accel(s4.y) };
      return {
        x: s.x + (k1.x + 2 * k2.x + 2 * k3.x + k4.x) * dt / 6,
        y: s.y + (k1.y + 2 * k2.y + 2 * k3.y + k4.y) * dt / 6,
        vx: s.vx + (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx) * dt / 6,
        vy: s.vy + (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy) * dt / 6,
      };
    }

    let states, trails;
    function reset() {
      const r = 1.4, omega = Math.sqrt(K);
      states = methods.map(() => ({ x: r, y: 0, vx: 0, vy: omega * r }));
      trails = methods.map(() => []);
    }
    reset();
    resetBtn.addEventListener("click", reset);

    function cssVar(name) { return getComputedStyle(document.body).getPropertyValue(name).trim(); }

    function frame() {
      const dt = parseFloat(dtSlider.value);
      dtOut.textContent = dt.toFixed(3);

      // Fit the drawing buffer to the element's CSS size (crisp on HiDPI).
      const cssW = canvas.clientWidth, cssH = 300;
      const dpr = window.devicePixelRatio || 1;
      if (canvas.width !== cssW * dpr || canvas.height !== cssH * dpr) {
        canvas.width = cssW * dpr; canvas.height = cssH * dpr; canvas.style.height = cssH + "px";
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, cssW, cssH);

      const cx = cssW / 2, cy = cssH / 2, scale = Math.min(cssW, cssH) / 5;
      const P = (x, y) => [cx + x * scale, cy - y * scale];

      // Axes.
      ctx.strokeStyle = cssVar("--border"); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(cssW, cy);
      ctx.moveTo(cx, 0); ctx.lineTo(cx, cssH); ctx.stroke();

      // Step + draw each integrator.
      let energyText = "";
      methods.forEach((m, i) => {
        states[i] = m.step(states[i], dt);
        const s = states[i];
        trails[i].push([s.x, s.y]);
        if (trails[i].length > 400) trails[i].shift();
        // Clamp runaway (explicit Euler can diverge to infinity).
        if (!isFinite(s.x) || Math.abs(s.x) > 12) { const r = 1.4, omega = Math.sqrt(K);
          states[i] = { x: r, y: 0, vx: 0, vy: omega * r }; trails[i] = []; }

        ctx.strokeStyle = m.color; ctx.globalAlpha = 0.55; ctx.lineWidth = 1.5;
        ctx.beginPath();
        trails[i].forEach((pt, j) => { const [px, py] = P(pt[0], pt[1]); j ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
        ctx.stroke();
        ctx.globalAlpha = 1;
        const [dx, dy] = P(s.x, s.y);
        ctx.fillStyle = m.color; ctx.beginPath(); ctx.arc(dx, dy, 4, 0, 7); ctx.fill();

        const E = 0.5 * (s.vx * s.vx + s.vy * s.vy) + 0.5 * K * (s.x * s.x + s.y * s.y);
        energyText += m.name + " E=" + (isFinite(E) ? E.toFixed(2) : "∞") + "   ";
      });
      energyOut.textContent = energyText;
      requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  }

  // --- Interactive widget: damping ------------------------------------------
  // Plots the settling curve of a mass-spring-damper as you drag the damping
  // ratio zeta. zeta < 1 overshoots and rings; zeta = 1 (critical) returns
  // fastest without overshoot; zeta > 1 is sluggish. Mirrors Chapter 7 scene 2.
  function initDampingWidget(root) {
    const canvas = root.querySelector("canvas");
    const ctx = canvas.getContext("2d");
    const slider = root.querySelector(".zeta");
    const zetaOut = root.querySelector(".zeta-val");
    const regimeOut = root.querySelector(".regime");
    const w0 = 7.0;           // natural frequency
    const T = 3.0;            // seconds of the plot

    function cssVar(n) { return getComputedStyle(document.body).getPropertyValue(n).trim(); }

    // Displacement over time for x(0)=1, x'(0)=0, given a damping ratio z.
    function displacement(z, t) {
      if (Math.abs(z - 1) < 1e-3) {                 // critically damped
        return Math.exp(-w0 * t) * (1 + w0 * t);
      } else if (z < 1) {                           // under-damped: rings
        const wd = w0 * Math.sqrt(1 - z * z);
        return Math.exp(-z * w0 * t) * (Math.cos(wd * t) + (z * w0 / wd) * Math.sin(wd * t));
      } else {                                      // over-damped: crawls back
        const wd = w0 * Math.sqrt(z * z - 1);
        return Math.exp(-z * w0 * t) * (Math.cosh(wd * t) + (z * w0 / wd) * Math.sinh(wd * t));
      }
    }

    function curveColor(z) {
      if (z < 0.98) return "#f0645f";               // under  → coral
      if (z <= 1.02) return "#29c8be";              // critical → teal
      return "#a578eb";                             // over   → violet
    }
    function regimeName(z) {
      if (z < 0.98) return "under-damped (overshoots and rings)";
      if (z <= 1.02) return "critically damped (fastest, no overshoot)";
      return "over-damped (slow, sluggish)";
    }

    function draw() {
      const z = parseFloat(slider.value);
      zetaOut.textContent = z.toFixed(2);
      regimeOut.textContent = regimeName(z);
      regimeOut.style.color = curveColor(z);

      const cssW = canvas.clientWidth || 680, cssH = 260;
      const dpr = window.devicePixelRatio || 1;
      if (canvas.width !== cssW * dpr || canvas.height !== cssH * dpr) {
        canvas.width = cssW * dpr; canvas.height = cssH * dpr; canvas.style.height = cssH + "px";
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, cssW, cssH);

      const padL = 34, padR = 12, padT = 14, padB = 22;
      const plotW = cssW - padL - padR, plotH = cssH - padT - padB;
      const yMax = 1.35, yMin = -0.7;
      const X = (t) => padL + (t / T) * plotW;
      const Y = (v) => padT + (1 - (v - yMin) / (yMax - yMin)) * plotH;

      // Zero (target) line and the starting-displacement line.
      ctx.strokeStyle = cssVar("--border"); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(padL, Y(0)); ctx.lineTo(padL + plotW, Y(0)); ctx.stroke();
      ctx.fillStyle = cssVar("--muted"); ctx.font = "11px system-ui";
      ctx.fillText("target (0)", padL + 2, Y(0) - 4);
      ctx.fillText("start (1)", padL + 2, Y(1) - 4);
      ctx.strokeStyle = cssVar("--border"); ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(padL, Y(1)); ctx.lineTo(padL + plotW, Y(1)); ctx.stroke();
      ctx.setLineDash([]);

      // Faint reference curves for the three canonical regimes.
      [[0.25, "#f0645f"], [1.0, "#29c8be"], [2.0, "#a578eb"]].forEach(([z0, col]) => {
        ctx.strokeStyle = col; ctx.globalAlpha = 0.18; ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let i = 0; i <= 240; i++) { const t = (i / 240) * T; const p = [X(t), Y(displacement(z0, t))]; i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); }
        ctx.stroke();
      });
      ctx.globalAlpha = 1;

      // The live curve for the chosen zeta.
      ctx.strokeStyle = curveColor(z); ctx.lineWidth = 2.5;
      ctx.beginPath();
      for (let i = 0; i <= 320; i++) { const t = (i / 320) * T; const p = [X(t), Y(displacement(z, t))]; i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); }
      ctx.stroke();
      ctx.fillStyle = cssVar("--muted"); ctx.fillText("time →", padL + plotW - 46, padT + plotH + 16);
    }

    slider.addEventListener("input", draw);
    window.addEventListener("resize", draw);
    draw();
  }

  // --- Interactive widget: SAT projection -----------------------------------
  // Two 2D shapes and a test axis you can spin. It draws each shape's "shadow"
  // (projection) on the axis; if you find one axis where the shadows don't
  // overlap, the shapes are apart — the Separating-Axis Theorem in miniature.
  function initSatWidget(root) {
    const canvas = root.querySelector("canvas");
    const ctx = canvas.getContext("2d");
    const sep = root.querySelector(".sep");
    const ang = root.querySelector(".angle");
    const out = root.querySelector(".satresult");
    function cssVar(n) { return getComputedStyle(document.body).getPropertyValue(n).trim(); }

    function draw() {
      const W = canvas.clientWidth || 680, H = 280;
      const dpr = window.devicePixelRatio || 1;
      if (canvas.width !== W * dpr || canvas.height !== H * dpr) {
        canvas.width = W * dpr; canvas.height = H * dpr; canvas.style.height = H + "px";
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);
      const cx = W / 2, cy = H / 2 - 6;

      const s = parseFloat(sep.value);
      const th = parseFloat(ang.value) * Math.PI / 180;
      const ax = [Math.cos(th), Math.sin(th)], pr = [-ax[1], ax[0]];

      const rect = (ox, oy, hw, hh) => [[ox-hw,oy-hh],[ox+hw,oy-hh],[ox+hw,oy+hh],[ox-hw,oy+hh]];
      const A = rect(cx - 55, cy, 52, 40);
      const B = rect(cx - 55 + s, cy, 42, 54);

      const drawPoly = (p, col) => {
        ctx.strokeStyle = col; ctx.fillStyle = col + "22"; ctx.lineWidth = 2;
        ctx.beginPath(); p.forEach((v, i) => i ? ctx.lineTo(v[0], v[1]) : ctx.moveTo(v[0], v[1]));
        ctx.closePath(); ctx.fill(); ctx.stroke();
      };
      // The test axis, drawn across the canvas through the centre.
      ctx.strokeStyle = cssVar("--border"); ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - ax[0]*400, cy - ax[1]*400); ctx.lineTo(cx + ax[0]*400, cy + ax[1]*400); ctx.stroke();

      drawPoly(A, "#29c8be"); drawPoly(B, "#f0a84a");

      const project = (p) => { let mn = 1e9, mx = -1e9;
        for (const v of p) { const d = (v[0]-cx)*ax[0] + (v[1]-cy)*ax[1]; mn = Math.min(mn,d); mx = Math.max(mx,d); }
        return [mn, mx]; };
      const pa = project(A), pb = project(B);

      // Draw each shadow as a thick segment offset to one side of the axis.
      const bar = (iv, col, off) => {
        ctx.strokeStyle = col; ctx.lineWidth = 5;
        const a = [cx + ax[0]*iv[0] + pr[0]*off, cy + ax[1]*iv[0] + pr[1]*off];
        const b = [cx + ax[0]*iv[1] + pr[0]*off, cy + ax[1]*iv[1] + pr[1]*off];
        ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();
      };
      bar(pa, "#29c8be", 92); bar(pb, "#f0a84a", 108);

      const overlap = pa[0] <= pb[1] && pb[0] <= pa[1];
      out.textContent = overlap ? "shadows OVERLAP on this axis — keep testing other axes"
                                : "GAP on this axis → a separating axis! The shapes are APART.";
      out.style.color = overlap ? "#f0645f" : "#7bd38a";
    }
    sep.addEventListener("input", draw);
    ang.addEventListener("input", draw);
    window.addEventListener("resize", draw);
    draw();
  }

  // --- Wire everything up ----------------------------------------------------
  document.addEventListener("DOMContentLoaded", function () {
    buildTopbar();
    const currentNum = document.body.getAttribute("data-chapter");

    // Only chapter/content pages get the sidebar+pager layout wrapper.
    const article = document.querySelector("article");
    if (article && !document.querySelector(".layout")) {
      const layout = document.createElement("div");
      layout.className = "layout";
      const content = document.createElement("div");
      content.className = "content";
      article.parentNode.insertBefore(layout, article);
      layout.appendChild(buildSidebar(currentNum || ""));
      content.appendChild(article);
      layout.appendChild(content);

      if (currentNum) {
        const pager = buildPager(currentNum);
        if (pager) article.appendChild(pager);
      }
    }

    highlightAll();
    document.querySelectorAll(".js-integrator-widget").forEach(initIntegratorWidget);
    document.querySelectorAll(".js-damping-widget").forEach(initDampingWidget);
    document.querySelectorAll(".js-sat-widget").forEach(initSatWidget);
  });

  // Expose the upcoming-parts list for the home page to render.
  window.__P3D_UPCOMING = UPCOMING;
})();
