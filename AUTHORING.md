# Authoring guide (for every module and track page)

Read `docs/m02-rigid-body-motions.md` fully before writing — it is the gold standard. Match its tone, depth and block order exactly. Also skim `docs/symbols.md` and `docs/connections.md`.

## Reader and voice
- Reader: Zaki (mechanical-engineering PhD, SciML/FEM, 3D vision NeRF/3DGS/SfM, genomic prediction GBLUP, ag phenotyping) and his students.
- 10th-grade reading level, technical terms kept, compact, no fluff. Math, numbers and visuals emphasized.
- Exactly ONE natural everyday analogy per page that scientists commonly use, carried as ONE story thread through the page (like "corkscrew" in M2, "latitude/longitude" in M1). Not forced.
- Follow Lynch & Park *Modern Robotics* notation (space frame {s}, body frame {b}, screw axes S/B, twists V, wrenches F, PoE, J_s/J_b). Use symbols already in `docs/symbols.md`; list any new symbols you introduce in your fragment file.

## Required blocks, in this order (headings as shown)
1. Title `# Mx · Name`, then tag line: `<span class="md-tag">Lynch & Park Ch N</span> <span class="md-tag">Used in: ...</span> <span class="md-tag">Tracks: ...</span>` (tracks pages: tag with hardware instead of chapter).
2. `> **Hook:** ...` one sentence to say out loud.
3. `## Intuition first` — the analogy and the story thread.
4. `## Problem → math basic → move → result` — 4 numbered lines.
5. Main teaching parts (`## Part 1 — ...` etc.) with step-by-step derivations. Each key step gets an italic *Why this move:*. Include at least one `??? question "What if we didn't ..."` or `??? warning` collapsible.
6. At least ONE live widget: `<div class="lab-widget" data-widget="NAME"></div>`, placed right after the idea it shows, followed by 1–3 lines "Try: ..." telling the reader what to drag and what to notice. Widgets should REUSE visual ideas from earlier modules/labs when natural (2-link/3-link planar arm from M1, frames/boxes from M2, screw pole, sliders → live matrix readouts, ellipses, log–log error plots, convergence curves) so concepts reinforce.
7. `## Math bridges` — table with columns `| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |`. Real, correct connections (FEM stiffness/mass matrices, weak forms, adjoints, time stepping, conservation; ridge/GBLUP/BLUP/kinship/least squares; SfM/BA/NeRF/3DGS/VGGT/camera poses). Use "—" where none.
8. `## In your robots` — table with Zaki's hardware: SO-ARM101 (5 revolute arm joints + gripper, LeRobot), Crazyflie 2.x with Flow deck + AI deck, planned custom under-canopy drone, Jetson Orin Nano rover (RGB-only, maize/soybean rows), ComXim motorized turntable 3D scanning station, Maize/Soybean Plot Builder synthetic scenes. Only claims that are true in general; don't invent specs (no exact link lengths unless labelled "example values").
9. `## Current research` — 3–5 bullets. MUST be grounded: run WebSearch (mode "standard"; several queries) and link real papers/pages you actually found. Well-known classics may be named without link (e.g., "Diffusion Policy, Chi et al., 2023"). Never invent titles, authors, numbers or URLs. Prefer 2024–2026 work. Agriculture-robotics angle where it exists.
10. `## Challenges` — 3–5 numbered, concrete (numbers where possible).
11. `## Next five years` — 3–5 bullets, include one agriculture/phenotyping bullet.
12. `## Check yourself` — exactly 3 quizzes, raw HTML, NO LaTeX inside (use unicode like θ, ω, ᵀ):
```html
<div class="quiz" data-q="m03-q1" data-answer="b" data-why="One-sentence reason.">
<p>Question?</p>
<ol><li data-k="a">…</li><li data-k="b">…</li><li data-k="c">…</li><li data-k="d">…</li></ol>
</div>
```
Vary the correct letter. Then `<div class="lab-progress" data-lab-progress></div>`.
13. `<div class="oneliner">One line: ...</div>`
14. `**Carries over to:** ...` links to other modules by filename.

## Math formatting rules (these broke before — follow exactly)
- Inline math `\( ... \)`; display math `\[ ... \]` on its own lines WITH A BLANK LINE BEFORE AND AFTER. Display math may sit inside admonitions (indent 4 spaces, still blank lines).
- Never put a literal `|` inside math in a Markdown table (it splits the cell). Use `\vert`, `\lVert x \rVert`.
- Macros available: `\SO`, `\SE`, `\so`, `\se`, `\Ad`, `\R`. Otherwise plain LaTeX.
- No math inside raw-HTML blocks (quizzes, widget readouts) — use unicode there.
- Mermaid allowed in ```mermaid fences.

## Widgets
- Put ALL your JS in your own file `docs/javascripts/widgets/<id>.js` (ids: m03 … m12, tb, tc, td, te). The file is already registered in mkdocs.yml. Do NOT edit `lab.js`, `lab.css`, `mkdocs.yml` or other people's files.
- Pattern:
```js
(function () {
  "use strict";
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg, matmul, rodrigues, drawFrame, drawBox, matrixHTML, project } = L;
  L.register("fk3r", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, 360, 300);            // {c, ctx, w, h}; ctx already scaled ×2
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const t1 = slider(ui, "θ₁", -180, 180, 1, 30, draw, "°");   // returns <input>
    function draw() { const k = colors(); /* k.fg k.light k.bg k.accent k.primary k.red k.green k.blue */ }
    draw();
  });
})();
```
- `slider(parent, label, min, max, step, value, onInput, unit)`; `canvas(parent, w, h)`; `fmt(x, digits)`; `el(tag, attrs, html)`; `matrixHTML(R, label)` renders a bracketed matrix; `project(p3, cx, cy, scale)` oblique 3D→2D; `drawFrame(ctx, R, cx, cy, s, alpha, width)`; `drawBox(ctx, R, cx, cy, s, color)`; `rodrigues(axis, angle)`; `matmul`, `transpose`, `det3`, `skew`.
- Widget names must be globally unique: prefix with your id (e.g., `m03-poe`, `tb-quad`).
- Clear canvas each draw; use `colors()` each draw so dark mode works; canvas 300–440 px wide; labels 12px sans-serif. Every number computed live. Keep readouts short.
- Compute things correctly (e.g., real Jacobian, real RK4 integration, real Kalman update). Test numerics with node where you can.

## Fragments for the shared pages (I merge these)
Write `_fragments/<id>.md` with three sections:
```
## bridges
| Robotics idea | Equation | DiffSim / FVEM | Genomics-Lab | 3D vision | Module |
|---|---|---|---|---|---|
(1–3 rows; Module column like M3)
## words
| Term | Plain words | Precise | Module |
|---|---|---|---|
(6–12 rows)
## symbols
| Symbol | Meaning | Size | Same idea in other labs |
|---|---|---|---|
(only NEW symbols, 0–5 rows)
```

## Checking your work (required before you finish)
1. `node --check docs/javascripts/widgets/<id>.js`
2. Build to a private dir (other writers build at the same time): `cd /home/claude/robotics-lab- && mkdocs build -q -d /tmp/claude-0/-home-claude/ec8b960e-c155-57d8-bd0a-3f1c4060948e/scratchpad/site_<id> 2>&1 | grep -v "not found among documentation files\|is included in the 'nav'" | grep -E "WARNING|ERROR"` — fix anything from YOUR pages. (Warnings about other not-yet-written pages are expected.)
3. Render check with headless Chromium: `python3 /tmp/claude-0/-home-claude/ec8b960e-c155-57d8-bd0a-3f1c4060948e/scratchpad/check.py /tmp/claude-0/-home-claude/ec8b960e-c155-57d8-bd0a-3f1c4060948e/scratchpad/site_<id> <page-slug> [<page-slug2>] <port>` — use your assigned port. It prints MathJax count, raw-TeX-left (must be 0), widget failures (must be 0), JS errors, and saves widget screenshots `w_<slug>_<i>.png` in the scratchpad. Read at least one screenshot per widget and fix layout problems.
4. Do NOT git commit or push. Report back in ≤150 words: files written, widget names, anything uncertain.
