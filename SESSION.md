# Session log

## 2026-10-18 — Step 1

**Decisions (Zaki):** audience = me + students; personas = roboticist, SciML bridge-builder, ag-robot field engineer, physical-AI researcher; tracks = manipulation, aerial, rover, physical AI; repo = znjubery/robotics-lab-; spine = Lynch & Park *Modern Robotics*.

**Built:**

- MkDocs Material site and a GitHub Pages workflow.
- Shared pages: index, roadmap, symbols, math bridges, glossary.
- M1 and M2 with 5 widgets.

## 2026-10-18 — Step 2 (auto mode, "until finish")

**Built:**

- M3–M12 (Lynch & Park Ch 4–13) and Tracks B (aerial), C (rover estimation and SLAM), D (physical AI) and E (learning and differentiable glue).
- Seven parallel writers did the work. Each page has its own widget file, `docs/javascripts/widgets/<id>.js`, registered with `Lab.register(name, fn)` from `lab.js`.
- Fragments were merged into connections.md (~50 bridge rows), words.md (~170 terms) and symbols.md (~70 symbols, with a clash note).

**Checks:**

- Strict build and headless Chromium render on all 21 pages: 0 raw TeX, 0 widget failures, 0 JS errors.
- Three independent reviewers fixed about 30 issues; the ranked list is in docs/review/step2-review.md.

**Waiting on:** Zaki's review (docs/review/step2-review.md, Part 2) and the open items 1–6 listed there.

**Possible next steps:**

- Part 8 capstones that use real data, e.g. calibrating the turntable screw axis from 3DGS poses, or SO-ARM101 system ID.
- A Plain/Full math toggle like DiffSim-Lab.
- Links to the Math-Foundation "refresh the basics" anchors.
- Measured hardware values to replace the example ones.

**Resume:**

1. Clone the repo.
2. `pip install -r requirements.txt`, then `mkdocs serve`.
3. Read AUTHORING.md for the page template and widget API before adding modules.
