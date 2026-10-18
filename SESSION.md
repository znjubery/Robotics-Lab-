# Session log

## 2026-10-18 — Step 1

**Decisions (Zaki):** audience = me + students; personas = roboticist, SciML bridge-builder, ag-robot field engineer, physical-AI researcher; tracks = manipulation, aerial, rover, physical AI; repo = znjubery/robotics-lab-; spine = Lynch & Park *Modern Robotics*.

**Built:** MkDocs Material site + Pages workflow; index, roadmap (Ch 2–13 → M1–M12, Tracks B–E), symbols, math-bridges table, glossary; M1 (Ch 2) and M2 (Ch 3) with 5 widgets (C-space/C-obstacle, Grübler, Rodrigues, non-commuting rotations, planar screw) and 6 self-checks.

**Checks:** `mkdocs build --strict` clean; headless Chromium render — MathJax typeset, 0 raw TeX, 0 widget failures, light + dark screenshots.

**Waiting on:** Zaki's review (docs/review/step1-review.md).

**Next:** Step 2 = M3 forward kinematics (PoE, SO-ARM101 screw axes, turntable-as-1-joint) + M4 Jacobian & manipulability ellipsoid widget; extend bridges (Jacobian ↔ FEM sensitivity, \(\tau = J^\top F\) ↔ adjoint).

**Resume:** clone repo → `pip install -r requirements.txt` → `mkdocs serve`. Widget code lives in `docs/javascripts/lab.js` (`WIDGETS` map at bottom).
