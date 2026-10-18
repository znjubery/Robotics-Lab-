# Robotics Learning Lab

Interactive companion to **Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*** — every concept with a hook, an everyday analogy, the full math, a live widget, links to my other labs (DiffSim-Lab, FVEM, Genomics-Lab, 3D vision), current research, challenges, and a five-year outlook.

**Live site:** https://znjubery.github.io/robotics-lab-/

## Contents

| Part | Pages |
|---|---|
| Shared | Roadmap, Symbols, Math bridges (~50 cross-lab rows), Words (~170 terms) |
| Part A · Lynch & Park | M1 C-space · M2 Rigid motions · M3 Forward kinematics · M4 Jacobian & statics · M5 Inverse kinematics · M6 Closed chains · M7 Dynamics · M8 Trajectories · M9 Motion planning · M10 Control · M11 Grasping · M12 Wheeled robots |
| Tracks | B Aerial (Crazyflie) · C Rover estimation & SLAM · D Physical AI (SO-ARM101 + LeRobot) · E Learning & differentiable glue |
| Review | Step 1 and Step 2 review pages (Claude first pass + my checklist) |

Every page: hook → one everyday analogy → derivations → live widget → math bridges to DiffSim/FVEM/Genomics/3D vision → in my robots → current research (linked) → challenges → next five years → 3 self-checks → one-line version. 20 widgets, all computed in the browser.

## Run locally

```bash
pip install -r requirements.txt
mkdocs serve        # http://127.0.0.1:8000
```

## Publish

Pushing to `main` runs `.github/workflows/pages.yml`: strict build → render check → deploy to GitHub Pages.
One-time setup: **Settings → Pages → Source: GitHub Actions**.

## Layout

```
docs/
  index.md  roadmap.md  symbols.md  connections.md  words.md
  m01…m12-*.md  t-b/t-c/t-d/t-e-*.md
  review/step1-review.md  review/step2-review.md
  javascripts/lab.js              # shared helpers, M1–M2 widgets, self-checks, Lab.register()
  javascripts/widgets/<id>.js     # one widget file per module/track
  stylesheets/lab.css
SESSION.md                        # where we left off
AUTHORING.md                      # page template + widget API for new modules
```
