# Robotics Learning Lab

Interactive companion to **Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*** — every concept with a hook, an everyday analogy, the full math, a live widget, links to my other labs (DiffSim-Lab, FVEM, Genomics-Lab, 3D vision), current research, challenges, and a five-year outlook.

**Live site:** https://znjubery.github.io/robotics-lab-/

## Contents

| Part | Status |
|---|---|
| Roadmap, symbols, math-bridges table, glossary | ✅ |
| M1 · Configuration space (Ch 2) — C-space/C-obstacle widget, Grübler calculator | ✅ |
| M2 · Rigid-body motions (Ch 3) — Rodrigues, non-commuting rotations, screw-motion widgets | 🟡 in review |
| M3–M12 (Ch 4–13), Tracks B–E (aerial, rover, physical AI, estimation) | planned |

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
  m01-configuration-space.md  m02-rigid-body-motions.md
  review/step1-review.md          # Claude first-pass + my review checklist
  javascripts/lab.js              # all widgets + self-checks (no dependencies)
  stylesheets/lab.css
SESSION.md                        # where we left off
```
