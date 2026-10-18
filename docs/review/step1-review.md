# Step 1 review — scaffold, M1, M2

Split: **Claude checks the lab is built right; Zaki checks it is the right lab.**

## Part 1 — Claude's first-pass review (done 2026-10-18)

Ranked critical → minor. ✅ = fixed before push.

| # | Severity | Finding | Status |
|---|---|---|---|
| 1 | Critical | Display math inside paragraphs rendered as raw `\[ … \]` text | ✅ blank lines added; headless render shows 0 raw TeX left on M1/M2 |
| 2 | Critical | Slider rows overlapped (grid cells narrower than label + slider + value) | ✅ wider cells, fixed flex widths; checked by screenshot |
| 3 | Major | `\|` inside a table cell math (\([R\,\vert\,t]\)) would split the Markdown table | ✅ replaced with `\vert` |
| 4 | Major | Screw widget drawn too small to see the circling motion | ✅ scale raised |
| 5 | Minor | C-space widget opened in a collision state — confusing first view | ✅ obstacle default moved |
| 6 | Minor | One "current research" link pointed at a paper whose claims I could not verify | ✅ replaced with a well-known reference (SE(3)-DiffusionFields), no unverified link |
| 7 | Open | Mermaid diagrams load from a CDN; headless check saw a few 404/403s from third-party assets (favicon / rate limit), not from lab files | watch on the live site |
| 8 | Open | "Current research" for M1/M2 cites some papers from memory (GCS 2023, cuRobo 2023, SE(3)-DiffusionFields 2023) without links | your call: add links or keep |

**Numerical checks run:** Grübler presets (four-bar = 1, 2R = 2, SO-ARM101 5R = 5, 6R = 6, Stewart 6-UPS = 6, free flyer = 6, planar rover = 3); Rodrigues gives \(R^\top R = I\) to ~1e-16 and \(\det R=1\); trace→angle round-trip; planar screw: after \(t=2\) s with \((\omega,v_x,v_y)=(1,0.5,-0.5)\) the body sits at (1.16, 0.25), exactly 0.707 from the pole (0.5, 0.5) = \(\|q\|\). ✔

**Cross-check suggestion:** paste the Grübler derivation and the Rodrigues series-collapse section into another LLM (GPT or Gemini) and ask it to find an error. Critical items worth a second opinion: #1 and the adjoint/wrench transpose statement in M2 Part 6.

## Part 2 — Your review (Zaki)

Tick each when done. Reply "go" to start Step 2 (M3 forward kinematics + M4 Jacobian), or tell me what to change.

- [ ] **Intent** — Is the module template (12 blocks) what you want for every chapter? Anything missing from the DiffSim-Lab feel (e.g., Plain/Full math toggle, "Refresh the basics" links)?
- [ ] **Domain correctness** — Do the robot tables match your hardware? (SO-ARM101 = 5 arm joints + gripper; Crazyflie 4 motors; rover differential drive; ComXim turntable 1 axis)
- [ ] **Analogies** — Latitude/longitude for M1, corkscrew for M2: easy to retell out loud? Swap any that feel forced.
- [ ] **Bridges** — Are the links to DiffSim, FVEM, Genomics-Lab and 3D vision the ones you actually use? Any other lab to tie in (Math-Foundation site)?
- [ ] **Research & 5-year sections** — Right depth? Want more agriculture-specific robotics (harvesting, phenotyping robots)?
- [ ] **Widgets** — Do the 5 instruments teach the idea? Any you'd reuse from DiffSim-Lab instead?
- [ ] **Sign-off** — accept or reject each open item (#7, #8).
