# Step 2 review — all modules (M3–M12, Tracks B–E)

Built in auto mode on 2026-10-18 by seven parallel writers. Three independent reviewers who had not written the pages then checked them.
Split: **Claude checks the lab is built right; Zaki checks it is the right lab.**

## Part 1 — Claude's first-pass review

**Automated checks (whole site):** strict build clean; all 21 pages render with MathJax, 0 raw TeX, 0 MathJax errors; all 20 widgets initialise; 0 JS errors.

**Numerical checks run in node** (by the writers, then re-run by the reviewers):

- PoE matches trig to 1e-15, and Jacobians match finite differences.
- IK iteration counts match the page; the four-bar loop closes to 1e-16.
- 2R dynamics: M, c, g and Ṁ−2C skew, with 27.5 and 7.2 N·m reproduced.
- Trajectory times of 1.65, 1.875 and 2.34 s reproduced.
- PID overshoot is 30.2 % simulated vs 30.0 % theory.
- Force closure: Nguyen's test agrees with the hull test on 9,000 random grasps.
- Odometry: Euler error is first order (0.097 → 0.024 m); the wiggle gives Δy = −0.355 vs −Nε² = −0.36.
- EKF: NIS ≈ 1.95–1.98 against the expected 2.
- Ridge: the SVD solution matches a direct solve to 1e-9.

**Errors found by the independent review — all FIXED:**

| Sev | Page | Issue |
|---|---|---|
| Major | M4 widget | Force ellipse was drawn with its axes swapped |
| Major | M7 | Long-run energy numbers for symplectic Euler and RK4 were wrong; the 32×-per-halving energy convergence is now explained |
| Major | M10 | Computed-torque gains should be identity-scaled, not multiplied by joint inertia |
| Major | M11 | Wrong sign in the separating-plane (force-closure) argument |
| Major | M6 | A single parallelogram four-bar is not a Grübler failure; the page now uses the double parallelogram and Bennett linkage |
| Major | Track B | Crazyflie defaults to a complementary filter and switches to the EKF with a Flow or positioning deck; one paper was misattributed |
| Major | Track D | π0 executes 25 of each 50-step chunk (0.5 s), not 1 s |
| Major | shared pages | λ vs λ² convention between DLS and ridge is now stated once; reused letters are flagged in Symbols |
| Minor | many | ~20 citation wordings tightened; one wrong link (GCS) replaced with the arXiv paper; two weak vendor/showcase sources removed from M1 |
| Minor | M8 widget | Now says when the trapezoid must raise s̈ to finish in time T |

**Open items (your call):**

1. Two MDPI pages and the Bitcraze estimator page block automated fetching. They were confirmed through search and the Bitcraze configure page only.
2. Friction values for leaf, husk and stalk in M11 are example values. Replace them with harvest-mechanics-lab measurements later.
3. SO-ARM101 example link lengths (M3, M4) are labelled "example values". Swap in measured values from your arm.
4. LeRobot CLI command names change between versions (Track D says so). The current guide says teleoperate auto-calibrates.
5. Track D widget: more demos (12 → 40) leaves the 74 % success rate unchanged. This supports the covariate-shift point, but it is a toy result.
6. M10 contact readout assumes a fixed wall damping of 20.

**Cross-check suggestion:** paste these three derivations into GPT or Gemini and ask it to find an error. They are the most load-bearing math in the lab.

- The push-through identity (M5)
- e_R ↔ sin θ ω̂ (Track B)
- BLUP = Kalman update (Track C)

## Part 2 — Your review (Zaki)

- [ ] **Intent** — Does the lab answer your goal: concepts, terms, math, and links to your other fields? Any topic missing (e.g., soft robots, legged robots, ROS 2, MuJoCo/Isaac practice)?
- [ ] **Domain correctness** — Hardware tables for SO-ARM101, Crazyflie + decks, Jetson rover, ComXim turntable. Agriculture claims: under-canopy flight, row following, harvesting contact.
- [ ] **Analogies** — Story per page: folding ruler (M3), bicycle gear (M4), cup in the dark (M5), folding chair (M6), playground swing (M7), stop signs (M8), hiking map (M9), shock absorber (M10), wet soap (M11), parallel parking (M12), broomstick (B), dark house (C), learning to cook (D), one key many doors (E). Swap any that don't retell well.
- [ ] **Bridges** — Is the "one equation" story on Track E and Math bridges what you want students to leave with?
- [ ] **Research & 5-year sections** — Right depth and freshness?
- [ ] **Widgets** — Try each one on your phone and laptop. Note any that feel slow or confusing.
- [ ] **Sign-off** — Accept or reject open items 1–6.
