# M12 · Wheeled mobile robots

<span class="md-tag">Lynch & Park Ch 13</span> <span class="md-tag">Used in: every rover, every row-following robot, every mobile manipulator</span> <span class="md-tag">Tracks: C (rover estimation & SLAM), D (physical AI)</span>

> **Hook:** A car can't slide sideways, yet it can still park in a tight gap. Wiggling forward and back, steering in between, makes a sideways step that no wheel can make on its own — and that hidden step is a Lie bracket.

## Intuition first

**Parallel parking.** You want to move your car one metre to the right into a gap. The wheels can roll forward and back and the car can turn, but it can never slide straight sideways. So you wiggle: back up while turning, then straighten while backing more, then pull forward a little. After a few wiggles, you are in.

That one scene carries the whole chapter:

1. **The rule the wheels obey** — no sideways sliding. That is a velocity constraint (the nonholonomic constraint from M1).
2. **The moves you have** — roll and turn. These are two **vector fields** on a 3D configuration space \((\phi, x, y)\).
3. **The wiggle** — doing "roll, turn, un-roll, un-turn" leaves a small net sideways step. That step is the **Lie bracket** of roll and turn.
4. **Knowing where you are** — counting wheel turns (odometry) and integrating the motion exactly on \(\SE(2)\).
5. **Getting there on purpose** — shortest parking paths (Dubins, Reeds–Shepp) and feedback that keeps you on the path.

## Problem → math basic → move → result

1. **Problem.** The rover has 3 configuration DOF \((\phi,x,y)\) but only 2 inputs, and it cannot move in one of the three directions at any instant.
2. **Math basic.** Write the no-slip rule in Pfaffian form \(A(q)\dot q = 0\) and the motion as \(\dot q = G(q)\,u\) (two allowed directions, two inputs).
3. **Move.** Compute the Lie bracket of the two allowed directions. If brackets fill in the missing direction, the robot can reach every pose (Chow's theorem). Integrate wheel speeds with the exact \(\SE(2)\) exponential from M2.
4. **Result.** Controllable but nonholonomic: every pose reachable, but only by curved, back-and-forth paths; odometry is exact for each constant-speed arc, and its error comes from step size and slip.

## Part 1 — Omnidirectional vs nonholonomic

| Robot type | Wheels | Can it slide sideways now? | Inputs | Model |
|---|---|---|---|---|
| **Omnidirectional** | mecanum or omniwheels (free rollers) | yes | 3 or more | \(\dot q\) can be any vector |
| **Unicycle / differential drive** | 2 driven + caster | no | 2 | nonholonomic |
| **Car-like** | front steering, rear drive | no, and turning radius \(\ge r_{\min}\) | 2 | nonholonomic + curvature limit |
| **Skid-steer / tracked** | 4 wheels or 2 tracks, no steering | no in theory; skids to turn | 2 | diff-drive + slip |

Omnidirectional robots use rollers so that the wheel can slide freely along one direction. They are easy to control (just invert a constant \(3\times n\) matrix) but rollers clog in mud and soil. Field robots are almost always nonholonomic.

## Part 2 — Differential-drive kinematics

Let wheel radius be \(r\), half the axle width \(d\), wheel speeds \(u_L, u_R\) (rad/s), and pose \(q = (\phi, x, y)\) (the book's order).

**Step 1 — each wheel's ground speed.** Rolling without slipping: left wheel moves at \(r u_L\), right at \(r u_R\).

**Step 2 — body twist.** The middle of the axle moves at the average; the difference turns the body:

\[
v = \frac{r\,(u_R + u_L)}{2},\qquad \omega = \frac{r\,(u_R - u_L)}{2d}
\]

*Why this move:* a rigid body moving in the plane has one planar twist \((\omega, v_x, v_y)\) (M2). Two wheels on one axle can only produce \(v_y = 0\) in the body frame.

**Step 3 — rotate into the space frame.**

\[
\dot q = \begin{bmatrix}\dot\phi\\ \dot x\\ \dot y\end{bmatrix} = \begin{bmatrix}0 & 1\\ \cos\phi & 0\\ \sin\phi & 0\end{bmatrix}\begin{bmatrix} v\\ \omega\end{bmatrix} = g_1(q)\,v + g_2(q)\,\omega
\]

with \(g_1 = (0,\cos\phi,\sin\phi)\) "drive forward" and \(g_2 = (1, 0, 0)\) "spin in place".

**Step 4 — the constraint it obeys.** No sideways velocity in the body frame:

\[
A(q)\,\dot q = \begin{bmatrix} 0 & \sin\phi & -\cos\phi\end{bmatrix}\dot q = \dot x\sin\phi - \dot y\cos\phi = 0
\]

This is exactly the Pfaffian constraint from [M1](m01-configuration-space.md). Check: \(A g_1 = \sin\phi\cos\phi - \cos\phi\sin\phi = 0\) and \(A g_2 = 0\). ✔

**Car-like robot.** Same \(g_1\), but turning is tied to speed: \(\omega = v\tan\psi / \ell\), with steering angle \(\psi\) and wheelbase \(\ell\). Since \(\lvert\psi\rvert \le \psi_{\max}\), the turning radius has a minimum \(r_{\min} = \ell/\tan\psi_{\max}\) — the car can't spin in place.

**Example values:** \(r = 0.05\) m, \(d = 0.15\) m, \(u_L = 4\), \(u_R = 6\) rad/s give \(v = 0.25\) m/s, \(\omega = 0.333\) rad/s, turning radius \(v/\omega = 0.75\) m.

## Part 3 — Odometry: integrate on \(\SE(2)\), not with Euler

Wheel encoders give \(u_L, u_R\) every time step \(\Delta t\). Over that step the twist is constant, so the motion is an **exact circular arc** — the planar screw from M2. In the body frame, with \(\Delta\phi = \omega\Delta t\):

\[
\Delta q_b = \begin{bmatrix}\Delta\phi\\ \dfrac{v}{\omega}\sin\Delta\phi\\[4pt] \dfrac{v}{\omega}\left(1 - \cos\Delta\phi\right)\end{bmatrix}\quad(\omega\ne 0),\qquad \Delta q_b = \begin{bmatrix}0\\ v\Delta t\\ 0\end{bmatrix}\quad(\omega = 0)
\]

then rotate the \((x, y)\) part by the current heading \(\phi_k\) and add.

*Why this move:* this is \(T_{k+1} = T_k\, e^{[\mathcal V_b]\Delta t}\), the exact solution of \(\dot T = T[\mathcal V_b]\) for constant \(\mathcal V_b\). Euler's update \(x \leftarrow x + v\cos\phi\,\Delta t\) instead walks along a straight chord and turns at the end, so it cuts the corner every step.

**How big is the Euler error?** Local error per step is \(O(\Delta t^2)\); over \(T/\Delta t\) steps it adds up to a global error \(O(\Delta t)\) — first order. With the example values above and \(T = 8\) s, the Euler end-point error is 0.097 m at \(\Delta t = 0.4\) s and 0.024 m at \(\Delta t = 0.1\) s: four times smaller step, four times smaller error. The exact arc has **zero** discretization error for piecewise-constant wheel speeds.

<div class="lab-widget" data-widget="m12-diffdrive"></div>

Try: set u_L = 4, u_R = 6, then drag the Euler step h from 1 s down to 0.01 s — the red dots hug the blue arc, and the orange dot slides down the slope-1 line (first order). Then add 20% left-wheel slip: the green true path leaves *both* — the integrator is exact, the inputs are wrong. Finally press ▶ Wiggle and compare the net Δy to −Nε².

??? warning "What if the wheels slip? (skid-steer in mud)"
    A skid-steer rover has no steering; it turns by dragging its wheels sideways, so the no-slip model is wrong by design. Its instantaneous centre of rotation (ICR) moves outward with load and soil, so the effective axle width grows to several times the real \(2d\). In wet soil one wheel may spin with 20–50% slip. Odometry then drifts no matter how small \(\Delta t\) is. Fixes: estimate effective \(d\) and slip online, or fuse wheels with camera or IMU (Track C).

## Part 4 — Lie brackets and controllability

The rover has only two allowed directions, \(g_1\) and \(g_2\). Can it reach a pose straight to its side?

**Step 1 — the wiggle.** Flow along \(g_1\) for time \(\varepsilon\), then \(g_2\) for \(\varepsilon\), then \(-g_1\), then \(-g_2\). If the flows commuted you would end where you started. They don't (just like the rotations in M2's "order matters" widget). Taylor-expanding the four flows gives the net motion

\[
\Delta q = \varepsilon^2\,[g_1, g_2] + O(\varepsilon^3),\qquad [g_1, g_2] = \frac{\partial g_2}{\partial q}\,g_1 - \frac{\partial g_1}{\partial q}\,g_2
\]

**Step 2 — compute it for the unicycle.** \(g_2\) is constant, so \(\partial g_2/\partial q = 0\). Only \(g_1\) depends on \(\phi\):

\[
\frac{\partial g_1}{\partial q}\,g_2 = \frac{\partial}{\partial\phi}\begin{bmatrix}0\\ \cos\phi\\ \sin\phi\end{bmatrix}\cdot 1 = \begin{bmatrix}0\\ -\sin\phi\\ \cos\phi\end{bmatrix}\quad\Longrightarrow\quad [g_1, g_2] = \begin{bmatrix}0\\ \sin\phi\\ -\cos\phi\end{bmatrix}
\]

At \(\phi = 0\) that is \((0, 0, -1)\): **pure sideways motion** — the direction the Pfaffian constraint forbids.

*Why this move:* \(\det[g_1\ g_2\ [g_1,g_2]] = -1 \ne 0\) for every \(\phi\), so the three vectors span all of \(\R^3\). By **Chow's theorem**, the rover is (small-time locally) controllable: every nearby pose is reachable. That is also why the constraint is *nonholonomic*: if it were holonomic, brackets would stay inside the allowed directions and some poses would be unreachable.

**Step 3 — the price.** Each wiggle of size \(\varepsilon\) buys only \(\varepsilon^2\) of sideways motion. With \(\varepsilon = 0.3\): 0.09 per cycle, so 4 cycles ≈ 0.36 (the widget shows −0.355; the \(O(\varepsilon^3)\) leftovers also add a little forward drift).

??? question "What if we tried a smooth feedback law to park at one point?"
    Brockett's theorem (1983) says no smooth, time-invariant feedback \(u = k(q)\) can stabilize a nonholonomic unicycle to a single pose. Practical fixes: time-varying or discontinuous feedback, or — most common — plan a feasible path first, then **track** it.

## Part 5 — Motion planning: shortest paths with a turning limit

- **Dubins car** (forward only, \(r \ge r_{\min}\)): the shortest path between two poses is one of six "words" made of maximum-curvature arcs (C) and straights (S): \(LSL, RSR, LSR, RSL, LRL, RLR\).
- **Reeds–Shepp car** (forward *and* reverse): a few dozen word types, with cusps where the car switches gear — parallel parking is one of them.
- These are exact steering functions for **sampling planners** (RRT, PRM from M9): connect two samples with the Dubins/Reeds–Shepp curve instead of a straight line.

## Part 6 — Feedback tracking and mobile manipulation

Plan gives a reference \((\phi_d, x_d, y_d)\) with speeds \((v_d, \omega_d)\). Express the error in the **robot's own frame**:

\[
\begin{bmatrix} x_e\\ y_e\end{bmatrix} = \begin{bmatrix}\cos\phi & \sin\phi\\ -\sin\phi & \cos\phi\end{bmatrix}\begin{bmatrix} x_d - x\\ y_d - y\end{bmatrix},\qquad \phi_e = \phi_d - \phi
\]

A classic tracking law (Kanayama et al., 1990), with gains \(k_1, k_2, k_3 > 0\):

\[
v = v_d\cos\phi_e + k_1 x_e,\qquad \omega = \omega_d + v_d\left(k_2\, y_e + k_3\sin\phi_e\right)
\]

*Why this move:* in the robot frame the sideways error \(y_e\) can't be fixed directly, so it is folded into steering — turn toward the line while driving. This is how row-following works: a camera estimates the crop line, \(y_e\) and \(\phi_e\) come from it, and the law steers.

**Mobile manipulation.** Put an arm on the rover and the C-space becomes \(\SE(2)\times\) (arm joints). The base contributes Jacobian columns too, but only along \(g_1\) and \(g_2\) — the base's sideways column is missing. The full end-effector twist is \(\mathcal V_e = J_{\text{base}}(q)\,u + J_{\text{arm}}(\theta)\,\dot\theta\), and a weighted pseudoinverse (M5) splits the motion between wheels and joints.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| exact arc \(T_{k+1}=T_k e^{[\mathcal V_b]\Delta t}\) vs Euler | exponential / geometric integrators vs forward Euler; global error \(O(\Delta t)\) for Euler | — | camera pose propagation between frames; VO keyframe chaining |
| odometry drift = error accumulates step by step | error accumulation in explicit time stepping; stability & consistency | — | drift in SfM / VGGT trajectories without loop closure or bundle adjustment |
| Lie bracket \([g_1,g_2]\) ≠ 0 | commutator / operator-splitting error (Strang splitting is \(O(\Delta t^2)\) because of it) | — | non-commuting rotations (M2) in pose updates |
| Pfaffian \(A(q)\dot q=0\) | velocity constraints by Lagrange multipliers (incompressibility \(\nabla\cdot u=0\)) | linear constraints on effects | epipolar constraint \(x_2^\top E x_1 = 0\) on camera motion |
| slip / effective axle width estimated online | parameter identification / inverse problems | random effects estimated from data (BLUP) | self-calibration of intrinsics |

## In your robots

| Where | What this module gives you |
|---|---|
| **Jetson Orin Nano rover** | Differential/skid-steer model, exact \(\SE(2)\) wheel odometry, and a row-following law that turns RGB crop-line estimates \((y_e, \phi_e)\) into \((v, \omega)\). |
| **Rover in wet soil** | Expect slip; calibrate effective axle width on your own field surface and fuse with visual odometry (Track C). |
| **Maize / Soybean Plot Builder** | Simulated rover video along Dubins/row paths gives ground-truth \(\SE(2)\) poses to test odometry drift and VGGT pose estimates. |
| **SO-ARM101 on the rover (future)** | Mobile manipulator: base Jacobian (2 columns) + arm Jacobian (5 columns) for reaching stalks or ears. |
| **Crazyflie with Flow deck** | Not wheeled, but the same odometry story: integrate optical-flow velocities, drift grows with time. |

## Current research

- **Low-cost under-canopy row following.** CropFollow++ uses learned semantic keypoints from a camera to drive cover-crop robots under maize canopy; about 25 km of autonomous driving, 767 m on average between interventions ([Sivakumar et al., 2024](https://arxiv.org/html/2404.17718v1)).
- **End-to-end learned row navigation.** A reinforcement-learning policy trained in simulation maps raw LiDAR scans to control commands in crop rows; it works on straight rows and degrades as curvature grows ([Mineiro, Affonso & Becker, 2025](https://arxiv.org/abs/2509.18608v2)).
- **Slip-aware models for skid/tracked robots.** Pseudo-kinematic models fit to terrain-interaction simulation support Lyapunov tracking control and planning for tracked vehicles ([Focchia et al., 2024](https://arxiv.org/pdf/2409.18641)).
- **Learned mobile manipulation.** HoMeR combines imitation learning with whole-body control for in-the-wild mobile manipulation ([arXiv 2506.01185](https://arxiv.org/pdf/2506.01185)); BEHAVIOR Robot Suite targets real-world whole-body household tasks ([Jiang et al., CoRL 2025](https://arxiv.org/html/2503.05652v1)).

## Challenges

1. **Slip breaks the model.** With the example rover (\(u_L=4,\ u_R=6\) rad/s), 20% left-wheel slip changes \(\omega\) from 0.333 to 0.467 rad/s — a heading error of about 7.6° every second. No integrator can fix wrong inputs.
2. **Drift is unbounded.** Pure odometry error grows with distance travelled; a 200 m maize row can end metres off without visual correction.
3. **Wiggles are slow.** Sideways motion scales as \(\varepsilon^2\): a 10× smaller wiggle gives 100× less sideways step. Tight headland turns between rows cost time.
4. **Narrow, changing corridors.** Rows narrow to tens of centimetres of clearance as the canopy closes; leaves occlude cameras and look like obstacles.
5. **Base–arm coordination.** A mobile manipulator has 7+ DOF with missing base directions; errors in base odometry show up directly at the gripper.

## Next five years

- **Learned, slip-aware odometry**: networks that correct wheel odometry from images and IMU, trained on simulated and real fields.
- **Foundation-model navigation**: VLA-style policies conditioned on language ("follow row 4, stop at plot 12") with classic tracking control underneath.
- **Mobile manipulators in the field**: rovers that drive, stop and grasp (leaf sampling, ear measurement) with whole-body control.
- **Agriculture:** fleets of small, cheap under-canopy rovers doing daily 3D phenotyping scans, with poses from fused odometry + VGGT-style reconstruction, replacing slow manual plot notes.

## Check yourself

<div class="quiz" data-q="m12-q1" data-answer="b" data-why="v = r(u_R + u_L)/2 = 0.05 × 10 / 2 = 0.25 m/s.">
<p>Wheel radius r = 0.05 m, u_L = 4 rad/s, u_R = 6 rad/s. The forward speed v is:</p>
<ol><li data-k="a">0.05 m/s</li><li data-k="b">0.25 m/s</li><li data-k="c">0.50 m/s</li><li data-k="d">1.00 m/s</li></ol>
</div>

<div class="quiz" data-q="m12-q2" data-answer="d" data-why="The bracket of drive and spin is the sideways direction, so drive + spin + bracket span all three directions (Chow's theorem).">
<p>A unicycle cannot slide sideways, yet it can reach any pose because:</p>
<ol><li data-k="a">the constraint is holonomic</li><li data-k="b">it has three motors</li><li data-k="c">Euler integration adds sideways motion</li><li data-k="d">the Lie bracket of drive and spin points sideways</li></ol>
</div>

<div class="quiz" data-q="m12-q3" data-answer="a" data-why="Euler is first order: global error scales with h, so halving h roughly halves the error.">
<p>Integrating rover odometry with forward Euler, you halve the step h. The end-point error roughly:</p>
<ol><li data-k="a">halves</li><li data-k="b">stays the same</li><li data-k="c">drops by 4×</li><li data-k="d">doubles</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: wheels forbid sliding sideways, but wiggling roll and turn makes a sideways step (the Lie bracket), so a rover can still park anywhere — and it should track itself with exact SE(2) arcs, not straight Euler steps.</div>

**Carries over to:** [M1](m01-configuration-space.md) (Pfaffian constraints), [M2](m02-rigid-body-motions.md) (planar screw = odometry arc), [M5](m05-inverse-kinematics.md) (weighted pseudoinverse for mobile manipulators), [M9](m09-motion-planning.md) (sampling planners with Dubins/Reeds–Shepp steering), [M11](m11-grasping-manipulation.md) (grasping from a moving base), Track C (fusing odometry with vision for SLAM).
