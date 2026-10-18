# M8 · Trajectory generation

<span class="md-tag">Lynch & Park Ch 9</span> <span class="md-tag">Used in: every arm move, camera fly-throughs, rover video capture</span> <span class="md-tag">Tracks: B, C, D</span>

> **Hook:** A path says *where* the robot goes; a trajectory also says *when* — and choosing the "when" well is the difference between a smooth ride and spilled coffee.

## Intuition first

**Driving from one stop sign to the next.** You know the road (the **path**). What you choose is how you use the pedals (the **time scaling**):

1. **Gas → cruise → brake.** Speed up at the most comfortable rate, hold the speed limit, brake at the same rate. That is the **trapezoidal** velocity profile — the fastest way given a speed limit and a pedal limit.
2. **A jerky new driver** stamps on the pedals: acceleration jumps from zero to full in an instant. Passengers' heads snap. That jump is infinite **jerk** (\(\dddot s\)), and cubic and trapezoid profiles both have it at the ends.
3. **A smooth chauffeur** eases onto the pedal and eases off at the stop. Acceleration starts and ends at zero → the **quintic** (5th-order) profile, or an **S-curve**.
4. **Stops along the way** (a mailbox, a coffee shop) are **via points**; the trick is to roll through them without stopping, which is what **splines** do.

The rest of the page derives each profile, adds speed limits from the robot's real motors, and then drives along curved roads in \(\SE(3)\).

## Problem → math basic → move → result

1. **Problem.** Go from configuration \(\theta_{\text{start}}\) to \(\theta_{\text{end}}\) in time \(T\), smoothly, without breaking velocity, acceleration or torque limits.
2. **Math basic.** Split the trajectory into geometry and timing: \(\theta(t) = \theta(s(t))\) with path parameter \(s\in[0,1]\).
3. **Move.** Pick \(s(t)\) as a polynomial whose coefficients satisfy boundary conditions (a small linear solve), or as piecewise-constant acceleration under limits.
4. **Result.** Closed-form profiles with known peaks: cubic \(\dot s_{\max} = \tfrac{3}{2T}\), quintic \(\dot s_{\max} = \tfrac{15}{8T}\); trapezoid time \(T = \tfrac{v^2 + a}{va}\) for a unit move.

## Part 1 — Path vs trajectory, and the chain rule

A **path** \(\theta(s)\), \(s\in[0,1]\), is pure geometry. A **time scaling** \(s(t)\), \(t\in[0,T]\), says how fast we move along it. Together they make the **trajectory** \(\theta(s(t))\).

For a straight line in joint space, \(\theta(s) = \theta_{\text{start}} + s\,(\theta_{\text{end}} - \theta_{\text{start}})\). The chain rule gives

\[
\dot\theta = \frac{d\theta}{ds}\,\dot s,\qquad \ddot\theta = \frac{d\theta}{ds}\,\ddot s + \frac{d^2\theta}{ds^2}\,\dot s^2
\]

*Why this move:* for a straight line \(d^2\theta/ds^2 = 0\), so joint velocity and acceleration are just \(\dot s, \ddot s\) times a fixed vector. Every limit on the joints becomes a limit on one scalar \(s\). We design one number's motion instead of \(n\).

## Part 2 — Polynomial time scalings

**Cubic.** \(s(t) = a_0 + a_1t + a_2t^2 + a_3t^3\). Four unknowns need four conditions: \(s(0)=0,\ s(T)=1,\ \dot s(0)=0,\ \dot s(T)=0\).

\[
a_0 = 0,\quad a_1 = 0,\qquad
\begin{bmatrix} T^2 & T^3\\ 2T & 3T^2\end{bmatrix}\begin{bmatrix}a_2\\a_3\end{bmatrix} = \begin{bmatrix}1\\0\end{bmatrix}
\;\Rightarrow\; a_2 = \frac{3}{T^2},\ a_3 = -\frac{2}{T^3}
\]

\[
\boxed{\ s(t) = 3\left(\tfrac tT\right)^2 - 2\left(\tfrac tT\right)^3\ }
\qquad \dot s_{\max} = \frac{3}{2T}\ \text{(at } t=T/2\text{)},\qquad \ddot s_{\max} = \frac{6}{T^2}\ \text{(at the ends)}
\]

*Why this move:* the first two conditions at \(t=0\) kill \(a_0, a_1\) at once; only a \(2\times2\) solve is left. With the unit time \(u = t/T\) the formula no longer depends on \(T\) — only the peaks scale as \(1/T\) and \(1/T^2\).

The catch: \(\ddot s(0) = 6/T^2 \ne 0\). Acceleration jumps from 0 to its max at the start — the jerky new driver.

**Quintic.** Add \(\ddot s(0) = \ddot s(T) = 0\): six conditions, six coefficients. The same elimination gives

\[
\boxed{\ s(t) = 10u^3 - 15u^4 + 6u^5,\quad u = t/T\ }
\qquad \dot s_{\max} = \frac{15}{8T},\qquad \ddot s_{\max} = \frac{10}{\sqrt3\,T^2}\approx\frac{5.77}{T^2}
\]

Price of smoothness: peak speed is 25% higher than the cubic (1.875 vs 1.5) for the same \(T\). Jerk is now finite everywhere (it still jumps at the ends; a 7th-order polynomial fixes that too).

??? question "What if we used one high-degree polynomial through many via points?"
    Fitting one degree-\(k\) polynomial through \(k+1\) points wiggles wildly between them near the ends — **Runge's phenomenon**. Robots use **piecewise** cubics or quintics instead: each segment is short, and neighbours share position, velocity (and acceleration) at the via points. That is a **spline**.

## Part 3 — Trapezoid and S-curve: limits first

Now flip the question: given limits \(\dot s\le v\) and \(\lvert\ddot s\rvert\le a\), what is the fastest unit move? Accelerate at \(a\) for \(t_a = v/a\), cruise at \(v\), brake at \(a\).

Distance covered in the two ramps: \(2\cdot\tfrac12 a t_a^2 = v^2/a\). The cruise covers the rest, \(1 - v^2/a\), in time \((1 - v^2/a)/v\). Total:

\[
T = \frac{2v}{a} + \frac{1 - v^2/a}{v} = \frac{v^2 + a}{va}
\]

*Why this move:* "full pedal, then full speed, then full brake" saturates a limit at every instant — that is what makes it time-optimal (a **bang–coast–bang** control). If \(v^2/a > 1\), the car never reaches the limit: the profile becomes a **triangle** with \(T = 2/\sqrt a\).

Example values: \(v = 0.8\) s⁻¹, \(a = 2\) s⁻². Trapezoid: \(T = (0.64 + 2)/1.6 = 1.65\) s. Cubic within the same limits needs \(T = \max(1.5/v,\ \sqrt{6/a}) = 1.875\) s; quintic \(\max(1.875/v,\ \sqrt{5.77/a}) = 2.34\) s. Fastest → smoothest: 1.65 → 1.88 → 2.34 s.

**S-curve.** Limit jerk too: the acceleration ramps up and down linearly instead of jumping. Seven phases (jerk +, 0, −, cruise, −, 0, +). Industrial drives use it because jerk excites vibration — in a robot arm and in a camera rig.

<div class="lab-widget" data-widget="m08-timescale"></div>

Try: in "Fastest motion" mode, raise s̈max and watch the trapezoid shrink to a triangle. Switch to "same time T": the quintic's s̈ starts at 0, the other two jump. Press Play: dots on the tip path are equal time steps — bunched at the ends, spread mid-move. Note the tip path is curved: a straight line in θ is not straight in task space.

## Part 4 — Via points and splines

For via points \(\theta_0,\theta_1,\ldots,\theta_k\) at times \(T_0 < T_1 < \cdots\), use a cubic per segment with **position and velocity** given at both ends:

\[
\theta(t) = H_1(u)\,\theta_j + H_2(u)\,\Delta T\,\dot\theta_j + H_3(u)\,\theta_{j+1} + H_4(u)\,\Delta T\,\dot\theta_{j+1}
\]

with \(H_1 = 1 - 3u^2 + 2u^3,\ H_2 = u - 2u^2 + u^3,\ H_3 = 3u^2 - 2u^3,\ H_4 = -u^2 + u^3\) and \(u = (t-T_j)/\Delta T\).

*Why this move:* these are exactly the **Hermite cubic shape functions** of an Euler–Bernoulli beam element in FEM (nodal value + nodal slope). \(H_3\) is our cubic time scaling. Choose the via velocities by hand, or solve a tridiagonal system so acceleration is also continuous (a natural cubic spline, \(C^2\)).

## Part 5 — Time-optimal scaling along a path (phase plane)

For a general path with full dynamics (M7), substitute \(\theta(s)\) into \(\tau = M\ddot\theta + c + g\):

\[
\tau = m(s)\,\ddot s + c(s)\,\dot s^2 + g(s),\qquad m(s) = M\frac{d\theta}{ds},\ \ldots
\]

Each torque limit \(\tau_{i,\min}\le\tau_i\le\tau_{i,\max}\) becomes a bound \(L(s,\dot s)\le\ddot s\le U(s,\dot s)\). In the **phase plane** \((s,\dot s)\):

- Where \(L > U\), no feasible motion; the boundary of that region (\(L = U\)) is the **velocity limit curve**.
- The fastest trajectory stays as high as possible under that curve, switching between max acceleration \(U\) and max deceleration \(L\) (bang–bang). The trapezoid is the special case with constant bounds.

Classic algorithm: integrate forward with \(U\), backward from the end with \(L\), find the switch points. Modern solvers (TOPP-RA) instead sweep \(s\) in a grid and solve a small linear program per step — robust and fast ([Pham & Pham, 2018](https://arxiv.org/pdf/1707.07239)).

??? warning "Time-optimal is rarely what you want on hardware"
    Bang–bang profiles saturate the motors, leaving zero margin for model error (M7 Challenge 1: friction) and causing jerk. Real systems scale the limits to 80–95% and add jerk bounds. The apple-harvester below runs at 95% of actuation capacity.

## Part 6 — Straight lines in task space and on \(\SE(3)\)

**Joint space vs task space.** A straight line in \(\theta\) is simple and stays within joint limits, but the tool sweeps a curve (see the widget). A straight line for the tool needs IK (M5) at every step and can pass near singularities where \(\dot\theta\) blows up.

**Pose interpolation with screws.** To move a camera or gripper from \(X_{\text{start}}\) to \(X_{\text{end}}\) in \(\SE(3)\), use M2's exp and log:

\[
X(s) = X_{\text{start}}\,\exp\!\big(\log(X_{\text{start}}^{-1}X_{\text{end}})\,s\big)
\]

*Why this move:* \(\log(X_{\text{start}}^{-1}X_{\text{end}})\) is one constant twist \([\mathcal V]\) — the corkscrew that carries start to end. Running it for "time" \(s\) gives a constant-screw motion that never leaves \(\SE(3)\) (no shearing or shrinking, unlike entry-wise averaging). Use any \(s(t)\) from Parts 2–3 on top.

Variant: interpolate position and rotation separately, \(p(s) = p_{\text{start}} + s\,\Delta p\) and \(R(s) = R_{\text{start}}\exp(\log(R_{\text{start}}^\top R_{\text{end}})s)\). The origin then moves in a straight line instead of along a helix — usually what you want for a tool tip or camera centre.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| Cubic with end values + slopes | Hermite cubic shape functions of a beam element (\(C^1\)) | — | cubic B-spline camera trajectories |
| Natural cubic spline (\(C^2\)) | global stiffness-like tridiagonal solve; spline = beam bending energy minimizer | smoothing splines / GAMs for growth curves | smooth NeRF/3DGS fly-through paths |
| Minimum-jerk / smoothness penalty | regularized weak form, \(\int\lVert\dddot\theta\rVert^2\,dt\) | ridge penalty \(\lambda\lVert\beta\rVert^2\); smoothing spline \(\lambda\int f''^2\) | pose-graph smoothness priors |
| Phase-plane bounds \(L\le\ddot s\le U\) | constrained optimization, LP per step | — | — |
| \(X(s) = X_0\exp(\log(X_0^{-1}X_1)s)\) | — | — | Lie-algebra interpolation of camera poses (motion-blur and rolling-shutter NeRF) |

## In your robots

| Where | What this module gives you |
|---|---|
| **SO-ARM101** | Teleop or policy targets arrive as jumps; a quintic (or trapezoid with jerk limit) between targets avoids servo overshoot and gripper shake. LeRobot policies output chunks of actions — the time scaling between them is your choice. |
| **ComXim turntable** | Use a trapezoid or S-curve per angular step, then a settle pause before each photo: leaves swing (M7) if the table stops with a jerk, and swinging leaves blur NeRF/3DGS inputs. |
| **Crazyflie** | The Crazyflie firmware's high-level commander follows polynomial trajectories (go-to, uploaded piecewise polynomials); minimum-snap is the drone version of minimum-jerk (Track B). |
| **Jetson rover** | Rows are paths; the time scaling sets speed vs. motion blur per frame. Constant speed in the row with smooth ramps at the headlands gives even frame spacing for SfM. |
| **Maize/Soybean Plot Builder** | Synthetic rover videos need camera paths: interpolate keyframe poses with the \(\SE(3)\) formula above and a quintic \(s(t)\) to match a real smooth drive. |

## Current research

- **Time-optimal path parameterization.** TOPP-RA solves the phase-plane problem via reachability analysis, one small LP per grid point ([Pham & Pham, 2018](https://arxiv.org/pdf/1707.07239)); it ships in [Drake](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_toppra.html) and [MATLAB Robotics Toolbox](https://www.mathworks.com/help/robotics/ug/generate-time-optimal-trajectories-with-velocity-and-acceleration-limits-using-toppra-solver.html).
- **GPU motion generation.** cuRobo runs parallel collision-free trajectory optimization with smoothness costs in milliseconds ([NVIDIA, Sundaralingam et al.](https://arxiv.org/abs/2310.17274v2)); a 2025 study extends it to industrial systems with extra DOFs ([arXiv 2508.04146](https://arxiv.org/html/2508.04146v2)).
- **Harvesting robots.** A dual-arm apple harvester uses 7th-order polynomials (zero end jerk) plus a control-barrier safety filter, with a 7.53 s mean arm cycle in two Washington orchards ([Zhu et al., 2026](https://arxiv.org/pdf/2606.14089)). Joint scheduling and trajectory planning for multi-arm Cartesian harvesters trades arm count against throughput ([Zhu & Vougioukas, 2025](https://arxiv.org/abs/2505.10028v1)).
- **Camera trajectories in neural rendering.** BAD-NeRF models the camera during exposure by interpolating poses in the \(\SE(3)\) Lie algebra (linear, or cubic B-spline) to undo motion blur ([Wang et al.](https://arxiv.org/pdf/2211.12853)); CRiM-GS does similar continuous rigid-motion modeling for Gaussian splatting ([arXiv 2407.03923](https://arxiv.org/pdf/2407.03923)).

## Challenges

1. **Speed vs smoothness is a hard trade.** Example values above: going from trapezoid to quintic costs 42% more time (1.65 → 2.34 s) under the same limits.
2. **Limits change with the path.** Torque limits make \(U, L\) depend on \(s\) and \(\dot s\); near singularities the velocity limit curve dips toward zero.
3. **Via-point timing.** Choosing \(T_j\) for each segment is itself an optimization; bad choices give overshoot loops between via points.
4. **Dynamic environments.** In a windy canopy the target (a leaf, a fruit) moves; trajectories must be re-planned at 10–100 Hz, not once.

## Next five years

- **Learned policies + classical scaling.** VLA/diffusion policies propose waypoints; TOPP-style scaling enforces limits — a standard two-layer stack.
- **GPU-parallel trajectory optimization** fast enough to replan every control cycle on a Jetson-class board.
- **Jerk-aware phenotyping capture**: camera paths for turntables, rovers and drones optimized jointly for coverage, motion blur and plant sway, so 3D reconstructions get sharper with no new hardware.
- **Differentiable trajectory layers** trained end-to-end with perception (gradients through \(s(t)\) and \(\SE(3)\) interpolation).

## Check yourself

<div class="quiz" data-q="m08-q1" data-answer="b" data-why="The quintic adds the conditions s̈(0) = s̈(T) = 0, so acceleration starts and ends at zero; the cubic jumps to 6/T².">
<p>Which time scaling has zero acceleration at the start and end?</p>
<ol><li data-k="a">Cubic</li><li data-k="b">Quintic</li><li data-k="c">Trapezoid</li><li data-k="d">None of them</li></ol>
</div>

<div class="quiz" data-q="m08-q2" data-answer="d" data-why="T = (v² + a)/(va) = (1 + 4)/(1·4) = 1.25 s, and v²/a = 0.25 &lt; 1 so it is a true trapezoid.">
<p>Unit move, trapezoid with v = 1 s⁻¹ and a = 4 s⁻². The total time is:</p>
<ol><li data-k="a">0.5 s</li><li data-k="b">1.0 s</li><li data-k="c">2.0 s</li><li data-k="d">1.25 s</li></ol>
</div>

<div class="quiz" data-q="m08-q3" data-answer="a" data-why="log(X₀⁻¹X₁) is one constant twist; running exp of it for s ∈ [0,1] stays on SE(3) and follows a single screw motion.">
<p>X(s) = X₀ exp(log(X₀⁻¹X₁) s) moves the pose:</p>
<ol><li data-k="a">along one constant screw motion, staying a valid pose</li><li data-k="b">in a straight line, entry by entry of the 4×4 matrix</li><li data-k="c">only in rotation, not position</li><li data-k="d">through the identity pose</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: a trajectory is a path plus a time scaling s(t) — cubic, quintic, trapezoid or S-curve trade speed against smoothness like gas–cruise–brake between stop signs, and on SE(3) the road itself is a screw.</div>

**Carries over to:** [M9](m09-motion-planning.md) (planners output paths; M8 times them), [M10](m10-robot-control.md) (controllers track \(\theta_d(t)\), \(\dot\theta_d\), \(\ddot\theta_d\)), [Track B](t-b-aerial.md) (minimum-snap drone trajectories), [Track C](t-c-rover-estimation.md) (continuous-time pose splines), [Track D](t-d-physical-ai.md) (action chunks from learned policies).
