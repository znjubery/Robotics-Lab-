# M6 · Closed chains

<span class="md-tag">Lynch & Park Ch 7</span> <span class="md-tag">Used in: delta pickers, Stewart platforms, legs, grippers, linkages</span> <span class="md-tag">Tracks: A (arms), D (physical AI)</span>

> **Hook:** A folding chair has many hinges but only one way to move — open or shut — because its links form loops; closed-chain robots trade freedom for stiffness and speed, and the math is "the loop must close."

## Intuition first

**The folding chair.** Grab the backrest and pull: the seat, legs and braces all swing together. You moved one thing; four or five hinges turned. Nobody drives those other hinges — the *loop* forces them. And if you try to close the chair the wrong way, it locks or snaps into a second, flipped shape.

The chapter follows that chair:

1. **Count the freedom** → many joints, few DOF (Grübler from M1).
2. **The loop must close** → loop-closure equations \(g(\theta)=0\).
3. **Who pushes, who follows** → active (motor) vs passive (free) joints.
4. **Two ways to fold** → assembly modes; forward kinematics of parallel robots has several answers.
5. **The chair locks or goes floppy** → singularities, read off the rank of the constraint Jacobian.

## Problem → math basic → move → result

1. **Problem.** In a closed chain the joint angles are not independent. Moving the motors moves passive joints in a way we must solve for.
2. **Math basic.** Walk around each loop with transforms; the product must return to the start: \(\prod_i T_i(\theta)=I\), i.e. \(g(\theta)=0\).
3. **Move.** Differentiate: \(\frac{\partial g}{\partial\theta}\dot\theta=0\). Split into active and passive columns and solve for the passive rates. Add forces with Lagrange multipliers.
4. **Result.** For parallel robots, IK is easy (often closed form), FK is hard (polynomial roots, Newton). Singularities appear where a block of the constraint Jacobian loses rank.

## Part 1 — Two classic closed chains

**Four-bar linkage (planar).** Ground \(g\), crank \(a\), coupler \(b\), rocker \(c\); 4 revolute joints. Grübler (M1): \(\text{dof}=3(4-1-4)+4=1\). One motor drives everything.

**Stewart–Gough platform (spatial).** A moving plate on 6 legs, each leg universal–prismatic–spherical (UPS). Grübler: \(N=14\), \(J=18\), \(\sum f_i=6\cdot(2+1+3)=36\):

\[
\text{dof} = 6(14-1-18)+36 = 6
\]

Six motors (the leg lengths) set six pose DOF. Used for flight simulators, precision positioners, and telescope mirrors.

**Delta robot.** Three parallel arms keep the platform *translating only* (3 DOF). Motors sit on the base, so the moving parts are light — the reason deltas dominate fast pick-and-place, including food and produce packing.

## Part 2 — Loop closure for the four-bar

Write each link as a planar vector \(L e^{i\phi}\) (complex-number notation: \(e^{i\phi}=\cos\phi+i\sin\phi\)). Going around the loop from pivot A back to A:

\[
a\,e^{i\theta_2} + b\,e^{i\theta_3} - c\,e^{i\theta_4} - g = 0
\]

Real and imaginary parts give two equations:

\[
g(\theta) = \begin{bmatrix} a\cos\theta_2 + b\cos\theta_3 - c\cos\theta_4 - g\\ a\sin\theta_2 + b\sin\theta_3 - c\sin\theta_4\end{bmatrix} = 0
\]

*Why this move:* 3 unknown angles, 2 equations → 1 free variable. That matches Grübler's dof = 1.

**Solving it (the circle-intersection trick).** Fix the crank \(\theta_2\); joint B is known. Joint C must be at distance \(b\) from B *and* distance \(c\) from D. Two circles meet in **0, 1 or 2** points:

- 2 points: two **assembly modes** (the chair's two ways to fold). They are the two roots of a quadratic, like elbow up/down in M5.
- 1 point (tangent circles): the modes merge — a singularity.
- 0 points: the loop can't close; the crank can't reach this angle.

With \(d=\lVert D-B\rVert\), the loop closes only if \(\lvert b-c\rvert\le d\le b+c\).

**Grashof's rule.** Let \(s\) = shortest link, \(l\) = longest, \(p,q\) the others. If \(s+l\le p+q\), at least one link can turn a full 360°. If \(s\) is the crank, you get a **crank-rocker** (a motor spins, the output rocks — windshield wipers, pumpjacks).

<div class="lab-widget" data-widget="m06-fourbar"></div>

Try: (1) tick "drive the crank" and watch the passive rocker follow while the coupler point traces its curve (solid = mode +1, dashed = mode −1). (2) Set crank a = 1.5, coupler 1.8, rocker 1.6: Grashof fails and the crank stops at angles where the loop can't close. (3) Push a link length until the det readout goes to ≈ 0 and the two modes meet.

??? question "What if we just simulated the four-bar as a 3-link serial arm?"
    You'd get 3 DOF instead of 1, and nothing stops the tip from drifting off pivot D. You must add the loop constraint back — either by solving \(g(\theta)=0\) at every step (this page), or by a penalty/spring (soft, drifts) or Lagrange multipliers (exact, Part 5). Physics engines that only support trees "cut" each loop and re-close it with a constraint for exactly this reason.

## Part 3 — Parallel robots: inverse easy, forward hard

**IK of the Stewart platform (easy).** Given the platform pose \(T_{sb}=(R,p)\), leg \(i\) joins base point \(a_i\) (in \(\{s\}\)) to platform point \(b_i\) (in \(\{b\}\)):

\[
\ell_i = \lVert p + R\,b_i - a_i\rVert,\qquad i=1,\dots,6
\]

*Why this move:* each leg is a straight line between two known points. Six square roots — done. No iteration, one answer.

**FK (hard).** Given six lengths \(\ell_i\), find \((R,p)\): six coupled nonlinear equations. The general Stewart–Gough platform has up to **40** complex solutions (assembly modes); Dietmaier (1998) found a design where all 40 are real. In practice: Newton–Raphson from the last known pose (same algorithm as M5, now on the constraint residual).

| | Serial arm (M3–M5) | Parallel robot |
|---|---|---|
| Forward kinematics | easy: product of exponentials | hard: solve polynomial system / Newton |
| Inverse kinematics | hard: Newton / analytic | easy: leg lengths in closed form |
| Stiffness, speed | lower; errors add along the chain | higher; loads shared, errors average |
| Workspace | large | small |

## Part 4 — The constraint Jacobian and singularities

Differentiate \(g(\theta)=0\) along a motion:

\[
\frac{d}{dt}g(\theta) = \underbrace{\frac{\partial g}{\partial\theta}}_{H(\theta)}\dot\theta = 0
\]

Split joints into active \(\theta_a\) (motors) and passive \(\theta_p\):

\[
H_a\,\dot\theta_a + H_p\,\dot\theta_p = 0 \;\Longrightarrow\; \dot\theta_p = -H_p^{-1}H_a\,\dot\theta_a
\]

*Why this move:* this is the implicit-function theorem. The passive joints are "solved for" as functions of the active ones, as long as \(H_p\) is invertible.

**Four-bar, active \(\theta_2\), passive \((\theta_3,\theta_4)\).** Differentiate the two loop equations:

\[
H_p = \begin{bmatrix} -b\sin\theta_3 & c\sin\theta_4\\ b\cos\theta_3 & -c\cos\theta_4\end{bmatrix},\qquad \det H_p = -bc\,\sin(\theta_4-\theta_3)
\]

\(\det H_p=0\) when coupler and rocker are collinear: the two circles are tangent, the modes merge. The widget prints this determinant live.

**Three kinds of singularity** (rank tells all):

| Type | Rank drop in | What happens | Chair picture |
|---|---|---|---|
| **Actuator** (parallel / "forward") singularity | \(H_p\) | passive joints not determined; the platform gains a free, uncontrolled motion; motor torques for some loads → ∞ | chair goes floppy |
| **End-effector** (serial / "inverse") singularity | end-effector Jacobian of the chain | platform loses a direction it can move in | chair locked flat |
| **C-space** singularity | full \(H\) | the constraint surface itself is not smooth (branches cross) | two folding shapes meet |

**Redundant constraints.** If two loop equations say the same thing (rows of \(H\) linearly dependent), \(H\) is rank-deficient everywhere. Grübler then *undercounts* the DOF (it assumes independent constraints) — a double parallelogram (a parallelogram four-bar with one extra parallel bar: Grübler says 0, yet it moves) and the spatial Bennett 4R linkage (Grübler says −2, yet it moves) are classic examples.

## Part 5 — Forces with Lagrange multipliers

To find motor torques that hold a load, minimize energy (or write power balance) *subject to* \(g(\theta)=0\). Add a multiplier \(\lambda\) per constraint:

\[
\mathcal L(\theta,\lambda) = U(\theta) + \lambda^\top g(\theta)
\]

Stationarity gives

\[
\nabla U(\theta) + H(\theta)^\top\lambda = \tau
\]

*Why this move:* \(H^\top\lambda\) is the set of internal joint forces the loop can exert on itself — forces that do no work along allowed motions (since \(H\dot\theta=0\)). \(\lambda\) is literally the force in the "cut" joint.

Linearized, statics and dynamics with constraints become a **saddle-point (KKT) system**:

\[
\begin{bmatrix} K & H^\top\\ H & 0\end{bmatrix}\begin{bmatrix}\Delta\theta\\ \lambda\end{bmatrix} = \begin{bmatrix} f\\ 0\end{bmatrix}
\]

The zero block makes it indefinite: Cholesky fails; use LDLᵀ, a Schur complement, or null-space elimination. If \(H\) loses rank (redundant constraints), \(\lambda\) is not unique — the "statically indeterminate" case.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| Loop closure \(g(\theta)=0\) | Dirichlet / multi-point constraints \(Bu=\bar u\) on FE unknowns | sum-to-zero constraints on effects | loop closure in pose graphs (drift around a loop must vanish) |
| Lagrange multipliers, \(H^\top\lambda\) | mixed formulations (Stokes pressure, contact) and weakly imposed Dirichlet BCs: \(\lambda\) = reaction force | constrained least squares (KKT) | gauge fixing in BA via constraints |
| Saddle-point system \(\begin{bmatrix}K&H^\top\\H&0\end{bmatrix}\) | same block structure in mixed FEM; needs inf–sup (LBB) stability | Henderson's mixed-model equations are a related block system | — |
| Rank-deficient \(H\) (redundant constraints) | over-constrained BCs → non-unique multipliers; locking | collinear markers / confounded effects | unfixed gauge → singular normal equations |
| \(\dot\theta_p=-H_p^{-1}H_a\dot\theta_a\) | static condensation / implicit-function sensitivities | — | implicit layers: differentiate through a solver |
| FK of parallel robot by Newton | Newton on nonlinear residual | — | multiple solutions like 5-point essential matrix (up to 10) |

## In your robots

| Where | What this module gives you |
|---|---|
| **SO-ARM101** | Serial arm, but its gripper jaw and any parallel-jaw fingers are small closed linkages. Grasping an object with two arms closes a loop through the object: the dual-arm task is a closed chain. |
| **ComXim turntable + camera rig** | A camera on an arm looking at a plant on the turntable, with both fixed to the same bench, forms a loop: bench → turntable → plant ↔ camera ← arm ← bench. Calibration (hand–eye, turntable axis) is solving that loop-closure equation, \(AX=XB\). |
| **Rover** | Suspension (rocker-bogie or four-bar) and steering linkages are closed chains; Grashof decides whether a link can flip over. |
| **Crazyflie / under-canopy drone** | A drone carrying a payload on two cables, or two drones sharing one load, forms closed chains in the air. |
| **Ag packing/harvesting** | Delta robots for produce pick-and-place: IK is closed-form and fast; plan to stay away from the actuator singularity at the edge of the workspace. |

## Current research

- **Delta robots in the field.** A delta (Delta X) parallel robot with a pneumatic cut-and-grip end effector for strawberry harvesting, with field tests ([Ochoa & Mo, *Actuators* 2025](https://www.mdpi.com/2076-0825/14/2/42)).
- **Learning Stewart FK.** DisGNet predicts a distance matrix with a graph neural network, then polishes with a GPU-parallel Newton–Raphson step — exactly "learned seed + exact refinement" ([Zhu et al., arXiv 2402.09077](https://arxiv.org/abs/2402.09077v1)).
- **Closed loops in optimal control.** Trajectory optimization that keeps the closure constraints with analytical derivatives (instead of approximating as serial chains), letting walkers exploit their linkages ([de Matteis et al., arXiv 2504.00642](https://arxiv.org/pdf/2504.00642)).
- **GPU simulation of loops for RL.** Kamino, a GPU multi-body solver on NVIDIA Warp, handles closed kinematic loops natively; it trains a six-loop biped across 4096 parallel environments ([Tsounis et al., arXiv 2603.16536](https://arxiv.org/html/2603.16536v1)).

## Challenges

1. **Many FK solutions.** Up to 40 for a general Stewart–Gough platform; Newton can jump between assembly modes unless steps are small.
2. **Actuator singularities inside the workspace** — torques blow up and the platform can collapse; workspaces must be planned to avoid them with a margin (e.g., keep the four-bar transmission angle above ~40°).
3. **Simulation stiffness.** Constraint drift and stiff saddle-point systems; Baumgarte stabilization or projection is needed every step.
4. **Learning across loops.** Most RL simulators and URDF assume trees; closed loops need special handling, which slows sim-to-real for linkage-rich robots.

## Next five years

- **Differentiable closed-chain simulators** (GPU, Lagrange-multiplier based) letting linkage geometry and control be optimized together — co-design.
- **Compliant and cable-driven parallel mechanisms** for safe, light manipulators near people and plants.
- **Learned + exact FK** for parallel robots on embedded hardware, fast enough for real-time control.
- **Agriculture**: delta and parallel-link pickers on rovers for strawberries, apples and seed handling, and parallel-linkage camera rigs for fast multi-view phenotyping scans.

## Check yourself

<div class="quiz" data-q="m06-q1" data-answer="b" data-why="Each leg length is the distance between two known points once the pose is known, so IK is six closed-form norms.">
<p>For a Stewart–Gough platform, which is easy?</p>
<ol><li data-k="a">forward kinematics: one unique answer</li><li data-k="b">inverse kinematics: leg length ℓᵢ = ‖p + R bᵢ − aᵢ‖</li><li data-k="c">both are equally hard</li><li data-k="d">neither has a solution in general</li></ol>
</div>

<div class="quiz" data-q="m06-q2" data-answer="d" data-why="s + l = 1 + 4 = 5 is greater than p + q = 2 + 2.5 = 4.5, so no link turns fully.">
<p>A four-bar has links 1, 2, 2.5 and 4. Grashof says:</p>
<ol><li data-k="a">crank-rocker</li><li data-k="b">double-crank</li><li data-k="c">change-point</li><li data-k="d">non-Grashof: no link can turn 360°</li></ol>
</div>

<div class="quiz" data-q="m06-q3" data-answer="a" data-why="When the passive block Hₚ is singular, passive joints are not fixed by the motors, so the mechanism gains an uncontrolled motion.">
<p>An actuator (parallel) singularity happens when:</p>
<ol><li data-k="a">the passive-joint block Hₚ of the constraint Jacobian loses rank</li><li data-k="b">a motor reaches its joint limit</li><li data-k="c">the Grübler count is negative</li><li data-k="d">the end effector is at the workspace centre</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: a closed chain is a folding chair — many hinges, one motion — because the loop must close; solve g(θ) = 0 for the passive joints, read singularities from the rank of its Jacobian, and get the internal forces from Lagrange multipliers.</div>

**Carries over to:** [M1](m01-configuration-space.md) (Grübler and constraints), [M5](m05-inverse-kinematics.md) (Newton for parallel FK), [M7](m07-dynamics.md) (constrained dynamics with multipliers), [M11](m11-grasping-manipulation.md) (grasps close loops through the object), [Math bridges](connections.md).
