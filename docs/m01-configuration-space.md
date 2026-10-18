# M1 · Configuration space

<span class="md-tag">Lynch & Park Ch 2</span> <span class="md-tag">Used in: every planner, every IK solve</span> <span class="md-tag">Tracks: all</span>

> **Hook:** Before a robot can move, you must answer one question — *how many numbers does it take to say exactly where every part of it is?* That count is its degrees of freedom; the space of all those numbers is its configuration space.

## Intuition first

**Latitude and longitude.** Two numbers pin down any spot on Earth, so Earth's surface has 2 degrees of freedom. But no flat paper map covers the whole globe without a tear or a stretch — Greenland always looks wrong somewhere.

A robot is the same story. A 2-joint arm needs two angles, so it has 2 DOF. But each angle wraps around (359° is next to 0°), so its space of configurations is not a flat sheet — it is the surface of a **donut (torus)**. Count the numbers *and* know the shape, and you know the robot's C-space.

## Problem → math basic → move → result

1. **Problem.** We need one compact description of "where the robot is" that planners, controllers and learners can all share.
2. **Math basic.** A rigid body in 3D needs 6 numbers (3 position + 3 orientation). Each joint *removes* some of those freedoms.
3. **Move.** Count freedoms of all bodies, subtract what the joints take away → **Grübler's formula**. Then ask what shape the allowed set has (flat, circle, torus, sphere).
4. **Result.** A C-space \(\mathcal C\) with dimension = DOF. Obstacles in the world become forbidden regions **in \(\mathcal C\)**, so motion planning becomes "draw a curve through the free part of \(\mathcal C\)".

## Picture it

Drag the joint sliders, or click anywhere in the right panel. Move the red obstacle and watch its "shadow" in C-space change shape. That shadow is a **C-obstacle**.

<div class="lab-widget" data-widget="cspace"></div>

What to notice:

- One point on the right = one whole pose of the arm on the left.
- A round obstacle in the workspace becomes a curved, band-like blob in C-space. Planners search on the right, not the left.
- Drag θ₁ past 180°: the dot leaves the right edge and appears on the left. That is the torus wrap-around.

## Step-by-step derivation: Grübler's formula

Let \(N\) = number of links **including the ground**, \(J\) = number of joints, \(m\) = DOF of one free rigid body (\(m=3\) in the plane, \(m=6\) in space), and \(f_i\) = freedoms allowed by joint \(i\) (revolute = 1, prismatic = 1, universal = 2, spherical = 3).

**Step 1 — count freedoms if nothing were connected.**

\[
\text{free freedoms} = m(N-1)
\]

*Why this move:* the ground link never moves, so only \(N-1\) bodies are free.

**Step 2 — each joint removes what it does not allow.** Joint \(i\) imposes \(c_i = m - f_i\) constraints.

\[
\text{dof} = m(N-1) - \sum_{i=1}^{J} (m - f_i)
\]

**Step 3 — collect terms.**

\[
\boxed{\ \text{dof} = m\,(N-1-J) + \sum_{i=1}^{J} f_i\ }
\]

**Check on the four-bar linkage** (\(m=3,\ N=4,\ J=4\), all revolute):

\[
3(4-1-4) + 4 = -3 + 4 = 1
\]

One crank angle sets the whole linkage. ✔

??? warning "What if we didn't check independence?"
    Grübler assumes every joint constraint is **independent**. Some mechanisms have redundant constraints (parallel bars, the Bennett linkage) and move even when the formula says dof ≤ 0. Grübler gives a *lower bound*; the true answer comes from the rank of the constraint Jacobian (see M6). Same lesson as FEM: counting equations is not the same as counting *independent* equations — that is a rank question.

<div class="lab-widget" data-widget="grubler"></div>

## Implicit vs explicit description

There are two ways to write down a configuration:

| Way | Example | Pro | Con |
|---|---|---|---|
| **Explicit** (minimal) | 3 Euler angles for orientation | as few numbers as DOF | always has a bad spot (gimbal lock), like the map tearing |
| **Implicit** (embedded) | a \(3\times3\) rotation matrix \(R\) | no bad spots | 9 numbers, so you need constraints |

Count for the rotation matrix:

\[
9 \text{ entries} - 6 \text{ constraints } \underbrace{(R^\top R = I)}_{3 \text{ unit columns} + 3 \text{ orthogonal pairs}} = 3 \text{ DOF}
\]

This "embed in bigger space + constraints" idea is exactly how Module 2 works.

## Constraints on velocity: holonomic vs nonholonomic

Constraints may also act on velocity. Write them in **Pfaffian form**:

\[
A(q)\,\dot q = 0
\]

- **Holonomic** (integrable): the velocity rule comes from a position rule \(g(q)=0\), because \(\frac{d}{dt}g(q) = \frac{\partial g}{\partial q}\dot q = 0\). It *reduces* C-space dimension. Example: the closed loop of a four-bar.
- **Nonholonomic** (not integrable): restricts which *directions* you can move, but every position is still reachable. Example: a wheel cannot slide sideways, \(\dot x\sin\phi - \dot y\cos\phi = 0\), yet a car can still parallel-park into any \((x,y,\phi)\).

*Everyday picture:* a shopping cart can't move sideways right now, but with a few back-and-forth wiggles it can end up anywhere. That is the rover track (M12).

## Workspace, task space, C-space

| Space | What it holds | 2R arm example |
|---|---|---|
| **C-space** | all joint values | torus \((\theta_1,\theta_2)\) |
| **Task space** | what the task cares about | tip position \((x,y)\) |
| **Workspace** | task-space points actually reachable | annulus with radii \(L_1-L_2\) and \(L_1+L_2\) (dashed circles above) |

Mapping C-space → task space is **forward kinematics** (M3). Going back is **inverse kinematics** (M5). One task point can have two arm shapes (elbow up / down) — two points in C-space, one point in task space.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| dof = freedoms − independent constraints | unknowns − Dirichlet nodes = solved DOFs | parameters − constraints (sum-to-zero effects) | BA gauge: fix 7 DOF (pose + scale) or the solve is singular |
| Redundant constraints → rank-deficient Jacobian | singular stiffness matrix when BCs are missing | collinear SNPs → singular \(X^\top X\) | under-constrained camera rig |
| C-space has a shape (torus, sphere) | mesh topology / periodic BCs | — | rotations live on \(\SO(3)\), not \(\R^3\) |
| C-obstacles | signed distance function to a boundary (shifted boundary method) | — | occupancy / SDF grids |

## In your robots

| Robot | C-space | DOF | Inputs | Note |
|---|---|---|---|---|
| SO-ARM101 | \(T^5\) (5 joint circles, limited ranges) + gripper | 5 + 1 | 6 servos | 5 < 6, so it cannot reach every orientation at every point |
| Crazyflie | \(\SE(3)\) = \(\R^3\times\SO(3)\) | 6 | 4 motors | **underactuated**: must tilt to move sideways |
| Jetson rover | \(\R^2\times S^1\) | 3 | 2 wheel speeds | nonholonomic |
| ComXim turntable scanner | \(S^1\) | 1 | 1 | camera poses lie on one circle — a strong prior for 3DGS |
| A maize leaf being grasped | continuous shape | ∞ | — | deformable objects need FEM-style discretization |

## Current research

- **Convex decomposition of C-space.** *Graphs of Convex Sets* (Marcucci et al., *Science Robotics*, 2023) carves free C-space into convex pieces (IRIS regions) so planning becomes a fast convex optimization.
- **GPU-parallel planning.** NVIDIA's cuRobo (2023) checks thousands of C-space samples at once against signed-distance fields — the same SDF idea as the shifted boundary method in DiffSim.
- **Learned collision functions.** Neural signed distance fields of the robot body predict "distance to collision" for any configuration, giving smooth gradients for optimizers.
- **Low-cost arms in research.** LeRobot's SO-100/SO-101 arms brought 5–6 DOF C-spaces into imitation-learning labs ([review](https://thinkrobotics.com/blogs/product-reviews-buying-guides/thinkrobotics-lerobot-so-101-6-axis-robotic-arm-review-ai-ready-open-source-and-built-for-learning); [sim-to-real pipeline example](https://aspireshowcase.dunked.com/building-a-sim-to-real-robotics-pipeline-imitation-learning-digital-twins-and-synthetic-training-for-the-so-101)).

## Challenges

1. **Curse of dimensionality.** A 30-DOF humanoid has a C-space you can never grid. A 120 × 120 grid works for 2 DOF; at 7 DOF it would be \(120^7 \approx 3.6\times10^{14}\) cells.
2. **C-obstacles have no closed form** in general — they must be sampled or learned.
3. **Deformable and plant targets.** Leaves, stalks and ears bend; their C-space is infinite-dimensional. Harvest and phenotyping robots need reduced models (FEM modes, learned latents).
4. **Topology traps.** Treating angles as plain numbers (forgetting wrap-around) causes 359°→0° jumps in learning and control.

## Next five years

- C-spaces of **robot + object + environment** treated jointly, with learned latent coordinates for deformables (crops, cloth, cables).
- Planners that run inside **differentiable simulators**, so C-space gradients flow end-to-end (ties to DiffSim-Lab).
- Policies that output actions on the right manifold (\(\SO(3)\), torus) by construction, instead of fixing topology after the fact.
- For agriculture: **reduced-order plant C-spaces** from procedural models (Maize/Soybean Plot Builder assets) used for manipulation planning.

## Check yourself

<div class="quiz" data-q="m1-q1" data-answer="b" data-why="Each body has 3 freedoms in the plane; Grübler gives 3(4−1−4)+4 = 1.">
<p>A planar four-bar has 4 links (incl. ground) and 4 revolute joints. Its DOF is:</p>
<ol><li data-k="a">0</li><li data-k="b">1</li><li data-k="c">4</li><li data-k="d">3</li></ol>
</div>

<div class="quiz" data-q="m1-q2" data-answer="c" data-why="Nonholonomic constraints limit velocity directions, not reachable positions — so dimension stays 3.">
<p>A rover cannot slide sideways. Its C-space dimension is:</p>
<ol><li data-k="a">2, because one direction is forbidden</li><li data-k="b">1</li><li data-k="c">3, the constraint is nonholonomic</li><li data-k="d">6</li></ol>
</div>

<div class="quiz" data-q="m1-q3" data-answer="a" data-why="Two angles that each wrap around: circle × circle = torus.">
<p>The C-space of a 2R arm with no joint limits is shaped like:</p>
<ol><li data-k="a">a torus (donut surface)</li><li data-k="b">a flat square</li><li data-k="c">a sphere</li><li data-k="d">a line</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: degrees of freedom = freedoms of the free bodies minus what the joints take away, and the space of those numbers has a shape — usually not flat.</div>

**Carries over to:** M2 (orientation as a 3-DOF curved space), M3–M5 (maps between C-space and task space), M9 (planning in free C-space), Track C (rover's nonholonomic C-space).
