# M11 · Grasping & manipulation

<span class="md-tag">Lynch & Park Ch 12</span> <span class="md-tag">Used in: every gripper, every pick, every push</span> <span class="md-tag">Tracks: D (physical AI), harvest mechanics lab</span>

> **Hook:** A grasp holds when the fingers can push back against *any* shove the world gives the object. Friction turns each fingertip from a single push into a cone of pushes, and the grasp is safe when those cones, added up, surround zero.

## Intuition first

**The wet bar of soap.** Pinch a dry wooden block between thumb and finger and it stays put, even if you hold it a bit off-center. Pinch a wet bar of soap the same way and it squirts out. Your fingers did not change. The friction did.

That one picture carries the whole chapter:

1. **Touching** — fingers can push on the soap but never pull it (contact is one-sided).
2. **Friction** — each finger can also push a little sideways, up to \(\mu\) times how hard it squeezes. That sweep of allowed pushes is a **friction cone**. Wet soap = thin cone; dry block = wide cone.
3. **Holding** — the grasp is safe if the cones from all fingers together can cancel *any* outside force and twist. If not, the soap squirts out along the direction the cones miss.
4. **How safe** — the size of the gap between "just holds" and "squirts" is a number: the grasp quality \(\varepsilon\).

## Problem → math basic → move → result

1. **Problem.** Contacts are one-sided (push, never pull) and friction is bounded, so the usual equations \(A x = b\) become **inequalities**.
2. **Math basic.** A contact force \(f\) at point \(p\) gives a wrench \(\mathcal F = (p\times f,\ f)\) (M2). Friction limits \(f\) to a cone \(\lVert f_t\rVert \le \mu f_n\).
3. **Move.** Replace each cone by its edge vectors. Every allowed contact wrench is a nonnegative mix of edge wrenches. Ask: do these mixes cover all of wrench space?
4. **Result.** **Force closure** \(\iff\) the origin lies strictly inside the convex hull of the edge wrenches. The distance from the origin to the hull boundary is the quality \(\varepsilon\).

## Part 1 — Contact kinematics: roll, slide, or break

Two bodies \(A\) and \(B\) touch at a point. Let \(\hat n\) be the contact normal pointing into \(A\). Bodies cannot pass through each other, so the speed at which they approach along the normal must not be negative:

\[
\hat n^\top\left(\dot p_A - \dot p_B\right) \ge 0
\]

*Why this move:* this is a **one-sided** constraint. Compare M1's \(A(q)\dot q = 0\): there, the robot can never move in the forbidden direction. Here it can — but only *away*.

Write the contact point velocity with twists (M2): \(\dot p = v + \omega\times p\). Then each contact is in one of three modes:

| Mode | Normal motion | Tangent motion | Label |
|---|---|---|---|
| **Breaking** | \(\hat n^\top(\dot p_A-\dot p_B) > 0\) | anything | B — the finger lifts off |
| **Rolling / sticking** | \(=0\) | \(=0\) | R — no slip at the contact |
| **Sliding** | \(=0\) | \(\ne 0\) | S — slips along the surface |

For a whole object, each contact gets a label, and the object's motion is a twist that satisfies all the inequalities at once. The set of allowed object twists is a **polyhedral cone** in twist space.

**First-order form closure.** If the contacts leave *no* allowed twist except zero, the object is caged by geometry alone — no friction needed. Each frictionless contact removes at most "half a direction", so you need at least

\[
n + 1 \text{ contacts}, \qquad n = 3 \text{ (planar)},\quad n = 6 \text{ (spatial)}
\]

So: 4 fingers to form-close a planar part, 7 for a spatial part.

*Why this move:* \(k\) half-spaces through the origin can only cut \(\R^n\) down to \(\{0\}\) if their normals *positively span* \(\R^n\), which needs at least \(n+1\) vectors — the same counting idea as Grübler in M1, now with inequalities.

??? question "What if we didn't treat contact as one-sided?"
    If we wrote contact as an equality \(\hat n^\top(\dot p_A - \dot p_B) = 0\), the model would say the finger can **pull** the soap. Grasp planners built that way pick grasps that look great on paper and drop the object the moment gravity pulls the wrong way. One-sidedness is the whole difference between a bolted joint (M6) and a contact.

## Part 2 — Coulomb friction cone

At a contact, split the force into a normal part \(f_n\) (squeeze) and a tangent part \(f_t\) (sideways). Coulomb's law says:

\[
f_n \ge 0,\qquad \lVert f_t\rVert \le \mu\, f_n
\]

Geometrically: the force arrow must lie inside a cone around the inward normal with half-angle

\[
\alpha = \arctan\mu
\]

| Surface pair (rough values) | \(\mu\) | \(\alpha\) |
|---|---|---|
| wet soap on skin | ~0.05 | ~3° |
| waxy maize leaf on steel | ~0.3 (example value) | ~17° |
| rubber pad on dry husk | ~0.8 (example value) | ~39° |

*Why this move:* a cone is exactly the shape you get from "sideways push grows linearly with squeeze". Double the squeeze and you double the allowed sideways push — the cone just scales.

**Polyhedral approximation.** In 3D the cone is round, which makes the math nonlinear. Replace it by a pyramid with \(k\) edges \(\hat f_1, \dots, \hat f_k\); every allowed force is then

\[
f = \sum_{j=1}^{k} \lambda_j \hat f_j,\qquad \lambda_j \ge 0
\]

In the plane the cone is just two edge rays (\(k=2\)), so the approximation is exact. In 3D, 4–8 edges is typical; more edges = less error, more variables.

**Contact wrench.** Each edge force at point \(p_i\) makes a wrench (M2). In the plane, \(\mathcal F = (m_z, f_x, f_y)\) with

\[
m_z = p_x f_y - p_y f_x
\]

## Part 3 — Force closure

A grasp is in **force closure** if the fingers can resist *any* external wrench \(\mathcal F_{\text{ext}}\):

\[
\forall\, \mathcal F_{\text{ext}}\ \exists\, \lambda \ge 0:\quad \sum_{i,j} \lambda_{ij}\, \mathcal F_{ij} = -\mathcal F_{\text{ext}}
\]

*Why this move:* this says the edge wrenches \(\mathcal F_{ij}\) **positively span** all of wrench space (\(\R^3\) planar, \(\R^6\) spatial).

**Step 1 — cone to hull.** Nonnegative mixes of the \(\mathcal F_{ij}\) fill all of \(\R^n\) exactly when the origin sits **strictly inside** their convex hull. If the origin were on or outside the hull, a separating plane with normal \(c\) would satisfy \(c^\top \mathcal F_{ij} \le 0\) for every edge, so the fingers could never produce a wrench with \(c^\top\mathcal F > 0\), and the external wrench \(\mathcal F_{\text{ext}} = -c\) could never be balanced.

**Step 2 — test it without an LP.** Build the convex hull of the edge wrenches. For each facet with outward unit normal \(\hat c\) and offset \(d\) (\(\hat c^\top x = d\) on the facet), the origin is inside iff \(d > 0\) for **every** facet.

**Step 3 — quality.** The smallest offset is the radius of the largest ball around the origin that fits in the hull:

\[
\varepsilon = \min_{\text{facets}} d
\]

This is the **Ferrari–Canny \(\varepsilon\) metric** (with unit edge forces): the largest disturbance wrench, in the worst direction, the grasp can resist when the total squeeze is 1. Moments are divided by a length \(\rho\) (object size) so that N and N·m can share one ball.

**Two-finger shortcut (planar).** With two contacts, force closure \(\iff\) the line joining the contacts lies inside **both** friction cones (Nguyen's test). Squeezing along that line balances itself; the cones' leftover width absorbs disturbances.

<div class="lab-widget" data-widget="m11-cone"></div>

Try: start with the ellipse, contacts at 10° and 180°, μ = 0.30 → force closure. Drop μ to 0.05 (wet soap): the green hull shrinks, the origin falls out, readout says NO. Then pick 3 contacts on the circle at 0°, 120°, 240° and slide μ to 0: closure is lost at exactly μ = 0 — frictionless contacts need at least 4 in the plane.

??? warning "Force closure is necessary, not sufficient, for a good grasp"
    Force closure says *some* nonnegative finger forces exist. It does not say the fingers can make them (motor limits), that they won't crush the object (bruising), or that the object stays put while the fingers close. That is why harvesting grippers add a **force cap**: squeeze just enough to stay inside the cone, never more. For soft fruit, the safe force window can be narrow.

## Part 4 — Pushing and soft objects

**Quasi-static pushing.** Many manipulations never grasp at all: a robot pushes a block across a table. "Quasi-static" means motions are slow enough that inertia is ignored — forces balance at every instant. The pusher's contact force must lie in its friction cone, and the support friction under the object resists with a **limit surface**: the set of friction wrenches the table can supply. The object slides with a twist normal to that surface. Result (Mason, 1980s): from the pusher's cone you can predict which way the block turns, then plan pushes like M9 plans paths.

**Contact as complementarity.** Every contact obeys an "either-or" rule:

\[
g \ge 0,\qquad \lambda_n \ge 0,\qquad g\,\lambda_n = 0
\]

where \(g\) is the gap and \(\lambda_n\) the normal force: either the gap is open and the force is zero, or they touch and the force pushes. Discretized in time and linearized, this becomes a **linear complementarity problem (LCP)**, the classic core of rigid-body simulators (ODE, Bullet and PhysX-style iterative solvers); MuJoCo and Drake instead solve convex relaxations of it. It is exactly the **Signorini condition** of contact mechanics in FEM.

**Soft and deformable objects.** A tomato, a maize leaf, or a fresh ear in its husk is not rigid. Soft contact spreads into a patch, which adds a small torsional friction (soft-finger model) and makes the cone a function of how far the finger sinks in. Leaves bend before they resist, so "where the contact is" changes with the force. Practical models:

| Object | Good-enough model | What to measure |
|---|---|---|
| ripe fruit | rigid shape + force cap below bruise threshold | stiffness \(k\) (N/mm), bruise force, \(\mu\) |
| maize ear in husk | rigid core + compliant shell | husk \(\mu\), shell stiffness |
| stalk | Euler–Bernoulli beam | bending stiffness \(EI\), break force |
| leaf | thin shell / cloth | \(\mu\), bending stiffness, tear force |

This is the job of a **harvest mechanics lab**: measure \(\mu\) (tilt-plate or pull test), stiffness (compression test), and damage threshold for each crop, so the gripper's cone and force cap come from data, not guesses.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| contact complementarity \(g\ge0,\ \lambda\ge0,\ g\lambda=0\) | Signorini contact conditions; solved by penalty, Lagrange multiplier or augmented Lagrangian in FEM | — | — |
| friction cone \(\lVert f_t\rVert\le\mu f_n\) | Coulomb friction in contact FEM; second-order cone constraint in convex solvers | — | — |
| force closure = origin inside convex hull of wrenches | — | — | convex hull / visibility tests in point clouds |
| polyhedral cone \(f=\sum\lambda_j\hat f_j,\ \lambda\ge0\) | nonnegativity constraints (e.g., contact pressures, densities) | nonnegative / constrained least squares | NeRF/3DGS densities and opacities kept \(\ge 0\) |
| wrench \(\mathcal F=(p\times f, f)\) from M2 | nodal force + moment resultants on a boundary patch | — | — |
| ε-metric: worst-case resisted wrench | worst-case load / factor of safety | — | robustness margin of a pose estimate |

## In your robots

| Where | What this module gives you |
|---|---|
| **SO-ARM101 gripper** | A two-jaw gripper = two contact patches. Nguyen's test says: close along a line that stays inside both cones. Rubber pads raise \(\mu\) and widen the cones. |
| **Harvest mechanics lab** | Measure \(\mu\), stiffness and bruise force for fruit, ears and stalks → feed the cone half-angle and force cap into grasp planning. |
| **ComXim turntable scanner** | A plant pot on a turntable is held by friction: its cone must cover the tangential force from acceleration, or the pot slides and the scan's camera poses drift. |
| **Maize / Soybean Plot Builder** | Synthetic stalks and ears with known geometry give contact normals for free → generate labelled grasp datasets. |
| **Crazyflie / under-canopy drone** | — (no grasping hardware; perching or leaf sampling would reuse the same cone math) |

## Current research

- **Gentle fruit grippers.** ANGEL, a cable-driven soft gripper with TPU pockets, picked 56 tomatoes with no visible damage and under 9% bruising after 5 days ([Patel et al., 2025](https://arxiv.org/html/2510.18127v1)).
- **Touch + 3D splats for fruit.** DexFruit uses optical tactile sensing to grip soft fruit gently and a Gaussian-splatting method (FruitSplat) to measure bruising from webcam video ([Swann et al., 2025](https://arxiv.org/pdf/2508.07118v3)) — a direct link between grasp quality and 3D phenotyping.
- **Compliant dexterous grasp synthesis.** SpringGrasp models grasping as a spring-like process to plan grasps that stay stable under shape uncertainty from partial views ([Chen, Bohg & Liu, 2024](https://arxiv.org/pdf/2404.13532)).
- **Tactile slip detection.** Learned slip detectors from optical tactile sensors estimate the contact force field to tell "about to slip" before it happens ([arXiv 2303.00935](https://arxiv.org/abs/2303.00935v2); [deformation-field severity, arXiv 2411.07442](https://arxiv.org/html/2411.07442)).
- **Classic metric still in use.** The Ferrari–Canny \(\varepsilon\) metric (1992) is still widely used to score and label grasps in learned grasp datasets.

## Challenges

1. **μ is not one number.** For leaves and husks, \(\mu\) changes with moisture, wax and speed; a swing from 0.3 to 0.1 shrinks \(\alpha\) from ~17° to ~6°.
2. **Narrow force windows.** Soft fruit must be squeezed hard enough to stay in the cone but below the bruise force; the gap can be a few newtons.
3. **Contacts move.** On deformable objects, the contact point and normal change as you squeeze, so the hull you checked is not the hull you get.
4. **Combinatorics.** Spatial force closure lives in \(\R^6\); with 3 fingers × 8 cone edges = 24 wrenches, hull checks and grasp search get expensive inside a planning loop.
5. **Clutter.** Stalks, leaves and neighbours block approach directions; the best-\(\varepsilon\) grasp is often unreachable.

## Next five years

- **Differentiable contact** (complementarity solved smoothly) inside simulators, so grasp and gripper designs optimize by gradient — the same tools as DiffSim.
- **Tactile sensing as standard**, closing the loop on slip and pressure in milliseconds rather than trusting a fixed \(\mu\).
- **Learned grasp policies with physical priors**: VLA policies proposing grasps, force-closure / \(\varepsilon\) checks filtering them (Track D).
- **Agriculture:** crop-specific mechanical property databases (friction, stiffness, bruise and break forces of fruit, ears and stalks) feeding gripper design and harvest simulation, measured alongside 3D phenotyping scans.

## Check yourself

<div class="quiz" data-q="m11-q1" data-answer="c" data-why="α = atan μ = atan 1 = 45°.">
<p>A finger pad has friction coefficient μ = 1. The half-angle of its friction cone is:</p>
<ol><li data-k="a">1°</li><li data-k="b">30°</li><li data-k="c">45°</li><li data-k="d">90°</li></ol>
</div>

<div class="quiz" data-q="m11-q2" data-answer="a" data-why="Planar wrench space is 3D; positively spanning R³ with frictionless normals needs at least 3 + 1 = 4 contacts.">
<p>Fewest frictionless point contacts that can form-close a planar object:</p>
<ol><li data-k="a">4</li><li data-k="b">2</li><li data-k="c">3</li><li data-k="d">7</li></ol>
</div>

<div class="quiz" data-q="m11-q3" data-answer="d" data-why="Force closure holds exactly when the cone-edge wrenches positively span wrench space, i.e., the origin is strictly inside their convex hull.">
<p>A grasp is in force closure when the convex hull of the cone-edge wrenches:</p>
<ol><li data-k="a">has positive volume</li><li data-k="b">touches the origin on one face</li><li data-k="c">lies entirely on one side of the origin</li><li data-k="d">contains the origin strictly inside</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: friction turns each fingertip into a cone of pushes, and a grasp holds when the wrenches from all the cones surround the origin — the wider the margin, the less the object squirts out like wet soap.</div>

**Carries over to:** [M2](m02-rigid-body-motions.md) (wrenches), [M4](m04-velocity-kinematics.md) (\(\tau = J^\top\mathcal F\) turns grasp forces into joint torques), [M6](m06-closed-chains.md) (a grasp closes a kinematic loop), [M9](m09-motion-planning.md) (planning pushes), [M12](m12-wheeled-mobile-robots.md) (mobile manipulation), Track D (learned grasping).
