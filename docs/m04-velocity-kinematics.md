# M4 · Velocity kinematics & statics

<span class="md-tag">Lynch & Park Ch 5</span> <span class="md-tag">Used in: IK (M5), control (M10), grasping (M11), every calibration</span> <span class="md-tag">Tracks: D (physical AI), E (learning & differentiable glue)</span>

> **Hook:** The Jacobian is the arm's gearbox: it says how fast the tip moves for each joint's speed, and, read backwards with a transpose, how hard each motor must push for a given tip force.

## Intuition first

**The bicycle gear.** On a bike, the gear ratio links pedal speed to wheel speed. A high gear makes the wheel fast, but pedalling feels heavy. A low gear makes the wheel slow, but pushing is easy. Power in = power out, so **speed gained is force lost**.

A robot arm is a bike whose gear ratio changes with its pose **and with direction**. One story thread runs through the page:

1. **The gear ratio** → the Jacobian \(J(\theta)\): joint speeds in, tip twist out, \(\mathcal V = J\dot\theta\).
2. **Gears that differ by direction** → the manipulability ellipse: the tip moves easily one way, slowly another.
3. **A gear that jams** → a singularity: some tip direction can't be reached at any joint speed.
4. **Same gearbox, read backwards** → statics, \(\tau = J^\top\mathcal F\): fast directions are weak, slow directions are strong.

## Problem → math basic → move → result

1. **Problem.** For given joint speeds \(\dot\theta\), how fast does the tool move? And what motor torques hold a given tool force?
2. **Math basic.** Differentiate the PoE formula (M3); each factor turns into one screw axis, carried to where it is now with \(\Ad\) (M2).
3. **Move.** Collect the carried axes as columns: \(J_s\) (space) or \(J_b\) (body). Then use "power is the same in joint space and in task space" to get statics for free.
4. **Result.** \(\mathcal V_s = J_s\dot\theta\), \(\mathcal V_b = J_b\dot\theta\), \(J_s = \Ad_{T_{sb}}J_b\), and \(\tau = J^\top\mathcal F\). The eigenvalues of \(JJ^\top\) give the ellipses, and a rank drop marks a singularity.

## Part 1 — The space Jacobian

Start from M3's space form \(T = e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M\) and the space twist \([\mathcal V_s] = \dot T\,T^{-1}\) (M2). By the product rule, \(\dot T\) is a sum of \(n\) terms, one per joint. Term \(i\) differentiates only factor \(i\):

\[
\frac{d}{dt}e^{[\mathcal S_i]\theta_i} = [\mathcal S_i]\,e^{[\mathcal S_i]\theta_i}\,\dot\theta_i
\]

Multiply by \(T^{-1}\) on the right. Everything to the right of factor \(i\) cancels with its own inverse. Write \(E_{i-1} = e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_{i-1}]\theta_{i-1}}\) for the factors to the left:

\[
[\mathcal V_s] = \sum_{i=1}^n E_{i-1}\,[\mathcal S_i]\,E_{i-1}^{-1}\;\dot\theta_i
\]

*Why this move:* \(E[\mathcal S]E^{-1} = [\Ad_E\mathcal S]\) is exactly "the same screw seen after the pose change \(E\)" (M2). So each term is joint \(i\)'s axis, **carried by the joints before it** to where it sits now:

\[
\boxed{\ \mathcal V_s = J_s(\theta)\dot\theta,\qquad J_{s,1} = \mathcal S_1,\qquad J_{s,i} = \Ad_{e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_{i-1}]\theta_{i-1}}}\,\mathcal S_i\ }
\]

**Column \(i\) of \(J_s\) is just the screw axis of joint \(i\) at the current pose.** That is the folding-ruler bookkeeping M3 let you skip: it shows up here instead.

For a planar arm with all pins along \(z\), joint \(i\) sitting now at point \(q_i(\theta)\) gives \(J_{s,i} = (\omega,\ v_x,\ v_y) = (1,\ q_{i,y},\ -q_{i,x})\).

**Worked numbers (2R, \(L_1=1.0,\ L_2=0.8\) m, \(\theta=(0^\circ, 90^\circ)\)).** Joint 1 sits at \((0,0)\), joint 2 at \((1,0)\), and the tip is at \(p=(1,\ 0.8)\).

\[
J_s = \begin{bmatrix}1&1\\0&0\\0&-1\end{bmatrix}\ \text{(rows } \omega, v_x, v_y\text{)}
\]

The tip's linear velocity is \(\dot p = v_s + \omega\times p\). Column by column this gives the \(2\times2\) "tip Jacobian" used in the widgets:

\[
J_v = \begin{bmatrix} -p_y & -(p_y-q_{2,y})\\ p_x & p_x-q_{2,x}\end{bmatrix} = \begin{bmatrix}-0.8&-0.8\\1.0&0\end{bmatrix}
\]

??? question "What if we just differentiated x(θ), y(θ) by hand?"
    For 2 or 3 planar joints, you can: \(\partial x/\partial\theta_1 = -L_1\sin\theta_1 - L_2\sin(\theta_1+\theta_2)\), and so on. That is the **analytic Jacobian** of a chosen set of coordinates. It works, but it depends on how you chose to write orientation (Euler angles bring back gimbal lock from M2). The **geometric** Jacobian \(J_s\) needs no coordinates for orientation, and each column has a physical meaning (the current screw axis). The widgets check \(J_v\) against finite differences of \(x(\theta), y(\theta)\).

## Part 2 — The body Jacobian, and how the two relate

From the body form \(T = Me^{[\mathcal B_1]\theta_1}\cdots e^{[\mathcal B_n]\theta_n}\) and \([\mathcal V_b] = T^{-1}\dot T\), the same steps carry each axis **backwards from the tool**:

\[
J_{b,n} = \mathcal B_n,\qquad J_{b,i} = \Ad_{e^{-[\mathcal B_n]\theta_n}\cdots e^{-[\mathcal B_{i+1}]\theta_{i+1}}}\,\mathcal B_i
\]

Both describe one physical motion, just seen from two frames, and twists change frame with \(\Ad\) (M2):

\[
\mathcal V_s = \Ad_{T_{sb}}\mathcal V_b\ \ \Longrightarrow\ \ \boxed{\ J_s(\theta) = \Ad_{T_{sb}(\theta)}\,J_b(\theta)\ }
\]

*Why this matters:* \(\Ad_T\) is always invertible, so \(J_s\) and \(J_b\) have the **same rank**. A singularity is a property of the arm, not of the frame you chose.

## Part 3 — Singularities: when the gear jams

\(J\) is \(6\times n\) (or \(2\times n\) for a planar tip). At a **singularity**, its rank drops: some tip direction gets zero speed from every joint combination.

For the 2R arm, \(\det J_v = L_1L_2\sin\theta_2\). It is zero at \(\theta_2 = 0^\circ\) (arm straight, at the reach limit) and at \(\theta_2 = 180^\circ\) (folded back). At \(\theta_2 = 90^\circ\): \(\det J_v = 1.0\times0.8\times1 = 0.8\).

*Why "jam" is the right picture:* with the arm straight, both joints move the tip sideways (perpendicular to the arm), and neither can move it along the arm. You could spin the motors at any speed and still not move outward. Near a singularity, inverse kinematics (M5) asks for **huge** joint speeds, \(\dot\theta = J^{-1}\mathcal V\) blows up like \(1/\sin\theta_2\).

??? warning "Wrist singularities on 6-axis arms"
    When two wrist axes line up (e.g., wrist roll parallel to base yaw), two columns of \(J\) become equal and the rank drops by one, even deep inside the workspace. Industrial controllers slow down or reroute near these. On the SO-ARM101, wrist roll lines up with the forearm, and the arm has only 5 joints, so rank ≤ 5 everywhere: one tool direction is always missing.

## Part 4 — Manipulability ellipse from \(JJ^\top\)

Spin the joints at unit total speed, \(\lVert\dot\theta\rVert = 1\). Where can the tip velocity \(v = J\dot\theta\) go? With the SVD \(J = U\Sigma V^\top\), the unit circle in joint speeds maps to an ellipse with **semi-axes \(\sigma_i\) along the columns \(u_i\) of \(U\)**:

\[
v^\top (JJ^\top)^{-1}\,v \le 1,\qquad JJ^\top = U\Sigma^2U^\top,\qquad \sigma_i = \sqrt{\lambda_i(JJ^\top)}
\]

*Why this move:* \(JJ^\top\) is symmetric and positive semi-definite, so its eigenvectors are the ellipse's axes and \(\sqrt{\lambda_i}\) are their lengths. It is the same "eigen lens" the FVEM repo uses on stiffness matrices: eigenvectors say *which direction*, eigenvalues say *how much*.

Three summary numbers:

| Measure | Formula | 2R at \(\theta=(0^\circ,90^\circ)\) | Meaning |
|---|---|---|---|
| Axes | \(\sigma_1, \sigma_2\) | \(1.397,\ 0.573\) m/rad | fastest / slowest tip direction |
| Volume (Yoshikawa) | \(w=\sqrt{\det JJ^\top}=\sigma_1\sigma_2\) | \(0.800\) | area of the ellipse, \(/\pi\) |
| Condition number | \(\kappa = \sigma_1/\sigma_2\) | \(2.44\) | 1 = round (isotropic), \(\infty\) = singular |

Check: \(JJ^\top = \begin{bmatrix}1.28&-0.8\\-0.8&1.0\end{bmatrix}\), \(\det = 1.28 - 0.64 = 0.64\), so \(\sigma_1\sigma_2 = \sqrt{0.64} = 0.8 = \det J_v\). Consistent.

<div class="lab-widget" data-widget="m04-ellipse"></div>

Try: drag θ₂ toward 0° and watch the ellipse squash into a line, σ₂ → 0 and κ → ∞. Switch to "force" or "both" — the force ellipse grows long exactly where the velocity ellipse is thin. Pick 3R and look for the faint arm: a joint motion that moves the elbow but not the tip.

## Part 5 — Statics: \(\tau = J^\top\mathcal F\)

Hold the arm still while the tool pushes on the world with wrench \(\mathcal F\). Which joint torques \(\tau\) do this? Use **power** (or virtual work, same idea): the power the motors put in must equal the power the tool puts out, for *any* small motion \(\dot\theta\):

\[
\tau^\top\dot\theta = \mathcal F^\top\mathcal V = \mathcal F^\top J\dot\theta \quad\text{for all }\dot\theta
\]

*Why this move:* if two linear functions of \(\dot\theta\) agree for every \(\dot\theta\), their coefficient vectors are equal. So

\[
\boxed{\ \tau = J^\top(\theta)\,\mathcal F\ } \qquad (\tau = J_s^\top\mathcal F_s = J_b^\top\mathcal F_b)
\]

**Worked numbers.** Same 2R pose, link 2 pointing straight up. Push the tip down on a soybean pod with \(f = (0, -10)\) N:

\[
\tau = J_v^\top f = \begin{bmatrix}-0.8&1.0\\-0.8&0\end{bmatrix}\begin{bmatrix}0\\-10\end{bmatrix} = \begin{bmatrix}-10\\0\end{bmatrix}\ \text{N·m}
\]

The shoulder carries \(10\ \text{N}\times1.0\ \text{m}\) (horizontal lever arm) and the elbow carries nothing, because the force points straight along link 2.

<div class="lab-widget" data-widget="m04-statics"></div>

Try: aim the force along link 2 and watch τ₂ go to zero. Then straighten the arm (θ₂ → 0°) and push along it: both torques vanish. At a singularity the structure, not the motors, holds the load. The power check row confirms fᵀv = τᵀ dθ/dt for every pose.

**Force ellipse.** Now limit the torques, \(\lVert\tau\rVert\le1\). Then \(f^\top JJ^\top f\le1\): an ellipse with the **same axes** but **inverted lengths** \(1/\sigma_i\). This is the bike gear: the direction where the tip is fastest is where it is weakest. At a singularity, the arm can resist an infinite force along the jammed direction (the straight arm pushing along its length), but cannot move that way at all.

## Part 6 — Redundancy and the null space

With more joints than task numbers (\(n > m\): a planar 3R arm placing a point, \(n=3,\ m=2\)), \(J\) is wide and has a **null space**: joint speeds that move nothing at the tip.

\[
\dot\theta = J^\dagger v + (I - J^\dagger J)\,z,\qquad J^\dagger = J^\top(JJ^\top)^{-1}
\]

*Why this move:* \(J^\dagger v\) is the smallest-norm joint speed that gives \(v\) (a least-squares solution, same as M5's damped IK). \((I-J^\dagger J)\) projects any \(z\) into the null space, so you can add it for free: dodge a maize leaf with the elbow, stay away from joint limits, or keep the camera level, while the tip stays on target.

For the 3R arm at \(\theta=(30^\circ, 60^\circ, -90^\circ)\): \(J_v = \begin{bmatrix}-1.3&-0.8&0\\1.366&0.5&0.5\end{bmatrix}\), and the cross product of its two rows gives the null direction (the widget draws it).

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| \(J=\partial(\text{tip})/\partial\theta\), columns = current screw axes | sensitivity \(\partial u/\partial p\); strain–displacement matrix \(B=\partial\varepsilon/\partial u\) | design matrix \(X=\partial\hat y/\partial\beta\) | reprojection Jacobian \(\partial(\text{pixel})/\partial(\text{pose, point})\) in bundle adjustment |
| \(\tau = J^\top\mathcal F\) from equal power | adjoint method: \(\nabla_p L = (\partial u/\partial p)^\top\lambda\); \(f = B^\top\sigma\) (internal forces from stress, virtual work) | normal equations \(X^\top(y - X\beta)=0\) | backprop: gradient w.r.t. pose \(=J^\top\) (residual) |
| \(JJ^\top\) eigen-axes = manipulability ellipse | eigen lens on \(K\): stiff vs soft modes; \(K=B^\top DB\) has the same "\(A^\top A\)" form | genomic relationship matrix \(G\propto ZZ^\top\); its eigenvectors are PCA axes of population structure | BA normal matrix \(J^\top J\): small eigenvalues = poorly constrained poses (e.g., turntable views all at one height) |
| Singular \(J\), \(\kappa\to\infty\) | singular \(K\) without enough boundary conditions | \(p\gg n\): \(X^\top X\) singular, needs ridge/GBLUP | gauge freedom; degenerate camera motion (pure rotation gives no depth) |
| Null space \((I-J^\dagger J)z\) | zero-energy (rigid-body) modes | effects not identifiable from data | scale/gauge directions in SfM |

## In your robots

| Where | What this module gives you |
|---|---|
| **SO-ARM101 (LeRobot)** | \(J\) is \(6\times5\), so rank ≤ 5 at every pose: one tool direction is always missing. Torques \(\tau=J^\top\mathcal F\) tell you how much payload the STS3215 servos can hold at each pose — far less with the arm stretched out. |
| **ComXim turntable** | One joint: \(J\) is a single column \(=\mathcal S\). A point at radius \(r\) moves at \(r\dot\theta\); a 1 m tall plant's leaf tips move fastest, so they blur first. |
| **Crazyflie / under-canopy drone** | The mixer matrix (four rotor thrusts → total thrust + three torques) is a Jacobian-like linear map; its condition number says how much control authority is left in each axis. |
| **Jetson Orin rover** | Wheel speeds → body twist is a Jacobian (M12). Its rank is 2 for a differential drive: no sideways speed. |
| **Maize/Soybean Plot Builder** | Render camera Jacobians on synthetic scenes to test which viewpoint sets make the BA normal matrix \(J^\top J\) well-conditioned before going to the field. |

## Current research

- **Manipulability as a learned skill.** Geometry-aware methods treat manipulability ellipsoids as points on the manifold of symmetric positive-definite matrices, so robots can learn, track and transfer them ([Jaquier, Rozo, Caldwell, Calinon](https://arxiv.org/pdf/1811.11050)). ManiDP extends this to diffusion policies for posture-dependent bimanual tasks ([Li et al., arXiv 2510.23016](https://arxiv.org/pdf/2510.23016)).
- **Manipulability in harvesting-robot design.** A 2025 fruit-harvesting system scores reachability maps with the Jacobian's singular values (manipulability = product of singular values, plus an isotropy index) ([Han, Cho, Kim, AgriEngineering 2025](https://www.mdpi.com/2624-7402/7/12/417)).
- **Jacobians inside learning.** Theseus makes nonlinear least squares (Gauss–Newton, Levenberg–Marquardt) differentiable in PyTorch, so Jacobian-based solvers for kinematics, SLAM and bundle adjustment can sit inside a network ([Pineda et al., NeurIPS 2022](https://arxiv.org/pdf/2207.09442)).
- **GPU-parallel kinematics.** cuRobo evaluates kinematics for many configurations in parallel for motion generation ([cuRobo](https://curobo.org)).

## Challenges

1. **Near-singular IK.** At \(\kappa = 100\), a 1 mm/s tip request needs joint speeds 100× larger along the weak direction. Damping (M5) trades accuracy for safety.
2. **Payload at full stretch.** For a 2-link arm of total reach \(\ell\), holding a weight \(mg\) horizontally needs shoulder torque \(mg\ell\). With an example \(\ell = 0.35\) m and 0.5 kg, that is about 1.7 N·m (17 kg·cm) — above the 15 kg·cm stall rating of an STS3215, before counting the arm's own weight.
3. **Unit mixing.** \(J\) stacks angular (rad/s) and linear (m/s) rows, so \(\kappa\) and "volume" depend on the length unit. Scale by a characteristic length before comparing poses.
4. **Estimating contact force from \(\tau\).** \(\mathcal F = (J^\top)^{\dagger}\tau\) amplifies servo-current noise near singularities, and hobby servos have friction that looks like force.

## Next five years

- **Manipulability-aware learned policies** (diffusion/VLA) that keep the arm in well-conditioned poses on their own.
- **Force sensing without force sensors**: \(\tau = J^\top\mathcal F\) with better motor-current models, making cheap arms touch-aware.
- **Differentiable kinematics everywhere**: Jacobians computed by autodiff, shared between robot control, calibration and 3D reconstruction (one optimizer for arm pose and camera pose).
- **Gentle plant handling**: arms that choose poses whose force ellipse is small in the direction of a fragile maize ear or soybean pod, limiting peak force during phenotyping or harvest.

## Check yourself

<div class="quiz" data-q="m04-q1" data-answer="b" data-why="Each column is joint i's screw axis carried by the joints before it to its current place: Ad of e^[S1]θ1⋯e^[S(i−1)]θ(i−1) applied to Sᵢ.">
<p>Column i of the space Jacobian Jₛ is:</p>
<ol><li data-k="a">the home screw axis Sᵢ, always</li><li data-k="b">the screw axis of joint i at the current pose</li><li data-k="c">the tip position divided by θᵢ</li><li data-k="d">the i-th row of the rotation matrix</li></ol>
</div>

<div class="quiz" data-q="m04-q2" data-answer="d" data-why="Equal power τᵀθ̇ = Fᵀ J θ̇ for every θ̇ forces τ = Jᵀ F.">
<p>The relation τ = Jᵀ F comes from:</p>
<ol><li data-k="a">Newton's second law with mass</li><li data-k="b">inverting J</li><li data-k="c">the condition number</li><li data-k="d">power in = power out for every joint motion</li></ol>
</div>

<div class="quiz" data-q="m04-q3" data-answer="a" data-why="The force ellipse has axes 1/σᵢ, so the shortest velocity axis becomes the longest force axis.">
<p>Near a singularity the velocity ellipse becomes a thin line. In the thin direction, the force ellipse is:</p>
<ol><li data-k="a">very long — the arm can resist large forces there</li><li data-k="b">also thin</li><li data-k="c">a perfect circle</li><li data-k="d">undefined because forces vanish</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: the Jacobian is the arm's pose-dependent gear ratio — its columns are the current screw axes, its JJᵀ eigenvalues draw the speed ellipse, and its transpose turns tip forces into motor torques.</div>

**Carries over to:** [M5](m05-inverse-kinematics.md) (Newton–Raphson and damped least-squares IK use \(J\)), [M7](m07-dynamics.md) (mass matrix \(M=\sum J_i^\top\mathcal G_iJ_i\)), [M10](m10-robot-control.md) (task-space and force control), [M11](m11-grasping-manipulation.md) (grasp maps and contact wrenches), [Track E](t-e-learning-glue.md) (autodiff Jacobians and adjoints).
