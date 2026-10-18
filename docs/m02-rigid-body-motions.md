# M2 · Rigid-body motions

<span class="md-tag">Lynch & Park Ch 3</span> <span class="md-tag">Used in: every pose, every camera, every drone</span> <span class="md-tag">Tracks: all</span>

> **Hook:** Every way a rigid object can move — a drone banking, an arm reaching, a camera orbiting a plant — is the motion of a corkscrew: turn about one line while sliding along it. Robotics writes that corkscrew as a single matrix exponential.

## Intuition first

**The corkscrew.** Push a corkscrew into a cork and it turns and advances at the same time, always around the same line. In 1830, Chasles proved that *any* rigid-body displacement is exactly such a screw motion: one axis, an angle turned about it, and a distance slid along it.

So the whole chapter is one story thread:

1. **Pure turning** (no slide) → rotation matrices \(R\in\SO(3)\).
2. **Turning + sliding** → poses \(T\in\SE(3)\).
3. **The speed of the corkscrew** → a twist \(\mathcal V\).
4. **Running the corkscrew for a while** → the exponential \(e^{[\mathcal S]\theta}\).

## Problem → math basic → move → result

1. **Problem.** Angles can't be added like numbers (order matters, see the widget below), and Euler angles break at gimbal lock.
2. **Math basic.** Store orientation as a \(3\times3\) matrix whose columns are the body's axes. Composition = matrix multiplication.
3. **Move.** Solve the simplest motion ODE, \(\dot R = [\omega]R\), exactly. Its solution is a matrix exponential, and the series collapses to a closed form (Rodrigues).
4. **Result.** Three numbers \(\hat\omega\theta\) (exponential coordinates) for orientation; six numbers \(\mathcal S\theta\) for any pose — with no gimbal lock and an exact way to integrate motion.

## Part 1 — Rotations, \(\SO(3)\)

\[
\SO(3) = \{R\in\R^{3\times3} : R^\top R = I,\ \det R = +1\}
\]

The columns of \(R_{sb}\) are the \(x,y,z\) axes of the body frame \(\{b\}\), written in the space frame \(\{s\}\). One matrix, three jobs:

| Job | Formula | Plain words |
|---|---|---|
| Represent orientation | \(R_{sb}\) | where \(\{b\}\) points, seen from \(\{s\}\) |
| Change frame | \(p_s = R_{sb}\,p_b\) | rewrite a vector in another frame (subscripts cancel) |
| Rotate | \(R\,p\) | physically turn a vector |

Facts that come free: \(R^{-1}=R^\top\) (cheap inverse), \(\det R=1\) (no mirror flip), \(\|Rp\|=\|p\|\) (lengths kept).

### Order matters

<div class="lab-widget" data-widget="noncommute"></div>

Rotate 90° about \(x\) then 90° about \(z\), and you end somewhere different than doing \(z\) first. That is why orientations are **multiplied**, never added.

## Part 2 — Angular velocity

Differentiate \(RR^\top = I\):

\[
\dot R R^\top + R\dot R^\top = 0 \;\Rightarrow\; \dot R R^\top = -(\dot R R^\top)^\top
\]

*Why this move:* a matrix that equals minus its own transpose is **skew-symmetric**, and every \(3\times3\) skew matrix is a cross product in disguise:

\[
[\omega] = \begin{bmatrix} 0 & -\omega_3 & \omega_2\\ \omega_3 & 0 & -\omega_1\\ -\omega_2 & \omega_1 & 0\end{bmatrix},\qquad [\omega]\,v = \omega\times v
\]

So

\[
\dot R = [\omega_s]\,R \quad\text{(space frame)},\qquad \dot R = R\,[\omega_b] \quad\text{(body frame)},\qquad \omega_s = R\,\omega_b
\]

The set of all \([\omega]\) is called \(\so(3)\) — the **Lie algebra**: the "velocity space" that lives on the tangent of \(\SO(3)\) at the identity.

## Part 3 — Exponential coordinates and Rodrigues

Spin at unit rate about a fixed unit axis \(\hat\omega\) for time \(\theta\):

\[
\dot R = [\hat\omega] R,\quad R(0)=I \;\Longrightarrow\; R(\theta) = e^{[\hat\omega]\theta} = I + [\hat\omega]\theta + \frac{[\hat\omega]^2\theta^2}{2!} + \frac{[\hat\omega]^3\theta^3}{3!} + \cdots
\]

*Why this move:* it is the same solution as the scalar ODE \(\dot x = ax \Rightarrow x = e^{at}\), now with a matrix.

**The trick.** For a unit axis, \([\hat\omega]^3 = -[\hat\omega]\). So powers repeat with a sign flip, and the series sorts into the sine and cosine series:

\[
e^{[\hat\omega]\theta} = I + \underbrace{\left(\theta - \tfrac{\theta^3}{3!} + \cdots\right)}_{\sin\theta}[\hat\omega] + \underbrace{\left(\tfrac{\theta^2}{2!} - \tfrac{\theta^4}{4!} + \cdots\right)}_{1-\cos\theta}[\hat\omega]^2
\]

\[
\boxed{\ R = I + \sin\theta\,[\hat\omega] + (1-\cos\theta)\,[\hat\omega]^2\ } \qquad\text{(Rodrigues' formula)}
\]

**Going back (matrix log):**

\[
\theta = \arccos\!\left(\frac{\operatorname{tr}R - 1}{2}\right),\qquad [\hat\omega] = \frac{R - R^\top}{2\sin\theta}\quad(0<\theta<\pi)
\]

At \(\theta = \pi\), \(\sin\theta = 0\) and the axis sign is ambiguous — the same kind of "bad spot" every 3-number chart has, just pushed to the edge.

<div class="lab-widget" data-widget="rotation"></div>

Try: set elevation 90° (axis = \(z\)) and check the matrix is the familiar 2D rotation in its top-left corner. Then push θ to ±180° and watch the log check.

??? question "What if we used Euler angles instead?"
    Roll–pitch–yaw works until pitch hits ±90°. Then roll and yaw turn about the same axis and one DOF vanishes — **gimbal lock**. It is the "map tear" from M1's latitude–longitude story (at the poles, longitude means nothing). Exponential coordinates and quaternions avoid it in practice; the Crazyflie firmware estimates attitude with quaternions for this reason.

## Part 4 — Poses, \(\SE(3)\)

Glue rotation and position into one \(4\times4\) matrix:

\[
T = \begin{bmatrix} R & p\\ 0 & 1\end{bmatrix}\in\SE(3),\qquad T^{-1} = \begin{bmatrix} R^\top & -R^\top p\\ 0 & 1\end{bmatrix},\qquad T_{ab}T_{bc} = T_{ac}
\]

*Why the extra row of \([0\ 0\ 0\ 1]\):* it turns "rotate then shift" (\(Rx+p\), not linear) into one matrix multiply on \((x,1)\). Every graphics engine, COLMAP and NeRF use this trick.

## Part 5 — Twists and screws

Velocity of a pose, again by "something times \(T\)":

\[
T^{-1}\dot T = [\mathcal V_b] = \begin{bmatrix}[\omega_b] & v_b\\ 0 & 0\end{bmatrix},\qquad \dot T T^{-1} = [\mathcal V_s]
\]

A **twist** \(\mathcal V = (\omega, v)\in\R^6\) is angular + linear velocity packed together.

**Screw axis.** Every twist is "spin rate × unit screw": \(\mathcal V = \mathcal S\dot\theta\). For an axis through point \(q\) with direction \(\hat s\) and pitch \(h\) (metres slid per radian turned):

\[
\mathcal S = \begin{bmatrix}\hat s\\ -\hat s\times q + h\hat s\end{bmatrix}
\]

- \(h=0\): pure rotation (a revolute joint). \(h=\infty\): pure translation, \(\mathcal S = (0, \hat v)\) (a prismatic joint).

**Exponential of a screw** — the corkscrew run for "angle" \(\theta\):

\[
e^{[\mathcal S]\theta} = \begin{bmatrix} e^{[\hat\omega]\theta} & G(\theta)\,v\\ 0 & 1\end{bmatrix},\qquad G(\theta) = I\theta + (1-\cos\theta)[\hat\omega] + (\theta - \sin\theta)[\hat\omega]^2
\]

This one formula is the engine of forward kinematics in M3: each joint contributes one \(e^{[\mathcal S_i]\theta_i}\).

The widget shows the planar case. The red dot is where the screw axis pierces the page: every point of the body circles it.

<div class="lab-widget" data-widget="screw"></div>

Set ω = 0 and the pole runs off to infinity — the corkscrew becomes a pure slide.

## Part 6 — Changing the frame of a twist: the adjoint

The same physical motion looks different from different frames:

\[
\mathcal V_s = \Ad_{T_{sb}}\,\mathcal V_b,\qquad \Ad_T = \begin{bmatrix} R & 0\\ [p]R & R\end{bmatrix}\in\R^{6\times6}
\]

Forces and moments pack into a **wrench** \(\mathcal F = (m, f)\). Power \(\mathcal V^\top\mathcal F\) can't depend on the frame, which forces

\[
\mathcal F_b = \Ad_{T_{sb}}^\top\,\mathcal F_s
\]

Velocities go *forward* with \(\Ad\); forces go *back* with \(\Ad^\top\). That transpose pairing returns as \(\tau = J^\top\mathcal F\) in M4 — and as the adjoint method in DiffSim-Lab.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| \(R = e^{[\hat\omega]\theta}\) solves \(\dot R=[\hat\omega]R\) | \(u(t)=e^{-Kt}u_0\) solves \(\dot u=-Ku\); exponential time integrators | — | pose integration between IMU samples |
| Rodrigues collapses the series via \([\hat\omega]^3=-[\hat\omega]\) | eigen-decomposition collapses \(e^{-Kt}\) to \(\sum e^{-\lambda_i t}\phi_i\phi_i^\top\) | — | Euler's \(e^{i\theta}=\cos\theta+i\sin\theta\) is the 2D case |
| \(R^\top R = I\), \(\det R=1\) | rotation of local element frames | PCA loadings are orthonormal | camera rotation; Procrustes / Kabsch alignment of point clouds |
| \(T\in\SE(3)\), homogeneous coords | rigid-body modes of a free FE body (6 zero eigenvalues) | — | camera extrinsics; NeRF stores camera-to-world \(T\) |
| Update on the group: \(T \leftarrow e^{[\delta\mathcal V]}T\) | — | — | Gaussian-splatting SLAM optimizes poses with Lie-algebra Jacobians ([Matsuki et al.](https://arxiv.org/pdf/2312.06741)) |
| \(\Ad\) forward, \(\Ad^\top\) back | adjoint/transpose for gradients | — | backprop through the renderer |

## In your robots

| Where | What this module gives you |
|---|---|
| **ComXim turntable scanner** | Each step turns the plant about one fixed axis \(\mathcal S\). Camera poses are \(T_k = e^{[\mathcal S]\theta_k}T_0\): calibrate **one** axis and get every pose — a clean initializer for NeRF/3DGS on the AI-testbed station. |
| **Crazyflie** | Attitude \(R\) (quaternion in firmware), body rates \(\omega_b\) from the gyro, thrust always along the body \(z\)-axis \(= R e_3\). Geometric controllers act directly on \(\SO(3)\) (Track B). |
| **SO-ARM101** | Each revolute joint is a zero-pitch screw. End-effector pose = product of five exponentials (M3). |
| **Rover** | Planar \(\SE(2)\): \((x, y, \phi)\). Wheel odometry integrates a twist with exactly the screw widget above. |
| **Maize Plot Builder / VGGT** | Rover-video camera paths are curves in \(\SE(3)\); VGGT predicts these poses directly. |

## Current research

- **Lie-group SLAM with Gaussian splats.** Gaussian Splatting SLAM tracks camera pose by optimizing on \(\SE(3)\) with analytic Lie-algebra Jacobians ([Matsuki et al.](https://arxiv.org/pdf/2312.06741)); newer systems handle RGB-only outdoor scenes ([arXiv 2502.15633](https://arxiv.org/pdf/2502.15633)) and robust initialization ([RGS-SLAM](https://arxiv.org/pdf/2601.00705)), and pose-free splatting starts from inaccurate poses ([Look Gauss, No Pose](https://arxiv.org/pdf/2410.08743)).
- **Learning with rotations.** Networks that regress orientations use continuous representations (e.g., 6D rotation features turned into \(R\) by Gram–Schmidt) because 3-number charts have jumps.
- **Generative models on \(\SE(3)\).** Diffusion models that sample grasp poses directly on \(\SE(3)\) (e.g., SE(3)-DiffusionFields, Urain et al., 2023). Vision-language-action (VLA) policies output end-effector pose changes, so this math sits under them too (Track D).

## Challenges

1. **Averaging and uncertainty on curved spaces.** You can't average two rotations entry-wise; the result leaves \(\SO(3)\). Covariances must live in the Lie algebra.
2. **Frame bookkeeping bugs.** COLMAP stores world-to-camera; NeRF tools store camera-to-world; OpenCV and OpenGL flip axes. Most "my reconstruction is garbage" bugs are a missing \(T^{-1}\).
3. **Discontinuities for learning.** Euler angles and quaternions (\(q\) and \(-q\) are the same rotation) give networks jumpy targets.
4. **Drift.** Integrating noisy twists accumulates pose error; it must be corrected by loop closure or GPS (Track C).

## Next five years

- **Equivariant networks** (outputs rotate when the input rotates) becoming standard for 3D perception and grasping, cutting data needs.
- **Pose-free 3D reconstruction** (VGGT-style models) predicting \(\SE(3)\) camera poses and geometry in one pass — directly relevant to field maize scans.
- **Differentiable Lie-group layers** inside simulators and renderers, so robot, camera and plant poses train jointly.
- **Calibration by motion**: robots that know their own screw axes (turntables, arms) used as live calibration rigs for phenotyping cameras.

## Check yourself

<div class="quiz" data-q="m2-q1" data-answer="c" data-why="Using tr R = 1 + 2cosθ: (2 − 1)/2 = 0.5, arccos(0.5) = 60°.">
<p>A rotation matrix has trace 2. Its rotation angle is:</p>
<ol><li data-k="a">0°</li><li data-k="b">30°</li><li data-k="c">60°</li><li data-k="d">90°</li></ol>
</div>

<div class="quiz" data-q="m2-q2" data-answer="b" data-why="Power is frame-independent, so wrenches transform with the transpose of the adjoint.">
<p>Twists change frame with Ad<sub>T</sub>. Wrenches change frame with:</p>
<ol><li data-k="a">Ad<sub>T</sub> as well</li><li data-k="b">the transpose of Ad<sub>T</sub></li><li data-k="c">R only</li><li data-k="d">T inverse</li></ol>
</div>

<div class="quiz" data-q="m2-q3" data-answer="d" data-why="A revolute joint turns without sliding, so the screw pitch is zero.">
<p>A revolute joint is a screw with pitch:</p>
<ol><li data-k="a">infinite</li><li data-k="b">1</li><li data-k="c">equal to the link length</li><li data-k="d">0</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: every rigid motion is a corkscrew — turn about an axis and slide along it — and the matrix exponential of a twist turns that corkscrew into a 4×4 pose you can multiply.</div>

**Carries over to:** M3 (product of exponentials), M4 (Jacobian columns are screw axes), M7 (dynamics with twists and wrenches), Track B (geometric drone control on \(\SO(3)\)), Track C (pose graphs and SLAM).
