# M5 · Inverse kinematics

<span class="md-tag">Lynch & Park Ch 6</span> <span class="md-tag">Used in: reaching, grasping, camera aiming, teleoperation</span> <span class="md-tag">Tracks: A (arms), D (learning)</span>

> **Hook:** Forward kinematics asks "where is my hand?"; inverse kinematics asks "which joint angles put my hand on the cup?" — and the best general answer is the same thing you do when you reach for a cup in the dark: feel the miss, nudge, repeat.

## Intuition first

**Reaching for a cup in the dark.** You know roughly where the cup is. You move your arm, feel how far off your fingers are, and correct. Each correction is small and aimed straight at the miss. Within two or three tries you touch it.

The chapter follows that one reach:

1. **You can see the cup and your arm is simple** → solve it with geometry, in one shot (analytic IK). There are usually *two* ways to reach: elbow up or elbow down.
2. **Feel the miss, correct** → Newton–Raphson: measure the error, turn it into joint changes with the Jacobian from M4, repeat.
3. **Arm fully stretched, cup still out of reach** → your corrections get wild. A sensible person reaches *cautiously*: damped least squares.
4. **Extra joints to spare** → you can also keep your elbow away from the table while reaching: null-space motion.

## Problem → math basic → move → result

1. **Problem.** Given a desired pose \(T_{sd}\), find \(\theta\) with \(T(\theta)=T_{sd}\). The map is nonlinear; there may be 0, 1, 2, … or infinitely many answers.
2. **Math basic.** Near a guess, FK is linear: a small joint change \(\Delta\theta\) moves the tip by \(J(\theta)\,\Delta\theta\) (M4).
3. **Move.** Measure the error as a twist, solve the *linear* least-squares problem \(J\Delta\theta\approx e\) (with a penalty \(\lambda^2\lVert\Delta\theta\rVert^2\) for safety), step, repeat.
4. **Result.** Quadratic convergence near a good solution (error exponent doubles each step), graceful behavior near singularities and unreachable targets — and an update that is *literally* ridge regression.

## Part 1 — Analytic IK for the 2R planar arm

Links \(L_1, L_2\), target \((x,y)\). The tip sits at distance \(r=\sqrt{x^2+y^2}\) from the base, and the triangle base–elbow–tip has sides \(L_1, L_2, r\).

**Step 1 — elbow angle from the law of cosines.**

\[
r^2 = L_1^2 + L_2^2 + 2L_1L_2\cos\theta_2 \;\Longrightarrow\; \cos\theta_2 = \frac{x^2+y^2-L_1^2-L_2^2}{2L_1L_2} \equiv D
\]

*Why this move:* the elbow angle is the only thing that sets the base-to-tip distance. Fix the distance first; worry about direction later.

\[
\theta_2 = \pm\arccos D
\]

The \(\pm\) are the two solutions: **elbow up** and **elbow down**, mirror images across the base–target line.

**Step 2 — shoulder angle = direction to target minus the elbow's offset.**

\[
\theta_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}\!\left(L_2\sin\theta_2,\; L_1 + L_2\cos\theta_2\right)
\]

*Why this move:* \(\operatorname{atan2}\) keeps the correct quadrant; plain \(\arctan\) would lose half the circle.

**Counting solutions.**

| \(D\) | Meaning | Solutions |
|---|---|---|
| \(\lvert D\rvert<1\) | inside the reachable ring | 2 (elbow up / down) |
| \(D=\pm1\) | on the outer or inner circle (arm straight or folded) | 1 — a **singularity**, \(\det J=L_1L_2\sin\theta_2=0\) |
| \(\lvert D\rvert>1\) | outside the ring | 0 |

**Example values.** \(L_1=1.0\), \(L_2=0.8\), target \((1.2, 0.6)\): \(D=(1.44+0.36-1-0.64)/1.6=0.1\), so \(\theta_2=\pm84.3^\circ\).

For spatial arms, analytic IK exists for special designs: a 6R arm with a **spherical wrist** (last three axes meet at a point) splits into "position from the first three joints, orientation from the wrist" and has up to 8 solutions; a general 6R arm has up to 16.

## Part 2 — Newton–Raphson: feel the miss, correct

**Scalar warm-up.** Solve \(f(\theta)=x_d\): linearize \(f(\theta+\Delta\theta)\approx f(\theta)+f'(\theta)\Delta\theta\), set it equal to \(x_d\):

\[
\Delta\theta = \frac{x_d - f(\theta)}{f'(\theta)}
\]

**Vector version.** Replace \(f'\) by the Jacobian and the division by an inverse:

\[
\theta^{k+1} = \theta^k + J^{\dagger}(\theta^k)\,e^k,\qquad e^k = x_d - f(\theta^k)
\]

**What is the "error" for a pose?** A difference \(T_{sd}-T_{sb}\) is not a pose and not a velocity. Instead ask: *which twist, run for one second, carries the current end-effector frame \(\{b\}\) onto the desired frame \(\{d\}\)?* That is the matrix log from M2:

\[
[\mathcal V_b] = \log\!\left(T_{sb}^{-1}(\theta^k)\,T_{sd}\right) = \log(T_{bd})
\]

*Why this move:* \(T_{sb}^{-1}T_{sd}=T_{bd}\) is the leftover pose seen from the hand (subscripts cancel). Its log is a 6-vector in the same frame as the body Jacobian \(J_b\), so the two can be combined.

**Lynch & Park's algorithm (body form).**

1. Guess \(\theta^0\). Set \(k=0\).
2. \([\mathcal V_b]=\log(T_{sb}^{-1}(\theta^k)T_{sd})\).
3. If \(\lVert\omega_b\rVert<\epsilon_\omega\) and \(\lVert v_b\rVert<\epsilon_v\): stop.
4. \(\theta^{k+1}=\theta^k + J_b^{\dagger}(\theta^k)\,\mathcal V_b\); \(k\leftarrow k+1\); go to 2.

**The pseudoinverse \(J^\dagger\).** For \(J\in\R^{m\times n}\) of full rank:

| Shape | \(J^\dagger\) | What it picks |
|---|---|---|
| square, \(n=m\) | \(J^{-1}\) | the unique step |
| fat, \(n>m\) (redundant arm) | \(J^\top(JJ^\top)^{-1}\) | the **smallest** \(\Delta\theta\) that hits \(e\) exactly |
| tall, \(n<m\) (e.g., 5-joint arm, 6-D pose) | \((J^\top J)^{-1}J^\top\) | the step with **least-squares** error |

Newton's speed: near a regular solution the error is squared each iteration, \(\lVert e^{k+1}\rVert\approx C\lVert e^k\rVert^2\). On a log plot that looks like a cliff: \(10^{-2}\to10^{-4}\to10^{-8}\).

??? warning "Newton only finds the solution near your guess"
    Start near elbow-up and you converge to elbow-up. Start badly and Newton can jump, cycle, or land on the other branch. Practical solvers restart from many random seeds (GPU solvers run thousands at once) and keep the best one.

## Part 3 — Damped least squares (cautious reaching)

Near a singularity a singular value \(\sigma_i\) of \(J\) goes to 0. Write \(J=\sum_i\sigma_i u_i v_i^\top\) (SVD). Then

\[
J^\dagger e = \sum_i \frac{1}{\sigma_i}\,(u_i^\top e)\,v_i
\]

*The problem:* \(1/\sigma_i\to\infty\). A tiny tip error in a "hard" direction asks for a huge joint step — the flailing of an outstretched arm.

**The fix — penalize big steps.** Solve

\[
\min_{\Delta\theta}\ \lVert J\Delta\theta - e\rVert^2 + \lambda^2\lVert\Delta\theta\rVert^2
\]

Set the gradient to zero: \(2J^\top(J\Delta\theta-e)+2\lambda^2\Delta\theta=0\), so

\[
\Delta\theta = (J^\top J + \lambda^2 I_n)^{-1}J^\top e
\]

**Push-through identity — derive it.** Start from an obvious equality (multiply out both sides):

\[
J^\top(JJ^\top + \lambda^2 I_m) = J^\top J J^\top + \lambda^2 J^\top = (J^\top J + \lambda^2 I_n)J^\top
\]

Multiply on the left by \((J^\top J+\lambda^2I_n)^{-1}\) and on the right by \((JJ^\top+\lambda^2I_m)^{-1}\) (both exist for \(\lambda>0\), since they are symmetric positive definite):

\[
\boxed{\ \Delta\theta = (J^\top J + \lambda^2 I_n)^{-1}J^\top e = J^\top(JJ^\top + \lambda^2 I_m)^{-1}e\ }
\]

*Why this move:* the left form inverts an \(n\times n\) matrix (joints); the right form inverts an \(m\times m\) matrix (task, \(m\le6\)). Pick the smaller. For a redundant arm, that is the \(m\times m\) one.

**SVD view.** Each direction gets gain \(\sigma_i/(\sigma_i^2+\lambda^2)\) instead of \(1/\sigma_i\):

| \(\sigma_i\) vs \(\lambda\) | Gain | Behavior |
|---|---|---|
| \(\sigma_i\gg\lambda\) | \(\approx 1/\sigma_i\) | same as Newton |
| \(\sigma_i=\lambda\) | \(1/(2\lambda)\) — the maximum | capped |
| \(\sigma_i\ll\lambda\) | \(\approx\sigma_i/\lambda^2\to0\) | hard direction ignored |

The price: the step is biased, so convergence drops from quadratic to linear — a straight sloped line on the log-error plot instead of a cliff. Common practice: large \(\lambda\) far away or near singularities, small \(\lambda\) near the answer (Levenberg–Marquardt does exactly this, see below).

### Live: Newton vs damped least squares

<div class="lab-widget" data-widget="m05-ik"></div>

Try: (1) with the default target, compare the curves — Newton's red cliff (quadratic) vs the blue straight slope (linear); raise λ and watch the blue slope flatten. (2) Click far outside the dashed circle: Newton's steps blow up and the arm thrashes; set log₁₀ λ ≥ −0.3 and the damped arm settles pointing straight at the target with error equal to the gap. (3) Pick the 3R arm and toggle the null-space pull: same tip, different elbow.

## Part 4 — The same equation in four fields

Rename the symbols and damped least squares becomes ridge regression:

| Robotics (DLS IK) | Genomic prediction (ridge / RR-BLUP) |
|---|---|
| \(J\in\R^{m\times n}\) | marker matrix \(X\in\R^{N\times p}\) (lines × markers) |
| task error \(e\) | phenotypes \(y\) (centered) |
| joint step \(\Delta\theta\) | marker effects \(\beta\) |
| \(\lambda^2\) | \(\lambda=\sigma_e^2/\sigma_\beta^2\) |
| \((J^\top J+\lambda^2I)^{-1}J^\top e\) | \(\hat\beta=(X^\top X+\lambda I)^{-1}X^\top y\) |

The push-through identity gives \(\hat\beta = X^\top(XX^\top+\lambda I_N)^{-1}y\). With 50 000 markers and 500 lines, that turns a \(50\,000\times50\,000\) solve into a \(500\times500\) one. Predicted genetic values:

\[
\hat g = X\hat\beta = XX^\top(XX^\top + \lambda I)^{-1}y = G\,(G + \tfrac{\lambda}{k} I)^{-1}y,\qquad G = \tfrac{1}{k}XX^\top
\]

That is **GBLUP** with the genomic relationship matrix \(G\) (scaling \(k\), e.g. \(2\sum p_j(1-p_j)\)). So RR-BLUP and GBLUP are the same predictor — the marker-space and line-space forms — exactly as the joint-space and task-space forms of DLS are the same step.

- **Tikhonov** (inverse problems, e.g., recovering a FEM load from sensor data): \(\min\lVert Ku-d\rVert^2+\alpha\lVert u\rVert^2\). Same equation, \(K\) is the forward operator.
- **Levenberg–Marquardt** (bundle adjustment in SfM/COLMAP): residuals \(r(x)\) of reprojection, step \((J^\top J+\lambda I)\delta=-J^\top r\) (often \(\lambda\,\mathrm{diag}(J^\top J)\)). Raise \(\lambda\) after a bad step, lower it after a good one. DLS IK is LM with one camera and no points.

??? question "What if we didn't damp — just used the pseudoinverse with a cutoff?"
    Truncated SVD (drop \(\sigma_i<\epsilon\)) also avoids blow-up, but the step jumps when \(\sigma_i\) crosses \(\epsilon\): the arm twitches. Damping fades the direction out smoothly. Ridge vs. principal-component regression in genomics is the same trade-off.

## Part 5 — Redundancy and the null space

If \(n>m\) (the widget's 3R arm reaching a 2-D point), infinitely many \(\theta\) hit the target. The joint motions that do **not** move the tip form the null space of \(J\). The projector onto it is \(I-J^\dagger J\). So

\[
\Delta\theta = J^\dagger e + (I - J^\dagger J)\,z,\qquad z = -k\,\nabla h(\theta)
\]

*Why this move:* the first term does the main task; the second descends a secondary cost \(h\) — joint centering \(h=\tfrac12\lVert\theta\rVert^2\) (the widget's toggle, \(k=0.3\)), distance to obstacles, or manipulability — **without disturbing the tip** to first order.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| Newton: \(\theta\leftarrow\theta+J^\dagger e\) | Newton–Raphson for nonlinear FEM: \(K_T\Delta u=r\); quadratic convergence on log-error plots | — | Gauss–Newton in BA |
| DLS \(\Delta\theta=J^\top(JJ^\top+\lambda^2I)^{-1}e\) | Tikhonov-regularized inverse problems | ridge / RR-BLUP \(\hat\beta=(X^\top X+\lambda I)^{-1}X^\top y\) | Levenberg–Marquardt in BA |
| Push-through: \(n\times n\) ↔ \(m\times m\) solve | primal vs dual (adjoint) form of a least-squares solve | RR-BLUP (markers) ↔ GBLUP (lines, \(G=XX^\top/k\)) | Schur complement: reduce BA to camera-only system |
| Gain \(\sigma/(\sigma^2+\lambda^2)\) | spectral filter of a regularized inverse | shrinkage; effective parameters \(\sum\sigma_i^2/(\sigma_i^2+\lambda)\) | damping of poorly constrained camera directions |
| Error as \(\log(T_{sb}^{-1}T_{sd})\) | — | — | pose residual \(\log(T_{\text{est}}^{-1}T_{\text{meas}})\) in pose graphs / 3DGS pose refinement |
| Null space \(I-J^\dagger J\) | constraint-preserving updates | non-estimable combinations of effects | gauge freedom (7-DOF similarity) |

## In your robots

| Where | What this module gives you |
|---|---|
| **SO-ARM101** | 5 arm joints vs a 6-number pose: \(J_b\) is \(6\times5\) (tall), so full-pose IK is a least-squares compromise. Weight position over orientation (scale rows of \(J\) and \(e\)), and damp near the straight-arm singularity. Use the analytic 2R solution for the shoulder–elbow pair as the seed. |
| **ComXim turntable** | Aiming a camera at a plant point is a small IK problem (pan/tilt or arm + table angle). The turntable adds a joint, making the system redundant: use the null space to keep the view angle while dodging leaves. |
| **Crazyflie** | Not an arm, but the same Newton/LM update fits the Flow-deck/AI-deck calibration and pose estimation. |
| **Rover** | Point-the-camera and arm-on-rover tasks add base joints to the chain: whole-body IK with a fat Jacobian and a null space. |
| **Maize/Soybean Plot Builder** | Generate reach targets on synthetic leaves/ears and test IK success rate vs. occlusion before field trials. |

## Current research

- **Generative IK.** Diffusion models sample *many* IK solutions at once instead of one: IKDiffuser for multi-arm robots, with inference-time objectives like manipulability ([Zhang & Jiao, arXiv 2506.13087](https://arxiv.org/html/2506.13087v4)), and GraphDiff-IK, a graph diffusion model built from the robot's URDF for single/dual-arm and torso robots ([Huang et al., arXiv 2606.00086](https://arxiv.org/pdf/2606.00086)).
- **GPU-parallel IK.** cuRobo poses motion generation as parallel optimization on the GPU and reports a collision-free IK solver at over 7000 queries/s ([Sundaralingam et al., arXiv 2310.17274](https://arxiv.org/abs/2310.17274v2)) — the many-seed fix for Newton's local nature.
- **Singularity-aware solvers.** Analytically informed IK computes a small perturbation off a singular pose from a local motion model, then hands it to a numerical solver; tested on a 7-DOF Kuka iiwa ([Müller, arXiv 2412.20409](https://arxiv.org/abs/2412.20409v1)). Classic background: selectively damped least squares ([Buss & Kim](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf)).
- **Agricultural redundancy.** Redundant manipulators for crops use the null space to reach around stems and leaves, e.g. [Modelling and Simulation of a Redundant Agricultural Manipulator with Virtual Prototyping](https://journal.umy.ac.id/index.php/jrc/article/view/17121).

## Challenges

1. **Multiple branches.** A 6R arm can have up to 16 solutions; choosing the wrong one mid-motion causes a 180° wrist flip.
2. **Choosing \(\lambda\).** Too small: blow-up near singularities (the widget's Newton took steps over 50 rad on an unreachable target). Too large: slow, linear convergence (λ = 1 needed ~58 iterations vs 5 for Newton to reach 10⁻⁶).
3. **Joint limits and collisions** turn IK into constrained optimization (QP), not one linear solve.
4. **Units in the error.** \(\omega_b\) is in rad, \(v_b\) in m; one norm mixes them. Weighting is a choice, and it changes the answer for a 5-joint arm.
5. **Real-time with many seeds** on embedded hardware (Jetson-class) for reactive grasping in moving canopies.

## Next five years

- **Learned seeds + exact refinement**: a network or diffusion model proposes diverse \(\theta\); a few damped Newton steps polish them to machine precision.
- **IK inside policies**: VLA policies output end-effector targets; differentiable IK layers let gradients flow from task loss to joints (Track D).
- **Whole-body IK** for mobile manipulators (rover + arm, drone + arm) as one fat-Jacobian, null-space-rich problem.
- **Phenotyping and harvesting**: IK that reaches *through* canopy gaps found by 3D reconstruction (NeRF/3DGS of the plot), choosing the solution branch that avoids stems.

## Check yourself

<div class="quiz" data-q="m05-q1" data-answer="c" data-why="cos θ₂ = (1.8² − 1 − 0.64)/1.6 = 1, so θ₂ = 0: one solution, arm straight, det J = 0.">
<p>A 2R arm with L₁ = 1.0, L₂ = 0.8 must reach a point 1.8 from the base. How many IK solutions?</p>
<ol><li data-k="a">0</li><li data-k="b">2, elbow up and elbow down</li><li data-k="c">1, and the arm is at a singularity</li><li data-k="d">infinitely many</li></ol>
</div>

<div class="quiz" data-q="m05-q2" data-answer="a" data-why="DLS gain σ/(σ² + λ²) goes to 0 as σ → 0, while the pseudoinverse gain 1/σ goes to infinity.">
<p>As a singular value σ of J goes to 0, damped least squares applies a gain in that direction that:</p>
<ol><li data-k="a">goes to 0</li><li data-k="b">goes to infinity, like 1/σ</li><li data-k="c">stays at 1/λ</li><li data-k="d">flips sign</li></ol>
</div>

<div class="quiz" data-q="m05-q3" data-answer="d" data-why="By the push-through identity, β̂ = Xᵀ(XXᵀ + λI)⁻¹y needs only the 500 × 500 line-space solve — the GBLUP form.">
<p>Ridge regression with 500 lines and 50 000 markers. The cheapest exact way to get the marker effects inverts a matrix of size:</p>
<ol><li data-k="a">50 000 × 50 000</li><li data-k="b">50 000 × 500</li><li data-k="c">none — ridge has no closed form</li><li data-k="d">500 × 500</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: inverse kinematics is reaching for a cup in the dark — measure the miss as a twist, turn it into joint changes with the Jacobian, and reach cautiously (damped least squares, which is ridge regression) when the arm is stretched.</div>

**Carries over to:** [M4](m04-velocity-kinematics.md) (the Jacobian it inverts), [M6](m06-closed-chains.md) (forward kinematics of parallel robots is solved the same Newton way), [M8](m08-trajectory-generation.md) (IK along a path), [M10](m10-robot-control.md) (task-space control uses \(J^\dagger\)), [Math bridges](connections.md).
