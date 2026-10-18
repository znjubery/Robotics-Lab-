# M10 · Robot control

<span class="md-tag">Lynch & Park Ch 11</span> <span class="md-tag">Used in: every servo, every flight, every gentle grasp</span> <span class="md-tag">Tracks: B, C, D</span>

> **Hook:** A robot controller is a car's suspension, made of software: a virtual spring pulls each joint toward where it should be, a virtual damper stops it from bouncing, and choosing those two numbers decides whether the arm is sluggish, snappy, or shaking.

## Intuition first

**The car's shock absorber.** Drive over a bump. The **spring** pushes the wheel back to where it belongs; the **damper** (shock absorber) eats the bounce. A soft spring with worn dampers: the car floats and bounces four times. A stiff spring with good dampers: one quick, firm return. Too much damping: the car feels like it's driving through honey.

A feedback controller builds exactly this, in software, around the **error** \(e = \theta_d - \theta\):

- the **P gain** \(K_p\) is the spring stiffness,
- the **D gain** \(K_d\) is the damper,
- the **I gain** \(K_i\) is a slow hand that pushes harder the longer the car sits too low (gravity sag),
- and if the car's mass is known, you can **pre-load** the spring to carry it — that's feedforward.

One story thread for the page: **spring + damper on the error** for one joint → for a whole arm (computed torque) → in the task space → and finally as a spring + damper the *world* feels when it pushes back (impedance control).

## Problem → math basic → move → result

1. **Problem.** M8 gave a trajectory \(\theta_d(t)\). Motors produce torque, not position, and the arm has inertia, friction and gravity. Make \(\theta(t)\) follow \(\theta_d(t)\).
2. **Math basic.** Newton for one joint: \(I\ddot\theta + b\dot\theta + mgl\sin\theta = \tau\). For many joints, M7's \(M(\theta)\ddot\theta + h(\theta,\dot\theta) = \tau\).
3. **Move.** Choose \(\tau\) so the **error** obeys a linear second-order ODE we pick: \(\ddot e + K_d\dot e + K_p e = 0\).
4. **Result.** Tuning becomes picking two numbers: natural frequency \(\omega_n\) and damping ratio \(\zeta\), with closed-form overshoot and settling time.

## Part 1 — Error dynamics: everything is a second-order ODE

Define \(e = \theta_d - \theta\). A good controller makes \(e\to 0\). The most common target is a damped oscillator:

\[
\ddot e + 2\zeta\omega_n\,\dot e + \omega_n^2\,e = 0
\]

*Why this move:* it is the one ODE every engineer already knows (mass–spring–damper, RLC circuit, FEM single mode). Try \(e = e^{st}\):

\[
s^2 + 2\zeta\omega_n s + \omega_n^2 = 0 \quad\Longrightarrow\quad s_{1,2} = -\zeta\omega_n \pm \omega_n\sqrt{\zeta^2 - 1}
\]

| \(\zeta\) | Roots | Response |
|---|---|---|
| \(0\) | \(\pm i\omega_n\) | rings forever |
| \(0<\zeta<1\) | complex, real part \(-\zeta\omega_n\) | **underdamped**: overshoot, then settles |
| \(1\) | double root \(-\omega_n\) | **critically damped**: fastest with no overshoot |
| \(>1\) | two negative reals | **overdamped**: slow, no overshoot |

**Overshoot.** For a step with \(0<\zeta<1\), the response is \(\theta(t) = \theta_d\big[1 - e^{-\zeta\omega_n t}(\cos\omega_d t + \tfrac{\zeta}{\sqrt{1-\zeta^2}}\sin\omega_d t)\big]\) with \(\omega_d = \omega_n\sqrt{1-\zeta^2}\). The first peak is at \(t_p = \pi/\omega_d\), so

\[
M_p = e^{-\pi\zeta/\sqrt{1-\zeta^2}}
\]

**Settling time.** The envelope \(e^{-\zeta\omega_n t}\) falls to 2% when \(\zeta\omega_n t = \ln 50 \approx 3.9\):

\[
t_s \approx \frac{4}{\zeta\omega_n}
\]

Numbers to remember: \(\zeta=0.7\Rightarrow M_p\approx 4.6\%\); \(\zeta = 0.5\Rightarrow 16\%\); \(\zeta=0.36\Rightarrow 30\%\).

## Part 2 — PD and PID on one joint

**The plant** (example values, about a 0.2 kg link at 0.3 m): inertia \(I = 0.05\ \text{kg·m}^2\), viscous friction \(b = 0.02\ \text{N·m·s}\), gravity torque \(mgl = 0.6\ \text{N·m}\), \(\theta = 0\) hanging down.

\[
I\ddot\theta + b\dot\theta + mgl\sin\theta = \tau
\]

**Step 1 — PD, no gravity.** Use \(\tau = K_p e - K_d\dot\theta\) (D on the measurement, so a step in \(\theta_d\) gives no "derivative kick"). With \(\theta_d\) constant, \(\ddot e = -\ddot\theta\):

\[
I\ddot e + (b + K_d)\dot e + K_p e = 0
\]

*Why this move:* divide by \(I\) and match to Part 1's standard form term by term.

\[
\boxed{\ \omega_n = \sqrt{\frac{K_p}{I}},\qquad \zeta = \frac{b + K_d}{2\sqrt{K_p I}}\ }
\]

**Check with the widget defaults** (\(K_p = 4,\ K_d = 0.3\), gravity off): \(\omega_n = \sqrt{4/0.05} = 8.94\) rad/s, \(\zeta = 0.32/(2\sqrt{0.2}) = 0.358\), so \(M_p = 30.0\%\) and \(t_s \approx 4/(0.358\cdot 8.94) = 1.25\) s. The RK4 simulation gives 30.2% and 1.22 s. ✔

**Step 2 — add gravity.** At steady state \(\dot\theta = \ddot\theta = 0\), so

\[
K_p\,e_{ss} = mgl\sin\theta_{ss}
\]

*Why this move:* the spring must stretch to hold the weight, so the arm **sags**. At \(\theta_d = 60^\circ\), \(K_p = 4\): \(e_{ss}\approx 0.6\cdot\sin 53^\circ/4 \approx 0.12\) rad \(\approx 6.9^\circ\) (the widget reports 6.87°). Doubling \(K_p\) halves the sag but makes the joint stiffer and louder.

Near \(\theta_d\), gravity also acts as an extra spring of stiffness \(mgl\cos\theta_d\), so the effective stiffness is \(k = K_p + mgl\cos\theta_d\).

**Step 3 — PID.** Add \(K_i\int e\,dt\). At steady state the integral can hold any constant torque while \(e = 0\): sag gone. Differentiate the closed loop once (linear, no gravity) to get a cubic:

\[
I s^3 + (b+K_d)s^2 + K_p s + K_i = 0
\]

Routh–Hurwitz says all roots have negative real part iff

\[
(b + K_d)\,K_p > I\,K_i
\]

With the defaults: \(K_i < 0.32\cdot 4/0.05 = 25.6\). Push \(K_i\) toward that and the response rings more and more.

**Step 4 — feedforward instead of integral.** If you know the model, cancel gravity directly:

\[
\tau = \underbrace{mgl\sin\theta_d}_{\text{feedforward}} + K_p e - K_d\dot\theta
\]

*Why this move:* the weight is carried by the model, not by stretching the spring, so \(e_{ss}=0\) instantly and \(K_p\) can stay small (gentle). This is the "pre-loaded suspension".

<div class="lab-widget" data-widget="m10-pid"></div>

Try: with gravity on and \(K_i = 0\), read the sag in the final error. Tick **feedforward**: error goes to ~0°. Untick it, raise \(K_i\) to 5: sag removed, overshoot up. Now drop the **torque limit** to 1 N·m and watch the integral wind up; tick **anti-windup** and the overshoot falls (38% → 14% in the simulation).

??? warning "Integral windup"
    When the motor saturates (\(\vert\tau\vert = \tau_{\max}\)), the error stays large, so \(\int e\,dt\) keeps growing even though more torque is impossible. When the joint finally arrives, the stored integral pushes it far past the target. **Fix:** stop integrating while saturated (clamping, used in the widget), or bleed the integral back ("back-calculation"). Every real servo and the Crazyflie firmware limit their integrators for this reason.

??? question "What if we used D on the error instead of on the measurement?"
    \(\tau = K_p e + K_d\dot e\) with a step in \(\theta_d\) gives \(\dot e = \infty\) at the step: a spike of torque ("derivative kick"). Putting D on \(-\dot\theta\) gives the same damping with no spike. The Crazyflie PID computes its D term from the measured value for exactly this reason ([Bitcraze docs](https://bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/)).

## Part 3 — Computed torque: feedforward + feedback for the whole arm

For \(n\) joints (M7): \(M(\theta)\ddot\theta + h(\theta,\dot\theta) = \tau\), where \(h\) = Coriolis + gravity + friction. Choose

\[
\tau = M(\theta)\Big(\ddot\theta_d + K_p e + K_i\!\int e\,dt + K_d\dot e\Big) + h(\theta,\dot\theta)
\]

Plug in, cancel \(h\), and multiply by \(M^{-1}\) (always invertible):

\[
\ddot e + K_d\dot e + K_p e + K_i\!\int e\,dt = 0
\]

*Why this move:* the model turns a coupled, nonlinear, configuration-dependent system into \(n\) **independent** copies of Part 2's ODE. Pick \(K_p = \omega_n^2 I\), \(K_d = 2\zeta\omega_n I\) once, and every joint behaves the same in every pose.

- \(M\ddot\theta_d + h\) is the **feedforward** (what the model says is needed).
- \(M(K_p e + K_d\dot e)\) is the **feedback** (fixes what the model got wrong).
- With a wrong model, the leftover error acts like a disturbance; feedback gains set how much it hurts.

## Part 4 — Task-space control

Often the error you care about is in the task (end-effector) frame, e.g. "keep the gripper 2 cm above the leaf". With task-space twist error \(\mathcal V_e\) and Jacobian \(J\) (M4):

\[
\tau = J^\top(\theta)\Big(\Lambda(\theta)\big(\dot{\mathcal V}_d + K_p X_e + K_d \mathcal V_e\big) + \eta(\theta,\mathcal V)\Big)
\]

where \(\Lambda = (J M^{-1} J^\top)^{-1}\) is the **task-space mass matrix** and \(X_e\) is the pose error (e.g. \(\log(T^{-1}T_d)\) from M2). *Why this move:* \(\tau = J^\top\mathcal F\) (M4) maps a desired task wrench back to joint torques; \(\Lambda\) makes the end effector feel like a mass you chose. Near singularities \(\Lambda\) blows up — the same warning as damped least-squares IK in M5.

## Part 5 — Force, impedance, admittance, hybrid

**Force control.** To press a probe on a stalk with force \(f_d\): \(\tau = J^\top\big(f_d + K_{fp}(f_d - f) + K_{fi}\!\int (f_d - f)\,dt\big) + g(\theta)\). Needs a force/torque sensor or good current sensing.

**Impedance control** — make the end effector *feel like* a spring–damper–mass to the world:

\[
M_d\ddot x + B\dot x + K(x - x_d) = -f_{\text{ext}}
\]

*Why this move:* you don't control position *or* force; you control the **relationship** between them. In free space it tracks \(x_d\); against a wall it pushes with \(f\approx K\,\Delta x\). The shock absorber is now between the robot and the world.

| | Measures | Commands | Best for |
|---|---|---|---|
| **Impedance** | motion | force/torque | torque-controlled, backdrivable arms |
| **Admittance** | force | motion (position/velocity) | stiff, position-controlled arms (most hobby servos) |

<div class="lab-widget" data-widget="m10-impedance"></div>

Try: set the target 5 cm *past* the wall. Steady force ≈ K·Δx, whatever the wall stiffness. Lower **damping** to 0 and watch the impact ring; raise **stiffness** and the peak force jumps — soft K is what makes contact with a plant safe.

**Hybrid motion–force control.** Split the task directions with a selection matrix \(P\): control **force** along the surface normal and **motion** along the surface. Example: slide a leaf clip along the midrib while pressing 1 N into it. The two subspaces must be orthogonal in the right (wrench/twist) sense — the pairing \(\mathcal V^\top\mathcal F = 0\) from M2.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| Error ODE \(I\ddot e + c\dot e + k e = 0\) | one modal equation of \(M\ddot u + C\dot u + Ku = 0\) (Rayleigh damping per mode) | — | — |
| Closed-loop roots \(s = -\zeta\omega_n \pm i\omega_d\) | eigenvalues of the semi-discrete system; a digital controller at step \(\Delta t\) is stable only if its discrete poles stay inside the unit circle — same rule as explicit time-stepper stability (\(\vert 1 + s\Delta t\vert<1\) for forward Euler) | — | — |
| Computed torque cancels \(M\), \(h\) | mass-matrix preconditioning / modal decoupling | — | — |
| Gain tuning \(K_p,K_i,K_d\) | — | hyperparameter tuning (\(\lambda\) in ridge, by CV or Bayesian optimization) | tuning learning rate / momentum (a discrete PD on the loss) |
| Impedance \(M_d\ddot x + B\dot x + Kx = f\) | penalty contact: wall = stiff spring \(k_w\), same as penalty BCs in FEM | — | — |
| Feedback + model (Track C: LQR \(\leftrightarrow\) Kalman duality) | data assimilation | BLUP = Gaussian conditioning (Kalman update) | visual–inertial fusion |

## In your robots

| Where | What this module gives you |
|---|---|
| **SO-ARM101 (Feetech STS3215 servos)** | Each servo runs its **own internal position PID**; LeRobot sends position targets. LeRobot's SO-101 setup writes conservative gains P = 16 (default 32), I = 0, D = 32 to avoid shaking ([LeRobot SO-101 notes](https://mintlify.wiki/huggingface/lerobot/robots/so-101)). Lower P = softer spring = more gravity sag — exactly Part 2, Step 2. |
| **SO-ARM101 compliance** | No joint torque sensor, so impedance in the strict sense is not available; an **admittance**-style outer loop (estimate force from load/current, move the target) is the realistic route. |
| **Crazyflie 2.x** | Default controller is a **cascaded PID**: position/velocity at 100 Hz → attitude at 500 Hz → attitude rate at 500 Hz → motors; D computed on the measurement ([Bitcraze](https://bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/)). Inner loops must be faster than outer ones — see Track B. |
| **Planned under-canopy drone** | Same cascade; gusts inside the canopy are the disturbance the I term and feedforward must reject. |
| **Jetson Orin rover** | Wheel-speed PID inside, path tracking (pure pursuit / Stanley) outside (M12, Track C). |
| **ComXim turntable** | One-axis position control; settling time per step sets scan throughput (step only when \(t_s\) has passed to avoid blur). |

## Current research

- **Learned compliance.** Diffusion models that output stiffness and damping, not just positions, for contact-rich tasks on a KUKA iiwa ([Geiger, Asfour, Hogan, Lachner, 2025](https://arxiv.org/abs/2509.19696v2)); compliant diffusion policies learned from demonstrations ([arXiv 2410.19235](https://arxiv.org/html/2410.19235v1)).
- **Admittance control in the field.** A vineyard harvesting robot embeds its level of autonomy in a **variable admittance** controller and switches to zero-stiffness hand guiding when leaves hide the peduncle ([Palmieri et al., IROS 2024, CANOPIES](https://iris.unicas.it/bitstream/11580/113404/1/canopies_iros24.pdf)).
- **Contact-aware canopy reaching.** RICE uses tactile feedback to trade off pushing through foliage against avoiding stiff branches ([Parayil et al., 2025](https://arxiv.org/html/2506.10383v1)).
- **Automated PID tuning.** Meta-learning initializes fuzzy gain-scheduled PID gains from robot physical features, then RL refines them, tested on a 9-DOF arm and a 12-DOF quadruped ([Wu & Yu, arXiv 2511.06500](https://arxiv.org/abs/2511.06500v2)).

## Challenges

1. **Model error.** Computed torque is only as good as \(M\), \(h\); payload or friction errors of 20–30% are common on low-cost arms, and they show up as tracking error.
2. **Saturation and windup.** Hobby servos hit their torque limit during fast moves; without anti-windup, overshoot can more than double (38% vs 14% in this page's example).
3. **Sample rate and delay.** A loop at \(f\) Hz with delay \(\Delta t\) loses phase \(\omega\Delta t\); USB/serial round trips of a few ms limit how stiff an outer PC loop can be.
4. **Contact stability.** Stiff impedance against a stiff surface lowers the effective \(\zeta\) (impedance widget defaults: ζ 0.58 in free space → 0.28 in contact), causing chatter; plants are soft *and* variable.
5. **Sensing force cheaply.** Without a force sensor, current-based estimates are noisy and biased by friction.

## Next five years

- **Policies that output gains.** VLA/diffusion policies predicting stiffness and damping alongside targets, with classical loops underneath running at kHz.
- **Differentiable control** inside simulators: tune \(K_p, K_d\) by gradients through the rollout (ties to DiffSim-Lab adjoints).
- **Safety filters** (control-barrier functions) wrapped around learned controllers as a standard layer.
- **Agriculture:** low-cost compliant arms that touch leaves and stalks with regulated force (\(\sim\)1 N) for sensor clips, tissue punches and stalk-strength probes during phenotyping.

## Check yourself

<div class="quiz" data-q="m10-q1" data-answer="b" data-why="ωn = √(Kp/I), so 4× Kp doubles ωn; ζ = (b+Kd)/(2√(Kp·I)) halves.">
<p>For a PD-controlled joint, you multiply Kp by 4 and keep Kd fixed. What happens?</p>
<ol><li data-k="a">ωn doubles and ζ doubles</li><li data-k="b">ωn doubles and ζ halves</li><li data-k="c">ωn is unchanged and ζ halves</li><li data-k="d">ωn quadruples and ζ is unchanged</li></ol>
</div>

<div class="quiz" data-q="m10-q2" data-answer="d" data-why="While the motor is saturated the error stays large, so the integral keeps growing and later causes a big overshoot.">
<p>Integral windup happens because:</p>
<ol><li data-k="a">Kd is too large</li><li data-k="b">the sample rate is too high</li><li data-k="c">gravity feedforward is on</li><li data-k="d">the integral keeps growing while the torque is saturated</li></ol>
</div>

<div class="quiz" data-q="m10-q3" data-answer="a" data-why="Impedance control sets the relation f ≈ K·Δx, so a soft K limits the force for a given position error.">
<p>An impedance-controlled gripper targets 5 cm past a leaf. To keep the contact force small, you should:</p>
<ol><li data-k="a">lower the virtual stiffness K</li><li data-k="b">raise K</li><li data-k="c">set damping B to zero</li><li data-k="d">add an integral term on position</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: control turns tracking error into a spring–damper you design — Kp sets the stiffness, Kd the damping, a model cancels the load, and impedance control lets the world feel that same shock absorber.</div>

**Carries over to:** [M7](m07-dynamics.md) (the \(M\), \(h\) used in computed torque), [M8](m08-trajectory-generation.md) (the \(\theta_d(t)\) being tracked), [M11](m11-grasping-manipulation.md) (contact and grasp forces), [Track B](t-b-aerial.md) (Crazyflie cascaded PID), [Track C](t-c-rover-estimation.md) (Kalman ↔ LQR duality), [Track D](t-d-physical-ai.md) (learned compliant policies).
