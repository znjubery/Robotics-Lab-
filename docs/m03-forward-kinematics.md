# M3 · Forward kinematics

<span class="md-tag">Lynch & Park Ch 4</span> <span class="md-tag">Used in: every arm pose, every turntable scan, IK (M5), Jacobians (M4)</span> <span class="md-tag">Tracks: D (physical AI), turntable station</span>

> **Hook:** Lay the arm out straight, mark where each hinge is, then fold the hinges one by one — the tip pose is just the product of those folds: \(T = e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M\).

## Intuition first

**The carpenter's folding ruler.** Unfold a wooden folding ruler flat on a table. That flat shape is the **home configuration**. Its hinges sit at known marks: 0 cm, 100 cm, 180 cm. Now fold it, starting from the hinge nearest the tip. While you fold that last hinge, the other hinges are still flat, so the last hinge is *exactly where you marked it*. Then fold the next hinge in: it swings the already-folded tip piece around with it, as one rigid lump. Work your way back to the base.

That is the whole chapter, told as one story:

1. **Lay it flat** → home pose \(M\) of the tip frame \(\{b\}\).
2. **Mark the hinges** → one screw axis \(\mathcal S_i\) per joint, read off the flat drawing (M2's corkscrew, with pitch 0).
3. **Fold from tip to base** → multiply exponentials \(e^{[\mathcal S_i]\theta_i}\); each one carries everything beyond it.
4. **Read the tip** → \(T(\theta)\in\SE(3)\).

## Problem → math basic → move → result

1. **Problem.** Given joint values \(\theta=(\theta_1,\dots,\theta_n)\), where is the tool, and which way does it point?
2. **Math basic.** Each revolute joint is a zero-pitch screw, and a screw motion is \(e^{[\mathcal S]\theta}\in\SE(3)\) (M2).
3. **Move.** Describe every joint axis once, in the fixed frame \(\{s\}\), at the home pose. Compose the screw motions in chain order and apply them to \(M\).
4. **Result.** The **product of exponentials (PoE)** formula: \(T(\theta) = e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M\). It needs \(n\) screw axes plus one \(M\), and no frame on every link.

## Part 1 — The product of exponentials, space form

Start with only the last joint moving. The tip frame starts at \(M\), and joint \(n\) applies a screw motion about its fixed axis:

\[
T = e^{[\mathcal S_n]\theta_n}\,M
\]

*Why this move:* left-multiplying by \(e^{[\mathcal S]\theta}\) means "move this pose by a screw motion whose axis is written in \(\{s\}\)" (M2). Because joints \(1,\dots,n-1\) are still at zero, joint \(n\)'s axis is still where it was in the home drawing.

Now move joint \(n-1\). It rigidly carries joint \(n\), the last link and the tip, all as one lump. Its own axis has not moved (only joints *beyond* it moved), so

\[
T = e^{[\mathcal S_{n-1}]\theta_{n-1}}\,e^{[\mathcal S_n]\theta_n}\,M
\]

Repeat down to the base:

\[
\boxed{\ T(\theta) = e^{[\mathcal S_1]\theta_1}\,e^{[\mathcal S_2]\theta_2}\cdots e^{[\mathcal S_n]\theta_n}\,M\ } \qquad\text{(space form)}
\]

*Why fold from the tip inward:* each new fold happens about an axis that has not moved yet. So **every \(\mathcal S_i\) is read off one picture, the home pose**. The order in the product is the physical chain order; matrix products don't commute (M2's widget), so the order matters.

??? question "What if we folded from the base outward instead?"
    Then after joint 1 turns, joint 2's axis has moved, and you would have to recompute it from \(\theta_1\) before you could use it. You'd get the same answer, but with extra bookkeeping. Folding tip-first lets you skip that: the formula itself carries each axis to its new place. In M4 you will see this hidden "carry the axis along" step come back as \(\Ad_{e^{[\mathcal S_1]\theta_1}\cdots}\mathcal S_i\) — the Jacobian columns.

## Part 2 — Reading screw axes off a drawing

For a revolute joint (pitch \(h=0\)) with unit axis \(\hat\omega_i\) passing through any point \(q_i\) on that axis, all written in \(\{s\}\) at home:

\[
\mathcal S_i = \begin{bmatrix}\hat\omega_i\\ v_i\end{bmatrix},\qquad v_i = -\hat\omega_i\times q_i
\]

For a prismatic joint: \(\mathcal S_i = (0,\ \hat v_i)\), where \(\hat v_i\) is the sliding direction.

*Why \(v_i=-\hat\omega_i\times q_i\):* \(v_i\) is the velocity of the point of the body that sits at the origin of \(\{s\}\) right now. Spinning at 1 rad/s about a line through \(q_i\), a point \(r\) moves at \(\hat\omega_i\times(r-q_i)\). Put \(r=0\) to get \(-\hat\omega_i\times q_i\). Any \(q_i\) on the line gives the same \(v_i\), because moving \(q_i\) along \(\hat\omega_i\) changes nothing in the cross product.

**Recipe (do it with a pencil):**

1. Draw the robot at \(\theta=0\). Put \(\{s\}\) at the base and \(\{b\}\) at the tool.
2. Write \(M = T_{sb}(0)\): the tool's rotation and position at home.
3. For each joint: write \(\hat\omega_i\) (which way the hinge pin points), pick one point \(q_i\) on the pin, and compute \(v_i=-\hat\omega_i\times q_i\).

Each revolute joint gives one exponential of the M2 form. With \(\theta\) about a line through \(q\):

\[
e^{[\mathcal S]\theta} = \begin{bmatrix} R & (I-R)\,q\\ 0 & 1\end{bmatrix},\qquad R = e^{[\hat\omega]\theta}
\]

*Why this shortcut:* "turn about a line through \(q\)" = shift \(q\) to the origin, rotate, shift back: \(p\mapsto R(p-q)+q = Rp + (I-R)q\). It equals M2's \(G(\theta)v\) form for zero pitch, but needs no series.

## Part 3 — Worked example: planar 3R arm, with numbers

Links \(L_1=1.0,\ L_2=0.8,\ L_3=0.5\) m. Home: the arm lies straight along \(x\). All joint axes point along \(z\) (out of the page). Joints sit at \(x=0,\,1.0,\,1.8\); the tool sits at \(x=2.3\).

**Home pose:**

\[
M = \begin{bmatrix} 1&0&0&2.3\\ 0&1&0&0\\ 0&0&1&0\\ 0&0&0&1\end{bmatrix}
\]

**Screw axes** (\(\hat\omega=\hat z\), \(q_i=(x_i,0,0)\), so \(v_i=-\hat z\times(x_i\hat x) = -x_i\hat y\)):

| Joint | \(\hat\omega_i\) | \(q_i\) | \(v_i=-\hat\omega_i\times q_i\) | \(\mathcal S_i\) |
|---|---|---|---|---|
| 1 | \((0,0,1)\) | \((0,0,0)\) | \((0,0,0)\) | \((0,0,1,\ 0,0,0)\) |
| 2 | \((0,0,1)\) | \((1,0,0)\) | \((0,-1,0)\) | \((0,0,1,\ 0,-1,0)\) |
| 3 | \((0,0,1)\) | \((1.8,0,0)\) | \((0,-1.8,0)\) | \((0,0,1,\ 0,-1.8,0)\) |

**One fold, by hand.** Take \(\theta_3=-90^\circ\). Then \(R=\begin{bmatrix}0&1\\-1&0\end{bmatrix}\) in the plane, and \((I-R)q_3 = (1.8,0)-(0,-1.8) = (1.8,\ 1.8)\). So the last hinge maps the home tip \((2.3,0)\) to \(R(2.3,0)+(1.8,1.8) = (0,-2.3)+(1.8,1.8) = (1.8,-0.5)\): the last link now points straight down from the third joint, as it should.

**All three.** For \(\theta=(30^\circ,60^\circ,-90^\circ)\), multiplying the three exponentials and \(M\) gives

\[
T = \begin{bmatrix} 1&0&0&1.366\\ 0&1&0&1.300\\ 0&0&1&0\\ 0&0&0&1\end{bmatrix}
\]

**Cross-check with plain trig:** \(x = 1.0\cos30^\circ + 0.8\cos90^\circ + 0.5\cos0^\circ = 0.866+0+0.5 = 1.366\); \(y = 0.5+0.8+0 = 1.300\); heading \(\phi=30+60-90 = 0^\circ\). They match.

<div class="lab-widget" data-widget="m03-poe"></div>

Try: start from all zeros and confirm \(T=M\). Then move only θ₃: the tip circles the red mark S₃, which never moves. Now move θ₁ and notice the PoE readout still agrees with trig to about 10⁻¹⁶, even though the drawn S₂, S₃ marks stay at home.

## Part 4 — Body form

Now write each joint axis in the **tool** frame \(\{b\}\) at home, \(\mathcal B_i = \Ad_{M^{-1}}\mathcal S_i\). Using \(M e^{[\mathcal B]\theta}M^{-1} = e^{[\Ad_M\mathcal B]\theta}\) on each factor (insert \(M^{-1}M\) between them):

\[
\boxed{\ T(\theta) = M\,e^{[\mathcal B_1]\theta_1}\,e^{[\mathcal B_2]\theta_2}\cdots e^{[\mathcal B_n]\theta_n}\ } \qquad\text{(body form)}
\]

*Why this move:* right-multiplying moves a pose about an axis written in its own body frame (M2). Here the story runs the other way: fold from the **base** outward, and each axis is described as seen by the tool. For the 3R arm, \(\{b\}\) sits at \(x=2.3\), so joint \(i\) sits at \(x_i-2.3\) in \(\{b\}\): \(\mathcal B_1=(0,0,1,\ 0,2.3,0)\), \(\mathcal B_2=(0,0,1,\ 0,1.3,0)\), \(\mathcal B_3=(0,0,1,\ 0,0.5,0)\). The widget checks this form too.

Use space form when your sensors and targets live in the world (a camera on a tripod). Use body form when they live on the tool (a wrist camera). M4 builds the two Jacobians from these two forms.

## Part 5 — Why PoE needs no link frames: comparison with Denavit–Hartenberg

**Denavit–Hartenberg (DH)** attaches a frame to *every* link under strict rules, then describes each link-to-link step with 4 numbers \((\alpha_{i-1}, a_{i-1}, d_i, \phi_i)\):

\[
T = T_{01}(\theta_1)\,T_{12}(\theta_2)\cdots T_{n-1,n}(\theta_n),\qquad T_{i-1,i} = \mathrm{Rot}_x(\alpha_{i-1})\,\mathrm{Trans}_x(a_{i-1})\,\mathrm{Trans}_z(d_i)\,\mathrm{Rot}_z(\theta_i)
\]

| | DH | PoE |
|---|---|---|
| Frames needed | one per link, placed by rules | only \(\{s\}\) and \(\{b\}\) |
| Numbers per joint | 4 (minimal) | 6 (screw axis), redundant but direct |
| How you get them | follow placement rules carefully | read \(\hat\omega_i, q_i\) off one drawing |
| Revolute vs prismatic | different rule sets | same formula, \(\mathcal S=(0,\hat v)\) |
| Nearly parallel axes | the common normal jumps: small build errors become large parameter changes | axes change smoothly with build errors |

*Why PoE avoids link frames:* every \(\mathcal S_i\) is written in the same frame \(\{s\}\), so nothing needs to be chained frame-to-frame. The cost is 6 numbers per joint instead of 4, but they are direct physical facts (pin direction, one point on the pin).

??? warning "Nearly parallel axes break DH calibration"
    The SO-ARM101's shoulder, elbow and wrist-pitch pins are parallel by design. Build a real one and they are off by a fraction of a degree. In DH, the common normal between two nearly parallel lines can slide far along the axis when the angle between them changes slightly. So a tiny build error becomes a big jump in \(d_i\): the identification problem is ill-conditioned. PoE parameters are just the measured lines, so they change smoothly. This is the main reason PoE is popular for **kinematic calibration**.

## Part 6 — URDF, briefly

ROS, MuJoCo, LeRobot and Isaac describe robots in **URDF** (an XML file). Each `<joint>` has an `<origin xyz rpy>` (the fixed transform from parent link to child link at \(\theta=0\)) and an `<axis xyz>` (the pin direction, in the child frame). URDF is a chain of link frames, like DH, but with free placement instead of DH's rules.

**URDF → PoE in three lines:**

1. Multiply the `origin` transforms from base to joint \(i\) to get that joint frame at home, \(T_{s,i}(0)\).
2. \(\hat\omega_i = R_{s,i}(0)\,\text{axis}_i\), and \(q_i = p_{s,i}(0)\) (the joint frame origin lies on the pin).
3. \(M\) = product of all origins down to the tool link.

## Part 7 — Your hardware as PoE

**Turntable scanner = 1-joint PoE.** The ComXim turntable has one revolute joint with vertical axis \(\hat\omega=\hat z\) through the platter centre \(q_c\). A point on the plant, or a frame stuck to the pot:

\[
T_{\text{pot}}(\theta) = e^{[\mathcal S]\theta}\,M_{\text{pot}},\qquad \mathcal S = (\hat z,\ -\hat z\times q_c)
\]

Seen from the fixed camera, the plant turning by \(\theta\) is the same as the camera orbiting by \(-\theta\): camera pose in the pot frame \(= T_{\text{pot}}(\theta)^{-1}\,T_{\text{cam}}\). Calibrate \(\hat\omega\) and \(q_c\) once and every view's pose follows. That gives NeRF/3DGS a clean pose initializer.

**SO-ARM101 = 5-joint PoE.** Five revolute joints plus a gripper: base yaw (pin \(\hat z\)), shoulder lift, elbow flex and wrist flex (three parallel horizontal pins), and wrist roll (pin along the forearm). Structure only; the lengths in the widget are **example values** (\(d_1=0.10,\ L_2=0.11,\ L_3=0.13,\ L_4=0.10\) m), not measured ones.

<div class="lab-widget" data-widget="m03-so101"></div>

Try: move only θ₂, θ₃, θ₄ — the arm stays in one vertical plane, because their pins are parallel. Then θ₅ spins the tool frame about the forearm without moving the tip. Five joints give at most five independent motions of a six-number pose.

## Math bridges

| Here | DiffSim / FVEM | Genomics-Lab | 3D vision |
|---|---|---|---|
| \(T=e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M\) | operator splitting: a time step as a product of sub-step exponentials \(e^{A\Delta t}e^{B\Delta t}\) | — | chaining camera extrinsics; a turntable scan is a 1-term PoE |
| Home config \(M\) + changes | reference configuration + displacement in solid mechanics (Lagrangian description) | baseline/intercept + effects | canonical space in NeRF/3DGS, deformed by per-frame transforms |
| Screw axis read off one drawing, all in \(\{s\}\) | global coordinates vs element-local frames (assembly) | — | world frame for all cameras in SfM, instead of camera-to-camera chains |
| DH ill-conditioned for parallel axes | choosing a well-conditioned parameterization of an inverse problem | collinear markers (LD) make effect estimates unstable | gauge choice in bundle adjustment |
| PoE calibration: fit \(\mathcal S_i, M\) to measured poses | parameter identification with a forward model | least-squares fit of effects | hand–eye calibration \(AX=XB\) |

## In your robots

| Where | What this module gives you |
|---|---|
| **ComXim turntable scanner** | A 1-joint PoE: \(T(\theta)=e^{[\mathcal S]\theta}M\). One axis calibration gives every view pose for the remote AI-testbed scans. |
| **SO-ARM101 (LeRobot)** | Five revolute screw axes + \(M\). LeRobot and MuJoCo models give the URDF; convert it to PoE with the three-line recipe to get your own FK for checks and calibration. |
| **Crazyflie / under-canopy drone** | No joints, but a gimbal or a down-facing camera mount is a short PoE from the body frame. |
| **Jetson Orin rover** | A pan–tilt camera head is a 2-joint PoE; the camera pose = rover pose × PoE. |
| **Maize/Soybean Plot Builder** | A plant can be modelled as a chain: stalk nodes as joints, leaves as links. Leaf angle sweeps = changing \(\theta\) in a PoE, a quick way to generate posed synthetic plants. |

## Current research

- **Calibration with PoE.** Local PoE models identified at several poses, blended over shoulder/elbow angles with a Fourier basis, cut the maximum positioning error by more than half on two 6-DoF industrial arms ([Lu, He, Julius, Wen, arXiv 2510.19962](https://arxiv.org/abs/2510.19962v1)).
- **Converting between DH, PoE and URDF automatically.** The RobKin Interpreter turns serial-robot descriptions into DH, roll–pitch–yaw or PoE form and writes URDF ([Huczala et al., arXiv 2204.02629](https://arxiv.org/abs/2204.02629v2)).
- **FK at GPU scale.** cuRobo runs forward and inverse kinematics for thousands of configurations in parallel on the GPU, for motion generation ([cuRobo](https://curobo.org)); differentiable FK layers let networks train through the kinematic chain ([arXiv 2301.09954](https://arxiv.org/pdf/2301.09954)).
- **Arm design for agriculture.** A 2025 CMU thesis automates the design of modular, reconfigurable arms (which modules, in which layout — i.e. where the screw axes go) for cluttered crops, and applies it to arm placement for a dual-arm pepper harvester ([Guri, CMU RI](https://www.ri.cmu.edu/event/design-optimization-of-modular-manipulators-for-manipulation-in-cluttered-agricultural-environments/)).

## Challenges

1. **Cheap arms are not their CAD.** Hobby servos like the STS3215 on the SO-ARM101 have backlash, and printed links flex. A 0.5° error at the shoulder moves a tool 0.35 m away by about 3 mm. That is enough to miss a soybean pod.
2. **Zero offsets.** The encoder "0" is rarely the drawing's \(\theta=0\). An offset \(\delta_i\) just shifts \(\theta_i\to\theta_i+\delta_i\), but it must be estimated.
3. **Frame conventions across tools.** URDF, MuJoCo XML and LeRobot calibration files each define home, axis signs and units their own way. One flipped `axis` sign silently mirrors a joint.
4. **Turntable axis wobble.** If the platter axis tilts by even 0.2°, a 1-joint PoE with a perfectly vertical axis puts the top of a 1 m maize plant about 3.5 mm off per side. That is visible in a 3DGS reconstruction.

## Next five years

- **Self-calibrating arms** that re-estimate their screw axes from wrist-camera video during normal work, instead of in a separate lab session.
- **Learned + analytic FK hybrids**: PoE for the rigid part plus a small network for flex and backlash in low-cost arms.
- **Design by differentiable FK**: optimizing joint layouts (screw axes) for a task, the way topology optimization designs a part.
- **Phenotyping stations as calibrated robots**: turntables, gantries and arm-mounted cameras sharing one PoE model, so every image of a maize or soybean plant comes with an exact, traceable pose.

## Check yourself

<div class="quiz" data-q="m03-q1" data-answer="c" data-why="v = −ω̂ × q = −(ẑ × 2x̂) = −2ŷ, so S = (0,0,1, 0,−2,0).">
<p>A revolute joint has axis ω̂ = ẑ passing through q = (2, 0, 0). Its screw axis S = (ω̂, v) is:</p>
<ol><li data-k="a">(0,0,1, 2,0,0)</li><li data-k="b">(0,0,1, 0,2,0)</li><li data-k="c">(0,0,1, 0,−2,0)</li><li data-k="d">(0,0,1, 0,0,2)</li></ol>
</div>

<div class="quiz" data-q="m03-q2" data-answer="a" data-why="At θ = 0 every exponential is the identity, so T equals the home pose M.">
<p>In T = e<sup>[S₁]θ₁</sup>⋯e<sup>[Sₙ]θₙ</sup>M, what is T when all θᵢ = 0?</p>
<ol><li data-k="a">M</li><li data-k="b">the identity</li><li data-k="c">the sum of all Sᵢ</li><li data-k="d">undefined</li></ol>
</div>

<div class="quiz" data-q="m03-q3" data-answer="d" data-why="Every space-form axis is read in {s} at the home pose, so no per-link frames are required.">
<p>The main reason PoE needs no link frames, unlike Denavit–Hartenberg, is:</p>
<ol><li data-k="a">it only works for planar arms</li><li data-k="b">it uses fewer numbers per joint</li><li data-k="c">it ignores joint order</li><li data-k="d">every screw axis is written in one fixed frame at the home pose</li></ol>
</div>

<div class="lab-progress" data-lab-progress></div>

<div class="oneliner">One line: lay the arm out straight, mark each hinge as a screw axis, and the tip pose is the product of the hinge folds times the home pose — T = e^[S₁]θ₁ ⋯ e^[Sₙ]θₙ M.</div>

**Carries over to:** [M4](m04-velocity-kinematics.md) (Jacobian columns are the carried screw axes), [M5](m05-inverse-kinematics.md) (inverse kinematics solves PoE for θ), [M7](m07-dynamics.md) (dynamics built on the same screws), [Track D](t-d-physical-ai.md) (policies that output poses for the SO-ARM101).
