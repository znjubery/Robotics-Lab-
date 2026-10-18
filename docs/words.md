# Words

Every term in plain words first, then the precise version. Grows with each module.

| Term | Plain words | Precise | Module |
|---|---|---|---|
| Configuration | where every point of the robot is | a point \(q\in\mathcal C\) | M1 |
| Configuration space (C-space) | the set of all possible robot poses | manifold of dimension = DOF | M1 |
| Degrees of freedom (DOF) | how many numbers you need | dimension of \(\mathcal C\) | M1 |
| Grübler's formula | count freedoms, subtract what joints remove | \(m(N-1-J)+\sum f_i\) | M1 |
| Holonomic constraint | a rule on positions | \(g(q)=0\) | M1 |
| Nonholonomic constraint | a rule on velocity directions that can't be integrated | \(A(q)\dot q=0\), not integrable | M1 |
| Pfaffian form | constraints written linear in velocity | \(A(q)\dot q=0\) | M1 |
| Task space | the coordinates the job cares about | e.g. tip \((x,y,z)\) | M1 |
| Workspace | the part of task space the robot can reach | image of forward kinematics | M1 |
| C-obstacle | the set of poses that collide | \(\{q : \text{robot}(q)\cap\text{obstacle}\ne\emptyset\}\) | M1 |
| Underactuated | fewer motors than DOF | rank of input map < dim \(\mathcal C\) | M1 |
| Torus | donut surface: two wrapping angles | \(S^1\times S^1\) | M1 |
| Rotation matrix | a 3×3 table whose columns are the body's axes | \(R\in\SO(3)\) | M2 |
| \(\SO(3)\) | the set of all 3D rotations | \(R^\top R=I,\ \det R=1\) | M2 |
| Skew-symmetric matrix | matrix version of a cross product | \([\omega]v=\omega\times v\) | M2 |
| Lie algebra \(\so(3)\) | all possible spin "velocities" at the start pose | skew \(3\times3\) matrices | M2 |
| Exponential coordinates | axis × angle, 3 numbers for a rotation | \(\hat\omega\theta\), \(R=e^{[\hat\omega]\theta}\) | M2 |
| Rodrigues' formula | closed form of the rotation exponential | \(I+\sin\theta[\hat\omega]+(1-\cos\theta)[\hat\omega]^2\) | M2 |
| Matrix log | recover axis and angle from \(R\) | \(\theta=\arccos\frac{\operatorname{tr}R-1}{2}\) | M2 |
| Gimbal lock | a 3-angle chart losing one DOF | singularity of Euler angles | M2 |
| Homogeneous transform | rotation + shift in one 4×4 matrix | \(T\in\SE(3)\) | M2 |
| Twist | angular + linear velocity together | \(\mathcal V=(\omega,v)\in\R^6\) | M2 |
| Screw axis | the corkscrew line, as a unit twist | \(\mathcal S=(\hat s,-\hat s\times q+h\hat s)\) | M2 |
| Pitch | slide per radian of turn | \(h\) | M2 |
| Chasles' theorem | every rigid displacement is a screw motion | — | M2 |
| Adjoint map | rewrites a twist in another frame | \(\Ad_T\in\R^{6\times6}\) | M2 |
| Wrench | moment + force together | \(\mathcal F=(m,f)\) | M2 |
| Forward kinematics | joint values in, tool pose out | map \(\theta\mapsto T(\theta)\in\SE(3)\) | M3 |
| Home configuration | the robot laid out at all-zero joints | \(M=T_{sb}(0)\) | M3 |
| Product of exponentials (PoE) | fold each hinge, multiply the folds | \(T=e^{[\mathcal S_1]\theta_1}\cdots e^{[\mathcal S_n]\theta_n}M\) | M3 |
| Space form / body form | axes written in the world frame / in the tool frame | \(\mathcal S_i\) in \(\{s\}\); \(\mathcal B_i=\Ad_{M^{-1}}\mathcal S_i\) in \(\{b\}\) | M3 |
| Body screw axis | a joint axis as seen by the tool at home | \(\mathcal B_i\), with \(T=Me^{[\mathcal B_1]\theta_1}\cdots\) | M3 |
| Denavit–Hartenberg (DH) | 4 numbers per link, frames placed by rules | \(T_{i-1,i}=\mathrm{Rot}_x(\alpha)\mathrm{Trans}_x(a)\mathrm{Trans}_z(d)\mathrm{Rot}_z(\theta)\) | M3 |
| URDF | XML robot description used by ROS, MuJoCo, LeRobot | chain of joint `origin` transforms + `axis` vectors | M3 |
| Kinematic calibration | measuring the real screw axes of a built robot | fit \(\mathcal S_i, M\) (and zero offsets) to measured poses | M3 |
| Space / body Jacobian | joint speeds → tool twist, seen from the world / the tool | \(\mathcal V_s=J_s\dot\theta,\ \mathcal V_b=J_b\dot\theta,\ J_s=\Ad_{T_{sb}}J_b\) | M4 |
| Geometric vs analytic Jacobian | screw-axis columns vs derivatives of chosen pose coordinates | \(J\) vs \(\partial x/\partial\theta\) for a coordinate chart \(x\) | M4 |
| Singularity | a pose where some tool direction can't be moved | \(\operatorname{rank}J(\theta)\) drops | M4 |
| Manipulability ellipse | tip speeds reachable with unit joint speed | \(v^\top(JJ^\top)^{-1}v\le1\) | M4 |
| Force ellipse | tip forces held with unit joint torque | \(f^\top JJ^\top f\le1\), axes \(1/\sigma_i\) | M4 |
| Condition number | how stretched the ellipse is | \(\kappa=\sigma_{\max}/\sigma_{\min}\) | M4 |
| Yoshikawa manipulability | area/volume of the ellipse | \(w=\sqrt{\det JJ^\top}=\prod\sigma_i\) | M4 |
| Statics duality | motor torques for a tool force | \(\tau=J^\top\mathcal F\) (equal power) | M4 |
| Redundancy | more joints than task numbers | \(n>m\), \(J\) has a null space | M4 |
| Null-space motion | joints move, tip stays still | \(\dot\theta\in\ker J\), projector \(I-J^\dagger J\) | M4 |
| Inverse kinematics (IK) | joint angles that put the hand where you want | solve \(T(\theta)=T_{sd}\) for \(\theta\) | M5 |
| Analytic IK | solve with geometry in one shot | closed-form solution (law of cosines, atan2) | M5 |
| Elbow up / elbow down | the two mirror-image ways a 2R arm reaches a point | \(\theta_2=\pm\arccos D\) | M5 |
| Newton–Raphson IK | measure the miss, correct, repeat | \(\theta^{k+1}=\theta^k+J^\dagger e^k\) | M5 |
| Pseudoinverse | the "best" inverse for non-square or rank-deficient matrices | \(J^\dagger=\sum\sigma_i^{-1}v_iu_i^\top\) over \(\sigma_i>0\) | M5 |
| Damped least squares | cautious steps that never blow up | \(\Delta\theta=J^\top(JJ^\top+\lambda^2I)^{-1}e\) | M5 |
| Push-through identity | flip which side you invert | \((A^\top A+\lambda I)^{-1}A^\top=A^\top(AA^\top+\lambda I)^{-1}\) | M5 |
| Null space | joint motions that don't move the hand | \(\{\dot\theta: J\dot\theta=0\}\), projector \(I-J^\dagger J\) | M5 |
| Levenberg–Marquardt | Newton with an adjustable safety brake | \((J^\top J+\lambda I)\delta=-J^\top r\), \(\lambda\) adapted | M5 |
| Closed chain | links forming at least one loop | mechanism whose graph has cycles | M6 |
| Loop-closure equation | going around the loop brings you back | \(\prod_i T_i(\theta)=I\), i.e. \(g(\theta)=0\) | M6 |
| Active / passive joint | motor-driven / free to follow | \(\theta=(\theta_a,\theta_p)\) | M6 |
| Assembly mode | one of the shapes a linkage can take for the same motor angles | a root of the FK equations | M6 |
| Grashof condition | can some link spin all the way round? | \(s+l\le p+q\) | M6 |
| Coupler curve | path traced by a point on the floating link | locus of a coupler point over the crank cycle | M6 |
| Constraint Jacobian | how the loop error changes with joints | \(H=\partial g/\partial\theta\) | M6 |
| Actuator singularity | motors no longer fix the mechanism; it goes floppy | \(H_p\) loses rank | M6 |
| Configuration singularity | end effector loses a direction of motion | end-effector Jacobian loses rank | M6 |
| Stewart–Gough platform | plate on six extendable legs | 6-UPS parallel mechanism, 6 DOF | M6 |
| Delta robot | three-arm fast translator | 3-DOF translational parallel robot | M6 |
| Saddle-point system | equations with a zero block from constraints | KKT matrix \(\begin{bmatrix}K&H^\top\\H&0\end{bmatrix}\) | M6 |
| Forward dynamics | push → motion | given \(\theta,\dot\theta,\tau\), find \(\ddot\theta=M^{-1}(\tau-c-g)\) | M7 |
| Inverse dynamics | motion → push needed | given \(\theta,\dot\theta,\ddot\theta\), find \(\tau\) | M7 |
| Mass matrix | how heavy the arm feels, per pose | SPD \(M(\theta)\) with \(\mathcal K=\tfrac12\dot\theta^\top M\dot\theta\) | M7 |
| Coriolis / centripetal terms | forces made by moving | \(c(\theta,\dot\theta)=C(\theta,\dot\theta)\dot\theta\), quadratic in \(\dot\theta\) | M7 |
| Gravity vector | torque just to hold still | \(g(\theta)=\partial\mathcal P/\partial\theta\) | M7 |
| Lagrangian | kinetic minus potential energy | \(\mathcal L=\mathcal K-\mathcal P\); Euler–Lagrange gives the EOM | M7 |
| Spatial inertia | a body's 6×6 "mass" for twists | \(\mathcal G_b=\mathrm{diag}(\mathcal I_b, mI)\) at the CoM | M7 |
| Recursive Newton–Euler (RNEA) | two sweeps along the chain | outward twists, inward wrenches; \(O(n)\) inverse dynamics | M7 |
| Symplectic integrator | time stepper that keeps energy bounded | preserves phase-space area of \((\theta,p)\) | M7 |
| System identification | measure the robot's masses from data | least squares on \(\tau=Y\pi\) | M7 |
| Path | the road, no clock | \(\theta(s)\), \(s\in[0,1]\) | M8 |
| Trajectory | road plus timetable | \(\theta(s(t))\), \(t\in[0,T]\) | M8 |
| Time scaling | how fast you move along the road | \(s:[0,T]\to[0,1]\) | M8 |
| Cubic / quintic time scaling | polynomial pedal plans | \(3u^2-2u^3\); \(10u^3-15u^4+6u^5\) | M8 |
| Trapezoidal profile | gas, cruise, brake | constant \(\pm a\), cruise at \(v\); \(T=(v^2+a)/(va)\) | M8 |
| S-curve | trapezoid with gentle pedal | jerk-limited, 7 phases | M8 |
| Jerk | how suddenly acceleration changes | \(\dddot s\) | M8 |
| Via point | stop you roll through | intermediate waypoint with set time | M8 |
| Spline | piecewise polynomial glued smoothly | \(C^1\)/\(C^2\) continuity at knots | M8 |
| Phase plane | speed-vs-position map of the move | \((s,\dot s)\) with bounds \(L\le\ddot s\le U\) | M8 |
| Time-optimal scaling (TOPP) | fastest timing along a fixed path | bang–bang in the phase plane | M8 |
| Free C-space | all poses that don't collide | \(\mathcal C_{\text{free}}=\mathcal C\setminus\mathcal C_{\text{obs}}\) | M9 |
| Minkowski sum | grow obstacles by the robot's size so the robot becomes a point | \(\mathcal B\oplus(-\mathcal A)\) | M9 |
| Complete planner | always finds a path if one exists, else says no | finite-time correct answer | M9 |
| Resolution-complete | complete at the chosen grid size | complete on the grid of spacing \(h\) | M9 |
| Probabilistically complete | failure chance shrinks to zero with more samples | \(\Pr[\text{fail}]\to0\) as \(N\to\infty\) | M9 |
| A\* | grid search that leans toward the goal | expand min \(f=g+h\) | M9 |
| Admissible heuristic | a guess that never overestimates remaining cost | \(0\le h(n)\le h^\*(n)\) | M9 |
| PRM | a reusable road network of random free samples | sampled graph + graph search | M9 |
| RRT / RRT\* | a random tree that grows into empty space; RRT\* rewires to get shorter | tree with nearest-extend; choose-parent + rewire in radius \(r_N\) | M9 |
| Potential field | walk downhill on goal-pull + obstacle-push | \(q\leftarrow q-\alpha\nabla U\) | M9 |
| Local minimum | a dip that is not the goal, where the walk stops | \(\nabla U=0,\ q\ne q_{\text{goal}}\) | M9 |
| Trajectory optimization | bend a whole path to be smooth and collision-free | CHOMP, TrajOpt, GCS | M9 |
| Error dynamics | the ODE the tracking error obeys | e.g. \(\ddot e+K_d\dot e+K_pe=0\) | M10 |
| Natural frequency | how fast the joint would ring with no damping | \(\omega_n=\sqrt{k/I}\) | M10 |
| Damping ratio | how much the bounce is eaten; 1 = no overshoot, fastest | \(\zeta=c/(2\sqrt{kI})\) | M10 |
| Overshoot | how far past the target it goes | \(M_p=e^{-\pi\zeta/\sqrt{1-\zeta^2}}\) | M10 |
| Settling time | time to stay within 2% | \(t_s\approx4/(\zeta\omega_n)\) | M10 |
| PID | spring (P) + slow push on accumulated error (I) + damper (D) | \(\tau=K_pe+K_i\int e+K_d\dot e\) | M10 |
| Integral windup | the I term keeps growing while the motor is maxed out | integrator state grows during saturation | M10 |
| Feedforward | torque from the model, applied before any error appears | e.g. \(mgl\sin\theta_d\), \(M\ddot\theta_d+h\) | M10 |
| Computed torque | cancel the arm's dynamics with its model, then use PD | \(\tau=M(\ddot\theta_d+K_pe+K_d\dot e)+h\) | M10 |
| Impedance control | make the robot feel like a spring–damper to the world | measure motion, command force | M10 |
| Admittance control | measure force, move the position target like a spring–damper would | measure force, command motion | M10 |
| Hybrid motion–force | control force in some directions, motion in the others | selection matrix \(P\) | M10 |
| Contact mode | what a touching point is doing: lifting off, sticking, or sliding | label B, R or S from the sign of normal and tangential relative velocity | M11 |
| Friction cone | all pushes a fingertip can make without slipping | \(\{f: \lVert f_t\rVert\le\mu f_n,\ f_n\ge0\}\), half-angle \(\arctan\mu\) | M11 |
| Polyhedral cone | a pyramid that stands in for a round cone | nonnegative combinations of edge vectors \(\sum\lambda_j\hat f_j\), \(\lambda_j\ge0\) | M11 |
| Form closure | object caged by geometry, no friction needed | contact normals' constraints admit only the zero twist; needs \(\ge n+1\) contacts | M11 |
| Force closure | fingers can resist any push on the object | contact wrench cone equals all of wrench space; origin strictly inside hull | M11 |
| Positive span | vectors whose nonnegative mixes reach every direction | \(\{\sum\lambda_i w_i:\lambda_i\ge0\}=\R^n\) | M11 |
| ε grasp quality | how big a worst-direction shove the grasp can take | radius of largest origin-centred ball inside the wrench hull (Ferrari–Canny) | M11 |
| Quasi-static | slow enough that inertia is ignored | force balance at every instant, no \(M\ddot q\) term | M11 |
| Limit surface | all friction wrenches a table can give a sliding object | boundary of the support-friction wrench set; sliding twist is normal to it | M11 |
| Complementarity (LCP) | either a gap or a force, never both | \(0\le g\perp\lambda\ge0\) | M11 |
| Soft-finger contact | a fingertip patch that also resists twisting | point contact + bounded torsional friction about the normal | M11 |
| Nonholonomic | can't move some directions now, but can reach every pose | non-integrable Pfaffian constraint \(A(q)\dot q=0\) | M12 |
| Differential drive | two wheels on one axle, steered by speed difference | \(v=r(u_R+u_L)/2,\ \omega=r(u_R-u_L)/(2d)\) | M12 |
| Omnidirectional | can slide any direction right now | wheels with free rollers; \(\dot q\) unconstrained | M12 |
| Skid-steer | turns by dragging wheels sideways | diff-drive with slip and a shifting instantaneous centre of rotation | M12 |
| Odometry | dead reckoning from wheel turns | integrating body twists from encoders on \(\SE(2)\) | M12 |
| Lie bracket | the net step left over from a small wiggle | \([g_1,g_2]\); flow \(g_1,g_2,-g_1,-g_2\) for \(\varepsilon\) gives \(\varepsilon^2[g_1,g_2]\) | M12 |
| Chow's theorem | if wiggles fill every direction, you can go anywhere | brackets span the tangent space ⇒ small-time locally controllable | M12 |
| Dubins / Reeds–Shepp path | shortest path for a car that can't turn too sharp (forward only / with reverse) | concatenations of max-curvature arcs and straight segments | M12 |
| Tracking error (robot frame) | how far off the path, seen from the robot | \((x_e,y_e,\phi_e)\) rotated into the body frame | M12 |
| Mobile manipulator | an arm riding on a moving base | C-space \(\SE(2)\times\) arm joints; base Jacobian missing the sideways column | M12 |
| Thrust axis | the one direction the rotors can push | \(Re_3\), third column of \(R\) | Track B |
| Mixer | turns "total push + twist" into four motor commands | \(f_i = A^{-1}(f,\tau)\) | Track B |
| Differential flatness | pick a smooth path for a few outputs and everything else follows | states/inputs = functions of \((x,y,z,\psi)\) and derivatives | Track B |
| Cascaded control | fast inner loop inside a slow outer loop | position → attitude → rate | Track B |
| Geometric control | steer the rotation matrix directly, no angles | Lee et al. 2010 on \(\SO(3)\) | Track B |
| Saturation | a motor asked for more than it can give | \(f_i\notin[0,f_{\max}]\) | Track B |
| Optical flow | how fast the ground image slides by | pixel velocity ≈ \(v/h-\omega\) | Track B |
| Downwash | the air jet pushed down by the rotors | induced velocity below the rotor disk | Track B |
| Dead reckoning | counting steps to guess where you are | integrate odometry without corrections | Track C |
| Belief | your best guess plus how unsure you are | \(p(x_k\mid z_{1:k},u_{1:k})\) | Track C |
| Covariance ellipse | the "I'm somewhere in here" blob | 95%: semi-axes \(\sqrt{5.991\lambda_i}\) | Track C |
| Kalman gain | how far to jump toward the measurement | \(K=P^-H^\top S^{-1}\) | Track C |
| Innovation | measurement minus what you expected | \(y=z-h(\hat x^-)\) | Track C |
| EKF | Kalman filter with Jacobians for curved models | linearize \(f,h\) at the estimate | Track C |
| NIS | a check that the filter's doubt is honest | \(y^\top S^{-1}y\), mean = measurement dimension | Track C |
| Factor graph | every measurement as a spring between unknowns | sum of weighted residuals | Track C |
| Loop closure | recognizing a place you've been | factor linking far-apart poses | Track C |
| Visual odometry | using the camera as the odometer | chain relative poses \(T_{k,k+1}\) | Track C |
| Policy | the robot's "what do I do now" rule | distribution \(\pi(a\mid o)\) over actions given observation | Track D |
| Behavior cloning (BC) | copy the expert's action in each situation | maximum-likelihood regression of \(a\) on \(o\) from demos | Track D |
| Covariate shift | test situations differ from training situations | learner's state distribution \(d_\pi\) ≠ expert's \(d_{\pi^\star}\) | Track D |
| Compounding error | one slip leads to more slips | BC cost can grow as \(\mathcal O(\epsilon T^2)\) | Track D |
| DAgger | expert corrects the learner's own mistakes | aggregate expert labels on learner-visited states | Track D |
| Action chunking | plan a short move, not one moment | predict and execute \(a_{t:t+k}\) per query (ACT) | Track D |
| Diffusion policy | refine noise into an action, step by step | learn the score of \(p(a\mid o)\), sample by denoising | Track D |
| Flow matching | learn a velocity that carries noise to actions | train \(v_\phi\), integrate \(\dot a=v_\phi\) from \(s=0\) to \(1\) | Track D |
| VLA | a vision-language model that also outputs motor commands | VLM backbone + action head (tokens, diffusion or flow) | Track D |
| Domain randomization | train in many fake worlds so the real one is just another | maximize \(\mathbb E_{\xi\sim p(\xi)}[J_{f_\xi}(\pi)]\) | Track D |
| World model | a learned simulator | \(\hat o_{t+1}=f_\psi(o_t,a_t)\), often video prediction | Track D |
| Leader–follower teleop | move a passive copy arm; the real arm follows | follower tracks leader joint angles; leader angles = action labels | Track D |
| Normal equations | set the misfit's slope to zero | \(A^\top Ax=A^\top b\) | Track E |
| Ridge / Tikhonov | fit, but keep the answer small | add \(\lambda^2\lVert x\rVert^2\) to the squared misfit | Track E |
| Gauss–Newton | linearize, solve least squares, repeat | step \((A^\top A)\Delta x=A^\top r\) with \(A=\partial f/\partial x\) | Track E |
| Filter factor | how much of each direction the fit keeps | \(f_i=\sigma_i^2/(\sigma_i^2+\lambda^2)\) | Track E |
| Effective parameters | how many directions really get fitted | \(\sum_i f_i=\mathrm{tr}(H)\) | Track E |
| L-curve | plot of answer size vs misfit as λ changes | \(\log\lVert x_\lambda\rVert\) vs \(\log\lVert Ax_\lambda-b\rVert\) | Track E |
| Posterior covariance | the error bars after seeing data | \(\sigma_e^2(A^\top A+\lambda^2I)^{-1}\) | Track E |
| Adjoint method | get all gradients with one backward solve | solve \(K^\top\lambda=\partial L/\partial u\) | Track E |
| Regressor matrix | the known part of the dynamics, built from motion | \(Y(\theta,\dot\theta,\ddot\theta)\) with \(\tau=Y\pi\) | Track E |
| Digital twin | a model kept in sync with the real thing | model + continuous data assimilation + decisions | Track E |
