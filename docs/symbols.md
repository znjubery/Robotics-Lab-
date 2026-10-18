# Symbols

One symbol, one meaning, across every module. Notation follows Lynch & Park so you can read the book side by side. Where FVEM or DiffSim uses a different letter for the same idea, it is listed.

| Symbol | Meaning | Size | Same idea in other labs |
|---|---|---|---|
| \(\theta\) | joint variables (angles or slides) | \(n\) | design/parameter vector \(p\) in DiffSim |
| \(q\) | configuration (general) | dim of C-space | state \(u\) in FVEM |
| \(R\) | rotation matrix, \(R\in\SO(3)\) | \(3\times3\) | camera rotation in SfM / COLMAP |
| \(p\) | position vector | \(3\) | camera centre / translation |
| \(T\) | pose (homogeneous transform), \(T\in\SE(3)\) | \(4\times4\) | camera extrinsic \([R\;\vert\;t]\) |
| \(\hat\omega\) | unit rotation axis | \(3\) | eigenvector of \(R\) with eigenvalue 1 |
| \(\omega\) | angular velocity | \(3\) | — |
| \([\omega]\) | skew matrix of \(\omega\), \([\omega]v=\omega\times v\) | \(3\times3\) | curl/cross-product operator in vector calculus |
| \(v\) | linear velocity part of a twist | \(3\) | — |
| \(\mathcal V=(\omega,v)\) | twist (spatial velocity) | \(6\) | — |
| \(\mathcal F=(m,f)\) | wrench (moment, force) | \(6\) | flux/force balance in FVM cell |
| \(\mathcal S\) | screw axis (unit twist) | \(6\) | basis function direction |
| \(J(\theta)\) | Jacobian, \(\mathcal V=J\dot\theta\) | \(6\times n\) | FEM Jacobian / sensitivity \(\partial u/\partial p\) |
| \(\Ad_T\) | adjoint map, changes frame of a twist | \(6\times6\) | change of basis |
| \(M(\theta)\) | mass matrix | \(n\times n\) | FEM mass matrix |
| \(\tau\) | joint torques/forces | \(n\) | load vector \(f\) |
| \(N,\ m,\ J\) (Grübler) | links (incl. ground), DOF of a free body, joints | scalars | — |
| \(f_i\) | freedoms of joint \(i\) | scalar | — |
| \(M\) (in FK) | home pose of the tool frame, \(T_{sb}(0)\) | \(4\times4\) | reference configuration in solid mechanics; canonical space in NeRF |
| \(\mathcal B_i\) | body-frame screw axis of joint \(i\) | \(6\) | — |
| \(q_i\) (in FK) | any point on joint \(i\)'s axis at home | \(3\) | — |
| \(J_s,\ J_b\) | space and body Jacobians | \(6\times n\) | — |
| \(\sigma_i\) | singular values of \(J\) (ellipse semi-axes) | scalars | \(\sqrt{\text{eigenvalues}}\) of \(G\) / \(K\) |
| \(\kappa\) | condition number \(\sigma_{\max}/\sigma_{\min}\) | scalar | condition number of \(X^\top X\) or \(K\) |
| \(J^\dagger\) | pseudoinverse \(J^\top(JJ^\top)^{-1}\) | \(n\times m\) | minimum-norm least-squares solution |
| \(\lambda\) (M5) | damping / regularization weight | scalar | ridge \(\lambda=\sigma_e^2/\sigma_\beta^2\); Tikhonov \(\alpha\); LM damping |
| \(e\) | task-space error (twist or position) | \(m\) | residual \(y-X\beta\) |
| \(T_{sd}\) | desired end-effector pose | \(4\times4\) | target camera pose |
| \(g(\theta)\) | loop-closure constraint residual | \(k\) | constraint \(Bu-\bar u\) in FEM |
| \(H(\theta)\) | constraint Jacobian \(\partial g/\partial\theta\) | \(k\times n\) | constraint matrix \(B\) |
| \(\theta_a,\ \theta_p\) | active (actuated) and passive joints | split of \(n\) | prescribed vs free DOFs |
| \(\lambda\) (M6) | Lagrange multipliers = constraint forces | \(k\) | reaction forces / pressure in mixed FEM |
| \(c(\theta,\dot\theta)\), \(C\) | Coriolis/centripetal vector, \(c=C\dot\theta\) | \(n\), \(n\times n\) | convective term in fluid FVM |
| \(\mathcal K,\ \mathcal P,\ \mathcal L\) | kinetic, potential energy, Lagrangian | scalars | strain + kinetic energy functionals |
| \(\mathcal G_b\) | spatial inertia matrix | \(6\times6\) | element mass matrix |
| \(\pi\) | inertial parameters (10 per link) | \(10n\) | effect vector \(\beta\) |
| \(s,\ \dot s,\ \ddot s\) | path parameter and its rates | scalars | reference coordinate \(\xi\) of an element |
| \(T\) (duration) | total move time (context separates it from pose \(T\)) | scalar | final time of a simulation |
| \(H_1\ldots H_4\) | Hermite cubic basis | scalars | beam shape functions |
| \(L(s,\dot s),\ U(s,\dot s)\) | min/max feasible \(\ddot s\) | scalars | — |
| \(\mathcal C_{\text{free}},\ \mathcal C_{\text{obs}}\) | free / colliding part of C-space | sets | domain vs embedded boundary in shifted boundary method |
| \(g(n),\ h(n),\ f(n)\) | cost-to-come, heuristic cost-to-go, \(f=g+h\) (A\*) | scalars per node | arrival time \(T\) in fast marching |
| \(\eta\) | RRT step size | scalar | step size / learning rate |
| \(U(q)\) | artificial potential | scalar field | energy functional |
| \(d(q)\) | signed distance to obstacles | scalar field | SDF |
| \(e=\theta_d-\theta\) | joint tracking error | \(n\) | residual |
| \(K_p,\ K_i,\ K_d\) | proportional, integral, derivative gains | scalars or \(n\times n\) | stiffness \(K\), damping \(C\) |
| \(\omega_n,\ \zeta\) | natural frequency, damping ratio | scalars | modal frequency, modal damping |
| \(\Lambda(\theta)\) | task-space mass matrix \((JM^{-1}J^\top)^{-1}\) | \(6\times6\) | reduced (Schur-complement) mass matrix |
| \(M_d,\ B,\ K\) (impedance) | desired mass, damping, stiffness of the end effector | \(6\times6\) | penalty-contact parameters |
| \(\mu\) | Coulomb friction coefficient | scalar | friction coefficient in contact FEM |
| \(\alpha=\arctan\mu\) | friction-cone half-angle | scalar | — |
| \(f_n,\ f_t\) | normal and tangential contact force | scalar, 1–2 | contact pressure / traction |
| \(\varepsilon\) | grasp quality (Ferrari–Canny) | scalar | factor of safety |
| \(g,\ \lambda_n\) | contact gap and normal force (complementarity) | scalars | gap function and Lagrange multiplier in Signorini contact |
| \(u_L,\ u_R\) | left / right wheel speeds | scalars (rad/s) | — |
| \(r,\ d\) | wheel radius, half axle width | scalars | — |
| \(g_i(q)\) | allowed-motion vector fields, \(\dot q=G(q)u\) | 3 each (planar) | — |
| \([g_1,g_2]\) | Lie bracket of two vector fields | 3 | commutator \([A,B]=AB-BA\) |
| \(e_3\) | unit vector \((0,0,1)\) | \(3\) | — |
| \(f,\ f_i\) | total thrust, thrust of rotor \(i\) | scalars | load magnitude |
| \(e_R,\ e_\omega\) | attitude and angular-rate error on \(\SO(3)\) | \(3\) each | residual |
| \(J\) (Track B) | body inertia matrix (not a Jacobian here) | \(3\times3\) | rotational mass matrix |
| \(\hat x,\ P\) | state estimate and its covariance | \(n,\ n\times n\) | posterior mean / PEV in BLUP |
| \(Q,\ R\) (Track C) | process and measurement noise covariances (\(R\) is not a rotation here) | \(n\times n,\ m\times m\) | \(\sigma_e^2 I\) residual variance |
| \(K\) (Track C) | Kalman gain | \(n\times m\) | BLUP shrinkage weights |
| \(H\) | measurement Jacobian | \(m\times n\) | incidence matrix \(Z\) |
| \(\xi\) | pose perturbation in the Lie algebra | \(3\) (SE(2)) or \(6\) | — |
| \(\pi_\phi(a\mid o)\) | policy with weights \(\phi\) | — | predictive model \(p(y\mid x)\) |
| \(o_t,\ a_t\) | observation, action at step \(t\) | images + \(n\); \(n\) or 7 | input \(x\), label \(y\) |
| \(k\) | action-chunk length | scalar | — |
| \(A,\ x,\ b\) | linearized model, unknowns, data misfit | \(m\times n,\ n,\ m\) | \(X,\ \beta,\ y\) in GBLUP |
| \(\lambda\) | regularization / damping strength | scalar | \(\sqrt{\sigma_e^2/\sigma_g^2}\) |
| \(\sigma_i,\ f_i\) | singular values, filter factors | \(\min(m,n)\) | eigenvalues of \(G\) |
| \(Y(\theta,\dot\theta,\ddot\theta)\) | dynamics regressor | \(n\times10n\) | design matrix |
| \(\pi\) (system ID) | stacked inertial parameters (10 per link) | \(10n\) | effects vector \(\beta\) |

**Conventions.** Frames: \(\{s\}\) = fixed space frame, \(\{b\}\) = body frame. \(T_{sb}\) reads "pose of \(b\) seen from \(s\)". Subscripts cancel like units: \(T_{ab}T_{bc}=T_{ac}\).

!!! note "Name clash"
    Grübler uses \(J\) for *number of joints*; everywhere else \(J\) is the Jacobian. The book does the same. Context always tells you which.
