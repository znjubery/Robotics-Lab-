## bridges
| Robotics idea | Equation | DiffSim / FVEM | Genomics-Lab | 3D vision | Module |
|---|---|---|---|---|---|
| Quadrotor Newton–Euler | \(m\ddot p=-mge_3+fRe_3,\ J\dot\omega+\omega\times J\omega=\tau\) | rigid-body ODE with RK4 time stepping | — | IMU propagation between frames | Track B |
| Attitude error on SO(3) | \(e_R=\tfrac12(R_d^\top R-R^\top R_d)^\vee=\sin\theta\,\hat\omega\) | — | — | rotation residual in pose graphs / BA | Track B |
| Flow needs height for scale | flow \(\approx v/h-\omega\) | — | — | monocular SfM scale ambiguity | Track B |
## words
| Term | Plain words | Precise | Module |
|---|---|---|---|
| Thrust axis | the one direction the rotors can push | \(Re_3\), third column of \(R\) | Track B |
| Mixer | turns "total push + twist" into four motor commands | \(f_i = A^{-1}(f,\tau)\) | Track B |
| Underactuated | fewer motors than ways to move | 4 inputs for 6 DOF | Track B |
| Differential flatness | pick a smooth path for a few outputs and everything else follows | states/inputs = functions of \((x,y,z,\psi)\) and derivatives | Track B |
| Cascaded control | fast inner loop inside a slow outer loop | position → attitude → rate | Track B |
| Geometric control | steer the rotation matrix directly, no angles | Lee et al. 2010 on \(\SO(3)\) | Track B |
| Saturation | a motor asked for more than it can give | \(f_i\notin[0,f_{\max}]\) | Track B |
| Optical flow | how fast the ground image slides by | pixel velocity ≈ \(v/h-\omega\) | Track B |
| Downwash | the air jet pushed down by the rotors | induced velocity below the rotor disk | Track B |
## symbols
| Symbol | Meaning | Size | Same idea in other labs |
|---|---|---|---|
| \(e_3\) | unit vector \((0,0,1)\) | \(3\) | — |
| \(f,\ f_i\) | total thrust, thrust of rotor \(i\) | scalars | load magnitude |
| \(e_R,\ e_\omega\) | attitude and angular-rate error on \(\SO(3)\) | \(3\) each | residual |
| \(J\) (Track B) | body inertia matrix (not a Jacobian here) | \(3\times3\) | rotational mass matrix |
