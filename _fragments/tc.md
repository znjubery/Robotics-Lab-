## bridges
| Robotics idea | Equation | DiffSim / FVEM | Genomics-Lab | 3D vision | Module |
|---|---|---|---|---|---|
| Kalman update = Gaussian conditioning | \(K=P^-H^\top(HP^-H^\top+R)^{-1}\) | EnKF data assimilation (e.g. APSIM LAI, soil moisture) | BLUP \(\hat u=GZ^\top(ZGZ^\top+\sigma_e^2I)^{-1}(y-X\hat\beta)\) | visual-inertial odometry | Track C |
| Covariance propagation | \(P^-=FPF^\top+Q\) | uncertainty through a linear time step | variance of a linear combination | — | Track C |
| SLAM factor graph | \(\min\sum\lVert r\rVert^2_{\Sigma^{-1}}\), sparse Gauss–Newton | sparse FE assembly | Henderson mixed-model equations | bundle adjustment (COLMAP), 3DGS SLAM | Track C |
## words
| Term | Plain words | Precise | Module |
|---|---|---|---|
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
## symbols
| Symbol | Meaning | Size | Same idea in other labs |
|---|---|---|---|
| \(\hat x,\ P\) | state estimate and its covariance | \(n,\ n\times n\) | posterior mean / PEV in BLUP |
| \(Q,\ R\) (Track C) | process and measurement noise covariances (\(R\) is not a rotation here) | \(n\times n,\ m\times m\) | \(\sigma_e^2 I\) residual variance |
| \(K\) (Track C) | Kalman gain | \(n\times m\) | BLUP shrinkage weights |
| \(H\) | measurement Jacobian | \(m\times n\) | incidence matrix \(Z\) |
| \(\xi\) | pose perturbation in the Lie algebra | \(3\) (SE(2)) or \(6\) | — |
