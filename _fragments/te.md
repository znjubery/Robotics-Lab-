## bridges
| Robotics idea | Equation | DiffSim / FVEM | Genomics-Lab | 3D vision | Module |
|---|---|---|---|---|---|
| One regularized least-squares equation | \(x_\lambda=(A^\top A+\lambda^2I)^{-1}A^\top b\) | Tikhonov inverse problem | ridge / GBLUP | LM in bundle adjustment | Track E |
| Prior = regularizer | \(\lambda^2=\sigma_e^2/\sigma_x^2\), \(\mathrm{Cov}=\sigma_e^2(A^\top A+\lambda^2I)^{-1}\) | Bayesian inversion / UQ | heritability sets λ; REML | pose covariance | Track E |
| System ID is linear in parameters | \(\tau=Y(\theta,\dot\theta,\ddot\theta)\pi\) | \(K(p)=\sum p_iK_i\) | \(y=X\beta+e\) | — | Track E |
## words
| Term | Plain words | Precise | Module |
|---|---|---|---|
| Normal equations | set the misfit's slope to zero | \(A^\top Ax=A^\top b\) | Track E |
| Ridge / Tikhonov | fit, but keep the answer small | add \(\lambda^2\lVert x\rVert^2\) to the squared misfit | Track E |
| Gauss–Newton | linearize, solve least squares, repeat | step \((A^\top A)\Delta x=A^\top r\) with \(A=\partial f/\partial x\) | Track E |
| Levenberg–Marquardt | Gauss–Newton with a safety brake | \((A^\top A+\lambda^2I)\Delta x=A^\top r\) | Track E |
| Filter factor | how much of each direction the fit keeps | \(f_i=\sigma_i^2/(\sigma_i^2+\lambda^2)\) | Track E |
| Effective parameters | how many directions really get fitted | \(\sum_i f_i=\mathrm{tr}(H)\) | Track E |
| L-curve | plot of answer size vs misfit as λ changes | \(\log\lVert x_\lambda\rVert\) vs \(\log\lVert Ax_\lambda-b\rVert\) | Track E |
| Posterior covariance | the error bars after seeing data | \(\sigma_e^2(A^\top A+\lambda^2I)^{-1}\) | Track E |
| Adjoint method | get all gradients with one backward solve | solve \(K^\top\lambda=\partial L/\partial u\) | Track E |
| Regressor matrix | the known part of the dynamics, built from motion | \(Y(\theta,\dot\theta,\ddot\theta)\) with \(\tau=Y\pi\) | Track E |
| System identification | measure the robot's own masses and friction | estimate \(\pi\) from logged \(\tau,\theta,\dot\theta,\ddot\theta\) | Track E |
| Digital twin | a model kept in sync with the real thing | model + continuous data assimilation + decisions | Track E |
## symbols
| Symbol | Meaning | Size | Same idea in other labs |
|---|---|---|---|
| \(A,\ x,\ b\) | linearized model, unknowns, data misfit | \(m\times n,\ n,\ m\) | \(X,\ \beta,\ y\) in GBLUP |
| \(\lambda\) | regularization / damping strength | scalar | \(\sqrt{\sigma_e^2/\sigma_g^2}\) |
| \(\sigma_i,\ f_i\) | singular values, filter factors | \(\min(m,n)\) | eigenvalues of \(G\) |
| \(Y(\theta,\dot\theta,\ddot\theta)\) | dynamics regressor | \(n\times10n\) | design matrix |
| \(\pi\) (system ID) | stacked inertial parameters (10 per link) | \(10n\) | effects vector \(\beta\) |
