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

**Conventions.** Frames: \(\{s\}\) = fixed space frame, \(\{b\}\) = body frame. \(T_{sb}\) reads "pose of \(b\) seen from \(s\)". Subscripts cancel like units: \(T_{ab}T_{bc}=T_{ac}\).

!!! note "Name clash"
    Grübler uses \(J\) for *number of joints*; everywhere else \(J\) is the Jacobian. The book does the same. Context always tells you which.
