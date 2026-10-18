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
