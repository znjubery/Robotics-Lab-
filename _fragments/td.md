## bridges
| Robotics idea | Equation | DiffSim / FVEM | Genomics-Lab | 3D vision | Module |
|---|---|---|---|---|---|
| Behavior cloning | \(\min_\phi\sum\lVert a_t-\mu_\phi(o_t)\rVert^2\) | fitting a surrogate to solver runs | GBLUP regresses phenotype on markers | VGGT regresses poses from images | Track D |
| Covariate shift / DAgger | \(d_{\pi^\star}\ne d_\pi\) | surrogate queried outside its training range | accuracy drop across populations / G×E | NeRF fails far from training views | Track D |
| Diffusion / flow policy | \(a^{k-1}=\alpha_k(a^k-\gamma_k\varepsilon_\phi)+\sigma_kz\) | iterative solvers, pseudo-time stepping | — | NeRF / 3DGS optimization from random init | Track D |
## words
| Term | Plain words | Precise | Module |
|---|---|---|---|
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
## symbols
| Symbol | Meaning | Size | Same idea in other labs |
|---|---|---|---|
| \(\pi_\phi(a\mid o)\) | policy with weights \(\phi\) | — | predictive model \(p(y\mid x)\) |
| \(o_t,\ a_t\) | observation, action at step \(t\) | images + \(n\); \(n\) or 7 | input \(x\), label \(y\) |
| \(k\) | action-chunk length | scalar | — |
| \(\xi\) | simulator parameters (randomized) | varies | uncertain PDE parameters |
