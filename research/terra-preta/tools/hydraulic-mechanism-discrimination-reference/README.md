# Hydraulic mechanism-discrimination reference v0.1

**Status:** executable synthetic known-truth benchmark. It is not soil data, field validation, or a reproduction of a published simulator-based BOED method.

## The research shift

Parameter precision and mechanism identification are not the same objective.

This fixture contains three competing hydraulic mechanisms:

| Mechanism | Steady total flow | Upper-path flow | Early transient flow |
|---|---:|---:|---:|
| Balanced parallel | 2.0 | 1.0 | 2.000 |
| Preferential path | 2.0 | 1.5 | 2.000 |
| Storage delay | 2.0 | 1.0 | 0.787 |

All three predict the same steady bulk conductance. Their pathway partition and transient predictions differ.

A bulk steady-flow observation can be highly informative about the scalar total-conductance parameter while providing zero information about which of these three mechanisms generated the data. The benchmark is designed to make that failure mode visible.

## Objectives kept separate

For discrete mechanism identity $M$ and a design's noisy observations $Y_D$:

- **Model discrimination:** $I(M;Y_D)=H(M)-E_{Y_D}[H(M\mid Y_D)]$.
- **Parameter precision:** worst-case scalar Fisher information for log total conductance, using the supplied physical derivative and declared observation noise.
- **Held-out predictive information gain:** $I(Y_D;Y_*) = I(M;Y_*) - E_{Y_D}[I(M;Y_*\mid Y_D)]$, the information acquired observations provide about a future late-transient observable $Y_*$. This is an EPIG-style predictive criterion, not a reproduction of the UAI 2026 estimator.
- **Decision value:** expected value of sample information (EVSI) for an explicit downstream action/loss matrix. For prior model probabilities and loss matrix `L(a,m)`, EVSI is `min_a E[L(a,M)] - E_Y[min_a E[L(a,M)|Y]`. This is an operational objective, not a synonym for model information gain.

The benchmark reports posterior model entropy, expected classification accuracy, Brier score and model log loss alongside information objectives. The fixture declares three downstream actions with distinct losses for balanced flow, preferential-path flow and delayed storage; the report independently enumerates the best EVSI design.

## Deterministic numerical method

Candidate subsets are exhaustively enumerated under the acquisition-cost budget. Expected posterior entropy and expected predictive information gain are estimated by a fixed seven-point product Gauss-Hermite rule over independent Gaussian observation noise. This avoids Monte Carlo seed variation in the small reference problem; it is a deterministic quadrature approximation, not a closed-form integral.

A uniform portfolio baseline enumerates every distinct portfolio with exactly the selected model-discrimination design's total cost and gives each equal probability. It is not a uniform spatial sensor-placement comparator.

## Run

```bash
cargo fmt --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml -- \
  research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/fixtures/matched-steady-transient.json \
  /tmp/terra-mechanism-discrimination.json
```

The CLI hashes the exact input bytes and refuses output overwrite.

## Closed-set discrepancy and abstention

A finite model set should not force a winner when every declared mechanism badly explains the observations. The fixture preregisters a four-standard-deviation discrepancy threshold. For an observed vector, the detector evaluates each candidate mechanism's largest absolute standardized residual and takes the minimum across mechanisms. If even the best-fitting mechanism exceeds the threshold, it returns an abstention signal.

This is a conservative out-of-family alarm under the declared independent Gaussian observation-noise model; it is not a posterior probability that all models are false, nor a substitute for a calibrated posterior-predictive check. The executable API exposes the detector, and the CLI now emits machine-readable in-family and out-of-family negative-control receipts. The focused workflow asserts that the in-family sample is accepted while the ten-sigma out-of-family sample triggers abstention. The next increment should connect this result to a runtime decision receipt that requests a new measurement or expands the model family instead of publishing a confident closed-set selection.

## Falsification controls

- Three mechanisms have identical steady bulk-flow predictions.
- Observationally equivalent mechanisms must produce zero model information gain.
- The parameter-precision design is expected to differ from the model-discrimination design.
- Candidate-order permutation must not change the selected physical measurements or objective.
- Rescaling every observable, derivative and noise standard deviation by the same unit factor must preserve information gains, Fisher information and selected designs.
- Feasible portfolios beyond the deterministic quadrature dimension cap are rejected before enumeration.
- The closed-set discrepancy detector accepts in-family observations and abstains when the best candidate mechanism exceeds the preregistered four-sigma residual threshold.
- Malformed or non-finite observed vectors fail closed.
- The exact-cost uniform baseline must have normalized portfolio probabilities and ordered quantiles.
- Decision EVSI must be positive for an informative action/loss matrix and exactly zero when every available action has an identical state-contingent loss vector.
- The Gaussian quadrature rule must integrate its normalization, first moment and second moment.

## Research comparators and next ladder

This is a small oracle, not an SOTA claim. The next benchmark should compare, at matched simulator/solve/acquisition budgets:

1. Random feasible and exact-cost uniform portfolios.
2. Nominal local Fisher / D-optimal design.
3. Finite-ensemble maximin D/E design (see [#6664](https://github.com/Luminous-Dynamics/mycelix/pull/6664)).
4. Expected model information gain / classification-based discrimination.
5. Held-out predictive information gain / EPIG-style utility on future observables, with the estimator and conditioning target stated precisely.
6. Decision-focused EVSI using the declared action/loss matrix, followed by matched alternate loss matrices and regret calibration.
7. Faithful maximin robust Bayesian design with its KL ambiguity-set, Sibson-$\alpha$-MI and PAC-Bayes assumptions reproduced.

Relevant current literature includes:

- Abdulsamad et al., *Maximin Robust Bayesian Experimental Design*, NeurIPS 2026: https://arxiv.org/abs/2603.14094
- Zaballa & Hui, *Optimizing Likelihoods via Mutual Information: Bridging Simulation-Based Inference and Bayesian Optimal Experimental Design*, UAI 2026 / PMLR 337: https://proceedings.mlr.press/v337/zaballa26a.html
- Rainforth et al., *Modern Bayesian Experimental Design*, Statistical Science 2024: https://doi.org/10.1214/23-STS915

The SBI-BOED work is particularly relevant because it connects mutual-information objectives, surrogate-likelihood training and design under fixed simulator budgets. The robust BOED work provides a materially stronger comparator than simply taking a worst case over a hand-selected finite list of scenarios. These methods must be reproduced faithfully before claiming a comparison or improvement.

## Industry and pore-scale research baselines

The competitive landscape has mature physics solvers and increasingly strong voxel-scale digital-rock methods. Terra Preta should benchmark against them instead of implying that experiment selection alone replaces a CFD solver.

### Commercial multiphysics baseline

[COMSOL's Porous Media Flow Module](https://www.comsol.com/porous-media-flow-module) supports Darcy, Richards and Brinkman formulations, fracture flow, free/porous coupling, multiphase transport, chemical species transport, heat transfer and poroelastic effects. Its 6.4 interface reference documents multiple coupled mass, momentum and energy transport formulations.

This is a broad solver/workflow baseline. We should compare equivalent physical formulations and validation cases, not claim to outperform COMSOL based on the synthetic model-discrimination fixture.

### Open-source pore-scale / voxel CFD baseline

A directly relevant current reference is Stewart et al. (10 November 2026), *A Conservative Darcy-Brinkman-Stokes Formulation for Digital Rock Flow Simulation with Embedded Second-Brinkman Effects*. Their DBS2 formulation is tested against DNS on a microchannel, a 2D heterogeneous micromodel and a 3D Bentheimer micro-CT sample. The paper reports exact microchannel velocity-field reproduction and permeability error of 1.5–3.8% in the micromodel/Bentheimer cases at matched resolution; the simplified DBS1 formulation overpredicts permeability by 14.1–37.9% in those cases. This is a very useful engineering benchmark, not a result achieved by this repository: https://arxiv.org/abs/2611.14348

OpenFOAM's [Darcy-Forchheimer porosity model](https://cpp.openfoam.org/v13/classFoam_1_1porosityModels_1_1DarcyForchheimer.html) supplies a standard continuum resistance term. GeoChemFoam builds on the OpenFOAM ecosystem and provides the cited voxel-based DBS comparison.

### Public benchmark data to adopt

- A 2026 two-phase pore-scale DNS dataset reports 100 million-plus GPU-hours, 50 relative-permeability curves and over 25,000 fluid configurations, validated against synchrotron experiments: https://www.nature.com/articles/s41597-026-06823-1
- A second 2026 dataset provides 540 DNS simulations over six porous media, nine viscosity ratios and ten contact angles, with spatial and temporal pressure/velocity data: https://doi.org/10.1038/s41597-026-07009-5
- A 19 October 2026 digital-rock study combines pore-resolved and continuum flow with CUDA-accelerated lattice Boltzmann simulation and validates against three carbonate samples: https://doi.org/10.1029/2026GL126174

These should be candidate common test sets only after checking data licenses, support definitions, boundary conditions and whether the exact raw data are actually downloadable. Dataset sizes or GPU-hours are not accuracy scores.

### What Terra Preta should aim to do better

Do not try to win by reproducing only a permeability scalar. Build a qualified evidence loop around the solver ecosystem:

1. **Geometry uncertainty:** carry multiple segmentation, resolution and registration variants instead of silently selecting a single binary image.
2. **Common estimands:** compare physically invariant boundary-response tensors/moments and sensor-support integrals, not pore IDs that change across segmentations.
3. **Observation operators:** derive Jacobians from actual forward operators and keep sensitivities separate from calibrated noise/covariance.
4. **Mechanism discrimination:** choose the next sensor/forcing to separate competing flow mechanisms and improve held-out predictions, not merely maximize precision conditional on one model.
5. **Misspecification response:** abstain when all candidate mechanisms fail predictive checks; request new observations or expand the model set.
6. **Evidence receipts:** publish exact image/segmentation/mesh, solver, boundary condition, calibration, source commit, toolchain, input hashes, outputs and independent verification.

### Matched industry bake-off

For the solver layer, use the same voxel geometries, physical properties, boundary conditions and resolution across:
- GeoChemFoam DBS2;
- OpenFOAM Darcy-Forchheimer / appropriate free-flow model;
- COMSOL Darcy/Brinkman/Richards as appropriate;
- an independently implemented DNS/LBM reference on tractable geometries.

Report permeability tensor error, velocity-field error, mass conservation residual, pressure/flux time histories, relative-permeability curves, breakthrough and dispersion, mesh/voxel convergence, solver iterations, wall time and peak memory. Do not compare different equations or boundary conditions and call the results apples-to-apples.

For the experiment-selection layer, freeze the same solver-call, simulation, acquisition-cost and wall-time budgets. Compare random/uniform, nominal D-optimal, maximin finite-ensemble D/E, model EIG, predictive information gain, decision EVSI, SBI-BOED and faithful maximin robust BED. Evaluate paired held-out predictive log score/CRPS, posterior coverage/calibration, model-classification calibration, decision regret, abstention and resource costs.

The defensible ambition is to outperform established workflows on **uncertainty-aware experiment selection, calibrated predictions and traceable qualification** while interoperating with the best physical solvers—not to assert an unsupported universal solver-speed or accuracy win.

## Qualification targets (targets, not results)

- Exact portfolio optimum on the finite candidate fixture.
- Zero false discrimination under observational equivalence.
- Posterior calibration and simulation-based calibration before evaluating approximate posterior methods.
- At least 10 distinct mechanism families and 1,000 paired synthetic worlds before broad synthetic claims.
- Paired uncertainty intervals for predictive log score, CRPS, classification calibration and decision regret.
- Report simulator calls, solver/PCG work, wall time, memory, abstention and failure rate.
- No physical-soil claim until the observation support, noise and mechanism family have been validated against measured data.

## Claim boundary

The fixture has known synthetic truth and a deliberately small mechanism family. It cannot establish that the family spans real pore-scale or soil hydraulic behavior. A system that confidently selects the best model among a misspecified candidate set can still be wrong; the next iteration must include model-discrepancy detection and a safe abstain / request-new-measurement outcome.
