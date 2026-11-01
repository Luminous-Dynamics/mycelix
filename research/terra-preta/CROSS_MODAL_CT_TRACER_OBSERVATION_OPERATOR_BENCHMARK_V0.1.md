# Cross-Modal CT–Tracer Observation-Operator Benchmark v0.1

**Status:** research specification only. No CT volume has been processed by this benchmark, no tracer experiment has been run, and no empirical predictive or decision advantage is claimed.

**Tracker:** [TERRA-OBS-001 / #5643](https://github.com/Luminous-Dynamics/mycelix/issues/5643)

**Dependencies:** #5536 (known-truth preferential-flow simulator), #5548 (3D pore-network connectivity), #5550 (CT-informed interface exchange), #5580 (root/connectivity causal mediation), #5601 (sequential non-additive design), #5602 (physical reference simulators), and #5632 (covariance-aware likelihoods and SBC).

## 1. Research thesis

A soil investigation needs to distinguish what a measurement can resolve from what the physical system is doing.

The benchmark separates:

- **Geometric topology** — pore bodies, throats, bottlenecks, connected components, orientation and resolution-dependent percolation.
- **Hydraulic activation** — which geometrically available pathways actually conduct water under the current saturation/matric state, boundary conditions and event forcing.
- **Transport response** — arrival-time distributions, breakthrough shape, early bypass, matrix exchange, tailing, residence time and recovered mass.
- **Measurement** — finite-support, noisy outputs produced by the CT or tracer observation operator.

The intended scientific bridge is:

```
latent pore geometry
       +
hydraulic state and boundary conditions
       |
       v
hydraulically active pore network
       |
       v
water and solute transport
       |
       +-------------------------+
       |                         |
       v                         v
  CT observation             tracer observation
  finite volume              finite depth/time
  finite resolution          finite sampling cadence
  segmentation               detection/censoring
  image noise                 concentration/mass error
       |                         |
       +------------+------------+
                    v
       support-aware inference/design
                    |
                    v
       posterior, discrepancy, decision
```

A geometrically percolating network is not a certificate of hydraulic activity. A breakthrough curve does not identify the pore-scale route uniquely. The combination may be highly informative, redundant, or misleading depending on the observation supports and error processes.

## 2. Observation-operator formulation

Let the latent physical state be

```text
Z = (G, A(x,t), H(x,t), C(x,t), B)
```

where:

- `G` is the geometric pore/throat network;
- `A(x,t)` is the hydraulically active state;
- `H(x,t)` denotes hydraulic state and forcing;
- `C(x,t)` is the transported tracer concentration field;
- `B` contains specimen, batch and other shared effects.

The observations are not direct readings of `Z`:

```text
Y_CT     = O_CT(G; resolution, FOV, reconstruction, segmentation)
           + image noise + CT/operator discrepancy

Y_tracer = O_tracer(G, A, H, C; injection, flow, collection support,
                    cadence, censoring, calibration)
           + measurement noise + transport/operator discrepancy
```

The two observation operators have distinct physical and numerical limitations. Correlation can arise through shared latent state, specimen, batch, instrument drift or processing, but it must be represented by declared mechanisms or estimated from suitable paired evidence. Support overlap alone must never be converted into a correlation coefficient.

The model must keep at least the following uncertainty sources separate:

1. physical parameter uncertainty;
2. measurement noise and instrument calibration uncertainty;
3. CT reconstruction and segmentation uncertainty;
4. tracer sampling and concentration-to-mass uncertainty;
5. shared specimen/batch/instrument effects;
6. physical-model discrepancy;
7. observation-operator discrepancy.

## 3. CT observation operator

The CT operator must accept or record:

- field-of-view geometry and sample dimensions;
- anisotropic voxel pitch and reconstruction version;
- spatial resolution / point-spread assumptions where known;
- partial-volume and boundary-truncation controls;
- reconstruction noise or synthetic artifact model;
- segmentation algorithm, threshold and parameter digest;
- minimum resolvable pore and throat sizes;
- connected-component and boundary-label rules;
- network extractor/version and all extraction parameters.

The operator outputs an observed network plus uncertainty/provenance metadata. It must not expose the synthetic ground-truth network to the selector.

A segmentation ensemble is preferred to a single unquestioned threshold. Report network topology separately for each variant; do not average incompatible graphs into an invented consensus network.

## 4. Tracer observation operator

The tracer operator must accept or record:

- injection mass, concentration and input waveform;
- antecedent state and hydraulic boundary conditions;
- collection depth and spatial support;
- sample start/end times and actual cadence;
- instrument calibration and detection limit;
- censored / below-detection observations;
- missing intervals and whether interpolation is prohibited;
- concentration-to-mass conversion and flow integration;
- recovered mass and uncertainty;
- declared non-reactive transport assumptions for the initial benchmark.

The score must not reduce a breakthrough curve to peak concentration or one global RMSE. Keep first observed arrival (interval-censored where appropriate), rising limb, peak, inflection/curve shape, cumulative recovered mass, long-time tail and residence-time summaries distinct.

## 5. Proving families and adversarial controls

The synthetic generator should include independent mechanism families, not just many seeds of one convenient fixture.

| Family | Adversarial condition | Required diagnostic |
|---|---|---|
| Same porosity / different topology | Equal bulk porosity, different bottlenecks and percolating paths | Topology and transport recovery |
| Geometric but inactive | CT sees a connected path that is dry, air-blocked or above its activation threshold | Zero false hydraulic-path certificate |
| Active but unresolved | Conducting path is below the CT resolution | Tracer-supported uncertainty / abstention |
| Segmentation sensitivity | Plausible thresholds create/remove apparent connectivity | Per-variant topology and calibrated uncertainty |
| Same geometry / different hydraulic state | Entry pressure, saturation or exchange differs | Conditional tracer prediction |
| Similar early arrival / different tail | Curves agree near peak but diverge in tail and recovery | Tail, residence-time and mass-recovery score |
| Shared specimen effects | CT and tracer share core/preparation/batch | Hierarchical dependence recovery |
| Repeated sensor drift | Same tracer sensor/assay has temporal drift | Temporal covariance ablation |
| Paired vs adjacent cores | Same core, paired block cores or independent specimens | Correct assignment/support hierarchy |
| Support mismatch | CT volume and tracer collection column do not coincide | Spatial-support uncertainty |
| Sparse/censored sampling | First arrival and late tail are missed | Interval/censoring-aware predictions |
| Out-of-family physics | Missing exchange, lateral flow or an unmodeled pathway | Predictive discrepancy and diagnostic/abstention response |

Include matrix-only, no-flow, disconnected-path, geometrically-connected-but-inactive and closed-mass-balance controls. A control with a known null must not become an apparent success merely because its uncertainty is broad.

## 6. Adaptive design bake-off

The candidate action pool should distinguish acquisition of new specimens from repeated measurements on the same specimen. Candidate actions include CT at multiple declared resolutions, tracer under specified forcing, repeated tracer collection, repeat CT/segmentation, and independent or paired-core replication.

Compare:

- CT-only, tracer-only, repeated-CT and repeated-tracer;
- static additive marginal-score selection;
- pairwise/joint selection with diagonal independence;
- support-aware joint likelihood and conditional information gain;
- decision-focused EVSI;
- discrepancy-aware diagnostic acquisition and abstention;
- random feasible and expert-fixed schedules;
- exact adaptive action-sequence oracle on small worlds.

The selector receives only declared candidate-model predictions, support metadata, costs, eligibility and permitted posterior state. The evaluator retains the hidden physical truth and uses it only after selection to generate observations and score decisions. Truth mutation must not alter selector inputs or selection.

Use common random numbers keyed by world, replicate and action ID where potential outcomes can validly be paired. Keep policy-dependent randomness separate.

### Non-additive quantities

For a latent target `θ`, report:

```text
I(θ; Y_CT)
I(θ; Y_tracer)
I(θ; Y_CT, Y_tracer)
I(θ; Y_tracer | Y_CT)
I(θ; Y_CT | Y_tracer)
```

Also report decision EVSI and regret. Joint information greater than the best single action is evidence of complementary information for the stated target, but it is not automatically decision value. A nuisance variable can be highly identifiable and irrelevant to the service decision.

## 7. Evaluation and qualification

### Scientific/physical

- CT percolation and bottleneck recovery by resolution and segmentation variant;
- geometric vs hydraulically active pathway discrimination;
- early, peak, shape, tail and residence-time tracer errors;
- cumulative solute mass recovery;
- water/solute conservation residual;
- parameter and discrepancy identifiability;
- false pathway certificates and missed active pathways.

### Statistical

- common held-out expected log predictive density;
- Brier score for declared categorical outcomes;
- CRPS / interval score for continuous outcomes where supported;
- nominal 90% interval coverage and width, overall and by support stratum;
- simulation-based calibration for model, parameter, decision-functional and data-dependent joint-likelihood quantities;
- posterior predictive checks and cross-seed reproducibility.

Prequential scores for policy-selected observations must be reported separately from a common held-out probe set: acquisition policies see different measurement streams, and selected-stream scores are not a matched calibration contest.

### Decision and operational

- true service-decision regret and utility;
- pairwise synergy over the best single action;
- cost spent on redundant actions;
- false diagnostic and missed-discrepancy rates;
- abstention and unsupported-domain rates;
- budget, instrument throughput, sample count, elapsed time and compute;
- exact-oracle gaps or certified bounds for approximations.

### Proposed gates (not results)

- 100% budget, eligibility and independent-unit target-coverage compliance;
- zero false hydraulic-path certificates on deterministic null controls;
- ≤1% water/solute mass-balance residual in closed noise-free synthetic references;
- 85–95% coverage for nominal 90% intervals on adequately replicated held-out worlds, reported by resolution/support stratum;
- the correct support-aware likelihood improves held-out predictive score over diagonal independence when dependence exists, while matching the independent likelihood in independence controls;
- no deterministic percolation claim when plausible segmentation variants disagree beyond a preregistered tolerance;
- exact small-world oracle regret of zero within numerical tolerance;
- any claimed ≥10% predictive/decision improvement over the strongest matched baseline must have a paired uncertainty interval excluding zero, with no material degradation in calibration, conservation, false certification or worst-stratum performance.

The gates require preregistration and adequate independent-world replication. They are not universal agronomic tolerances or evidence of field superiority.

## 8. Relation to existing physical comparators

HYDRUS, MACRO and RZWQM2 are not interchangeable competitors. Match them only on supported configurations and observables. The HYDRUS 6 DualPerm module documentation describes a two-dimensional dual-permeability flow/transport formulation in the HYDRUS (2D/3D) package; this add-on should not be described as a generic 1D–3D DualPerm capability. MACRO 5.2 is a one-dimensional water-flow and reactive-solute model; USDA lists RZWQM2 4.8 as a one-dimensional whole-system model with pseudo-2D drainage.

This benchmark does not need to outperform those simulators on every physical-forward task. A credible advantage would be lower experiment cost or regret for a specified decision, better calibrated transfer under support shift, earlier detection of model inadequacy, or better measurement selection while matching the physical comparator's supported task.

## 9. Literature anchors and their limits

- Van Offenwert, Cnudde & Bultreys (2019), *Pore-Scale Visualization and Quantification of Transient Solute Transport Using Fast Microcomputed Tomography*, Water Resources Research, DOI: https://doi.org/10.1029/2019WR025880. Demonstrates pore-resolved transient tracer imaging in sintered glass and Bentheimer sandstone. The authors report substantial single-voxel concentration uncertainty that falls sharply under pore-level aggregation. This is a measurement-method analogue, not direct soil validation.
- *Zero-tillage induces significant changes to the soil pore network and hydraulic function after 7 years* (2024), Geoderma: https://www.sciencedirect.com/science/article/pii/S0016706124001630. Combines X-ray CT and solute breakthrough in undisturbed soil columns.
- Chandel et al. (2026), *Non-reactive solute transport in coupled macro-meso-micro porous system*, Journal of Contaminant Hydrology 284, 105144, DOI: https://doi.org/10.1016/j.jconhyd.2026.105144. Reports segment-dependent sensitivity across breakthrough and highlights cumulative mass recovery and tail behavior as important fitting targets.
- HYDRUS 6 DualPerm official documentation: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermModule.html.

These publications motivate the test cases; they do not establish that our implementation improves on a published method.

## 10. Implementation stages

1. Implement a small deterministic pore/throat graph and hydraulic-activation truth generator.
2. Implement the CT and tracer operators as separate truth-side modules; expose only support-bounded observations.
3. Add specimen/batch/instrument random effects and covariance controls.
4. Connect the support-aware joint likelihood to #5632, preserving the selector/evaluator firewall.
5. Add report schemas, oracle checks, negative controls, and a focused Rust workflow.
6. Freeze a multi-family seed manifest and compare all policies at equal resources.
7. Only after source/license review and qualified scientific review, evaluate public datasets; no local South African transfer claim without independent local reference measurements.

## 11. Evidence and safety boundary

Software and synthetic benchmark only. This specification does not authorize sample collection, CT scanning, tracer injection, land access, amendment application, material handling or field trials. Empirical data use requires source/license review, provenance, data governance and qualified review. No agronomic recommendation, product efficacy or field-transfer claim follows from synthetic success.


## Multiscale CT and tensorial hydraulic bridge — 2026 literature update

A new physical qualification dependency is tracked in [#5650 TERRA-PHYS-002](https://github.com/Luminous-Dynamics/mycelix/issues/5650) and specified in `MULTISCALE_CT_TENSORIAL_PERMEABILITY_BENCHMARK_V0.1.md`.

The cross-modal operator benchmark must now treat resolution, ROI / field-of-view and core diameter as joint design variables. Kaur et al. (2025) report that these choices change pore counts, isolated volume, connectivity and anisotropy. Jiang et al. (2025) show how segmentation uncertainty can propagate into hydraulic-property uncertainty. Gerke et al. (2026) show that pore-scale solver agreement does not guarantee agreement with experimental Ksat and report directional anisotropy. Sobucki et al. (2026) show domain-dependent CT–hydraulic relationships across cohesive and non-cohesive horizons.

For CT–tracer experiments, preserve both:
- a multiscale geometric observation with uncertainty; and
- an independently supported directional hydraulic response / tensor where identified.

Do not infer event-specific hydraulic activation from a saturated conductivity tensor alone. The link remains:

```text
CT geometry ensemble
 -> directional saturated-flow properties
 -> antecedent state + event forcing
 -> active pathways and exchange
 -> tracer breakthrough operator
```

The tensor is a bridge/physical prior, not a shortcut to tracer prediction. No solver or physical dataset was run by this update.


## Event-conditioned macropore–matrix exchange (2026 research update)

A new implementation track is open as [#5658 TERRA-EXCHANGE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5658). Chaif et al. (2025), Journal of Hydrology 661, 133791, DOI [10.1016/j.jhydrol.2025.133791](https://doi.org/10.1016/j.jhydrol.2025.133791), propose a macropore–matrix exchange term formed from CT-derived specific wetted interface area and macropore water-flux density. Their reported dual-permeability model improved against measured drained/stored-water responses; exchange was largest following rainfall onset and fell toward zero before matrix saturation.

The Rust library now includes a **synthetic reference operator** in `src/transient_exchange.rs`:

```text
q_exchange(t) = wetted_interfacial_area(t) * macropore_flux_density(t)
```

It linearly interpolates the two inputs and analytically integrates their product (quadratic within each interval), with unit/sign validation and a paired macropore-loss/matrix-gain storage check. It does not derive the interface from CT, solve Richards flow, or reproduce the complete published model. The paper's raw CT/infiltration data are stated as available upon request; no such data were acquired. See #5658 for the full comparative work plan.


## State–parameter assimilation comparator update

A new 2026 reduced-adjoint VDA study provides a useful comparator for joint state–parameter estimation: Heidary, Farhadi & Altaf, Environmental Modelling & Software 203, 107035, DOI [10.1016/j.envsoft.2026.107035](https://doi.org/10.1016/j.envsoft.2026.107035). It combines POD model reduction with a reduced-order adjoint in HYDRUS-1D and tests structural, initial-state and forcing uncertainty in synthetic experiments. This should be included as a matched comparator in #5658. The public accompanying repository is MATLAB-based; no code was imported or executed by this update.


## End-to-end correlated action-information bake-off

The Rust binary `conditional-information-bakeoff` now connects the stable coordinate layout, retained CT observation history, multivariate Gaussian conditional predictions and seeded information-gain estimator. Its fixture encodes two candidate models with identical one-dimensional CT and tracer marginals but opposite cross-modal correlations. The executable compares CT-only, tracer-only and joint acquisition from the prior, then scores tracer conditional on an observed CT value. Focused CI asserts zero individual marginal information, positive joint information, positive post-CT tracer information and hard budget/eligibility compliance; it validates the report schema and uploads a digest-bound artifact. This is a synthetic model-discrimination benchmark, not field validation or decision EVSI.


## Joint state–parameter assimilation oracle

The Rust crate adds `state_parameter_assimilation.rs`, an exact linear-Gaussian reference for the augmented latent vector of physical states, hydraulic/exchange parameters, bias and nuisance variables. Given a prior (z\sim N(\mu,P)), a linear observation operator (H) and noise covariance (R), it constructs the joint ([z,y]) Gaussian and conditions on the measurement. This preserves cross-covariance between water state and hydraulic/exchange parameters. It returns posterior means/covariance, innovation and log marginal likelihood.

The nine initial reference tests cover analytical scalar recovery, physical-state/exchange-parameter covariance, observation predictive covariance, batch-versus-sequential equality for independent noise, correlated cross-modal measurement noise, stable variable-role metadata, evidence/innovation output and fail-closed inputs. This is a synthetic conjugate oracle, not a substitute for nonlinear Richards/dual-permeability assimilation.


## CT voxel-interface measurement bridge

The `ct_wetted_interface.rs` module now calculates a geometric voxel-face approximation of the macropore–matrix pore interface, using anisotropic physical voxel spacing and an explicit local wet fraction per pore voxel. Each unique face is counted once; its wetted area is weighted by the lesser wet fraction of the adjacent macropore and matrix-pore voxels. Total wetted interface area is normalized by the complete CT support volume to obtain specific wetted area (a_{wetted}\,[m^{-1}]), and the result converts into a transient exchange observation with a synchronized macropore flux.

This closes the software interface from **segmented CT + supplied wetting state → wetted interface area → time-varying exchange operator**. It does not close the physics: wet fractions are inputs, and the minimum-fraction rule is a simple synthetic proxy. A validated dynamic wetting/retention model, partial-volume treatment and experimental interfacial-area calibration remain open. It also assumes co-located/consistent CT support metadata; it does not infer cross-support covariance.


## Exact oracle vs finite-ensemble Kalman approximation

The crate now has a deterministic finite-ensemble Gaussian-moment comparator in `ensemble_assimilation.rs`. It samples the augmented state/parameter prior with a recorded seed, estimates empirical prior mean/covariance, and performs a linear EnKF-style moment update using the same (H,R,y) as the exact joint-Gaussian oracle. This allows posterior mean/covariance error to be measured against an exact truth under controlled ensemble sizes.

The current reference does not implement perturbed-observation or square-root EnKF variants, nonlinear Richards dynamics or particle filtering. It is the first matched approximation control; the next benchmark should vary ensemble size, parameter correlation, observation sparsity, skew/multimodal posterior mismatch and process-noise misspecification.


## Reproducible ensemble-size error curve

The focused sequential workflow now runs `state-parameter-assimilation-bakeoff` on a SHA-256-bound synthetic scenario. It compares 32, 256 and 2,048-member Gaussian ensembles, with eight independent deterministic seeds per size, against the exact joint state/parameter posterior. It reports per-run and per-size posterior mean RMSE, covariance-relative Frobenius error and standard errors, together with the exact posterior and observation marginal likelihood. The CI control checks the analytical posterior and a preregistered largest-ensemble error bound; it does not require every smaller-seed batch to be monotonically better.


## Detection-limit and interval-censoring model for tracer observations

A tracer measurement below the detection limit is an event (Y\leq LOD), not an observed value of zero. A sampling cadence that only locates arrival between two collection times yields interval-censored information (t_L<T_{arrival}\leq t_U); sensor saturation can produce right-censored observations (Y>UQL). A missing sample is not a nondetect and must follow an explicit missingness assumption/mechanism.

This distinction is especially important in breakthrough analysis. Klaus (2026), *Hydrological Processes*, DOI [10.1002/hyp.70634](https://doi.org/10.1002/hyp.70634), reports that detection-limit truncation can bias slug-tracer stream BTCs toward faster paths and studies the dependence on tracer mass and sensor detection limit using 120,000 virtual streams. This is stream hydrology rather than soil-core validation, but it motivates a direct observation-process stress test. Field et al. (2011), *Water Research*, DOI [10.1016/j.watres.2011.03.018](https://doi.org/10.1016/j.watres.2011.03.018), show that censored background tracer concentrations affect background correction and subsequent BTC analysis. Shoari & Dubé (2018), DOI [10.1002/etc.4046](https://doi.org/10.1002/etc.4046), review left-censored environmental concentrations and the bias risk of substitution; George et al. (2021), DOI [10.1021/acs.est.0c07022](https://doi.org/10.1021/acs.est.0c07022), similarly review trace-level environmental censoring and statistical treatment.

The new `censored_tracer.rs` reference uses CDF probabilities for below-LOD, interval probability mass for interval-censored values, survival probabilities for above-range values, and a no-update outcome for missing-at-random. `censored-tracer-bakeoff` integrates it with the CT-conditioned tracer prediction and tests a negative control that substitutes exact zero for a nondetect. The synthetic fixture has a CT observation that changes the conditional tracer predictions, after which the censoring event shifts the physical-model posterior. The Gaussian scalar feature may be log-concentration or another declared transformed variable; distributional and instrument calibration adequacy remain separate tasks.
