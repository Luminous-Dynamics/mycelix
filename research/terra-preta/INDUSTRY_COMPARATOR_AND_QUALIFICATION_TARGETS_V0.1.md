# Terra Preta industry comparator and qualification targets v0.1

**Status:** research benchmark plan. All numeric thresholds below are proposed qualification targets, not achieved performance. No model has yet passed a head-to-head benchmark in this programme.

**As-of date:** 2026-10-31.

## 1. The strategic question

The goal is not to claim that a new soil model is superior because it has more state variables, a larger graph, or an adaptive-design label. The goal is to demonstrate that a coupled evidence-and-model workflow can make better *physical predictions and experiment decisions* under the same data, cost, compute and target-population constraints.

A defensible differentiator is a closed loop:

1. preserve immutable provenance for every sample, study, instrument, material batch, model version and processing transform;
2. express competing physical mechanisms and the observation operators that can distinguish them;
3. choose measurements using conditional decision value, target representativeness, redundancy/complementarity and model-discrepancy risk;
4. simulate and score water, solute, structure and service endpoints against measurements;
5. calibrate uncertainty and abstain when the target domain is unsupported;
6. update the next experiment only after an independent evaluator accepts the previous evidence.

This positions Mycelix as an evidence/provenance and research-governance layer, Symtropy as a physical simulation and decision substrate, and Symthaea as a candidate experiment-selection and model-reasoning system. The roles are hypotheses for engineering integration, not proof of scientific validity.

## 2. Industry comparator map

The following descriptions are taken from current official product or agency documentation. They are capability comparisons, not performance rankings.

| System | Documented scope | Strong baseline to challenge | Important limit for this programme |
|---|---|---|---|
| HYDRUS 6 / DualPerm | HYDRUS 6 provides 1D, 2D and 3D variably saturated water, heat and solute simulation. DualPerm simulates variably saturated water and solute movement in dual-permeability media, representing matrix and macropore domains. | Spatially resolved mechanistic flow/transport; explicit matrix/macropore exchange; inverse parameter estimation in the wider HYDRUS suite. | DualPerm documentation recommends smaller transport domains; it is a forward-model module, not an end-to-end provenance or adaptive field-experiment governance system. |
| MACRO 5.2 | One-dimensional process-oriented dual-permeability model. Micropore water flow uses Richards' equation; macropore flow is gravity-driven; solute transport and first-order inter-domain exchange are represented. Used in pesticide leaching and regulatory risk contexts. | Established preferential-flow and reactive-solute benchmark; relevant to leaching and tile-drainage cases. | Its documented 1D structural representation cannot by itself resolve every spatially heterogeneous pore-network or multi-modal measurement question. |
| RZWQM2 4.8 | USDA model for soil-plant management, water, nutrients, pesticides and surface energy balance. One-dimensional with pseudo-2D drainage/water-table treatment; preferential/macropore flow, crop-model options, sub-hourly water/chemical movement, automated calibration and non-linear uncertainty analysis. | Whole-system crop/soil/water-quality management and established calibration/uncertainty workflows. | Its documented dimensional structure and daily/hourly forcing requirements define comparison conditions; it is not a universal reference for all pore-scale geometries. |
| On-farm experimentation methods | Current reviews describe mixed models, Bayesian spatial approaches, permutation methods and ML to address spatial heterogeneity, sparse/noisy data, limited replication and temporal variation. | Real-world spatial experimental design and analysis, not just simulator fit. | Trial-design constraints, local representativeness and cross-season transfer must be evaluated rather than assumed. |

### Official and peer-reviewed references

- HYDRUS 6 overview: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/HYDRUSOverview.html
- HYDRUS 6 DualPerm module: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermModule.html
- MACRO 5.2, Swedish University of Agricultural Sciences (SLU): https://www.slu.se/en/about-slu/organisation/collaborative-centres/slu-centre-for-pesticides-in-the-environment/areas-of-operation/macro-models/macro-5.2
- RZWQM2 4.8, USDA Agricultural Research Service (released 2026-08-28): https://www.ars.usda.gov/research/software/download/?modecode=30-12-30-25&softwareid=412
- Riley, S. S. (2026), *Advanced data analytics for on-farm experimentation: methods, challenges, and opportunities for modern agronomic decision making*, published online 2026-04-30: https://doi.org/10.1080/1343943X.2026.2664854
- *What makes on-farm experimental data suitable for data-driven decision-making?* (2025): https://doi.org/10.1007/s11119-025-10280-y

## 3. What not to do

- Do not benchmark only against a weak or convenient baseline. Include HYDRUS DualPerm, MACRO and RZWQM2 on the physical cases each can represent, plus transparent statistical/ML baselines where data permits.
- Do not force a model to represent unsupported processes. State model domain, dimensionality, boundary conditions, calibration data and valid prediction horizon before comparing.
- Do not tune on the held-out site, season, source article, instrument batch or material batch.
- Do not compare one system on raw measurements and another on pre-smoothed or selectively filtered values.
- Do not treat lower RMSE as proof of a better experiment policy, or higher information gain as proof of better soil-management decisions.
- Do not claim South African transfer from global literature or soil maps alone. A South African target-domain result requires source-audited local measurements and independent validation.
- Do not claim a universal amendment recipe or an instrument is always optimal. The right measurement depends on the mechanism, soil, spatial support, costs and predeclared decision.

## 4. Qualification ladder

These are proposed gates. They must be preregistered and may be revised only before a benchmark release, with a recorded rationale.

### Gate 0 — evidence integrity

- 100% of confirmatory rows link to reviewed source, experiment, site, treatment, assignment unit, instrument, material batch and processing lineage where those fields exist.
- 0 connected-component leakage across frozen train/validation/test groups.
- 100% of evaluator-only truth inaccessible to acquisition selectors and model fitting.
- 100% of ineligible actions excluded; hard coverage and budget constraints never traded for score.
- Missing data, unknown provenance and unsupported target strata cause explicit warnings or abstention, not silent imputation.

### Gate 1 — physical and numerical validity

- Closed synthetic reference cases: water and solute mass-balance residual target ≤1% of the relevant input/output scale.
- Mesh/time-step refinement target: ≤2% change in predeclared primary endpoints after two successive refinements on a fixed suite.
- Non-negative concentrations, bounded water states, physically admissible flux direction and explicit boundary accounting.
- Every physical conservation failure blocks predictive score reporting for that case.
- Report solver failures, non-convergence and missing output; never exclude them silently.

The 1% and 2% thresholds are engineering targets for the chosen benchmark cases, not universal tolerances for all soil experiments.

### Gate 2 — external predictive adequacy

- Leave-one-study/source, leave-one-site, leave-one-season and leave-one-material-batch evaluation where the data support those splits.
- Report MAE, RMSE, signed bias, interval coverage and width, proper interval score or CRPS, and group-clustered uncertainty.
- Nominal 90% predictive interval target: 85–95% empirical coverage overall, plus subgroup coverage gaps no greater than 10 percentage points where subgroup sample size is adequate.
- Ambitious advancement target: ≥10% relative MAE reduction against the strongest applicable comparator on at least two independent external target groups, with a group-bootstrap uncertainty interval supporting improvement and no material degradation in calibration or conservation.
- Also report cases where HYDRUS, MACRO or RZWQM2 is better. A fair benchmark includes losing cases.

A single random split, pooled row-level R² or a within-study interpolation result cannot satisfy this gate.

### Gate 3 — experiment-selection value

For at least 1,000 preregistered synthetic worlds spanning redundancy, complementarity, observation noise, target shift, budget and model misspecification:

- 100% selector/evaluator truth isolation and hard eligibility enforcement.
- 100% hard target-coverage and budget compliance.
- ≥90% recovery of deliberately high-signal complementary pairs when the required modalities are feasible and identifiable.
- ≤10% of acquisition budget wasted on deliberately redundant actions in scenarios where complementary alternatives are identifiable.
- Median normalized regret ≤0.10 and 95th-percentile regret ≤0.25 against exact small-world or certified-bound oracles.
- ≥20% lower mean decision regret than the strongest static additive, OFAT, random and expert-fixed baseline under equal budget and target coverage.

Report the fraction of worlds where the oracle itself is non-identifiable, infeasible or tied. Do not hide them in the denominator.

### Gate 4 — sequential calibration and misspecification robustness

- Prior predictive and posterior predictive checks on every scenario family.
- Simulation-based calibration and parameter-recovery coverage on known-truth worlds.
- Conditional posterior predictive 90% interval coverage within 85–95% over held-out seeds and target strata.
- A model-discrepancy stress suite where at least one candidate model omits a true process. The selector must detect degraded predictive adequacy, request a new modality, or abstain rather than merely become more confident.
- Report decision regret and posterior calibration separately. Parameter identifiability does not imply predictive adequacy.

### Gate 5 — physical-world transfer and practical value

Only after Gates 0–4 pass:

- retrospective validation on independently acquired physical datasets;
- laboratory inter-operator and cross-instrument reproducibility;
- independent site/season validation;
- prospective, preregistered field evaluation with appropriate randomization, replication, blocking and qualified review;
- comparison of full experiment cost, elapsed time, sample throughput, missingness and decision utility;
- domain-specific review for South African soils and operating contexts.

A proposed operational goal is ≥20% lower cost per *decision-qualified* experiment than the best baseline, without reducing independent replication, target representation or endpoint quality. The denominator is decision-qualified information, not raw sample count.

## 5. Benchmark suite architecture

| Suite | Question | Comparators | Primary outputs |
|---|---|---|---|
| Physical reference suite | Are water/solute equations, boundary conditions and conservation correct? | Analytic solutions, manufactured solutions, HYDRUS/MACRO/RZWQM2 where applicable | Conservation residual, numerical convergence, runtime |
| Measurement identifiability suite | Which modalities distinguish competing physical mechanisms? | Single-modality, OFAT, static EIG, joint/batch design | Parameter recovery, mechanism classification, design regret |
| Spatial field suite | Does the analysis survive soil heterogeneity and limited replication? | Randomized blocks, replicated strips, mixed models, Bayesian spatial models, permutation inference | Treatment-effect bias, interval calibration, site-level regret |
| Non-monotonic response suite | Are dose × root × wet–dry reversals recovered rather than averaged away? | Factorial regression, hierarchical nonlinear response models, GAM/boosted trees as predictive baselines | Reversal detection, held-out response-surface error, uncertainty |
| Transportability suite | Does performance survive new studies, sites, seasons and material batches? | Grouped classical and ML baselines, physical simulators | External error, calibration, abstention, subgroup failures |
| Decision suite | Does better prediction change a real service decision? | Current practice and each qualified model/policy | Water, nutrient export, infiltration, storage, aeration, drainage and cost trade-offs |

## 6. Immediate engineering sequence

1. Finish and run the pairwise synthetic policy benchmark. Its current v0.1 model is static and pairwise; this is only a controlled failure-mode test.
2. Add observation likelihoods and covariance-aware joint information. Generate noisy observations from evaluator-only latent physical worlds.
3. Implement conditional posterior updates and discrepancy-state updates. Compare sequential greedy EIG, batch EIG, decision-focused expected value of sample information and robust discrepancy-aware selection.
4. Add observation-support metadata: CT resolution and imaged volume, tracer time/depth interval, sensor depth/cadence, core volume, pressure range and sample mass. Two measurements with different support must not be treated as duplicate or complementary without a physical mapping.
5. Run HYDRUS, MACRO and RZWQM2 as version-pinned external reference simulators on matched cases, with parameter/calibration budgets documented.
6. Add cross-site and cross-season held-out benchmark packs only after exact source files, licenses, grouping keys and measurement units have been audited.
7. Release reports with code SHA, data SHA, simulator versions, seed manifests, environment, all failed runs and a machine-readable claim state.

## 7. Current capability boundary

At this revision the repository has evidence/source-register tooling, grouped split/leakage checks, water-retention metrology audits, response-surface audits, a static additive adaptive-design bake-off, and a new static pairwise complementarity prototype. It does not yet establish physical-model parity with HYDRUS, MACRO or RZWQM2, nor external predictive superiority, field efficacy, operational savings or local South African transfer.

The benchmark plan is designed to make such comparisons possible and falsifiable. Passing software tests is necessary but not sufficient for physical validation.


## 8. Capability verification refresh — 2026-11-01

The current official documentation adds useful dimensional nuance to the
comparison table above:

- HYDRUS 6 is a unified environment for 1D, 2D and 3D variably saturated water,
  heat and solute modelling. The documented DualPerm add-on specifically
  represents two-dimensional matrix/macropore dual-permeability flow and
  transport; it is not interchangeable with every 3D HYDRUS configuration.
  The module documentation also advises using smaller transport domains.
  Sources: HYDRUS 6 overview
  https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/HYDRUSOverview.html and
  DualPerm module
  https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermModule.html
- SLU's MACRO 5.2 capability page was reviewed on 2026-07-06. It describes a
  one-dimensional process-oriented dual-permeability model with Richards flow
  and convection-dispersion transport in micropores, gravity-driven macropore
  flow and first-order approximate inter-domain exchange:
  https://www.slu.se/en/about-slu/organisation/collaborative-centres/slu-centre-for-pesticides-in-the-environment/areas-of-operation/macro-models/macro-5.2
- USDA ARS lists RZWQM2 4.8 as the latest version, released 2026-08-28. It
  describes a one-dimensional model with pseudo-2D drainage / water-table
  dynamics, preferential flow, management/crop modules, sub-hourly water and
  chemical movement, automated calibration and non-linear uncertainty
  analysis:
  https://www.ars.usda.gov/research/software/download/?softwareid=412

A 2026 review of HYDRUS analysed 3,154 peer-reviewed articles published over
1993–2024 and discusses continuing knowledge gaps in parameterisation and
process representation. The implication is not that HYDRUS is deficient as a
whole; rather, model-data coupling, parameter identifiability, and
process-specific external validation remain legitimate research questions
even for a mature and widely used simulator:
https://doi.org/10.1016/j.envsoft.2025.106801

## 9. Updated acquisition-method baseline

The current sequential work must compare methods with different objectives:

| Method family | Primary objective | Failure mode to test |
|---|---|---|
| Static EIG / entropy reduction | Reduce prior model uncertainty | Can buy information irrelevant to the predeclared service decision |
| Conditional EIG | Recompute information after each observation | Can still focus on model entropy rather than predictive or decision quality |
| Decision-focused EVSI | Improve expected downstream decision utility | Depends on a meaningful, validated utility/loss function |
| Batch / joint design | Account for correlated observations and complementary measurements | Joint design can be expensive; approximation gap must be recorded |
| Prediction-oriented sequential design | Improve response prediction in regions with local model complexity | May be compute-intensive and still depends on the observation/discrepancy model |
| Discrepancy-aware design | Learn model-system mismatch alongside physical parameters | Discrepancy can be confounded with parameters or measurement error |
| Representative/informative/de-amplifying design | Reduce error amplification under shift and misspecification | Target representativeness must be measured rather than asserted |
| Random / expert-fixed / OFAT | Establish practical baselines | Useful controls, not sufficient competitive baselines alone |

The newly executable categorical engine provides a controlled finite baseline for
static EIG, conditional EIG, EVSI and surprise-triggered diagnostic selection.
It does not yet reproduce prediction-oriented Gaussian-process design, the
Kennedy–O'Hagan discrepancy framework, differentiable ensemble Kalman
inversion, or the published R-IDeA objective. Issue #5632 tracks covariance,
discrepancy posterior and simulation-based calibration work required for a
fairer comparison.

The strategic SOTA claim must eventually be narrow and empirical: improved
decision value, predictive calibration or experiment cost under matched
physics, data, compute, measurement support and held-out target populations.


## 10. Acquisition-method and physical-observation benchmark refresh — 2026-11-01

### Current sequential Bayesian design comparators

The next comparison must cover more than the classic EIG/entropy baseline:

- **Prediction-oriented sequential Bayesian design:** Zhu and Hainy (2026, v2 revised 2026-09-29) use the Kennedy–O'Hagan simulator/discrepancy/noise framework, coupled Gaussian processes, mutual information plus a local response-complexity criterion, Gaussian-mixture compression and Schur/rank-one covariance updates. Their paper includes synthetic problems and a Duffing oscillator case. Our finite categorical engine is not a reproduction of this method; a fair later comparison must share observation operators, simulator calls and held-out prediction targets. https://arxiv.org/abs/2603.16756
- **Discrepancy-aware design:** Yang, Dong and Wu (JCP 545, 114469; 2026-01-15) combine Bayesian design for low-dimensional physical parameters with auto-differentiable ensemble Kalman inversion for high-dimensional discrepancy parameters. The method includes Gaussian assumptions and an approximate information-gain calculation; benchmark approximation error and calibration rather than treating speed as accuracy. https://doi.org/10.1016/j.jcp.2025.114469
- **Compute-efficient pooled posterior estimation:** Yang, Dong and Wu (2026-04-20 preprint) group outer samples and construct geometric pooled posterior proposals with EKI, with an importance-sampling diagnostic for proposal-target alignment. It is relevant to future simulator-call and nested-inference budgets, not an implemented baseline yet. https://arxiv.org/abs/2604.18505
- **Misspecification-robust active learning:** Tang, Sloman and Kaski (AISTATS 2026) propose balancing representativeness, informativeness and de-amplification. Our local R-IDeA-inspired score remains only a transparent heuristic, not their published objective. https://proceedings.mlr.press/v300/tang26d.html
- **SBC test-quantity design:** Modrák et al. (Bayesian Analysis 20(2), 2025) show that calibration checks depend on the chosen test quantities and recommend data-dependent statistics such as joint data likelihood. Our discrete SBC control is a reference test, not a certification of the continuous inverse problem. https://doi.org/10.1214/23-BA1404

### New HYDRUS DualPerm comparison insight

A recently published Chandel et al. study (Journal of Contaminant Hydrology 284, 105144; online 2026-10-20) combines a stratified porous-medium tracer experiment with HYDRUS DualPerm, global sensitivity analysis and PEST parameter estimation. It reports different controlling parameter groups on early breakthrough, the rising limb and the tail. The authors note that fitting breakthrough-curve concentration can emphasize peak concentration while overlooking cumulative mass recovery, residence time and spatial variance; they evaluate cumulative mass recovery as an alternative feature. DOI: https://doi.org/10.1016/j.jconhyd.2026.105144

This sharpens the physical benchmark. A tracer case must score at least:
- early breakthrough / arrival-time interval;
- peak concentration and peak timing;
- curve shape and inflection points;
- cumulative recovered mass and mass-balance residual;
- residence-time distribution / tailing;
- spatial variance where the experimental support permits it.

Do not tune only to peak concentration and then call the resulting parameters identified. Profile likelihood, global sensitivity, equifinality and held-out prediction must be reported alongside goodness of fit.

### Fair dimensionality

HYDRUS 6 standard capabilities span 1D, 2D and 3D depending on edition; its documented DualPerm add-on is specifically a 2D matrix/macropore dual-permeability model. Do not attribute every HYDRUS capability to DualPerm or force MACRO/RZWQM2 into unsupported 2D/3D configurations. The 2026 HYDRUS review synthesizes 3,154 articles (1993–2024) and identifies persistent parameterization, nonequilibrium-process and validation challenges. Sources:
- https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/HYDRUSOverview.html
- https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermModule.html
- https://doi.org/10.1016/j.envsoft.2025.106801
