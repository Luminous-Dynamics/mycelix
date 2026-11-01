# Industry and Physics-Based Comparator Benchmark v0.1

**Status:** Comparator specification and research plan. No head-to-head result is claimed. This document authorizes no field application, amendment recommendation, pesticide registration claim, product certification or regulatory decision.

**Additional stateful comparator:** [Dynamic soil structure/root architecture benchmark](./DYNAMIC_STRUCTURE_ROOT_ARCHITECTURE_BENCHMARK_V0.1.md) specifies a Hirmas-style image-informed dynamic macropore model and a Symtropy state-dependent morphology candidate. This is a new comparator family, not a reported performance win.
**3D morphology comparator:** [CT pore-network and wetted-interface benchmark](./PORE_NETWORK_3D_CONNECTIVITY_BENCHMARK_V0.1.md) with the [Rust graph auditor](./tools/pore-network-audit/README.md). Its targets are proposed; no CT-informed head-to-head run is claimed.
**CT-informed exchange term:** [Interface-exchange auditor](./tools/interface-exchange-audit/README.md) provides a dimensional audit and gross time integration for a declared wetted-area × macropore-flux series. It is a preprocessing/evidence calculator, not a Richards or dual-permeability solver.


**CT parameter-substitution comparator:** include the Lissy et al. (2020) CT-parameterized dual-porosity model as a distinct benchmark variant. The paper reports the classical version fit better overall, although its exchange behavior was deficient; CT parameter substitution did not automatically improve the fit. Keep this separate from the Chaif et al. (2025) wetted-interface exchange-mechanism comparison. See [3D pore-network benchmark](./PORE_NETWORK_3D_CONNECTIVITY_BENCHMARK_V0.1.md).

## 1. The correct question is not “neural network versus HYDRUS?”

Preferential flow and event-scale transport are served by several established physical model families. They solve different sub-problems, embody different assumptions, and are used in different operational contexts. A fair comparison must pin the exact executable, module, version, parameterization and boundary conditions.

The benchmark separates three claims:

1. **Physical forward prediction:** Can a model predict measured soil-water profiles, infiltration, drainage and solute breakthrough under a specified soil, forcing and hydraulic history?
2. **Event diagnosis:** Can the model identify event-response timing and apparent preferential-flow signatures?
3. **Transfer and intervention evaluation:** Does the model generalize to unseen sites/soils and estimate the effect of a treatment under matched forcing?

A high score on one claim does not establish the others. In particular, the Li et al. (2026) NEON PF classifier predicts a Rosetta3-`Ksat`-derived event proxy; it is not a direct measurement of macropore geometry or an independently tracer-confirmed pathway.

## 2. Established comparator families

| Comparator | Documented capability | Best role in this benchmark | Important boundary |
|---|---|---|---|
| **USSF (Uppsala model of Soil Structure and Function)** | 1D soil-crop profile model with seasonal-to-centennial dynamics in bulk density, porosity, retention and conductivity driven by roots, earthworms, tillage, shrink–swell and organic-matter turnover; evaluated with 30-year scenarios on loam and clay. | Slow structural evolution and soil–plant–organic matter feedback comparator; useful for a second-timescale state module. | First published version does not include preferential macropore water flow or solute transport; nutrient cycling and traffic compaction are also omitted. It is not a stand-alone PF event or nutrient export baseline. |
| **MACRO 5.2 (SLU)** | 1D process-oriented dual-permeability water and reactive-solute model. Matrix water follows Richards' equation and solute transport uses convection-dispersion; macropore water is gravity-driven and macropore solute transport is convective; inter-domain exchange uses first-order physically based expressions. The model family is used in European pesticide-leaching scenarios. | Strong domain-matched physical baseline for matrix/macropore partition, event leaching and reactive-solute transport. Include non-reactive conservative tracer and nutrient-relevant chemistry only where parameterized. | Modelled for macroporous agricultural soils and contaminant/leaching applications; default pesticide degradation/sorption assumptions are not automatically valid for N/P/K or biochar. Pin exact parameter file, source/build and FOCUS package context. |
| **RZWQM2 4.8 (USDA-ARS)** | 1D soil–plant management model with pseudo-2D drainage and water-table fluctuation; simulates macropore/preferential flow, crop/management effects, water, nutrient and pesticide movement, and erosion sediment loss. Weather can be daily/hourly; water and chemical movement has sub-hourly steps. USDA lists version 4.8, released 28 August 2026. | Agricultural-system comparator for coupled management, crop, water/nutrient fate and drainage, including event-scale water/chemical movement. | Not a pure event-classification model; distinguish its sub-hourly internal movement from the temporal resolution of forcing and observed labels. Verify exact input/output step and configuration. |
| **HYDRUS 6 standard modules** | Richards-equation variably saturated flow and advection-dispersion heat/solute transport, root uptake, equilibrium/non-equilibrium reactions, and mobile/immobile dual-porosity solute transport. Standard modules include inverse parameter optimization using measured transient or steady data. | Conventional high-fidelity hydraulic/transport baseline and inverse-parameter reference; standard dual-porosity/mobile-immobile transport comparison. | Dual porosity is not equivalent to two actively conducting matrix/macropore domains. Record exact version, modules, dimensionality and solver options. |
| **HYDRUS DualPerm add-on** | Specialized two-dimensional dual-permeability model for variably saturated water and solute movement in interacting matrix and macropore/fracture domains. | Higher-fidelity physical benchmark for active water movement in both pore domains and exchange. | A separate module/add-on and 2D configuration; do not report it as the default standard HYDRUS configuration or compare without matched geometry and boundary conditions. |
| **Rosetta3 / hydraulic pedotransfer functions** | Pedotransfer estimates of soil hydraulic parameters from texture and related properties; in the 2026 NEON paper, a Rosetta3-derived (K_{sat}) threshold contributes to the PF proxy label. | Reproduce the published threshold baseline and test sensitivity to uncertainty in the hydraulic prior. | Not an event-scale preferential-flow simulator, independent PF ground truth or tracer reference. |
| **Published Li et al. (2026) ResNet** | Joint event-level classification/regression using NEON precipitation and multi-depth soil-moisture signals; paper reports test F1 0.91 and apparent-velocity (R^2=0.31) on a chronological within-sensor split. | Exact machine-learning reproduction baseline. | F1 is against a Rosetta3-(K_{sat})-derived PF proxy; split is not site-held-out; velocity is apparent sensor-response velocity, not direct pore-water velocity. |

### Official model references

- SLU, **MACRO 5.2**, page reviewed 6 July 2026: https://www.slu.se/en/about-slu/organisation/collaborative-centres/slu-centre-for-pesticides-in-the-environment/areas-of-operation/macro-models/macro-5.2
- European Commission JRC, **MACRO** and FOCUS_MACRO 5.5.3/5.5.4 context: https://esdac.jrc.ec.europa.eu/projects/macro
- USDA Agricultural Research Service, **RZWQM2**, latest listed version 4.8, release 28 August 2026: https://www.ars.usda.gov/research/software/download/?modecode=30-12-30-25&softwareid=412
- HYDRUS 6, **Overview and modules**: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/HYDRUSOverview.html
- HYDRUS 6, **Dual-permeability equations**: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermeabilityModels1.html
- Li et al. (2026), **NEON event-scale PF ResNet reference**: https://doi.org/10.1029/2026EF008296
- Jarvis et al. (2024), **USSF soil structure and function model**: https://doi.org/10.1111/ejss.13455
- Hirmas et al. (2025), **image-informed dynamic macroporosity model**: https://doi.org/10.1029/2024WR039836
- Guthrie et al. (2025), **deep root loss/regeneration and soil structure dataset study**: https://doi.org/10.1029/2025EF006326

### Published machine-learning comparators

The benchmark also needs to pin the two published NEON learning baselines as distinct tasks:

- **Li et al. (2025) Random Forest:** predicts site-level PF occurrence probabilities separately for VT and NSR; uses 40 sites/17 ecoregions and a random 24-site training / 16-site testing split. Event eligibility is at least 2 mm cumulative precipitation. The VT target is evaluated under maximum, median and median-plus-standard-deviation depth-weighted Rosetta Ksat variants.
- **Li et al. (2026) ResNet ensemble:** predicts event-sensor apparent response velocity and the Ksat-threshold binary proxy; uses 33 sites, 586 sensors, 15,554 samples, a chronological within-sensor split and 50 bootstrap model realizations.

These cannot be ranked by a common headline F1: the 2025 output is site-level PF probability while the 2026 F1 is event-sensor proxy classification. A common site-held-out retest must use identical outer-site folds and report both target units separately. Both papers draw on NEON and are not independent external validation from each other.

References: Li et al. (2025), https://doi.org/10.1029/2025GL118045; Li et al. (2026), https://doi.org/10.1029/2026EF008296.

## 3. Head-to-head parity protocol

### 3.1 Exact executable and model configuration

Every result must identify:
- model name, release/version, executable/build hash, module/add-on, dimensionality and operating environment;
- governing equations and constitutive options (single/dual-domain, hysteresis, mobile/immobile exchange, sorption and reactions);
- parameter source (measured, independently estimated, literature prior, calibrated or inferred);
- numerical solver, tolerances, time-step controls, convergence/failure status and any manual intervention;
- complete initial and boundary conditions, input file digests and output digests.

“HYDRUS” or “MACRO” alone is not a reproducible model identity. A baseline label must identify the exact variant and configuration. Do not mix standard HYDRUS mobile-immobile dual-porosity and HYDRUS DualPerm 2D results as if they were the same model.

### 3.2 Information parity

For each benchmark fold:
1. Supply the same observed rainfall/irrigation forcing, event window, soil profile and declared control-volume boundary to every candidate.
2. Preserve actual input resolution and use a separate coarsening sensitivity track; do not give one model minute-scale rainfall and another hourly totals without recording the difference.
3. Use the same measured hydraulic and chemical parameters where the model supports them. Distinguish measured parameters from inferred or calibrated parameters.
4. Fit/calibrate only on training sites/cores and their designated calibration events. No validation/test observations may be used for parameter selection, feature engineering, inverse estimation, threshold selection or interval calibration.
5. Declare model-specific parameters that cannot be independently identified. A model may be run with a documented prior, but its predictions are not equivalent to a fully measured-parameter forecast.
6. Run all candidates on identical held-out events and preserve failed convergence, non-identifiability and abstention rather than removing difficult cases.

### 3.3 Observable-level scoring

Compare models on observables they can actually predict, not solely on model-specific latent states:

| Observable | Scoring |
|---|---|
| Multi-depth soil moisture / pressure head | MAE, RMSE, bias, event onset/peak timing, depth-wise residuals and predictive interval score |
| Infiltration / drainage / runoff hydrographs | Cumulative volume error, peak magnitude/time error, hydrograph shape and event water-balance residual |
| Tracer breakthrough | Arrival time, peak timing/concentration, tailing, cumulative recovered mass and mass-recovery residual |
| Nutrient breakthrough | Species-specific concentration and cumulative measured mass export; dissolved and particulate separately |
| PF proxy classification | Sensitivity, specificity, precision, F1, MCC, AUCPR and Brier score, with the proxy's derivation stated |
| Tracer-confirmed pathway classification | Independent sensitivity/specificity and indeterminate rate, scored only where reference data exist |
| Velocity distribution | MAE/RMSE, median error, P90/P95 quantile error, tail bias and calibrated predictive distributions |
| Operational value | Runtime, parameter-data burden, failure rate, calibration time and measurements needed per accepted prediction |

Do not force a single scalar score over these observables. Use a Pareto profile and prespecified veto gates for conservation, calibration, external validity and physical interpretability.

## 4. Three benchmark tracks to prevent unfair conclusions

### Track P — forward physical transport

Compare MACRO, RZWQM2, HYDRUS standard/dual-porosity, HYDRUS DualPerm and Symtropy physical kernels against measured transient moisture, drainage and tracer/solute breakthrough on the same soil columns/field events. Use only models with supported equations and inputs for the target observable. A missing model output is “not supported/not evaluated,” not a zero-error win.

### Track E — event diagnosis

Compare the ResNet reproduction, interpretable tabular models, sequence/state-space models and physics-derived event predictors against the same event definitions and targets. Score both:
- the published Rosetta3-(K_{sat}) PF proxy; and
- independent dye/tracer/geophysical corroboration, where available.

Never train on the proxy and then describe test F1 as tracer-confirmed PF detection.

### Track T — site and treatment transfer

Use nested site-/ecoregion-held-out partitions for prediction. For causal treatment claims, require an independently assigned amendment/control design with matched hydrologic forcing, soil batch, geometry, crop/management, and nutrient programme. Predictive accuracy is not a treatment effect. A model's ability to reconstruct an untreated storm does not establish that biochar changed the response.

## 5. How Symtropy/Symthaea/Mycelix should beat the best valid baseline

The defensible differentiator is not simply another large neural network:

- **Symtropy:** measured-parameter matrix/macropore transport, event water/solute conservation, branch/history-dependent hydraulic state and matched reference solver comparison.
- **Symthaea:** uncertainty-aware competition among physical and learned hypotheses; chooses the next measurement by expected reduction in decision-relevant uncertainty, not merely by predictive entropy; abstains when the model is not identifiable.
- **Mycelix:** pins exact data releases, site/soil/core/batch/event identity, parameter provenance, model configuration, split manifests, calibration records and reviewer attestations.

Candidate architecture: a multi-resolution, depth-aware state-space model with a physically constrained matrix-flow prior and a learned preferential-path residual/latent domain. It must produce predicted moisture response and flux distributions and identify which evidence supports any inferred pathway.

### Proposed competitive acceptance criteria

Targets below are proposed, not achieved:

1. At least 10% lower error than the strongest applicable physical or ML baseline on a preregistered primary observable, using identical outer folds.
2. At least 20% lower P90/P95 velocity or breakthrough-tail error than the best matched baseline, without material median-error degradation.
3. Nominal 90% intervals achieve 85–95% empirical coverage on independent held-out sites, with sharpness and proper interval score reported.
4. No held-out site, event or model failure silently omitted; publish worst-site performance and failure/abstention rates.
5. No worse water/nutrient conservation qualification, tracer agreement or critical domain-specific endpoint in exchange for better average F1.
6. Parameter identifiability and physical-model parity demonstrated before claiming that a hybrid model improves the mechanistic state estimate.
7. All improvements replicated on a second independent network or laboratory. NEON alone cannot establish South African transfer.

A 10% average MAE improvement is not sufficient if extreme-event errors, calibration, conservation or worst-site behaviour worsen. Do not claim “beats industry” until the relevant executable and its exact configuration have been rerun on the same frozen data.

## 6. Limits on transfer of industry models

- MACRO's documented target is macroporous soil water flow and reactive solute transport, especially pesticide leaching and regulatory scenarios. Its pesticide-specific transformation/sorption defaults must not be reused as nutrient chemistry without independently justified parameters.
- RZWQM2 is an agricultural soil–plant management model with macropore flow and crop options. Its results depend on detailed weather, soil and management inputs; daily/hourly forcing and sub-hourly internal water/chemical movement must not be mistaken for one-minute rainfall event observations.
- HYDRUS standard dual-porosity mobile/immobile transport, HYDRUS DualPerm dual-permeability flow, and its geochemical add-ons are different configurations. Record the module, dimensions and equations explicitly.
- Rosetta3-derived (K_{sat}) is a prior/threshold input, not ground truth. Quantify its uncertainty and label sensitivity.
- Regulatory use in a pesticide-leaching framework does not confer regulatory approval for biochar, fertilizer, N/P/K leaching, greenhouse-gas, contaminant or South African field claims.

## 6A. Saturated-conductivity model bake-off: RAC and measurement-method regimes

A relevant industry/SOTA benchmark was published in *Geoderma* 471 (July 2026), article 117866, DOI [10.1016/j.geoderma.2026.117866](https://doi.org/10.1016/j.geoderma.2026.117866). Qin et al. compared eight water-retention-derived saturated hydraulic conductivity (`Ks`) models: three semi-empirical models and five models integrating a water-retention-derived pore-size distribution, using both Brooks–Corey and van Genuchten retention forms. They evaluated a uniform-method set of 378 samples (HYP-UNI) and a mixed-institution/mixed-method set of 1,734 samples (HYPRES).

The main result changes the benchmark design:

- Relative air capacity (RAC) is the fraction of total porosity that is air-filled at -10 kPa. It was used as the structural regime descriptor.
- In the uniform-method dataset, integral-based models, particularly Brooks–Corey variants, did better at RAC >= 5%, but often underpredicted Ks when RAC < 5%.
- On the mixed-method dataset, prediction performance deteriorated and semi-empirical models were relatively more competitive; underprediction in RAC < 5% persisted.
- Overprediction was associated with lower total porosity (around 40% or lower) in the uniform-method analysis, but that relationship did not generalize cleanly to the mixed-method dataset.
- The paper cautions that water-retention curves with few points and measurements made on different samples can obscure structure–Ks relationships. It is not enough to pool model scores and call the top pooled model “best”.

This becomes a separate **Ks regime track**, not another preferential-flow classifier. The new executable is [`tools/ks-regime-audit`](./tools/ks-regime-audit/README.md), tracked by [issue #5567](https://github.com/Luminous-Dynamics/mycelix/issues/5567).

### Required matched comparison

Evaluate the same frozen held-out samples and source identities with:

1. Brooks–Corey pore-spectrum integration;
2. van Genuchten pore-spectrum integration;
3. Kozeny–Carman and other declared semi-empirical conductance baselines;
4. Rosetta3 pedotransfer predictions;
5. Symtropy's physics-based Ks candidate;
6. any hybrid or learned-residual model.

Every implementation must pin source/build/version, retention-curve model, hydraulic parameters, pore-size integration bounds, tortuosity/conductance assumptions, calibration folds, and model-configuration digest. If a model requires more water-retention points or independently measured parameters, that information burden must be reported and matched or scored in a separate track.

### Mandatory strata and metrics

Use the following separately and publish every model's count and independent-group count:

- RAC < 5% and RAC >= 5%;
- total porosity <= 40% and > 40%;
- exact Ks measurement method and protocol;
- exact water-retention measurement method and protocol;
- RAC × Ks-measurement-method;
- uniform-method and multi-method evaluation tracks, never pooled into a single headline number.

Primary metrics should be MAE/RMSE/bias in log10(Ks), concordance correlation coefficient where supported, residual distribution, and the fractions underpredicting/overpredicting. Preserve the paper-style residual categorization using +/-0.5 times the pooled standard deviation of observed log10(Ks), while clearly stating its reference population. Report worst-regime error and the direction of bias, not just the pooled mean.

### What would count as an advance

The competitive goal is not simply lower mean error. A new model should improve the low-RAC (<5%) regime without sacrificing the high-RAC structured-soil regime, remain calibrated across measurement protocols, and show that a pore-structure term improves prediction on genuinely held-out groups. It must also survive a methodologically uniform dataset and a separate mixed-method dataset. A method-sensitive rank reversal is an important result, not noise to average away.

**Important boundary:** the 378 and 1,734 sample counts are the 2026 paper's reported datasets; Terra Preta has not downloaded or reproduced them in this branch. The RAC and porosity boundaries are diagnostic strata suggested by that study, not universal physical transitions. The CLI accepts provenance-bound RAC values; it does not calculate RAC or validate the source lab measurement.

## 7. References

- Larsbo et al., SLU MACRO 5.2 official description (reviewed 2026-07-06): https://www.slu.se/en/about-slu/organisation/collaborative-centres/slu-centre-for-pesticides-in-the-environment/areas-of-operation/macro-models/macro-5.2
- European Commission Joint Research Centre, MACRO / FOCUS_MACRO context: https://esdac.jrc.ec.europa.eu/projects/macro
- USDA-ARS, RZWQM2 official software page (version 4.8, 2026-08-28): https://www.ars.usda.gov/research/software/download/?modecode=30-12-30-25&softwareid=412
- PC-Progress, HYDRUS 6 overview: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/HYDRUSOverview.html
- PC-Progress, HYDRUS 6 dual-permeability equations: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermeabilityModels1.html
- Li et al. (2026), NEON preferential-flow event model: https://doi.org/10.1029/2026EF008296


## New pore-scale hydraulic comparator family — 2026 literature update

Issue #5650 and `MULTISCALE_CT_TENSORIAL_PERMEABILITY_BENCHMARK_V0.1.md` extend the benchmark beyond dual-permeability profile simulators to image-resolved pore-scale flow:

- **Gerke et al. (2026), Soil and Tillage Research 106790**, DOI [10.1016/j.still.2025.106790](https://doi.org/10.1016/j.still.2025.106790): finite-difference Stokes, lattice-Boltzmann and pore-network simulation on XCT images. The paper reports inter-solver agreement but overestimation relative to experimental conductivity in the tested horizons, plus directional anisotropy. Treat this as a required negative-control/comparator family. It is not evidence that all pore-scale solvers always overestimate.
- **Kaur et al. (2025), Journal of Soils and Sediments**, DOI [10.1007/s11368-025-04105-8](https://doi.org/10.1007/s11368-025-04105-8): image resolution, field of view and core diameter alter observed pore counts, isolation, connectivity and anisotropy.
- **Jiang et al. (2025), Plant and Soil**, DOI [10.1007/s11104-024-07030-w](https://doi.org/10.1007/s11104-024-07030-w): segmentation uncertainty propagates through CT-derived morphology into permeability in a vegetated porous-media analogue.
- **Sobucki et al. (2026), Journal of Soil Science and Plant Nutrition**, DOI [10.1007/s42729-026-03422-2](https://doi.org/10.1007/s42729-026-03422-2): CT–hydraulic relationships vary between cohesive/non-cohesive horizons and across sites; some groups show no significant correlations.

### New required qualification

1. Compare finite-difference Stokes, LBM and PNM on identical frozen geometries where supported.
2. Propagate reconstruction, segmentation, network-extraction and solver uncertainty into permeability; no hydraulic-target threshold tuning on held-out specimens.
3. Estimate directional conductivity tensor components and principal directions where the experimental design identifies them; do not reduce all claims to scalar (K_{sat}).
4. Test joint resolution–field-of-view–core-size / representative-volume effects.
5. Stratify and validate across soil structural domains, horizons, specimens and sites; a universal CT-feature-to-(K) regression is not assumed.
6. Keep saturated hydraulic tensor distinct from unsaturated event activation and tracer transport. The latter remains linked to #5643.

No new solver is implemented or run by this documentation update. These are comparator requirements and proposed tests, not benchmark results.
