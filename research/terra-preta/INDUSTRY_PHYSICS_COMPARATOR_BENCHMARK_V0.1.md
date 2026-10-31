# Industry and Physics-Based Comparator Benchmark v0.1

**Status:** Comparator specification and research plan. No head-to-head result is claimed. This document authorizes no field application, amendment recommendation, pesticide registration claim, product certification or regulatory decision.

**Additional stateful comparator:** [Dynamic soil structure/root architecture benchmark](./DYNAMIC_STRUCTURE_ROOT_ARCHITECTURE_BENCHMARK_V0.1.md) specifies a Hirmas-style image-informed dynamic macropore model and a Symtropy state-dependent morphology candidate. This is a new comparator family, not a reported performance win.
**3D morphology comparator:** [CT pore-network and wetted-interface benchmark](./PORE_NETWORK_3D_CONNECTIVITY_BENCHMARK_V0.1.md) with the [Rust graph auditor](./tools/pore-network-audit/README.md). Its targets are proposed; no CT-informed head-to-head run is claimed.


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

## 7. References

- Larsbo et al., SLU MACRO 5.2 official description (reviewed 2026-07-06): https://www.slu.se/en/about-slu/organisation/collaborative-centres/slu-centre-for-pesticides-in-the-environment/areas-of-operation/macro-models/macro-5.2
- European Commission Joint Research Centre, MACRO / FOCUS_MACRO context: https://esdac.jrc.ec.europa.eu/projects/macro
- USDA-ARS, RZWQM2 official software page (version 4.8, 2026-08-28): https://www.ars.usda.gov/research/software/download/?modecode=30-12-30-25&softwareid=412
- PC-Progress, HYDRUS 6 overview: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/HYDRUSOverview.html
- PC-Progress, HYDRUS 6 dual-permeability equations: https://www.pc-progress.com/en/OnlineHelp/HYDRUS6/DualPermeabilityModels1.html
- Li et al. (2026), NEON preferential-flow event model: https://doi.org/10.1029/2026EF008296
