# Preferential-Flow Transfer and Velocity Benchmark v0.1

**Status:** Research benchmark and data-acquisition plan. No field application, amendment recommendation, nutrient-loss claim, or site-specific forecast is authorized by this document.

**Executable preprocessing:** [Preferential-flow eventizer](./tools/preferential-flow-eventizer/README.md) and [synthetic known-truth benchmark](./SYNTHETIC_KNOWN_TRUTH_BENCHMARK_V0.1.md). The eventizer consumes QA-filtered 10-minute normalized series; it does not parse raw NEON packages or certify pathways.

**Implementation trackers:** [PF reproduction/transfer](https://github.com/Luminous-Dynamics/mycelix/issues/5534) · [NEON cohort and normalization](https://github.com/Luminous-Dynamics/mycelix/issues/5537).

## 1. Why this is the next benchmark

The strongest recent event-scale preferential-flow (PF) reference located for this work is Li et al. (2026), *Earth's Future*, DOI [10.1029/2026EF008296](https://doi.org/10.1029/2026EF008296). It trained a two-block residual neural network on NEON soil-moisture and precipitation observations from 33 sites / 16 ecoregions. The post-QA dataset contained 15,554 event–sensor samples from 586 sensors. The paper reports test PF-classification F1 of 0.91 and accuracy of 0.85, while test event-response velocity prediction reached only (R^2=0.31).

This creates an opportunity, but the comparison must be read carefully:

- **The binary label is a hydraulic proxy, not independent tracer ground truth.** PF was assigned when a sensor-derived response velocity exceeded a texture-based maximum matrix-flow (K_{sat}) threshold estimated with Rosetta3.
- **The continuous velocity is an apparent sensor-response velocity.** It is sensor depth divided by elapsed time from precipitation onset to a threshold soil-moisture response. It is not a direct measurement of pore-water velocity in a mapped macropore.
- **The published evaluation is chronological within each sensor, not site-held-out.** Each sensor's observations were kept in temporal order for the 70/15/15 partition; sensors with fewer than 20 samples were assigned to training. The same sites can therefore contribute records to multiple splits. This tests future events at represented sensor locations, not transfer to a new site.
- **The model smooths the velocity extremes.** The authors report under-representation of the lowest and highest apparent velocities and over-representation near the mean.
- **The paper's source statement uses a precipitation product that NEON has since retired.** Its original data must be pinned from a historical release before the published pipeline can be reproduced faithfully.

The paradigm shift is to benchmark *pathway response distributions under hydrologic forcing*, not only a thresholded binary label. We should predict event arrival and apparent velocity distributions, distinguish matrix-flow proxies from tracer-corroborated preferential pathways, quantify uncertainty and abstention, and test generalization to unseen sites and ecoregions.

## 2. SOTA reference: reproduce before claiming to beat it

### 2.1 Published data and protocol to reproduce

Li et al. (2026) report:

| Item | Published reference |
|---|---|
| Observatories | 33 of 40 NEON terrestrial sites |
| Ecoregions | 16 |
| Soil-moisture sensors | 586 |
| Post-QA event–sensor samples | 15,554 |
| Soil-moisture instrumentation | Sentek EnviroSCAN TriSCAN |
| Sensor geometry | Up to eight depths, approximately 6–200 cm; five soil plots per site |
| Soil moisture sampling | 1-minute observations, resampled to 10-minute medians |
| Model | Two ResNet-style residual blocks, joint regression and PF classification |
| Ensemble | 50 model realizations |
| Published chronological split | 11,498 train / 1,837 validation / 2,219 test |
| Test classification | F1 0.91; accuracy 0.85 |
| Test apparent velocity | (R^2=0.31) |

These are reported paper figures, not independently reproduced values. We must not assume the same event count or site roster until the exact data release and filters are audited.

### 2.2 Exact eventization and target construction

The reproduction track must preserve the paper's event and label semantics:

1. Soil-moisture data are quality-filtered using NEON flags and profile porosity constraints.
2. The paper resamples soil moisture to 10-minute medians; fewer than seven good 1-minute observations in a 10-minute interval produce a missing interval. Gaps shorter than six hours are linearly interpolated.
3. A precipitation event begins at a non-zero 10-minute precipitation pulse. Additional precipitation within the following six hours is assigned to that event; event duration is capped at five days.
4. A sensor response begins when its volumetric water content increase reaches at least 0.01 m³/m³.
5. Apparent response velocity is calculated as sensor depth divided by the elapsed time from precipitation onset to sensor response onset. Preserve units and the log transform used by the model.
6. PF proxy label is positive when this derived velocity exceeds the maximum expected matrix-flow velocity estimated from Rosetta3 (K_{sat}), soil texture and the soil thickness above the sensor.
7. Sensors between 10 and 100 cm are used for the PF classification analysis.

Do not silently change event gaps, duration caps, resampling, response thresholds, (K_{sat}) assumptions or depth eligibility when reproducing the published baseline. Any alternate threshold or eventizer is a separate sensitivity track.

### 2.3 Baseline split reproduction

The first benchmark run is a faithful replication of the paper's chronological per-sensor split and preprocessing. The published split counts are audit targets. Any difference must be explained by release version, site availability, sensor geometry, quality flags, event definitions, exclusions or implementation details.

Reproduction tolerance (proposed acceptance, not achieved):
- classification F1 within ±0.03 of the reported value;
- accuracy within ±0.03;
- apparent-velocity (R^2) within ±0.05;
- same held-out rows, preprocessing, feature set and target definitions for every model compared in this track.

A match on this split is a replication result, not evidence of site-level generalization.

## 3. Correct the data-source version before modelling

### 3.1 Soil moisture

The NEON soil-water product is **DP1.00094.001**. NEON's RELEASE-2026 landing page reports a data range of April 2016 through June 2025 and describes five instrumented soil plots per terrestrial site with up to eight sensor depths. The NEON tutorial documents 1-minute and 30-minute time indices. For reproduction, use the 1-minute observations where available and reproduce the paper's 10-minute preprocessing explicitly.

### 3.2 Precipitation: legacy and replacement products are not interchangeable

The 2026 paper cites the former **DP1.00006.001** precipitation product. NEON announced on 6 October 2025 that it had split the legacy product into:

- **DP1.00044.001 — precipitation, weighing gauge**;
- **DP1.00045.001 — precipitation, tipping bucket**;
- **DP1.00046.001 — precipitation, throughfall**.

NEON's notification states that the new weighing-gauge product is published at hourly and daily aggregations, while tipping-bucket and throughfall products include 1-minute and 30-minute aggregations. NEON's 21 October 2025 notice says DP1.00006.001 is deprecated and no longer publishes new data; historical releases remain accessible.

Implications:

- **Faithful historical reproduction:** acquire the exact archived DP1.00006.001 release used by the paper, if available, and pin the release, files, filters and checksums.
- **Current high-frequency extension:** evaluate DP1.00045.001 tipping-bucket data as a separate input source. Do not claim it is identical to the retired secondary precipitation series without a site/time/method crosswalk.
- **Independent gauge sensitivity:** include DP1.00044.001 only at its actual temporal resolution; hourly/daily totals cannot reproduce minute-scale intensity peaks.
- **Throughfall:** DP1.00046.001 is under-canopy throughfall, not open precipitation. Keep it as a distinct water-input pathway and never concatenate it as if it were the same gauge measurement.

The exact release and site-by-month availability for each product must be fetched and frozen before analysis. The companion machine-readable source plan records candidates but deliberately contains no fabricated site list, row count, file digest or acquisition receipt.

## 4. Evaluation ladder: separate the claims

### Track A — historical reproduction

Reproduce the published eventizer, target proxy, features, model family, 50-model ensemble and chronological split. Publish exact site/sensor/event counts and a discrepancy report before tuning a new model.

### Track B — site-held-out transfer

Freeze outer folds by **entire NEON site**. No site, plot or sensor may appear in more than one outer split. Tune only within training sites using inner site-grouped validation. Preprocessing statistics, feature selection, calibration and thresholds are fitted on training folds only.

Report per-site and pooled macro metrics. This is the first fair test of whether the model transfers to an unobserved site.

### Track C — ecoregion-held-out transfer

Hold out complete ecoregions, not random sites from every ecoregion. The 16-ecoregion source offers a starting frame, but the number of groups and balance must be examined before claiming a stable estimate. Treat small or unrepresented ecoregions as exploratory/OOD rather than silently merging them into a convenient group.

### Track D — prospective release transfer

Train on a frozen historical release and test on a later, non-overlapping time window from newly released data. Because NEON changed precipitation products, report product-specific input sensitivity and calibration drift. Do not mix releases without source identity and duplicate-record reconciliation.

### Track E — independent pathway evidence

Evaluate tracer- or otherwise independently corroborated PF events separately from the Rosetta3-threshold proxy. Report how often the proxy label agrees with the independent reference, by site, depth, soil and event intensity. A high score on the proxy cannot substitute for this track.

### Track F — South African transportability

NEON is a US/Puerto Rico observatory network, not a South African validation set. A local external track requires independently acquired South African rainfall, multi-depth soil-water response and preferably tracer/breakthrough data, with calibrated instruments, site metadata and qualified review. Soils4Africa and Western Highveld datasets can inform context and stratification; they are not PF event labels or biochar-response evidence.

## 5. Required model bake-off

All models must use the same event records, targets, outer folds and preprocessing scope within each evaluation track.

| Model | Role | Required comparison |
|---|---|---|
| Paper ResNet reproduction | Published deep-learning reference | Match the original feature and chronological-split protocol |
| Hydraulic threshold / Rosetta3 | Mechanistic proxy baseline | Quantify dependence on the Ksat threshold and texture inputs |
| Logistic / generalized additive model | Interpretable event-feature baseline | Test nonlinear event intensity and duration thresholds |
| Random forest / gradient-boosted trees | Strong tabular ML baselines | Tune only within inner grouped training folds |
| Temporal convolution / state-space model | Raw event-sequence model | Test rainfall and multi-depth moisture sequences without future leakage |
| Depth-graph model | Sensors as ordered depth nodes with hydraulic connections | Test whether cross-depth arrival patterns add value beyond independent sensor summaries |
| Hybrid physics–ML model | Hydraulic model plus learned residual/latent bypass component | Require parameter identifiability, water-balance consistency and independent transient validation |
| Calibrated ensemble | Distributional decision model | Report velocity quantiles and probability calibration, not only a point estimate |

The novelty is not merely a larger neural network. The strongest candidate is a **multi-resolution, physics-constrained, depth-aware state-space model** with distinct latent matrix and preferential pathways. It should predict:
- event-response onset time and uncertainty at each depth;
- the apparent velocity distribution, including upper/lower tails;
- the paper's PF proxy probability;
- tracer-corroborated PF probability where such labels exist;
- water and solute flux distributions only where their required measurements exist.

The model must abstain when event onset, sensor calibration, (K_{sat}) reference, forcing coverage or soil-domain support is inadequate.

## 5.1 Physical and industry comparator parity

Use the companion [Industry and Physics-Based Comparator Benchmark](./INDUSTRY_PHYSICS_COMPARATOR_BENCHMARK_V0.1.md)
before claiming a competitive result. It distinguishes MACRO 5.2, RZWQM2 4.8,
HYDRUS standard dual-porosity and HYDRUS DualPerm 2D; these are not one
interchangeable “HYDRUS” baseline. The benchmark also separates physical
forward prediction, event classification and site/treatment transfer.

## 6. Metrics and proposed targets

All numerical targets below are **proposed research goals**, not measured results.

**Classification:** F1, precision, recall, specificity, balanced accuracy, MCC, AUCPR, Brier score, log loss, calibration curve and false negatives per site. Report pooled and macro-site metrics.

**Velocity:** (R^2), MAE/RMSE in physical units, log-velocity MAE, median absolute error, P90/P95 quantile error, tail bias and predictive interval score. Include measured-label uncertainty and the sensitivity of the onset/threshold-derived target.

**Calibration and transfer:** nominal 90% interval coverage, width, CRPS or interval score, per-site calibration, worst-site degradation, OOD detection/abstention and coverage under site/ecoregion shift.

**Proposed goals:**

| Gate | Proposed target |
|---|---|
| A — exact reproduction | F1 within ±0.03, accuracy within ±0.03, and (R^2) within ±0.05 of the cited paper on a verified equivalent split |
| B — site-held-out PF proxy | Macro-site F1 ≥0.85, with per-site distributions and group-bootstrap uncertainty |
| B — site-held-out apparent velocity | (R^2) ≥0.50 and ≥20% lower MAE than the strongest matched baseline; report if the target is not attainable |
| Velocity tails | ≥20% lower P90/P95 quantile error than the strongest matched baseline, without degrading median error materially |
| Calibration | Nominal 90% intervals achieve 85–95% coverage on held-out sites, with interval width and CRPS/interval score reported |
| Transfer | No held-out site silently omitted; report worst-site performance and all domain failures |
| Independent reference | Proxy-vs-tracer agreement reported separately; no proxy-only score described as tracer-confirmed PF detection |
| Robustness | Reported performance under sensor dropout, rainfall-gauge substitution, temporal coarsening, onset-threshold changes and calibration drift |
| Physical consistency | Zero negative physical water volumes; all event water balances and nutrient exports report residual/coverage status and cannot hide unmeasured pathways |

Do not compare the site-held-out goals directly against the paper's 0.91 F1 as if they were the same test. The paper's score is a temporal per-sensor result; the proposed benchmark is a harder spatial-transfer estimand. To claim a numerical win, all candidate models must be rerun on the identical frozen site-held-out folds.

## 6.1 Treatment-effect boundary

NEON PF prediction is a transportability benchmark, not amendment-effect
evidence. Any claim that a soil amendment changes event response must use the
[Paired-Forcing Causal Benchmark](./PAIRED_FORCING_CAUSAL_BENCHMARK_V0.1.md)
with randomized treatment units, matched/measured hydrologic forcing and
independent nutrient-mass outcomes.

## 7. Adversarial and counterfactual challenge suite

The model should be tested on event pairs that distinguish precipitation amount from event structure:

- Equal total precipitation, different peak intensities and durations.
- Equal peak intensity, different storm totals and inter-pulse gaps.
- Same rainfall forcing, different antecedent moisture states.
- A first-flush event followed by repeated wetting/drying cycles.
- Different sensor depths and soil profiles under matched storm forcing.
- Sensor dropouts, delayed timestamps, quality-flag changes and calibration drift.
- Rainfall gauge swap: legacy DP1.00006.001 vs DP1.00045.001 tipping bucket where overlap exists.
- Time-resolution degradation: 1-minute vs 10-minute vs 30-minute precipitation.
- Sensitivity to 0.005, 0.01 and 0.02 m³/m³ soil-moisture response thresholds.
- Sensitivity to Rosetta3 (K_{sat}) uncertainty and alternative plausible hydraulic parameterizations.
- Extreme velocities outside the training range and sites with unusual soil structure/root/bio-pore properties.
- Proxy-positive event with no independent tracer corroboration, and tracer-positive event that fails the proxy threshold.

A credible model should widen uncertainty or abstain when the data are insufficient; it should not become confidently wrong when gauge resolution or soil structure changes.

## 8. Paradigm shifts worth pursuing

1. **From binary PF to pathway distributions:** estimate arrival-time distributions and velocity quantiles by depth, not only yes/no PF.
2. **From a single event scalar to a depth-resolved event graph:** represent each sensor/depth as a node with calibrated observations and physical depth links; model rapid bypass and slower matrix response as competing pathways.
3. **From pure pattern recognition to physics-constrained residual learning:** let a validated hydraulic model provide a matrix-flow prior, and learn only identifiable departures attributable to transient structure/preferential pathways.
4. **From pooled average scores to transport guarantees:** report calibration, worst-site performance, OOD abstention and independent tracer agreement.
5. **From fixed sensor layout to value-of-information design:** select depths/plots that most reduce uncertainty about arrival time, pathway partition and nutrient breakthrough; score the sensor selection by measured uncertainty reduction per cost.
6. **From rainfall totals to event-shape counterfactuals:** test how duration, peak intensity, pulse spacing and antecedent state alter pathway probabilities and upper-tail velocity.
7. **From hydrology-only to coupled mass transport:** only after independently measured discharge and solute/sediment samples exist, predict event-integrated nutrient export. NEON soil moisture and rainfall alone do not provide biochar effects or nutrient-flux labels.

## 8A. Derived-database bridge track

The 2025 HydroShare [NEON Soil Preferential Flow Database v1.1](https://doi.org/10.4211/hs.847b15cd15524b78acd58a2a8be242c1) is a practical bridge dataset: its readme includes event-level precipitation/soil-moisture response, smBeforePrecip, smOnsetTime, apparent velocity and multiple Rosetta Ksat variants. It is derived from NEON RELEASE-2023 and provisional observations plus soil/site metadata, and its resource page declares CC BY 4.0. The reported package is approximately 1.0 GB and its contents are externally hosted.

Use it to bootstrap data-dictionary and eventizer crosswalk work, test Ksat-threshold sensitivity and inspect missingness before rebuilding the raw 2026 cohort. It is not independent of NEON, not necessarily identical to the 2026 article's model matrix, and not a substitute for exact-release raw reconstruction. The source-specific caveat that BONA, DEJU, HEAL, TOOL and BARR are not recommended, while KONA is excluded, must be retained as dataset metadata rather than converted into a guessed 2026 site roster.

**Complementary validation design:** [Multi-method PF triangulation benchmark](./PREFERENTIAL_FLOW_MULTIMETHOD_TRIANGULATION_BENCHMARK_V0.1.md) and [implementation issue #5538](https://github.com/Luminous-Dynamics/mycelix/issues/5538).

## 9. Data provenance, overlap and stop rules

- Freeze release tag, site roster, file inventory, publisher checksum, local SHA-256, query/API response, product code, sensor metadata and preprocessing commit before modelling.
- The 2025 study on preferential flow across 17 ecoregions and the 2026 NEON velocity study are not independent field networks simply because they are separate publications. Both use NEON observations; reconcile shared sites/sensors/events before any combined synthesis or external-validation claim.
- Keep precipitation products and measurement methods as distinct input sources. Throughfall is not gross precipitation.
- Never assign a new product release to a historical paper without a version crosswalk and overlap audit.
- A site, sensor, event, source record or transformation must not cross confirmatory partitions when the target claim requires its independence.
- Preserve original raw data and checksums. Synthetic fixtures are software conformance tests, not observations.
- Stop confirmatory analysis if the exact site/sensor mapping, release history, data license, or QA flags cannot be reconciled.

## 10. Role of the Luminous stack

- **Mycelix:** immutable source, site, plot, sensor, event, release, calibration, tracer, split and reviewer lineage; connected-component leakage checks.
- **Symtropy:** reference hydraulic model, depth-resolved event propagation, matrix/preferential pathway hypotheses, conservation and matched transient solver comparisons.
- **Symthaea:** competing model/hypothesis tracking, uncertainty-aware sensor value-of-information and abstention when site support or evidence is insufficient.
- **Independent hydrologists/labs:** validate the physical meaning of the proxy, tracer reference, instrument errors, soil structure and local transfer.

Hashes establish artifact identity. They do not prove that a sensor measures a unique pore pathway, that a threshold is ground truth, that model parameters are identifiable, or that an amendment caused a change.

## 11. Reproducible acquisition

The companion `tools/acquire-neon-data.sh` helper queries the NEON Data Query
endpoint for one product/site/month range, preserves the exact query response,
verifies publisher file size and MD5, computes local SHA-256, and refuses to
overwrite existing artifacts. It reads `NEON_API_TOKEN` from the environment,
writes it to a protected temporary header file, and does not forward it to a
signed storage URL. A file is not declared acquired until its checksum record
is present in the local acquisition manifest.

## References

- Li, B. et al. (2026). “Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes.” *Earth's Future*. https://doi.org/10.1029/2026EF008296
- Li, B. et al. (2025). “Ubiquity and Causes of Soil Water Preferential Flow Across 17 Ecoregions.” *Geophysical Research Letters*. https://doi.org/10.1029/2025GL118045
- NEON (RELEASE-2026). “Soil water content and water salinity,” DP1.00094.001. https://data.neonscience.org/data-products/DP1.00094.001/RELEASE-2026
- NEON (6 October 2025). “Precipitation (DP1.00006.001) has been split into three data products.” https://www.neonscience.org/impact/observatory-blog/precipitation-dp100006001-has-been-split-three-data-products
- NEON (21 October 2025). “Data Product DP1.00006.001 (Precipitation) has been Deprecated.” https://www.neonscience.org/impact/observatory-blog/data-product-dp100006001-precipitation-has-been-deprecated
- NEON Data API. “Data Products Endpoint” and “Data Query Endpoint.” https://data.neonscience.org/data-api/endpoints/products/ and https://data.neonscience.org/data-api/endpoints/data-query/
