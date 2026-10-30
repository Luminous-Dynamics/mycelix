# Preferential-Flow Multi-Method Triangulation Benchmark v0.1

**Status:** benchmark specification. No multi-instrument field campaign, tracer validation, or multi-method score has been executed. Thresholds below are proposed design targets, not measured performance.

## 1. Why a single PF label is not ground truth

Preferential flow is spatially localized and transient. A point soil-moisture sensor can miss a pathway that bypasses its small sensing volume. A rapid theta rise can also be caused by piston displacement of pre-event water or a rising water table, not necessarily a connected macropore. The nonsequential-response (NSR) method and velocity-threshold (VT) method therefore fail differently: NSR can be conservative and yield false negatives; VT can classify fast responses as preferential when other processes explain them.

The 2025 review by Nimmo et al. recommends testing soil-moisture PF methods against independent evidence such as tracer, geophysical and groundwater observations. It also notes that these methods have different temporal/spatial coverage and that no universal reference method exists. The benchmark should therefore retain a vector of evidence, not collapse everything into one unqualified PF Boolean.

## 2. Evidence ladder and claim boundaries

| Evidence tier | Measurement | What it supports | What it does not establish alone |
|---|---|---|---|
| E0 — rainfall/event | Calibrated high-frequency rainfall, antecedent moisture | Event forcing, timing and exposure | Subsurface flow mechanism |
| E1 — point response | Single-depth theta response / response velocity | Local wetting signature and sensor lag | Connected pathway or vertical continuity |
| E2 — profile coherence | Multi-depth NSR, VT, depth ordering, pressure head | Vertical response structure; candidate bypass | Unique pore structure or tracer mass transport |
| E3 — spatial imaging | Time-lapse ERT/GPR with calibrated observation model | Wetting-front heterogeneity and possible lateral/vertical structure | Resolution of every narrow macropore or exact flux without inversion uncertainty |
| E4 — tracer breakthrough | Conservative tracer / isotopic tracer at depth and drainage, time-resolved samples | Arrival-time distribution, pathway transport and mass recovery | Full pore geometry from one sampler or one event |
| E5 — destructive pathway map | Dye staining, excavation and image analysis | Spatial pathway morphology at sampled location/time | Continuous event frequency or landscape-wide prevalence |
| E6 — boundary confirmation | Water table, lysimeter, drainage flux and chemical mass | Deep arrival / profile or plot-scale boundary export | Mechanism without complementary profile evidence |

Do not set E1–E2 proxy labels equal to E4–E6 physical evidence. Dye tracing is spatially informative but usually a destructive snapshot; continuous solute/isotope tracers and breakthrough curves complement it. ERT/GPR can help distinguish lateral from vertical wetting, but spatial resolution and inversion uncertainty remain explicit.

## 3. Required simultaneous diagnostics

### 3.1 Point sensor and event response

- Per-sensor theta response, event onset, antecedent theta, response magnitude and response delay.
- Time resolution and missingness must be sensitivity variables, not just metadata.
- Record sensor type, sensing volume, calibration, depth, installation disturbance and vertical alignment.
- Use event-specific QC; sensor response may be absent under saturated/frozen antecedent conditions.

### 3.2 Nonsequential response (NSR)

For each rainfall event and profile, compare response times across ordered sensor depths. An NSR pair is a deeper sensor that responds before a shallower sensor. Preserve all pairs and censoring; a single aggregate NSR flag loses information. Require at least two responding depths and report profile coverage. A sequential response is not proof of matrix-only flow because a pathway may bypass uninstrumented layers or sensors.

### 3.3 Velocity threshold (VT)

Calculate the declared sensor/event response velocity and compare with an explicitly named matrix-flow reference. Run sensitivity across Ksat point estimate, upper/lower uncertainty bounds, and independent matrix-flow model quantiles. Preserve the velocity and reference in common units and the exact reference evidence. A Ksat-threshold flag is a model-conditional proxy, not tracer truth.

### 3.4 Geophysical and tracer references

- Time-lapse ERT/GPR: spatially map wetting and test vertical versus lateral routing. Register survey geometry to sensor depths and provide inversion resolution/uncertainty.
- Conservative tracer: sample concentration and flow through time at multiple depths/boundaries; compute full breakthrough curves, first arrival, centroid, peak, tail quantiles and recovered mass.
- Reactive tracer: use only with calibrated sorption/decay and a mass-balance model; do not interpret delayed arrival as absence of water bypass.
- Dye: quantify stained area, connectedness and depth extent with repeatable image segmentation and blinded review. Treat as event/site-specific structural evidence.
- Water-table response: distinguish deep connected transport from local wetting; model saturated-zone response and antecedent water-table state.

## 4. Known-confounder challenge set

| Confounder | Expected failure | Required control |
|---|---|---|
| Matrix piston flow displaces pre-event water | Rapid deep theta response appears like PF | Conservative tracer and matrix-only simulator with pre-event water tags |
| Rising water table intersects sensors | Rapid theta rise produces false PF | Water-table record and saturated-zone state |
| Localized macropore bypasses a probe | Sensor misses true PF | Replicated probes, ERT/GPR, tracer and layout subsampling |
| Lateral interflow along restrictive horizon | NSR appears to imply vertical bypass | 2D/3D imaging and downslope samplers |
| High antecedent saturation | Theta sensor cannot register a large increase | Pressure head, drainage and tracer response; mark theta method unavailable |
| Sensor intervals too coarse | Fast transient response is missed | Downsample high-frequency truth to 1, 5, 10, 20, 30, 60 min and quantify degradation |
| Sparse sensor depths | Profile ordering is ambiguous | Depth-ablation and missing-depth simulation |
| Soil heterogeneity / sensor volume mismatch | Local sensors disagree | Replicated sensor types, calibration and measurement support model |
| Ksat uncertainty / pedotransfer shift | VT label flips without physical change | Propagate Ksat uncertainty; run bound/ensemble sensitivity |
| Tracer non-recovery / sampler bias | False-negative tracer result | Blank/spike/recovery controls and independently measured water balance |

## 5. Benchmark design and metrics

Use paired events/sites with all feasible methods recorded concurrently. Freeze event and profile definitions before examining labels. Have an independent team adjudicate tracer/geophysical/boundary evidence blinded to model output. Keep uncertain/mixed evidence as such; use tiered positive/negative/indeterminate labels.

Report:

- NSR/VT agreement, discordance matrix and event-level Cohen's kappa only as agreement, not accuracy against truth.
- Sensitivity/specificity/MCC/AUCPR against each independent reference tier separately, with sampling uncertainty and clustered resampling at the experimental unit/site.
- First-arrival, median/centroid and P90/P95 breakthrough-tail timing errors; peak concentration error; recovered tracer/water mass.
- Pathway map overlap and connectivity metrics for ERT/GPR/dye, accounting for survey registration and spatial resolution.
- Water/solute mass-balance closure with uncertainty; separate measured and model-estimated terms.
- Calibration and abstention: selective risk versus coverage, reliability diagrams and worst-site/family coverage.
- Robustness versus sensor depth count, vertical spacing, horizontal replication, sensor type and sampling cadence.
- Acquisition burden: installed sensors, sampling labour, tracer handling, geophysical survey time, data processing and cost per independently adjudicated event.

## 6. Proposed acceptance targets

These are planning goals. They are not results and should be frozen only after a pilot estimates achievable measurement noise.

| Gate | Proposed target |
|---|---|
| Matrix-only negative controls | Zero false physical pathway certificates; proxy flags may be reported as proxy disagreements but cannot be promoted to confirmation |
| Strong tracer-positive controls | At least 0.90 sensitivity for high-signal events, with independent reference quality stated |
| Mass balance | Conservative tracer and water residuals within pre-registered combined measurement/numerical uncertainty |
| Calibration | Nominal 90% predictive intervals cover 85–95% on external profile/site strata, with worst-stratum coverage shown |
| Sampling design | Identify the smallest sensor/cadence configuration that remains within 5 percentage points of best observed detection sensitivity, without increasing false certificates |
| Method robustness | Publish performance by soil structure, antecedent moisture, event intensity, sensor layout and site; no pooled-only headline score |
| Experiment value | Sensor or tracer additions must yield measurable reduction in decision-relevant uncertainty per cost against a fixed baseline design |

## 7. Design-of-experiment paradigm shift

Do not instrument every plot identically by default. Use a staged, adaptive design:

1. A low-cost monitoring screen identifies candidate events and site strata using high-frequency rainfall and multi-depth moisture.
2. An uncertainty-aware selector chooses where to add pressure sensors, replicated probes, tracer samplers or ERT/GPR transects to discriminate competing explanations.
3. A randomized, paired lysimeter/core experiment validates matrix-only versus dual-domain mechanism under matched forcing.
4. A subset of events receives tracer/geophysical adjudication. Sampling times are selected to resolve early arrival and late tail, not just peak concentration.
5. All selection probabilities, stopping rules and missing/failed samples are recorded to avoid verification bias.

The value-of-information policy must be evaluated prospectively: compare adaptive sensor choice with fixed-layout control, using total experiment cost and downstream mechanism/flux error.

## 8. Data and implementation links

- NEON event-scale predictor and transfer track: PREFERENTIAL_FLOW_TRANSFER_BENCHMARK_V0.1.md.
- Derived HydroShare event dataset and raw NEON acquisition: ACQUISITION_V0.1.md.
- Synthetic mechanism identifiability suite: SYNTHETIC_KNOWN_TRUTH_BENCHMARK_V0.1.md.
- Industry model parity: INDUSTRY_PHYSICS_COMPARATOR_BENCHMARK_V0.1.md.
- Paired hydrologic forcing causal design: PAIRED_FORCING_CAUSAL_BENCHMARK_V0.1.md.

Implementation tracker: [PF-TRIANG-001](https://github.com/Luminous-Dynamics/mycelix/issues/5538).

**Executable first diagnostic:** [profile-flow-audit](./tools/profile-flow-audit/README.md) implements pairwise multi-depth NSR response ordering; [preferential-flow-eventizer](./tools/preferential-flow-eventizer/README.md) supplies the single-sensor event response/VT proxy. Neither independently certifies a physical pathway.

## References

- Nimmo, J. R., Wiekenkamp, I., Araki, R., et al. (2025). Identifying preferential flow from soil moisture time series: Review of methodologies. Vadose Zone Journal, 24(2), e70017. https://doi.org/10.1002/vzj2.70017
- Li, B. et al. (2026). Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes. Earth's Future. https://doi.org/10.1029/2026EF008296
- NEON Soil Preferential Flow Database v1.1 (2025). HydroShare. https://doi.org/10.4211/hs.847b15cd15524b78acd58a2a8be242c1