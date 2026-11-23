# Soil Moisture Spatial-Heterogeneity Benchmark v0.1

**Status:** benchmark specification plus a deterministic Rust spatial-dispersion auditor at [tools/spatial-heterogeneity-audit](./tools/spatial-heterogeneity-audit/README.md). No field score, tracer validation or physical pathway certificate is claimed. Thresholds are proposed and must be preregistered.

## 1. Why this method family matters

Point sensors can miss narrow or laterally displaced flow paths. A localized pathway may produce a spatially heterogeneous wetting response even when individual depth traces look sequential or inconclusive. The 2025 review by Nimmo et al. identifies spatial patterns and evolution of soil-moisture heterogeneity as one of six method families for PF identification. It also warns that finite sensor spacing can miss narrow pathways and that heterogeneity signatures may reflect general heterogeneous flow rather than uniquely preferential flow.

The scientific target is therefore **spatially localized transient wetting**, not an unqualified PF Boolean. A heterogeneous response is a useful signature that must be triangulated with profile order, velocity, flow balance, geophysical imaging and tracer evidence.

## 2. Required observation geometry
For NEON DP1.00094.001 specifically, use `swc_depthsV3.csv` rather than the ordinary `sensor_positions` zOffset for depth. Join `sensor_positions` by HOR/VER and temporal validity; sensor relocations must split the spatial record. Use coordinate offsets with their reference metadata and uncertainty, not raw degree differences. See [NEON soil-sensor tutorial](https://www.neonscience.org/resources/learning-hub/tutorials/soil-sensors-intro) and [NEON geolocation tutorial](https://www.neonscience.org/resources/learning-hub/tutorials/neon-spatial-data-basics).


Each event record should identify:

- site-level spatial_group_id, event and time window, with multiple nested plot IDs allowed;
- sensor ID, plot ID, depth_group_id (a declared physical-depth band), corrected measured depth, and the mapping evidence defining that band;
- sensor x/y in one common site-level projected coordinate reference, measurement volume/support, sensor type, calibration and installation metadata;
- common event-time grid, source sampling interval and aggregation rule;
- directly observed, interpolated, missing, flagged, saturated/frozen and sensor-fault states;
- pre-event baseline, event forcing, and depth-specific theta response.

Do not pool unlike sensor types or depth layers without a measurement-support crosswalk. For a multi-plot network, compare sensors within predeclared depth_group_id bands while preserving actual min/median/max corrected depth and per-plot sensor counts. Plot-local offsets must be transformed using each position's reference origin into a shared projected site frame before computing cross-plot distances; raw x/y offsets from different reference origins are not comparable. Arrays should include at least three spatial locations per assessed depth for a basic dispersion estimate; variogram or two-dimensional spatial structure requires broader spatial support and meaningful separation distances. Report the actual plot/depth layout rather than asserting that three sensors are sufficient to map a pathway.

## 3. Event-relative spatial features

For sensor i at position s_i and depth z, define the event-relative change:

\[\Delta\theta_i(t,z)=\theta_i(t,z)-\theta_{i,baseline}(z).\]

At each event time and depth, report the number of usable sensors, positive-response fraction, mean, median, standard deviation, interquartile range, median absolute deviation, robust coefficient of variation where defined, full range, and the maximum-to-median positive-response ratio. Preserve missing and indeterminate counts; never impute them to zero.

Where spatial geometry is adequate, report:

- pairwise absolute response differences as a function of sensor separation;
- empirical semivariogram \(\gamma(h)=\frac{1}{2N(h)}\sum_{(i,j)\in h}(\Delta\theta_i-\Delta\theta_j)^2\), with pair counts and distance-bin limits;
- spatial clustering/autocorrelation only when the sensor count and spatial graph support it, with the weight matrix and null model included;
- sensor-layout sensitivity after removing locations or subsampling to the density of a candidate deployment.

These metrics quantify heterogeneity and localization. They do not by themselves infer pore connectivity, vertical direction or water flux.

## 4. Challenge families and negative controls

| Family | Known/controlled structure | Confounder to distinguish |
|---|---|---|
| Homogeneous matrix response | Similar response across locations at each depth | False spatial-PF flag from random noise |
| Localized connected path | One narrow wetting path crosses some sensors and reaches a lower boundary | True localized bypass that sparse layouts may miss |
| Localized disconnected path | Strong local wetting without lower-boundary breakthrough | Local anomaly mistaken for profile-spanning preferential flow |
| Finger flow | One or more spatially localized fingers with known widths/trajectories | Heterogeneity is present but a sensor array may intersect only a small part |
| Lateral interflow | Wetting follows a restrictive horizon with downslope propagation | Lateral routing mislabelled as vertical macropore bypass |
| Texture/structure boundary | Persistent spatial contrast independent of event flow regime | Static soil heterogeneity mistaken for event-generated PF |
| Root/biopore patch | Localized response co-located with known roots/biopores | Natural spatial heterogeneity versus connected fast transport |
| Sensor drift / calibration offset | One sensor slowly shifts independent of water input | Instrument error mistaken for a wetting hotspot |
| Unequal sensor support | Same true field but different measurement volumes/types | Apparent variability caused by noncomparable sensors |
| Saturated/frozen/flagged | Some sensors cannot measure event theta response reliably | Invalid zero-change interpreted as homogeneous/no PF |
| Layout aliasing | Same latent field sampled by dense, sparse, clustered and transect layouts | False confidence from convenient sensor placement |

Include pre-event placebo windows and dry/no-input windows. A spatial hotspot that persists unchanged before rainfall should not be scored as a newly triggered event pathway.

## 5. Multi-method triangulation

Pair spatial heterogeneity with:

- NSR response-order intervals and sensor coverage;
- VT proxy values under a declared Ksat reference and uncertainty range;
- trajectory maximum-rate and amplitude signatures;
- pressure head and profile water balance where measured;
- ERT/GPR imaging with inversion resolution/uncertainty;
- conservative tracer breakthrough and recovered mass at depth/boundary;
- dye mapping or destructive pore-network evidence where sampled.

Report agreement and discordance by evidence tier. Do not train one proxy as the target and then describe agreement with that proxy as independent validation. Heterogeneity-positive / tracer-negative and heterogeneity-negative / tracer-positive cases are essential learning cases, not inconvenient exclusions.

## 6. Metrics and proposed acceptance targets

| Dimension | Metric | Proposed target (not achieved) |
|---|---|---|
| Null safety | False localized-pathway claims on homogeneous, static-texture and sensor-drift controls | Zero physical pathway certificates; proxy signatures may be reported as such |
| Localized-path detection | Sensitivity against independently tracer/geophysical-adjudicated events | At least 0.90 for high-signal synthetic paths and report real-field uncertainty |
| Spatial recovery | Hotspot localization error, variogram error, connectivity/map overlap | At least 20% improvement over a uniform-response baseline at matched sensor count |
| Layout robustness | Sensitivity/false-positive versus sensor count, spacing and transect orientation | Publish complete performance curves; no single density is assumed universally sufficient |
| Calibration | Predictive intervals for spatial response features | Nominal 90% intervals achieve 85–95% coverage on held-out profiles/sites |
| Decision value | Additional sensors or geophysical transects per reduction in localization/flux uncertainty | Demonstrate a cost-normalized improvement over a fixed-layout design |

A candidate fails if spatial anomaly detection improves while sensor-drift false positives, mass-balance validity, tracer agreement or worst-layout performance deteriorates.

## 7. Adaptive measurement design

Use the spatial audit to choose where to measure next. Under a fixed budget, compare:

1. uniform grid placement;
2. stratified placement by horizon/topography/root zones;
3. adaptive placement near predicted wetting-front heterogeneity;
4. a hybrid: sparse permanent monitoring plus event-triggered mobile ERT/GPR/tracer sampling.

Use expected reduction in decision-relevant uncertainty (pathway localization, lower-boundary arrival or nutrient export), not entropy alone. Record the selection probability and whether each planned measurement was actually acquired; this avoids verification bias from validating only model-selected hotspots.

## 8. Implementation sequence

The v0.1 executable computes per-depth-group/time event-relative dispersion and distance-binned empirical semivariograms across a site-level spatial group that may contain several plots. It preserves actual depth ranges, per-plot sensor counts and pair support, and tests sensor-order and translation/rotation invariance plus a homogeneous matrix-only null. It is a descriptive statistic engine; localized-path classification, layout ablation, and ERT/GPR/tracer validation remain open.

1. Build a deterministic spatial observation-operator generator with homogeneous, localized, disconnected and lateral-flow truth patterns.
2. Implement per-depth spatial dispersion and distance-binned semivariogram reports with strict support and missingness checks.
3. Add sensor drift, calibration offsets and unequal support negative controls.
4. Run dense-truth to sparse-layout ablations and cadence/threshold sensitivity.
5. Join spatial signatures to the existing profile-flow, threshold, trajectory and triangulation reports through immutable event/profile IDs and source digests.
6. Validate on independent ERT/GPR/tracer data before any pathway-certificate claim.

## 9. References and related work

- Nimmo, J. R., Wiekenkamp, I., Araki, R., et al. (2025). Identifying preferential flow from soil moisture time series: Review of methodologies. Vadose Zone Journal, 24(2), e70017. https://doi.org/10.1002/vzj2.70017
- Li, B. et al. (2026). Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes. Earth's Future. https://doi.org/10.1029/2026EF008296
- Implementation tracker: [PF-SPATIAL-001](https://github.com/Luminous-Dynamics/mycelix/issues/5557).