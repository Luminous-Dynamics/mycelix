# Hirmas Dynamic Morphology–Hydrology Reproduction Benchmark v0.1

**Status:** replication contract and staged acceptance plan. The Hirmas Dryad archive has been identified but has not been acquired or independently hashed in this work. The new per-pore Rust CLI is a software implementation candidate; it is not yet a verified reproduction. All targets below are proposed acceptance gates, not achieved results.

**Trackers:** [TERRA-MORPH-DYN-001](https://github.com/Luminous-Dynamics/mycelix/issues/5545) · [TERRA-HIRMAS-PORE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5568).

## 1. Why this is a high-value benchmark

The Hirmas et al. (2025) study is a tractable cross-scale bridge between 2D pore morphology, matrix-water state, dynamic interpedal pore geometry, water retention and hydraulic conductivity. Its released archive is about 4.6 MB, contains three horizon-specific ImageJ MLT object tables (Ap1, Bt and Btkss2), soil-water observations at 10/40/120 cm, soil physical properties, model outputs and analysis scripts. It is small enough to be a useful independent implementation benchmark before the larger NEON acquisition is complete.

However, it is a single Konza Prairie APT soil profile with shrink–swell behaviour. Its coefficients and model assumptions must not be transferred directly to Terra Preta, African soils or South African fields.

## 2. Frozen source acquisition

Source: Hirmas et al. (2025), Dryad DOI [10.5061/dryad.zs7h44jkn](https://doi.org/10.5061/dryad.zs7h44jkn), CC0 1.0.

Acquire the exact version, including the data-analysis archive and README. Before parsing, record:

- landing page and file URLs, filenames, byte sizes, publisher checksums (if supplied), local SHA-256 and the license notice;
- the analysis-script archive's exact hash and all dependent R package versions;
- the three horizon ImageJ tables, raw/processed matrix-water series, soil properties, weather/forcing, reported model outputs and excluded/missing intervals;
- the image scaling (180 µm grid), ROI, segmentation threshold, object size filter and column-heading/unit mapping;
- author script entry points and generated intermediate/output filenames.

No row count, horizon count, timestamp range or unit conversion should be assumed from a third-party catalogue. Reconcile them from the downloaded README, exact files and scripts.

## 3. Replication lanes

### Lane A — author reference

Run the archived R workflow in a pinned environment and capture all inputs, outputs, console logs, script digest, package lock and environment information. The original R outputs are the reference implementation, not infallible ground truth: independently inspect units, hidden defaults, missing-data rules, image thresholds and the published Eq. 37 sign inconsistency.

### Lane B — Rust average-geometry model

Run `tools/dynamic-macropore-audit` on a crosswalked summary of each horizon. This implementation covers the dynamic average-geometry/saturated-conductivity core but does not yet implement all per-pore terms or the unsaturated conductivity function.

### Lane C — Rust per-pore model

Run `tools/hirmas-pore-resolved-audit` on the exact ImageJ object table, dynamic report and provenance-bound config. It implements object-level Eq. 35/36 fractions, Eq. 33 pore matric head and Eq. 46 cumulative retention curve, while preserving the aggregate Eq. 22 fraction in parallel.

The proportional per-object width scaling `d_i(theta_m) = d_i,dry × d(theta_m)/d_ds` is currently an explicit implementation assumption. It must be reconciled against author R code before claiming numerical reproduction.

## 4. Equation and artifact parity gates

| Gate | Required comparison | Proposed acceptance target |
|---|---|---|
| Image object crosswalk | retained object IDs/count, area, perimeter, MinFeret, units, scale and ROI | Exact object/column mapping and counts; no silent drops |
| Dry geometry | A_p, perimeter, geometric-mean MinFeret, w_ds and W | Relative error <= 1e-6 after exact unit conversion, or a documented source/rounding discrepancy |
| Matrix-water partition | matrix/macropore theta per timestamp and sensor | Compare exact timestamps; max absolute theta error <= 1e-6 where algorithms and inputs are identical |
| Dynamic geometry | d(theta_m), w(theta_m), COLE and f_p per state | Compare every state; no averaged-only headline score |
| Per-pore retention | d_i, R_pi, f_pi, h_i and cumulative theta_p(h) | Per-pore point count and sorting identical; max relative error <= 1e-6 where equations/inputs exactly match |
| Saturated conductivity | K_sp and whole-soil K_s by horizon/time | Log10(Ks) MAE <= 0.01 on equation-identical cases, with all differences explained |
| Unsaturated conductivity | K(theta, theta_m) surface and pressure dependence | Not claimed by the current Rust v0.1 tool; separate implementation and parity gate required |
| Validation | original versus Stewart et al. (2016) model comparison as reported by authors | Reproduce authors' metric definitions, partitions and reported values; no new fit to the same validation points |

These are engineering parity targets, not evidence that all published uncertainty is negligible. When numerical parity is blocked by an ambiguous paper equation or undocumented data-cleaning rule, label the output unresolved rather than loosening the tolerance after seeing the result.

## 5. Publication ambiguities and explicit adjudication

### Structural alpha relation

The body prints Eq. 37 with a negative beta_1 term, while Figure 6 and Table 1 support a positive near-1:1 relationship and positive beta_1. The current Rust average-geometry tool follows the plotted Figure 6/Table 1 relationship provisionally. The author R script must determine which expression generated the actual reported outputs. Report both branches during parity analysis; do not silently choose one.

### Image object threshold

The source protocol retains pores with projected object area **strictly greater than pi mm²**. An object exactly at the threshold is excluded. Preserve the source's precise inequality and exact image scale. Do not generalize that minimum detectable pore size to other scanners, soils or study questions.

### Pore width representation

The average-geometry Ks equations and per-pore retention equations use different levels of representation. Do not assume that the sum of individual f_pi terms must equal the aggregate Eq. 22 estimate. The new CLI reports the difference so the original assumptions and object geometry can be investigated.

## 6. Generalization and scientific challenge set

After reproducing the Konza reference, freeze a second-stage challenge that varies one scientific dimension at a time:

1. **Geometry resolution:** multiple voxel sizes, image orientation and segmentation thresholds; quantify object split/merge and MinFeret sensitivity.
2. **Pore shape:** equal-area pores with different MinFeret, aspect ratio and perimeter; test when average width fails to preserve retention.
3. **Horizon:** Ap1 versus Bt versus Btkss2; no coefficient retuning on a held-out horizon for the zero-shot transfer test.
4. **Moisture history:** wetting and drying sequences at the same matrix theta; identify the current model's lack of hysteresis as an explicit limitation.
5. **Sensor depth:** 10, 40 and 120 cm; test lag, missing intervals, sensor saturation and logger downtime rather than interpolating across invalid periods.
6. **Uncertainty:** propagate segmentation/scale error, bulk-density and COLE uncertainty, matrix-water partition uncertainty and model coefficient uncertainty.
7. **Independent transport:** compare retention/Ks predictions with measured transient flux, tracer breakthrough and water balance where data support those endpoints.

## 7. Industry and SOTA comparison

Compare the same frozen profiles/forcing and measurement support against: the author's original R implementation; Hirmas's published dynamic morphology model; the corresponding static-morphology version; HYDRUS (pin exact module/geometry/parameter set); a matrix-only Richards baseline; and a clearly declared empirical Ks baseline.

Score per horizon and event, not merely across all rows:

- dynamic pore width and pore-volume-fraction trajectory error;
- retention-curve point error and integrated curve error in log suction;
- saturated Ks error in log space, bias and calibration;
- water/solute boundary-flux residual when transport outputs are actually measured;
- robustness to image segmentation, resolution, sensor cadence and matrix-water partition;
- runtime, image processing burden, parameter count and independent measurements required.

The useful paradigm shift is not only a better fit. It is a **geometry-to-retention-to-flux evidence chain** in which every inferred intermediate state is compared with independent measurements and its uncertainty is propagated forward.

## 8. What is not yet validated

- The Dryad archive has not been downloaded, hashed or run.
- The Rust average-geometry and per-pore CLIs have not passed exact-head CI yet.
- The per-pore width scaling has not been checked against author R scripts.
- The Rust CLIs do not yet reproduce the matrix-water partition, unsaturated conductivity, full water-retention parameter refitting or the reported Stewart et al. validation workflow.
- No result establishes 3D connectivity, actual storm-time pore wetting, preferential flux, amendment efficacy or transfer to South African soils.

## References

- Hirmas et al. (2025). Predicting Soil Interpedal Macroporosity and Hydraulic Conductivity Dynamics. *Water Resources Research*, 61(9), e2024WR039836. https://doi.org/10.1029/2024WR039836
- Hirmas et al. (2025). Dryad dataset and analysis files. https://doi.org/10.5061/dryad.zs7h44jkn
- Chaif et al. (2025). CT-derived matrix–macropore interface exchange. *Journal of Hydrology*, 661, 133791. https://doi.org/10.1016/j.jhydrol.2025.133791