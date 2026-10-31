# Dynamic Macropore Morphology–Hydrology Coupling Benchmark v0.1

**Status:** reproducibility and validation plan. The Hirmas et al. Dryad dataset has been identified but not acquired or rerun in this work. An independent Rust equation-level audit now exists at [tools/dynamic-macropore-audit](./tools/dynamic-macropore-audit/README.md); it is covered by PR CI but its tests have not yet been verified on the current exact head. All acceptance thresholds below are proposed targets, not achieved performance. No field treatment or hydraulic recommendation is authorized.

**Implementation tracker:** [TERRA-MORPH-DYN-001](https://github.com/Luminous-Dynamics/mycelix/issues/5545).

## 1. Paradigm shift: pore geometry is a state, not just a static soil attribute

Many hydraulic workflows characterize a soil profile with fixed retention and conductivity curves. In structured shrink–swell soils, interpedal pores can expand as the matrix dries and contract as matrix water content rises. The geometry that controls bypass and the conductivity connecting pore domains can therefore evolve over the same wetting/drying event being predicted.

Hirmas et al. (2025) present a useful bridge: combine dry-state multistripe laser triangulation (MLT) cross-section geometry with time-varying matrix water content to estimate interpedal pore width and dynamic hydraulic conductivity, and partition total sensor water content into matrix and macropore contributions. Their Konza APT dataset includes three imaged horizons and moisture sensors at 10, 40 and 120 cm. This is a mechanism-focused benchmark source, not a broad external validation set.

## 2. Frozen reproduction dataset

Source: [Hirmas et al. (2025), Dryad DOI 10.5061/dryad.zs7h44jkn](https://doi.org/10.5061/dryad.zs7h44jkn). Dataset landing page reports CC0 1.0, published 14 August 2025, 4.60 MB total.

Key files documented in the source README:

| File | Evidence/use |
|---|---|
| `KONZA_APT_Ap1_0.18mmResolution_Analyze.csv` | ImageJ 2D geometry for Ap1 horizon |
| `KONZA_APT_Bt_0.18mmResolution_Analyze.csv` | ImageJ 2D geometry for Bt horizon |
| `KONZA_APT_Btkss2_0.18mmResolution_Analyze.csv` | ImageJ 2D geometry for Btkss2 horizon |
| `wchold.csv` | Soil moisture at 10, 40 and 120 cm |
| `20220312_KansasEPSCoR_KonzaAPT_SelectedData_aDRH.csv` | Horizon depths, bulk density, porosity, COLE, water-retention and Rosetta inputs, plus image-area and pore summaries |
| R scripts and output tables | Published partition, macropore water-retention, pore-response, conductivity-surface and sensitivity workflows |

The paper reports an MLT-derived 180 μm grid. ImageJ geometry lengths in the archive are mm. For its macropore analysis, objects with projected area below π mm² (equivalent circular radius 1 mm) were excluded. Our morphology auditor supports this as an explicit `--min-area-mm2 3.141592653589793` choice; it is not the universal default.

## 3. Exact reproduction before model improvement

1. Acquire the Dryad dataset and record exact version, filenames, file sizes, publisher digests and local SHA-256. Keep raw archive immutable.
2. Reproduce the paper's exact horizon cropping, ImageJ object filtering, area/perimeter/MinFeret statistics, total cross-section area, dry-state pore-width calculations and depth mapping.
3. Reproduce the moisture preprocessing, matrix/macropore water partition, structural-width relationship, transient hydraulic conductivity and all published sensitivity analyses.
4. Compare every available intermediate and final output against the publisher's CSVs/tables, recording tolerances and unresolved differences. Do not merely compare final figures.
5. Keep data and scripts pinned by the dataset version and SHA-256; record the R package environment used for the original analysis as a comparator. New Rust implementations should implement the same equations independently and demonstrate numerical parity.

## 4. Model bake-off

Evaluate the following models against identical frozen inputs:

- **M0 — static morphology:** single dry-state pore geometry and static hydraulic parameters.
- **M1 — dynamic geometry:** the Hirmas matrix-water-content-conditioned pore-width and hydraulic-conductivity formulation.
- **M2 — calibrated static dual-domain physics:** a static matrix/macropore transport baseline.
- **M3 — dynamic dual-domain physics:** transient geometry and conductivity coupled to matrix/macropore storage and exchange, with conservation enforced.
- **M4 — hybrid state-space:** M3 physical state transition plus a learned residual only where parameters are identifiable; explicit uncertainty and abstention.

Comparators such as HYDRUS standard dual-porosity and HYDRUS DualPerm must be pinned to exact module, equations, dimensionality, parameterization and solver. They are not interchangeable configurations. MACRO and RZWQM2 should be included only when their supported equations and inputs match the evaluated observable.

## 5. Core scientific challenge: can morphology explain dynamic hydraulic response?

Fit and test the chain:

\[
\text{image geometry} \rightarrow d(\theta_m) \rightarrow f_p(\theta_m) \rightarrow K(\theta_m) \rightarrow \text{water/tracer response}
\]

Each arrow is an independently falsifiable relationship. Do not treat a good match on downstream conductivity as proof that intermediate pore width, water partition and connectivity are all correct.

Track at least:

- pore area and width distribution by horizon, resolution and threshold;
- circularity, Feret aspect ratio, orientation distribution and nematic anisotropy (2D descriptors only);
- predicted matrix water content, macropore water content and macropore width through time;
- saturated and unsaturated conductivity response through wetting/drying cycles;
- predicted moisture response at each instrumented depth;
- event arrival/lag and, where independently sampled, conservative tracer breakthrough and recovered mass;
- mass-balance residuals and uncertainty propagation.

### Essential holdouts

- **Temporal holdout:** hold out contiguous storms/seasons, not random time rows.
- **Horizon holdout:** test transfer among Ap1, Bt and Btkss2, while acknowledging all horizons are from one pedon and therefore not independent site replications.
- **Parameter holdout:** fit morphology/hydraulic relations on a subset of measured soil variables and predict withheld horizons or events.
- **Resolution/segmentation holdout:** perturb the image threshold, ROI, object area cut-off and effective pixel resolution.
- **Cross-study holdout:** only claim external validity after an independent soil/profile dataset with comparable geometry and dynamic moisture observations is acquired.

## 6. Uncertainty and identifiability

The published study explicitly investigates sensitivity of predicted pore width and saturated hydraulic conductivity to image-derived total pore area, total perimeter and dry-state pore width, including ±5% perturbations. Reproduce that analysis exactly, then expand it:

- image scale and stitching uncertainty;
- segmentation threshold and ROI boundary uncertainty;
- object-size threshold sensitivity, including 0, π and higher area cutoffs;
- uncertainty in bulk density, COLE, field-capacity and wilting-point water contents;
- temporal sensor noise, missing intervals, depth uncertainty and calibration drift;
- covariance between pore area, perimeter, width, matrix moisture and fitted retention parameters.

Report parameter profile likelihoods or posterior ridges. If pore width, structural-unit width, shrinkage and water partition cannot be separated from the available data, report identifiable combinations or abstain rather than returning point estimates with false precision.

## 7. Proposed acceptance targets (not achieved)

| Gate | Target |
|---|---|
| Numerical reproduction | Intermediate geometry and published output tables match within a preregistered tolerance; all discrepancies traced |
| Conservation | Water and conservative-tracer residuals lie within independently propagated numerical/measurement uncertainty |
| Dynamic value | At least 10% lower held-out transient-K or moisture-response error than M0, with confidence interval excluding zero improvement |
| Robustness | At least 20% lower P90 event-response or tracer-tail error than best matched static baseline, without degradation in median error |
| Calibration | Nominal 90% intervals achieve 85–95% coverage on held-out events and publish worst-horizon coverage |
| Sensitivity | Reported ±5% Ap/P/dds response reproduces paper trends; larger segmentation/scale sensitivity is propagated rather than concealed |
| Identifiability | Parameters reported as point estimates only when the test design demonstrates practical identifiability |
| External transfer | No cross-region claim until replicated on a second pedon/site and independent soil family |

## 8. Scientific limits that must remain explicit

- The image is a 2D dry-state cross-section. It does not directly measure 3D pore network connectivity or every hydraulic pathway.
- Dynamic pore width is inferred from the model and matrix moisture; it is not repeatedly imaged at every time step.
- The Konza APT soil is a particular shrink–swell-prone fine-silty mineral soil. Its fitted coefficients must not be transferred directly to South African soils or Terra Preta amendments.
- The paper compares transient conductivity with a previously published multidomain model. This is useful model-to-model validation, not equivalent to a new direct pore-scale or tracer measurement at every time point.
- No amendment effect, carbon permanence, nutrient-retention benefit or field-rate recommendation follows from this benchmark.

## 9. Luminous architecture

- **Mycelix:** pin raw image archive, segmentation thresholds, ROI, horizon, moisture sensor calibration, image scale, model parameters, split manifests and independent verifier receipts.
- **Symtropy:** implement dynamic geometry-to-hydraulic state transitions and conservative matrix/macropore transport; compare with original equations and industry solvers.
- **Symthaea:** choose the next image resolution, sampling time, sensor depth or tracer measurement by expected reduction in uncertainty about mechanism and decision-relevant flux.

Implementation tracker: [TERRA-MORPH-DYN-001](https://github.com/Luminous-Dynamics/mycelix/issues/5539).

## References

- Hirmas, D. R. et al. (2025). Predicting Soil Interpedal Macroporosity and Hydraulic Conductivity Dynamics: A Model for Integrating Laser-Scanned Profile Imagery With Soil Moisture Sensor Data. Water Resources Research, 61(9), e2024WR039836. https://doi.org/10.1029/2024WR039836
- Hirmas, D. R. et al. (2025). Data: same title [Dataset]. Dryad. https://doi.org/10.5061/dryad.zs7h44jkn
- Cao, X. et al. (2024). A review of advancements in the theory and characterization of soil macropore structure. PeerJ, 12, e18442. https://doi.org/10.7717/peerj.18442