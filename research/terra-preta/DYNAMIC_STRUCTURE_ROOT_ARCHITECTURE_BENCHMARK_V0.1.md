# Dynamic Soil Structure, Root Architecture and Hydraulic Memory Benchmark v0.1

**Status:** research benchmark proposal. No soil-profile imagery, root architecture data, dynamic hydraulic kernel, or multi-site validation has been acquired or executed for this track. Numerical goals below are proposed, not achieved.

## 1. Scientific opportunity

The 2026 NEON event model predicts the Ksat-threshold PF proxy well (F1 0.91, accuracy 0.85) but predicts apparent response velocity much less accurately (test R² 0.31). Its feature matrix does not include organic carbon or root architecture, and the authors identify missing soil structure, root effects and organic matter as likely limits on velocity prediction. This is a stronger research opportunity than simply adding another event classifier: make the evolving hydraulic structure a measured state variable.

Hirmas et al. (2025; Water Resources Research, DOI 10.1029/2024WR039836) demonstrate a physically based route: use high-resolution laser-scanned intact profile cross-sections to measure interpedal macropore geometry, couple it to soil-moisture time series, estimate dynamic macropore widths, partition matrix/macropore water and calculate moisture-dependent hydraulic conductivity. This establishes a useful method family, not a universal calibrated solution.

### Executable reference dataset and morphology audit

The authors publish the supporting case as [Dryad DOI 10.5061/dryad.zs7h44jkn](https://doi.org/10.5061/dryad.zs7h44jkn), under CC0 1.0. Its README describes a 4.60 MB archive containing a 10/40/120 cm moisture series, ImageJ geometry exports for the Ap1, Bt and Btkss2 horizons at 0.18 mm resolution, soil shrink-swell/COLE data, bulk density and retention parameters, daily weather, R source scripts, functions and model outputs. This is a single Konza APT profile case; some supporting COLE measurements include other Kansas pedons, but that is not a multi-site validation of the primary hydraulic model.

The file endpoint returned HTTP 403 during this pass. The source metadata and file list are recorded in `neon-preferential-flow-source-plan.v0.1.json`; the archive has **not** been acquired or hashed locally. The archive is still valuable for a reproducible model crosswalk once access is resolved.

The new [morphology-audit CLI](./tools/morphology-audit/README.md) accepts already segmented ImageJ object tables with explicit column mappings. It computes distributions of projected area, perimeter, minimum/maximum Feret diameter, a 10-degree Feret-angle histogram, and optionally a projected 2D area fraction when the exact ROI area is supplied. It does not segment raw images, infer 3D connectivity or predict hydraulic conductivity.


The Hirmas model has material assumptions: uniform cube-like structural units, isotropic shrinkage, and smooth/straight planar-slit macropores. The authors note that real geometry is more complex and that conductivity may be overestimated; they also state that image-measurement error needs further evaluation. The benchmark below makes these assumptions explicit and stress-tests them.

## 2. Paradigm shift: soil structure is a dynamic latent state, not a static covariate

Replace the static descriptor vector with a state-space representation:

- **Structural state:** pore-size distribution, aperture distribution, orientation, anisotropy, interpedal area/perimeter, depth-specific void fraction, connectedness, tortuosity, aggregate/ped geometry and root/biopore indicators.
- **Hydraulic state:** matrix and macropore water content, pressure head, dynamic aperture, water exchange between domains, transient matrix/macropore conductivity, wetting/drying history and hydraulic hysteresis.
- **Biological/management state:** root diameter/density/depth and turnover, fauna/macropore proxies, organic matter, aggregation/stability, compaction, tillage/traffic, vegetation and management history.
- **Forcing state:** event hyetograph, antecedent water status, temperature, drying interval and repeated-cycle sequence.

Structure should evolve with moisture, shrink-swell, root growth/decay and management disturbance. A single static Rosetta Ksat or texture vector must remain as a matched baseline, not the only physical description.

## 3. Required measurements and representation

### 3.1 Image-derived structural measurements

Use intact profile-wall imaging with laser multi-triangulation (MLT), calibrated high-resolution photography or a validated equivalent. Preserve raw imagery and image-analysis parameters. Extract, per horizon/depth:

- pore/void area, perimeter, aperture and minimum/maximum Feret diameters;
- pore orientation distributions and anisotropy;
- connected-component size, apparent connectivity, branch/node density and tortuosity proxies;
- ped/aggregate size, shape and interpedal void geometry;
- image resolution, scale, threshold/segmentation algorithm, illumination, registration and uncertainty.

Report the lower detectable pore size and segmentation uncertainty. Excluded small features must be recorded as censored morphology, not zero porosity. A 2D profile slice does not prove 3D connectivity; measure orthogonal sections or use CT/geophysics on a calibration subset.

### 3.2 Root architecture and biopore context

Measure root diameter distribution, root length density, depth distribution, species/functional type where relevant, turnover/decomposition and visible root channels. Distinguish live roots, dead roots, fauna channels and shrinkage cracks. Register samples to the hydraulic profile. Root density alone is not a pore-connectivity label.

### 3.3 Repeated structure and hydraulic history

Collect paired structural images and moisture/pressure records before and after predeclared wetting/drying cycles. Where destructive imaging prevents repeating the same wall, use matched replicate cores/profiles with randomized treatment and shared reference imaging; explicitly represent sample-to-sample variability. Include seasonal and management-induced structure changes where feasible.

Record moisture/pressure profiles, rainfall intensity, infiltration, runoff, drainage, evaporation and conservative-tracer breakthrough. Couple imaging to flux measurement rather than judging the model on a derived moisture proxy alone.

## 4. Model family and matched comparators

| Model | Structural representation | Purpose |
|---|---|---|
| Static matrix Richards | Texture/pedotransfer retention and Ksat | Matrix baseline |
| Static dual-domain | Fixed macropore fraction/aperture and exchange | Conventional preferential-flow physics |
| Hirmas-style image-informed dynamic model | Image-derived interpedal geometry with moisture-dependent aperture/K(s) | Published dynamic morphology comparator |
| Image + root feature model | Image geometry plus measured root/biopore descriptors | Tests incremental biological structure information |
| Dynamic hybrid Symtropy candidate | State-dependent morphology, hysteresis, domain exchange, conservative solute transport | Candidate advance |
| Statistical multimodal baseline | Image/root/weather/sensor features with grouped splits | Tests predictive value without claiming physical causality |

Pin executable version, equations, geometry assumptions, parameter priors, observation operators, calibration data and solver settings for every comparator. Calibrate only inside training profiles; do not tune on test events.

## 5. Identifiability and geometry stress tests

Generate known-truth scenes and measurement operators that vary:

1. aperture distribution and pore-size tails;
2. isotropic versus anisotropic shrinkage, swelling and orientation;
3. straight slit versus branched, curved, rough and tortuous pathways;
4. isolated voids versus 3D connected pathways with identical 2D area;
5. root channels and root-decay voids versus abiotic cracks;
6. sensor depth/spacing and image-to-sensor registration error;
7. segmentation thresholds, resolution, occlusion and image measurement noise;
8. matrix/macropore exchange and preferential-path flux fraction;
9. wetting/drying cycle count, soil history and structural ageing;
10. unseen mineralogy, texture, climate, management and soil biological activity.

Use Jacobian rank/singular values, profile likelihoods and parameter recovery to determine whether aperture, connectedness, exchange coefficient and macropore flux fraction can actually be distinguished from each measurement combination. When they are not identifiable, report an uncertainty set or abstain. Do not output a precise 3D network from a single 2D slice.

## 6. Required outcomes and metrics

- Structural reconstruction: pore-area/aperture distribution error, orientation error, connected-component precision/recall and uncertainty calibration against CT or orthogonal reference sections.
- Hydraulic prediction: depth-resolved theta and pressure head, dynamic K(s), infiltration and drainage hydrographs, hysteresis loop area and profile redistribution.
- Preferential transport: conservative tracer first arrival, median/centroid, peak, P90/P95 tail timing and mass recovery.
- Mechanism inference: matrix-only null false-certificate rate, connected-path detection, disconnected-path abstention and parameter identifiability.
- Generalization: leave-profile, leave-site, leave-soil-family, leave-management and leave-climate-region out. Keep all repeated cycles and derived images from one specimen/profile within one partition.
- Efficiency: imaging effort, CT calibration subset, sensors per profile, field labour, processing time and cost per independent profile/event.

## 7. Proposed improvement targets

Targets are hypotheses for a preregistered bake-off; they are not results:

| Metric | Target |
|---|---|
| Velocity prediction | At least 20% lower MAE than the best matched static-structure baseline on held-out profiles, with confidence intervals |
| Tracer-tail prediction | At least 20% lower P90/P95 timing error than fixed-parameter dual-domain baseline |
| Hydraulic state calibration | Nominal 90% intervals achieve 85–95% coverage on held-out soil/profile strata |
| Conservation | Water and conservative-solute residuals remain within propagated measurement plus numerical error; no silent closure correction |
| Null safety | Zero false physical pathway certificates on deterministic matrix-only and disconnected-channel controls |
| OOD transfer | Publish worst-family/site performance and abstention; no pooled-only score |
| Cost efficiency | Demonstrate Pareto improvement in independent tracer/hydraulic accuracy per imaging and sensor cost |

A model does not pass by improving velocity R² while degrading P95 tracer-tail prediction, false certificates, conservation, calibration or worst-soil transfer.

## 8. How to connect the Luminous architecture

- **Symtropy:** implement the dynamic hydraulic state and morphology-conditioned constitutive laws; couple matrix and macropore domains with explicit exchange and water/solute conservation.
- **Symthaea:** maintain competing structural hypotheses; quantify whether the next image, sensor depth, tracer time or wet/dry cycle will distinguish them; select measurements by expected reduction in decision-relevant uncertainty per cost.
- **Mycelix:** bind image bytes, segmentation version, specimen/profile genealogy, calibration, sensor map, soil horizon, management history, model build, solver config, split assignment and reviewer attestation.

Image hashes prove which image was used. They do not prove segmentation correctness, 3D connectivity, physical causality or representativeness. Independent soil physicists and imaging specialists must review the measurement model.

## 9. Execution sequence

1. Obtain the published Hirmas equations, input definitions and any reusable image-analysis assets; document licensing and exact reproduction conditions.
2. Create a tiny synthetic morphology suite with known aperture distributions, connectivity and 2D projection ambiguity.
3. Add explicit image measurement error and segmentation sensitivity before training any learned component.
4. Reproduce a static matrix, static dual-domain and Hirmas-style dynamic image baseline on identical forcing.
5. Couple the dynamic image-derived structural state to Symtropy's hydraulic kernel and test conservation.
6. Add root architecture as a distinct input/latent variable; evaluate its incremental value on grouped held-out profiles.
7. Validate on independent CT/orthogonal imaging and tracer breakthrough; only then broaden site/soil transfer.

## References

- Hirmas, D. R. et al. (2025). Predicting Soil Interpedal Macroporosity and Hydraulic Conductivity Dynamics: A Model for Integrating Laser-Scanned Profile Imagery With Soil Moisture Sensor Data. Water Resources Research, 61(9), e2024WR039836. https://doi.org/10.1029/2024WR039836
- Li, B. et al. (2026). Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes. Earth's Future. https://doi.org/10.1029/2026EF008296
- Nimmo, J. R. et al. (2025). Identifying preferential flow from soil moisture time series: Review of methodologies. Vadose Zone Journal, 24(2), e70017. https://doi.org/10.1002/vzj2.70017