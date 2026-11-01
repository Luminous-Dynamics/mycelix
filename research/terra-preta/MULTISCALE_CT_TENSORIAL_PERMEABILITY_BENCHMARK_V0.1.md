# Multiscale CT-to-Tensorial Permeability Benchmark v0.1

**Status:** research specification and synthetic qualification plan. No new CT sample was scanned, no pore-scale solver was run by this addition, and no measured hydraulic improvement is claimed.

**Tracker:** [TERRA-PHYS-002 / #5650](https://github.com/Luminous-Dynamics/mycelix/issues/5650)

**Dependencies:** #5536 (known-truth preferential-flow simulator), #5548 (3D pore-network connectivity), #5602 (matched physical simulator), #5632 (covariance and Bayesian calibration), #5633 (multi-world comparisons), #5643 (CT–tracer observation operators), #5649 (sequential correlated histories).

## 1. Why the next benchmark must be tensorial and multiscale

A scalar porosity or scalar saturated hydraulic conductivity can conceal whether a sample transmits water differently along different directions. A segmented network can be topologically connected while its effective conductance depends strongly on bottlenecks, anisotropic orientation and sample boundaries.

The next benchmark must qualify the entire transformation:

```text
reconstruction
 -> phase segmentation
 -> pore/throat graph or voxel geometry
 -> pore-scale flow solver
 -> homogenized conductivity tensor
 -> independent directional hydraulic measurement
```

Uncertainty must propagate through each arrow. Agreement between numerical solvers is not external validation: several solvers can share the same biased geometry, segmentation or boundary assumptions.

## 2. Evidence motivating the benchmark

### Gerke et al. (2026): pore-scale solver agreement is not enough

Gerke et al., *Soil hydraulic properties derived from pore-scale simulations: Digital assessment of Ksat through model intercomparison and verification with experimental data*, Soil and Tillage Research 106790, DOI [10.1016/j.still.2025.106790](https://doi.org/10.1016/j.still.2025.106790), compare three pore-scale simulation families on XCT images from A, Ah and B horizons:

- finite-difference solution of Stokes equations;
- lattice-Boltzmann method;
- pore-network model.

The paper reports similar saturated-conductivity results across the simulation approaches, but substantial overestimation relative to experimental values in the horizons tested. It also reports directional anisotropy and argues that tensor-valued hydraulic properties can contain information hidden by conventional scalar measurements. The authors caution that simple segmentation-parameter fitting to match hydraulic conductivity can produce non-physical values.

**Benchmark consequence:** compare numerical solvers on the same frozen image/segmentation and then validate against independent directional hydraulic measurements. Do not tune segmentation thresholds against the held-out hydraulic target.

### Resolution and sample size jointly change apparent topology

Kaur et al. (2025), *Effect of image resolution and soil core diameter on soil pore characteristics quantified using X-ray computed tomography*, Journal of Soils and Sediments 25, 2611–2625, DOI [10.1007/s11368-025-04105-8](https://doi.org/10.1007/s11368-025-04105-8), varied voxel resolution and core diameter. Their results indicate that finer resolution / smaller field of view, core diameter and ROI influence pore counts, isolated pore fraction, connectivity and anisotropy. A smaller core may capture fewer pores while displaying a different connectivity estimate; a larger core can better include larger structural pores but at a different resolution.

**Benchmark consequence:** resolution and representative sample volume cannot be optimized independently. Report a joint resolution–FOV–core-size frontier.

### Segmentation uncertainty can amplify into permeability uncertainty

Jiang, Leung & Liu (2025), *Segmentation uncertainty of vegetated porous media propagates during X-ray CT image-based analysis*, Plant and Soil 511, 969–995, DOI [10.1007/s11104-024-07030-w](https://doi.org/10.1007/s11104-024-07030-w), quantify propagation from CT phase segmentation into volumetric, morphological and hydraulic properties in vegetated glass-bead media. They report parameter-dependent propagation and sensitivity of permeability to upstream segmentation uncertainty.

**Benchmark consequence:** propagate segmentation ensembles through the whole hydraulic calculation. A narrow solver confidence interval conditional on one segmentation is not total uncertainty.

### Soil-domain dependence limits universal CT-to-hydraulics mappings

Sobucki et al. (2026), *Quantifying Pore Network Differences Between Cohesive and Non-Cohesive Soils via X-ray CT and Hydraulic Measurements*, Journal of Soil Science and Plant Nutrition, DOI [10.1007/s42729-026-03422-2](https://doi.org/10.1007/s42729-026-03422-2), report domain-dependent relationships between CT pore descriptors and hydraulic properties in northeastern-Brazil Acrisols/Lixisols. In some non-cohesive groups CT morphology correlations with hydraulic outcomes were not significant, and one site showed an exception to the general cohesive/non-cohesive pattern.

**Benchmark consequence:** stratify by soil/horizon and test transfer. Do not train one global scalar mapping and assume it generalizes.

These papers motivate benchmark requirements; they do not validate our implementation or provide a universal CT-to-(K) relationship.

## 3. Physical and observation layers

Keep these layers separately identified:

1. **Image support:** core/sample, field of view, voxel spacing, orientation and ROI.
2. **Reconstruction:** scanner, calibration, reconstruction algorithm, kernel and noise model.
3. **Segmentation:** phase labels, method/version, parameters, uncertainty realization and digest.
4. **Geometry:** voxel representation and/or pore/throat network with graph/volume digest.
5. **Flow model:** Stokes, LBM or PNM solver, fluid properties, boundary condition, convergence and discretization.
6. **Homogenized outcome:** permeability/conductivity tensor and uncertainty, using an explicitly declared normalization and coordinate basis.
7. **Experimental reference:** independently measured directional hydraulic response, apparatus, boundary condition, temperature/viscosity correction, specimen and replicate lineage.

Never confuse an observed 2D section with a measured 3D network, a geometric graph with a conducting network, or saturated effective conductivity with event-specific preferential transport.

## 4. Tensorial hydraulic target

For a declared coordinate frame, estimate a symmetric positive-definite conductivity tensor:

```text
K = [ Kxx  Kxy  Kxz
      Kxy  Kyy  Kyz
      Kxz  Kyz  Kzz ]
```

For each image/segmentation/solver realization report:

- diagonal directional conductivities;
- off-diagonal terms only where the solver and boundary design identify them;
- eigenvalues / principal conductivities;
- principal direction vectors;
- anisotropy ratios with propagated uncertainty;
- predicted flux for held-out gradient directions.

The reference experiment must provide sufficient independent directional excitation to identify the declared tensor elements. If the physical reference only supports vertical/horizontal components, do not pretend it validates the full six-parameter symmetric tensor. Report which components are identified, weakly identified or unsupported.

A numerically positive-definite tensor is a physical admissibility check, not proof of correct orientation or magnitude.

## 5. Multiscale design and representative volume

Freeze a nested support design with:

- more than one voxel pitch where imaging permits;
- more than one field-of-view / ROI dimension;
- more than one core diameter where independent specimens permit;
- aligned specimen orientation and coordinate transforms;
- explicit relationship between scanned subvolume and hydraulic-test specimen.

Report topology, percolation, bottleneck, pore density, surface density, tortuosity, isolated volume and estimated conductivity tensor against all scale parameters.

A small high-resolution scan may resolve fine pores but omit rare large channels. A larger low-resolution scan may represent more of the specimen but miss narrow bottlenecks. The benchmark should estimate when each limitation dominates rather than declaring one resolution universally optimal.

## 6. Segmentation and numerical uncertainty propagation

Use a preregistered segmentation ensemble. For every segmentation realization:

1. retain input CT digest and reconstruction identity;
2. retain algorithm/version, thresholds and phase-label rules;
3. compute segmented-volume digest;
4. extract pore network / voxel geometry with full parameters;
5. run each supported physical solver under matched boundary conditions;
6. record solver convergence and conservation;
7. calculate tensor outcomes;
8. preserve the full distribution across plausible segmentation and solver realizations.

Do not use the validation (K) measurement to choose a segmentation threshold. If segmentation parameters are estimated from training hydraulic data, evaluate only on independent specimens and keep that calibration layer explicit.

Separate uncertainty sources:
- scan/reconstruction;
- phase segmentation;
- pore/throat extraction;
- unresolved sub-resolution pathways;
- physical parameters/fluid properties;
- boundary-condition and finite-sample effects;
- numerical discretization and convergence;
- experimental reference repeatability and calibration;
- between-specimen, batch, horizon and site variability.

## 7. Solver bake-off

Run the following on the same frozen image geometry where the methods support a matched problem:

| Solver | Physical representation | Required comparison |
|---|---|---|
| Finite-difference Stokes | Resolved pore-fluid domain | Grid convergence, flux closure, tensor components |
| Lattice-Boltzmann | Resolved pore-fluid domain | Lattice resolution, relaxation parameters, flux closure, tensor components |
| Pore-network model | Extracted pore/throat graph | Extraction sensitivity, conductance-law assumptions, tensor components |

Require:
- same fluid viscosity/density and temperature convention;
- same specimen orientation and imposed gradients;
- boundary condition equivalence and explicit unsupported cases;
- independent mesh/lattice/time refinement;
- mass/flux conservation;
- CPU/GPU runtime and memory;
- solver-to-solver disagreement;
- solver-to-experiment bias.

Do not force a method into a dimension, geometry or boundary configuration it does not support. Unsupported is not zero error.

## 8. Domain transportability

Predeclare grouped splits by independent specimen and, where available, horizon, site, material batch, management and soil structural class. Include:
- cohesive vs non-cohesive horizons;
- texture/clay and aggregate stability regimes;
- depth and horizon;
- compaction / tillage;
- specimen size, resolution and ROI;
- source laboratory and batch.

Report leave-specimen, leave-horizon, leave-site and leave-domain performance separately. An apparent global correlation can disappear or reverse in a new domain; calibrated abstention is preferable to unsupported tensor prediction.

## 9. Integration with CT–tracer design

Only after the saturated tensor benchmark is qualified should the programme use it as a prior for event-scale transport. Integrate with #5643:

```text
CT geometry ensemble
 -> hydraulic tensor / local conductance
 -> matric state + boundary/event forcing
 -> active pore pathways and matrix/macropore exchange
 -> tracer breakthrough observation operator
```

The saturated tensor does not uniquely identify unsaturated activation, dynamic wetting, exchange or preferential bypass. These remain separate physical states and observation targets.

## 10. Metrics and proposed qualification gates (targets, not results)

**Physics / numerical**
- water/flux conservation residual;
- successive mesh/lattice/time refinement change;
- tensor component bias and proper score;
- principal-direction error;
- eigenvalue / anisotropy ratio error;
- solver-to-solver vs solver-to-experiment disagreement.

**Uncertainty**
- empirical coverage and width by tensor component, scale and soil domain;
- uncertainty propagation from segmentation and image scale;
- calibration of unsupported / abstention decisions.

**Design**
- additional measurement value of another resolution, larger core, independent specimen or directional hydraulic test;
- cost per reduction in tensor uncertainty;
- exact small-case design oracle and solver gaps for larger search.

Proposed target gates:
- 100% exact provenance/digest binding from CT to tensor output;
- no hydraulic target used to tune its own held-out segmentation;
- deterministic manufactured cases satisfy preregistered conservation/convergence tolerances;
- nominal 90% intervals target 85–95% coverage in adequately replicated independent groups;
- each reported tensor element has an independently supported directional measurement or is labelled unresolved;
- no material degradation in calibration / worst-domain performance when claiming ≥10% improvement over a matched solver;
- all solver non-convergence and unsupported configurations remain visible.

These thresholds require preregistration and feasibility evaluation; they are not achieved results or universal tolerances.

## 11. Implementation sequence

1. Define a synthetic 3D pore/throat geometry generator with known anisotropy, disconnected and sub-resolution channels.
2. Implement deterministic tensor recovery and manufactured-flow controls before connecting any real CT segmentation.
3. Implement one pore-network forward solver and its boundary/conservation checks; then add Stokes and LBM comparators.
4. Add segmentation/resolution/REV ensemble fixtures and uncertainty propagation.
5. Bind specimen and independent hydraulic references with exact provenance and grouped splits.
6. Add paired directional measurements and posterior calibration.
7. Connect the qualified tensor output to CT–tracer #5643 and sequential acquisition #5649.

## 12. Evidence and safety boundary

Research software and synthetic benchmark only. No CT scanning, physical specimen collection, flow testing, tracer injection, soil amendment, land access or field trial is authorized. Empirical datasets require license, provenance, specimen lineage and qualified scientific review. No field or agronomic recommendation follows from synthetic solver success.


## 13. Public solver and image-reference candidates (metadata reviewed, not acquired)

The following resources can make the first solver qualification reproducible. Their evidentiary roles must remain distinct.

### POREMAPS 1.0.0 — Stokes/tensor solver verification

- DaRUS DOI: https://doi.org/10.18419/DARUS-3676
- Code repository: https://git.rwth-aachen.de/david.krach/poremaps
- Release: 1.0.0; finite-difference Stokes solver designed for binary 3D image inputs and permeability-tensor calculation.
- Dataset record documents Poiseuille, channel, sphere-packing, flat/pseudo-3D and ellipsoid benchmarks, with velocity/pressure outputs; applications include compressed foam and calcite-clogged porous structures.
- The record specifies MIT licensing for software and CC BY 4.0 for the research data. Check terms on the exact downloaded version before any redistribution.
- Importantly, application datasets were preprocessed for the specified periodic boundary conditions and non-connected pore space was removed. Record those transformations and do not compare them as raw-soil samples.

**Use:** independent FDM/Stokes numerical verification and tensor-output reference. It is not a soil-specific validation dataset and does not replace independent measured soil conductivity.

### Loess tunnel erosion micro-CT — soil morphology / erosion stress test

- Mendeley Data DOI: https://doi.org/10.17632/rm69k2z4cm.1
- Published 27 March 2026; CC BY 4.0 per landing-page metadata.
- Record description includes raw and derived 3D reconstructed pore structures, pore morphology parameters, disintegration experimental results and analytical outputs.

**Use:** image-pipeline reproducibility, pore-structure/erosion association and resolution/segmentation stress test. The landing-page description does not establish independent directional saturated-conductivity measurements or tracer breakthrough. Do not label it a (K)-tensor ground-truth set until the exact files and paper show those measurements.

### Raw pore data from X-ray CT — soil morphology table candidate

- Mendeley Data DOI: https://doi.org/10.17632/3rks5xn2gx.1
- Published 2 April 2026; CC BY 4.0 per landing-page metadata.
- The record describes 3D CT reconstruction and Avizo segmentation of soil samples. Its listed file is an Excel workbook (~690 KB), not a declared raw 3D image archive.

**Use:** candidate for table/descriptor schema and segmentation-output audit after workbook inspection. Do not assume the underlying CT volumes, directional (K), specimen grouping or raw pixel arrays are included.

### DRP-372 — general porous-media solver/transfer stress test

- Dataset paper: https://doi.org/10.1038/s41597-022-01664-0
- Includes 217 digital porous-media samples and 1,736 flow/electrical simulation results; images are available at 256³ and 480³ domains, with structural/topological descriptors and solver results. The collection spans many porous lithologies and includes non-soil materials.

**Use:** larger-scale general porous-media solver regression, resolution sensitivity and out-of-domain transfer stress testing. Keep its sample groups and material classes intact; only a reviewed subset identified as soil may support soil-specific analysis. Simulated outputs are not independent experimental hydraulic truth.

### Admission checklist before acquisition

For every candidate, pin exact DOI/version, file inventory, license text, checksums, specimen/sample IDs, imaging resolution/FOV, segmentation provenance, boundary conditions, solver/version, observed vs simulated hydraulic variables, independent experimental-unit structure, and whether raw volumes are actually available. Store raw assets outside Git; no source is promoted to confirmatory external validation until this audit passes.


## State–parameter assimilation reference implementation

Issue #5658 now has a reusable exact linear-Gaussian state/parameter assimilation oracle in `src/state_parameter_assimilation.rs`. It augments physical water states and exchange/hydraulic parameters into one Gaussian vector, applies (y=Hz+\epsilon), and calculates the exact posterior by conditioning the joint Gaussian. It keeps state/parameter cross-covariance visible instead of fitting the state first and then treating hydraulic parameters as fixed.

Use this exact oracle to qualify approximations:
- deterministic/ensemble Kalman filter and square-root variants;
- particle filter / resample-move;
- reduced-adjoint variational assimilation with POD;
- batch Bayesian state/parameter calibration.

Each method must be evaluated on identical synthetic worlds, with posterior means, covariance/credible coverage, CRPS/log score, innovations, conservation and compute cost. The linear oracle does not replace nonlinear hydraulic-model validation.
