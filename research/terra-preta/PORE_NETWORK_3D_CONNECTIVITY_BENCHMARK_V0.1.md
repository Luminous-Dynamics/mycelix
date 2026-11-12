# 3D Pore-Network Connectivity and CT-Informed Exchange Benchmark v0.1

**Status:** benchmark specification plus a first pore/throat graph auditor. No raw CT volume has been segmented by this work, no measured core network has been audited, and no dual-permeability exchange model has been compared. Numerical targets are proposed, not achieved.

**Implementation trackers:** [TERRA-PORE3D-001](https://github.com/Luminous-Dynamics/mycelix/issues/5548) · [TERRA-EXCHANGE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5550).

## 1. Paradigm shift: from pore volume to connected, wetted, exchanging structure

Bulk macroporosity, mean pore width and a two-dimensional section are incomplete descriptors of preferential flow. A small number of connected bottlenecks can dominate top-to-bottom transport; a large pore-volume fraction can be disconnected; a connected geometric channel may remain dry, air-occluded or hydraulically inactive during a particular event.

The next model state should distinguish four concepts:

1. **Geometric network:** pore bodies, throats, orientation, volume, size and connectivity from segmented 3D CT.
2. **Hydraulic network:** which throats are water-accessible/conducting at the current matric state, with conductance and entry constraints.
3. **Dynamic wetted interface:** the matrix–macropore surface actually wetted over time, not just total geometric surface area.
4. **Exchange flux:** transfer of water and solute across that interface, with a physically explicit sign convention and mass balance.

Do not collapse these into one macropore fraction or one morphology score.

## 2. Research basis and candidate breakthrough

Chaif et al. (2025), Journal of Hydrology 661, 133791, DOI [10.1016/j.jhydrol.2025.133791](https://doi.org/10.1016/j.jhydrol.2025.133791), describe a dual-permeability exchange formulation based on specific wetted matrix–macropore interfacial area multiplied by macropore water flux density. Their abstract reports improved agreement with experimental data versus a classical pressure-head-based exchange term. This suggests an important model comparison: CT-informed wetted-interface exchange versus conventional pressure-head exchange—not simply more detailed geometry versus less detailed geometry.

**A caution from earlier CT-parameter substitution:** Lissy et al. (2020), Geoderma 380, 114643, DOI [10.1016/j.geoderma.2020.114643](https://doi.org/10.1016/j.geoderma.2020.114643), compared a classical dual-porosity model with an advanced version substituting CT-derived percolating macroporosity and mean inter-macropore distance for fitted structural parameters. Their abstract reports that the classical version gave the better overall fit, while its water exchange behavior was not properly simulated; the CT-parameterized version performed worse overall. This is not evidence that CT is unhelpful. It is a warning that **parameter substitution and mechanism improvement are different hypotheses**. Our bake-off therefore includes a CT-parameter-substitution baseline separately from a wetted-interface exchange mechanism.

PoreSpy's SNOW2 and regions-to-network workflow can extract pore/throat networks from 3D segmented images, including region volumes, pore coordinates, inscribed diameters, throat connections and lengths. The extraction itself is a model step: segmentation thresholds, voxel spacing, object filtering and watershed choices can alter network topology. Preserve these choices and run sensitivity analysis.

References: Chaif et al. (2025), https://doi.org/10.1016/j.jhydrol.2025.133791; PoreSpy SNOW2 documentation, https://porespy.org/ (see SNOW2 and regions-to-network).

## 3. New Rust graph auditor

`tools/pore-network-audit/` accepts a provenance-bearing pore/throat graph exported by a declared CT segmentation/network-extraction pipeline. It does not segment CT or simulate flow.

The v0.1 pore-network audit computes:

- connected components and largest-component pore-volume fraction;
- top-to-bottom percolating components and percolating pore-volume fraction;
- widest top-to-bottom path bottleneck (maximum possible minimum throat inscribed radius);
- shortest top-to-bottom graph path length plus centroid-to-boundary offsets and vertical tortuosity;
- connectivity/percolation changes over a declared throat-radius threshold sweep.

The input records source CT, segmentation output and network SHA-256 digests; image dimensions and anisotropic voxel spacing; segmentation method/threshold/software; network extractor/version/parameters; pore centroids/volumes/radii/boundary contacts; and throat endpoints/lengths/radii.

The separate [interface-exchange auditor](./tools/interface-exchange-audit/README.md) now checks the dimensional product and trapezoidal time integral of declared wetted interface area × macropore-to-matrix flux density. It consumes the two input series; it does not derive either series.

All outputs are geometric descriptors. A widest-path radius is not a capillary-entry pressure calculation; shortest-path tortuosity is not flow-weighted; percolation does not establish water saturation or hydraulic conductance. These are screening and provenance measures to connect imaging to later physical validation.

## 4. Required next extension: image uncertainty and network robustness

Run the same raw CT sample through a preregistered segmentation ensemble:

- threshold families (global, local/adaptive, watershed marker settings, learned segmentation only as a separately versioned candidate);
- voxel-resolution sensitivity and downsampling/reconstruction controls;
- minimum pore/throat object size and minimum throat-radius cutoffs;
- boundary-region erosion/label uncertainty;
- SNOW2/OpenPNM mapping and parameter sensitivity.

Report for every segmentation variant: pore/throat count, porosity, connected-component distribution, top-to-bottom percolation probability across variants, percolating-volume fraction, widest bottleneck, shortest-path tortuosity, orientation and graph digest. Do not average incompatible network graphs into a single synthetic network.

Proposed acceptance criteria (not achieved):

1. Exact recovery of deterministic synthetic graph controls.
2. Zero false top-to-bottom percolation certificates on disconnected, lateral-only and dead-end controls.
3. Report threshold at which top-to-bottom connectivity first appears and the fraction of segmentation variants that retain it.
4. Quantify variation in bottleneck radius and tortuosity under segmentation/resolution sensitivity; aim for less than 10% coefficient of variation on stable high-resolution reference scans, subject to pilot-derived feasibility.
5. Independent analyst reruns produce equivalent graph digests after canonical node/edge ordering and identical declared parameters.

## 5. Matched physical model comparison

Use the same imaged soil core, boundary conditions, initial matric state, rainfall/irrigation forcing and measured tracer input. Compare at least:

| Candidate | Exchange / geometry treatment | Required observables |
|---|---|---|
| M0: matrix-only Richards | No preferential domain | Multi-depth theta/pressure, infiltration, drainage and tracer breakthrough |
| M1: classical dual-permeability | Conventional pressure-head-based matrix–macropore exchange | Same observables, domain fluxes and calibrated parameter burden |
| M2: CT-parameter substitution | Replace declared structural parameters (for example percolating macroporosity and mean inter-macropore distance) with CT-derived values, while retaining the classical exchange law | Reproduce the Lissy et al. 2020 caution case; test whether geometry substitution helps or harms each observable |
| M3: CT geometry + classical exchange | CT-derived pore/throat network and geometric connectivity, with classical exchange | Pathway connectivity, breakthrough timing/tails and parameter identifiability |
| M4: CT-informed wetted interface | Specific wetted interfacial area × macropore flux density exchange term, as in the Chaif et al. candidate formulation | Time-varying interface area, exchange flux, tracer breakthrough and mass closure |
| M5: hybrid state-space candidate | M4 physical prior plus learned residual only where identifiable | Prediction/calibration/transfer plus evidence-specific attribution and abstention |

Pin exact executable/version/module, dimensionality, parameters, solver and time-step controls. Calibrate only on training cores/events; score frozen held-out cores and storms. If a model lacks an observable, mark it unsupported rather than awarding a zero error.

Required scoring:

- theta and pressure-head depth profiles: MAE, RMSE, bias, event onset and peak timing;
- infiltration/drainage/runoff: hydrograph shape, volume error and event balance;
- conservative tracer: first arrival, centroid, peak, P90/P95 tail, recovered mass and breakthrough-curve score;
- exchange term: independently measured or constrained exchange flux and time-varying wetted interface uncertainty;
- conservation: water and tracer residuals within propagated numerical and measurement uncertainty;
- operational burden: CT resolution/cost, segmentation labour, hydraulic parameter measurements, solver failures and runtime.

Proposed target: at least 10% improvement in held-out tracer breakthrough or exchange-flux error over M1, and at least 20% improvement in P90/P95 tail error, while not degrading water/tracer balance or nominal 90% interval coverage (target 85–95%). These are research goals, not benchmark results.

## 6. Coupling to soil function, roots and amendments

Only after M0–M3 have been validated on unamended cores should amendments enter the experiment. Biochar, compost and root growth can change pore geometry, hydrophobicity, contact angle, packing, matrix retention and interfacial chemistry. A pre/post-CT comparison can map geometry changes but cannot by itself infer hydraulically active wetting or treatment efficacy.

Use randomized matched cores/lysimeters with repeated wetting histories. Acquire CT and hydraulic/tracer measurements at baseline and declared intervals. Link matrix/macropore exchange to root-water uptake, oxygen transport, nutrient species and dissolved/particulate export only where those fluxes are actually measured. Do not infer a better amendment from geometric connectivity alone.

## 7. How this improves on current practice

The strongest near-term innovation is an auditable measurement-to-model bridge: 3D geometry and segmentation uncertainty → topology/percolation → dynamic wetted interface → exchange flux → tracer/mass-balance validation. It is more informative than static macroporosity and avoids claiming that an image segmentation is a hydraulic experiment.

Symtropy should own the physical kernels and matched dual-permeability simulations; Symthaea should select the next CT resolution, tracer sampling time or sensor location by expected reduction in decision-relevant uncertainty; Mycelix should bind raw scan, segmentation, network extraction, model configuration, split, calibration and reviewer attestations. Any non-identifiable exchange parameter should trigger a measurement recommendation or abstention, not a forced point estimate.

## 8. References and linked work

- Lissy et al. (2020). Can structure data obtained from CT images substitute for parameters of a preferential flow model? Geoderma 380, 114643. https://doi.org/10.1016/j.geoderma.2020.114643
- Chaif et al. (2025). CT-informed specific wetted interfacial area for matrix–macropore exchange. Journal of Hydrology 661, 133791. https://doi.org/10.1016/j.jhydrol.2025.133791
- PoreSpy SNOW2 and regions-to-network documentation. https://porespy.org/
- Dynamic macropore morphology–hydrology benchmark: [DYNAMIC_MACROPORE_MORPHOLOGY_COUPLING_BENCHMARK_V0.1.md](./DYNAMIC_MACROPORE_MORPHOLOGY_COUPLING_BENCHMARK_V0.1.md).
- Industry physics comparator benchmark: [INDUSTRY_PHYSICS_COMPARATOR_BENCHMARK_V0.1.md](./INDUSTRY_PHYSICS_COMPARATOR_BENCHMARK_V0.1.md).
- Implementation: [pore-network-audit README](./tools/pore-network-audit/README.md) and [issue #5548](https://github.com/Luminous-Dynamics/mycelix/issues/5548).