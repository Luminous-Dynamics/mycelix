# Root Architecture and Pore-Connectivity Causal Mediation Benchmark v0.1

**Status:** research specification. The linked datasets have not yet been acquired or independently audited in this repository. No causal effect, mediation effect, field recommendation, amendment formulation, or South African transfer result is claimed.

## 1. Research question

When a soil-management treatment changes event-scale water or solute transport, does the effect operate through root architecture and the formation, persistence, or activation of connected pore pathways—and under which soil textures, hydraulic states, and forcing regimes?

This benchmark connects, without duplicating, Mycelix #5535 (paired-forcing treatment effects), #5545 (dynamic macropore morphology), #5536 (known-truth PF and identifiability), #5527 (temporal persistence), #5532 (coupled water/nutrient transport), and #5525 (functional soil systems).

The central distinction is:

- **Pore abundance / porosity:** how much pore space is present.
- **Pore morphology:** the shape, size, orientation, and spatial arrangement of observed pores.
- **Geometric connectivity:** whether a pore network is connected under a declared image-resolution and segmentation rule.
- **Hydraulically active connectivity:** whether connected pathways actually carry water under a specific initial state and forcing.
- **Transport consequence:** whether those pathways change arrival time, tracer breakthrough, drainage, lateral flow, or nutrient mass export.

These are related but non-equivalent quantities. A 2D dye or image metric does not establish 3D connectivity; a 3D geometric connection does not prove event-specific water conduction; a fast sensor response does not by itself prove a connected macropore.

## 2. Evidence motivating the benchmark

### 2.1 Cover-crop / dye-tracing dataset

Villarreal (2025), Mendeley Data version 2, DOI 10.17632/kdffpwdbdv.2, is a public CC BY 4.0 dataset. Its description identifies two Pampas-region settings:

- Chascomús, loam: bare fallow versus Vicia / barley cover crops, with maize as the main crop.
- Rivadavia, sandy loam: bare fallow versus rye cover crop, with soybean as the main crop.

The dataset description reports texture-dependent preferential-flow responses: no significant PF effect in the sandy-loam setting and a root-associated increase in the loam setting that was not persistent over time. It also reports increased macropore connectivity at both sites.

This is a high-value mechanism lead, not automatically a randomized causal dataset. Before causal use, audit the paper and workbook for assignment mechanism, blocks, plot IDs, experimental units, repeated observations, time points, dye protocol, image support, missingness, and whether the reported connectivity metric is geometric, 2D, or otherwise operationally defined. Treat site, crop, texture, treatment, and time as potentially confounded until the design establishes otherwise.

Dataset: https://data.mendeley.com/datasets/kdffpwdbdv/2

### 2.2 Independent transport / tracer reference

Asadollahi et al. (2020), DOI 10.1029/2019WR025539, provide a public EPFL vegetated lysimeter dataset at Zenodo DOI 10.5281/zenodo.3457960. The record describes a 1 m long, 1.2 m diameter lysimeter, a one-hour simultaneous injection of five fluorobenzoate tracers, hourly precipitation / irrigation / evapotranspiration / bottom drainage, and irregularly sampled bottom tracer concentrations. The publisher-native hydrologic file labels the drainage column Leakage (mm/h); preserve that source label until the paper and collection method are reconciled. A direct inspection of the publisher-served tracer file shows that the first post-injection sample is 2016-03-06 09:13 (UTC+1), about 67.2 hours after the stated 2016-03-03 14:00 injection. Therefore this dataset cannot resolve sub-day first arrival immediately after injection; report the first observed concentration / interval censoring, not a precise arrival time. It is a transport and water-age reference, not a treatment-randomization dataset or an independent macropore-geometry label.

Dataset: https://zenodo.org/records/3457960

The measured tracer samples are irregularly spaced. Reconstructing breakthrough mass therefore requires explicit treatment of sample timing, measured drainage, interpolation uncertainty, detection limits, and recovery. Concentration alone is not exported mass.

### 2.3 Stable-isotope profile evidence

Pyschik and Weiler (2026), HESS 30, 485–501, DOI 10.5194/hess-30-485-2026, describe stable-water-isotope depth profiles across four catchments, with approximately 100 soil cores per catchment, cores 1–3 m deep, and sampling at 10–20 cm depth intervals. The paper uses deviations from seasonal matrix-infiltration reference profiles as indicators of vertical or lateral preferential-flow influence.

This is a complementary spatial / water-age evidence family, not event-specific tracer truth. Isotope profiles can be influenced by mixing, evaporation, sampling time, and lateral redistribution. Their use must preserve profile-level evidence and the method's own uncertainty.

Paper: https://doi.org/10.5194/hess-30-485-2026

## 3. Causal model and estimands

Use the following conceptual structure, then adapt the DAG to each actual design:

~~~text
Baseline soil / site / block / crop history / season
       |                  |                  |
       v                  v                  v
Treatment assignment --> Root architecture --> Pore topology
       |                       |                  |
       |                       v                  v
       +--------------------> Soil structure --> Active pathway state
       |                                          |
       v                                          v
Direct treatment mechanisms ----------------> Event transport response
                                                  ^
                                                  |
Rainfall / irrigation forcing + antecedent hydraulic state
                                                  |
                                                  v
                                   Tracer / drainage / nutrient export
~~~

The graph is a hypothesis map, not a claim that all arrows are identified in every observational dataset. In field studies, root architecture and pore topology are post-treatment mediators; mediator–outcome confounding, treatment-induced confounding, measurement error, and unmeasured management differences can prevent point identification even when treatment allocation was randomized.

Predeclare distinct estimands:

1. **Total treatment effect:** assigned treatment versus comparator on a primary event transport outcome, under the target distribution of measured hydrologic forcing.
2. **Treatment effect heterogeneity:** change in that contrast across soil texture / mineralogy, antecedent wetness, event intensity, event duration, pulse spacing, crop/root state, and elapsed time since treatment.
3. **Mediator response:** treatment effect on root traits and pore-network descriptors, measured at a declared time and spatial support.
4. **Mediation estimand:** an interventional indirect effect through a prespecified mediator distribution, only where assumptions and data support identification. Do not report natural direct / indirect effects by default when cross-world assumptions or mediator–outcome exchangeability are implausible.
5. **Mechanistic prediction:** held-out prediction of pathway activation and tracer / flux outcomes using morphology and connectivity features, compared with hydraulic-property-only baselines.

Random treatment assignment identifies a total treatment effect under the randomization and follow-up assumptions; it does not automatically identify the causal effect of a mediator. Where mediator identification is not defensible, report descriptive mediation-compatible associations, sensitivity analysis, or bounds—not a causal mechanism claim.

## 4. Measurements needed to distinguish the mechanisms

Each specimen / plot / core requires immutable lineage from source soil and baseline through treatment, image / root observations, event forcing, sensor response, tracer collection, and final mass balance.

### Root architecture

Where measured, preserve root length density by depth, diameter distribution, root surface area, orientation, branching, root turnover / mortality, species, crop stage, and sampling method. Do not substitute above-ground cover or biomass for below-ground root architecture.

### Pore morphology and topology

Preserve raw images and acquisition metadata; voxel / pixel size; field of view; depth and orientation; sample preparation; segmentation algorithm and version; threshold / uncertainty; pore-size support; connected-component IDs; and edge / boundary handling.

Report pore volume / porosity separately from topology. Candidate descriptors include connected-component size distribution, spanning probability across declared horizons, throat bottlenecks, tortuosity, anisotropy, Euler characteristic / Betti features where validated, and resolution-dependent topology stability. No single descriptor is a universal connectivity truth.

### Active hydrologic connectivity

Bind morphology to an event-specific forcing and state. Use tracer breakthrough / recovery, depth-resolved water / pressure response, drainage / runoff volume, and where available geophysics or dye mapping. Keep vertical, lateral, perched, disconnected, and bypass pathways distinct. A path is not marked hydraulically active from geometry alone.

### Transport endpoints

Prefer directly measured first-arrival intervals, breakthrough quantiles, peak time, tail mass, recovered fraction, drainage flux, and depth-resolved redistribution. For nutrients, integrate synchronized concentration and measured water volume / flow and preserve dissolved and particulate export separately. Report collection coverage and censoring. Do not convert concentration to mass without flow or collected volume.

## 5. Experimental tiers

### Tier A — retrospective mechanism audit

Start with the cover-crop dye dataset and the public lysimeter tracer dataset. Audit source files, licensing, treatment allocation, units, experimental units, repetitions, time alignment, image / tracer measurement support, and denominator logic before modeling.

The cover-crop dataset can support a texture- and time-stratified mechanism case study if its metadata permit. It must not be described as randomized unless the assignment is verified. The lysimeter dataset is for tracer transport model calibration / evaluation; it does not identify a cover-crop or amendment treatment effect.

### Tier B — known-truth computational experiment

Extend #5536 with matched-observation counterexamples:

- Same porosity, different network connectivity.
- Similar pore topology, different hydraulic activation under wet / dry antecedent states.
- Similar depth-sensor traces generated by matrix piston displacement versus connected macropore bypass.
- Connected but hydraulically inactive pores.
- Root-channel / biopore network with disconnected lower boundary.
- Lateral / perched flow producing a vertical sensor signature.
- Dye-visible path with limited event water mass, and event-water pathway poorly represented in a destructive dye section.
- Pore segmentation or image resolution changing apparent connected-component / spanning classification.
- Identical mean Ks or retention curves but different breakthrough tails.
- Treatment changing morphology without changing transport, and treatment changing transport with little measured morphology change.

Latent pathway truth, water / tracer flux, observation operators, and measurement faults must be emitted separately. Verify conservation independently. Split by scenario family, soil profile, forcing, segmentation setting, and random seed. Synthetic success is a software control, not field validation.

### Tier C — prospective controlled cores / lysimeters

Where separately approved by qualified soil scientists and the relevant institutions, randomize treatment at the actual core / mesocosm unit within source-soil / batch blocks. Measure pre-treatment and follow-up root / pore / hydraulic state. Apply matched, calibrated event hyetographs and measure tracer breakthrough and mass recovery.

Repeated events, depths, sensors, image tiles, aliquots, and time points remain nested observations. They do not create additional treatment replicates. Randomization inference permutes labels at the assignment unit within blocks.

### Tier D — field transfer

Use randomized blocks when feasible, measured plot-level forcing, baseline structural measurements, repeated time points, and multiple sites / seasons. Natural storms are shared time blocks, not independent treatment replicates. Record run-on, slope, throughfall, canopy, antecedent state, edge effects, lateral flow, and collection coverage.

## 6. Comparator bake-off

All models receive the same permitted information and frozen group partitions for each task. Separate data-fitting from evaluation.

| Comparator | Question |
|---|---|
| Porosity / pore-count baseline | Does pore abundance alone explain the outcome? |
| Texture + bulk density + retention + Ks | What does standard hydraulic information explain? |
| van Genuchten–Mualem / Richards matrix flow | Does a single-domain physical model suffice? |
| MACRO / HYDRUS DualPerm / RZWQM2 where configuration supports the question | Does an established dual-domain model explain the event and tracer observations? |
| 2D image morphology / dye features | Does observed section morphology add explanatory value? |
| 3D CT topology features | Does measured 3D geometric connectivity improve out-of-sample inference? |
| Hirmas-style dynamic morphology | Does time-varying pore morphology improve transient hydraulic prediction? |
| Topology-informed dual-domain hybrid | Does an identifiable connectivity / exchange term improve prediction beyond a calibrated physical baseline? |
| Hierarchical / state-space model | Does partial pooling and temporal state tracking improve transfer and calibration? |
| Symthaea measurement policy | Does adaptive sampling reduce uncertainty in a predeclared estimand per unit cost? |

A more complex dual-domain / topology model is retained only if its additional parameters are identifiable from available observations and it improves a preregistered endpoint on frozen external groups. If a simpler model performs similarly, prefer the simpler one.

## 7. Splits, metrics, and proposed targets

Freeze connected dependency groups before model fitting. Group by study, site, block, experimental unit, soil source, treatment batch, image specimen, and event lineage as applicable. A specimen, image tile, tracer sample, or repeated event must not cross confirmatory partitions through another identifier.

Report pooled and per-site / per-soil / per-texture / per-time / per-forcing results. Include failures, missingness, censoring, non-identifiable parameters, and abstention.

### Primary physical metrics

- Tracer first-arrival interval error and coverage.
- Median / P90 / P95 breakthrough-time error and tail-mass error.
- Cumulative recovered mass fraction and recovery uncertainty.
- Event water / solute mass-balance residual relative to propagated measurement uncertainty.
- Depth-profile response timing and amplitude error.
- Lateral-versus-vertical route classification only where the reference can support it.

### Model / decision metrics

- MAE / RMSE / bias in physical units.
- CRPS or proper interval score, interval coverage and width.
- Worst-site / worst-texture performance and out-of-domain abstention.
- Parameter recovery and identifiability rate on known-truth cases.
- Calibration under sensor dropout, image-resolution changes, segmentation perturbation, event cadence changes, and forcing shift.
- Expected reduction in primary-estimand uncertainty per measurement cost / elapsed time.

### Proposed gates (targets, not results)

- Zero false physical pathway certificates on deterministic matrix-only and disconnected-path negative controls.
- At least 0.90 sensitivity on high-signal synthetic cases with independent latent truth, without violating the null-control gate.
- Nominal 90% predictive intervals achieve 85–95% coverage on independent held-out groups, with width and proper score reported.
- At least 10% lower primary tracer / transport error than the strongest valid comparator, with a group-aware uncertainty interval for the improvement excluding zero; no material worst-domain degradation.
- At least 20% lower P90 / P95 breakthrough-tail error without material median-error degradation.
- No parameter estimate is presented as identified unless the known-truth recovery and identifiability diagnostics support it; otherwise return an uncertainty set or abstain.
- 100% of confirmatory causal contrasts identify the actual randomized unit, comparator, baseline, follow-up, and nested sample lineage.
- No mediation claim where the mediator, temporal ordering, measurement error, or required confounding assumptions are not defensible.
- No water / nutrient mass claim without measured flow / volume, coverage, time alignment, and an uncertainty-qualified balance.

The numerical thresholds above are engineering targets for benchmark qualification, not universal agronomic importance thresholds.

## 8. Adaptive measurement design

Symthaea should select the next measurement that most reduces uncertainty about the distinction that matters: pore geometry, active connection, transport consequence, or treatment mechanism.

Compare against random sampling, fixed sensor / image layout, and a preregistered expert schedule under the same budget. Candidate actions include an additional depth sensor, a spatially shifted sampling location, CT imaging, dye section, isotope profile, tracer sample timing, or an additional hydraulic measurement. An action is selected only if it is feasible, approved, and expected to reduce uncertainty in the predeclared primary estimand—not merely reduce model entropy.

Use value of information:
expected reduction in decision-relevant uncertainty divided by monetary cost and elapsed time.
Preserve an exploration budget for out-of-domain and null cases so the selector does not repeatedly sample only where its current model is confident.

## 9. Execution plan and ownership

1. Audit the Mendeley version-2 workbook and its associated paper / supplements; publish a source-and-design reconciliation before treating observations as independent.
2. Acquire and checksum the three Zenodo lysimeter files; parse timestamps, units, hourly water flux, irregular tracer samples, concentration units, and injection masses into versioned schemas.
3. Reproduce at least one transparent breakthrough / water-age baseline and a matched HYDRUS / transport comparison only where initial and boundary conditions are supported.
4. Extend #5536 known-truth scenarios with topology / active-connectivity counterexamples and non-identifiability cases.
5. Integrate root / morphology descriptors from #5545 and the pore-network auditor without treating 2D morphology as 3D truth.
6. Link treatment contrasts to #5535; link time-varying structure and outcomes to #5527; link system-level function endpoints to #5525.
7. Publish a preregistered analysis and independent review before any confirmatory mediation or field efficacy statement.

- **Mycelix:** source custody, sample / batch / treatment lineage, experimental authority, frozen split manifests, and auditable contrast / report receipts.
- **Symtropy:** conservative physical forward models, constitutive laws, domain exchange, tracer transport, and solver / parameter identifiability checks.
- **Symthaea:** competing-mechanism hypotheses, uncertainty and abstention, causal question decomposition, and adaptive measurement selection.
- **Independent reviewers:** evaluate source fidelity, experimental assignment, model parity, statistical assumptions, and all physical / regulatory / community constraints.

## 10. Boundaries and nonclaims

This benchmark does not authorize material formulation, biochar production or application, field trials, application rates, nutrient substitution, safety approval, carbon permanence claims, or commercial release. Cover-crop evidence is not biochar evidence. Archaeological Amazonian Dark Earth is not a product recipe. Synthetic scenarios and model-to-model agreement are not independent field validation. Local South African transfer requires independently measured local soils, forcing, reference observations, permissions, and qualified review.

## References

- Villarreal, R. (2025), *Preferential flow is influenced by cover crops inclusion depending on soil texture*, Mendeley Data v2, DOI 10.17632/kdffpwdbdv.2.
- Asadollahi, M. et al. (2020), *Transport and Water Age Dynamics in Soils: A Comparative Study of Spatially Integrated and Spatially Explicit Models*, Water Resources Research, DOI 10.1029/2019WR025539; dataset DOI 10.5281/zenodo.3457960.
- Pyschik, J. and Weiler, M. (2026), *Detecting the occurrence of preferential flow in soils with stable water isotopes*, Hydrology and Earth System Sciences 30, 485–501, DOI 10.5194/hess-30-485-2026.
- Nimmo, J. R. et al. (2025), *Identifying preferential flow from soil moisture time series: Review of methodologies*, Vadose Zone Journal, DOI 10.1002/vzj2.70017.
- Hirmas, D. R. et al. (2025), dynamic macropore morphology–hydraulic conductivity model, DOI 10.1029/2024WR039836.
