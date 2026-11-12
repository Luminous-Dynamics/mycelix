# Preferential-Flow Sensor Placement and Adaptive Measurement Design Benchmark v0.1

**Status:** research specification and synthetic cadence/leave-one-sensor-out harness are committed. A validated physical candidate-depth simulator, field deployment experiment and costed adaptive-placement result do not yet exist. All thresholds and targets below are proposed, not achieved.

## 1. The objective mismatch in current sensor-layout practice

Krueger et al. (2026, Vadose Zone Journal, DOI 10.1002/vzj2.70100) provide a useful industry reference for sensor number and depth selection. Their objective is accurate profile-average water content/storage, not detection of localized preferential-flow pathways or solute mass export. The study compared 34 measured soil profiles with 6,576 modeled profiles; for storage measurement it recommends approximately 5, 20 and 40 cm for a 0–50 cm profile, and 10, 30, 60 and 90 cm for a 0–100 cm profile. It found little additional storage-error improvement beyond three sensors at 50 cm and four at 100 cm. The familiar 5, 10, 20, 50 and 100 cm layout had median absolute storage error 1.8–2.8 times that of the best four-sensor layout in its comparison.

These are valuable storage-monitoring baselines. They are **not preferential-flow optima**. A layout that reconstructs the profile mean can still miss a narrow, disconnected, lateral or rapidly transmitting pathway. Conversely, additional sensors may improve pathway detection while adding little to profile-average storage accuracy. Never transfer the Krueger depth recommendations into PF recommendations without testing the PF objective.

### Reusable industry storage-layout reference

Krueger et al. provide a public [Soil Moisture Sensor Layout Explorer](https://soilwater.github.io/soil-moisture-sensor-depths/). Its reference profiles are 1-m cores sliced at 5-cm intervals; a hypothetical sensor reading is simulated as the average of the adjacent 1-cm layers, and storage is estimated by assigning each sensor a vertical layer bounded by midpoints to neighboring sensors. The tool reports relative/signed storage error and optionally shows a ±0.03 m³/m³ sensor-accuracy band.

This is a useful reproducible comparator for *storage representation*. It does not measure event response, pathway continuity, tracer breakthrough or event boundary mass. Its idealized sensor readings should be kept separate from a realistic PF sensor observation model with spatial support, noise, drift, saturation and event timing. Use the tool as a reference, not as a PF validation or a substitute for our independent candidate-layout experiment.

## 2. Paradigm shift: place sensors to distinguish mechanisms, not merely represent an average

The design target should be a distribution of physically plausible mechanisms and fluxes, not one spatial mean:

- matrix-dominated piston displacement;
- connected vertical macropore bypass;
- disconnected/dead-end macropore wetting;
- lateral flow along a textural/structural interface;
- perched water or a rising water table;
- dual-domain transport with matrix–macropore exchange;
- high-antecedent-moisture response and sensor saturation;
- localized pathways that miss one or more point sensors.

A design is valuable when its measurements discriminate between these alternatives and reduce uncertainty in decision-relevant quantities (e.g. lower-boundary water/tracer/nutrient mass), not just when its sensors correlate with each other.

## 3. Candidate layouts and fair comparators

Every site/profile must define the available candidate actions before examining test outcomes. Actions may include sensor depth, horizontal position/replicate, measurement technology, cadence, pressure-head instrument, water-table sensor, tracer sampler, ERT/GPR transect or lysimeter boundary measurement.

For a 0–100 cm profile, begin with the following *comparison layouts*, not prescribed field deployments:

1. **Krueger storage baseline:** 10, 30, 60, 90 cm.
2. **Legacy fixed layout:** 5, 10, 20, 50, 100 cm.
3. **Even-spacing PF reference:** approximately evenly spaced depths, constrained so the deepest point is above the target control-volume boundary.
4. **Horizon-informed layout:** candidate points immediately above/below measured restrictive horizons, ped boundaries or major structural transitions.
5. **Root/biopore-informed layout:** candidate depths informed by independently measured root density and pore morphology, not inferred from the response being scored.
6. **Spatial-replication layout:** fewer depths with replicated horizontal positions to estimate lateral heterogeneity.
7. **Random fixed layout:** many seeded random layouts to establish a distribution, not one favorable random comparator.
8. **Adaptive candidate:** greedily or optimally selected actions using a training-only mechanism/measurement model and a declared acquisition cost.

For 0–50 cm, include the Krueger 5/20/40 cm storage baseline. For other control-volume depths, enumerate candidate depths explicitly and declare the lower boundary; do not interpolate the published storage recommendation as if it were a PF law.

Every comparison must use the same soil profiles, storms, sensor models, calibration error, missingness model, budget, held-out scenarios and independent reference evidence.

For the lateral-heterogeneity track, the 2025 Nimmo et al. review reports that many studies use 10–30 cm spacing in both horizontal and vertical directions; finger-flow detection can require 5–10 cm spacing or better. It cautions that mm-scale macropore flow is generally not resolved by this method unless surrounding matrix wetting expands over a substantial width. These are method-specific resolution observations, not universal deployment prescriptions. Candidate layouts must be evaluated against the expected wetted footprint, sensor support volume and the target mechanism, with explicit false-negative limits for pathways narrower than the sensor spacing.

## 4. Utility: expected reduction in decision-relevant uncertainty per total cost

Let H be the latent mechanism/parameter state and D the observations already collected. An action a is a candidate measurement or sensor deployment. A first information-theoretic score is:

`IG(a) = H[H | D] - E_{y ~ p(y|D,a)} H[H | D,y,a]`

This score is useful for mechanism discrimination but can overvalue uncertainty that does not change the scientific or operational decision. Prefer an expected reduction in Bayes decision risk where the loss matrix is explicit:

`VOI(a) = R(D) - E_y[R(D,y,a)] - C(a)`

where C(a) includes sensor purchase, installation, calibration, power/telemetry, maintenance, data QA, sample collection and laboratory/geophysical processing. Report the gross risk reduction, each cost component and net value separately; do not hide a negative-value action behind a normalized ratio.

Use robust selection across predeclared soil, climate, antecedent-state and forcing strata. Report mean, worst-stratum and lower-tail utility; consider CVaR or maximin selection when a rare but consequential pathway must not be systematically missed. Do not optimize a single pooled average when localized pathways are the target.

The observation model must include sensing volume, calibration uncertainty, depth/position uncertainty, temporal cadence, drift, noise, saturation/frozen/fault states, missingness and installation disturbance. For dielectric/FDR sensors, calibration uncertainty must be stratified by soil type, temperature and electrical conductivity rather than represented by a single global noise constant. A 2026 review of FDR-based capacitive sensors identifies these as material interference variables and argues for physically decoupled compensation; treat this as an instrumentation baseline to test, not proof that a universal calibration can be transferred across soils.

The Krueger et al. (2026) robust-design analysis perturbed hypothetical sensor measurements with Gaussian error (SD 0.01 m³/m³ and a 0.02 sensitivity case) and selected layouts by worst-case R² across error realizations. We should reproduce this kind of measurement-error stress test but score PF-specific mechanism/flux utility instead of profile-mean storage error.

## 5. Required implementation protocol

### 5.1 Design simulation

1. Enumerate candidate actions and their full cost.
2. Generate or simulate mechanism-specific latent states using validated physical kernels; distinguish analytically controlled toy models from calibrated soil simulators.
3. Apply each sensor's observation operator and realistic measurement error to the latent profile.
4. Evaluate every fixed comparison layout and the adaptive policy on identical held-out scenario seeds, profiles, forcing and measurement realizations.
5. Train/tune selection policies only on training profiles; freeze the action policy before outer test.
6. Evaluate all chosen layouts against latent known truth and independent tracer/geophysical/boundary observations as separate evidence layers.

### 5.2 Prospective randomized sensor-placement trial

For a real field/lysimeter subset, randomize candidate sensor layouts within comparable blocks or replicate cores. Keep a high-resolution reference instrumentation configuration on an adjudication subset so that adaptive selection does not choose the only sensors used to evaluate itself. Record the probability of each selected action and retain a random exploration arm to avoid selection/verification bias.

Measure response signatures, water/solute breakthrough, mass recovery and installation/maintenance burden. An algorithm cannot validate its own placement policy using only the sensors it selected.

### 5.3 Design constraints

- A sensor at the profile boundary is not a substitute for a measured boundary flux.
- At least two distinct depths are necessary for a vertical NSR pair, but two sensors alone can leave most profile structures unobserved.
- Horizontal replication is required to expose spatially sparse pathways and estimate the probability of missing them.
- Pressure head, moisture, tracer concentration and drainage are distinct observables; do not treat them as interchangeable.
- Measurement designs should include the target horizon and extend below the target control volume where feasible, while keeping the actual flux boundary independently measured.
- Sensor locations must be feasible, maintainable and not create preferential flow along installation holes.

## 6. Metrics and proposed gates

All targets are proposals until pilot uncertainty and costs are measured.

| Objective | Required metric | Proposed target |
|---|---|---|
| Null safety | Matrix-only physical false-certificate rate | Zero false certificates on deterministic matrix-only controls; on field data, upper confidence bound at a preregistered low operating point |
| Detection | Sensitivity against high-signal tracer-confirmed connected pathways | At least 0.90 at no more than 5% false-positive rate |
| Mechanism discrimination | Worst-stratum balanced accuracy / proper score | At least 0.85 in each prespecified major mechanism/soil stratum, otherwise calibrated abstention |
| Flux relevance | Water/tracer/nutrient boundary-mass error and P90/P95 tail timing | At least 20% lower error than the best matched fixed layout |
| Calibration | Nominal 90% interval coverage | 85–95% overall and no severe worst-stratum undercoverage |
| Sensor efficiency | Total cost at matched performance | At least 20% lower total cost than the best fixed layout at the same sensitivity/FPR operating point |
| Robustness | Leave-profile/site/soil-family/event-regime-out | No pooled-only headline; publish worst-case strata and OOD abstention |
| Policy validity | Adaptive vs randomized fixed-layout control | Prospective improvement with cluster-aware uncertainty; no retrospective-only policy claim |

A sensor design fails if it improves NSR/proxy agreement but worsens tracer-mass recovery, physical false certificates, calibration, or worst-stratum performance. A high F1 is not a substitute for this multi-objective gate.

## 7. Existing executable work and current gap

- [NEON soil-water normalizer](./tools/neon-swc-normalizer/README.md) maps pinned raw sensor series, applies declared QA/porosity filters, aggregates to 5–30 minute medians and keeps a raw-normalized series separate from the short-gap-interpolated paper-analysis series.
- [Profile response extractor](./tools/profile-response-extractor/README.md) creates interval-censored response times and abstains on unusable or missing evidence.
- [Profile-flow audit](./tools/profile-flow-audit/README.md) compares all depth pairs using onset intervals rather than forcing ambiguous pairs into a class.
- [Profile threshold-stability audit](./tools/profile-threshold-stability-audit/README.md) tests threshold-dependent signature changes.
- [Spatial heterogeneity audit](./tools/spatial-heterogeneity-audit/README.md) measures position/depth-conditioned dispersion and semivariograms.
- [Profile design sensitivity runner](./tools/run-profile-design-sensitivity.sh) reprocesses the same 1-minute profile at 5/10/15/30-minute cadences and performs leave-one-sensor-out ablations. It is a robustness tool for an existing layout, **not** a new-depth optimizer and not a field validation.
- [Known-truth generator](./tools/known-truth-generator/README.md) is currently lumped/two-reservoir and does not generate spatially resolved candidate-depth observations.

The next implementation must connect the validated spatial hydraulic/measurement model to candidate depth/location actions and then optimize the explicit utility above. Until that exists, report ablation sensitivity but do not claim optimal sensor placement.

## 8. Architecture responsibilities

- **Symtropy:** generate spatially resolved latent hydraulic/tracer states; provide observation operators for candidate sensors and physical conservation checks.
- **Symthaea:** maintain competing mechanism hypotheses, estimate identifiability and select the next measurement by expected reduction in decision loss per fully loaded cost, with robust stratum constraints.
- **Mycelix:** bind candidate action roster, costs, policy/model SHA, training/test profiles, selection propensity, randomized arm, measurements, raw-file hashes, calibration and independent adjudication.

## References

- Krueger, E. S. et al. (2026). Optimizing soil moisture sensor installation depths and number of sensors for in situ monitoring networks. *Vadose Zone Journal*, 25(3), e70100. https://doi.org/10.1002/vzj2.70100
- Xu, Y. et al. (2026). Advances in Calibration Methods for FDR-Based Capacitive Soil Moisture Sensors. *Sensors*, 26(11), 3366. https://doi.org/10.3390/s26113366
- Nimmo, J. R. et al. (2025). Identifying preferential flow from soil moisture time series: Review of methodologies. *Vadose Zone Journal*, 24(2), e70017. https://doi.org/10.1002/vzj2.70017
- Li, B. et al. (2026). Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes. *Earth's Future*. https://doi.org/10.1029/2026EF008296
