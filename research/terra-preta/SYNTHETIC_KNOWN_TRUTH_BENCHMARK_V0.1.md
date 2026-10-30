# Synthetic Known-Truth and Identifiability Benchmark v0.1

**Status:** executable challenge specification. The eventizer is a first preprocessing component; a coupled physical scenario generator has not yet been implemented. Scenarios and targets below are proposals, not simulated or measured results. No field application is authorized.

**Implementation tracker:** [TERRA-SYNTH-001](https://github.com/Luminous-Dynamics/mycelix/issues/5536).

## 1. Paradigm shift: evaluate whether the model knows what it claims to know

A sensor classifier can accurately reproduce a label derived from those same sensors and a hydraulic prior. That is useful proxy prediction; it is not independent evidence that a connected macropore exists, water bypassed the matrix, or nutrient mass crossed the root-zone boundary.

Maintain three truth layers:

1. Observable truth: rainfall, moisture, pressure head, runoff, drainage, tracer concentration and recovered mass.
2. Latent physical truth: known synthetic matrix/macropore configuration, pathway connectivity, domain flux, exchange coefficient, tracer residence-time distribution and boundary flux.
3. Reference-method truth: independent dye/tracer/geophysical observations, detection limits, and known false-positive/false-negative mechanisms.

The model predicts observations and uncertainty about latent state. It cannot promote a sensor signature or proxy label to confirmed pathway status.

## 2. Synthetic challenge families

| Family | Known truth | Failure exposed |
|---|---|---|
| Null matrix-only | No connected preferential pathway; matrix flow only | False certificates from rapid wetting, sensor noise or low Ksat prior |
| Single connected macropore | Known fast path, travel time and flux fraction | Missed high-signal tracer-confirmed bypass |
| Dual-domain exchange | Known matrix/macropore fractions and exchange coefficient | Non-identifiability between macropore fraction, velocity and exchange |
| Disconnected macropore | Fast local wetting, no lower-boundary breakthrough | Local sensor response misread as profile-spanning bypass |
| Layer interface/perched response | Known hydraulic contrast and lateral redistribution | Perched/lateral flow mislabeled as vertical bypass |
| Runoff-limited extreme storm | Known infiltration capacity and runoff split | Assuming greater rainfall intensity always means faster subsurface flow |
| Hydraulic hysteresis | Different wetting/drying scanning curves and conductivity | Memoryless retention/transport assumption |
| Structural evolution | Prespecified pore-connectivity change over repeated perturbations | One static parameter set used to represent soil aging |
| Observation corruption | Known drift, missingness, calibration offsets and collector recovery | Overconfidence from incomplete or biased sensors |
| Domain shift | Held-out texture, climate, storms, roots, sensor spacing and profile structure | Random-row scores hiding real-world transfer failure |

A first generator should use analytically controlled dual-domain response functions and deterministic observation operators. Couple it later to validated Symtropy Richards and dual-permeability kernels. A toy analytical generator must never be described as a field-calibrated soil simulator.

## 3. Truth and split artifacts

Every scenario emits a generator version, deterministic seed, parameter vector and units, governing equations, forcing, initial/boundary conditions, latent domain fluxes, conservative tracer mass, observation operator, noise/drift/missingness, sensor/event labels, and SHA-256 digests for inputs, truth, observations and split manifests.

Splits are grouped by scenario family, seed, profile and forcing regime. No seed, parameter realization, profile, forcing template or transformed derivative may cross outer partitions. Reserve entire scenario families for the hardest OOD test.

## 4. Identifiability and active experiment design

Before optimizing prediction error, report local sensitivity/Jacobian rank and singular values, profile likelihood or posterior ridges, parameter recovery under repeated noise realizations, predictive checks at unobserved depths/times, and the difference between structural and practical identifiability.

If macropore fraction and exchange coefficient cannot be separated from available observations, report the identifiable combination or abstain. A converged optimizer is not proof of identifiability.

Symthaea should choose the next sensor depth, tracer sampling time or rainfall pulse by expected reduction in decision-relevant uncertainty (for example, separating a disconnected pathway from lower-boundary bypass), not predictive entropy alone.

## 5. Metrics and proposed targets

| Dimension | Metrics | Proposed target (not achieved) |
|---|---|---|
| Null safety | False-positive pathway rate, false certificates | Zero false certificates on deterministic matrix-only suite; abstention allowed |
| Strong-path detection | Sensitivity, specificity, MCC, AUCPR, detection delay | At least 0.90 sensitivity on high-signal tracer-confirmed cases without violating null safety |
| Transport | Tracer arrival/peak/tail error; recovered mass | At least 20% lower P90/P95 tail error than strongest matched baseline |
| Parameter recovery | Relative error and interval coverage by parameter | At most 20% median error only for identifiable parameters; otherwise calibrated abstention |
| Uncertainty | 90% interval coverage, width, proper interval score | 85–95% coverage on held-out families; report sharpness and worst-family coverage |
| Conservation | Water and conservative-solute residuals | Within independently propagated measurement/numerical uncertainty; no silent closure repair |
| Transfer | Leave-family/profile/forcing/sensor-design out | No random-row-only report; publish worst-family performance and abstention |
| Efficiency | Runtime, sensors, calibration burden, experiment cost | Explicit cost/performance Pareto improvement, not runtime alone |

A candidate fails if average F1 improves while false certification, tracer-tail error, conservation, calibration or worst-domain behavior materially deteriorates.

## 6. Matched model bake-off

Compare matrix-only Richards, dual-permeability physical models, interpretable event-feature logistic/GAM, Random Forest and gradient-boosted trees, published ResNet reproduction, temporal/state-space models, Symtropy's physical kernel and a hybrid physics-constrained state-space candidate. Use identical observations, forcing, boundary conditions, outer splits and independent references. Declare model-specific inputs and parameter burden. Unsupported outputs are not zero-error wins; retain solver failures and abstentions.

## 7. Keep the scientific claims separate

- NEON event prediction evaluates transfer of observational predictors against the published sensor/Ksat-derived proxy. It does not establish amendment effects or tracer-confirmed pathways.
- Synthetic known-truth evaluation tests recovery of controlled generative mechanisms.
- Tracer/lysimeter experiments test physical pathway claims and event mass balance.
- Randomized paired-forcing studies estimate amendment effects.

None of these evidence classes substitutes for another.

## 8. Execution order

1. Freeze the 10-minute eventizer and reconcile the response baseline with author code/supplement.
2. Build a deterministic two-domain known-truth generator with latent-state and observation manifests.
3. Pass matrix-only null controls and conservation checks before training candidates.
4. Add identifiability, interval calibration and abstention tests.
5. Run physical and statistical comparators on identical held-out scenarios.
6. Reproduce the historical NEON split, then run site- and ecoregion-held-out transfer.
7. Validate pathway claims with independent tracer and measured mass balance.
8. Only then progress to preregistered randomized amendment experiments under paired hydrologic forcing.

The proposed breakthrough is a closed-loop experimental digital twin: Symtropy predicts conservation-respecting outcomes, Symthaea chooses the next measurement that distinguishes mechanisms, and Mycelix binds raw data, releases, model configurations, split manifests, calibration and independent review into provenance. It must be able to say a mechanism is not identifiable yet and identify the cheapest discriminating experiment.