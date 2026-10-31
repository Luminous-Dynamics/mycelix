# Sequential non-additive adaptive experiment design v0.1

**Status:** research specification. This is a synthetic known-truth extension to #5600. It confers no experiment, land, material, sample-collection or field-trial authorization.

## 1. Why static additive value is insufficient

The v0.1 adaptive-design bake-off sums per-action synthetic utility and ranks actions using static model scores. That isolates several important failure modes, but it cannot represent the central structure of sequential scientific experimentation:

- A second tracer sample at almost the same time and depth may be largely redundant with the first.
- A CT pore-network scan and an event tracer measurement may be complementary because the first measures geometry and the second measures active transport.
- A new observation changes the posterior over physical parameters and competing mechanisms, so the next action's value is conditional on what has already been observed.
- Two actions can be individually low-value but jointly identify a mechanism that neither can distinguish alone.
- A model can be confidently wrong under both actions if the shared observation operator omits an important process.

The correct object is therefore a *sequential, non-additive design policy*, not a one-shot ranking of independent actions.

## 2. Research objective

Compare experiment policies that select a sequence or batch of observations, update uncertainty after each observation, and optimize a predeclared decision under matched budget and population-coverage constraints.

The policy must value new information conditional on the current evidence, account for correlated observation errors and overlapping measurement support, and retain the ability to expose model discrepancy.

## 3. Physical cases to represent

Use mechanism-discriminating tasks already established in the Terra Preta programme:

- geometry versus hydraulically active connectivity;
- matrix-flow versus macropore-bypass transport;
- root-channel creation versus root-channel occupation / clogging;
- intact-core conductivity versus crack-mediated effective conductivity;
- hydraulic retention versus event transport and tracer recovery;
- amendment × root × wet–dry interactions;
- morphology change without functional change, and functional change with little observed morphology change;
- service-envelope trade-offs across infiltration, storage, aeration, drainage, cracks and nutrient export.

A measurement action must state its observation operator and support: e.g. CT resolution/volume, image section, tracer collection interval, sensor depth/cadence, sample volume, retention pressure range, or gas-permeability method.

## 4. Candidate action contract

Extend the #5600 experiment-action contract with:

- action-level observation-model ID and likelihood family;
- target parameter/mechanism IDs and expected information about each;
- action-pair redundancy/covariance descriptors, with provenance or simulator-derived support;
- possible complementarity / joint-identifiability relations;
- monetary and elapsed-time costs, sample count and laboratory throughput;
- feasible status and approval reference;
- target strata and measurement support;
- posterior update recipe and model discrepancy representation;
- evaluator-only synthetic truth and simulator state, which must never enter the selector.

The selector sees only prior distributions, candidate likelihoods, costs, current posterior state and permitted model predictions. The evaluator sees known latent truth only after the policy has selected and observed the simulated action.

## 5. Sequential decision loop

1. Freeze initial prior, model ensemble, target population, service profile and cost budget.
2. Select an eligible action subject to non-compensable coverage and budget constraints.
3. Generate a synthetic observation from the hidden physical world and the action-specific observation operator.
4. Update model/parameter posterior and discrepancy state.
5. Recompute conditional marginal value of remaining actions.
6. Continue until the budget or predeclared stopping condition is reached.
7. Evaluate the final service decision and compare with the exact feasible oracle on small action sets.

Do not update only the parameter posterior while holding model discrepancy fixed. If residual structure is systematic, the policy should consider model inadequacy, alternative mechanisms or a request for a new measurement modality.

## 6. Comparator policies

Compare under identical prior, observation simulators, target strata and cost:

- random feasible sequence;
- expert-fixed sequence;
- OFAT schedule;
- static EIG ranking (v0.1 baseline);
- conditional greedy EIG with posterior updating;
- batch EIG / joint design;
- decision-focused expected value of sample information;
- R-IDeA-inspired representative/informative/de-amplifying acquisition;
- robust decision-focused sequential policy with discrepancy and coverage constraints;
- exact best feasible subset/sequence oracle on small synthetic worlds.

The R-IDeA-inspired policy remains a transparent local comparator, not a claim to reproduce Tang et al.'s published objective.

## 7. Known-truth tests

Construct scenario families where:
- two same-modality actions have highly correlated measurement noise and nearly identical support;
- CT geometry plus event tracer jointly separates geometric from active connectivity;
- one additional modality reveals a missing process that all current candidate models omit;
- observation noise increases with moisture, depth or sensor saturation;
- one action's outcome shifts the value of another (positive or negative complementarity);
- sequential EIG becomes myopic and misses a high-value pair;
- batch design is better when setup costs are shared;
- information gain is high but decision value is near zero;
- a policy repeatedly samples a well-characterized stratum while rare target soils remain uncovered;
- posterior confidence rises under a misspecified likelihood but held-out target error worsens.

## 8. Metrics and proposed qualification targets

Targets, not achieved results:

- **No additive double-counting:** selected action-pair joint information is computed from the joint likelihood/covariance, not the sum of marginal information scores.
- **Redundancy accounting:** 100% of pairwise correlations supplied by the scenario are reflected in conditional marginal value.
- **Complementarity recovery:** identify high-signal synergistic action pairs or sequences in known-truth worlds.
- **Sequential calibration:** posterior predictive intervals remain calibrated after each update and on independent target strata.
- **Decision value:** lower final decision regret at equal budget than static EIG, random, expert-fixed and OFAT baselines.
- **Misspecification awareness:** posterior concentration without external predictive improvement triggers discrepancy warning, model competition or abstention.
- **Coverage:** no target stratum violates its minimum independent-unit requirement.
- **Oracle gap:** on small action sets, report normalized regret against exact feasible batch/sequence oracle; larger settings must report approximation quality and bound gaps.
- **Cost:** compare both total cost and elapsed wall-clock/throughput, including instrument setup and shared batch costs.

## 9. Computation and identifiability controls

Exact sequence enumeration is factorial in candidate count; exact batch subset enumeration is exponential. Restrict exact oracles to small cases, and use documented relaxations/upper bounds for larger settings. Report solver optimality gaps. Do not compare a greedy heuristic against an exact oracle on an unfairly small set then generalize the result to large field programmes.

Use posterior predictive checks, prior predictive checks, simulation-based calibration and parameter-recovery tests on known-truth worlds. Distinguish parameter identifiability from predictive adequacy: different parameter combinations or mechanisms can produce nearly identical observable outputs.

## 10. Relationship to existing work

- #5599: robust decision-focused adaptive experiment design;
- #5600: v0.1 synthetic additive policy bake-off;
- #5598: non-monotonic dose × root × wet–dry response surfaces;
- #5580: root/pore connectivity causal mediation;
- #5586: function-conditioned hydraulic service envelopes;
- #5536: known-truth preferential-flow and model-identifiability scenarios;
- #5528: hydraulic retention, hysteresis and transient flux.

## 11. Reference literature and industry context

Tang, R., Sloman, S. J., and Kaski, S. (2026). *Representative, Informative, and De-Amplifying: Requirements for Robust Bayesian Active Learning under Model Misspecification*. AISTATS 2026, PMLR 300, 3016–3024. https://proceedings.mlr.press/v300/tang26d.html

Yang, H., Dong, X., and Wu, J.-L. (2026). *Bayesian experimental design for model discrepancy calibration: An auto-differentiable ensemble Kalman inversion approach*. Journal of Computational Physics 545, 114469. https://doi.org/10.1016/j.jcp.2025.114469

Established hydraulic simulators such as HYDRUS DualPerm, MACRO and RZWQM2 remain forward-model comparators. This research direction does not replace their physical equations; it asks how to choose complementary measurements that distinguish their competing mechanisms and expose model discrepancy.

## 12. Governance boundary

This is a synthetic research benchmark only. It does not authorize any sampling, material handling, amendment, field trial or access to land. All real experiment proposals remain subject to qualified scientific, landholder, institutional, safety, regulatory and community review.
