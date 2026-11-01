# Sequential non-additive adaptive experiment design v0.1

**Status:** research specification plus a synthetic pairwise prototype. The prototype is a static pairwise look-ahead extension to #5600, not the full sequential posterior engine. It confers no experiment, land, material, sample-collection or field-trial authorization.

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


## 13. Executed prototype: TERRA-VOI-004

A first executable slice is committed under
`research/terra-preta/tools/adaptive-pairwise-bakeoff/` with workflow
`.github/workflows/terra-preta-adaptive-pairwise.yml`.

The v0.1 implementation compares:

- seeded random feasible acquisition;
- static additive greedy acquisition;
- pair-aware joint look-ahead over candidate single actions and action pairs;
- an exact feasible-subset oracle for small known-truth scenarios.

Selector/evaluator separation is enforced by type boundary: selection receives only
`PolicyContext`, `PolicyAction`, and `SelectorPair`. These contain permitted
standalone predictions and estimated pair interactions, but no evaluator-only
standalone truth or pairwise truth. The exact oracle and score evaluator receive
the full scenario only after selection. A negative-control test mutates every
truth field and requires selections to remain unchanged.

The designed fixture represents two nearby tracer measurements with high
standalone predicted values but a negative joint interaction, versus CT pore
geometry and event tracer measurements with lower standalone values but positive
joint interaction. Under matched budget and required coverage, the intended
failure mode is that static additive selection chooses the redundant tracer
pair while pair-aware look-ahead chooses the complementary CT/tracer pair.
The exact oracle evaluates the hidden synthetic utility independently.

**Interpretation restriction:** the fixture is a constructed software test.
It is not a physical model, an instrument recommendation, a measured effect,
or proof that CT plus tracer is universally superior. No fixture output is
reported as achieved until the exact-head Rust workflow passes.

## 14. Remaining scientific work after the prototype

This implementation does not yet model observation likelihoods, correlated
noise covariance matrices, posterior conditioning, parameter recovery,
simulation-based calibration, model-discrepancy updates, higher-order
interactions or adaptive stopping. Pairwise interaction estimates are static.
A policy can therefore select a complementary pair, but it cannot yet simulate
how the first observation changes the value of the second.

The next engine must make observations first-class simulated data:
`latent world -> observation operator -> noisy measurement -> posterior /
model-discrepancy update -> conditional next-action value`. The evaluator must
retain hidden truth, with independent held-out observation seeds and target
strata. It must compare greedy conditional EIG, batch/joint design,
decision-focused EVSI and discrepancy-aware abstention under matched cost,
coverage and compute budgets. It must report parameter recovery separately from
predictive calibration and final service-decision regret.


## 15. Executable categorical sequential engine — TERRA-VOI-005

A finite categorical sequential Bayesian prototype is now added at
`research/terra-preta/tools/sequential-bayes-design/`, with focused workflow
`.github/workflows/terra-preta-sequential-bayes.yml`.

Unlike the prior pairwise score prototype, this engine performs real Bayesian
conditioning on each generated observation:

`prior weights -> predictive outcome probabilities -> observed category ->
posterior model weights -> next action`.

It compares five policies under the same seed ensemble and hard budget /
coverage rules:

- static EIG (frozen initial-prior scores);
- conditional EIG (recomputed after each observation);
- decision-focused expected value of sample information (EVSI);
- discrepancy-guarded EVSI (after a low posterior-predictive probability,
  prioritizes an available predeclared diagnostic modality);
- seeded random feasible.

The evaluator samples observations from hidden truth outcome distributions,
which may lie outside the candidate model family. It scores selected management
decisions against evaluator-only truth utilities. It also records posterior
predictive negative log score, multiclass Brier score, surprise-event rate,
diagnostic-action frequency, final posterior entropy, cost and selection
frequency. The selector receives none of the hidden outcome probabilities or
truth decision utilities. An authored negative control mutates all evaluator
truth fields and requires the selector's first action to remain unchanged.

The first fixture represents matrix-flow, dual-permeability and root-channel
candidate models plus a deliberately out-of-family preferential-flow world.
The scenario is a method stress test, not a claim that those categorical
probabilities or utility values describe real soil. No numeric policy ranking
is promoted as evidence before the exact-head Rust workflow produces and
validates the machine-readable report.

### What this version actually adds

- posterior model-weight updates after every categorical observation;
- exact finite predictive probability, entropy and expected decision-value
  calculations for each candidate measurement;
- static versus conditional acquisition comparison;
- a predeclared surprise threshold and diagnostic-modality response;
- repeated deterministic synthetic-world evaluation;
- proper predictive scoring (negative log score and multiclass Brier score)
  alongside final decision regret.

### What it deliberately does not claim

- continuous observation likelihoods, censored measurements or sensor
  calibration;
- correlated or heteroscedastic observation noise across actions;
- a Gaussian-process or other learned posterior over model discrepancy;
- a continuous physical inverse solver, differentiable simulator or
  Symtropy/HYDRUS/MACRO implementation;
- simulation-based calibration (SBC) qualification or external posterior
  calibration;
- an identified cause for a predictive surprise.

The guard's alert means only that the current candidate-model mixture assigned
low predictive probability to an observed outcome. The event could arise from
an omitted physical mechanism, bad measurement, data corruption, wrong boundary
conditions, or a misspecified likelihood. A diagnostic action is therefore a
controlled next measurement, not an automatic correction or causal conclusion.

## 16. Research comparison and next scientific gate

The benchmark must compare against stronger sequential methods, not only
static EIG:

- Zhu and Hainy (2026, v2 dated 2026-09-29) develop prediction-oriented
  sequential Bayesian design for physical experiments informed by computer
  models, combining mutual information with local predictive-complexity
  information and a Kennedy–O'Hagan simulator/discrepancy/noise framework:
  https://arxiv.org/abs/2603.16756
- Yang, Dong and Wu (2026) propose discrepancy-aware Bayesian experimental
  design with auto-differentiable ensemble Kalman inversion:
  https://doi.org/10.1016/j.jcp.2025.114469
- Tang, Sloman and Kaski (AISTATS 2026) show why representativeness,
  informativeness and error de-amplification must be considered together under
  model misspecification:
  https://proceedings.mlr.press/v300/tang26d.html
- Modrák et al. (Bayesian Analysis, 2025) show that SBC sensitivity depends on
  the selected test quantities and recommend data-dependent checks such as
  joint likelihood quantities:
  https://doi.org/10.1214/23-BA1404

The next gate (#5603) is to extend this finite categorical baseline with
covariance-aware joint likelihoods, observation-support metadata, independent
SBC worlds and an explicit discrepancy process. A later continuous benchmark
must compare prediction-oriented sequential design, discrepancy-aware design,
conditional EIG, decision-focused EVSI and batch/joint design under the same
simulator-call, instrument-cost and target-coverage budgets.


## 17. Sequential Bayesian prototype implementation notes and falsifiable controls

The v0.1 categorical engine executes each policy over a shared preregistered
replicate seed set. It samples outcomes from evaluator-only truth, computes the
candidate-model posterior predictive distribution before every measurement,
records the negative log predictive score and multiclass Brier score, applies
Bayes' rule to update model weights, and re-evaluates the next acquisition from
the updated posterior.

The scenario adds a high-information / low-decision-value control: a nuisance
signature distinguishes two candidate physical models that prescribe the same
service decision. An independent calculation of the declared prior and
likelihoods gives expected information gain **0.3774776716 nats** and expected
decision EVSI **0** (within floating-point tolerance). This is a mathematical
fixture check, not executed Rust output. The regression test requires
`EIG > 0.1` and `EVSI < 1e-10`.

This is a critical benchmark property: posterior entropy reduction can be real
while management-decision value is zero. An acquisition system must not treat
the two objectives as interchangeable.

The discrepancy-guarded comparator is intentionally a testable heuristic, not
a proposed production policy. When an outcome has low probability under the
candidate model mixture, it can prioritize a previously declared diagnostic
modality. The report must retain predictive log score, Brier score, surprise
rate, decision regret and diagnostic-action frequency separately. If the guard
raises surprise counts but does not improve decision regret or predictive
adequacy, that is a policy failure to report—not a reason to tune away the
negative result.

## 18. Required next extensions

The finite prototype still assumes conditional independence across action
observations. Before any continuous physical trial, #5632 must add:

1. joint likelihoods or covariance-aware observation models for repeated and
   cross-modality measurements;
2. support-aware measurement contracts for image volume/resolution, core
   geometry, tracer sampling time/depth, sensor cadence and detection limits;
3. model discrepancy distinct from parameter uncertainty and measurement noise;
4. independent prior-predictive worlds and SBC quantities that include the
   joint data likelihood;
5. held-out predictive calibration across replicate seeds and target strata;
6. explicit abstention and controlled candidate-model expansion under
   out-of-family observations.

The benchmark literature is not a single settled SOTA leaderboard. Relevant
classes include prediction-oriented sequential design with local response
complexity (Zhu & Hainy, 2026), discrepancy-aware Bayesian design (Yang et al.,
2026), representative/informative/de-amplifying acquisition under shift and
misspecification (Tang et al., AISTATS 2026), and strengthened SBC checks using
data-dependent quantities (Modrák et al., 2025). These methods must be
implemented or fairly adapted and compared under identical observation
operators, simulator calls, measurement budgets, seeds and target strata.


## 19. Common held-out predictive calibration

The sequential engine now requires a fixed `validation_probes` array. These probes
are evaluator-only: they are not passed into the selector, are not eligible
acquisition actions, and do not consume the experiment budget. Every policy's
final posterior is scored on the same held-out probe definitions, candidate
likelihoods and evaluator-truth outcome distribution.

For a probe with posterior predictive probabilities \(p_k\) and known synthetic
truth probabilities \(q_k\), expected negative log score and expected
multiclass Brier score are computed as:

\[
\operatorname{NLS}(p,q) = -\sum_k q_k\log p_k
\]

\[
\operatorname{Brier}(p,q) =
\sum_k q_k\sum_j(p_j-\mathbb{1}[j=k])^2
\]

This is evaluated per target stratum and pooled by probe count. The report
distinguishes:
- **prequential selected-stream scores**, which describe predictions for
  different measurement operators chosen by each policy; and
- **common held-out scores**, which compare final posteriors on identical
  evaluation probes.

The latter are the matched cross-policy predictive comparison. The former
remain useful operational diagnostics, but should not be interpreted as a
fully controlled calibration contest because adaptive policies change the
observed measurement mix.

The in-family regression fixture sets each held-out truth distribution equal
to the DUAL_PERMEABILITY candidate likelihood. A test requires a posterior
concentrated on that model to obtain strictly better expected log and Brier
scores than a competing model posterior. Another negative control mutates
held-out truth without changing the selector's first choice. These are
software/reference-case tests, not calibration certification on a real
physical process.

This addition addresses one fairness problem but does not replace SBC. A
posterior can score well on a small fixed probe set and still be miscalibrated
elsewhere; future #5632 work must use prior-predictive worlds, independent
truths and data-dependent SBC quantities, including joint likelihood checks.
