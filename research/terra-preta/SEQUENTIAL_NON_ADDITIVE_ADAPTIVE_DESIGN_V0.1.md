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


## 20. Preliminary policy-run evidence: preserve negative results

A previous exact Rust workflow execution produced two 128-replicate JSON reports
before the later selector-state Clippy refactor and common-held-out-probe
extension. The workflow passed formatting, all 14 then-authored tests, both
scenario schema validations, both report schema validations, execution, and
artifact upload; it failed only at Clippy because `next_action` had too many
arguments. The selector was then refactored into `SelectionState`.

Run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41289633870
Artifact ID: 13077010803 (the two synthetic JSON reports).

On the deliberately out-of-family preferential-flow scenario, the preliminary
**mean true decision regret** over 128 replicates was:

| Policy | Mean regret |
|---|---:|
| Seeded random feasible | 0.0664 |
| Discrepancy-guarded EVSI | 0.1328 |
| Conditional EIG | 0.1641 |
| Static EIG | 0.1992 |
| Decision-focused EVSI | 0.2109 |

The discrepancy-guarded heuristic selected a mean 1.92 diagnostic actions and
recorded 2.77 surprise events per trajectory, yet its *prequential
selected-stream* negative log score (3.423) and Brier score (1.195) were worse
than the random baseline (3.070 / 1.119). Static EIG had better prequential
scores still (2.132 / 0.875) while having worse decision regret. These metrics
refer to policy-selected measurement streams and are not the common held-out
comparison introduced in section 19.

In the in-family DUAL_PERMEABILITY control, the random baseline again had the
lowest decision regret (0.000); decision EVSI (0.0188) and discrepancy-guarded
EVSI (0.0234) followed, while static and conditional EIG both had 0.0609. Static
EIG had the best prequential log/Brier scores in that fixture (0.595 / 0.382).
This single designed control does not establish random as generally superior,
but it does show that the new heuristics do not yet dominate simple baselines.

**Interpretation:** the current discrepancy flag plus diagnostic-action rule
is not a successful discrepancy-learning policy in the stress case. It can
spend heavily on diagnostics while leaving the posterior inadequately
calibrated. The current observation outcome labelled `unmodeled` has nearly
uniform low likelihood across the candidate physical models, so it does not
provide a direction for posterior correction. A real next step is an explicit
discrepancy state / model-expansion mechanism with calibrated uncertainty and
abstention—not a stronger score multiplier on the same heuristic.

These report values are preliminary outputs from the prior code snapshot; the
latest implementation adds common held-out probe scores and additional tests.
Re-run the latest exact commit before using any figures as current benchmark
evidence. No external, physical or field-performance claim follows from these
synthetic results.


## 21. Common-held-out run results (workflow #27)

The updated source at PR head
`fe6b5647e9702caf34f668523cbfa613ddf90012` passed the dedicated sequential
workflow. The run compiled the crate, passed all **17** authored Rust tests,
validated both scenario fixtures, executed both 128-replicate policy runs,
validated both output reports against the report schema, uploaded the reports,
and passed Clippy.

Run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41290246028
Artifact: 13076404930
Artifact SHA-256:
`530a0ce7a9fe5c103311e92955a6b4a662afa1b891eeee9b5758b8f654a534e9`.

The common held-out scores generated by that exact run are:

| Synthetic world | Policy | Mean decision regret | Held-out expected NLS | Held-out expected Brier |
|---|---|---:|---:|---:|
| Out-of-family preferential flow | Static EIG | 0.1992 | 3.4567 | 1.2338 |
| Out-of-family preferential flow | Conditional EIG | 0.1641 | 3.4563 | 1.2325 |
| Out-of-family preferential flow | Decision EVSI | 0.2109 | 3.4558 | 1.2327 |
| Out-of-family preferential flow | Discrepancy-guarded EVSI | 0.1328 | **3.4478** | **1.2117** |
| Out-of-family preferential flow | Seeded random | **0.0664** | 3.4515 | 1.2157 |
| In-family DUAL_PERMEABILITY | Static EIG | 0.0609 | 0.6133 | 0.3921 |
| In-family DUAL_PERMEABILITY | Conditional EIG | 0.0609 | 0.6133 | 0.3921 |
| In-family DUAL_PERMEABILITY | Decision EVSI | 0.0188 | 0.6156 | 0.3902 |
| In-family DUAL_PERMEABILITY | Discrepancy-guarded EVSI | 0.0234 | 0.6072 | 0.3841 |
| In-family DUAL_PERMEABILITY | Seeded random | **0.0000** | **0.5800** | **0.3600** |

Held-out expected negative log score (NLS) and Brier score are lower-is-better.
The discrepancy-guarded policy has a small predictive-score edge over random
in the out-of-family fixture, but random has materially lower decision regret.
In the in-family fixture, random is best on both regret and held-out scores.
This is not a universal policy ranking: the fixtures are deliberately small,
the measurement distributions are categorical and synthetic, and the
simulation seed set is finite.

The main scientific finding is that *there is no single winner across the
declared objectives*. Information reduction, predictive scoring and decision
utility disagree. A production-quality acquisition system must expose this
multi-objective trade-off rather than compress it into an unqualified
"information gain" score. Future comparisons require more scenario families,
common held-out probes, uncertainty intervals over replicate worlds, and
matched compute / measurement costs. No SOTA claim follows from these runs.


## 22. Explicit discrepancy-state model-expansion control

The sequential benchmark now includes a third scenario:
`synthetic-discrepancy-state-control.json`. It extends the three physical
candidate models with a fourth categorical `DISCREPANCY_STATE` hypothesis.

The added state has a predeclared prior weight of 0.05 and a broad outcome
likelihood that assigns 0.70 probability to the residual / "unmodeled" outcome
for the three-outcome measurement modalities. The physical candidate models
assign 0.01 to that same outcome. Under the sandy event-tracer residual
observation, the exact finite Bayes update gives:

[
P(D=1\mid y=\text{unmodeled}) =
\frac{0.05\times0.70}
{0.38\times0.01+0.33\times0.01+0.24\times0.01+0.05\times0.70}
\approx 0.7865
]

The regression test requires posterior mass above 0.75 and that the
model-expected decision becomes `DIAGNOSE_FIRST`. The synthetic world gives
that decision the highest truth utility because the residual process has made
the current mechanistic choice unsafe to resolve from the candidate physical
models alone.

This makes the difference between a **surprise flag** and a **posterior
discrepancy hypothesis** concrete. The flag only reports low predictive
probability; a discrepancy-state hypothesis supplies a likelihood and can
change the posterior and downstream decision.

The control is deliberately simple and has an important limitation: the
discrepancy-state prior and broad likelihood are specified by the benchmark
author. This is not a learned discrepancy field, not a Gaussian process, not
a proof that the true residual process is represented, and not a continuous
physical model. An ill-calibrated catch-all component can absorb bad data or
mask an incorrect measurement operator. The next phase must infer and
calibrate discrepancy structure with data-dependent checks, distinguish it
from observation error and parameter uncertainty, and test sensitivity to
prior/likelihood choice. Never add a broad component merely to make model
surprises disappear.


## 23. Paired synthetic outcomes and replicate uncertainty

The evaluator now uses common random numbers keyed by
`(scenario seed, replicate ID, action ID)`, not by acquisition step. Thus an
action's potential synthetic outcome is stable across policies in the same
replicate even when selected at different sequence positions. This reduces
Monte Carlo noise in cross-policy comparisons and makes same-action contrasts
paired at the replicate/action level. Random policy tie-breaking remains keyed
by step, because acquisition order is part of that policy.

Every policy summary now reports the standard error and 95% normal-approximation
interval for mean decision regret and common held-out expected log/Brier
scores. These intervals use sample variance across the 128 independent
replicates and are clamped to each metric's known support where applicable.
They are Monte Carlo uncertainty intervals for a fixed synthetic scenario,
not Bayesian credible intervals and not intervals over the population of
possible scenario families.

The benchmark still needs paired bootstrap intervals for policy differences,
scenario-family random effects or hierarchical summaries, and independent
seed suites. Common random numbers reduce action-level noise but do not remove
the need for many worlds, independent data-generating mechanisms, or matched
compute and measurement budgets.


## 24. Measurement-support contract: prerequisite to covariance-aware likelihoods

A machine-readable observation-support contract is now committed:

- Specification: `research/terra-preta/OBSERVATION_SUPPORT_CONTRACT_V0.1.md`
- Schema: `research/terra-preta/tools/sequential-bayes-design/observation-support.schema.json`
- Synthetic fixture: `research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-observation-support.json`

Four synthetic records exercise CT pore geometry, event tracer breakthrough,
water-retention curve and gas permeability. Each record distinguishes
measurement/support/sample IDs, treatment-assignment unit, source batch,
modality, measurand/unit, observation-operator ID, spatial support and temporal
support, instrument/calibration/detection metadata, and uncertainty provenance.

This is not merely metadata decoration. A CT volume and a tracer collection
window answer different physical questions. Repeated tracer intervals can be
correlated through a shared sample or time series; CT segmentation can share
calibration/processing effects; retention and gas-permeability measurements
taken on related cores can share sample lineage. The benchmark must model these
dependencies only when supported by repeatability, paired, inter-operator or
cross-instrument evidence, or by a declared synthetic simulator.

**Do not infer a covariance coefficient from geometric overlap alone.**
Support overlap is an auditable reason to investigate dependence; it does not
determine the correlation value. Non-overlapping supports can still be
correlated through spatial fields, temporal autocorrelation or shared
calibration drift.

The focused sequential workflow now parses the support schema/fixture and
validates the fixture through the standalone schema-audit tool. This verifies
the metadata contract only; the current categorical Bayesian engine does not
yet consume the support records or use a covariance matrix. That is a distinct
next implementation gate under #5632.


## 25. Discrete simulation-based calibration (SBC) reference gate

The sequential Bayesian crate now contains an in-family finite-model SBC test
with a data-dependent joint-likelihood test quantity. Across 4,096 deterministic
prior-predictive worlds, the test:

1. samples the true candidate model from the declared prior;
2. generates two categorical observations from that model's likelihoods;
3. updates the posterior with the same declared model likelihoods;
4. computes a randomized posterior rank for the true discrete model index;
5. computes a randomized posterior rank for the joint log likelihood of the
   realized data under each candidate model.

The parameter-index rank is a basic discrete SBC quantity. The joint
log-likelihood rank is data-dependent and is included because marginal
parameter-rank checks alone can miss important posterior pathologies. The
regression negative control deliberately misspecifies one model/action
likelihood while keeping the truth generator unchanged; at least one rank
histogram must then fail the same tolerance.

An independent reproduction of the exact deterministic seed construction
gave in-family parameter-rank bin counts from 383 to 432 and joint-likelihood
bin counts from 394 to 443 (4,096 replicates; 10 bins). The deliberate
misspecification produced parameter-rank counts from 280 to 691 and
joint-likelihood counts from 318 to 671. The authored Rust test uses the
preregistered broad 7–13% frequency band for each bin. These counts are
independent calculations, not Rust test output; only exact-head CI can
establish that the Rust test compiles and passes.

This is a discrete reference case, not an SBC certification of the whole
engine. Next steps must add more test quantities, including model indicator,
decision functional and joint data likelihood; multiple priors and model
families; and independent scenario-generation seeds. A continuous simulator
requires its own SBC design, including prior predictive simulation and
posterior draws for discrepancy and sensor-covariance parameters. Refer to
Modrák et al. (Bayesian Analysis, 2025), who show the sensitivity of SBC to
test-quantity choice and emphasize data-dependent quantities such as the
joint likelihood: https://doi.org/10.1214/23-BA1404.


## 26. Executed three-world sequential benchmark — workflow #49

Exact source head:
`56c333ebc2ff0944151678e5a018678c1279532e`

Workflow:
https://github.com/Luminous-Dynamics/mycelix/actions/runs/41291225381

The focused workflow passed formatting, **20/20 Rust tests**, schema validation for
the observation-support contract and all three scenario fixtures, three
128-replicate executions, all three report-schema validations, artifact upload,
and Clippy. Artifact digest:
`sha256:cad10016645f93bd2ebfb634b5634b39198b6118d86603518a76cab9d3dd8454`.

The exact report artifact contains these 128-replicate mean decision regrets:

| Synthetic world | Static EIG | Conditional EIG | Decision EVSI | Discrepancy-guarded EVSI | Random |
|---|---:|---:|---:|---:|---:|
| Out-of-family preferential flow (3 physical models) | 0.2227 | 0.1758 | 0.2344 | **0.1016** | 0.1797 |
| In-family DUAL_PERMEABILITY | 0.0422 | 0.0422 | **0.0188** | **0.0188** | 0.0422 |
| Explicit discrepancy-state hypothesis | 0.0880 | 0.0641 | 0.0364 | **0.0128** | 0.0378 |

Common held-out expected scores (lower is better):

| Synthetic world | Policy | Expected NLS | Expected Brier |
|---|---|---:|---:|
| Out-of-family preferential flow | Discrepancy-guarded EVSI | **3.4500** | **1.2167** |
| Out-of-family preferential flow | Random | 3.4535 | 1.2234 |
| In-family DUAL_PERMEABILITY | Discrepancy-guarded EVSI | **0.5990** | **0.3775** |
| In-family DUAL_PERMEABILITY | Random | 0.6217 | 0.3974 |
| Explicit discrepancy state | Discrepancy-guarded EVSI | **0.9647** | **0.5250** |
| Explicit discrepancy state | Random | 1.3031 | 0.6823 |

The full artifact contains every policy, confidence interval and action-selection
frequency. For example, in the explicit discrepancy-state world the guarded
policy selected the predeclared CT and gas diagnostic modalities in 96.9% and
97.7% of replicates, respectively, and averaged 1.945 diagnostic actions.
It also selected the nuisance-signature action only 3.9% of the time. This is
consistent with the fixture's model-expansion design, but it is not evidence
that an arbitrary real discrepancy process can be inferred or diagnosed.

The common-heldout result changes the earlier interpretation. On the previous
two-world experiment, random had been the lowest-regret baseline. Under
common-random-number pairing and the expanded third-world control, the
discrepancy-guarded heuristic performs best on the designed stress worlds and
ties decision EVSI on the in-family world. No universal ranking is justified:
the synthetic prior, likelihoods, action budget and utility model determine the
problem. The 95% intervals are replicate-level normal approximations, not
paired differences or scenario-family uncertainty intervals.

**Key limitation:** the explicit discrepancy state is an author-specified
categorical hypothesis with a broad residual likelihood and a strong
"diagnose first" utility in this designed world. These results verify that
Bayesian model expansion can route an out-of-family outcome to a different
decision when that hypothesis is present and correctly useful. They do not
validate how the hypothesis/prior/likelihood should be learned for real soil.
Sensitivity to prior weight, residual likelihood width, diagnostic utility,
false positives, corrupted measurements and competing omitted mechanisms is
mandatory before drawing any practical conclusion.

Report artifact digest and source SHA are pinned above. Do not replace the
negative findings from earlier runs; retain all versioned outputs so the
benchmark history remains auditable.


## 27. Discrepancy prior / likelihood sensitivity

The discrepancy-state regression now includes a sensitivity control rather than
only testing the nominal 0.05 prior / 0.70 residual likelihood. With a residual
observation assigned probability 0.01 by the aggregate physical candidate
models, the posterior discrepancy weight is:

\[
w_D = \frac{p_D \ell_D}
{p_D\ell_D + (1-p_D)\,0.01}.
\]

For the test grid:
- prior 0.001, residual likelihood 0.30 gives posterior discrepancy mass
  0.0292 and the declared optimal decision remains RETAIN_WATER;
- prior 0.001, residual likelihood 0.90 gives posterior mass 0.0826 and
  changes the expected decision to DIAGNOSE_FIRST;
- prior 0.01, residual likelihood 0.30 gives posterior mass 0.2326;
- nominal fixture prior 0.05, likelihood 0.70 gives posterior mass 0.7865
  and DIAGNOSE_FIRST.

The regression requires posterior mass to increase monotonically with prior
weight and residual likelihood, and checks the policy-relevant decision
threshold at the stated settings. This exposes how a "diagnose first" decision
depends on prior and likelihood assumptions instead of hiding those choices
inside an opaque surprise score.

The test does not identify an empirically correct prior or likelihood. The
catch-all model can still soak up corrupted data, sensor faults, omitted
boundary conditions or true process discrepancy. Future qualification needs
prior sensitivity over a broad grid, model expansion penalties/complexity
control, false-alert costs, alternative residual shapes and calibration against
independent repeatability / instrument QA evidence.


## 28. Correlated continuous likelihood reference primitive

A test-only module
`research/terra-preta/tools/sequential-bayes-design/src/correlated_likelihood.rs`
implements the normalized bivariate Gaussian joint log density. Its five tests
check:
- exact reduction to the product of univariate densities at zero correlation;
- correct conditional-Gaussian factorization;
- positive / negative correlation effects on concordant and discordant residuals;
- rejection of singular or invalid covariance parameters;
- rejection of non-finite means and observations.

This is a mathematical reference, not a covariance-aware acquisition engine.
It does not estimate correlations, model multiple repeated observations, fit
random effects, or link support metadata to an empirical covariance matrix.
The tests are behind `cfg(test)` so the unused reference routine does not
enter the production selector. Integration with a support-aware joint
likelihood remains open under #5632.
