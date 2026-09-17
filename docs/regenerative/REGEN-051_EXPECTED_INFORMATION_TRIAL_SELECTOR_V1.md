# REGEN-051 — Expected-Information Trial Selector v1

Status: preregistration only. This contract defines recommendation-only information-value analysis over already-bounded trial candidates. It creates no intervention authority and no obligation to run a trial.

## 1. Purpose

Once REGEN-050 has generated candidates inside an already-qualified design space, Symthaea/Mycelix may estimate which candidates would most reduce declared uncertainty. The selector must not turn mathematical information value into a hidden utility, ethics, safety, or authority function.

Core theorem:

```text
eligible bounded candidate
+ exact prior/model state
+ exact endpoint/observation model
+ explicit uncertainty target
+ deterministic information metric
= expected-information estimate
```

not:

```text
highest expected information
= best treatment
= safest treatment
= most beneficial treatment
= trial that should be run
= authorized intervention
```

## 2. Upstream gate

REGEN-051 consumes only candidates that already satisfy REGEN-050's bounded-candidate theorem.

An `IneligibleCandidate` or `UnresolvedCandidate` cannot become selectable merely because a model predicts high information gain.

```text
hard eligibility precedes information value
```

## 3. Exact selector capsule

One selector invocation should bind at least:

- exact candidate-set identity;
- exact evidence snapshot;
- exact prior/model identity and revision;
- exact uncertainty target/question;
- exact endpoint set;
- exact outcome/measurement model;
- exact observation-noise assumptions;
- exact missingness assumptions where modeled;
- exact information metric/revision;
- exact deterministic seed for stochastic approximation;
- exact candidate costs/burdens only when separately represented;
- exact selector policy revision.

A score without this capsule is not replayable evidence.

## 4. Information target must be explicit

The selector must name what uncertainty it is intended to reduce, for example one exact parameter, model discrimination question, response surface region, or bounded hypothesis family.

```text
information about X
!= information about everything
```

High information about a narrow surrogate endpoint cannot silently become high information about agronomic benefit, nutrition, resilience, ecology, or long-term outcome.

## 5. No universal information score

The core should not expose an unqualified field such as:

```text
information_score = 0.91
```

without metric identity, target, units/interpretation where applicable, model/prior identity, and uncertainty assumptions.

Different information metrics may be incomparable.

## 6. Candidate selection states

At minimum the selector should preserve:

```text
Evaluable { estimate }
NotEvaluable { reasons }
DominatedForInformation { evidence }
InformationEquivalent { set }
```

A later recommendation layer may choose among evaluable candidates, but REGEN-051 itself does not manufacture authorization.

## 7. Expected information != expected benefit

The following remain separate dimensions:

- expected information gain;
- predicted treatment benefit;
- expected harm/burden;
- ecological load;
- cost/resource use;
- resilience effect;
- carbon effect;
- implementation difficulty;
- time to result.

The selector must not hide a weighted aggregate of them under an information label.

## 8. Null/control preservation

Control/null candidates remain valid members of the information analysis when required by the trial design.

If the control is required to identify the effect, it cannot be dropped merely because an intervention arm has a higher standalone model-entropy reduction.

## 9. Preposterior discipline

Expected-information calculations are pre-outcome estimates.

The exact simulated/predictive outcome distribution used to compute expected information must be bound to the selector capsule.

Observed trial results may update later model state, but must not retroactively rewrite the preregistered expected-information estimate.

## 10. No outcome peeking

Candidate selection for a frozen trial round must not inspect outcomes from that same round before the selection is finalized.

Adaptive/sequential trials require their own explicit update and stopping policy.

```text
adaptive
!= post-hoc
```

## 11. Adaptive-round lineage

For sequential experiment design, every round must bind:

- parent selector state;
- evidence added since parent;
- model/prior update identity;
- candidate pool revision;
- information metric revision;
- stopping/continuation decision.

No round may silently rewrite earlier candidates or priors.

## 12. Model misspecification remains visible

Expected information is conditional on the model family used to calculate it.

```text
high EIG under model M
!= high real-world information if M is wrong
```

Sensitivity across plausible model families/priors may be reported, but disagreement must remain visible rather than averaged away into false certainty.

## 13. Unknown is not zero information

A candidate that cannot be evaluated because the outcome model is missing or uncertainty is poorly specified is `NotEvaluable`, not `zero information`.

Likewise, failure to estimate burden/cost does not mean burden/cost is zero.

## 14. Measurement feasibility

Information value depends on whether the declared outcome can actually be measured at the required fidelity and time.

The selector should distinguish:

```text
latent information potential
!= practically observable information
```

Measurement availability, specimen/sampling constraints, detection limits, missingness, and endpoint timing remain explicit dependencies.

## 15. Correlated/redundant candidates

Multiple candidates may provide largely redundant information.

The selector may expose conditional/marginal information relative to a declared selected set, but must bind the exact set/order semantics.

A collection of individually high-information candidates is not automatically jointly high-information.

## 16. Common-mode data risk

Candidates relying on the same sensor, laboratory, sampling frame, weather event, site, model assumption, or operator may share an information failure domain.

The selector should expose these shared dependencies rather than treating observations as independent by default.

## 17. Missingness and censoring

Expected-information estimates that depend on assumed observation completeness must declare that assumption.

Dropout, censoring, non-detects, unavailable specimens, failed sensors, and protocol deviations cannot silently disappear from the information model.

## 18. Ethical / hard-gate firewall

No expected-information estimate may override:

- safety constraints;
- ecological hard constraints;
- consent;
- rights/authority;
- adopted trial ethics/governance requirements;
- contamination eligibility;
- material/site suitability;
- resource limits.

A highly informative prohibited experiment remains prohibited.

## 19. Resource/burden boundary

If a later selector chooses among candidates under budget or participant/community burden, that is a constrained multi-dimensional decision theorem, not pure expected information.

REGEN-051 should therefore preserve resource/burden metadata without silently pricing it into the information metric.

## 20. Stopping rules

The system must permit a valid recommendation to stop generating/running experiments when, for the declared question:

- no eligible candidate remains;
- expected information is below an adopted threshold;
- measurement feasibility is unresolved;
- evidence is sufficient for the declared decision boundary;
- hard constraints prevent further experimentation.

Stopping is not failure.

## 21. Negative/null evidence

Trials with null, adverse, or unexpectedly noisy outcomes can still be informative.

REGEN-051 must not define information value in a way that rewards only positive treatment effects.

## 22. Deterministic replay

For deterministic estimators, identical canonical inputs must produce identical estimates/order/equivalence sets.

For Monte Carlo or other stochastic estimators, exact algorithm, seed, sample count, convergence criteria, and implementation revision must be captured.

Approximation error must remain distinguishable from scientific uncertainty.

## 23. Comparison boundary

If two candidates' information estimates overlap within declared numerical/approximation uncertainty, the system may report them as unresolved/equivalent rather than manufacturing an ordering.

Tie-breaking rules, if needed for deterministic presentation, must not be misrepresented as scientific superiority.

## 24. Information leakage tests

Qualification should include adversarial fixtures where:

- a prohibited candidate has artificially huge information value;
- a candidate has high information but unacceptable burden metadata;
- a model assigns high information to an unmeasurable endpoint;
- duplicated/correlated candidates appear independently valuable;
- unknown model state is incorrectly mapped to zero;
- a later outcome is leaked into an earlier selector round;
- a changed prior/model fails to change selector identity;
- null/adverse outcomes are incorrectly treated as uninformative.

## 25. Initial executable direction

A first kernel should be pure and dependency-light, owning only:

- selector input identity/capsule;
- explicit information target;
- candidate-evaluability states;
- deterministic reference metrics/baselines;
- replayable stochastic-estimator metadata if needed;
- equivalence/unresolved handling;
- no-authority result types.

Model training, HDC inference, LLMs, Holochain, databases, network clients, schedulers, grant/funding systems, and physical execution remain adapters or separate domains.

## 26. Deliberate non-claims

REGEN-051 establishes no optimal treatment, causal effect, ethical acceptability, human/land consent, ecological safety, agronomic efficacy, financial value, carbon value, trial authorization, scheduling authority, process execution, or physical actuation.

Its proposition is deliberately narrow: estimate information value over already-eligible bounded candidates while preserving the difference between learning what would be useful and deciding what may actually be done.
