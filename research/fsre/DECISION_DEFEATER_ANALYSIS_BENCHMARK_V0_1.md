# FSRE decision defeater analysis benchmark v0.1

**Status:** Research protocol and E0 reference specification. No implementation, measured result, physical validation or SOTA claim is implied by this document.

**Parent research:** #6210  
**Composes with:** #6184, #6186, #6190, #6192, #6197, #6200, #6201, #6203, #6206 and #6207.

## 1. Purpose

A decision may be robust over its currently declared admissible worlds and still be fragile to a small, plausible, decision-relevant change in the evidence or model.

FSRE should therefore answer a question that ordinary bound reporting and optimisation do not answer by themselves:

> What is the least-cost, admissible, reproducible change to the declared model that defeats a specified decision property?

The output is a **decision defeater**: a concrete witness that a declared property fails under a specific admissible perturbation. Where the search is complete, the engine may also report a **robustness margin**: the minimum declared perturbation cost needed to produce such a witness.

This is not a probability of failure. It is conditional on the perturbation universe, mutation-cost model, evidence cutoffs and decision properties supplied to the analysis.

## 2. Formal problem

Let:

- M₀ be a frozen baseline decision model;
- P = {p₁, …, pₙ} be a finite set of typed, admissible perturbations;
- cᵢ ≥ 0 be the declared integer cost of perturbation pᵢ;
- S ⊆ P be a selected perturbation subset;
- M(S) be the model obtained by applying S to M₀;
- Q(M) be a caller-declared decision property.

A subset is a defeater when:

    Q(M(S)) = false

The E0 reference objective is:

    ρ_Q = min { Σ cᵢ : S ⊆ P, Q(M(S)) = false }

If no subset defeats the property, the correct result is **NO_DEFEATER_FOUND_IN_DECLARED_UNIVERSE**, not a claim of universal safety. If the search is incomplete, the engine must not claim a minimum.

The baseline must first be qualified. A blocked source-use or authority gate prevents the analysis from evaluating numeric decision content. Unknown eligibility is not a perturbation with a high cost; it is a hard precondition failure.

## 3. Decision properties

The caller must select and identify the property under test. The engine must not silently invent a scalar utility or infer which trade-offs are acceptable.

Initial E0 properties:

| Property | Defeater condition |
|---|---|
| ROBUST_SAFE_ACTION_RETAINED | A named baseline-safe action is no longer safe in every admitted world |
| NONEMPTY_SAFE_ACTION_SET | The mutated model has no robustly safe action |
| INVARIANT_UNIQUE_OPTIMUM | The baseline unique invariant optimum is no longer the unique invariant optimum |
| MINIMAX_REGRET_SET_UNCHANGED | The mutated minimax-regret action set differs from the baseline set |
| SERVICE_FLOOR_PRESERVED | A required service floor is violated in at least one admitted world |
| DENOMINATOR_INVARIANT | A declared population denominator or allocation invariant changes |
| ESTIMATOR_ADMISSIBILITY_PRESERVED | A required source-dependence condition invalidates estimator admissibility |
| AS_OF_REPLAY_INVARIANT | A replay result differs at a frozen recorded-time/effective-time cutoff |

A property can be non-scalar and non-compensatory. Rights, authority and safety remain hard qualification predicates. A weighted objective cannot compensate for their violation.

## 4. Typed perturbations

The initial vocabulary should include, as separately versioned mutation kinds:

1. **World-set:** add, remove or replace a declared admissible world.
2. **Evidence:** revise an evidence assertion, source snapshot, evidence validity interval or adjudication reference.
3. **Source dependency:** add or revise a declared common collection process, shared snapshot or upstream ancestry edge.
4. **Identity:** add, remove or revise a typed identity or relationship edge.
5. **Allocation:** revise a service-boundary, population or entitlement allocation under an explicit rule.
6. **Temporal:** add a late-arriving event, correction or effective-time change.
7. **Operating condition:** vary a physical parameter inside a declared domain.
8. **Assumption:** relax, strengthen or replace one explicitly versioned modelling assumption.
9. **Observation:** vary a measurement result under a declared observation and reliability model.

Every mutation needs a stable identifier, kind, subject, scope, evidence reference, effective/recorded times where relevant, admissibility basis, cost and cost unit.

### 4.1 Admissibility is not a cost

A perturbation that requires unauthorised source use, ungranted authority or an impermissible operation must be blocked before numerical search. Such a mutation cannot be made eligible by assigning a large cost. The report must distinguish:

- not admissible;
- admissibility unknown;
- admissible within the declared model.

A mutation's admissibility assertion is itself not authenticated merely because the parser accepts it.

### 4.2 Cost semantics

Mutation costs are analyst- or governance-declared units, not universal physical probabilities. A cost model might represent inspection effort, implementation effort, elapsed time, evidence acquisition cost or a pre-agreed lexicographic burden.

Do not add incomparable units into one sum. Use either a single documented unit or a lexicographic cost vector with an explicit ordering. Preserve the original components in output. Report sensitivity to alternative defensible cost models.

## 5. E0 exhaustive reference oracle

The first executable implementation should be dependency-free and small-instance oriented.

### Inputs

A frozen episode manifest must identify:

- baseline input snapshot and decision cutoff;
- baseline decision result and property under test;
- admissible perturbation IDs and typed mutation payloads;
- source-use, authority and scope qualifications;
- mutation cost model and unit;
- maximum search size / resource budget;
- rule, parser and reference-engine versions.

The finite perturbation universe must be explicitly declared. Missing or unparsed mutation classes cannot be treated as searched.

### Search

- Enumerate every subset of the declared finite perturbation set within the supported E0 limit.
- Apply mutations using deterministic, typed rules.
- Re-evaluate the selected decision property against the same reference semantics.
- Order candidate subsets deterministically by total cost and stable mutation IDs.
- Record every qualifying minimum-cost witness when multiple minima exist.
- Preserve ties; do not invent a single preferred explanation.
- If any required evaluation is invalid, unsupported or incomplete, retain that status rather than silently skipping the subset.

The E0 oracle is intentionally exponential. Its purpose is correctness on small cases, not scale.

### Witness replay

Every returned witness must contain enough data to reconstruct the exact mutated model and decision result:

- baseline manifest and input snapshot references;
- property identifier and version;
- selected mutation IDs and canonical payloads;
- per-mutation and total declared cost;
- baseline decision signature;
- mutated decision signature;
- exact failed invariant(s);
- input, rule and engine versions;
- search-completeness and minimality status.

An independent replay must reproduce the witness and decision delta. A witness that cannot be replayed is not a valid defeater.

## 6. Output status and minimality semantics

Use typed terminal states. Suggested v0.1 values:

| Status | Meaning |
|---|---|
| DEFEATER_FOUND | At least one valid witness was found |
| NO_DEFEATER_FOUND_IN_DECLARED_UNIVERSE | Complete search found no witness in the declared universe |
| BLOCKED_SOURCE_USE | Source-use gate failed before numerical evaluation |
| BLOCKED_AUTHORITY | Authority gate failed before numerical evaluation |
| NOT_IDENTIFIED_PERTURBATION_UNIVERSE | The perturbation universe or required preconditions are incomplete |
| INVALID_INPUT | Input is malformed or violates the schema |
| SEARCH_LIMIT_EXCEEDED | Search did not complete within declared resource limits |
| UNSUPPORTED_MUTATION | At least one required mutation type is not implemented |

Minimality must be reported separately:

- PROVEN_MINIMAL: complete exhaustive search checked every lower-cost candidate and all equal-cost candidates required by the specification;
- BOUNDED_MINIMAL: a valid lower bound and a matching witness establish the declared optimum;
- WITNESS_ONLY: a valid defeater exists, but minimality was not established;
- NOT_ESTABLISHED: no defensible minimality statement is available.

Never map a timeout, unsupported mutation or incomplete search to NO_DEFEATER_FOUND.

Likewise, NO_DEFEATER_FOUND_IN_DECLARED_UNIVERSE must never be rendered as PROVEN_ROBUST_AGAINST_ALL_REALITY.

## 7. Counterexample-guided refinement

A later scalable backend may use LP/MILP, branch-and-bound, symbolic search or counterexample-guided abstraction refinement.

It must preserve three separate objects:

1. **Abstract candidate:** a possible defeater in a reduced or abstract state space.
2. **Concrete witness:** a mutation sequence that is valid in the original typed model.
3. **Replay result:** independent re-evaluation of the original decision property after applying the concrete witness.

An abstract candidate that cannot be concretised is spurious. It may drive refinement, but it must not be reported as a real defeater. If concretisation is unknown, the result is indeterminate.

A solver's optimality or dual certificate applies only to the encoded optimisation problem. It does not authenticate the evidence, admissibility rules, mutation costs or completeness of the perturbation universe.

## 8. Composition with evidence acquisition (#6206)

A concrete defeater should expose the predicates and evidence dependencies that make a decision fragile. Measurement acquisition can then evaluate which lawful observation would distinguish the baseline from the defeater.

The composition is:

    Decision property
    → Defeater witness
    → Unresolved predicate
    → Eligible measurement
    → Updated admissible worlds
    → Requalified decision

An observation that cannot distinguish the baseline from a decision-changing witness has no value for that specific property, even if it reduces generic entropy. A measurement plan cannot authorize an intervention before all applicable gates pass.

## 9. E0 adversarial corpus

At minimum, include the following interaction families with positive and negative controls:

- **Identity × denominator:** one disputed asset-to-boundary relationship changes service population allocation.
- **Temporal × identity:** late-arriving corrections alter historical replay only after their recorded-time cutoff.
- **Source dependency × coverage:** two apparently separate frames acquire a common upstream collection process.
- **Common cause × safety:** one pump/power dependency invalidates apparently independent service capacity.
- **Assumption × safety floor:** a small accepted threshold or degradation-model change removes all safe actions.
- **World set × ranking:** one admissible world changes the invariant optimum or minimax-regret set.
- **Measurement × correlation:** two inspections share a failure mode and do not provide independent evidence.
- **Evidence rights × search:** blocked source material must stop analysis before numerical rows are evaluated.
- **Authority × action:** a mathematically attractive action is not eligible under the granted authority.
- **Cost model × minimality:** equal-cost ties and different valid cost units change witness ordering without changing the underlying defeater set.

Include fixtures where a perturbation does not change the decision. Avoid a corpus in which every mutation is adversarially effective.

## 10. Metamorphic and correctness tests

- Input row permutation preserves the canonical decision signature and witness set.
- Stable identifiers yield deterministic tie ordering.
- A complete exhaustive search agrees with independently enumerated known answers.
- Every reported witness replays against the unabstracted reference model.
- A proven minimum equals the exhaustive minimum on E0.
- Multiplying all costs by a positive integer factor scales the scalar robustness margin by that factor.
- Adding a new admissible perturbation cannot increase the exact minimum defeater cost, provided the previous universe and cost semantics are unchanged.
- Introducing a blocked or unknown rights/authority gate does not produce a numeric margin.
- Incomplete or unsupported mutation classes never become negative evidence.
- Cost ties return all qualifying minimum witnesses or an explicit, documented cap/indeterminate status.

## 11. Benchmark metrics

Report each metric separately:

| Dimension | Measure |
|---|---|
| Defeater detection | Recall against exhaustive E0 oracle |
| Witness validity | Fraction independently replayed successfully |
| Minimality | Exact minimum agreement; false-minimality count |
| Search completeness | Completed subsets / required subsets |
| Robustness | Declared cost margin and cost-model sensitivity |
| Decision impact | Exact property delta and affected action set |
| Safety | False-safe and hard-gate bypass counts |
| Abstraction quality | Spurious candidate rate and refinement count |
| Efficiency | p50/p95 runtime, peak memory, evaluated subsets, mutations/second |
| Reproducibility | Exact replay and deterministic output across repeated executions |
| Operational utility | Fraction of witnesses resolved by an eligible measurement; cost/time per resolved decision-critical predicate |

Never collapse hard-gate failures, false-safe decisions, witness invalidity and runtime into one weighted score.

## 12. Matched baselines

Compare on identical frozen models, properties, mutation universes, cost models, hardware and resource limits:

- one-at-a-time and local sensitivity analysis;
- global sensitivity methods where a justified probability model exists;
- conventional robust optimisation / scenario stress tests;
- minimal counterfactual or algorithmic-recourse methods where their action semantics fit;
- bounded model checking and counterexample-guided refinement;
- FSRE E0 enumeration and later scalable FSRE backends.

The relevant advantage is not merely finding an adverse scenario. It is producing a **typed, admissible, decision-changing, independently replayable witness** and establishing the minimum declared perturbation cost where the search supports it.

## 13. E1 and E2 evaluation

### E1 — validated physical models

Use calibrated water, energy and transport models with disclosed boundary conditions, parameter uncertainty, calibration data and known model-form limitations. Report whether the witness maps to a physically realisable scenario; do not equate a solver-feasible vector with a field-possible condition.

### E2 — authorised operational evidence

Use rights-authorised snapshots, independently adjudicated identity and dependency evidence, field inspection records and realised interventions. Record whether the identified defeater occurred, was observed, was ruled out or remains unresolved. Do not treat a synthetic witness as an incident report.

## 14. Proposed acceptance targets

These are research targets, not achieved results:

- 100% agreement with exhaustive enumeration across the frozen E0 correctness corpus.
- 100% independent replay of returned concrete witnesses.
- 0 spurious abstract candidates reported as concrete defeaters.
- 0 hard-gate bypasses.
- 0 false claims of minimality.
- 100% correct distinction between complete negative search and incomplete search.
- 100% deterministic output under permitted input permutations.
- Explicit indeterminate outputs for timeout, unsupported mutation or incomplete model states.
- Report sensitivity to at least three alternative defensible mutation-cost models before claiming a stable robustness ranking.

A finite zero-failure corpus is evidence about that corpus, not a universal guarantee. Publish corpus size, scenario families, held-out splits, confidence intervals where sampling is used and all unresolved cases.

## 15. Research references

- Rossetto & Antonucci (2026), [Partial Identification under Causal Orders by Linear Programming](https://proceedings.mlr.press/v343/rossetto26a.html).
- Choe et al. (2026), [Canonical Domain Reduction for Partial Counterfactual Identification](https://proceedings.mlr.press/v337/choe26a.html).
- Zaffalon et al. (2024), [Efficient computation of counterfactual bounds](https://doi.org/10.1016/j.ijar.2023.109111).
- Huang et al. (2026), [Formal Verification of Digital Twins with the Temporal Logic of Actions](https://doi.org/10.1016/j.jcp.2026.115196).
- Li & Parlikad (2026), [Value-Based Active Sensing for Improving Sampling Efficiency](https://doi.org/10.1109/TSMC.2026.3745855).

## 16. Implementation sequence

1. Freeze the typed E0 perturbation and decision-property schemas.
2. Implement a dependency-free exhaustive reference engine for a bounded number of mutations.
3. Add an adversarial corpus with interacting identity, dependency, temporal, denominator and safety perturbations.
4. Produce canonical witness and replay receipts.
5. Cross-check every scalable backend against the E0 oracle on overlapping cases.
6. Connect defeater witnesses to #6206's measurement candidates.
7. Add the component and its ablations to #6207.

**Claim boundary:** v0.1 is a protocol. It does not claim an implementation, benchmark pass, infrastructure-safety result, physical incident, universal robustness or superiority to existing tools.
