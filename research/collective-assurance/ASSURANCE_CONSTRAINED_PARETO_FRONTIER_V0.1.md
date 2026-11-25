# Assurance-Constrained Pareto Frontier v0.1

**Status:** reference comparator and experiment protocol. No candidate performance or Pareto result is claimed.

## Why a Pareto frontier?

A single aggregate score lets a candidate trade a severe safety or rights regression for more task accuracy, lower cost or faster completion. Weighted sums also hide stakeholder preferences and make cross-domain comparisons falsely commensurable.

This protocol uses two stages:
1. **Non-compensatory eligibility:** every required authority, privacy, evidence, execution, evaluator and rights gate must pass. Failure blocks performance comparison; unknown evidence remains indeterminate.
2. **Confidence-aware Pareto comparison:** compare matched outcome vectors without a scalar aggregate. Report candidate dominance, baseline dominance or incomparability.

The field gap is material. A 2026 review of 15 agent benchmarks reported that none included safety/security or cost efficiency in primary scoring, and 13/15 relied on binary success alone. A separate 2026 survey of 17 benchmarks reported no complete standardized run-cost record and no evidence that contamination, nondeterminism and cost were jointly controlled. These conclusions apply to the selected samples of those reviews, not every benchmark in existence. [Akhunzada et al., 2026](https://doi.org/10.1007/s10462-026-11571-0); [LLM agent evaluation survey, 2026](https://link.springer.com/article/10.1007/s10462-026-11678-4).

## The paradigm shift

Move from:

`one benchmark score → rank → winner`

to:

`qualified benchmark → matched run manifests → mandatory safety/rights gates → outcome vector with uncertainty → Pareto frontier`

A candidate that is more accurate but violates authority is not "a little better with a safety cost." It is ineligible. A candidate that is faster but more expensive in human review is not automatically efficient. A candidate that is safer only because it refuses every authorized task must show the utility/over-refusal trade-off.

## Contract

### Mandatory gates

The caller freezes the set of mandatory gate IDs before the tournament. Each run must supply exactly one result and evidence digest for every gate. Missing or unknown evidence is `INDETERMINATE`; a failed gate is `FAIL`.

Suggested common gates:
- authority and exact-effect authorization;
- privacy, information-rights and disclosure scope;
- evidence provenance and integrity;
- AI-off execution qualification when the claim depends on withdrawal;
- benchmark, oracle, scorer and evaluator integrity;
- accessibility and consent for human/team studies.

The reference comparator does not decide which gate policy is appropriate. It verifies the preregistered gate set and its evidence references. The responsible policy/authority owners determine gate definitions and evidence authentication.

### Outcome vector

The module supports:
- task quality and authorized task completion;
- evidence coverage and unique-information preservation;
- agency, calibration and burden;
- unauthorized action, unauthorized disclosure, false AI-off pass and over-refusal rates;
- p95 latency, total cost, energy, human-review time, input/output tokens and tool calls.

Each metric carries lower, point and upper estimates, a confidence level, a simultaneous-interval declaration, one shared preregistered inference-plan digest, independent-unit count, observation count, a paired-unit cohort digest and measurement-evidence digest. Baseline and candidate must use the same independent-unit cohort and inference plan for each metric. Non-simultaneous marginal intervals cannot support a vector-wide dominance claim. Normalized rates must be within [0,1]; resource metrics must be non-negative. Clustered team work must count teams as independent units, not messages or repeated questions.

### Conservative dominance

The reference implementation compares confidence bounds at the preregistered confidence level:
- Higher-is-better: candidate is non-worse only if candidate lower bound ≥ baseline upper bound.
- Lower-is-better: candidate is non-worse only if candidate upper bound ≤ baseline lower bound.
- Candidate dominates only if it is non-worse on every required metric and strictly better on at least one.
- Baseline dominance is symmetric.
- Otherwise the runs are `Incomparable`.

This is deliberately conservative. Overlapping intervals do not establish non-inferiority. The comparator consumes supplied intervals; it does not calculate them. Their statistical method, clustering, multiplicity and calibration must be qualified separately.

The metric set and direction are code-defined. A candidate cannot relabel a safety rate as higher-is-better to manufacture dominance. There is no hidden weighted score.

## Cost and resource accounting

For each run, report every available field and preserve unavailable values as missing/unknown, never zero:
- model/provider and tool/API charges;
- input/output tokens and cache read/write tokens;
- tool calls and external-service calls;
- wall-clock time, active CPU/GPU time, and peak memory;
- storage/network I/O;
- energy and carbon only when metered with a stated boundary and method;
- human setup, supervision, review and adjudication time;
- failures, retries, timeouts, restarts and discarded runs.

Currency is recorded in integer micro-units with an explicit currency code and pricing-source digest. Do not compare cost across currencies or price dates without a documented conversion policy. Report both total cost per assigned task and cost per successful authorized task; failures and abstentions remain in the denominator and the ledger.

## Statistical and reproducibility rules

- Baseline and candidate must pass the existing `MatchedSystem` manifest validator.
- Pin benchmark, version, split, scorer, model/tokenizer, prompts, tools, decoding, context/output/token/time/compute/memory budget, agent count and seed policy.
- Require preregistered metric set, directions, confidence level and independent-unit minimum.
- Use paired evaluation and common random numbers where valid. The confidence bands must provide simultaneous family-wise coverage over the preregistered metric vector; ordinary per-metric 95% intervals are insufficient for a 95% vector-wide dominance claim.
- Cluster inference at the randomization unit (team, environment or task family); do not treat messages/questions as independent where they are nested.
- Report p50/p95 latency and cost distributions, tail failures, worst-decile group results, missingness, exclusions, over-refusal and abstentions.
- Report Pareto relations across the full required vector; never suppress a regression because the mean aggregate score improved.

## Research targets

Prospective targets, not achieved results:
- 100% of candidate comparisons require every mandatory gate to pass before Pareto analysis.
- 0 missing measurements silently encoded as zero.
- 100% of reported resource metrics carry a measurement receipt and unit/boundary.
- 100% of normalized outcomes carry confidence bounds and independent-unit counts.
- no scalar aggregate is published as the sole comparative outcome.
- 0 false candidate-dominance claims in a synthetic matrix of at least 10,000 generated metric vectors with an independent oracle.
- exact parity required for benchmark, split, evaluator, scorer, model, prompt, tools, budgets and seeds in the matched track.
- every candidate publishes task-quality, unauthorized-action/disclosure, over-refusal, cost, latency, energy (if measured), human review and agency outcomes where applicable.

The 10,000-case target is a reference-model property-test goal, not evidence of real-world zero risk.

## Industry comparison strategy

Reproduce official systems and harnesses first:
- GroupAssistBench for open-ended assistance (L1 recall and L2 assistance, including M1/M2/M3);
- GroupMemBench for multi-party memory and knowledge update;
- COLLABMEM for visibility, routing, provenance, conflict and retention;
- HiddenBench for distributed information discovery;
- AgentDojo, AgentDyn and MASpi for agent security;
- ControlArena for control-protocol evaluation.

Then publish two separate frontiers:
1. **Equal-envelope frontier:** matched model, task, scorer, token/context/compute/time/memory budgets, tools and seeds.
2. **Native-envelope frontier:** each architecture's natural operating point, with quality/cost/safety Pareto curves.

Cross-model comparisons remain contextual. The best industry baseline is not beaten by claiming a larger headline score; it is beaten by a reproducible matched experiment, better confidence-bounded outcomes and no violation of mandatory gates.

## Integration and ownership

- #6619 owns matched run manifests and the evidence ledger.
- #6655/#6657 own the AI-off execution protocol and reference witness receipts.
- #6656 owns GroupAssistBench parity.
- #6676 owns the six-stage human/team capability outcome panel.
- #6677 owns source-origin counterfactual replay.
- #6678 owns benchmark definition/evaluator qualification.
- Symthaea SAT #13574 owns wider system-level sovereignty and adaptive-control qualification.

This module composes with those contracts. It does not create a new authority kernel, privacy engine, benchmark runner or independent evidence service.

## Claim ceiling

A `CandidateDominates` result means only that, under the supplied matched manifests, passing mandatory gates and confidence intervals, the candidate is conservatively no worse on all required metrics and strictly better on at least one. It does not prove universal superiority, causality, deployment safety or correctness of the supplied confidence intervals and evidence receipts.
