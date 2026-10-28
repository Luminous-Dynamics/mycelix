# CECB Robust Policy Tournament v0.1

**Status:** research protocol proposal; not an executed benchmark, simulator qualification, policy recommendation, or field-impact result.  
**Date:** 2026-10-29  
**Tracking:** [CECB-ROBUST-001 / issue #5510](https://github.com/Luminous-Dynamics/mycelix/issues/5510)  
**Scope:** synthetic housing-access experiments composed through the existing Civilization Twin.

## 1. Why another protocol?

The CECB v0.1 specification already requires fair paired comparisons, held-out scoring, non-compensable physical/rights/privacy guardrails, multiple outcome dimensions, and a separation between contract correctness and real-world validity. The result verifier in PR #5509 adds artifact-byte integrity.

Those controls answer important questions:

- Were result rows produced for the declared schedule?
- Were the paired arms exposed to the same declared exogenous world?
- Are the claimed artifact bytes the bytes that were actually read?
- Were outcome units, unknown values, and hard guardrails handled according to contract?

They do **not** answer this question:

> Would the conclusion survive if a plausible but consequential assumption about how the economy works were wrong?

Thousands of extra seeds within one structural model reduce Monte Carlo uncertainty, but cannot by themselves resolve structural uncertainty. If a policy appears best only because the simulator assumes unusually elastic housing supply, frictionless relocation, or muted default propagation, that dependence is the scientific result—not a footnote.

This protocol adds a **model-family-held-out policy tournament**. Its desired output is a conditional robustness map, not a universal ranking of institutions.

## 2. Scientific boundary and owning components

This protocol does not introduce a new simulator.

- **Civilization Twin:** owns state transitions, economic/world semantics, exogenous-event generation, and policy-counterfactual interpretation.
- **Existing accounting/production/allocation contracts:** own their respective invariants and policy mechanisms.
- **CECB runner:** constructs the pre-registered design, invokes only reviewed adapters, records each run's disposition, and packages outputs.
- **Independent evaluator:** owns held-out family assignment, hidden scenario truth, scoring, and any evaluator-only oracle.
- **CECB result verifier:** validates the result/artifact contract and byte digests. It does not establish model validity or policy superiority.
- **Stakeholders/domain reviewers:** define what outcomes matter, hard constraints, practical-effect thresholds, and acceptable risk. Software must not silently choose these normative weights.

No single combined welfare score may trade a rights, authority, privacy, or physical-feasibility failure against an average gain elsewhere.

## 3. Experimental units and identity

Every scored observation must be addressable by the full tuple:

`(protocol_version, model_family_id, model_variant_digest, scenario_id,
configuration_id, replicate_id, seed_schedule_digest, policy_profile_digest)`

The tuple must be sufficient to reconstruct which assumptions and policy profile generated a row. IDs are not substitutes for digests.

Define the nested hierarchy explicitly:

1. **Model family** — a qualitatively distinct, defensible account of a relevant mechanism.
2. **Model variant** — a registered parameterization or bounded structural variant within a family.
3. **Scenario/configuration** — a declared environment and stress condition.
4. **Replicate** — a stochastic realization under that model/scenario.
5. **Policy arm** — a frozen institutional or allocation mechanism.

Seeds nested inside the same model family do not count as independent structural replications. Report counts at every level. Avoid estimating cross-family generalization from a very large number of within-family replicates.

## 4. Model-family split: no tuning on the qualification family

### Development set

Policy authors may inspect results from designated development model families, public known-truth fixtures, and public stress cases. Development results can be used to fix implementation defects and tune parameters—but every change must be logged against a versioned protocol.

### Qualification set

Before confirmatory scoring, the independent evaluator must freeze and digest-bind at least one held-out structural family or a predeclared set of semantic structural mutations that policy authors have not used for tuning. The holdout assignment must be outside candidate control.

A single held-out family is an **initial holdout check**, not enough to justify a broad claim of robustness across all economic structures. A stronger generalization claim requires multiple meaningfully distinct held-out families and/or independent replications, chosen prospectively for the use case.

Do not count these as independent evidence:

- more seeds from a seen family;
- different parameter values that leave the disputed mechanism unchanged;
- renaming a scenario while retaining the same causal structure;
- random perturbations without a documented mechanism or plausible range.

If an evaluator-owned holdout becomes visible to a policy author, mark it compromised, freeze the disclosure, and create a new versioned holdout. Do not quietly re-label it unseen.

## 5. Structural stress families for synthetic housing

Start with semantic model alternatives, not noise sprinkled over one implementation. The following are candidate dimensions to review with the Civilization Twin owners; they are not claims that the current Twin supports them yet.

| Mechanism family | Plausible variants to expose | Failure question |
|---|---|---|
| Housing supply response | slow/fast construction, capacity caps, nonlinear marginal cost | Does a policy look successful because new capacity is unrealistically easy to create? |
| Construction and maintenance | correlated delays, scarce labor/material inputs, deterioration and repair backlogs | Does short-horizon access hide future stock degradation? |
| Household adaptation | relocation cost, search friction, household-size change, location preferences | Does a policy assume people adjust instantly or without cost? |
| Finance and loss-bearing | arrears, defaults, refinancing, reserve limits, correlated income shock | Are losses displaced to households, public budgets, lenders, or future maintenance? |
| Allocation behavior | price response, queue abandonment, strategic reports, tie-breaks, gaming | Does an access rule fail under plausible strategic response? |
| Information quality | stale inventory, delayed eligibility evidence, conflicting observations | Does the policy require information unavailable at decision time? |
| Governance and capture | eligibility capture, maintenance underinvestment, rule manipulation | Can an institution's rules be gamed or captured while headline averages improve? |
| Correlated shocks | income loss plus material disruption plus high demand | Does the policy survive compound stress rather than only one-factor tests? |

Each variant must declare: changed mechanism, source/rationale, affected state transitions, parameter or categorical range, expected directional hypotheses if any, and what would falsify the model. Keep purely adversarial impossible worlds separate from plausible stress worlds; report both and label them.

## 6. Pairing contract within each model/scenario

For policy arms intended as matched counterfactuals, hold constant:

- initial world-state bytes;
- exogenous event schedule and realized event-stream bytes;
- physical/resource/compute envelope, unless budget is itself the registered treatment;
- available observations, freshness and evidence budget;
- permitted action interface, horizon, and decision opportunities;
- accounting boundary, outcome definitions, and evaluator;
- tuning budget and policy execution budget.

Use event-keyed or otherwise event-aligned exogenous randomness when the Twin implements a reviewed contract. A shared seed is not sufficient if branch-dependent random consumption shifts subsequent exogenous draws. Policy-internal lottery, tie-break, or agent-choice randomness must be isolated from exogenous randomness.

A genuinely policy-induced event (for example, a relocation caused by an access rule) is an **endogenous outcome**, not an event to force identical across arms. The event contract must distinguish exogenous inputs from endogenous responses.

Run positive and negative metamorphic controls:

- extra policy-internal draws do not perturb later exogenous events;
- changing policy order does not change the event stream;
- identical model/scenario/replicate/event keys reproduce byte-identical exogenous values;
- an intentionally changed exogenous key changes the intended event value;
- intentionally mismatched initial state or event stream fails the paired verifier.

Record pairing failure as an invalid experiment, not as a low policy score.

## 7. Diagnostics: keep each outcome dimension separate

Let (x_{p,m,s,r,j}) be outcome dimension (j) for policy (p), model variant (m), scenario (s), and replicate (r). Each outcome's units, direction, aggregation, and missing-data policy come from the versioned CECB outcome contract.

### 7.1 Paired effects and uncertainty

For each policy-vs-baseline comparison, calculate paired within-block effects for every registered outcome. Report effect estimates, interval method, interval bounds, sample hierarchy, missingness, exclusions, and protocol departures. Keep scenario/model-family variation visible; do not pool all run rows as independent observations.

A hierarchical bootstrap or other inferential procedure must resample at the level(s) appropriate to the declared target of generalization. The protocol must specify and justify that procedure before confirmatory scoring. A bootstrap cannot create model-family diversity that the experiment did not include.

### 7.2 Tail and subgroup outcomes

Report full distributions and, where preregistered, tail measures such as the 90th/95th percentile or CVaR of unmet safe-housing need/person-days. Report every relevant group separately, including the worst group; do not substitute the overall mean for incidence analysis. State whether a tail is calculated over replicates, scenarios, groups, or a clearly defined nested unit.

### 7.3 Guardrail risk

For each non-compensable guardrail, publish its violation count, eligible denominator, unknown/missing count, and a conservative uncertainty bound using a preregistered method. Unknown evidence must not become zero violations. A hard violation makes that run ineligible regardless of average benefits.

When zero violations are observed, report the upper uncertainty bound rather than saying risk is zero. A numerical acceptable-risk threshold, if used, must be chosen by the relevant authority/stakeholders before scoring; it is not hard-coded by this generic protocol.

### 7.4 Outcome-specific regret

For each outcome dimension separately, orient the metric according to its registered direction. Let (q_{p,m,s,r,j}) be the direction-adjusted value for that one dimension. For a completed scenario, define the evaluator-only outcome-specific hindsight gap:

`regret_{p,j} = max_{a in feasible arms} q_{a,m,s,r,j} - q_{p,m,s,r,j}`

This is a diagnostic comparison to the best *observed feasible arm for that single outcome in that scenario*, not a deployable omniscient policy and not a single multi-outcome oracle. The best arm can differ by outcome. Report median, upper-tail, and worst observed per-outcome regret across held-out model/scenario units; publish all failures and missing values.

If no arm is feasible, report the scenario as no-feasible-policy / guardrail failure. Do not hide it by dropping the scenario from the denominator.

### 7.5 Pareto and ranking robustness

Report, across held-out model families:

- frequency of strict Pareto dominance by each policy over each comparator;
- changes in Pareto-front membership and policy ordering between development and held-out families;
- the policy-by-model-family matrix of guardrail, outcome, cost, and failure results;
- scenarios where each policy performs well and where it breaks.

A win rate is descriptive, not a policy mandate. If policy profiles trade off outcomes, there is no meaningful scalar regret unless stakeholders explicitly register a particular use-case utility with its authority and rationale. The default artifact remains a multidimensional outcome vector and a Pareto set.

## 8. Uncertainty decomposition

Keep at least these sources distinct:

1. **Aleatory variation:** randomness within a fixed model/scenario.
2. **Parameter uncertainty:** uncertain parameter values within the declared model family.
3. **Structural uncertainty:** alternative causal or behavioral mechanisms.
4. **Scenario-distribution uncertainty:** which future shocks, configurations, or populations are represented and with what weights.
5. **Measurement/scoring uncertainty:** incomplete, noisy, revised, or contested observations and definitions.
6. **Implementation uncertainty:** unresolved adapter semantics, numerical tolerances, bugs, or unknown provenance.

The output must state which sources were quantified, stress-tested qualitatively, or not assessed. Do not combine uncertainty sources into a single interval without stating the model and assumptions that justify the combination.

If no defensible probability distribution exists over structural alternatives, do not invent weights. Show a sensitivity/robustness map over the declared family set and label it as a set-based result.

## 9. Preregistration and stop conditions

Before opening qualification outputs, freeze:

- the target decision/question and evidence level;
- model-family taxonomy, variants, development/holdout split, and digests;
- candidate and baseline policy-profile identities;
- the primary estimand and minimum practically important effect;
- all non-compensable guardrails and their denominators;
- tail, subgroup, and Pareto diagnostics;
- inferential method, multiplicity handling, and missing-data rules;
- scenario and replicate schedule, compute/time budgets, and stopping rules;
- explicit failure/disposition taxonomy and report generator version.

Stop and label the comparison **invalid or inconclusive** if paired inputs drift unexpectedly, a held-out family leaks into tuning, the outcome contract changes after scores are seen, failure rows are selectively omitted, or required provenance cannot be verified. A result may be structurally valid but scientifically inconclusive.

## 10. Qualification ladder and benchmark targets

These are proposed engineering targets, not achieved results or universal claims about the economic effect size.

| Gate | Qualification target | What it permits—and does not permit |
|---|---|---|
| Q0 — contract/invariants | 100% of scheduled runs have an explicit disposition; all preregistered malformed or paired-invariance mutations are rejected; no known hard-guardrail false pass in the test corpus | Contract correctness only |
| Q1 — known-truth mechanisms | Small analytically checkable worlds match an independent oracle on exact invariants and directional cases; positive, negative, and adversarial controls are included | Mechanism behavior in the tested synthetic worlds |
| Q2 — within-family sensitivity | Parameters and stochastic replicates are varied; uncertainty sources and sensitivity method are documented | Repeatability and parameter sensitivity, not structural generalization |
| Q3 — held-out structure | At least one evaluator-controlled model-family holdout is untouched by tuning; multiple held-out families are required for a broader cross-family robustness claim | Evidence about specified model-family transfer only |
| Q4 — empirical model adequacy | Calibration targets and holdout observations are separated; compare distributional fit/forecast baselines where prediction is claimed; include historical retrodiction when suitable episodes exist | Context-limited empirical adequacy, not proof of any unobserved policy counterfactual |
| Q5 — external replication | Independent source/evaluator or governed prospective/shadow setting, with legal, privacy, stakeholder and safety review | The claim supported by that replication design only |

Additional minimum reporting targets:

- 100% accounting of planned model families, scenarios, runs, and failures;
- no known false-pass in the registered hard-guardrail and event-pairing negative-control suite;
- per-outcome effect, tail, subgroup, regret and Pareto diagnostics, with unknown values preserved;
- explicit distinction between public development cases and genuinely evaluator-controlled holdouts;
- compute, implementation, external funding, maintenance, loss-bearing, and displaced costs reported at the same boundary;
- every claim tagged with its highest completed qualification gate.

Do not declare a policy superior because the verifier returns PASS. PASS means the declared contract/artifacts passed the verifier; it is not a model-validation or causal-impact result.

## 11. State-of-the-art comparison

There is no single “state-of-the-art economy simulator.” Compare each relevant capability against the strongest fit-for-purpose method, including simpler baselines. For CECB housing, that can include a transparent rule-based allocator, a strong configured price/market baseline, a queue/lottery/need-priority baseline, a calibrated statistical or econometric reference where suitable, and any feasible optimizer using the same observations and resources.

A policy/AI method must not receive more tuning compute, scenario access, or hidden information than its comparator without that difference being declared as the treatment. Complexity is a cost: report latency, compute, implementation, maintenance and operational burden.

The scientific advancement claim should be a demonstrated improvement in **decision reliability under structural uncertainty**—for example, fewer hidden guardrail failures, lower tail unmet need, or lower per-outcome regret across held-out model families—without pretending that unlike outcomes can be collapsed into a universal score. Thresholds and trade-offs must be set by the affected stakeholders for the actual decision.

## 12. Limits and non-goals

This protocol does not guarantee that all important model structures have been enumerated. A held-out test measures robustness to the variants actually tested; it is not universal proof. Nor does structural robustness establish real-world validity: empirical data, domain knowledge, prospective testing, and independent replication remain separate evidence.

It does not authorize deployment, change entitlement/legal authority, create a new economic simulator, or claim that a capability-oriented profile beats market/cooperative/public/mutual-credit alternatives in general.

## References

- Buffalo, Pearson & Klein (2026), [Realizing Common Random Numbers: Event-Keyed Hashing for Causally Valid Stochastic Models](https://arxiv.org/abs/2603.11084). Supports separating exogenous event randomness from execution-order-dependent random draws.
- Jamalia & Lazarova-Molnar (2026), [Agent-Based modeling in financial markets: Modeling frameworks, validation challenges, and emerging applications](https://doi.org/10.3934/nhm.2026043). Distinguishes stylized-fact replication, out-of-sample prediction, and credible policy counterfactuals.
- Grimm et al. (2023), [How to keep it adequate: A protocol for ensuring validity in agent-based simulation](https://doi.org/10.1016/J.ENVSOFT.2022.105559). Treats validity as context-dependent and requires uncertainty and methods to remain coherent throughout the modelling process.
- NIST (2026), [TEVV-Athlon Framework for Evaluating AI Systems](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems). A draft framework for structured test, evaluation, verification and validation.
- Shavazipour & Stewart (2026), [A novel multi-stage multi-scenario multi-objective optimisation framework for adaptive robust decision-making under deep uncertainty](https://doi.org/10.1016/j.omega.2025.103405). A relevant adjacent approach to multi-objective decisions under deep uncertainty, not proof that one algorithm is universally best.
