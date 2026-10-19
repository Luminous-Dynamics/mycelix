# CIV-IMPACT-001 — Incremental Economic Value Benchmark

**Protocol version:** 0.1  
**Status:** proposed design; not yet executed  
**Tracking issue:** [Mycelix #5046](https://github.com/Luminous-Dynamics/mycelix/issues/5046)  
**Benchmark class:** cross-project, counterfactual, evidence-bounded decision support

> This document defines how to test whether composing Symthaea, Symtropy, Sol-Atlas, and Mycelix creates incremental economic value. It does not report benchmark results, certify any project, or claim that the integrated stack outperforms existing alternatives.

## 1. The paradigm shift

Most technology evaluations measure a component in isolation: task accuracy, simulation throughput, route cost, or transaction latency. Those measures are useful but do not answer the economic question.

The target is **outcome per fully loaded resource**, under a fair counterfactual:

`same world + same evidence availability + same authority + same horizon + same resource budget`

`baseline policy vs composed policy`

The benchmark must determine:
1. Whether service or production outcomes improve.
2. Whether the improvement is incremental to the strongest fair baseline and attributable to the composition.
3. Whether the benefit survives lifecycle cost, uncertainty, stress, distributional, and rights/guardrail analysis.
4. Where composition helps, harms, or has no material effect.

This is a falsifiable test. Null and negative results are valid results.

## 2. Reuse existing architecture

This protocol is a benchmark/economic measurement layer, not another implementation of existing responsibilities.

- [CIV-ECON-006: Civilization Twin](https://github.com/Luminous-Dynamics/mycelix/issues/4816) owns the coupled world, institutional counterfactuals, causal events, and outcome vector.
- [CIV-CORE-003: Independent evaluation and oracle separation](https://github.com/Luminous-Dynamics/mycelix/issues/4835) defines the evaluation/authority boundary.
- [LOG-COMMONS-S1](https://github.com/Luminous-Dynamics/mycelix/issues/5044) defines a non-authoritative Mycelix consumer for Symtropy transport proposals.
- [Symtropy transport benchmark portfolio](https://github.com/Luminous-Dynamics/symtropy/issues/1667) is the initial domain benchmark seam.
- [Sol-Atlas logistics network explorer](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/106) is the spatial/network evidence seam.

Do not duplicate the world simulator, route optimizer, identity system, reputation system, or event ledger. Consume versioned outputs through declared interfaces and retain their original evidence identities.

## 3. Ownership contract

| System | Role in this benchmark | Must not claim |
|---|---|---|
| Sol-Atlas | Spatial/network evidence; assets, links, capacity context, geography, provenance, coverage and freshness | That map coverage proves the real world is fully observed |
| Symthaea | Candidate generation, reasoning, policy proposals, uncertainty and abstention | That a model proposal is an observation or an authorized action |
| Symtropy | Declared simulation/optimization, feasibility and resource allocation under explicit constraints | That simulated feasibility proves operational success |
| Mycelix | Actor and authority references, coordination, provenance, disputes, attestations and result federation | That a ledger entry, vote, reputation, or attestation proves economic/physical truth |

Each subsystem's output remains typed as a proposal, model result, observation, authorization, or execution receipt. Adapters must not collapse those categories.

## 4. Initial vertical: transport and logistics allocation

Start with one bounded question, such as:

> Under matched demand, network, capacity, and operating constraints, does the composed decision pipeline reduce fully loaded cost per successfully served demand unit, or improve service at a predeclared acceptable cost, compared with the best fair incumbent baseline?

Use source-pinned, permission-cleared data. The first run may be synthetic or replay-only; it must be labeled as such. Do not introduce live dispatch, real reservations, payments, or physical side effects in the initial benchmark.

Other potential verticals—energy resilience, food networks, housing access and infrastructure investment—are follow-on profiles, not part of the initial acceptance claim.

## 5. Experimental design

### 5.1 Freeze the comparison before outcomes

The preregistration must freeze:
- the causal question and primary endpoint;
- scenario universe, inclusion/exclusion rules, source versions and usage rights;
- baseline implementations and versions;
- the complete policy and configuration for every arm;
- resource budgets and the accounting boundary;
- treatment/ablation matrix;
- held-out scenario identifiers and access restrictions;
- statistical procedure, practical-effect threshold, guardrails and claim ceiling;
- all planned metrics, subgroup slices and stopping rules.

Any change after outcome exposure requires a new protocol version and must preserve the earlier result as historical evidence.

### 5.2 Required experiment arms

At minimum:
1. **Incumbent baseline:** strongest feasible existing human, rule-based, or domain optimizer method, declared before scoring.
2. **Best fair partial-stack baseline:** the strongest simpler/partial configuration allowed by the same data and authority.
3. **Full stack:** all four systems in their declared roles.
4. **Leave-one-out ablations:** full stack with each of Symthaea, Symtropy, Sol-Atlas, and Mycelix removed in turn.
5. **Stress controls:** the same arms under missing/stale inputs, demand or capacity shocks, conflicting claims, coordinator outage and adversarially malformed evidence.

Add component-only arms when feasible and useful for attribution. Report all arms; do not report only the winner. Give the baseline the same access to permitted data, tuning budget, compute budget, and evaluation opportunities as the treatment. A larger model, more search, or more human review cannot be hidden inside the treatment.

### 5.3 Initial synthetic corpus targets

Proposed starting targets (engineering targets, not standards or achieved results):
- At least 100 preregistered scenario configurations.
- At least five named regimes, with at least 20 configurations per regime.
- 30 paired random seeds per scenario configuration for the initial simulation phase.
- Every scheduled arm/scenario/seed receives a recorded disposition, including invalid input, non-convergence, timeout, missing output, evaluator error, or success.
- Keep a held-out scenario family untouched during policy development and baseline selection.
- Include nominal conditions and stress cases; ensure that a single easy regime cannot dominate the aggregate.

These counts are an initial floor to exercise the protocol, not evidence that the corpus represents the global economy or is statistically powered for every possible effect. The confirmatory sample-size calculation must be updated from pilot variance without inspecting confirmatory outcomes.

## 6. Outcome model: value is a vector before it is a score

### 6.1 Required metric families

**Service and production**
- completed demand and completion rate;
- on-time service, delay and unmet demand;
- constraint violations and infeasible proposals;
- throughput and reliability under the declared operating envelope.

**Full economic accounting**
- fully loaded cost per successfully served unit;
- realized cost avoided and gross benefit;
- inference/compute, data acquisition/licensing, hosting and integration costs;
- human review, dispute/reconciliation, monitoring, failure and recovery costs;
- amortization assumptions and evaluation horizon;
- incremental net benefit under a declared, reviewable accounting rule.

**Resilience**
- worst-decile and tail outcomes;
- downside loss under registered shocks;
- recovery time, fallback success and failure propagation;
- degradation when individual systems or data sources are unavailable.

**Distribution and agency**
- outcomes by geography, organization size and affected population where legally and ethically permissible;
- who captures the benefit and who pays the cost;
- access failures, switching/exit costs, dependency and coordinator/verifier concentration;
- whether aggregate improvement conceals serious subgroup harm.

**Physical and ecological**
- use-case-specific energy, fuel, materials, emissions, land/water impacts or waste, only where data and measurement methods support the claim;
- show unpriced externalities separately rather than pretending they are zero.

**Coordination**
- conflicts, failed handoffs, expired/rejected plans, duplicate commitments and reconciliation time;
- time from valid proposal to authorized decision, separating time saved from decisions skipped.

**Evidence quality**
- input coverage, freshness, provenance integrity, missingness and source rights;
- forecast/simulation calibration where a valid observed reference exists;
- abstention behavior, error severity, verifier independence and stale-result rejection.

### 6.2 Economic accounting invariants

Use a declared measurement boundary and horizon. A useful primary metric for the transport profile is:

`fully loaded cost per completed unit = (operational + capital amortization + data + compute + integration + review + coordination + failure/recovery costs) / successfully served units`

This equation is incomplete until every numerator term has a stated inclusion rule, unit, observation source, and allocation method. Publish those rules.

The analyzer reports both a macro-average (mean of per-run ratios) and a pooled ratio (sum of the registered cost numerator divided by sum of successfully served units). They represent different estimands: equal weight per run versus equal weight per served unit. Select the primary estimand before confirmatory evaluation based on the intended population and decision; retain the alternative as a sensitivity view. A change in weighting must not be chosen after observing which version favors the treatment.

Distinguish **net new value** from **transfers**. A payment shifted from one participant to another is not, by itself, an increase in total system value. Show distributional transfers in separate fields. Report non-monetized outcomes and externalities even when they cannot defensibly be reduced to dollars.

Do not collapse the vector into a universal “flourishing score.” If a decision-specific utility function is necessary, publish its units, weights, provenance, sensitivity and stakeholder authority; register it before evaluation; and retain the full underlying metric vector.

## 7. Statistical decision rule

Select exactly one primary endpoint and a practical-effect threshold for each confirmatory profile before outcomes are exposed. Predeclare the direction, units, measurement horizon, missing-data treatment, statistical unit, confidence/credible interval method, clustering, multiplicity handling, and stopping rule.

For the initial protocol, use a paired or otherwise design-appropriate comparison and a 95% uncertainty interval as a proposed default. The interval must respect the scenario/problem sampling unit and relevant organization/site clusters. Repeated random seeds are not automatically independent real-world observations.

A proposed first economic threshold is **at least a 5% improvement in fully loaded cost per successfully served unit** versus the strongest fair baseline. A different primary endpoint (for example, service completion) may be selected if it better represents the decision; its practical threshold must be stated before scoring. The 5% value is a proposal to test, not a published standard or expected result.

A candidate can qualify for an incremental-benefit claim only if:
1. the preregistered effect meets the practical threshold;
2. the predeclared uncertainty interval supports improvement relative to zero/no-improvement;
3. all non-compensable guardrails satisfy their limits;
4. cost and failure accounting is complete enough for the stated claim;
5. the exact artifacts and complete run census pass independent verification;
6. the result survives the held-out test.

Otherwise report **no improvement**, **regression**, or **inconclusive** as appropriate. Do not redefine the endpoint, threshold, excluded cases, or scenario universe after seeing results.

## 8. Guardrails and failure accounting

Guardrails are constraints, not tradable points in a composite score. Each profile must state limits for the relevant safety, privacy, data-rights, accessibility, fairness, ecological and authority properties.

Adversarial cases must include:
- stale or tampered source identity;
- missing geographic areas or participants;
- a favorable average that hides materially worse tail outcomes;
- infeasible plans mislabeled as successful;
- model proposals promoted into observations or dispatch authority;
- duplicated observations counted as independent evidence;
- coordinator/verifier outage or collusion;
- post-hoc changes to scenario, metric, policy, budget, weights, or holdout;
- a full-stack run receiving more compute or review than its baseline;
- a changed implementation evaluated under a stale result receipt.

Report invalid and failed runs as part of the scheduled census. Do not replace or silently discard failures. State the upper confidence bound when no guardrail failures are observed; zero observations is not proof of zero risk.

## 9. Evidence ladder and claim ceiling

| Level | Minimum evidence | Permitted claim |
|---|---|---|
| E0 — Contract | Schema/invariant checks and negative controls | Contract behavior only |
| E1 — Synthetic | Reproducible effects-disabled corpus, declared toy oracle | Synthetic scenarios only |
| E2 — Source replay | Permitted frozen inputs and independent reconstruction | Replayed dataset result |
| E3 — Held-out simulation | Frozen held-out scenarios, fair budget and ablations | Modelled comparative result in the registered scope |
| E4 — Shadow | No real dispatch; outputs compared against observed operations | Shadow performance only |
| E5 — Prospective pilot | Predeclared live outcome measurement, proper consent and independent oversight | Pilot result for that population/horizon |
| E6 — Independent replication | Separate operator and/or independently sourced setting | Replicated result within stated scope |

A higher level does not follow from a lower one automatically. Passing CI, compiling, replaying a synthetic fixture, or receiving an attestation is not evidence of real economic benefit by itself.

## 10. Required result receipt

Every completed run must bind:
- benchmark/protocol version and frozen protocol identity;
- exact source, rights manifest, scenario corpus digest and holdout identity;
- all four implementation references and versions, plus enabled/disabled components;
- baseline and treatment configurations;
- solver/model/tool versions, seeds, resource budget and runtime environment;
- complete scheduled-run census and per-run disposition;
- raw/normalized artifact identities, metric definitions and uncertainty results;
- guardrail outcomes, missingness, exclusions, failure causes and negative controls;
- independent evaluator identity, declared independence/conflicts, verification result and claim ceiling.

The result receipt must become stale when a material subject, profile, source, evaluator, or protocol identity changes. A verifier must not trust cached summary metrics without validating the artifacts they summarize.

## 11. Execution sequence

1. **Contract first:** finalize metric and scenario contracts, source/rights manifest, and schema; add malformed-input and post-hoc-change negative fixtures.
2. **Baseline first:** replay the incumbent and simplest deterministic policy before adding learned search or full composition.
3. **Ablate:** isolate the marginal contribution of each component; preserve failures and equal budgets.
4. **Confirm:** score on the untouched scenario family with registered metrics/statistics.
5. **Shadow:** compare to permitted real operations without authorizing actions.
6. **Pilot and replicate:** only after operational, privacy, rights and human-oversight gates are met.

No need to wait for hosted runners to author the protocol, schema, source manifests, corpus inventories, independent verifier design, or synthetic baseline. However, do not label any check a pass until its exact source revision and evidence are available; queued, cancelled, skipped, or absent runs are not passes.

## 12. External methodological anchors

- [NIST TEVV-Athlon Framework for Evaluating AI Systems (2026)](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems) — adaptable testing, evaluation, verification and validation, including real-world outcomes.
- [NIST AI Risk Management Framework FAQs](https://www.nist.gov/itl/ai-risk-management-framework/ai-risk-management-framework-faqs) — validity, reliability, safety, security, accountability, transparency, privacy and fairness.
- [World Bank Group: Digital Public Infrastructure and Services](https://www.worldbank.org/ext/en/topic/digital-and-ai/digital-public-infrastructure-and-services) — interoperability, inclusion, governance, privacy and cybersecurity for shared infrastructure.
- [NBER, Shifting Work Patterns with Generative AI (2025)](https://www.nber.org/papers/w33795) — a 66-firm, 7,137-worker field experiment reported email time savings without detecting task quantity/composition changes from individual-level AI provision over the study horizon. Task-time gains alone therefore do not establish broad economic transformation.
- [NBER, Generative AI at Work (2023)](https://www.nber.org/papers/w31161) — reports heterogeneous productivity effects across worker groups, motivating distributional and subgroup analysis.

## 13. Executable schedule/census prototype

A first standalone Rust tool is being authored under [tools/civ-impact-census](https://github.com/Luminous-Dynamics/mycelix/tree/research/civ-impact-v0-1-review/tools/civ-impact-census). It is intentionally narrower than an optimizer or economic simulator.

- `generate` deterministically emits a balanced five-regime synthetic scenario set and paired comparison schedule (default: 100 scenarios × 30 seeds × seven arms = 21,000 planned runs).
- `verify` checks the exact schedule artifact SHA-256 embedded in the results manifest, complete run census, unique run IDs, explicit failure dispositions, completed-run demand bounds, and reconciliation across compute, operational, capital-amortization, data, integration, review, coordination and failure-recovery costs. Changing arms, implementation references, seed count, or any other schedule field invalidates result reuse even if the scenario-set digest remains unchanged.
- `CIV_IMPACT_RUN_CENSUS_V0_1.schema.json` provides the machine-readable structural contract for run results, including the required `schedule_artifact_sha256` link to exact schedule bytes.
- `CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json` defines the descriptive paired-comparison report shape, the stratified resampling interval shape, and the non-qualification claim flags.
- The tool's report states that benchmark validity and economic benefit are **not** established. A generated plan does not execute any arm; synthetic stress parameters are not operational data or estimated disruption probabilities.
- Generated arm implementation references are placeholders. They must be replaced with pinned implementations under a frozen preregistration before confirmatory use.
- Scenario hashing binds a deterministic serialization, and result receipts hash exact schedule/result bytes. Hashes alone are not authentication or immutable preregistration; anchoring and independent evaluator controls remain necessary.

Commands:
```sh
cargo run --manifest-path tools/civ-impact-census/Cargo.toml -- generate /tmp/civ-impact-run 100 30
cargo run --manifest-path tools/civ-impact-census/Cargo.toml -- verify /tmp/civ-impact-run/schedule.json /tmp/civ-impact-run/results.json
cargo run --manifest-path tools/civ-impact-census/Cargo.toml -- analyze /tmp/civ-impact-run/schedule.json /tmp/civ-impact-run/results.json
cargo test --manifest-path tools/civ-impact-census/Cargo.toml
```

If any registered incumbent/full-stack pair is incomplete, the analyzer withholds headline means and deltas rather than presenting a complete-case subset as if it represented the registered schedule. The verifier consumes a result file from a separately implemented evaluator. The paired analyzer reports descriptive cost-per-demand and cost-per-successfully-served-unit estimands plus service-rate changes. When the primary pair set is complete and nonzero-service, it also computes a reproducible, regime-stratified scenario-cluster percentile resampling interval (10,000 replicates). These are descriptive resampling intervals for the registered synthetic corpus, **not population confidence intervals or evidence of real-world generalization**. The analysis does not qualify causal savings or economic benefit.

## 14. Current status

- Protocol: authored proposal.
- Preregistration, run-census, and comparison-report schema files: authored and fetched back from the branch; JSON parsing confirmed. Rust unit tests are authored to validate each schema against its JSON Schema meta-schema, validate serialized run/report instances, and enforce schema/code field-set alignment. These tests remain unexecuted until the exact-head Rust workflow runs.
- Rust `1.99.1` toolchain and direct dependency versions (including the `jsonschema` validator) are pinned in the standalone crate; a generated and committed `Cargo.lock` remains outstanding for transitive dependency reproducibility.
- Rust schedule/census prototype and 30 unit tests: authored and fetched back from the branch; **not executed locally** because this environment does not expose `cargo` or `rustc`.
- Draft 2020-12 validation is implemented in the test harness via the pinned Rust `jsonschema` crate; schema meta-validation and instance-validation have not yet been executed in this environment.
- Benchmark runs: none claimed.
- Physical/economic outcomes: none claimed.
- Exact-head qualification is a separate gate. Query status at the exact current PR head; queued, pending, skipped, cancelled and absent checks are not passes.

The success criterion is not that the integrated stack wins. It is a trustworthy answer to **whether, where, for whom, at what cost, and under what conditions** integration helps.
