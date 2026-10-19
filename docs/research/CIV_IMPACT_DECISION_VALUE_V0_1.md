# CIV-IMPACT Decision Value v0.1 — Evidence, Abstention, and Regret

**Status:** proposed benchmark protocol; not executed  
**Tracking:** [CIV-IMPACT-002](https://github.com/Luminous-Dynamics/mycelix/issues/5053)  
**Parent benchmark:** [CIV-IMPACT-001](https://github.com/Luminous-Dynamics/mycelix/issues/5046)  
**Scope:** synthetic, effects-disabled policy evaluation

> This protocol evaluates decision policies under incomplete evidence. It does not establish real-world performance, authorize evidence collection, dispatch resources, or turn proposals into commitments.

## 1. The paradigm shift

A conventional planner benchmark asks: *Did the selected plan work, and at what cost?* A composed reasoning system must also answer: *Did it know enough to act, did it know when it did not know, and was additional evidence worth its price and risk?*

The treatment is therefore the entire **epistemic decision policy**, not just the final action:

`observe → estimate uncertainty → (act | request evidence | abstain/fallback | wait/replan) → evaluate outcome`

Every branch has a cost and consequence. A system must not win by answering only easy cases, abstaining without penalty, querying unlimited sources, counting mirrors as independent evidence, or seeing the evaluator's hidden truth.

This complements rather than replaces:
- [Symthaea's value-of-information evidence planner (#5596)](https://github.com/Luminous-Dynamics/symthaea/issues/5596);
- [Symthaea's counterfactual planning and regret qualification (#2574)](https://github.com/Luminous-Dynamics/symthaea/issues/2574);
- [Symthaea's epistemic-action qualification (#2579)](https://github.com/Luminous-Dynamics/symthaea/issues/2579);
- [CIV-IMPACT's cost/service census](https://github.com/Luminous-Dynamics/mycelix/pull/5050);
- [Symtropy's test-only route oracle and transport benchmark work](https://github.com/Luminous-Dynamics/symtropy/pull/1662).

## 2. Strict truth/evidence separation

Each episode has distinct artifacts:

1. **Policy-visible observation view**: only facts, source identities, provenance families, freshness, coverage, conflict and permitted actions the policy is allowed to see. Its exact file SHA-256 is computed externally and recorded in the evaluator receipt; it is not a hash field inside that same file.
2. **Evidence-request model**: frozen rules describing what a permitted request may return, with cost, delay, coverage, sampling bias, error and provenance properties.
3. **Evaluator-only truth**: latent state and outcome rules used exclusively by the independent scorer. Never pass it to the policy process.
4. **Decision receipt**: selected action or defer/fallback choice, pre-outcome uncertainty/confidence, requested evidence and budget use.
5. **Scoring receipt**: outcome, realized loss, regret, full costs, guardrail outcomes and exact artifact digests.

The policy-input schema must reject unknown fields; in particular, it must reject a `hidden_truth`, oracle score, future outcome, evaluator label, or realized loss field. Store truth in a separate artifact and bind the exact observation-view bytes by digest in the evaluator receipt. A fixture test must prove that changing evaluator-only truth leaves the bytes exposed to the policy unchanged.

Evidence requests are proposals. Their execution is simulated in this track; there is no live scraping, outreach, purchase, experiment, dispatch, reservation, or external side effect.

## 3. Compared policies

Compare all arms on the same episode/seed pairs, initial evidence, available actions, time horizon, action authority, and resource budget:

- **Strongest fair incumbent**: rule-based, domain or current production policy chosen before scoring.
- **Forced action**: must select a plan, even when evidence is inadequate.
- **Always defer**: always uses fallback; prevents free abstention from appearing optimal.
- **Query-all**: requests as much evidence as the registered query budget permits; exposes unpriced information collection.
- **No-query / fixed-query**: simple baselines for the value of targeted selection.
- **Symthaea VoI policy**: pinned version, explicit uncertainty model and acquisition-cost assumptions.
- **Composed policy**: full stack with each project's declared role and pinned versions.
- **Clairvoyant oracle**: scorer-only upper bound using evaluator truth. Never treat as an achievable competitor.

Add leave-one-out ablations when they answer a preregistered attribution question. The baseline and treatment receive equal compute and tuning opportunities; evidence actually requested by a policy is charged to its own budget.

## 4. Outcome vector (not a universal score)

**Decision regret.** Report nonnegative regret versus the minimum-loss *feasible* action under evaluator truth, with the oracle's feasibility and authority constraints frozen before scoring. Keep realized decision loss separate from resource cost. For comparative performance, report the paired candidate-minus-comparator end-to-end loss: (candidate decision loss + all candidate costs) − (comparator decision loss + all comparator costs). This signed delta can be negative when the candidate improves. The comparator must be a pinned policy evaluated on the same episode/seed and matched budgets; do not select an action using hidden truth and call it a current-evidence baseline. The oracle gap is an upper bound, not a forecast of realizable improvement.

**Net value of information.** For each request, compare expected downstream decision-loss reduction against acquisition, processing, delay, fallback and operational costs. The receipt's `realized_decision_loss_microcredits` excludes items separately itemized in `costs`; end-to-end loss adds total cost exactly once. The harness reports a signed `realized_end_to_end_loss_reduction_vs_comparator_microcredits` field, calculated as baseline end-to-end loss minus candidate end-to-end loss on the same episode. Positive means the candidate had lower end-to-end loss; it is descriptive, not a causal estimate of information value. A paired cost-adjusted delta and a request-level value estimate must identify their counterfactual and may not subtract the same query cost twice. Distinguish:
- EVPI: value of perfect information, a theoretical upper bound;
- EVSI: expected value of the specific available study/request;
- realized request value: outcome under the frozen request/observation model.

Do not infer that a high information-gain score implies a high decision value: the fact may not change the chosen action.

**Selective quality.** Report action coverage, risk among acted-on cases, abstention/defer rate, fallback success, end-to-end loss with all deferred cases retained, and the cost/latency of fallback. Report matched-coverage comparisons and the full risk/coverage curve; do not select a threshold after viewing test outcomes.

**Tail and robustness.** Report median, p90 and p95 regret; worst-decile loss; outcome per stress regime; recovery time; and guardrail failure counts. Do not infer low risk from a favorable mean.

**Evidence quality.** Track source independence, mirror/copy families, freshness, missingness, conflicts, coverage, request yield and (where forecasts are probabilistic) Brier score and calibration diagnostics. Calibration metrics require their own known-outcome denominator and uncertainty statement.

**Distribution and rights.** Report whether abstention, wait time, fallback burden and error harm differ by preregistered groups when lawful and meaningful. Privacy, consent, source rights and acquisition authority are hard gates—not costs that can be offset by benefit elsewhere.

Keep all metrics visible. Any profile-specific scalar loss must be decision-specific, preregistered, and accompanied by its components and sensitivity.

## 5. Synthetic test matrix

At minimum, include these regime families:

1. nominal complete observations;
2. stale or partially missing spatial/capacity data;
3. copied or mirrored sources falsely appearing independent;
4. conflicting claims with traceable distinct provenance;
5. a minimally sufficient independent aggregate observation that can change the selected action;
6. an informative-looking observation that cannot change the action;
7. distribution shift in observation error/freshness;
8. evidence cost/latency spike or query budget exhaustion;
9. coordinator/source outage;
10. malformed, replayed, tampered or unauthorized evidence receipts;
11. high-consequence decision where defer/fallback is optimal;
12. low-risk decision where an inexpensive reliable observation makes action preferable to abstention.

Use known-truth synthetic worlds first. Keep a held-out scenario family unavailable during policy tuning. Use paired seeds and preserve every disposition, including invalid input, timeout, non-convergence, refusal, missing result and evaluator error.

## 6. Anti-gaming and negative controls

The independent test suite must demonstrate:

- **Truth isolation:** changing hidden truth never changes policy-visible input bytes or policy choices under fixed visible input and deterministic policy seed.
- **No free abstention:** always-defer incurs fallback cost, delay and unserved-demand loss; abstained cases remain in all end-to-end denominators.
- **No free queries:** acquisition, compute, delay, human review and recovery costs are charged consistently.
- **No mirror inflation:** copies sharing a provenance family do not count as independent confirmation.
- **No oracle leakage:** policies cannot read evaluator truth or post-outcome metrics.
- **Authority fail-closed:** disallowed acquisition and external effects are rejected even when estimated information value is high.
- **Receipt freshness:** changes to the policy, initial observation, evidence model, budget, seed or scoring profile invalidate a previous score receipt.
- **No tail hiding:** per-regime and tail metrics remain present when overall averages improve.
- **No denominator deletion:** failures, fallbacks and deferred cases remain counted.
- **Reproducibility:** identical frozen artifacts and seed reproduce the same decision/scoring artifacts, apart from explicitly documented environment metadata.

## 7. Initial benchmark targets

These are **engineering hypotheses**, not published standards and not claims about current Symthaea performance:

| Gate | Initial target | Interpretation |
|---|---|---|
| Authority/privacy/rights/budget invariant violations | 0 on the adversarial fixture corpus | Any violation blocks qualification |
| Scheduled decision dispositions | 100% accounted for | Completeness, not success |
| Held-out mean decision regret | Aim for ≥10% lower than the strongest fair non-VoI baseline | Only if the metric is defined and baseline regret is nonzero |
| High-priority stress-regime regression | No more than 2% relative regression per preregistered regime | Prevent aggregate gains hiding meaningful stress losses; define absolute tolerances where ratios are invalid |
| Evidence request quality | Better net decision loss than query-all and fixed-query controls under equal budgets | Must include cost, latency, privacy and selection bias |
| Abstention | Lower end-to-end loss at matched coverage, plus disclosed coverage curve | Selective risk alone is insufficient |
| Reproducibility / stale-receipt rejection | 100% on deterministic fixtures | Contract property only |
| Calibration | Report Brier score/reliability diagnostics with uncertainty where probability forecasts exist | No universal ECE threshold without a justified binning/sample design |

Freeze the primary endpoint, loss units, threshold, target population, sampling unit, uncertainty procedure and guardrails before scoring. Revise targets only through a versioned preregistration, never post-hoc to favor a system. If the 10% threshold is inapplicable (zero baseline regret, undefined denominator), report it as inapplicable rather than forcing a ratio.

## 8. Evidence ladder and claim ceilings

- **E0 — Contract:** schemas, truth separation and adversarial invariants only.
- **E1 — Synthetic known-truth:** reproducible policy comparisons in the registered generator's world only.
- **E2 — Frozen source replay:** permitted, source-pinned observations and independent reconstruction.
- **E3 — Held-out simulation:** comparison on untouched scenario families under equal budgets.
- **E4 — Shadow:** no dispatch/commit; policy output is compared with observed operations.
- **E5 — Prospective pilot:** requires a separate authorized protocol, rights/consent checks and independent oversight.
- **E6 — Independent replication:** separate operator or independently sourced setting.

A valid schema, authored test, green CI, synthetic win, or attestation does not alone establish real-world economic benefit. Queued, skipped, cancelled or absent checks are not passes.

## 9. Ownership boundaries

- **Symthaea:** evidence candidates, uncertainty/regret estimates, epistemic action selection and model qualification.
- **Symtropy:** optimization and physical/resource feasibility under declared objectives.
- **Sol-Atlas:** read-only spatial/network evidence, source conflict, coverage and freshness visibility.
- **Mycelix:** integration contracts, cross-system census, rights/authority separation and result federation.
- **Independent evaluator:** hidden truth, outcome scoring, oracle comparison, artifact verification and claim ceiling.

Add an adapter only after a field-by-field audit identifies a genuinely missing field. Do not create a second route, evidence, identity, event, reputation or authorization model.

## 10. Research anchors

- [NIST TEVV-Athlon Framework for Evaluating AI Systems (2026)](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems) motivates use-case-specific assessment and real-world outcome measurement.
- [Symthaea VoI evidence planner](https://github.com/Luminous-Dynamics/symthaea/issues/5596) owns evidence-acquisition planning semantics.
- [Symthaea counterfactual regret qualification](https://github.com/Luminous-Dynamics/symthaea/issues/2574) and [epistemic-action qualification](https://github.com/Luminous-Dynamics/symthaea/issues/2579) own the related model-level questions.
- [Conformalized Abstention Policies (PMLR 304, 2025)](https://proceedings.mlr.press/v304/tayebati26a.html) is one relevant research anchor for context-adaptive abstention; its reported results are not transferable performance claims for this stack.

## 11. Executable prototype and next steps

An initial Rust harness is now authored under [tools/civ-impact-decision-value](https://github.com/Luminous-Dynamics/mycelix/tree/research/civ-impact-adaptive-decision-value-v0-1/tools/civ-impact-decision-value). It creates 96 deterministic toy episodes (12 regimes × 8 seeds) and compares seven policies/controls for 672 episode-policy runs. It writes a report with per-policy and per-regime summaries and individual scoring receipts.

The four Draft 2020-12 contracts now cover:
- `CIV_IMPACT_DECISION_EPISODE_V0_1.schema.json`: policy-visible observation view;
- `CIV_IMPACT_DECISION_TRUTH_V0_1.schema.json`: evaluator-only latent facts and simulated request outcomes;
- `CIV_IMPACT_DECISION_SCORE_V0_1.schema.json`: per-episode decision/cost/regret receipt;
- `CIV_IMPACT_DECISION_REPORT_V0_1.schema.json`: aggregate census, policy summary and regime strata.

The test suite is authored to validate the policy view, hidden-truth object, aggregate report, and every generated scoring receipt; it also checks truth-field rejection, unauthorized acquisition, invalid-receipt non-ingestion, unnecessary/low-value evidence costs, distribution-shift control failures, cost-ledger and paired-loss arithmetic, forced-action safety failures, and preservation of all regime strata. The exact-head workflow is queued/pending until it completes. These tests have not been reported as passes.

The `value_of_information` arm in this first harness is a reference heuristic only: it requests a declared high-reliability independent source when current evidence is not fresh, the declared expected decision-loss reduction exceeds acquisition plus delay cost, and budget/rights constraints permit it. A negative control supplies nominally high-reliability evidence whose declared decision value is below its cost. It is **not** the production Symthaea VoI algorithm and does not yet estimate calibrated EVSI or optimize the full decision-loss model. The reliability/value labels and toy losses are synthetic; 8 seeds per regime is a smoke/contract corpus, not an inferential sample size.

Run with:

```sh
cargo test --manifest-path tools/civ-impact-decision-value/Cargo.toml
cargo run --manifest-path tools/civ-impact-decision-value/Cargo.toml -- evaluate /tmp/civ-impact-decision-value-report.json
```

Next: confirm that the exact-head format, build, tests, and report generation succeed; commit a generated Cargo.lock and build metadata for reproducibility; have reviewers audit loss/cost/guardrail semantics; then connect pinned Symthaea evidence-request output through its existing contract. Only after the synthetic contract layer qualifies should the work progress to permitted frozen-source replay, held-out simulation, and then a no-side-effect shadow study. No measured real-world effect or economic benefit is claimed.

 
## 12. Cross-repository field map and adapter gate

The toy schema is not yet an adapter contract for the actual four-project stack. Before adapting real output, pin the exact producer commits and schema versions, then map every producer field into the benchmark receipt. A missing or non-equivalent field must be declared a gap; silently dropping it makes the evaluation weaker while giving the appearance of integration.

| Owner / source | Producer semantics to preserve | Current v0.1 mapping | Known gap / required gate |
|---|---|---|---|
| Symthaea evidence planner ([#5596](https://github.com/Luminous-Dynamics/symthaea/issues/5596), plus [regret #2574](https://github.com/Luminous-Dynamics/symthaea/issues/2574) and [epistemic actions #2579](https://github.com/Luminous-Dynamics/symthaea/issues/2579)) | Parent finding/decision receipt; uncertainty component; proposed evidence/study type; expected decision-loss reduction and uncertainty about that estimate; competing hypotheses affected; source independence; freshness/coverage/sample design; cost; privacy, access/consent/legal constraints; operational risk, bias/selection risk, reversibility; excluded candidates and reasons | episode_id; request_option_id and request_type; declared expected loss reduction, declared reliability, expected cost/latency/coverage; candidate source-family refs; sensitivity and authorization | Current fields are intentionally only a partial projection. It lacks the full uncertainty component, estimate uncertainty, sample design, competing-hypothesis effects, bias, reversibility and the full Pareto/exclusion receipt. Preserve a pinned raw producer receipt digest and mark these as unscored until represented; never turn uncertainty reduction into decision value automatically. |
| Symtropy transport/optimization ([PR #1662](https://github.com/Luminous-Dynamics/symtropy/pull/1662)) | Exact implementation/profile; source/network snapshot and demand-set identities; selected objective and units; route/candidate identities; feasibility scope; capacity/resource ledger; served and unserved demand; objective value; unknown/conflicted inputs; candidate-pool completeness/optimality scope | The toy fixture's action options, costs, units and per-episode truth/output receipts exercise only the evaluator mechanics | No actual Symtropy route/capacity report is consumed. Before a transport adapter, pin producer schema and objective units, reconcile every assignment against independent capacity conservation, retain unserved units, and preserve candidate-pool/optimality limits. Do not score a claimed objective total as proof by itself. |
| Sol-Atlas spatial/network evidence ([issue #106](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/106), [PR #111](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/111)) | Spatial subject/network-link identity; source snapshot and provenance; area/edge coverage; freshness; source conflict and missingness; observation rights | subject_ref, source_ref, source-family ref, provenance digest, age/freshness limit, observation state, rights profile and request coverage | v0.1 does not represent full geometry or network coverage topology. Spatial aggregate coverage must not be interpreted as complete visibility. Pin the read-only projection and retain unavailable, missing and conflicted cells/links in the denominator. |
| Mycelix proposal/coordination boundary ([#5044](https://github.com/Luminous-Dynamics/mycelix/issues/5044)) | Proposal identity and exact producer artifacts; actor/authority context; permissions; decision vs proposal vs authorization vs commitment state; provenance and disputes | Policy-view digest, action ID, evidence/scoring receipt digests, source-rights/authorization guardrails and explicit no-side-effect claim | v0.1 is effects-disabled. It has no live reservation, dispatch, custody or delivery transition. A proposal cannot be promoted into an authorized action by the adapter; any operational authority needs a separately governed implementation and test boundary. |
| Independent evaluator | Hidden truth, expected evidence-request outcomes, action loss, guardrails, paired comparator, artifact census and claim ceiling | Separate truth object, independent scorer, policy/truth SHA-256 fields, four schemas and per-regime aggregate | In this prototype policy and evaluator share a process, so the API boundary is not an adversarial sandbox. Production-quality evaluation must isolate them into separate processes/containers with distinct inputs and credentials, then verify output digests externally. |

### Mandatory adapter tests

Before any cross-project adapter is described as integrated:
1. Assert schema/field-set alignment against exact producer commit digests and reject unknown fields or unrecognized enum values.
2. Test unit incompatibility (per shipment vs per demand unit, cost currency/scale, physical units and horizon) as a hard failure rather than attempting inferred conversion.
3. Mutate route assignments, objective, demand-set digest, capacity ledger, source snapshot, request rights and implementation version independently; every material mutation must invalidate the derived receipt or fail independent reconstruction.
4. Keep unknown, partial, conflicted, stale, unauthorized, infeasible and unserved outcomes visible in the run census. Never delete them to improve average scores.
5. Require a field-by-field mapping review from each owning project before promoting a consumer adapter beyond synthetic fixtures.

The current harness qualifies none of those producer contracts. The next integration milestone is mapping and replay with pinned artifacts—not calling the toy bridge policy a full-stack result.
