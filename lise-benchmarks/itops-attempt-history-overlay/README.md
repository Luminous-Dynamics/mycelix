# ITOPS Attempt-History Overlay v0.1

**Status:** public synthetic reference suite; no production operation-history adapter, provider integration, live recovery run, or retry execution is claimed.

## Why this overlay exists

The base ITOPS oracle tests terminal recovery dispositions. It does not, by itself, qualify whether a subsequent retry is safe across a complete sequence of attempts. A timeout is not proof of no effect, an empty event list is not proof of no attempt, and a provider acknowledgement is not an independent observation of service state.

This overlay tests the composition of existing owners rather than creating a second ledger:

- Mycelix #6214 owns scoped operation-history completeness and independent effect reconciliation.
- Mycelix #6139 owns operation-bound finality and retry-safety reference semantics.
- Mycelix #6357 owns independently verified terminal outcome evidence.
- Mycelix #6853 owns the base public ITOPS outcome oracle.
- Mycelix #6863 hardens the base oracle's work denominator; this overlay is stacked on that prerequisite branch.

## Paradigm shift

Do not evaluate only whether the last attempt appears successful. Evaluate the *entire operation history* at a named knowledge frontier:

`operation -> ordered attempts -> scoped history completeness -> per-action effect evidence -> idempotency qualification -> permitted next step`

The benchmark's output is a bounded next-step disposition, not an autonomous authorization. A retry is eligible only when the history is complete for the exact provider/account/resource scope, the exact action is independently reconciled as not applied, and a separately qualified idempotency contract binds the same request digest and idempotency key. Otherwise the result is hold/no-retry/invalid-history.

## Contract

Each case pins:
- exact operation, request digest, idempotency key and ordered action set;
- history query scope, cursor completeness, gaps, retention coverage, backfill status and as-of frontier;
- append-only attempt identity, ordinal, predecessor, action, request and idempotency binding, dispatch/response/effect state, and distinct attempted-at versus recorded-at timestamps;
- typed evidence with source identity, dependency group, independence, freshness, action/request binding, exact attempt ID, event-time and observation-time; an observation cannot be reused to support two attempts;
- no-effect evidence reference and same-key idempotency qualification as separate conditions;
- exact expected next step and hard-gate violations.

An attempt record is a fixture representation for exercising the owners' composition. It is **not** a replacement canonical operation ledger, history-completeness receipt, finality certificate, or outcome ledger.

## Frozen known-answer families

The 18 public cases cover: initial no-attempt baseline; independently observed applied effect; timeout with incomplete history; provider acknowledgement without effect proof; qualified no-effect plus same-key retry; missing idempotency qualification; shared-dependency observer; blind retry after unknown effect; safe continuation of an unattempted multi-action suffix; empty but incomplete history; broken attempt chain; delayed event with preserved earlier frontier and reproducible event-time/knowledge-time separation; request-digest drift; idempotency-key drift; conflicting observations; partial multi-action execution with an unknown suffix; a second dispatch after a confirmed applied effect even when its own result is still unknown; and a separately evidenced duplicate effect across two attempts.

Public fixtures are evaluator references, not hidden agent challenge cases. A future sealed task set must be separately versioned and access-controlled.

## Qualification gates

The focused validator checks the frozen corpus and a mutation suite. It uses jq as an offline semantic fixture gate; it is not a full JSON Schema 2020-12 validator, cryptographic verifier, provider-history completeness proof, idempotency proof, or production Rust adapter.

Required future owner integration:
1. consume a verified operation-history completeness receipt from #6214;
2. consume operation-bound no-effect/finality and same-key replay semantics from #6139;
3. consume verifier-created per-operation outcome evidence from #6357;
4. reject incomplete/unknown/conflicting history without retry;
5. preserve earlier knowledge-time views when late events or backfill revise current knowledge; bind each prior frontier's known attempt and evidence IDs exactly to records observable at that as-of time; and reject event/observation timestamps that run backwards or exceed the current frontier;
6. qualify per-action recovery without replaying an already-applied prefix.

## Metrics to report

Report separately, never as one accuracy score:
- history-completeness false-positive and false-negative rates;
- blind-retry and duplicate-effect rate;
- safe-retry precision and missed-safe-retry rate;
- provider-acknowledgement-to-effect confusion;
- shared-dependency false-independence rate;
- per-action partial/unknown classification;
- unknown-to-reconciled latency;
- prior-frontier replay stability under delayed events/backfill;
- human reconciliation and recovery minutes;
- action/tool count, time, compute and fully burdened cost per independently accepted restoration.

For the frozen synthetic reference suite, target 100% exact disposition and zero false-safe retry/duplicate-effect outcomes. The gate separates a prohibited second dispatch after an applied first attempt from a *proven duplicate effect*, which requires applied-effect evidence linked to each distinct attempt ID. These are fixture gates, not industry-level performance results.

## Recommended agent-evaluation protocol (next phase)

The 18 public cases are a reference oracle and regression suite, not a representative estimate of operational success. A leaderboard claim needs an independent sealed challenge set and matched system arms.

### Task construction

Build a sealed set of at least 96 operation-recovery scenarios spanning at least eight independent incident families (at least 12 scenario roots per family). Include provider timeouts with late commits, incomplete paginated history, retention-window truncation, reordered/duplicated/backfilled events, same-key and changed-key replay, non-idempotent side effects, shared-observer failure domains, multi-action partial completion, and authorization/scope changes. Vary the provider, account, resource, tool response and time ordering without changing the underlying answer. Keep public fixtures, task generator, hidden answer key and verifier independently versioned.

A scenario is an independent test unit; repeated model runs on one scenario estimate stochastic variability but are not additional independent scenarios. Report confidence intervals clustered by incident family and scenario root. Preserve every task exclusion and every attempted run.

### Matched evaluation arms

Use the same foundation model, tools, permissions, task data and resource budgets in every arm:

- **A0 — baseline agent:** current agent/tool stack, with no new attempt-history overlay.
- **A1 — explicit operation state:** A0 plus a persistent operation/action/attempt frontier and explicit history-completeness handling.
- **A2 — evidence-qualified recovery:** A1 plus independent effect reconciliation, provenance/failure-domain checks and exact idempotency binding.
- **A3 — full composed system:** A2 plus Symthaea planning/reconciliation and Mycelix authority/provenance/outcome verification.
- **Human comparator:** trained operator using the same evidence and tools, with time-to-recovery and interventions recorded separately.

Keep agent ablations and human comparison separate; do not average the human result into a model leaderboard. Run three or more repetitions per agent arm for stochastic variation, while reporting scenario-level paired differences.

### Primary and safety endpoints

Pre-register one primary operational endpoint: **independently verified service restoration per assigned incident**, under a fixed wall-clock, action, token and cost budget. Retain intention-to-treat accounting: timeouts, crashes, unresolved work and tasks handed to humans remain in the denominator.

Report separate non-compensatory endpoints:
- false-safe retry and duplicate-effect rate;
- unauthorized/wrong-scope mutation rate;
- precision of retry-eligible decisions and missed safe-retry opportunities;
- final-state and fix-specific functional postcondition pass rate;
- incomplete-history false-completeness rate;
- provider-acknowledgement-to-effect confusion;
- false-independence rate across declared failure domains;
- unknown-to-reconciled latency and prior-frontier replay stability after delayed events;
- human intervention/recovery minutes, operator workload and recovery quality;
- fully burdened cost per independently verified restoration.

No weighted score may compensate for a false-safe retry, duplicate effect, unauthorized mutation, missing provenance or failed critical postcondition.

### Quantitative gates

- Public reference gate: exact disposition on all 18 frozen cases and rejection of all 21 seeded mutations.
- Held-out development gate: at least 96 distinct scenario roots across eight or more incident families; report family-clustered intervals, not only aggregate point estimates.
- Matched contribution gate: at least +3 percentage points in verified restoration versus A0 on the same held-out tasks, with no increase in any critical safety failure and no degradation in family-level performance.
- External comparison gate: only claim superiority to SWEOps-Bench after reproducing its exact 55 tasks, operator repositories, injected faults, workload, harness, time budget and verification rubric. A prospective stretch target is at least 60% verified recovery, versus the published highest configuration at 52.7%; it is not a directly comparable claim on this overlay's different scenarios.
- Rare-event safety gate: zero observed false-safe retries is necessary but not sufficient. With zero failures, approximately 299 independent adverse retry opportunities are needed for a one-sided 95% binomial upper bound below 1% (and roughly 2,995 for below 0.1%). Correlated repeats of the same scenario do not count as independent opportunities. Publish the exact interval and do not describe a finite test as proof of zero risk.

This makes the benchmark a *safety-and-outcome qualification study*, not an average task-success contest. A high diagnosis score cannot compensate for unsafe execution; a safe hold is correct when the evidence is insufficient; a recovery only counts when independently verified.

## Claim ceiling

A passing fixture gate means only that this public corpus has the expected structure and the reference semantic checks reject the listed mutations. It does not establish completeness of a real provider history, true physical or service effect, real-world idempotency, authorization to retry, production security, or SOTA recovery performance.
