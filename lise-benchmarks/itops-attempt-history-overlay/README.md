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
- append-only attempt identity, ordinal, predecessor, action, request and idempotency binding, dispatch/response/effect state;
- typed evidence with source identity, dependency group, independence, freshness and action/request binding;
- no-effect evidence reference and same-key idempotency qualification as separate conditions;
- exact expected next step and hard-gate violations.

An attempt record is a fixture representation for exercising the owners' composition. It is **not** a replacement canonical operation ledger, history-completeness receipt, finality certificate, or outcome ledger.

## Frozen known-answer families

The 16 public cases cover: initial no-attempt baseline; independently observed applied effect; timeout with incomplete history; provider acknowledgement without effect proof; qualified no-effect plus same-key retry; missing idempotency qualification; shared-dependency observer; blind retry after unknown effect; safe continuation of an unattempted multi-action suffix; empty but incomplete history; broken attempt chain; delayed event with preserved earlier frontier; request-digest drift; idempotency-key drift; conflicting observations; and partial multi-action execution with an unknown suffix.

Public fixtures are evaluator references, not hidden agent challenge cases. A future sealed task set must be separately versioned and access-controlled.

## Qualification gates

The focused validator checks the frozen corpus and a mutation suite. It uses jq as an offline semantic fixture gate; it is not a full JSON Schema 2020-12 validator, cryptographic verifier, provider-history completeness proof, idempotency proof, or production Rust adapter.

Required future owner integration:
1. consume a verified operation-history completeness receipt from #6214;
2. consume operation-bound no-effect/finality and same-key replay semantics from #6139;
3. consume verifier-created per-operation outcome evidence from #6357;
4. reject incomplete/unknown/conflicting history without retry;
5. preserve earlier knowledge-time views when late events or backfill revise current knowledge;
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

For the frozen synthetic reference suite, target 100% exact disposition and zero false-safe retry/duplicate-effect outcomes. These are fixture gates, not industry-level performance results.

## Claim ceiling

A passing fixture gate means only that this public corpus has the expected structure and the reference semantic checks reject the listed mutations. It does not establish completeness of a real provider history, true physical or service effect, real-world idempotency, authorization to retry, production security, or SOTA recovery performance.
