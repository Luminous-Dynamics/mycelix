# Treasury outbox recovery model v1

**Status:** effects-disabled Rust/Go reference state machine. Not a production worker, storage engine, Treasury zome integration or provider adapter.

Related obligations:
- [FIN-PAYMENT-SAGA-001 / #5744](https://github.com/Luminous-Dynamics/mycelix/issues/5744)
- [FIN-SAFE Treasury outbox recovery / #4965](https://github.com/Luminous-Dynamics/mycelix/issues/4965)
- [AC-172/AC-168 implementation candidate / #4958](https://github.com/Luminous-Dynamics/mycelix/pull/4958)
- [Provider adapter qualification / #4957](https://github.com/Luminous-Dynamics/mycelix/pull/4957)
- [Immutable journal replay / #6021](https://github.com/Luminous-Dynamics/mycelix/pull/6021)

## Paradigm shift: classify the crash boundary, not just the response

A conventional outbox solves the dual-write problem by persisting business intent and later delivering it. Delivery is normally at-least-once: a worker may publish an effect and crash before recording success, so duplicate delivery must be tolerated by an idempotent destination/consumer.

Mycelix makes the uncertainty boundary explicit in durable state:

`Ready → LeasedForPreflight → DispatchAuthorized → (effect call) → outcome`

The key rule:

- Lease expiry while only preflight is leased: safe to return to Ready and query again.
- Lease expiry after DispatchAuthorized was durably recorded: the call may have happened. Transition to AwaitingReconciliation, not Ready.
- An Indeterminate response has the same treatment.
- Only a destination-qualified authoritative absence may return the item to Ready. The next worker must perform a fresh preflight before it can authorize another dispatch.
- A discovered receipt must exactly match operation, stable effect ID, destination and payload digest before Delivered can be recorded.

This deliberately chooses safety over liveness when the destination cannot prove presence or absence. Unknown remains unknown.

## State machine

| State | Meaning | Permitted next operation |
|---|---|---|
| Ready | Due for processing; no current lease | Claim preflight |
| LeasedForPreflight | One worker owns a short read-only preflight lease | Query destination |
| DispatchAuthorized | Exact absence proof and lease persisted; one dispatch may be attempted | Dispatch original stable effect |
| AwaitingReconciliation | An effect may have happened or destination state is uncertain | Claim read-only reconciliation |
| LeasedForReconciliation | One worker owns a read-only reconciliation lease | Query same effect ID |
| Delivered | Matching destination receipt is recorded | No automatic mutation |
| DeadLetter | Permanent rejection or attempt cap reached | Operator review / explicit remediation |
| ManualReview | Invalid evidence, mismatched receipt, or unclassified response | Human investigation; no automatic dispatch |

Lease generation and worker identity fence stale workers in the reference model. A worker whose lease has expired or been replaced cannot commit an outcome through the state-machine API. In production this fence must be enforced by a durable compare-and-swap / transactional update, not an in-process map or mutex.

## Typed outcomes

Destination query:
- `AuthoritativeAbsence`: adapter has proved absence under a declared destination, key scope, consistency and retention contract.
- `Receipt`: destination returned a receipt candidate that must bind to the exact immutable outbox subject.
- `Uncertain`: no safe presence/absence conclusion; keep reconciling with bounded backoff.

Dispatch:
- `Committed`: receipt candidate must match all immutable fields.
- `DefinitelyNoEffectTransient`: adapter contract proves no destination effect occurred; retry same stable ID after backoff.
- `Indeterminate`: effect may have occurred; reconcile.
- `PermanentRejection`: stop automatic delivery and dead-letter.
- `SourceEvidenceInvalid`: stop and require manual review.

**Important:** the reference type `AuthoritativeAbsence { proof_digest }` only checks digest syntax and state transitions. It does not authenticate a provider proof or determine whether a provider's ordinary 404/empty read is authoritative. A real adapter must define and verify the absence semantics using the provider's product-specific API contract, key namespace, retention period, query consistency, destination account/region, and receipt/callback authentication. If those semantics are not qualified, return Uncertain.

## Retry, lease and scheduling rules

- Stable effect ID and immutable payload survive all retries.
- A new operation ID is never minted because a timeout or acknowledgement was lost.
- Bounded batch size: at most 64 items per claim in the reference model.
- Lease is generation-fenced; stale lease completion is rejected.
- Exponential delay is capped; positive jitter is deterministic from effect ID and attempt number so independent Rust/Go implementations produce the same schedule.
- The policy includes finite dispatch-attempt and reconciliation-query caps plus a maximum reconciliation age. When an unknown outcome exceeds its reconciliation budget, it moves to ManualReview—not Ready, Delivered or DeadLetter—and preserves the unresolved evidence. A final valid matching receipt can still resolve the original effect.
- Every item has one explicit disposition; queue metrics expose ready, leased, awaiting-reconciliation, delivered, dead-letter, manual-review and oldest unresolved age.
- Last attempt time, dispatch/reconciliation counters, reconciliation-start time, next dispatch time, next reconciliation time, last sanitized error class and evidence digests are retained by the reference item.
- Reconciliation attempts are counted when a read-only reconciliation lease is claimed. A worker crash consumes that bounded attempt; its expired lease returns to AwaitingReconciliation, never to dispatch.

A Holochain callback is not assumed to host an indefinite worker. Production scheduling can be an explicitly managed worker/service polling the durable queue, with a documented cadence, maximum batch, lease TTL, restart/recovery behavior, alerts and operator ownership. The poller is a delivery mechanism, not an authority to change immutable intent or bypass the source/owner checks.

## Failure windows the model covers

1. Exact enqueue retry returns the same immutable effect; changed payload under the same effect ID conflicts.
2. Another worker cannot claim an already-leased item in the same serial state-machine book.
3. A dispatch call cannot be recorded before a preflight absence decision.
4. Expired preflight lease returns to Ready; stale generation cannot authorize it.
5. Expired DispatchAuthorized lease moves to AwaitingReconciliation and cannot re-enter dispatch directly.
6. Indeterminate dispatch never becomes retryable without reconciliation.
7. Matching receipt resolves the original effect; wrong destination/subject becomes ManualReview.
8. Authoritative absence after reconciliation returns to Ready, followed by another preflight.
9. Definitely-no-effect transient retry preserves the original effect ID and uses deterministic bounded backoff.
10. Permanent rejection dead-letters; source/evidence uncertainty requires manual review.

These are deterministic reference-state tests. They are not process-kill tests, durable database transaction tests, a real Holochain conductor test, a multi-process lock proof, or a live provider sandbox test.

## Industry comparison

AWS's transactional-outbox guidance addresses the business-write/message-publish dual-write problem and explicitly anticipates duplicate delivery, which is why idempotent consumers matter: https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/transactional-outbox.html

Mycelix should not claim to eliminate at-least-once delivery. It aims to make the uncertainty boundary inspectable and fail-closed: no blind replay after dispatch may have occurred, destination receipt is bound to immutable source intent, and retry semantics depend on a qualified adapter contract.

Provider idempotency is not universal. Adyen documents company-account key scope, a 7–14 day retention period, and regional-endpoint caveats; a provider key must therefore be scoped to the exact product/API profile rather than treated as a permanent global uniqueness theorem: https://docs.adyen.com/development-resources/api-idempotency

The provider matrix in #4957 already owns adapter-specific scope/retention behavior. Do not duplicate those claims here or silently extend an expired provider key's guarantee. An expired/unknown key outcome must reconcile to the original effect identity or stop for manual investigation; it must not invent a new logical payment.

## Qualification targets

### Non-compensatory safety gates

- 0 duplicate economic effects in the frozen crash/timeout/retry corpus.
- 0 dispatches from AwaitingReconciliation, LeasedForReconciliation, DeadLetter or ManualReview.
- 0 dispatch without a persisted preflight decision.
- 0 acceptance of mismatched operation/effect/destination/payload receipts.
- 100% same-effect identity across transient retries and reconciliations.
- 100% state accounting: every item appears in exactly one state.
- 100% stale lease/generation completions rejected.
- Reconciliation attempt and age caps always terminate in ManualReview, never automatic redispatch.
- Every unknown destination query remains non-delivered and non-dispatchable.

### Queue-scale benchmarks

The exact-head qualification now includes paired Rust and Go in-memory queue benchmarks for:
- 1,000 / 10,000 / 100,000 queued effects;
- bounded claims of 64 due items, including lease-expiry recovery;
- read-only metrics scans over the same queue sizes;
- 30 Rust raw samples per operation/size with p50/p95/p99 summaries;
- 30 Go benchmark runs per operation/size with ns/op, B/op and allocs/op, summarized using nearest-rank p50/p95/p99.

These are synthetic in-memory data-structure measurements. They exclude durable-store transactions, fsync, multiple processes/hosts, destination queries, Holochain conductor/DHT costs, provider API latency and external settlement. No performance result is claimed until the exact-head workflow completes and the raw outputs are inspected.

### Performance and operability (targets, not measured results)

Measure separately:
- state transition CPU and allocation costs at 1k / 10k / 100k queued items, compared with the newly added paired microbenchmarks;
- claim and lease transaction p50/p95/p99 for batch sizes 1 / 16 / 64;
- throughput and contention with 1 / 2 / 4 / 8 workers;
- reconciliation delay and time-to-resolution under destination timeouts, partial reads and unavailable periods;
- maximum-reconciliation-budget activation and manual-review backlog age;
- queue oldest age, awaiting backlog, dead-letter and manual-review rates;
- durable-store write amplification, lease churn and crash-recovery scan duration;
- actual provider latency and provider-specific key retention separately from internal queue latency.

Capture 30+ raw samples per scenario, exact source SHA, toolchain, host/CPU, storage backend, test seeds, full failure schedule and all negative results. A local state-machine benchmark is not comparable to an end-to-end payment benchmark. Safety correctness is never traded for throughput.

## Claim ceiling

This reference is not a durable outbox, autonomous service, provider adapter, distributed lease, Holochain action validator or bank settlement integration. A valid receipt candidate is not independently authenticated here. This model does not prove exactly-once effects, global idempotency, provider settlement, account conservation, DHT convergence, regulatory compliance or legal finality. Production qualification requires the persistent queue/lease transaction, destination-specific query/receipt proof, crash/restart integration tests, multi-worker contention, owner-authorized source records and exact provider sandbox contracts.
