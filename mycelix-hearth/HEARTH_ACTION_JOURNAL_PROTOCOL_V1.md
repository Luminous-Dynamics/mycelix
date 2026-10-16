# Hearth Action Journal Dispatch Gate V1

**Status:** pure Rust protocol seam with deterministic fault-injection tests. No persistent browser adapter or live action integration exists yet.

## Why this layer exists

The state model in `action_lifecycle.rs` defines which operation states are legal. This companion module specifies the ordering between that model, a journal writer, and a one-shot transport call:

```text
persist(Prepared)
        |
        v
persist(Submitted) -- commit not confirmed --> DO NOT SEND
        |
        v
dispatch exactly once
   | success response          | ambiguous error
   v                           v
persist(SourceChainCommitted)        persist(OutcomeUnknown)
```

A process-local operation ID or state enum is not a durable journal. This module does not implement storage, network dispatch, Holochain idempotency, or exactly-once effects. Its purpose is to make the dispatch ordering independently testable before a browser storage adapter is written.

## Contract implemented

- `persist_prepared` returns an opaque `PreparedAction` permit only after the caller's persistence function reports a successful commit for Prepared.
- `dispatch_after_journal` accepts only that permit while its lifecycle status remains Prepared.
- It transitions to Submitted and calls the persistence function before invoking the supplied one-shot dispatch closure.
- If the Submitted persistence operation does not report success, the dispatch closure is never called.
- A conclusive successful response moves the in-process attempt to SourceChainCommitted. The result explicitly states whether the confirmation update was persisted.
- An untyped transport error is OutcomeUnknown, not ExplicitlyRejected; the module attempts to persist that state and reports whether that write was confirmed.
- A second dispatch call with the same permit is rejected by the state guard.
- If the remote response arrived but writing SourceChainCommitted failed, the present process may know the result; after a restart the journal may contain only Submitted. Recovery must conservatively convert the recovered Submitted record to OutcomeUnknown and reconcile.
- If the OutcomeUnknown write fails, the previously committed Submitted state remains the conservative recovery input. The permit cannot be dispatched again.

## Fake-journal fault injection

The Rust test harness simulates the persistence adapter with a journal that can fail on a selected write. It includes cases for:
1. Prepared write failure;
2. Submitted write failure and zero transport calls;
3. success response with proof that Submitted was committed before dispatch;
4. ambiguous transport response and no re-dispatch;
5. confirmation commit loss after an observed remote success;
6. OutcomeUnknown write failure;
7. ordering for all six action kinds;
8. refusal to mint a permit from the wrong state.

These are protocol-model tests, not tests of real browser persistence. The fake writer's successful return is treated as a commit solely for the purpose of exercising this ordering contract.

## Browser persistence research and implementation requirements

IndexedDB is the appropriate next adapter to evaluate for structured transactional journal records. Its transactions can atomically modify records within a transaction scope. IndexedDB 3.0 also defines a `strict` durability hint that can reduce risk from OS crash or power loss, at possible performance and battery cost. **The hint does not prove absolute power-loss safety.**

Browser origin storage is best-effort by default. The Storage API's `persist()` method asks the browser for persistent storage; the user agent may deny the request. Hearth should query and expose the resulting storage posture instead of promising the journal can never be evicted. Storage can also be deliberately cleared by the user. See:
- [W3C Indexed Database API 3.0 — transaction durability](https://www.w3.org/TR/IndexedDB/)
- [MDN — Storage quotas and eviction criteria](https://developer.mozilla.org/en-US/docs/Web/API/Storage_API/Storage_quotas_and_eviction_criteria)
- [MDN — StorageManager.persist()](https://developer.mozilla.org/en-US/docs/Web/API/StorageManager/persist)
- [MDN — Using IndexedDB](https://developer.mozilla.org/en-US/docs/Web/API/IndexedDB_API/Using_IndexedDB)

The future browser adapter must wait for the IndexedDB transaction completion event before it resolves the journal commit promise; a queued request or successful individual request event alone is insufficient. Where supported, select strict durability for the minimal pre-dispatch journal state, measure its impact, and record the actual reported capability. If opening the database, the transaction, or commit fails, do not send the remote action.

Persist only the minimum operation metadata: schema version, operation ID, allowlisted action kind, opaque target fingerprint, selected Hearth scope fingerprint, lifecycle status, and timestamps/version needed for recovery. Do not persist invitation/gratitude message bodies, household names, or raw member keys in the journal. Define retention, user-visible clear-history behavior, migration, quota handling, corruption handling, and multi-tab writer coordination before production use.

## Holochain boundary

The journal is a client-side recovery aid, not an authorization grant or deduplication authority. Holochain zome calls are atomic on one source chain, but DHT validation is not a globally atomic transaction. A remote call can commit and then lose its response. The frontend must therefore avoid blind resend, while a separate backend design must specify operation-key scope, canonical parameter binding, concurrent same-key behavior, and prior-result lookup.

References:
- [Holochain zome functions — atomic, transactional commits](https://developer.holochain.org/build/zome-functions/)
- [Holochain validation](https://developer.holochain.org/build/validation/)
- [AWS Builders' Library — timeouts, retries, backoff and jitter](https://builder.aws.com/content/3EumjoZascWd1oZiEgL8ORlv3qE/timeouts-retries-and-backoff-with-jitter)
- [Stripe — idempotent requests](https://docs.stripe.com/api/idempotent_requests)

## Benchmarks

All targets below are proposed acceptance criteria, not measured results.

### Protocol-model targets

- Zero transport calls without a committed Submitted write.
- Zero dispatch after a failed/aborted Submitted write.
- 100% of ambiguous transport errors remain OutcomeUnknown.
- Zero direct repeat dispatch using the same permit.
- 100% of the six action kinds obey the same commit-before-send ordering.
- For a failed confirmation write, preserve the current response to the caller while retaining Submitted as the durable recovery predecessor.
- All protocol tests run on the exact reviewed commit; authored, queued, skipped, and missing results must never be labeled PASS.

### Future browser-adapter gates

- 100% of remote sends have a successful IndexedDB transaction-completion signal for Submitted recorded before the send.
- Crash/reload injection at each boundary (before Prepared, between Prepared and Submitted, after Submitted and before send, in flight, after remote commit, after response, during confirmation persistence, and during recovery) yields no blind client replay.
- 10,000 deterministic restart schedules with zero client-side automatic re-dispatch of an unknown attempt.
- Journal corruption, unknown schema version, quota/commit failure, and unavailable storage all fail closed before dispatch.
- 100% of recovered Submitted records become OutcomeUnknown before presentation.
- Report persistent-storage grant/denial without overstating it; no user-facing claim that data is impossible to evict.
- Multi-tab tests show a single local dispatch permit per operation ID, or fail closed when coordination cannot be established.

### Future backend idempotency gates

The above is not an exactly-once benchmark. A separate server/zome contract must prove same-key/same-canonical-payload replay behavior, reject same-key/different-payload reuse, bind results to authorization and target scope, and survive concurrent requests and lost responses. Distributed duplicate-effect tests are meaningful only after that implementation exists.

## Nonclaims

Passing this protocol model establishes only the stated call ordering and failure classification for the supplied closures. It does not prove actual IndexedDB durability, UI integration, Holochain serialization, multi-tab fencing, remote uniqueness, exactly-once execution, or resident comprehension.
