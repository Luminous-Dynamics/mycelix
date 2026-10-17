# ADR: Typed Effect Lifecycle and Durable Recovery

**Status:** Proposed, pending implementation review  
**Date:** 2026-10-17  
**Tracking issue:** [#5159 — ADR: use typed effect states and qualify real-store recovery](https://github.com/Luminous-Dynamics/mycelix/issues/5159)  
**Related work:** [Regional policy kernel PR #5098](https://github.com/Luminous-Dynamics/mycelix/pull/5098), [runtime commit-classification PR #780](https://github.com/Luminous-Dynamics/mycelix/pull/780), [AGENT-009 #803](https://github.com/Luminous-Dynamics/mycelix/issues/803)

## Decision to evaluate

Prefer a **versioned tagged lifecycle record** for the persistence boundary and a transition API that only permits legal state changes. Do not keep expanding a flat record with independent phase, disposition, dispatch, uncertainty, and optional evidence fields indefinitely.

This ADR is a proposal, not a claim that a typed encoding or a durable adapter has been implemented.

## Problem

The reference model in PR #5098 now checks phase/revision consistency, dispatch history, whether uncertainty was observed, and whether terminal phase agrees with the sink disposition. Its mutation test explores 80 single-field mutations across ten valid snapshot histories.

That is useful defense-in-depth, but it reveals a design pressure: a flat snapshot permits many field combinations that cannot represent meaningful lifecycle states. Restore-time validation must reject those combinations repeatedly.

There is also a hard limit to value-level validation. A coherent-looking record can have its identity or evidence digest replaced without violating the shape rules. **State consistency is not record authenticity, and record authenticity is not rollback resistance.** Those must remain separate obligations of the durable adapter and exact-store commit-classification procedure.

## Alternatives

| Option | Advantages | Costs / risks | Recommendation |
|---|---|---|---|
| Flat struct plus more invariants | Minimal public API change; easy to serialize | Many mutually dependent optionals and flags; incomplete validation can admit impossible combinations | Keep only as compatibility/reference form |
| Tagged enum snapshot | State-specific fields live in distinct variants; incompatible fields become harder to express | Serialization format/versioning still needs validation; deserializers can be handed hostile data | Preferred persistence representation to prototype |
| Generic Rust typestate (Attempt<Reserved>, etc.) | The compiler can prevent illegal calls within a live process | Awkward across async workflows and persistence boundaries; reconstructed values still need validation | Consider for internal transition API, not as the only persisted representation |
| Append-only transition/event log | Supports audit, replay, and explicit transition history | Requires atomic sequencing, retention, replay protection, and a trustworthy head/high-water mark | Longer-term option after store semantics are qualified |

## Candidate record shape

Keep the CAS revision and schema version in an envelope, then carry one tagged state value. The sketch below is illustrative; exact names and serde representation require implementation review.

    EffectAttemptEnvelopeV2 {
        schema_version,
        revision,
        identity_commitment,
        state,
    }

    enum EffectStateV2 {
        Prepared,
        Reserved { operation_id },
        DispatchStarted { operation_id },
        OutcomeUnknown { operation_id },
        Committed {
            operation_id,
            outcome_digest,
            resolution: Direct | ReconciledFromUnknown,
        },
        RejectedBeforeDispatch {
            operation_id: Optional,
            reason_code,
        },
        RejectedBySink {
            operation_id,
            outcome_digest,
            resolution: Direct | ReconciledFromUnknown,
        },
    }

The state variants should make these combinations structurally unavailable:

- Committed with a definitive rejection disposition.
- RejectedBySink without destination evidence.
- Reserved, DispatchStarted, or OutcomeUnknown carrying a terminal sink disposition.
- RejectedBeforeDispatch carrying a post-dispatch outcome.
- A reconciled terminal outcome represented as if uncertainty had never occurred.

The envelope revision still requires transition validation. For the current reference path, direct terminal resolution is revision 3 and resolution after OutcomeUnknown is revision 4. Do not assume those numbers generalize to a future multi-attempt protocol.

The identity commitment must bind the authority domain, idempotency key, exact target digest, and policy-binding digest. The terminal outcome reference must bind the operation ID and verified sink disposition. Avoid storing payload bytes, personal data, credentials, private keys, prompts, or hidden reasoning in the journal.

## Trust boundaries that must not collapse

The implementation must preserve four independent facts:

1. **Local persistence classification:** Committed, DefinitelyNotCommitted, or IndeterminateCommit.
2. **External effect observation:** applied, definitively rejected, or unknown.
3. **Current authorization:** whether this action or a later attempt is authorized now.
4. **Outcome quality:** whether the action was correct, safe, beneficial, or otherwise acceptable.

No one of these facts proves any other. In particular:

- DefinitelyNotCommitted is not retry authorization.
- IndeterminateCommit is reconciliation-only.
- A local database commit does not prove that a provider performed the external effect.
- A verified provider outcome does not prove that the outcome was wise, safe, or beneficial.
- Serialized state must not reconstruct execution authority after process restart.

## Implementation sequence

1. **Prototype the typed state** in the existing dependency-free reference crate. Keep the old snapshot available temporarily for differential tests.
2. **Differential conformance:** run the current transition corpus against both encodings. Every legal current history must round-trip to the same public phase/revision semantics; every illegal combination must fail closed.
3. **Versioning and migration:** make the decoding boundary explicit. Reject unknown schema versions by default. Do not silently reinterpret old rows or guess migration outcomes.
4. **Storage composition:** when the production adapter is eligible to be wired, reuse PR #780's exact-store witness/classification method and AGENT-009's authority/retry separation. Do not build a second outbox or duplicate classifier.
5. **Real fault injection:** exercise transaction boundaries for reservation, dispatch-start persistence, sink delivery, and terminal receipt persistence. On recovery, query/reconcile with the same operation identity and exact target digest.
6. **Independent observation:** count sink-applied effects independently from local records; do not use the journal's own conclusion as its only verifier.
7. **Rollout:** keep the typed record as a shadow/conformance encoding first. Switch persistence only after migrations, exact-head qualification, and recovery tests are reviewed.

## Qualification and benchmark targets

These are proposed exit targets, not measured results.

| Gate | Target |
|---|---|
| Typed-state transition corpus | 100% of positive and negative conformance cases produce the specified result |
| State mutation | Zero accepted invalid phase/evidence combinations in deterministic field mutation suites |
| Concurrency | No duplicate reservation winner in modeled schedules, then in actual store-CAS races |
| Fault injection | 100,000 adapter fault scenarios with zero duplicate sink effects and zero false-commit classifications |
| Uncertain outcome | No new logical operation from OutcomeUnknown; reconciliation uses the same operation identity |
| Latency | Report p50/p95/p99 for flat and typed forms on the same hardware and toolchain; initially target no more than 10% p95 regression unless assurance gains justify it |
| Durable performance | Measure storage commit, recovery, and provider observation separately; never equate an in-memory benchmark with production throughput |
| Privacy | No raw payloads or unrelated sensitive data in snapshots, receipts, or operational logs |

A completed exact-head workflow run is required to claim tests passed. A queued, missing, cancelled, stale-head, or compile-only result is not a pass.

## Research basis

- [Microsoft Rust patterns book: type-state and newtype patterns](https://github.com/microsoft/RustTraining/blob/main/rust-patterns-book/src/ch03-the-newtype-and-type-state-patterns.md) — type-state can enforce ordering for live values, but does not authenticate deserialized records.
- [Loom](https://github.com/tokio-rs/loom) — model concurrent Rust executions around synchronization the model actually represents.
- [SQLite atomic-commit crash testing](https://www.sqlite.org/atomiccommit.html#testing_atomic_commit_behavior) and [SQLite testing methodology](https://www2.sqlite.org/testing.html) — inject write loss/corruption at varied points, reopen, and check transaction recovery.
- [Jepsen](https://github.com/jepsen-io/jepsen) — history-based verification under concurrent operations and injected faults.

## Non-goals

- Claiming exactly-once side effects from a Rust enum alone.
- Adding a parallel production outbox or another persistence classifier.
- Treating in-memory models, source inventories, or benchmark compilation as durable-system evidence.
- Using civic/reputation scores as a substitute for explicit, contestable authorization policy.
