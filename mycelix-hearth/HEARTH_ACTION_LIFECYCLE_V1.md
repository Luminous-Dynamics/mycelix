# Hearth Action Lifecycle V1

**Status:** pure Rust contract prototype; not a durable journal and not connected to action dispatch.

## Why this is the next paradigm shift

A toast describes what the interface can currently say. It is not a durable proof of what happened remotely. If a call times out, the app restarts, or a response cannot be decoded after a side effect may have committed, replaying the command can create a duplicate or conflicting change.

Hearth should treat an action as a tracked operation with an explicit evidence-backed lifecycle, not as a button callback plus a toast.

## State machine

```text
Prepared ──► Submitted ──► Confirmed
    │             ├──────► ExplicitlyRejected
    │             └──────► OutcomeUnknown ──► ReconciledApplied
    │                                      └─► ReconciledNotApplied
    └────────────► NotSent
```

- **Prepared:** local operation metadata exists; no remote request is authorized to have been sent.
- **Submitted:** the intent to dispatch was durably recorded before the network call. Once recovered after a restart, treat it as ambiguous.
- **Confirmed:** a decodable successful domain response was received.
- **ExplicitlyRejected:** a typed domain result proves the requested effect was not applied. A generic transport error is not a rejection.
- **OutcomeUnknown:** the remote operation might have applied. Do not submit the same operation again. Query authoritative state or use a backend-supported idempotency contract.
- **ReconciledApplied:** authoritative evidence establishes that the intended effect exists.
- **ReconciledNotApplied:** authoritative evidence establishes that the intended effect does not exist. Only then can the user decide to start a *new* attempt; this does not re-open the old operation.
- **NotSent:** a local preflight guard stopped dispatch before any request was sent.

Repeating a status is an idempotent no-op. No direct transition exists from OutcomeUnknown to Submitted, Confirmed, ExplicitlyRejected, Prepared, or NotSent.

## Required durable dispatch protocol (future integration)

The model in `src/action_lifecycle.rs` is deliberately pure; it does not supply disk persistence, networking, or server-side deduplication. A real adapter must enforce the following ordering:

1. Create a unique operation ID and immutable target fingerprint. Store only the minimum metadata required for reconciliation; exclude human-readable messages, names, and raw household identities.
2. Durably append Prepared.
3. Durably append Submitted **before** attempting network dispatch. If this durable write fails, do not send the request.
4. Dispatch once under that operation ID.
5. Record Confirmed only from a valid successful response; record ExplicitlyRejected only from a typed domain response whose contract proves no effect occurred. For timeout, restart, disconnect, or ambiguous decode failure, record OutcomeUnknown.
6. On restart, convert every recovered Submitted state to OutcomeUnknown before displaying it. Never replay it automatically.
7. Reconcile against an authoritative, correctly scoped domain read or a server-supported idempotency result. An inconclusive read remains OutcomeUnknown.
8. A retry after proven non-application is a **new user-authorized attempt** with its own operation ID. If/when the backend supports idempotency, retries of the same logical operation must reuse the same key, and the backend must bind that key to the canonical operation parameters.

The durable adapter must also handle an error after a remote commit but before the local journal records the response. The only safe answer is reconciliation or server-supported idempotency, not a client-generated claim of exactly-once execution.

## Identity and privacy constraints

- Operation ID is immutable within an attempt and unique in the relevant persistence scope.
- Target fingerprint binds the operation to a target without copying raw identifying data into the journal. Its canonicalization, hashing, key rotation, and collision requirements remain future work.
- The action class is an allowlisted enum, not arbitrary dispatch text.
- The journal is an observation of command lifecycle, not authorization. Every backend action must still validate signer, active household, membership, role, and domain invariants.
- A journal record cannot prove that a zome call succeeded; only domain response/evidence can establish a result.
- Reconciliation reads must be scoped to the selected authoritative Hearth. Do not guess the first household or aggregate from unrelated households.
- No UI state, Symthaea suggestion, local simulation, journal state, or reputation value may independently authorize external or physical effects.

## Current implementation boundary

The initial public-repository implementation includes a dependency-free state model, transition validator, dispatch-eligibility predicate, restart recovery rule, and deterministic tests. It does **not** integrate with local storage, IndexedDB, Holochain zomes, or the action toasts. It must not be represented as durable recovery or end-to-end duplicate protection.

The next implementation tranche is a persistence adapter with crash-injection qualification. Only after that should a separate zome/backend tranche add canonical operation identity, same-key/different-payload rejection, authorized replay semantics, and authoritative result lookup.

## Benchmark protocol

Targets below are **not results**:

### Pure state-machine gates

1. Exhaustively test all 64 status-pair transitions; accept only the declared edges.
2. Zero forbidden transitions accepted.
3. 100% of recovered Submitted attempts become OutcomeUnknown.
4. Zero dispatch-eligible states other than Prepared.
5. Zero direct transitions from OutcomeUnknown to Submitted, Confirmed, ExplicitlyRejected, Prepared, or NotSent.
6. 100% of inconclusive reconciliation remains OutcomeUnknown.
7. Preserve operation ID and target fingerprint through every state transition.
8. Run every test at the exact reviewed commit; queued, skipped, or absent checks are not passes.

### Future durable integration gates

Inject a crash at each boundary: before Prepared is committed; after Prepared but before Submitted; after Submitted but before dispatch; while waiting for the response; after the remote effect commits but before response delivery; after response receipt but before local confirmation commits; during reconciliation; and after journal compaction/restart.

- 10,000 deterministic failure/replay schedules with zero duplicate effects **only under a server-enforced idempotency contract**.
- Same operation ID plus same canonical payload returns the same recorded outcome.
- Same operation ID plus different canonical payload is rejected.
- Any uncertain record is never silently converted to failure or success.
- 100% of rejected or not-sent operations have a contract-supported reason.
- Journal corruption, unavailable storage, incompatible schema version, stale household scope, or unavailable reconciliation fails closed.
- Verify crash durability with a separate process and actual storage, not only an in-memory simulator.

### Human/accessibility gates

- At least 90% of study participants correctly distinguish OutcomeUnknown from ExplicitlyRejected.
- At least 90% choose “check current state” instead of immediate resubmission after an unknown outcome.
- Screen readers announce a status change without forcing focus to jump, in line with [WCAG 2.2 SC 4.1.3](https://www.w3.org/WAI/WCAG22/Understanding/status-messages.html).
- Maintain resident control: no surprise automatic retry, no inferred consent, and no undo control unless reversal semantics are actually supported and clearly described.

## Research references

- [AWS Builders' Library: Timeouts, retries, and backoff with jitter](https://builder.aws.com/content/3EumjoZascWd1oZiEgL8ORlv3qE/timeouts-retries-and-backoff-with-jitter) — a failed/slow response does not prove a side effect did not happen; retries need to respect idempotency and load.
- [Stripe: Idempotent requests](https://docs.stripe.com/api/idempotent_requests) — server-held keys bind duplicate retries to the stored result and parameters; this property is not replicated by a client-side UUID alone.
- [W3C WCAG 2.2: Status Messages](https://www.w3.org/WAI/WCAG22/Understanding/status-messages.html) — communicate non-focus status updates to assistive technology.
- [Apple Human Interface Guidelines: Feedback](https://developer.apple.com/design/human-interface-guidelines/feedback) — feedback should describe status and outcomes at an appropriate level of interruption.
- [Apple Human Interface Guidelines: Undo and redo](https://developer.apple.com/design/human-interface-guidelines/undo-and-redo) — reversal must be predictable and visibly reflected; Hearth must not claim undo unless the domain supports it.

## Nonclaims

Passing the pure Rust test suite would establish only the specified state-machine behavior. It would not prove persistence, UI integration, Holochain serialization, server authorization, cross-device consistency, exactly-once effects, or human comprehension. Those require separate evidence at their actual boundaries.
