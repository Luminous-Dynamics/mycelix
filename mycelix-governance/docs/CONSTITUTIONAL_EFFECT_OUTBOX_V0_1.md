# Constitutional Effect Outbox Formal Model v0.1

Status: experimental bounded formal tranche for #1544 / MYC-CONST-003D1A.

This model explores crash consistency and logical exactly-once effect semantics using an **abstract stable operation identity**. It does not claim that runtime operation identity is cryptographically authenticated; that remains dependent on qualified 003B4 ClaimBinding plus the #1532 refinement crosswalk.

## Durable design

The model intentionally collapses constitutional commit and durable outbox creation into one action: `CommitAndEnqueue(op)`.

That produces:

- one committed operation for the use;
- one durable outbox identity equal to that committed operation;
- `EffectPending` as the first post-commit phase;
- conflicting already-prepared operations transition to `Aborted`.

There is no separate durable `ConstitutionallyCommitted` phase because the current safe design does not need a crash-observable state in which constitutional authority is committed but its effect intent has not been recorded.

If an implementation later introduces such a persistence gap, it is a different refinement and must be re-modeled and re-qualified.

## Durable versus volatile state

Durable/model-history state includes:

- operation phase;
- committed operation;
- outbox identity;
- external logical effect identity;
- observed effect;
- durable receipt;
- commit/effect/receipt history;
- reconciliation history;
- integrity halt.

Volatile state includes:

- worker claim;
- in-flight request;
- caller acknowledgement.

`Crash` clears only volatile state and advances the bounded event counter. The safety invariants retain independent history sets so a mutant that erases durable commit/effect/receipt state after a crash can be detected.

## External effect abstraction

`ExternalDeliver(op)` represents delivery to an external subsystem with stable idempotency identity `op`.

Multiple deliveries of the same `op` are allowed. They increase `deliveryAttempts` but keep:

- one `externalEffect` identity;
- `effectCount = 1`;
- one `effectHistory` member.

This is a **logical idempotency assumption**, not evidence that a real provider implements exactly-once physical execution.

The model separates delivery from acknowledgement:

- `ExternalDeliver(op)` may perform the logical effect;
- `ReceiveSuccessAck(op)` records a known success;
- `LoseOrTimeoutAck(op)` enters `UnknownOutcome` without deciding whether delivery happened.

Timeout is therefore never treated as failure.

## Unknown outcome

`UnknownOutcome` blocks retries and conflicting operations until reconciliation.

Two reviewed reconciliation paths exist:

- `ReconcileSuccess(op)` — external evidence proves the effect happened;
- `ReconcileNoEffect(op)` — external evidence proves no effect happened, returning the same committed `op` to `EffectPending` so a retry preserves identity.

A contradictory external status may trigger `ObserveContradiction(op)`, entering `IntegrityHalted` without erasing any already-recorded effect or receipt.

## Canonical safety properties

The baseline config checks:

- `AtMostOneCommittedOpPerUse`;
- `EffectRequiresConstitutionalCommit`;
- `ReceiptRequiresObservedEffect`;
- `ExternalEffectIdentityMatchesCommittedOp`;
- `RetryDoesNotChangeOperationIdentity`;
- `UnknownOutcomeBlocksConflictingOp`;
- `CrashPreservesDurableHistory`;
- `ReceiptMonotonic`;
- `IntegrityHaltDoesNotEraseEffect`;
- `CallerAckIsNotConstitutionalState`;
- `LogicalEffectAtMostOnce`;
- `OutboxBeforeEffect`.

## Named non-vacuity predicates

Qualification should independently establish reachability for:

- `CrashAfterPrepareReached`;
- `CrashAfterCommitBeforeEffectReached`;
- `UnknownOutcomeWithEffectReached`;
- `UnknownOutcomeNoEffectReached`;
- `ReconcileSuccessReached`;
- `ReconcileNoEffectReached`;
- `DuplicateDeliveryReached`;
- `ReceiptAckLostReached`;
- `ContradictionHaltReached`.

A safety PASS without these witnesses must not be treated as evidence that the relevant crash/recovery behaviors were exercised.

## Required mutation controls

A future exact-head qualifier should deliberately break at least:

1. `CommitAndEnqueue` so effect delivery can occur without a durable outbox/commit;
2. retry identity so `worker` or `inFlight` can target a different `Op`;
3. the `UnknownOutcome` retry/conflict block;
4. receipt-before-observation protection;
5. `Crash` so it may erase committed/effect/receipt durable state;
6. external idempotency so duplicate delivery increments logical effect count beyond one.

Each mutant should expose the corresponding named invariant.

## Bound sensitivity

The initial checked config uses `MaxStep = 8`. Qualification should execute at multiple finite horizons, for example 8, 10 and 12, and retain generated/distinct state counts and search depth.

Report only `PASS at bounds {...}`. Do not call the bounded model an unbounded exactly-once theorem.

## Refinement boundary

This model assumes abstract operation identity equality is trustworthy.

Before runtime refinement can cite it:

1. 003B4 must qualify exact concrete ClaimBinding semantics;
2. #1532 must map one abstract `Op` to one stable authenticated runtime operation identity;
3. #1535 must define actual durable commit boundaries and provider adapter semantics;
4. any crash-observable runtime intermediate absent from this model must trigger a refinement/model update.

## Non-goals

No Holochain implementation, no physical exactly-once claim, no external-provider SLA, no automatic compensation for irreversible effects, no assumption that unknown outcomes always reconcile, and no recovery authority to invent a different target/payload/operation.
