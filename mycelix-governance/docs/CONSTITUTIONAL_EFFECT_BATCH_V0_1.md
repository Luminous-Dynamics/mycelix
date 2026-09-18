# Constitutional Effect Batch and Provider Capability Contract v0.1

Status: **experimental / source-bound / unqualified**  
Tranche: **MYC-CONST-003D1B**  
Tracks: #1607  
Predecessor: MYC-CONST-003D1A / #1545

## Purpose

D1A establishes the crash-consistent single-effect rule:

> constitutional commit and durable effect intent must exist before effect dispatch.

D1B extends that rule to the execution coordinator's actual multi-action shape. It distinguishes two properties that must never be conflated:

1. **atomic constitutional intent** — one authorized operation commits a stable ordered action plan before any physical effect; and
2. **atomic physical effects** — every physical action succeeds or fails as one indivisible provider transaction.

The first is a protocol requirement. The second is a provider capability and is **not observed** in the current execution coordinator.

## Exact runtime observation subject

D1B binds its runtime observations to:

`15b9c89adf0ac3c6c5a73681614d6bfcd368820a`

That is the D1A semantic head. D1A does not modify the production execution zome, so the bound execution source blobs are:

```text
execution coordinator
mycelix-governance/zomes/execution/coordinator/src/lib.rs
3dbb8a8f69b377e494ccf24164c94bd80f54e0ef

execution integrity
mycelix-governance/zomes/execution/integrity/src/lib.rs
657edaee9a314f100a0c4b1609a4596cf243e61d
```

The capability manifest is `ObservedSourceBound`. It is deliberately narrower than a design document: unknown provider behavior stays `Unknown`.

## Runtime mismatch frozen by D1B

The source-visible execution path currently has all of these properties:

- action dispatch occurs before the durable `Execution` entry is created;
- the execution identifier is created only after action dispatch;
- an action array is dispatched sequentially;
- a later failure can occur after a successful prefix;
- `ExecutionStatus::PartialSuccess` exists in integrity types but the coordinator selects only `Success` or `Failed`;
- a partially executed batch is therefore collapsed into `Failed`;
- integrity validation requires a present `Execution.result` to be valid JSON;
- the coordinator constructs human-readable result strings rather than JSON;
- `EmitEvent` discards the `emit_signal` result and returns an `[emitted]` description.

D1B records these as observations. It does not infer a live exploit or a deployed failure rate.

## Capability profile

Each effect class is described independently across dimensions:

- `effect_boundary`
- `replay_safety`
- `outcome_observability`
- `compensation`
- `provider_batch_atomicity`
- whether retry without reconciliation is justified
- whether the action may participate in a claimed physically atomic batch

The current source does **not** establish stable-key idempotency, provider-token idempotency, authoritative postcondition queries, compensation contracts, or provider-level atomic batching for any of the three action classes. Those fields therefore remain conservative.

A timeout, cross-zome error, or missing acknowledgement is not permission to invent a successful or failed physical outcome.

## Required runtime structure

A runtime refinement should use append-only evidence-bearing records rather than one post-hoc aggregate record:

```text
ConstitutionalOperation
  stable operation identity
  exact authorization / ClaimBinding
  proposal + timelock identity
  ordered action commitments

        | durable before any dispatch
        v

ActionIntent[0..n]
  stable action identity
  action commitment
  capability-profile identity
  idempotency identity, if qualified
  ordering constraint

        | attempts append evidence
        v

ActionAttempt / EffectObservation
  attempt identity
  dispatch evidence
  KnownSuccess | KnownNoEffect | UnknownOutcome
  provider receipt / query evidence when available

        v

OperationResolution
  Completed
  FailedNoEffect
  PartiallyCompleted
  CompensationRequired
  UnknownOutcome
  IntegrityHalted
```

The existing immutable `Execution` entry should not be stretched into a mutable recovery ledger. An append-only operation/action evidence chain makes retries, reconciliation, and partial completion independently auditable.

## Stable identity boundary

D1B still uses abstract stable operation and action identities.

A concrete action identity should eventually bind at least:

```text
operation identity
action ordinal
exact action content commitment
provider capability-profile identity
authorization epoch / ClaimBinding
```

D1B does **not** choose the final cryptographic encoding. Concrete identity remains blocked on qualified 003B4 ClaimBinding and the runtime refinement crosswalk.

## Multi-action semantics

The default batch rule is ordered, not physically transactional:

1. commit the operation and all ordered action intents;
2. action `i` may start only after all required predecessors are observed successful;
3. if action `i` enters `UnknownOutcome`, no later action may start;
4. a non-replay-safe action may retry only after authoritative reconciliation proves `NoEffect`;
5. a known no-effect failure before any successful effect may resolve to `FailedNoEffect`;
6. once any prior physical effect succeeded, a later failure must not collapse the operation to plain failure;
7. successful prefixes therefore remain visible as `PartiallyCompleted`, `CompensationRequired`, `UnknownOutcome`, or another explicit evidence-bearing recovery state;
8. `Completed` requires every required action to be observed successful.

## Compensation boundary

Compensation is not rollback.

A compensating action is a new governed forward effect with its own stable identity, intent, attempt history, unknown-outcome handling, and receipt. An irreversible action can never be made historically absent by labeling a later operation "rollback".

D1B records the requirement but intentionally does not model the full compensation executor. That should be a successor tranche after provider-specific compensation capabilities exist.

## Formal model

`ConstitutionalEffectBatch.tla` models a three-action ordered operation with:

- durable batch intent before requests;
- one in-flight action at a time;
- explicit `UnknownOutcome`;
- reconciliation success and reconciliation no-effect;
- a non-replay-safe retry gate;
- known no-effect failure;
- truthful partial completion;
- crash-to-unknown behavior;
- integrity halt.

The model uses representative sets `ReplaySafe` and `Queryable` to exercise protocol branches. **Those sets are not claims about the production action classes.** Production capability claims live only in the source-bound JSON manifest.

The baseline config uses `MaxStep = 10`. A future qualifier should run multiple bounds and independent reachability configurations.

## Safety properties

The baseline model checks:

- `IntentBeforeAnyRequest`
- `EffectRequiresCommittedIntent`
- `EffectsRespectActionOrder`
- `ObservedSuccessRequiresEffect`
- `KnownNoEffectFailureRequiresNoEffect`
- `UnknownOutcomeBlocksParallelWork`
- `UnknownOutcomeBlocksLaterActions`
- `NonReplayRetryRequiresNoEffectProof`
- `CompletedMeansAllActionsObserved`
- `FailedNoEffectMeansNoPhysicalEffect`
- `PartialCompletionIsTruthful`
- `KnownFailureStopsForwardDispatch`
- `IntegrityHaltIsExplicit`

## Non-vacuity targets

A future exact-head qualifier should independently demonstrate reachability of:

- `PartialFailureReached`
- `UnknownWithPriorEffectReached`
- `NonReplayRetryAfterNoEffectProofReached`
- `UnreconcilableUnknownReached`
- `CompletedBatchReached`
- `FailedNoEffectReached`
- `CrashIntoUnknownReached`

Safety PASS without these witnesses is insufficient.

## Runtime repair ordering

The implementation path should be split so evidence remains interpretable:

1. **D1B** — this formal/profile tranche;
2. **D1C** — introduce durable operation/action intent and structured observation types without enabling provider dispatch through the new path;
3. **D1D** — provider adapters earn idempotency/reconciliation/compensation capabilities independently;
4. **D1E** — switch execution dispatch to the durable outbox/action-intent path;
5. **D1F** — crash/restart and partial-completion qualification against the real Holochain runtime.

Do not jump directly from the current coordinator to "exactly once".

## Qualification boundary

This semantic PR does not add another hosted qualification workflow while the existing formal lanes are queue-bound.

A future D1B qualifier should bind:

- the exact D1B semantic head;
- this document;
- the TLA+ model/config;
- the closed JSON schema;
- the source-bound capability manifest;
- the independent Python validator;
- exact runtime source blobs;
- multiple TLC bounds;
- reachability witnesses;
- mutation controls for order, unknown-outcome blocking, non-replay retry, partial-result truthfulness, source blob binding, and capability inflation.

## Non-claims

D1B does not establish:

- a runtime repair;
- physical exactly-once execution;
- provider idempotency;
- provider reconciliation;
- provider compensation;
- physical batch atomicity;
- concrete cryptographic operation identity;
- deployment currentness;
- qualification PASS.
