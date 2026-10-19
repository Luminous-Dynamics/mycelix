# Constitutional Effect Policy v0.1

Status: experimental pure policy tranche for #1561 / MYC-CONST-003D1B.

This crate does **not** execute effects. It defines what the constitutional runtime is allowed to believe about effect providers and what execution strategies are admissible from those beliefs.

## Why this exists

The current execution coordinator runs a JSON action list sequentially and writes its `Execution` entry after action dispatch. A later action can fail after earlier effects have already occurred. The integrity layer has `ExecutionStatus::PartialSuccess`, but the coordinator currently records only binary success/failure for that path.

The current action types also have materially different semantics:

- `TransferCredits` intends a value transfer;
- `UpdateParameter` intends a durable governance-state mutation;
- `EmitEvent` emits a signal and intentionally ignores the signal result.

They must not share one undifferentiated `execute() -> success/failure` reliability model.

A second distinction is equally important: **the semantic effect required by an action is not necessarily the semantic effect established by the endpoint it calls.** The current `governance_bridge.transfer_credits` endpoint records a governance event; by its own contract it does not itself establish settlement of the value transfer. Therefore a successful bridge call cannot be used as proof that the `ValueTransfer` effect occurred.

## Core rules

**Provider capabilities are evidence-bearing inputs, not assumptions inferred from an API name.**

**Every non-notification effect must require durable constitutional success.** A `ValueTransfer`, `DurableStateMutation`, or `ExternalIrreversibleEffect` cannot opt out of that requirement merely because the caller supplied `requires_durable_success = false`. Conversely, `BestEffortNotification` may never claim durable constitutional success.

**Provider effect semantics must satisfy the action's required effect.** Idempotency, reconciliation, and atomicity do not compensate for semantic mismatch. An endpoint that only records intent/audit evidence cannot satisfy a value-settlement requirement.

The policy therefore separates:

- required action effect class;
- provider effect semantics;
- delivery/idempotency semantics;
- reconciliation mechanism;
- outcome observability;
- reversibility/compensation semantics;
- provider batch atomicity;
- evidence provenance.

A provider profile that claims anything stronger than `Unproven`/`Unknown` must carry a non-empty evidence reference.

A profile being representable does **not** make the corresponding action executable. For a durable action, semantic mismatch or unproven delivery/reconciliation/observability causes fail-closed admission even for a single-action plan.

## Effect contracts

`EffectClass` describes what the constitutional action requires:

- `DurableStateMutation`;
- `ValueTransfer`;
- `BestEffortNotification`;
- `ExternalIrreversibleEffect`.

`ProviderEffectSemantics` separately describes what the provider endpoint can establish:

- `PersistsDurableStateMutation`;
- `SettlesValueTransfer`;
- `EmitsBestEffortNotification`;
- `ExecutesExternalIrreversibleEffect`;
- `RecordsIntentOrAuditOnly`;
- `Unproven`.

The validation mapping is exact. For example, `ValueTransfer` requires `SettlesValueTransfer`; `RecordsIntentOrAuditOnly` is insufficient even when the intent record itself is durable and authentic.

Best-effort notification is intentionally excluded from durable constitutional success. Notification actions are split into `post_commit_notifications` by the planner rather than counted as durable batch members.

## Retry semantics

`classify_retry_policy` is conservative:

- idempotent-by-key / naturally idempotent providers may be retried only with the same stable operation identity;
- non-idempotent providers require authoritative proof of no-effect before retry;
- at-most-once providers do not auto-retry;
- unproven providers do not auto-retry;
- absence of stable operation identity disables automatic retry even for otherwise idempotent providers.

`AutoRetrySameOperation` is a **capability-level permission**, not a claim that the current D1A runtime profile performs direct retry from `UnknownOutcome`. D1A remains stricter: unknown outcome is a blocking state until reconciliation. Runtime integration must either preserve that stricter policy or qualify an explicit D1A extension.

## Batch planning

`plan_batch` preserves original action order and deterministically partitions actions into:

- durable actions;
- post-commit best-effort notifications.

It then chooses only among explicit strategies:

- `SingleActionOnly`;
- `ProviderAtomicBatch`;
- `SequentialCheckpointed`;
- `SagaWithExplicitCompensation`;
- `BestEffortNotificationFanout`.

### Provider atomic batch

This is admissible only when all durable actions use the exact same evidence-bearing provider capability profile and that profile claims `ProviderAtomicBatch`.

Provider ID/version equality alone is insufficient: two profiles with the same labels but different effect or reliability assertions are not one atomic provider contract.

### Sequential checkpointed batch

This requires every durable action to have a safe retry/reconciliation policy and rejects irreversible/unknown effects that cannot authoritatively establish no-effect.

Partial completion is always `Required` for this strategy.

### Saga

Saga planning is never automatic. The caller must explicitly request it and bind compensation authority. Compensation is a new effect; the policy does not let a failed action silently donate authority to its inverse.

## Stale-plan protection

A `BatchPlan` contains full action/provider profile snapshots, positions, retry decisions and policy version. `validate_batch_plan` recomputes the plan and requires exact structural equality.

This makes the following fail closed:

- provider capability version/evidence drift;
- provider effect-semantics drift;
- action reordering;
- changed retry assumptions;
- changed compensation-authority options;
- changed constitutional effect-policy version.

A later cryptographic/runtime layer should bind a digest of this canonical plan representation; this crate deliberately does not select the hash/signature algorithm.

## Conservative classification of current execution actions

The helper `conservative_existing_action_profile` encodes only what the current coordinator/provider code proves.

### TransferCredits

- required class: `ValueTransfer`;
- `requires_durable_success = true`;
- called provider: `governance_bridge.transfer_credits`;
- provider effect semantics: `RecordsIntentOrAuditOnly`;
- delivery/reconciliation/observability: `Unproven`;
- reversibility: `Unknown`;
- batch atomicity: `Unproven`.

The bridge implementation documents that it records a governance event and that actual fund movement is handled elsewhere. Therefore the current provider is rejected with `ProviderEffectMismatch`: recording transfer intent/audit evidence is not settlement.

A reverse transfer is **not** assumed to restore the original world state.

### UpdateParameter

- required class: `DurableStateMutation`;
- `requires_durable_success = true`;
- provider: `constitution.update_parameter`;
- provider effect semantics: `PersistsDurableStateMutation`;
- delivery/reconciliation/observability remain `Unproven` until the target contract receives provider-specific qualification.

The semantic effect matches, but the profile is still fail-closed for execution because uncertain delivery cannot yet be reconciled safely. Repeated assignment to the same value may look naturally idempotent, but this tranche does not infer that property from syntax.

### EmitEvent

- required class: `BestEffortNotification`;
- `requires_durable_success = false`;
- provider effect semantics: `EmitsBestEffortNotification`;
- current code ignores the result of `emit_signal`;
- it therefore cannot satisfy durable constitutional-effect success.

## Partial completion

The current execution coordinator already admits the factual state “some actions executed before a later action failed.” This policy treats that as a first-class semantic requirement:

- provider-atomic batch: partial completion may be claimed impossible only from provider evidence;
- sequential/saga: `PartialCompletionSemantics::Required`;
- single action / notification-only: batch partial completion is not applicable.

Future runtime records should persist per-action intent/result state rather than compress this into one binary `Failed` status with prose.

## Qualification boundary

The crate is intentionally a standalone pure workspace under its own `Cargo.toml`; it is **not yet added to the live governance workspace** and has no HDK/HDI dependency.

Before live execution integration:

1. this pure policy tranche must receive exact-head Rust qualification with mutation sensitivity;
2. 003B4 must qualify stable concrete operation/ClaimBinding identity;
3. 003CR1 must qualify the relevant runtime/formal mapping;
4. D1A crash/outbox semantics must qualify;
5. each provider capability stronger than `Unproven` must have its own evidence/refinement boundary;
6. a value-transfer action must point to a provider contract that actually establishes settlement, not merely records transfer intent.

## Non-goals

No live zome rewrite, no physical exactly-once claim, no Holochain distributed transaction claim, no automatic reverse-transfer compensation, no assumption that signals are durable, no assumption that a provider is idempotent because its API appears setter-like, no inference that an intent record equals settlement, and no runtime authority activation.
