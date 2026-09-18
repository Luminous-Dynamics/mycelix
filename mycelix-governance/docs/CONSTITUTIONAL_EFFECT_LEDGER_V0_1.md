# Constitutional Effect Evidence Ledger v0.1

Status: **experimental / inert runtime contract / unqualified**  
Tranche: **MYC-CONST-003D1C**  
Tracks: #1616  
Predecessor: MYC-CONST-003D1B / #1609

## Purpose

D1C turns the D1B append-only execution architecture into compile-checkable Rust without changing live provider dispatch and without registering a new Holochain DHT entry surface.

The new workspace crate is:

`mycelix-governance/crates/constitutional-effect-ledger`

It defines and validates:

- `ConstitutionalOperation`
- `ActionIntent`
- `ActionAttempt`
- `EffectObservation`
- `OperationResolution`
- provider `CapabilitySnapshot`
- whole-operation `EvidenceBundle`

The crate is deliberately pure Rust + Serde. It does not call HDK/HDI host functions and cannot execute external effects.

## Why these are not DHT entry types yet

Registering the records as writable Holochain entry types before the concrete authority/refinement chain is qualified would create a new persistence surface whose authorship and transition authority would itself need to be secured.

D1C therefore establishes the data model and fail-closed validation semantics first.

A later wiring tranche may register these structures only after it can bind creation and transition authority to the qualified constitutional operation identity.

## Identity boundary

D1C preserves D1B's abstract identity boundary.

`operation_id`, `action_id`, `claim_binding`, commitments, evidence identifiers, and idempotency identities are treated as opaque bounded values. D1C does **not** choose a final cryptographic encoding and does not claim that current runtime IDs are securely authenticated.

Concrete identity remains blocked on:

- qualified MYC-CONST-003B4 ClaimBinding;
- qualified MYC-CONST-003CR1 runtime refinement evidence;
- a successor mapping from identity material to the runtime encoding.

## Provider capability snapshots

Every `ActionIntent` carries a `CapabilitySnapshot` with:

- replay safety;
- outcome observability;
- compensation capability;
- provider batch atomicity;
- whether blind retry is allowed;
- the exact capability profile identity/revision;
- optional qualification evidence identity.

Positive provider claims require a qualification evidence identifier. Unknown capabilities remain conservative.

A caller cannot set `safe_retry_without_reconciliation = true` while replay safety is `Unknown`.

## Ordered durable intent

A valid operation contains the complete ordered action commitment list before attempts are represented.

The evidence bundle requires exactly one `ActionIntent` for each ordinal, with:

- contiguous ordinals starting at zero;
- exact commitment equality with the operation plan;
- exact capability profile identity/revision equality;
- unique action identities.

This gives a concrete Rust representation of D1B's **atomic constitutional intent** without claiming atomic physical provider effects.

## Attempt semantics

Every attempt binds to one action identity and carries:

- a stable attempt identity;
- one-based contiguous attempt ordinal;
- monotonic operation event sequence;
- the same idempotency identity as the original action intent;
- explicit retry authority.

Retry authority is one of:

```text
Initial
ReplaySafe
ReconciledNoEffect { observation_id }
```

The first attempt must be `Initial`.

`ReplaySafe` retries require a provider capability profile that explicitly permits retry without reconciliation.

For providers without qualified replay safety, a retry must cite earlier authoritative `KnownNoEffect` evidence for the immediately preceding attempt.

A known-successful action may never be retried.

## Effect observations

Physical knowledge is represented explicitly as:

```text
KnownSuccess
KnownNoEffect
UnknownOutcome
```

An `EffectObservation` binds to one exact action attempt and records a machine-readable evidence commitment.

Critically:

> a local call error, timeout, or missing acknowledgement is not evidence of `KnownNoEffect`.

`KnownNoEffect` is accepted only when based on:

- `ProviderQuery`, or
- `Reconciliation`.

Provider receipts require an explicit provider receipt commitment.

A reconciliation observation must identify the earlier `UnknownOutcome` it resolves, must refer to the same attempt, and must occur later in the operation event sequence.

## Unknown outcome and ordering

Later actions cannot begin until their immediate predecessor has an observed `KnownSuccess` event that occurs before the later attempt.

Therefore an unresolved unknown naturally blocks forward execution.

This is stronger than merely storing an aggregate batch result: the evidence chain proves the order in which authority and knowledge became available.

## Contradictory evidence

D1C intentionally does not implement last-write-wins for physical-effect knowledge.

For one attempt:

```text
KnownSuccess + KnownNoEffect
```

or regression from known terminal knowledge back to `UnknownOutcome` derives:

`IntegrityHalted`

The contradiction remains visible. It is not erased by selecting the newest observation.

## Operation resolution

The validator derives action state from the append-only evidence and then checks the declared operation resolution against it.

Supported resolution classes are:

- `Completed`
- `FailedNoEffect`
- `PartiallyCompleted`
- `CompensationRequired`
- `UnknownOutcome`
- `IntegrityHalted`

A resolution cannot invent its own success/no-effect/unknown census; those ordinal sets must exactly equal the state derived from attempts and observations.

Important rules include:

- `Completed` requires all actions observed successful;
- `FailedNoEffect` requires zero successful effects and at least one authoritative no-effect result;
- a successful prefix prevents `FailedNoEffect`;
- `PartiallyCompleted` requires at least one success and an incomplete/no-effect remainder;
- `CompensationRequired` may reference only actions whose effects were observed successful;
- `UnknownOutcome` requires an unresolved unknown;
- `IntegrityHalted` requires contradictory evidence.

## Compensation boundary

D1C still does not execute compensation.

`CompensationRequired` records constitutional recovery state only. A compensation must later become its own governed forward effect with its own stable identity, attempts, observations, unknown-outcome behavior, and receipt.

## Tests

The crate includes tests covering:

1. ordered two-action completion;
2. rejection of a later action started before predecessor success;
3. rejection of blind retry for a non-replay-safe provider;
4. acceptance of retry only after authoritative reconciliation proves no effect;
5. unknown outcome blocking a later action;
6. rejection of `FailedNoEffect` when a successful prefix exists;
7. contradictory same-attempt evidence deriving `IntegrityHalted`.

These are source-level tests only. No hosted exact-head qualification claim is made by this tranche.

## Runtime relationship

The existing execution coordinator is unchanged.

Current path remains conceptually:

```text
execute_actions(...)
    -> provider/cross-zome effects
    -> create legacy Execution
    -> update Timelock
```

D1C only creates the contract that a future safe path should satisfy:

```text
ConstitutionalOperation + all ActionIntent records
    -> durable authority boundary
    -> ActionAttempt
    -> EffectObservation
    -> explicit OperationResolution
```

No code in D1C routes `execute_timelock()` through that path.

## Next tranche

The next strongest step is **MYC-CONST-003D1D: provider capability adapters**.

Each provider should independently earn properties such as:

- stable-key or provider-token idempotency;
- authoritative result lookup;
- receipt verification;
- known-no-effect proof;
- compensation support;
- provider-level transaction atomicity, if any.

Until earned, the D1B source-bound capability profile remains `Unknown`/`NoneObserved` and D1C will not justify blind retries from those providers.

After provider capabilities exist, a later D1E can register/wire the durable evidence records and move real dispatch behind the durable operation boundary.

## Non-claims

D1C does not establish:

- production execution repair;
- DHT persistence of the new records;
- runtime author/authority validation for the new records;
- physical exactly-once execution;
- current provider idempotency or reconciliation;
- automatic compensation;
- physical batch atomicity;
- concrete cryptographic operation identity;
- deployment currentness;
- exact-head qualification PASS.
