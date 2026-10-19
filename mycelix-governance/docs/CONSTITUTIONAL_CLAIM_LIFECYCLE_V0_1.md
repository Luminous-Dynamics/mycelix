# Constitutional Claim Lifecycle v0.1

Status: experimental, non-activating semantic/reference tranche.

## Purpose

`constitutional-consumption` owns single-finalization/effect safety. `constitutional-temporal-provenance` owns accepted evidence and closure history. `constitutional-closure-coverage` decides whether a revocation interval currently authorizes terminalization.

This layer composes those responsibilities into one explicit claim lifecycle without deleting evidence or weakening the qualified parent semantics.

## Composite-state rule

The lifecycle reference machine **owns** its `TemporalEvidenceState` and `ConsumptionState`.

Callers cannot supply an interchangeable temporal snapshot to `finalize`, revocation, closure resolution, or effect application. Public evidence-ingest operations mutate the owned temporal state, then lifecycle/consumption consequences are applied to that same staged composite state.

Fresh construction accepts only a pristine temporal state. A pre-populated temporal snapshot is rejected. Persisted recovery must deserialize the complete composite lifecycle state and run `validate_restored()`; attaching arbitrary historical evidence to a fresh consumption machine is outside this contract.

Public state-changing operations are transactional in the reference model: they execute on a clone, check the complete cross-layer invariants, and replace the live value only on success. This is a semantic atomicity model, not a claim that a distributed runtime gets ACID transactions for free.

## States

- `PendingExecutable` — live claim with no active revocation.
- `BlockedAwaitingEvidenceClosure` — non-executable under a known revocation but still eligible for authenticated historical finality effective before that revocation.
- `Finalized` — accepted finality won the use index; status binds the exact finality-evidence ID and proof.
- `RejectedConflict` — another claim won the same use index; losing evidence remains inspectable and the status binds the winning claim, evidence, and proof.
- `RevokedClosed` — the pre-revocation evidence interval is authoritatively closed and the claim is terminally revoked.
- `IntegrityHalted` — unresolved work is halted because consumption or temporal assumptions are contradictory.

Terminal history is monotonic. A later fault does not rewrite already-finalized or already-closed history; it halts unresolved work and future effects while preserving evidence.

## Composition rules

1. Lifecycle claims are bound to the owned temporal finality domain and one shared use budget.
2. Finalization requires a matching `FinalityEvidence` already accepted by the **owned** temporal-provenance state. Bare sequence numbers or evidence from another snapshot cannot drive lifecycle finalization.
3. Revocation is ingested into the owned temporal state and immediately applied to the same lifecycle/consumption state. The earliest effective observed revocation remains active.
4. Finality-vs-revocation precedence uses effective order only. Verifier observation order is intake provenance and is never substituted for legal precedence.
5. On first successful finalization, every live same-use competitor is resolved to `RejectedConflict` in the same transactional lifecycle operation.
6. Revocation makes unresolved executable work `BlockedAwaitingEvidenceClosure` immediately. A blocked claim may still finalize if the owned temporal state accepts historical finality effective before that revocation.
7. Terminal revocation consumes only typed `PreRevocationCoverage` from `constitutional-closure-coverage`:
   - `Open` remains blocked;
   - healthy `Closed` or `EmptyPreRevocationInterval` becomes `RevokedClosed`;
   - `IntegrityFault` halts unresolved work and cannot issue a terminal revocation receipt.
8. Closure acceptance is authority-bearing and is blocked after a lifecycle/temporal integrity halt. Evidence observation may continue into the temporal quarantine path after faults.
9. Effect application requires lifecycle `Finalized` status and delegates to the qualified consumption state. Historical duplicate effect delivery remains an idempotent read even after a later fault.
10. Evidence retention does not affect `remaining_uses()`: only finalized use indexes consume the budget.

## Resolution receipts

Every lifecycle transition receives a globally unique, contiguous ordinal and records the claim ID, use index, MatterId, envelope/target/payload commitments, previous/new status, and typed reason.

Finalization binds the accepted owned finality-evidence ID. Conflict resolution binds the winning claim, winning finality evidence, and winning proof. Revocation closure retains the exact closure object or explicit empty-interval provenance. Fault halts retain the exact temporal or consumption fault.

## Safety invariants

- the owned temporal and consumption domains remain compatible;
- a non-pristine temporal state cannot be attached through the fresh constructor;
- every observed revocation has one monotonic active earliest-effective lifecycle representation;
- lifecycle `Finalized` exactly refines an underlying finalized consumption **and** an accepted owned finality-evidence record;
- `RejectedConflict` references a real finalized winner and its accepted finality evidence;
- at most one lifecycle claim is finalized per use index;
- live pending work cannot remain executable under revocation or a global fault;
- terminal states never transition back to live states;
- an active internal fault implies a lifecycle halt and no live claims;
- transition receipt ordinals are globally unique and contiguous and each claim's receipt chain is internally continuous;
- effects require a finalized lifecycle claim;
- historical finalized/closed/effect evidence remains readable after later fault.

## Formal model

`ConstitutionalClaimLifecycle.tla` is a bounded two-claim/one-use abstraction for the **composite** lifecycle state. It does not expose caller-supplied temporal snapshots; temporal/revocation/coverage facts are part of the single modeled state.

Canonical safety remains separate from named non-vacuity/reachability cases:

- conflict rejection;
- revocation blocking;
- closure terminalization;
- late pre-revocation finality;
- integrity halt;
- preservation of already-terminal history after later fault.

The TLA+ model is intentionally smaller than Rust. Concrete claim/proof/closure identities, MatterId, accepted-evidence IDs, receipt ordinals, policy strings, constructor-pristine checks, and Rust's staged-clone transaction boundary remain Rust/refinement obligations. A qualifier must therefore bind both Rust and TLA+ bytes and include implementation-level negative controls.

## Evidence boundary

This tranche does not establish Holochain/runtime refinement, distributed atomic commit, unbounded liveness, recovery after integrity fault, or production authorization. It must be qualified on top of exact qualified closure coverage before #1426 quiescence/liveness or #1191 runtime enforcement consumes its results.
