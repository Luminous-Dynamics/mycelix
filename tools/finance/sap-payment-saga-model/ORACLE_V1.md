# Deterministic saga trace oracle v1

**Status:** Rust reference interpreter and independent Go implementation for frozen Saga Trace V1. The evaluator is effects-disabled. It does not call a production Finance zome, move funds or authenticate an external provider.

## Separation of responsibilities

The qualification harness separates three components:

1. Trace generator: emits deterministic canonical input bytes from fixed SplitMix64 seed derivation and the frozen V1 command grammar.
2. System under test: the payment journal, Treasury outbox, checkpoint and exposure implementations. These must be executed independently in a later harness stage.
3. Oracle: independently interprets the V1 command grammar, emits a disposition and reason for every command, and computes a checked economic conservation vector.

The oracle does not call the payment journal's transition helpers. Rust and Go implement the oracle separately and compare canonical evaluation digests against the same known-answer fixture.

**Important qualification distinction:** the present tranche is an expected-disposition reference interpreter. It is not yet a full command executor against the real Rust/Go saga models, does not yet shrink counterexamples, and does not report a million-trace campaign.

## Frozen input contract

Input is MXSAP-SAGA-TRACE-V1. Do not change its:
- domain separator, field layout, reserved byte or big-endian widths;
- 18 opcode assignments;
- owner/fault code assignments;
- six SplitMix64 draws and draw order;
- eight scenario templates;
- 64-command upper bound.

Any added exposure, delegated-budget or handoff commands require a new trace profile version. Do not mutate TraceV1.

## Disposition vocabulary

Every command produces one of:
- Applied: modeled state transition accepted;
- NoOp: exact idempotent replay or explicitly not-required operation;
- Rejected: transition not authorized by the modeled authority, identity, dependency or fault profile;
- Indeterminate: outcome remains unresolved; no inferred absence or new effect.

Each result also carries a reason code and the cumulative maximum Treasury effect count after that command. A rejection does not mutate the modeled economic state.

## Fault profile validation

A command's fault code is constrained by its opcode before its handler can mutate state:

| Command family | Allowed fault |
|---|---|
| Intent, authorization, local payment, Treasury preparation/dispatch/commit, retry, compensation | None |
| MarkIndeterminate | ResponseLostBeforeEffect or ResponseLostAfterEffect |
| ReconcileTreasury | None, LookupUncertain, ReceiptMismatch or AuthoritativeAbsence |
| CheckpointPayments | None or MissingDependency |
| CheckpointTreasury | None or StaleFrontier |
| MutateIntent / MutateEvent | PayloadMutation |
| DuplicateDelivery | DuplicateDelivery |
| StaleCheckpoint | WrongOwner |
| ForkCheckpoint | ForkedCheckpoint |

Unexpected fault/opcode combinations reject with InvalidFault. This prevents a fault annotation from being ignored by a command that otherwise would mutate economic state.

## Treasury semantics

The reference models one stable Treasury effect ID.

- Local payment commit is the prerequisite for Treasury fee preparation.
- Dispatch authorization does not prove destination effect or finality.
- ResponseLostBeforeEffect and ResponseLostAfterEffect remain Indeterminate until a destination-qualified reconciliation result.
- AuthoritativeAbsence is accepted only when it does not contradict the modeled possible-effect count.
- LookupUncertain remains Indeterminate and retains the complete fee exposure.
- RetrySameEffect is permitted only after AuthoritativelyAbsent and reuses the same effect ID.
- Positive reconciliation of an after-effect response loss creates ObservedUnsettled; a separate commit transition is then required.
- Duplicate delivery is conservatively modeled as at most one possible effect under the same stable ID; it never increments the effect count twice.
- ReceiptMismatch rejects without resolving the unknown outcome.

A timeout, missing response or uncertain lookup is never treated as authoritative absence.

## Checkpoint dependencies

A Payments checkpoint requires the local payment transaction to be committed. The Treasury checkpoint now additionally requires:
- a matching committed (or explicitly not-required) Treasury state;
- the same checkpoint value already accepted by the Payments checkpoint;
- no stale-frontier fault.

A Treasury checkpoint cannot leapfrog the Payments checkpoint, even if the Treasury fee itself has committed. Stale, wrong-owner and forked checkpoint commands reject without altering accepted checkpoint state.

## Compensation semantics

Compensation is a new effect, not retroactive deletion of the local payment or Treasury history.

The V1 model requires:
- the original local transaction to be committed;
- if a nonzero Treasury fee is required, its effect to be committed;
- a distinct compensation subject/effect identity, different from both the original operation nonce and the Treasury effect ID;
- the full modeled amount;
- the Sender owner and no injected fault.

An exact repeat of the same compensation identity/amount is idempotent. Reusing an identity with changed amount conflicts. The report retains original payer debit, recipient credit and Treasury credit and records the compensation as a separate balanced movement.

This reference only models the narrow compensation shape carried by V1. A production compensation protocol needs explicit authority, destination/recipient identity, bounded amount, a new immutable intent digest, stable effect ID and its own dispatch/evidence/settlement lifecycle.

## Economic evaluation vector

Each evaluation carries:
- principal and fee minor units;
- sender debit;
- recipient credit;
- fee held as an outstanding obligation;
- confirmed Treasury credit;
- maximum plausible unresolved Treasury exposure;
- compensation source debit and recipient credit;
- local commit state and Treasury state;
- count of possible/committed Treasury effects;
- compensation count;
- per-command disposition/reason sequence;
- signed conservation delta;
- invariant failure bitmask.

The model checks the balance equation:

sender debit = recipient credit + fee held + Treasury credit

Compensation is separately balanced and does not erase the original operation's historical entries.

The report also requires:
- no more than one possible/committed Treasury effect under the stable ID;
- no more than one compensation effect;
- a committed Treasury fee to have exactly one modeled effect;
- an AuthoritativelyAbsent state to have zero possible effects;
- no compensation identity collision with the original operation.

## Shared known answers

Fixture: fixtures/saga-oracle-v1.kat.

Eight fixed seed/index vectors each pin scenario, command count and the canonical evaluation SHA-256. The Rust and Go test suites independently regenerate the trace, evaluate it, verify the safety vector and compare the full report digest. The 4,096-seed checks are regression smoke tests and scenario-coverage tests; they are not a million-trace qualification run.

## Additional adversarial tests in this increment

- Remove the Payments checkpoint from a normal scenario; Treasury checkpoint must reject MissingDependency.
- Put LookupUncertain on CommitTreasury; the command must reject InvalidFault and leave the effect uncommitted/unresolved.
- Reuse the Treasury effect ID for compensation; compensation must reject identity reuse and leave the original Treasury effect intact.

## Benchmark plan

Run the same frozen seed sets and mutation corpus across Rust and Go:
- 10k / 100k / 1m traces;
- 4–64 commands per trace;
- 1 / 8 / 32 / 64 workers;
- all eight scenario families with balanced strata plus owner, amount, identity, fault and command-order mutations.

Report traces/sec, commands/sec, p50/p95/p99 latency, peak memory, oracle overhead, per-scenario disposition distribution, invariant coverage, mutation-kill rate and exact replay agreement. Every result binds the exact code SHA, input trace digest, oracle profile digest and fixture revision.

The next distinct stage is a deterministic command executor against the Rust/Go model, then delta-debugging shrinker. These are not claimed complete by this reference oracle tranche.

## Claim ceiling

A passing KAT and oracle suite proves only that the two reference interpreters agree on the frozen model's canonical dispositions and computed conservation vector. It does not prove:
- the system under test actually follows the oracle;
- production Holochain source-chain atomicity, fork exclusion or DHT convergence;
- durable outbox/recovery behavior;
- provider idempotency, authoritative absence or final settlement;
- real liquidity, collateral sufficiency, solvency, legal netting or regulatory compliance.

Production and provider qualification remain independent stages.
