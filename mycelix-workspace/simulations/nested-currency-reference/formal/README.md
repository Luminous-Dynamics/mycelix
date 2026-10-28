# NCF-FEN bounded formal models

This folder contains independent TLA+ abstractions of risk-right fencing, two-shard escrow, and transfer crash/recovery, plus deliberately unsafe mutants. These are model definitions until an exact-head qualification receipt confirms an actual TLC run; do not treat workflow definitions or expected mutant outcomes as proof receipts.

## Contents

- `NCF_FEN_1.tla`: small finite model of old-owner obligations/settlement, drain, evidence resolution, terminal-sequence reconciliation, fence, transfer and successor spending.
- `NCF_FEN_1.cfg`: bounded check with `Capacity = 2` and `MaxSteps = 4`.
- `NCF_FEN_1_TimeoutMutant.tla`: negative control that transfers capacity while merely `DRAIN_REQUESTED`, before evidence resolution/reconciliation/fencing; expected failure on `NoTransferWithoutFence`.
- `NCF_FEN_1_StaleOwnerMutant.tla` and its config: a stale old owner writes after transfer; expected failure on `NoOldWriteOutsideActive`.
- `NCF_ESCROW_1.tla` and config: two-shard escrow model with disjoint reservations, bounded local spending, and aggregate-cap invariants.
- `NCF_ESCROW_1_DuplicateAllocationMutant.tla` and config: gives a second shard the full budget while another shard already owns a slice; expected failure on `ReservedCapacityBound`.
- `NCF_ESCROW_1_PartitionResizeMutant.tla` and config: mutates a shard reservation while partitioned; expected failure on `PartitionedReservationsStayFixed`.
- `NCF_RECOVERY_1.tla` and config: transfer prepare/commit, abort before commit, explicit response loss, crash/restart, same-payload retry bound to the original transfer ID/receipt, conflicting-payload rejection, old-liability settlement, and successor spending. Small bounds are `Capacity = 2` and `MaxOps = 2`.
- `NCF_RECOVERY_1_CommitReplayMutant.tla` and config: deliberately credits the successor a second time after the commit response is lost; expected failure on `NoDoubleCommit`.
- `NCF_RECOVERY_1_ConflictingReplayMutant.tla` and config: deliberately accepts the same transfer ID with a changed amount payload after response loss; expected failure on `NoConflictingReplayAccepted`.

## Running the NCF-FEN models

Use a pinned, locally available TLA+ tools JAR and record its version/hash. Run from this directory so extended modules resolve:

```sh
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1.cfg NCF_FEN_1.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1_TimeoutMutant.cfg NCF_FEN_1_TimeoutMutant.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1_StaleOwnerMutant.cfg NCF_FEN_1_StaleOwnerMutant.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_ESCROW_1.cfg NCF_ESCROW_1.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_ESCROW_1_DuplicateAllocationMutant.cfg NCF_ESCROW_1_DuplicateAllocationMutant.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_ESCROW_1_PartitionResizeMutant.cfg NCF_ESCROW_1_PartitionResizeMutant.tla
```

The two safe models should terminate without invariant violations. The mutants should fail on their named invariants. Parser/tool errors are not successful detections.

The exact-head wrapper runs these safe models and all four original mutants. It requires a clean checkout and stores logs, metadata, and a receipt outside the repository:

```sh
TLA2TOOLS_JAR=/absolute/path/to/tla2tools.jar \
  bash mycelix-workspace/simulations/nested-currency-reference/run-ncf-fen-tlc.sh
```

The current environment previously lacked a local `tla2tools.jar` and outbound DNS was unavailable, so no local result is claimed. The pinned toolchain must be obtained and the exact-head workflow receipt inspected before treating this campaign as qualified.

## Running NCF-RECOVERY

`NCF_RECOVERY_1.tla` models a finite crash/restart system with an atomic durable commit boundary. Its safe configuration checks capacity conservation, old/new liability bounds, aggregate exposure, no successor spend before commit, a stable commit receipt, original-receipt retry behavior, conflicting-payload rejection, and the fence/reconciliation invariants. Explicit response states distinguish an unobserved commit response, a lost response, and a recovered/acknowledged response. A same-ID/same-payload retry returns the original receipt; a same-ID/different-payload retry is rejected.

Run the safe model and both deliberately unsafe retry mutants from this directory:

```sh
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_RECOVERY_1.cfg NCF_RECOVERY_1.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_RECOVERY_1_CommitReplayMutant.cfg NCF_RECOVERY_1_CommitReplayMutant.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_RECOVERY_1_ConflictingReplayMutant.cfg NCF_RECOVERY_1_ConflictingReplayMutant.tla
```

The commit-replay mutant must fail specifically on `NoDoubleCommit`; the conflicting-payload mutant must fail specifically on `NoConflictingReplayAccepted`. A generic nonzero exit does not count as successful mutant detection. The wrapper checks both distinctions and binds the logs/receipt to the exact source tree:

```sh
TLA2TOOLS_JAR=/absolute/path/to/tla2tools.jar \
  bash mycelix-workspace/simulations/nested-currency-reference/run-ncf-recovery-tlc.sh
```

The dedicated workflow [`.github/workflows/ncf-recovery-model-check.yml`](../../../../.github/workflows/ncf-recovery-model-check.yml) checks out the exact PR head, verifies the pinned TLA+ JAR by SHA-256, runs the safe model and both expected-failing retry mutants, and uploads logs and the receipt. It is a model qualification path, not proof that any current implementation has been qualified. The additional contract is documented in [NCF-FEN-2 recovery](../docs/NCF-FEN-2-recovery.md).

## Dedicated NCF-FEN Actions qualification

The original draft branch includes [`.github/workflows/ncf-fen-model-check.yml`](../../../../.github/workflows/ncf-fen-model-check.yml), independent of generic CI that excludes draft pull requests. It checks out the exact PR-head/pushed commit, pins checkout / Java setup / artifact upload by full commit SHA, downloads the immutable TLA+ tools mirror release, verifies SHA-256, runs the safe models and four expected-failing mutants, and retains raw logs plus a provenance receipt for 30 days.

The upstream TLA+ v1.8.0 tag is a rolling prerelease channel, not an immutable byte-level pin; the project documented artifact changes without a version bump ([tlaplus/tlaplus#1399](https://github.com/tlaplus/tlaplus/issues/1399)). The workflow uses mirror release `tla2tools-2026.08.11.125311` with SHA-256 `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f` ([release](https://github.com/zactionsz/tla-tools/releases/tag/tla2tools-2026.08.11.125311)).

A job that is queued/running/skipped—or a log that only shows parser failure—does not qualify the model. Qualification requires the safe models to complete with no invariant violations and each mutant to fail specifically on its target invariant. All results must be tied to the exact source SHA and reviewed receipt artifact.

## Model boundary and subsequent steps

These models do not implement cryptographic receipt validation, a real distributed control authority, quorum membership, DHT operations, multi-edge aggregate caps, real external settlement, or a production storage stack. TLC is exhaustive only over the declared finite abstraction and bounds. The recovery model assumes state transitions marked durable are atomic and survive restart; real backend-specific crash tests must establish whether that assumption holds.

The next steps after the bounded model passes are: compare every TLA+ transition with the Rust state-machine oracle; replay counterexamples as deterministic Rust regressions; exhaust small retry/receipt-reordering schedules; then run crash injection at every actual storage/Holochain commit boundary and a 100-contender last-unit race. A proposed target is 10,000 reproducible adversarial schedules with zero cap violations, double transfers, old-epoch writes, or conflicting idempotency acceptance. Those are qualification targets, not achieved results.

The Rust function `ten_thousand_generated_fencing_lifecycles_preserve_capacity_bound` is itself only a definition until an exact-head test receipt confirms it ran. Its generated sequential input sweep is not a concurrent schedule explorer. Keep model-checking, generated sequential tests, concurrency tests, and real multi-agent integration as separate evidence classes.

This follows the repository's model-check-first pattern in [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091) and complements the revision-fenced reservation boundary in [#5153](https://github.com/Luminous-Dynamics/mycelix/issues/5153).
