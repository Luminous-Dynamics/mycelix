# NCF-FEN-1 bounded formal model

This folder contains an **independent TLA+ abstraction** of the risk-right fencing lifecycle and a deliberately unsafe timeout-reallocation mutant. Neither has been executed in the current environment; treat them as model definitions, not proof receipts.

## Contents

- `NCF_FEN_1.tla`: small finite model of old-owner obligations/settlement, drain, evidence resolution, terminal-sequence reconciliation, fence, transfer and successor spending.
- `NCF_FEN_1.cfg`: bounded check with `Capacity = 2` and `MaxSteps = 4`.
- `NCF_FEN_1_TimeoutMutant.tla`: negative control that transfers capacity while merely `DRAIN_REQUESTED`, before evidence resolution/reconciliation/fencing.
- `NCF_FEN_1_TimeoutMutant.cfg`: **expected failure** on `NoTransferWithoutFence`.
- `NCF_FEN_1_StaleOwnerMutant.tla` and its config: a second negative control accepts an old-owner write after transfer; **expected failure** on `NoOldWriteOutsideActive`.

## Running TLC

Use a pinned, locally available TLA+ tools JAR and record its version/hash. Run from this directory so the mutant can resolve its extended module:

```sh
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1.cfg NCF_FEN_1.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1_TimeoutMutant.cfg NCF_FEN_1_TimeoutMutant.tla
```

The first command should terminate with no invariant violation for the configured finite model. The second should produce a counterexample to `NoTransferWithoutFence`. A third command must fail on `NoOldWriteOutsideActive`:

```sh
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1_StaleOwnerMutant.cfg NCF_FEN_1_StaleOwnerMutant.tla
```

These expected outcomes have **not yet been confirmed** by running TLC. The commands alone are not evidence.

The repository wrapper performs the paired safe/unsafe checks and rejects unexpected outcomes, including parser failures masquerading as a successful mutant test:

```sh
TLA2TOOLS_JAR=/absolute/path/to/tla2tools.jar \
  bash mycelix-workspace/simulations/nested-currency-reference/run-ncf-fen-tlc.sh
```

It requires a clean checkout, runs one safe model and both expected-failing mutants, writes raw logs and a receipt outside the repository, and binds them to the exact source/tree, all model/config digests and the JAR digest. The current environment has Java but no local `tla2tools.jar`; outbound DNS/download is unavailable, so this wrapper has not been executed here.

## Dedicated GitHub Actions qualification

The draft branch includes [`.github/workflows/ncf-fen-model-check.yml`](../../../../.github/workflows/ncf-fen-model-check.yml), independent of the generic workflow that excludes draft pull requests. It checks out the exact PR head/pushed commit, pins checkout / Java setup / artifact upload by full commit SHA, downloads a versioned TLA+ tools build, verifies SHA-256 before execution, runs the safe model and both expected-failing mutants, and retains raw logs plus the provenance receipt for 30 days.

The upstream TLA+ v1.8.0 tag is a rolling prerelease channel, not an immutable byte-level pin; the project has documented artifact changes without a version bump ([tlaplus/tlaplus#1399](https://github.com/tlaplus/tlaplus/issues/1399)). The workflow therefore uses the immutable mirror release `tla2tools-2026.08.11.125311`, which identifies its mirrored build and publishes SHA-256 `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f` ([mirror release](https://github.com/zactionsz/tla-tools/releases/tag/tla2tools-2026.08.11.125311)). The checksum is verified before running the model.

The workflow has been committed, but no completed result is asserted here. A job that is queued/running/skipped—or a log that only shows a parser failure—does not qualify the model. Qualification requires the safe model to complete with no invariant violations, the timeout mutant to fail specifically on `NoTransferWithoutFence`, and the stale-owner mutant to fail specifically on `NoOldWriteOutsideActive`.

## What this model captures

- Every accepted old-owner obligation is created only while `ACTIVE`; a stale write attempt after drain is represented as rejected.
- Reconciliation cannot proceed until evidence completeness is resolved and captures the exact old-operation sequence and outstanding/pending amounts.
- The fence binds to the reconciled terminal sequence.
- The successor gets no more than the proven free allocation; spend is bounded by both the successor slice and aggregate risk capacity.
- Old liabilities may settle after fencing/transfer, but settlement cannot reopen old-epoch exposure.
- Risk-right conservation and actual-exposure conservation are checked separately.

## What it deliberately does not capture

The model has one old allocation and one successor, integer capacities in a small finite domain, and a simplified completeness flag. It has no cryptographic receipt validation, real distributed control authority, quorum membership, DHT operations, independent counterparty histories, instrument/profile migration, multi-edge aggregate caps, external settlement, clock/lease semantics, or network stack. TLC's exhaustive result is only for this declared abstraction and finite configuration.

## Qualification discipline

1. Verify that the safe model has no invariant violation under the published bounds.
2. Verify that the timeout mutant is rejected by `NoTransferWithoutFence`; preserve the minimized counterexample trace.
3. Add at least one negative control per critical invariant and ensure each mutant is detected.
4. Cross-check generated transitions against the Rust reference model, then replay every counterexample as a Rust regression test.
5. Only after the abstract model is qualified, add bounded multi-owner/distributed schedules (or a Rust-native model such as Stateright) and real multi-agent/Holochain integration.
6. Record exact commit, model/config digests, tool versions, state/transition counts, invariant outcomes and raw counterexample digests. Keep defined, queued, failed, expected-negative and passed results distinct.

This follows the repository's model-check-first pattern in [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091) and complements the revision-fenced reservation boundary in [#5153](https://github.com/Luminous-Dynamics/mycelix/issues/5153).