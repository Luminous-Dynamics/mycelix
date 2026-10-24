# NCF-FEN-1 bounded formal model

This folder contains an **independent TLA+ abstraction** of the risk-right fencing lifecycle and a deliberately unsafe timeout-reallocation mutant. Neither has been executed in the current environment; treat them as model definitions, not proof receipts.

## Contents

- `NCF_FEN_1.tla`: small finite model of old-owner obligations/settlement, drain, evidence resolution, terminal-sequence reconciliation, fence, transfer and successor spending.
- `NCF_FEN_1.cfg`: bounded check with `Capacity = 2` and `MaxSteps = 4`.
- `NCF_FEN_1_TimeoutMutant.tla`: negative control that transfers capacity while merely `DRAIN_REQUESTED`, before evidence resolution/reconciliation/fencing.
- `NCF_FEN_1_TimeoutMutant.cfg`: checks the same invariants against the unsafe mutant. **Expected result: fail** on `NoTransferWithoutFence`. A passing mutant is a test-harness/semantic failure, not success.

## Running TLC

Use a pinned, locally available TLA+ tools JAR and record its version/hash. Run from this directory so the mutant can resolve its extended module:

```sh
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1.cfg NCF_FEN_1.tla
java -cp /path/to/tla2tools.jar tlc2.TLC -config NCF_FEN_1_TimeoutMutant.cfg NCF_FEN_1_TimeoutMutant.tla
```

The first command should terminate with no invariant violation for the configured finite model. The second should produce a counterexample to `NoTransferWithoutFence`. These expected outcomes have **not yet been confirmed** by running TLC. The commands alone are not evidence.

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