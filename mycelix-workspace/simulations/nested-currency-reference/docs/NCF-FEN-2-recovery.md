# NCF-FEN-2: Crash/recovery and idempotent transfer commit

**Status:** bounded formal model added; TLC qualification pending. This is a follow-on to NCF-FEN-1 and FED-CUR-002, not a production fencing implementation.

## Why this is a separate gate

NCF-FEN-1 models drain, reconciliation, fencing, and transfer ordering. That does not by itself qualify the storage boundary at which a transfer becomes durable. A practical failure case is: the transfer commit succeeds, its response is lost, and the caller retries. A retry must return the original result; it must not credit a second successor allocation. Another failure case is a crash after prepare but before commit. Prepared intent must not grant spend authority, and restart must not create two independently spendable rights.

The new bounded model in `formal/NCF_RECOVERY_1.tla` explicitly explores old-owner pending exposure, reconciliation, fencing, transfer preparation, commit, abort-before-commit, old-liability settlement, successor spend, crash, restart, and committed-operation retry. Its deliberate negative control in `formal/NCF_RECOVERY_1_CommitReplayMutant.tla` repeats successor credit after commit and must violate `NoDoubleCommit`.

## Model contract

The model assumes its declared state transitions are durable and atomic at the model boundary. Crash/restart preserves those state variables. It does **not** show that any production database, Holochain source chain, DHT, quorum, or RPC has that property; the implementation-specific durability contract must be separately tested.

Safety invariants checked by the safe config:

- `CapacityConserved`: old and successor spend rights do not allocate more than the original cap.
- `OldLiabilityBound` and `NewLiabilityBound`: outstanding/pending obligations remain within their corresponding rights.
- `AggregateOutstandingBound`: combined old and successor exposure stays within the cap.
- `NoSuccessorSpendBeforeCommit`: a successor can spend only after the transfer commit is recorded.
- `NoDoubleCommit`: one logical transfer cannot create multiple commits.
- `FenceBeforeSuccessorActivation`: preparation/activation requires the old epoch to be fenced.
- `ReconciliationCutStable`: the old-operation sequence does not move behind a recorded terminal cut.
- `PreparedTransferWithinVerifiedRemainder`: prepared transfer is no greater than the verified remainder.
- `TypeOK`: state stays within the finite model domains.

The negative-control config intentionally checks `NoDoubleCommit` only. A nonzero TLC exit is not enough: the runner must find the expected invariant violation in the trace log. Parser, module-resolution, timeout, or generic tool errors must fail qualification rather than count as mutant detection.

## Reproducible qualification

The dedicated `.github/workflows/ncf-recovery-model-check.yml` workflow checks out the exact PR-head SHA, verifies the pinned TLA+ tools JAR with SHA-256, runs the safe model and expected-failing mutant, and uploads raw logs plus a provenance receipt. The runner records the source commit/tree, model and config digests, tool digest/version, state-statistics line, log hashes, and expected invariant name. Results are pending until a completed exact-head workflow artifact confirms both safe-model success and the named mutant counterexample.

## Next benchmark gates

1. Explore at least 10,000 reproducible crash/restart/retry schedules after the finite model and transition oracle are qualified. Include crashes before and after every durable transition, lost commit responses, duplicate/reordered control receipts, and retry of identical versus conflicting payloads.
2. Inject crash after every real storage write / Holochain commit boundary; compare recovered state to the abstract model's allowed states.
3. Run a real 100-contender last-unit race using the chosen serialization authority; at most one transfer may commit.
4. Report capacity used versus stranded, recovery latency, unresolved operations, retries, rejected conflicting IDs, CPU, memory, and p50/p95/p99 latency beside the safety outcomes.
5. Preserve the static-quota baseline. Compare drain-acknowledged and any linearizable/quorum authority only under the same traces and fault model.

## Research boundary

This track adapts established idempotent settlement principles and formal fault-model practice; it does not claim to invent them. Interledger's settlement-engine guidance requires idempotency keys for settlement side effects and stable retry behavior ([Interledger](https://interledger.org/developers/rfcs/settlement-engines/)). Holochain validation is deterministic, may return unresolved dependencies, and does not validate all actions in a local source-chain transaction as one atomic cross-agent transaction ([Holochain validation guide](https://developer.holochain.org/build/validation/)). Risk management and finality also need clear operational and legal boundaries ([CPMI-IOSCO PFMI](https://www.bis.org/committees/cpmi/pfmi/overview)).

**Claim ceiling:** a bounded TLA+ result over this abstract state machine only. It is not evidence of production Holochain behavior, cryptographic authorization, legal settlement finality, an unbounded proof, or a measured performance win.
