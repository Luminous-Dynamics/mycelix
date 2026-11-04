# NCF-ALLOC-8 — Crash-boundary and independent-process admission study

**Status:** bounded local research; not production qualification.  
**Scope:** file-backed SQLite WAL, synchronous=FULL, abrupt application-process exit, recovery/retry, and independent writer processes on one host.  
**Execution source SHA-256:** 2b479098258a3594fd0651b84ef32d3b55ef54ab5072d7914cf57a9b7297794d  
**Runtime:** Python 3.13.5 / SQLite 3.46.1 / Linux.

## Why this increment matters

NCF-ALLOC-7 showed a transaction-based local baseline under threads. NCF-ALLOC-8 shifts the boundary from “the database returned the right answer” to “the service dies at the worst point and recovers without creating a partial financial fact.” It adds independent OS processes, an explicit start barrier, os._exit kill points inside a transaction, ledger re-open, invariant checks, and retries after recovery.

The intended atomicity contract is simple: a new obligation, request ID, cap debit, and receipt are one commit unit. After a process crash before COMMIT, reopening should observe none of them; after COMMIT but before the caller receives the acknowledgement, reopening should observe all of them and an identical retry must return the original receipt without a second debit.

## Local results

The checked-in campaign used six crash boundaries, three repetitions each (18 crash-injection trials), and three independent-process race cases, three trials each (9 paired race trials, 18 writer processes). It observed **zero invariant failures** in those bounded runs.

| Crash boundary | Expected state after reopening, before retry | Result |
|---|---|---|
| After BEGIN IMMEDIATE | No committed obligation | 3/3 matched |
| After canonical-work insert | No committed obligation | 3/3 matched |
| After request-record insert | No committed obligation | 3/3 matched |
| After cap update | No committed obligation | 3/3 matched |
| Immediately before COMMIT | No committed obligation | 3/3 matched |
| After COMMIT, before acknowledgement | Exactly one complete obligation | 3/3 matched |

Every case then retried using a fresh connection; pre-commit crashes accepted the claim exactly once, while post-commit/pre-ack crashes replayed the stored receipt with no extra debit.

| Independent-process race | Expected outcome | Result |
|---|---|---|
| Two distinct 60-unit claims against cap 100 | One accept, one cap rejection, usage 60 | 3/3 matched |
| Same canonical work, different request IDs | One accept, one replay, one debit | 3/3 matched |
| Same request ID, conflicting work digests | One payload binds; other rejects | 3/3 matched |

The machine-readable record stores each case, observed state, environment, source hash, failure count, and claim ceiling. Six embedded unit tests also pass locally. The small run is a smoke/fault-model result, not a reliability probability estimate.

## Important qualification caveat found during research

The local Python SQLite library is **3.46.1**. SQLite's official WAL documentation records a rare WAL-reset race affecting versions from 3.7.0 through 3.51.2, fixed in 3.51.3, with backports listed for some older release lines. The issue requires a narrow interaction among multiple connections, checkpointing, and a WAL reset; this experiment does not deliberately reproduce that internal SQLite bug. Therefore these results must not be used to certify this runtime's WAL/checkpoint behavior. Before any stronger SQLite claim, repeat on SQLite 3.54.1 or later (at minimum 3.51.3 or an explicitly verified vendor backport) and add adversarial checkpoint/reopen stress. See https://www.sqlite.org/wal.html.

SQLite's own transactional documentation states that it tests atomicity under program crashes, OS crashes and power failure using a special harness: https://www.sqlite.org/transactional.html. This work does **not** duplicate that harness. os._exit tests application-process death only. It cannot simulate kernel panic, sudden power loss, lying storage caches, filesystem corruption, or physical-device faults.

## Design and evidence limits

- SQLite serializes writers on a single host; WAL does not operate over a network filesystem and is not a multi-host database protocol. See https://www.sqlite.org/isolation.html and https://www.sqlite.org/wal.html.
- This is not a Holochain multi-agent test. Holochain's validation guidance says validation runs over individual DHT operations and multiple actions written in an atomic transaction are not validated together as one atomic transaction. Tryorama multi-agent scenarios are still required: https://developer.holochain.org/build/validation/.
- The supplied digest is a deterministic test input, not proof that the claimed work occurred. No cryptographic verifier, identity/Sybil defense, legal obligation, FX conversion, netting policy, or external settlement rail is qualified.
- A follow-up 5 × 10 campaign completed against this exact v1 source SHA: 30 crash injections and 30 independent-process race pairs, zero observed invariant failures, 45.6385 seconds recorded elapsed time. The terminal tool returned a timeout even though the output artifacts were fully written; the JSON was inspected and its source SHA matches v1. See `ncf-alloc8-crash-results-5x10.json`. The exact v1 source is preserved as `ncf_alloc8_crash_consistency_v1_original.py` (Git blob `9c17eff8301e5c58495c6b1047ba49f39e95bedb`); the mutable `ncf_alloc8_crash_consistency.py` path later received a default-parameter edit and should not be used to re-derive the historical source hash. This was not evidence of a reproduced worker deadlock.
- A static review found implicit SQLite connection cleanup in the original harness. A resource-safe v2 source adds explicit `closing(...)` around the three connection lifetimes. Its six unit tests passed with `ResourceWarning` promoted to error, and its own fresh 3 × 3 campaign is recorded separately in `ncf-alloc8-crash-results-fixed-source.json`. The old v1 result remains bound to the v1 SHA.\n- All claims remain local Python/SQLite reference research. No Rust, Holochain, multi-host, kernel-crash, power-loss, or production receipt is claimed.

## Next frontier

1. Distinguish a command/tool timeout from a completed campaign artifact by checking exit state, output completeness, and source hash.
2. Re-run on a current SQLite runtime (minimum 3.51.3 for the documented WAL-reset fix, or a verified backport).
3. Add checkpoint concurrency and recovery under WAL growth; validate integrity_check, used-cap ledger, canonical-work uniqueness, request binding, and replay receipt after each process restart.
4. Add a storage-adapter contract suite and execute the same abstract histories against the Rust implementation, SQLite adapter, and Holochain/Tryorama multi-agent topology.
5. Model unknown commit outcome explicitly: after a timeout, the caller must query/retry by idempotency key, never infer rejection from a missing acknowledgement.

## References

- SQLite — Transactional behavior: https://www.sqlite.org/transactional.html
- SQLite — Write-ahead logging and documented WAL-reset bug: https://www.sqlite.org/wal.html
- SQLite — Isolation and single-writer serialization: https://www.sqlite.org/isolation.html
- Holochain — Validation and multi-agent testing considerations: https://developer.holochain.org/build/validation/
