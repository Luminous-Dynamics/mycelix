# NCF-ALLOC-9 — checkpoint concurrency and ledger-consistency stress

**Status:** bounded local Python/SQLite stress completed; engine-level and production storage claims remain open.  
**Execution source SHA-256:** `534ba9ac776e085b632f453e6cb9282b12c9d8e60f93501146c62a5d7191eeff`  
**Git blob SHA:** `436e58d6a48f9be58fb11602069c6c169f035161` — verified byte-for-byte against the repository source.  
**Environment:** Python 3.13.5 / SQLite 3.46.1 / Linux / one local filesystem.

## Paradigm shift

NCF-ALLOC-8 tested application-process crash boundaries. NCF-ALLOC-9 treats the storage lifecycle as a concurrent system: writers commit obligations while readers hold transaction-scoped snapshots and a separate thread repeatedly attempts PASSIVE, RESTART, and TRUNCATE checkpoints.

The harness records the linked runtime's `sqlite_source_id()` and supports `--require-fixed-sqlite`. The gate was executed in this environment and correctly returned exit code 2 / `NOT_QUALIFIED` because the linked SQLite runtime is 3.46.1, below the minimum fixed version 3.51.3. Application stress remains runnable for research, but cannot silently claim fixed-engine qualification.

## Results

| Workload | Accepted writes | Checkpoint attempts | Busy/partial | Consistent read snapshots | Wall time | Writes/s |
|---|---:|---:|---:|---:|---:|---:|
| 1,000 writes | 1,000/1,000 | 750 | 1 | 900/900 | 0.5748 s | 1,739.7 |
| 8,000 writes | 8,000/8,000 | 2,500 | 6 | 4,000/4,000 | 2.7445 s | 2,914.9 |
| 20,000 writes, run 1 | 20,000/20,000 | 5,000 | 6 | 6,000/6,000 | 5.9192 s | 3,378.8 |
| 20,000 writes, run 2 | 20,000/20,000 | 5,000 | 323 | 6,000/6,000 | 6.8891 s | 2,903.1 |

All four runs recorded **zero invariant failures**. Every accepted write remained represented, every read snapshot observed the counter equal to aggregate obligation amount, and the final state reconciled used units with unique obligations. Each final state had no duplicate work digests, passed `PRAGMA integrity_check`, had zero foreign-key violations, and returned `[0,0,0]` on the final quiescent checkpoint.

The two 20k runs ended at 99,992 used units for 20,000 unique obligations. Write p50/p95/p99 latency was 0.2239/0.4683/0.9548 ms for run 1 and 0.2531/0.5517/2.2015 ms for run 2. Identical workloads show variability, so these rates are local experimental measurements—not a production or distributed-SOTA claim.

Busy/partial checkpoint observations varied substantially between repeated runs. They are checkpoint-progress outcomes, not corruption counts. The deterministic held-reader test starts a read snapshot, commits a writer on another connection, confirms a TRUNCATE checkpoint returns busy/partial, releases the reader, and then requires the final checkpoint to succeed.

## The engine-level gate remains open

SQLite's official WAL documentation says a rare WAL-reset bug affects WAL-mode databases on versions through 3.51.2. It requires a narrow race between at least two connections in threads/processes which write/checkpoint simultaneously; developers needed special test logic to trigger it, so ordinary load that does not reproduce it is not proof that it is absent. See [SQLite WAL-reset bug documentation](https://www.sqlite.org/wal.html).

As of **28 October 2026**, the latest official SQLite release is **3.54.1**, released on 16 October 2026. The fix is present from 3.51.3 onward; SQLite also lists some older backports. This environment's SQLite 3.46.1 fails the explicit version gate, and no fixed-engine PASS is claimed. See [SQLite release history](https://sqlite.org/changes.html) and [SQLite 3.54.1 release notes](https://www.sqlite.org/releaselog/3_54_1.html).

This campaign is **not** a reproducer for the low-level WAL-reset bug. It also does not simulate kernel panic, power loss, lying storage caches, filesystem faults, or physical-device faults. SQLite's own [transactional behavior page](https://www.sqlite.org/transactional.html) describes its special testing approach for OS/power-failure behavior; these application-level tests do not duplicate it.

## Industry comparison and meaningful target

The current rates (roughly 1.7k–3.4k accepted writes/s in this host-level Python harness) are not comparable to production financial rails or distributed settlement engines. BIS Project FuSSE reported 10,000 TPS in a controlled proof of concept, but BIS explicitly describes it as architecture exploration, not a performance benchmark or a production-ready implementation. See [BIS Project FuSSE](https://www.bis.org/publications/project-fusse-exploring-flexible-scalable-and-secure-settlement-engines).

The relevant immediate objective is **zero safety violations under independently reproducible fault injection**, with exact commit/retry semantics and exact source/runtime receipts. Then we can compare end-to-end throughput, p99 latency, recovery time, storage overhead, and safety outcomes against matched transaction semantics in Rust and Holochain multi-agent topologies. That is a defensible SOTA pursuit; a standalone TPS number is not.

## Next steps

1. Pin SQLite 3.54.1 (or at least 3.51.3 / a verified backport) and capture the exact runtime source ID.
2. Run SQLite's dedicated low-level WAL-reset regression; do not substitute generic stress.
3. Run the same operation histories against the Rust storage adapter and Holochain/Tryorama multi-agent topology.
4. Inject ambiguous commit, disk-full, lock exhaustion, checkpoint interruption, and retry after later generations commit.
5. Scale to 100k and 1m obligations while preserving all safety gates; never weaken correctness to optimize throughput.

## Evidence

- Source: `ncf_alloc9_checkpoint_stress.py`
- Summary: `ncf-alloc9-checkpoint-results-summary.json`
- Campaign: `ncf-alloc9-checkpoint-campaign.json`
- Test record: `NCF-ALLOC-9-TEST-RESULTS.txt`
- Combined local suite: 11 tests pass with `ResourceWarning` promoted to error.
