# NCF-ALLOC-9 — checkpoint concurrency and ledger-consistency stress

**Status:** bounded local Python/SQLite stress completed; engine-level and production storage claims remain open.  
**Execution source SHA-256:** \`e429db1a25427a707be54778dd5c400a06534914114981e7504128a8dc54ee91\`  
**Git blob SHA (repository source):** \`24106e44118c0f607a4178f738103e3915e28ee6\` — verified byte-for-byte against the local source.  
**Environment:** Python 3.13.5 / SQLite 3.46.1 / Linux / one local filesystem.

## Paradigm shift

NCF-ALLOC-8 treated the application-process crash boundary as a critical failure mode. NCF-ALLOC-9 treats the storage lifecycle as a concurrent system: writers commit new obligations while readers take transaction-scoped snapshots and a separate thread repeatedly attempts PASSIVE, RESTART, and TRUNCATE checkpoints.

Each accepted write must remain in the ledger. Each reader's counter and obligation-sum queries must agree within one transaction. After all concurrent activity stops, the ledger must reconcile and a final TRUNCATE checkpoint must return \`[0, 0, 0]\`.

A checkpoint that is busy or only partially completes is not automatically corruption. The harness records intermediate busy/partial outcomes separately, continues checking safety, and applies a stronger final-quiescence checkpoint gate.

## Results

| Workload | Accepted writes | Checkpoint attempts | Busy/partial | Read snapshots | Consistent snapshots | Wall time | Accepted writes/s |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1,000 writes | 1,000/1,000 | 750 | 1 | 900 | 900/900 | 0.418 s | 2,395 |
| 8,000 writes | 8,000/8,000 | 2,500 | 70 | 4,000 | 4,000/4,000 | 3.436 s | 2,329 |
| 20,000 writes, run 1 | 20,000/20,000 | 5,000 | 1,443 | 6,000 | 6,000/6,000 | 6.234 s | 3,208 |
| 20,000 writes, run 2 | 20,000/20,000 | 5,000 | 375 | 6,000 | 6,000/6,000 | 5.816 s | 3,439 |

Every run recorded **zero invariant failures**. Each final state had \`used_units\` equal to the sum of unique obligation amounts, no duplicate work digests, no foreign-key violations, \`PRAGMA integrity_check = ok\`, and a successful final checkpoint result \`[0, 0, 0]\`.

The 20,000-write runs ended at 99,992 used units for 20,000 unique obligations. Write latency for 20k run 1 was p50/p95/p99 \`0.2370/0.4823/1.4555 ms\`; run 2 was \`0.2172/0.4524/1.1195 ms\`. Throughput varied between runs; publish the range and workload rather than a single headline number. These are Python + local SQLite results, not a financial-rail or distributed-database SOTA comparison.

Busy/partial checkpoint observations varied substantially between identical 20k runs (1,443 vs 375 out of 5,000). This is why checkpoint completion must be reported as its own metric, not interpreted as a corruption rate. The final quiescent checkpoint and ledger gates passed on both runs.

## Engine-level caveat: this is not the WAL-reset bug reproducer

This environment uses SQLite 3.46.1. SQLite's official WAL documentation says the WAL-reset bug affects versions from 3.7.0 through 3.51.2, was fixed in 3.51.3, and requires a narrow interleaving of multiple connections writing/checkpointing. SQLite's own developers state ordinary stress did not reproduce it organically and that special testing logic was needed. This NCF-ALLOC-9 campaign is **not** that low-level reproducer; zero observed failures here does not qualify the internal engine.

SQLite's official news page lists 3.53.3 as the latest release observed for this research date, 2026-10-28. Repeat on 3.53.3, or at minimum 3.51.3 / a verified vendor backport, then add the dedicated low-level regression approach before stronger WAL claims.

References:
- https://www.sqlite.org/wal.html
- https://sqlite.org/changes.html
- https://www.sqlite.org/transactional.html

## Limits

- No reproduction or certification of SQLite's internal WAL-reset race.
- No kernel crash, power-loss, storage-cache, filesystem-fault, or physical-device-fault injection.
- No Rust adapter, Holochain DHT, Tryorama multi-agent, multi-host, or production settlement qualification.
- A single-host Python+SQLite rate is not a like-for-like comparison with financial rails or distributed database SOTA.

## Next frontier

1. Pin SQLite 3.53.3 (or verify the exact vendor backport and source ID at runtime), then rerun.
2. Implement the dedicated low-level WAL-reset regression rather than assuming generic concurrent load can reveal it.
3. Run the same histories against the Rust storage adapter and Holochain/Tryorama multi-agent path.
4. Add injected commit ambiguity, disk-full, lock exhaustion, checkpoint interruption, and retry after later generations commit.
5. Scale to 100k and 1m obligations while recording ledger correctness, WAL growth, checkpoint progress, p99 latency and recovery time. Do not weaken safety gates to improve throughput.
