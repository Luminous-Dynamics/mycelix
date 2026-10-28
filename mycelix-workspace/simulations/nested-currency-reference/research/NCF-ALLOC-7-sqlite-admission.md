# NCF-ALLOC-7 — Durable SQLite admission baseline

**Status:** seven local Python tests passed against a file-backed SQLite database; 150 concurrent pair trials and a 2,000-attempt microbenchmark ran locally. The checked-in source is a condensed, semantically synchronized copy and is not byte-identical to the locally executed script. This is not Rust, Holochain, multi-host or production evidence.

## Why add a real storage baseline?

NCF-ALLOC-6 found the race class in a finite abstract scheduler. NCF-ALLOC-7 asks a narrower but stronger question: can a concrete local storage strategy enforce the desired invariants under real competing connections?

The research implementation uses SQLite WAL, synchronous FULL, BEGIN IMMEDIATE, a unique request-ID table, a unique canonical-work digest table, a principal/instrument cap row, and durable receipt JSON. Deduplication, the cap check/debit, accepted work, and response persistence occur within a single write transaction. Each request ID records the verifier version and receipt digest accepted for that ID. Canonical aliases return the original work receipt while preserving their own evidence binding for subsequent retries.

The verifier flag remains a trusted test input. No signatures, principals or real-world obligations are authenticated here. The model uses concurrent local threads and independent database connections; it does not simulate separate hosts or Holochain.

## Local results

The seven-test suite passed on 2026-10-28. Three concurrent race classes ran for 50 trials each:

| Concurrent workload | Trials | Accepted | Replay | Rejected | Cap/integrity failures |
|---|---:|---:|---:|---:|---:|
| Two distinct 60-unit claims, cap 100 | 50 | 50 | 0 | 50 | **0** |
| Same work, distinct request IDs | 50 | 50 | 50 | 0 | **0** |
| Same request ID, conflicting payloads | 50 | 50 | 0 | 50 | **0** |

The “accepted” and “rejected” counts refer to individual request outcomes across each paired trial. In the cap case, one 60-unit request wins and the other is rejected; the cap is never exceeded. In the same-work case, the second request is an idempotent replay, and the cap is debited once. For a conflicting ID, only one payload can bind the request ID. Across all 150 trials, **zero modeled invariant failures were observed**.

A separate 2,000-attempt run contained 1,000 unique obligations and 1,000 exact replays:

| Metric | Local observation |
|---|---:|
| Admission attempts | 2,000 |
| Accepted unique obligations | 1,000 |
| Exact replays | 1,000 |
| Throughput | 1,098.59 attempts/s |
| Local transaction latency p50 | 1.1528 ms |
| Local transaction latency p95 | 1.8039 ms |
| Local transaction latency p99 | 2.1586 ms |

Environment: Python 3.13.5, SQLite 3.46.1, Linux x86-64; file-backed WAL and synchronous FULL. The script uses a separate SQLite transaction/connection path per request, so these timings include Python and local database overhead. They are not a comparison to networked settlement systems and do not predict throughput in a multi-agent deployment. The full measurement record is in ncf-alloc7-sqlite-results.json; source-hash provenance explicitly records that the checked-in source differs from the local execution bytes.

## What this does and does not demonstrate

It provides a concrete local baseline that avoids the abstract model's stale-read pattern by serializing writers and persisting idempotency state. It does **not** prove this exact schema is correct under every storage fault, that SQLite's configuration maps to every durability contract, or that a production Holochain path can enforce a global unique key. It also does not test file-system or power-loss recovery, network partitions, multi-host writes, verifier compromise, key management, or source-chain/DHT behavior.

## Industry comparison and qualification goals

- [Interledger Settlement Engines RFC](https://interledger.org/developers/rfcs/settlement-engines/) requires persisted idempotency keys and response state, with retry semantics that avoid duplicate effects. NCF should declare and test its own replay-retention policy, rather than assume an ID can be forgotten after an arbitrary interval.
- [Holochain validation guidance](https://developer.holochain.org/build/validation/) stresses deterministic validation and notes that actions inside a local atomic write are not validated together as a cross-agent atomic transaction. A local database invariant cannot be assumed to become a global DHT uniqueness invariant.
- [BIS Project Agorá](https://www.bis.org/project/agora) reports a July 2026 real-value testing phase spanning 17 scenarios and approximately CHF 800,000, with an average around 80 seconds from initiation to settlement in that controlled prototype. That is an end-to-end workflow result, not a same-class benchmark against this local admission function.
- [BIS Project FuSSE](https://www.bis.org/publications/project-fusse-exploring-flexible-scalable-and-secure-settlement-engines) describes 10,000 TPS in controlled proof-of-concept testing, while explicitly stating its result is not a general performance benchmark or implementation reference. NCF must not claim parity from a local 1.1k attempts/s microbenchmark.

Next hard gates for the actual implementation:
1. Run NCF-ALLOC-7 histories against the real Rust admission path and durable storage adapter.
2. Test crash after commit but before acknowledgment, restart, same-ID retry and conflicting-ID retry; require exact original receipt and zero duplicate debit.
3. Repeat the racing tests at higher contention (2, 8, 32, 128 writers), with same-principal hot spots, distinct principals, replay storms and cap boundaries.
4. Run the same histories against the real Holochain/multi-agent path. Use explicit fencing/preallocated rights if global serializability is not available.
5. Target **zero** duplicate canonical-work acceptances, cap overshoots, conflicting-ID acceptance, or lost acknowledged obligations. Only after these pass, set/measure the performance frontier: initially 1,000 accepted local operations/s for 30 minutes and p99 local admission below 100 ms on a declared testbed; publish full-path measurements separately from local admission.

The novelty claim is not “SQLite makes a settlement network safe.” The contribution is a repeatable storage baseline that exercises the concrete safety invariants, preserves the claim boundary, and gives the Rust/distributed implementation a set of real race histories to match.
