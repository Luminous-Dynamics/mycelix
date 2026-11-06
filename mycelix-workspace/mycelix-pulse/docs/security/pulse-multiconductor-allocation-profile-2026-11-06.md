# Pulse multi-conductor auditor — phase allocation profile

**Result:** PASS for the instrumented synthetic structural-history benchmark  
**Workflow:** [Pulse Multi-Conductor Causal History Audit, run #97](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42161357203)  
**Benchmark artifact:** [raw reports and allocation measurements](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42161357203/artifacts/13476807632), ID `13476807632`  
**Audit artifact:** [test and safe-history receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42161357203/artifacts/13477022424), ID `13477022424`  
**PR head at run:** `f6076c297ce1465ce0d9356a12d838bd4cf414e4`  
**Merge checkout SHA in benchmark receipt:** `858d73ac9ac5d89e5680568fc431c2ba40731084`  
**Rust / Cargo:** 1.96.0  
**Cargo.lock SHA-256:** `6e74c464cb7c89ff6e96914ec671a4fc2196c0164da3f28bd245b757baf16eaf`  
**Instrumented verifier source SHA-256:** `15d91d593e1239958b77ae9ac0f91851702cb3c277321d3d319d0a464a9210b0`  
**Raw results JSONL SHA-256:** `f241fb808d3a03b12a5a97d89f5b98eb627c1f5d9b130713750b1543889933d7`  
**Summary JSON SHA-256:** `63b29ccadc5690b8c32e7cde73781867a6dc4e70bc3944f54025f001d436c338`  
**Archive SHA-256:** `ec0eb46f1fb12920e3ee924678ce47c2e2c766bd7f2f711c71d5163f4c60c71e`

## Method

The benchmark uses the same deterministic two-conductor synthetic safe workload as the prior baseline: independent local sequence counters, unverified remote-signal events, and no cross-conductor causal edges. It runs 30 independent processes at 1k, 10k and 100k events, plus one 1M capacity probe. Every run returned PASS.

A benchmark-only global counting allocator records requested allocation bytes, live bytes after history build and audit, and phase-specific live-memory high-water marks. Atomic accounting adds overhead to alloc/dealloc operations; its throughput figures are therefore **not directly comparable** with the earlier uninstrumented baseline. Process peak RSS is still captured separately with `/usr/bin/time`.

## Phase allocation results

| Events | Processes | History live after build (median) | Build requested allocations (median) | Audit additional peak live bytes (median) | Audit requested allocations (median) | Process max RSS |
|---:|---:|---:|---:|---:|---:|---:|
| 1,000 | 30 | 2.53 MB | 2.53 MB | 0.68 MB | 0.78 MB | 6,280 KiB |
| 10,000 | 30 | 25.33 MB | 25.33 MB | 6.85 MB | 7.98 MB | 38,408 KiB |
| 100,000 | 30 | 253.30 MB | 253.30 MB | 68.22 MB | 79.58 MB | 360,580 KiB |
| 1,000,000 | 1 | 2.533 GB | 2.533 GB | 680.09 MB | 777.71 MB | 3,582,372 KiB |

Values above are decimal MB/GB for allocation bytes; RSS is the workflow's KiB measurement. The 1M sample is one capacity probe, not a percentile population.

At 1M events:
- build leaves `2,533,001,449` live bytes;
- audit raises the live high-water mark to `3,213,091,791` bytes;
- incremental audit peak over the pre-audit live baseline is `680,090,096` bytes;
- audit requests `777,708,873` bytes cumulatively, some of which are later freed;
- external peak RSS is `3,582,372 KiB`.

The 30-run populations at 1k, 10k and 100k are deterministic enough that allocation phase medians and maxima differ by only a few bytes per process. The largest growth is linear with event count in this fixture.

## Interpretation

The current `History` representation is a fully owned `Vec<Event>` with each `Event.data` stored as a nested `serde_json::Value`. In this synthetic remote-signal workload, history construction alone accounts for roughly 2,533 bytes/event. Audit-time indexes and semantic state add roughly 680 bytes/event at peak. The first optimization should therefore focus on per-event representation and ownership; graph/index work is the second target.

A useful optimization sequence is:

1. Replace general-purpose per-event JSON maps with closed typed event variants for the six-plane schema, while retaining strict unknown-field rejection.
2. Intern repeated conductor IDs, source strings, event types, trust-class values and other vocabulary; use compact integer IDs in indexes.
3. Prototype borrowed `RawValue` or field-level borrowed parsing for the on-disk path, retaining only typed join facts after validation.
4. Keep event IDs/action hashes and causal adjacency exact; do not discard evidence references or collapse cross-conductor order into wall-clock timestamps.
5. Differentially compare the optimized auditor against the current materialized verifier on all safe fixtures and all mutation tests.

## Claim ceiling and limitations

- The benchmark is synthetic and uses unverified remote-signal events only; no cross-conductor causal edges are present.
- It does not measure Holochain conductor throughput, DHT gossip/convergence, validation cost, network partitions, runtime authorization latency, or dense/hostile causal histories.
- Allocator counters are instrumented application allocations, not a replacement for OS RSS. They do not account for allocator fragmentation or all runtime/kernel memory.
- The new benchmark's throughput includes atomic instrumentation overhead and must not be compared directly with the earlier uninstrumented throughput baseline.
- The 1M sample is one capacity probe. No percentile is claimed.
- These measurements do not show an optimization yet; they identify the current memory contributors.

Optimization target and differential correctness gates are tracked in [#5872](https://github.com/Luminous-Dynamics/mycelix/issues/5872).
