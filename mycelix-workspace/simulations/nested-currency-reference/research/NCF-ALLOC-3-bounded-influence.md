# NCF-ALLOC-3 — Bounded-influence demand allocation

**Status:** Rust policy and regression-test definitions added on the Mycelix branch. The independent Python model and randomized sweep have executed locally; Rust compilation, TLC, CI qualification, distributed behavior and production use remain unverified.

## Research question

How sensitive is capacity placement to false self-reported demand, and what is the service cost of limiting that influence for legitimate hotspots?

The allocator now includes DemandWeightedWithEqualFloorAndMedianCap with a declared equal-floor fraction and a cap multiple applied to the lower median of historical mean-demand weights. The matched-trace Rust benchmark has eight policies and should emit sixteen reports (eight policies × two demand modes).

## Independent model

The Python research harness uses 16 stable shard IDs, 100,000 total capacity units, a 5% reserve and a fixed seed. It compares equal placement, raw weighted allocation, a 50% equal floor, 2×/4× lower-median caps, and an idealized true-backlog oracle. The randomized study includes 20,000 cases where 1–4 of 16 shards inflate claims 2–20× while actual backlog does not change. It deliberately does not create Sybil identities, and the honest-hotspot case assumes fresh rights each epoch; it does not model partitions or outstanding obligations.

Run:

    python3 -m unittest -v test_ncf_alloc3.py
    python3 ncf_alloc3.py --output ncf-alloc3-results.json --sweep-cases 20000

The output file retains the exact script SHA-256, seed, profile, sweep digest, policy metrics and an explicit claim ceiling. The Python harness is a research reference, not production code and not an alternative to Rust qualification.

## Results

| Scenario / policy | Demand served | Worst-shard service | Jain fairness | Stranded units | Attacker share |
|---|---:|---:|---:|---:|---:|
| Honest rotating 4× hotspot — raw weighted | 84.82% | 84.82% | 100.00% | 0 | n/a |
| Honest rotating 4× hotspot — equal floor 50%, 2× cap | 70.42% | 48.24% | 93.78% | 16,125 | n/a |
| Honest rotating 4× hotspot — equal floor 50%, 4× cap | 77.70% | 60.97% | 96.61% | 7,981 | n/a |
| Randomized claim inflation — raw weighted | 40.13% | 28.38% | 68.21% | 28,440 | 57.31% |
| Randomized claim inflation — equal floor 50%, raw claims | 50.31% | 30.17% | 82.71% | 11,591 | 36.44% |
| Randomized claim inflation — equal floor 50%, 2× cap | **54.10%** | **39.87%** | **89.74%** | **5,505** | **21.41%** |
| Randomized claim inflation — equal floor 50%, 4× cap | 53.06% | 36.58% | 87.60% | 7,180 | 27.31% |
| Randomized claim inflation — true-backlog oracle | 55.61% | 44.59% | 91.82% | 3,113 | 15.57% |

The results support the bounded-influence hypothesis for this workload, not a universal winner. The tighter cap improves the declared minority attack, but clips legitimate hotspots. The oracle uses trusted true backlog and is not a deployable result unless that signal can be independently verified.

## Core paradigm shift: evidence-backed demand

Median capping is not authentication, Sybil resistance or a fairness theorem. It assumes stable shard identity and a minority of claimants; it fails if an attacker controls enough identities or if honest demand is very skewed. Prefer a typed DemandEvidence record carrying canonical instrument/shard/epoch IDs, a digest of eligible deduplicated queued obligations, a verifier/version, monotonic sequence and replay policy, claimed and independently derived amounts, reason codes and a reproducible audit receipt.

Keep safety, liveness and allocation quality separate. Forecasts must never authorize spending or resize active rights during a partition. Any capacity transfer requires its own fencing and reconciliation proof.

## Qualification targets

- **Correctness:** zero capacity-conservation errors, negative allocations, double allocation or cap violations; Rust unit/property tests and 10,000+ deterministic generated cases.
- **Adversarial demand:** multi-seed attacks with 1/4/8/15 attacker identities, identity churn, honest hotspots, forecast error, and true-backlog upper bound. Report per-shard service and attacker allocation, not aggregate utilization alone.
- **Distributed safety:** zero lost acknowledged obligations and zero duplicate final settlement under retries, crashes, partitions and reconnect. Use model checking, fault-injection histories and a multi-agent runtime separately.
- **Performance target (proposed):** after safety qualification, sustain 1,000 accepted local operations/s for 30 minutes on a declared 16-shard testbed with p99 local admission below 100 ms. Treat 6,000 end-to-end settlement transactions/s as a stretch goal and 10,000/s as a later investigation, not achieved results.

## Industry comparison and claim ceiling

BIS Project FuSSE describes a 10,000 TPS controlled proof of concept and explicitly does not present it as a general performance benchmark. Icon Solutions/MongoDB published a 6,000 end-to-end payment/s result with 12 processing steps in September 2026; it is vendor-published and requires a matched environment before direct comparison. These full-system numbers are not comparable to this single-process allocator. Use fault-injection history checking (Jepsen-style) as a correctness campaign and a separate controlled performance campaign. Holochain's validation model also distinguishes local atomic writes from cross-agent atomic validation.

The contribution to test is a measurable safety × fairness × manipulation-resistance × cost frontier, not a claim of state-of-the-art throughput. Python results are checked into research/ncf-alloc3-results.json; the Rust policy, test definitions and 16-report benchmark need an exact-head Rust receipt before they can be described as executed or passing.
