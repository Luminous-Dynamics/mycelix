# Pulse multi-conductor auditor — borrowed parser microbenchmark receipt

**Workflow:** [Pulse Multi-Conductor Causal History Audit, run #166](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42215970824)  
**Parser artifacts:** [raw owned-vs-borrowed runs](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42215970824/artifacts/13501252101), ID `13501252101`  
**Audit artifact:** [test and safe-history receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42215970824/artifacts/13500828166), ID `13500828166`  
**Exact PR head:** `8b9a2de05c1c562e6f6135fc2645679e756a6cff`  
**Merge checkout:** `3c2725ca`  
**Rust/Cargo:** 1.96.0  
**Verifier source:** `tools/pulse-multiconductor-audit/src/main.rs` and `src/borrowed.rs`

## Results

The benchmark uses 30 independent processes at 1k, 10k and 100k events, plus one 1M capacity probe. Both parsers produced the same structural summary for each run. The measured operation is parse + closed event-shape validation only.

| Events | Owned median parse | Borrowed median parse | Speedup | Owned requested allocations | Borrowed requested allocations | Allocation reduction |
|---:|---:|---:|---:|---:|---:|---:|
| 1,000 | 3.563 ms | 1.109 ms | 3.21× | 2.516 MB | 0.107 MB | 23.5× |
| 10,000 | 37.889 ms | 11.308 ms | 3.35× | 26.140 MB | 1.704 MB | 15.3× |
| 100,000 | 388.333 ms | 113.436 ms | 3.42× | 256.250 MB | 13.632 MB | 18.8× |
| 1,000,000 | 3.826 s | 1.165 s | 3.28× | 2.522 GB | 109.052 MB | 23.1× |

The 1M workload is one capacity probe, not a percentile population. Allocation numbers are application allocator request/peak metrics, not OS RSS. The candidate uses borrowed `RawValue` payload slices and a bounded raw-field representation; strict unknown-field and duplicate-key checks remain enabled.

## Critical implementation boundary

This is **not yet a complete borrowed/streaming auditor**.

The current production `parse_history_strict` path does:

1. `borrowed::borrowed_parse_validate(raw_text)` as a strict preflight;
2. `serde_json::from_slice(raw)` to materialize the owned `History`;
3. the full semantic audit over `Vec<Event>`, where each `Event.data` remains a `serde_json::Value`.

Therefore the microbenchmark does not show that production full-audit memory or latency has improved. It shows the borrowed parser/shape validator is a promising candidate component. The current full synthetic auditor still peaks around 3.4 GiB at 1M events.

## Next engineering gate

Implement a separate borrowed/compact semantic auditor while preserving the materialized auditor as the differential oracle. The promotion gate must require:

- exact closed-world event validation and duplicate JSON key rejection before lossy materialization;
- identical PASS/FAIL/INCOMPLETE status, findings, counts and claim ceiling for all safe and mutation fixtures;
- retained exact joins for grant ActionHash, principal, operation/attempt, source action, validator event, observer checkpoint and causal edge;
- retained per-conductor sequence ordering only within the same conductor;
- no cross-conductor wall-clock or scalar-height inference;
- no removal of unknown/ambiguous outcomes;
- identical or canonically equivalent reports across owned and compact paths;
- 30 independent full-audit runs at 1k/10k/100k; 1M and 5M capacity probes with memory limits.

## Claim ceiling

The observed speedup applies only to synthetic parse/shape-validation. It is not whole-history audit throughput, Holochain runtime performance, DHT convergence, authorization latency, or evidence that the complete auditor now fits under 1 GiB.
