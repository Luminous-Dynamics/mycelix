# Pulse multi-conductor auditor — synthetic benchmark receipt

**Result:** PASS for the synthetic structural-history workload and exact run  
**Workflow:** [Pulse Multi-Conductor Causal History Audit, run #81](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42094896905)  
**Audit / benchmark artifact:** [benchmark archive](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42094896905/artifacts/13447635821), artifact ID `13447635821`  
**Secondary audit artifact:** [safe-history report and receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42094896905/artifacts/13447086891), artifact ID `13447086891`  
**Checkout source SHA:** `f308333ada8a77166fb3684db9bc38af47a2d03d`  
**Rust:** `rustc 1.96.0 (ac68faa20 2026-05-25)`  
**Cargo:** `cargo 1.96.0 (30a34c682 2026-05-25)`  
**Cargo.lock SHA-256:** `6e74c464cb7c89ff6e96914ec671a4fc2196c0164da3f28bd245b757baf16eaf`  
**Verifier source SHA-256:** `8bd1ed58edc5d42ae9fa79a11990613f088aa17028e57f9b2da17326935bbe01`  
**Raw results JSONL SHA-256:** `b56875acb7935e0263a1bc47732da5efa42336d016e58c90f3cc4f5a060c9c84`  
**Summary JSON SHA-256:** `fac4a3b0acaa5fbe1b9ad21613c6ff6baf1e45cca5cc188c85bd0f8a4b2f740d`  
**Downloaded benchmark ZIP SHA-256:** `1a472d4f62289a2d4eea404e4b7d6e37b072582b1017861b676b2cdcbe9be2bd`

## Workload and method

The benchmark generator creates deterministic synthetic safe histories with two peer conductors, independent local sequence counters, and unverified remote-signal events. It contains no cross-conductor causal edges; this run therefore measures structural parsing/audit cost for that controlled workload, not complex causal-graph cost.

The workflow ran 30 independent processes at 1,000, 10,000 and 100,000 events, plus one 1,000,000-event capacity probe. Each run returned `PASS`. Per-process peak RSS was captured with `/usr/bin/time`. Summary percentiles are within each run-size population; the 1M probe is explicitly not a percentile sample.

## Observed results

| Events | Independent processes | Median audit time | p95 audit time | Min–max audit time | Median events/s | Max RSS |
|---:|---:|---:|---:|---:|---:|---:|
| 1,000 | 30 | 1.780 ms | 2.514 ms | 1.695–2.740 ms | 562,973 | 6,236 KiB (6.09 MiB) |
| 10,000 | 30 | 27.251 ms | 29.848 ms | 25.342–30.289 ms | 368,283 | 38,396 KiB (37.50 MiB) |
| 100,000 | 30 | 327.147 ms | 335.232 ms | 319.231–349.609 ms | 305,854 | 360,548 KiB (352.10 MiB) |
| 1,000,000 | 1 | 3.595 s | n/a | 3.595 s | 278,187 | 3,582,296 KiB (3.42 GiB) |

All reported status counts were `PASS` (30/30 for the first three sizes, 1/1 for the capacity probe).

## Interpretation

The primary near-term bottleneck is memory growth, not a reason to weaken semantic checks. At 1M events the current materialized representation peaks at about 3.42 GiB. The 100k p95 is about 335 ms. This gives the next optimization target a measured baseline.

The current implementation materializes `Vec<Event>` with each `Event.data` as `serde_json::Value`, then constructs indexes and causal/semantic joins. A borrowed/streaming pipeline should reduce per-event allocation and retain only the compact fields needed for exact joins. The candidate must preserve closed-world event shape, source identities, per-conductor order, evidence-referenced causal edges, revocation checks, validator/observer evidence and every existing negative control.

## Claim ceiling and limitations

- This measures the structural multi-conductor history verifier on synthetic data, not Holochain conductor throughput, DHT convergence, network delivery or production authorization latency.
- The 1M run is a single capacity probe. No 1M p95/p99 claim is made.
- The generator has two conductors and no cross-conductor causal edges; it does not establish performance on dense causal DAGs, adversarial fork histories or actual runtime captures.
- The numbers are specific to the recorded GitHub Actions runner environment and Rust build. They are not directly comparable to Porcupine, Jepsen, Zanzibar, SpiceDB or OpenFGA without matched semantics, workload, hardware, consistency and measurement boundaries.
- A faster structural checker is not evidence of linearizability or correctness beyond its encoded checks.

Optimization work is tracked in [#5872](https://github.com/Luminous-Dynamics/mycelix/issues/5872). Runtime histories and independent conductor evidence remain separately tracked in [#5772](https://github.com/Luminous-Dynamics/mycelix/issues/5772).
