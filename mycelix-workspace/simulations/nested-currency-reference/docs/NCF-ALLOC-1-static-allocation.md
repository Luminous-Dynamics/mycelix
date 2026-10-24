# NCF-ALLOC-1: deterministic pre-partition escrow allocation

**Status:** Rust reference implementation and deterministic benchmark harness committed; formatting, compile, tests, and benchmark execution are not yet confirmed. This defines a planning policy, not a distributed capacity authority or production-money feature.

## Research question

Can placement of non-overlapping escrow rights before a partition improve the amount of economic demand served, without adding any right to resize or reclaim another owner's allocation during the partition?

This deliberately separates two problems:

1. **Allocation placement:** choose which pre-authorized owner/shard receives each unit of the existing gross-exposure budget.
2. **Reclamation:** change an allocation after it has become spendable, which requires a separately qualified fencing/reconciliation protocol.

This work explores the first question and must not be used to bypass the second.

## Policies implemented

The dependency-free Rust module is `src/allocation.rs`.

- **EqualSplit:** equal integer weights, satisfying explicit per-shard minimum slices first.
- **DemandWeighted:** weight by the ceiling of the mean of the declared integer demand samples.
- **RobustQuantile:** weight by a nearest-rank historical quantile plus an explicit proportional margin. Quantiles are in basis points (9,500 = 95th percentile); margin is in parts per million.

The plan may also preserve an explicit contingency reserve that is not allocated to an isolated shard. Inputs must already be normalized to the same exact canonical instrument/unit and comparable training window. Demand history is a forecast signal, never spending authority.

## Integer and reproducibility contract

Allocation is integer-only. The implementation uses checked arithmetic, a stable shard-ID ordering, and largest-remainder apportionment with lexical-ID tie-breaking. This avoids floating-point drift and makes the same canonical inputs produce the same per-shard rights regardless of input list order.

For total capacity C, contingency reserve R, and slice amounts a_i:

- `C = R + sum(a_i)`
- `a_i >= minimum_i` for every shard
- `R >= 0` and no shard is allocated the reserve
- the sum of minima must fit within `C - R`, otherwise allocation fails closed
- no output slice can expand a configured community/agreement budget
- this function does not mutate an existing reservation or transfer rights between owners

The module exposes `AllocationPlan::validate` so downstream code can check conservation independently of the policy that produced the plan.

## Matched-trace experiment

The benchmark binary is `src/bin/ncf-allocation-bench.rs`; the clean-checkout wrapper is `run-allocation-bench.sh`.

Example:

```sh
bash mycelix-workspace/simulations/nested-currency-reference/run-allocation-bench.sh \
  16 32 1000 100000 50000 0x4d5943454c495831
```

Arguments are shard count, historical samples per shard, evaluation scenarios per mode, total capacity, contingency reserve in ppm, and seed. The wrapper binds the result to the exact commit, profile digest, Rust compiler/toolchain, binary digest, output digest, and receipt digest. JSON output reports all three allocation policies over stationary and shifted demand, including requested/served/unserved demand, unused reserved-capacity units over time, demand-served ppm, allocated-capacity utilization, worst-scenario service, safety accounting checks, and the actual per-shard slices.

The synthetic generator deliberately has an uneven historical demand profile, bounded jitter, occasional bursts, and a second evaluation mode that rotates the demand profile to simulate distribution shift. The same stationary and shifted traces are shared by all policies. This makes comparisons fair within a benchmark run, but it is a synthetic workload—not evidence of real community demand or an industry performance result.

## Required experiments before a strong claim

1. Run unit tests and the exact-head benchmark with several recorded seeds.
2. Compare equal split, mean-weighted, and robust-quantile allocation on stationary, shifted, bursty, and held-out traces.
3. Tune any weights or quantile only on a training corpus; do not select them on the held-out evaluation set.
4. Add workload-derived traces and independent seeds; report per-shard unmet demand and essential-service classes, not just aggregate utilization.
5. Compare all policies under the same community/agreement caps and same total contingency reserve.
6. Verify the resulting slices through `ReservationBook` and the actual authority path before using any policy operationally.
7. Run concurrent and multi-agent fault workloads separately; this benchmark cannot qualify races, partitions, fencing, Holochain DHT behavior, or recovery.

## Industry comparison and novelty boundary

The policy primitives are not claimed as novel. Escrow allocation and invariant confluence are established approaches for preserving upper bounds while allowing disconnected local operations. Interledger's settlement architecture already uses peer-specific credit limits and idempotent settlement behavior ([Settlement Engines RFC](https://interledger.org/developers/rfcs/settlement-engines/)). Research on coordination avoidance formalizes when invariants can be maintained without coordination ([Bailis et al.](https://arxiv.org/abs/1402.2237)).

Mycelix's testable contribution here is a deterministic policy interface whose output is itself a conserved risk-right plan, paired with held-out demand-shift evaluation and a hard rule that forecast optimization cannot resize active rights during a partition. Any improvement must be demonstrated as a Pareto improvement under matched traces, with zero safety violations; do not claim state-of-the-art performance from a synthetic single-process run.

## Industry comparison and a meaningful performance target

Do not benchmark a reference allocator by comparing its single-process operation count to a payment-rail TPS figure. The workloads and semantics differ.

The BIS Innovation Hub's **Project FuSSE** report (29 January 2026) describes a modular settlement-engine proof of concept that reached 10,000 transactions per second under controlled test conditions and demonstrated independent scaling of services. The report explicitly says it is not a performance benchmark or implementation reference, and does not claim PFMI compliance. It also warns that microservices increase operational complexity and attack surface, while post-quantum cryptography may add computational and bandwidth cost ([BIS report](https://www.bis.org/publications/project-fusse-exploring-flexible-scalable-and-secure-settlement-engines)). Interledger separately defines peer-specific credit limits and idempotent settlement behavior ([Settlement Engines RFC](https://interledger.org/developers/rfcs/settlement-engines/)). These are useful comparison points, not direct evidence that Mycelix is faster or safer.

For a future three-node Mycelix/Holochain deployment, use a staged performance target **after** semantic and distributed correctness are qualified:

- **Functional gate:** zero cap, replay, double-transfer, and post-fence violations during a 10-million-operation mixed workload and injected recovery faults.
- **Initial load target:** sustain 1,000 accepted local ledger operations per second for 30 minutes, while separately reporting local acceptance and finality/settlement latency.
- **Stretch load target:** 10,000 accepted local obligation/settlement events per second on a declared 16-shard testbed, with p99 local-accept latency below 100 ms under the published healthy-network workload. This is an aspirational engineering target, not an asserted achievable result and not comparable to the BIS result until the event semantics, hardware, security settings, and end-to-end finality boundaries are matched.
- **Scale target:** publish throughput and tail latency at 1, 4, 8, and 16 workers; report efficiency as throughput gain divided by worker-count gain, plus memory, CPU, storage I/O, network bytes, receipt verification cost, and cryptographic verification cost.
- **Fault target:** run the same 10-million-operation trace with partitions, stale owner retries and crash/restart injection. Any safety violation invalidates the run irrespective of throughput.
- **Security-agility target:** measure receipt size, CPU cost, and p95/p99 latency for each approved signature/verification profile separately; preserve the exact cryptographic profile and software versions in every receipt.

A benchmark receipt must record hardware model/CPU count/RAM, OS/kernel, Rust commit and compiler, build profile, shard/peer topology, durability settings, security profile, trace digest, seeds, warmup and measurement intervals, source commit, binary digest, output digest, and run-to-run variation. Do not report a single best run as the headline; publish repeated runs and distributions.

## Claim ceiling

A future passing result can establish only that this Rust reference allocator and the named synthetic traces satisfied the declared integer accounting checks on an exact source/toolchain. It does not prove a global distributed cap, safe ownership transfer, the quality of any real demand forecast, production Holochain enforcement, solvency, legal tender status, or economic welfare.
