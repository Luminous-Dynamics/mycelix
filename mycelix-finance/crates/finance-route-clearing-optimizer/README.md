# finance-route-clearing-optimizer

Pure Rust reference optimizer for FIN-ECO-005E2. Given externally qualified net positions, one asset, one netting set, participant liquidity limits, and directed settlement routes, it computes a maximum feasible allocation and minimizes linear route cost among maximum-flow allocations.

This is deliberately a bounded optimizer, not a general payment system.

## Model and safety boundary

- One network, one asset, one policy version and one netting set per run.
- Positive net position means the participant owes; negative means the participant is owed.
- Debtor source capacity is min(net amount owed, caller-supplied available liquidity).
- Each enabled directed debtor-to-creditor route has a minor-unit capacity and a non-negative linear cost weight.
- Identical route observations with the same canonical route ID are deduplicated. Changed terms under an existing ID fail closed.
- Missing, disabled, wrong-profile, zero-capacity and role-incompatible routes cannot silently become available.
- Arithmetic uses integer amounts and checked totals. The min-cost maximum-flow solver uses successive shortest augmenting paths with reduced-cost Dijkstra and integer capacities.
- The greedy cheapest-edge-first method is included as a deliberately weaker comparison baseline.
- Residual debtor and creditor amounts are returned explicitly. A partial flow is not called full allocation.

The authority enum is an explicit caller-side gate, not a cryptographic verifier. The evidence reference is an audit link, not proof that the arrangement is legally enforceable. The caller must qualify the netting arrangement and the source records before invoking this crate. The output is not a transfer instruction, a ledger mutation, a guarantee of participant solvency, or a settlement receipt.

The cost value is a dimensionless linear optimization weight per minor unit. It is not a provider's actual fee quote. This first version intentionally does not model fixed-per-leg charges, fee floors/caps, FX spreads, deadlines, multihop routes, failed settlement, participant defaults, or nonlinear costs. A future extension should retain this linear solver as a baseline and add those costs without silently mixing objective units.

## Fixed activation charges

The `fixed_charge` module adds an exact small-instance extension for route activation costs. A route can be expensive to open even when its unit cost is low; a plan with fewer activated routes may therefore beat the pure linear min-cost-flow result. The exact search evaluates every route subset and calls the linear optimizer as its subproblem.

This problem is combinatorial, so the implementation hard-limits exact search to 12 distinct route IDs (at most 4,096 subsets) and returns an error beyond that limit. It does not silently switch to a heuristic. The objective is still lexicographic: maximize feasible amount, then minimize unit costs plus one fixed activation charge per used route. The activation charge is a modeled weight, not a provider quote.

The returned plan preserves excluded routes from the full candidate input even when those routes were not part of the chosen subset. The existing linear solver remains the scalability baseline; fixed-charge search is an exact reference/oracle for small instances.

Run the bounded fixed-charge timing comparison with:

    cargo run --release --manifest-path mycelix-finance/Cargo.toml -p finance-route-clearing-optimizer --example fixed_charge_benchmark

That example compares the linear-cost allocation against exact fixed-charge search on deterministic graphs with 4, 6, 8, 10 and 12 route IDs. It prints both objective breakdowns and p50/p95 runtime over five warmup runs followed by 101 measured repetitions. The 12-route case evaluates 4,096 subsets, so the fixed-charge measurements must not be extrapolated to larger route sets.

## Paradigm shift

Do not optimize a single opaque “savings” score. First maximize the amount that is actually route- and liquidity-feasible; among plans with the same flow, minimize modeled routing cost. The residual is visible, not hand-waved away. That lexicographic objective separates service completion from price optimization and avoids trading a large amount of unmet obligations for a cosmetically cheap plan.

The linear solver is exact for its stated model, and the fixed-charge extension is exact only within its explicit 12-route bound. Neither is an assertion of superiority to production RTGS liquidity-saving mechanisms (LSMs), nor that every real-world clearing profile reduces to a simple flow problem.

## Tests and benchmark harness

The crate includes tests for global route-cost allocation versus greedy, liquidity and route-capacity residuals, absent routes, profile mismatch, authority gating, duplicate/conflicting observations, input-order invariance, overflow and comparison with an exhaustive integer oracle across 64 deterministic small graphs. The fixed-charge module also tests route-choice changes caused by activation fees, one-time charging, input-order invariance, preservation of excluded routes, flow-first objective ordering, duplicate handling and the explicit route-count ceiling.

A reproducible, dependency-free synthetic timing harness is available as:

    cargo run --release --manifest-path mycelix-finance/Cargo.toml -p finance-route-clearing-optimizer --example route_clearing_benchmark

It prints CSV for 10, 30, 70, 140 and 700 candidate route arcs across three deterministic scenarios: a sparse cyclic network, constrained per-route capacity, and periodic source-route outages. Each row includes participant counts, flow/cost output, full-allocation status, optimizer-vs-greedy flow gain, same-flow cost savings, and p50/p95 runtime for both algorithms. The capacity-stress case uses 400-unit arcs specifically because 500-unit arcs would still let a debtor route its full 1,000-unit obligation across two outgoing arcs; the 400-unit cap creates a known 20% capacity deficit. The benchmark asserts the optimizer's lexicographic dominance over greedy and exact expected flow ceilings for each synthetic scenario. The exact-head benchmark workflow records OS, CPU, Rust version, commit SHA, source digests and measured outputs, and provisionally gates p95 below 100 ms at 700 arcs. Run distributions must be compared on a pinned machine before interpreting latency differences. Synthetic runtime or a greedy-baseline win is not evidence of real-world settlement savings or superiority to a production liquidity-saving mechanism.

Suggested next benchmark gates:

- **Correctness target:** 100% agreement with the exhaustive oracle on every frozen graph within the declared small-instance bound; zero capacity, liquidity, profile or conservation violations.
- **Provisional performance target, not yet measured:** p95 below 100 ms for 700 candidate route arcs in release mode on a pinned reference machine. Record CPU, OS, Rust commit, and run distributions; optimize measured bottlenecks rather than weakening safeguards.
- **Quality target:** zero flow/cost gap versus the exact min-cost-flow solution for the linear model. For nonlinear tariffs or time-dependent LSMs, report separate oracle gaps and never compare unlike objectives.
- The min-cost-flow plan must match the exhaustive oracle's maximum flow and minimum cost on all frozen small fixtures.
- The fixed-charge exact search must select the minimum total cost at maximum flow, charging each used route once, and reject any instance beyond 12 distinct route IDs.
- Zero cross-profile allocations, zero allocation above supplied liquidity or route capacity, zero net-position conservation errors.
- Identical output for reordered inputs and stable output for a pinned toolchain.
- Report optimality gap only against an exact oracle on instances where that oracle actually completed.
- Publish p50/p95/p99, memory, maximum graph size and failure counts for frozen synthetic runs; keep route availability, liquidity, and price assumptions alongside each report.
- Compare on common fixtures with gross/FCFS, this crate's greedy baseline, small-instance exhaustive oracle, and later queueing/annealing/auction approaches. Do not compare incomparable objective functions.

## Industry context and research anchors

- The Bank of Canada, Staff Working Paper 2026-32 (September 2026), evaluates liquidity optimization for TARGET2 payment batches of 70 and 140 and compares a constrained quadratic model with simulated annealing. It reports results calibrated to its study sample; those euro amounts are not portable benchmark targets for Mycelix. https://www.bankofcanada.ca/2026/09/staff-working-paper-2026-32/
- BIS Working Paper 1318 (16 December 2025) evaluates an auction-based liquidity-saving mechanism using simulations calibrated to Peru's BCRP-RTGS, South Africa's SAMOS, SADC-RTGS and Switzerland's SIC. It reports 2.6%–12.2% liquidity savings against conventional RTGS in its trials and an average 79% auction success rate, with a trade-off between liquidity efficiency and delay. These are study-specific, not claims about Mycelix. https://www.bis.org/publications/working-paper-1318-auction-based-liquidity-saving-mechanisms
- CPMI-IOSCO Principles for Financial Market Infrastructures emphasize enforceable legal basis, comprehensive risk management, liquidity risk and clearly defined settlement finality. An optimizer cannot substitute for those controls. https://www.bis.org/committees/cpmi/pfmi/overview
- BIS Project Nexus material provides useful context for cross-border governance, risk, resilience, participant and operating-model questions. https://www.bis.org/publications/nexus-enabling-instant-cross-border-payments

## Claim ceiling

This crate demonstrates a reference algorithm and authored tests. Until the exact commit is compiled and those tests run, they are not passing evidence. Even when tests pass, they establish only behavior on the modeled inputs; they do not establish legal set-off, external liquidity availability, production-grade safety, actual settlement finality, all-in fee savings or superiority to TARGET2, SAMOS, Nexus, or any production LSM.

## Bounded atomic-batch clearing reference

The `batch` module adds a fourth conceptual layer to the research model: `select_atomic_batch` chooses an all-or-nothing subset of eligible queued payments whose **post-batch** participant balances remain non-negative and whose total use of each caller-supplied remaining route capacity is respected. Because selected credits and debits are applied atomically, it can clear a closed cycle when every participant begins with zero gross liquidity—where strict sequential gross settlement cannot start.

This is a **bounded exact reference/oracle**, not a production LSM. It requires the caller's explicit `ExternallyQualified` authority assertion, current route availability and remaining-window capacities. It does not authenticate legal authority or evidence, perform external settlement, model participant auctions/side payments, FX, fraud/default workflows, or prove settlement finality. Its objective is explicit: maximize total selected gross payment value, then minimize modeled fees, then maximize count, then deterministic payment-ID tie-breaking. Fees are reported as model metrics and are not debited from balances. It fails closed above 16 eligible orders (65,536 subsets); larger rolling batches require a separate measured heuristic compared against this oracle on tiny cases.

The selector treats a selected payment as late only when `tick > deadline`; an eligible payment still unselected at `tick >= deadline` counts as missed. A dedicated boundary test verifies this distinction, and aging-priority arithmetic saturates at extreme tick values rather than wrapping. The unit test contrasts a zero-funded three-party cycle under `StrictFifo` (zero settled gross value) with the atomic batch selector (all three orders selected, while every final balance remains zero). This demonstrates a difference in settlement model, not proof that an arbitrary rail supports atomic batch processing or has legal netting authority. An independent integration-test oracle also checks the value/fee/count/ID objective against all 16 subsets on 64 deterministic four-payment graphs. A release microbenchmark checks fully funded dense payment cycles with 4/8/12/16 orders (up to 65,536 exact subsets), five warmups and 101 measured repetitions. The dense fixture makes every subset balance-feasible so the 16-order case exercises full objective evaluation instead of benchmarking only early liquidity-based rejection. The exact-head benchmark artifact includes `atomic-batch-results.csv`, `route-clearing-results.csv` and the queue-policy CSV, source digests, standalone lockfile and host/toolchain metadata. Provisional thresholds (below 250 ms at 16 atomic orders and below 100 ms at 700 route arcs) are local regression gates, not production-throughput claims.

## Event-driven gross-payment queue simulator

The `queue` module is a separate discrete-tick model, not a replacement for the static exact flow optimizer. It replays individual gross payments with balances, arrivals, deadline targets, route outages, per-window capacity and three policies:

- `StrictFifo`: stop attempting later queue entries when the head is blocked.
- `LiquidityAwareFifo`: scan in arrival order, skip blocked entries and retry after a successful transfer.
- `PriorityScan`: higher priority first, then arrival time and payment ID; skip blocked entries and retry after successful transfers.
- `AgingPriorityScan { interval_ticks }`: adds one effective priority level per configured number of elapsed ticks, then uses arrival time and ID to break ties. The benchmark compares intervals of 1, 5 and 20 ticks; these are sensitivity settings, not industry standards, and zero is rejected.
- `EarliestDeadlineFirst`: explicit earliest deadline first, with missing deadlines last, then arrival time and payment ID; this is a heuristic, not an optimal deadline scheduler.

This separation is important: a static allocation can be feasible without being executable at a particular time. The queue model makes that gap measurable while keeping fees as modeled metrics (not balance debits or provider quotes), and preserves excluded and unsettled items explicitly. It reports participant starting/final/minimum/maximum balances and balance-time in minor-unit ticks; aggregate balance-time is not itself a liquidity-savings percentage. Deadlines are target times, not automatic expiry. Route outage intervals are half-open `[start_tick, end_tick)`; capacity resets at multiples of the configured window length. A fixed event budget makes pathological fixtures fail closed.

Run the reproducible synthetic policy comparison:

    cargo run --release --manifest-path mycelix-finance/Cargo.toml -p finance-route-clearing-optimizer --example queue_policy_benchmark

The harness sweeps 70, 140 and 700 orders across all seven policy configurations under baseline, scarce-liquidity and outage-heavy synthetic scenarios. The `.github/workflows/finance-queue-benchmark.yml` workflow checks out the exact pull-request head, copies this dependency-free crate outside the larger Finance workspace, generates an offline standalone lockfile, runs the release benchmark, and preserves CSV, the standalone lockfile, source-file hashes, toolchain/machine metadata and SHA-256 checksums as an artifact. This isolation prevents unrelated workspace lockfile drift from blocking or silently changing the benchmark. No measured results should be claimed before that exact-head job completes successfully. It also fails closed unless all 63 expected case/policy rows are present, the synthetic cases have zero excluded inputs, and every 700-order case stays below the provisional 100 ms p95 target; that gate is a local performance threshold, not proof of parity with a production LSM. It reports settled/unsettled/excluded counts, settled amount, late and missed-deadline counts, modeled fees, total/p95/maximum queue wait (with unresolved eligible orders measured at their age at the horizon), p50/p95/p99 runtime after five warmup runs and 101 measured repetitions, p95 participant balance-time and maximum observed participant balance. An independent one-tick exhaustive sequence oracle in `tests/queue_sequence_oracle.rs` compares all seven policy configurations against optimal tiny-instance schedules over 64 deterministic traces. These are policy-comparison procedures, not industry benchmark results; run them and preserve the CSV before making performance claims. Record machine, OS, compiler, exact commit, and run outputs; do not compare against TARGET2/SAMOS savings percentages unless the data, objective and settlement rules are aligned.

Current model limits: one asset/profile per run; gross single-route payments; integer balances; no multilateral offset, FX, dynamic participant default, external rail adapter, queue-order-dependent network auctions, actual provider tariffs or finality verification. The simulation output is neither a transfer instruction nor settlement evidence.
